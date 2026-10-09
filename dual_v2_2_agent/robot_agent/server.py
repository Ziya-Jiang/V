#!/usr/bin/env python3
"""Persistent, loopback-only NDJSON server for robot planning and execution.

The process is the sole owner of RobotArm, cameras and perception models.
Planning is hardware-free and creates an immutable server-side Plan referenced
by UUID. Direct operator execution requires explicit confirmation. Harness
execution first creates an inert request and can start only after a separate
on-site approval of that exact Plan.
"""

from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import secrets
import signal
import socket
import sys
import threading
import time
from typing import Any, Dict, Optional
import uuid

from .base import PlanError
from .context import RobotContext, write_status
from .executor import ExecutionCancelled, Executor, resolve_object_ref
from .registry import build_default_registry

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = int(
    os.environ.get(
        "ROBOT_AGENT_PORT",
        os.environ.get("ROBOTRAM_BRIDGE_PORT", "15555"),
    )
)
HARDWARE_CONFIRMATION = "ENABLE_DUAL_V2_2_HARDWARE"
ACTIVE_TASK_STATUSES = {"queued", "running"}
FINAL_TASK_STATUSES = {"succeeded", "failed", "cancelled"}
HARNESS_EXECUTION_SKILLS = {
    "perceive_scene",
    "pick_and_place",
    "tidy_table",
    "safety_reset",
}


def _sanitize(value: Any) -> Any:
    """Recursively convert numpy scalars/arrays to JSON-safe Python values."""
    try:
        import numpy as np

        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, np.ndarray):
            return _sanitize(value.tolist())
    except ImportError:
        pass
    if isinstance(value, dict):
        return {str(k): _sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class RobotAgentServer:
    def __init__(self, output_dir=None, instruction="clean the table") -> None:
        self.ctx = RobotContext(
            output_dir=output_dir,
            instruction=instruction,
            plan_only=False,
        )
        self.registry = build_default_registry(self.ctx)
        self.ctx.load_categories_from_output()
        self._default_output_dir = self.ctx.output_dir
        self.agent = None

        self._token = os.environ.get("ROBOT_AGENT_TOKEN")
        self._plans: Dict[str, Dict[str, Any]] = {}
        self._tasks: Dict[str, Dict[str, Any]] = {}
        self._execution_requests: Dict[str, Dict[str, Any]] = {}
        self._active_task_id: Optional[str] = None
        self._legacy_call_active = False
        self._state_lock = threading.RLock()
        self._agent_lock = threading.Lock()
        self._hardware_lock = threading.Lock()
        self._cleanup_lock = threading.Lock()
        self._server_socket_lock = threading.Lock()
        self._server_socket: Optional[socket.socket] = None
        self._shutdown = threading.Event()
        self._cleaned_up = False

        self._ready = threading.Event()
        self._boot_error: Exception | None = None
        self._bootstrap_started = False
        self._bootstrap_lock = threading.Lock()
        self._plan_only = os.environ.get("ROBOT_AGENT_PLAN_ONLY", "0") == "1"
        # Plan-only mode is a process-level safety boundary for Harness
        # integration. It must never initialize cameras, arms, or perception
        # models, even if the ordinary production default requests eager boot.
        self._eager_bootstrap = (
            not self._plan_only
            and os.environ.get("ROBOT_AGENT_EAGER_BOOTSTRAP", "1") != "0"
        )
        if self._eager_bootstrap:
            self._start_bootstrap()

    def _start_bootstrap(self) -> None:
        with self._bootstrap_lock:
            if self._bootstrap_started:
                return
            self._bootstrap_started = True
            threading.Thread(
                target=self._bootstrap,
                name="robot-agent-bootstrap",
                daemon=True,
            ).start()

    def _bootstrap(self) -> None:
        try:
            self.ctx.bootstrap()
        except Exception as error:  # noqa: BLE001
            self._boot_error = error
            write_status("error", error=str(error))
        finally:
            self._ready.set()

    def _ensure_bootstrapped(self) -> None:
        self._start_bootstrap()
        self._ready.wait()
        if self._boot_error is not None:
            raise self._boot_error

    def request_shutdown(self) -> None:
        """Stop accepting requests and cooperatively cancel active hardware work."""
        self._shutdown.set()
        with self._state_lock:
            task = (
                self._tasks.get(self._active_task_id)
                if self._active_task_id is not None
                else None
            )
            if task is not None and task["status"] in ACTIVE_TASK_STATUSES:
                task["cancelRequested"] = True
                task["cancelEvent"].set()

        with self._server_socket_lock:
            server_socket = self._server_socket
        if server_socket is not None:
            try:
                server_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                server_socket.close()
            except OSError:
                pass

    def close(self, timeout_s: float = 10.0) -> None:
        """Release the listener and initialized hardware resources exactly once."""
        with self._cleanup_lock:
            if self._cleaned_up:
                return
            self._cleaned_up = True

        self.request_shutdown()
        if self._bootstrap_started and not self._ready.wait(timeout=max(0.0, timeout_s)):
            return

        acquired = self._hardware_lock.acquire(timeout=max(0.0, timeout_s))
        if not acquired:
            return
        try:
            self.ctx.cleanup()
        finally:
            self._hardware_lock.release()

    # ------------------------------------------------------------------ #
    def handle(self, request: Dict[str, Any]) -> Dict[str, Any]:
        try:
            if not isinstance(request, dict):
                raise PlanError("request must be a JSON object")
            self._authenticate(request)
            method = request.get("method")
            args = request.get("args") or {}
            if not isinstance(args, dict):
                raise PlanError("'args' must be a JSON object")
            if self._plan_only and method not in {"health", "manifest", "plan"}:
                raise PlanError(
                    f"method unavailable in plan-only mode: {method}"
                )
            if method == "health":
                return self._health()
            if method == "manifest":
                return self._manifest()
            if method == "plan":
                return self._plan(args)
            if method == "execute":
                return self._execute(args)
            if method == "request_execution":
                return self._request_execution(args)
            if method == "execution_request_status":
                return self._execution_request_status(args)
            if method == "pending_execution_requests":
                return self._pending_execution_requests()
            if method == "approve_execution":
                return self._approve_execution(args)
            if method == "deny_execution":
                return self._deny_execution(args)
            if method == "task_status":
                return self._task_status(args)
            if method == "task_events":
                return self._task_events(args)
            if method == "cancel":
                return self._cancel(args)
            if method == "call":
                return self._call(args)
            raise PlanError(f"unknown method: {method}")
        except PlanError as error:
            return {"ok": False, "message": str(error), "error": str(error)}
        except Exception as error:  # noqa: BLE001
            method = request.get("method") if isinstance(request, dict) else None
            return {"ok": False, "message": f"{method} failed: {error}", "error": str(error)}

    def _authenticate(self, request: Dict[str, Any]) -> None:
        if not self._token:
            return
        provided = request.get("token")
        if not isinstance(provided, str) or not secrets.compare_digest(
            provided,
            self._token,
        ):
            raise PlanError("unauthorized")

    def _health(self) -> Dict[str, Any]:
        if self._boot_error is not None:
            bootstrap_state = "error"
        elif self._ready.is_set():
            bootstrap_state = "ready"
        elif self._bootstrap_started:
            bootstrap_state = "loading"
        else:
            bootstrap_state = "not_started"
        with self._state_lock:
            active_task_id = self._active_task_id
        return {
            "ok": True,
            "message": "healthy",
            "data": {
                "host": DEFAULT_HOST,
                "port": DEFAULT_PORT,
                "planOnly": self._plan_only,
                "allowedMethods": (
                    ["health", "manifest", "plan"]
                    if self._plan_only
                    else [
                        "health",
                        "manifest",
                        "plan",
                        "execute",
                        "request_execution",
                        "execution_request_status",
                        "pending_execution_requests",
                        "approve_execution",
                        "deny_execution",
                        "task_status",
                        "task_events",
                        "cancel",
                        "call",
                    ]
                ),
                "bootstrapState": bootstrap_state,
                "bootstrapError": (
                    str(self._boot_error)
                    if self._boot_error is not None
                    else None
                ),
                "eagerBootstrap": self._eager_bootstrap,
                "activeTaskId": active_task_id,
            },
        }

    def _manifest(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "message": "manifest",
            "data": {
                "tools": self.registry.tool_manifest(),
                "skills": self.registry.skill_manifest(),
            },
        }

    def _plan(self, args: Dict[str, Any]) -> Dict[str, Any]:
        from .planner import Agent
        from .llm import make_llm_from_env

        instruction = args.get("instruction", "clean the table")
        if not isinstance(instruction, str) or not instruction.strip():
            raise PlanError("'instruction' must be a non-empty string")
        output_dir = args.get("output_dir")
        if output_dir is not None and not isinstance(output_dir, str):
            raise PlanError("'output_dir' must be a string")

        with self._agent_lock:
            if self.agent is None:
                self.agent = Agent(self.registry, llm=make_llm_from_env())
            plan = self.agent.plan(instruction)

        plan_id = str(uuid.uuid4())
        created_at = _timestamp()
        ttl_seconds = max(
            30,
            int(os.environ.get("ROBOT_AGENT_PLAN_TTL_S", "600")),
        )
        record = {
            "planId": plan_id,
            "instruction": instruction,
            "output_dir": output_dir,
            "createdAt": created_at,
            "expiresMonotonic": time.monotonic() + ttl_seconds,
            "plan": copy.deepcopy(plan),
        }
        with self._state_lock:
            self._plans[plan_id] = record
        return {
            "ok": True,
            "message": "plan created",
            "data": {
                "planId": plan_id,
                "instruction": instruction,
                "output_dir": output_dir,
                "createdAt": created_at,
                "plan": plan.to_dict(),
            },
        }

    def _execute(self, args: Dict[str, Any]) -> Dict[str, Any]:
        plan_id = args.get("planId")
        confirmation = args.get("confirmation")
        if not isinstance(plan_id, str) or not plan_id:
            raise PlanError("'planId' is required")
        if confirmation != HARDWARE_CONFIRMATION:
            raise PlanError(
                f"'confirmation' must equal {HARDWARE_CONFIRMATION!r}"
            )

        with self._state_lock:
            plan_record = self._plans.get(plan_id)
            if plan_record is None:
                raise PlanError(f"unknown planId: {plan_id}")
            if time.monotonic() >= plan_record["expiresMonotonic"]:
                self._plans.pop(plan_id, None)
                raise PlanError(f"expired planId: {plan_id}")
            if self._legacy_call_active:
                raise PlanError("a legacy hardware call is already active")
            if self._active_task_id is not None:
                active = self._tasks.get(self._active_task_id)
                if active and active["status"] in ACTIVE_TASK_STATUSES:
                    raise PlanError(
                        f"another task is already active: {self._active_task_id}"
                    )
                self._active_task_id = None

            task_id = str(uuid.uuid4())
            created_at = _timestamp()
            task = {
                "taskId": task_id,
                "planId": plan_id,
                "status": "queued",
                "createdAt": created_at,
                "updatedAt": created_at,
                "cancelRequested": False,
                "cancelEvent": threading.Event(),
                "events": [],
                "results": None,
                "error": None,
            }
            self._tasks[task_id] = task
            self._active_task_id = task_id
            self._append_event(
                task_id,
                "plan.created",
                "plan accepted for execution",
                timestamp=plan_record["createdAt"],
            )
            self._append_event(
                task_id,
                "task.status",
                "task queued",
                status="queued",
            )

        self._launch_task(task_id)
        return {
            "ok": True,
            "message": "task queued",
            "data": {
                "taskId": task_id,
                "planId": plan_id,
                "status": "queued",
            },
        }

    @staticmethod
    def _validate_selector(value: Any, label: str) -> None:
        if isinstance(value, str):
            if not value.startswith("object_") or not value[7:].isdigit():
                raise PlanError(f"{label} has an invalid object id")
            return
        if not isinstance(value, dict):
            raise PlanError(f"{label} must be a semantic object selector")
        if set(value) - {"category", "color"}:
            raise PlanError(f"{label} contains unsupported fields")
        category = value.get("category")
        if not isinstance(category, str) or not category or len(category) > 200:
            raise PlanError(f"{label}.category must be a short string")
        color = value.get("color")
        if color is not None and (
            not isinstance(color, str) or not color or len(color) > 200
        ):
            raise PlanError(f"{label}.color must be a short string")

    @classmethod
    def _validate_placement(cls, value: Any, label: str) -> None:
        if not isinstance(value, dict):
            raise PlanError(f"{label} must be an object")
        allowed = {
            "object",
            "target",
            "placement_type",
            "offset",
            "plan_only",
        }
        if set(value) - allowed:
            raise PlanError(f"{label} contains unsupported fields")
        cls._validate_selector(value.get("object"), f"{label}.object")
        cls._validate_selector(value.get("target"), f"{label}.target")
        placement_type = value.get("placement_type", "on_center")
        if placement_type not in {"on_center", "beside"}:
            raise PlanError(f"{label}.placement_type is not allowed")
        offset = value.get("offset")
        if offset is not None:
            if (
                not isinstance(offset, list)
                or len(offset) != 3
                or any(
                    not isinstance(item, (int, float)) or isinstance(item, bool)
                    for item in offset
                )
                or abs(offset[0]) > 200
                or abs(offset[1]) > 200
                or abs(offset[2]) > 100
            ):
                raise PlanError(f"{label}.offset exceeds the approved range")
        plan_only = value.get("plan_only")
        if plan_only is not None and not isinstance(plan_only, bool):
            raise PlanError(f"{label}.plan_only must be boolean")

    @classmethod
    def _validate_harness_execution_plan(cls, plan: Any) -> None:
        steps = getattr(plan, "steps", None)
        if not isinstance(steps, list) or not 1 <= len(steps) <= 12:
            raise PlanError("Harness execution Plan must contain 1-12 Skills")
        motion_budget = 0
        for index, step in enumerate(steps):
            skill = getattr(step, "skill", None)
            args = getattr(step, "args", None)
            if skill not in HARNESS_EXECUTION_SKILLS:
                raise PlanError(
                    f"Harness execution does not allow Skill: {skill}"
                )
            if not isinstance(args, dict):
                raise PlanError(f"Plan step {index + 1} args must be an object")
            if len(json.dumps(args, ensure_ascii=False)) > 8192:
                raise PlanError(f"Plan step {index + 1} args exceed 8 KiB")
            if skill == "perceive_scene":
                if set(args) - {"instruction", "targets", "categories", "selectors"}:
                    raise PlanError("perceive_scene contains unsupported fields")
                instruction = args.get("instruction")
                if instruction is not None and (
                    not isinstance(instruction, str)
                    or not instruction
                    or len(instruction) > 2000
                ):
                    raise PlanError(
                        "perceive_scene.instruction must be a short string"
                    )
                for field in ("targets", "categories", "selectors"):
                    items = args.get(field)
                    if items is not None and (
                        not isinstance(items, list)
                        or len(items) > 32
                        or any(
                            not isinstance(item, (dict, str))
                            if field == "selectors"
                            else not isinstance(item, dict)
                            for item in items
                        )
                    ):
                        raise PlanError(
                            f"perceive_scene.{field} exceeds approved limits"
                        )
            elif skill == "pick_and_place":
                cls._validate_placement(args, "pick_and_place")
                motion_budget += 1
            elif skill == "tidy_table":
                if set(args) - {"instruction", "placements"}:
                    raise PlanError("tidy_table contains unsupported fields")
                placements = args.get("placements")
                if placements is None:
                    motion_budget += 4
                else:
                    if not isinstance(placements, list) or len(placements) > 8:
                        raise PlanError(
                            "tidy_table.placements exceeds 8 actions"
                        )
                    for placement_index, placement in enumerate(placements):
                        cls._validate_placement(
                            placement,
                            f"tidy_table.placements[{placement_index}]",
                        )
                    motion_budget += len(placements)
            elif skill == "safety_reset":
                if set(args) - {"arms", "kind"}:
                    raise PlanError("safety_reset contains unsupported fields")
                if args.get("arms", "both") not in {"left", "right", "both"}:
                    raise PlanError("safety_reset.arms is not allowed")
                if args.get("kind", "safe") != "safe":
                    raise PlanError(
                        "safety_reset.kind must be safe for Harness execution"
                    )
                motion_budget += 1
            if motion_budget > 8:
                raise PlanError(
                    "Harness execution Plan exceeds 8 motion units"
                )

    def _expire_execution_requests_locked(self) -> None:
        now = time.monotonic()
        for request in self._execution_requests.values():
            if (
                request["status"] == "awaiting_approval"
                and now >= request["expiresMonotonic"]
            ):
                request["status"] = "expired"
                request["updatedAt"] = _timestamp()

    def _execution_request_data_locked(
        self,
        request: Dict[str, Any],
    ) -> Dict[str, Any]:
        data = {
            key: copy.deepcopy(value)
            for key, value in request.items()
            if key != "expiresMonotonic"
        }
        task_id = request.get("taskId")
        task = self._tasks.get(task_id) if isinstance(task_id, str) else None
        if task is not None:
            data["task"] = {
                "taskId": task["taskId"],
                "planId": task["planId"],
                "status": task["status"],
                "createdAt": task["createdAt"],
                "updatedAt": task["updatedAt"],
                "cancelRequested": task["cancelRequested"],
                "error": task["error"],
            }
        return _sanitize(data)

    def _request_execution(self, args: Dict[str, Any]) -> Dict[str, Any]:
        plan_id = args.get("planId")
        if not isinstance(plan_id, str) or not plan_id:
            raise PlanError("'planId' is required")
        with self._state_lock:
            self._expire_execution_requests_locked()
            plan_record = self._plans.get(plan_id)
            if plan_record is None:
                raise PlanError(f"unknown planId: {plan_id}")
            if time.monotonic() >= plan_record["expiresMonotonic"]:
                self._plans.pop(plan_id, None)
                raise PlanError(f"expired planId: {plan_id}")
            for existing in self._execution_requests.values():
                if (
                    existing["planId"] == plan_id
                    and existing["status"] == "awaiting_approval"
                ):
                    return {
                        "ok": True,
                        "message": "execution approval already pending",
                        "data": self._execution_request_data_locked(existing),
                    }
            if self._active_task_id is not None:
                active = self._tasks.get(self._active_task_id)
                if active and active["status"] in ACTIVE_TASK_STATUSES:
                    raise PlanError(
                        f"another task is already active: {self._active_task_id}"
                    )
            self._validate_harness_execution_plan(plan_record["plan"])
            ttl_seconds = max(
                30,
                min(
                    900,
                    int(os.environ.get("ROBOT_AGENT_APPROVAL_TTL_S", "600")),
                ),
            )
            remaining_plan_ttl = (
                plan_record["expiresMonotonic"] - time.monotonic()
            )
            expires_in = max(1, min(ttl_seconds, int(remaining_plan_ttl)))
            request_id = str(uuid.uuid4())
            created_at = _timestamp()
            request = {
                "requestId": request_id,
                "planId": plan_id,
                "status": "awaiting_approval",
                "source": "deepseek-harness",
                "instruction": plan_record["instruction"],
                "plan": plan_record["plan"].to_dict(),
                "createdAt": created_at,
                "updatedAt": created_at,
                "expiresInSeconds": expires_in,
                "expiresMonotonic": time.monotonic() + expires_in,
                "taskId": None,
            }
            self._execution_requests[request_id] = request
            return {
                "ok": True,
                "message": "execution awaits per-Plan human approval",
                "data": self._execution_request_data_locked(request),
            }

    def _execution_request_status(
        self,
        args: Dict[str, Any],
    ) -> Dict[str, Any]:
        request_id = args.get("requestId")
        if not isinstance(request_id, str) or not request_id:
            raise PlanError("'requestId' is required")
        with self._state_lock:
            self._expire_execution_requests_locked()
            request = self._execution_requests.get(request_id)
            if request is None:
                raise PlanError(f"unknown requestId: {request_id}")
            return {
                "ok": True,
                "message": "execution request status",
                "data": self._execution_request_data_locked(request),
            }

    def _pending_execution_requests(self) -> Dict[str, Any]:
        with self._state_lock:
            self._expire_execution_requests_locked()
            requests = [
                self._execution_request_data_locked(request)
                for request in self._execution_requests.values()
                if request["status"] == "awaiting_approval"
            ]
        return {
            "ok": True,
            "message": "pending execution approvals",
            "data": {"requests": requests},
        }

    def _approve_execution(self, args: Dict[str, Any]) -> Dict[str, Any]:
        request_id = args.get("requestId")
        confirmation = args.get("confirmation")
        checklist = args.get("checklist")
        if not isinstance(request_id, str) or not request_id:
            raise PlanError("'requestId' is required")
        if confirmation != HARDWARE_CONFIRMATION:
            raise PlanError(
                f"'confirmation' must equal {HARDWARE_CONFIRMATION!r}"
            )
        required_checks = {
            "operatorPresent",
            "workspaceClear",
            "emergencyStopReady",
            "robotInspected",
        }
        if (
            not isinstance(checklist, dict)
            or any(checklist.get(key) is not True for key in required_checks)
        ):
            raise PlanError("all on-site approval checks must be true")
        with self._state_lock:
            self._expire_execution_requests_locked()
            request = self._execution_requests.get(request_id)
            if request is None:
                raise PlanError(f"unknown requestId: {request_id}")
            if request["status"] != "awaiting_approval":
                raise PlanError(
                    f"execution request is not pending: {request['status']}"
                )
            request["status"] = "approving"
            request["updatedAt"] = _timestamp()
            plan_id = request["planId"]
        try:
            task_response = self._execute({
                "planId": plan_id,
                "confirmation": HARDWARE_CONFIRMATION,
            })
        except Exception:
            with self._state_lock:
                request["status"] = "failed"
                request["updatedAt"] = _timestamp()
            raise
        task_data = task_response["data"]
        with self._state_lock:
            request["status"] = "approved"
            request["taskId"] = task_data["taskId"]
            request["updatedAt"] = _timestamp()
            data = self._execution_request_data_locked(request)
        return {
            "ok": True,
            "message": "execution approved and task queued",
            "data": data,
        }

    def _deny_execution(self, args: Dict[str, Any]) -> Dict[str, Any]:
        request_id = args.get("requestId")
        if not isinstance(request_id, str) or not request_id:
            raise PlanError("'requestId' is required")
        with self._state_lock:
            self._expire_execution_requests_locked()
            request = self._execution_requests.get(request_id)
            if request is None:
                raise PlanError(f"unknown requestId: {request_id}")
            if request["status"] != "awaiting_approval":
                raise PlanError(
                    f"execution request is not pending: {request['status']}"
                )
            request["status"] = "denied"
            request["updatedAt"] = _timestamp()
            data = self._execution_request_data_locked(request)
        return {
            "ok": True,
            "message": "execution request denied",
            "data": data,
        }

    def _launch_task(self, task_id: str) -> None:
        threading.Thread(
            target=self._run_task,
            args=(task_id,),
            name=f"robot-task-{task_id}",
            daemon=True,
        ).start()

    def _run_task(self, task_id: str) -> None:
        with self._state_lock:
            task = self._tasks[task_id]
            plan_record = self._plans[task["planId"]]
            cancel_event = task["cancelEvent"]

        if cancel_event.is_set():
            self._finish_task(task_id, "cancelled", "task cancelled while queued")
            return

        self._set_task_status(task_id, "running", "task running")
        try:
            with self._hardware_lock:
                if cancel_event.is_set():
                    raise ExecutionCancelled("task cancelled before bootstrap")
                self._ensure_bootstrapped()
                self.ctx.instruction = plan_record["instruction"]
                self.ctx.output_dir = (
                    Path(plan_record["output_dir"])
                    if plan_record["output_dir"]
                    else self._default_output_dir
                )
                self.ctx.plan_only = False
                self.ctx.scene = {}
                self.ctx.object_id_to_type = {}
                self.ctx.load_categories_from_output()

                results = Executor(self.registry).run(
                    plan_record["plan"],
                    self.ctx,
                    emit=lambda event: self._append_executor_event(task_id, event),
                    cancel_event=cancel_event,
                )

            serialized_results = [result.to_dict() for result in results]
            with self._state_lock:
                self._tasks[task_id]["results"] = serialized_results
            if cancel_event.is_set():
                self._finish_task(task_id, "cancelled", "task cancelled")
            elif len(results) != len(plan_record["plan"].steps) or any(
                not result.ok for result in results
            ):
                last = results[-1] if results else None
                error = (
                    (last.error or last.message)
                    if last is not None
                    else "execution returned no results"
                )
                self._finish_task(task_id, "failed", error)
            else:
                self._finish_task(task_id, "succeeded", "task completed")
        except ExecutionCancelled as error:
            self._finish_task(task_id, "cancelled", str(error))
        except Exception as error:  # noqa: BLE001
            if cancel_event.is_set():
                self._finish_task(task_id, "cancelled", str(error))
            else:
                self._finish_task(task_id, "failed", str(error))

    def _append_executor_event(
        self,
        task_id: str,
        event: Dict[str, Any],
    ) -> None:
        self._append_event(
            task_id,
            str(event.get("type", "task.status")),
            str(event.get("message", "")),
            status=event.get("status"),
            step_id=event.get("stepId"),
        )

    def _append_event(
        self,
        task_id: str,
        event_type: str,
        message: str,
        *,
        status: Optional[str] = None,
        step_id: Optional[str] = None,
        timestamp: Optional[str] = None,
    ) -> None:
        event: Dict[str, Any] = {
            "timestamp": timestamp or _timestamp(),
            "taskId": task_id,
            "type": event_type,
            "message": message,
        }
        if status is not None:
            event["status"] = status
        if step_id is not None:
            event["stepId"] = step_id
        with self._state_lock:
            self._tasks[task_id]["events"].append(event)

    def _set_task_status(
        self,
        task_id: str,
        status: str,
        message: str,
        error: Optional[str] = None,
    ) -> None:
        with self._state_lock:
            task = self._tasks[task_id]
            task["status"] = status
            task["updatedAt"] = _timestamp()
            if error is not None:
                task["error"] = error
        self._append_event(
            task_id,
            "task.status",
            message,
            status=status,
        )

    def _finish_task(self, task_id: str, status: str, message: str) -> None:
        error = message if status == "failed" else None
        self._set_task_status(task_id, status, message, error=error)
        with self._state_lock:
            if self._active_task_id == task_id:
                self._active_task_id = None

    def _public_task(self, task: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "taskId": task["taskId"],
            "planId": task["planId"],
            "status": task["status"],
            "createdAt": task["createdAt"],
            "updatedAt": task["updatedAt"],
            "cancelRequested": task["cancelRequested"],
            "results": _sanitize(task["results"]),
            "error": task["error"],
        }

    def _task_status(self, args: Dict[str, Any]) -> Dict[str, Any]:
        task_id = args.get("taskId")
        if not isinstance(task_id, str) or not task_id:
            raise PlanError("'taskId' is required")
        with self._state_lock:
            task = self._tasks.get(task_id)
            if task is None:
                raise PlanError(f"unknown taskId: {task_id}")
            data = self._public_task(task)
        return {"ok": True, "message": "task status", "data": data}

    def _task_events(self, args: Dict[str, Any]) -> Dict[str, Any]:
        task_id = args.get("taskId")
        cursor = args.get("cursor", 0)
        if not isinstance(task_id, str) or not task_id:
            raise PlanError("'taskId' is required")
        if isinstance(cursor, bool) or not isinstance(cursor, int) or cursor < 0:
            raise PlanError("'cursor' must be a non-negative integer")
        with self._state_lock:
            task = self._tasks.get(task_id)
            if task is None:
                raise PlanError(f"unknown taskId: {task_id}")
            events = [dict(event) for event in task["events"][cursor:]]
            next_cursor = len(task["events"])
        return {
            "ok": True,
            "message": "task events",
            "data": {
                "taskId": task_id,
                "events": events,
                "nextCursor": next_cursor,
            },
        }

    def _cancel(self, args: Dict[str, Any]) -> Dict[str, Any]:
        task_id = args.get("taskId")
        if not isinstance(task_id, str) or not task_id:
            raise PlanError("'taskId' is required")
        with self._state_lock:
            task = self._tasks.get(task_id)
            if task is None:
                raise PlanError(f"unknown taskId: {task_id}")
            if task["status"] in FINAL_TASK_STATUSES:
                status = task["status"]
            else:
                status = task["status"]
                if not task["cancelRequested"]:
                    task["cancelRequested"] = True
                    task["updatedAt"] = _timestamp()
                    task["cancelEvent"].set()
                    self._append_event(
                        task_id,
                        "task.status",
                        "cancellation requested",
                        status="cancelling",
                    )
        return {
            "ok": True,
            "message": "cancellation requested",
            "data": {
                "taskId": task_id,
                "status": status,
                "cancelRequested": task["cancelRequested"],
            },
        }

    def _call(self, args: Dict[str, Any]) -> Dict[str, Any]:
        kind = args.get("kind", "tool")
        name = args.get("name")
        if not name:
            raise PlanError("missing 'name'")
        call_args = args.get("args", {}) or {}
        dry_run = bool(args.get("dry_run", True))
        if not dry_run and args.get("confirmation") != HARDWARE_CONFIRMATION:
            raise PlanError(
                f"'confirmation' must equal {HARDWARE_CONFIRMATION!r}"
            )

        target = self.registry.tool(name) if kind == "tool" else self.registry.skill(name)
        if dry_run:
            with self._hardware_lock:
                output_dir = args.get("output_dir")
                if output_dir:
                    self.ctx.output_dir = Path(output_dir)
                    self.ctx.load_categories_from_output()
                resolved = {
                    key: resolve_object_ref(value, self.ctx)
                    for key, value in call_args.items()
                }
                return {
                    "ok": True,
                    "message": f"[dry-run] {kind} {name} -- hardware not touched",
                    "data": {
                        "dry_run": True,
                        "kind": kind,
                        "name": name,
                        "args": _sanitize(resolved),
                    },
                }

        with self._state_lock:
            active = (
                self._tasks.get(self._active_task_id)
                if self._active_task_id is not None
                else None
            )
            if active and active["status"] in ACTIVE_TASK_STATUSES:
                raise PlanError(
                    f"another task is already active: {self._active_task_id}"
                )
            if self._legacy_call_active:
                raise PlanError("another legacy hardware call is already active")
            self._legacy_call_active = True
        try:
            with self._hardware_lock:
                output_dir = args.get("output_dir")
                if output_dir:
                    self.ctx.output_dir = Path(output_dir)
                    self.ctx.load_categories_from_output()
                resolved = {
                    key: resolve_object_ref(value, self.ctx)
                    for key, value in call_args.items()
                }
                self._ensure_bootstrapped()
                result = target(self.ctx, **resolved)
        finally:
            with self._state_lock:
                self._legacy_call_active = False
        envelope: Dict[str, Any] = {
            "ok": result.ok,
            "message": result.message,
            "data": {
                "dry_run": False,
                "kind": kind,
                "name": name,
                "payload": _sanitize(result.payload),
            },
        }
        if result.error:
            envelope["error"] = result.error
        return envelope

    # ------------------------------------------------------------------ #
    def serve_forever(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        if host != DEFAULT_HOST:
            raise ValueError("robot_agent server must bind to 127.0.0.1")
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(16)
        with self._server_socket_lock:
            self._server_socket = srv
        print(json.dumps({"ok": True, "message": "listening", "port": port}), flush=True)
        try:
            while not self._shutdown.is_set():
                try:
                    conn, _ = srv.accept()
                except OSError:
                    if self._shutdown.is_set():
                        break
                    raise
                threading.Thread(
                    target=self._serve_one,
                    args=(conn,),
                    daemon=True,
                ).start()
        finally:
            with self._server_socket_lock:
                if self._server_socket is srv:
                    self._server_socket = None
            try:
                srv.close()
            except OSError:
                pass
            self.close()

    def _serve_one(self, conn: socket.socket) -> None:
        try:
            stream = conn.makefile("rwb")
            for line in stream:
                if not line.strip():
                    continue
                try:
                    request = json.loads(line.decode("utf-8"))
                    response = self.handle(request)
                except Exception as error:  # noqa: BLE001
                    response = {
                        "ok": False,
                        "message": str(error),
                        "error": str(error),
                    }
                stream.write(
                    (json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8")
                )
                stream.flush()
        except Exception as error:  # noqa: BLE001
            try:
                conn.sendall(
                    (json.dumps({"ok": False, "message": str(error), "error": str(error)}) + "\n").encode("utf-8")
                )
            except Exception:  # noqa: BLE001
                pass
        finally:
            conn.close()


def main() -> int:
    port = DEFAULT_PORT
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    server = RobotAgentServer()
    for signal_number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signal_number, lambda _signum, _frame: server.request_shutdown())
    try:
        server.serve_forever(port=port)
    finally:
        server.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
