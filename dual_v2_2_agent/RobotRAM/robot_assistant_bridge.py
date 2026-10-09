#!/usr/bin/env python3
"""Loopback-only atomic-operation bridge for Robot Assistant."""

from __future__ import annotations

import argparse
import json
import os
import secrets
import signal
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any


HARDWARE_CONFIRMATION = "ENABLE_DUAL_V2_2_HARDWARE"
MAX_REQUEST_BYTES = 16 * 1024
OPERATIONS = {"approach", "grasp", "place", "home"}
OBJECTS = {"cup", "bowl", "bread", "fork", "scene-object"}
ACTION_GROUPS = {
    "approach": {
        "move_above_grasp",
        "move_down_to_grasp",
        "close_to_object_center",
    },
    "grasp": {"close_gripper", "lift_object"},
    "place": {
        "move_to_target",
        "adjust_orientation_for_place",
        "move_down_to_place",
        "open_gripper",
        "move_up_after_release",
    },
    "home": {"reset_all"},
}


class BridgeError(RuntimeError):
    """Request error with an HTTP status code."""

    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.status = status


def parse_operation_request(value: Any) -> tuple[str, dict[str, str]]:
    if not isinstance(value, dict) or set(value) != {"operation", "input"}:
        raise BridgeError("request must contain only operation and input")
    operation = value["operation"]
    skill_input = value["input"]
    if (
        not isinstance(operation, str)
        or operation not in OPERATIONS
        or not isinstance(skill_input, dict)
    ):
        raise BridgeError("operation or input is invalid")
    if set(skill_input).difference({"object", "destination"}):
        raise BridgeError("input contains unsupported fields")
    for key, item in skill_input.items():
        if not isinstance(item, str) or not item:
            raise BridgeError(f"input.{key} must be a non-empty string")
    object_name = skill_input.get("object")
    if operation != "home" and object_name not in OBJECTS:
        raise BridgeError(f"unsupported object: {object_name}")
    return operation, skill_input


class OperationState:
    """Enforce approach -> grasp -> place sequences before backend dispatch."""

    def __init__(self) -> None:
        self.active_object: str | None = None
        self.expected = "approach"

    def validate(self, operation: str, skill_input: dict[str, str]) -> None:
        object_name = skill_input.get("object")
        if operation == "home":
            if self.active_object is not None:
                raise BridgeError("place the active object before home", 409)
            return
        if operation != self.expected:
            raise BridgeError(
                f"expected {self.expected}, received {operation}",
                409,
            )
        if operation == "approach":
            self.active_object = object_name
        elif object_name != self.active_object:
            raise BridgeError("operation object does not match active object", 409)

    def commit(self, operation: str) -> None:
        if operation == "approach":
            self.expected = "grasp"
        elif operation == "grasp":
            self.expected = "place"
        elif operation == "place":
            self.active_object = None
            self.expected = "approach"

    def recover(self) -> None:
        self.active_object = None
        self.expected = "approach"


class DryRunBackend:
    """Non-hardware backend used for protocol and UI development."""

    def execute(self, operation: str, skill_input: dict[str, str]) -> str:
        target = skill_input.get("object", "robot")
        return f"dry-run {operation} {target}"

    def close(self) -> None:
        return


class HardwareBackend:
    """Execute preplanned RobotRAM task fragments through the existing controller."""

    def __init__(
        self,
        robotram_root: Path,
        task_manifest: Path,
        categories_manifest: Path,
    ) -> None:
        scripts_root = robotram_root / "scripts"
        sys.path[:0] = [str(scripts_root), str(robotram_root)]
        from controllor.hkstp_arm import RobotArm
        from step5_execute import (
            execute_single_task,
            load_object_categories,
            recover_to_reset_after_failure,
        )

        with task_manifest.open(encoding="utf-8") as task_file:
            tasks = json.load(task_file)
        with categories_manifest.open(encoding="utf-8") as category_file:
            categories = json.load(category_file)
        if not isinstance(tasks, list) or not isinstance(categories, list):
            raise ValueError("RobotRAM manifests must contain JSON arrays")

        self.tasks = tasks
        self.object_categories = load_object_categories(categories)
        self.execute_single_task = execute_single_task
        self.recover_after_failure = recover_to_reset_after_failure
        self.robot = RobotArm(
            target_host="127.0.0.1",
            target_local_port=10165,
            target_remote_port=10166,
        )
        self.robot.initialize_target_command()
        self.completed_objects: set[str] = set()
        self.active_object_id: str | None = None
        self.did_first_reset = False

    def _select_object_id(self, category: str) -> str:
        candidates = [
            object_id
            for object_id, object_category in self.object_categories.items()
            if object_category == category and object_id not in self.completed_objects
        ]
        if not candidates:
            raise BridgeError(f"no unprocessed RobotRAM object for {category}", 409)
        return sorted(candidates)[0]

    def _run_tasks(self, tasks: list[dict[str, Any]]) -> int:
        if not tasks:
            raise BridgeError("preplanned task fragment is empty", 409)
        for index, task in enumerate(tasks):
            _, result = self.execute_single_task(
                self.robot,
                task,
                index,
                self.object_categories,
            )
            if not result:
                self.recover_after_failure(self.robot, task)
                raise BridgeError(
                    f"RobotRAM action failed: {task.get('action', 'unknown')}",
                    502,
                )
        return len(tasks)

    def execute(self, operation: str, skill_input: dict[str, str]) -> str:
        if operation == "home":
            home_tasks = [
                task for task in self.tasks
                if task.get("action") in ACTION_GROUPS["home"]
            ]
            if home_tasks:
                count = self._run_tasks([home_tasks[-1]])
            else:
                if not self.robot.control_joint_position(
                    left_pos=True,
                    right_pos=True,
                    action="robot_assistant_home",
                ):
                    raise BridgeError("RobotRAM home operation failed", 502)
                count = 1
            return f"RobotRAM home completed ({count} action)"

        object_name = skill_input["object"]
        if operation == "approach":
            self.active_object_id = self._select_object_id(object_name)
            if not self.did_first_reset:
                reset_tasks = [
                    task for task in self.tasks
                    if task.get("action") == "first_reset"
                ]
                if reset_tasks:
                    self._run_tasks([reset_tasks[0]])
                self.did_first_reset = True
        if self.active_object_id is None:
            raise BridgeError("no active RobotRAM object", 409)

        fragment = [
            task for task in self.tasks
            if task.get("object") == self.active_object_id
            and task.get("action") in ACTION_GROUPS[operation]
        ]
        count = self._run_tasks(fragment)
        if operation == "place":
            self.completed_objects.add(self.active_object_id)
            self.active_object_id = None
        return (
            f"RobotRAM {operation} completed for {object_name} "
            f"({count} actions)"
        )

    def close(self) -> None:
        self.robot.shutdown()


class BridgeApplication:
    """Serialize state validation, execution, and commit."""

    def __init__(self, backend: DryRunBackend | HardwareBackend) -> None:
        self.backend = backend
        self.state = OperationState()
        self.lock = threading.Lock()

    def execute(self, operation: str, skill_input: dict[str, str]) -> str:
        with self.lock:
            self.state.validate(operation, skill_input)
            try:
                detail = self.backend.execute(operation, skill_input)
            except Exception:
                self.state.recover()
                raise
            self.state.commit(operation)
            return detail

    def close(self) -> None:
        self.backend.close()


def make_handler(application: BridgeApplication, token: str):
    class BridgeHandler(BaseHTTPRequestHandler):
        server_version = "RobotRAMBridge/0.1"

        def _json(self, status: int, payload: dict[str, Any]) -> None:
            encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("content-type", "application/json; charset=utf-8")
            self.send_header("content-length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

        def do_GET(self) -> None:
            if self.path != "/health":
                self._json(404, {"ok": False, "detail": "not found"})
                return
            self._json(200, {"ok": True, "detail": "RobotRAM bridge ready"})

        def do_POST(self) -> None:
            if self.path != "/v1/operations":
                self._json(404, {"ok": False, "detail": "not found"})
                return
            supplied_token = self.headers.get("x-robot-bridge-token", "")
            if not secrets.compare_digest(supplied_token, token):
                self._json(401, {"ok": False, "detail": "unauthorized"})
                return
            try:
                content_length = int(self.headers.get("content-length", "0"))
                if content_length < 1 or content_length > MAX_REQUEST_BYTES:
                    raise BridgeError("invalid request size", 413)
                payload = json.loads(self.rfile.read(content_length))
                operation, skill_input = parse_operation_request(payload)
                detail = application.execute(operation, skill_input)
                self._json(200, {"ok": True, "detail": detail})
            except BridgeError as error:
                self._json(error.status, {"ok": False, "detail": str(error)})
            except (json.JSONDecodeError, UnicodeDecodeError, ValueError) as error:
                self._json(400, {"ok": False, "detail": str(error)})
            except Exception as error:
                self._json(500, {"ok": False, "detail": str(error)})

        def log_message(self, format_string: str, *args: Any) -> None:
            print(f"[RobotRAM Bridge] {self.address_string()} {format_string % args}")

    return BridgeHandler


def parse_args() -> argparse.Namespace:
    robotram_root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("dry-run", "hardware"), default="dry-run")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument(
        "--token",
        default=os.environ.get("ROBOTRAM_BRIDGE_TOKEN", ""),
    )
    parser.add_argument(
        "--hardware-confirm",
        default=os.environ.get("ROBOT_ASSISTANT_HARDWARE_CONFIRM", ""),
    )
    parser.add_argument(
        "--task-manifest",
        type=Path,
        default=robotram_root / "output" / "scene_57" / "detailed_subtasks_manual.json",
    )
    parser.add_argument(
        "--categories-manifest",
        type=Path,
        default=robotram_root / "output" / "scene_57" / "vlm_categories.json",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.host != "127.0.0.1":
        raise ValueError("RobotRAM bridge host must be 127.0.0.1")
    if not 1 <= args.port <= 65535:
        raise ValueError("RobotRAM bridge port must be in 1..65535")
    if len(args.token) < 24:
        raise ValueError("ROBOTRAM_BRIDGE_TOKEN must contain at least 24 characters")

    if args.mode == "hardware":
        if args.hardware_confirm != HARDWARE_CONFIRMATION:
            raise ValueError(
                f"hardware mode requires {HARDWARE_CONFIRMATION}"
            )
        backend: DryRunBackend | HardwareBackend = HardwareBackend(
            Path(__file__).resolve().parent,
            args.task_manifest.expanduser().resolve(),
            args.categories_manifest.expanduser().resolve(),
        )
    else:
        backend = DryRunBackend()

    application = BridgeApplication(backend)
    server = HTTPServer((args.host, args.port), make_handler(application, args.token))

    def request_shutdown(_signum: int, _frame: Any) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    print(
        f"RobotRAM bridge listening on http://{args.host}:{args.port} "
        f"mode={args.mode}"
    )
    try:
        server.serve_forever()
    finally:
        server.server_close()
        application.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
