"""Hardware-free protocol and executor tests."""

from __future__ import annotations

import json
import os
from pathlib import Path
import socket
from types import SimpleNamespace
import tempfile
import threading
import time
import unittest
from unittest import mock
import uuid

from .base import Plan, SkillCall, ToolResult
from .context import write_status
from .executor import ExecutionCancelled, Executor
from .server import HARDWARE_CONFIRMATION, RobotAgentServer
from .skills.start_robot import write_robot_status
from .task_status import write_task_status


class RobotAgentServerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.environment = mock.patch.dict(
            os.environ,
            {
                "ROBOT_AGENT_EAGER_BOOTSTRAP": "0",
                "ROBOT_AGENT_PLAN_ONLY": "0",
                "ROBOT_AGENT_TOKEN": "",
            },
        )
        self.environment.start()
        self.server = RobotAgentServer()
        self.server._launch_task = mock.Mock()

    def tearDown(self) -> None:
        self.environment.stop()

    def create_plan(self, instruction: str = "clean the table") -> dict:
        response = self.server.handle(
            {
                "method": "plan",
                "args": {
                    "instruction": instruction,
                    "output_dir": "/tmp/test-scene",
                },
            }
        )
        self.assertTrue(response["ok"], response)
        return response["data"]

    def execute(self, plan_id: str, **extra_args) -> dict:
        args = {
            "planId": plan_id,
            "confirmation": HARDWARE_CONFIRMATION,
            **extra_args,
        }
        return self.server.handle({"method": "execute", "args": args})

    def test_plan_is_stored_without_bootstrapping(self) -> None:
        data = self.create_plan()

        uuid.UUID(data["planId"])
        record = self.server._plans[data["planId"]]
        self.assertEqual(record["instruction"], "clean the table")
        self.assertEqual(record["output_dir"], "/tmp/test-scene")
        self.assertEqual(record["plan"].to_dict(), data["plan"])
        self.assertFalse(self.server._bootstrap_started)

    def test_execute_uses_only_server_stored_plan(self) -> None:
        data = self.create_plan()
        data["plan"]["instruction"] = "tampered response"
        data["plan"]["steps"].clear()
        malicious_plan = {
            "instruction": "tampered",
            "steps": [{"skill": "start_robot", "args": {}}],
        }

        response = self.execute(data["planId"], plan=malicious_plan)

        self.assertTrue(response["ok"], response)
        task = self.server._tasks[response["data"]["taskId"]]
        self.assertEqual(task["planId"], data["planId"])
        self.assertNotIn("plan", task)
        self.assertEqual(
            self.server._plans[data["planId"]]["plan"].instruction,
            "clean the table",
        )

    def test_execute_requires_exact_confirmation(self) -> None:
        data = self.create_plan()

        missing = self.server.handle(
            {"method": "execute", "args": {"planId": data["planId"]}}
        )
        wrong = self.server.handle(
            {
                "method": "execute",
                "args": {
                    "planId": data["planId"],
                    "confirmation": "yes",
                },
            }
        )

        self.assertFalse(missing["ok"])
        self.assertFalse(wrong["ok"])
        self.assertEqual(self.server._tasks, {})

    def test_execute_rejects_expired_server_plan(self) -> None:
        data = self.create_plan()
        self.server._plans[data["planId"]]["expiresMonotonic"] = (
            time.monotonic() - 1
        )

        response = self.execute(data["planId"])

        self.assertFalse(response["ok"])
        self.assertIn("expired planId", response["error"])
        self.assertEqual(self.server._tasks, {})

    def test_harness_execution_waits_for_per_plan_human_approval(self) -> None:
        data = self.create_plan()
        requested = self.server.handle(
            {
                "method": "request_execution",
                "args": {"planId": data["planId"]},
            }
        )

        self.assertTrue(requested["ok"], requested)
        request = requested["data"]
        self.assertEqual(request["status"], "awaiting_approval")
        self.assertEqual(request["source"], "deepseek-harness")
        self.assertEqual(self.server._tasks, {})
        self.server._launch_task.assert_not_called()

        pending = self.server.handle(
            {"method": "pending_execution_requests", "args": {}}
        )
        self.assertEqual(
            [item["requestId"] for item in pending["data"]["requests"]],
            [request["requestId"]],
        )

        denied_approval = self.server.handle(
            {
                "method": "approve_execution",
                "args": {
                    "requestId": request["requestId"],
                    "confirmation": "yes",
                    "checklist": {
                        "operatorPresent": True,
                        "workspaceClear": True,
                        "emergencyStopReady": True,
                        "robotInspected": True,
                    },
                },
            }
        )
        self.assertFalse(denied_approval["ok"])
        self.assertEqual(self.server._tasks, {})

        approved = self.server.handle(
            {
                "method": "approve_execution",
                "args": {
                    "requestId": request["requestId"],
                    "confirmation": HARDWARE_CONFIRMATION,
                    "checklist": {
                        "operatorPresent": True,
                        "workspaceClear": True,
                        "emergencyStopReady": True,
                        "robotInspected": True,
                    },
                },
            }
        )
        self.assertTrue(approved["ok"], approved)
        self.assertEqual(approved["data"]["status"], "approved")
        task_id = approved["data"]["taskId"]
        self.assertEqual(self.server._tasks[task_id]["status"], "queued")
        self.server._launch_task.assert_called_once_with(task_id)

        status = self.server.handle(
            {
                "method": "execution_request_status",
                "args": {"requestId": request["requestId"]},
            }
        )
        self.assertEqual(status["data"]["task"]["status"], "queued")

    def test_harness_execution_can_be_denied_without_creating_task(self) -> None:
        data = self.create_plan()
        requested = self.server.handle(
            {
                "method": "request_execution",
                "args": {"planId": data["planId"]},
            }
        )
        request_id = requested["data"]["requestId"]
        denied = self.server.handle(
            {
                "method": "deny_execution",
                "args": {"requestId": request_id},
            }
        )

        self.assertTrue(denied["ok"], denied)
        self.assertEqual(denied["data"]["status"], "denied")
        self.assertEqual(self.server._tasks, {})
        self.server._launch_task.assert_not_called()

    def test_harness_execution_rejects_unapproved_skills(self) -> None:
        data = self.create_plan()
        self.server._plans[data["planId"]]["plan"] = Plan(
            instruction="start robot",
            steps=[SkillCall(skill="start_robot", args={})],
        )

        response = self.server.handle(
            {
                "method": "request_execution",
                "args": {"planId": data["planId"]},
            }
        )

        self.assertFalse(response["ok"])
        self.assertIn("does not allow Skill", response["error"])
        self.assertEqual(self.server._execution_requests, {})
        self.assertEqual(self.server._tasks, {})

    def test_legacy_hardware_call_requires_exact_confirmation(self) -> None:
        response = self.server.handle(
            {
                "method": "call",
                "args": {
                    "kind": "skill",
                    "name": "safety_reset",
                    "dry_run": False,
                },
            }
        )

        self.assertFalse(response["ok"])
        self.assertIn("confirmation", response["error"])

    def test_only_one_task_can_be_queued_or_running(self) -> None:
        data = self.create_plan()

        first = self.execute(data["planId"])
        second = self.execute(data["planId"])

        self.assertTrue(first["ok"], first)
        self.assertFalse(second["ok"])
        self.assertIn("already active", second["error"])
        self.assertEqual(len(self.server._tasks), 1)

    def test_event_cursor_status_and_cancel(self) -> None:
        data = self.create_plan()
        execute = self.execute(data["planId"])
        task_id = execute["data"]["taskId"]

        initial = self.server.handle(
            {
                "method": "task_events",
                "args": {"taskId": task_id, "cursor": 0},
            }
        )
        cursor = initial["data"]["nextCursor"]
        self.assertEqual(
            [event["type"] for event in initial["data"]["events"]],
            ["plan.created", "task.status"],
        )
        for event in initial["data"]["events"]:
            self.assertEqual(event["taskId"], task_id)
            self.assertIn("timestamp", event)
            self.assertIn("message", event)

        cancelled = self.server.handle(
            {"method": "cancel", "args": {"taskId": task_id}}
        )
        delta = self.server.handle(
            {
                "method": "task_events",
                "args": {"taskId": task_id, "cursor": cursor},
            }
        )
        status = self.server.handle(
            {"method": "task_status", "args": {"taskId": task_id}}
        )

        self.assertTrue(cancelled["data"]["cancelRequested"])
        self.assertEqual(len(delta["data"]["events"]), 1)
        self.assertEqual(delta["data"]["events"][0]["status"], "cancelling")
        self.assertEqual(delta["data"]["nextCursor"], cursor + 1)
        self.assertTrue(status["data"]["cancelRequested"])
        self.assertTrue(self.server._tasks[task_id]["cancelEvent"].is_set())

    def test_configured_token_is_required(self) -> None:
        with mock.patch.dict(
            os.environ,
            {
                "ROBOT_AGENT_EAGER_BOOTSTRAP": "0",
                "ROBOT_AGENT_TOKEN": "secret-token",
            },
        ):
            server = RobotAgentServer()

        denied = server.handle({"method": "health", "args": {}})
        allowed = server.handle(
            {"method": "health", "args": {}, "token": "secret-token"}
        )

        self.assertFalse(denied["ok"])
        self.assertEqual(denied["error"], "unauthorized")
        self.assertTrue(allowed["ok"])
        self.assertEqual(allowed["data"]["bootstrapState"], "not_started")

    def test_health_exposes_fatal_bootstrap_error(self) -> None:
        self.server._boot_error = RuntimeError("No RealSense devices found")
        self.server._ready.set()

        health = self.server.handle({"method": "health", "args": {}})

        self.assertTrue(health["ok"], health)
        self.assertEqual(health["data"]["bootstrapState"], "error")
        self.assertEqual(
            health["data"]["bootstrapError"],
            "No RealSense devices found",
        )

    def test_plan_only_mode_never_bootstraps_and_rejects_execution_plane(self) -> None:
        with mock.patch.dict(
            os.environ,
            {
                "ROBOT_AGENT_EAGER_BOOTSTRAP": "1",
                "ROBOT_AGENT_PLAN_ONLY": "1",
                "ROBOT_AGENT_TOKEN": "",
            },
        ):
            server = RobotAgentServer()

        health = server.handle({"method": "health", "args": {}})
        manifest = server.handle({"method": "manifest", "args": {}})
        plan = server.handle(
            {
                "method": "plan",
                "args": {
                    "instruction": "clean the table",
                    "output_dir": "/tmp/test-scene",
                },
            }
        )
        execute = server.handle(
            {
                "method": "execute",
                "args": {
                    "planId": plan["data"]["planId"],
                    "confirmation": HARDWARE_CONFIRMATION,
                },
            }
        )
        request_execution = server.handle(
            {
                "method": "request_execution",
                "args": {"planId": plan["data"]["planId"]},
            }
        )
        legacy_call = server.handle(
            {
                "method": "call",
                "args": {
                    "kind": "skill",
                    "name": "safety_reset",
                    "dry_run": True,
                },
            }
        )

        self.assertTrue(health["ok"], health)
        self.assertTrue(health["data"]["planOnly"])
        self.assertFalse(health["data"]["eagerBootstrap"])
        self.assertEqual(health["data"]["bootstrapState"], "not_started")
        self.assertEqual(
            health["data"]["allowedMethods"],
            ["health", "manifest", "plan"],
        )
        self.assertTrue(manifest["ok"], manifest)
        self.assertTrue(plan["ok"], plan)
        self.assertFalse(execute["ok"])
        self.assertIn("plan-only mode", execute["error"])
        self.assertFalse(request_execution["ok"])
        self.assertIn("plan-only mode", request_execution["error"])
        self.assertFalse(legacy_call["ok"])
        self.assertFalse(server._bootstrap_started)
        self.assertEqual(server._tasks, {})

    def test_ndjson_connection_accepts_multiple_requests(self) -> None:
        server_socket, client_socket = socket.socketpair()
        worker = threading.Thread(
            target=self.server._serve_one,
            args=(server_socket,),
        )
        worker.start()
        try:
            client_socket.sendall(
                b'{"method":"health","args":{}}\n'
                b'{"method":"manifest","args":{}}\n'
            )
            client_socket.shutdown(socket.SHUT_WR)
            stream = client_socket.makefile("rb")
            responses = [json.loads(stream.readline()), json.loads(stream.readline())]
        finally:
            client_socket.close()
            worker.join(timeout=2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(
            [response["message"] for response in responses],
            ["healthy", "manifest"],
        )

    def test_shutdown_closes_listener_and_cleans_context_once(self) -> None:
        self.server.ctx.cleanup = mock.Mock()
        worker = threading.Thread(
            target=self.server.serve_forever,
            kwargs={"port": 0},
        )
        worker.start()
        deadline = time.monotonic() + 2
        while self.server._server_socket is None and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertIsNotNone(self.server._server_socket)
        self.server.request_shutdown()
        worker.join(timeout=2)
        self.server.close()

        self.assertFalse(worker.is_alive())
        self.server.ctx.cleanup.assert_called_once_with()

    def test_shutdown_requests_active_task_cancellation(self) -> None:
        data = self.create_plan()
        execute = self.execute(data["planId"])
        task = self.server._tasks[execute["data"]["taskId"]]

        self.server.request_shutdown()

        self.assertTrue(task["cancelRequested"])
        self.assertTrue(task["cancelEvent"].is_set())

    def test_status_directory_is_environment_configurable(self) -> None:
        with tempfile.TemporaryDirectory() as status_dir:
            with mock.patch.dict(
                os.environ,
                {"ROBOT_AGENT_STATUS_DIR": status_dir},
            ):
                write_status("loading")
                write_task_status("loading")
                write_robot_status("loading")

            self.assertEqual(
                {path.name for path in Path(status_dir).iterdir()},
                {
                    ".robot_agent_status.json",
                    ".robot_task_status.json",
                    ".robot_startup_status.json",
                },
            )

    def test_worker_emits_required_lifecycle_without_hardware(self) -> None:
        data = self.create_plan()
        self.server._plans[data["planId"]]["plan"] = Plan(
            instruction="test",
            steps=[
                SkillCall(skill="one", ref="one"),
                SkillCall(skill="two", ref="two", constraints={"depends_on": ["one"]}),
            ],
        )
        self.server.registry = _FakeRegistry()
        self.server._ensure_bootstrapped = mock.Mock()
        execute = self.execute(data["planId"])
        task_id = execute["data"]["taskId"]

        self.server._run_task(task_id)

        status = self.server._task_status({"taskId": task_id})
        events = self.server._task_events({"taskId": task_id, "cursor": 0})
        event_types = [event["type"] for event in events["data"]["events"]]
        self.assertEqual(status["data"]["status"], "succeeded")
        self.assertIn("plan.created", event_types)
        self.assertIn("task.status", event_types)
        self.assertIn("step.started", event_types)
        self.assertIn("step.completed", event_types)


class _FakeRegistry:
    def __init__(self) -> None:
        self._skills = {
            "one": lambda _ctx: ToolResult.success("one complete"),
            "two": lambda _ctx: ToolResult.success("two complete"),
        }

    def has_skill(self, name: str) -> bool:
        return name in self._skills

    @property
    def skills(self):
        return [SimpleNamespace(name=name) for name in self._skills]

    def skill(self, name: str):
        return self._skills[name]


class ExecutorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.registry = _FakeRegistry()
        self.ctx = SimpleNamespace(cancel_event=None, emit=None)
        self.plan = Plan(
            instruction="test",
            steps=[
                SkillCall(skill="one", ref="one"),
                SkillCall(
                    skill="two",
                    ref="two",
                    constraints={"depends_on": ["one"]},
                ),
            ],
        )

    def test_executor_emits_step_lifecycle(self) -> None:
        events = []

        results = Executor(self.registry).run(
            self.plan,
            self.ctx,
            emit=events.append,
            cancel_event=threading.Event(),
        )

        self.assertEqual(len(results), 2)
        self.assertEqual(
            [event["type"] for event in events],
            [
                "step.started",
                "step.completed",
                "step.started",
                "step.completed",
            ],
        )

    def test_executor_checks_cancel_at_skill_boundary(self) -> None:
        cancel_event = threading.Event()
        cancel_event.set()
        events = []

        with self.assertRaises(ExecutionCancelled):
            Executor(self.registry).run(
                self.plan,
                self.ctx,
                emit=events.append,
                cancel_event=cancel_event,
            )

        self.assertEqual(events[0]["type"], "step.failed")
        self.assertEqual(events[0]["status"], "cancelled")


if __name__ == "__main__":
    unittest.main()
