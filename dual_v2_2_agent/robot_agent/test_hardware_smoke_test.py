"""Tests for the operator-gated real-hardware smoke runner."""

from __future__ import annotations

import io
from pathlib import Path
import stat
import tempfile
import unittest

from .hardware_smoke_test import (
    AgentConfig,
    HARDWARE_CONFIRMATION,
    HardwareResetSmokeTest,
    HardwareSmokeError,
    load_token,
    operator_approval,
)


READY_HEALTH = {
    "planOnly": False,
    "bootstrapState": "ready",
    "activeTaskId": None,
    "allowedMethods": [
        "health",
        "manifest",
        "plan",
        "execute",
        "task_status",
        "task_events",
        "cancel",
        "call",
    ],
}


class FakeAgent:
    def __init__(self, health=None) -> None:
        self.health = dict(READY_HEALTH if health is None else health)
        self.requests = []

    def request(self, method, args):
        self.requests.append((method, args))
        if method == "health":
            return dict(self.health)
        if method == "manifest":
            return {
                "tools": [
                    {
                        "name": "reset",
                        "description": "reset one arm",
                        "parameters": {},
                    }
                ],
                "skills": [],
            }
        if method == "call":
            call_args = args["args"]
            return {
                "dry_run": bool(args["dry_run"]),
                "kind": "tool",
                "name": args["name"],
                "args": dict(call_args),
                "payload": {},
            }
        raise AssertionError(f"unexpected method: {method}")


class HardwareResetSmokeTestTests(unittest.TestCase):
    def test_preflight_only_uses_health_manifest_and_dry_run(self) -> None:
        fake = FakeAgent()
        runner = HardwareResetSmokeTest(
            request=fake.request,
            check_stack=lambda: "all components running",
        )

        report = runner.preflight("left")

        self.assertEqual(report["action"], "reset_pos")
        self.assertEqual(
            [method for method, _args in fake.requests],
            ["health", "manifest", "call"],
        )
        self.assertTrue(fake.requests[-1][1]["dry_run"])
        self.assertNotIn("confirmation", fake.requests[-1][1])

    def test_execute_is_fixed_to_one_reset_pos_with_confirmation(self) -> None:
        fake = FakeAgent()
        runner = HardwareResetSmokeTest(
            request=fake.request,
            check_stack=lambda: "all components running",
        )

        result = runner.execute("right")

        self.assertFalse(result["dry_run"])
        method, args = fake.requests[-1]
        self.assertEqual(method, "call")
        self.assertEqual(args["kind"], "tool")
        self.assertEqual(args["name"], "reset")
        self.assertEqual(args["args"], {"arms": "right", "kind": "pos"})
        self.assertEqual(args["confirmation"], HARDWARE_CONFIRMATION)

    def test_preflight_fails_closed_for_plan_only_or_active_task(self) -> None:
        plan_only = FakeAgent({**READY_HEALTH, "planOnly": True})
        with self.assertRaisesRegex(HardwareSmokeError, "plan-only"):
            HardwareResetSmokeTest(
                request=plan_only.request,
                check_stack=lambda: "running",
            ).preflight("left")

        active = FakeAgent({**READY_HEALTH, "activeTaskId": "task-1"})
        with self.assertRaisesRegex(HardwareSmokeError, "活动任务"):
            HardwareResetSmokeTest(
                request=active.request,
                check_stack=lambda: "running",
            ).preflight("left")

    def test_preflight_rejects_both_arms(self) -> None:
        with self.assertRaisesRegex(HardwareSmokeError, "left 或 right"):
            HardwareResetSmokeTest(
                request=FakeAgent().request,
                check_stack=lambda: "running",
            ).preflight("both")


class OperatorApprovalTests(unittest.TestCase):
    def test_requires_hardware_environment_confirmation_and_tty(self) -> None:
        with self.assertRaisesRegex(HardwareSmokeError, "export"):
            operator_approval("left", {}, is_tty=True)
        with self.assertRaisesRegex(HardwareSmokeError, "交互式终端"):
            operator_approval(
                "left",
                {"ROBOT_ASSISTANT_HARDWARE_CONFIRM": HARDWARE_CONFIRMATION},
                is_tty=False,
            )

    def test_requires_every_check_and_exact_action_phrase(self) -> None:
        responses = iter(
            [
                "yes",
                "yes",
                "yes",
                "yes",
                "MOVE_LEFT_ARM_TO_RESET_POSITION",
            ]
        )
        output = io.StringIO()

        operator_approval(
            "left",
            {"ROBOT_ASSISTANT_HARDWARE_CONFIRM": HARDWARE_CONFIRMATION},
            input_fn=lambda _prompt: next(responses),
            is_tty=True,
            output=output,
        )

        self.assertIn("物理急停", output.getvalue())

    def test_rejects_incomplete_checklist(self) -> None:
        with self.assertRaisesRegex(HardwareSmokeError, "现场确认未通过"):
            operator_approval(
                "left",
                {"ROBOT_ASSISTANT_HARDWARE_CONFIRM": HARDWARE_CONFIRMATION},
                input_fn=lambda _prompt: "no",
                is_tty=True,
            )


class ConfigurationTests(unittest.TestCase):
    def test_agent_config_accepts_loopback_only(self) -> None:
        AgentConfig(
            host="127.0.0.1",
            port=15555,
            token="x" * 24,
        ).validate()
        with self.assertRaisesRegex(HardwareSmokeError, "非 loopback"):
            AgentConfig(
                host="0.0.0.0",
                port=15555,
                token="x" * 24,
            ).validate()

    def test_load_token_prefers_environment_and_requires_private_file(self) -> None:
        self.assertEqual(
            load_token(
                {"ROBOT_AGENT_TOKEN": "environment-token-value-123"},
                Path("/missing"),
            ),
            "environment-token-value-123",
        )
        with tempfile.TemporaryDirectory() as directory:
            token_file = Path(directory) / "agent.token"
            token_file.write_text("file-token-value-123456789", encoding="utf-8")
            token_file.chmod(stat.S_IRUSR | stat.S_IWUSR)
            self.assertEqual(load_token({}, token_file), "file-token-value-123456789")

            token_file.chmod(0o644)
            with self.assertRaisesRegex(HardwareSmokeError, "权限过宽"):
                load_token({}, token_file)


if __name__ == "__main__":
    unittest.main()
