#!/usr/bin/env python3
"""Operator-gated real-hardware smoke test through the persistent Python Agent.

The only physical action this program can request is a single-arm ``reset_pos``.
It never imports RobotRAM or a robot driver: the already supervised Python
Agent remains the sole hardware owner.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
import json
import os
from pathlib import Path
import socket
import stat
import subprocess
import sys
from typing import Any, Callable, Dict, Mapping, Optional, Sequence


HARDWARE_CONFIRMATION = "ENABLE_DUAL_V2_2_HARDWARE"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 15555
DEFAULT_TIMEOUT_S = 300.0
DEFAULT_MAX_RESPONSE_BYTES = 1024 * 1024
ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_LAUNCHER = ROOT_DIR / "launch_robot_assistant.sh"
DEFAULT_TOKEN_FILE = ROOT_DIR / ".robot_assistant_runtime" / "agent.token"
CHECKLIST = (
    "机械臂运动范围内无人、无障碍物",
    "急停按钮可立即触达并已确认有效",
    "机器人当前未抓持物体",
    "现场只有一名操作员负责下发本次动作",
)

JsonObject = Dict[str, Any]
RequestFunction = Callable[[str, JsonObject], JsonObject]
StackCheckFunction = Callable[[], str]


class HardwareSmokeError(RuntimeError):
    """A failed safety gate, protocol check, or hardware smoke action."""


@dataclass(frozen=True)
class AgentConfig:
    """Connection details for the already supervised Python Agent."""

    host: str
    port: int
    token: str
    timeout_s: float = DEFAULT_TIMEOUT_S
    max_response_bytes: int = DEFAULT_MAX_RESPONSE_BYTES

    def validate(self) -> None:
        try:
            address = ipaddress.ip_address(self.host)
        except ValueError as error:
            raise HardwareSmokeError(
                "ROBOT_AGENT_HOST 必须是数值 loopback 地址"
            ) from error
        if not address.is_loopback:
            raise HardwareSmokeError("拒绝连接非 loopback Python Agent")
        if not 1 <= self.port <= 65535:
            raise HardwareSmokeError("ROBOT_AGENT_PORT 必须在 1..65535")
        if len(self.token) < 24:
            raise HardwareSmokeError("Robot Agent token 缺失或短于 24 字符")
        if self.timeout_s <= 0:
            raise HardwareSmokeError("timeout 必须大于 0")
        if self.max_response_bytes < 1024:
            raise HardwareSmokeError("max_response_bytes 不能小于 1024")


def load_token(
    environment: Mapping[str, str],
    token_file: Path,
) -> str:
    """Load the shared token without printing it."""
    configured = environment.get("ROBOT_AGENT_TOKEN", "").strip()
    if configured:
        return configured
    try:
        mode = stat.S_IMODE(token_file.stat().st_mode)
        if mode & 0o077:
            raise HardwareSmokeError(
                f"token 文件权限过宽（要求 0600）: {token_file}"
            )
        token = token_file.read_text(encoding="utf-8").strip()
    except FileNotFoundError as error:
        raise HardwareSmokeError(
            "找不到 Robot Agent token；请先启动 launch_robot_assistant.sh"
        ) from error
    except OSError as error:
        raise HardwareSmokeError(f"无法读取 token 文件: {error}") from error
    return token


def request_agent(
    config: AgentConfig,
    method: str,
    args: JsonObject,
) -> JsonObject:
    """Send one bounded NDJSON request to the loopback Python Agent."""
    config.validate()
    request = {
        "method": method,
        "args": args,
        "token": config.token,
    }
    payload = (json.dumps(request, ensure_ascii=False) + "\n").encode("utf-8")
    received = bytearray()
    try:
        with socket.create_connection(
            (config.host, config.port),
            timeout=config.timeout_s,
        ) as connection:
            connection.settimeout(config.timeout_s)
            connection.sendall(payload)
            connection.shutdown(socket.SHUT_WR)
            while True:
                chunk = connection.recv(65536)
                if not chunk:
                    break
                received.extend(chunk)
                if len(received) > config.max_response_bytes:
                    raise HardwareSmokeError("Python Agent 响应超过大小上限")
    except HardwareSmokeError:
        raise
    except (OSError, TimeoutError) as error:
        raise HardwareSmokeError(
            f"无法连接 Python Agent {config.host}:{config.port}: {error}"
        ) from error

    lines = [line for line in bytes(received).splitlines() if line.strip()]
    if len(lines) != 1:
        raise HardwareSmokeError("Python Agent 必须返回且只能返回一条 NDJSON")
    try:
        envelope = json.loads(lines[0].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise HardwareSmokeError("Python Agent 返回了无效 JSON") from error
    if not isinstance(envelope, dict) or not isinstance(envelope.get("ok"), bool):
        raise HardwareSmokeError("Python Agent 响应信封无效")
    if not envelope["ok"]:
        detail = envelope.get("error") or envelope.get("message") or "未知错误"
        raise HardwareSmokeError(f"Python Agent 拒绝 {method}: {detail}")
    data = envelope.get("data")
    if not isinstance(data, dict):
        raise HardwareSmokeError(f"Python Agent {method} 缺少 object data")
    return data


def check_supervised_stack(
    launcher: Path = DEFAULT_LAUNCHER,
    timeout_s: float = 20.0,
) -> str:
    """Require Supervisor, GUI, Agent, RT, Upper, and Planner to be running."""
    if not launcher.is_file() or not os.access(launcher, os.X_OK):
        raise HardwareSmokeError(f"找不到可执行 Supervisor: {launcher}")
    try:
        result = subprocess.run(
            [str(launcher), "status"],
            cwd=str(ROOT_DIR),
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise HardwareSmokeError(f"无法检查 Robot Assistant 状态: {error}") from error
    output = "\n".join(
        part.strip() for part in (result.stdout, result.stderr) if part.strip()
    )
    if result.returncode != 0:
        raise HardwareSmokeError(
            "Robot Assistant 完整监督栈未就绪"
            + (f":\n{output}" if output else "")
        )
    return output


class HardwareResetSmokeTest:
    """Constrained real-machine test for one arm's verified reset position."""

    def __init__(
        self,
        request: RequestFunction,
        check_stack: StackCheckFunction,
    ) -> None:
        self._request = request
        self._check_stack = check_stack

    def preflight(self, arm: str) -> JsonObject:
        """Verify every software gate and preview the exact action."""
        _validate_arm(arm)
        stack_status = self._check_stack()
        health = self._request("health", {})
        if health.get("planOnly") is not False:
            raise HardwareSmokeError(
                "Python Agent 仍处于 plan-only 或状态不可验证，禁止真机动作"
            )
        if health.get("bootstrapState") != "ready":
            raise HardwareSmokeError(
                f"Python Agent bootstrap 未就绪: {health.get('bootstrapState')!r}"
            )
        if health.get("activeTaskId") is not None:
            raise HardwareSmokeError(
                f"存在活动任务: {health.get('activeTaskId')!r}"
            )
        allowed = health.get("allowedMethods")
        if not isinstance(allowed, list) or "call" not in allowed:
            raise HardwareSmokeError("Python Agent 未明确开放受确认的 call 路径")

        manifest = self._request("manifest", {})
        tools = manifest.get("tools")
        if not isinstance(tools, list) or not any(
            isinstance(tool, dict) and tool.get("name") == "reset"
            for tool in tools
        ):
            raise HardwareSmokeError("Python Agent manifest 中没有 reset Tool")

        action_args = {"arms": arm, "kind": "pos"}
        preview = self._request(
            "call",
            {
                "kind": "tool",
                "name": "reset",
                "args": action_args,
                "dry_run": True,
            },
        )
        if (
            preview.get("dry_run") is not True
            or preview.get("name") != "reset"
            or preview.get("args") != action_args
        ):
            raise HardwareSmokeError("reset dry-run 预览与固定动作不一致")
        return {
            "arm": arm,
            "action": "reset_pos",
            "stack": stack_status,
            "health": health,
            "preview": preview,
        }

    def execute(self, arm: str) -> JsonObject:
        """Perform the one real reset action after external operator approval."""
        _validate_arm(arm)
        result = self._request(
            "call",
            {
                "kind": "tool",
                "name": "reset",
                "args": {"arms": arm, "kind": "pos"},
                "dry_run": False,
                "confirmation": HARDWARE_CONFIRMATION,
            },
        )
        if result.get("dry_run") is not False or result.get("name") != "reset":
            raise HardwareSmokeError("Python Agent 真机结果信封无效")
        return result


def _validate_arm(arm: str) -> None:
    if arm not in {"left", "right"}:
        raise HardwareSmokeError("真机冒烟测试只允许 left 或 right 单臂")


def operator_approval(
    arm: str,
    environment: Mapping[str, str],
    *,
    input_fn: Callable[[str], str] = input,
    is_tty: Optional[bool] = None,
    output: Any = sys.stdout,
) -> None:
    """Require environment enablement, a TTY checklist, and an action phrase."""
    if environment.get("ROBOT_ASSISTANT_HARDWARE_CONFIRM") != HARDWARE_CONFIRMATION:
        raise HardwareSmokeError(
            f"必须先 export ROBOT_ASSISTANT_HARDWARE_CONFIRM={HARDWARE_CONFIRMATION}"
        )
    if is_tty is None:
        is_tty = sys.stdin.isatty()
    if not is_tty:
        raise HardwareSmokeError("真机动作必须由交互式终端确认，拒绝管道或 CI")

    print("\n真机动作：单次 %s 臂回安全复位位。" % arm, file=output)
    print("Ctrl-C/关闭终端不等于急停；异常时必须按物理急停按钮。", file=output)
    for item in CHECKLIST:
        answer = input_fn(f"[现场确认] {item}？输入 yes: ").strip().lower()
        if answer != "yes":
            raise HardwareSmokeError(f"现场确认未通过: {item}")

    phrase = f"MOVE_{arm.upper()}_ARM_TO_RESET_POSITION"
    entered = input_fn(f"输入 {phrase} 批准一次动作: ").strip()
    if entered != phrase:
        raise HardwareSmokeError("一次性动作确认短语不匹配")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "通过受监督 Python Agent 对单臂执行一次真实 reset_pos。"
            "默认只做预检，不移动机器人。"
        )
    )
    parser.add_argument("--arm", choices=("left", "right"), default="left")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="在全部人工确认后真实移动所选机械臂",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("ROBOT_AGENT_HOST", DEFAULT_HOST),
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("ROBOT_AGENT_PORT", str(DEFAULT_PORT))),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help="一次 Agent 请求的超时秒数",
    )
    parser.add_argument(
        "--token-file",
        type=Path,
        default=DEFAULT_TOKEN_FILE,
    )
    parser.add_argument(
        "--launcher",
        type=Path,
        default=DEFAULT_LAUNCHER,
    )
    return parser


def _print_report(report: JsonObject, output: Any) -> None:
    health = report["health"]
    preview = report["preview"]
    print("真机预检通过：", file=output)
    print(f"  arm: {report['arm']}", file=output)
    print(f"  action: {report['action']}", file=output)
    print(f"  bootstrapState: {health.get('bootstrapState')}", file=output)
    print(f"  activeTaskId: {health.get('activeTaskId')}", file=output)
    print(f"  preview: {json.dumps(preview, ensure_ascii=False)}", file=output)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        token = load_token(os.environ, args.token_file)
        config = AgentConfig(
            host=args.host,
            port=args.port,
            token=token,
            timeout_s=args.timeout,
        )
        config.validate()
        runner = HardwareResetSmokeTest(
            request=lambda method, payload: request_agent(
                config,
                method,
                payload,
            ),
            check_stack=lambda: check_supervised_stack(args.launcher),
        )

        report = runner.preflight(args.arm)
        _print_report(report, sys.stdout)
        if not args.execute:
            print(
                "\n未移动机器人。确认现场安全后，加 --execute 重新运行。",
                file=sys.stdout,
            )
            return 0

        operator_approval(args.arm, os.environ)
        # Revalidate the complete stack and active-task state after the operator
        # may have spent time reading and confirming the physical checklist.
        runner.preflight(args.arm)
        result = runner.execute(args.arm)
        print(
            "\n真机 reset_pos 完成："
            + json.dumps(result, ensure_ascii=False),
            file=sys.stdout,
        )
        return 0
    except KeyboardInterrupt:
        print(
            "\n通信中断不代表机器人已停止；如动作已开始，请立即使用物理急停。",
            file=sys.stderr,
        )
        return 130
    except HardwareSmokeError as error:
        print(f"真机测试已拒绝/失败: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
