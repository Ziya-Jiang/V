"""``start_robot`` -- the robot startup SKILL.

Launches the full control stack through the workspace's canonical
``launch_dual_v2_2.sh``:

1. rt_control        (real-time control, sudo + fault-recovery flags)
2. upper_software    (headless ``--headless-auto``: hardware mode + auto Remote Connect/Start)
3. upper_planner     (CuRobo motion planner, conda ``curobo`` env)

Each stage is announced through a status file so the GUI's *blue* robot-startup
progress bar can track it live.
"""

from __future__ import annotations

import json
import os
import subprocess
from typing import Any, Dict

from ..base import Skill, ToolResult, object_schema

LAUNCH_SCRIPT = "/mnt/ssd/user/V/dual_v2_2_control/launch_dual_v2_2.sh"
LAUNCH_DIR = "/mnt/ssd/user/V/dual_v2_2_control"

ROBOT_STATUS_PATH = "/mnt/ssd/user/V/dual_v2_2_agent (copy)/.robot_startup_status.json"

ROBOT_STAGES = (
    ("rt_control", "rt_control 实时控制"),
    ("upper_software", "upper_software 硬件模式连接"),
    ("planner", "upper_planner 运动规划"),
)

# The launch script's own log lines that signal a stage finished.
STAGE_MARKERS = {
    "rt_control": "rt_control 已就绪",
    "upper_software": "upper_software 硬件模式",
    "planner": "upper_planner 已就绪",
}


def write_robot_status(state, current=0, label="", detail="", error=None) -> None:
    payload: Dict[str, Any] = {
        "state": state,           # "loading" | "ready" | "error"
        "current": int(current),
        "total": len(ROBOT_STAGES),
        "label": label,
        "detail": detail,
        "stages": [s[1] for s in ROBOT_STAGES],
        "error": error,
    }
    try:
        with open(ROBOT_STATUS_PATH, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
    except Exception:  # noqa: BLE001
        pass


def make_start_robot(registry) -> Skill:
    def run(ctx, monitor_after_start: bool = False) -> ToolResult:
        env = dict(os.environ)
        env["CUARM_MONITOR_AFTER_START"] = "1" if monitor_after_start else "0"

        # Announce each stage as the script's output reports it finished.
        proc = subprocess.Popen(
            ["bash", LAUNCH_SCRIPT, "start"],
            cwd=LAUNCH_DIR,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        completed = 0
        for line in proc.stdout or ():
            line = line.rstrip()
            for key, marker in STAGE_MARKERS.items():
                if marker in line and ROBOT_STAGES:
                    idx = [s[0] for s in ROBOT_STAGES].index(key)
                    write_robot_status("loading", idx, ROBOT_STAGES[idx][1], marker)
                    completed = max(completed, idx + 1)

        returncode = proc.wait()
        if returncode != 0:
            write_robot_status("error", error="robot startup failed")
            return ToolResult.failure(
                "robot startup failed",
                f"launch_dual_v2_2.sh exited {returncode}",
            )

        write_robot_status("ready", len(ROBOT_STAGES), "机器人已就绪", "全部组件已启动")
        return ToolResult.success(
            "robot control stack started",
            payload={"components": [s[0] for s in ROBOT_STAGES]},
        )

    return Skill(
        name="start_robot",
        description="Start the robot control stack automatically: rt_control -> upper_software (hardware mode + headless auto-connect) -> planner.",
        parameters=object_schema(
            {"monitor_after_start": {"type": "boolean", "default": False}},
            required=[],
        ),
        run=run,
        category="startup",
    )
