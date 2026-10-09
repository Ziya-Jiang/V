"""Task-progress status reporting for the green progress bar.

Every task-level SKILL (``tidy_table``, standalone ``pick_and_place``) writes one
snapshot per completed step so the GUI can show the user "第 3/6 步 · 叉→盘".
On failure the step is *preserved* (state ``error``, current stays at the failed
step) so a retry can resume where it left off, exactly like the product spec
asks.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

TASK_STATUS_PATH = "/mnt/ssd/user/V/dual_v2_2_agent (copy)/.robot_task_status.json"


def write_task_status(
    state: str,
    current: int = 0,
    total: int = 0,
    label: str = "",
    stages: Optional[List[str]] = None,
    error: Optional[str] = None,
) -> None:
    payload: Dict[str, Any] = {
        "state": state,           # "loading" | "ready" | "error"
        "current": int(current),
        "total": int(total),
        "label": label,
        "stages": list(stages) if stages else [],
        "error": error,
    }
    try:
        with open(TASK_STATUS_PATH, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
    except Exception:  # noqa: BLE001
        pass
