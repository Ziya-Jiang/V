#!/usr/bin/env python3
"""JSON bridge that lets a non-Python process (e.g. the DSH Host) drive robot_agent.

Usage::

    python3 robot_agent/bridge.py <method> <base64-json-args>

``method`` is one of ``manifest``, ``plan``, or ``call``; the second argument is
the base64-encoded JSON payload (base64 avoids shell-quoting issues). The bridge
prints exactly one JSON object to stdout and exits 0 on success, 1 on error.

* ``manifest`` -> ``{ok, tools, skills}``
* ``plan``     -> ``{ok, plan}`` (offline; never touches hardware)
* ``call``     -> ``{ok, message, error, data}`` -- invoke one tool/skill.
  ``dry_run`` (default True) returns the intended call without touching
  hardware; ``dry_run: false`` bootstraps arm/cameras/models and executes.
"""

from __future__ import annotations

import base64
import json
import sys
from typing import Any, Dict


def _decode_args() -> Dict[str, Any]:
    if len(sys.argv) > 2:
        try:
            return json.loads(base64.b64decode(sys.argv[2]).decode("utf-8"))
        except Exception as error:  # noqa: BLE001
            return {"error": f"bad base64/json args: {error}"}
    return {}


def _sanitize(value: Any) -> Any:
    """Recursively convert numpy scalars/arrays to JSON-safe Python values."""
    import numpy as np

    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return _sanitize(value.tolist())
    if isinstance(value, dict):
        return {str(k): _sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _emit(obj: Any, ok: bool = True) -> None:
    print(json.dumps(_sanitize({"ok": ok, **obj}), ensure_ascii=False))


def _fail(message: str, error: str = "") -> None:
    print(json.dumps({"ok": False, "message": message, "error": error}, ensure_ascii=False))
    sys.exit(1)


def do_manifest() -> None:
    from .context import RobotContext
    from .registry import build_default_registry

    ctx = RobotContext(plan_only=True)
    registry = build_default_registry(ctx)
    _emit(
        {
            "message": "manifest",
            "data": {
                "tools": registry.tool_manifest(),
                "skills": registry.skill_manifest(),
            },
        }
    )


def do_plan(args: Dict[str, Any]) -> None:
    from .context import RobotContext
    from .planner import Agent
    from .registry import build_default_registry

    instruction = args.get("instruction", "clean the table")
    ctx = RobotContext(
        output_dir=args.get("output_dir"),
        instruction=instruction,
        plan_only=True,
    )
    registry = build_default_registry(ctx)
    agent = Agent(registry)
    try:
        plan = agent.plan(instruction)
    except Exception as error:  # noqa: BLE001
        _fail(f"planning failed: {error}", str(error))
    _emit({"message": "plan", "data": {"plan": plan.to_dict()}})


def do_call(args: Dict[str, Any]) -> None:
    from .base import PlanError
    from .context import RobotContext
    from .executor import resolve_object_ref
    from .registry import build_default_registry

    kind = args.get("kind", "tool")
    name = args.get("name")
    if not name:
        _fail("missing 'name'", "name is required")
    dry_run = bool(args.get("dry_run", True))
    call_args = args.get("args", {}) or {}

    ctx = RobotContext(
        output_dir=args.get("output_dir"),
        instruction=args.get("instruction", "clean the table"),
        plan_only=dry_run,
    )
    # Hydrate semantic categories from the output dir so {category, color}
    # object specs resolve to object_<id> even in a fresh process.
    ctx.load_categories_from_output()

    registry = build_default_registry(ctx)
    try:
        target = registry.tool(name) if kind == "tool" else registry.skill(name)
    except PlanError as error:
        _fail(str(error), str(error))

    resolved = {k: resolve_object_ref(v, ctx) for k, v in call_args.items()}

    if dry_run:
        _emit(
            {
                "message": f"[dry-run] {kind} {name} -- hardware not touched",
                "data": {
                    "dry_run": True,
                    "kind": kind,
                    "name": name,
                    "args": _sanitize(resolved),
                },
            }
        )
        return

    try:
        ctx.bootstrap()
    except Exception as error:  # noqa: BLE001
        _fail(f"bootstrap failed (hardware/models unavailable?): {error}", str(error))

    try:
        result = target(ctx, **resolved)
    except Exception as error:  # noqa: BLE001
        _fail(f"{kind} {name} failed: {error}", str(error))

    envelope = {
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
    _emit(envelope, ok=result.ok)


def main() -> int:
    method = sys.argv[1] if len(sys.argv) > 1 else "manifest"
    args = _decode_args()
    if "error" in args:
        _fail(args["error"], args["error"])

    try:
        if method == "manifest":
            do_manifest()
        elif method == "plan":
            do_plan(args)
        elif method == "call":
            do_call(args)
        else:
            _fail(f"unknown method: {method}", f"unknown method: {method}")
    except SystemExit:
        raise
    except Exception as error:  # noqa: BLE001
        _fail(f"bridge error: {error}", str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
