"""``tidy_table`` -- a composite skill for the canonical "clean the table" task.

A convenience wrapper over ``perceive_scene`` + a list of ``pick_and_place``
steps + a final ``reset``. The Agent may invoke it directly, or expand the same
recipe into an explicit skill sequence (see ``planner``).
"""

from __future__ import annotations

from ..base import Skill, ToolResult, object_schema
from ..task_status import write_task_status
from .pick_and_place import describe_placement

# The canonical recipe, expressed *semantically* (category/color) so it
# generalises across scenes instead of hard-coding ``object_0``... ids.
# Mirrors the legacy ``predefined_tasks`` for task_type == "all".
CLEAN_TABLE_PLACEMENTS = [
    {"object": {"category": "bowl", "color": "green"}, "target": {"category": "plate", "color": "yellow"}, "placement_type": "on_center"},
    {"object": {"category": "bread"}, "target": {"category": "plate", "color": "yellow"}, "placement_type": "on_center"},
    {"object": {"category": "cup", "color": "blue"}, "target": {"category": "plate", "color": "yellow"}, "placement_type": "beside", "offset": [100, 120, 0]},
]


def _placement_label(spec) -> str:
    obj = spec.get("object", {})
    tgt = spec.get("target", {})
    obj_name = obj.get("category", "物体") if isinstance(obj, dict) else str(obj)
    tgt_name = tgt.get("category", "目标") if isinstance(tgt, dict) else str(tgt)
    return f"{obj_name}→{tgt_name}"


def _perception_selectors(placements):
    selectors = []
    for spec in placements:
        for field in ("object", "target"):
            selector = spec.get(field)
            if selector is None or selector in selectors:
                continue
            selectors.append(dict(selector) if isinstance(selector, dict) else selector)
    return selectors


def _resolve_selector(ctx, selector):
    if isinstance(selector, str):
        categories = ctx.scene.get("vlm_categories_dict") or {}
        return selector if selector in categories else None
    if isinstance(selector, dict):
        return ctx.find_object_id(
            selector.get("category", ""),
            selector.get("color"),
        )
    return None


def _filter_available_placements(ctx, placements):
    available = []
    skipped = []
    for spec in placements:
        missing = []
        if _resolve_selector(ctx, spec.get("object")) is None:
            missing.append("object")
        if _resolve_selector(ctx, spec.get("target")) is None:
            missing.append("target")
        if missing:
            skipped.append(
                {
                    "placement": spec,
                    "label": _placement_label(spec),
                    "reason": "missing detected " + " and ".join(missing),
                }
            )
        else:
            available.append(spec)
    return available, skipped


def make_tidy_table(registry) -> Skill:
    def run(ctx, instruction=None, placements=None) -> ToolResult:
        instruction = instruction or ctx.instruction
        placements = placements if placements is not None else CLEAN_TABLE_PLACEMENTS

        write_task_status("loading", 0, 1, "感知桌面", ["感知桌面"])
        result = registry.skill("perceive_scene")(
            ctx,
            instruction=instruction,
            selectors=_perception_selectors(placements),
        )
        if not result.ok:
            write_task_status(
                "error",
                0,
                1,
                "感知桌面",
                ["感知桌面"],
                result.error or result.message,
            )
            return result

        placements, skipped = _filter_available_placements(ctx, placements)
        stages = ["感知桌面"] + [
            describe_placement(
                ctx,
                spec.get("object"),
                spec.get("target"),
                spec.get("placement_type", "on_center"),
            )
            for spec in placements
        ]
        total = len(stages)
        emit = getattr(ctx, "emit", None)
        logger = getattr(ctx, "logger", None)
        for item in skipped:
            message = (
                f"整理桌面跳过 {item['label']}：{item['reason']}。"
            )
            if logger is not None:
                logger.warning(message)
            if callable(emit):
                try:
                    emit(
                        {
                            "type": "placement.skipped",
                            "status": "skipped",
                            "message": message,
                            "placement": item["placement"],
                        }
                    )
                except Exception:  # noqa: BLE001
                    pass

        pick_and_place = registry.skill("pick_and_place")
        for i, spec in enumerate(placements):
            step = i + 1
            operation = stages[step]
            write_task_status("loading", step, total, operation, stages)
            if callable(emit):
                try:
                    emit(
                        {
                            "type": "tidy.placement.started",
                            "status": "running",
                            "message": (
                                f"整理桌面：开始第 {step}/{len(placements)} 项，"
                                f"{operation}"
                            ),
                            "operation": operation,
                            "placementIndex": step,
                            "placementTotal": len(placements),
                        }
                    )
                except Exception:  # noqa: BLE001
                    pass
            result = pick_and_place(ctx, report_progress=False, **spec)
            if not result.ok:
                cause = result.error or result.message or "未知执行错误"
                message = (
                    f"整理桌面失败；第 {step}/{len(placements)} 项「{operation}」；"
                    f"{cause}"
                )
                write_task_status(
                    "error",
                    step,
                    total,
                    operation,
                    stages,
                    message,
                )
                if logger is not None:
                    logger.error(message)
                if callable(emit):
                    try:
                        emit(
                            {
                                "type": "tidy.placement.failed",
                                "status": "failed",
                                "message": message,
                                "operation": operation,
                                "placementIndex": step,
                                "placementTotal": len(placements),
                            }
                        )
                    except Exception:  # noqa: BLE001
                        pass
                return ToolResult.failure(message, message)
            if callable(emit):
                try:
                    emit(
                        {
                            "type": "tidy.placement.completed",
                            "status": "completed",
                            "message": f"整理桌面：{operation}完成",
                            "operation": operation,
                            "placementIndex": step,
                            "placementTotal": len(placements),
                        }
                    )
                except Exception:  # noqa: BLE001
                    pass

        write_task_status("ready", total, total, "任务完成", stages)
        return ToolResult.success(
            (
                f"tidied table ({len(placements)} placements, "
                f"{len(skipped)} skipped)"
            ),
            payload={"placements": placements, "skipped": skipped},
        )

    return Skill(
        name="tidy_table",
        description="Clean/tidy the table: perceive the scene, then put each object in its place and reset the arms. Accepts an optional list of pick_and_place specs.",
        parameters=object_schema(
            {
                "instruction": {"type": "string"},
                "placements": {"type": "array", "items": {"type": "object"}},
            },
            required=[],
        ),
        run=run,
        category="task",
    )


__all__ = [
    "CLEAN_TABLE_PLACEMENTS",
    "make_tidy_table",
    "_filter_available_placements",
    "_perception_selectors",
]
