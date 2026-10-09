"""``tidy_table`` -- a composite skill for the canonical "clean the table" task.

A convenience wrapper over ``perceive_scene`` + a list of ``pick_and_place``
steps + a final ``reset``. The Agent may invoke it directly, or expand the same
recipe into an explicit skill sequence (see ``planner``).
"""

from __future__ import annotations

from ..base import Skill, ToolResult, object_schema
from ..task_status import write_task_status

# The canonical recipe, expressed *semantically* (category/color) so it
# generalises across scenes instead of hard-coding ``object_0``... ids.
# Mirrors the legacy ``predefined_tasks`` for task_type == "all".
CLEAN_TABLE_PLACEMENTS = [
    {"object": {"category": "bowl", "color": "green"}, "target": {"category": "plate", "color": "yellow"}, "placement_type": "on_center"},
    {"object": {"category": "bread"}, "target": {"category": "bowl", "color": "green"}, "placement_type": "on_center"},
    {"object": {"category": "fork", "color": "brown"}, "target": {"category": "plate", "color": "yellow"}, "placement_type": "beside", "offset": [0, -120, 0]},
    {"object": {"category": "cup", "color": "blue"}, "target": {"category": "plate", "color": "yellow"}, "placement_type": "beside", "offset": [100, 120, 0]},
]


def _placement_label(spec) -> str:
    obj = spec.get("object", {})
    tgt = spec.get("target", {})
    obj_name = obj.get("category", "物体") if isinstance(obj, dict) else str(obj)
    tgt_name = tgt.get("category", "目标") if isinstance(tgt, dict) else str(tgt)
    return f"{obj_name}→{tgt_name}"


def make_tidy_table(registry) -> Skill:
    def run(ctx, instruction=None, placements=None) -> ToolResult:
        instruction = instruction or ctx.instruction
        placements = placements if placements is not None else CLEAN_TABLE_PLACEMENTS

        stages = ["感知桌面"] + [_placement_label(spec) for spec in placements]
        total = len(stages)

        write_task_status("loading", 0, total, "感知桌面", stages)
        result = registry.skill("perceive_scene")(ctx, instruction=instruction)
        if not result.ok:
            write_task_status("error", 0, total, "感知桌面", stages, result.error or result.message)
            return result

        pick_and_place = registry.skill("pick_and_place")
        for i, spec in enumerate(placements):
            step = i + 1
            write_task_status("loading", step, total, stages[step], stages)
            result = pick_and_place(ctx, report_progress=False, **spec)
            if not result.ok:
                write_task_status("error", step, total, stages[step], stages, result.error or result.message)
                return result

        write_task_status("ready", total, total, "任务完成", stages)
        return ToolResult.success(
            f"tidied table ({len(placements)} placements)",
            payload={"placements": placements},
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
