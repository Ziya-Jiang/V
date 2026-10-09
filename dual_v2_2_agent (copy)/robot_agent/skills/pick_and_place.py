"""``pick_and_place`` -- the core manipulation skill.

Turns a single high-level spec ("put <object> on / beside <target>") into a
concrete motion plan (reusing the legacy Step4 ``planning_main`` per-object pose
logic) and executes it as a sequence of the atomic meta-operation tools:
approach → grasp → transport → place → reset.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from ..base import PlanError, Skill, ToolResult, object_schema
from ..task_status import write_task_status


def _action_to_tool(task: Dict[str, Any]) -> Tuple[str, Dict[str, Any]]:
    """Map one legacy detailed subtask to a meta-operation tool call."""
    action = task.get("action")
    arm = task.get("arm") or "right"
    pos = task.get("target_position")
    rot = task.get("target_rotation")
    obj = task.get("object")

    if action == "move_above_grasp":
        return "approach", {"arm": arm, "position": pos, "rotation": rot, "phase": "above"}
    if action == "move_down_to_grasp":
        return "approach", {"arm": arm, "position": pos, "rotation": rot, "phase": "descend"}
    if action == "close_gripper":
        return "grasp", {"arm": arm, "object_id": obj, "position": pos, "rotation": rot}
    if action == "open_gripper":
        return "place", {"arm": arm, "object_id": obj, "position": pos, "rotation": rot}
    if action == "lift_object":
        return "transport", {"arm": arm, "position": pos, "rotation": rot, "phase": "lift"}
    if action == "move_to_target":
        return "transport", {"arm": arm, "position": pos, "rotation": rot, "phase": "move"}
    if action == "adjust_orientation_for_place":
        return "transport", {"arm": arm, "position": pos, "rotation": rot, "phase": "move"}
    if action == "move_down_to_place":
        return "transport", {"arm": arm, "position": pos, "rotation": rot, "phase": "lower"}
    if action == "move_up_after_release":
        return "transport", {"arm": arm, "position": pos, "rotation": rot, "phase": "retreat"}
    if action == "close_to_object_center":
        return "transport", {"arm": arm, "position": pos, "rotation": rot, "phase": "move"}
    if action == "reset_pos":
        return "reset", {"arms": arm, "kind": "pos"}
    if action == "first_reset":
        return "reset", {"arms": "both", "kind": "first"}
    if action == "reset_for_fork":
        return "reset", {"arms": "both", "kind": "fork"}
    if action == "reset_all":
        return "reset", {"arms": "both", "kind": "safe"}
    raise PlanError(f"unmapped legacy action: {action}")


def make_pick_and_place(registry) -> Skill:
    def run(
        ctx,
        object: str,
        target: str,
        placement_type: str = "on_center",
        offset: Optional[list] = None,
        plan_only: Optional[bool] = None,
        report_progress: bool = True,
    ) -> ToolResult:
        if ctx.scene.get("objects_info") is None:
            # Not yet perceived in this process. Perceive once, then continue;
            # when a composite skill (tidy_table) already perceived, this is a
            # no-op because objects_info is present.
            perceived = registry.skill("perceive_scene")(ctx, instruction=ctx.instruction)
            if not perceived.ok:
                return perceived

        # Accept either an object_<id> string or a semantic {category, color}
        # spec; resolve the latter to its id now that perception has run.
        from ..executor import resolve_object_ref

        object = resolve_object_ref(object, ctx)
        target = resolve_object_ref(target, ctx)

        offset = list(offset) if offset else [0, 0, 0]
        if placement_type == "on_center":
            offset = [0, 0, 0]

        stages = ["规划抓取位姿", "执行抓取放置"]
        if report_progress:
            write_task_status("loading", 0, 2, "规划抓取位姿", stages)

        # Step4 reuses the legacy per-object pose logic unchanged.
        from ..context import ensure_robotram_on_path
        ensure_robotram_on_path()
        from step4_manual_planning import planning_main

        scene = ctx.scene
        cal = ctx.calibration
        predefined_tasks = [(object, target, offset[0], offset[1], offset[2])]
        try:
            detailed = planning_main(
                scene["rgb"],
                str(ctx.output_dir),
                scene["objects_info"],
                scene["vlm_categories_dict"],
                scene["masks"],
                cal["K"],
                cal["R"],
                cal["T"],
                logger=ctx.logger,
                predefined_tasks=predefined_tasks,
            )
        except Exception as error:  # noqa: BLE001
            if report_progress:
                write_task_status("error", 0, 2, "规划抓取位姿", stages, str(error))
            return ToolResult.failure("motion planning failed", str(error))

        if (plan_only if plan_only is not None else ctx.plan_only):
            if report_progress:
                write_task_status("ready", 2, 2, "规划完成", stages)
            return ToolResult.success(
                f"planned {len(detailed)} subtasks for {object}",
                payload={"subtasks": detailed},
            )

        if report_progress:
            write_task_status("loading", 1, 2, "执行抓取放置", stages)

        for task in detailed:
            tool_name, kwargs = _action_to_tool(task)
            result = registry.tool(tool_name)(ctx, **kwargs)
            if not result.ok:
                # Motion-planning (or any execution) failure: recover to the
                # safe prep position before surfacing the error.
                if report_progress:
                    write_task_status("error", 1, 2, "执行抓取放置", stages, result.error or result.message)
                recovered = registry.skill("safety_reset")(ctx)
                if not recovered.ok:
                    return ToolResult.failure(
                        f"{result.message} (safety_reset also failed: {recovered.message})",
                        result.error,
                    )
                return ToolResult.failure(result.message, result.error)

        if report_progress:
            write_task_status("ready", 2, 2, "抓取放置完成", stages)
        return ToolResult.success(
            f"pick_and_place {object} -> {target} ({placement_type})",
            payload={"subtasks": detailed},
        )

    return Skill(
        name="pick_and_place",
        description="Grasp one object and place it on or beside another. placement_type 'on_center' or 'beside' (with an [x,y,z] mm offset). Requires perceive_scene first.",
        parameters=object_schema(
            {
                "object": {"type": "string", "description": "object_<id> to move"},
                "target": {"type": "string", "description": "object_<id> to place on/near"},
                "placement_type": {"type": "string", "enum": ["on_center", "beside"], "default": "on_center"},
                "offset": {"type": "array", "items": {"type": "number"}, "description": "[x,y,z] mm offset for 'beside'"},
                "plan_only": {"type": "boolean", "default": False},
            }
        ),
        run=run,
        category="manipulation",
    )
