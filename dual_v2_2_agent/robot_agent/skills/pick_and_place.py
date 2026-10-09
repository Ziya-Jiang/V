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


_CATEGORY_LABELS = {
    "bowl": "碗",
    "plate": "盘子",
    "cup": "杯子",
    "bread": "面包",
    "fork": "叉子",
}
_COLOR_LABELS = {
    "green": "绿色",
    "yellow": "黄色",
    "blue": "蓝色",
    "brown": "棕色",
}


def _semantic_label(ctx, selector) -> str:
    info = selector if isinstance(selector, dict) else {}
    if isinstance(selector, str):
        info = (ctx.scene.get("vlm_categories_dict") or {}).get(selector, {})
    category = info.get("category") if isinstance(info, dict) else None
    color = info.get("color") if isinstance(info, dict) else None
    if category:
        return f"{_COLOR_LABELS.get(color, color or '')}{_CATEGORY_LABELS.get(category, category)}"
    return str(selector or "未知物体")


def describe_placement(ctx, object_spec, target_spec, placement_type) -> str:
    relation = "旁边" if placement_type == "beside" else "上"
    return (
        f"把{_semantic_label(ctx, object_spec)}放到"
        f"{_semantic_label(ctx, target_spec)}{relation}"
    )


def describe_action(action: str, object_label: str, target_label: str) -> str:
    labels = {
        "first_reset": "双臂进入任务初始安全位",
        "reset_for_fork": "双臂进入叉子任务安全位",
        "reset_pos": "机械臂回到单臂安全位",
        "reset_all": "双臂回到最终安全位",
        "move_above_grasp": f"移动到{object_label}抓取点上方",
        "move_down_to_grasp": f"下降到{object_label}抓取位",
        "close_to_object_center": f"水平靠近{object_label}",
        "close_gripper": f"闭合夹爪抓取{object_label}",
        "lift_object": f"抬起{object_label}",
        "move_to_target": f"搬运{object_label}到{target_label}上方",
        "adjust_orientation_for_place": f"在{target_label}上方调整放置姿态",
        "move_down_to_place": f"下降到{target_label}放置位",
        "open_gripper": f"松开夹爪释放{object_label}",
        "move_up_after_release": f"从{target_label}上方抬升撤离",
    }
    return labels.get(action, f"执行底层动作 {action}")


def _emit_operation_event(ctx, event_type: str, status: str, message: str, **details) -> None:
    emit = getattr(ctx, "emit", None)
    if not callable(emit):
        return
    try:
        emit(
            {
                "type": event_type,
                "status": status,
                "message": message,
                **details,
            }
        )
    except Exception:  # noqa: BLE001
        pass


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
        stages = ["规划抓取位姿", "执行抓取放置"]
        object_spec = object
        target_spec = target
        operation = describe_placement(
            ctx,
            object_spec,
            target_spec,
            placement_type,
        )
        object_label = _semantic_label(ctx, object_spec)
        target_label = _semantic_label(ctx, target_spec)

        def cancellation_requested() -> bool:
            event = getattr(ctx, "cancel_event", None)
            return event is not None and event.is_set()

        def fail_with_recovery(
            message: str,
            error: Optional[str],
            progress: int,
        ) -> ToolResult:
            if report_progress:
                write_task_status("error", progress, 2, stages[progress], stages, error or message)
            try:
                recovered = registry.skill("safety_reset")(ctx)
            except Exception as recovery_error:  # noqa: BLE001
                return ToolResult.failure(
                    f"{message} (safety_reset raised: {recovery_error})",
                    error,
                )
            if not recovered.ok:
                return ToolResult.failure(
                    f"{message} (safety_reset also failed: {recovered.message})",
                    error,
                )
            return ToolResult.failure(message, error)

        if cancellation_requested():
            return fail_with_recovery("pick_and_place cancelled", "cancelled", 0)

        if ctx.scene.get("objects_info") is None:
            # Not yet perceived in this process. Perceive once, then continue;
            # when a composite skill (tidy_table) already perceived, this is a
            # no-op because objects_info is present.
            perceived = registry.skill("perceive_scene")(ctx, instruction=ctx.instruction)
            if not perceived.ok:
                return perceived
            if cancellation_requested():
                return fail_with_recovery("pick_and_place cancelled", "cancelled", 0)

        # Accept either an object_<id> string or a semantic {category, color}
        # spec; resolve the latter to its id now that perception has run.
        from ..executor import resolve_object_ref

        object = resolve_object_ref(object, ctx)
        target = resolve_object_ref(target, ctx)

        offset = list(offset) if offset else [0, 0, 0]
        if placement_type == "on_center":
            offset = [0, 0, 0]

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
            detail = (
                f"{operation}失败；环节：规划抓取与放置路径；"
                f"底层反馈：{error}"
            )
            _emit_operation_event(
                ctx,
                "operation.stage.failed",
                "failed",
                detail,
                operation=operation,
                phase="planning",
            )
            return fail_with_recovery(detail, detail, 0)

        if cancellation_requested():
            return fail_with_recovery("pick_and_place cancelled", "cancelled", 1)

        if (plan_only if plan_only is not None else ctx.plan_only):
            if report_progress:
                write_task_status("ready", 2, 2, "规划完成", stages)
            return ToolResult.success(
                f"planned {len(detailed)} subtasks for {object}",
                payload={"subtasks": detailed},
            )

        if report_progress:
            write_task_status("loading", 1, 2, "执行抓取放置", stages)

        total_actions = len(detailed)
        for action_index, task in enumerate(detailed, start=1):
            if cancellation_requested():
                return fail_with_recovery("pick_and_place cancelled", "cancelled", 1)
            action = str(task.get("action") or "unknown")
            arm = str(task.get("arm") or "right")
            stage_label = describe_action(
                action,
                object_label,
                target_label,
            )
            stage_message = (
                f"{operation}：环节 {action_index}/{total_actions}「{stage_label}」"
            )
            _emit_operation_event(
                ctx,
                "operation.stage.started",
                "running",
                stage_message,
                operation=operation,
                action=action,
                arm=arm,
                actionIndex=action_index,
                actionTotal=total_actions,
                targetPosition=task.get("target_position"),
                targetRotation=task.get("target_rotation"),
            )
            try:
                tool_name, kwargs = _action_to_tool(task)
                result = registry.tool(tool_name)(ctx, **kwargs)
            except Exception as error:  # noqa: BLE001
                detail = (
                    f"{operation}失败；环节 {action_index}/{total_actions}"
                    f"「{stage_label}」；机械臂：{arm}；底层动作：{action}；"
                    f"目标位置：{task.get('target_position')}；底层反馈：{error}"
                )
                _emit_operation_event(
                    ctx,
                    "operation.stage.failed",
                    "failed",
                    detail,
                    operation=operation,
                    action=action,
                    arm=arm,
                    actionIndex=action_index,
                    actionTotal=total_actions,
                )
                return fail_with_recovery(
                    detail,
                    detail,
                    1,
                )
            if not result.ok:
                # Motion-planning (or any execution) failure: recover to the
                # safe prep position before surfacing the error.
                feedback = result.error or result.message or "底层未确认动作完成"
                detail = (
                    f"{operation}失败；环节 {action_index}/{total_actions}"
                    f"「{stage_label}」；机械臂：{arm}；底层动作：{action}；"
                    f"目标位置：{task.get('target_position')}；底层反馈：{feedback}"
                )
                _emit_operation_event(
                    ctx,
                    "operation.stage.failed",
                    "failed",
                    detail,
                    operation=operation,
                    action=action,
                    arm=arm,
                    actionIndex=action_index,
                    actionTotal=total_actions,
                )
                return fail_with_recovery(detail, detail, 1)
            _emit_operation_event(
                ctx,
                "operation.stage.completed",
                "completed",
                stage_message + "完成",
                operation=operation,
                action=action,
                arm=arm,
                actionIndex=action_index,
                actionTotal=total_actions,
            )

        if report_progress:
            write_task_status("ready", 2, 2, "抓取放置完成", stages)
        return ToolResult.success(
            f"{operation}完成",
            payload={"operation": operation, "subtasks": detailed},
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


__all__ = [
    "describe_action",
    "describe_placement",
    "make_pick_and_place",
]
