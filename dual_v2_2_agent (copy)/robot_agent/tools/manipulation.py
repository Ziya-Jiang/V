"""Manipulation meta-operations.

The robot's motion vocabulary is reduced to five atomic tools:

* ``approach``  -- move the open gripper toward a pose (``above`` / ``descend``).
* ``grasp``     -- close the gripper on the object.
* ``place``     -- open the gripper to release the object.
* ``transport`` -- move the (held or empty) gripper between poses (``lift`` /
  ``move`` / ``lower`` / ``retreat``).
* ``reset``     -- return one or both arms to a known joint-space pose.

Each tool is a thin, typed wrapper over the existing ``step5_execute`` action
vocabulary (``move_above_grasp`` ... ``reset_all``), so the intricate low-level
arm/gripper logic stays in ``RobotArm`` and is reused unchanged.
"""

from __future__ import annotations

from typing import List

from ..base import Tool, ToolResult, object_schema

ARM_SCHEMA = {"type": "string", "enum": ["left", "right"]}
VEC3_SCHEMA = {
    "type": "array",
    "items": {"type": "number"},
    "minItems": 3,
    "maxItems": 3,
    "description": "[x, y, z] in robot-base millimetres",
}
ROT3_SCHEMA = {
    "type": "array",
    "items": {"type": "number"},
    "minItems": 3,
    "maxItems": 3,
    "description": "[rx, ry, rz] Euler angles in degrees",
}


class Manipulator:
    """Facade over the raw ``step5_execute`` action executor.

    The Agent never touches ``RobotArm.master_control`` directly; skills call the
    semantic tools above, which land here as one concrete action dict.
    """

    def __init__(self, ctx) -> None:
        self.ctx = ctx

    def _execute(self, action, arm, position=None, rotation=None, object_id=None, description="") -> ToolResult:
        if self.ctx.plan_only:
            return ToolResult.success(f"[dry-run] {action} {arm}", payload={"action": action, "arm": arm})

        from step5_execute import execute_single_task

        task = {
            "action": action,
            "arm": arm,
            "object": object_id,
            "target_position": list(position) if position is not None else None,
            "target_rotation": list(rotation) if rotation is not None else None,
            "description": description,
        }
        try:
            _, ok = execute_single_task(
                self.ctx.robot,
                task,
                0,
                self.ctx.object_id_to_type or {},
            )
        except Exception as error:  # noqa: BLE001
            return ToolResult.failure(f"{action} failed", str(error))
        if not ok:
            return ToolResult.failure(f"{action} did not confirm completion")
        return ToolResult.success(f"{action} {arm}")

    def approach(self, arm, position, rotation, phase="above") -> ToolResult:
        action = "move_above_grasp" if phase == "above" else "move_down_to_grasp"
        return self._execute(action, arm, position, rotation, description=f"approach {phase}")

    def grasp(self, arm, object_id=None, position=None, rotation=None) -> ToolResult:
        return self._execute("close_gripper", arm, position=position, rotation=rotation, object_id=object_id, description="grasp")

    def place(self, arm, object_id=None, position=None, rotation=None) -> ToolResult:
        return self._execute("open_gripper", arm, position=position, rotation=rotation, object_id=object_id, description="place")

    def transport(self, arm, position, rotation, phase="move") -> ToolResult:
        action = {
            "lift": "lift_object",
            "move": "move_to_target",
            "lower": "move_down_to_place",
            "retreat": "move_up_after_release",
        }[phase]
        return self._execute(action, arm, position, rotation, description=f"transport {phase}")

    def reset(self, arms="both", kind="safe") -> ToolResult:
        if kind == "first":
            return self._execute("first_reset", "right", description="first reset")
        if kind == "fork":
            return self._execute("reset_for_fork", "right", description="reset for fork")
        if arms in ("left", "right"):
            return self._execute("reset_pos", arms, description=f"reset {arms}")
        return self._execute("reset_all", "right", description="reset all")


# --------------------------------------------------------------------------- #
# Tool definitions
# --------------------------------------------------------------------------- #
def _approach(ctx, arm, position, rotation, phase="above") -> ToolResult:
    return Manipulator(ctx).approach(arm, position, rotation, phase)


def _grasp(ctx, arm, object_id=None, object_category="cup", position=None, rotation=None) -> ToolResult:
    return Manipulator(ctx).grasp(arm, object_id, position=position, rotation=rotation)


def _place(ctx, arm, object_id=None, object_category="cup", position=None, rotation=None) -> ToolResult:
    return Manipulator(ctx).place(arm, object_id, position=position, rotation=rotation)


def _transport(ctx, arm, position, rotation, phase="move") -> ToolResult:
    return Manipulator(ctx).transport(arm, position, rotation, phase)


def _reset(ctx, arms="both", kind="safe") -> ToolResult:
    return Manipulator(ctx).reset(arms, kind)


MANIPULATION_TOOLS: List[Tool] = [
    Tool(
        name="approach",
        description="Move the open gripper toward a grasp pose. phase='above' targets the pre-grasp waypoint, 'descend' the grasp point.",
        parameters=object_schema(
            {
                "arm": ARM_SCHEMA,
                "position": VEC3_SCHEMA,
                "rotation": ROT3_SCHEMA,
                "phase": {"type": "string", "enum": ["above", "descend"], "default": "above"},
            }
        ),
        fn=_approach,
        returns="ToolResult",
        category="manipulation",
    ),
    Tool(
        name="grasp",
        description="Close the gripper of the given arm to grasp the object.",
        parameters=object_schema(
            {
                "arm": ARM_SCHEMA,
                "object_id": {"type": "string", "description": "object_<id> being grasped (optional)"},
                "object_category": {"type": "string", "default": "cup"},
            },
            required=["arm"],
        ),
        fn=_grasp,
        returns="ToolResult",
        category="manipulation",
    ),
    Tool(
        name="place",
        description="Open the gripper of the given arm to release/place the object.",
        parameters=object_schema(
            {
                "arm": ARM_SCHEMA,
                "object_id": {"type": "string", "description": "object_<id> being placed (optional)"},
                "object_category": {"type": "string", "default": "cup"},
            },
            required=["arm"],
        ),
        fn=_place,
        returns="ToolResult",
        category="manipulation",
    ),
    Tool(
        name="transport",
        description="Move the gripper between poses. phase='lift' raises the object, 'move' carries it above the target, 'lower' descends to the placement, 'retreat' backs away.",
        parameters=object_schema(
            {
                "arm": ARM_SCHEMA,
                "position": VEC3_SCHEMA,
                "rotation": ROT3_SCHEMA,
                "phase": {"type": "string", "enum": ["lift", "move", "lower", "retreat"], "default": "move"},
            }
        ),
        fn=_transport,
        returns="ToolResult",
        category="manipulation",
    ),
    Tool(
        name="reset",
        description="Return one or both arms to a known safe joint-space pose. kind='safe' (reset_all), 'first' (first_reset), 'fork' (reset_for_fork), 'pos' (single-arm reset_pos).",
        parameters=object_schema(
            {
                "arms": {"type": "string", "enum": ["left", "right", "both"], "default": "both"},
                "kind": {"type": "string", "enum": ["safe", "first", "pos", "fork"], "default": "safe"},
            },
            required=[],
        ),
        fn=_reset,
        returns="ToolResult",
        category="manipulation",
    ),
]


__all__ = ["MANIPULATION_TOOLS", "Manipulator"]
