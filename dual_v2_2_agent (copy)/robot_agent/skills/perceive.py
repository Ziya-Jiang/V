"""``perceive_scene`` -- the perception skill.

Composes the three perception tools (ground → estimate → localize) into a single
reusable unit that produces robot-base object metadata in ``ctx.scene``.
"""

from __future__ import annotations

from ..base import Skill, ToolResult, object_schema


def make_perceive_scene(registry) -> Skill:
    def run(ctx, instruction=None, targets=None, categories=None) -> ToolResult:
        instruction = instruction or ctx.instruction
        ground = registry.tool("ground")
        estimate = registry.tool("estimate_grasp_poses")
        localize = registry.tool("localize")

        result = ground(ctx, instruction=instruction, targets=targets)
        if not result.ok:
            return result
        result = estimate(ctx, categories=categories)
        if not result.ok:
            return result
        result = localize(ctx)
        if not result.ok:
            return result
        return ToolResult.success(
            "scene perceived",
            payload={"objects": ctx.scene.get("objects_info")},
        )

    return Skill(
        name="perceive_scene",
        description="Perceive the scene: ground semantic objects, estimate 6-DoF poses and grasp poses, and localize them to the robot base frame. Prerequisite for any manipulation.",
        parameters=object_schema(
            {
                "instruction": {"type": "string"},
                "targets": {"type": "array", "items": {"type": "object"}},
                "categories": {"type": "array", "items": {"type": "object"}},
            },
            required=[],
        ),
        run=run,
        category="perception",
    )
