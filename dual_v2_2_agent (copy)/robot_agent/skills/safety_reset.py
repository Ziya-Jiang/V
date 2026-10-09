"""``safety_reset`` -- failure-recovery SKILL.

After a motion-planning error the arm can be left mid-task, possibly still
holding an object. This skill restores a safe state in the correct order:

1. open both grippers (release anything held),
2. return the arms to the verified safe joint-space prep position.

It is a *composition* of the atomic ``place`` and ``reset`` tools, so it is the
canonical "回预备位" recovery invoked automatically after any execution failure.
"""

from __future__ import annotations

from ..base import Skill, ToolResult, object_schema


def make_safety_reset(registry) -> Skill:
    def run(ctx, arms="both", kind="safe") -> ToolResult:
        place = registry.tool("place")

        result = place(ctx, arm="left")
        if not result.ok:
            return result
        result = place(ctx, arm="right")
        if not result.ok:
            return result

        return registry.tool("reset")(ctx, arms=arms, kind=kind)

    return Skill(
        name="safety_reset",
        description="Failure recovery after a motion-planning error: open both grippers to release any held object, then return the arms to the safe prep position.",
        parameters=object_schema(
            {
                "arms": {"type": "string", "enum": ["left", "right", "both"], "default": "both"},
                "kind": {"type": "string", "enum": ["safe", "first", "pos", "fork"], "default": "safe"},
            },
            required=[],
        ),
        run=run,
        category="manipulation",
    )
