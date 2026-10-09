"""Plan executor.

Walks a ``Plan`` produced by the Agent, resolving semantic object references
after perception, honouring ordering/dependency constraints, and delegating each
step to its skill. Skills then drive the atomic tools through the registry.
"""

from __future__ import annotations

from typing import Any, Dict, List

from .base import Plan, PlanError, ToolResult


def resolve_object_ref(value: Any, ctx) -> Any:
    """Recursively resolve semantic object specs (``{"category": ...}``) to ids.

    Object ids are only known after ``perceive_scene`` populates
    ``ctx.scene["vlm_categories_dict"]``, so resolution happens at execution
    time, not planning time.
    """
    if isinstance(value, dict):
        if "category" in value:
            obj_id = ctx.find_object_id(value["category"], value.get("color"))
            if obj_id is None:
                raise PlanError(f"cannot resolve object spec to an object id: {value}")
            return obj_id
        return {key: resolve_object_ref(val, ctx) for key, val in value.items()}
    if isinstance(value, list):
        return [resolve_object_ref(item, ctx) for item in value]
    return value


class Executor:
    def __init__(self, registry) -> None:
        self.registry = registry

    def run(self, plan: Plan, ctx) -> List[ToolResult]:
        self._validate(plan)

        results: List[ToolResult] = []
        completed: set = set()

        for step in plan.steps:
            self._check_dependencies(step, completed)

            args = {k: resolve_object_ref(v, ctx) for k, v in step.args.items()}
            skill = self.registry.skill(step.skill)
            result = skill(ctx, **args)
            results.append(result)

            if not result.ok:
                break
            if step.ref:
                completed.add(step.ref)

        return results

    def _validate(self, plan: Plan) -> None:
        for step in plan.steps:
            if not self.registry.has_skill(step.skill):
                raise PlanError(
                    f"plan references unknown skill {step.skill!r}; "
                    f"available: {[s.name for s in self.registry.skills]}"
                )

    @staticmethod
    def _check_dependencies(step, completed: set) -> None:
        for ref in step.constraints.get("depends_on", []):
            if ref not in completed:
                raise PlanError(
                    f"step {step.ref or step.skill} depends on unresolved step {ref!r}"
                )
