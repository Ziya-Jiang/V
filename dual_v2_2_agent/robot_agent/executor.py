"""Plan executor.

Walks a ``Plan`` produced by the Agent, resolving semantic object references
after perception, honouring ordering/dependency constraints, and delegating each
step to its skill. Skills then drive the atomic tools through the registry.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional

from .base import Plan, PlanError, ToolResult


class ExecutionCancelled(PlanError):
    """Raised when a task reaches a cooperative cancellation boundary."""


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

    def run(
        self,
        plan: Plan,
        ctx,
        emit: Optional[Callable[[Dict[str, Any]], None]] = None,
        cancel_event=None,
    ) -> List[ToolResult]:
        self._validate(plan)

        results: List[ToolResult] = []
        completed: set = set()
        previous_cancel_event = getattr(ctx, "cancel_event", None)
        previous_emit = getattr(ctx, "emit", None)
        ctx.cancel_event = cancel_event
        ctx.emit = emit

        try:
            for index, step in enumerate(plan.steps):
                step_id = step.ref or f"step-{index}"
                if cancel_event is not None and cancel_event.is_set():
                    self._emit(
                        emit,
                        type="step.failed",
                        status="cancelled",
                        stepId=step_id,
                        message=f"{step.skill} cancelled before execution",
                    )
                    raise ExecutionCancelled(f"execution cancelled before step {step_id}")

                self._emit(
                    emit,
                    type="step.started",
                    status="running",
                    stepId=step_id,
                    message=f"starting {step.skill}",
                )
                try:
                    self._check_dependencies(step, completed)
                    # These skills consume semantic selectors before perception
                    # has populated object ids, so preserve their raw args.
                    args = (
                        dict(step.args)
                        if step.skill in {"tidy_table", "perceive_scene"}
                        else {
                            key: resolve_object_ref(value, ctx)
                            for key, value in step.args.items()
                        }
                    )
                    skill = self.registry.skill(step.skill)
                    result = skill(ctx, **args)
                except Exception as error:
                    status = (
                        "cancelled"
                        if isinstance(error, ExecutionCancelled)
                        or (
                            cancel_event is not None
                            and cancel_event.is_set()
                        )
                        else "failed"
                    )
                    self._emit(
                        emit,
                        type="step.failed",
                        status=status,
                        stepId=step_id,
                        message=str(error),
                    )
                    raise

                results.append(result)
                if not result.ok:
                    status = (
                        "cancelled"
                        if cancel_event is not None and cancel_event.is_set()
                        else "failed"
                    )
                    self._emit(
                        emit,
                        type="step.failed",
                        status=status,
                        stepId=step_id,
                        message=result.error or result.message,
                    )
                    break

                self._emit(
                    emit,
                    type="step.completed",
                    status="completed",
                    stepId=step_id,
                    message=result.message or f"completed {step.skill}",
                )
                if step.ref:
                    completed.add(step.ref)

            return results
        finally:
            ctx.cancel_event = previous_cancel_event
            ctx.emit = previous_emit

    @staticmethod
    def _emit(emit, **event) -> None:
        if emit is None:
            return
        try:
            emit(event)
        except Exception:  # noqa: BLE001
            # Observability must never interrupt robot recovery/execution.
            pass

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
