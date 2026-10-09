"""Core data structures for the Agent + Skill + Tool architecture.

The whole grasping system is decomposed into three layers:

* **Tool**  -- an atomic meta-operation (approach / grasp / place / ...) or a
  perception primitive (ground / estimate / localize). Tools are the leaves of
  the execution graph.
* **Skill** -- a reusable, parameterized composition of tools (and optionally
  other skills) that fulfils one sub-task, e.g. ``pick_and_place``.
* **Agent** -- the layer that turns a natural-language request into an ordered,
  constrained ``Plan`` of ``SkillCall`` entries.

Every Tool and Skill is *self-describing*: it carries a JSON-Schema ``parameters``
block, so the Agent can plan against the registry using exactly the manifest an
LLM consumes for function calling.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional


def object_schema(properties: Dict[str, Any], required: Optional[List[str]] = None) -> Dict[str, Any]:
    """Build a JSON-Schema ``object`` node from a ``{name: schema}`` mapping.

    ``required`` defaults to every declared property, which is the right default
    for most primitive tools: an ``approach`` without a pose is meaningless.
    """
    return {
        "type": "object",
        "properties": dict(properties),
        "required": list(required if required is not None else properties.keys()),
    }


# --------------------------------------------------------------------------- #
# Tool result
# --------------------------------------------------------------------------- #
@dataclass
class ToolResult:
    """Uniform outcome of one tool / skill execution.

    ``payload`` carries the structured data a downstream step needs (masks,
    poses, objects_info, ...). ``error`` is a machine-facing reason when the
    execution failed, kept separate from the human-facing ``message``.
    """

    ok: bool
    message: str = ""
    payload: Any = None
    error: Optional[str] = None

    @staticmethod
    def success(message: str = "", payload: Any = None) -> "ToolResult":
        return ToolResult(ok=True, message=message, payload=payload)

    @staticmethod
    def failure(message: str, error: Optional[str] = None) -> "ToolResult":
        return ToolResult(ok=False, message=message, error=error)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "message": self.message,
            "error": self.error,
        }


# --------------------------------------------------------------------------- #
# Tool
# --------------------------------------------------------------------------- #
@dataclass
class Tool:
    """An atomic, self-describing meta-operation or perception primitive."""

    name: str
    description: str
    parameters: Dict[str, Any]
    fn: Callable[..., ToolResult]  # fn(ctx, **params) -> ToolResult
    returns: str = ""
    category: str = "generic"  # "perception" | "manipulation"

    def to_function_spec(self) -> Dict[str, Any]:
        """OpenAI-style function-calling spec for the LLM planner."""
        return {
            "name": self.name,
            "description": self.description,
            "parameters": self.parameters,
        }

    def __call__(self, ctx, **params) -> ToolResult:
        return self.fn(ctx, **params)


# --------------------------------------------------------------------------- #
# Skill / Plan
# --------------------------------------------------------------------------- #
@dataclass
class Skill:
    """A reusable composition of tools (and possibly other skills)."""

    name: str
    description: str
    parameters: Dict[str, Any]
    run: Callable[..., ToolResult]  # run(ctx, **params) -> ToolResult
    category: str = "task"

    def to_spec(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "parameters": self.parameters,
        }

    def __call__(self, ctx, **params) -> ToolResult:
        return self.run(ctx, **params)


@dataclass
class SkillCall:
    """One bound invocation of a skill inside a plan.

    ``args`` binds the skill's parameters (objects, targets, offsets).
    ``constraints`` records the ordering / dependency / safety requirements the
    Agent attached to this step, e.g. ``{"depends_on": "scene", "arm": "right"}``.
    """

    skill: str
    args: Dict[str, Any] = field(default_factory=dict)
    constraints: Dict[str, Any] = field(default_factory=dict)
    ref: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "skill": self.skill,
            "args": self.args,
            "constraints": self.constraints,
            "ref": self.ref,
        }


@dataclass
class Plan:
    """The Agent's output: an ordered, constrained sequence of skill calls."""

    instruction: str
    steps: List[SkillCall] = field(default_factory=list)
    constraints: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "instruction": self.instruction,
            "constraints": self.constraints,
            "steps": [step.to_dict() for step in self.steps],
        }

    def to_json(self, indent: int = 2) -> str:
        import json

        return json.dumps(self.to_dict(), ensure_ascii=False, indent=indent)


class PlanError(RuntimeError):
    """Raised when the Agent cannot produce a valid plan for a request."""
