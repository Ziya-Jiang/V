"""Tool + Skill registry.

The registry is the catalogue the Agent plans against and the Executor runs
against. It keeps the two layers separated but uniformly addressable:

* ``tool_manifest()`` emits the OpenAI-style function specs an LLM planner
  consumes for *tool* calling.
* ``skill_manifest()`` emits the higher-level recipes the Agent composes into a
  ``Plan`` -- this is the "SKILL combination" the product is built around.
"""

from __future__ import annotations

from typing import Dict, List

from .base import PlanError, Skill, Tool


class Registry:
    def __init__(self) -> None:
        self._tools: Dict[str, Tool] = {}
        self._skills: Dict[str, Skill] = {}

    # ------------------------------------------------------------------ #
    # Registration
    # ------------------------------------------------------------------ #
    def register_tool(self, tool: Tool) -> Tool:
        if tool.name in self._tools:
            raise ValueError(f"tool already registered: {tool.name}")
        self._tools[tool.name] = tool
        return tool

    def register_skill(self, skill: Skill) -> Skill:
        if skill.name in self._skills:
            raise ValueError(f"skill already registered: {skill.name}")
        self._skills[skill.name] = skill
        return skill

    # ------------------------------------------------------------------ #
    # Lookup
    # ------------------------------------------------------------------ #
    def tool(self, name: str) -> Tool:
        try:
            return self._tools[name]
        except KeyError:
            raise PlanError(f"unknown tool: {name}") from None

    def skill(self, name: str) -> Skill:
        try:
            return self._skills[name]
        except KeyError:
            raise PlanError(f"unknown skill: {name}") from None

    def has_skill(self, name: str) -> bool:
        return name in self._skills

    @property
    def tools(self) -> List[Tool]:
        return list(self._tools.values())

    @property
    def skills(self) -> List[Skill]:
        return list(self._skills.values())

    # ------------------------------------------------------------------ #
    # Manifests
    # ------------------------------------------------------------------ #
    def tool_manifest(self) -> List[dict]:
        return [tool.to_function_spec() for tool in self._tools.values()]

    def skill_manifest(self) -> List[dict]:
        return [skill.to_spec() for skill in self._skills.values()]


def build_default_registry(ctx=None) -> Registry:
    """Construct the registry with every built-in tool and skill wired up."""
    from . import skills as skill_module
    from . import tools as tool_module

    registry = Registry()

    for tool in tool_module.builtin_tools(ctx):
        registry.register_tool(tool)

    for skill in skill_module.builtin_skills(registry, ctx):
        registry.register_skill(skill)

    return registry
