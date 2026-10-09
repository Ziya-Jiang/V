"""robot_agent -- the Agent + Skill + Tool grasping framework.

Decomposes robot manipulation into three layers:

* ``tools``    -- atomic meta-operations (approach / grasp / place / transport /
  reset) and perception primitives (ground / estimate / localize).
* ``skills``   -- reusable compositions of tools (perceive_scene,
  pick_and_place, tidy_table).
* ``planner``  -- the Agent that turns a natural-language request into an
  ordered, constrained ``Plan`` of skills; ``executor`` runs it.

Typical usage::

    from robot_agent.context import RobotContext
    from robot_agent.registry import build_default_registry
    from robot_agent.planner import Agent
    from robot_agent.executor import Executor

    ctx = RobotContext(output_dir="output/scene_57", instruction="帮我整理桌面")
    registry = build_default_registry(ctx)
    agent = Agent(registry)
    plan = agent.plan("帮我整理桌面")          # NL -> skill composition
    print(plan.to_json())                     # plan-only: no hardware touched
    # ctx.bootstrap()                         # attach arm/cameras/models
    # Executor(registry).run(plan, ctx)       # execute on the robot
"""

from .base import Plan, PlanError, Skill, SkillCall, Tool, ToolResult
from .executor import Executor, resolve_object_ref
from .planner import Agent
from .registry import Registry, build_default_registry

__all__ = [
    "Plan",
    "PlanError",
    "Skill",
    "SkillCall",
    "Tool",
    "ToolResult",
    "Agent",
    "Executor",
    "Registry",
    "build_default_registry",
    "resolve_object_ref",
]

__version__ = "0.1.0"
