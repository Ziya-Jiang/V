"""Built-in skills: parameterized compositions of tools (and other skills)."""

from __future__ import annotations

from typing import List, Optional

from ..base import Skill
from .perceive import make_perceive_scene
from .pick_and_place import make_pick_and_place
from .safety_reset import make_safety_reset
from .start_robot import make_start_robot
from .tidy_table import make_tidy_table


def builtin_skills(registry, ctx=None) -> List[Skill]:
    """Return every built-in skill, in registration order."""
    return [
        make_perceive_scene(registry),
        make_pick_and_place(registry),
        make_tidy_table(registry),
        make_safety_reset(registry),
        make_start_robot(registry),
    ]


__all__ = ["builtin_skills"]
