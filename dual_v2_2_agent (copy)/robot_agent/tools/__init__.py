"""Built-in tools: atomic meta-operations and perception primitives."""

from __future__ import annotations

from typing import List, Optional

from ..base import Tool
from .manipulation import MANIPULATION_TOOLS
from .perception import PERCEPTION_TOOLS


def builtin_tools(ctx=None) -> List[Tool]:
    """Return every built-in tool, in registration order."""
    return [*PERCEPTION_TOOLS, *MANIPULATION_TOOLS]


__all__ = ["builtin_tools", "MANIPULATION_TOOLS", "PERCEPTION_TOOLS"]
