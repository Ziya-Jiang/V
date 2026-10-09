"""The Agent: natural language → an ordered, constrained ``Plan`` of skills.

Two planning backends are provided:

* **rule-based** -- deterministic intents for the known tasks (e.g. "clean the
  table"), always available and fully offline.
* **LLM-based**   -- an optional pluggable LLM that plans against the skill
  manifest, for open-ended requests.

The Agent always prefers the rule backend and falls back to the LLM when no
rule matches. A ``Plan`` references objects *semantically* (category/color), so
it stays valid across scenes; the Executor resolves them to ``object_<id>``
after ``perceive_scene`` runs.
"""

from __future__ import annotations

import json
import re
from typing import Any, Dict, List, Optional

from .base import Plan, PlanError, SkillCall
from .skills.tidy_table import CLEAN_TABLE_PLACEMENTS


# Lightweight natural-language -> semantic object vocabulary for rule intents.
# Keywords are plain substrings: \b word boundaries do not fire on CJK text.
_OBJECT_VOCAB = [
    (("bowl", "碗"), {"category": "bowl", "color": "green"}),
    (("plate", "盘子", "盘"), {"category": "plate", "color": "yellow"}),
    (("cup", "杯子", "杯"), {"category": "cup", "color": "blue"}),
    (("bread", "面包"), {"category": "bread"}),
    (("fork", "叉子", "叉"), {"category": "fork", "color": "brown"}),
]


class Agent:
    def __init__(self, registry, llm=None) -> None:
        self.registry = registry
        self.llm = llm

    # ------------------------------------------------------------------ #
    def plan(self, instruction: str) -> Plan:
        plan = self._plan_by_rules(instruction)
        if plan is None and self.llm is not None:
            plan = self._plan_by_llm(instruction)
        if plan is None:
            raise PlanError(
                f"cannot plan request (no rule matched and no LLM configured): {instruction!r}"
            )
        return plan

    # ------------------------------------------------------------------ #
    # Rule backend
    # ------------------------------------------------------------------ #
    def _plan_by_rules(self, instruction: str) -> Optional[Plan]:
        text = instruction.lower().strip()

        if self._is_clean_table_intent(text):
            return self._clean_table_plan(instruction)

        spec = self._extract_put_intent(instruction)
        if spec is not None:
            return self._single_pick_plan(instruction, spec)

        return None

    @staticmethod
    def _is_clean_table_intent(text: str) -> bool:
        tidy = any(w in text for w in ("clean", "tidy", "clear", "整理", "清理", "收拾", "打扫"))
        surface = any(w in text for w in ("table", "desk", "桌面", "桌子", "台面"))
        return tidy and surface

    def _clean_table_plan(self, instruction: str) -> Plan:
        plan = Plan(
            instruction=instruction,
            constraints={"task": "clean_table", "ordering": "sequential"},
        )
        scene = SkillCall(
            skill="perceive_scene",
            args={"instruction": instruction},
            constraints={"role": "perception"},
            ref="scene",
        )
        plan.steps.append(scene)

        for i, spec in enumerate(CLEAN_TABLE_PLACEMENTS):
            plan.steps.append(
                SkillCall(
                    skill="pick_and_place",
                    args=dict(spec),
                    constraints={"depends_on": ["scene"], "ordering": f"step-{i + 1}"},
                    ref=f"pp{i}",
                )
            )
        return plan

    def _extract_put_intent(self, instruction: str) -> Optional[Dict[str, Any]]:
        """Recognise 'put/move <X> on/in/beside <Y>' and return a pick spec."""
        if not re.search(r"put|move|place|放|移动|拿", instruction, re.IGNORECASE):
            return None

        # Match by *position in the sentence*: the moved object precedes the
        # target (e.g. "put the bread into the bowl" -> bread -> bowl).
        found: List[tuple] = []
        for keywords, spec in _OBJECT_VOCAB:
            for kw in keywords:
                idx = instruction.find(kw)
                if idx != -1:
                    found.append((idx, spec))
                    break
        if len(found) < 2:
            return None
        found.sort(key=lambda item: item[0])

        object_spec, target_spec = found[0][1], found[1][1]
        beside = bool(re.search(r"beside|旁边|next to|旁边", instruction, re.IGNORECASE))
        spec = {
            "object": object_spec,
            "target": target_spec,
            "placement_type": "beside" if beside else "on_center",
        }
        if beside:
            for placement in CLEAN_TABLE_PLACEMENTS:
                if (
                    placement.get("object") == object_spec
                    and placement.get("target") == target_spec
                    and placement.get("placement_type") == "beside"
                ):
                    spec["offset"] = list(placement.get("offset", [0, 0, 0]))
                    break
        return spec

    def _single_pick_plan(self, instruction: str, spec: Dict[str, Any]) -> Plan:
        plan = Plan(
            instruction=instruction,
            constraints={"task": "pick_and_place", "ordering": "sequential"},
        )
        plan.steps.append(
            SkillCall(
                skill="perceive_scene",
                args={"instruction": instruction},
                constraints={"role": "perception"},
                ref="scene",
            )
        )
        plan.steps.append(
            SkillCall(
                skill="pick_and_place",
                args=dict(spec),
                constraints={"depends_on": ["scene"]},
                ref="pp0",
            )
        )
        return plan

    # ------------------------------------------------------------------ #
    # LLM backend
    # ------------------------------------------------------------------ #
    def _plan_by_llm(self, instruction: str) -> Optional[Plan]:
        prompt = self._build_llm_prompt(instruction)
        raw = self.llm.chat(prompt)
        data = self._parse_json(raw)
        if not isinstance(data, dict) or "steps" not in data:
            return None

        plan = Plan(
            instruction=instruction,
            constraints=data.get("constraints", {}),
        )
        for item in data["steps"]:
            skill = item.get("skill")
            if not self.registry.has_skill(skill):
                continue
            plan.steps.append(
                SkillCall(
                    skill=skill,
                    args=item.get("args", {}),
                    constraints=item.get("constraints", {}),
                    ref=item.get("ref"),
                )
            )
        if not plan.steps:
            return None
        return plan

    def _build_llm_prompt(self, instruction: str) -> str:
        manifest = json.dumps(self.registry.skill_manifest(), ensure_ascii=False, indent=2)
        return f"""You are a robot task planner. Convert the user's request into an ordered list of SKILL calls.

AVAILABLE SKILLS (JSON schema):
{manifest}

Respond with ONLY a JSON object, no other text:
{{
  "constraints": {{"ordering": "sequential"}},
  "steps": [
    {{"skill": "perceive_scene", "args": {{...}}, "constraints": {{...}}, "ref": "scene"}},
    {{"skill": "pick_and_place", "args": {{...}}, "constraints": {{"depends_on": ["scene"]}}, "ref": "pp0"}}
  ]
}}

USER REQUEST: {instruction}
JSON:"""

    @staticmethod
    def _parse_json(raw: str) -> Any:
        """Best-effort JSON extraction, tolerant of code fences / prose."""
        text = raw.strip()
        fence = re.search(r"```(?:json)?\s*(.*?)```", text, re.DOTALL)
        if fence:
            text = fence.group(1).strip()
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            start, end = text.find("{"), text.rfind("}")
            if start != -1 and end > start:
                try:
                    return json.loads(text[start : end + 1])
                except json.JSONDecodeError:
                    return None
        return None
