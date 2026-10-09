from __future__ import annotations

import logging
import unittest

import numpy as np

from robot_agent.base import ToolResult
from robot_agent.planner import Agent
from robot_agent.skills.pick_and_place import describe_action, describe_placement
from robot_agent.skills.tidy_table import (
    CLEAN_TABLE_PLACEMENTS,
    _perception_selectors,
    make_tidy_table,
)
from robot_agent.tools.perception import (
    filter_detected_categories,
    select_perception_categories,
)


class _Context:
    def __init__(self) -> None:
        self.instruction = "整理桌面"
        self.scene = {
            "vlm_categories_dict": {
                "object_0": {
                    "category": "bowl",
                    "color": "green",
                },
                "object_1": {
                    "category": "plate",
                    "color": "yellow",
                },
                "object_2": {
                    "category": "cup",
                    "color": "blue",
                },
            }
        }
        self.logger = logging.getLogger("test-scene-adaptive-tidy")
        self.events = []
        self.emit = self.events.append

    def find_object_id(self, category, color=None):
        for object_id, info in self.scene["vlm_categories_dict"].items():
            if info["category"] != category:
                continue
            if color and info["color"] != color:
                continue
            return object_id
        return None


class _Registry:
    def __init__(self) -> None:
        self.pick_calls = []
        self.perception_calls = []

    def skill(self, name):
        if name == "perceive_scene":
            def perceive(_ctx, **kwargs):
                self.perception_calls.append(kwargs)
                return ToolResult.success("perceived")

            return perceive
        if name == "pick_and_place":
            def pick(_ctx, **kwargs):
                self.pick_calls.append(kwargs)
                return ToolResult.success("placed")

            return pick
        raise KeyError(name)


class SceneAdaptiveTidyTests(unittest.TestCase):
    def test_selective_perception_preserves_original_object_ids(self) -> None:
        categories = [
            {"id": 0, "name": "green bowl", "category": "bowl", "color": "green"},
            {"id": 1, "name": "yellow plate", "category": "plate", "color": "yellow"},
            {"id": 2, "name": "blue cup", "category": "cup", "color": "blue"},
            {"id": 3, "name": "bread", "category": "bread", "color": ""},
        ]

        selected, object_ids = select_perception_categories(
            categories,
            [
                {"category": "cup", "color": "blue"},
                {"category": "plate", "color": "yellow"},
            ],
            target_count=4,
        )

        self.assertEqual([item["id"] for item in selected], [1, 2])
        self.assertEqual(object_ids, [1, 2])

    def test_perception_keeps_only_masked_objects_with_valid_poses(self) -> None:
        categories = [
            {"id": 0, "name": "green bowl", "category": "bowl"},
            {"id": 1, "name": "wooden fork", "category": "fork"},
            {"id": 2, "name": "blue cup", "category": "cup"},
        ]
        masks = np.zeros((3, 2, 2), dtype=bool)
        masks[0, 0, 0] = True
        masks[2, 0, 0] = True
        class_ids = np.array([0, -1, 0])
        objects_meta = {
            "0": {"centroid": np.eye(4)},
            "1": {"centroid": np.eye(4)},
        }

        detected, skipped = filter_detected_categories(
            categories,
            masks,
            class_ids,
            objects_meta,
        )

        self.assertEqual([item["id"] for item in detected], [0])
        self.assertEqual([item["id"] for item, _ in skipped], [1, 2])

    def test_clean_table_plan_uses_scene_adaptive_composite_skill(self) -> None:
        plan = Agent(registry=None).plan("请帮我整理桌面")

        self.assertEqual(len(plan.steps), 1)
        self.assertEqual(plan.steps[0].skill, "tidy_table")
        self.assertNotIn("placements", plan.steps[0].args)
        self.assertTrue(plan.constraints["sceneAdaptive"])

    def test_single_pick_plan_selects_only_object_and_target(self) -> None:
        plan = Agent(registry=None).plan("把蓝色杯子放到黄色盘子旁边")

        self.assertEqual(plan.steps[0].skill, "perceive_scene")
        self.assertEqual(
            plan.steps[0].args["selectors"],
            [
                {"category": "cup", "color": "blue"},
                {"category": "plate", "color": "yellow"},
            ],
        )

    def test_tidy_table_places_bread_on_plate(self) -> None:
        bread = next(
            item
            for item in CLEAN_TABLE_PLACEMENTS
            if item["object"]["category"] == "bread"
        )

        self.assertEqual(bread["target"]["category"], "plate")
        self.assertEqual(bread["target"]["color"], "yellow")
        self.assertNotIn(
            "fork",
            {
                item["object"]["category"]
                for item in CLEAN_TABLE_PLACEMENTS
            },
        )

    def test_hardware_operation_labels_name_objects_and_stage(self) -> None:
        context = _Context()

        operation = describe_placement(
            context,
            {"category": "bread"},
            {"category": "plate", "color": "yellow"},
            "on_center",
        )
        stage = describe_action(
            "move_above_grasp",
            "面包",
            "黄色盘子",
        )

        self.assertEqual(operation, "把面包放到黄色盘子上")
        self.assertEqual(stage, "移动到面包抓取点上方")

    def test_tidy_table_skips_missing_bread(self) -> None:
        registry = _Registry()
        context = _Context()
        skill = make_tidy_table(registry)

        result = skill(context)

        self.assertTrue(result.ok, result)
        self.assertEqual(len(registry.perception_calls), 1)
        self.assertEqual(
            registry.perception_calls[0]["selectors"],
            _perception_selectors(CLEAN_TABLE_PLACEMENTS),
        )
        self.assertNotIn(
            "fork",
            {
                selector.get("category")
                for selector in registry.perception_calls[0]["selectors"]
                if isinstance(selector, dict)
            },
        )
        self.assertEqual(len(registry.pick_calls), 2)
        self.assertEqual(
            [call["object"]["category"] for call in registry.pick_calls],
            ["bowl", "cup"],
        )
        self.assertEqual(len(result.payload["skipped"]), 1)
        self.assertEqual(
            {item["placement"]["object"]["category"] for item in result.payload["skipped"]},
            {"bread"},
        )
        self.assertEqual(
            [event["type"] for event in context.events],
            [
                "placement.skipped",
                "tidy.placement.started",
                "tidy.placement.completed",
                "tidy.placement.started",
                "tidy.placement.completed",
            ],
        )


if __name__ == "__main__":
    unittest.main()
