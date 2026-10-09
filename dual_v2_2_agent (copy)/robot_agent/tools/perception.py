"""Scene-perception primitives.

Three tools turn pixels into robot-base object metadata:

* ``ground``    -- detect the requested semantic objects (Grounding-DINO + SAM2)
                  and return masks / boxes / class ids.
* ``estimate``  -- estimate 6-DoF poses and grasp poses for every grounded
                  object (VGGT metric depth + RAM).
* ``localize``  -- transform the camera-frame poses into robot-base coordinates.

Together these reproduce the legacy Step1 → Step2 → Step3 pipeline, but each is
an independent, typed tool a Skill can call (and reorder / reuse).
"""

from __future__ import annotations

import os
from typing import Any, Dict, List

from ..base import Tool, ToolResult, object_schema
from ..context import ROBOTRAM_ROOT, ensure_robotram_on_path


def _load_json(path: str) -> Any:
    import json

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


class Perceiver:
    """Facade over the legacy Step1-3 functions, threading state via ``ctx.scene``."""

    def __init__(self, ctx) -> None:
        self.ctx = ctx

    # ------------------------------------------------------------------ #
    def _targets_from_output(self) -> List[Dict[str, Any]]:
        output_dir = str(self.ctx.output_dir)
        objects_path = os.path.join(output_dir, "vlm_objects.json")
        categories_path = os.path.join(output_dir, "vlm_categories.json")
        targets = _load_json(objects_path) if os.path.exists(objects_path) else []
        categories = _load_json(categories_path) if os.path.exists(categories_path) else []
        if not categories:
            categories = [
                {"id": i, "name": t.get("name", ""), "color": t.get("color", ""), "category": t.get("category", "")}
                for i, t in enumerate(targets)
            ]
        return targets, categories

    # ------------------------------------------------------------------ #
    def ground(self, instruction, targets=None) -> ToolResult:
        """Detect semantic objects and populate ctx.scene with masks/boxes."""
        if targets is None:
            targets, _ = self._targets_from_output()
        if not targets:
            return ToolResult.failure("no targets to ground")

        ensure_robotram_on_path()
        from step1_grounding import grounding_create_output_dir, grounding_main

        output_dir, grounding_debug_dir = grounding_create_output_dir(
            instruction, str(self.ctx.output_dir), self.ctx.logger
        )
        try:
            (rgb, left, right, depth, masks, xyxys, class_ids) = grounding_main(
                instruction,
                output_dir,
                grounding_debug_dir,
                targets,
                self.ctx.robot,
                None,  # legacy 'vlm' slot, unused by grounding_main
                self.ctx.extern_camera,
                self.ctx.vgm,
                only_grounding=True,
                logger=self.ctx.logger,
                enable_debug=False,
            )
        except Exception as error:  # noqa: BLE001
            return ToolResult.failure("grounding failed", str(error))

        # RAM consumes the Orbbec frames through extern_camera_class, so mirror
        # the legacy data threading explicitly here.
        camera = self.ctx.extern_camera_class
        if camera is not None:
            camera.rgb_image = rgb
            camera.left_image = left
            camera.right_image = right
            camera.depth_map = depth

        self.ctx.scene.update(
            {
                "rgb": rgb,
                "left": left,
                "right": right,
                "depth": depth,
                "masks": masks,
                "xyxys": xyxys,
                "class_ids": class_ids,
            }
        )
        return ToolResult.success(
            f"grounded {len(targets)} semantic targets",
            payload={"num_targets": len(targets)},
        )

    # ------------------------------------------------------------------ #
    def estimate(self, categories=None) -> ToolResult:
        """Estimate object + grasp poses; populate ctx.scene with objects_meta."""
        scene = self.ctx.scene
        masks = scene.get("masks")
        if masks is None:
            return ToolResult.failure("estimate requires ground() to run first")

        if categories is None:
            _, categories = self._targets_from_output()

        ensure_robotram_on_path()
        from step2_ram import ram_main

        try:
            objects_meta, _ = ram_main(
                self.ctx.instruction,
                str(self.ctx.output_dir),
                os.path.join(str(self.ctx.output_dir), "ram"),
                str(ROBOTRAM_ROOT),
                self.ctx.extern_camera2,
                self.ctx.extern_camera_class,
                categories,
                masks,
                scene["xyxys"],
                scene["class_ids"],
                self.ctx.ram_model,
                self.ctx.ram_template,
                self.ctx.ram_io,
                device=self.ctx.vggt_device,
                dtype=self.ctx.vggt_dtype,
                vggt_model=self.ctx.vggt_model,
                use_vggt_service=False,
                vggt_service_url="http://localhost:5001",
                logger=self.ctx.logger,
                enable_debug=False,
                camera_pose_correction_callback=getattr(self.ctx, "camera_pose_correction_callback", None),
                stop_after_camera_pose_correction=False,
            )
        except Exception as error:  # noqa: BLE001
            return ToolResult.failure("pose estimation failed", str(error))

        # Build the object-id -> {name,color,category} mapping and the executor's
        # object-id -> category mapping, both consumed by later skills.
        vlm_categories_dict = {}
        for item in categories:
            vlm_categories_dict[f"object_{item['id']}"] = {
                "name": item.get("name", ""),
                "color": item.get("color", ""),
                "category": item.get("category", ""),
            }
        object_id_to_type = {}
        for obj_key, info in vlm_categories_dict.items():
            object_id_to_type[obj_key] = info["category"]

        self.ctx.scene["objects_meta"] = objects_meta
        self.ctx.scene["vlm_categories_dict"] = vlm_categories_dict
        self.ctx.object_id_to_type = object_id_to_type

        # Persist objects_meta (mirrors the legacy pipeline) so a fresh process
        # can resume from perception without re-running VGGT/RAM.
        try:
            import numpy as np

            np.save(os.path.join(str(self.ctx.output_dir), "ram_objects_meta.npy"), objects_meta)
        except Exception as error:  # noqa: BLE001
            self.ctx.logger.warning("failed to persist objects_meta: %s", error)

        return ToolResult.success(
            f"estimated poses for {len(vlm_categories_dict)} objects",
            payload={"categories": vlm_categories_dict},
        )

    # ------------------------------------------------------------------ #
    def localize(self) -> ToolResult:
        """Transform camera-frame poses into robot-base coordinates."""
        scene = self.ctx.scene
        objects_meta = scene.get("objects_meta")
        if objects_meta is None:
            return ToolResult.failure("localize requires estimate() to run first")

        ensure_robotram_on_path()
        from step3_read_object_info import read_object_main

        try:
            objects_info = read_object_main(
                objects_meta,
                str(self.ctx.output_dir),
                False,
                c2w_matrix=self.ctx.calibration["T_base_cam"],
                camera_pose_unit="m",
            )
        except Exception as error:  # noqa: BLE001
            return ToolResult.failure("localization failed", str(error))

        self.ctx.scene["objects_info"] = objects_info

        # Persist so a later, fresh process (e.g. a separate DSH tool call) can
        # re-hydrate the world-frame metadata without re-running perception.
        import json

        objects_json = os.path.join(str(self.ctx.output_dir), "objects_info_world.json")
        try:
            with open(objects_json, "w", encoding="utf-8") as f:
                json.dump(objects_info, f, indent=2, ensure_ascii=False)
        except Exception as error:  # noqa: BLE001
            self.ctx.logger.warning("failed to persist objects_info: %s", error)

        return ToolResult.success(
            f"localized {len(objects_info)} objects to robot base",
            payload={"objects": objects_info},
        )


# --------------------------------------------------------------------------- #
# Tool definitions
# --------------------------------------------------------------------------- #
def _ground(ctx, instruction, targets=None) -> ToolResult:
    return Perceiver(ctx).ground(instruction, targets)


def _estimate(ctx, categories=None) -> ToolResult:
    return Perceiver(ctx).estimate(categories)


def _localize(ctx) -> ToolResult:
    return Perceiver(ctx).localize()


PERCEPTION_TOOLS: List[Tool] = [
    Tool(
        name="ground",
        description="Detect the requested semantic objects in the current camera frame (Grounding-DINO + SAM2). Returns masks, boxes, and class ids for each target.",
        parameters=object_schema(
            {
                "instruction": {"type": "string", "description": "scene-level prompt, e.g. 'clean the table'"},
                "targets": {"type": "array", "items": {"type": "object"}, "description": "objects to detect: [{name, color, category}]"},
            },
            required=["instruction"],
        ),
        fn=_ground,
        returns="masks, xyxys, class_ids",
        category="perception",
    ),
    Tool(
        name="estimate_grasp_poses",
        description="Estimate 6-DoF object poses and grasp poses for every grounded object (VGGT metric depth + RAM). Populates objects_meta.",
        parameters=object_schema(
            {"categories": {"type": "array", "items": {"type": "object"}}},
            required=[],
        ),
        fn=_estimate,
        returns="objects_meta",
        category="perception",
    ),
    Tool(
        name="localize",
        description="Transform camera-frame object/grasp poses into robot-base coordinates using the hand-eye calibration.",
        parameters=object_schema({}, required=[]),
        fn=_localize,
        returns="objects_info",
        category="perception",
    ),
]


__all__ = ["PERCEPTION_TOOLS", "Perceiver"]
