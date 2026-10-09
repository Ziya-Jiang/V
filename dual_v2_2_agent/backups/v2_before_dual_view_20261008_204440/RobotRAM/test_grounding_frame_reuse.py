from __future__ import annotations

import sys
import tempfile
import types
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace

import numpy as np


ROBOTRAM_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROBOTRAM_ROOT))
sys.path.insert(0, str(ROBOTRAM_ROOT / 'scripts'))

image_utils = types.ModuleType("utils.image_utils")
image_utils.save_depth_npy = lambda *_args, **_kwargs: None
image_utils.save_rgb_png = lambda *_args, **_kwargs: None
image_utils.draw_label_at_mask_center = lambda image, *_args, **_kwargs: image
sys.modules["utils.image_utils"] = image_utils

from scripts.step1_grounding import grounding_main


class _Camera:
    def __init__(self) -> None:
        self.rgb = np.zeros((3, 4, 3), dtype=np.uint8)
        self.left_image = self.rgb.copy()
        self.right_image = self.rgb.copy()

    def capture_image(self):
        return self.rgb, np.ones((3, 4), dtype=np.float32)


class _Detections(SimpleNamespace):
    def __len__(self):
        return len(self.mask)


class _PreparedGrounding:
    def __init__(self) -> None:
        self.prepare_calls = 0
        self.prompt_calls = []

    def prepare_image(self, image) -> None:
        self.prepare_calls += 1
        self.prepared_image = image

    def grounding_prepared(self, prompt):
        self.prompt_calls.append(prompt)
        mask = np.zeros((1, 3, 4), dtype=bool)
        mask[0, 1, 1] = True
        return _Detections(
            mask=mask,
            xyxy=np.array([[0, 0, 2, 2]], dtype=np.float32),
            class_id=np.array([0], dtype=np.int64),
        )

    def grounding(self, _image, _prompt):
        raise AssertionError("legacy grounding path should not run")


class _RankedGrounding(_PreparedGrounding):
    def grounding_prepared(self, prompt):
        self.prompt_calls.append(prompt)
        masks = np.zeros((2, 3, 4), dtype=bool)
        masks[0, 0, 0] = True
        masks[1, 2, 3] = True
        return _Detections(
            mask=masks,
            xyxy=np.array(
                [[0, 0, 1, 1], [2, 1, 4, 3]],
                dtype=np.float32,
            ),
            confidence=np.array([0.37, 0.73], dtype=np.float32),
            class_id=np.array([4, 9], dtype=np.int64),
        )


class GroundingFrameReuseTests(unittest.TestCase):
    def test_v2_instances_skip_dino_and_sam2_preparation(self):
        import scripts.step1_grounding as step1
        vgm=_PreparedGrounding();camera=_Camera()
        def segment(rgb,depth,k,c2w):
            self.assertIs(rgb,camera.rgb)
            return np.ones((3,4),bool),np.array([0,0,4,3])
        segment.instance_first=True
        with patch.object(step1.book_geom,'BOOK_GRASP_MODE','cart+'), \
             patch.object(step1.book_geom,'segment_cart_book_with_sam3',segment), \
             tempfile.TemporaryDirectory() as out:
            result=grounding_main('book',out,out,[{'name':'book','category':'book'}],
                robot=None,vlm=None,extern_camera=camera,vgm=vgm,logger=None,enable_debug=False)
        self.assertEqual(vgm.prepare_calls,0)
        self.assertEqual(vgm.prompt_calls,[])
        self.assertTrue(result[4][0].all())

    def test_v2_box_first_prepares_dino_only_and_passes_same_detector(self):
        import scripts.step1_grounding as step1
        vgm=_PreparedGrounding();vgm.box_prepare_calls=0
        def prepare_boxes(image):
            vgm.box_prepare_calls+=1
            vgm.box_rgb=image
        vgm.prepare_boxes_image=prepare_boxes
        def segment(rgb,depth,k,c2w,*,box_detector):
            self.assertIs(box_detector,vgm)
            self.assertIs(rgb,vgm.box_rgb)
            mask=np.ones((3,4),bool)
            return mask,np.array([0,0,4,3])
        segment.box_first=True
        with patch.object(step1.book_geom,'BOOK_GRASP_MODE','cart+'), \
             patch.object(step1.book_geom,'segment_cart_book_with_sam3',segment), \
             tempfile.TemporaryDirectory() as out:
            result=grounding_main('book',out,out,[{'name':'book','category':'book'}],
                robot=None,vlm=None,extern_camera=_Camera(),vgm=vgm,logger=None,enable_debug=False)
        self.assertEqual(vgm.box_prepare_calls,1)
        self.assertEqual(vgm.prepare_calls,0)
        self.assertEqual(vgm.prompt_calls,[])
        self.assertTrue(result[4][0].all())

    def test_one_frame_embedding_is_reused_for_selected_prompts(self) -> None:
        targets = [
            {"name": "green bowl", "color": "green", "category": "bowl"},
            {"name": "yellow plate", "color": "yellow", "category": "plate"},
            {"name": "blue cup", "color": "blue", "category": "cup"},
        ]
        vgm = _PreparedGrounding()

        with tempfile.TemporaryDirectory() as output_dir:
            result = grounding_main(
                "move cup beside bowl",
                output_dir,
                output_dir,
                targets,
                robot=None,
                vlm=None,
                extern_camera=_Camera(),
                vgm=vgm,
                logger=None,
                enable_debug=False,
                target_object_ids=[0, 2],
            )

        masks, class_ids = result[4], result[6]
        self.assertEqual(vgm.prepare_calls, 1)
        self.assertEqual(vgm.prompt_calls, ["green bowl", "blue cup"])
        self.assertEqual(masks.shape, (3, 3, 4))
        self.assertFalse(np.any(masks[1]))
        np.testing.assert_array_equal(class_ids, np.array([0, -1, 0]))

    def test_highest_confidence_candidate_is_selected(self) -> None:
        targets = [
            {"name": "yellow plate", "color": "yellow", "category": "plate"},
        ]
        vgm = _RankedGrounding()

        with tempfile.TemporaryDirectory() as output_dir:
            result = grounding_main(
                "把面包放到黄色盘子上",
                output_dir,
                output_dir,
                targets,
                robot=None,
                vlm=None,
                extern_camera=_Camera(),
                vgm=vgm,
                logger=None,
                enable_debug=False,
            )

        masks, xyxys, class_ids = result[4], result[5], result[6]
        self.assertFalse(masks[0, 0, 0])
        self.assertTrue(masks[0, 2, 3])
        np.testing.assert_array_equal(
            xyxys[0],
            np.array([2, 1, 4, 3], dtype=np.float32),
        )
        self.assertEqual(class_ids[0], 9)


if __name__ == "__main__":
    unittest.main()
