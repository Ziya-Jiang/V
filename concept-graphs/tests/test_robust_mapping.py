import unittest

import numpy as np
import open3d as o3d
import torch

from conceptgraph.slam.mapping import match_detections_to_objects
from conceptgraph.slam.slam_classes import DetectionList, MapObjectList
from conceptgraph.slam.utils import (
    align_point_cloud_to_target,
    compute_overlap_matrix_general,
    filter_gobs,
)


def make_point_cloud(points: np.ndarray) -> o3d.geometry.PointCloud:
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud


def make_object(points: np.ndarray, class_name: str) -> dict:
    point_cloud = make_point_cloud(points)
    return {
        "pcd": point_cloud,
        "bbox": point_cloud.get_axis_aligned_bounding_box(),
        "class_name": class_name,
    }


class RobustMappingTest(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(7)
        self.target_points = rng.uniform(
            low=[-0.05, -0.03, -0.02],
            high=[0.05, 0.03, 0.02],
            size=(300, 3),
        )

    def test_overlap_tolerates_small_depth_shift(self):
        target = make_object(self.target_points, "cup")
        shifted = make_object(
            self.target_points + np.array([0.03, 0.0, 0.0]),
            "cup",
        )

        overlap = compute_overlap_matrix_general(
            MapObjectList([target]),
            DetectionList([shifted]),
            overlap_distance=0.04,
        )

        self.assertGreater(overlap[0, 0], 0.9)

    def test_class_gate_rejects_higher_scoring_wrong_class(self):
        detections = DetectionList([{"class_name": "cup"}])
        objects = MapObjectList(
            [{"class_name": "bowl"}, {"class_name": "cup"}]
        )
        aggregate = torch.tensor([[1.5, 1.1]])

        matches = match_detections_to_objects(
            aggregate,
            detection_threshold=1.0,
            detection_list=detections,
            objects=objects,
            class_aware_matching=True,
        )

        self.assertEqual(matches, [1])

    def test_guarded_icp_aligns_small_shift(self):
        target = make_point_cloud(self.target_points)
        source = make_point_cloud(
            self.target_points + np.array([0.03, -0.01, 0.005])
        )
        before_error = np.linalg.norm(
            np.asarray(source.get_center()) - np.asarray(target.get_center())
        )

        aligned, metrics = align_point_cloud_to_target(
            source,
            target,
            max_correspondence_distance=0.04,
            max_iteration=30,
            min_fitness=0.25,
            max_translation=0.08,
        )
        after_error = np.linalg.norm(
            np.asarray(aligned.get_center()) - np.asarray(target.get_center())
        )

        self.assertTrue(metrics["accepted"])
        self.assertLess(after_error, before_error)
        self.assertLess(after_error, 0.01)

    def test_guarded_icp_rejects_large_shift(self):
        target = make_point_cloud(self.target_points)
        source = make_point_cloud(
            self.target_points + np.array([0.20, 0.0, 0.0])
        )

        aligned, metrics = align_point_cloud_to_target(
            source,
            target,
            max_correspondence_distance=0.04,
            max_iteration=20,
            min_fitness=0.25,
            max_translation=0.08,
        )

        self.assertFalse(metrics["accepted"])
        np.testing.assert_allclose(
            np.asarray(aligned.points),
            np.asarray(source.points),
        )

    def test_large_object_bbox_exception_only_keeps_table(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        gobs = {
            "xyxy": np.array([[0, 0, 90, 90], [0, 0, 90, 90]]),
            "class_id": np.array([0, 1]),
            "classes": ["table", "cup"],
            "mask": np.ones((2, 100, 100), dtype=bool),
            "confidence": np.array([0.9, 0.9]),
            "detection_class_labels": ["table 0", "cup 1"],
            "captions": [],
        }

        filtered = filter_gobs(
            gobs,
            image,
            skip_bg=False,
            BG_CLASSES=[],
            mask_area_threshold=25,
            max_bbox_area_ratio=0.5,
            mask_conf_threshold=0.25,
            large_object_classes=["table"],
            large_object_max_bbox_area_ratio=0.85,
        )

        self.assertEqual(filtered["class_id"].tolist(), [0])


if __name__ == "__main__":
    unittest.main()
