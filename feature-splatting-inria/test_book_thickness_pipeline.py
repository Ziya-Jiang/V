import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import liblzfse
import numpy as np

from book_thickness_pipeline import (
    GRIPPER_PAD_HEIGHT_MM,
    BookTrack,
    Detection,
    camera_to_world,
    compute_grasp_pose,
    decode_record3d,
    estimate_spine_normal,
    extract_plane_candidates,
    fit_spine_geometry,
    gripper_wireframe,
    load_depth_image,
    load_intrinsic_matrix,
    normal_angle_deg,
    project_camera_points,
    project_world_point,
    ray_plane_intersection,
    select_grasp_poses,
)

GRASP_INTRINSIC = np.array(
    [[1000.0, 0.0, 50.0], [0.0, 1000.0, 200.0], [0.0, 0.0, 1.0]]
)


class BookThicknessGeometryTests(unittest.TestCase):
    def test_spine_plane_consensus_ignores_visible_side_plane(self) -> None:
        rng = np.random.default_rng(7)
        mask = np.zeros((400, 100), dtype=np.uint8)
        cv2.rectangle(mask, (30, 10), (70, 390), 1, thickness=cv2.FILLED)
        filled, geometry = fit_spine_geometry(mask.astype(bool))
        detection = Detection(
            frame=0,
            shelf=1,
            score=0.9,
            mask=filled,
            box_xyxy=np.array([30, 10, 70, 390], dtype=np.float64),
            geometry=geometry,
        )

        spine_x = rng.uniform(-0.020, 0.005, 600)
        spine_y = rng.uniform(-0.18, 0.18, 600)
        spine_z = 1.0 + rng.normal(0, 0.0008, 600)
        side_x = 0.005 + rng.normal(0, 0.0008, 200)
        side_y = rng.uniform(-0.18, 0.18, 200)
        side_z = rng.uniform(1.0, 1.12, 200)
        detection.surface_points = np.vstack(
            (
                np.column_stack((spine_x, spine_y, spine_z)),
                np.column_stack((side_x, side_y, side_z)),
            )
        )
        detection.surface_pixels_uv = np.vstack(
            (
                np.column_stack(
                    (rng.uniform(31, 55, 600), rng.uniform(15, 385, 600))
                ),
                np.column_stack(
                    (rng.uniform(56, 69, 200), rng.uniform(15, 385, 200))
                ),
            )
        )

        detection.plane_candidates = extract_plane_candidates(detection)
        consensus = estimate_spine_normal([detection])

        self.assertGreaterEqual(len(detection.plane_candidates), 2)
        self.assertLess(
            normal_angle_deg(consensus, np.array([0.0, 0.0, 1.0])), 5.0
        )
        self.assertTrue(
            any(
                normal_angle_deg(
                    candidate.normal, detection.plane_candidates[0].normal
                )
                > 35
                for candidate in detection.plane_candidates[1:]
            )
        )

    def test_generic_uint16_depth_and_json_intrinsics(self) -> None:
        raw_depth = np.array([[0, 800], [1200, 1500]], dtype=np.uint16)
        with TemporaryDirectory() as directory:
            root = Path(directory)
            depth_path = root / "depth.png"
            intrinsics_path = root / "intrinsics.json"
            cv2.imwrite(str(depth_path), raw_depth)
            intrinsics_path.write_text(
                json.dumps({"fx": 1000, "fy": 1001, "cx": 640, "cy": 360})
            )

            depth, scale = load_depth_image(depth_path, None)
            intrinsic = load_intrinsic_matrix(intrinsics_path)

        self.assertEqual(scale, 0.001)
        np.testing.assert_allclose(depth, raw_depth.astype(np.float32) * 0.001)
        np.testing.assert_allclose(
            intrinsic,
            [[1000, 0, 640], [0, 1001, 360], [0, 0, 1]],
        )

    def test_depth_decoding_does_not_require_confidence(self) -> None:
        expected = np.linspace(0.8, 1.2, 12, dtype=np.float32).reshape(3, 4)
        with TemporaryDirectory() as directory:
            rgbd = Path(directory) / "rgbd"
            rgbd.mkdir()
            (rgbd / "7.depth").write_bytes(
                liblzfse.compress(expected.tobytes())
            )

            depth, confidence = decode_record3d(
                Path(directory), 7, expected.shape, use_confidence=False
            )

        np.testing.assert_array_equal(depth, expected)
        np.testing.assert_array_equal(
            confidence, np.full(expected.shape, 2, dtype=np.uint8)
        )

    def test_rotated_spine_width(self) -> None:
        mask = np.zeros((640, 480), dtype=np.uint8)
        corners = cv2.boxPoints(((240, 320), (42, 310), -11))
        cv2.fillConvexPoly(mask, np.rint(corners).astype(np.int32), 1)

        _, geometry = fit_spine_geometry(mask.astype(bool))

        self.assertAlmostEqual(geometry.width_px, 42, delta=1.5)
        self.assertAlmostEqual(geometry.length_px, 0.94 * 310, delta=4)

    def test_ray_plane_width_is_metric(self) -> None:
        intrinsic = np.array(
            [[1000.0, 0.0, 500.0], [0.0, 1000.0, 500.0], [0.0, 0.0, 1.0]]
        )
        pixels = np.array([[490.0, 500.0], [510.0, 500.0]])

        points = ray_plane_intersection(
            pixels, intrinsic, np.array([0.0, 0.0, 1.0]), -1.0
        )

        self.assertAlmostEqual(np.linalg.norm(points[1] - points[0]), 0.020, places=9)

    def test_pose_projection_round_trip(self) -> None:
        metadata = {"poses": [[0.0, 0.0, 0.0, 1.0, 0.2, 0.3, -0.4]]}
        intrinsic = np.array(
            [[900.0, 0.0, 720.0], [0.0, 900.0, 960.0], [0.0, 0.0, 1.0]]
        )
        transform = camera_to_world(metadata, 0)
        point_camera = np.array([0.1, -0.05, 1.2])
        point_world = (
            transform[:3, :3] @ point_camera + transform[:3, 3]
        )

        pixel = project_world_point(point_world, intrinsic, transform)

        expected = np.array(
            [
                900.0 * point_camera[0] / point_camera[2] + 720.0,
                900.0 * point_camera[1] / point_camera[2] + 960.0,
            ]
        )
        np.testing.assert_allclose(pixel, expected, atol=1e-9)


class BookGraspPoseTests(unittest.TestCase):
    def build_pose(
        self,
        insertion_mm: float = 20.0,
        approach_mm: float = 70.0,
        spine_offset_mm: float = 0.0,
    ):
        mask = np.zeros((400, 100), dtype=np.uint8)
        cv2.rectangle(mask, (30, 40), (70, 360), 1, thickness=cv2.FILLED)
        filled, geometry = fit_spine_geometry(mask.astype(bool))
        detection = Detection(
            frame=0,
            shelf=2,
            score=0.9,
            mask=filled,
            box_xyxy=np.array([30, 40, 70, 360], dtype=np.float64),
            geometry=geometry,
        )
        normal = np.array([0.0, -0.2, 0.98])
        detection.plane_normal = normal / np.linalg.norm(normal)
        detection.plane_offset = -1.0
        detection.thickness_mm = 40.0
        track = BookTrack(
            book_id="S2-01",
            shelf=2,
            reference=detection,
            measurements=[detection],
            match_residual_px=[0.0],
        )
        summary = {
            "quality": "low",
            "thickness_mm": 40.0,
            "recommended_gripper_opening_mm": 46.0,
        }
        pose = compute_grasp_pose(
            track,
            summary,
            GRASP_INTRINSIC,
            insertion_mm,
            approach_mm,
            spine_offset_mm,
        )
        self.assertIsNotNone(pose)
        return pose, detection

    def test_grasp_frame_is_orthonormal_and_plane_aligned(self) -> None:
        pose, detection = self.build_pose()

        np.testing.assert_allclose(
            pose.rotation.T @ pose.rotation, np.eye(3), atol=1e-9
        )
        self.assertAlmostEqual(float(np.linalg.det(pose.rotation)), 1.0, places=9)
        np.testing.assert_allclose(
            pose.approach_axis, detection.plane_normal, atol=1e-12
        )
        for point in (pose.left_contact, pose.right_contact, pose.contact_center):
            self.assertAlmostEqual(
                float(point @ detection.plane_normal + detection.plane_offset),
                0.0,
                places=12,
            )
        self.assertLess(pose.spine_axis[1], -0.9)
        self.assertAlmostEqual(
            float(np.linalg.norm(pose.right_contact - pose.left_contact)) * 1000,
            40.0,
            delta=1.5,
        )
        np.testing.assert_allclose(
            pose.tcp,
            pose.contact_center + pose.approach_axis * 0.020,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            pose.pregrasp,
            pose.contact_center - pose.approach_axis * 0.070,
            atol=1e-12,
        )

    def test_spine_offset_lifts_grasp_along_the_book(self) -> None:
        centred, _ = self.build_pose()
        lifted, _ = self.build_pose(spine_offset_mm=30.0)

        shift = lifted.contact_center - centred.contact_center
        np.testing.assert_allclose(
            shift, centred.spine_axis * 0.030, atol=1e-12
        )
        self.assertLess(lifted.contact_center[1], centred.contact_center[1])

    def test_gripper_wireframe_matches_commanded_opening(self) -> None:
        pose, _ = self.build_pose()
        paths = gripper_wireframe(pose, 45.0)
        fingers = [path for role, path in paths if role == "finger"]
        contact = next(path for role, path in paths if role == "contact")

        self.assertEqual(len(fingers), 2)
        offsets = [
            (finger - pose.contact_center) @ pose.close_axis
            for finger in fingers
        ]
        self.assertAlmostEqual(
            float(offsets[1].mean() - offsets[0].mean()) * 1000,
            pose.opening_mm,
            places=6,
        )
        for finger, offset in zip(fingers, offsets):
            np.testing.assert_allclose(offset, offset[0], atol=1e-12)
            depth = (finger - pose.contact_center) @ pose.approach_axis
            height = (finger - pose.contact_center) @ pose.spine_axis
            self.assertAlmostEqual(
                float(depth.max() - depth.min()) * 1000, 45.0, places=6
            )
            self.assertAlmostEqual(
                float(height.max() - height.min()) * 1000,
                GRIPPER_PAD_HEIGHT_MM,
                places=6,
            )
            self.assertGreater(float(depth.max()) * 1000, 0.0)
            self.assertLess(float(depth.min()) * 1000, 0.0)
        np.testing.assert_allclose(
            contact, np.vstack((pose.left_contact, pose.right_contact))
        )

    def test_camera_projection_rejects_points_behind_the_camera(self) -> None:
        point = np.array([0.1, -0.05, 1.2])

        pixel = project_camera_points(point[None, :], GRASP_INTRINSIC)[0]
        behind = project_camera_points(
            np.array([[0.1, -0.05, -1.2]]), GRASP_INTRINSIC
        )[0]

        np.testing.assert_allclose(
            pixel, project_world_point(point, GRASP_INTRINSIC, np.eye(4)), atol=1e-9
        )
        self.assertTrue(np.all(np.isnan(behind)))

    def test_selects_one_grasp_pose_by_case_insensitive_book_id(self) -> None:
        pose, _ = self.build_pose()

        selected, normalized = select_grasp_poses([pose], " s2-01 ")

        self.assertEqual(len(selected), 1)
        self.assertIs(selected[0], pose)
        self.assertEqual(normalized, "S2-01")

    def test_rejects_unknown_grasp_book_id(self) -> None:
        pose, _ = self.build_pose()

        with self.assertRaisesRegex(ValueError, "Available book IDs: S2-01"):
            select_grasp_poses([pose], "S3-99")


if __name__ == "__main__":
    unittest.main()
