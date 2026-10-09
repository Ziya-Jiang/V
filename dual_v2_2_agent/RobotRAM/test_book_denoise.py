import unittest

import numpy as np

from book_grasp import (
    _nearest_book_depth_indices,
    _standing_book_body_mask,
    fill_standing_book_meta,
)


def _box(rng, count, low, high):
    return rng.uniform(low, high, size=(count, 3))


class BookDenoiseTests(unittest.TestCase):
    def test_depth_keeps_the_near_book_not_the_larger_background(self):
        rng = np.random.default_rng(0)
        depth = np.concatenate([
            rng.uniform(0.68, 0.78, 400),
            rng.uniform(1.15, 1.35, 2000),
        ])

        kept = depth[_nearest_book_depth_indices(depth)]

        self.assertGreater(len(kept), 300)
        self.assertLess(float(np.max(kept)), 0.95)

    def test_spatial_denoise_drops_the_cart_and_keeps_one_book(self):
        rng = np.random.default_rng(1)
        book = _box(rng, 800, [0.40, -0.30, -0.08], [0.63, -0.26, 0.08])
        cart = _box(rng, 2000, [0.30, -0.95, -0.22], [0.75, -0.55, -0.20])
        cloud = np.vstack([book, cart])

        kept = cloud[_standing_book_body_mask(cloud)]
        center = np.median(kept, axis=0)

        self.assertLess(abs(center[0] - 0.515), 0.04)
        self.assertLess(abs(center[1] + 0.28), 0.04)
        self.assertLess(len(kept), 1200)

    def test_fill_standing_book_rejects_background_depth(self):
        rng = np.random.default_rng(2)
        book_base = _box(rng, 900, [0.40, 0.68, -0.08], [0.63, 0.72, 0.08])
        cart_base = _box(rng, 2200, [0.35, 1.15, -0.02], [0.75, 1.35, 0.02])
        cloud_base = np.vstack([book_base, cart_base])
        # Camera Z is base Y, so the cart is the farther depth surface.
        rotation = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        ])
        points_cam = (rotation.T @ cloud_base.T).T
        c2w = np.eye(4)
        c2w[:3, :3] = rotation

        geometry = fill_standing_book_meta({}, points_cam, c2w)

        self.assertLess(geometry["thickness_mm"], 80.0)
        self.assertGreater(geometry["length_mm"], 150.0)
        self.assertLess(abs(geometry["center_base_mm"][1] - 700.0), 40.0)
        self.assertLess(geometry["denoised_points"], geometry["input_points"])
        self.assertEqual(len(geometry["point_mask"]), len(points_cam))


    def test_spine_samples_cover_the_whole_book(self):
        from book_grasp import spine_sample_offsets_mm

        offsets = spine_sample_offsets_mm([1.0, 0.0], 150.0)
        positions = np.array(offsets)[:, 0]

        self.assertGreaterEqual(len(offsets), 5)
        self.assertIn(0.0, np.round(positions, 6).tolist())
        self.assertLess(abs(float(np.min(positions)) + 60.0), 1.0)
        self.assertLess(abs(float(np.max(positions)) - 60.0), 1.0)
        self.assertTrue(np.all(np.abs(np.array(offsets)[:, 1]) < 1e-6))

    def test_cart_plus_approaches_along_plus_x(self):
        from book_grasp import book_cart_plus_targets, set_book_grasp_mode

        def xyz(x, y, z):
            return {"x": x, "y": y, "z": z}

        object_info = {
            "center_position": xyz(400.0, -300.0, 80.0),
            "bounding_box": {
                "bbox_plane_x1": xyz(390.0, -300.0, 80.0),
                "bbox_plane_x2": xyz(410.0, -300.0, 80.0),
                "bbox_plane_y1": xyz(400.0, -370.0, 80.0),
                "bbox_plane_y2": xyz(400.0, -230.0, 80.0),
                "bbox_plane_z1": xyz(400.0, -300.0, 20.0),
                "bbox_plane_z2": xyz(400.0, -300.0, 140.0),
            },
        }
        set_book_grasp_mode("cart+")
        try:
            targets = book_cart_plus_targets(object_info)
        finally:
            set_book_grasp_mode("shelf")

        self.assertLess(targets["pregrasp"][0], targets["grasp"][0])
        self.assertAlmostEqual(targets["grasp"][0], 400.0)
        self.assertAlmostEqual(targets["place"][1], -240.0)
        self.assertGreater(targets["grasp"][0] - targets["pregrasp"][0], 80.0)


if __name__ == "__main__":
    unittest.main()
