"""Offline geometry checks; no detector, camera, planner, or robot is started."""
import types
import unittest

import numpy as np

from book_grasp_1_1_local import _install_spine_center, _spine_center_from_mask


class SpineCenterTests(unittest.TestCase):
    def setUp(self):
        self.mask = np.zeros((120, 160), dtype=bool)
        self.mask[20:100, 70:81] = True
        self.depth = np.full(self.mask.shape, 800., dtype=float)
        self.k = np.array([[200., 0, 80], [0, 200., 60], [0, 0, 1]])

    def test_midline_uses_local_depth_and_accepts_both_depth_units(self):
        self.depth[~self.mask] = 3000
        self.depth[20:40, 70:81] = 1400
        for scale in (1., .001):
            result = _spine_center_from_mask(self.mask, self.depth * scale, self.k)
            np.testing.assert_allclose(result['uv'], [75, 59.5])
            np.testing.assert_allclose(result['point_camera_m'], [-.02, -.002, .8])

    def test_slanted_spine_and_disconnected_noise(self):
        self.mask[:] = False
        for row in range(20, 100):
            middle = 70 + (row - 20) // 8
            self.mask[row, middle-4:middle+5] = True
        self.mask[5:8, 120:123] = True
        result = _spine_center_from_mask(self.mask, self.depth, self.k)
        np.testing.assert_allclose(result['uv'], [74.5, 59.5], atol=.1)
        self.assertFalse(result['mask'][6, 121])

    def test_rejects_small_wide_clipped_and_invalid_depth_regions(self):
        for region in ((slice(85, 110), slice(70, 80)),
                       (slice(20, 100), slice(40, 100)),
                       (slice(40, 120), slice(70, 80))):
            mask = np.zeros_like(self.mask)
            mask[region] = True
            with self.assertRaises(ValueError):
                _spine_center_from_mask(mask, self.depth, self.k)
        for invalid in (0., np.nan):
            with self.assertRaises(ValueError):
                _spine_center_from_mask(self.mask, self.depth * invalid, self.k)
        self.depth[60:70, 70:81] = 1600
        with self.assertRaisesRegex(ValueError, '深度混入'):
            _spine_center_from_mask(self.mask, self.depth, self.k)

    def test_selection_transform_aliasing_and_failed_frame_cache(self):
        calls = []
        detections = [(.7, self.mask, None)]

        def raw(*args, prompt):
            calls.append(prompt)
            return detections

        def original(meta, points, transform):
            meta['centroid'] = np.eye(4)
            meta['centroid'][:3, 3] = [.2, .3, .9]
            meta['grasp_0'] = meta['centroid']
            for axis in 'xyz':
                for side in (1, 2):
                    meta[f'bbox_plane_{axis}{side}'] = meta['centroid'].copy()
            return {'point_mask': np.ones(len(points), dtype=bool)}

        def to_base(points, transform):
            return (points @ transform[:3, :3].T + transform[:3, 3],
                    transform[:3, :3], transform[:3, 3])

        geom = types.SimpleNamespace(
            BOOK_GRASP_MODE='cart+', fill_book_meta=original,
            _load_sam3_detector=lambda: (None, None, types.SimpleNamespace(fromarray=lambda x: x)),
            _sam3_raw_books=raw, _camera_to_base=to_base)
        _install_spine_center(geom)
        transform = np.eye(4)
        self.assertIsNotNone(geom.segment_cart_book_with_sam3(
            np.zeros((120, 160, 3), dtype=np.uint8), self.depth, self.k, transform))
        self.assertEqual(calls, ['book spine'])
        # Geometry must use the latest extrinsics, not those cached at segmentation.
        transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        transform[:3, 3] = [.1, .2, .3]
        meta = {}
        result = geom.fill_book_meta(meta, np.zeros((32, 3)), transform)
        np.testing.assert_allclose(meta['centroid'][:3, 3], [-.02, -.002, .8])
        np.testing.assert_allclose(meta['grasp_0'], meta['centroid'])
        np.testing.assert_allclose(result['center_base_mm'], [102, 180, 1100])
        self.assertEqual(result['center_kind'], 'visible_book_spine_midline')
        self.assertTrue(result['point_mask'].all())
        detections.clear()
        self.assertIsNone(geom.segment_cart_book_with_sam3(
            np.zeros((120, 160, 3), dtype=np.uint8), self.depth, self.k, transform))
        with self.assertRaisesRegex(ValueError, '本轮没有'):
            geom.fill_book_meta({}, np.zeros((32, 3)), transform)
        geom.BOOK_GRASP_MODE = 'cart'
        self.assertNotIn('center_kind', geom.fill_book_meta({}, np.zeros((32, 3)), transform))


if __name__ == '__main__':
    unittest.main()
