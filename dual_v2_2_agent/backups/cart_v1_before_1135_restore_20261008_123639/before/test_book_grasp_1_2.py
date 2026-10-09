"""Offline tilt geometry and detector fallback tests; no SAM3/robot startup."""
import types
import json
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import book_grasp_1_1_local as old
import book_grasp_1_2_local as new


def spine_mask(angle, center=(160, 150), width=16, length=160):
    mask = np.zeros((300, 320), dtype=np.uint8)
    corners = cv2.boxPoints((center, (width, length), angle))
    cv2.fillConvexPoly(mask, np.rint(corners).astype(np.int32), 1)
    return mask.astype(bool)


class TiltSpineTests(unittest.TestCase):
    def setUp(self):
        self.depth = np.full((300, 320), 800.)
        self.k = np.array([[300., 0, 160], [0, 300., 150], [0, 0, 1]])

    def test_center_for_vertical_diagonal_and_horizontal_spines(self):
        for angle in (0, 15, 30, 45, 60, 80, 90, -30, -60):
            with self.subTest(angle=angle):
                result = new._spine_center_from_mask(spine_mask(angle), self.depth, self.k)
                np.testing.assert_allclose(result['uv'], [160, 150], atol=1.)
                np.testing.assert_allclose(result['point_camera_m'], [0, 0, .8], atol=.003)
                self.assertGreater(result['depth_samples'], 12)
                self.assertNotIn('spine_center_recovery', result)

    def test_old_rejection_now_accepted_without_lowering_depth_requirements(self):
        mask = spine_mask(45)
        with self.assertRaises(ValueError):
            old._spine_center_from_mask(mask, self.depth, self.k)
        new._spine_center_from_mask(mask, self.depth, self.k)
        with self.assertRaises(ValueError):
            new._spine_center_from_mask(mask, np.zeros_like(self.depth), self.k)
        depth = self.depth.copy()
        depth[::2] = 1600
        with self.assertRaisesRegex(ValueError, 'background'):
            new._spine_center_from_mask(mask, depth, self.k)

    def test_center_patch_ignores_background_and_accepts_meter_units(self):
        mask = spine_mask(-45)
        depth = np.where(mask, .8, 3.)
        result = new._spine_center_from_mask(mask, depth, self.k)
        self.assertAlmostEqual(result['depth_m'], .8)

    def test_region_center_and_tilt_follow_stable_data_in_either_direction(self):
        for angle in (0, 30, -30, 90):
            mask = spine_mask(angle)
            _, pixels, _, _, _, t, _, _, _, _ = new._spine_frame(mask)
            for side in (-1, 1):
                depth = self.depth.copy()
                bad = pixels[(side*t > -3) & (side*t < 35)].astype(int)
                depth[bad[:, 1], bad[:, 0]] = 740.
                for scale in (1., .001):
                    with self.subTest(angle=angle, side=side, scale=scale):
                        result = new._spine_center_from_mask(mask, depth*scale, self.k)
                        self.assertEqual(result['spine_center_recovery'], 'stable_region')
                        self.assertEqual(result['spine_3d_status'], 'valid')
                        self.assertLess(side*result['spine_center_shift_px'], 0)
                        np.testing.assert_allclose(result['uv'], np.mean(result['spine_stable_region_uv'], axis=0))
                        self.assertAlmostEqual(result['depth_m'], .8)
                        self.assertLess(result['spine_axis_residual_mm'], 1.)
                        self.assertLess(abs(result['spine_axis_camera'][2]), .01)

    def test_missing_center_depth_recovers_from_a_continuous_region(self):
        depth = self.depth.copy()
        depth[140:165] = 0.
        result = new._spine_center_from_mask(spine_mask(0), depth, self.k)
        self.assertEqual(result['spine_center_recovery'], 'stable_region')
        self.assertLess(result['uv'][1], 140)
        self.assertAlmostEqual(result['depth_m'], .8)

    def test_disconnected_short_depth_patches_cannot_be_joined(self):
        depth = self.depth.copy()
        depth[100:103] = 0.
        depth[125:128] = 0.
        depth[155:158] = 0.
        depth[185:188] = 0.
        depth[210:213] = 0.
        # Force the original whole-line check to request regional recovery.
        with patch.object(new, 'spine_axis_3d', side_effect=ValueError('inconsistent')):
            result = new._spine_center_from_mask(spine_mask(0), depth, self.k)
        self.assertNotIn('spine_axis_camera', result)
        self.assertTrue(result['spine_center_recovery'].startswith('Failed:'))

    def test_smooth_depth_gradient_is_valid_inside_selected_region(self):
        depth = 800.+.3*(np.indices(self.depth.shape)[0]-150)
        depth[120:150] -= 70.
        result = new._spine_center_from_mask(spine_mask(0), depth, self.k)
        self.assertEqual(result['spine_center_recovery'], 'stable_region')
        self.assertGreater(abs(result['spine_axis_camera'][2]), .05)
        self.assertGreater(result['uv'][1], 150.)

    def test_recovery_config_is_hot_loaded_without_a_fixed_center_offset(self):
        depth = self.depth.copy()
        depth[115:130] = 740.
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory)/'geometry.json'
            with patch.object(new, '_GEOMETRY_CONFIG_PATH', config):
                settings = dict(enabled=False, min_region_fraction=.25, max_depth_jump_mm=25.)
                config.write_text(json.dumps(settings))
                result = new._spine_center_from_mask(spine_mask(0), depth, self.k)
                self.assertNotIn('spine_axis_camera', result)
                np.testing.assert_allclose(result['uv'], [160, 150], atol=1.)
                settings['enabled'] = True
                config.write_text(json.dumps(settings))
                result = new._spine_center_from_mask(spine_mask(0), depth, self.k)
                self.assertEqual(result['spine_center_recovery'], 'stable_region')
                settings['min_region_fraction'] = .8
                config.write_text(json.dumps(settings))
                result = new._spine_center_from_mask(spine_mask(0), depth, self.k)
                self.assertNotIn('spine_axis_camera', result)

    def test_rejects_cover_small_fragment_and_clipped_spine(self):
        for mask in (spine_mask(45, width=100), spine_mask(20, length=20),
                     spine_mask(45, center=(4, 40))):
            with self.assertRaises(ValueError):
                new._spine_center_from_mask(mask, self.depth, self.k)

    def test_rotation_mapping_preserves_original_center_and_mask(self):
        mask = spine_mask(40)
        for angle in (-45, 45, 90):
            rotated, matrix = new._rotate_for_detection(np.zeros((300, 320, 3), np.uint8), angle)
            rotated_mask = cv2.warpAffine(mask.astype(np.uint8), matrix,
                (rotated.shape[1], rotated.shape[0]), flags=cv2.INTER_NEAREST)
            restored = new._restore_detections([(.8, rotated_mask, None)], matrix, rotated.shape[:2], mask.shape)[0][1]
            self.assertGreater((restored & mask).sum() / (restored | mask).sum(), .93)
            result = new._spine_center_from_mask(restored, self.depth, self.k)
            np.testing.assert_allclose(result['uv'], [160, 150], atol=1.)

    def test_detector_fallback_and_proxy_write_through(self):
        mask = spine_mask(45)
        rgb = np.zeros((300, 320, 3), np.uint8)
        rotated, matrix = new._rotate_for_detection(rgb, -45)
        rotated_mask = cv2.warpAffine(mask.astype(np.uint8), matrix,
            (rotated.shape[1], rotated.shape[0]), flags=cv2.INTER_NEAREST)
        calls = []
        def raw(processor, to_numpy, image, prompt):
            calls.append((np.asarray(image).shape, prompt))
            return [] if len(calls) == 1 else [(.8, rotated_mask, None)]
        geom = types.SimpleNamespace(_sam3_raw_books=raw,
            _load_sam3_detector=lambda: (None, None, types.SimpleNamespace(fromarray=lambda x: x)))
        proxy = new._TiltDetector(geom)
        result = proxy._sam3_raw_books(None, None, rgb, prompt='book spine')
        self.assertEqual(len(calls), 2)
        self.assertEqual(result[0][1].shape, self.depth.shape)
        np.testing.assert_allclose(new._spine_center_from_mask(result[0][1], self.depth, self.k)['uv'], [160, 150], atol=1.)
        proxy.BOOK_SELECTION_NOTE = 'current result'
        self.assertEqual(geom.BOOK_SELECTION_NOTE, 'current result')

    def test_good_first_detection_and_whole_book_do_not_rotate(self):
        for prompt in ('book', 'book spine'):
            with self.subTest(prompt=prompt):
                raw = unittest.mock.Mock(return_value=[(.8, spine_mask(60), None)])
                proxy = new._TiltDetector(types.SimpleNamespace(_sam3_raw_books=raw))
                proxy._sam3_raw_books(None, None, np.zeros((300, 320, 3)), prompt=prompt)
                self.assertEqual(raw.call_count, 1)

    def test_entrypoint_uses_separate_config_and_does_not_start_on_import(self):
        original_install, original_geometry, config = old._install_spine_center, old._spine_center_from_mask, old._CART_PLUS_CONFIG_PATH
        try:
            def check_installed():
                self.assertEqual(old._CART_PLUS_CONFIG_PATH.name, 'book_grasp_1_2_local_config.json')
                self.assertTrue(old._CART_PLUS_CONFIG_PATH.is_file())
                self.assertIs(old._spine_center_from_mask, new._spine_center_from_mask)
            with patch.object(old, 'main', side_effect=check_installed) as main:
                new.main()
                main.assert_called_once_with()
                self.assertIs(old._spine_center_from_mask, original_geometry)
        finally:
            old._install_spine_center, old._spine_center_from_mask, old._CART_PLUS_CONFIG_PATH = original_install, original_geometry, config


if __name__ == '__main__':
    unittest.main()
