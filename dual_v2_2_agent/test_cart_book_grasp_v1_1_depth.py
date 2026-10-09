"""Offline checks for the strictly conditional v1.1 depth fallback."""
import types
import unittest
from unittest.mock import patch
import numpy as np
import book_grasp_1_2_local as original
import cart_book_grasp_v1_1_perception as new
import cart_book_grasp_v1_1_motion as motion
from book_grasp_visualization import preview_targets
from test_book_grasp_1_2 import spine_mask


class DepthFallbackTests(unittest.TestCase):
    def setUp(self):
        self.mask = spine_mask(30)
        self.depth = np.full(self.mask.shape, 800.)
        self.k = np.array([[300., 0, 160], [0, 300., 150], [0, 0, 1]])

    def inconsistent(self):
        _, pixels, _, _, _, t, _, _, _, _ = new._spine_frame(self.mask)
        pixels = pixels[(t > -35) & (t < -3)].astype(int)
        depth = self.depth.copy()
        depth[pixels[:, 1], pixels[:, 0]] = 740.
        return depth

    def test_valid_v1_result_is_unchanged_and_does_not_even_read_fallback_settings(self):
        expected = original._spine_center_from_mask(self.mask, self.depth, self.k)
        with patch.object(new, 'recover_stable_spine') as recover, patch.object(new, 'load_wrist_config') as config:
            actual = new._spine_center_from_mask(self.mask, self.depth, self.k)
            recover.assert_not_called()
            config.assert_not_called()
        self.assertEqual(actual.keys(), expected.keys())
        for key in actual:
            np.testing.assert_equal(actual[key], expected[key])

    def test_inconsistent_depth_blocks_v1_but_v1_1_uses_one_stable_region(self):
        depth = self.inconsistent()
        expected = original._spine_center_from_mask(self.mask, depth, self.k)
        self.assertNotIn('spine_axis_camera', expected)
        for scale in (1., .001):
            with patch.object(new, 'load_wrist_config', return_value={'enabled': True}):
                actual = new._spine_center_from_mask(self.mask, depth*scale, self.k)
            self.assertEqual(actual['spine_center_recovery'], 'stable_region')
            self.assertEqual(actual['spine_3d_status'], 'valid')
            np.testing.assert_allclose(actual['uv'], np.mean(actual['spine_stable_region_uv'], axis=0))
            self.assertLess(actual['spine_axis_residual_mm'], 1.)
            self.assertAlmostEqual(actual['depth_m'], .8)

    def test_disabled_wrist_does_not_trigger_fallback_for_unneeded_tilt(self):
        depth = self.inconsistent()
        expected = original._spine_center_from_mask(self.mask, depth, self.k)
        with patch.object(new, 'load_wrist_config', return_value={'enabled': False}), patch.object(new, 'recover_stable_spine') as recover:
            actual = new._spine_center_from_mask(self.mask, depth, self.k)
            recover.assert_not_called()
        np.testing.assert_equal(actual['uv'], expected['uv'])
        self.assertNotIn('spine_center_recovery', actual)

    def test_geometry_and_missing_depth_errors_never_use_consistency_fallback(self):
        with patch.object(new, 'recover_stable_spine') as recover:
            for mask, depth in ((np.zeros_like(self.mask), self.depth), (self.mask, np.zeros_like(self.depth))):
                with self.assertRaises(ValueError):
                    new._spine_center_from_mask(mask, depth, self.k)
            tiny_3d = new._spine_center_from_mask(self.mask, self.depth*.125, self.k)
            self.assertNotIn('spine_axis_camera', tiny_3d)
            recover.assert_not_called()

    def test_unrecoverable_depth_still_cancels_instead_of_using_an_invalid_center(self):
        depth = self.depth.copy()
        depth[::2] = 1600.
        with self.assertRaisesRegex(ValueError, 'stable-region fallback failed'):
            new._spine_center_from_mask(self.mask, depth, self.k)

    def test_center_depth_failure_uses_fallback_even_without_wrist_rotation(self):
        depth = self.depth.copy()
        depth[140:159:2] = 1600.
        with self.assertRaisesRegex(ValueError, 'background'):
            original._spine_center_from_mask(self.mask, depth, self.k)
        with patch.object(new, 'load_wrist_config', return_value={'enabled': False}):
            result = new._spine_center_from_mask(self.mask, depth, self.k)
        self.assertEqual(result['spine_center_recovery'], 'stable_region')

    def test_recovered_center_never_changes_the_fixed_approach_height(self):
        parameters = motion._load_cart_plus_parameters()
        plan = preview_targets([300, -250, 100], [10, 20, 30], [550, -200, -180], parameters)
        np.testing.assert_allclose(plan['points_mm'][1:4, 2], 100.)
        np.testing.assert_allclose(plan['points_mm'][5:9],
            [[550, -200, 250], [550, -200, 200], [550, -200, 150], [550, -200, 100]])
        robot = types.SimpleNamespace(get_current_state=lambda: {'right_arm': {'position': [.3, -.25, .1]}})
        step5 = types.SimpleNamespace(current_tool_orientation_as_euler=lambda *_: [10., 20., 30.])
        task = dict(action='move_above_grasp', arm='right', book_grasp_mode='cart+', book_center_xy=[550, -200])
        with patch.object(motion, '_load_cart_plus_parameters', return_value=parameters):
            target = motion._rewrite_cart_plus_level_task(robot, task, step5)
        np.testing.assert_allclose(target['target_position'], [300, -200, 100])


if __name__ == '__main__':
    unittest.main()
