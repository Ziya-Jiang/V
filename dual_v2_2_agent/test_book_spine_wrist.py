"""Offline geometry and mocked J7 commands; no hardware connections."""
import types
import unittest
from unittest.mock import Mock, patch
import numpy as np

from book_spine_wrist import base_tilt, j7_proposal, align_j7, hold_with_j7_undo
from book_grasp_1_2_local import _spine_center_from_mask
from test_book_grasp_1_2 import spine_mask


class SpineWristTests(unittest.TestCase):
    def setUp(self):
        self.config = dict(enabled=True, max_delta_deg=60., deadband_deg=1., limit_margin_deg=5.)
        self.geometry = dict(spine_3d_status='valid', spine_axis_base=[0, -.5, np.sqrt(.75)])
        self.rotation = dict(complete=True, delta_deg=30., undone=False)
        self.joints = np.zeros((2, 7))
        self.joints[1, 6] = np.radians(5)
        self.robot = types.SimpleNamespace(right_safe_joint_deg=np.zeros(7))
        self.robot.get_current_state = lambda: dict(joint_state=self.joints.copy(),
            right_arm=dict(orientation=[.5, .5, .5, .5]))

    def test_world_vertical_uses_calibration_not_image_angle(self):
        k = np.array([[300., 0, 160], [0, 300., 150], [0, 0, 1]])
        result = _spine_center_from_mask(spine_mask(45), np.full((300, 320), 800.), k)
        self.assertIn('spine_axis_camera', result)
        transform = np.eye(4)
        transform[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
        geometry = base_tilt(result, transform)
        self.assertAlmostEqual(geometry['spine_vertical_tilt_deg'], 45., delta=1.)
        transform[:3, :3] = np.eye(3)
        geometry = base_tilt(result, transform)
        self.assertAlmostEqual(geometry['spine_vertical_tilt_deg'], 90., delta=.1)

    def test_j7_sign_and_target_align_tool_y_to_spine(self):
        result = j7_proposal(self.geometry, self.robot.get_current_state(), self.config, -90, 90)
        self.assertAlmostEqual(result['j7_delta_deg'], 30.)
        self.assertAlmostEqual(result['j7_target_deg'], 35.)
        np.testing.assert_allclose(result['right_target_deg'][:6], 0.)
        self.geometry['spine_axis_base'] = -np.asarray(self.geometry['spine_axis_base'])
        self.assertAlmostEqual(j7_proposal(self.geometry, self.robot.get_current_state(), self.config, -90, 90)['j7_delta_deg'], 30.)

    def test_rejects_missing_geometry_parallel_axis_excessive_delta_and_limits(self):
        for geometry, config, limits in (
            ({}, self.config, (-90, 90)),
            (dict(spine_3d_status='valid', spine_axis_base=[1, 0, 0]), self.config, (-90, 90)),
            (self.geometry, dict(self.config, max_delta_deg=20), (-90, 90)),
            (self.geometry, self.config, (-30, 30))):
            with self.assertRaises(ValueError):
                j7_proposal(geometry, self.robot.get_current_state(), config, *limits)

    def test_disabled_sends_no_commands(self):
        self.robot.get_current_state = Mock(side_effect=AssertionError('no state read needed'))
        self.assertTrue(align_j7(self.robot, {}, dict(self.config, enabled=False), -90, 90))

    def test_alignment_changes_only_j7_and_restores_reset_pose(self):
        def control(**kwargs):
            self.assertEqual(kwargs, dict(left_pos=False, right_pos=True, action='book_align_j7'))
            np.testing.assert_allclose(self.robot.right_safe_joint_deg, [0, 0, 0, 0, 0, 0, 35])
            self.joints[1] = np.radians(self.robot.right_safe_joint_deg)
            return True
        self.robot.control_joint_position = control
        self.assertTrue(align_j7(self.robot, self.geometry, self.config, -90, 90))
        np.testing.assert_array_equal(self.robot.right_safe_joint_deg, np.zeros(7))

    def test_records_actual_rotation_not_requested_delta(self):
        def control(**kwargs):
            self.joints[1] = np.radians(self.robot.right_safe_joint_deg)
            self.joints[1, 6] -= np.radians(.5)
            return True
        self.robot.control_joint_position = control
        record = {'delta_deg': 99, 'undone': True}
        self.assertTrue(align_j7(self.robot, self.geometry, self.config, -90, 90, record))
        self.assertTrue(record['complete'])
        self.assertFalse(record['undone'])
        self.assertAlmostEqual(record['delta_deg'], 29.5)
        self.assertAlmostEqual(record['before_deg'], 5.)
        self.assertAlmostEqual(record['after_deg'], 34.5)

    def test_disabled_alignment_records_zero_and_hold_sends_no_rotation(self):
        record = {}
        self.robot.get_current_state = Mock(side_effect=AssertionError('unexpected robot read'))
        self.assertTrue(align_j7(self.robot, {}, dict(self.config, enabled=False), -90, 90, record))
        with (patch('book_spine_wrist.time.monotonic', side_effect=[1., 1.]),
              patch('book_spine_wrist.time.sleep') as sleep):
            self.assertTrue(hold_with_j7_undo(self.robot, 3., record, -90, 90))
            sleep.assert_called_once_with(3.)

    def test_negative_rotation_is_reversed_and_cannot_be_applied_twice(self):
        self.rotation['delta_deg'] = -30.
        def control(**kwargs):
            self.assertAlmostEqual(self.robot.right_safe_joint_deg[6], 35.)
            self.joints[1] = np.radians(self.robot.right_safe_joint_deg)
            return True
        self.robot.control_joint_position = control
        with patch('book_spine_wrist.time.sleep'):
            self.assertTrue(hold_with_j7_undo(self.robot, 3., self.rotation, -90, 90))
            with self.assertRaises(ValueError):
                hold_with_j7_undo(self.robot, 3., self.rotation, -90, 90)

    def test_missing_record_or_inverse_outside_limits_sends_nothing(self):
        self.robot.control_joint_position = Mock()
        for record in (None, {}, {'complete': False}, dict(complete=True, delta_deg=120.)):
            with self.assertRaises(ValueError):
                hold_with_j7_undo(self.robot, 3., record, -90, 90)
        self.robot.control_joint_position.assert_not_called()

    def test_failed_planner_restores_reset_pose(self):
        self.robot.control_joint_position = Mock(return_value=False)
        self.assertFalse(align_j7(self.robot, self.geometry, self.config, -90, 90))
        np.testing.assert_array_equal(self.robot.right_safe_joint_deg, np.zeros(7))

    def test_requires_ready_pose_and_checks_actual_arrival(self):
        self.robot.control_joint_position = Mock(return_value=True)
        self.joints[1, 0] = np.radians(10)
        with self.assertRaises(ValueError):
            align_j7(self.robot, self.geometry, self.config, -90, 90)
        self.robot.control_joint_position.assert_not_called()
        self.joints[1, 0] = 0
        self.assertFalse(align_j7(self.robot, self.geometry, self.config, -90, 90))

    def test_hold_undoes_delta_from_lifted_angle_and_counts_motion_time(self):
        self.joints[1] = np.radians([10, 20, 30, 40, 50, -20, 35])
        self.robot.right_safe_joint_deg[6] = -17.
        def control(**kwargs):
            self.assertEqual(kwargs, dict(left_pos=False, right_pos=True, action='book_hold_undo_j7'))
            np.testing.assert_allclose(self.robot.right_safe_joint_deg, [10, 20, 30, 40, 50, -20, 5])
            self.joints[1] = np.radians(self.robot.right_safe_joint_deg)
            return True
        self.robot.control_joint_position = control
        with (patch('book_spine_wrist.time.monotonic', side_effect=[10., 11.2]),
              patch('book_spine_wrist.time.sleep') as sleep):
            self.assertTrue(hold_with_j7_undo(self.robot, 3., self.rotation, -90, 90))
            self.assertAlmostEqual(sleep.call_args.args[0], 1.8)
        np.testing.assert_array_equal(self.robot.right_safe_joint_deg, [0, 0, 0, 0, 0, 0, -17])

    def test_slow_inverse_finishes_without_extra_three_second_wait(self):
        def control(**kwargs):
            self.joints[1] = np.radians(self.robot.right_safe_joint_deg)
            return True
        self.robot.control_joint_position = control
        with (patch('book_spine_wrist.time.monotonic', side_effect=[10., 14.]),
              patch('book_spine_wrist.time.sleep') as sleep):
            self.assertTrue(hold_with_j7_undo(self.robot, 3., self.rotation, -90, 90))
            sleep.assert_not_called()

    def test_failed_inverse_stops_hold_and_restores_ready_pose(self):
        self.robot.control_joint_position = Mock(return_value=False)
        with patch('book_spine_wrist.time.sleep') as sleep:
            self.assertFalse(hold_with_j7_undo(self.robot, 3., self.rotation, -90, 90))
            sleep.assert_not_called()
        np.testing.assert_array_equal(self.robot.right_safe_joint_deg, np.zeros(7))

    def test_inverse_rejects_unconfirmed_arrival_and_missing_state(self):
        self.robot.control_joint_position = Mock(return_value=True)
        with patch('book_spine_wrist.time.sleep') as sleep:
            self.assertFalse(hold_with_j7_undo(self.robot, 3., self.rotation, -90, 90))
            sleep.assert_not_called()
        self.robot.get_current_state = lambda: None
        self.rotation = dict(complete=True, delta_deg=30., undone=False)
        with self.assertRaises(ValueError):
            hold_with_j7_undo(self.robot, 3., self.rotation, -90, 90)


if __name__ == '__main__':
    unittest.main()
