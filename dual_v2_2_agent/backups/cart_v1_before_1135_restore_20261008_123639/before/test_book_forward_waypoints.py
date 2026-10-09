"""Offline XYZ approach checks with a fake executor; no robot connection."""
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
import numpy as np
import book_grasp_1_1_local as local
from book_cartesian_grasp import grasp_plan, tool_rotation, retreat_targets


class ForwardWaypointTests(unittest.TestCase):
    def setUp(self):
        self.position = [.3, -.2, .1]
        self.robot = types.SimpleNamespace(get_current_state=lambda: {'right_arm': {'position': self.position}})
        self.step5 = types.SimpleNamespace(current_tool_orientation_as_euler=lambda *_: [0., -90., 0.])
        self.task = dict(object='book0', arm='right', action='move_above_grasp',
                         book_grasp_mode='cart+', book_center_xyz=[550, -200, 130])
        self.config = dict(forward_step_mm=125., grasp_insertion_mm=50., fingertip_offset_mm=50.,
                           pregrasp_clearance_mm=50., align_step_mm=50.)
        self.sent = []
        self.context = {}

    def execute(self, robot, task, index, objects, **kwargs):
        self.sent.append(task)
        # Measured drift must not accumulate into future waypoint targets.
        self.position[:] = np.asarray(task['target_position'])/1000+[.003, .002, -.002]
        return 'book0', True

    def align(self, executor=None):
        with patch.object(local, '_load_cart_plus_parameters', return_value=self.config) as load:
            result = local._execute_cart_plus_align(self.robot, self.task, 0, {}, self.step5,
                executor or self.execute, self.context)
            load.assert_called_once()
        return result

    def forward(self, executor=None):
        return local._execute_cart_plus_forward(self.robot, dict(self.task, action='move_down_to_grasp'),
            1, {}, self.step5, executor or self.execute, retreat_context=self.context)

    def test_measured_xyz_sets_height_and_endpoint_not_a_fixed_forward_distance(self):
        self.assertTrue(self.align()[1])
        self.assertTrue(self.forward()[1])
        np.testing.assert_allclose([t['target_position'] for t in self.sent],
            [[300, -200, 130], [375, -200, 130], [450, -200, 130], [550, -200, 130]])
        self.assertTrue(all(t['target_rotation'] == [0., -90., 0.] for t in self.sent))
        self.assertNotIn('target_position', self.task)
        self.assertTrue(self.context['book0']['forward_complete'])

    def test_failed_align_or_forward_stops_later_commands(self):
        def fail(*args, **kwargs):
            self.execute(*args, **kwargs)
            return 'book0', False
        self.assertFalse(self.align(fail)[1])
        self.assertFalse(self.forward()[1])
        self.assertEqual(len(self.sent), 1)
        self.setUp()
        self.align()
        self.assertFalse(self.forward(fail)[1])
        self.assertEqual(len(self.sent), 2)
        self.assertFalse(self.context['book0']['forward_complete'])

    def test_approach_geometry_is_frozen_after_alignment(self):
        self.align()
        self.config['grasp_insertion_mm'] = 150.
        self.config['forward_step_mm'] = 20.
        self.forward()
        self.assertEqual(len(self.sent), 4)
        np.testing.assert_allclose(self.sent[-1]['target_position'], [550, -200, 130])

    def test_already_aligned_still_sends_alignment_action_to_open_gripper(self):
        self.position[2] = .13
        self.assertTrue(self.align()[1])
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.sent[0]['action'], 'move_above_grasp')

    def test_changed_orientation_cancels_approach(self):
        self.align()
        self.step5.current_tool_orientation_as_euler = lambda *_: [0., -80., 0.]
        self.assertFalse(self.forward()[1])
        self.assertEqual(len(self.sent), 1)

    def test_invalid_xyz_or_pose_cannot_fall_back_to_old_height(self):
        for center in (None, [550, -200], [550, -200, float('nan')]):
            self.task['book_center_xyz'] = center
            self.assertFalse(self.align()[1])
        self.task['book_center_xyz'] = [550, -200, 130]
        self.position[0] = float('nan')
        self.assertFalse(self.align()[1])
        self.assertFalse(self.sent)

    def test_tool_offset_and_insertion_follow_rotated_finger_axis(self):
        rotation = [20., -75., 10.]
        parameters = dict(self.config, grasp_insertion_mm=30.)
        center = np.array([550., -200., 180.])
        plan = grasp_plan([300., -240., 100.], rotation, center, parameters)
        direction = -tool_rotation(rotation)[:, 2]
        np.testing.assert_allclose(plan['grasp_mm']+50*direction, center+30*direction)
        np.testing.assert_allclose(plan['pregrasp_mm']+50*direction, center-50*direction)
        self.assertNotAlmostEqual(plan['grasp_mm'][2], 100.)
        for target in plan['forward']:
            relative = target-plan['aligned_mm']
            np.testing.assert_allclose(relative-direction*np.dot(relative, direction), 0, atol=1e-10)
        retreat = retreat_targets(plan['grasp_mm'], 300., direction, 125.)
        np.testing.assert_allclose(retreat[-1], plan['aligned_mm'])

    def test_target_tracks_new_book_height_and_rejects_pointing_away(self):
        a = grasp_plan([300, -200, 100], [0, -90, 0], [550, -200, 130], self.config)
        b = grasp_plan([300, -200, 100], [0, -90, 0], [620, -200, 210], self.config)
        np.testing.assert_allclose(b['grasp_mm']-a['grasp_mm'], [70, 0, 80])
        with self.assertRaises(ValueError):
            grasp_plan([300, -200, 100], [0, 0, 0], [550, -200, 130], self.config)
        with self.assertRaises(ValueError):
            grasp_plan([500, -200, 100], [0, -90, 0], [550, -200, 130], self.config)

    def test_step_config_is_hot_loaded_and_invalid_value_rejected(self):
        document = json.loads(local._CART_PLUS_CONFIG_PATH.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'config.json'
            with patch.object(local, '_CART_PLUS_CONFIG_PATH', path):
                for step in (125., 50.):
                    document['cart_plus']['forward_step_mm'] = step
                    path.write_text(json.dumps(document))
                    self.assertEqual(local._load_cart_plus_parameters()['forward_step_mm'], step)
                for invalid in (0, -1, True):
                    document['cart_plus']['forward_step_mm'] = invalid
                    path.write_text(json.dumps(document))
                    with self.assertRaises(ValueError):
                        local._load_cart_plus_parameters()


if __name__ == '__main__':
    unittest.main()
