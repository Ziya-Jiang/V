"""Offline forward-action checks with a fake executor (no robot connection)."""
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

import book_grasp_1_1_local as local


class ForwardWaypointTests(unittest.TestCase):
    def setUp(self):
        self.position = [.3, -.2, .5]
        self.robot = types.SimpleNamespace(get_current_state=lambda: {
            'right_arm': {'position': self.position}})
        self.step5 = types.SimpleNamespace(current_tool_orientation_as_euler=lambda *_: [10., 20., 30.])
        self.task = dict(object='book0', arm='right', action='move_down_to_grasp',
                         book_grasp_mode='cart+', target_position=[999, 999, 999])
        self.config = dict(forward_mm=250., forward_step_mm=125.)
        self.sent = []

    def execute(self, robot, task, index, types, **kwargs):
        self.sent.append(task)
        # Simulate tracking error at intermediate arrival. The next target
        # must still be calculated from the original starting pose.
        self.position[:] = np.asarray(task['target_position']) / 1000 + [.003, .002, -.002]
        return 'book0', True

    def run_forward(self, executor=None):
        with patch.object(local, '_load_cart_plus_parameters', return_value=self.config) as load:
            result = local._execute_cart_plus_forward(
                self.robot, self.task, 2, {'book0': 'book'}, self.step5,
                executor or self.execute)
            load.assert_called_once()
        return result

    def test_midpoint_and_final_keep_original_y_z_orientation(self):
        self.assertEqual(self.run_forward(), ('book0', True))
        np.testing.assert_allclose([t['target_position'] for t in self.sent],
                                   [[425, -200, 500], [550, -200, 500]])
        for task in self.sent:
            self.assertEqual(task['target_rotation'], [10., 20., 30.])
            self.assertEqual(task['action'], 'move_down_to_grasp')
            self.assertEqual(task['arm'], 'right')
        self.assertEqual(self.task['target_position'], [999, 999, 999])

    def test_failure_stops_before_final_target(self):
        def fail(*args, **kwargs):
            self.execute(*args, **kwargs)
            return 'book0', False
        self.assertEqual(self.run_forward(fail), ('book0', False))
        self.assertEqual(len(self.sent), 1)

    def test_smaller_step_and_exact_final_distance(self):
        self.config['forward_step_mm'] = 60.
        self.run_forward()
        self.assertEqual(len(self.sent), 5)
        np.testing.assert_allclose([t['target_position'][0] for t in self.sent],
                                   [350, 400, 450, 500, 550])

    def test_invalid_state_and_zero_distance_send_nothing(self):
        self.position[0] = float('nan')
        self.assertEqual(self.run_forward(), ('book0', False))
        self.position[0] = .3
        self.config['forward_mm'] = 0
        self.assertEqual(self.run_forward(), ('book0', True))
        self.assertFalse(self.sent)

    def test_step_config_is_hot_loaded_and_invalid_value_rejected(self):
        document = json.loads(local._CART_PLUS_CONFIG_PATH.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
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
