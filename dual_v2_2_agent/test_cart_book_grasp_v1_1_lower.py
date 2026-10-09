"""Offline descent targets and failure sequencing; no hardware imports."""
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
import numpy as np
import cart_book_grasp_v1_1_motion as local


class LowerWaypointTests(unittest.TestCase):
    def setUp(self):
        self.position = [.3, -.2, .5]
        self.robot = types.SimpleNamespace(get_current_state=lambda: {'right_arm': {'position': self.position}})
        self.step5 = types.SimpleNamespace(current_tool_orientation_as_euler=lambda *_: [10., 20., 30.])
        self.task = dict(object='book0', arm='right', action='move_down_to_place', book_grasp_mode='cart+')
        self.config = dict(lower_mm=200., lower_step_mm=50.)
        self.sent = []

    def execute(self, robot, task, index, objects, **kwargs):
        self.sent.append(task)
        self.position[:] = np.asarray(task['target_position'])/1000+[.003, -.002, .001]
        self.config['lower_mm'] = 400.  # A live edit must not change this motion.
        return 'book0', True

    def run_lower(self, executor=None):
        with patch.object(local, '_load_cart_plus_parameters', side_effect=lambda: dict(self.config)) as load:
            result = local._execute_cart_plus_lower(self.robot, self.task, 0, {}, self.step5, executor or self.execute)
            load.assert_called_once()
            return result

    def test_all_points_hold_original_xy_rotation_despite_measured_drift(self):
        self.assertEqual(self.run_lower(), ('book0', True))
        np.testing.assert_allclose([t['target_position'] for t in self.sent],
            [[300, -200, 450], [300, -200, 400], [300, -200, 350], [300, -200, 300]])
        self.assertTrue(all(t['target_rotation'] == [10., 20., 30.] for t in self.sent))
        self.assertEqual([t['book_lower_index'] for t in self.sent], [1, 2, 3, 4])
        self.assertNotIn('target_position', self.task)

    def test_non_divisible_distance_still_reaches_exact_endpoint(self):
        self.config['lower_step_mm'] = 60.
        self.config['lower_mm'] = 205.
        self.run_lower()
        self.assertEqual(len(self.sent), 4)
        self.assertAlmostEqual(self.sent[-1]['target_position'][2], 295.)

    def test_failure_stops_remaining_descent(self):
        def fail(*args, **kwargs):
            self.execute(*args, **kwargs)
            return 'book0', False
        self.assertEqual(self.run_lower(fail), ('book0', False))
        self.assertEqual(len(self.sent), 1)

    def test_zero_distance_and_invalid_pose_send_nothing(self):
        self.config['lower_mm'] = 0.
        self.assertTrue(self.run_lower()[1])
        self.position[0] = float('nan')
        self.assertFalse(self.run_lower()[1])
        self.assertFalse(self.sent)

    def test_lower_step_hot_reload_and_invalid_values(self):
        document = json.loads(local._CART_PLUS_CONFIG_PATH.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'config.json'
            with patch.object(local, '_CART_PLUS_CONFIG_PATH', path):
                for step in (50., 25.):
                    document['cart_plus']['lower_step_mm'] = step
                    path.write_text(json.dumps(document))
                    self.assertEqual(local._load_cart_plus_parameters()['lower_step_mm'], step)
                for invalid in (0, -1, True, float('nan')):
                    document['cart_plus']['lower_step_mm'] = invalid
                    path.write_text(json.dumps(document))
                    with self.assertRaises(ValueError):
                        local._load_cart_plus_parameters()


if __name__ == '__main__':
    unittest.main()
