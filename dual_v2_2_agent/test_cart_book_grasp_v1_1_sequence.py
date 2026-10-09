"""Offline release/withdraw/reset sequencing; no robot or camera imports."""
import sys
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np
import cart_book_grasp_v1_1_motion as local


class ReleaseResetTests(unittest.TestCase):
    def run_sequence(self, failure=None):
        commands = []
        position = np.array([.3, -.2, .1])
        config = dict(forward_mm=250., forward_step_mm=125., lift_mm=200.,
                      lower_mm=200., lower_step_mm=50., hold_seconds=0., initial_j6_deg=-20.3164)
        robot = types.SimpleNamespace(
            get_current_state=lambda: {'right_arm': {'position': position.copy()}},
            right_safe_joint_deg=np.zeros(7))

        def reset(**kwargs):
            commands.append(('reset', kwargs))
            return True

        robot.control_joint_position = reset

        def execute(robot, task, index, objects, **kwargs):
            commands.append((task['action'], dict(task)))
            if task['action'] == failure:
                return task['object'], False
            if task['action'] not in ('open_gripper', 'close_gripper'):
                position[:] = np.array(task['target_position']) / 1000
            if task['action'] == 'open_gripper':
                # Changing forward distance after release must not change
                # the saved withdrawal endpoint for this grasp.
                config['forward_mm'] = 400.
            return task['object'], True

        step5 = types.SimpleNamespace(execute_single_task=execute,
            current_tool_orientation_as_euler=lambda *_: [0., 0., 0.],
            recover_to_reset_after_failure=Mock())

        def main(tasks, objects, robot, **kwargs):
            for index, task in enumerate(tasks):
                result = step5.execute_single_task(robot, task, index, objects)
                if not result[1]:
                    return result
            return result

        agent = types.SimpleNamespace(execute_main=main, grounding_main=Mock())
        geom = types.SimpleNamespace(fill_book_meta=Mock())
        visualization = types.SimpleNamespace(logger=None, active=False, safe=Mock())
        tasks = [dict(object='book0', arm='right', book_grasp_mode='cart+',
                      action=action, book_center_xy=[550, -200])
                 for action in ('move_above_grasp', 'move_down_to_grasp', 'close_gripper', 'lift_object')]
        with (patch.dict(sys.modules, {'dual_v2_2_agent_pcl': agent, 'step5_execute': step5, 'book_grasp': geom}),
              patch('book_grasp_visualization.install_book_visualization', return_value=visualization),
              patch.object(local, '_load_cart_plus_parameters', side_effect=lambda: dict(config)),
              patch.object(local.time, 'sleep')):
            local._install_right_arm_reset()
            result = agent.execute_main(tasks, {'book0': 'book'}, robot)
            # A reset bypassing the withdrawal stage must be rejected.
            if failure:
                reset_task = dict(tasks[0], action='first_reset', book_post_release_reset=True)
                self.assertFalse(step5.execute_single_task(robot, reset_task, 99, {})[1])
        return result, commands, robot

    def test_release_then_withdraw_saved_distance_then_right_only_reset(self):
        result, commands, robot = self.run_sequence()
        self.assertTrue(result[1])
        actions = [action for action, _ in commands]
        self.assertEqual(actions[-4:], ['open_gripper', 'withdraw_after_release', 'withdraw_after_release', 'reset'])
        withdraw = [task['target_position'] for action, task in commands if action == 'withdraw_after_release']
        np.testing.assert_allclose(withdraw, [[425, -200, 100], [300, -200, 100]])
        self.assertEqual(commands[-1][1], dict(left_pos=False, right_pos=True, action='first_reset'))
        self.assertEqual(robot.right_safe_joint_deg[5], -20.3164)
        np.testing.assert_allclose([t['target_position'] for a,t in commands if a == 'move_down_to_place'],
            [[550, -200, 250], [550, -200, 200], [550, -200, 150], [550, -200, 100]])

    def test_failed_descent_sends_no_release_or_reset(self):
        result, commands, _ = self.run_sequence('move_down_to_place')
        self.assertFalse(result[1])
        self.assertEqual(commands[-1][0], 'move_down_to_place')
        self.assertFalse({'open_gripper', 'withdraw_after_release', 'reset'} & {a for a, _ in commands})

    def test_failed_release_sends_no_withdraw_or_reset(self):
        result, commands, _ = self.run_sequence('open_gripper')
        self.assertFalse(result[1])
        self.assertEqual(commands[-1][0], 'open_gripper')
        self.assertNotIn('reset', [action for action, _ in commands])

    def test_failed_withdraw_stops_remaining_waypoints_and_reset(self):
        result, commands, _ = self.run_sequence('withdraw_after_release')
        self.assertFalse(result[1])
        self.assertEqual(sum(action == 'withdraw_after_release' for action, _ in commands), 1)
        self.assertNotIn('reset', [action for action, _ in commands])


if __name__ == '__main__':
    unittest.main()
