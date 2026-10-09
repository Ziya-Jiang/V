"""Offline execution regression: crossing Y=0 does not bypass planner results."""
import ast
from pathlib import Path
import types
import unittest
from unittest.mock import Mock


def execution_functions():
    source = Path(__file__).parent/'RobotRAM/scripts/step5_execute.py'
    names = {'expected_arm_for_target_y', 'validate_arm_target_side', 'execute_single_task'}
    tree = ast.parse(source.read_text())
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = {}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace


class CartPlusArmSideTests(unittest.TestCase):
    def setUp(self):
        self.functions = execution_functions()

    def test_only_cart_plus_right_arm_is_exempt(self):
        check = self.functions['validate_arm_target_side']
        for action in ('move_to_safe_entry', 'move_above_grasp', 'move_down_to_grasp', 'close_gripper', 'lift_object'):
            for y in (-26.8, 0., 26.80770287849327):
                self.assertTrue(check('right', [500, y, 100], action, book_grasp_mode='cart+'))
            for mode in (None, 'cart', 'shelf'):
                self.assertFalse(check('right', [500, 26.8, 100], action, book_grasp_mode=mode))
            self.assertFalse(check('left', [500, -26.8, 100], action, book_grasp_mode='cart+'))
            self.assertTrue(check('left', [500, 26.8, 100], action))

    def test_positive_y_reaches_normal_execution_and_preserves_planner_failure(self):
        execute = self.functions['execute_single_task']
        task = dict(action='move_above_grasp', arm='right', object='book0', book_grasp_mode='cart+',
                    target_position=[493., 26.80770287849327, 100.], target_rotation=[-90., 0., 90.])
        for success in (True, False):
            robot = types.SimpleNamespace(master_control=Mock(return_value=success))
            self.assertEqual(execute(robot, task, 0, {'book0': 'book'}), ('book0', success))
            args, kwargs = robot.master_control.call_args
            self.assertIsNone(args[0])  # No left-arm motion request.
            self.assertEqual(args[1], task['target_position']+task['target_rotation'])
            self.assertEqual(kwargs, dict(action='move_above_grasp', object_category='book'))

    def test_other_modes_still_reject_before_sending_motion(self):
        robot = types.SimpleNamespace(master_control=Mock())
        task = dict(action='move_above_grasp', arm='right', object='book0', book_grasp_mode='cart',
                    target_position=[493., 26.8, 100.], target_rotation=[-90., 0., 90.])
        self.assertEqual(self.functions['execute_single_task'](robot, task, 0, {}), ('book0', False))
        robot.master_control.assert_not_called()

if __name__ == '__main__':
    unittest.main()
