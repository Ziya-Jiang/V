"""Offline v1.2: base-frame compensation and operator interruption, no devices."""
import ast
from pathlib import Path
import os
import pty
import threading
import types
import unittest
from unittest.mock import Mock, patch
import numpy as np
import cart_book_grasp_v1_2 as cart
import cart_book_grasp_v1_2_motion as motion
import cart_book_grasp_v1_2_perception as perception
from cart_book_grasp_v1_2_geometry import base_z_compensation
from cart_book_grasp_v1_2_visualization import preview_targets
from cart_book_grasp_v1_2_pause import PauseRequested, operator_robot, open_and_reset
from test_book_grasp_1_2 import spine_mask


class HeightTests(unittest.TestCase):
    def test_base_rotation_and_translation_units_and_direction(self):
        selected = dict(spine_center_recovery='stable_region',
                        spine_original_point_camera_m=[.1, .2, .8], point_camera_m=[.12, .23, .85])
        transform = np.eye(4)
        transform[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
        for translation in ([1, 2, 3], [1000, 2000, 3000]):
            transform[:3, 3] = translation
            result = base_z_compensation(selected, transform)
            np.testing.assert_allclose(result['depth_recovery_center_delta_base_mm'], [20, 50, -30])
            self.assertAlmostEqual(result['depth_recovery_delta_z_mm'], -30)
        self.assertEqual(base_z_compensation({}, transform)['depth_recovery_delta_z_mm'], 0)
        selected['spine_center_recovery'] = 'Failed'
        self.assertEqual(base_z_compensation(selected, transform)['depth_recovery_delta_z_mm'], 0)

    def test_real_fallback_preserves_original_camera_point_and_normal_has_no_shift(self):
        mask = spine_mask(30)
        depth = np.full(mask.shape, 800.)
        k = np.array([[300., 0, 160], [0, 300., 150], [0, 0, 1]])
        original = perception._spine_center_from_mask(mask, depth, k)
        self.assertEqual(base_z_compensation(original, np.eye(4)), {'depth_recovery_delta_z_mm': 0.})
        _, pixels, _, _, _, t, _, _, _, _ = perception._spine_frame(mask)
        pixels = pixels[(t > -35) & (t < -3)].astype(int)
        depth[pixels[:, 1], pixels[:, 0]] = 740.
        with patch.object(perception, 'load_wrist_config', return_value={'enabled': True}):
            result = perception._spine_center_from_mask(mask, depth, k)
        np.testing.assert_allclose(result['spine_original_point_camera_m'], original['point_camera_m'])
        transform = np.eye(4)
        transform[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
        expected = -1000*(result['point_camera_m'][1]-original['point_camera_m'][1])
        self.assertAlmostEqual(base_z_compensation(result, transform)['depth_recovery_delta_z_mm'], expected)

    def test_height_command_keeps_xy_orientation_and_then_forward_keeps_new_z(self):
        for shift in (-35., 35., 0.):
            position = np.array([300., -200., 100.])
            robot = types.SimpleNamespace(get_current_state=lambda: {'right_arm': {'position': position/1000}})
            step5 = types.SimpleNamespace(current_tool_orientation_as_euler=lambda *_: [10., 20., 30.])
            commands = []
            def execute(robot, task, *args, **kwargs):
                commands.append(task)
                position[:] = task['target_position']
                return 'book0', True
            task = dict(object='book0', action='move_above_grasp', arm='right', book_grasp_mode='cart+')
            result = motion.execute_depth_z_compensation(robot, task, step5, execute,
                {'depth_recovery_delta_z_mm': shift}, 0, {'book0': 'book'})
            self.assertTrue(result[1])
            self.assertEqual(len(commands), int(shift != 0))
            if commands:
                np.testing.assert_allclose(commands[0]['target_position'], [300, -200, 100+shift])
                np.testing.assert_allclose(commands[0]['target_rotation'], [10, 20, 30])
            motion._execute_cart_plus_forward(robot, dict(task, action='move_down_to_grasp'),
                1, {'book0': 'book'}, step5, execute)
            np.testing.assert_allclose(commands[-1]['target_position'], [550, -200, 100+shift])
            plan = preview_targets([300, -250, 100], [10, 20, 30], [500, -200, -180],
                                   motion._load_cart_plus_parameters(), shift)
            np.testing.assert_allclose(plan['grasp_mm'], commands[-1]['target_position'])
            self.assertEqual(len(plan['labels']), len(plan['points_mm']))

    def test_perception_wrapper_orders_y_then_z_and_blocks_z_after_y_failure(self):
        # Exercise actual installation hooks and command wrappers without imports
        # of hardware, SAM or the task runtime.
        for y_success in (True, False):
            issued = []
            step5 = types.SimpleNamespace(RIGHT_ARM_SOFT_LOWER_DEG=[-90]*7,
                RIGHT_ARM_SOFT_UPPER_DEG=[90]*7,
                current_tool_orientation_as_euler=lambda *_: [1., 2., 3.])
            robot = types.SimpleNamespace(get_current_state=lambda: {'right_arm': {'position': [.3, -.2, .1]}})
            def original(robot, task, *args, **kwargs):
                issued.append(task)
                return 'b', y_success
            step5.execute_single_task = original
            selected = dict(spine_tilt_deg=0, spine_center_recovery='stable_region',
                spine_original_point_camera_m=[0, 0, .8], point_camera_m=[0, 0, .82])
            def fake_install(_geometry, visualization=None, on_geometry=None):
                on_geometry(selected, np.eye(4), {})
            def run(**kwargs):
                perception.previous._install_spine_center(object())
                perception.previous._install_right_arm_reset()
                return step5.execute_single_task(robot,
                    dict(object='b', action='move_above_grasp', arm='right', book_grasp_mode='cart+'), 0, {})
            with patch.dict('sys.modules', {'step5_execute': step5}), \
                 patch.object(perception.previous, '_install_spine_center', side_effect=fake_install), \
                 patch.object(perception.previous, '_install_right_arm_reset'), \
                 patch.object(perception.previous, 'main', side_effect=run), \
                 patch.object(perception, 'align_j7', return_value=True), \
                 patch.object(perception, 'load_wrist_config', return_value={'enabled': False}):
                perception.main(task_types=['cart+'])
            self.assertEqual([t['action'] for t in issued],
                ['move_above_grasp', 'book_depth_z_compensation'] if y_success else ['move_above_grasp'])


class PauseTests(unittest.TestCase):
    def robot_type(self, calls):
        class FakeRobot:
            def __init__(self): pass
            def get_current_state(self): return {'joint_state': np.zeros(14)}
            def control_joint_position(self, **kwargs):
                calls.append(kwargs['action'])
                return self._wait_for_robot_stop()
            def master_control(self, **kwargs): calls.append('open'); return True
            def _wait_for_robot_stop(self, **kwargs):
                self.get_current_state()
                return True
        return FakeRobot

    def test_p_unwinds_inside_motion_wait_before_open_reset_and_waits_enter(self):
        calls=[]
        loop=cart.AutoCartLoop()
        agent=types.SimpleNamespace(RobotArm=self.robot_type(calls))
        with operator_robot(agent, loop):
            robot=agent.RobotArm()
            loop.next_task()
            def wait(**kwargs):
                loop.request_pause()
                robot.get_current_state()
            with patch.object(agent.RobotArm.__bases__[0], '_wait_for_robot_stop', side_effect=wait):
                with self.assertRaises(PauseRequested):
                    robot.control_joint_position(action='forward')
            with patch.object(robot, 'operator_sleep'), patch.object(motion, '_reload_initial_right_j6'):
                self.assertTrue(loop.handle_interrupt(PauseRequested()))
            self.assertEqual(calls, ['forward', 'book_operator_hold', 'open', 'first_reset'])
            self.assertFalse(loop.last_success)
            self.assertFalse(loop.pause.is_set())
            output=[]
            thread=threading.Thread(target=lambda: output.append(loop.next_task()), daemon=True)
            thread.start()
            self.assertTrue(loop.waiting_for_retry.wait(1))
            loop.request_retry()
            thread.join(1)
            self.assertEqual(output, ['cart+'])

    def test_hold_or_open_failure_prevents_reset(self):
        for failure in ('hold', 'open'):
            robot=types.SimpleNamespace(control_joint_position=Mock(return_value=failure!='hold'),
                master_control=Mock(return_value=False), operator_sleep=Mock())
            self.assertFalse(open_and_reset(robot, Mock()))
            self.assertEqual(robot.control_joint_position.call_count, 1)
            self.assertEqual(robot.master_control.call_count, int(failure=='open'))

    def test_single_p_key_no_enter_and_ctrl_c_not_handled(self):
        loop=cart.AutoCartLoop()
        master, slave=pty.openpty()
        try:
            with cart.stop_key_listener(loop, types.SimpleNamespace(isatty=lambda: True, fileno=lambda: slave)):
                os.write(master,b'p')
                self.assertTrue(loop.pause.wait(1))
                self.assertFalse(loop.stop.is_set())
            self.assertFalse(loop.handle_interrupt(KeyboardInterrupt()))
        finally:
            os.close(master);os.close(slave)

    def test_sleep_is_interrupted_and_no_commands_sent_after_pending_p(self):
        calls=[];loop=cart.AutoCartLoop();agent=types.SimpleNamespace(RobotArm=self.robot_type(calls))
        with operator_robot(agent, loop):
            robot=agent.RobotArm();loop.next_task();loop.request_pause()
            for operation in (lambda: robot.operator_sleep(3),
                              lambda: robot.control_joint_position(action='lift'),
                              lambda: robot.master_control(right_gripper=1)):
                with self.assertRaises(PauseRequested):operation()
            self.assertEqual(calls, [])

    def test_real_joint_command_replaces_target_at_measured_joints_before_reset(self):
        # Compile the production method without importing SciPy or robot drivers.
        source=Path(cart.__file__).with_name('RobotRAM')/'controllor'/'hkstp_arm.py'
        tree=ast.parse(source.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RobotArm')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='control_joint_position')
        namespace=dict(np=np,time=types.SimpleNamespace(time_ns=lambda:123),
            TargetCommandArmCommandType=types.SimpleNamespace(JOINT=1))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])), '<joint>', 'exec'),namespace)
        measured=np.arange(14)*.01
        payloads=[]
        command=types.SimpleNamespace(arm_tool_command=np.zeros((2,7)))
        robot=types.SimpleNamespace(get_current_state=lambda:{'joint_state':measured.copy()},
            arm_size=2,target_command=command,right_safe_joint_deg=np.arange(7)+10,
            intermediate_reset_pos=np.zeros((2,7)), _set_arm_tool_command=Mock(),
            _wait_for_robot_stop=Mock(return_value=True),operator_sleep=Mock(),
            master_control=Mock(return_value=True))
        robot.udp=types.SimpleNamespace(send=lambda cmd:payloads.append((cmd.description,cmd.arm_joint_command.copy())))
        robot.control_joint_position=types.MethodType(namespace['control_joint_position'],robot)
        self.assertTrue(open_and_reset(robot,Mock()))
        self.assertEqual(len(payloads),2)
        self.assertTrue(payloads[0][0].startswith('book_operator_hold:'))
        np.testing.assert_allclose(payloads[0][1].ravel(),measured)
        np.testing.assert_allclose(payloads[1][1][0],measured[:7])
        np.testing.assert_allclose(payloads[1][1][1],np.deg2rad(np.arange(7)+10))

    def test_p_during_success_delay_interrupts_before_next_round(self):
        loop=cart.AutoCartLoop(delay_seconds=10)
        loop.next_task();loop.record_result(('b',True))
        def press_p(_seconds):
            loop.request_pause()
            return False
        with patch.object(loop.stop,'wait',side_effect=press_p):
            with self.assertRaises(PauseRequested):loop.next_task()
        self.assertEqual(loop.cycles,1)

    def test_p_during_retry_prompt_unwinds_wait_and_discards_early_enter(self):
        loop=cart.AutoCartLoop();loop.next_task();loop.record_result(('b',False))
        errors=[]
        def wait():
            try:loop.next_task()
            except PauseRequested:errors.append('paused')
        thread=threading.Thread(target=wait,daemon=True);thread.start()
        self.assertTrue(loop.waiting_for_retry.wait(1))
        loop.request_pause();thread.join(1)
        self.assertEqual(errors,['paused'])
        self.assertFalse(loop.waiting_for_retry.is_set())
        loop.request_retry()
        self.assertFalse(loop._retry_requested)

    def test_shared_cycle_catches_only_operator_interrupt_and_returns_to_provider(self):
        tree=ast.parse(Path(cart.__file__).with_name('dual_v2_2_agent_pcl.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        body=next(n for n in main.body if isinstance(n,ast.While))
        code=compile(ast.fix_missing_locations(ast.Module(body=[body],type_ignores=[])),'<cycle>','exec')
        handler=Mock(return_value=True)
        provider=Mock(side_effect=['cart+',None])
        namespace=dict(EXCLUSIVE_EXTENSION_MODE=True,EXTRA_TASK_TYPES=['cart+'],TASK_TYPE_PROVIDER=provider,
            valid_task_types=['cart+'],EXTRA_TASK_BUILDER=Mock(side_effect=PauseRequested()),
            vlm_categories=[],TASK_FAILURE_HANDLER=None,TASK_KEYBOARD_INTERRUPT_HANDLER=handler)
        exec(code,namespace)
        self.assertEqual(handler.call_count,1)
        self.assertEqual(provider.call_count,2)

if __name__=='__main__':unittest.main()
