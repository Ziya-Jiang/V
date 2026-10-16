"""No hardware: image-Jacobian finite differences, rolling control, backend guards."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
from test_cart_book_grasp_v4_1 import configuration, observation
from book_top_tilt_servo import image_features, image_jacobian, image_correction, ContinuousTiltController, ContinuousRobotIO

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT/'dual_v2_2_control/cuarm_upper_planner/scripts'))
sys.path.insert(0, str(ROOT/'dual_v2_2_configuration/robot_platform_utils/python/scripts'))
from v4_1_direct_servo import TiltExecutionGuard, TiltDirectServo


class ImageControlTests(unittest.TestCase):
    def test_client_sends_without_waiting_for_arrival(self):
        from collections import deque
        import time
        calls = []
        state = dict(right_arm=dict(position=[.6, 0, .158]))
        io = ContinuousRobotIO.__new__(ContinuousRobotIO)
        io.c, io.rotation = configuration(), np.eye(3)
        io.guard = lambda: state
        io.started_sending = None
        io.pending = deque()
        def master(**kwargs):
            calls.append(kwargs)
            return True
        io.robot = SimpleNamespace(master_control=master,
            target_command=SimpleNamespace(arm_tool_command=np.zeros((2, 7))))
        io.send(np.array([599., 0., 158.]), time.time())
        self.assertFalse(calls[0]['wait_for_completion'])
        self.assertEqual(calls[0]['action'], 'book_v3_1_stream')

    def test_image_jacobian_matches_numerical_projection(self):
        point = np.array([120., -75., 650.])
        r = Rotation.from_euler('xyz', [12, 24, -30], degrees=True).as_matrix()
        numeric = np.column_stack([(image_features(point+r[:, i]*.001)-image_features(point))/.001 for i in range(3)])
        np.testing.assert_allclose(image_jacobian(point, r), numeric, atol=1e-8)

    def test_eye_to_hand_sign_and_depth_error_reduce(self):
        transform = np.eye(4)
        pad = np.array([10., 50., 650.])
        target = pad+[15., -8., -10.]
        for _ in range(60):
            old = np.linalg.norm(image_features(target)-image_features(pad))
            delta, _, _ = image_correction(pad, target, transform)
            self.assertLessEqual(np.linalg.norm(delta), 2.+1e-9)
            pad += delta
            self.assertLessEqual(np.linalg.norm(image_features(target)-image_features(pad)), old+1e-14)
        np.testing.assert_allclose(pad, target, atol=1e-4)

    def controller(self):
        c = configuration()
        tcp = np.array([600., 0., 158.])
        ctrl = ContinuousTiltController(c, observation(), tcp, np.eye(3), np.eye(4))
        ctrl.phase = 'APPROACH'
        return ctrl, np.array([600., 0., 150.5])

    def test_targets_update_before_robot_arrival(self):
        ctrl, tcp = self.controller()
        first = ctrl.decide_stream(observation(stamp=1), tcp, np.eye(3), tcp, np.eye(3))
        moving = tcp+.1*(first['target']-tcp)
        second = ctrl.decide_stream(observation(stamp=1.1), moving, np.eye(3), moving, np.eye(3))
        self.assertEqual(first['phase'], 'PROBE')
        self.assertEqual(second['phase'], 'PROBE')
        self.assertIsNotNone(second['target'])
        self.assertGreater(np.linalg.norm(second['target']-moving), .1)

    def test_nonresponding_book_cannot_accumulate_lead(self):
        ctrl, tcp = self.controller()
        ctrl.decide_stream(observation(stamp=1), tcp, np.eye(3), tcp, np.eye(3))
        anchor = ctrl.probe_goal.copy()
        for stamp in np.arange(1.1, 2.9, .1):
            ctrl.decide_stream(observation(stamp=stamp), tcp, np.eye(3), tcp, np.eye(3))
            np.testing.assert_allclose(ctrl.probe_goal, anchor)
        with self.assertRaisesRegex(ValueError, 'No measured book response'):
            ctrl.decide_stream(observation(stamp=3.1), tcp, np.eye(3), tcp, np.eye(3))

    def test_goal_frame_identity_is_still_required(self):
        ctrl, tcp = self.controller()
        ctrl.decide_stream(observation(), tcp, np.eye(3), tcp, np.eye(3))
        with self.assertRaisesRegex(ValueError, 'did not advance'):
            ctrl.decide_stream(observation(), tcp, np.eye(3), tcp, np.eye(3))

    def test_ideal_book_closed_loop_runs_without_waiting_for_arrival(self):
        ctrl, tcp = self.controller()
        obs = observation()
        done = False
        for i in range(1500):
            obs['captured_unix_s'] = 1+i*.1
            d = ctrl.decide_stream(obs, tcp, np.eye(3), tcp, np.eye(3))
            if d['phase'] == 'HOLD':
                done = True
                break
            # Continuously moving plant: follows only 25% of each new goal,
            # never waits for a waypoint-completion event.
            tcp += .25*(d['target']-tcp)
            if d['phase'] in ('PROBE', 'TILT'):
                obs['top_base_mm'] = tcp+[0, 0, 50]+ctrl.normal*ctrl.c['contact_gap_mm']
        self.assertTrue(done)


def snapshot(stamp=10.):
    return dict(stamp=stamp, joints=np.zeros(14), velocity=np.zeros(14),
                position=np.array([[0., 0., 0.], [.6, 0., .2]]),
                orientation=np.array([[1., 0., 0., 0.], [1., 0., 0., 0.]]))


class GuardTests(unittest.TestCase):
    def test_xyz_motion_not_fixed_z(self):
        g = TiltExecutionGuard(lambda c: None, clock=lambda: 10.)
        s = snapshot()
        g.begin_xyz(s)
        s['position'][1] += [-.1, 0, -.06]
        g.update(s)
        self.assertIsNone(g.fault)

    def test_orientation_violation_stops(self):
        g = TiltExecutionGuard(lambda c: None, clock=lambda: 10.)
        s = snapshot()
        g.begin_xyz(s)
        q = Rotation.from_euler('x', 2, degrees=True).as_quat()
        s['orientation'][1] = q[[3, 0, 1, 2]]
        g.update(s)
        self.assertIn('orientation', g.fault)

    def test_independent_missing_vision_deadline(self):
        g = TiltExecutionGuard(lambda c: None, clock=lambda: 10.)
        g.begin_xyz(snapshot())
        g.last_target_time = 8.9
        g.update(snapshot())
        self.assertIn('stream absent', g.fault)

    def test_stale_feedback_trips_even_without_new_command(self):
        g = TiltExecutionGuard(lambda c: None, clock=lambda: 10.)
        g.begin_xyz(snapshot(9.5))
        g.update(None)
        self.assertIn('stale', g.fault)


class BackendTests(unittest.TestCase):
    def setup_backend(self):
        q = np.zeros(14)
        q[7:10] = [.6, 0, .2]
        self.q = q
        def tcp(values):
            values = np.atleast_2d(values)
            pos = np.zeros((len(values), 2, 3))
            pos[:, 1] = values[:, 7:10]
            ori = np.zeros((len(values), 2, 4)); ori[:, :, 0] = 1
            qr = Rotation.from_rotvec(values[:, 10:13]).as_quat()
            ori[:, 1] = qr[:, [3, 0, 1, 2]]
            return pos, ori
        def snap(max_age):
            s = snapshot(self.now)
            s['joints'] = self.q.copy()
            s['position'], s['orientation'] = [x[0] for x in tcp(self.q)]
            return s
        self.now = 10.
        guard = TiltExecutionGuard(lambda c: None, clock=lambda: self.now)
        bridge = SimpleNamespace(tcp=tcp, snapshot=snap, guard=guard, serial=0)
        collision = SimpleNamespace(check=lambda q: True)
        self.planner = SimpleNamespace(_v31_bridge=bridge, arm_soft_joint_limits=np.array([[-3]*14, [3]*14]),
            target_arm_tool_position=snapshot()['position'].copy(),
            target_arm_tool_orientation=snapshot()['orientation'].copy(), gripper_joint_position=[],
            reorder_input=lambda q, grip, as_curobo: q, curobo_cmd=SimpleNamespace(), set_gripper_command=lambda: None)
        self.backend = TiltDirectServo(self.planner, collision)

    def test_continuous_xyz_ticks_without_motiongen_or_arrival(self):
        self.setup_backend()
        start = self.q.copy()
        for i in range(30):
            self.now = 10+i*.02
            self.planner.target_arm_tool_position[1] = self.q[7:10]+[-.001, 0, -.001]
            with patch('v4_1_direct_servo.time.monotonic', return_value=self.now):
                self.backend.compute(self.now)
            self.assertTrue(self.backend.output_ready)
            self.backend.dispatched(True)
            self.q += .8*(self.backend.proposed-self.q)
        self.assertLess(self.q[7], start[7])
        self.assertLess(self.q[9], start[9])
        self.assertEqual(self.planner.curobo_cmd.waypoints_size, 0)

    def test_visual_lease_freezes_joint_target(self):
        self.setup_backend()
        with patch('v4_1_direct_servo.time.monotonic', return_value=10.):
            self.backend.compute(10.)
        self.backend.dispatched(True)
        self.now = 10.31
        self.backend.last_tick = 10.30
        with patch('v4_1_direct_servo.time.monotonic', return_value=self.now):
            self.backend.compute(10.)
        self.assertTrue(self.backend.paused)
        np.testing.assert_allclose(self.backend.proposed, self.q)

    def test_joint_collision_rejects_dispatch(self):
        self.setup_backend()
        self.backend.collision.check = lambda q: False
        with patch('v4_1_direct_servo.time.monotonic', return_value=10.):
            with self.assertRaisesRegex(ValueError, 'collision'):
                self.backend.compute(10.)
        self.assertFalse(self.backend.output_ready)

    def test_large_lead_is_rejected(self):
        self.setup_backend()
        self.planner.target_arm_tool_position[1, 0] -= .01
        with patch('v4_1_direct_servo.time.monotonic', return_value=10.):
            with self.assertRaisesRegex(ValueError, '3 mm'):
                self.backend.compute(10.)




class PerceptionPauseTests(unittest.TestCase):
    def test_pause_preserves_session_and_finite_timeout(self):
        now=[10.]
        g=TiltExecutionGuard(lambda c:None,clock=lambda:now[0])
        g.begin_xyz(snapshot(now[0]));origin=g.origin.copy()
        g.last_target_time=9.9
        g.pause_visual(snapshot(now[0]))
        self.assertIsNone(g.last_target_time)
        now[0]=60.
        g.pause_visual(snapshot(now[0]))
        self.assertIsNone(g.fault)
        self.assertEqual(g.paused_at,10.)
        np.testing.assert_array_equal(g.origin,origin)
        now[0]=86.
        g.update(snapshot(now[0]))
        self.assertIn('75 seconds',g.fault)

    def test_pause_is_not_fault_reset(self):
        g=TiltExecutionGuard(lambda c:None,clock=lambda:10.)
        g.begin_xyz(snapshot());g.trip('collision')
        with self.assertRaisesRegex(ValueError,'collision'):g.pause_visual(snapshot())
        with self.assertRaisesRegex(ValueError,'collision'):g.resume_visual(snapshot())
        self.assertEqual(g.fault,'collision')

    def test_pause_keeps_missing_feedback_protection(self):
        now=[10.]
        g=TiltExecutionGuard(lambda c:None,clock=lambda:now[0])
        g.pause_visual(snapshot())
        now[0]=10.3;g.update(None)
        self.assertIn('stale',g.fault)

    def test_pause_keeps_xyz_envelope(self):
        g=TiltExecutionGuard(lambda c:None,clock=lambda:10.)
        g.pause_visual(snapshot())
        s=snapshot();s['position'][1,2]+=.21
        g.update(s)
        self.assertIn('envelope',g.fault)

    def test_resume_requires_stationary_and_keeps_original_origin(self):
        g=TiltExecutionGuard(lambda c:None,clock=lambda:10.)
        g.pause_visual(snapshot());origin=g.origin.copy()
        s=snapshot();s['velocity'][7]=.02
        with self.assertRaisesRegex(ValueError,'stationary'):g.resume_visual(s)
        self.assertIsNotNone(g.paused_at)
        s['velocity'][:]=0
        g.resume_visual(s)
        self.assertIsNone(g.paused_at)
        np.testing.assert_array_equal(g.origin,origin)

    def test_client_pause_sends_only_private_hold_and_waits_new_feedback(self):
        from collections import deque
        from cuarm_state import TargetCommandArmCommandType
        now=[10.]
        sent=[]
        state=dict(right_arm=dict(position=[.6,0,.2],orientation=[1.,0,0,0]),
                   left_arm=dict(position=[0.,0,0],orientation=[1.,0,0,0]),
                   joint_velocity=np.zeros(14))
        command=SimpleNamespace(arm_command_type=TargetCommandArmCommandType.CARTESIAN,
            arm_tool_command=[],gripper_mode_command=[0,0])
        io=ContinuousRobotIO.__new__(ContinuousRobotIO)
        io.pending=deque([(1,[])])
        io.observation_anchor=None
        io.rotation=np.eye(3)
        io.log=lambda value:None
        io.robot=SimpleNamespace(target_command=command,udp=SimpleNamespace(send=lambda cmd:sent.append(copy.deepcopy(cmd))))
        def guard():
            return dict(state,pose_sample_monotonic=now[0],received_monotonic=now[0],
                        processed_arm_tool_command=io.robot.target_command.arm_tool_command)
        io.guard=guard
        with patch('book_top_tilt_servo.time.monotonic',side_effect=lambda:now[0]), patch(
                'book_top_tilt_servo.time.sleep',side_effect=lambda duration:now.__setitem__(0,now[0]+duration)):
            with io.perception_pause():
                self.assertGreaterEqual(now[0],10.3)
                np.testing.assert_allclose(io.observation_anchor,[600,0,200])
            self.assertIsNone(io.observation_anchor)
        self.assertEqual(len(sent),1)
        self.assertEqual(sent[0].description,'book_v4_1_pause')
        self.assertEqual(sent[0].gripper_mode_command,[0,0])
        self.assertEqual(len(io.pending),0)

    def test_client_pause_no_ack_cannot_enter_inference(self):
        from collections import deque
        now=[10.]
        state=dict(right_arm=dict(position=[.6,0,.2],orientation=[1.,0,0,0]),
                   left_arm=dict(position=[0.,0,0],orientation=[1.,0,0,0]),joint_velocity=np.zeros(14))
        io=ContinuousRobotIO.__new__(ContinuousRobotIO)
        io.pending=deque();io.rotation=np.eye(3)
        io.robot=SimpleNamespace(target_command=SimpleNamespace(arm_tool_command=[]),udp=SimpleNamespace(send=lambda cmd:None))
        io.guard=lambda:dict(state,pose_sample_monotonic=now[0],received_monotonic=now[0],processed_arm_tool_command=[])
        with patch('book_top_tilt_servo.time.monotonic',side_effect=lambda:now[0]), patch(
                'book_top_tilt_servo.time.sleep',side_effect=lambda duration:now.__setitem__(0,now[0]+duration)):
            with self.assertRaisesRegex(ValueError,'not acknowledged'):
                with io.perception_pause():self.fail('API must not run')


class PauseDispatchTests(unittest.TestCase):
    def test_private_pause_clears_active_output_and_acks_without_motiongen(self):
        from v4_1_direct_servo import compute_perception_pause
        from cuarm_state import CuroboStatus
        b=BackendTests();b.setup_backend()
        p=b.planner
        p.curobo_state=SimpleNamespace()
        held=[]
        def hold():
            held.append(True);p.curobo_cmd.waypoints_size=0
        p._hold_current_arm_command=hold;p._v31_direct=b.backend
        p._v31_direct.active=True
        compute_perception_pause(p)
        self.assertEqual(p.status,CuroboStatus.PLANNING_SUCCEEDED)
        self.assertFalse(p._v31_direct.active)
        self.assertTrue(held)
        np.testing.assert_array_equal(p._prev_target_tool_position,p.target_arm_tool_position)
        self.assertIsNotNone(p._v31_bridge.guard.paused_at)
        p._v31_bridge.guard.trip('collision remains latched')
        compute_perception_pause(p)
        self.assertEqual(p.status,CuroboStatus.PLANNING_FAILED)
        self.assertEqual(p._v31_bridge.guard.fault,'collision remains latched')

if __name__ == '__main__':
    unittest.main()
