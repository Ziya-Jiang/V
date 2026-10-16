"""Offline controller feedback tests; never construct a robot/socket."""
import contextlib
import io
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from RobotRAM.controllor.hkstp_arm import (
    RobotArm, CuroboStatus, RobotStatus, TargetCommandArmCommandType,
    _position_tolerance_for_task, CuarmUdpThread)
from RobotRAM.controllor.servo_arrival import ServoArrival


TARGET=np.array([[0.,0.,0.,1.,0.,0.,0.],[.5186258,.0373376,.0158017,1.,0.,0.,0.]])


def state(stamp, offset=0., angle=0.):
    pose=TARGET.copy();pose[1,1]+=offset
    pose[1,3:]=[np.cos(angle/2),0,0,np.sin(angle/2)]
    return dict(received_monotonic=stamp,
        left_arm=dict(position=pose[0,:3].tolist(),orientation=pose[0,3:].tolist()),
        right_arm=dict(position=pose[1,:3].tolist(),orientation=pose[1,3:].tolist()),
        processed_arm_tool_command=TARGET.copy(),
        robot_status=RobotStatus.AVAILABLE,curobo_status=CuroboStatus.MOTION_FINISHED)


class GateTests(unittest.TestCase):
    def test_three_distinct_fresh_packets_required(self):
        g=ServoArrival(10.,(1,))
        self.assertFalse(g.update(state(10.1),TARGET,10.1))
        self.assertFalse(g.update(state(10.1),TARGET,10.2));self.assertEqual(g.count,1)
        self.assertFalse(g.update(state(10.2),TARGET,10.2))
        self.assertTrue(g.update(state(10.3),TARGET,10.3))

    def test_missing_stale_future_and_precommand_packets_rejected(self):
        for stamp in (None,9.9,10.,10.1,12.,float('nan')):
            g=ServoArrival(10.,(1,))
            self.assertFalse(g.update(state(stamp),TARGET,11.));self.assertEqual(g.count,0)

    def test_logged_no_motion_and_bad_orientation_rejected(self):
        for offset,angle in ((.0020255,0.),(0.,np.deg2rad(1.1))):
            g=ServoArrival(10.,(1,))
            for stamp in (10.1,10.2,10.3):
                self.assertFalse(g.update(state(stamp,offset,angle),TARGET,stamp))
            self.assertEqual(g.count,0)

    def test_bad_sample_and_in_tolerance_drift_restart_stability(self):
        g=ServoArrival(10.,(1,))
        for stamp,offset in ((10.1,0.),(10.2,.002),(10.3,0.),(10.4,.0007)):
            self.assertFalse(g.update(state(stamp,offset),TARGET,stamp))
        self.assertEqual(g.count,1)
        self.assertFalse(g.update(state(10.5,.0007),TARGET,10.5))
        self.assertTrue(g.update(state(10.6,.0007),TARGET,10.6))

    def test_two_mm_accepts_logged_1089mm_but_requires_stable_feedback(self):
        g=ServoArrival(10.,(1,))
        for stamp in (10.1,10.2,10.3):result=g.update(state(stamp,.001089442),TARGET,stamp)
        self.assertTrue(result)

    def test_fast_poll_cannot_shorten_stability_duration(self):
        g=ServoArrival(10.,(1,))
        for stamp in (10.1,10.12,10.14):self.assertFalse(g.update(state(stamp),TARGET,stamp))
        self.assertTrue(g.update(state(10.3),TARGET,10.3))

    def test_small_target_inside_two_mm_needs_actual_progress(self):
        initial=state(9.9,.0012);g=ServoArrival(10.,(1,),initial_state=initial)
        for stamp in (10.1,10.2,10.3):
            self.assertFalse(g.update(state(stamp,.0012),TARGET,stamp))
        self.assertIn('real servo displacement',g.detail)
        for stamp in (10.4,10.5,10.6):result=g.update(state(stamp,.0008),TARGET,stamp)
        self.assertTrue(result)

    def test_invalid_pose_and_uncommanded_arm(self):
        g=ServoArrival(10.,(1,));s=state(10.1);s['right_arm']['position'][0]=float('nan')
        self.assertFalse(g.update(s,TARGET,10.1))
        for stamp in (10.2,10.3,10.4):
            s=state(stamp);s['left_arm']['position'][0]=20
            result=g.update(s,TARGET,stamp)
        self.assertTrue(result)


class ControllerTests(unittest.TestCase):
    def wait(self, samples, *, same_target=False):
        robot=RobotArm.__new__(RobotArm)
        robot.target_command=SimpleNamespace(arm_command_type=TargetCommandArmCommandType.CARTESIAN,
                                             arm_tool_command=TARGET.copy())
        robot.prev_arm_tool_command=TARGET.copy() if same_target else None
        robot._cartesian_commanded_arms=(1,);robot._cartesian_command_sent_monotonic=10.
        t=[10.];reads=[]
        def read():
            s=samples[min(len(reads),len(samples)-1)]
            reads.append(s);return s
        robot.get_current_state=read
        def sleep(seconds):t[0]+=seconds
        with patch('RobotRAM.controllor.hkstp_arm.time.time',side_effect=lambda:t[0]), \
             patch('RobotRAM.controllor.hkstp_arm.time.monotonic',side_effect=lambda:t[0]), \
             patch('RobotRAM.controllor.hkstp_arm.time.sleep',side_effect=sleep), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            result=robot._wait_for_robot_stop(max_wait_time=2.,completion_settle_timeout=.8,
                position_tolerance_m=.001,orientation_tolerance_rad=np.deg2rad(1.),require_fresh_stable=True)
        return result,reads,output.getvalue()

    def test_early_finished_waits_for_actual_pose_and_three_packets(self):
        samples=[state(10.),state(10.09,.002),state(10.19),state(10.29),state(10.39)]
        result,reads,output=self.wait(samples)
        self.assertTrue(result);self.assertEqual(len(reads),5)
        self.assertIn('stable_samples=3/3',output)

    def test_repeated_finished_cache_cannot_succeed_even_for_same_target(self):
        result,_,output=self.wait([state(10.),state(10.09)],same_target=True)
        self.assertFalse(result);self.assertIn('伺服到位检查失败',output)

    def test_logged_no_motion_times_out(self):
        result,_,output=self.wait([state(10.+i*.099,.002026) for i in range(15)])
        self.assertFalse(result);self.assertIn('2.026',output)

    def test_wrong_command_packets_do_not_count(self):
        wrong=state(10.19);wrong['processed_arm_tool_command'][1,1]+=.002
        samples=[state(10.),state(10.09),wrong,state(10.29),state(10.39),state(10.49)]
        result,reads,_=self.wait(samples)
        self.assertTrue(result);self.assertEqual(len(reads),6)

    def test_only_servo_has_two_mm_tolerance(self):
        self.assertEqual(_position_tolerance_for_task('book','book_v3_xyz_servo'),.002)
        self.assertEqual(_position_tolerance_for_task('book','move_down_to_grasp'),.008)
        self.assertEqual(_position_tolerance_for_task('book','move_above_grasp'),.030)

    def test_master_control_enables_gate_only_for_servo(self):
        for action in ('book_v3_xyz_servo','move_above_grasp'):
            robot=Mock(spec=RobotArm)
            robot.ik_lib='curobo';robot.udp=Mock()
            robot.target_command=SimpleNamespace(arm_tool_command=TARGET.copy(),gripper_size=0)
            robot.get_current_state.return_value=state(10.)
            robot._convert_pos_rot_to_quaternion.return_value=(TARGET[1,:3],TARGET[1,3:])
            robot._wait_for_robot_stop.return_value=True
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertTrue(RobotArm.master_control(robot,right_arm=[518.,37.,15.,0.,0.,0.],
                                                       object_category='book',action=action))
            options=robot._wait_for_robot_stop.call_args.kwargs
            if action=='book_v3_xyz_servo':
                self.assertTrue(options['require_fresh_stable'])
                self.assertEqual(options['position_tolerance_m'],.002)
                self.assertAlmostEqual(options['orientation_tolerance_rad'],np.deg2rad(1.))
                self.assertIsInstance(robot._cartesian_command_sent_monotonic,float)
            else:
                self.assertNotIn('require_fresh_stable',options)
                self.assertEqual(options['position_tolerance_m'],.030)

    def test_task_tolerance_override_and_invalid_value_before_send(self):
        for tolerance in (1.,2.,3.,3.01):
            robot=Mock(spec=RobotArm);robot.ik_lib='curobo';robot.udp=Mock()
            robot._v3_servo_position_tolerance_mm=tolerance
            robot.target_command=SimpleNamespace(arm_tool_command=TARGET.copy(),gripper_size=0)
            robot.get_current_state.return_value=state(10.)
            robot._convert_pos_rot_to_quaternion.return_value=(TARGET[1,:3],TARGET[1,3:])
            robot._wait_for_robot_stop.return_value=True
            with contextlib.redirect_stdout(io.StringIO()):
                if tolerance>3.:
                    with self.assertRaises(ValueError):RobotArm.master_control(robot,right_arm=[518.,37.,15.,0.,0.,0.],object_category='book',action='book_v3_xyz_servo')
                    robot.udp.send.assert_not_called()
                else:
                    self.assertTrue(RobotArm.master_control(robot,right_arm=[518.,37.,15.,0.,0.,0.],object_category='book',action='book_v3_xyz_servo'))
                    self.assertEqual(robot._wait_for_robot_stop.call_args.kwargs['position_tolerance_m'],tolerance/1000)
                    self.assertEqual(robot._wait_for_robot_stop.call_args.kwargs['check_interval'],.02)

    def test_cached_packet_keeps_real_receipt_timestamp(self):
        udp=CuarmUdpThread.__new__(CuarmUdpThread)
        udp._terminate_thread=True;udp._closed=True;udp._data_lock=threading.Lock()
        udp._last_receive_time=10.
        udp._data=SimpleNamespace(arm_tool_pose=TARGET.copy(),arm_joint_position=np.zeros((2,7)),
            robot_status=RobotStatus.AVAILABLE,curobo_status=CuroboStatus.MOTION_FINISHED)
        robot=RobotArm.__new__(RobotArm);robot.udp=udp
        a=robot.get_current_state();b=robot.get_current_state()
        self.assertEqual(a['received_monotonic'],10.)
        self.assertEqual(a['received_monotonic'],b['received_monotonic'])
        udp._last_receive_time=10.1
        self.assertEqual(robot.get_current_state()['received_monotonic'],10.1)


if __name__=='__main__':unittest.main()
