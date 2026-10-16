"""Offline settling and exposure guards; no hardware, sockets or model API."""
import copy
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np
import book_top_model_coarse as coarse
from cart_book_grasp_v4_1 import RobotIO


class ObservationReady(unittest.TestCase):
    def setUp(self):
        self.now=10.
        self.clock=NS(monotonic=lambda:self.now,time=lambda:self.now+1000,sleep=self.sleep)
        self.io=NS(guard=self.state,log=Mock())

    def sleep(self,dt):self.now+=dt

    def state(self):
        return dict(pose_sample_monotonic=self.now,joint_velocity=[0.]*14,
                    left_arm=dict(position=[.04,1.17,.38],orientation=[1.,0.,0.,0.]),
                    right_arm=dict(position=[.41,-.05,0.],orientation=[1.,0.,0.,0.]))

    def wait(self):
        with patch.object(coarse,'time',self.clock):
            return coarse.wait_observation_ready(self.io,'head_center')

    def test_transient_velocity_then_continuous_window(self):
        def read():
            s=self.state()
            if self.now<10.2:s['joint_velocity'][7]=.007
            return s
        self.io.guard=read
        s=self.wait()
        self.assertGreaterEqual(s['pose_sample_monotonic'],10.3)
        self.assertLess(self.now,10.4)
        self.assertEqual(self.io.log.call_args.args[0]['event'],'observation_ready')

    def test_speed_spike_warns_without_restarting_window(self):
        def read():
            s=self.state()
            if 10.19<self.now<10.22:s['joint_velocity'][0]=.006
            return s
        self.io.guard=read;self.wait()
        self.assertLess(self.now,10.4)
        self.assertTrue(any(x.args[0]['event']=='head_observation_velocity_warning'
                            for x in self.io.log.call_args_list))

    def test_slow_pose_drift_restarts_window_even_with_zero_velocity(self):
        def read():
            s=self.state();s['right_arm']['position'][0]+=min(self.now-10.,.5)*.002
            return s
        self.io.guard=read;self.wait()
        self.assertGreater(self.now,10.65)

    def test_continuous_high_speed_with_stable_pose_does_not_time_out(self):
        def read():
            s=self.state();s['joint_velocity'][7]=.03;return s
        self.io.guard=read;self.wait()
        self.assertLess(self.now,10.4)

    def test_actual_pose_never_settles_times_out(self):
        def read():
            s=self.state();s['right_arm']['position'][0]+=(self.now-10.)*.01;return s
        self.io.guard=read
        with self.assertRaisesRegex(ValueError,'timed out.*TCP span'):self.wait()
        self.assertLess(self.now,13.02)

    def test_duplicate_samples_do_not_accumulate(self):
        def read():
            s=self.state();s['pose_sample_monotonic']=10.01;return s
        self.io.guard=read
        with self.assertRaisesRegex(ValueError,'fresh pose feedback'):self.wait()
        self.assertFalse(any(x.args[0]['event']=='observation_ready' for x in self.io.log.call_args_list))

    def test_invalid_velocity_is_not_stationary(self):
        self.io.guard=lambda:dict(self.state(),joint_velocity=[float('nan')]*14)
        with self.assertRaisesRegex(ValueError,'invalid 14-joint'):self.wait()

    def test_faults_during_settling_are_not_retried(self):
        self.io.guard=Mock(side_effect=ValueError('backend failed'))
        with self.assertRaisesRegex(ValueError,'backend failed'):self.wait()
        self.io.guard.assert_called_once()

    def test_capture_skips_buffered_pre_settle_frame(self):
        camera=NS(captured_unix_s=1009.99);calls=[]
        def capture():
            calls.append(1)
            camera.captured_unix_s=1009.99 if len(calls)==1 else self.clock.time()
            return len(calls),None
        camera.capture_image=capture;guard=Mock()
        with patch.object(coarse,'time',self.clock):
            rgb,_=coarse.capture_observation_frame(camera,coarse.load_config(),guard)
        self.assertEqual(rgb,2);self.assertEqual(guard.call_count,4)

    def test_no_new_frame_times_out_before_model(self):
        camera=NS(captured_unix_s=1009.99,capture_image=lambda:(None,None))
        with patch.object(coarse,'time',self.clock):
            with self.assertRaisesRegex(ValueError,'No fresh head frame'):
                coarse.capture_observation_frame(camera,coarse.load_config(),Mock())

    def test_post_capture_motion_is_still_rejected(self):
        camera=NS(captured_unix_s=1010.,capture_image=lambda:(None,None))
        guard=Mock(side_effect=[None,ValueError('Arm moved during book observation')])
        with patch.object(coarse,'time',self.clock):
            with self.assertRaisesRegex(ValueError,'Arm moved'):
                coarse.capture_observation_frame(camera,coarse.load_config(),guard)

    def test_real_observation_guard_reports_speed_and_drift(self):
        io=RobotIO.__new__(RobotIO);s=self.state()
        s.update(robot_status=0,curobo_status=9,left_gripper=0,right_gripper=1)
        io.read=lambda:copy.deepcopy(s);io.deadline=20.;io.rotation=np.eye(3)
        io.left_start=np.array(s['left_arm']['position'])*1000;io.gripper_modes=[0,1]
        io.current_segment_start=None;io.observation_anchor=np.array([410.,-50.,0.])
        io.c=dict(calibrated_gripper_position=1.,orientation_error_deg=1.)
        io.robot=NS(udp=NS(receive_with_timestamp=lambda:(NS(gripper_joint_position=[[],[1.]]),self.now)))
        with patch('cart_book_grasp_v4_1.time',self.clock):
            io.guard()
            s['joint_velocity'][7]=.007
            with self.assertRaisesRegex(ValueError,'velocity=0.007000.*joint_index_1based=8'):io.guard()
            s['joint_velocity'][7]=0.;s['right_arm']['position'][0]+=.0006
            with self.assertRaisesRegex(ValueError,'right_tcp_drift=0.600'):io.guard()
            # Only the coarse entry opts in. Debounce does not relax TCP drift.
            s['right_arm']['position'][0]-=.0006
            s['joint_velocity'][7]=.006283;io.c['observation_velocity_debounce']=True
            io.guard()
            s['right_arm']['position'][0]+=.0006
            with self.assertRaisesRegex(ValueError,'right_tcp_drift=0.600'):io.guard()


if __name__=='__main__':unittest.main()
