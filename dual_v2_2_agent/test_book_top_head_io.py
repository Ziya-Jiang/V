"""Head-camera geometry and arm holding are separate contracts; no hardware."""
import copy
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
from book_top_head_io import HeadCoarseRobotIO
from cart_book_grasp_v4_1 import RobotIO
from book_top_model_coarse import load_config,move_stage


class HeadHold(unittest.TestCase):
    def setUp(self):
        self.now=10.
        self.s=dict(received_monotonic=10.,pose_sample_monotonic=10.,
            joint_velocity=[0.]*14,robot_status=0,curobo_status=9,
            left_gripper=0,right_gripper=1,
            left_arm=dict(position=[.04,1.17,.38],orientation=[1.,0.,0.,0.]),
            right_arm=dict(position=[.41,-.05,0.],orientation=[1.,0.,0.,0.]))
        self.robot=NS(get_current_state=lambda:copy.deepcopy(self.s),
            udp=NS(receive_with_timestamp=lambda:(NS(gripper_joint_position=[[],[1.]]),self.now)))
        self.c=dict(max_seconds=100.,calibrated_gripper_position=1.,orientation_error_deg=1.,head_observation_hold_mm=3.)
        self.clock=patch('cart_book_grasp_v4_1.time',NS(monotonic=lambda:self.now))
        self.clock.start();self.addCleanup(self.clock.stop)
        self.io=HeadCoarseRobotIO(self.robot,self.c,Mock())
        self.io.begin_head_observation(self.io.guard(),'head_center')

    def advance(self,dt=.02):
        self.now+=dt;self.s.update(received_monotonic=self.now,pose_sample_monotonic=self.now)

    def test_reported_0551_drift_and_0002094_speed_no_longer_invalidate_head(self):
        self.advance();self.s['right_arm']['position'][0]+=.000551
        self.s['joint_velocity'][8]=.002094
        self.io.guard();self.io.end_head_observation()
        self.assertAlmostEqual(self.io.log.call_args.args[0]['peak_right_tcp_drift_mm'],.551)

    def test_position_budget_is_fixed_not_a_sliding_reanchor(self):
        for drift in [.8,1.6,2.4]:
            self.advance();self.s['right_arm']['position'][0]=.410+drift/1000;self.io.guard()
        self.advance();self.s['right_arm']['position'][0]=.4132
        with self.assertRaisesRegex(ValueError,'arm hold exceeded.*3.200'):self.io.guard()

    def warnings(self):
        return [x.args[0] for x in self.io.log.call_args_list
                if x.args[0]['event']=='head_observation_velocity_warning']

    def test_large_speed_warns_and_continues(self):
        self.advance();self.s['joint_velocity'][8]=.03
        self.io.guard()
        self.assertEqual(self.warnings()[0]['joint_index_1based'],9)
        self.assertEqual(self.warnings()[0]['policy'],'warn_only_continue')

    def test_reported_sustained_speed_warns_without_cancel_or_log_flood(self):
        for _ in range(120):
            self.advance();self.s['joint_velocity'][8]=.006283;self.io.guard()
        self.assertEqual(len(self.warnings()),3)
        for _ in range(10):self.io.guard()
        self.assertEqual(len(self.warnings()),3)

    def test_velocity_warning_does_not_disable_position_protection(self):
        self.advance();self.s['joint_velocity'][8]=.03;self.io.guard()
        self.advance();self.s['right_arm']['position'][0]+=.0031
        with self.assertRaisesRegex(ValueError,'arm hold exceeded'):self.io.guard()

    def test_invalid_velocity_remains_feedback_fault(self):
        self.advance();self.s['joint_velocity'][8]=float('nan')
        with self.assertRaisesRegex(ValueError,'Invalid joint velocity'):self.io.guard()

    def test_warning_policy_preserves_timestamp_reversal_check(self):
        self.advance();self.s['joint_velocity'][8]=.03;self.io.guard()
        self.s['pose_sample_monotonic']-=.01
        with self.assertRaisesRegex(ValueError,'timestamp reversed'):self.io.guard()

    def test_task_orientation_tolerance_is_independent_of_image(self):
        self.advance()
        self.s['right_arm']['orientation']=Rotation.from_euler('z',.2,degrees=True).as_quat()[[3,0,1,2]]
        self.io.guard()
        self.advance()
        self.s['right_arm']['orientation']=Rotation.from_euler('z',1.1,degrees=True).as_quat()[[3,0,1,2]]
        with self.assertRaisesRegex(ValueError,'orientation drift'):self.io.guard()

    def test_stale_feedback_still_stops(self):
        self.now+=.3
        with self.assertRaisesRegex(ValueError,'feedback stale'):self.io.guard()

    def test_long_consumer_pause_with_fresh_feedback_only_records_sampling_gap(self):
        self.advance(.36)
        self.io.guard()
        gaps=[x.args[0] for x in self.io.log.call_args_list
              if x.args[0]['event']=='head_observation_sampling_gap']
        self.assertEqual(len(gaps),1)
        self.assertAlmostEqual(gaps[0]['checked_sample_gap_seconds'],.36)
        self.assertFalse(gaps[0]['continuous_reception_verified'])

    def test_fresh_receive_does_not_mask_stale_pose(self):
        self.now+=.36;self.s['received_monotonic']=self.now
        with self.assertRaisesRegex(ValueError,'feedback stale: pose_sample_monotonic'):self.io.guard()

    def test_fresh_pose_does_not_mask_stale_receive(self):
        self.now+=.36;self.s['pose_sample_monotonic']=self.now
        with self.assertRaisesRegex(ValueError,'feedback stale: received_monotonic'):self.io.guard()

    def test_left_arm_motion_still_stops(self):
        self.advance();self.s['left_arm']['position'][0]+=.0011
        with self.assertRaisesRegex(ValueError,'Left arm moved'):self.io.guard()

    def test_gripper_change_still_stops(self):
        self.advance();self.s['right_gripper']=0
        with self.assertRaisesRegex(ValueError,'Gripper command changed'):self.io.guard()

    def test_legacy_observation_rule_unchanged(self):
        old=RobotIO(self.robot,self.c,Mock());old.observation_anchor=np.array([410.,-50.,0.])
        self.advance();self.s['right_arm']['position'][0]+=.000551
        with self.assertRaisesRegex(ValueError,'right_tcp_drift=0.551'):old.guard()

    def test_next_stage_uses_current_pose_for_held_axes(self):
        self.advance();self.s['right_arm']['position']=[.410551,-.05,.0002]
        self.io.guard();self.io.end_head_observation();targets=[]
        def move(io,target,stage):
            targets.append(np.array(target));self.advance()
            self.s['right_arm']['position']=(target/1000).tolist()
        geometry=dict(spine_center_base_mm=np.array([600.,-60.,30.]),
                      top_base_mm=np.array([600.,-60.,100.]),depth_center_base_mm=np.array([600.,-60.,30.]))
        with patch('book_top_v34_runtime.coarse_move',side_effect=move):
            move_stage(self.io,'align_y',geometry,load_config(),[0.])
        np.testing.assert_allclose(targets[0],[410.551,-60.,.2])


if __name__=='__main__':unittest.main()
