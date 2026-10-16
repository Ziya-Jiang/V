"""Real protocol -> RobotArm -> timestamp pairing, with no sockets or motion."""
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np

from test_v3_1_stream import v31_planner
from RobotRAM.controllor.hkstp_arm import RobotArm
from cuarm_udp import CuarmUdp
from cuarm_state import CuroboStatus, RobotStatus
from cuarm_state_initializers import get_initialized_curobo_state
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'dual_v2_2_control/cuarm_upper_planner/scripts'))
from v3_1_feedback import pack_feedback, unpack_feedback
from cart_book_grasp_v3_1_runtime import StreamTransport, FeedbackStale
from cart_book_grasp_v3_1_stream import load_stream_config
from cart_book_grasp_v3_column import load_config
from cart_book_grasp_v3_servo import ServoError


class FeedbackTests(unittest.TestCase):
    def planner(self):
        p=v31_planner()
        p.curobo_state=get_initialized_curobo_state(arm_size=2,arm_joint_size=[7,7],gripper_size=0,gripper_joint_size=[])
        p.curobo_state.curobo_status=CuroboStatus.PLANNING_SUCCEEDED
        p.curobo_state.robot_status=RobotStatus.AVAILABLE
        p.arm_joint_position=np.zeros(14)  # Main planner's intentionally old input.
        panel=SimpleNamespace(arm_joint_position=np.full((2,7),.2),arm_joint_velocity=np.zeros((2,7)),gripper_joint_position=[])
        source=[panel,time.monotonic()-.01]
        p._panel_feedback_source=lambda:tuple(source)
        def fk(q):
            return np.array([[0.,0.,0.],[.5+q[7]/100.,0.,0.]]),np.array([[1.,0,0,0],[1.,0,0,0]])
        p._compute_fk_without_lock=Mock(side_effect=fk)
        return p,source

    def robot(self,state,receipt):
        codec=object.__new__(CuarmUdp)
        wire=pack_feedback(codec,state)
        decoded=unpack_feedback(codec,wire)
        robot=object.__new__(RobotArm)
        robot.udp=SimpleNamespace(receive_with_timestamp=lambda:(decoded,receipt),terminate=lambda:None)
        return robot,wire

    def test_panel_snapshot_drives_fk_and_cached_heartbeats_keep_source_time(self):
        p,source=self.planner();p._refresh_feedback_pose()
        np.testing.assert_array_equal(p._compute_fk_without_lock.call_args.args[0],np.full(14,.2))
        self.assertEqual(p.curobo_state.pose_sample_monotonic,source[1])
        initial=p.curobo_state.arm_tool_pose
        source[:]=[SimpleNamespace(arm_joint_position=np.full((2,7),.3),
                    arm_joint_velocity=np.zeros((2,7)),gripper_joint_position=[]),time.monotonic()]
        with p._gpu_lock:
            p._refresh_feedback_pose()
        self.assertEqual(p._compute_fk_without_lock.call_count,1)
        self.assertEqual(p.curobo_state.arm_tool_pose,initial)
        old_time=p.curobo_state.pose_sample_monotonic
        tr=None
        for receipt in (old_time+.02,old_time+.08,old_time+.15):
            robot,_=self.robot(p.curobo_state,receipt)
            if tr is None:tr=StreamTransport(robot,[0,0,0],load_config(),load_stream_config(),clock=lambda:receipt)
            tr.robot=robot;state=tr.read()
            self.assertEqual(state['pose_sample_monotonic'],old_time)
            self.assertEqual(state['received_monotonic'],receipt)
        self.assertEqual(len(tr.timeline.samples),1)
        p._refresh_feedback_pose()
        self.assertEqual(p._compute_fk_without_lock.call_count,2)
        self.assertEqual(p.curobo_state.pose_sample_monotonic,source[1])
        self.assertNotEqual(p.curobo_state.arm_tool_pose,initial)

    def test_old_source_is_stale_even_when_udp_packet_just_arrived(self):
        p,source=self.planner();p._refresh_feedback_pose()
        received=source[1]+.3
        robot,_=self.robot(p.curobo_state,received)
        tr=StreamTransport(robot,[0,0,0],load_config(),load_stream_config(),clock=lambda:received)
        with self.assertRaisesRegex(FeedbackStale,'stale pose sample'):tr.read()
        self.assertEqual(len(tr.timeline.samples),0)
        tr.pending=np.zeros((2,7));tr.sent_at=source[1]
        self.assertFalse(tr.ready());self.assertIsNotNone(tr.pending)
        robot.master_control=Mock()
        self.assertFalse(tr.send(np.zeros(3),0.))
        robot.master_control.assert_not_called()

    def test_stale_source_duplicate_and_stale_fk_result_are_not_published(self):
        p,source=self.planner();p._refresh_feedback_pose()
        stamp=p.curobo_state.pose_sample_monotonic
        p._refresh_feedback_pose();self.assertEqual(p._compute_fk_without_lock.call_count,1)
        source[1]=stamp+.01
        with patch.object(time,'monotonic',return_value=source[1]+.3):p._refresh_feedback_pose()
        self.assertEqual(p._compute_fk_without_lock.call_count,1)
        with patch.object(time,'monotonic',side_effect=[source[1]+.01,source[1]+.3]):p._refresh_feedback_pose()
        self.assertEqual(p._compute_fk_without_lock.call_count,2)
        self.assertEqual(p.curobo_state.pose_sample_monotonic,stamp)
        self.assertFalse(p._gpu_lock.locked())

    def test_real_protocol_roundtrip_and_missing_provenance_rejected(self):
        p,source=self.planner();p._refresh_feedback_pose()
        robot,wire=self.robot(p.curobo_state,time.monotonic())
        state=robot.get_current_state()
        self.assertEqual(state['pose_sample_monotonic'],source[1])
        np.testing.assert_array_equal(state['right_arm']['position'],p.arm_tool_position[1])
        codec=object.__new__(CuarmUdp)
        for bad in (wire.split('V31POSE1#')[0],wire.replace('V31POSE1#','V31POSE1#nan#'),
                    wire.split('V31POSE1#')[0]+'V31POSE1#2#1#'):
            with self.assertRaises(ValueError):unpack_feedback(codec,bad)
        # Legacy decoder still understands the body; original planner unchanged.
        legacy=CuarmUdp.unpack_curobo_state(codec,wire.split('V31POSE1#')[0])
        self.assertFalse(hasattr(legacy,'pose_sample_monotonic'))

    def test_source_time_backwards_and_missing_metadata_are_fatal(self):
        p,source=self.planner();p._refresh_feedback_pose()
        robot,_=self.robot(p.curobo_state,time.monotonic())
        tr=StreamTransport(robot,[0,0,0],load_config(),load_stream_config())
        tr.read()
        state=robot.get_current_state();state['pose_sample_monotonic']-=.01
        tr.robot=SimpleNamespace(get_current_state=lambda:state)
        with self.assertRaisesRegex(ServoError,'backwards'):tr.read()
        del state['pose_sample_monotonic']
        with self.assertRaisesRegex(ServoError,'invalid or stale'):tr.read()


if __name__=='__main__':unittest.main()
