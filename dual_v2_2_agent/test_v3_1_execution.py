"""Offline transition / independent-height tests. No sockets or robot objects."""
import contextlib
import io
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import torch

SCRIPTS=Path(__file__).resolve().parents[1]/'dual_v2_2_control/cuarm_upper_planner/scripts'
sys.path.insert(0,str(SCRIPTS))
from v3_1_execution import ExecutionGuard, PLANNED_Z_MM, MONITOR_Z_MM, HARD_Z_MM, handoff_matches
from v3_1_panel_path import PanelPath
from v3_1_motion_bridge import MotionBridge, SnapshotUnavailable
from test_v3_1_stream import Sim, v31_planner


def snapshot(z=0., x=.5, stamp=1.):
    q=np.zeros(14);q[7]=x;q[9]=z
    return dict(joints=q,velocity=np.zeros(14),position=np.array([[0.,0.,0.],[x,0.,z]]),
                orientation=np.array([[1.,0.,0.,0.]]*2),stamp=stamp)


def command():
    return SimpleNamespace(arm_waypoint_size=6,arm_waypoint_dt=.2,
        arm_waypoint_command=np.zeros((6,2,7)).tolist(),gripper_joint_command=[[.9],[.2]])


class ExecutionTests(unittest.TestCase):
    def test_height_monitor_runs_while_vision_and_gpu_are_blocked(self):
        sent=[];t=[1.];g=ExecutionGuard(sent.append,clock=lambda:t[0],log=lambda s:None)
        g.begin(0.);g.update(snapshot());self.assertTrue(g.publish(command(),snapshot(),1))
        gpu_lock=threading.Lock()
        with gpu_lock:
            # Simulate the vision thread blocked for 0.35s. The independent
            # feedback thread sees the excursion without ever taking GPU lock.
            def feedback():
                for i in range(1,19):
                    t[0]=1.+i*.02;g.update(snapshot(z=i*.0002,stamp=t[0]))
            thread=threading.Thread(target=feedback);thread.start();thread.join(timeout=1.)
            self.assertFalse(thread.is_alive())
        self.assertIn('independent Z monitor',g.fault)
        self.assertEqual(g.reference,0.)
        self.assertAlmostEqual(g.hold_command.arm_waypoint_command[0][1][2],.003)
        self.assertEqual(g.hold_command.gripper_joint_command,[[.9],[.2]])
        self.assertFalse(g.publish(command(),snapshot(stamp=t[0]),2))
        # Hold is immutable; newer drifting measurements cannot drag it along.
        self.assertEqual(sent[-1].arm_waypoint_command,g.hold_command.arm_waypoint_command)

    def test_checked_start_expired_before_dispatch_cannot_publish(self):
        sent=[];g=ExecutionGuard(sent.append,clock=lambda:1.,log=lambda s:None)
        g.begin(0.);checked=snapshot();g.update(snapshot(x=.502))
        self.assertFalse(g.publish(command(),checked,1));self.assertEqual(sent,[])
        g.update(snapshot());self.assertTrue(g.publish(command(),checked,2))
        self.assertEqual(g.published_key,2)

    def test_handoff_accepts_expected_motion_but_not_prediction_error(self):
        old=snapshot();old['velocity'][7]=.1;old['tcp_velocity']=np.array([.1,0.,0.])
        current=snapshot(x=.503,stamp=1.03)
        self.assertTrue(handoff_matches(old,current,1.035))
        current['position'][1,0]+=.001
        self.assertFalse(handoff_matches(old,current,1.035))
        self.assertFalse(handoff_matches(old,snapshot(x=.505,stamp=1.05),1.05))

    def test_feedback_timeout_cancels_last_motion_and_does_not_clear_reference(self):
        sent=[];t=[1.];g=ExecutionGuard(sent.append,clock=lambda:t[0],log=lambda s:None)
        g.begin(-.01);g.update(snapshot(z=-.01));g.publish(command(),snapshot(z=-.01),1)
        t[0]=1.3;g.update(None)
        self.assertIn('stale',g.fault);self.assertEqual(g.reference,-.01)
        self.assertIsNotNone(g.hold_command)
        with self.assertRaises(ValueError):g.begin(-.014)
        g.reset();g.begin(-.011);self.assertEqual(g.reference,-.011)

    def test_thresholds_leave_tracking_margin_and_plane_cannot_silently_change(self):
        self.assertEqual(PLANNED_Z_MM,2.);self.assertEqual(MONITOR_Z_MM,3.);self.assertEqual(HARD_Z_MM,4.)
        g=ExecutionGuard(lambda c:None,log=lambda s:None);g.begin(-.01)
        with self.assertRaisesRegex(ValueError,'fixed Z'):g.begin(-.015)

    def test_correction_after_delayed_image_never_commands_backward_x(self):
        sim=Sim();observe=sim.observe;send=sim.send;sent_at=[]
        def delayed():
            o=observe()
            if sim.observations>7:
                o['image_q']=o['q'].copy();o['image_roll']=o['roll']
                # Arm advances while the image is processed.
                sim.read()
            return o
        def record(q,r):
            sent_at.append((sim.q[0],q[0]));return send(q,r)
        sim.observe=delayed;sim.send=record
        self.assertTrue(sim.run()['distance_complete'])
        self.assertTrue(all(target>=actual-1e-9 for actual,target in sent_at))

    def test_cpu_feedback_branch_does_not_need_cuda_lock(self):
        p=v31_planner();s=snapshot(stamp=time.monotonic())
        p.curobo_state=SimpleNamespace()
        s['panel']=SimpleNamespace(gripper_joint_position=[])
        guard=Mock()
        p._v31_bridge=SimpleNamespace(snapshot=lambda **kw:s,guard=guard)
        p._panel_feedback_source=lambda:(s['panel'],s['stamp'])
        with p._gpu_lock:p._refresh_feedback_pose()
        guard.update.assert_called_once_with(s)
        self.assertEqual(p.curobo_state.pose_sample_monotonic,s['stamp'])

    def test_validation_success_is_not_acknowledged_before_panel_publish(self):
        p=v31_planner();guard=ExecutionGuard(lambda c:None,log=lambda s:None)
        guard.published_key=2
        p._v31_bridge=SimpleNamespace(serial=3,guard=guard)
        p.curobo_state=SimpleNamespace();p._first_robot_state_received=False
        p._refresh_feedback_pose=Mock()
        p._pose_sample_monotonic=time.monotonic()
        from cuarm_state import CuroboStatus
        p.status=CuroboStatus.PLANNING_SUCCEEDED
        p._update_curobo_state()
        self.assertEqual(p.curobo_state.curobo_status,CuroboStatus.PLANNING)

    def test_dispatch_ack_overrides_stale_previous_target_through_real_protocol(self):
        from cuarm_state import CuroboStatus,RobotStatus
        from cuarm_state_initializers import get_initialized_curobo_state
        from test_v3_1_feedback import FeedbackTests
        from cart_book_grasp_v3_1_runtime import StreamTransport
        from cart_book_grasp_v3_1_stream import load_stream_config
        from cart_book_grasp_v3_column import load_config
        old=np.array([[.0401,1.1774,.3821,1.,0.,0.,0.],
                      [.65910186,-.14262576884,-.01042733276,1.,0.,0.,0.]])
        requested=old.copy();requested[1,1]=-.14149125972
        pending=requested.copy()
        g=ExecutionGuard(lambda c:None,clock=lambda:1.,log=lambda s:None)
        g.begin(0.);g.update(snapshot())
        self.assertTrue(g.publish(command(),snapshot(),27,requested=requested))
        requested[1,1]=0.  # The dispatch record must own an immutable copy.
        p=v31_planner();p._v31_bridge=SimpleNamespace(serial=27,guard=g)
        p._first_robot_state_received=False;p._refresh_feedback_pose=Mock()
        p.status=CuroboStatus.PLANNING_SUCCEEDED
        p.curobo_state=get_initialized_curobo_state(arm_size=2,arm_joint_size=[7,7],gripper_size=0,gripper_joint_size=[])
        p.curobo_state.robot_status=RobotStatus.AVAILABLE
        p.curobo_state.arm_tool_pose=old.tolist()
        p.curobo_state.processed_arm_tool_command=old.copy()
        p.curobo_state.pose_sample_monotonic=1.;p.curobo_state.pose_fk_completed_monotonic=1.01
        p._update_curobo_state()
        robot,_=FeedbackTests().robot(p.curobo_state,1.02)
        tr=StreamTransport(robot,[0,0,0],load_config(),load_stream_config(),clock=lambda:1.03)
        tr.pending=pending;tr.sent_at=.99
        self.assertTrue(tr.ready())
        # A new candidate that has not been dispatched must not get status=4.
        p._v31_bridge.serial=28;p._update_curobo_state()
        self.assertEqual(p.curobo_state.curobo_status,CuroboStatus.PLANNING)
        self.assertEqual(g.acknowledgement()[0],27)
        p.status=CuroboStatus.COLLISION_CHECK_FAILED;p._update_curobo_state()
        self.assertEqual(p.curobo_state.curobo_status,CuroboStatus.COLLISION_CHECK_FAILED)
        g.trip('fault');p._update_curobo_state()
        self.assertEqual(p.curobo_state.curobo_status,CuroboStatus.PLANNING_FAILED)
        g.reset();self.assertIsNone(g.acknowledgement()[1])

    def test_transient_source_gap_defers_planning_without_reusing_old_start(self):
        p=v31_planner();p._hold_current_arm_command=Mock()
        p._plan_v31_backoff_attempts=Mock(side_effect=SnapshotUnavailable('waiting'))
        from cuarm_state import CuroboStatus
        self.assertFalse(p._plan_v31_with_z_backoff(None,None,None,None,None))
        self.assertTrue(p._v31_handoff_replan);self.assertFalse(p._v31_backoff_active)
        self.assertEqual(p.status,CuroboStatus.PLANNING)
        p._hold_current_arm_command.assert_called_once()


class PanelPathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.path=PanelPath()

    def test_real_cpp_includes_current_to_first_waypoint_segment(self):
        points=np.linspace(np.zeros(7),np.ones(7)*.1,6)
        q,duration=self.path.sample(np.zeros(7),points,.2,np.ones(7))
        self.assertAlmostEqual(duration,1.2,places=5) # six, not five intervals
        np.testing.assert_allclose(q[0],0.,atol=1e-10)
        np.testing.assert_allclose(q[-1],points[-1],atol=1e-7)
        # Real controller reconstructs the path when measured start changes.
        changed,_=self.path.sample(np.ones(7)*.03,points,.2,np.ones(7))
        np.testing.assert_allclose(changed[0],.03,atol=1e-10)
        self.assertGreater(np.max(np.abs(changed-q)),.02)

    def bridge(self):
        q=np.zeros(14);q[7]=.5
        current=snapshot(stamp=time.monotonic())
        p=SimpleNamespace(config={'robot':{'arm':[{'safety':{'joint_soft_limit':{'velocity':[100.]*7}}}]*2}},
            target_link_names=['left','right'],end_effector_offset=np.zeros((2,3)),
            target_arm_tool_position=np.array([[0.,0.,0.],[.5,0.,0.]]),target_gripper_joint=[],
            reorder_input=lambda q,*args,**kwargs:q,_gpu_lock=threading.Lock(),
            tensor_args=SimpleNamespace(to_device=lambda a:torch.as_tensor(a)),
            _motion_gen=SimpleNamespace(rollout_fn=SimpleNamespace(rollout_constraint=Mock(
                side_effect=lambda x,**kw:SimpleNamespace(feasible=torch.ones(x.shape[:2],dtype=torch.bool))))))
        b=MotionBridge(p,None,self.path)
        b.snapshot=lambda:current
        b.tcp=lambda q:(np.asarray(q).reshape(-1,2,7)[:,:,:3],np.tile([1.,0.,0.,0.],(len(np.atleast_2d(q)),2,1)))
        return b,q,current

    def test_validate_actual_panel_path_and_collision_batch(self):
        b,q,current=self.bridge();points=np.tile(q,(6,1));points[:,7]+=np.linspace(.001,.02,6)
        with contextlib.redirect_stdout(io.StringIO()):self.assertTrue(b.validate(points,.2))
        self.assertIs(b.checked_start,current);self.assertEqual(b.serial,1)
        self.assertGreater(b.p._motion_gen.rollout_fn.rollout_constraint.call_count,0)

    def test_batch_collision_preserves_grippers_joint_order_and_all_three_paths(self):
        b,q,current=self.bridge();points=np.tile(q,(6,1));points[:,7]+=np.linspace(.001,.02,6)
        b.p.target_gripper_joint=[.012,.034]
        b.p.reorder_input=lambda q,grip=None,**kw:np.r_[q,grip][::-1] if grip else q
        with contextlib.redirect_stdout(io.StringIO()):self.assertTrue(b.validate(points,.2))
        checked=np.concatenate([call.args[0][:,0].numpy() for call in
            b.p._motion_gen.rollout_fn.rollout_constraint.call_args_list])
        path,_=self.path.sample(q[7:],points[:,7:],.2,b.limits[1])
        self.assertEqual(len(checked),3*len(path))
        for start in range(0,len(checked),len(path)):
            np.testing.assert_allclose(checked[start],np.r_[q,b.p.target_gripper_joint][::-1])
            np.testing.assert_allclose(checked[start+len(path)-1],np.r_[points[-1],b.p.target_gripper_joint][::-1])

    def test_height_margin_rejects_path_below_old_four_mm_limit(self):
        b,q,_=self.bridge();points=np.tile(q,(6,1));points[:,9]=.0027
        with contextlib.redirect_stdout(io.StringIO()):self.assertFalse(b.validate(points,.2))
        self.assertEqual(b.failure,'height_exceeded');self.assertEqual(b.serial,0)

    def test_splice_back_to_old_x_is_rejected_even_if_z_is_perfect(self):
        b,q,_=self.bridge();points=np.tile(q,(6,1));points[:,7]=np.linspace(.49,.52,6)
        with contextlib.redirect_stdout(io.StringIO()):self.assertFalse(b.validate(points,.2))
        self.assertEqual(b.failure,'handoff');self.assertEqual(b.serial,0)

    def test_reconstructed_curve_collision_is_rejected(self):
        b,q,_=self.bridge();points=np.tile(q,(6,1));points[:,7]+=np.linspace(.001,.01,6)
        b.p._motion_gen.rollout_fn.rollout_constraint.side_effect=None
        b.p._motion_gen.rollout_fn.rollout_constraint.return_value=SimpleNamespace(feasible=torch.tensor(False))
        with contextlib.redirect_stdout(io.StringIO()):self.assertFalse(b.validate(points,.2))
        self.assertEqual(b.failure,'collision')


if __name__=='__main__':unittest.main()
