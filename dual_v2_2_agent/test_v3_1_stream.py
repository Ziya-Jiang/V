"""Offline moving-arm simulation; does not connect to devices."""
import contextlib
import io
import unittest
import threading
import time
import copy
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation as RotationR,Slerp
from cart_book_grasp_v3_1_stream import run_stream,load_stream_config,translation_budget
from cart_book_grasp_v3_1_runtime import StreamTransport
from cart_book_grasp_v3_column import load_config
from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted
from cart_book_grasp_v3_servo import ServoError


def v31_planner(delta=.002,action='book_v3_1_stream'):
    import ast,importlib.util,sys
    import test_v3_planner_microsteps as old
    if str(old.SCRIPTS) not in sys.path:sys.path.insert(0,str(old.SCRIPTS))
    path=old.SCRIPTS/'curobo_planner_v3_1.py'
    policy_path=old.SCRIPTS/'servo_cartesian_policy_v3_1.py'
    spec=importlib.util.spec_from_file_location('v31_policy_test',policy_path)
    policy=importlib.util.module_from_spec(spec);spec.loader.exec_module(policy)
    original=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='CuroboPlanner')
    from cuarm_state import PlannerConnectionMethod
    names=old.names|{'_check_v31_right_tcp_z_path','_retrieve_motion_gen_result',
                    '_initialize_fk_models','raise_if_state_thread_failed','_curobo_state_sending_task','terminate',
                    '_make_stream_plan_config','_plan_v31_with_z_backoff','_plan_v31_backoff_attempts','_plan_cartesian_candidate',
                    '_refresh_feedback_pose','_send_curobo_state','reorder_input'}
    cls=ast.ClassDef(name='V31Planner',bases=[],keywords=[],body=[n for n in original.body if isinstance(n,ast.FunctionDef) and n.name in names],decorator_list=[])
    ns=dict(old.ns,is_book_servo=policy.is_book_servo,poses_close=policy.poses_close,RotationR=RotationR,Slerp=Slerp,MotionGenResult=object,
            threading=threading,time=time,copy=copy,PlannerConnectionMethod=PlannerConnectionMethod,CudaRobotModel=Mock())
    from v3_1_motion_bridge import SnapshotUnavailable
    ns['SnapshotUnavailable']=SnapshotUnavailable
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(path),'exec'),ns)
    p=old.planner(delta,action);p.__class__=ns['V31Planner'];p._gpu_lock=threading.Lock();p._state_thread_error=None
    p.arm_joint_size=[7,7];p._pose_sample_monotonic=None;p._panel_feedback_max_age=.25
    p._panel_feedback_source=lambda:(SimpleNamespace(arm_joint_position=np.zeros((2,7)),
        arm_joint_velocity=np.zeros((2,7)),gripper_joint_position=[]),time.monotonic())
    return p


class Sim:
    def __init__(self,loss=False,blocked=False):
        self.t=1.;self.q=np.array([500.,0.,10.]);self.r=0.;self.goal=self.q.copy();self.roll=0.
        self.sent=[];self.leads=[];self.probes=[];self.finished=[];self.events=[];self.observations=0;self.loss=loss;self.blocked=blocked
        self.j=np.array([[2.,-.2],[.01,1.]])
    def read(self):
        self.t+=.04
        d=self.goal-self.q;length=np.linalg.norm(d)
        if length:self.q+=d*min(1.,.8/length)
        self.r+=np.clip(self.roll-self.r,-.3,.3)
        return dict(q=self.q.copy(),roll=self.r,angles=np.array([0.,0.,self.r]))
    def observe(self):
        state=self.read();self.observations+=1
        if self.loss and self.observations>=10:raise FeaturePointsExhausted(5)
        return dict(state,error=self.j@np.array([self.q[1]+1.,self.r-1.]),scale=1.,stamp=self.t)
    def blocking(self,q,r):
        self.probes.append((q.copy(),r));self.q=q.copy();self.goal=q.copy();self.r=r;self.roll=r;return True
    def send(self,q,r):
        self.leads.append(float(np.linalg.norm(q-self.q)))
        self.sent.append((q.copy(),r,np.linalg.norm(self.goal-self.q)))
        self.goal=q.copy();self.roll=r
    def finish(self,q,r):
        self.finished.append((q.copy(),r));self.q=q.copy();self.goal=q.copy();self.r=r;self.roll=r;return True
    def run(self,settings=None,goal_x=525.):
        return run_stream(self.observe,self.read,self.blocking,self.send,lambda:not self.blocked,self.finish,
            goal_x,load_config(),load_stream_config() if settings is None else settings,log=self.events.append,clock=lambda:self.t,sleep=lambda dt:setattr(self,'t',self.t+dt))


class StreamTests(unittest.TestCase):
    def interval_result(self, dt):
        import torch
        p=v31_planner()
        class JointPlan:
            joint_names=[str(i) for i in range(14)]
            position=torch.zeros((6,14))
            velocity=position.clone();acceleration=position.clone();jerk=position.clone()
            def get_ordered_joint_state(self,names):return self
            def __len__(self):return len(self.position)
        js=JointPlan()
        from cuarm_state import CuroboCommand
        p._motion_gen=SimpleNamespace(get_full_js=lambda plan:plan)
        p.total_arm_joint_names=js.joint_names;p.arm_joint_position=np.zeros(14);p.arm_joint_size=[7,7]
        p._fix_non_moving_arm=lambda q,current:q;p._clip_arm_joint_command=lambda q:q
        p._reshape_list=lambda q,sizes:[q[:7],q[7:]]
        p._check_v31_right_tcp_z_path=Mock(return_value=True)
        p._hold_current_arm_command=Mock();p._check_target_arm_collision=Mock(return_value=True)
        p._move_arm=[False,True]
        p.curobo_cmd=CuroboCommand(gripper_joint_command=[[1.,0.,0.,0.,0.,0.],[.002]])
        result=SimpleNamespace(success=torch.tensor(True),get_interpolated_plan=lambda:js,
            optimized_plan=js,optimized_dt=torch.tensor(dt,dtype=torch.float32),
            ik_time=0.,graph_time=0.,trajopt_time=0.,finetune_time=0.,solve_time=0.,total_time=0.)
        return p,result

    def test_accepted_trajectory_interval_through_actual_panel_serializer(self):
        # No sockets: use the real converter and packer without UDP __init__.
        import test_v3_planner_microsteps  # Register canonical protocol paths.
        from cuarm_state import RemotePanelCommand,PanelTargetMode
        from cuarm_utils import convert_curobo_command_to_remote_panel_command
        from cuarm_udp import CuarmUdp
        packer=object.__new__(CuarmUdp)
        for dt in (.2,[.2],[[.2]]):
            with self.subTest(dt=dt):
                p,result=self.interval_result(dt)
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertTrue(p._retrieve_motion_gen_result(result))
                self.assertIs(type(p.curobo_cmd.waypoints_dt),float)
                panel=RemotePanelCommand(arm_size=2,arm_joint_size=[7,7],
                    arm_joint_command=[[0.]*7,[0.]*7],gripper_size=2,gripper_joint_size=[6,1])
                convert_curobo_command_to_remote_panel_command(panel,p.curobo_cmd,None,PanelTargetMode.WAYPOINT)
                message=packer.pack_remote_panel_command(panel)
                self.assertEqual(panel.arm_waypoint_size,6)
                self.assertAlmostEqual(panel.arm_waypoint_dt,.2,places=6)
                # Reference wire message uses the same data with a Python float.
                panel.arm_waypoint_dt=float(result.optimized_dt.item())
                self.assertEqual(message,packer.pack_remote_panel_command(panel))
                panel.arm_waypoint_dt=np.array(.2,dtype=np.float32)
                with self.assertRaisesRegex(TypeError,'0-d array'):
                    packer.pack_remote_panel_command(panel)

    def test_invalid_trajectory_interval_is_rejected_before_publish(self):
        import test_v3_planner_microsteps  # Register canonical protocol paths.
        from cuarm_state import CuroboStatus
        for dt in (0.,-.2,float('nan'),float('inf'),[],[.2,.3]):
            with self.subTest(dt=dt):
                p,result=self.interval_result(dt)
                before=p.curobo_cmd.waypoints_size
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertFalse(p._retrieve_motion_gen_result(result))
                self.assertEqual(p.status,CuroboStatus.PLANNING_FAILED)
                self.assertEqual(p.curobo_cmd.waypoints_size,before)
                p._hold_current_arm_command.assert_called_once()
                p._check_target_arm_collision.assert_not_called()

    def test_splice_validation_runs_after_last_gpu_collision_check(self):
        p,result=self.interval_result(.2);order=[]
        p._check_v31_right_tcp_z_path=Mock(side_effect=lambda q:order.append('height') or True)
        p._check_target_arm_collision=Mock(side_effect=lambda *args:order.append('collision') or True)
        p._v31_bridge=SimpleNamespace(validate=lambda *args:order.append('splice') or True)
        with contextlib.redirect_stdout(io.StringIO()):self.assertTrue(p._retrieve_motion_gen_result(result))
        self.assertEqual(order,['height','collision','splice'])

    def backoff_planner(self,failures=1,fatal=False):
        p=v31_planner();p.target_arm_tool_position[1,0]+=.1
        p.target_arm_tool_position[1,2]+=.001
        p.target_arm_tool_orientation[1]=RotationR.from_euler('z',2,degrees=True).as_quat()[[3,0,1,2]]
        p._stream_plan_config=SimpleNamespace(time_dilation_factor=0.9)
        p._plan_cartesian_candidate=Mock(side_effect=lambda js,pos,rot,cfg:(pos.copy(),rot.copy()))
        attempts=[]
        def retrieve(result,*,retry_height):
            attempts.append(result)
            p._v31_retryable_height=not fatal
            if len(attempts)<=failures:return False
            p.limited_waypoints=np.zeros((6,14));p.waypoints_dt=.2
            return True
        p._retrieve_motion_gen_result=Mock(side_effect=retrieve)
        p._hold_current_arm_command=Mock()
        return p,attempts

    def execute_backoff(self,p):
        with contextlib.redirect_stdout(io.StringIO()):
            return p._plan_v31_with_z_backoff(object(),p.arm_tool_position,p.arm_tool_orientation,
                                            p.target_arm_tool_position,p.target_arm_tool_orientation)

    def test_stream_snapshot_accepts_list_gripper_feedback_and_preserves_joint_order(self):
        # Real _retrieve_robot_state/gripper_convertion returns a list. The
        # previous CUDA smoke fixture supplied ndarray and hid this crash.
        for container in (list, np.array, tuple):
            with self.subTest(container=container):
                p,_=self.backoff_planner(failures=1)
                arm=np.arange(14,dtype=float)/100
                grip=np.arange(7,dtype=float)/1000
                velocity=arm/10
                p.total_arm_joint_names=[f'arm_{i}' for i in range(14)]
                p.total_gripper_joint_names=[f'grip_{i}' for i in range(7)]
                p.curobo_input_arm_joint_names=p.total_arm_joint_names[::-1]
                p.curobo_input_joint_names=(p.total_arm_joint_names+p.total_gripper_joint_names)[::-1]
                p.total_gripper_joint_size=7
                p.gripper_joint_position=container(grip)
                p.tensor_args=SimpleNamespace(to_device=np.asarray)
                p._v31_bridge=SimpleNamespace(guard=None,duration=1.,snapshot=lambda:dict(
                    joints=arm,velocity=velocity,position=p.arm_tool_position,
                    orientation=p.arm_tool_orientation))
                with patch.dict(type(p)._plan_v31_backoff_attempts.__globals__,JointState=SimpleNamespace):
                    self.assertTrue(self.execute_backoff(p))
                js=p._plan_cartesian_candidate.call_args.args[0]
                np.testing.assert_array_equal(js.position,np.r_[arm,grip][::-1])
                np.testing.assert_array_equal(js.velocity,np.r_[velocity,np.zeros(7)][::-1])
                self.assertEqual(js.joint_names,p.curobo_input_joint_names)
                np.testing.assert_array_equal(p._v31_bridge.requested_target,
                    np.c_[p.target_arm_tool_position,p.target_arm_tool_orientation])
                self.assertNotEqual(p._plan_cartesian_candidate.call_args.args[1][1,0],p.target_arm_tool_position[1,0])

    def test_stream_time_scale_is_cloned_without_changing_other_actions(self):
        p=v31_planner();p._plan_config=SimpleNamespace(time_dilation_factor=.4,clone=lambda:SimpleNamespace(time_dilation_factor=.4))
        fast=p._make_stream_plan_config()
        self.assertEqual(fast.time_dilation_factor,0.9)
        self.assertEqual(p._plan_config.time_dilation_factor,.4)

    def test_height_retry_halves_xy_and_roll_preserves_z_and_requested_goal(self):
        p,attempts=self.backoff_planner();original=p.target_arm_tool_position.copy()
        self.assertTrue(self.execute_backoff(p));self.assertEqual(len(attempts),2)
        np.testing.assert_allclose(attempts[1][0][1,:2],(p.arm_tool_position[1,:2]+original[1,:2])/2)
        self.assertEqual(attempts[1][0][1,2],original[1,2])
        angle=RotationR.from_quat(attempts[1][1][1,[1,2,3,0]]).magnitude()
        self.assertAlmostEqual(np.degrees(angle),1.)
        np.testing.assert_array_equal(p.target_arm_tool_position,original)
        self.assertIsNotNone(p._v31_partial_goal);self.assertFalse(p._v31_backoff_active)
        p._hold_current_arm_command.assert_not_called()

    def test_height_retry_is_bounded_and_other_failures_do_not_retry(self):
        for fatal,count in ((False,5),(True,1)):
            p,attempts=self.backoff_planner(failures=100,fatal=fatal)
            self.assertFalse(self.execute_backoff(p));self.assertEqual(len(attempts),count)
            self.assertFalse(p._v31_backoff_active)
            if not fatal:p._hold_current_arm_command.assert_called_once()

    def test_height_retry_deadline_does_not_reset_between_attempts(self):
        p,attempts=self.backoff_planner(failures=100)
        with patch.dict(type(p)._plan_v31_backoff_attempts.__globals__,time=SimpleNamespace(monotonic=Mock(side_effect=[0.,2.]))):
            self.assertFalse(self.execute_backoff(p))
        self.assertEqual(len(attempts),1)

    def test_partial_arrival_continues_original_request_without_reporting_final_arrival(self):
        import test_v3_planner_microsteps as old
        p,_=self.backoff_planner();self.execute_backoff(p)
        p._prev_target_tool_position=p.target_arm_tool_position.copy()
        p._prev_target_tool_orientation=p.target_arm_tool_orientation.copy()
        self.assertFalse(p._cartesian_target_changed())
        p.arm_tool_position=p._v31_partial_goal[0].copy();p.arm_tool_orientation=p._v31_partial_goal[1].copy()
        self.assertTrue(p._cartesian_target_changed())
        self.assertEqual(old.PlannerTests().status(p),old.CuroboStatus.PLANNING_SUCCEEDED)

    def test_planning_retry_cannot_emit_motion_finished(self):
        import test_v3_planner_microsteps as old
        p=v31_planner(0.);p._v31_backoff_active=True
        self.assertEqual(old.PlannerTests().status(p,old.CuroboStatus.PLANNING),old.CuroboStatus.PLANNING)

    def test_feedback_lock_is_required_without_cuda_graph_and_released_on_failure(self):
        for graph in (False,True):
            p=v31_planner();p._use_cuda_graph=graph;p._first_robot_state_received=False
            p.arm_joint_position=np.zeros(14)
            p._compute_fk_without_lock=Mock(side_effect=RuntimeError('FK buffer error'))
            with p._gpu_lock:
                worker=threading.Thread(target=p._update_curobo_state)
                worker.start();worker.join(timeout=1.)
                self.assertFalse(worker.is_alive())
                p._compute_fk_without_lock.assert_not_called()
            with self.assertRaisesRegex(RuntimeError,'FK buffer error'):p._update_curobo_state()
            self.assertFalse(p._gpu_lock.locked())

    def test_fk_models_have_separate_instances_and_mutable_buffers(self):
        p=v31_planner();config=object()
        factory=Mock(side_effect=lambda cfg:SimpleNamespace(buffer=[]))
        with patch.dict(type(p)._initialize_fk_models.__globals__,CudaRobotModel=factory):
            p._initialize_fk_models(config)
        self.assertEqual(factory.call_count,2)
        self.assertIsNot(p._feedback_kinematics,p._path_kinematics)
        p._path_kinematics.buffer.extend(range(4))
        self.assertEqual(p._feedback_kinematics.buffer,[])

    def test_thread_failure_is_latched_and_propagated_instead_of_silent_death(self):
        from cuarm_state import PlannerConnectionMethod
        p=v31_planner();p._connection_method=PlannerConnectionMethod.REMOTE_TARGET_AND_LOCAL_ROBOT
        p._robot_udp=None;p._terminate_flag=False
        p._update_curobo_state=Mock(side_effect=RuntimeError('batch 1 vs 4'))
        p._send_curobo_state=Mock()
        with contextlib.redirect_stdout(io.StringIO()):p._curobo_state_sending_task()
        self.assertTrue(p._terminate_flag);p._send_curobo_state.assert_not_called()
        with self.assertRaisesRegex(RuntimeError,'refusing further commands') as raised:
            p.raise_if_state_thread_failed()
        self.assertEqual(str(raised.exception.__cause__),'batch 1 vs 4')

    def test_termination_does_not_wait_indefinitely_for_state_thread(self):
        p=v31_planner();p._curobo_state_sending_thread=Mock();p._robot_udp=Mock();p._target_udp=Mock()
        with contextlib.redirect_stdout(io.StringIO()):p.terminate()
        p._curobo_state_sending_thread.join.assert_called_once_with(timeout=2.0)
        p._robot_udp.terminate.assert_called_once();p._target_udp.terminate.assert_called_once()

    def test_logged_2073_mm_drift_recovers_without_advancing_xy_roll(self):
        sim=Sim();observe=sim.observe;injected=[]
        sent_from=[];send=sim.send
        def record_send(q,r):
            sent_from.append((sim.q.copy(),sim.r))
            return send(q,r)
        sim.send=record_send
        def drift():
            o=observe()
            if sim.observations==7:
                sim.q[2]+=2.0734283;o['q']=sim.q.copy()
                sim.t+=.11;o['stamp']=sim.t
                injected.append((len(sim.sent),sim.q.copy(),sim.r))
            return o
        sim.observe=drift;result=sim.run()
        index,q,r=injected[0];goal,roll,_=sim.sent[index]
        # Hold uses the newest read, not the older image-time pose.
        q,r=sent_from[index]
        np.testing.assert_allclose(goal[:2],q[:2]);self.assertEqual(roll,r)
        self.assertEqual(goal[2],10.)
        self.assertTrue(result['converged']);self.assertEqual(len(sim.finished),1)
        self.assertIn('z_recovery_start',[e['event'] for e in sim.events])
        self.assertIn('z_recovery_complete',[e['event'] for e in sim.events])
        self.assertEqual(sim.finished[-1][0][2],10.)

    def test_four_mm_hard_limit_reports_measured_drift(self):
        sim=Sim();observe=sim.observe
        def drift():
            o=observe()
            if sim.observations==7:sim.q[2]+=4.1;o['q']=sim.q.copy()
            return o
        sim.observe=drift
        with self.assertRaisesRegex(ServoError,r'delta=\+4.100mm, motion_limit=4.000mm'):sim.run()
        self.assertEqual(sim.events[-1]['event'],'z_drift_stop')
        self.assertEqual(sim.finished,[])

    def test_final_height_is_rechecked_before_completion(self):
        sim=Sim();finish=sim.finish
        def misleading_finish(q,r):
            result=finish(q,r)
            if len(sim.finished)==1:
                sim.q[2]+=2.5;sim.goal=sim.q.copy()
            return result
        sim.finish=misleading_finish
        result=sim.run()
        self.assertTrue(result['converged'])
        self.assertEqual(len(sim.finished),2)
        self.assertAlmostEqual(sim.q[2],10.)

    def test_unrecovered_z_times_out_even_with_other_measured_motion(self):
        sim=Sim();observe=sim.observe
        def drift():
            o=observe()
            if sim.observations>=7:
                sim.q[2]=12.5;sim.q[1]+=.1;o['q']=sim.q.copy()
            return o
        sim.observe=drift
        with self.assertRaisesRegex(ServoError,'Z recovery timed out'):sim.run()
        self.assertEqual(sim.finished,[])

    def test_fixed_z_target_is_not_scaled_with_lateral_correction(self):
        sim=Sim();observe=sim.observe
        def drift():
            o=observe()
            if sim.observations==4:
                sim.q[2]=11.9;o['q']=sim.q.copy();o['error']=np.array([100.,0.])
            return o
        sim.observe=drift;sim.run()
        self.assertTrue(all(abs(q[2]-10.)<1e-9 for q,_,_ in sim.sent))
        self.assertLessEqual(max(sim.leads),load_config()['max_path_mm']+1e-9)

    def test_right_tcp_path_gate_uses_tool_offset_and_rejects_interior_excursion(self):
        p=v31_planner();p.target_link_names=['arm_end_effector_l','arm_end_effector_r']
        p.end_effector_offset=np.array([[0.,0.,0.],[0.,0.,.05]])
        p.arm_joint_position=np.zeros(14);p._gpu_lock=contextlib.nullcontext()
        p.reorder_input=lambda q,as_curobo: q
        p._fix_non_moving_arm=lambda q,current:q
        p._clip_arm_joint_command=lambda q:q
        p.tensor_args=SimpleNamespace(to_device=lambda q:q)
        def tensor(x):return SimpleNamespace(cpu=lambda:SimpleNamespace(numpy=lambda:x))
        calls=[]
        def fk(q,link_names):
            calls.append((len(q),link_names))
            position=np.zeros((len(q),1,3));position[:,0,2]=q[:,0]-.05
            quat=np.tile([1.,0,0,0],(len(q),1,1))
            return SimpleNamespace(position=tensor(position),quaternion=tensor(quat))
        p._path_kinematics=SimpleNamespace(get_link_poses=fk)
        p._motion_gen=SimpleNamespace(kinematics=Mock())
        samples=np.zeros((257,14));samples[:,0]=p.target_arm_tool_position[1,2]
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(p._check_v31_right_tcp_z_path(samples))
            self.assertEqual([n for n,_ in calls],[128,128,1])
            self.assertTrue(all(names==['arm_end_effector_r'] for _,names in calls))
            samples[100,0]+=.0041
            self.assertFalse(p._check_v31_right_tcp_z_path(samples))
            samples[100,0]=np.nan
            self.assertFalse(p._check_v31_right_tcp_z_path(samples))
            p.target_description='move_down_to_grasp'
            self.assertTrue(p._check_v31_right_tcp_z_path(samples))

    def test_rejected_z_path_is_held_before_waypoints_are_published(self):
        p=v31_planner()
        def tensor(x):return SimpleNamespace(cpu=lambda:SimpleNamespace(numpy=lambda:np.asarray(x)))
        joints=np.zeros((3,14));names=[str(i) for i in range(14)]
        js=SimpleNamespace(joint_names=names,position=tensor(joints),velocity=tensor(joints),
                           acceleration=tensor(joints),jerk=tensor(joints))
        js.get_ordered_joint_state=lambda names:js
        p._motion_gen=SimpleNamespace(get_full_js=lambda plan:plan)
        p.total_arm_joint_names=names;p.arm_joint_position=np.zeros(14);p.arm_joint_size=[7,7]
        p._fix_non_moving_arm=lambda q,current:q;p._clip_arm_joint_command=lambda q:q
        p._reshape_list=lambda q,sizes:q
        p._check_v31_right_tcp_z_path=Mock(return_value=False)
        p._hold_current_arm_command=Mock();p._check_target_arm_collision=Mock()
        p.curobo_cmd=SimpleNamespace(waypoints='unpublished')
        result=SimpleNamespace(success=SimpleNamespace(item=lambda:True),get_interpolated_plan=lambda:js,
                               optimized_plan=js,optimized_dt=tensor(.1))
        with contextlib.redirect_stdout(io.StringIO()):self.assertFalse(p._retrieve_motion_gen_result(result))
        p._hold_current_arm_command.assert_called_once()
        p._check_target_arm_collision.assert_not_called()
        self.assertEqual(p.curobo_cmd.waypoints,'unpublished')
        self.assertEqual(p._check_v31_right_tcp_z_path.call_args.args[0].shape,(7,14))
        from cuarm_state import CuroboStatus
        for reason,expected in (('height_exceeded',CuroboStatus.PLANNING),('fk_error',CuroboStatus.PLANNING_FAILED)):
            p._v31_z_path_failure=reason
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertFalse(p._retrieve_motion_gen_result(result,retry_height=True))
            self.assertEqual(p.status,expected)
            self.assertEqual(p._v31_retryable_height,reason=='height_exceeded')
            self.assertEqual(p.curobo_cmd.waypoints,'unpublished')

    def test_targets_update_before_arrival_and_only_final_waits(self):
        sim=Sim();result=sim.run()
        self.assertTrue(result['converged']);self.assertEqual(len(sim.probes),2)
        self.assertEqual(len(sim.finished),1);self.assertGreater(len(sim.sent),3)
        self.assertTrue(any(remaining>2. for _,_,remaining in sim.sent[1:]))
        self.assertAlmostEqual(sim.q[0],525.)
        self.assertTrue(all(q[0]<=525. and q[2]==10. for q,_,_ in sim.sent))

    def test_point_loss_stops_observing_and_freezes_y_roll(self):
        sim=Sim(loss=True);result=sim.run()
        self.assertFalse(result['converged']);self.assertTrue(result['distance_complete'])
        self.assertEqual(sim.observations,10);self.assertEqual(len(sim.finished),2)
        frozen,roll=sim.finished[0];final,finalroll=sim.finished[-1]
        np.testing.assert_array_equal(frozen[1:],final[1:]);self.assertEqual(roll,finalroll)

    def test_point_loss_preserves_original_z_despite_hold_feedback_drift(self):
        sim=Sim();observe=sim.observe;finish=sim.finish
        def lose_with_drift():
            if sim.observations==9:
                sim.q[2]+=1.;raise FeaturePointsExhausted(5)
            return observe()
        def imperfect_hold(q,r):
            result=finish(q,r)
            if len(sim.finished)==1:
                sim.q[2]+=.8;sim.goal=sim.q.copy()
            return result
        sim.observe=lose_with_drift;sim.finish=imperfect_hold
        result=sim.run()
        self.assertTrue(result['distance_complete'])
        self.assertTrue(all(q[2]==10. for q,_,_ in sim.sent))
        self.assertTrue(all(q[2]==10. for q,_ in sim.probes+sim.finished))
        event=next(e for e in sim.events if e['event']=='point_loss_direct_switch')
        self.assertEqual(event['fixed_xyz_mm'][2],10.)

    def test_pending_planner_cannot_accumulate_commands(self):
        sim=Sim(blocked=True)
        with self.assertRaisesRegex(ServoError,'no measured progress'):sim.run()
        self.assertEqual(sim.sent,[]);self.assertEqual(sim.finished,[])

    def test_geometry_failure_does_not_become_direct_motion(self):
        sim=Sim();observe=sim.observe
        def failing():
            if sim.observations>8:raise ServoError('bad geometry')
            return observe()
        sim.observe=failing
        with self.assertRaisesRegex(ServoError,'bad geometry'):sim.run()
        self.assertEqual(sim.finished,[])

    def test_stale_frame_stops(self):
        sim=Sim();observe=sim.observe
        def stale():
            o=observe();o['stamp']-=1.;return o
        sim.observe=stale
        with self.assertRaisesRegex(ServoError,'stale image'):sim.run()
        self.assertEqual(sim.sent,[])

    def test_controller_nonblocking_is_scoped_and_never_waits(self):
        from RobotRAM.controllor.hkstp_arm import RobotArm
        from test_v3_servo_arrival import TARGET,state
        robot=Mock(spec=RobotArm);robot.ik_lib='curobo';robot.udp=Mock()
        robot._v3_servo_position_tolerance_mm=2.
        robot.target_command=SimpleNamespace(arm_tool_command=TARGET.copy(),gripper_size=0)
        robot.get_current_state.return_value=state(10.)
        robot._convert_pos_rot_to_quaternion.return_value=(TARGET[1,:3],TARGET[1,3:])
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(RobotArm.master_control(robot,right_arm=[518,37,15,0,0,0],
                            action='book_v3_1_stream',object_category='book',wait_for_completion=False))
        robot._wait_for_robot_stop.assert_not_called();robot.udp.send.assert_called_once()
        with self.assertRaises(ValueError):RobotArm.master_control(robot,right_arm=[518,37,15,0,0,0],
                            action='book_v3_xyz_servo',wait_for_completion=False)

    def test_transport_requires_matching_fresh_ack_not_arrival(self):
        from test_v3_servo_arrival import TARGET,state
        t=[10.];robot=SimpleNamespace(get_current_state=lambda:dict(state(t[0],offset=.004),
            pose_sample_monotonic=t[0],pose_fk_completed_monotonic=t[0]))
        events=[]
        tr=StreamTransport(robot,[0,0,0],load_config(),load_stream_config(),clock=lambda:t[0],log=events.append)
        tr.pending=TARGET.copy();tr.sent_at=10.
        self.assertFalse(tr.ready())
        t[0]=10.1;self.assertTrue(tr.ready())  # Still 4 mm from target.
        tr.pending=TARGET.copy();tr.pending[1,0]+=.1;tr.sent_at=10.
        self.assertFalse(tr.ready());t[0]=14.
        with self.assertRaisesRegex(ServoError,'acknowledgement'):tr.ready()
        self.assertEqual(events[-1]['event'],'stream_ack_timeout')
        self.assertFalse(events[-1]['target_matches'])
        np.testing.assert_array_equal(events[-1]['requested'],tr.pending)

    def test_stream_planner_uses_fine_comparison_and_holds_left(self):
        p=v31_planner(.002,'book_v3_1_stream');self.assertTrue(p._cartesian_target_changed())
        p._update_move_arm_cartesian(p.target_arm_tool_position,p.target_arm_tool_orientation,
                                    p.arm_tool_position,p.arm_tool_orientation)
        self.assertEqual(p._move_arm,[False,True])


    def test_transport_rejects_stale_and_planning_failure(self):
        from test_v3_servo_arrival import state
        for mode in ('stale','failure'):
            sample=state(10.);sample['curobo_status']=3 if mode=='failure' else 9
            tr=StreamTransport(SimpleNamespace(get_current_state=lambda:sample),[0,0,0],load_config(),
                               load_stream_config(),clock=lambda:11. if mode=='stale' else 10.)
            with self.assertRaises(ServoError):tr.read()

    def test_stream_motiongen_starts_from_measured_velocity_only_for_v31(self):
        from test_v3_planner_microsteps import PlannerCuroboSolverType,PlannerInputStateType
        for action in ('book_v3_1_stream','book_v3_xyz_servo'):
            p=v31_planner(.002,action);p._solver_type=PlannerCuroboSolverType.MOTION_GEN
            p.get_input_state=lambda key:np.ones(14)*.1
            p.end_effector_offset=np.zeros((2,3));p.target_size=2
            p.tensor_args=SimpleNamespace(to_device=lambda x:np.asarray(x))
            captured={}
            def joint(**kw):captured.update(kw);raise RuntimeError('captured')
            with patch.dict(type(p)._motion_gen_plan_single.__globals__,JointState=joint),contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError,'captured'):p._motion_gen_plan_single()
            np.testing.assert_allclose(captured['velocity'],.1 if action=='book_v3_1_stream' else 0.)

    def test_readiness_rejects_absent_old_or_reused_pid(self):
        import importlib.util,json,tempfile
        from pathlib import Path
        source=Path(__file__).resolve().parent.parent/'dual_v2_2_control/cuarm_upper_planner/scripts/v3_1_readiness.py'
        spec=importlib.util.spec_from_file_location('readiness_test',source)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as d,patch.object(module,'receipt_path',return_value=Path(d)/'ready.json'):
            with self.assertRaises(ValueError):module.require_ready()
            module.publish_ready(10176);module.require_ready()
            path=Path(d)/'ready.json';data=json.loads(path.read_text());data['start']='old'
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):module.require_ready()


    def test_original_planner_does_not_enable_v31_behavior(self):
        from test_v3_planner_microsteps import planner,policy
        self.assertFalse(policy.is_book_servo('book_v3_1_stream'))
        self.assertFalse(planner(.002,'book_v3_1_stream')._cartesian_target_changed())

    def test_agent_port_scope_restores_on_error(self):
        from cart_book_grasp_v3_1_ports import dedicated_planner
        from cuarm_udp import CuarmUdp
        previous=CuarmUdp.unpack_curobo_state
        agent=SimpleNamespace(TARGET_LOCAL_PORT=10165,TARGET_REMOTE_PORT=10166)
        with self.assertRaises(RuntimeError):
            with dedicated_planner(agent):
                self.assertEqual((agent.TARGET_LOCAL_PORT,agent.TARGET_REMOTE_PORT),(10175,10176))
                self.assertIsNot(CuarmUdp.unpack_curobo_state,previous)
                raise RuntimeError('test')
        self.assertEqual((agent.TARGET_LOCAL_PORT,agent.TARGET_REMOTE_PORT),(10165,10166))
        self.assertIs(CuarmUdp.unpack_curobo_state,previous)

    def test_new_entry_uses_separate_modules_and_ports(self):
        from pathlib import Path
        root=Path(__file__).resolve().parent.parent/'dual_v2_2_control/cuarm_upper_planner'
        original=(root/'dual_v2_2_upper_planner.py').read_text()
        new=(root/'dual_v2_2_upper_planner_v3_1.py').read_text()
        self.assertNotIn('publish_ready',original)
        self.assertNotIn('book_v3_1_stream',(root/'scripts/curobo_planner.py').read_text())
        self.assertIn('from curobo_planner_v3_1 import CuroboPlanner',new)
        self.assertIn('"CUARM_V31_TARGET_LOCAL_PORT", "10176"',new)
        self.assertIn('"CUARM_V31_TARGET_REMOTE_PORT", "10175"',new)
        self.assertIn('check_panel_port_available(panel_local)',new)


    def test_stable_profile_keeps_forward_lookahead_separate_from_v3(self):
        profile=load_stream_config();base=load_config()
        self.assertEqual((profile['gain'],profile['lookahead_mm']),(.6,None))
        self.assertEqual((base['gain'],base['max_step_mm']),(.5,5.))
        fast=Sim();fast.run()
        slow=Sim();slow.run(dict(profile,gain=.5,lookahead_mm=5.))
        self.assertGreater(max(fast.leads),10.)
        self.assertLessEqual(max(fast.leads),base['max_path_mm']+1e-9)
        self.assertGreater(abs(fast.sent[0][0][1]-3.),abs(slow.sent[0][0][1]-3.))
        self.assertTrue(all(goal[0]<=525. for goal,_,_ in fast.sent))

    def test_no_per_target_cap_can_exceed_100_mm_without_overshooting_endpoint(self):
        sim=Sim();result=sim.run(goal_x=700.)
        self.assertTrue(result['converged'])
        self.assertGreater(max(sim.leads),100.)
        self.assertTrue(all(np.isfinite(q).all() and q[0]<=700. for q,_,_ in sim.sent))
        self.assertLessEqual(result['path_mm'],load_config()['max_path_mm'])

    def test_uncapped_translation_still_has_finite_remaining_path_budget(self):
        c=load_config();s=load_stream_config()
        self.assertEqual(translation_budget(c,s,390.),10.)
        with self.assertRaisesRegex(ServoError,'path budget'):translation_budget(c,s,c['max_path_mm'])
        self.assertEqual(translation_budget(c,dict(s,lookahead_mm=5.),390.),5.)

    def test_transport_names_each_planner_failure(self):
        from test_v3_servo_arrival import state
        for code,name in ((3,'PLANNING_FAILED'),(7,'IK_FAILED'),(8,'COLLISION_CHECK_FAILED')):
            packet=state(10.);packet['curobo_status']=code
            tr=StreamTransport(SimpleNamespace(get_current_state=lambda:packet),[0,0,0],load_config(),load_stream_config(),clock=lambda:10.)
            with self.assertRaisesRegex(ServoError,f'status={code}.*{name}'):tr.read()


if __name__=='__main__':unittest.main()
