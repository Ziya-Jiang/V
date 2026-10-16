"""Offline tests: no cameras, model API, sockets or robot commands."""
import copy
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
from book_top_model_coarse import load_config,stage_target,preview,decode_selection,sample_geometry,execute_sequence,run,confirmed_reset,main


def geometry():
    return dict(spine_center_base_mm=np.array([610.,-120.,30.]),top_base_mm=np.array([600.,-110.,180.]),
                depth_center_base_mm=np.array([650.,-110.,40.]))


def rotation():return Rotation.from_euler('y',-90,degrees=True).as_matrix()


class Targets(unittest.TestCase):
    def test_first_four_steps_exact_axes_and_distinct_depth(self):
        c=load_config();p=preview([410,-50,-10],rotation(),geometry(),c,4)
        np.testing.assert_allclose([s['target_tcp_mm'] for s in p['stages']],
            [[410,-120,-10],[410,-120,230],[600,-120,230]],atol=1e-9)
        self.assertEqual([s['axis'] for s in p['stages']],['Y','Z','X'])
        self.assertFalse(p['contact_confirmed'])
    def test_unresolved_final_offset_cannot_move(self):
        c=load_config();c['final_z_offset_from_top_mm']=None
        with self.assertRaisesRegex(ValueError,'尚未明确'):preview([410,-50,-10],rotation(),geometry(),c,5)
    def test_configured_endpoint_is_70mm_below_hover(self):
        c=load_config();self.assertEqual(c['final_z_offset_from_top_mm'],-20.)
        stages=preview([410,-50,-10],rotation(),geometry(),c,5)['stages']
        before=stages[-2]['target_tcp_mm'];after=stages[-1]['target_tcp_mm']
        np.testing.assert_allclose(before-after,[0,0,70],atol=1e-9)
        offset=rotation()@np.asarray(c['tcp_to_finger_tool_mm'])
        self.assertAlmostEqual(after[2]+offset[2],geometry()['top_base_mm'][2]-20.)
    def test_explicit_signed_offset_does_not_mean_relative_step(self):
        c=load_config();c['final_z_offset_from_top_mm']=30.
        g,axis=stage_target('lower_z',[600,-120,230],rotation(),geometry(),c)
        np.testing.assert_allclose(g,[600,-120,210]);self.assertEqual(axis,2)
        c['final_z_offset_from_top_mm']=-20.
        g,_=stage_target('lower_z',[600,-120,230],rotation(),geometry(),c)
        np.testing.assert_allclose(g,[600,-120,160])
    def test_tool_offset_includes_rotation(self):
        c=load_config();c['tcp_to_finger_tool_mm']=[0,0,20]
        g,_=stage_target('raise_z',[410,-50,0],np.eye(3),geometry(),c)
        self.assertEqual(g[2],210)
    def test_reverse_forward_nonfinite_and_budget_rejected(self):
        with self.assertRaisesRegex(ValueError,'opposite'):stage_target('forward_x',[700,0,0],rotation(),geometry(),load_config())
        g=geometry();g['top_base_mm'][2]=float('nan')
        with self.assertRaises(ValueError):stage_target('raise_z',[400,0,0],rotation(),g,load_config())
        c=load_config();c['max_path_mm']=100
        with self.assertRaisesRegex(ValueError,'budget'):preview([410,-50,-10],rotation(),geometry(),c,4)
    def test_tiny_step_arrival_mismatch_rejected(self):
        c=load_config();c['descent_step_mm']=5
        import json
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'c.json';p.write_text(json.dumps(c))
            with self.assertRaisesRegex(ValueError,'twice'):load_config(p)


class HeadModel(unittest.TestCase):
    def response(self):
        return dict(scene_id='s',status='selected',confidence=.9,top_visible=True,same_book=True,full_spine_visible=False,
                    spine_quad=[[400,200],[500,200],[500,900],[400,900]],book_quad=[[300,150],[700,150],[700,950],[300,950]])
    def test_partial_visible_center_is_explicit(self):
        r=self.response();d=decode_selection(r,'s',(400,400,3));self.assertFalse(d['full_spine_visible'])
        r['top_visible']=False
        with self.assertRaises(ValueError):decode_selection(r,'s',(400,400,3))
    def test_other_book_rejected_but_approximate_regions_need_not_contain(self):
        r=self.response();r['same_book']=False
        with self.assertRaises(ValueError):decode_selection(r,'s',(400,400,3),True)
        r=self.response();r['spine_quad']=[[50,200],[100,200],[100,800],[50,800]]
        selected=decode_selection(r,'s',(400,400,3))
        np.testing.assert_allclose(selected['spine_quad'][0],[19.95,79.8],atol=1e-5)
    def test_depth_center_uses_same_book_not_top_point(self):
        depth=np.full((240,240),600.);depth[60:181,100:126]=500.
        selection=dict(spine_quad=np.float32([[100,60],[125,60],[125,180],[100,180]]),
            book_quad=np.float32([[70,30],[190,30],[190,210],[70,210]]),full_spine_visible=False)
        k=np.array([[400,0,120],[0,400,120],[0,0,1]],float);c2w=np.eye(4)
        c2w[:3,:3]=[[0,0,1],[1,0,0],[0,1,0]]
        g=sample_geometry(None,depth,k,np.zeros(5),c2w,selection)
        self.assertAlmostEqual(g['top_base_mm'][0],500)
        self.assertAlmostEqual(g['depth_center_base_mm'][0],600)
        self.assertEqual(g['spine_center_scope'],'visible_spine_region')
        depth[:]=0
        with self.assertRaises(ValueError):sample_geometry(None,depth,k,np.zeros(5),c2w,selection)


class Sequence(unittest.TestCase):
    def test_center_then_y_then_refresh_height_then_z_x_z(self):
        c=load_config();c['final_z_offset_from_top_mm']=0
        q=np.array([410.,-50.,0]);events=[];ref=object();io=NS(guard=lambda:dict(right_arm=dict(position=q/1000)))
        def detect(reference,name):
            events.append(name)
            if name=='head_upper_edge':self.assertIs(reference,ref)
            return geometry(),ref
        def move(io,stage,g,c,b):events.append(stage);q[:],_=stage_target(stage,q,rotation(),g,c)
        result=execute_sequence(io,c,detect,move,5)
        self.assertEqual(events,['head_center','align_y','head_upper_edge','raise_z','forward_x','lower_z'])
        self.assertFalse(result['contact_confirmed'])
    def test_failed_y_never_raises_or_descends(self):
        io=NS();det=Mock(return_value=(geometry(),object()));move=Mock(side_effect=ValueError('Y failed'))
        with self.assertRaisesRegex(ValueError,'Y failed'):execute_sequence(io,load_config(),det,move,4)
        self.assertEqual(det.call_count,1);self.assertEqual(move.call_count,1)
    def test_step5_rejected_before_robot_or_camera_connect(self):
        c=load_config();c['final_z_offset_from_top_mm']=None
        with patch('book_top_v34_runtime.connection',side_effect=AssertionError('robot')),patch('cart_grasp_alignment.open_head_camera',side_effect=AssertionError('camera')):
            with self.assertRaisesRegex(ValueError,'尚未明确'):run(NS(execute=True,through_step=5),c)


class ResetRouting(unittest.TestCase):
    def reset_fixture(self,wrong=False,ack=True):
        clock=[10.];samples=[0]
        def now():return clock[0]
        def sleep(dt):clock[0]+=dt
        old=Mock(return_value=ack)
        robot=NS(_wait_for_robot_stop=old,right_safe_joint_deg=[115,85,-65,115,90,-25,-30])
        def control(*a,**kw):
            self.assertEqual(a,(False,True,'first_reset'));self.assertTrue(kw['force'])
            return robot._wait_for_robot_stop(require_fresh_stable=True)
        robot.control_joint_position=control
        def state(_):
            samples[0]+=1
            joints=np.r_[np.zeros(7),np.deg2rad(robot.right_safe_joint_deg)]
            if wrong or samples[0]<4:joints[7]+=.05
            return dict(joint_state=joints,joint_velocity=np.zeros(14),
                pose_sample_monotonic=clock[0],right_arm=dict(position=[.409,-.039,-.012]))
        return robot,old,NS(monotonic=now,sleep=sleep),state,samples

    def test_reset_checks_joints_not_cartesian_and_waits_for_fresh_stability(self):
        robot,old,clock,state,samples=self.reset_fixture();log=Mock()
        with patch('book_top_model_coarse.time',clock),patch('book_top_coarse_runtime.read_state',side_effect=state):
            confirmed_reset(robot,log)
        self.assertIs(robot._wait_for_robot_stop,old)
        self.assertFalse(old.call_args.kwargs['require_fresh_stable'])
        self.assertAlmostEqual(old.call_args.kwargs['joint_tolerance_rad'],np.deg2rad(.3))
        event=log.call_args.args[0]
        self.assertGreaterEqual(event['stable_seconds'],.2)
        self.assertGreaterEqual(event['stable_samples'],3)
        self.assertGreater(samples[0],4)
        self.assertAlmostEqual(event['max_joint_error_deg'],0)

    def test_reset_does_not_accept_wrong_joints_or_failed_ack(self):
        for wrong,ack,error in [(True,True,'fresh stable'),(False,False,'first_reset failed')]:
            robot,old,clock,state,_=self.reset_fixture(wrong,ack);log=Mock()
            with patch('book_top_model_coarse.time',clock),patch('book_top_coarse_runtime.read_state',side_effect=state):
                with self.assertRaisesRegex(ValueError,error):confirmed_reset(robot,log)
            self.assertIs(robot._wait_for_robot_stop,old);log.assert_not_called()

    def test_reset_stationary_speed_matches_close_gate(self):
        robot,old,clock,state,_=self.reset_fixture()
        def moving(r):
            s=state(r);s['joint_velocity'][7]=.007;return s
        with patch('book_top_model_coarse.time',clock),patch('book_top_coarse_runtime.read_state',side_effect=moving):
            with self.assertRaisesRegex(ValueError,'fresh stable'):confirmed_reset(robot,Mock())

    def test_reset_only_before_visual_and_real_joint_type(self):
        root=Path(__file__).resolve().parent.parent
        sys.path[:0]=[str(root/'dual_v2_2_control/cuarm_upper_planner'),str(root/'dual_v2_2_configuration/robot_platform_utils/python/scripts')]
        from dual_v2_2_upper_servo_v4_1 import install_landing_routing
        from cuarm_state import TargetCommandArmCommandType as T
        for wrong_type,visual,fault in [(False,False,None),(True,False,None),(False,True,None),(False,False,'latched')]:
            calls=[];g=NS(fault=fault);g.trip=lambda reason:setattr(g,'fault',reason)
            p=NS(_first_target_command_received=True,target_description='book_v3_1_stream' if visual else 'first_reset:joint_reset:123',
                target_arm_type=T.CARTESIAN if wrong_type else T.JOINT,_v31_bridge=NS(guard=g),
                compute=lambda:calls.append('compute'),curobo_cmd={},curobo_state={},_hold_current_arm_command=lambda:calls.append('hold'))
            install_landing_routing(p)
            if visual:p.compute();calls.clear();p.target_description='first_reset:joint_reset:123'
            p.compute();self.assertEqual(calls,['compute'] if not(wrong_type or visual or fault) else ['hold'])


class OperatorAndVisualization(unittest.TestCase):
    def test_missing_viewer_records_file_without_connecting_grpc(self):
        from book_top_model_ui import CoarseVisualization
        with tempfile.TemporaryDirectory() as td,patch('rerun.spawn') as spawn,\
                patch('book_top_model_ui.viewer_ready',return_value=False),\
                patch('rerun.GrpcSink',side_effect=AssertionError('must not connect')) as grpc:
            v=CoarseVisualization(Path(td),spawn_viewer=True)
            spawn.assert_called_once_with(connect=False,recording=v.recording)
            v.safe('event',dict(event='stopped',reason='backend already failed'))
            v.safe('close');self.assertFalse(v.warning);grpc.assert_not_called()
            self.assertGreater((Path(td)/'head_coarse.rrd').stat().st_size,1000)

    def test_ready_viewer_attaches_file_and_grpc_to_explicit_recording(self):
        from book_top_model_ui import CoarseVisualization
        recording=Mock();file_sink=Mock();grpc_sink=Mock()
        with tempfile.TemporaryDirectory() as td,patch('rerun.RecordingStream',return_value=recording),\
                patch('rerun.spawn') as spawn,patch('book_top_model_ui.viewer_ready',return_value=True),\
                patch('rerun.FileSink',return_value=file_sink),patch('rerun.GrpcSink',return_value=grpc_sink):
            v=CoarseVisualization(Path(td),spawn_viewer=True)
            spawn.assert_called_once_with(connect=False,recording=recording)
            recording.set_sinks.assert_called_once_with(file_sink,grpc_sink)
            self.assertFalse(v.warning);v.safe('close')

    def test_viewer_readiness_timeout_is_bounded(self):
        from book_top_model_ui import viewer_ready
        clock=[0.]
        def sleep(dt):clock[0]+=dt
        with patch('book_top_model_ui.time',NS(monotonic=lambda:clock[0],sleep=sleep)),\
                patch('book_top_model_ui.socket.create_connection',side_effect=OSError('not listening')):
            self.assertFalse(viewer_ready(timeout=.2))
        self.assertAlmostEqual(clock[0],.2)

    def test_real_terminal_s_listener_and_terminal_restore(self):
        import os,termios
        from book_top_model_ui import CycleKeys
        from cart_book_grasp_v1_3 import stop_key_listener
        master,slave=os.openpty()
        try:
            with os.fdopen(slave,'r') as stream:
                before=termios.tcgetattr(stream.fileno());keys=CycleKeys()
                with stop_key_listener(keys,stream):
                    os.write(master,b's')
                    self.assertTrue(keys.stop.wait(1))
                    keys.wait_after_failure()  # already requested: never hangs
                self.assertEqual(termios.tcgetattr(stream.fileno()),before)
        finally:os.close(master)

    def test_no_motion_or_fault_clearing_on_retry_or_pause_keys(self):
        from book_top_model_ui import CycleKeys
        keys=CycleKeys();keys.request_retry();keys.request_pause()
        self.assertFalse(keys.stop.is_set())
        keys.request_stop();self.assertTrue(keys.stop.is_set())

    def test_failure_exit_preserves_error_and_success_does_not_repeat(self):
        from contextlib import contextmanager
        @contextmanager
        def listener(keys):
            keys.request_stop();yield
        with patch('cart_book_grasp_v1_3.stop_key_listener',listener):
            with patch('book_top_model_coarse.run',side_effect=ValueError('reset failed')) as run_mock:
                self.assertEqual(main(['--execute','--no-viewer']),2)
                run_mock.assert_called_once()
            with patch('book_top_model_coarse.run') as run_mock:
                self.assertEqual(main(['--execute','--no-viewer']),0)
                run_mock.assert_called_once()

    def test_real_rerun_file_with_frame_targets_and_feedback_no_viewer(self):
        import time
        from book_top_model_ui import CoarseVisualization
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);v=CoarseVisualization(p,spawn_viewer=False)
            self.assertIsNotNone(v.recording)
            selection=dict(spine_quad=np.array([[8,5],[12,5],[12,25],[8,25]]),
                           book_quad=np.array([[5,3],[20,3],[20,28],[5,28]]))
            k=np.array([[40.,0,16],[0,40.,16],[0,0,1.]])
            v.safe('frame',np.zeros((32,32,3),np.uint8),np.full((32,32),700),k,np.zeros(5),np.eye(4),selection)
            v.safe('geometry',geometry())
            v.safe('event',dict(event='stage',name='align_y',start_tcp_mm=[400,0,0],target_tcp_mm=[400,50,0]))
            v.safe('state',dict(right_arm=dict(position=[.4,.05,0]),pose_sample_monotonic=time.monotonic()))
            v.safe('event',dict(event='stopped',reason='test failure, no hardware'))
            v.safe('close');self.assertFalse(v.warning)
            self.assertGreater((p/'head_coarse.rrd').stat().st_size,1000)
            np.testing.assert_allclose(v.points,[[.4,.05,0]])


if __name__=='__main__':unittest.main()
