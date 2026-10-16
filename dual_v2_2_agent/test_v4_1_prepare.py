"""Offline tests; no hardware/API/CUDA. Fake runtime exercises action ordering."""
import copy
from pathlib import Path
import sys
import tempfile
import threading
from contextlib import nullcontext
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
from book_top_coarse_alignment import coarse_plan,settings,Alignment,fit_response,centerline_u
from book_top_coarse_runtime import execute_sequence,runtime_config,run,read_state
# Import dependencies before scoped class mocks; avoid caching a mocked
# selector in another module and contaminating later regression tests.
import book_top_landing_runtime
from v4_1 import load_config
ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT.parent/'dual_v2_2_control/cuarm_upper_planner/scripts'),str(ROOT.parent/'dual_v2_2_configuration/robot_platform_utils/python/scripts')]
from v4_1_direct_servo import compute_closed_fingers
from cuarm_state import CuroboStatus,GripperMode

def verified_config():
    c=load_config();c["head_camera_review_required"]=False;return c
def config():return runtime_config(verified_config())
def rotation():return Rotation.from_euler('y',-90,degrees=True).as_matrix()
def observation():return dict(top_base_mm=np.array([650.,-160,150]),corners_base_mm=np.array([[650,-175,150],[650,-145,150],[670,-145,-50],[670,-175,-50]],float))

class Geometry(unittest.TestCase):
    def test_tilted_book_top_offset_and_order(self):
        p=coarse_plan(observation(),[410,-50,-10],rotation(),config())
        np.testing.assert_allclose(p['offset_xyz_mm'],[-10,0,100])
        self.assertEqual([k for k,_ in p['stages']],['align_y','raise_z','forward_x'])
        np.testing.assert_allclose([x for _,x in p['stages']],[[410,-160,-10],[410,-160,190],[540,-160,190]])
        self.assertFalse(p['contact_confirmed'])
    def test_already_high_never_lowers(self):
        self.assertEqual(coarse_plan(observation(),[410,-50,250],rotation(),config())['hover_tcp_mm'][2],250)
    def test_retreat_and_wrong_orientation_rejected(self):
        with self.assertRaisesRegex(ValueError,'retreat'):coarse_plan(observation(),[600,-50,0],rotation(),config())
        with self.assertRaisesRegex(ValueError,'horizontal'):coarse_plan(observation(),[410,-50,0],np.eye(3),config())
    def test_nonfinite_config(self):
        c=config();c['coarse_alignment']['gain']=float('nan')
        with self.assertRaises(ValueError):settings(c)
    def test_slanted_line_uses_semantic_intersection(self):
        self.assertAlmostEqual(centerline_u([[600,200],[660,500]],(1280,720)),631.9)
        with self.assertRaisesRegex(ValueError,'supported'):centerline_u([[600,0],[610,100]],(1280,720))
    def test_measured_probe_return_consistency(self):
        j,e=fit_response([0,3.7,.1],[640,647.4,640.2],config());self.assertAlmostEqual(j,2)
        with self.assertRaisesRegex(ValueError,'measured'):fit_response([0,.1,0],[640,648,640],config())
        with self.assertRaisesRegex(ValueError,'inconsistent'):fit_response([0,4,0],[640,648,648],config())

class Controller(unittest.TestCase):
    def test_converges_for_both_jacobian_signs_without_xz_motion(self):
        for j in (2.,-3.):
            q=np.array([540.,-160.,190.]);origin=q.copy();ctrl=Alignment(config(),origin,j,639.5);done=False
            for i in range(200):
                o=dict(captured_unix_s=1+i*.1,u=639.5+j*(q[1]+153),tracking_valid=True)
                r=ctrl.decide(o,q.copy(),q);q+=.6*(r['target']-q)
                np.testing.assert_allclose(q[[0,2]],origin[[0,2]])
                if r['done']:done=True;break
            self.assertTrue(done);self.assertLessEqual(abs(j*(q[1]+153)),3)
    def test_latency_uses_measured_y(self):
        ctrl=Alignment(config(),[540,0,190],2,639.5)
        r=ctrl.decide(dict(captured_unix_s=1,u=647.5,tracking_valid=True),[540,0,190],[540,-4,190])
        self.assertEqual(r['error_px'],0);self.assertEqual(r['target'][1],-4)
    def test_loss_repeat_fixed_z_and_travel_stop(self):
        for change,q in [({'tracking_valid':False},[540,0,190]),({},[540,0,193]),({},[540,31,190])]:
            ctrl=Alignment(config(),[540,0,190],2,639.5);o=dict(captured_unix_s=1,u=640,tracking_valid=True);o.update(change)
            with self.assertRaises(ValueError):ctrl.decide(o,q,q)
        ctrl=Alignment(config(),[540,0,190],2,639.5);o=dict(captured_unix_s=1,u=640,tracking_valid=True)
        ctrl.decide(o,[540,0,190],[540,0,190])
        with self.assertRaises(ValueError):ctrl.decide(o,[540,0,190],[540,0,190])

class Sequence(unittest.TestCase):
    def test_stage_order_and_segment_limit(self):
        q=np.array([410.,-50.,-10.]);events=[];moves=[];io=NS(rotation=rotation(),guard=lambda:dict(right_arm=dict(position=q/1000)))
        h=observation();head=NS(stable=lambda guard:h)
        def move(io,target):
            self.assertLessEqual(np.linalg.norm(target-q),50.00001);moves.append(target.copy());q[:]=target
        r=execute_sequence(io,config(),lambda:(head,h),move,lambda head:'aligned',events.append)
        self.assertEqual(r,'aligned');self.assertEqual([x['name'] for x in events if x['event']=='stage'],['align_y','raise_z','forward_x'])
        for p in moves:
            if p[0]>410:self.assertEqual(p[2],190)
        np.testing.assert_allclose(q,[540,-160,190])
    def test_book_motion_stops_before_raise(self):
        q=np.array([410.,-50.,-10.]);h=observation();bad=copy.deepcopy(h);bad['top_base_mm'][0]+=5
        io=NS(rotation=rotation(),guard=lambda:dict(right_arm=dict(position=q/1000)));align=Mock()
        def move(io,target):q[:]=target
        with self.assertRaisesRegex(ValueError,'book moved'):execute_sequence(io,config(),lambda:(NS(stable=lambda g:bad),h),move,align,lambda r:None)
        self.assertEqual(q[0],410);self.assertEqual(q[2],-10);align.assert_not_called()
    def test_observation_does_not_connect_robot(self):
        cam=Mock();head=Mock();head.stable.return_value=observation()
        with tempfile.TemporaryDirectory() as td,patch('book_top_tilt_servo.servo_connection',side_effect=AssertionError('no robot')) as conn,patch('cart_grasp_alignment.open_head_camera',return_value=cam),patch('book_top_landing_vision.LandingHeadVision.automatic',return_value=head),patch('book_top_landing_vision.LandingSelector'):
            run(NS(output=Path(td)/'run',execute=False),load_config());conn.assert_not_called();cam.close.assert_called_once()
    def test_execute_closes_then_head_then_moves_then_aligns(self):
        order=[];q=np.array([410.,-50.,-10.]);robot=Mock();cam=Mock();head=Mock();head.stable.return_value=observation()
        io=NS(guard=lambda:dict(right_arm=dict(position=q/1000)),rotation=rotation(),close=Mock(),cancel=Mock(),observation_anchor=None,perception_pause=lambda:nullcontext())
        def detect(*a,**k):order.append('head');return head
        def move(io,target):order.append('move');q[:]=target
        def align(*a):order.append('align');return dict(status='aligned',contact_confirmed=False)
        with tempfile.TemporaryDirectory() as td,patch('book_top_tilt_servo.servo_connection',return_value=nullcontext(robot)),patch('book_top_coarse_runtime.close_fingers',side_effect=lambda *a:order.append('close') or 1.),patch('book_top_tilt_servo.ContinuousRobotIO',return_value=io),patch('cart_grasp_alignment.open_head_camera',return_value=cam),patch('book_top_landing_vision.LandingHeadVision.automatic',side_effect=detect),patch('book_top_landing_vision.LandingSelector'),patch('book_top_landing_runtime.coarse_move',side_effect=move),patch('cart_book_grasp_v3_camera.WristCamera'),patch('book_top_landing_vision.LandingWristVision'),patch('book_top_coarse_runtime.servo_align',side_effect=align):
            run(NS(output=Path(td)/'run',execute=True),verified_config())
        self.assertEqual(order[:2],['close','head']);self.assertEqual(order[-1],'align');io.cancel.assert_called_once()

class GripperBackend(unittest.TestCase):
    def planner(self,speed=0):
        class Transport:
            def __init__(self):self.lock=threading.RLock();self.sent=[]
            def begin(self):pass
            def _eligible(self):return NS()
            def _hold(self,p):return NS(gripper_joint_command=[[.3]*6,[.2]],arm_joint_command=[[0]*7,[.1]*7])
            def send(self,c):self.sent.append(copy.deepcopy(c))
        t=Transport();s=dict(velocity=np.full(14,speed),position=np.zeros((2,3)),stamp=0.);guard=NS(trip=Mock())
        p=NS(_v31_bridge=NS(snapshot=lambda **kw:s,guard=guard),_panel_begin_planning=t.begin,target_arm_tool_position=np.zeros((2,3)),target_arm_tool_orientation=np.ones((2,4)),target_gripper_mode=[GripperMode.OPEN,GripperMode.CLOSE],set_gripper_command=Mock(),curobo_cmd=NS(gripper_joint_command=[[0]*6,[1.]]),curobo_state=NS(),_hold_current_arm_command=Mock())
        return p,t
    def test_latched_arm_hold_left_preserved_close_only_once(self):
        p,t=self.planner()
        for stamp in [0.,.1,.25,.3]:
            p._v31_bridge.snapshot()['stamp']=stamp
            with patch('v4_1_direct_servo.time.monotonic',return_value=stamp):compute_closed_fingers(p)
        self.assertEqual(len(t.sent),1);self.assertEqual(t.sent[0].gripper_joint_command,[[.3]*6,[1.]])
        self.assertEqual(t.sent[0].arm_joint_command,[[0]*7,[.1]*7]);self.assertEqual(p.status,CuroboStatus.PLANNING_SUCCEEDED)
    def test_moving_arm_prevents_close(self):
        p,t=self.planner(.1)
        with patch('v4_1_direct_servo.time.monotonic',return_value=0):compute_closed_fingers(p)
        self.assertEqual(t.sent,[]);self.assertEqual(p.status,CuroboStatus.PLANNING)
        p._v31_bridge.guard.trip.assert_not_called()
        p._v31_bridge.snapshot()['stamp']=2.01
        with patch('v4_1_direct_servo.time.monotonic',return_value=2.01):compute_closed_fingers(p)
        self.assertEqual(t.sent,[]);self.assertEqual(p.status,CuroboStatus.PLANNING_FAILED)
        self.assertIn('max_speed=0.100000',p._v31_bridge.guard.trip.call_args.args[0])
    def test_transient_speed_spike_recovers_without_fault_or_early_close(self):
        p,t=self.planner()
        for stamp,speed in [(0.,0.),(.1,.006),(.2,0.),(.3,0.),(.41,0.)]:
            p._v31_bridge.snapshot().update(stamp=stamp,velocity=np.full(14,speed))
            with patch('v4_1_direct_servo.time.monotonic',return_value=stamp):compute_closed_fingers(p)
            if stamp<.41:self.assertEqual(t.sent,[])
        self.assertEqual(len(t.sent),1);p._v31_bridge.guard.trip.assert_not_called()
    def test_repeated_feedback_does_not_count_as_stability(self):
        p,t=self.planner()
        for now in [0.,.1,.3,.5]:
            with patch('v4_1_direct_servo.time.monotonic',return_value=now):compute_closed_fingers(p)
        self.assertEqual(t.sent,[]);self.assertEqual(len(p._v41_close_stable),1)
    def test_wait_does_not_relax_pose_guard(self):
        p,t=self.planner();p.target_arm_tool_position[1,0]=.002
        compute_closed_fingers(p)
        self.assertEqual(t.sent,[]);self.assertEqual(p.status,CuroboStatus.PLANNING_FAILED)

class StartupDiagnostics(unittest.TestCase):
    def test_known_wrong_camera_blocks_before_connection(self):
        c=verified_config();c['head_camera_review_required']=True
        with patch('book_top_tilt_servo.servo_connection') as connection:
            with self.assertRaisesRegex(ValueError,'目标头部视角'):run(NS(execute=True),c)
            connection.assert_not_called()

    def test_existing_failure_is_reported_before_send(self):
        import time
        now=time.monotonic()
        s=dict(received_monotonic=now,pose_sample_monotonic=now,robot_status=0,curobo_status=CuroboStatus.PLANNING_FAILED)
        robot=NS(get_current_state=lambda:s)
        with self.assertRaisesRegex(ValueError,'status=3.*本轮闭指/运动指令尚未发送'):read_state(robot)
        robot._v41_prepare_command_sent=True
        with self.assertRaisesRegex(ValueError,'本轮准备指令发出后'):read_state(robot)
    def test_preexisting_fault_does_not_send_another_hold(self):
        robot=Mock();robot._v41_prepare_command_sent=False
        with tempfile.TemporaryDirectory() as td,patch('book_top_tilt_servo.servo_connection',return_value=nullcontext(robot)),patch('book_top_coarse_runtime.close_fingers',side_effect=ValueError('old failure')),patch('cart_grasp_alignment.open_head_camera') as camera,patch('builtins.print'):
            with self.assertRaisesRegex(ValueError,'old failure'):run(NS(output=Path(td)/'run',execute=True),verified_config())
            robot.master_control.assert_not_called();camera.assert_not_called()
    def test_semantic_uncertainty_preserves_model_reason(self):
        from book_top_landing_vision import LandingSelector
        selector=LandingSelector('book','head',require_pad=False,book_only=True,config={})
        with self.assertRaisesRegex(ValueError,'目标书上沿不可见'):
            selector.decode(dict(scene_id='s',status='uncertain',reason='目标书上沿不可见'),'s',(800,1280,3),False)

if __name__=='__main__':unittest.main()
