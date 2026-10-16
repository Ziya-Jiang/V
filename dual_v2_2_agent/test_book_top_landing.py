"""No hardware/API: landing geometry, observer ambiguity, servo and phase routing."""
import copy
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from cart_book_grasp_v4_1 import load_config,main
from book_top_landing import coarse_plan,head_geometry,fit_jacobian,LandingController,contact_reference,pad_fingerprint
from book_top_landing_vision import measured_pad,quad_center,LandingSelector


def config():
    c=load_config();c.update(pad_source='measured',pad_calibrated=True,scene_setup_confirmed=True,
        calibrated_gripper_position=0.,pad=dict(center_tcp_mm=[0,0,50],normal_tcp=[0,0,-1],
        along_tcp=[1,0,0],length_mm=60,width_mm=16,edge_margin_mm=2))
    return c

class GeometryTests(unittest.TestCase):
    def test_tilted_spine_coarse_raise_then_translate(self):
        h=dict(corners_base_mm=np.array([[600,-15,200],[600,15,200],[620,15,0],[620,-15,0]],float))
        np.testing.assert_allclose(head_geometry(h)['offset_xyz_mm'],[-10,0,100])
        plan=coarse_plan(h,np.array([400,0,0.]),np.eye(3),config())
        np.testing.assert_allclose(plan['tcp_hover_mm'],[600,0,162])
        np.testing.assert_allclose(plan['waypoints_mm'][0],[400,0,162])
        with self.assertRaisesRegex(ValueError,'budget'):coarse_plan(h,np.array([-1000,0,0.]),np.eye(3),config())
    def test_auto_pad_measured_underside_not_vertical_side(self):
        p=np.array([[570,-8,200],[630,-8,200],[630,8,200],[570,8,200]],float)
        pad=measured_pad(p,np.array([600,0,150]),np.eye(3))
        np.testing.assert_allclose(pad['center_tcp_mm'],[0,0,50]);self.assertEqual(pad['length_mm'],60)
        np.testing.assert_allclose(pad['normal_tcp'],[0,0,-1])
        p=np.array([[570,0,200],[630,0,200],[630,0,216],[570,0,216]],float)
        with self.assertRaisesRegex(ValueError,'horizontal'):measured_pad(p,np.zeros(3),np.eye(3))
    def test_perspective_patch_center(self):
        np.testing.assert_allclose(quad_center([[0,0],[10,0],[7,10],[3,10]]),[5,50/7])
    def test_actual_probe_motion_and_rank(self):
        q=np.array([[3.3,.1,0],[-3.2,-.1,.1],[0,3.4,0],[0,-3.5,0],[0,.1,3.7],[0,0,-3.8]])
        true=np.array([[2,-.2,.3],[.1,3,-.7]])
        j,err=fit_jacobian(q,q@true.T,config());np.testing.assert_allclose(j,true,atol=1e-12)
        self.assertLess(err,1e-10)
        with self.assertRaisesRegex(ValueError,'distinguish'):fit_jacobian(np.eye(3),np.array([[2,0],[2,0],[1,1]]),config())
    def test_reference_invalidation(self):
        c=config();cam=SimpleNamespace(size=(1280,720),check_reference=lambda v:None)
        r=dict(kind='human_taught_visual_contact',coordinate_system='undistorted_K',contact_uv=[600,300],pad_fingerprint=pad_fingerprint(c))
        np.testing.assert_allclose(contact_reference(r,cam,c),[600,300]);c['pad']['center_tcp_mm'][0]=1
        with self.assertRaisesRegex(ValueError,'changed'):contact_reference(r,cam,c)

class LandingTests(unittest.TestCase):
    def setUp(self):
        self.c=config();self.j=np.array([[2.,.2,.3],[-.1,3.,-.5]])
        self.origin=np.array([600.,0,162]);self.goal=np.array([640.,360.]);self.top=np.array([600.,0,200])
        self.ctrl=LandingController(self.c,self.top,self.origin,np.eye(3),self.j,self.goal)
    def obs(self,tcp,stamp,uv=None):
        v=self.goal+self.j@(tcp-[600,0,150]) if uv is None else np.asarray(uv)
        return dict(top_base_mm=self.top.copy(),captured_unix_s=stamp,tracking_valid=True,wrist_contact_uv=v,pad_contact_uv=self.goal.copy())
    def decide(self,tcp,stamp,stationary=False,uv=None):
        return self.ctrl.decide(self.obs(tcp,stamp,uv),tcp,np.eye(3),tcp,np.eye(3),stationary=stationary)
    def test_pixel_overlap_is_not_contact(self):
        d=self.decide(self.origin,1,uv=self.goal);self.assertFalse(d['landing_evidence'])
    def test_zero_gap_outside_pad_not_contact(self):
        with self.assertRaisesRegex(ValueError,'incomplete'):
            self.decide(np.array([600.,10,150]),1,uv=self.goal)
    def test_alignment_precedes_descent(self):
        tcp=self.origin+[-5,0,0];d=self.decide(tcp,1)
        self.assertEqual(d['phase'],'ALIGN');self.assertEqual(d['target'][2],tcp[2])
    def test_bounded_descent(self):
        d=self.decide(self.origin,1);self.assertEqual(d['phase'],'DESCEND')
        self.assertGreaterEqual(d['target'][2],self.origin[2]-.5)
    def test_success_only_after_pause_and_stable_duration(self):
        tcp=np.array([600.,0,150.2]);self.assertEqual(self.decide(tcp,1)['phase'],'PAUSE_VERIFY')
        for i in range(10):self.assertEqual(self.decide(tcp,1.1+i*.09,True)['phase'],'VERIFY')
        d=self.decide(tcp,2.2,True);self.assertEqual(d['phase'],'LANDED_VISUAL');self.assertFalse(d['physical_contact_confirmed'])
    def test_repeated_frames_not_stable_samples(self):
        tcp=np.array([600.,0,150.2]);self.decide(tcp,1,True)
        with self.assertRaisesRegex(ValueError,'repeated'):self.decide(tcp,1,True)
    def test_book_movement_or_loss_cannot_continue(self):
        for changed,pattern in [({'top_base_mm':self.top+[3,0,0]},'Book moved'),({'tracking_valid':False},'tracking')]:
            self.setUp();o=self.obs(self.origin,1);o.update(changed)
            with self.assertRaisesRegex(ValueError,pattern):self.ctrl.decide(o,self.origin,np.eye(3),self.origin,np.eye(3))
    def test_negative_gap_stops(self):
        with self.assertRaisesRegex(ValueError,'penetration'):self.decide(np.array([600.,0,149]),1)
    def test_inconsistent_modalities_hold(self):
        with self.assertRaisesRegex(ValueError,'incomplete'):self.decide(np.array([603.,0,150.2]),1,uv=self.goal+self.j[:,2]*.2)
    def test_ideal_continuous_closed_loop(self):
        tcp=self.origin+[-4,1,0];done=False
        for i in range(1000):
            d=self.decide(tcp,1+i*.1)
            if d['phase']=='PAUSE_VERIFY':done=True;break
            tcp+=.7*(d['target']-tcp)
        self.assertTrue(done);self.assertLess(d['gap_mm'],1)
    def test_image_pose_latency_compensation(self):
        p=self.origin+[-4,0,0];q=p+[1,0,0]
        d=self.ctrl.decide(self.obs(p,1),p,np.eye(3),q,np.eye(3))
        self.assertGreater(d['target'][0],q[0]);self.assertLessEqual(np.linalg.norm(d['target']-q),2)

class SemanticTests(unittest.TestCase):
    def test_head_waits_for_post_hold_frame_without_tracking_cached_frame(self):
        from book_top_landing_vision import LandingHeadVision
        from unittest.mock import Mock
        cam=Mock();stamps=iter([9.9,10.1]);rgb=np.zeros((40,40,3),np.uint8)
        def capture():cam.captured_unix_s=next(stamps);return rgb,np.ones((40,40))
        cam.capture_image.side_effect=capture
        tracker=Mock();tracker.points=[1]*20
        v=SimpleNamespace(camera=cam,c={'frame_max_age_seconds':1.},last_stamp=0.,tracker=tracker,
                          geometry_scope='full_spine',geometry_options={})
        with patch('book_top_landing_vision.time.time',return_value=10.2),patch('book_top_landing_vision.time.sleep'),patch('book_top_landing_vision.measure_face',return_value={}):
            result=LandingHeadVision.observe(v,after_stamp=10.)
        self.assertEqual(result['captured_unix_s'],10.1);tracker.update.assert_called_once()
    def test_head_frozen_stream_still_times_out(self):
        from book_top_landing_vision import LandingHeadVision
        from unittest.mock import Mock
        cam=Mock(captured_unix_s=9.9);cam.capture_image.return_value=(None,None)
        v=SimpleNamespace(camera=cam,c={'frame_max_age_seconds':1.},last_stamp=0.)
        with patch('book_top_landing_vision.time.time',return_value=10.2),patch('book_top_landing_vision.time.monotonic',side_effect=[0.,1.1]):
            with self.assertRaisesRegex(ValueError,'no fresh'):LandingHeadVision.observe(v,after_stamp=10.)
    def response(self):
        return dict(scene_id='s',status='selected',confidence=.9,top_visible=True,full_spine_visible=True,same_book=True,
            same_finger=True,pad_underside_visible=True,book_quad=[[300,300],[500,300],[500,800],[300,800]],
            contact_uv=[400,300],pad_quad=[[300,100],[600,100],[600,200],[300,200]])
    def selector(self,view):return LandingSelector('target',view,config=dict(min_confidence=.7))
    def test_wrist_partial_spine_allowed(self):
        r=self.response();r['full_spine_visible']=False;self.selector('wrist').decode(r,'s',(720,1280,3),True)
        with self.assertRaisesRegex(ValueError,'spine'):self.selector('head').decode(r,'s',(720,1280,3),True)
    def test_scoped_head_top_patch_without_invented_bottom(self):
        r=self.response();r['full_spine_visible']=False
        s=LandingSelector('top','head',require_pad=False,book_only=True,upper_edge_only=True,config=dict(min_confidence=.7))
        result=s.decode(r,'s',(720,1280,3),False)
        self.assertEqual(result['geometry_scope'],'upper_edge_patch')
        self.assertIsNone(result['pad_quad'])
        r['top_visible']=False
        with self.assertRaisesRegex(ValueError,'spine'):s.decode(r,'s',(720,1280,3),False)
    def test_partial_patch_does_not_report_full_spine_center(self):
        from book_top_landing import head_geometry
        obs=dict(geometry_scope='upper_edge_patch',corners_base_mm=[[500,-20,100],[500,20,100],[500,20,80],[500,-20,80]])
        result=head_geometry(obs)
        self.assertNotIn('spine_center_base_mm',result)
        np.testing.assert_allclose(result['top_base_mm'],[500,0,100])
        np.testing.assert_allclose(result['visible_patch_center_base_mm'],[500,0,90])
    def test_cropped_patch_insets_only_artificial_bottom(self):
        from book_top_landing_vision import inset_visible_patch
        from book_top_tilt_perception import validate_quad
        q=np.array([[690,765],[715,760],[713,799],[688,799]],float)
        adjusted=inset_visible_patch(q,(800,1280,3))
        np.testing.assert_allclose(adjusted[:2],q[:2])
        self.assertLessEqual(max(adjusted[2:,1]),797.)
        validate_quad(adjusted,(800,1280,3))
    def test_occluded_underside_not_invented(self):
        r=self.response();r['pad_underside_visible']=False
        with self.assertRaisesRegex(ValueError,'underside'):self.selector('head').decode(r,'s',(720,1280,3),False)
    def test_wrong_book_not_reselected(self):
        r=self.response();r['same_book']=False
        with self.assertRaisesRegex(ValueError,'same book'):self.selector('wrist').decode(r,'s',(720,1280,3),True)

class RoutingTests(unittest.TestCase):
    def test_coarse_forbidden_after_visual_no_fault_reset(self):
        root=Path(__file__).resolve().parent.parent
        sys.path[:0]=[str(root/'dual_v2_2_control/cuarm_upper_planner'),str(root/'dual_v2_2_configuration/robot_platform_utils/python/scripts')]
        from dual_v2_2_upper_servo_v4_1 import install_landing_routing
        calls=[];g=SimpleNamespace(fault=None);g.trip=lambda reason:setattr(g,'fault',reason)
        p=SimpleNamespace(_first_target_command_received=True,target_description='book_v4_1_coarse',_v31_bridge=SimpleNamespace(guard=g),
            compute=lambda:calls.append('compute'),curobo_cmd={},curobo_state={},_hold_current_arm_command=lambda:calls.append('hold'))
        install_landing_routing(p);p.compute();self.assertEqual(calls,['compute'])
        p.target_description='book_v3_1_stream';p.compute();p.target_description='book_v4_1_coarse';p.compute()
        self.assertIsNotNone(g.fault);self.assertEqual(calls[-1],'hold');p.compute();self.assertEqual(calls.count('compute'),2)
    def test_auto_requires_runtime_measurement_not_manual_flag(self):
        c=load_config(execute=True);self.assertEqual(c['pad_source'],'auto_rgbd');self.assertIsNone(c['pad']['center_tcp_mm'])
        with patch('cart_book_grasp_v4_1.run') as run:
            self.assertEqual(main(['--execute']),0);self.assertTrue(run.call_args.args[0].execute)
        with patch('cart_book_grasp_v4_1.run',side_effect=AssertionError('I/O')):self.assertEqual(main(['--check-config']),0)

if __name__=='__main__':unittest.main()

class RuntimeIntegrationTests(unittest.TestCase):
    def test_full_landing_routes_coarse_before_visual_and_holds_after_verification(self):
        from contextlib import contextmanager
        import tempfile,json
        import book_top_landing_runtime as runmod
        c=config();calls=[];clock=[10.];q=[np.array([400.,0,150.])]
        top=np.array([600.,0,200.]);corners=np.array([[600,-15,200],[600,15,200],[600,15,0],[600,-15,0]],float)
        def state():return dict(right_arm=dict(position=q[0]/1000,orientation=[1,0,0,0]))
        @contextmanager
        def connection():yield object()
        class IO:
            def __init__(self,*args):self.rotation=np.eye(3);self.cancelled=False
            def guard(self):
                if self.cancelled:raise AssertionError('Guard after terminal cancel')
                return state()
            @contextmanager
            def perception_pause(self):
                calls.append('pause');yield self.guard();self.guard()
            def cancel(self):calls.append('hold');self.cancelled=True
            def close(self):calls.append('closed')
            def send(self,*args):calls.append('stream')
        head=SimpleNamespace(reference={'rgb':np.zeros((5,5,3),np.uint8)},save=lambda *a:None,
            stable=lambda *a,**kw:dict(top_base_mm=top,corners_base_mm=corners))
        wrist=SimpleNamespace(initialize=lambda *a:None,save=lambda *a:None)
        camera=SimpleNamespace(capture=lambda *a:None,close=lambda:None)
        def observe(*a):
            clock[0]+=.2
            return dict(top_base_mm=top,wrist_contact_uv=np.array([640.,359.9]),pad_contact_uv=np.array([640.,360.]),
                captured_unix_s=clock[0],tracking_valid=True,wrist_points=30),dict(q=q[0].copy(),angles=np.zeros(3)),state()
        def coarse(io,goal):calls.append('coarse');q[0]=np.asarray(goal).copy()
        def probes(*a):
            calls.append('probes');q[0]=np.array([600.,0,150.2]);obs,p,s=observe()
            return np.array([[2.,0,0],[0,2.,-.5]]),obs,p,s
        with tempfile.TemporaryDirectory() as td:
            args=SimpleNamespace(output=Path(td),execute=True,select=False,selection=None,auto_select=False,target=None,frames=2)
            with patch('cart_grasp_alignment.open_head_camera',return_value=camera),patch('cart_book_grasp_v3_camera.WristCamera',return_value=camera), \
                 patch('book_top_tilt_servo.servo_connection',connection),patch('book_top_tilt_servo.ContinuousRobotIO',IO), \
                 patch.object(runmod.LandingHeadVision,'automatic',return_value=head),patch.object(runmod,'LandingWristVision',return_value=wrist), \
                 patch.object(runmod,'LandingSelector'),patch.object(runmod,'coarse_move',side_effect=coarse), \
                 patch.object(runmod,'observe_pair',side_effect=observe),patch.object(runmod,'probe_jacobian',side_effect=probes), \
                 patch.object(runmod.time,'sleep'),patch('builtins.print'), \
                 patch('cart_book_grasp_v2_gpt.GPTBookClient._request',side_effect=AssertionError('No API')):
                runmod.run_landing(args,c)
            report=json.loads((Path(td)/'landing_result.json').read_text())
            self.assertEqual(report['event'],'landed_visual');self.assertFalse(report['physical_contact_confirmed'])
            self.assertLess(calls.index('coarse'),calls.index('probes'));self.assertEqual(calls[-2:],['hold','closed'])
            self.assertEqual(calls.count('hold'),1)
