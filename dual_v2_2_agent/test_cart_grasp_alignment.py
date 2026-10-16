import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import cv2
import numpy as np
from cart_grasp import HandleMarker,dictionary,main
from cart_grasp_alignment import (load_config,marker_error,head_point,coarse_target,
    coarse_move,align_loop,MarkerLost)


class FakeLoop:
    def __init__(self):
        self.now=0.;self.q=np.array([500.,0.,-10.]);self.roll=0.
        self.sent=[];self.holds=0;self.observations=0;self.events=[]
        self.jac=np.array([[3.,-.5],[-.1,-1.]])
        self.lost=set();self.bad_z=False
    def read(self):
        return dict(q=self.q.copy(),roll=self.roll,pose_sample_monotonic=self.now)
    def observe(self,guard):
        self.now+=.11;guard();self.observations+=1
        if self.observations in self.lost:raise MarkerLost('target_marker_missing')
        if self.bad_z:self.q[2]+=4.;guard()
        e=np.array([20.,4.])+self.jac@np.array([self.q[1],self.roll])
        return e,self.read()
    def send(self,q,roll,blocking):
        self.sent.append((q.copy(),roll,blocking));self.q=q.copy();self.roll=roll
    def hold(self):self.holds+=1
    def run(self,c=None,**kwargs):
        return align_loop(self.read,self.observe,self.send,lambda:True,self.hold,
            np.array([500.,0.,-10.]),c or load_config(),self.events.append,lambda:self.now,**kwargs)


class AlignmentTests(unittest.TestCase):
    def image(self):
        bgr=np.full((720,1280,3),255,np.uint8)
        bgr[240:480,520:760]=cv2.cvtColor(cv2.aruco.generateImageMarker(dictionary(),0,240),cv2.COLOR_GRAY2BGR)
        return bgr
    def test_marker_horizontal_and_angle_ignore_height(self):
        d=HandleMarker();im=self.image()
        a=marker_error(d.detect(im))
        shifted=cv2.warpAffine(im,np.float32([[1,0,20],[0,1,50]]),(1280,720),borderValue=(255,255,255))
        b=marker_error(d.detect(shifted))
        np.testing.assert_allclose(b-a,[20,0],atol=.1)
        result=d.detect(im);result['tracking_valid']=False
        with self.assertRaises(MarkerLost):marker_error(result)
    def test_tilted_marker_projective_center_axis(self):
        im=self.image();H=cv2.getRotationMatrix2D((640,360),10,1)
        transformed=cv2.warpAffine(im,H,(1280,720),borderValue=(255,255,255))
        self.assertAlmostEqual(marker_error(HandleMarker().detect(transformed))[1],10,delta=.2)
    def test_head_mm_transform_and_missing_depth(self):
        r=HandleMarker().detect(self.image());depth=np.full((720,1280),600.,np.float32)
        k=np.array([[600.,0,639.5],[0,600.,359.5],[0,0,1]])
        transform=np.eye(4);transform[:3,3]=[100,200,300]
        np.testing.assert_allclose(head_point(r,depth,k,np.zeros(5),transform),[100,200,900],atol=.1)
        depth[:]=0
        with self.assertRaisesRegex(ValueError,'depth'):head_point(r,depth,k,np.zeros(5),transform)
    def test_coarse_only_y_and_guard(self):
        np.testing.assert_array_equal(coarse_target([400,0,20],[700,-100,90],load_config()),[400,-100,20])
        with self.assertRaisesRegex(ValueError,'bound'):coarse_target([400,0,20],[700,-250,90],load_config())
    def test_coarse_uses_curobo_and_strict_arrival_no_gripper(self):
        original=Mock(return_value=True);robot=SimpleNamespace(_wait_for_robot_stop=original)
        calls=[]
        def master(**kwargs):
            calls.append(kwargs);return robot._wait_for_robot_stop(position_tolerance_m=.030)
        robot.master_control=master
        coarse_move(robot,[500,2,10],[0,0,0])
        self.assertIs(robot._wait_for_robot_stop,original)
        self.assertEqual(calls[0]['action'],'cart_handle_coarse')
        self.assertNotIn('right_gripper',calls[0]);self.assertNotIn('left_arm',calls[0])
        self.assertEqual(original.call_args.kwargs['position_tolerance_m'],.003)
        self.assertTrue(original.call_args.kwargs['require_fresh_stable'])
        original.return_value=False
        with self.assertRaises(ValueError):coarse_move(robot,[500,2,10],[0,0,0])
        self.assertIs(robot._wait_for_robot_stop,original)
    def test_continuous_convergence_and_fixed_axes(self):
        f=FakeLoop();result=f.run()
        self.assertEqual(result['event'],'cart_aligned')
        self.assertLess(abs(result['error_px_deg'][0]),3)
        self.assertLess(abs(result['error_px_deg'][1]),1.5)
        self.assertEqual([b for q,r,b in f.sent[:2]],[True,True])
        self.assertTrue(all(not b for q,r,b in f.sent[2:]))
        for q,r,b in f.sent:np.testing.assert_array_equal(q[[0,2]],[500,-10])
        self.assertGreater(f.holds,0)
    def test_loss_holds_and_reacquires_same_id(self):
        f=FakeLoop();f.lost={4,5,6};f.run()
        self.assertEqual(sum(e['event']=='marker_lost_hold' for e in f.events),1)
        self.assertTrue(any(e['event']=='marker_reacquired' for e in f.events))
    def test_loss_timeout_never_blind_forward(self):
        f=FakeLoop();f.lost=set(range(4,100))
        with self.assertRaisesRegex(ValueError,'reacquisition'):f.run()
        self.assertEqual(len(f.sent),2);self.assertGreater(f.holds,0)
    def test_fixed_z_error_stops_without_target(self):
        f=FakeLoop();f.bad_z=True
        with self.assertRaisesRegex(ValueError,'fixed X/Z'):f.run()
        self.assertEqual(len(f.sent),0);self.assertGreater(f.holds,0)
    def test_bounds_not_bypassed(self):
        c=load_config();c['max_servo_y_mm']=5;f=FakeLoop()
        with self.assertRaisesRegex(ValueError,'bounds|bound'):f.run(c)
        for q,r,b in f.sent:self.assertLessEqual(abs(q[1]),5)
    def test_head_depth_goal_uses_fingertip_and_standoff(self):
        from cart_grasp_depth import approach_goal,load_approach
        c=load_approach()
        result=approach_goal(dict(marker_id=0,point_base_mm=[700,20,10]),[500,0,10],[0,-90,0],c)
        self.assertAlmostEqual(result['goal_tcp_x_mm'],630.)
        self.assertAlmostEqual(result['remaining_x_mm'],130.)
        with self.assertRaisesRegex(ValueError,'face'):
            approach_goal(dict(marker_id=0,point_base_mm=[700,20,10]),[500,0,10],[0,0,0],c)
        with self.assertRaisesRegex(ValueError,'behind'):
            approach_goal(dict(marker_id=0,point_base_mm=[500,20,10]),[500,0,10],[0,-90,0],c)
        with self.assertRaisesRegex(ValueError,'bound'):
            approach_goal(dict(marker_id=0,point_base_mm=[1000,20,10]),[500,0,10],[0,-90,0],c)
    def test_forward_with_visual_correction_and_fixed_z(self):
        from cart_grasp_depth import load_approach
        f=FakeLoop();result=f.run(goal_x=560.,approach=load_approach())
        self.assertGreaterEqual(result['tcp_mm'][0],557.)
        self.assertLessEqual(result['tcp_mm'][0],560.)
        self.assertTrue(all(q[2]==-10 for q,r,b in f.sent))
        self.assertTrue(all(f.sent[i][0][0]>=f.sent[i-1][0][0] for i in range(1,len(f.sent))))
        self.assertTrue(all(q[0]<=560 for q,r,b in f.sent))
        for event in f.events:
            if event['event']=='cart_depth_approach' and event['step_mm']>0:
                self.assertTrue(event['aligned'])
    def test_forward_loss_never_switches_to_blind_approach(self):
        from cart_grasp_depth import load_approach
        f=FakeLoop();f.lost=set(range(25,200))
        with self.assertRaisesRegex(ValueError,'reacquisition'):
            f.run(goal_x=650.,approach=load_approach())
        self.assertLess(f.q[0],647.)
        self.assertGreater(f.holds,0)
    def test_terminal_endpoint_is_latched_with_lag_and_boundary_noise(self):
        from cart_grasp_depth import load_approach
        class LagLoop(FakeLoop):
            target=None
            def send(self,q,roll,blocking):
                self.sent.append((q.copy(),roll,blocking))
                if blocking:self.q=q.copy();self.roll=roll
                else:self.target=q.copy();self.q[1:]=q[1:];self.roll=roll
            def observe(self,guard):
                if self.target is not None:
                    self.q[0]+=.18*(self.target[0]-self.q[0])
                return super().observe(guard)
        f=LagLoop();result=f.run(goal_x=550.,approach=load_approach())
        terminal=[e for e in f.events if e['event']=='cart_terminal_target']
        self.assertEqual(len(terminal),1)
        self.assertEqual(terminal[0]['tcp_mm'][0],550.)
        self.assertLess(abs(result['tcp_mm'][0]-550.),3.)
        self.assertEqual(f.holds,1)  # only the final hold, no boundary chatter
        self.assertEqual(sum(b for q,r,b in f.sent),2)  # probes only
        self.assertGreater(sum(q[0]==550 for q,r,b in f.sent),5)
        for a,b in zip(f.sent,f.sent[1:]):self.assertLessEqual(b[0][0]-a[0][0],6.0001)
    def test_settling_more_than_five_seconds_keeps_stream_target(self):
        class SlowSettle(FakeLoop):
            def observe(self,guard):
                e,p=super().observe(guard)
                # Subpixel motion keeps the measured pose unsettled temporarily.
                if self.now<8 and self.q[1]<-5:
                    self.q[1]+=.31 if self.observations%2 else -.31
                    e=np.array([20.,4.])+self.jac@np.array([self.q[1],self.roll])
                    p=self.read()
                return e,p
        f=SlowSettle();f.run()
        self.assertGreater(f.now,8.)
        self.assertEqual(f.holds,1)
        self.assertGreater(len(f.sent),30)
        self.assertTrue(all(not b for q,r,b in f.sent[2:]))
    def test_cleanup_failure_does_not_hide_primary_fault(self):
        f=FakeLoop();f.bad_z=True
        f.hold=Mock(side_effect=ValueError('secondary hold fault'))
        with self.assertRaisesRegex(ValueError,'fixed X/Z'):f.run()
        self.assertEqual(f.events[-1]['event'],'cart_cleanup_hold_failed')
    def test_terminal_marker_loss_cancels_endpoint_updates(self):
        from cart_grasp_depth import load_approach
        class TerminalLoss(FakeLoop):
            terminal_sent=False
            def send(self,q,roll,blocking):
                super().send(q,roll,blocking)
                if q[0]==510:self.terminal_sent=True
            def observe(self,guard):
                if self.terminal_sent:
                    self.now+=.11;guard();raise MarkerLost('terminal occlusion')
                return super().observe(guard)
        f=TerminalLoss()
        with self.assertRaisesRegex(ValueError,'reacquisition'):
            f.run(goal_x=510.,approach=load_approach())
        self.assertEqual(sum(q[0]==510 for q,r,b in f.sent),1)
        self.assertFalse(any(e['event']=='cart_aligned' for e in f.events))
    def test_approach_requires_endpoint_and_bound(self):
        from cart_grasp_depth import load_approach
        for kwargs in (dict(goal_x=550),dict(approach=load_approach()),dict(goal_x=900,approach=load_approach())):
            f=FakeLoop()
            with self.assertRaises(ValueError):f.run(**kwargs)
            self.assertEqual(f.sent,[])
    def test_forward_ramp_resets_on_misalignment(self):
        from cart_grasp_depth import ForwardStep,load_approach
        f=ForwardStep(load_approach())
        self.assertEqual(f.update(0,100,True),0)
        self.assertGreater(f.update(.2,100,True),0)
        self.assertEqual(f.update(.3,100,False),0)
        self.assertLess(f.update(.31,100,True),1.)
    def test_missing_target_reports_other_decoded_ids(self):
        result=HandleMarker(1).detect(self.image())
        self.assertFalse(result['tracking_valid'])
        self.assertEqual(result['detected_ids'],[0])
        self.assertIsInstance(result['rejected_candidate_count'],int)
    def test_config_cli_is_offline(self):
        with patch('cart_grasp_alignment.run',side_effect=AssertionError('Hardware opened')):
            self.assertEqual(main(['align','--check-config']),0)

if __name__=='__main__':unittest.main()
