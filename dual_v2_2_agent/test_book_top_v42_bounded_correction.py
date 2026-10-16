import unittest
from unittest.mock import patch
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from book_top_v42_bounded_correction import correction_at_roll_bound
import book_top_v42_servo as servo

class BoundTests(unittest.TestCase):
    def setUp(self):
        self.c=servo.load_config()['controller'];self.t=np.array([3.,1.5])
    def test_reported_19_66_degree_case_prefers_y_only(self):
        j=np.array([[3.48,-.03],[-.188,-1.107]])
        e=np.array([4.264,.807])
        v,mode=correction_at_roll_bound(np.array([-.6,.5]),j,e,19.664,self.t,10,self.c)
        self.assertEqual(mode,'y_only_roll_boundary');self.assertEqual(v[1],0)
        self.assertLess(abs((e+j@v)[0]),abs(e[0]));self.assertLessEqual(abs((e+j@v)[1]),1.5)
    def test_negative_boundary(self):
        v,mode=correction_at_roll_bound(np.array([.6,-.5]),np.diag([3.,1.]),np.array([-4.,-.8]),-19.8,self.t,10,self.c)
        self.assertEqual(mode,'y_only_roll_boundary');self.assertEqual(v[1],0);self.assertGreater(v[0],0)
    def test_joint_solution_recomputes_y_when_roll_saturates(self):
        j=np.array([[3.,2.],[0.,-1.]])
        v,mode=correction_at_roll_bound(np.array([-2.,3.]),j,np.array([8.,8.]),19.8,self.t,10,self.c)
        self.assertEqual(mode,'bounded_y_roll');self.assertLessEqual(19.8+v[1],20.)
        self.assertLessEqual(abs(v[0]),5.);self.assertLessEqual(abs(v[1]),3.)
        a=j/self.t[:,None];b=np.array([8.,8.])/self.t
        h=a.T@a+.02*np.eye(2);g=.5*a.T@b
        self.assertAlmostEqual(v[0],-(g[0]+h[0,1]*v[1])/h[0,0])
    def test_y_only_does_not_push_angle_out_of_tolerance(self):
        j=np.array([[3.,0.],[-3.,-1.]])
        e=np.array([6.,1.49]);v,mode=correction_at_roll_bound(np.array([-1.,2.]),j,e,19.,self.t,10,self.c)
        if mode=='y_only_roll_boundary':self.assertLessEqual(abs((e+j@v)[1]),1.5+1e-12)
        self.assertLessEqual(19+v[1],20)
    def test_ordinary_behavior_unchanged(self):
        preferred=np.array([1.,.2]);v,mode=correction_at_roll_bound(preferred,np.eye(2),np.array([4.,2.]),0.,self.t,10.,self.c)
        np.testing.assert_equal(v,preferred);self.assertEqual(mode,'ordinary')
    def test_randomized_commands_obey_total_and_step_bounds(self):
        rng=np.random.default_rng(1)
        for _ in range(200):
            r=rng.choice([-1,1])*rng.uniform(17,20.8)
            j=np.array([[3.,rng.uniform(-2,2)],[rng.uniform(-.3,.3),-1.]])
            e=rng.uniform(-10,10,2)
            v,_=correction_at_roll_bound(np.array([5.,3.]),j,e,r,self.t,8,self.c)
            self.assertLessEqual(abs(r+v[1]),20+1e-12)
            self.assertLessEqual(abs(v[0]),5+1e-12);self.assertLessEqual(abs(v[1]),3+1e-12)

class ControllerTests(unittest.TestCase):
    def test_initially_tilted_spine_converges_without_exceeding_20(self):
        for sign in (1,-1):
            q=np.array([410.,-100.,0.]);r=0.;now=[100.];commands=[];events=[]
            def observe():
                now[0]+=.02
                f=np.array([[3.,1.],[0.,-1.]])@np.array([q[1]+100,r-sign*20.5])
                return dict(xyz_mm=q.copy(),roll_deg=r,uv=[639.5+f[0],359.5],angle_deg=f[1],scale=1.,captured_at=now[0])
            def move(target,roll):
                nonlocal q,r
                q=target.copy();r=roll;commands.append((q.copy(),r));return True
            result=servo.run_raise(observe,move,639.5,20,servo.load_config(),path_available=500,clock=lambda:now[0],log=events.append)
            self.assertTrue(result['converged']);self.assertTrue(all(abs(r)<=20 for q,r in commands))
            self.assertTrue(any(e['event']=='roll_bounded_correction' for e in events))
    def test_no_useful_bounded_step_requests_gpt_before_new_command(self):
        q=np.array([410.,-100.,0.]);r=18.;now=[100.];commands=[];reviews=[]
        def observe():
            now[0]+=.02
            return dict(xyz_mm=q.copy(),roll_deg=r,uv=[650+3*(q[1]+100),359.5],angle_deg=0.,scale=1.,captured_at=now[0])
        def move(target,roll):
            nonlocal q,r
            q=target.copy();r=roll;commands.append(1);return True
        def review(reason):reviews.append(reason);raise ServoError('test GPT failure')
        # Supply a valid calibrated response; zero bounded candidate cannot
        # improve a non-aligned feature, so it must trigger review, not motion.
        # Patch only optimizer candidate, keep probes and real controller gates.
        # Angle response must remain observable for calibration.
        original=observe
        def observed():
            o=original();o['angle_deg']=-(r-18);return o
        with patch('book_top_v42_control.correction_at_roll_bound',return_value=(np.zeros(2),'bounded_y_roll')):
            with self.assertRaisesRegex(ServoError,'test GPT failure'):
                servo.run_raise(observed,move,639.5,20,servo.load_config(),path_available=500,clock=lambda:now[0],recover=review)
        self.assertEqual(len(commands),2);self.assertEqual(len(reviews),1)
    def test_four_unproductive_bounded_steps_review_instead_of_repeating(self):
        q=np.array([410.,-100.,0.]);r=18.;now=[100.];commands=[];reviews=[]
        def observe():
            now[0]+=.02
            error=np.array([10.+3*(q[1]+100),-(r-18)]) if len(commands)<=2 else np.array([25.,-2.])
            return dict(xyz_mm=q.copy(),roll_deg=r,uv=[639.5+error[0],359.5],angle_deg=error[1],scale=1.,captured_at=now[0])
        def move(target,roll):
            nonlocal q,r
            q=target.copy();r=roll;commands.append(1);return True
        def review(reason):reviews.append(reason);raise ServoError('test bounded stall review')
        with patch('book_top_v42_control.correction_at_roll_bound',return_value=(np.array([-.1,0.]),'bounded_y_roll')):
            with self.assertRaisesRegex(ServoError,'test bounded stall review'):
                servo.run_raise(observe,move,639.5,20,servo.load_config(),path_available=500,clock=lambda:now[0],recover=review)
        self.assertEqual(len(commands),6);self.assertEqual(len(reviews),1)
