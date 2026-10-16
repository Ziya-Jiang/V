"""Stationary-before-capture regression tests using measured-feedback traces."""
import unittest
from unittest.mock import Mock
from types import SimpleNamespace
import numpy as np
from cart_book_grasp_v3_gpt_recovery import wait_stationary
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_4_wrist_gpt import recover

class StationaryTests(unittest.TestCase):
    def setUp(self):self.t=10.;self.record={}
    def sleep(self,dt):self.t+=dt
    def state(self,x=0.,angle=0.,stamp=None):
        return dict(q=np.array([x,0.,-10.]),angles=np.array([0.,0.,angle]),
                    pose_sample_monotonic=self.t if stamp is None else stamp)
    def wait(self,read,deadline=20.):
        return wait_stationary(read,lambda:None,self.record,deadline,-10.,4.,clock=lambda:self.t,sleep=self.sleep)
    def test_arrival_residual_converges_before_anchor_is_latched(self):
        def read():return self.state(x=max(0.,2.-4*(self.t-10.)))
        out=self.wait(read)
        self.assertLess(out['q'][0],.25)
        self.assertGreaterEqual(self.t,10.7)
        self.assertGreaterEqual(self.record['stationary_hold']['stable_seconds'],.3)
    def test_stationary_arm_only_waits_stability_window(self):
        self.wait(self.state);self.assertLess(self.t,10.35)
    def test_repeated_feedback_packets_do_not_count_as_stable(self):
        with self.assertRaisesRegex(ServoError,'fresh measured feedback'):
            self.wait(lambda:self.state(stamp=10.))
    def test_continuous_motion_times_out_without_capture(self):
        with self.assertRaisesRegex(ServoError,'stationary hold timed out'):
            self.wait(lambda:self.state(x=(self.t-10.)*5))
        self.assertLess(self.t,13.05)
    def test_rotation_must_settle_too(self):
        with self.assertRaisesRegex(ServoError,'stationary hold timed out'):
            self.wait(lambda:self.state(angle=(self.t-10.)*3))
    def test_height_guard_remains_active_while_settling(self):
        state=self.state();state['q'][2]=-15.
        with self.assertRaisesRegex(ServoError,'Height drift'):self.wait(lambda:state)
    def test_total_budget_remains_in_effect(self):
        with self.assertRaises(ServoError):self.wait(self.state,deadline=10.15)
    def test_motion_guard_during_gpt_is_not_relabelled_detection_failure(self):
        error=ServoError('Arm moved during GPT tracking recovery')
        client=SimpleNamespace(select=Mock(side_effect=error));diagnostics={}
        config=dict(enabled=True,model='gpt-6-astra',timeout_seconds=1,min_confidence=.7)
        with self.assertRaises(ServoError) as caught:
            recover(np.zeros((100,100,3),np.uint8),diagnostics,detector=None,client=client,config=config)
        self.assertIs(caught.exception,error)
        self.assertEqual(diagnostics['status'],'interrupted')

if __name__=='__main__':unittest.main()
