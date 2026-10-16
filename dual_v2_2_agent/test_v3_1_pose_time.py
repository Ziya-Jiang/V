"""Timestamp pairing and moving-capture integration without hardware."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_1_pose_time import PoseTimeline,PoseNotBracketed,PoseSupportUnavailable
from cart_book_grasp_v3_1_stream import load_stream_config
from cart_book_grasp_v3_servo import ServoError


def sample(t,x,angle=0.):
    return dict(received_monotonic=t,pose_sample_monotonic=t,q=np.array([x,0.,0.]),angles=np.array([0.,0.,angle]))


class PairTests(unittest.TestCase):
    def recovery_fixture(self,resume=1.45,fault=None):
        from cart_book_grasp_v3_1_runtime import StreamTransport
        from cart_book_grasp_v3_column import load_config
        t=[1.];last=[.999];captures=[];events=[]
        def state():
            if t[0]<1.02 or t[0]>=resume:last[0]=t[0]-.001
            return dict(right_arm=dict(position=[.5,0.,0.],orientation=[1.,0.,0.,0.]),
                        received_monotonic=t[0],pose_sample_monotonic=last[0],pose_fk_completed_monotonic=last[0],
                        curobo_status=3 if fault and t[0]>=fault else 9,robot_status=0)
        robot=SimpleNamespace(get_current_state=state,master_control=Mock())
        transport=StreamTransport(robot,np.zeros(3),load_config(),load_stream_config(),clock=lambda:t[0])
        def capture(check):
            captures.append(t[0]);t[0]+=.02;check()
            return dict(rgb=np.zeros((80,100,3),np.uint8),captured_at=t[0]-.005)
        kwargs=dict(clock=lambda:t[0],sleep=lambda dt:t.__setitem__(0,t[0]+dt),total_deadline=10.)
        return transport,SimpleNamespace(capture=capture),events,captures,kwargs,robot,t

    def test_pair_outage_waits_for_new_feedback_and_recaptures_without_commands(self):
        from cart_book_grasp_v3_1_runtime import capture_paired_frame
        tr,camera,events,captures,kwargs,robot,t=self.recovery_fixture()
        frame,paired,*_=capture_paired_frame(tr,camera,lambda:None,events.append,-np.inf,**kwargs)
        self.assertEqual(len(captures),2)
        self.assertGreaterEqual(captures[1],1.45)
        self.assertGreater(frame['captured_at'],1.45)
        self.assertLessEqual(paired['pair_gap_seconds'],.1)
        self.assertEqual([e['event'] for e in events],['pose_pair_failed','pose_pair_recovery_start','pose_pair_recovered'])
        robot.master_control.assert_not_called()

    def test_persistent_outage_times_out_without_recapture_or_commands(self):
        from cart_book_grasp_v3_1_runtime import capture_paired_frame
        tr,camera,events,captures,kwargs,robot,t=self.recovery_fixture(resume=100.)
        with self.assertRaisesRegex(ServoError,'pairing recovery timed out'):
            capture_paired_frame(tr,camera,lambda:None,events.append,-np.inf,**kwargs)
        self.assertEqual(len(captures),1);robot.master_control.assert_not_called()
        self.assertLess(t[0],3.2)
        self.assertEqual(events[-1]['event'],'pose_pair_recovery_timeout')

    def test_planner_fault_during_recovery_is_not_swallowed(self):
        from cart_book_grasp_v3_1_runtime import capture_paired_frame
        tr,camera,events,captures,kwargs,robot,t=self.recovery_fixture(resume=100.,fault=1.3)
        with self.assertRaisesRegex(ServoError,'planner/IK/collision failure'):
            capture_paired_frame(tr,camera,lambda:None,events.append,-np.inf,**kwargs)
        self.assertLess(t[0],1.31);robot.master_control.assert_not_called()

    def test_repeated_failed_frames_do_not_reset_recovery_deadline(self):
        from cart_book_grasp_v3_1_runtime import capture_paired_frame
        tr,camera,events,captures,kwargs,robot,t=self.recovery_fixture(resume=0.)
        tr.pair_frame=Mock(side_effect=PoseNotBracketed('missing support'))
        with self.assertRaisesRegex(ServoError,'pairing recovery timed out'):
            capture_paired_frame(tr,camera,lambda:None,events.append,-np.inf,**kwargs)
        self.assertGreater(len(captures),2);self.assertLess(t[0],3.06)
        self.assertEqual(sum(e['event']=='pose_pair_recovery_start' for e in events),1)
        robot.master_control.assert_not_called()

    def test_pair_precision_and_frame_freshness_are_not_relaxed_on_retry(self):
        from cart_book_grasp_v3_1_runtime import capture_paired_frame
        for failure in ('wide','stale'):
            tr,camera,events,captures,kwargs,robot,t=self.recovery_fixture()
            original=tr.pair_frame
            def pair(stamp,*args):
                if t[0]<1.45:return original(stamp,*args)
                if failure=='wide':raise ServoError('V3.1 pose pair support too wide')
                result=original(stamp,*args);t[0]+=.26;return result
            tr.pair_frame=pair
            expected='support too wide' if failure=='wide' else 'frame age exceeded'
            with self.assertRaisesRegex(ServoError,expected):
                capture_paired_frame(tr,camera,lambda:None,events.append,-np.inf,**kwargs)
            robot.master_control.assert_not_called()

    def test_rejected_wide_support_is_recaptured_with_unchanged_limits(self):
        from cart_book_grasp_v3_1_runtime import capture_paired_frame
        tr,camera,events,captures,kwargs,robot,t=self.recovery_fixture(resume=0.)
        original=tr.pair_frame
        def pair(stamp,*args):
            if len(captures)==1:
                h=PoseTimeline();h.add(sample(stamp-.0107,0.));h.add(sample(stamp+.0107,12.542))
                return h.pair(stamp,tr.s)
            return original(stamp,*args)
        tr.pair_frame=pair
        frame,paired,*_=capture_paired_frame(tr,camera,lambda:None,events.append,-np.inf,**kwargs)
        self.assertEqual(len(captures),2);self.assertEqual(tr.s['max_pose_pair_uncertainty_mm'],3.)
        self.assertLessEqual(paired['pair_motion_bound_mm'],3.)
        self.assertAlmostEqual(events[0]['support']['pair_motion_bound_mm'],6.271,places=5)
        self.assertEqual(events[-1]['event'],'pose_pair_recovered')
        robot.master_control.assert_not_called()

    def test_persistent_wide_support_exits_at_original_recovery_deadline(self):
        from cart_book_grasp_v3_1_runtime import capture_paired_frame
        tr,camera,events,captures,kwargs,robot,t=self.recovery_fixture(resume=0.)
        tr.pair_frame=Mock(side_effect=PoseSupportUnavailable('wide support',pair_motion_bound_mm=6.271))
        with self.assertRaisesRegex(ServoError,'pairing recovery timed out'):
            capture_paired_frame(tr,camera,lambda:None,events.append,-np.inf,**kwargs)
        self.assertLess(t[0],3.06);self.assertGreater(len(captures),2)
        self.assertEqual(sum(e['event']=='pose_pair_recovery_start' for e in events),1)
        robot.master_control.assert_not_called()

    def test_operator_cancel_and_total_budget_remain_active_during_recovery(self):
        from cart_book_grasp_v3_1_runtime import capture_paired_frame
        for cancel in (False,True):
            tr,camera,events,captures,kwargs,robot,t=self.recovery_fixture(resume=100.)
            if not cancel:kwargs['total_deadline']=1.3
            def checkpoint():
                if cancel and t[0]>=1.3:raise KeyboardInterrupt()
            with self.assertRaises(KeyboardInterrupt if cancel else ServoError):
                capture_paired_frame(tr,camera,checkpoint,events.append,-np.inf,**kwargs)
            self.assertLess(t[0],1.31);robot.master_control.assert_not_called()

    def test_interpolates_position_and_short_quaternion_arc(self):
        h=PoseTimeline();h.add(sample(1.,0.,179.));h.add(sample(1.04,2.,-179.))
        s=load_stream_config();s['max_pose_pair_uncertainty_deg']=1.01
        p=h.pair(1.02,s)
        self.assertAlmostEqual(p['q'][0],1.)
        self.assertAlmostEqual(abs(p['angles'][2]),180.)
        self.assertAlmostEqual(p['pair_gap_seconds'],.04)

    def test_total_wait_motion_is_not_pair_uncertainty(self):
        h=PoseTimeline()
        for i in range(11):h.add(sample(1.+i*.02,i))
        p=h.pair(1.19,load_stream_config())
        self.assertAlmostEqual(p['q'][0],9.5)  # Total movement 10 mm; local support 0.5 mm.
        self.assertAlmostEqual(p['pair_motion_bound_mm'],.5)

    def test_never_extrapolates_or_duplicates_cached_feedback(self):
        h=PoseTimeline();h.add(sample(1.,0.));h.add(sample(1.,99.))
        self.assertEqual(len(h.samples),1)
        self.assertEqual(h.pair(1.,load_stream_config())['q'][0],0.)
        for t in (.99,1.01):
            with self.assertRaises(PoseNotBracketed):h.pair(t,load_stream_config())
        with self.assertRaises(ServoError):h.add(sample(.9,0.))

    def test_repeated_packets_do_not_turn_old_pose_into_short_support(self):
        h=PoseTimeline()
        for receipt in (10.,10.08,10.16,10.244):
            cached=sample(10.,0.);cached['received_monotonic']=receipt
            h.add(cached)
        h.add(sample(10.2654,12.542))
        self.assertEqual(len(h.samples),2)
        with self.assertRaises(PoseSupportUnavailable) as caught:
            h.pair(10.255,load_stream_config())
        self.assertAlmostEqual(caught.exception.details['pair_gap_seconds'],.2654)

    def test_gap_and_local_jump_still_fail(self):
        for dt,x,angle in ((.2,.1,0.),(.04,8.,0.),(.04,0.,4.)):
            h=PoseTimeline();h.add(sample(1.,0.));h.add(sample(1.+dt,x,angle))
            with self.assertRaises(ServoError):h.pair(1.+dt/2,load_stream_config())

    def integration(self,collect=True,late_loss=False):
        import cart_book_grasp_v3_1_runtime as runtime
        from cart_book_grasp_v3_column import load_config
        t=[1.];events=[];received=[]
        def state():
            stamp=t[0]-.01
            arm=dict(position=[(.5+20*(stamp-1)/1000),0.,0.],orientation=[1.,0.,0.,0.])
            return dict(right_arm=arm,received_monotonic=t[0],pose_sample_monotonic=stamp,
                        pose_fk_completed_monotonic=stamp,curobo_status=9,robot_status=0)
        robot=SimpleNamespace(get_current_state=state,operator_sleep=lambda dt:t.__setitem__(0,t[0]+dt))
        def capture(check):
            for _ in range(6):
                t[0]+=.04
                if collect:check()
            return dict(rgb=np.zeros((80,100,3),np.uint8),captured_at=t[0]-.03)
        tracker=SimpleNamespace(observe=lambda rgb:dict(uv=np.array([50.,40.]),angle_deg=0.,scale=1.,
            points=np.array([[50.,30.],[50.,40.]]),visible=np.ones(2,bool),line=np.array([[50.,0.],[50.,79.]]),inliers=8))
        if late_loss:
            from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted
            def delayed_loss(rgb):
                t[0]+=.3;raise FeaturePointsExhausted(5)
            tracker.observe=delayed_loss
        def controller(observe,*args,**kwargs):
            received.append(observe());return dict(converged=True,completion_mode='stream_visual')
        with tempfile.TemporaryDirectory() as folder,patch.object(runtime.time,'monotonic',side_effect=lambda:t[0]),patch.object(runtime,'run_stream',side_effect=controller):
            result=runtime.run_stream_session(robot,SimpleNamespace(capture=capture),tracker,np.array([50.,40.]),600.,
                np.zeros(3),load_config(),load_stream_config(),lambda *a:True,lambda:None,events.append,Path(folder),None,started=1.)
        return received[0],events

    def test_runtime_pairs_moving_capture_larger_than_old_three_mm_gate(self):
        o,events=self.integration()
        self.assertAlmostEqual(o['image_q'][0],504.2)
        self.assertAlmostEqual(o['q'][0],504.6)
        event=next(e for e in events if e['event']=='stream_observation')
        self.assertGreater(event['acquisition_motion_mm'],3.)
        self.assertLess(event['pair_motion_bound_mm'],1.)
        self.assertEqual(event['timestamp_basis'],'host_read_completion_panel_receipt')

    def test_runtime_rejects_missing_feedback_support(self):
        with self.assertRaisesRegex(ServoError,'pairing recovery timed out'):self.integration(collect=False)

    def test_delayed_point_loss_cannot_bypass_frame_freshness(self):
        with self.assertRaisesRegex(ServoError,'perception latency'):self.integration(late_loss=True)


if __name__=='__main__':unittest.main()
