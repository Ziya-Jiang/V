"""No API/device calls: GPT reanchor data freshness and controller integration."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_feature_centerline import CenterlineGeometryError
from book_top_v42_gpt_recovery import reanchor
from book_top_v42_tracking import TrackingStream
import test_book_top_v42 as baseline
import book_top_v42_servo as servo

class ReanchorTests(unittest.TestCase):
    def setup_case(self,*,same=True,drift=False,stale=False):
        now=[100.];selected=[False];captures=[]
        old=NS(reference_rgb=np.zeros((20,20,3),np.uint8),reference_mask=np.ones((20,20),bool),last_u=1,last_angle=0)
        def read():
            return dict(q=np.array([410.+(4 if drift and selected[0] else 0),-100.,20.]),
                angles=np.zeros(3),pose_sample_monotonic=now[0]-(1 if stale else 0))
        def capture(check):
            check();now[0]+=.01
            f=dict(rgb=np.zeros((20,20,3),np.uint8),captured_at=now[0]);captures.append(f);return f
        def select(rgb,record,**kw):
            now[0]+=15;selected[0]=True;kw['checkpoint']()
            return dict(identity_confirmed=same,mask=old.reference_mask,
                        centerline=dict(center_u=18,angle_deg=35,line=[[3,1],[18,19]]))
        candidate=NS(observe=Mock(return_value=dict(inliers=20,line=[[3,1],[18,19]])))
        factory=Mock(return_value=candidate)
        def settled(reader,check,*a,**kw):check();return reader()
        def run():
            with tempfile.TemporaryDirectory() as td,patch('book_top_v42_gpt_recovery.wait_stationary',side_effect=settled):
                result=reanchor(old,capture,select,read,lambda:None,Path(td),200,reason='drift',
                    clock=lambda:now[0],factory=factory)
                record=(Path(td)/'recovery.json').read_text()
                return result,record
        return run,captures,candidate,factory
    def test_new_gpt_line_wins_over_old_line_and_api_aged_image_not_used_for_control(self):
        run,captures,candidate,factory=self.setup_case()
        result,record=run();self.assertIs(result,candidate)
        self.assertEqual(factory.call_args.args[1]['centerline']['angle_deg'],35)
        self.assertGreater(captures[1]['captured_at']-captures[0]['captured_at'],15)
        candidate.observe.assert_called_once_with(captures[1]['rgb'])
        self.assertIn('recovered',record)
    def test_identity_still_required(self):
        run,_,candidate,_=self.setup_case(same=False)
        with self.assertRaisesRegex(ServoError,'same book'):run()
        candidate.observe.assert_not_called()
    def test_motion_during_api_does_not_issue_control_from_image(self):
        run,_,candidate,_=self.setup_case(drift=True)
        with self.assertRaisesRegex(ServoError,'Arm moved|arm moved'):run()
        candidate.observe.assert_not_called()
    def test_real_feedback_staleness_still_fails(self):
        run,captures,_,_=self.setup_case(stale=True)
        with self.assertRaisesRegex(ServoError,'stale feedback'):run()
        self.assertEqual(captures,[])

class ControllerRecoveryTests(unittest.TestCase):
    def check_recovery(self,geometry=False):
        observe,move,clock,commands,events=baseline.LiftTests().simulation()
        recovered=[False];requests=[]
        def read():
            o=observe()
            if len(commands)>=3 and not recovered[0]:
                if geometry:return dict(xyz_mm=o['xyz_mm'],roll_deg=o['roll_deg'],captured_at=o['captured_at'],geometry_failure=True,reason='drifted geometry')
                o['uv'][0]+=25
            return o
        def recover(reason):requests.append(reason);recovered[0]=True
        result=servo.run_raise(read,move,639.5,40,servo.load_config(),path_available=500,
            clock=lambda:clock[0],log=events.append,recover=recover)
        self.assertTrue(result['converged']);self.assertEqual(len(requests),1)
        self.assertEqual(sum(e['event']=='gpt_reference_reset' for e in events),1)
        self.assertGreaterEqual(sum(e['event']=='jacobian_y_roll' for e in events),2)
        paths=[e['path_mm'] for e in events if e['event']=='centerline_step']
        self.assertEqual(paths,sorted(paths));self.assertGreater(result['path_mm'],40)
    def test_bad_visual_response_reanchors_and_reidentifies_model(self):self.check_recovery()
    def test_geometry_marker_reanchors_at_observation_boundary(self):self.check_recovery(True)
    def test_api_failure_is_not_silently_converted_to_history(self):
        observe,move,clock,commands,_=baseline.LiftTests().simulation()
        def read():
            o=observe()
            if len(commands)>=3:return dict(o,geometry_failure=True,reason='drift')
            return o
        with self.assertRaisesRegex(ServoError,'GPT unavailable'):
            servo.run_raise(read,move,639.5,40,servo.load_config(),path_available=500,
                clock=lambda:clock[0],recover=Mock(side_effect=ServoError('GPT unavailable')))
        self.assertEqual(len(commands),3)

class GeometryStreamTests(unittest.TestCase):
    def test_geometry_failure_is_explicit_marker_not_old_pixels(self):
        import time
        def capture(check):
            time.sleep(.005);check();return dict(rgb=0,captured_at=time.monotonic())
        stream=TrackingStream(NS(capture=capture),NS(observe=Mock(side_effect=CenterlineGeometryError('drift'))),allow_geometry_recovery=True).start()
        try:
            _,o,_=stream.after(time.monotonic(),1)
            self.assertTrue(o['geometry_failure']);self.assertNotIn('uv',o)
            self.assertNotIn('point_loss',o)
        finally:stream.close()

class HistoricalBoundReviewTests(unittest.TestCase):
    def scenario(self,*,failure=False,persistent_loss=False,escape=False,path=500.):
        q=np.array([410.,-100.,0.]);now=[100.];reviewed=[False]
        commands=[];reviews=[];events=[]
        def observe():
            now[0]+=.02
            o=dict(xyz_mm=q.copy(),roll_deg=0.,captured_at=now[0])
            if (not reviewed[0] and q[2]>=30) or (reviewed[0] and persistent_loss):
                return dict(o,point_loss=True,tracking_valid=False)
            # After GPT reanchor, the new visual reference is aligned except
            # in the escape test, where a correction would cross original Y bound.
            error=-12 if reviewed[0] and escape else 0.
            return dict(o,uv=[639.5+error,359.5],angle_deg=0.,scale=1.)
        def move(target,roll):
            nonlocal q
            commands.append((target.copy(),roll));q=target.copy()
            if not reviewed[0]:q[1]=-100.+.1*q[2]
            return True
        def review(reason):
            reviews.append((reason,len(commands),q.copy()))
            if failure:raise ServoError('GPT unavailable')
            reviewed[0]=True
        c=servo.load_config();c['max_lateral_mm']=4.
        def run():
            return servo.run_raise(observe,move,639.5,60.,c,path_available=path,
                recover=review,clock=lambda:now[0],log=events.append)
        return run,commands,reviews,events
    def test_extrapolated_bound_reviews_before_another_command_and_resumes_visual(self):
        run,commands,reviews,events=self.scenario()
        result=run()
        self.assertTrue(result['converged']);self.assertEqual(len(reviews),1)
        self.assertEqual(reviews[0][1],3)
        self.assertIn('origin_Y=-100.000mm',reviews[0][0])
        self.assertIn('limit=4.000mm',reviews[0][0])
        self.assertEqual(sum(e['event']=='history_direction_gpt_review_required' for e in events),1)
        self.assertFalse(any(e['event']=='point_loss_history_switch' for e in events))
        self.assertGreaterEqual(result['path_mm'],59.)
        self.assertGreaterEqual(result['actual_z_mm'],58.)
        self.assertTrue(all(abs(q[1]+100)<=4 for q,r in commands))
        self.assertTrue(all(abs(q[1]+97)<1e-6 for q,r in commands[3:]))
    def test_review_failure_sends_no_extrapolated_motion(self):
        run,commands,reviews,_=self.scenario(failure=True)
        with self.assertRaisesRegex(ServoError,'GPT unavailable'):run()
        self.assertEqual(len(commands),3);self.assertEqual(len(reviews),1)
    def test_new_reference_still_obeys_original_y_origin(self):
        run,commands,reviews,_=self.scenario(escape=True)
        with self.assertRaisesRegex(ServoError,'Y correction command bound'):run()
        self.assertEqual(len(commands),3);self.assertEqual(len(reviews),1)
    def test_budget_is_not_reset_by_review(self):
        run,commands,reviews,_=self.scenario(path=35.)
        with self.assertRaisesRegex(ServoError,'path budget'):run()
        self.assertEqual(len(commands),3);self.assertEqual(len(reviews),1)
    def test_failed_new_tracking_does_not_reuse_old_direction(self):
        run,commands,reviews,_=self.scenario(persistent_loss=True)
        with self.assertRaisesRegex(ServoError,'did not restore visual tracking'):run()
        self.assertEqual(len(commands),3);self.assertEqual(len(reviews),1)
