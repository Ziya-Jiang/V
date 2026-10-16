"""Offline only: early point loss, bounded reacquisition and fresh-frame checks."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted,InitialFeaturesUnavailable
from book_top_v43_gpt_recovery import verify_tracking,reanchor,TrackingReacquisitionFailed
from book_top_v43_servo import run_raise,load_config


class ControllerTests(unittest.TestCase):
    def run_case(self,*,persistent=False,loss_after=0,path=500,expired=False):
        q=np.array([410.,-100.,0.]);now=[100.];commands=[];reviews=[];events=[]
        recovered=[False]
        def observe():
            now[0]+=.02
            common=dict(xyz_mm=q.copy(),roll_deg=0.,captured_at=now[0])
            if len(commands)>=loss_after and (persistent or not recovered[0]):
                return dict(common,point_loss=True,remaining_points=3,minimum_points=6)
            return dict(common,uv=[639.5,359.5],angle_deg=0.,scale=1.)
        def move(target,roll):
            nonlocal q
            commands.append(target.copy());q=target.copy();return True
        def recover(reason):
            reviews.append((len(commands),q.copy()));recovered[0]=True
            if expired:now[0]+=901
        def run():return run_raise(observe,move,639.5,40.,load_config(),path_available=path,
            recover=recover,clock=lambda:now[0],log=events.append)
        return run,commands,reviews,events
    def test_initial_loss_recovers_before_any_motion(self):
        run,commands,reviews,events=self.run_case()
        result=run();self.assertTrue(result['converged'])
        self.assertEqual(reviews[0][0],0);self.assertEqual(len(reviews),1)
        self.assertGreater(len(commands),0)
        self.assertFalse(any(e['event']=='point_loss_history_switch' for e in events))
    def test_persistent_loss_has_three_reviews_and_zero_motion(self):
        run,commands,reviews,_=self.run_case(persistent=True)
        with self.assertRaisesRegex(ServoError,'recovery limit'):run()
        self.assertEqual(len(reviews),3);self.assertEqual(commands,[])
    def test_insufficient_history_recovers_without_resetting_path_budget(self):
        run,commands,reviews,_=self.run_case(loss_after=1,path=16.)
        with self.assertRaisesRegex(ServoError,'path budget'):run()
        self.assertEqual(len(reviews),1);self.assertEqual(reviews[0][0],1)
        self.assertLessEqual(sum(np.linalg.norm(b-a) for a,b in zip(
            [np.array([410.,-100.,0.])]+commands,commands)),16.)
    def test_recovery_does_not_reset_time_budget(self):
        run,commands,reviews,_=self.run_case(expired=True)
        with self.assertRaisesRegex(ServoError,'time budget'):run()
        self.assertEqual(commands,[]);self.assertEqual(len(reviews),1)


class VerificationTests(unittest.TestCase):
    def test_three_new_frames_after_old_api_reference(self):
        now=[117.];images=[]
        def capture(check):
            check();now[0]+=.02;rgb=np.full((5,5,3),len(images),np.uint8);images.append(rgb)
            return dict(rgb=rgb,captured_at=now[0])
        lock=NS(observe=Mock(return_value=dict(inliers=8)))
        fresh,_=verify_tracking(lock,capture,lambda:None,100.,clock=lambda:now[0])
        self.assertEqual(lock.observe.call_count,3);self.assertEqual(fresh['captured_at'],now[0])
        for call,rgb in zip(lock.observe.call_args_list,images):self.assertIs(call.args[0],rgb)
    def test_real_feature_lock_across_three_translated_frames(self):
        import cv2
        from cart_book_grasp_v3_feature_centerline import FeatureCenterlineLock
        rng=np.random.default_rng(21)
        rgb=rng.integers(0,256,(160,160,3),dtype=np.uint8)
        mask=np.zeros((160,160),bool);mask[20:140,50:110]=True
        candidate=FeatureCenterlineLock(rgb,mask,centerline=dict(center_u=80.,angle_deg=0.,line=[[80,20],[80,140]]))
        now=[100.];count=[0]
        def capture(check):
            check();count[0]+=1;now[0]+=.02
            frame=cv2.warpAffine(rgb,np.float32([[1,0,count[0]],[0,1,0]]),(160,160))
            return dict(rgb=frame,captured_at=now[0])
        _,o=verify_tracking(candidate,capture,lambda:None,99.,clock=lambda:now[0])
        self.assertGreaterEqual(o['inliers'],6)
        self.assertAlmostEqual(o['uv'][0],83.,delta=.3)
    def test_duplicate_stale_and_processing_expired_frames_fail(self):
        for stamp in (100.,98.):
            lock=NS(observe=Mock())
            with self.assertRaisesRegex(ServoError,'distinct fresh'):
                verify_tracking(lock,lambda check:dict(rgb=0,captured_at=stamp),lambda:None,100.,clock=lambda:101.)
            lock.observe.assert_not_called()
        now=[100.]
        def observe(rgb):now[0]+=3;return {}
        with self.assertRaisesRegex(ServoError,'expired'):
            verify_tracking(NS(observe=observe),lambda c:dict(rgb=0,captured_at=100.),lambda:None,99.,clock=lambda:now[0])
    def test_failure_on_third_frame_is_not_accepted(self):
        now=[100.]
        def capture(c):now[0]+=.02;return dict(rgb=0,captured_at=now[0])
        lock=NS(observe=Mock(side_effect=[{}, {},FeaturePointsExhausted(5)]))
        with self.assertRaises(FeaturePointsExhausted):
            verify_tracking(lock,capture,lambda:None,99.,clock=lambda:now[0])
        self.assertEqual(lock.observe.call_count,3)


class ReanchorTests(unittest.TestCase):
    def case(self,*,visual_failure=False,drift=False,stale=False):
        now=[100.];selected=[False];order=[]
        old=NS(reference_rgb=np.zeros((20,20,3),np.uint8),reference_mask=np.ones((20,20),bool))
        def read():return dict(q=np.array([410.+(4 if drift and selected[0] else 0),-100.,20.]),
            angles=np.zeros(3),pose_sample_monotonic=now[0]-(1 if stale else 0))
        def capture(check):check();now[0]+=.02;order.append('capture');return dict(rgb=old.reference_rgb,captured_at=now[0])
        def select(rgb,record,**kw):
            order.append('GPT');now[0]+=17;selected[0]=True;kw['checkpoint']()
            self.assertIs(kw['reference_rgb'],old.reference_rgb)
            return dict(identity_confirmed=True,mask=old.reference_mask,centerline={'line':[[9,0],[9,19]]})
        lock=NS(observe=Mock(side_effect=FeaturePointsExhausted(3)) if visual_failure else
            Mock(return_value=dict(inliers=8,line=[[9,0],[9,19]])))
        def settled(reader,check,*args,**kw):order.append('stationary');check();return reader()
        with tempfile.TemporaryDirectory() as td,patch('book_top_v43_gpt_recovery.wait_stationary',side_effect=settled):
            try:
                result=reanchor(old,capture,select,read,lambda:None,Path(td),200.,reason='initial loss',
                    factory=lambda *a:lock,clock=lambda:now[0])
            finally:
                record=json.loads((Path(td)/'recovery.json').read_text())
            self.assertEqual(record['verified_frames'],3)
        self.assertEqual(order[0],'stationary');self.assertEqual(lock.observe.call_count,3)
        return result
    def test_same_book_and_three_frames_after_stationary(self):self.case()
    def test_visual_failure_is_retryable(self):
        with self.assertRaises(TrackingReacquisitionFailed):self.case(visual_failure=True)
    def test_motion_or_feedback_fault_is_not_retryable(self):
        for kwargs in ({'drift':True},{'stale':True}):
            with self.assertRaises(ServoError) as caught:self.case(**kwargs)
            self.assertNotIsInstance(caught.exception,TrackingReacquisitionFailed)


class RuntimeTests(unittest.TestCase):
    def case(self,*,initial_failure=False,verification_failure=False,retry_failures=0,fatal=False):
        from contextlib import ExitStack
        import book_top_v43_runtime as rt
        import book_top_v34_inherited as inherited
        config=inherited.load_config(Path(__file__).with_name('cuart_book_grasp_v4_3_config.json'))
        order=[];streams=[]
        def state():
            return dict(received_monotonic=time.monotonic(),pose_sample_monotonic=time.monotonic(),curobo_status=0,
                right_arm=dict(position=np.array([.410,-.100,0.]),orientation=[1.,0.,0.,0.]))
        robot=NS(get_current_state=state,operator_checkpoint=Mock(),_wait_for_robot_stop=Mock(),master_control=Mock())
        camera=NS(calibration_hash='test',capture=lambda check:dict(rgb=np.zeros((20,20,3),np.uint8),captured_at=time.monotonic()),close=Mock())
        def stream_factory(*a,**kw):
            stream=NS(check=lambda:None,close=Mock(side_effect=lambda:order.append('close')))
            stream.start=lambda:(order.append('start') or stream)
            streams.append(stream);return stream
        candidate=NS(last_u=9.5,last_angle=0.,reference_rgb=np.zeros((20,20,3)),reference_mask=np.ones((20,20)))
        attempts=[]
        def recover(*args,**kw):
            order.append('recover');attempts.append(kw)
            self.assertEqual(len(commands),0)
            if not initial_failure and not verification_failure:self.assertTrue(streams[0].close.called)
            if fatal:raise ServoError('stale feedback')
            if len(attempts)<=retry_failures:raise TrackingReacquisitionFailed('points lost')
            return candidate
        commands=[]
        def runner(observe,move,*a,**kw):
            if not initial_failure and not verification_failure:kw['recover']('early loss')
            commands.append('controller resumed')
            return dict(converged=True,path_mm=0.)
        with tempfile.TemporaryDirectory() as td,ExitStack() as stack:
            for target,kwargs in [
                ('cart_book_grasp_v3_camera.WristCamera',dict(return_value=camera)),
                ('cart_book_grasp_v3_2_selection.select_wrist_book',dict(return_value=({'mask':np.ones((20,20))},{}))),
                ('cart_book_grasp_v3_gpt_recovery.make_lock',dict(side_effect=InitialFeaturesUnavailable('no points')) if initial_failure else dict(return_value=candidate)),
                ('book_top_v43_gpt_recovery.verify_tracking',dict(side_effect=FeaturePointsExhausted(3)) if verification_failure else dict(return_value=(None,None))),
                ('book_top_v43_gpt_recovery.reanchor',dict(side_effect=recover))]:stack.enter_context(patch(target,**kwargs))
            stack.enter_context(patch.object(rt,'__file__',str(Path(td)/'runtime.py')))
            stack.enter_context(patch.object(rt,'TrackingStream',side_effect=stream_factory))
            stack.enter_context(patch.object(rt,'run_raise',side_effect=runner))
            scene=dict(geometry={'top_base_mm':[500,-100,0]},path=0.,wrist_detector=Mock(),display=None)
            call=lambda:rt.execute_raise(robot,{'action':'book_v41_top_raise_z','object':'object_0'},0,{},Mock(),None,scene,config)
            if retry_failures>=3 or fatal:
                with self.assertRaisesRegex(ServoError,'stale feedback' if fatal else 'recovery limit'):call()
                self.assertEqual(commands,[]);robot.master_control.assert_called_once()
            else:
                self.assertTrue(call()[1]);self.assertEqual(commands,['controller resumed'])
                robot.master_control.assert_not_called()
        self.assertEqual(len(attempts),1 if fatal else min(3,retry_failures+1))
        camera.close.assert_called_once()
        self.assertTrue(all(s.close.called for s in streams))
    def test_initial_cross_frame_loss_recovers_before_controller(self):self.case(verification_failure=True)
    def test_initial_missing_texture_retries_then_resumes(self):self.case(initial_failure=True,retry_failures=1)
    def test_stream_closes_before_recovery_and_restarts_after_success(self):self.case(retry_failures=1)
    def test_three_failed_recoveries_hold_without_motion(self):self.case(retry_failures=3)
    def test_feedback_fault_does_not_retry(self):self.case(fatal=True)


if __name__=='__main__':unittest.main()
