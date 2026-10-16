"""Offline GPT recovery: no API requests or robot connections."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from cart_book_grasp_v3_gpt_recovery import recover_tracking
from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted
from cart_book_grasp_v3_servo import ServoError
from test_v3_2_stream import Sim32


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.directory=Path(self.tmp.name);self.t=10.;self.events=[]
        self.state=dict(q=np.array([530.,-120.,-10.]),angles=np.zeros(3),roll=0.)
        self.transport=SimpleNamespace(s=dict(max_z_drift_mm=4.,frame_max_age_seconds=.25),
                                       read=self.read,finish=self.hold)
        self.old=SimpleNamespace(last_u=50.,last_angle=0.)
        self.candidate=SimpleNamespace(last_u=50.,last_angle=0.,observe=lambda rgb:dict(inliers=20,scale=1.))
    def read(self):
        return dict(self.state,pose_sample_monotonic=self.t)
    def sleep(self,dt):self.t+=dt
    def hold(self,q,r):
        self.events.append('hold');self.state['q']=q.copy();return True
    def capture(self,check):
        check();self.t+=.02;self.events.append('capture')
        return (dict(rgb=np.zeros((100,100,3),np.uint8),captured_at=self.t),None,None,None,0.)
    def select(self,rgb,record,**kw):
        self.events.append('gpt');self.t+=8.;kw['checkpoint']()
        return dict(mask=np.ones((100,100),bool))
    def run_recovery(self,**kwargs):
        args=dict(old_lock=self.old,transport=self.transport,capture=self.capture,select=self.select,
                  checkpoint=lambda:None,directory=self.directory,deadline=100.,reference_z=-10.,
                  clock=lambda:self.t,factory=lambda *a,**kw:self.candidate,sleep=self.sleep)
        args.update(kwargs);return recover_tracking(**args)
    def test_hold_then_gpt_then_new_image(self):
        candidate,frame,obs=self.run_recovery()
        self.assertEqual(self.events,['hold','capture','gpt','capture'])
        self.assertIs(candidate,self.candidate)
        self.assertEqual(frame[0]['captured_at'],self.t)
        record=json.loads((self.directory/'recovery.json').read_text())
        self.assertGreater(record['verification_stamp']-record['reference_stamp'],8.)
        self.assertEqual(record['status'],'recovered')
    def test_hold_keeps_original_height(self):
        self.state['q'][2]= -9.4;self.run_recovery()
        self.assertEqual(self.state['q'][2],-10.)
    def test_unconfirmed_hold_never_calls_gpt(self):
        self.transport.finish=lambda *a:False
        with self.assertRaisesRegex(ServoError,'hold not confirmed'):self.run_recovery()
        self.assertNotIn('gpt',self.events)
    def test_motion_during_api_rejects_region(self):
        def moved(*a,**kw):
            self.state['q']=self.state['q']+np.array([0.,1.1,0.]);return self.select(*a,**kw)
        with self.assertRaisesRegex(ServoError,'Arm moved'):self.run_recovery(select=moved)
    def test_deadline_includes_api_time(self):
        with self.assertRaisesRegex(ServoError,'total time budget'):self.run_recovery(deadline=12.)
        self.assertEqual(json.loads((self.directory/'recovery.json').read_text())['status'],'failed')
    def test_wrong_book_jump_rejected(self):
        self.candidate.last_u=140.
        with self.assertRaisesRegex(ServoError,'disagrees'):self.run_recovery()
    def test_repeated_image_rejected(self):
        def repeated(check):
            data=self.capture(check);data[0]['captured_at']=10.;return data
        with self.assertRaisesRegex(ServoError,'newer verification'):self.run_recovery(capture=repeated)
    def test_expired_verification_rejected(self):
        def slow(rgb):self.t+=1.;return dict(inliers=20)
        self.candidate.observe=slow
        with self.assertRaisesRegex(ServoError,'verification image expired'):self.run_recovery()
    def test_failed_reacquisition_is_not_direct_forward_fallback(self):
        def lost(rgb):raise FeaturePointsExhausted(3)
        self.candidate.observe=lost
        with self.assertRaises(ServoError) as caught:self.run_recovery()
        self.assertNotIsInstance(caught.exception,FeaturePointsExhausted)
        self.assertIn('verification failed',str(caught.exception))
    def test_api_failure_propagates(self):
        def fail(*a,**kw):raise ServoError('API unavailable')
        with self.assertRaisesRegex(ServoError,'API unavailable'):self.run_recovery(select=fail)


class StreamRecoveryTests(unittest.TestCase):
    def recovering_sim(self,at):
        sim=Sim32();observe=sim.observe
        def recovering():
            o=observe()
            if sim.observations==at:
                sim.goal=sim.q.copy();sim.roll=sim.r;sim.t+=8.
                o.update(stamp=sim.t,tracking_recovered=True)
            return o
        sim.observe=recovering;return sim
    def test_api_pause_does_not_trigger_no_progress(self):
        sim=self.recovering_sim(6);result=sim.run()
        self.assertTrue(result['converged']);self.assertGreater(sim.t,9.)
    def test_recovery_during_probe_restarts_identification(self):
        sim=self.recovering_sim(2);result=sim.run()
        self.assertTrue(result['converged']);self.assertEqual(len(sim.probes),3)
        np.testing.assert_allclose(next(e['matrix'] for e in sim.events if e['event']=='jacobian_y_roll'),sim.j)
    def test_total_time_budget_not_reset(self):
        from test_v3_2_stream import load_config
        c=load_config();c['max_seconds']=4.
        sim=self.recovering_sim(6)
        with patch('test_v3_2_stream.load_config',return_value=c):
            with self.assertRaisesRegex(ServoError,'total time budget'):sim.run()


class RuntimeIntegrationTests(unittest.TestCase):
    def test_geometry_failure_routes_to_gpt_and_preserves_scale(self):
        import cart_book_grasp_v3_2_runtime as runtime
        from test_v3_2_stream import load_config,load_stream_config
        from cart_book_grasp_v3_feature_centerline import CenterlineGeometryError
        from unittest.mock import Mock
        state=dict(q=np.array([530.,0.,10.]),roll=0.,angles=np.zeros(3),received_monotonic=10.,
                   pose_sample_monotonic=10.,pose_fk_completed_monotonic=10.)
        paired=dict(state,pose_before_stamp=9.99,pose_after_stamp=10.01,pair_gap_seconds=.02,
                    pair_motion_bound_mm=0.,pair_angle_bound_deg=0.)
        frame=dict(rgb=np.zeros((100,100,3),np.uint8),captured_at=10.)
        data=(frame,paired,state,state,0.)
        def measurement(scale):
            return dict(uv=np.array([50.,50.]),angle_deg=0.,inliers=20,scale=scale,
                        line=np.array([[50.,0.],[50.,99.]]))
        old=SimpleNamespace(last_scale=2.,observe=Mock(side_effect=[measurement(2.),CenterlineGeometryError('consensus')]))
        new=SimpleNamespace(observe=Mock(return_value=measurement(1.5)))
        transport=Mock();transport.read.return_value=state;transport.last_angles=np.zeros(3)
        selected=Mock();outputs=[]
        def stream(observe,*args,**kw):
            outputs.extend([observe(),observe(),observe()]);return dict(converged=True)
        with tempfile.TemporaryDirectory() as td, \
             patch.object(runtime,'StreamTransport',return_value=transport), \
             patch.object(runtime,'capture_paired_frame',return_value=data), \
             patch.object(runtime,'run_stream',side_effect=stream), \
             patch.object(runtime,'draw_points',return_value=frame['rgb'].copy()), \
             patch.object(runtime.time,'monotonic',return_value=10.), \
             patch('cart_book_grasp_v3_gpt_recovery.recover_tracking',return_value=(new,data,measurement(1.2))) as recover:
            runtime.run_stream_session(SimpleNamespace(),None,old,[50.,50.],600.,np.zeros(3),
                load_config(),load_stream_config(),
                None,lambda:None,lambda e:None,Path(td),None,started=1.,geometry_recovery=selected)
        self.assertEqual([x['scale'] for x in outputs],[2.,2.4,3.])
        self.assertEqual([x['tracking_recovered'] for x in outputs],[False,True,False])
        self.assertIs(recover.call_args.args[3],selected)
        self.assertEqual(recover.call_args.kwargs['reference_z'],10.)
        transport.abort.assert_not_called()


if __name__=='__main__':unittest.main()
