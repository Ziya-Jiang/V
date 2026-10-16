"""V3.2 offline recovery and transport regression; never opens hardware."""
import unittest
from unittest.mock import patch
import tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from test_v3_1_stream import Sim
from cart_book_grasp_v3_2_stream import run_stream,load_stream_config
from cart_book_grasp_v3_2_lock import DirectPoseLock
from cart_book_grasp_v3_column import load_config as load_column
from cart_book_grasp_v3_servo import ServoError


def load_config():
    return load_column(Path(__file__).with_name('cart_book_grasp_v3_2_column.json'))


class Sim32(Sim):
    def run(self,settings=None,goal_x=525.):
        return run_stream(self.observe,self.read,self.blocking,self.send,lambda:not self.blocked,self.finish,
                          goal_x,load_config(),load_stream_config() if settings is None else settings,
                          log=self.events.append,clock=lambda:self.t,sleep=lambda dt:setattr(self,'t',self.t+dt))


class RecoveryTests(unittest.TestCase):
    def test_arrival_profile_uses_three_mm_on_both_sides(self):
        source=Path(__file__).resolve().parent.parent/'dual_v2_2_control/cuarm_upper_planner/scripts/v3_1_direct_servo.json'
        import json
        self.assertEqual(load_config()['max_pose_error_mm'],3.)
        self.assertEqual(json.loads(source.read_text())['arrival_position_mm'],3.)
        self.assertEqual(load_config()['x_tolerance_mm'],2.)
        self.assertEqual(load_column()['max_pose_error_mm'],2.)

    def guard(self):
        self.t=1.;self.events=[]
        self.state=dict(q=np.array([556.,-170.,-10.7]),angles=np.zeros(3),pose_sample_monotonic=1.)
        return DirectPoseLock(self.state,self.state['q'],load_stream_config(),load_config(),lambda:self.t,self.events.append)

    def test_full_orientation_catches_tilt_even_without_roll(self):
        g=self.guard();self.state['angles'][1]=2.1
        with self.assertRaisesRegex(ServoError,'orientation=2.100deg'):g.check(self.state)
        self.assertEqual(self.events[-1]['event'],'direct_pose_stop')

    def test_transient_above_old_two_mm_limit_recovers_with_fresh_stable_feedback(self):
        g=self.guard();self.state['q'][1]+=2.3;g.check(self.state);self.assertTrue(g.active)
        self.state['q'][1]-=2.3
        self.t+=.1;self.state['pose_sample_monotonic']=self.t;g.check(self.state)
        for _ in range(5):g.check(self.state)
        self.assertTrue(g.active) # one packet cannot establish stability
        for _ in range(3):
            self.t+=.11;self.state['pose_sample_monotonic']=self.t;g.check(self.state)
        self.assertFalse(g.active)
        self.assertIn('direct_pose_recovery_complete',[e['event'] for e in self.events])

    def test_persistent_drift_times_out_and_reports_axes(self):
        g=self.guard();self.state['q'][1]+=2.2;g.check(self.state);self.t+=2.01
        with self.assertRaisesRegex(ServoError,'recovery timed out: Y=\\+2.200mm'):g.check(self.state)

    def test_large_drift_stops_immediately(self):
        g=self.guard();self.state['q'][1]+=4.01
        with self.assertRaisesRegex(ServoError,'hard limit'):g.check(self.state)
        self.assertFalse(g.active)

    def test_reference_is_immutable_and_euler_wrapping_is_not_drift(self):
        g=self.guard();self.state['angles'][2]=360.;g.check(self.state)
        self.state['q'][1]+=.2
        self.assertEqual(g.reference[1],-170.)
        self.assertFalse(g.active)

    def test_visual_updates_remain_continuous(self):
        sim=Sim32();result=sim.run()
        self.assertTrue(result['converged']);self.assertEqual(len(sim.probes),2)
        self.assertEqual(len(sim.finished),1)
        self.assertTrue(any(residual>2 for _,_,residual in sim.sent[1:]))

    def test_hold_error_does_not_change_reference_for_send_check_or_finish(self):
        sim=Sim32(loss=True);finish=sim.finish
        def biased_hold(q,r):
            result=finish(q,r)
            if len(sim.finished)==1:
                sim.q[1]+=.8;sim.q[2]+=.3;sim.r+=.2
                sim.goal=sim.q.copy();sim.roll=sim.r
            return result
        sim.finish=biased_hold;result=sim.run()
        self.assertTrue(result['distance_complete'])
        initial,roll=sim.finished[0];final,lastroll=sim.finished[-1]
        np.testing.assert_allclose(initial[1:],final[1:]);self.assertEqual(roll,lastroll)
        self.assertEqual(final[2],10.)

    def test_recovery_targets_pause_x_then_resume_and_reach_distance_endpoint(self):
        sim=Sim32(loss=True);read=sim.read;send=sim.send;injected=[];records=[]
        def disturbed():
            o=read()
            if len(sim.finished)==1 and not injected:
                sim.q[1]+=2.4;o['q']=sim.q.copy();injected.append(True)
            return o
        def record(q,r):
            phases=[e for e in sim.events if e['event'] in ('direct_pose_recovery_start','direct_pose_recovery_complete')]
            recovering=phases and phases[-1]['event']=='direct_pose_recovery_start'
            records.append((bool(recovering),q[0]-sim.q[0]))
            send(q,r)
        sim.read=disturbed;sim.send=record
        result=sim.run();self.assertTrue(result['distance_complete'])
        recovery=[dx for recovering,dx in records if recovering]
        self.assertTrue(recovery);self.assertLessEqual(max(recovery),1e-9)
        self.assertTrue(any(dx>2 for recovering,dx in records if not recovering))

    def test_final_hold_cannot_hide_pose_drift(self):
        sim=Sim32(loss=True);finish=sim.finish
        def bad_finish(q,r):
            result=finish(q,r)
            if len(sim.finished)>=2:sim.q[1]+=4.2;sim.goal=sim.q.copy()
            return result
        sim.finish=bad_finish
        with self.assertRaisesRegex(ServoError,'hard limit'):sim.run()

    def test_runtime_locks_full_orientation_and_uses_v32_loop(self):
        import cart_book_grasp_v3_2_runtime as runtime
        angles=np.array([-88.75,-18.09,94.24]);calls=[]
        robot=SimpleNamespace()
        def fake_read(self):return dict(q=np.array([556.,-170.,-10.7]),roll=1.1,angles=angles)
        def control(**kw):
            calls.append(kw);robot.target_command=SimpleNamespace(arm_tool_command=np.zeros((2,7)));return True
        robot.master_control=control
        def loop(observe,read,blocking,send,ready,finish,*args,**kwargs):
            kwargs['log'](dict(event='point_loss_direct_switch',fixed_angles_deg=angles.tolist()))
            finish(np.array([556.,-170.,-10.7]),1.1)
            send(np.array([586.,-170.,-10.7]),1.1)
            return dict(completion_mode='direct_distance_after_point_loss')
        with tempfile.TemporaryDirectory() as d,patch.object(runtime.StreamTransport,'read',fake_read),patch.object(runtime,'run_stream',side_effect=loop):
            runtime.run_stream_session(robot,None,None,None,586.,np.zeros(3),load_config(),load_stream_config(),None,
                                       lambda:None,lambda e:None,Path(d),None,started=1.)
        for call in calls:np.testing.assert_allclose(call['right_arm'][3:],angles)
        self.assertEqual(calls[-1]['action'],'book_v3_1_stream') # existing wire protocol

    def test_direct_fallback_with_real_raw_filter_and_delayed_joint_response(self):
        from test_v3_1_visual_closed_loop import ClosedLoop
        from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted
        sim=ClosedLoop(.6);observe=sim.observe;lost=[]
        def feature_frame():
            o=observe()
            if sim.state()['q'][0]>545.:
                lost.append(True)
                raise FeaturePointsExhausted(4)
            return o
        def sleep(dt):
            for _ in range(max(1,int(np.ceil(dt/.02)))):sim.tick()
        try:
            result=run_stream(feature_frame,sim.state,sim.settle,sim.send,lambda:True,sim.settle,
                              600.,load_config(),load_stream_config(),log=sim.events.append,
                              clock=lambda:sim.t,sleep=sleep)
        finally:sim.raw.close()
        self.assertTrue(lost);self.assertTrue(result['distance_complete'])
        self.assertFalse(result['visual_alignment_verified'])
        self.assertLess(abs(sim.state()['q'][0]-600.),2.)
        self.assertLess(result['path_mm'],180.)
        self.assertLessEqual(max(o['q'][0] for o in sim.samples),602.)

    def test_terminal_approach_reduces_lead_without_changing_endpoint(self):
        sim=Sim32(loss=True);send=sim.send;near=[]
        def record(q,r):
            remaining=525.-sim.q[0]
            if 2.<remaining<=20. and q[0]>sim.q[0]:near.append((q[0]-sim.q[0],remaining))
            send(q,r)
        sim.send=record;sim.run()
        self.assertTrue(near)
        self.assertTrue(all(dx<=.35*remaining+1e-8 for dx,remaining in near))
        self.assertEqual(sim.finished[-1][0][0],525.)


if __name__=='__main__':unittest.main()
