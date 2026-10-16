"""Budget and continuous approach regression with injected robot feedback."""
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from cart_book_grasp_v3_2_stream import run_stream,path_budget_exceeded,translation_budget,load_stream_config as old_settings
from cart_book_grasp_v3_4_policy import ForwardLead,load_stream_config
from cart_book_grasp_v3_column import load_config
from cart_book_grasp_v3_servo import ServoError
from test_v3_1_stream import Sim

CONFIG=Path(__file__).with_name('cart_book_grasp_v3_4_column.json')

class CoupledSim(Sim):
    """Forward motion perturbs Y/Z; fresh feedback, no device interfaces."""
    def __init__(self):super().__init__(loss=True)
    def read(self):
        old=self.q.copy();o=super().read()
        dx=max(0.,self.q[0]-old[0])
        self.q[1]+=.035*dx;self.q[2]+=.05*dx
        o['q']=self.q.copy();o['pose_sample_monotonic']=self.t
        return o


def run(sim,settings,goal=650.,config=None):
    return run_stream(sim.observe,sim.read,sim.blocking,sim.send,lambda:True,sim.finish,goal,
                      load_config(CONFIG) if config is None else config,settings,
                      log=sim.events.append,clock=lambda:sim.t,sleep=lambda dt:setattr(sim,'t',sim.t+dt))


class MotionTests(unittest.TestCase):
    def test_budget_exact_boundary_roundoff_vs_real_excess(self):
        q=np.array([643.0674874439532,-212.92372670675286,14.534493027818417])
        for used in np.linspace(370.05,370.15,1001):
            budget=400.-used;yz=np.array([-213.26823517546683,16.117932774847283])
            dx=np.sqrt(budget**2-np.linalg.norm(yz-q[1:])**2)
            goal=np.r_[q[0]+dx,yz]
            self.assertFalse(path_budget_exceeded(used,np.linalg.norm(goal-q),400.))
        self.assertFalse(path_budget_exceeded(370.,30.,400.))
        self.assertTrue(path_budget_exceeded(370.,30.000001,400.))
        self.assertTrue(path_budget_exceeded(400.000001,0.,400.))
        with self.assertRaises(ServoError):path_budget_exceeded(float('nan'),1,400)
        self.assertEqual(translation_budget({'max_path_mm':400.},{'lookahead_mm':None},400.),0.)
        with self.assertRaises(ServoError):translation_budget({'max_path_mm':400.},{'lookahead_mm':None},400.000001)

    def test_forward_lead_fades_before_recovery_and_ramps_after(self):
        policy=ForwardLead(load_stream_config())
        self.assertEqual(policy.update(1.,0.,True),0.)
        self.assertAlmostEqual(policy.update(1.2,0.,True),10.)
        self.assertAlmostEqual(policy.update(1.4,0.,True),20.)
        self.assertAlmostEqual(policy.update(1.41,.8,True),8.)
        self.assertEqual(policy.update(1.42,1.,True),0.)
        self.assertEqual(policy.update(1.8,0.,False),0.)
        self.assertAlmostEqual(policy.update(1.9,0.,True),5.)

    def test_both_continuous_visual_and_loss_paths_complete(self):
        for loss in (False,True):
            with self.subTest(loss=loss):
                sim=Sim(loss=loss);result=run(sim,load_stream_config(),525.)
                self.assertTrue(result['distance_complete'])
                self.assertEqual(result['visual_alignment_verified'],not loss)
                self.assertLess(result['path_mm'],100.)
                self.assertTrue(any(e['event']=='path_budget_status' for e in sim.events))
                # End point and motion hard limits remain unchanged.
                self.assertEqual(sim.finished[-1][0][0],525.)

    def test_coupled_motion_reduces_recovery_switches_and_path(self):
        results=[]
        for settings in (old_settings(),load_stream_config()):
            sim=CoupledSim();result=run(sim,settings)
            recoveries=sum(e['event'] in ('z_recovery_start','direct_pose_recovery_start') for e in sim.events)
            results.append((recoveries,result['path_mm']))
            self.assertTrue(result['distance_complete'])
        print('Coupled synthetic replay: old/new recovery count and path',results)
        self.assertLess(results[1][0],results[0][0])
        self.assertLess(results[1][1],results[0][1])

    def test_true_budget_exhaustion_is_not_bypassed(self):
        sim=Sim(loss=True);c=load_config(CONFIG);c['max_path_mm']=10.
        with self.assertRaisesRegex(ServoError,'budget'):run(sim,load_stream_config(),550.,c)

    def test_raw_filter_and_delayed_joint_response_still_reach_endpoint(self):
        from test_v3_1_visual_closed_loop import ClosedLoop
        from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted
        sim=ClosedLoop(.35);original=sim.observe
        def observe():
            o=original()
            if o['q'][0]>545.:raise FeaturePointsExhausted(3)
            return o
        def sleep(dt):
            for _ in range(max(1,int(np.ceil(dt/.02)))):sim.tick()
        try:
            result=run_stream(observe,sim.state,sim.settle,sim.send,lambda:True,sim.settle,
                600.,load_config(CONFIG),load_stream_config(),log=sim.events.append,
                clock=lambda:sim.t,sleep=sleep)
        finally:sim.raw.close()
        self.assertTrue(result['distance_complete'])
        self.assertFalse(result['visual_alignment_verified'])
        self.assertLess(abs(sim.state()['q'][0]-600.),2.)
        self.assertLess(result['path_mm'],180.)

    def test_config_keeps_total_budget_and_hard_limits(self):
        c=load_config(CONFIG);s=load_stream_config();old=old_settings()
        self.assertEqual(c['max_path_mm'],400.)
        self.assertEqual(s['gain'],.35)
        for key in ('max_z_drift_mm','direct_stop_y_mm','direct_stop_angle_deg'):
            self.assertEqual(s[key],old[key])
        with tempfile.TemporaryDirectory() as folder:
            file=Path(folder)/'bad.json';s['forward_ramp_seconds']=0.;file.write_text(json.dumps(s))
            with self.assertRaises(ValueError):load_stream_config(file)

if __name__=='__main__':unittest.main()
