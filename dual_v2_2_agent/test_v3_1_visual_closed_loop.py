"""Outer visual loop + direct joint servo + real RAW filter + delayed plant.

Synthetic Cartesian kinematics and linear image model; no devices, detector or
claim to replicate all real robot dynamics. Uses the recorded image Jacobian.
"""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from test_v3_1_direct_servo import tcp
from v3_1_direct_servo import DifferentialStep,load_settings
from v3_1_panel_path import RawPanel
from cart_book_grasp_v3_1_stream import run_stream,load_stream_config
from cart_book_grasp_v3_column import load_config
from cart_book_grasp_v3_servo import ServoError


class ClosedLoop:
    def __init__(self,gain):
        self.t=1.;self.q=np.zeros(14);self.q[7]=.5/.3;self.q[9]=.01/.3
        self.goal=self.state();self.history=[self.state() for _ in range(8)]
        self.commands=[self.q.copy() for _ in range(3)]
        self.settings=load_settings();self.stream=load_stream_config();self.stream['gain']=gain
        self.stepper=DifferentialStep(tcp,np.array([[-3.]*14,[3.]*14]),self.settings)
        self.raw=RawPanel(self.q,np.ones(14)*1.745*self.settings['raw_velocity_ratio'])
        self.j=np.array([[3.49699311955,.08429180163],[-.17905420473,-1.12631750503]])
        self.events=[];self.samples=[]

    def state(self):
        return dict(q=self.q[7:10].copy()*300.,roll=float(np.rad2deg(self.q[10])),
                    angles=np.array([np.rad2deg(self.q[10]),0.,0.]))

    def tick(self):
        quat=Rotation.from_rotvec([np.deg2rad(self.goal['roll']),0.,0.]).as_quat()[[3,0,1,2]]
        cmd,vel,_=self.stepper.step(self.q,np.r_[self.goal['q']/1000.,quat],.02)
        self.stepper.command=cmd;self.stepper.velocity=vel
        self.commands.append(cmd);raw=self.raw.advance(self.commands.pop(0),20)[-1]
        self.q+=np.tile([.25,.2,.16,.22,.18,.26,.15],2)*(raw-self.q)
        self.t+=.02;self.history.append(self.state());self.samples.append(self.state())

    def observe(self):
        for _ in range(4):self.tick()
        image=self.history[-4];state=self.state()
        error=self.j@np.array([image['q'][1]+20.,image['roll']-3.])
        return dict(state,error=error,scale=1.,stamp=self.t-.06,
                    image_q=image['q'].copy(),image_roll=image['roll'])

    def send(self,q,r):self.goal=dict(q=q.copy(),roll=float(r))

    def settle(self,q,r):
        self.send(q,r)
        for _ in range(300):self.tick()
        return bool(np.linalg.norm(self.state()['q']-q)<2 and abs(self.state()['roll']-r)<1)

    def run(self):
        config=load_config();config['max_seconds']=90.
        try:
            return run_stream(self.observe,self.state,self.settle,self.send,lambda:True,self.settle,
                              600.,config,self.stream,log=self.events.append,clock=lambda:self.t,
                              sleep=lambda dt:None)
        finally:self.raw.close()


class VisualClosedLoopTests(unittest.TestCase):
    def test_old_gain_reproduces_large_lateral_path(self):
        sim=ClosedLoop(5.)
        with self.assertRaisesRegex(ServoError,'path budget'):sim.run()
        y=np.array([s['q'][1] for s in sim.samples])
        self.assertGreater(np.abs(np.diff(y)).sum(),250.)

    def test_configured_gain_converges_with_raw_and_lag(self):
        sim=ClosedLoop(load_stream_config()['gain']);result=sim.run()
        self.assertTrue(result['converged']);self.assertLess(result['path_mm'],180.)
        self.assertLess(abs(sim.state()['q'][1]+20),2.)
        print(f'Visual closed loop: gain={sim.stream["gain"]}, elapsed={sim.t-1:.2f}s, '
              f'path={result["path_mm"]:.2f}mm, final_y={sim.state()["q"][1]:.3f}mm')


if __name__=='__main__':unittest.main()
