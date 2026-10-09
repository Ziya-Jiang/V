"""Simulation-only atomic dual-arm action, reusing the existing cockpit guards.

Not installed into the live hardware server. No socket, driver startup or claim
is performed here. Hardware instances reject this action unconditionally.
"""
import math,time
import numpy as np
from .geometry import finite,Rejected
ACTION='cart_dual_joint_step'

def controller_class(base):
    class CartSimulationController(base):
        def action(self,owner,message):
            if message.get('action')!=ACTION:return super().action(owner,message)
            with self.lock:
                if not self.simulation:raise Rejected('Cart trajectory action is simulation-only; hardware commissioning incomplete')
                self.verify(owner,message);self.check_live()
                if not self.active:raise Rejected('No active control owner')
                if type(message.get('feedback_sequence')) is not int or message['feedback_sequence']!=self.state.get('sequence'):raise Rejected('Joint target based on different feedback sequence')
                q=finite(message.get('joint_positions_v2_rad'),(2,7));duration=float(finite(message.get('duration_s'),()))
                if not 2<=duration<=30:raise Rejected('Duration must be 2–30 seconds')
                targets=[]
                for side in range(2):
                    row=[]
                    for joint in range(7):
                        lo,hi=self.calibration.bounds(side,joint,self.state['arms'][side]['soft_position'][joint]);hlo,hhi=self.calibration.bounds(side,joint,self.state['arms'][side]['hard_position'][joint]);lo,hi=max(lo,hlo),min(hi,hhi)
                        if self.model_limits is not None:lo,hi=max(lo,self.model_limits[side][joint][0]),min(hi,self.model_limits[side][joint][1])
                        if not lo<=q[side,joint]<=hi:raise Rejected('One joint outside calibrated RT/URDF bounds; both arms rejected')
                        raw=self.calibration.to_rt(side,joint,float(q[side,joint]));measured=float(finite(self.state['arms'][side]['position'][joint],()))
                        delta=abs(raw-measured)
                        if delta>.10 or math.pi*.5*delta/duration>.08:raise Rejected('Local segment exceeds displacement/cosine peak-speed limit')
                        row.append(raw)
                    targets.append(row)
                # Commit after every target validates, preserving all gripper targets.
                self.target=targets;self.duration=duration;self.heartbeat=time.monotonic();self.message='Cart simulation: atomic dual-arm local segment'
        def snapshot(self,owner=None):
            result=super().snapshot(owner);result['capabilities']['cart_dual_joint_step_simulation_only']=bool(self.simulation);return result
    return CartSimulationController
