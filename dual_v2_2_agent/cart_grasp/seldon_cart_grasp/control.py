"""Shadow-only grasp supervisor and cart-centred, rate-limited base commands."""
from dataclasses import dataclass
import numpy as np
from .geometry import Rejected,finite,wrap

@dataclass
class Evidence:
    stamp: float
    source: str
    contact: bool
    normal_force_n: float
    upward_force_n: float

def contact_valid(e,now,cfg):
    return (e.source in ('tactile','wrist_force') and e.contact and
            np.isfinite([e.stamp,e.normal_force_n,e.upward_force_n]).all() and
            0<=now-e.stamp<=cfg['max_input_age_s'] and
            .5<=e.normal_force_n<=cfg['max_wrist_force_n'] and
            abs(e.upward_force_n)<=cfg['max_upward_force_n'])

class Supervisor:
    def __init__(self,cfg):self.c=cfg;self.state='IDLE';self.reason='';self.last_now=None;self.epoch=0
    def stop(self,reason):self.state='STOPPED';self.reason=reason;self.epoch+=1;return self.state
    def reset(self):self.state='IDLE';self.reason='';self.last_now=None;self.epoch+=1
    def advance(self,now,*,observe_stamp,feedback_stamp,calibrated,stable,plan_valid,at_pregrasp,at_contact,left,right,operator_deadman,waist_unchanged,collision_clear,finished=False):
        if self.state=='STOPPED':return self.state
        if not np.isfinite(now) or self.last_now is not None and (now<self.last_now or now-self.last_now>.3):return self.stop('Control clock jump or missed watchdog deadline')
        self.last_now=now
        if not np.isfinite([observe_stamp,feedback_stamp]).all() or not all(0<=now-s<=self.c['max_input_age_s'] for s in [observe_stamp,feedback_stamp]):return self.stop('Stale/reordered sensor or arm feedback')
        if not calibrated or not waist_unchanged or not collision_clear:return self.stop('Calibration, fixed waist or collision gate invalid')
        if not operator_deadman:return self.stop('Operator lease absent')
        if self.state=='IDLE' and stable:self.state='LOCALIZED'
        elif self.state=='LOCALIZED' and plan_valid:self.state='PREGRASP'
        elif self.state=='PREGRASP' and at_pregrasp:self.state='APPROACH'
        elif self.state=='APPROACH' and at_contact:self.state='CLOSE'
        elif self.state=='CLOSE' and all(contact_valid(e,now,self.c) for e in [left,right]):self.state='GRASPED'
        elif self.state in ('GRASPED','PUSH'):
            if not all(contact_valid(e,now,self.c) for e in [left,right]):return self.stop('Measured contact lost/overloaded')
            self.state='DONE' if finished else 'PUSH'
        return self.state

class CartFollower:
    def __init__(self,cfg,route):
        self.c=cfg;self.route=finite(route)
        if self.route.ndim!=2 or self.route.shape[1]!=2 or len(self.route)<2:raise Rejected('Route needs XY waypoints')
        self.segment=np.diff(self.route,axis=0);self.length=np.linalg.norm(self.segment,axis=1)
        if np.any(self.length<1e-4):raise Rejected('Duplicate route waypoints')
        self.s=np.r_[0,np.cumsum(self.length)];self.progress=0.;self.command=np.zeros(3);self.initialized=False
    def update(self,*args,**kwargs):
        try:return self._update(*args,**kwargs)
        except (ValueError,TypeError,IndexError):
            self.command[:]=0.
            raise
    def _update(self,cart_xy,cart_yaw,robot_xy,robot_yaw,measured_cart_twist,dt,contact_ok,cart_tilt,clearance,now,stamp):
        xy=finite(cart_xy,(2,));robot=finite(robot_xy,(2,));twist=finite(measured_cart_twist,(3,))
        values=finite([dt,now,stamp,cart_yaw,robot_yaw,cart_tilt,clearance]);
        if dt<=0 or dt>.1 or not 0<=now-stamp<=self.c['max_input_age_s'] or not contact_ok or abs(cart_tilt)>self.c['max_cart_tilt_rad'] or clearance<.1:
            self.command[:]=0;raise Rejected('Transport guard: timing/contact/tilt/clearance')
        # Only a physically reachable progress window is considered at crossings.
        forward=.20 if not self.initialized else max(.015,dt*(np.linalg.norm(twist[:2])+self.c['max_linear_m_s'])*2)
        lo=max(0,self.progress-.04);hi=min(self.s[-1],self.progress+forward);candidates=[]
        for i,(a,d,l) in enumerate(zip(self.route,self.segment,self.length)):
            if self.s[i]>hi or self.s[i+1]<lo:continue
            u=np.clip(np.dot(xy-a,d)/l**2,max(0,(lo-self.s[i])/l),min(1,(hi-self.s[i])/l));point=a+u*d;candidates.append((np.linalg.norm(xy-point),self.s[i]+u*l,point))
        error,s,nearest=min(candidates,key=lambda item:item[0]);self.progress=max(self.progress,s);self.initialized=True
        if error>self.c['path_error_abort_m']:self.command[:]=0;raise Rejected('Path error abort')
        remaining=self.s[-1]-self.progress
        if remaining<self.c['goal_tolerance_m'] and np.linalg.norm(xy-self.route[-1])<self.c['goal_tolerance_m']:
            desired=np.zeros(3);done=True
        else:
            look=min(self.s[-1],self.progress+.25);i=min(len(self.length)-1,max(0,np.searchsorted(self.s,look,side='right')-1));target=self.route[i]+(look-self.s[i])/self.length[i]*self.segment[i];direction=target-xy;heading=np.arctan2(direction[1],direction[0]);speed=min(self.c['max_linear_m_s'],remaining*.3)*max(.4,1-error/self.c['path_error_abort_m']);v=speed*np.array([np.cos(heading),np.sin(heading)])
            # v_cart = v_base + omega x lever. Compensate actual rotation first.
            lever=xy-robot;vbase=v-twist[2]*np.array([-lever[1],lever[0]]);c,sn=np.cos(robot_yaw),np.sin(robot_yaw);desired=np.r_[np.array([[c,sn],[-sn,c]])@vbase,np.clip(.8*wrap(heading-cart_yaw),-self.c['max_yaw_rad_s'],self.c['max_yaw_rad_s'])];done=False
        norm=np.linalg.norm(desired[:2]);desired[:2]*=min(1,self.c['max_linear_m_s']/max(norm,1e-12));change=desired[:2]-self.command[:2];change*=min(1,self.c['max_acceleration_m_s2']*dt/max(np.linalg.norm(change),1e-12));self.command[:2]+=change;self.command[2]+=np.clip(desired[2]-self.command[2],-self.c['max_yaw_acceleration_rad_s2']*dt,self.c['max_yaw_acceleration_rad_s2']*dt)
        return {'base_twist':self.command.tolist(),'cart_path_error_m':float(error),'progress_m':float(self.progress),'at_goal':done,'stopped':done and np.linalg.norm(self.command)<1e-4,'hardware_output':False}
