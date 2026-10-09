"""Fixed-waist dual-arm IK and conservatively screened joint paths.

All planning coordinates use measured base_T_upper. Waist joints are never
commanded or assigned invented encoder values. Collision boxes are broad-phase
conservative envelopes, not a certified continuous collision checker.
"""
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import Rejected,finite,pose,transform,apply

class ArmModel:
    def __init__(self,path,cfg):
        self.path=Path(path);self.doc=ET.parse(path).getroot();self.cfg=cfg
        self.parents={j.find('child').get('link'):j for j in self.doc.findall('joint')};self.joints={j.get('name'):j for j in self.doc.findall('joint')}
        self.names=cfg['joint_names'];self.chains=[];self.bounds=[]
        for names,tip in zip(self.names,cfg['tip_links']):
            chain=[];link=tip
            while link!=cfg['root_link']:
                if link not in self.parents:raise Rejected('Tip does not descend from upper frame')
                j=self.parents[link]
                if j.get('type')!='fixed' and j.get('name') not in names:raise Rejected('Unobserved joint inside IK chain')
                chain.append(j);link=j.find('parent').get('link')
            self.chains.append(chain[::-1]);bounds=[]
            for n in names:
                limit=self.joints[n].find('limit');bounds.append([float(limit.get('lower')),float(limit.get('upper'))])
            bounds=finite(bounds,(7,2))
            if np.any(bounds[:,0]>=bounds[:,1]):raise Rejected('Invalid arm limits')
            self.bounds.append(bounds)
    @staticmethod
    def origin(j):
        o=j.find('origin');return pose([float(x) for x in o.get('xyz','0 0 0').split()],[float(x) for x in o.get('rpy','0 0 0').split()]) if o is not None else np.eye(4)
    def fk(self,side,q,jacobian=False):
        q=finite(q,(7,));values=dict(zip(self.names[side],q));t=np.eye(4);pivots=[]
        for j in self.chains[side]:
            t=t@self.origin(j)
            if j.get('name') in values:
                axis=finite([float(x) for x in j.find('axis').get('xyz').split()],(3,));axis/=np.linalg.norm(axis);pivots.append((t[:3,3].copy(),t[:3,:3]@axis))
                delta=np.eye(4);delta[:3,:3]=Rotation.from_rotvec(axis*values[j.get('name')]).as_matrix();t=t@delta
        if not jacobian:return t
        return t,np.column_stack([np.r_[np.cross(axis,t[:3,3]-p),axis] for p,axis in pivots])
    def ik(self,side,desired,seed,rt_bounds=None):
        desired=transform(desired);q=finite(seed,(7,)).copy();bounds=self.bounds[side].copy()
        if rt_bounds is not None:
            rt_bounds=finite(rt_bounds,(7,2));bounds[:,0]=np.maximum(bounds[:,0],rt_bounds[:,0]);bounds[:,1]=np.minimum(bounds[:,1],rt_bounds[:,1])
        if np.any(bounds[:,0]>=bounds[:,1]) or np.any(q<bounds[:,0]) or np.any(q>bounds[:,1]):raise Rejected('Seed/limits inconsistent with measured calibrated RT')
        for _ in range(240):
            t,j=self.fk(side,q,True);error=np.r_[desired[:3,3]-t[:3,3],Rotation.from_matrix(desired[:3,:3]@t[:3,:3].T).as_rotvec()]
            if np.linalg.norm(error[:3])<self.cfg['ik_position_tolerance_m'] and np.linalg.norm(error[3:])<self.cfg['ik_rotation_tolerance_rad']:return q
            weighted=j.copy();weighted[3:]*=.3;error[3:]*=.3
            dq=weighted.T@np.linalg.solve(weighted@weighted.T+.015**2*np.eye(6),error);dq*=min(1.,.035/max(1e-9,np.max(abs(dq))))
            candidate=np.clip(q+dq,bounds[:,0]+1e-4,bounds[:,1]-1e-4)
            if np.linalg.norm(candidate-q)<1e-8:break
            q=candidate
        raise Rejected('Target unreachable from measured posture; no waist motion or joint clipping fallback')

def contact_targets(observation,base_T_upper,tip_T_contacts,approach_m):
    base_T_upper=transform(base_T_upper);axis=finite(observation.bar_axis,(3,)).copy()
    if axis[0]<0:axis=-axis
    targets=[]
    for point,tcp in zip(observation.handles,tip_T_contacts):
        observed_normal=getattr(observation,'approach_normal',None)
        z=np.array([0.,0.,1.]) if observed_normal is None else finite(observed_normal,(3,)).copy()
        if np.linalg.norm(z)<1e-6:raise Rejected('Invalid observed approach normal')
        z/=np.linalg.norm(z);y=axis.copy();y-=np.dot(y,z)*z
        if np.linalg.norm(y)<1e-6:raise Rejected('Handle axis parallel to approach normal')
        y/=np.linalg.norm(y);x=np.cross(y,z)
        contact=np.eye(4);contact[:3,:3]=np.column_stack((x,y,z));contact[:3,3]=point+z*approach_m
        targets.append(np.linalg.inv(base_T_upper)@contact@np.linalg.inv(transform(tcp)))
    return targets

class BoxScreen:
    """Caller supplies conservative robot geometry at each joint configuration.

    Pair exemptions must correspond to actual adjacent links or intentionally
    touching fingers; no whole-arm/cart exemption. Reject if geometry is absent.
    """
    def __init__(self,geometry,obstacles,excluded_pairs=(),margin=.02):
        self.geometry=geometry;self.obstacles=obstacles;self.exclusions={frozenset(p) for p in excluded_pairs};self.margin=float(margin)
        if margin<=0 or not obstacles:raise Rejected('Require measured body/environment/cart obstacle boxes')
    @staticmethod
    def separation(a,b):
        ac,ah=finite(a[0],(3,)),finite(a[1],(3,));bc,bh=finite(b[0],(3,)),finite(b[1],(3,))
        if np.any(ah<=0) or np.any(bh<=0):raise Rejected('Invalid collision box')
        return float(np.max(abs(ac-bc)-ah-bh))
    def __call__(self,q):
        boxes=self.geometry(q)
        if len(boxes)<14:raise Rejected('Robot collision geometry incomplete')
        for name,box in boxes.items():
            for obstacle in self.obstacles:
                if self.separation(box,(obstacle['center'],obstacle['half']))<self.margin:return False
        for (an,a),(bn,b) in itertools.combinations(boxes.items(),2):
            if frozenset((an,bn)) not in self.exclusions and self.separation(a,b)<self.margin:return False
        return True

class DualPlanner:
    def __init__(self,model):self.model=model
    def plan(self,observation,base_T_upper,tip_T_contacts,measured_q,rt_bounds,collision_screen):
        if collision_screen is None:raise Rejected('Collision screening required before planning')
        current=finite(measured_q,(2,7)).copy();bounds=finite(rt_bounds,(2,7,2));waypoints=[]
        if not collision_screen(current):raise Rejected('Measured start pose fails collision screen')
        for phase,distance in [('pregrasp',.08),('approach',.04),('contact',0.)]:
            goals=contact_targets(observation,base_T_upper,tip_T_contacts,distance)
            target=np.array([self.model.ik(i,goals[i],current[i],bounds[i]) for i in range(2)])
            steps=max(2,int(np.ceil(np.max(abs(target-current))/.005)))
            if steps>1000:raise Rejected('Joint path too large for local approach')
            # Inflate obstacles by >=2 cm; 0.005 rad samples screen the local path.
            for s in np.linspace(0,1,steps+1):
                q=current+s*(target-current)
                if not collision_screen(q):raise Rejected('Approach intersects collision envelope')
            duration=max(2.,1.6*float(np.max(abs(target-current)))/self.model.cfg['max_joint_speed_rad_s'])
            if duration>30:raise Rejected('Approach requires repositioning rather than a large arm move')
            waypoints.append({'phase':phase,'joint_positions_v2_rad':target.tolist(),'duration_s':duration});current=target
        return {'type':'dual_arm_grasp_plan','waist_motion':False,'left_end_effector':'inspire_hand','right_end_effector':'gripper','source_observation_stamp':observation.stamp,'steps':waypoints,'collision_screen':'conservative discrete envelopes; hardware continuous-motion validation still required','grasp_confirmation':'external measured contact required; command echo is not confirmation','hardware_executable':False}
