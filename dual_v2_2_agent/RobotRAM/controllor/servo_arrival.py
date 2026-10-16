"""Read-only arrival gate for small Cartesian servo commands."""
import numpy as np


class ServoArrival:
    def __init__(self, sent_at, arms, position_tolerance_m=.002,
                 orientation_tolerance_rad=np.deg2rad(1), *, initial_state=None):
        self.sent_at=sent_at;self.arms=arms
        self.position_tolerance=position_tolerance_m
        self.orientation_tolerance=orientation_tolerance_rad
        self.last_stamp=-np.inf;self.anchor=None;self.count=0
        self.detail='Waiting for fresh servo feedback'
        self.initial_state=initial_state;self.anchor_stamp=None

    def reset(self):
        self.anchor=None;self.count=0;self.anchor_stamp=None

    @staticmethod
    def angle(a,b):
        return 2*np.arccos(np.clip(abs(np.dot(a,b)),0,1))

    def update(self,state,target,now):
        stamp=state.get('received_monotonic')
        if (not isinstance(stamp,(float,int)) or not np.isfinite(stamp)
                or stamp<=self.sent_at or not 0<=now-stamp<=.5):
            self.reset();self.detail='Missing, stale or pre-command UDP feedback';return False
        if stamp<=self.last_stamp:
            self.detail=f'Repeated UDP feedback; stable_samples={self.count}/3';return False
        self.last_stamp=stamp
        try:
            actual=np.asarray([state[name]['position']+state[name]['orientation']
                               for name in ('left_arm','right_arm')],float)
            target=np.asarray(target,float)
            if actual.shape!=(2,7) or target.shape!=(2,7):raise ValueError('shape')
            actual=actual[list(self.arms)].copy();target=target[list(self.arms)].copy()
            for poses in (actual,target):
                norms=np.linalg.norm(poses[:,3:],axis=1)
                if not np.isfinite(poses).all() or np.any(norms<1e-6):raise ValueError('pose')
                poses[:,3:]/=norms[:,None]
        except (KeyError,TypeError,ValueError):
            self.reset();self.detail='Invalid measured/target servo pose';return False
        error=np.linalg.norm(actual[:,:3]-target[:,:3],axis=1)
        angles=np.array([self.angle(a[3:],b[3:]) for a,b in zip(actual,target)])
        # A goal inside a 2 mm ball must not pass just because the arm has
        # not moved. Require measured progress toward nontrivial translation.
        if self.initial_state is not None:
            try:
                start=np.asarray([self.initial_state[name]['position'] for name in ('left_arm','right_arm')],float)[list(self.arms)]
                if start.shape!=actual[:,:3].shape or not np.isfinite(start).all():raise ValueError('start')
                direction=target[:,:3]-start;distance=np.linalg.norm(direction,axis=1)
                progress=np.sum((actual[:,:3]-start)*direction,axis=1)/np.maximum(distance,1e-12)
                required=np.minimum(.0005,.25*distance)
                if np.any((distance>.00005)&(progress+1e-9<required)):
                    self.reset();self.detail=f'Waiting for real servo displacement: progress_mm={np.round(progress*1000,3).tolist()}, required_mm={np.round(required*1000,3).tolist()}'
                    return False
            except (KeyError,TypeError,ValueError,IndexError):
                self.reset();self.detail='Invalid pre-command servo pose';return False
        if np.any(error>self.position_tolerance) or np.any(angles>self.orientation_tolerance):
            self.reset()
        elif (self.anchor is None or np.any(np.linalg.norm(actual[:,:3]-self.anchor[:,:3],axis=1)>.0005)
              or any(self.angle(a[3:],b[3:])>np.deg2rad(.2) for a,b in zip(actual,self.anchor))):
            self.anchor=actual.copy();self.count=1;self.anchor_stamp=stamp
        else:
            self.count+=1
        self.detail=(f'target_mm={np.round(target[:,:3]*1000,3).tolist()}, '
            f'actual_mm={np.round(actual[:,:3]*1000,3).tolist()}, '
            f'position_error_mm={np.round(error*1000,3).tolist()}, '
            f'orientation_error_deg={np.round(np.rad2deg(angles),3).tolist()}, '
            f'stable_samples={min(self.count,3)}/3, '
            f'stable_seconds={0. if self.anchor_stamp is None else stamp-self.anchor_stamp:.3f}')
        return self.count>=3 and stamp-self.anchor_stamp>=.2-1e-9
