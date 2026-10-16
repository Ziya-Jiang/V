"""Match camera host read-completion times to source panel receipt-time poses.

These are host-time proxies, not hardware exposure/encoder timestamps. Bounds
below constrain interpolation support; they do not calibrate transport latency.
"""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from cart_book_grasp_v3_servo import ServoError


class PoseNotBracketed(ServoError):pass


class PoseSupportUnavailable(ServoError):
    """Reject this frame's interpolation support; a new frame may recover."""
    def __init__(self,message,**details):
        super().__init__(message);self.details=details


class PoseTimeline:
    def __init__(self):self.samples=deque(maxlen=512)

    def add(self,state):
        t=float(state['pose_sample_monotonic']);q=np.asarray(state['q'],float);a=np.asarray(state['angles'],float)
        if q.shape!=(3,) or a.shape!=(3,) or not np.isfinite([t,*q,*a]).all():
            raise ServoError('V3.1 invalid pose history sample')
        if self.samples and t<self.samples[-1][0]:raise ServoError('V3.1 feedback timestamp moved backwards')
        if self.samples and t==self.samples[-1][0]:return  # Cached packet is not a new sample.
        self.samples.append((t,q.copy(),Rotation.from_euler('xyz',a,degrees=True)))
        while len(self.samples)>2 and self.samples[1][0]<t-3.:self.samples.popleft()

    def pair(self,stamp,settings):
        if not np.isfinite(stamp):raise ServoError('V3.1 invalid frame timestamp')
        if not self.samples or stamp<self.samples[0][0] or stamp>self.samples[-1][0]:
            raise PoseNotBracketed('V3.1 frame timestamp has no bracketing feedback')
        right=next(i for i,s in enumerate(self.samples) if s[0]>=stamp)
        if self.samples[right][0]==stamp:
            t,q,r=self.samples[right]
            return dict(q=q.copy(),angles=r.as_euler('xyz',degrees=True),pair_gap_seconds=0.,
                        pair_motion_bound_mm=0.,pair_angle_bound_deg=0.,pose_before_stamp=t,pose_after_stamp=t)
        t0,q0,r0=self.samples[right-1];t1,q1,r1=self.samples[right];gap=t1-t0
        if gap>settings['max_pose_pair_gap_seconds']:
            raise PoseSupportUnavailable(f'V3.1 pose pair gap too large: {gap:.4f}s',
                pose_before_stamp=t0,pose_after_stamp=t1,pair_gap_seconds=gap)
        weight=(stamp-t0)/gap;q=q0+(q1-q0)*weight
        r=Slerp([t0,t1],Rotation.concatenate([r0,r1]))([stamp])[0]
        bound=max(np.linalg.norm(q-q0),np.linalg.norm(q1-q))
        angular=max(np.degrees((r.inv()*r0).magnitude()),np.degrees((r.inv()*r1).magnitude()))
        if bound>settings['max_pose_pair_uncertainty_mm'] or angular>settings['max_pose_pair_uncertainty_deg']:
            raise PoseSupportUnavailable(f'V3.1 pose pair support too wide: {bound:.3f}mm, {angular:.3f}deg, gap={gap:.4f}s',
                pose_before_stamp=t0,pose_after_stamp=t1,pair_gap_seconds=gap,
                pose_before_mm=q0.tolist(),pose_after_mm=q1.tolist(),
                pair_motion_bound_mm=float(bound),pair_angle_bound_deg=float(angular))
        return dict(q=q,angles=r.as_euler('xyz',degrees=True),pair_gap_seconds=gap,
                    pair_motion_bound_mm=float(bound),pair_angle_bound_deg=float(angular),
                    pose_before_stamp=t0,pose_after_stamp=t1)
