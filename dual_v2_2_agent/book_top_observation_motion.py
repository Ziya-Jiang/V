"""Fresh-feedback debounce for the V4.1 head coarse observation only."""
import numpy as np
from scipy.spatial.transform import Rotation


class ObservationVelocityWarning:
    """Report valid above-threshold speeds at most once per feedback second."""
    def __init__(self,log,observation,phase):
        self.log=log;self.observation=observation;self.phase=phase
        self.last_sample=-np.inf;self.last_warning=-np.inf

    def check(self,velocity,stamp):
        if stamp<=self.last_sample:return
        self.last_sample=stamp
        speed=float(np.max(abs(velocity)))
        if speed>.005 and stamp-self.last_warning>=1.:
            self.last_warning=stamp
            self.log(dict(event='head_observation_velocity_warning',observation=self.observation,
                phase=self.phase,max_joint_velocity_rad_s=speed,
                joint_index_1based=int(np.argmax(abs(velocity)))+1,
                warning_threshold_rad_s=.005,policy='warn_only_continue'))


class ObservationMotionGate:
    def __init__(self,state,*,angle_limit_deg=.15,velocity_warning=None,sampling_gap_warning=None):
        self.last=-np.inf;self.fast_start=None;self.fast_count=0
        self.rotations=self._rotations(state)
        self.angle_limit_deg=angle_limit_deg
        self.velocity_warning=velocity_warning
        self.sampling_gap_warning=sampling_gap_warning

    @staticmethod
    def _rotations(state):
        q=np.asarray([state[k]['orientation'] for k in ('left_arm','right_arm')],float)
        if q.shape!=(2,4) or not np.isfinite(q).all() or (abs(np.linalg.norm(q,axis=1)-1)>.01).any():
            raise ValueError('Invalid observation quaternion feedback')
        return Rotation.from_quat(q[:,[1,2,3,0]])

    def check(self,state):
        velocity=np.asarray(state.get('joint_velocity',[]),float)
        if velocity.shape!=(14,) or not np.isfinite(velocity).all():
            raise ValueError('Invalid joint velocity during book observation')
        speed=float(np.max(abs(velocity)));joint=int(np.argmax(abs(velocity)))+1
        angle=float(np.max(np.rad2deg((self._rotations(state)*self.rotations.inv()).magnitude())))
        if angle>self.angle_limit_deg:
            raise ValueError(f'Arm rotated during book observation: {angle:.4f}deg (limit={self.angle_limit_deg})')
        if speed>.02 and self.velocity_warning is None:
            raise ValueError(f'Arm moved during book observation: significant velocity={speed:.6f}rad/s, joint={joint}, limit=0.02')
        stamp=float(state['pose_sample_monotonic'])
        if not np.isfinite(stamp):raise ValueError('Invalid observation pose timestamp')
        if stamp<self.last:raise ValueError('Observation feedback timestamp reversed')
        if stamp==self.last:return
        if self.last!=-np.inf and stamp-self.last>.25:
            if self.sampling_gap_warning is None:
                raise ValueError('Observation feedback gap exceeds 0.25s')
            # These are samples read by the consumer, not every received packet.
            # The caller must independently validate the latest receive/pose age.
            self.sampling_gap_warning(stamp-self.last)
        self.last=stamp
        if self.velocity_warning is not None:
            self.velocity_warning.check(velocity,stamp)
            return
        if speed>.005:
            if self.fast_start is None:self.fast_start=stamp
            self.fast_count+=1
            if self.fast_count>=3 and stamp-self.fast_start>=.15:
                raise ValueError(f'Arm moved during book observation: sustained velocity={speed:.6f}rad/s, '
                                 f'joint={joint}, duration={stamp-self.fast_start:.3f}s, samples={self.fast_count}')
        else:
            self.fast_start=None;self.fast_count=0
