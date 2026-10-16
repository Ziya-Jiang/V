"""Head-coarse task: arm holding is independent of fixed-camera geometry."""
import numpy as np
from cart_book_grasp_v4_1 import RobotIO,pose_mm
from book_top_observation_motion import ObservationMotionGate,ObservationVelocityWarning


class HeadCoarseRobotIO(RobotIO):
    def begin_head_observation(self,state,name):
        self.observation_anchor=pose_mm(state).copy()
        self._head_motion=ObservationMotionGate(state,angle_limit_deg=self.c['orientation_error_deg'],
            velocity_warning=ObservationVelocityWarning(self.log,name,'capture_and_model'),
            sampling_gap_warning=lambda gap:self.log(dict(event='head_observation_sampling_gap',
                observation=name,checked_sample_gap_seconds=gap,
                policy='latest_feedback_age_checked_separately',
                continuous_reception_verified=False)))
        self._head_motion.check(state)
        self._head_peak_drift=0.;self._head_observation=name
        self._head_hold_limit=float(self.c['head_observation_hold_mm'])
        if not np.isfinite(self._head_hold_limit) or not .5<=self._head_hold_limit<=3.:
            raise ValueError('Invalid head observation arm hold budget')
        self.log(dict(event='head_observation_started',observation=name,
                      right_hold_anchor_mm=self.observation_anchor,
                      right_hold_limit_mm=self._head_hold_limit,
                      orientation_limit_deg=self.c['orientation_error_deg'],
                      image_geometry='fixed head RGB-D extrinsics, independent of right TCP'))

    def check_observation(self,s,tcp):
        if self.observation_anchor is None:return
        if not hasattr(self,'_head_motion'):
            raise ValueError('Head observation hold not initialized')
        drift=float(np.linalg.norm(tcp-self.observation_anchor))
        if not np.isfinite(drift):raise ValueError('Invalid head observation arm feedback')
        self._head_peak_drift=max(self._head_peak_drift,drift)
        if drift>self._head_hold_limit:
            raise ValueError(f'Head observation arm hold exceeded: right_tcp_drift={drift:.3f}mm, '
                             f'limit={self._head_hold_limit:.3f}mm; this is an arm hold fault, not image pairing')
        self._head_motion.check(s)

    def end_head_observation(self):
        self.observation_anchor=None
        self.log(dict(event='head_observation_hold_summary',observation=self._head_observation,
                      peak_right_tcp_drift_mm=self._head_peak_drift,
                      right_hold_limit_mm=self._head_hold_limit))
