"""Immutable direct-approach reference and bounded pose recovery, no device IO."""
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_servo import ServoError


class DirectPoseLock:
    def __init__(self, observation, reference, settings, config, clock, log):
        self.reference = np.asarray(reference, float).copy()
        self.angles = np.asarray(observation['angles'], float).copy()
        self.rotation = Rotation.from_euler('xyz', self.angles, degrees=True)
        self.s, self.c, self.clock, self.log = settings, config, clock, log
        self.active = False
        self.started = None
        self.stable = 0
        self.stable_since = None
        self.last_stamp = -np.inf
        self.last_log = -np.inf

    def check(self, observation):
        now = self.clock()
        q = np.asarray(observation['q'], float)
        angles = np.asarray(observation['angles'], float)
        if q.shape != (3,) or angles.shape != (3,) or not np.isfinite([*q, *angles]).all():
            raise ServoError('V3.2 invalid direct-mode pose')
        delta_y = float(q[1] - self.reference[1])
        angle = float(np.rad2deg((Rotation.from_euler('xyz', angles, degrees=True)
                                 * self.rotation.inv()).magnitude()))
        details = dict(reference_mm=self.reference.tolist(), actual_mm=q.tolist(),
                       reference_angles_deg=self.angles.tolist(), actual_angles_deg=angles.tolist(),
                       y_error_mm=delta_y, orientation_error_deg=angle,
                       z_error_mm=float(q[2] - self.reference[2]),
                       recovery_y_mm=self.s['direct_recovery_y_mm'],
                       recovery_angle_deg=self.s['direct_recovery_angle_deg'],
                       stop_y_mm=self.s['direct_stop_y_mm'],
                       stop_angle_deg=self.s['direct_stop_angle_deg'])
        def fail(reason):
            self.log(dict(event='direct_pose_stop', reason=reason, **details))
            raise ServoError(f'V3.2 direct-mode {reason}: Y={delta_y:+.3f}mm, '
                             f'orientation={angle:.3f}deg, '
                             f'limits={self.s["direct_stop_y_mm"]:.3f}mm/'
                             f'{self.s["direct_stop_angle_deg"]:.3f}deg')
        if abs(delta_y) > self.s['direct_stop_y_mm'] or angle > self.s['direct_stop_angle_deg']:
            fail('hard limit')
        if not self.active and (abs(delta_y) > self.s['direct_recovery_y_mm'] or
                                angle > self.s['direct_recovery_angle_deg']):
            self.active = True
            self.started = now
            self.stable = 0
            self.stable_since = None
            self.log(dict(event='direct_pose_recovery_start', **details))
        if self.active:
            # A duplicate feedback packet must not count as multiple stable samples.
            stamp = float(observation.get('pose_sample_monotonic', now))
            inside = (abs(delta_y) <= self.s['direct_recovery_y_mm'] * .5 and
                      angle <= self.s['direct_recovery_angle_deg'] * .5)
            if not inside:
                self.stable = 0
                self.stable_since = None
            elif stamp > self.last_stamp:
                self.stable += 1
                if self.stable_since is None:
                    self.stable_since = stamp
            self.last_stamp = max(stamp, self.last_stamp)
            if (inside and self.stable >= self.c['stable_frames'] and
                    stamp - self.stable_since >= .2):
                self.active = False
                self.log(dict(event='direct_pose_recovery_complete',
                              elapsed_seconds=now-self.started, **details))
            elif now - self.started >= self.s['direct_recovery_seconds']:
                fail('recovery timed out')
        if now - self.last_log >= .5:
            self.log(dict(event='direct_pose_feedback', recovering=self.active, **details))
            self.last_log = now
