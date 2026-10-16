"""Continuous V4.1 visual controller and private-port transport.

Head camera is eye-to-hand. Image Jacobian acts on moving pad position,
not the opposite-sign eye-in-hand camera velocity. Depth resolves the third
translation DOF. All client geometry is in mm; angles are degrees.
"""
from collections import deque
from contextlib import contextmanager
import copy
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v4_1 import TiltController, RobotIO, pose_mm, tool_rotation
from book_top_tilt_perception import observed_pad_bias


def image_features(point_camera_mm):
    x, y, z = np.asarray(point_camera_mm, float)
    if not np.isfinite([x, y, z]).all() or z <= 50:
        raise ValueError('Pad point is behind/too close to head camera')
    return np.array([x/z, y/z, np.log(z)])


def image_jacobian(point_camera_mm, rotation_camera_base):
    """d(x,y,log Z)/d(TCP_base_mm), pad offset fixed by tool attitude."""
    x, y, z = np.asarray(point_camera_mm, float)
    image_features([x, y, z])
    return np.array([[1/z, 0., -x/z**2], [0., 1/z, -y/z**2],
                     [0., 0., 1/z]]) @ np.asarray(rotation_camera_base)


def image_correction(pad_base_mm, desired_pad_base_mm, c2w, gain=.6, limit_mm=2.):
    r = c2w[:3, :3].T
    current = r @ (pad_base_mm-c2w[:3, 3])
    target = r @ (desired_pad_base_mm-c2w[:3, 3])
    error = image_features(target)-image_features(current)
    j = image_jacobian(current, r)
    # Scale both feature rows and errors by current depth to condition the
    # solve without changing its units; damping then has a dimensionless scale.
    a, e = current[2]*j, current[2]*error
    delta = gain*a.T @ np.linalg.solve(a@a.T+.02**2*np.eye(3), e)
    delta *= min(1., limit_mm/max(np.linalg.norm(delta), 1e-12))
    return delta, error, j


class ContinuousTiltController(TiltController):
    """Every new frame updates a rolling goal, without waypoint completion.

    Probe lead remains fixed until measured book progress, never accumulated
    from robot commands. A response timeout holds/stops a nonresponding book.
    """
    def __init__(self, c, obs, tcp, rotation, c2w):
        super().__init__(c, obs, tcp, rotation)
        self.c2w = np.asarray(c2w, float).copy()
        self.response_anchor = None
        self.probe_goal = None
        self.latest_rotation = rotation
        self.pad_bias = np.zeros(3)

    def decide_stream(self, obs, paired_tcp, paired_rotation, latest_tcp, latest_rotation):
        stamp = float(obs['captured_unix_s'])
        if not np.isfinite(stamp) or stamp <= self.last_stamp:
            raise ValueError('Book frame did not advance')
        self.last_stamp = stamp
        self.path_mm += float(np.linalg.norm(latest_tcp-self.previous_tcp))
        self.previous_tcp = latest_tcp.copy()
        if np.rad2deg(Rotation.from_matrix(self.rotation.T @ latest_rotation).magnitude()) > self.c['orientation_error_deg']:
            raise ValueError('Horizontal tool orientation drift')
        top, angle, progress = self.book_guard(obs)
        if obs.get('finger_reference_uv') is not None:
            self.pad_bias = observed_pad_bias(obs, paired_tcp, paired_rotation,
                                             obs['K'], obs['D'], self.c2w)
        elif self.c.get('pad_visual_required', True):
            raise ValueError('Current frame lacks a measured finger landmark')
        else:
            self.pad_bias = np.zeros(3)
        surface = self.pad.observe_book_point(top, paired_tcp+self.pad_bias, paired_rotation)
        if self.phase in ('PROBE', 'TILT'):
            if not surface['inside_pad'] or abs(surface['signed_gap_mm']) > self.c['max_pad_gap_mm']:
                raise ValueError('Upper edge left pad region or contact gap')
            if progress >= abs(self.goal)-self.c['angle_tolerance_deg']:
                self.goal_stable += 1
                return dict(phase='HOLD' if self.goal_stable >= 3 else 'VERIFY', target=latest_tcp.copy(),
                            angle_deg=angle, contact_confirmed=False)
            self.goal_stable = 0
        if self.phase == 'ALIGN':
            delta = top-(paired_tcp+paired_rotation@self.pad.center_tcp_mm+self.pad_bias)
            lateral = delta-self.normal*np.dot(delta, self.normal)
            if np.linalg.norm(lateral) > self.c['arrival_mm']:
                return self.visual_command(paired_tcp+self.pad_bias+lateral, paired_tcp, paired_rotation,
                                           latest_tcp, latest_rotation, angle)
            self.phase = 'APPROACH'
        if self.phase == 'APPROACH':
            if not surface['inside_pad'] or surface['signed_gap_mm'] < -self.c['arrival_mm']:
                raise ValueError('Invalid upper-edge placement before near-contact')
            if surface['signed_gap_mm'] > self.c['contact_gap_mm']+self.c['arrival_mm']:
                desired = top-self.normal*self.c['contact_gap_mm']-self.rotation@self.pad.center_tcp_mm
                return self.visual_command(desired, paired_tcp, paired_rotation, latest_tcp, latest_rotation, angle)
            self.phase = 'PROBE'
        radius = np.linalg.norm(np.cross(self.ref.axis_base, self.top0-self.bottom0))
        if self.response_anchor is not None:
            old_progress, old_stamp = self.response_anchor
            response = np.deg2rad(progress-old_progress)*radius
            if response >= self.c['min_book_response_mm']:
                self.response_anchor = None
                self.phase = 'TILT'
            elif stamp-old_stamp >= self.c['response_timeout_seconds']:
                raise ValueError('No measured book response during bounded visual lead; stop')
        if self.response_anchor is None:
            nominal = self.ref.incremental_target(angle, self.goal,
                max_tip_step_mm=self.c['step_mm'], max_angle_step_deg=self.c['angle_step_deg'])
            self.probe_goal = top+nominal['tip_base_mm']-self.ref.target(angle)['tip_base_mm']
            self.probe_goal -= self.normal*self.c['contact_gap_mm']+self.rotation@self.pad.center_tcp_mm
            self.response_anchor = (progress, stamp)
        return self.visual_command(self.probe_goal, paired_tcp, paired_rotation, latest_tcp, latest_rotation, angle)

    def visual_command(self, desired_tcp, paired_tcp, paired_rotation, latest_tcp, latest_rotation, angle):
        paired_pad = paired_tcp+paired_rotation@self.pad.center_tcp_mm+self.pad_bias
        desired_pad = desired_tcp+self.rotation@self.pad.center_tcp_mm
        # Predict the controlled pad feature to dispatch time using measured
        # arm displacement since exposure proxy, not an old pose as a new goal.
        current_pad = latest_tcp+latest_rotation@self.pad.center_tcp_mm+self.pad_bias
        delta, error, j = image_correction(current_pad, desired_pad, self.c2w,
            self.c['visual_gain'], self.c['step_mm'])
        if self.phase == 'ALIGN':
            delta -= self.normal*np.dot(delta, self.normal)
        requested = np.linalg.norm(delta)
        if self.path_mm+requested > self.c['max_path_mm']+1e-6:
            raise ValueError('V4.1 continuous measured path budget exceeded')
        return dict(phase=self.phase, target=latest_tcp+delta, angle_deg=angle,
                    image_error=error, image_jacobian=j,
                    paired_pad_base_mm=paired_pad, current_pad_base_mm=current_pad,
                    pad_visual_bias_mm=self.pad_bias.copy(),
                    contact_confirmed=False)


@contextmanager
def servo_connection():
    root = Path(__file__).resolve().parent
    sys.path[:0] = [str(root/'RobotRAM'), str(root.parent/'dual_v2_2_control/cuarm_upper_planner/scripts')]
    from v4_1_readiness import require_ready
    require_ready()
    from controllor.hkstp_arm import RobotArm
    from cart_book_grasp_v3_1_ports import dedicated_planner
    robot = None
    # Context installs the versioned feedback decoder. Robot endpoints below
    # are explicit V4.1 endpoints; no V3 process or port is reused.
    with dedicated_planner(SimpleNamespace(TARGET_LOCAL_PORT=10185, TARGET_REMOTE_PORT=10186)):
        try:
            robot = RobotArm(target_local_port=10185, target_remote_port=10186)
            until = time.monotonic()+3
            state = None
            while time.monotonic() < until:
                state = robot.get_current_state()
                if state and state.get('received_monotonic') and time.monotonic()-state['received_monotonic'] < .25:
                    break
                time.sleep(.02)
            if not state or time.monotonic()-state['received_monotonic'] >= .25:
                raise ValueError('No fresh V4.1 servo feedback')
            velocity = np.asarray(state.get('joint_velocity', []), float)
            if velocity.shape != (14,) or not np.isfinite(velocity).all() or max(abs(velocity)) > .005:
                raise ValueError('Start V4.1 at rest; continuous updates begin after initialization')
            robot.target_command.gripper_mode_command = [getattr(state[k], 'value', state[k])
                                                        for k in ('left_gripper', 'right_gripper')]
            yield robot
        finally:
            if robot:
                robot.cleanup()


class ContinuousRobotIO(RobotIO):
    """Pose sampler is independent of vision; no background motion-target repeat."""
    def __init__(self, robot, c, log):
        super().__init__(robot, c, log)
        from cart_book_grasp_v3_1_pose_time import PoseTimeline
        self.timeline = PoseTimeline()
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.sample_error = None
        self.pending = deque(maxlen=40)
        self.last_ack = time.monotonic()
        self.started_sending = None
        self.unix_to_monotonic = time.monotonic()-time.time()
        self.thread = threading.Thread(target=self._sample, daemon=True, name='v41-pose-sampler')
        self.thread.start()

    def _sample(self):
        try:
            while not self.stop_event.is_set():
                s = self.read()
                a = Rotation.from_matrix(tool_rotation(s)).as_euler('xyz', degrees=True)
                with self.lock:
                    self.timeline.add(dict(q=pose_mm(s), angles=a, pose_sample_monotonic=s['pose_sample_monotonic']))
                self.stop_event.wait(.01)
        except Exception as exc:
            self.sample_error = exc

    def guard(self):
        if getattr(self, 'sample_error', None) is not None:
            raise ValueError(f'V4.1 pose sampler failed: {self.sample_error}')
        s = super().guard()
        if hasattr(self, 'pending') and self.pending:
            while self.pending and time.monotonic()-self.pending[0][0] > .5:
                self.pending.popleft()
            echoed = np.asarray(s.get('processed_arm_tool_command', []), float)
            if any(echoed.shape == t.shape and np.allclose(echoed, t, atol=1e-6, rtol=0)
                   and s['received_monotonic'] > sent for sent, t in self.pending):
                self.last_ack = time.monotonic()
            if time.monotonic()-self.last_ack > 1.:
                raise ValueError('V4.1 continuous target dispatch not acknowledged')
        return s

    def pair(self, obs):
        from cart_book_grasp_v3_1_pose_time import PoseNotBracketed
        if abs(time.monotonic()-time.time()-self.unix_to_monotonic) > .01:
            raise ValueError('Host wall clock changed; image-pose time mapping invalid')
        stamp = obs['captured_unix_s']+self.unix_to_monotonic
        cfg = dict(max_pose_pair_gap_seconds=.1, max_pose_pair_uncertainty_mm=.5,
                   max_pose_pair_uncertainty_deg=.25)
        deadline = time.monotonic()+.12
        while True:
            self.guard()
            try:
                with self.lock:
                    return self.timeline.pair(stamp, cfg)
            except PoseNotBracketed:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.005)

    def send(self, target, obs_stamp):
        state = self.guard()
        if not 0 <= time.time()-obs_stamp <= self.c['frame_max_age_seconds']:
            raise ValueError('V4.1 image expired before streaming target')
        if np.linalg.norm(target-pose_mm(state)) > self.c['step_mm']+.2:
            raise ValueError('V4.1 new target exceeds current measured lead')
        angles = Rotation.from_matrix(self.rotation).as_euler('xyz', degrees=True)
        if self.started_sending is None:
            self.started_sending = self.last_ack = time.monotonic()
        if self.robot.master_control(right_arm=[*target, *angles], action='book_v3_1_stream',
                object_category='book', wait_for_completion=False) is not True:
            raise ValueError('V4.1 streaming send failed')
        self.pending.append((time.monotonic(), np.asarray(self.robot.target_command.arm_tool_command).copy()))

    @contextmanager
    def perception_pause(self):
        """Explicit nonmoving pause with fresh ACK and continuous pose checks.

        Does not refresh old stream targets or clear controller budgets. Only
        the dedicated V4.1 backend recognizes this reversible pause verb.
        """
        state = self.guard()
        self.pending.clear()
        self.started_sending = None
        sent = time.monotonic()
        # The common master_control whitelist intentionally stays untouched.
        # This private-port message authorizes HOLD only, never a new motion.
        from cuarm_state import TargetCommandArmCommandType
        command = copy.deepcopy(self.robot.target_command)
        command.arm_command_type = TargetCommandArmCommandType.CARTESIAN
        command.description = 'book_v4_1_pause'
        command.arm_tool_command = [[*state[arm]['position'],*state[arm]['orientation']]
                                    for arm in ('left_arm','right_arm')]
        self.robot.target_command = command
        self.robot.udp.send(command)
        target = np.asarray(command.arm_tool_command).copy()
        stable = []; last = -np.inf; acknowledged = False
        deadline = time.monotonic()+3.
        while time.monotonic()<deadline:
            state = self.guard()
            echoed = np.asarray(state.get('processed_arm_tool_command',[]),float)
            acknowledged |= (state['received_monotonic']>sent and echoed.shape==target.shape and
                             np.allclose(echoed,target,atol=1e-6,rtol=0))
            stamp = state['pose_sample_monotonic']
            if stamp>last:
                last = stamp
                velocity = np.asarray(state.get('joint_velocity',[]),float)
                if (velocity.shape!=(14,) or not np.isfinite(velocity).all() or max(abs(velocity))>.005 or
                    (stable and np.linalg.norm(pose_mm(state)-stable[0][1])>.2)):
                    stable=[]
                else:
                    stable.append((stamp,pose_mm(state)))
                    if acknowledged and len(stable)>=3 and stamp-stable[0][0]>=.3:
                        break
            time.sleep(.01)
        else:
            raise ValueError('V4.1 pause not acknowledged/stationary within 3 seconds')
        self.observation_anchor = pose_mm(state)
        self.log(dict(event='perception_pause_confirmed',confirmed=True))
        try:
            yield state
            self.guard()
        finally:
            self.observation_anchor = None
            self.last_ack = time.monotonic()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=1.)
