#!/usr/bin/env python3
"""Independent right-finger top-edge book tipping, fixed horizontal tool pose.

Default: auto semantic selection + observation only (GPT API enabled).
--select retains manual book selection as an observation-only fallback.
--execute uses head coarse positioning, then wrist visual landing only.
Automatic visible-underside RGB-D geometry is the default; no manual teaching required.
Uses private-port CPU differential XYZ servo; no per-frame MotionGen planning.
No camera, robot, network or paid API is opened on import.
"""
import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from book_top_tilt_geometry import FingerPad, TiltReference, vector
from book_top_tilt_perception import (FaceTracker, FingerTracker, TrackingLost,
    measure_face, validate_quad, physical_midpoints, project_base)

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT/'cart_book_grasp_v4_1_config.json'


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, default=jsonable, ensure_ascii=False,
                                   indent=2, allow_nan=False)+'\n')


def load_config(path=CONFIG, execute=False):
    c = json.loads(Path(path).read_text())
    bounds = dict(goal_angle_deg=(1, 45), step_mm=(.5, 2), angle_step_deg=(.1, 1),
        contact_gap_mm=(0, 1), precontact_gap_mm=(5, 15), max_initial_gap_mm=(15, 50),
        max_align_mm=(1, 30), arrival_mm=(.1, .8), orientation_error_deg=(.1, 1),
        max_pivot_drift_mm=(.5, 3), max_shape_error_mm=(1, 5),
        max_pad_gap_mm=(1, 3), min_book_response_mm=(.2, 1),
        max_path_mm=(10, 500), max_seconds=(10, 900), motion_timeout_seconds=(2, 15),
        angle_tolerance_deg=(.2, 2), frame_max_age_seconds=(.05, .5),
        observation_spread_mm=(.2, 2), horizontal_tolerance_deg=(.2, 3),
        visual_gain=(.05, 1.), visual_hz=(1., 15.), response_timeout_seconds=(.5, 3.))
    for key, (lo, hi) in bounds.items():
        v = c.get(key)
        if type(v) not in (float, int) or not np.isfinite(v) or not lo <= v <= hi:
            raise ValueError(f'Invalid {key}: expected finite number in [{lo}, {hi}]')
    if c['step_mm'] < 2*c['arrival_mm']:
        raise ValueError('step_mm must be at least twice arrival_mm')
    if c['contact_gap_mm'] >= c['precontact_gap_mm']:
        raise ValueError('Contact and precontact gaps are inconsistent')
    toward = vector(c['towards_robot_base'], 'towards_robot_base')
    if np.linalg.norm(toward) < .9:
        raise ValueError('Specify a nonzero measured base direction towards robot')
    if type(c.get('pad_visual_required', True)) is not bool:
        raise ValueError('pad_visual_required must be boolean')
    if type(c.get('max_semantic_recoveries', 2)) is not int or not 0 <= c.get('max_semantic_recoveries', 2) <= 2:
        raise ValueError('max_semantic_recoveries must be 0..2')
    if not isinstance(c.get('target_description', ''), str):
        raise ValueError('target_description must be text')
    from book_top_landing import settings
    settings(c)
    if c.get('pad_source', 'auto_rgbd') not in ('auto_rgbd', 'measured'):
        raise ValueError('pad_source must be auto_rgbd or measured')
    if execute and c.get('pad_source', 'auto_rgbd') == 'measured':
        if c.get('pad_calibrated') is not True or c.get('scene_setup_confirmed') is not True:
            raise ValueError('Execution needs measured pad calibration and verified scene setup; see README')
        FingerPad(**c['pad'])
        aperture = c.get('calibrated_gripper_position')
        if type(aperture) not in (int, float) or not np.isfinite(aperture) or not 0 <= aperture <= 1:
            raise ValueError('Record calibrated_gripper_position (measured normalized opening, 0..1)')
    return c


def tool_rotation(state):
    q = np.asarray(state['right_arm']['orientation'], float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .01:
        raise ValueError('Invalid right TCP quaternion')
    return Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()


def pose_mm(state):
    return vector(state['right_arm']['position'], 'right TCP')*1000


class TiltController:
    """Pure state machine. One decision per NEW stationary observation.

    The caller executes each returned finite waypoint and obtains NEW measured
    book/robot data before asking again. A nonresponding book cannot accumulate
    trial displacement. Contact is always a visual hypothesis, never force.
    """
    def __init__(self, config, observation, tcp, rotation):
        self.c = config
        self.pad = FingerPad(**config['pad'])
        self.rotation = np.asarray(rotation, float).copy()
        self.normal = self.rotation @ self.pad.normal_tcp
        if np.dot(self.normal, [0, 0, -1]) < np.cos(np.deg2rad(config['horizontal_tolerance_deg'])):
            raise ValueError('Pad underside must already be horizontal, outward normal down')
        self.top0 = vector(observation['top_base_mm'], 'top')
        self.bottom0 = vector(observation['bottom_base_mm'], 'bottom')
        self.axis = vector(observation['axis_base'], 'support axis')
        self.ref = TiltReference(self.bottom0, self.axis, self.top0,
                                 self.pad.center_tcp_mm, self.rotation)
        self.goal = self.ref.angle_towards(config['towards_robot_base'], config['goal_angle_deg'])
        # The selected physical bottom edge must be approximately horizontal.
        if abs(self.ref.axis_base[2]) > .08 or self.top0[2]-self.bottom0[2] < 40:
            raise ValueError('Selected bottom edge is not an observable horizontal support axis')
        self.phase = 'ALIGN'
        self.last_stamp = -np.inf
        self.path_mm = 0.
        self.previous_tcp = vector(tcp, 'initial TCP')
        self.initial_tcp = self.previous_tcp.copy()
        self.pending_response = None
        self.last_progress = None
        self.goal_stable = 0
        surface = self.pad.observe_book_point(self.top0, tcp, self.rotation)
        if not config['precontact_gap_mm'] <= surface['signed_gap_mm'] <= config['max_initial_gap_mm']:
            raise ValueError('Start above the book: pad gap must be within configured precontact/initial range')
        lateral = self.top0-(tcp+self.rotation@self.pad.center_tcp_mm)
        lateral -= self.normal*np.dot(lateral, self.normal)
        if np.linalg.norm(lateral) > config['max_align_mm']:
            raise ValueError('Start from a near-hover pose; lateral offset exceeds V4.1 local approach range')

    def book_guard(self, obs):
        top = vector(obs['top_base_mm'], 'observed top')
        bottom = vector(obs['bottom_base_mm'], 'observed bottom')
        axis = vector(obs['axis_base'], 'observed axis')
        if np.linalg.norm(axis) < .9:
            raise ValueError('Book support axis invalid')
        if np.linalg.norm(bottom-self.bottom0) > self.c['max_pivot_drift_mm']:
            raise ValueError('Book support edge moved; stop tipping')
        if np.dot(axis/np.linalg.norm(axis), self.ref.axis_base) < np.cos(np.deg2rad(3)):
            raise ValueError('Book support axis changed')
        angle = self.ref.measured_angle(top, bottom)
        expected = self.ref.target(angle)['tip_base_mm']
        if np.linalg.norm(top-expected) > self.c['max_shape_error_mm']:
            raise ValueError('Book geometry no longer follows the measured support axis')
        progress = angle*np.sign(self.goal)
        if progress < -1 or progress > abs(self.goal)+2:
            raise ValueError('Book rotated opposite to goal or beyond the allowed angle')
        if self.phase in ('ALIGN', 'APPROACH') and abs(angle) > 2:
            raise ValueError('Book moved before the contact trial')
        return top, angle, progress

    def decide(self, obs, tcp, rotation):
        stamp = float(obs['captured_unix_s'])
        if not np.isfinite(stamp) or stamp <= self.last_stamp:
            raise ValueError('Book observation did not advance')
        self.last_stamp = stamp
        tcp = vector(tcp, 'TCP')
        error = Rotation.from_matrix(self.rotation.T @ rotation).magnitude()
        if np.rad2deg(error) > self.c['orientation_error_deg']:
            raise ValueError('Horizontal tool orientation drift')
        self.path_mm += float(np.linalg.norm(tcp-self.previous_tcp))
        self.previous_tcp = tcp.copy()
        top, angle, progress = self.book_guard(obs)
        surface = self.pad.observe_book_point(top, tcp, rotation)
        if self.phase in ('PROBE', 'TILT'):
            if not surface['inside_pad'] or abs(surface['signed_gap_mm']) > self.c['max_pad_gap_mm']:
                raise ValueError('Book top left the calibrated pad region/gap')
            if self.pending_response is not None:
                old_top, old_progress = self.pending_response
                responded_mm = np.deg2rad(progress-old_progress)*np.linalg.norm(np.cross(
                    self.ref.axis_base, self.top0-self.bottom0))
                if responded_mm < self.c['min_book_response_mm']:
                    raise ValueError('No measured book response to the finite trial; no further push')
                if np.linalg.norm(top-old_top) > self.c['step_mm']+self.c['max_shape_error_mm']:
                    raise ValueError('Book response jumped beyond the commanded trial')
                self.pending_response = None
                self.phase = 'TILT'
            if progress >= abs(self.goal)-self.c['angle_tolerance_deg']:
                self.goal_stable += 1
                return dict(phase='HOLD' if self.goal_stable >= 3 else 'VERIFY',
                            target=None, angle_deg=angle, contact_confirmed=False)
            self.goal_stable = 0
        if self.phase == 'ALIGN':
            delta = top-(tcp+self.rotation@self.pad.center_tcp_mm)
            lateral = delta-self.normal*np.dot(delta, self.normal)
            if np.linalg.norm(lateral) > self.c['arrival_mm']:
                # Horizontal translation only, above the book.
                return self.command(tcp+lateral, tcp, angle)
            self.phase = 'APPROACH'
        if self.phase == 'APPROACH':
            if not surface['inside_pad']:
                raise ValueError('Top edge not contained by pad before descent')
            gap = surface['signed_gap_mm']
            if gap < -self.c['arrival_mm']:
                raise ValueError('Estimated pad already below book top; do not press further')
            if gap > self.c['contact_gap_mm']+self.c['arrival_mm']:
                return self.command(top-self.normal*self.c['contact_gap_mm']-
                                    self.rotation@self.pad.center_tcp_mm, tcp, angle)
            self.phase = 'PROBE'
        nominal = self.ref.incremental_target(angle, self.goal,
            max_tip_step_mm=self.c['step_mm'], max_angle_step_deg=self.c['angle_step_deg'])
        # Compensate actual book location. Keep pad horizontal and avoid an
        # extra push into the book to compensate a visual contact uncertainty.
        delta = nominal['tip_base_mm']-self.ref.target(angle)['tip_base_mm']
        target = top+delta-self.normal*self.c['contact_gap_mm']-self.rotation@self.pad.center_tcp_mm
        decision = self.command(target, tcp, angle)
        self.pending_response = (top.copy(), progress)
        return decision

    def command(self, target, tcp, angle):
        delta = np.asarray(target)-tcp
        distance = np.linalg.norm(delta)
        if distance < .05:
            raise ValueError('Next visual target is below resolvable motion; hold for review')
        target = tcp+delta*min(1., self.c['step_mm']/distance)
        requested = float(np.linalg.norm(target-tcp))
        if self.path_mm+requested > self.c['max_path_mm']+1e-6:
            raise ValueError(f'V4.1 path budget: used={self.path_mm:.6f}, next={requested:.6f}')
        return dict(phase=self.phase, target=target, angle_deg=angle, contact_confirmed=False)


class Vision:
    def __init__(self, camera, selection, c, out):
        self.camera, self.c, self.out = camera, c, Path(out)
        if isinstance(selection, (str, Path)):
            with np.load(selection, allow_pickle=False) as f:
                data = {key:f[key].copy() for key in f.files}
        else:
            data = selection
        rgb, quad = data['rgb'], data['quad']
        if not np.allclose(data['c2w'], camera.c2w, atol=1e-6):
            raise ValueError('Head extrinsic changed after target selection')
        self.k, self.d = data['K'].copy(), data['D'].copy()
        self.reference = data
        self.tracker = FaceTracker(rgb, quad, landmarks=data.get('landmarks'), exclude=data.get('finger_quad'))
        fq = data.get('finger_quad')
        self.finger = None if fq is None or np.asarray(fq).size==0 else FingerTracker(rgb, fq)
        self.last_stamp = -np.inf
        self.rgb = rgb
        self.last_observation = None

    @classmethod
    def automatic(cls, camera, selector, c, out, *, checkpoint=lambda: None,
                  reference=None, pad_hint=None):
        out = Path(out); out.mkdir(parents=True, exist_ok=True)
        checkpoint()
        rgb, depth = camera.capture_image()
        stamp = float(camera.captured_unix_s)
        if not 0<=time.time()-stamp<=c['frame_max_age_seconds']:
            raise ValueError('Stale initialization image')
        record = dict(status='pending', captured_unix_s=stamp)
        data = dict(rgb=rgb, depth=depth, K=camera.K.copy(), D=camera.D.copy(),
                    c2w=camera.c2w.copy(), captured_unix_s=stamp)
        try:
            chosen = selector.select(rgb, out, checkpoint=checkpoint, reference=reference, pad_hint=pad_hint)
            data.update(chosen)
            # GPT provides semantics only. Reconstruct from THIS image's depth,
            # then carry those landmarks to a NEW frame before any command.
            data['landmarks'] = physical_midpoints(chosen['quad'], depth, data['K'], data['D'], data['c2w'])
            vision = cls(camera, data, c, out)
            record.update(status='selected_pending_new_frames', method=chosen['method'])
            return vision
        except BaseException as exc:
            record.update(status='failed', reason=str(exc)); raise
        finally:
            np.savez_compressed(out/'selection_frame.npz', **{k:v for k,v in data.items() if v is not None})
            image = cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            for key,color in [('quad',(0,215,255)),('finger_quad',(0,220,0))]:
                if data.get(key) is not None:
                    cv2.polylines(image,[np.rint(data[key]).astype(np.int32)],True,color,2)
            cv2.imwrite(str(out/'first_detection.jpg'),image)
            write_json(out/'first_detection.json',record)

    def bind_finger(self, state):
        if self.finger is None:
            if self.c.get('pad_visual_required',True):
                raise ValueError('V4.1 needs a visible selected finger side/lower edge')
            return
        self.finger.bind(pose_mm(state),tool_rotation(state),FingerPad(**self.c['pad']),
                         self.k,self.d,self.camera.c2w)

    def observe(self):
        rgb, depth = self.camera.capture_image()
        self.rgb = rgb
        stamp = float(self.camera.captured_unix_s)
        if not 0 <= time.time()-stamp <= self.c['frame_max_age_seconds'] or stamp <= self.last_stamp:
            raise ValueError('Head frame stale or repeated')
        if not np.allclose(self.k, self.camera.K) or not np.allclose(self.d, self.camera.D):
            raise ValueError('Head intrinsics changed after selection')
        try:
            quad = self.tracker.update(rgb)
            value = measure_face(quad, depth, self.camera.K, self.camera.D, self.camera.c2w,
                                 self.tracker.edge_midpoints)
            if self.finger:
                value.update(self.finger.update(rgb))
        except ValueError as exc:
            cv2.imwrite(str(self.out/'rejected_rgb.jpg'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            np.savez_compressed(self.out/'rejected_rgbd.npz', rgb=rgb, depth=depth,
                                K=self.camera.K, D=self.camera.D, c2w=self.camera.c2w)
            write_json(self.out/'rejection.json', dict(reason=str(exc), captured_unix_s=stamp,
                last_valid_observation=self.last_observation))
            raise
        value.update(captured_unix_s=stamp, points=len(self.tracker.points), quad=quad,
                     edge_midpoints_uv=self.tracker.edge_midpoints.copy(),
                     tracking_method=self.tracker.method, edge_source='fixed_landmarks_via_homography',
                     K=self.k.copy(), D=self.d.copy())
        self.last_stamp, self.last_observation = stamp, value
        return value

    def stable(self, checkpoint=lambda: None, *, min_stamp=-np.inf):
        values = []
        deadline = time.monotonic()+3.
        while len(values) < 3:
            if time.monotonic() > deadline:
                raise ValueError('No three new book frames after stationary acquisition began')
            checkpoint()
            value = self.observe()
            checkpoint()
            if value['captured_unix_s'] >= min_stamp:
                values.append(value)
        for key in ('top_base_mm', 'bottom_base_mm'):
            xyz = np.asarray([v[key] for v in values])
            if np.max(np.linalg.norm(xyz-xyz[-1], axis=1)) > self.c['observation_spread_mm']:
                raise ValueError('Book RGB-D is not stable enough for a contact decision')
        return values[-1]

    def save(self, name='latest'):
        bgr = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR)
        cv2.polylines(bgr, [np.rint(self.tracker.quad).astype(np.int32)], True, (0, 215, 255), 2)
        for label, point in zip(('top-L', 'top-R', 'bottom-R', 'bottom-L', 'top-center', 'support'),
                               np.vstack([self.tracker.quad, self.tracker.edge_midpoints])):
            pixel = tuple(np.rint(point).astype(int))
            cv2.circle(bgr, pixel, 5, (20, 230, 20), -1)
            cv2.putText(bgr, label, pixel, cv2.FONT_HERSHEY_SIMPLEX, .5, (20, 230, 20), 1)
        if self.finger:
            cv2.polylines(bgr,[np.rint(self.finger.tracker.quad).astype(np.int32)],True,(255,160,0),2)
            for pt in self.finger.tracker.edge_midpoints:
                cv2.circle(bgr,tuple(np.rint(pt).astype(int)),4,(255,80,0),-1)
        cv2.imwrite(str(self.out/f'{name}.jpg'), bgr)
        if self.last_observation:
            write_json(self.out/f'{name}.json', self.last_observation)


class RobotIO:
    """Shared feedback, calibration and cancellation guards for V4.1.

    ContinuousRobotIO adds a pose sampler and nonblocking private-port sends.
    No waypoint-planning or arrival-wait method is exposed here.
    """
    def __init__(self, robot, c, log):
        self.robot, self.c, self.log = robot, c, log
        self.old_read = robot.get_current_state
        initial = self.read()
        self.rotation = tool_rotation(initial)
        self.start = pose_mm(initial)
        self.left_start = vector(initial['left_arm']['position'], 'left TCP')*1000
        self.gripper_modes = [getattr(initial[k], 'value', initial[k]) for k in ('left_gripper', 'right_gripper')]
        self.current_segment_start = None
        self.deadline = time.monotonic()+c['max_seconds']
        self.observation_anchor = None

    def read(self):
        s = self.old_read()
        if not s:
            raise ValueError('V4.1 robot feedback missing')
        for key in ('received_monotonic', 'pose_sample_monotonic'):
            stamp = s.get(key)
            if stamp is None or not np.isfinite(stamp) or not 0 <= time.monotonic()-stamp <= .25:
                raise ValueError(f'V4.1 robot feedback stale: {key}')
        return s

    def guard(self):
        if time.monotonic() > self.deadline:
            raise ValueError('V4.1 total time budget')
        s = self.read()
        if getattr(s['robot_status'], 'value', s['robot_status']) not in (0, 1):
            raise ValueError('Robot unavailable')
        if getattr(s['curobo_status'], 'value', s['curobo_status']) in (3, 7, 8):
            raise ValueError('CuRobo rejected the V4.1 waypoint')
        tcp = pose_mm(s)
        if np.linalg.norm(np.asarray(s['left_arm']['position'])*1000-self.left_start) > 1:
            raise ValueError('Left arm moved during right-finger task')
        if [getattr(s[k], 'value', s[k]) for k in ('left_gripper', 'right_gripper')] != self.gripper_modes:
            raise ValueError('Gripper command changed during pad calibration use')
        raw, stamp = self.robot.udp.receive_with_timestamp()
        if raw is None or stamp is None or not 0 <= time.monotonic()-stamp <= .25:
            raise ValueError('Missing fresh measured gripper aperture')
        opening = np.asarray(raw.gripper_joint_position[1], float).reshape(-1)
        if (opening.size != 1 or not np.isfinite(opening).all() or
                abs(opening[0]-self.c['calibrated_gripper_position']) > .01):
            raise ValueError('Gripper opening differs from the measured pad calibration')
        if np.rad2deg(Rotation.from_matrix(self.rotation.T @ tool_rotation(s)).magnitude()) > self.c['orientation_error_deg']:
            raise ValueError('Tool orientation drift during motion/observation')
        if self.current_segment_start is not None:
            if np.linalg.norm(tcp-self.current_segment_start) > self.c['step_mm']+2*self.c['arrival_mm']:
                raise ValueError('CuRobo path departed from V4.1 local motion envelope')
        self.check_observation(s, tcp)
        return s

    def check_observation(self, s, tcp):
        """Image/hand coupling used by the original finger observation task."""
        if self.observation_anchor is not None:
            velocity = np.asarray(s.get('joint_velocity', []), float)
            if velocity.shape != (14,) or not np.isfinite(velocity).all():
                raise ValueError('Invalid joint velocity during book observation; do not pair this image')
            speed = float(np.max(abs(velocity)))
            drift = float(np.linalg.norm(tcp-self.observation_anchor))
            debounced = self.c.get('observation_velocity_debounce',False)
            if (not debounced and speed > .005) or drift > .5:
                raise ValueError('Arm moved during book observation; do not pair this image: '
                    f'max_joint_velocity={speed:.6f}rad/s (limit=0.005000, '
                    f'joint_index_1based={int(np.argmax(abs(velocity)))+1}), '
                    f'right_tcp_drift={drift:.3f}mm (limit=0.500)')
            if debounced:
                from book_top_observation_motion import ObservationMotionGate
                if getattr(self,'_observed_anchor',None) is not self.observation_anchor:
                    self._observed_anchor=self.observation_anchor
                    self._observation_motion=ObservationMotionGate(s)
                self._observation_motion.check(s)
        else:
            self._observed_anchor=None

    def stationary_observation(self, vision):
        s = self.guard()
        previous_anchor = self.observation_anchor
        self.observation_anchor = pose_mm(s)
        acquisition_start_unix = time.time()
        try:
            obs = vision.stable(self.guard, min_stamp=acquisition_start_unix)
            final = self.guard()
            if time.time()-obs['captured_unix_s'] > self.c['frame_max_age_seconds']:
                raise ValueError('Book frame expired before target calculation')
            return obs, final
        finally:
            self.observation_anchor = previous_anchor

    def cancel(self):
        # Existing planner cancellation uses its freshest measured joints;
        # this is not a retreat, new grasp, or request to open either gripper.
        s = self.read()
        angles = Rotation.from_matrix(tool_rotation(s)).as_euler('xyz', degrees=True)
        result = self.robot.master_control(right_arm=[*pose_mm(s), *angles],
            action='book_v3_1_hold', object_category='book', wait_for_completion=False)
        self.log(dict(event='hold_requested', sent=result is True, confirmed=False))


def select_face(camera, path):
    rgb, depth = camera.capture_image()
    clicks = []
    scale = min(1., 1200/rgb.shape[1], 850/rgb.shape[0])
    title = 'V4.1 book face: top-left, top-right, bottom-right, bottom-left; Enter save; R clear; Esc cancel'
    display = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), None, fx=scale, fy=scale)
    cv2.namedWindow(title)
    def click(event, x, y, flags, data):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append([x/scale, y/scale])
    cv2.setMouseCallback(title, click)
    try:
        while True:
            image = display.copy()
            for i, p in enumerate(clicks):
                pt = tuple(np.rint(np.asarray(p)*scale).astype(int))
                cv2.circle(image, pt, 4, (0, 215, 255), -1)
                cv2.putText(image, str(i+1), pt, cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
            cv2.imshow(title, image)
            key = cv2.waitKey(30) & 255
            if key == 27:
                raise ValueError('Selection cancelled')
            if key in (ord('r'), ord('R')):
                clicks.clear()
            if key in (10, 13) and len(clicks) == 4:
                quad = validate_quad(clicks, rgb.shape)
                measure_face(quad, depth, camera.K, camera.D, camera.c2w)
                path.parent.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(path, rgb=rgb, quad=quad, K=camera.K, D=camera.D,
                                    c2w=camera.c2w, captured_unix_s=camera.captured_unix_s)
                print(f'V4.1 书面参考已保存：{path}')
                return
    finally:
        cv2.destroyWindow(title)


def recovery_consistent(old, new, c):
    if old is None:
        raise ValueError('No last valid observation for same-book recovery')
    for key, limit in [('top_base_mm', c['step_mm']+c['max_shape_error_mm']),
                       ('bottom_base_mm', c['max_pivot_drift_mm'])]:
        if np.linalg.norm(np.asarray(old[key])-new[key])>limit:
            raise ValueError('Recovered book geometry jumped; do not switch contact identity')
    a,b = np.asarray(old['corners_base_mm']),np.asarray(new['corners_base_mm'])
    if np.max(abs(np.linalg.norm(np.roll(a,-1,axis=0)-a,axis=1)-
                  np.linalg.norm(np.roll(b,-1,axis=0)-b,axis=1)))>c['max_shape_error_mm']:
        raise ValueError('Recovered book dimensions changed')


def pad_projection_hint(io, c, camera):
    if io is None: return None
    state=io.guard();pad=FingerPad(**c['pad'])
    corners=np.array([pad.center_tcp_mm+a*pad.along_tcp*pad.length_mm/2+
        b*pad.across_tcp*pad.width_mm/2 for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]])
    pixels=project_base(corners@tool_rotation(state).T+pose_mm(state),camera.K,camera.D,camera.c2w)
    return dict(camera='head', source='calibrated_prediction_not_measurement',
                pad_corners_pixels=pixels.tolist())


def run(args, c):
    from book_top_landing_runtime import run_landing
    return run_landing(args, c)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=CONFIG)
    p.add_argument('--selection', type=Path, help='Explicit saved reference; default selects from current image')
    p.add_argument('--target', help='Description of the target book; default from configuration')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--auto-select', action='store_true', help='Automatic GPT semantic selection and new-frame check; no robot I/O')
    mode.add_argument('--select', action='store_true', help='Click book face; no robot connection')
    mode.add_argument('--execute', action='store_true', help='Head coarse approach, wrist visual landing, then hold; no tilt')
    mode.add_argument('--teach-contact', action='store_true', help='Optional real contact pixel teaching; camera only, no movement')
    mode.add_argument('--check-config', action='store_true', help='Validate without devices or API calls')
    p.add_argument('--frames', type=int, default=30, help='Observation-only frame groups')
    p.add_argument('--output', type=Path)
    args = p.parse_args(argv)
    try:
        if args.frames < 1:
            raise ValueError('--frames must be positive')
        c = load_config(args.config, args.execute)
        if args.check_config:
            print('V4.1 参数格式有效。模式：head_coarse_wrist_landing；指腹来源：'+c.get('pad_source','auto_rgbd')+'；实机就绪尚需新帧几何验证。')
            return 0
        run(args, c)
        return 0
    except (ValueError, OSError, RuntimeError, KeyError) as exc:
        print(f'V4.1: {type(exc).__name__}: {exc}')
        return 2
    except KeyboardInterrupt:
        print('V4.1 已取消；不自动松手或复位。')
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
