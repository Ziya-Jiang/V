"""Bounded, planner-paced image servo in robot BASE Y/Z (mm).

No robot/camera imports. All observations and confirmed motions are injected.
The local Jacobian is measured anew, never guessed from camera axes.
"""
import json
from pathlib import Path
import time

import numpy as np


CONFIG_PATH = Path(__file__).with_name('cart_book_grasp_v3_servo.json')


class ServoError(ValueError):
    pass


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ServoError(f'Invalid {name}')
    return result.copy()


def load_config(path=CONFIG_PATH, *, require_reference=True):
    c = json.loads(Path(path).read_text(encoding='utf-8'))
    bounds = {
        'probe_mm': (.5, 5), 'max_step_mm': (.1, 5),
        'max_offset_mm': (1, 30), 'max_path_mm': (5, 150),
        'tolerance_px': (.5, 10), 'gain': (.05, .8),
        'damping_px_per_mm': (.01, 5), 'max_condition': (1, 30),
        'min_response_px_per_mm': (.05, 10), 'max_fit_error_px': (.1, 5),
        'max_frame_age_seconds': (.1, 5), 'max_seconds': (10, 300),
        'max_pose_error_mm': (.1, 2), 'max_orientation_error_deg': (.1, 3),
        'max_reference_gap_error_mm': (.1, 15),
        'max_reference_orientation_error_deg': (.1, 5),
        'min_surface_gap_mm': (60, 300), 'settle_seconds': (0, 2),
    }
    if set(c) != set(bounds) | {'reference', 'max_iterations', 'stable_frames'}:
        raise ServoError('Unexpected or missing V3 servo configuration fields')
    for key, (lo, hi) in bounds.items():
        if type(c[key]) not in (int, float) or not np.isfinite(c[key]) or not lo <= c[key] <= hi:
            raise ServoError(f'Invalid V3 setting: {key} ({lo}..{hi})')
    for key, lo, hi in [('max_iterations', 1, 30), ('stable_frames', 2, 5)]:
        if type(c[key]) is not int or not lo <= c[key] <= hi:
            raise ServoError(f'Invalid V3 setting: {key}')
    if c['max_offset_mm'] < c['probe_mm'] or c['max_path_mm'] < 8*c['probe_mm']:
        raise ServoError('Motion budget is smaller than the Jacobian probes')
    ref = c['reference']
    if ref is None:
        if require_reference:
            raise ServoError('V3 尚未设置实测抓取参考像素。请先按 V3 README 登记 reference；未启动机器人。')
        return c
    if not isinstance(ref, dict) or set(ref) != {'uv', 'surface_gap_mm', 'tool_euler_deg', 'device', 'width', 'height'}:
        raise ServoError('Invalid reference schema')
    uv = vector(ref['uv'], 2, 'reference uv')
    vector(ref['tool_euler_deg'], 3, 'reference tool Euler angles')
    if any(type(ref[k]) is not int or ref[k] < 1 for k in ('width', 'height')):
        raise ServoError('Invalid reference resolution')
    if not (0 <= uv[0] < ref['width'] and 0 <= uv[1] < ref['height']):
        raise ServoError('Reference pixel is outside the image')
    gap = ref['surface_gap_mm']
    if type(gap) not in (int, float) or not np.isfinite(gap) or gap < c['min_surface_gap_mm']:
        raise ServoError('Reference must be in free space before the book surface')
    if not isinstance(ref['device'], str) or not ref['device'].startswith('/dev/'):
        raise ServoError('Invalid reference camera device')
    return c


def fit_jacobian(displacements, pixel_displacements, config):
    """Fit delta_uv = J @ measured_delta_base_yz; include off-axis drift."""
    q, s = np.asarray(displacements, float), np.asarray(pixel_displacements, float)
    if q.shape != (4, 2) or s.shape != (4, 2) or not np.isfinite(q).all() or not np.isfinite(s).all():
        raise ServoError('Four finite measured Y/Z probes are required')
    if np.linalg.matrix_rank(q) != 2 or np.linalg.cond(q) > config['max_condition']:
        raise ServoError('Probe motions do not independently excite base Y and Z')
    j = np.linalg.lstsq(q, s, rcond=None)[0].T
    singular = np.linalg.svd(j, compute_uv=False)
    if singular[-1] < config['min_response_px_per_mm'] or singular[0]/singular[-1] > config['max_condition']:
        raise ServoError('Image Jacobian is weak or ill-conditioned; approach cancelled')
    residual = np.linalg.norm(q @ j.T-s, axis=1)
    if residual.max() > config['max_fit_error_px']:
        raise ServoError('Inconsistent image response during Jacobian sampling')
    return j


def correction(jacobian, error, config):
    j = np.asarray(jacobian, float)
    e = vector(error, 2, 'pixel error')
    if j.shape != (2, 2) or not np.isfinite(j).all():
        raise ServoError('Invalid Jacobian')
    delta = -config['gain'] * j.T @ np.linalg.solve(
        j @ j.T + config['damping_px_per_mm']**2*np.eye(2), e)
    length = np.linalg.norm(delta)
    if length > config['max_step_mm']:
        delta *= config['max_step_mm']/length
    return delta


def run_servo(observe, move, config, *, checkpoint=lambda: None,
              log=lambda event: None, clock=time.monotonic):
    """Observe returns {uv, yz_mm, captured_at}; move confirms absolute base YZ.

    Planner failure, stale frames or target loss propagate; no automatic retreat,
    gripper command or fallback to V2 is performed here.
    """
    started = clock()
    path = 0.
    origin = None
    last_stamp = -np.inf
    target = vector(config['reference']['uv'], 2, 'target uv')

    def check():
        checkpoint()
        if clock()-started > config['max_seconds']:
            raise ServoError('V3 servo time budget exceeded')

    def read():
        nonlocal last_stamp
        check()
        observation = observe()
        check()
        uv = vector(observation['uv'], 2, 'observed uv')
        yz = vector(observation['yz_mm'], 2, 'measured base YZ')
        stamp = float(observation['captured_at'])
        age = clock()-stamp
        if not np.isfinite(age) or age < 0 or age > config['max_frame_age_seconds']:
            raise ServoError('Stale wrist frame; no correction sent')
        if stamp <= last_stamp:
            raise ServoError('Repeated or out-of-order wrist frame')
        last_stamp = stamp
        if origin is not None and np.linalg.norm(yz-origin) > config['max_offset_mm']+config['max_pose_error_mm']:
            raise ServoError('Measured Y/Z left the local servo region')
        log(dict(event='observation', uv=uv.tolist(), yz_mm=yz.tolist(),
                 error_px=(uv-target).tolist()))
        return uv, yz

    def go(destination, measured):
        nonlocal path
        check()
        destination = vector(destination, 2, 'correction destination')
        distance = float(np.linalg.norm(destination-measured))
        if np.linalg.norm(destination-origin) > config['max_offset_mm']:
            raise ServoError('V3 correction exceeds total Y/Z offset limit')
        if path+distance > config['max_path_mm']:
            raise ServoError('V3 accumulated motion limit exceeded')
        # Calibration uses bounded probe-sized moves; corrections are smaller.
        if distance > max(config['probe_mm'], config['max_step_mm'])+2*config['max_pose_error_mm']:
            raise ServoError('V3 individual waypoint exceeds step limit')
        log(dict(event='move', yz_mm=destination.tolist()))
        if not move(destination):
            raise ServoError('V3 waypoint not confirmed; subsequent grasp cancelled')
        path += distance
        uv, actual = read()
        if np.linalg.norm(actual-destination) > config['max_pose_error_mm']:
            raise ServoError('Measured TCP did not reach the servo waypoint')
        return uv, actual

    uv, origin = read()
    origin = origin.copy()
    current = origin.copy()
    stable = 0
    # A correctly aligned image does not need exploratory motion.
    while np.linalg.norm(uv-target) <= config['tolerance_px']:
        stable += 1
        if stable >= config['stable_frames']:
            log(dict(event='converged', iterations=0, path_mm=path))
            return dict(iterations=0, path_mm=path, error_px=float(np.linalg.norm(uv-target)))
        uv, current = read()

    motions, pixels = [], []
    for axis in range(2):
        for sign in (-1, 1):
            baseline_uv, baseline_yz = read()
            goal = origin.copy()
            goal[axis] += sign*config['probe_mm']
            probe_uv, probe_yz = go(goal, baseline_yz)
            if np.linalg.norm(probe_yz-baseline_yz) < config['probe_mm']*.5:
                raise ServoError('Actual probe displacement is too small')
            motions.append(probe_yz-baseline_yz)
            pixels.append(probe_uv-baseline_uv)
            uv, current = go(origin, probe_yz)
            if np.linalg.norm(uv-baseline_uv) > config['max_fit_error_px']:
                raise ServoError('Target moved or tracking drifted during return probe')
    j = fit_jacobian(motions, pixels, config)
    log(dict(event='jacobian', pixels_per_base_mm=j.tolist()))
    previous_error = None
    increases = stable = 0
    for index in range(config['max_iterations']+config['stable_frames']):
        error = uv-target
        magnitude = float(np.linalg.norm(error))
        log(dict(event='iteration', index=index, error_px=magnitude))
        if magnitude <= config['tolerance_px']:
            stable += 1
            if stable >= config['stable_frames']:
                log(dict(event='converged', iterations=index, path_mm=path))
                return dict(iterations=index, path_mm=path, error_px=magnitude)
            uv, current = read()
            continue
        stable = 0
        if previous_error is not None and magnitude > previous_error+config['max_fit_error_px']:
            increases += 1
        else:
            increases = 0
        if increases >= 2:
            raise ServoError('Pixel error increased twice; no further descent/approach')
        previous_error = magnitude
        if index >= config['max_iterations']:
            break
        uv, current = go(current+correction(j, error, config), current)
    raise ServoError('V3 pixel error did not converge; approach cancelled')


def point_image_jacobian(uv, intrinsic, depth_m):
    """Robot Academy point interaction matrix, for offline/calibrated use.

    Maps CAMERA [vx, vy, vz (m/s), wx, wy, wz (rad/s)] to [du,dv] px/s.
    The V3 runtime uses the measured BASE-YZ restriction instead, since this
    camera has no intrinsics/hand-eye calibration yet. Never feed base velocity
    directly into this camera-frame matrix.
    https://robotacademy.net.au/lesson/the-image-jacobian/
    """
    u, v = vector(uv, 2, 'point pixel')
    k = np.asarray(intrinsic, float)
    if (k.shape != (3, 3) or not np.isfinite(k).all() or
            k[0, 0] <= 0 or k[1, 1] <= 0 or
            not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1])+abs(k[1, 0]) > 1e-9):
        raise ServoError('Expected a finite zero-skew pinhole intrinsic matrix')
    if not np.isfinite(depth_m) or depth_m <= 0:
        raise ServoError('Point depth must be positive camera-Z in meters')
    x, y = (u-k[0, 2])/k[0, 0], (v-k[1, 2])/k[1, 1]
    return np.diag([k[0, 0], k[1, 1]]) @ np.array([
        [-1/depth_m, 0, x/depth_m, x*y, -(1+x*x), y],
        [0, -1/depth_m, y/depth_m, 1+y*y, -x*y, -x]])
