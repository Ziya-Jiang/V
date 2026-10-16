"""Spine 3D inclination and optional right J7 alignment, independent of image roll."""
import json
import time
from pathlib import Path
import numpy as np

CONFIG_PATH = Path(__file__).with_name('book_grasp_1_2_wrist_config.json')


def load_wrist_config():
    values = json.loads(CONFIG_PATH.read_text())
    if set(values) != {'enabled', 'max_delta_deg', 'deadband_deg', 'limit_margin_deg'}:
        raise ValueError('Invalid wrist configuration fields')
    if type(values['enabled']) is not bool:
        raise ValueError('wrist enabled must be a boolean')
    for key in ('max_delta_deg', 'deadband_deg', 'limit_margin_deg'):
        if isinstance(values[key], bool) or not isinstance(values[key], (int, float)) or not np.isfinite(values[key]):
            raise ValueError(f'Invalid wrist parameter: {key}')
    if not (0 < values['max_delta_deg'] <= 90 and 0 <= values['deadband_deg'] < values['max_delta_deg']
            and 0 < values['limit_margin_deg'] <= 30):
        raise ValueError('Wrist parameters outside allowed ranges')
    return values


def spine_axis_3d(pixels, t, s, middle, length, width, fit, depth, intrinsic):
    """Fit a line through depth medians of separated inner-spine strips."""
    points = []
    for offset in np.linspace(-.35, .35, 9)*length:
        selected = ((np.abs(t-middle-offset) <= max(2., .04*length))
                    & (np.abs(s-np.polyval(fit, t-middle)) <= max(1., .25*width)))
        uv = pixels[selected].astype(int)
        if len(uv) < 8:
            continue
        z = depth[uv[:, 1], uv[:, 0]].astype(float)
        good = np.isfinite(z) & (z > 0)
        if good.sum() < 8:
            continue
        z, uv = z[good], uv[good]
        if np.median(z) > 20:
            z *= .001
        if np.ptp(np.percentile(z, [10, 90])) > .04:
            continue
        ray = np.linalg.solve(intrinsic, [*np.median(uv, axis=0), 1.])
        points.append(ray/ray[2]*np.median(z))
    if len(points) < 6:
        raise ValueError('Too few reliable depth strips for 3D spine tilt')
    points = np.asarray(points)
    center = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points-center, full_matrices=False)
    axis = vt[0]
    projection = (points-center)@axis
    residual = np.linalg.norm(points-center-projection[:, None]*axis, axis=1)
    if (np.ptp(projection) < .04 or singular[0] < 4*singular[1]
            or np.percentile(residual, 90) > .012):
        raise ValueError('3D spine line is too short or inconsistent')
    endpoints = center + np.array([projection.min(), projection.max()])[:, None]*axis
    return dict(spine_axis_camera=axis.tolist(), spine_endpoints_camera_m=endpoints.tolist(),
                spine_axis_residual_mm=float(np.percentile(residual, 90)*1000),
                spine_depth_strip_count=len(points))


def base_tilt(selected, c2w):
    output = dict(spine_image_tilt_deg=selected['spine_tilt_deg'])
    if 'spine_axis_camera' not in selected:
        output['spine_3d_status'] = selected.get('spine_3d_status', '3D tilt unavailable')
        return output
    transform = np.asarray(c2w, dtype=float)
    axis = transform[:3, :3] @ np.asarray(selected['spine_axis_camera'])
    axis /= np.linalg.norm(axis)
    if axis[2] < 0:
        axis = -axis
    translation = transform[:3, 3]
    if np.linalg.norm(translation) > 10:
        translation = translation*.001
    endpoints = np.asarray(selected['spine_endpoints_camera_m'])@transform[:3, :3].T + translation
    output.update(spine_axis_base=axis.tolist(), spine_endpoints_base_m=endpoints.tolist(),
                  spine_vertical_tilt_deg=float(np.degrees(np.arccos(np.clip(axis[2], -1, 1)))),
                  spine_3d_status='valid', spine_axis_residual_mm=selected['spine_axis_residual_mm'])
    return output


def quaternion_matrix(wxyz):
    q = np.asarray(wxyz, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid right tool quaternion')
    w, x, y, z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class J7GeometryUnavailable(ValueError):
    """A measured spine cannot be aligned by the optional J7-only rotation."""


def j7_proposal(geometry, state, config, lower_deg, upper_deg):
    if geometry.get('spine_3d_status') != 'valid':
        raise ValueError('Reliable 3D spine tilt required for J7 alignment')
    joints = np.asarray(state['joint_state'], dtype=float).reshape(2, 7)
    if not np.isfinite(joints).all():
        raise ValueError('Invalid joint state')
    rotation = quaternion_matrix(state['right_arm']['orientation'])
    axis = np.asarray(geometry['spine_axis_base'], dtype=float)
    roll_axis, reference = rotation[:, 2], rotation[:, 1]
    projected = axis-roll_axis*np.dot(axis, roll_axis)
    norm = np.linalg.norm(projected)
    if not np.isfinite(norm):raise ValueError('Non-finite spine projection for J7 alignment')
    if norm < .5:
        raise J7GeometryUnavailable('Spine is too parallel to the finger approach axis for J7-only alignment')
    projected /= norm
    delta = np.degrees(np.arctan2(np.dot(roll_axis, np.cross(reference, projected)), np.dot(reference, projected)))
    delta = float((delta+90) % 180-90)  # Spine is an unoriented line.
    if abs(delta) > config['max_delta_deg']:
        raise J7GeometryUnavailable(f'Required J7 delta {delta:.1f} deg exceeds configured maximum')
    target = np.rad2deg(joints[1]).copy()
    target[6] += delta
    if not lower_deg+config['limit_margin_deg'] <= target[6] <= upper_deg-config['limit_margin_deg']:
        raise ValueError('Proposed J7 target violates its soft-limit margin')
    return dict(j7_current_deg=float(np.rad2deg(joints[1, 6])), j7_delta_deg=delta,
                j7_target_deg=float(target[6]), right_target_deg=target.tolist(),
                wrist_enabled=config['enabled'], wrist_status='ready')


def align_j7(robot, geometry, config, lower_deg, upper_deg, rotation_record=None):
    """Optional joint adjustment at the ready pose, before lateral alignment."""
    if rotation_record is not None:
        rotation_record.clear()
        rotation_record.update(complete=False, delta_deg=0., undone=False)
    if not config['enabled']:
        if rotation_record is not None:
            rotation_record['complete'] = True
        return True
    state = robot.get_current_state()
    try:
        proposal = j7_proposal(geometry, state, config, lower_deg, upper_deg)
    except J7GeometryUnavailable as error:
        if not config.get('skip_reviewed_geometry_unavailable',False):raise
        if rotation_record is not None:
            rotation_record.update(complete=True,delta_deg=0.,skip_reason=str(error))
        print(f'GPT 复核后 J7 单轴预对齐仍不可用：{error}；跳过预旋转，保持当前姿态。',flush=True)
        return True
    measured = np.rad2deg(np.asarray(state['joint_state']).reshape(2, 7)[1])
    saved = np.asarray(robot.right_safe_joint_deg, dtype=float).copy()
    if np.max(np.abs(measured[:6]-saved[:6])) > 3.:
        raise ValueError('J7 alignment requires the right arm to be at first_reset')
    if abs(proposal['j7_delta_deg']) <= config['deadband_deg']:
        if rotation_record is not None:
            rotation_record['complete'] = True
        return True
    return _command_right_j7(robot, state, proposal['j7_target_deg'], 'book_align_j7', rotation_record)


def _command_right_j7(robot, state, target_j7_deg, action, rotation_record=None):
    """Preserve the other joint targets and gripper command while rotating J7."""
    before = np.rad2deg(np.asarray(state['joint_state'], dtype=float).reshape(2, 7))
    if not np.isfinite(before).all():
        raise ValueError('Invalid joint state for J7 rotation')
    target = before[1].copy()
    target[6] = target_j7_deg
    saved = np.asarray(robot.right_safe_joint_deg, dtype=float).copy()
    try:
        # Reuse the existing joint planner/wait path, changing only J7 in the
        # measured target. Restore the ready pose even on planner failure.
        robot.right_safe_joint_deg = target
        success = robot.control_joint_position(left_pos=False, right_pos=True, action=action)
    finally:
        robot.right_safe_joint_deg = saved
    if not success:
        return False
    after = robot.get_current_state()
    measured_after = np.rad2deg(np.asarray(after['joint_state']).reshape(2, 7))
    confirmed = bool(np.isfinite(measured_after).all()
                     and np.max(np.abs(measured_after[0]-before[0])) <= 2.
                     and np.max(np.abs(measured_after[1]-target)) <= 2.)
    if confirmed and rotation_record is not None:
        rotation_record.update(complete=True, before_deg=float(before[1, 6]),
                               after_deg=float(measured_after[1, 6]),
                               delta_deg=float(measured_after[1, 6]-before[1, 6]))
    return confirmed


def hold_with_j7_undo(robot, hold_seconds, rotation_record, lower_deg, upper_deg, margin_deg=5.):
    """Undo this pick's measured J7 rotation relative to the lifted joint angle.

    If motion takes longer than the hold, await confirmed completion rather
    than letting a timer trigger descent while the wrist is still rotating.
    """
    if not np.isfinite(hold_seconds) or hold_seconds < 0:
        raise ValueError('Invalid hold duration')
    if not rotation_record or not rotation_record.get('complete'):
        raise ValueError('No confirmed J7 rotation recorded for this pick')
    if rotation_record.get('undo_started') or rotation_record.get('undone'):
        raise ValueError('This pick\'s inverse rotation was already attempted')
    delta = float(rotation_record['delta_deg'])
    if not np.isfinite(delta):
        raise ValueError('Invalid recorded J7 rotation')
    start = time.monotonic()
    if abs(delta) > 1e-8:
        state = robot.get_current_state()
        if state is None or 'joint_state' not in state:
            raise ValueError('No joint state for inverse J7 rotation after lift')
        current_j7 = float(np.rad2deg(np.asarray(state['joint_state']).reshape(2, 7)[1, 6]))
        target_j7 = current_j7-delta
        if not lower_deg+margin_deg <= target_j7 <= upper_deg-margin_deg:
            raise ValueError('Inverse J7 target violates the configured soft-limit margin')
        rotation_record.update(undo_started=True, undo_target_deg=target_j7)
        if not _command_right_j7(robot, state, target_j7, 'book_hold_undo_j7'):
            return False
    remaining = max(0., hold_seconds-(time.monotonic()-start))
    if remaining:
        getattr(robot, 'operator_sleep', time.sleep)(remaining)
    rotation_record['undone'] = True
    return True
