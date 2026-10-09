"""Pure geometry for the cart+ spine-centered TCP approach, in mm/degrees."""
import numpy as np


def tool_rotation(euler):
    angles = np.asarray(euler, dtype=float)
    if angles.shape != (3,) or not np.isfinite(angles).all():
        raise ValueError('Invalid tool Euler angles')
    x, y, z = np.deg2rad(angles)
    cx, cy, cz = np.cos([x, y, z])
    sx, sy, sz = np.sin([x, y, z])
    return (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
            @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]))


def segment_targets(start, end, step_mm):
    start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    if (start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all()
            or not np.isfinite(step_mm) or step_mm < 1):
        raise ValueError('Invalid Cartesian segment')
    distance = np.linalg.norm(end-start)
    if distance < 1e-6:
        return []
    count = int(np.ceil(distance/step_mm))
    return [start+(end-start)*i/count for i in range(1, count+1)]


def grasp_plan(start_mm, euler, center_mm, parameters):
    start, center = np.asarray(start_mm, dtype=float), np.asarray(center_mm, dtype=float)
    if start.shape != (3,) or center.shape != (3,) or not np.isfinite([start, center]).all():
        raise ValueError('A valid XYZ spine center and current TCP pose are required')
    # Verified physical right fingers extend along tool -Z (J7 rotates about Z).
    direction = -tool_rotation(euler)[:, 2]
    if direction[0] < .5:
        raise ValueError('Cart approach requires fingers pointing toward base +X')
    offset = parameters['fingertip_offset_mm']
    insertion = parameters['grasp_insertion_mm']
    clearance = parameters['pregrasp_clearance_mm']
    if not np.isfinite([offset, insertion, clearance]).all() or min(offset, insertion) < 0 or clearance <= 0:
        raise ValueError('Invalid fingertip offset, insertion, or pregrasp clearance')
    tip_grasp = center+insertion*direction
    grasp = tip_grasp-offset*direction
    pregrasp = center-(clearance+offset)*direction
    if start[0] > pregrasp[0]+1e-6:
        raise ValueError('Current TCP is too near the book for Y/Z alignment; return to first_reset')
    # Align Y/Z on a plane through current X, then follow the finger axis.
    aligned = pregrasp+direction*((start[0]-pregrasp[0])/direction[0])
    align = segment_targets(start, aligned, parameters['align_step_mm'])
    # The alignment action also opens the gripper; do not skip it at zero gap.
    if not align:
        align = [aligned.copy()]
    forward = (segment_targets(aligned, pregrasp, parameters['forward_step_mm'])
               + segment_targets(pregrasp, grasp, parameters['forward_step_mm']))
    return dict(start_mm=start.copy(), center_mm=center.copy(), aligned_mm=aligned,
                pregrasp_mm=pregrasp, grasp_mm=grasp, fingertip_grasp_mm=tip_grasp,
                direction=direction, rotation_deg=np.asarray(euler, dtype=float).copy(),
                align=align, forward=forward, parameters=dict(parameters))


def retreat_targets(start_mm, original_x_mm, direction, step_mm):
    start, direction = np.asarray(start_mm, dtype=float), np.asarray(direction, dtype=float)
    if direction.shape != (3,) or not np.isfinite(direction).all() or direction[0] < .5:
        raise ValueError('Invalid saved approach direction')
    if start[0] < original_x_mm-1.:
        raise ValueError('TCP is already behind the original approach X')
    distance = max(0., (start[0]-original_x_mm)/direction[0])
    return segment_targets(start, start-distance*direction, step_mm)
