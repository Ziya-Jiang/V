"""Whole-book depth and base-X-only approach; never aligns to absolute book Z."""
import numpy as np


def whole_book_depth(mask, depth, intrinsic, c2w):
    mask = np.asarray(mask, dtype=bool)
    depth = np.asarray(depth, dtype=float)
    k = np.asarray(intrinsic, dtype=float)
    transform = np.asarray(c2w, dtype=float)
    if mask.ndim != 2 or depth.shape != mask.shape:
        raise ValueError('Full-book mask must match the aligned depth image')
    if k.shape != (3, 3) or transform.shape != (4, 4) or not np.isfinite(k).all() or not np.isfinite(transform).all():
        raise ValueError('Invalid calibrated camera-to-base geometry')
    valid = mask & np.isfinite(depth) & (depth > 0)
    count, total = int(valid.sum()), int(mask.sum())
    if count < 100 or count < .5*total:
        raise ValueError('Too few valid depths in the full-book mask (need 100 pixels and 50% coverage)')
    yy, xx = np.nonzero(valid)
    z = depth[yy, xx].copy()
    if np.median(z) > 20:
        z *= .001
    rays = np.linalg.solve(k, np.array([xx, yy, np.ones(count)]))
    if not np.isfinite(rays).all() or np.any(rays[2] <= 0):
        raise ValueError('Invalid full-book backprojection')
    camera_points = (rays / rays[2] * z).T
    translation = transform[:3, 3].copy()
    if np.linalg.norm(translation) > 10:
        translation *= .001
    points = camera_points @ transform[:3, :3].T + translation
    # Median along the actual advance axis; camera Z is not robot-base X.
    x_mm = points[:, 0]*1000.
    if not np.isfinite(x_mm).all():
        raise ValueError('Invalid full-book base X depths')
    median_x = float(np.median(x_mm))
    # A median of an even-sized cloud may lie between samples. Mark the
    # nearest measured pixel, choosing a central pixel for tied depths.
    error = np.abs(x_mm-median_x)
    candidates = np.flatnonzero(np.isclose(error, error.min(), rtol=0., atol=1e-8))
    center_uv = np.median(np.column_stack((xx, yy)), axis=0)
    central_distance = (xx[candidates]-center_uv[0])**2 + (yy[candidates]-center_uv[1])**2
    marker_index = candidates[np.argmin(central_distance)]
    return dict(book_surface_x_mm=median_x,
        whole_book_median_uv=[int(xx[marker_index]), int(yy[marker_index])],
        whole_book_median_pixel_depth_mm=float(z[marker_index]*1000.),
        whole_book_depth_median_mm=float(np.median(z)*1000.),
        whole_book_depth_samples=count, whole_book_depth_valid_fraction=count/total,
        whole_book_base_x_p10_p90_mm=np.percentile(x_mm, [10, 90]).tolist())


def tool_rotation(euler):
    angles = np.asarray(euler, dtype=float)
    if angles.shape != (3,) or not np.isfinite(angles).all():
        raise ValueError('Invalid tool Euler angles')
    x, y, z = np.deg2rad(angles)
    cx, cy, cz = np.cos([x, y, z]); sx, sy, sz = np.sin([x, y, z])
    return (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
            @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]))


def forward_parameters(start_mm, euler, surface_x_mm, parameters):
    start = np.asarray(start_mm, dtype=float)
    if start.shape != (3,) or not np.isfinite(start).all() or surface_x_mm is None:
        raise ValueError('Current TCP pose and full-book base X median are required')
    surface = float(surface_x_mm)
    offset = float(parameters['fingertip_offset_mm'])
    insertion = float(parameters['grasp_insertion_mm'])
    if not np.isfinite([surface, offset, insertion]).all() or min(offset, insertion) < 0:
        raise ValueError('Invalid book depth, fingertip offset or insertion')
    # Physical fingers extend along tool -Z. Only its X projection is used;
    # do not follow the finger vector or alter base Y/Z during the approach.
    finger_x = float(-tool_rotation(euler)[0, 2])
    if finger_x < .5:
        raise ValueError('Fingers must point toward base +X before approach')
    fingertip_x_offset = offset*finger_x
    goal_x = surface + insertion - fingertip_x_offset
    distance = goal_x-start[0]
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError('Depth-derived target is not ahead of the current TCP; no forward motion')
    return dict(parameters, forward_mm=float(distance), book_surface_x_mm=surface,
                fingertip_x_offset_mm=fingertip_x_offset, grasp_tcp_x_mm=float(goal_x))
