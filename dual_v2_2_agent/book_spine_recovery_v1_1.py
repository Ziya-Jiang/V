"""Depth-only fallback for cart v1.1. Never imported by the original v1."""
from pathlib import Path
import json
import numpy as np

_GEOMETRY_CONFIG_PATH = Path(__file__).with_name('cart_book_grasp_v1_1_geometry_config.json')

class DepthConsistencyError(ValueError):
    """Measured depths, rather than geometry or missing data, are inconsistent."""

def fit_spine_axis(points):
    """Validate a measured 3D line with the same gates for all fit regions."""
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

def _center_measurement(mask, pixels, t, s, target_t, fit_middle, width, fit,
                        half_window, center, depth, k, max_spread):
    ui, vi = np.rint(center).astype(int)
    if not (0 <= vi < mask.shape[0] and 0 <= ui < mask.shape[1] and mask[vi, ui]):
        raise ValueError('Spine center falls outside the mask')
    selected = ((np.abs(t-target_t) <= half_window)
                & (np.abs(s-np.polyval(fit, t-fit_middle)) <= max(1., .25*width)))
    uv = pixels[selected].astype(int)
    samples = depth[uv[:, 1], uv[:, 0]]
    samples = samples[np.isfinite(samples) & (samples > 0)]
    if len(samples) < max(12, .5*len(uv)):
        raise ValueError('Too few valid depths near the spine center')
    if np.median(samples) > 20:
        samples = samples*.001
    p10, p90 = np.percentile(samples, [10, 90])
    if p90-p10 > max_spread:
        raise ValueError('Spine center depth is mixed with background')
    ray = np.linalg.solve(k, [*center, 1.])
    if not np.isfinite(ray).all() or ray[2] <= 0:
        raise ValueError('Invalid spine center ray')
    z = float(np.median(samples))
    return dict(uv=center.tolist(), point_camera_m=ray/ray[2]*z, depth_m=z,
                depth_samples=len(samples), depth_p10_p90_mm=[float(p10*1000), float(p90*1000)])

def recover_stable_spine(result, pixels, origin, along, across, t, s,
                           middle, length, width, fit, depth, k):
    """Choose the longest contiguous, measured stable interval, with no offset.

    Non-overlapping strips with missing/mixed depth and abrupt depth steps
    break continuity. Smooth depth gradients from a tilted book remain valid.
    Center and 3D tilt both use only the winning interval's measurements.
    """
    config = json.loads(_GEOMETRY_CONFIG_PATH.read_text())
    if (set(config) != {'enabled', 'min_region_fraction', 'max_depth_jump_mm'}
            or type(config['enabled']) is not bool):
        raise ValueError('Invalid stable-spine recovery configuration')
    for key, low, high in (('min_region_fraction', .25, .8), ('max_depth_jump_mm', 1., 40.)):
        value = config[key]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not np.isfinite(value) or not low <= value <= high):
            raise ValueError(f'Invalid stable-spine parameter: {key}')
    if not config['enabled']:
        raise ValueError('Stable-spine recovery disabled')
    # Leave an end margin to avoid caps; these are sampling bins, not targets.
    edges = np.linspace(middle-.45*length, middle+.45*length, 25)
    strips = []
    inner = np.abs(s-np.polyval(fit, t-middle)) <= max(1., .25*width)
    for left, right in zip(edges[:-1], edges[1:]):
        uv = pixels[inner & (t >= left) & (t < right)].astype(int)
        z = depth[uv[:, 1], uv[:, 0]]
        good = np.isfinite(z) & (z > 0)
        if good.sum() < max(8, .8*len(uv)):
            strips.append(None)
            continue
        z, uv = z[good], uv[good]
        if np.median(z) > 20:
            z = z*.001
        if np.ptp(np.percentile(z, [10, 90])) > .04:
            strips.append(None)
            continue
        ray = np.linalg.solve(k, [*np.median(uv, axis=0), 1.])
        strips.append(ray/ray[2]*np.median(z))
    # Longest supported region first; break ties by line residual, then distance
    # to the original center. Never join across a bad strip or a depth jump.
    for count in range(len(strips), 5, -1):
        candidates = []
        for start in range(len(strips)-count+1):
            stop = start+count
            left, right = edges[start], edges[stop]
            if (right-left)/length < config['min_region_fraction']:
                continue
            segment = strips[start:stop]
            if any(point is None for point in segment):
                continue
            points = np.asarray(segment)
            if np.any(np.abs(np.diff(points[:, 2]))*1000 > config['max_depth_jump_mm']):
                continue
            try:
                axis = fit_spine_axis(points)
                direction = np.asarray(axis['spine_axis_camera'])
                relative = points-points.mean(axis=0)
                if np.max(np.linalg.norm(relative-(relative@direction)[:, None]*direction, axis=1)) > .012:
                    continue
                region_t = (left+right)/2
                region = (t >= left) & (t < right)
                # Refit image midline within this same region; no upper/lower bias.
                bins = np.floor((t-left)/2).astype(int)
                row_t, row_s = [], []
                for index in np.unique(bins[region]):
                    member = region & (bins == index)
                    if member.sum() >= 4:
                        row_t.append(float(np.median(t[member]))-region_t)
                        row_s.append(float(np.mean(np.percentile(s[member], [5, 95]))))
                if len(row_t) < 8:
                    continue
                local_fit = np.polyfit(row_t, row_s, 1)
                if np.percentile(np.abs(np.asarray(row_s)-np.polyval(local_fit, row_t)), 90) > 2.:
                    continue
                center = origin+region_t*along+local_fit[1]*across
                measured = _center_measurement(result['mask'], pixels[region], t[region], s[region],
                    region_t, region_t, width, local_fit, max(3., .1*(right-left)),
                    center, depth, k, .04)
                relative_center = measured['point_camera_m']-points.mean(axis=0)
                if np.linalg.norm(relative_center-direction*np.dot(relative_center, direction)) > .012:
                    continue
                uv_ends = np.array([origin+v*along+np.polyval(local_fit, v-region_t)*across
                                    for v in (left, right)])
                local_axis = along+local_fit[0]*across
                local_axis /= np.linalg.norm(local_axis)
                candidates.append((axis['spine_axis_residual_mm'], abs(region_t-middle),
                    dict(**axis, **measured, spine_original_center_uv=result['uv'],
                         spine_center_recovery='stable_region', spine_stable_region_uv=uv_ends.tolist(),
                         spine_stable_region_fraction=float((right-left)/length),
                         spine_center_shift_px=float(region_t-middle),
                         spine_axis_uv=local_axis.tolist(),
                         spine_tilt_deg=float(np.degrees(np.arctan2(local_axis[0], local_axis[1]))))))
            except (ValueError, np.linalg.LinAlgError):
                continue
        if candidates:
            result.update(min(candidates, key=lambda item: item[:2])[2])
            return
    raise ValueError('No sufficiently long continuous stable-depth spine region')

def spine_axis_3d(pixels, t, s, middle, length, width, fit, depth, intrinsic):
    """Fit a line through depth medians of separated inner-spine strips."""
    points = []
    mixed_strips = 0
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
            mixed_strips += 1
            continue
        ray = np.linalg.solve(intrinsic, [*np.median(uv, axis=0), 1.])
        points.append(ray/ray[2]*np.median(z))
    if len(points) < 6:
        if mixed_strips and len(points)+mixed_strips >= 6:
            raise DepthConsistencyError('Too many mixed-depth strips for 3D spine tilt')
        raise ValueError('Too few reliable depth strips for 3D spine tilt')
    points = np.asarray(points)
    center = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points-center, full_matrices=False)
    axis = vt[0]
    projection = (points-center)@axis
    residual = np.linalg.norm(points-center-projection[:, None]*axis, axis=1)
    if np.ptp(projection) < .04:
        raise ValueError('3D spine line is too short')
    if singular[0] < 4*singular[1] or np.percentile(residual, 90) > .012:
        raise DepthConsistencyError('3D spine depths are inconsistent')
    endpoints = center + np.array([projection.min(), projection.max()])[:, None]*axis
    return dict(spine_axis_camera=axis.tolist(), spine_endpoints_camera_m=endpoints.tolist(),
                spine_axis_residual_mm=float(np.percentile(residual, 90)*1000),
                spine_depth_strip_count=len(points))
