"""Tilt-aware spine detection; reuse the 1.1 local motion/visualization flow.

    python book_grasp_1_2_local.py --assume-head-camera-calibrated

The original 1.1 entrypoint is unchanged. This process substitutes only its
spine geometry and detector adapter, and uses a separate hot-load config.
"""
from pathlib import Path
import json

import cv2
import numpy as np

import book_grasp_1_1_local as previous
from book_spine_wrist import spine_axis_3d, fit_spine_axis, base_tilt, load_wrist_config, j7_proposal, align_j7, hold_with_j7_undo

_GEOMETRY_CONFIG_PATH = Path(__file__).with_name('book_grasp_1_2_geometry_config.json')


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


def _recover_stable_spine(result, pixels, origin, along, across, t, s,
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


def _spine_frame(mask, shape=None):
    mask = np.asarray(mask, dtype=np.uint8)
    if mask.ndim != 2:
        raise ValueError('Invalid spine mask dimensions')
    if shape is not None and mask.shape != shape:
        mask = cv2.resize(mask, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count < 2:
        raise ValueError('Empty spine mask')
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = labels == component
    yy, xx = np.nonzero(mask)
    if len(xx) < 100:
        raise ValueError('Spine mask has fewer than 100 pixels')
    if xx.min() <= 1 or yy.min() <= 1 or xx.max() >= mask.shape[1]-2 or yy.max() >= mask.shape[0]-2:
        raise ValueError('Spine is clipped by the image border')
    pixels = np.column_stack((xx, yy)).astype(float)
    origin = pixels.mean(axis=0)
    _, vectors = np.linalg.eigh(np.cov((pixels-origin).T))
    along = vectors[:, -1]
    if along[1] < 0 or (abs(along[1]) < 1e-8 and along[0] < 0):
        along = -along
    across = np.array([-along[1], along[0]])
    longitudinal = (pixels-origin) @ along
    lateral = (pixels-origin) @ across
    lo, hi = np.percentile(longitudinal, [1, 99])
    width = float(np.diff(np.percentile(lateral, [1, 99]))[0])
    length = hi-lo
    if length < 40 or width < 2 or length < 2*width:
        raise ValueError(f'Not an elongated spine: axial length={length:.1f}, width={width:.1f} px')
    return mask, pixels, origin, along, across, longitudinal, lateral, lo, hi, width


def _spine_center_from_mask(mask, depth, intrinsic):
    """Fit boundary midpoints in the spine's own axes, independent of tilt."""
    depth = np.asarray(depth, dtype=float)
    if depth.ndim != 2:
        raise ValueError('Invalid depth dimensions')
    mask, pixels, origin, along, across, t, s, lo, hi, width = _spine_frame(mask, depth.shape)
    middle = (lo+hi)/2
    length = hi-lo
    bin_ids = np.floor((t-lo)/2).astype(int)
    row_t, row_s = [], []
    for index in np.unique(bin_ids[np.abs(t-middle) <= .25*length]):
        member = bin_ids == index
        if member.sum() < 4:
            continue
        edge_lo, edge_hi = np.percentile(s[member], [5, 95])
        row_t.append(float(np.median(t[member])))
        row_s.append(float((edge_lo+edge_hi)/2))
    if len(row_t) < 8:
        raise ValueError('Too few spine cross-sections')
    row_t, row_s = np.asarray(row_t)-middle, np.asarray(row_s)
    fit = np.polyfit(row_t, row_s, 1)
    residual = np.abs(row_s-np.polyval(fit, row_t))
    good = residual <= max(2., 3*np.median(residual))
    if good.sum() < 8:
        raise ValueError('Unstable spine midline')
    fit = np.polyfit(row_t[good], row_s[good], 1)
    center = origin + middle*along + fit[1]*across
    k = np.asarray(intrinsic, dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all():
        raise ValueError('Invalid camera intrinsic matrix')
    yy, xx = np.nonzero(mask)
    result = dict(mask=mask, box=np.array([xx.min(), yy.min(), xx.max()+1, yy.max()+1], dtype=np.float32),
                uv=center.tolist(), spine_axis_uv=along.tolist(), spine_length_px=float(length),
                spine_width_px=width, spine_tilt_deg=float(np.degrees(np.arctan2(along[0], along[1]))))
    center_valid = False
    try:
        result.update(_center_measurement(mask, pixels, t, s, middle, middle, width, fit,
                      max(3., .1*length), center, depth, k, .04))
        center_valid = True
        result.update(spine_axis_3d(pixels, t, s, middle, length, width, fit, depth, k))
    except (ValueError, np.linalg.LinAlgError) as error:
        result['spine_3d_status'] = str(error)
        result['spine_full_fit_failure'] = str(error)
        try:
            _recover_stable_spine(result, pixels, origin, along, across, t, s,
                                  middle, length, width, fit, depth, k)
        except (ValueError, np.linalg.LinAlgError, OSError) as recovery_error:
            if not center_valid:
                raise ValueError(f'{error}; stable-region recovery failed: {recovery_error}') from recovery_error
            result['spine_center_recovery'] = f'Failed: {recovery_error}'
        else:
            result['spine_3d_status'] = 'valid'
            print(f"1.2 书脊局部修正：自动选择连续稳定区域（书脊长度的 {result['spine_stable_region_fraction']:.0%}），"
                  f"使用该区域中心和倾角，中心深度 {result['depth_m']*1000:.1f} mm。", flush=True)
    return result


def _rotate_for_detection(rgb, angle):
    """Expand the canvas so rotation never crops the book before detection."""
    height, width = rgb.shape[:2]
    matrix = cv2.getRotationMatrix2D(((width-1)/2, (height-1)/2), angle, 1.)
    cos, sin = abs(matrix[0, 0]), abs(matrix[0, 1])
    new_width = int(np.ceil(width*cos + height*sin))
    new_height = int(np.ceil(height*cos + width*sin))
    matrix[:, 2] += [(new_width-width)/2, (new_height-height)/2]
    return cv2.warpAffine(rgb, matrix, (new_width, new_height)), matrix


def _restore_detections(detections, matrix, rotated_shape, original_shape):
    restored = []
    inverse = cv2.invertAffineTransform(matrix)
    for score, mask, _ in detections:
        mask = np.asarray(mask, dtype=np.uint8)
        if mask.ndim != 2:
            continue
        if mask.shape != rotated_shape:
            mask = cv2.resize(mask, (rotated_shape[1], rotated_shape[0]), interpolation=cv2.INTER_NEAREST)
        mask = cv2.warpAffine(mask, inverse, (original_shape[1], original_shape[0]), flags=cv2.INTER_NEAREST).astype(bool)
        yy, xx = np.nonzero(mask)
        if len(xx):
            restored.append((score, mask, np.array([xx.min(), yy.min(), xx.max()+1, yy.max()+1], dtype=np.float32)))
    return restored


def _has_spine(detections):
    for score, mask, _ in detections:
        if not np.isfinite(score) or score < .10:
            continue
        try:
            _spine_frame(mask)
            return True
        except (ValueError, np.linalg.LinAlgError):
            pass
    return False


class _TiltDetector:
    """Forward geometry state to the original module; adapt only spine SAM3."""
    def __init__(self, geometry):
        object.__setattr__(self, '_geometry', geometry)

    def __getattr__(self, name):
        return getattr(self._geometry, name)

    def __setattr__(self, name, value):
        setattr(self._geometry, name, value)

    def _sam3_raw_books(self, processor, to_numpy, image, prompt='book'):
        raw = self._geometry._sam3_raw_books
        detections = raw(processor, to_numpy, image, prompt=prompt)
        if prompt != 'book spine' or _has_spine(detections):
            return detections
        rgb = np.asarray(image)
        _, _, image_cls = self._geometry._load_sam3_detector()
        notes = []
        for angle in (-45, 45, 90):
            rotated, matrix = _rotate_for_detection(rgb, angle)
            try:
                candidates = raw(processor, to_numpy, image_cls.fromarray(rotated), prompt=prompt)
                candidates = _restore_detections(candidates, matrix, rotated.shape[:2], rgb.shape[:2])
            except Exception as error:
                notes.append(f'{angle} deg: {error}')
                continue
            if _has_spine(candidates):
                print(f'1.2 书脊检测：旋转 {angle}° 后检出，掩码已映射回原始相机图像。', flush=True)
                return candidates
        if notes:
            print('1.2 旋转检测未完成：' + '; '.join(notes), flush=True)
        return detections


def main(*, task_types=None, config_path=None, wrist_config_path=None, detector_factory=None):
    import book_spine_wrist as wrist
    saved_wrist_config = wrist.CONFIG_PATH
    original_install = previous._install_spine_center
    original_runtime = previous._install_right_arm_reset
    latest = {}
    rotations = {}
    display = [None]

    def install_spine(geometry, visualization=None):
        display[0] = visualization
        def on_geometry(selected, c2w, result):
            import step5_execute as step5
            latest.clear()
            rotations.clear()
            latest.update(base_tilt(selected, c2w))
            for key in ('spine_center_recovery', 'spine_center_shift_px', 'spine_stable_region_uv',
                        'spine_stable_region_fraction', 'spine_original_center_uv', 'spine_full_fit_failure'):
                if key in selected:
                    result[key] = selected[key]
            try:
                config = load_wrist_config()
                latest['wrist_enabled'] = config['enabled']
                if visualization is not None and visualization.robot is not None:
                    latest.update(j7_proposal(latest, visualization.robot.get_current_state(), config,
                        step5.RIGHT_ARM_SOFT_LOWER_DEG[6], step5.RIGHT_ARM_SOFT_UPPER_DEG[6]))
            except Exception as error:
                latest['wrist_status'] = str(error)
            result.update(latest)
        detector = (_TiltDetector(geometry) if detector_factory is None
                    else detector_factory(geometry, visualization))
        original_install(detector, visualization, on_geometry=on_geometry)

    def install_runtime():
        original_runtime()
        import step5_execute as step5
        execute = step5.execute_single_task

        def execute_with_wrist(robot, task, *args, **kwargs):
            if (task.get('book_grasp_mode') == 'cart+' and task.get('arm') == 'right'
                    and task.get('action') == 'wait_after_lift'):
                try:
                    parameters = previous._load_cart_plus_parameters()
                    config = load_wrist_config()
                    record = rotations.get(task.get('object'))
                    if not record or not record.get('complete'):
                        raise ValueError('本轮没有已确认的 J7 旋转记录')
                    inverse_deg = -record['delta_deg']
                    print(f"1.2 抬升后保持夹紧，J7 从当前位置反向转 {inverse_deg:+g}°；转动时间计入 {parameters['hold_seconds']:g} 秒停顿。", flush=True)
                    if display[0] is not None:
                        display[0].safe('log_text_document', 'book/wrist',
                            f'After lift: keep gripping. Undo the measured pre-grasp J7 rotation: {inverse_deg:+g} deg from the current joint angle.\n\nRotation time counts toward the hold. Descent requires confirmed completion.')
                    result = hold_with_j7_undo(robot, parameters['hold_seconds'], record,
                        step5.RIGHT_ARM_SOFT_LOWER_DEG[6], step5.RIGHT_ARM_SOFT_UPPER_DEG[6],
                        config['limit_margin_deg'])
                    if not result:
                        print('1.2 J7 转正未确认完成，取消下降和松手。')
                    if display[0] is not None:
                        display[0].safe('log_text_document', 'book/wrist',
                            f'Inverse J7 rotation ({inverse_deg:+g} deg) confirmed; hold complete. Ready to lower.' if result else
                            'J7 return failed. Descent and release cancelled; keep gripping.')
                    return task.get('object'), result
                except Exception as error:
                    print(f'1.2 抬升后转正失败：{error}；取消下降和松手。')
                    return task.get('object'), False
            if (task.get('book_grasp_mode') == 'cart+' and task.get('arm') == 'right'
                    and task.get('action') == 'move_above_grasp'):
                try:
                    config = load_wrist_config()
                    record = {}
                    rotations[task.get('object')] = record
                    if not align_j7(robot, latest, config, step5.RIGHT_ARM_SOFT_LOWER_DEG[6],
                                    step5.RIGHT_ARM_SOFT_UPPER_DEG[6], rotation_record=record):
                        print('1.2 J7 调整未确认完成，取消后续抓取。')
                        return task.get('object'), False
                except Exception as error:
                    print(f'1.2 J7 调整已取消：{error}')
                    return task.get('object'), False
            return execute(robot, task, *args, **kwargs)

        step5.execute_single_task = execute_with_wrist

    saved = {name: getattr(previous, name) for name in
             ('_spine_center_from_mask', '_install_spine_center', '_install_right_arm_reset', '_CART_PLUS_CONFIG_PATH')}
    try:
        previous._spine_center_from_mask = _spine_center_from_mask
        previous._install_spine_center = install_spine
        previous._install_right_arm_reset = install_runtime
        previous._CART_PLUS_CONFIG_PATH = (Path(config_path).resolve() if config_path is not None
            else Path(__file__).resolve().with_name('book_grasp_1_2_local_config.json'))
        if wrist_config_path is not None:
            wrist.CONFIG_PATH = Path(wrist_config_path).resolve()
        print('1.2：记录抓取前 J7 的实际旋转量；抬升后在停顿期间从当前位置反向旋转同样角度，完成后再下降。', flush=True)
        if task_types is None:
            previous.main()
        else:
            previous.main(task_types=task_types)
    finally:
        wrist.CONFIG_PATH = saved_wrist_config
        for name, value in saved.items():
            setattr(previous, name, value)


if __name__ == '__main__':
    main()
