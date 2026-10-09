"""Tilt-aware spine detection; reuse the 1.1 local motion/visualization flow.

    python cart_book_grasp_v2.py --assume-head-camera-calibrated

The original 1.1 entrypoint is unchanged. This process substitutes only its
spine geometry and detector adapter, and uses a separate hot-load config.
"""
from pathlib import Path

import cv2
import numpy as np

import cart_book_grasp_v2_motion as previous
from book_spine_wrist import base_tilt, load_wrist_config, j7_proposal, align_j7, hold_with_j7_undo

from book_spine_recovery_v1_1 import spine_axis_3d, DepthConsistencyError
from cart_book_grasp_v2_geometry import recover_stable_spine, base_z_compensation


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
    ui, vi = np.rint(center).astype(int)
    if not (0 <= vi < mask.shape[0] and 0 <= ui < mask.shape[1] and mask[vi, ui]):
        raise ValueError('Spine center falls outside the mask')
    # Sample the inner central patch in the same rotated frame. RGB, depth,
    # K and extrinsics all remain in their ORIGINAL camera coordinates.
    central = ((np.abs(t-middle) <= max(3., .1*length))
               & (np.abs(s-np.polyval(fit, t-middle)) <= max(1., .25*width)))
    coords = pixels[central].astype(int)
    samples = depth[coords[:, 1], coords[:, 0]]
    samples = samples[np.isfinite(samples) & (samples > 0)]
    if len(samples) < 12:
        raise ValueError('Too few valid depths near the spine center')
    if np.median(samples) > 20:
        samples *= .001
    p10, p90 = np.percentile(samples, [10, 90])
    center_depth_inconsistent = p90-p10 > .10
    k = np.asarray(intrinsic, dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all():
        raise ValueError('Invalid camera intrinsic matrix')
    ray = np.linalg.solve(k, [*center, 1.])
    if not np.isfinite(ray).all() or ray[2] <= 0:
        raise ValueError('Invalid spine center ray')
    z = float(np.median(samples))
    yy, xx = np.nonzero(mask)
    result = dict(mask=mask, box=np.array([xx.min(), yy.min(), xx.max()+1, yy.max()+1], dtype=np.float32),
                uv=center.tolist(), point_camera_m=ray/ray[2]*z, depth_m=z,
                depth_samples=len(samples), depth_p10_p90_mm=[p10*1000, p90*1000],
                spine_axis_uv=along.tolist(), spine_length_px=float(length), spine_width_px=width,
                spine_tilt_deg=float(np.degrees(np.arctan2(along[0], along[1]))))
    try:
        if center_depth_inconsistent:
            raise DepthConsistencyError('Spine center depth is mixed with background')
        result.update(spine_axis_3d(pixels, t, s, middle, length, width, fit, depth, k))
    except (ValueError, np.linalg.LinAlgError) as error:
        result['spine_3d_status'] = str(error)
        # Keep the original result unless this specific depth failure blocks
        # center validation or an enabled J7 alignment. Other errors never fall back.
        if isinstance(error, DepthConsistencyError) and (center_depth_inconsistent or load_wrist_config()['enabled']):
            result['spine_full_fit_failure'] = str(error)
            try:
                recover_stable_spine(result, pixels, origin, along, across, t, s,
                                     middle, length, width, fit, depth, k)
            except (ValueError, OSError, np.linalg.LinAlgError) as recovery_error:
                if center_depth_inconsistent:
                    raise ValueError(f'{error}; stable-region fallback failed: {recovery_error}') from recovery_error
                result['spine_center_recovery'] = f'Failed: {recovery_error}'
                print(f'cart+ 1.2 深度备用计算未通过：{recovery_error}', flush=True)
            else:
                result['spine_3d_status'] = 'valid'
                print(f"cart+ 1.2 原深度一致性检测失败，启用稳定区域（长度占比 {result['spine_stable_region_fraction']:.0%}）；"
                      f"使用该区域中心和倾角；仅按中心偏移补偿基座 Z。", flush=True)
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


def main(*, task_types=None, config_path=None, wrist_config_path=None):
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
            latest.update(base_z_compensation(selected, c2w))
            for key in ('book_surface_x_mm', 'whole_book_depth_median_mm',
                        'whole_book_depth_samples', 'whole_book_depth_valid_fraction',
                        'whole_book_base_x_p10_p90_mm'):
                latest[key] = selected[key]
            try:
                config = load_wrist_config()
                latest['wrist_enabled'] = config['enabled']
                if visualization is not None and visualization.robot is not None:
                    latest.update(j7_proposal(latest, visualization.robot.get_current_state(), config,
                        step5.RIGHT_ARM_SOFT_LOWER_DEG[6], step5.RIGHT_ARM_SOFT_UPPER_DEG[6]))
            except Exception as error:
                latest['wrist_status'] = str(error)
            result.update(latest)
        original_install(_TiltDetector(geometry), visualization, on_geometry=on_geometry)

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
            if (task.get('book_grasp_mode') == 'cart+' and task.get('arm') == 'right'
                    and task.get('action') == 'move_down_to_grasp'):
                task = dict(task, book_surface_x_mm=latest.get('book_surface_x_mm'))
            result = execute(robot, task, *args, **kwargs)
            if (task.get('book_grasp_mode') == 'cart+' and task.get('arm') == 'right'
                    and task.get('action') == 'move_above_grasp' and result[1]):
                return previous.execute_depth_z_compensation(robot, task, step5, execute,
                    latest, *args, **kwargs)
            return result

        step5.execute_single_task = execute_with_wrist

    saved = {name: getattr(previous, name) for name in
             ('_spine_center_from_mask', '_install_spine_center', '_install_right_arm_reset', '_CART_PLUS_CONFIG_PATH')}
    try:
        previous._spine_center_from_mask = _spine_center_from_mask
        previous._install_spine_center = install_spine
        previous._install_right_arm_reset = install_runtime
        previous._CART_PLUS_CONFIG_PATH = (Path(config_path).resolve() if config_path is not None
            else Path(__file__).resolve().with_name('cart_book_grasp_v2_config.json'))
        if wrist_config_path is not None:
            wrist.CONFIG_PATH = Path(wrist_config_path).resolve()
        print('cart+ v2：按整书深度动态前伸；记录抓取前 J7 的实际旋转量；抬升后在停顿期间从当前位置反向旋转同样角度，完成后再下降。', flush=True)
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
