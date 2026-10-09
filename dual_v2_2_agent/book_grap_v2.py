"""Local Orbbec book grasp, with a measured fingertip approach for cart+.

    python book_grap_v2.py --assume-head-camera-calibrated
    python book_grap_v2.py --assume-head-camera-calibrated --book-preview-only

Operator measurements: tool origin is 50 mm behind the fingertips, fully
open inner jaw gap is 60 mm. Offsets are along physical finger +Z = tool -Z.
Preview mode skips the pre-capture reset and every execution command.
The visible point cloud cannot recover an occluded cover or validate camera
extrinsics. This is a bounded approach, not a full collision checker.
"""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys

import numpy as np


# The configured right-arm ready pose points at about +10.1 degrees in XY.
# Allow its small yaw offset; jaw-width and near-edge clearance gates still apply.
MAX_APPROACH_YAW_ERROR_DEG = 15.0


@dataclass(frozen=True)
class Settings:
    fingertip_offset_mm: float = 50.0
    open_gap_mm: float = 60.0
    side_margin_mm: float = 8.0
    insertion_mm: float = 10.0
    clearance_mm: float = 40.0
    approach_step_mm: float = 5.0
    min_score: float = 0.20
    preview_only: bool = False


def parse_options(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--book-fingertip-offset-mm', type=float, default=50.0)
    parser.add_argument('--book-open-gap-mm', type=float, default=60.0)
    parser.add_argument('--book-side-margin-mm', type=float, default=8.0)
    parser.add_argument('--book-insertion-mm', type=float, default=10.0)
    parser.add_argument('--book-clearance-mm', type=float, default=40.0)
    parser.add_argument('--book-approach-step-mm', type=float, default=5.0)
    parser.add_argument('--book-min-score', type=float, default=0.20)
    parser.add_argument('--book-preview-only', action='store_true')
    args, rest = parser.parse_known_args(argv)
    values = {name.removeprefix('book_'): value for name, value in vars(args).items()}
    options = Settings(**values)
    for name in ('fingertip_offset_mm', 'open_gap_mm', 'side_margin_mm',
                 'insertion_mm', 'clearance_mm', 'approach_step_mm', 'min_score'):
        if not np.isfinite(getattr(options, name)):
            parser.error(f'{name} 必须是有限数值')
    if options.fingertip_offset_mm < 0 or options.open_gap_mm <= 0:
        parser.error('指尖偏移不能为负，张开间距必须大于零')
    if options.side_margin_mm < 5 or 2 * options.side_margin_mm >= options.open_gap_mm:
        parser.error('每侧余量至少 5 mm，且两侧余量之和必须小于张开间距')
    if not 0 < options.insertion_mm <= 20:
        parser.error('插入深度必须在 (0, 20] mm 内')
    if options.clearance_mm < 20 or not 0 < options.approach_step_mm <= 10:
        parser.error('预接近余量至少 20 mm，接近步长必须在 (0, 10] mm 内')
    if not 0 < options.min_score <= 1:
        parser.error('分割置信度必须在 (0, 1] 内')
    return options, rest


def estimate_geometry(points_base_mm):
    """Observed bounds, not a surface-density-weighted median center."""
    points = np.asarray(points_base_mm, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 80:
        raise ValueError('书本有效深度点少于 80，取消接近')
    low, high = np.percentile(points, [2, 98], axis=0)
    points = points[np.all((points >= low - 5) & (points <= high + 5), axis=1)]
    horizontal = points[:, :2]
    eigenvalues, eigenvectors = np.linalg.eigh(np.cov(horizontal.T))
    if eigenvalues[1] <= 0 or eigenvalues[1] < 4 * eigenvalues[0]:
        raise ValueError('书的长边方向不明确，可能混入背景或多本书')
    along = eigenvectors[:, 1]
    if along[0] < 0:
        along = -along
    across = np.array([-along[1], along[0]])
    length_bounds = np.percentile(horizontal @ along, [2, 98])
    width_bounds = np.percentile(horizontal @ across, [2, 98])
    z_bounds = np.percentile(points[:, 2], [2, 98])
    thickness = float(np.diff(width_bounds)[0])
    length = float(np.diff(length_bounds)[0])
    height = float(np.diff(z_bounds)[0])
    if not (8 <= thickness <= 95 and 60 <= length <= 400 and 50 <= height <= 400):
        raise ValueError(f'可见书体尺寸不可信：厚 {thickness:.1f}、长 {length:.1f}、高 {height:.1f} mm')
    center_xy = along * np.mean(length_bounds) + across * np.mean(width_bounds)
    return {
        'center_base_mm': [*center_xy.tolist(), float(np.mean(z_bounds))],
        'thickness_mm': thickness, 'length_mm': length, 'height_mm': height,
        'approach_xy': along.tolist(), 'z_bounds_mm': z_bounds.tolist(),
        'points_base_mm': points,
    }


def plan_approach(geometry, tool_mm, tool_rotation, options):
    """Keep height/orientation; reject a pose that cannot straddle this book."""
    tool = np.asarray(tool_mm, dtype=float).reshape(3)
    rotation = np.asarray(tool_rotation, dtype=float).reshape(3, 3)
    if not np.isfinite(tool).all() or not np.isfinite(rotation).all():
        raise ValueError('当前工具位姿无效')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or np.linalg.det(rotation) < 0.99:
        raise ValueError('当前工具旋转矩阵无效')
    forward, jaw = -rotation[:, 2], rotation[:, 0]
    if abs(forward[2]) > np.sin(np.deg2rad(5)) or abs(jaw[2]) > np.sin(np.deg2rad(5)):
        raise ValueError('当前手指/开合方向不接近水平；请调整准备姿态')
    along = forward[:2] / np.linalg.norm(forward[:2])
    across = np.array([-along[1], along[0]])
    direction_cos = float(along @ np.asarray(geometry['approach_xy']))
    yaw_error = float(np.rad2deg(np.arccos(np.clip(direction_cos, -1.0, 1.0))))
    hand_yaw = float(np.rad2deg(np.arctan2(along[1], along[0])))
    book_yaw = float(np.rad2deg(np.arctan2(*np.asarray(geometry['approach_xy'])[::-1])))
    print(f'v2 方向检查：手指={hand_yaw:.1f}°，书长边={book_yaw:.1f}°，'
          f'偏差={yaw_error:.1f}°，允许={MAX_APPROACH_YAW_ERROR_DEG:.0f}°', flush=True)
    if yaw_error > MAX_APPROACH_YAW_ERROR_DEG:
        raise ValueError(f'当前接近方向与书长边偏差 {yaw_error:.1f}°，'
                         f'超过 {MAX_APPROACH_YAW_ERROR_DEG:.0f}°；手指={hand_yaw:.1f}°，书={book_yaw:.1f}°')
    tip = tool + options.fingertip_offset_mm * forward
    z_low, z_high = geometry['z_bounds_mm']
    if not z_low + 10 <= tip[2] <= z_high - 10:
        raise ValueError(f'指尖高度 {tip[2]:.1f} mm 不在书体内部高度区间 [{z_low+10:.1f}, {z_high-10:.1f}]')
    points = geometry['points_base_mm']
    band = points[np.abs(points[:, 2] - tip[2]) <= 20]
    if len(band) < 40:
        raise ValueError('指尖高度附近深度点不足，取消接近')
    s = band[:, :2] @ along
    front = float(np.percentile(s, 2))
    # Estimate the near-edge corridor rather than the far, possibly tilted cover.
    near = band[s <= front + max(40.0, options.insertion_mm + 20.0)]
    if len(near) < 20:
        raise ValueError('书前缘附近点不足，无法确定两指间的对齐中心')
    q_low, q_high = np.percentile(near[:, :2] @ across, [2, 98])
    width = float(q_high - q_low)
    if width < 8 or width < geometry['thickness_mm'] * 0.6:
        raise ValueError('书前缘只看到局部表面，不能可靠估计两侧边界')
    needed = max(width, geometry['thickness_mm']) + 2 * options.side_margin_mm
    if needed > options.open_gap_mm:
        raise ValueError(f'书厚加两侧余量需 {needed:.1f} mm，超过夹爪张开 {options.open_gap_mm:.1f} mm')
    tip_forward_offset = options.fingertip_offset_mm * float(forward[:2] @ along)
    current_s = float(tool[:2] @ along)
    if current_s + tip_forward_offset > front - options.clearance_mm:
        raise ValueError('当前指尖已过于靠近书前缘，不执行横向对齐；请先退回准备位')
    # Finger offset can also have a small cross-track component.
    center_q = float((q_low + q_high) / 2 - options.fingertip_offset_mm * (forward[:2] @ across))

    def pose(s_value):
        xy = s_value * along + center_q * across
        return [float(xy[0]), float(xy[1]), float(tool[2])]

    pre_s = front - options.clearance_mm - tip_forward_offset
    grasp_s = front + options.insertion_mm - tip_forward_offset
    if grasp_s - current_s > 350:
        raise ValueError('目标距当前工具超过 350 mm，请检查标定和书本定位')
    count = int(np.ceil((grasp_s - pre_s) / options.approach_step_mm))
    grasp = pose(grasp_s)
    lift = [grasp[0], grasp[1], grasp[2] + 100.0]
    return {
        'align': pose(current_s), 'pregrasp': pose(pre_s), 'grasp': grasp,
        'approach_steps': [pose(sv) for sv in np.linspace(pre_s, grasp_s, count + 1)[1:]],
        'lift': lift, 'observed_width_mm': width,
        'side_clearance_mm': float((options.open_gap_mm - max(width, geometry['thickness_mm'])) / 2),
        'front_projection_mm': front, 'fingertip_offset_mm': options.fingertip_offset_mm,
        'insertion_mm': options.insertion_mm,
        'hand_yaw_deg': hand_yaw, 'book_yaw_deg': book_yaw,
        'yaw_error_deg': yaw_error,
    }


def build_tasks(plan, euler, object_id):
    def task(action, position, description):
        return dict(action=action, arm='right', object=object_id,
                    book_grasp_mode='cart+', target_position=position,
                    target_rotation=list(euler), description=description)
    tasks = [task('move_above_grasp', plan['align'], '在书前方对齐厚度中心'),
             task('move_above_grasp', plan['pregrasp'], '到书前缘预接近位置')]
    tasks.extend(task('move_down_to_grasp', p, '按指尖终点分段水平接近') for p in plan['approach_steps'])
    tasks.extend([task('close_gripper', plan['grasp'], '前缘浅插入后合爪'),
                  task('lift_object', plan['lift'], '保持姿态提升 100 mm'),
                  task('open_gripper', None, '提起后张开夹爪'),
                  task('first_reset', None, '右臂回准备位')])
    return tasks


def clean_mask(mask):
    """Drop disconnected detections without eroding a thin book's edges."""
    import cv2
    binary = np.asarray(mask, dtype=np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    if count < 2:
        return binary.astype(bool)
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == largest


def refine_segmentation(book_geom, rgb, depth, options):
    """Confidence gate, one-book component, then ROI inference with IoU gate."""
    import cv2
    processor, to_numpy, image_cls = book_geom._load_sam3_detector()
    height, width = np.asarray(rgb).shape[:2]

    def candidates(image):
        return book_geom._sam3_raw_books(processor, to_numpy, image_cls.fromarray(np.asarray(image)))

    ranked = []
    diagnostics = []
    for index, (score, mask, box) in enumerate(candidates(rgb), start=1):
        label = f'#{index} score={score:.3f}'
        if not np.isfinite(score) or score < options.min_score:
            diagnostics.append(f'{label}：低于阈值 {options.min_score:.2f} 或无效')
            continue
        if mask.shape != (height, width):
            mask = cv2.resize(mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST)
        mask = clean_mask(mask)
        area = int(mask.sum())
        if area < 100:
            diagnostics.append(f'{label}：连通区域仅 {area} 像素，少于 100')
            continue
        distance = book_geom._mask_median_depth_m(mask, depth)
        if distance is not None and np.isfinite(distance) and distance > 0:
            diagnostics.append(f'{label}：面积 {area}，中位深度 {distance*1000:.0f} mm，通过')
            ranked.append((distance, -score, mask))
        else:
            diagnostics.append(f'{label}：面积 {area}，有效深度不足或无效')
    summary = '; '.join(diagnostics) if diagnostics else 'SAM3 未返回 book 候选（模型内部阈值 0.10）'
    print(f'v2 分割筛选：{summary}', flush=True)
    if not ranked:
        book_geom.BOOK_SELECTION_NOTE = f'v2 没有达到要求的书本：{summary}'
        return None
    distance, neg_score, mask = min(ranked, key=lambda item: item[:2])
    ys, xs = np.nonzero(mask)
    padding = max(24, int(max(np.ptp(xs), np.ptp(ys)) * 0.3))
    x0, x1 = max(0, xs.min()-padding), min(width, xs.max()+padding+1)
    y0, y1 = max(0, ys.min()-padding), min(height, ys.max()+padding+1)
    best_iou, refined = 0.4, mask
    did_refine = False
    for score, local, _ in candidates(rgb[y0:y1, x0:x1]):
        if not np.isfinite(score) or score < options.min_score:
            continue
        local = cv2.resize(local.astype(np.uint8), (x1-x0, y1-y0), interpolation=cv2.INTER_NEAREST)
        full = np.zeros_like(mask)
        full[y0:y1, x0:x1] = clean_mask(local)
        area_ratio = full.sum() / max(mask.sum(), 1)
        iou = np.count_nonzero(full & mask) / max(np.count_nonzero(full | mask), 1)
        if 0.6 <= area_ratio <= 1.4 and iou > best_iou:
            best_iou, refined = iou, full
            did_refine = True
    ys, xs = np.nonzero(refined)
    box = np.array([xs.min(), ys.min(), xs.max()+1, ys.max()+1], dtype=np.float32)
    book_geom.BOOK_SELECTION_NOTE = (
        f'v2 SAM3 局部复分割；score={-neg_score:.2f}，深度={distance*1000:.0f} mm，'
        + (f'ROI匹配={best_iou:.2f}' if did_refine else 'ROI未通过匹配，保留原mask')
        + '；仍需三维尺寸检查'
    )
    return refined, box


def install_v2(agent, book, options):
    import book_grasp as book_geom
    import step5_execute as step5
    from utils.transform_utils import angle2rotation

    book.install_maximum_pregrasp_height()
    original_fill = book_geom.fill_book_meta
    original_grounding = agent.grounding_main
    original_planning = agent.planning_main
    original_execute = agent.execute_main
    original_single = step5.execute_single_task
    cache = {'geometry': None, 'output_dir': None}

    def fill_meta(meta, points_cam, c2w):
        if book_geom.BOOK_GRASP_MODE != 'cart+':
            return original_fill(meta, points_cam, c2w)
        cache['geometry'] = None
        raw = np.asarray(points_cam, dtype=float).reshape(-1, 3)
        valid = np.isfinite(raw).all(axis=1) & (raw[:, 2] > 0)
        points = raw[valid]
        if not len(points):
            raise ValueError('没有有效深度')
        if np.median(points[:, 2]) > 20:
            points = points * 0.001
        base, rotation, translation = book_geom._camera_to_base(points, c2w)
        # Component rejection before PCA removes isolated tray/background points.
        labels = book_geom._component_labels(base, voxel_m=0.012)
        ids, counts = np.unique(labels, return_counts=True)
        keep = labels == ids[np.argmax(counts)]
        geometry = estimate_geometry(base[keep] * 1000)
        keep_mask = np.zeros(len(raw), dtype=bool)
        keep_mask[np.flatnonzero(valid)[keep]] = True
        result = book_geom._fill_mask_depth_center(meta, points[keep], c2w)
        center_m = np.asarray(geometry['center_base_mm']) * 0.001
        old_center = meta['centroid'][:3, 3].copy()
        new_center = rotation.T @ (center_m - translation)
        seen = set()
        for key in ('centroid', 'grasp_0', 'bbox_plane_x1', 'bbox_plane_x2',
                    'bbox_plane_y1', 'bbox_plane_y2', 'bbox_plane_z1', 'bbox_plane_z2'):
            value = meta[key]
            if id(value) not in seen:
                value[:3, 3] += new_center - old_center
                seen.add(id(value))
        result.update({k: v for k, v in geometry.items() if k != 'points_base_mm'})
        result.update(point_mask=keep_mask, input_points=len(raw), denoised_points=int(keep.sum()))
        cache['geometry'] = geometry
        return result

    def grounding(*args, **kwargs):
        cache['geometry'] = None
        cache['output_dir'] = Path(kwargs.get('output_dir', args[1] if len(args) > 1 else '.'))
        robot = kwargs.get('robot', args[4] if len(args) > 4 else None)
        if not options.preview_only and book_geom.BOOK_GRASP_MODE == 'cart+':
            if robot is None or not robot.control_joint_position(left_pos=False, right_pos=True, action='first_reset'):
                raise RuntimeError('Grounding failed for all objects in the scene.')
        return original_grounding(*args, **kwargs)

    def planning(*args, **kwargs):
        tasks = original_planning(*args, **kwargs)
        if book_geom.BOOK_GRASP_MODE == 'cart+':
            for task in tasks:
                task['description'] = 'v2 待当前工具位姿生成指尖接近路径'
        return tasks

    def segment(rgb, depth, intrinsic, c2w):
        try:
            return refine_segmentation(book_geom, rgb, depth, options)
        except (ValueError, RuntimeError, OSError, ImportError) as error:
            book_geom.BOOK_SELECTION_NOTE = f'v2 分割失败，取消本轮：{error}'
            return None

    def single(robot, task, index, object_types, **kwargs):
        if task.get('action') in {'first_reset', 'reset_pos', 'reset_all'}:
            return task.get('object'), robot.control_joint_position(
                left_pos=False, right_pos=True, action=task['action'])
        if task.get('arm') != 'right':
            print('book_grap_v2 拒绝非右臂动作：', task.get('action'))
            return task.get('object'), False
        return original_single(robot, task, index, object_types, **kwargs)

    def recover_right_only(robot, failed_task, *, operator_requested=False):
        if options.preview_only:
            return False
        reason = '操作员中止' if operator_requested else '任务失败'
        print(f'book_grap_v2 {reason}：只打开右夹爪、只复位右臂。')
        try:
            if not step5.fully_open_gripper_before_pick(
                robot, 'right', (failed_task or {}).get('object'), 'book'
            ):
                print('右夹爪打开未确认，取消自动复位。')
                return False
            return bool(robot.control_joint_position(
                left_pos=False, right_pos=True,
                action='book_v2_right_recovery', force=True))
        except Exception as error:
            print(f'book_grap_v2 右臂恢复失败：{error}')
            return False

    def execute(tasks, object_types, robot, **kwargs):
        if not tasks:
            return None, False
        object_id = next((t.get('object') for t in tasks if t.get('book_grasp_mode') == 'cart+'), None)
        if object_id is None:
            if options.preview_only:
                print('v2 预览模式：未发送动作。')
                return None, False
            return original_execute(tasks, object_types, robot, **kwargs)
        try:
            geometry = cache['geometry']
            if geometry is None:
                raise ValueError('没有本轮书本几何数据')
            state = robot.get_current_state()
            arm = None if state is None else state.get('right_arm')
            if arm is None:
                raise ValueError('无法读取右臂工具位姿')
            euler = robot._convert_quaternion_to_pos_rot([0, 0, 0, *arm['orientation']])[3:]
            plan = plan_approach(geometry, np.asarray(arm['position']) * 1000,
                                 angle2rotation(*euler), options)
            new_tasks = build_tasks(plan, euler, object_id)
        except (ValueError, KeyError, TypeError) as error:
            print(f'book_grap_v2 取消接近：{error}')
            return object_id, False
        output = cache['output_dir'] / 'book_grap_v2_plan.json'
        output.write_text(json.dumps({'plan': plan, 'tasks': new_tasks}, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f"v2 书前缘={plan['front_projection_mm']:.1f} mm，工具终点={np.round(plan['grasp'], 1)}，"
              f"指尖前伸={options.fingertip_offset_mm:.0f} mm，插入={options.insertion_mm:.0f} mm，"
              f"每侧可见余量={plan['side_clearance_mm']:.1f} mm；路径：{output}")
        if kwargs.get('rerun_logger'):
            kwargs['rerun_logger'].log_manual_planning_tasks(new_tasks)
        if options.preview_only:
            print('v2 预览模式：未复位、未张合爪、未发送接近动作。')
            return object_id, False
        if not step5.fully_open_gripper_before_pick(robot, 'right', object_id, 'book'):
            return object_id, False
        return original_execute(new_tasks, object_types, robot, **kwargs)

    book_geom.segment_cart_book_with_sam3 = segment
    book_geom.fill_book_meta = fill_meta
    agent.grounding_main = grounding
    agent.planning_main = planning
    agent.execute_main = execute
    step5.execute_single_task = single
    # execute_main calls this helper directly, bypassing execute_single_task.
    step5.recover_to_reset_after_failure = recover_right_only


def main():
    options, rest = parse_options(sys.argv[1:])
    if '--help' in rest or '-h' in rest:
        print(__doc__)
        print('v2 参数：--book-preview-only、--book-fingertip-offset-mm 50、'
              '--book-open-gap-mm 60、--book-insertion-mm 10、--book-side-margin-mm 8、'
              '--book-clearance-mm 40、--book-approach-step-mm 5、--book-min-score 0.20')
        return
    import book_grasp_1_1 as book
    import dual_v2_2_agent_pcl as agent
    install_v2(agent, book, options)
    sys.argv = [sys.argv[0], *[arg for arg in rest if arg not in {'--remote-rgbd', '--local-camera'}]]
    agent.EXTRA_TASK_TYPES = ['shelf', 'cart', 'cart+']
    agent.EXTRA_TASK_BUILDER = book.build_book_tasks
    agent.EXCLUSIVE_EXTENSION_MODE = True
    agent.GROUNDING_BOX_THRESHOLD_OVERRIDE = 0.18
    agent.GROUNDING_TEXT_THRESHOLD_OVERRIDE = 0.20
    agent.EXTRA_SCENE_OBJECTS = [{'name': 'book', 'color': '', 'category': 'book'}]
    print('book_grap_v2：本机 Orbbec。输入 cart+ 使用前缘定位和指尖偏移；'
          '保持当前高度/姿态，张爪后分段接近，合爪、提起、松开、右臂复位。')
    agent.main()


if __name__ == '__main__':
    main()
