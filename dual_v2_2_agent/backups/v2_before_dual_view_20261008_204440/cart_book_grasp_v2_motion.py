"""Same book grasp as ``book_grasp_1_1.py``, using the local USB Orbbec.

RGB and aligned depth come from the Orbbec plugged into this machine.
The remote erazer stream is not started.
cart+ distances (mm), hold time (seconds), and initial right J6 (degrees) are read from
cart_book_grasp_v2_config.json before each relevant action.
Edits apply to the next action, never mid-motion.

    python cart_book_grasp_v2.py --assume-head-camera-calibrated
"""

import json
from pathlib import Path
import sys
import time

import numpy as np

import book_grasp_1_1 as book
from cart_book_grasp_v2_depth import whole_book_depth, forward_parameters
from cart_book_grasp_v2_visualization import forward_targets, withdraw_targets, lower_targets, match_book_mask


def _local_camera_argv(argv):
    """Drop remote-camera flags so the agent opens the local Orbbec."""
    skipped = {"--remote-rgbd", "--local-camera"}
    return [item for item in argv if item not in skipped]


_SELECTION_AGENT = None  # Bound by the language entrypoint for this process.

_RIGHT_ARM_RESET_ACTIONS = {"first_reset", "reset_pos", "reset_all"}
_CART_PLUS_LEVEL_ACTIONS = {"move_above_grasp", "move_down_to_grasp"}
_CART_PLUS_ACTIONS = (
    "move_above_grasp",
    "move_down_to_grasp",
    "close_gripper",
    "lift_object",
)
_CART_PLUS_CONFIG_PATH = Path(__file__).resolve().with_name(
    "cart_book_grasp_v2_config.json"
)


def _load_cart_plus_parameters():
    """Read a complete fresh configuration; never reuse stale/default values."""
    try:
        document = json.loads(_CART_PLUS_CONFIG_PATH.read_text(encoding="utf-8"))
        if not isinstance(document, dict) or set(document) != {"cart_plus"}:
            raise ValueError("顶层必须只包含 cart_plus")
        values = document["cart_plus"]
        fields = {"fingertip_offset_mm", "grasp_insertion_mm", "forward_step_mm", "lift_mm", "hold_seconds", "lower_mm", "initial_j6_deg", "lower_step_mm"}
        if not isinstance(values, dict) or set(values) != fields:
            raise ValueError("cart_plus 必须包含且只包含 initial_j6_deg、fingertip_offset_mm、grasp_insertion_mm、forward_step_mm、lift_mm、hold_seconds、lower_mm、lower_step_mm")
        parameters = {}
        for name in ("fingertip_offset_mm", "grasp_insertion_mm", "forward_step_mm", "lift_mm", "hold_seconds", "lower_mm", "initial_j6_deg", "lower_step_mm"):
            value = values[name]
            unit = "度" if name == "initial_j6_deg" else "秒" if name == "hold_seconds" else "mm"
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} 必须是以 {unit} 为单位的数字")
            if name == "initial_j6_deg":
                if not np.isfinite(value) or not -90 <= value <= 90:
                    raise ValueError("initial_j6_deg 必须在右臂 J6 软限位 [-90, 90] 度内")
            elif not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} 必须是非负有限数值（{unit}）")
            if name in {"forward_step_mm", "lower_step_mm"} and value < 1:
                raise ValueError(f"{name} 必须至少为 1 mm")
            parameters[name] = float(value)
    except (OSError, ValueError, TypeError, OverflowError) as error:
        raise ValueError(f"cart+ 配置读取失败 {_CART_PLUS_CONFIG_PATH}：{error}") from error
    print(
        f"cart+ 参数已热加载：初始 J6={parameters['initial_j6_deg']:g}°，按整书深度动态前伸，指尖偏移 {parameters['fingertip_offset_mm']:g} mm，伸入 {parameters['grasp_insertion_mm']:g} mm，"
        f"前伸每段不超过 {parameters['forward_step_mm']:g} mm，抬升 {parameters['lift_mm']:g} mm，停顿 {parameters['hold_seconds']:g} 秒，"
        f"下降 {parameters['lower_mm']:g} mm，每段不超过 {parameters['lower_step_mm']:g} mm（{_CART_PLUS_CONFIG_PATH.name}）",
        flush=True,
    )
    return parameters


def _reload_initial_right_j6(robot):
    parameters = _load_cart_plus_parameters()
    ready = np.asarray(robot.right_safe_joint_deg, dtype=float).copy()
    if ready.shape != (7,) or not np.isfinite(ready).all():
        raise ValueError("右臂准备位必须是 7 个有效关节角")
    ready[5] = parameters["initial_j6_deg"]
    robot.right_safe_joint_deg = ready


def _right_tool_mm_euler(robot, step5):
    state = robot.get_current_state()
    arm_state = None if state is None else state.get("right_arm")
    if arm_state is None:
        return None
    position_mm = np.asarray(arm_state["position"], dtype=float) * 1000.0
    euler = step5.current_tool_orientation_as_euler(robot, "right")
    if euler is None or position_mm.shape != (3,) or not np.isfinite(position_mm).all():
        return None
    euler = np.asarray(euler, dtype=float)
    if euler.shape != (3,) or not np.isfinite(euler).all():
        return None
    return position_mm, euler


def _spine_center_from_mask(mask, depth, intrinsic):
    """Midpoint of the visible standing spine's midline, with local depth."""
    import cv2

    depth = np.asarray(depth, dtype=float)
    mask = np.asarray(mask, dtype=np.uint8)
    if depth.ndim != 2 or mask.ndim != 2:
        raise ValueError("书脊 mask 或深度图维度无效")
    if mask.shape != depth.shape:
        mask = cv2.resize(mask, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count < 2:
        raise ValueError("书脊 mask 为空")
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = labels == component
    ys, xs = np.nonzero(mask)
    x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
    height, width = y1-y0+1, x1-x0+1
    if x0 <= 1 or y0 <= 1 or x1 >= mask.shape[1]-2 or y1 >= mask.shape[0]-2:
        raise ValueError("书脊被画面边缘截断")
    if height < 40 or height < 2 * width or mask.sum() < 100:
        raise ValueError(f"不是清晰的竖立书脊：宽 {width}、高 {height} 像素")

    v = (y0 + y1) / 2.0
    rows, midpoints = [], []
    center_strip = np.zeros_like(mask)
    for row in range(y0, y1+1):
        cols = np.flatnonzero(mask[row])
        if len(cols) < 2:
            continue
        left, right = float(cols[0]), float(cols[-1])
        middle = (left + right) / 2
        if abs(row-v) <= 0.25 * height:
            rows.append(row)
            midpoints.append(middle)
        if abs(row-v) <= max(3., height * 0.1):
            inner = cols[np.abs(cols-middle) <= max(1., (right-left) * 0.25)]
            center_strip[row, inner] = True
    if len(rows) < 10:
        raise ValueError("书脊中段边界不足")
    rows = np.asarray(rows, dtype=float)
    midpoints = np.asarray(midpoints, dtype=float)
    fit = np.polyfit(rows-v, midpoints, 1)
    residual = np.abs(midpoints - np.polyval(fit, rows-v))
    good = residual <= max(2., 3 * np.median(residual))
    if np.count_nonzero(good) < 10:
        raise ValueError("书脊中线不稳定")
    u = float(np.polyfit(rows[good]-v, midpoints[good], 1)[1])
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= vi < mask.shape[0] and 0 <= ui < mask.shape[1] and mask[vi, ui]):
        raise ValueError("书脊中心落在分割区域外")
    samples = depth[center_strip]
    samples = samples[np.isfinite(samples) & (samples > 0)]
    if len(samples) < 12:
        raise ValueError("书脊中心附近有效深度不足")
    if np.median(samples) > 20:
        samples = samples * 0.001
    if np.percentile(samples, 90)-np.percentile(samples, 10) > 0.10:
        raise ValueError("书脊中心深度混入背景或不稳定")
    k = np.asarray(intrinsic, dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all():
        raise ValueError("相机内参无效")
    ray = np.linalg.solve(k, np.array([u, v, 1.]))
    if not np.isfinite(ray).all() or ray[2] <= 0:
        raise ValueError("书脊中心反投影无效")
    z_m = float(np.median(samples))
    return {
        "mask": mask, "box": np.array([x0, y0, x1+1, y1+1], dtype=np.float32),
        "uv": [u, v], "point_camera_m": ray / ray[2] * z_m,
        "depth_m": z_m,
        "depth_samples": int(len(samples)),
        "depth_p10_p90_mm": (np.percentile(samples, [10, 90])*1000).tolist(),
    }


def _install_spine_center(book_geom, visualization=None, on_geometry=None):
    """Scope spine segmentation/centering to this entrypoint's cart+ mode."""
    original_fill = book_geom.fill_book_meta
    cache = {"selected": None}

    def segment(rgb, depth, intrinsic, c2w, *, box_detector=None):
        cache["selected"] = None
        notes, candidates = [], []
        try:
            if _SELECTION_AGENT is None:
                raise ValueError('v2 requires a language-selection agent and a current instruction')
            def publish_recognition():
                if visualization is not None:
                    if _SELECTION_AGENT.last_image is not None:
                        visualization.safe('log_image', 'book/images/candidates', _SELECTION_AGENT.last_image)
                    if _SELECTION_AGENT.last_gate_image is not None:
                        visualization.safe('log_image', 'book/images/cart_filter', _SELECTION_AGENT.last_gate_image)
                    if _SELECTION_AGENT.last_review_image is not None:
                        visualization.safe('log_image', 'book/images/candidate_review', _SELECTION_AGENT.last_review_image)
                    summary = [dict(id=item['id'], mask_source=item['recognition'],
                        base_y_mm=item['base_y_mm'], geometry_status=item['geometry_status'],
                        geometry_error=item['error'], gpt_review=item.get('gpt_review'),
                        verified_book=item.get('verified_book')) for item in _SELECTION_AGENT.last_candidates]
                    visualization.safe('log_json_document', 'book/selection',
                        dict(decision=_SELECTION_AGENT.last_decision, candidates=summary,
                             cart_filter=_SELECTION_AGENT.last_filter))
                    visualization.safe('flush_book')
            try:
                from cart_book_grasp_v2_target import prepare_instance_target
                def prepare_selected(candidate):
                    return prepare_instance_target(candidate,rgb,depth,intrinsic,c2w,book_geom,
                        _spine_center_from_mask,_SELECTION_AGENT.config_path,_SELECTION_AGENT.checkpoint)
                candidate = _SELECTION_AGENT.select_instances(rgb, depth, intrinsic, c2w,
                    book_geom, prepare_selected, on_recognized=publish_recognition)
            finally:
                publish_recognition()
            selected = candidate['geometry']
            selected['selection_id'] = candidate['id']
            selected['selection_decision'] = dict(_SELECTION_AGENT.last_decision)
            cache['selected'] = selected
            book_mask = candidate['book_mask']
            book_geom.BOOK_SELECTION_NOTE = (
                f"GPT 选择 {candidate['id']}；书脊中心 {np.round(selected['uv'],1).tolist()}，"
                f"整书基座 X 中位数 {selected['book_surface_x_mm']:.1f} mm")
            if visualization is not None:
                visualization.capture(rgb, depth, intrinsic, c2w, selected, book_mask)
            return selected["mask"], selected["box"]
        except Exception as error:
            book_geom.BOOK_SELECTION_NOTE = f"选书或目标几何检查未完成：{error}"
            # Reach the v2 cycle-failure handler directly, rather than turning
            # a language clarification into the misleading 'object_0 missing'.
            raise RuntimeError(book_geom.BOOK_SELECTION_NOTE) from error

    def fill(meta, points, c2w):
        if book_geom.BOOK_GRASP_MODE != "cart+":
            return original_fill(meta, points, c2w)
        selected = cache["selected"]
        if selected is None:
            raise ValueError("本轮没有有效书脊中心")
        result = original_fill(meta, points, c2w)
        old_center = meta["centroid"][:3, 3].copy()
        new_center = selected["point_camera_m"]
        seen = set()
        for key in ("centroid", "grasp_0", "bbox_plane_x1", "bbox_plane_x2",
                    "bbox_plane_y1", "bbox_plane_y2", "bbox_plane_z1", "bbox_plane_z2"):
            value = meta[key]
            if id(value) not in seen:
                value[:3, 3] += new_center-old_center
                seen.add(id(value))
        base, _, _ = book_geom._camera_to_base(new_center.reshape(1, 3), c2w)
        result["center_base_mm"] = (base[0]*1000).tolist()
        result["center_kind"] = "visible_book_spine_midline"
        result["book_selection_id"] = selected["selection_id"]
        result["book_selection_decision"] = selected["selection_decision"]
        result["spine_center_uv"] = selected["uv"]
        result["spine_center_depth_mm"] = selected["depth_m"]*1000
        result["spine_depth_samples"] = selected["depth_samples"]
        result["spine_depth_p10_p90_mm"] = selected["depth_p10_p90_mm"]
        for key in ('spine_center_recovery', 'spine_original_center_uv', 'spine_full_fit_failure',
                    'spine_stable_region_uv', 'spine_stable_region_fraction', 'spine_center_shift_px'):
            if key in selected:
                result[key] = selected[key]
        if on_geometry is not None:
            on_geometry(selected, c2w, result)
        if visualization is not None:
            visualization.update_geometry(result, c2w)
        print(f"书脊中心（基座 mm）：{np.round(base[0]*1000, 1).tolist()}", flush=True)
        return result

    segment.instance_first = True
    book_geom.segment_cart_book_with_sam3 = segment
    book_geom.fill_book_meta = fill


def _rewrite_cart_plus_level_task(robot, task, step5):
    """Keep the current hand height and orientation for the cart+ grasp."""
    if (
        task.get("book_grasp_mode") != "cart+"
        or task.get("arm") != "right"
        or task.get("action") not in _CART_PLUS_LEVEL_ACTIONS
    ):
        return task
    try:
        parameters = _load_cart_plus_parameters()
    except ValueError as error:
        print(f"{error}；取消当前动作，不使用旧参数。")
        return None
    current = _right_tool_mm_euler(robot, step5)
    if current is None:
        print("cart+ 水平抓取：读不到右臂当前位姿。")
        return None
    position_mm, euler = current
    center = task.get("book_center_xy")
    if not center or len(center) < 2:
        print("cart+ 水平抓取：任务里没有头部相机给出的书本水平位置。")
        return None
    book_xy = np.asarray(center[:2], dtype=float)
    hand_xy = position_mm[:2]
    gap_xy = book_xy - hand_xy
    if task["action"] == "move_above_grasp":
        target_xy = np.array([hand_xy[0], book_xy[1]])
        stage = "只沿 Y 对准书脊中心，X 保持不动"
    else:
        raise ValueError('Dynamic forward motion must use the segmented executor')
    task["target_position"] = [
        float(target_xy[0]),
        float(target_xy[1]),
        float(position_mm[2]),
    ]
    task["target_rotation"] = euler.tolist()
    print(
        f"cart+ {stage}：当前 "
        f"[{hand_xy[0]:.1f}, {hand_xy[1]:.1f}, {position_mm[2]:.1f}] mm，"
        f"书脊中心水平 [{book_xy[0]:.1f}, {book_xy[1]:.1f}] mm，"
        f"水平差距 [{gap_xy[0]:.1f}, {gap_xy[1]:.1f}] mm，"
        f"目标 {[round(value, 1) for value in task['target_position']]} mm，"
        f"姿态保持 {[round(value, 1) for value in euler]}°。"
    )
    return task


def _execute_cart_plus_forward(robot, task, task_index, object_id_to_type,
                               step5, execute, *, retreat_context=None, **kwargs):
    """Reach collinear targets sequentially, all anchored to the same start pose.

    These are planner endpoint constraints, not a Cartesian straight-line guarantee.
    Configuration is snapshotted once for the entire forward action.
    """
    try:
        parameters = _load_cart_plus_parameters()
    except ValueError as error:
        print(f"{error}；取消前伸。")
        return task.get("object"), False
    current = _right_tool_mm_euler(robot, step5)
    if current is None:
        print("cart+ 分段前伸：读不到有效右臂位姿，取消前伸。")
        return task.get("object"), False
    start, euler = current
    try:
        parameters = forward_parameters(start, euler, task.get('book_surface_x_mm'), parameters)
    except (ValueError, TypeError) as error:
        print(f'整书深度前伸计算失败：{error}；取消前伸。', flush=True)
        return task.get('object'), False
    print(f"动态前伸：整书基座 X={parameters['book_surface_x_mm']:.1f} mm，"
          f"指尖 X 偏移={parameters['fingertip_x_offset_mm']:.1f} mm，"
          f"TCP 目标 X={parameters['grasp_tcp_x_mm']:.1f} mm，前伸 {parameters['forward_mm']:.1f} mm。", flush=True)
    if retreat_context is not None:
        retreat_context[task.get('object')] = dict(start_x_mm=float(start[0]),
                                                  forward_complete=False, released=False,
                                                  withdrawn=False)
    distance = parameters["forward_mm"]
    if distance == 0:
        if retreat_context is not None:
            retreat_context[task.get('object')]['forward_complete'] = True
        return task.get("object"), True
    targets = forward_targets(start, parameters)
    segments = len(targets)
    print(f"cart+ 前伸 {distance:g} mm 分成 {segments} 段；各点固定起点的 Y、Z 和姿态。", flush=True)
    for index, target in enumerate(targets, 1):
        waypoint = dict(task)
        waypoint["target_position"] = target.tolist()
        waypoint["target_rotation"] = euler.tolist()
        waypoint["book_forward_index"] = index
        waypoint["book_forward_count"] = segments
        waypoint["description"] = f"分段前伸 {index}/{segments}，距起点 {distance * index / segments:g} mm"
        print(f"cart+ {waypoint['description']}，目标 {np.round(target, 1).tolist()} mm", flush=True)
        result = execute(robot, waypoint, task_index, object_id_to_type, **kwargs)
        if not result[1]:
            print(f"cart+ 前伸第 {index}/{segments} 段失败，取消剩余前伸和后续抓取。")
            return result
    if retreat_context is not None:
        retreat_context[task.get('object')]['forward_complete'] = True
    return result


def _execute_cart_plus_withdraw(robot, task, task_index, object_id_to_type,
                                step5, execute, retreat_context, **kwargs):
    """Withdraw only after release; use this run's approach X, not a new distance."""
    state = retreat_context.get(task.get('object'), {})
    if not state.get('forward_complete') or not state.get('released'):
        print('cart+ 未确认本轮前伸完成和松手成功，取消抽回及复位。')
        return task.get('object'), False
    try:
        parameters = _load_cart_plus_parameters()
    except ValueError as error:
        print(f'{error}；取消抽回及复位。')
        return task.get('object'), False
    current = _right_tool_mm_euler(robot, step5)
    if current is None:
        return task.get('object'), False
    start, euler = current
    if start[0] < state['start_x_mm'] - 1.:
        print('cart+ 当前 X 已在前伸起点后方，取消抽回及自动复位，请检查位置。')
        return task.get('object'), False
    targets = withdraw_targets(start, state['start_x_mm'], parameters['forward_step_mm'])
    for index, target in enumerate(targets, 1):
        waypoint = dict(task, target_position=target.tolist(), target_rotation=euler.tolist(),
                        book_withdraw_index=index, book_withdraw_count=len(targets),
                        description=f'分段抽回 {index}/{len(targets)}，保持 Y、Z 和姿态')
        result = execute(robot, waypoint, task_index, object_id_to_type, **kwargs)
        if not result[1]:
            print('cart+ 抽回失败，停止后续抽回和 first_reset。')
            return result
    state['withdrawn'] = True
    return task.get('object'), True


def _execute_cart_plus_lower(robot, task, task_index, object_id_to_type, step5, execute, **kwargs):
    """Snapshot descent geometry once; stop at the first unconfirmed segment.

    Collinear endpoints shorten planner arcs but do not guarantee a perfectly
    straight Cartesian path between endpoints.
    """
    try:
        parameters = _load_cart_plus_parameters()
    except ValueError as error:
        print(f'{error}；取消下降，保持夹紧。')
        return task.get('object'), False
    current = _right_tool_mm_euler(robot, step5)
    if current is None:
        print('cart+ 分段下降：读不到有效右臂位姿，取消下降。')
        return task.get('object'), False
    start, euler = current
    targets = lower_targets(start, parameters)
    print(f"cart+ 下降 {parameters['lower_mm']:g} mm 分成 {len(targets)} 段；各点固定起点的 X、Y 和姿态。", flush=True)
    for index, target in enumerate(targets, 1):
        waypoint = dict(task, target_position=target.tolist(), target_rotation=euler.tolist(),
                        book_lower_index=index, book_lower_count=len(targets),
                        description=f'分段下降 {index}/{len(targets)}，保持 X、Y 和姿态')
        result = execute(robot, waypoint, task_index, object_id_to_type, **kwargs)
        if not result[1]:
            print(f'cart+ 下降第 {index}/{len(targets)} 段失败，取消剩余下降、松手和复位。')
            return result
    return task.get('object'), True


def _rewrite_cart_plus_lift(robot, task, step5):
    """Lift or lower by a freshly loaded distance, keeping XY/orientation."""
    if (
        task.get("book_grasp_mode") != "cart+"
        or task.get("arm") != "right"
        or task.get("action") not in {"lift_object", "move_down_to_place"}
    ):
        return task
    try:
        parameters = _load_cart_plus_parameters()
    except ValueError as error:
        print(f"{error}；取消当前动作，不使用旧参数。")
        return None
    current = _right_tool_mm_euler(robot, step5)
    if current is None:
        print("cart+ 升降：读不到右臂当前位姿。")
        return None
    position_mm, euler = current
    lifting = task["action"] == "lift_object"
    distance_mm = parameters["lift_mm"] if lifting else parameters["lower_mm"]
    signed_distance_mm = distance_mm if lifting else -distance_mm
    task["target_position"] = [
        float(position_mm[0]),
        float(position_mm[1]),
        float(position_mm[2] + signed_distance_mm),
    ]
    task["target_rotation"] = euler.tolist()
    print(
        f"cart+ {'提升' if lifting else '下降'} {distance_mm / 10:g} cm：当前 "
        f"[{position_mm[0]:.1f}, {position_mm[1]:.1f}, {position_mm[2]:.1f}] mm，"
        f"目标 {[round(value, 1) for value in task['target_position']]} mm，"
        f"姿态保持 {[round(value, 1) for value in euler]}°。"
    )
    return task


def _keep_cart_plus_grasp_only(tasks):
    """cart+ releases after lowering, withdraws, then resets the right arm."""
    if not tasks or not any(
        task.get("book_grasp_mode") == "cart+" for task in tasks
    ):
        return tasks
    kept = []
    for task in tasks:
        if task.get("action") not in _CART_PLUS_ACTIONS:
            continue
        kept.append(task)
        if task.get("action") == "lift_object":
            for action, description in (
                ("wait_after_lift", "抬升完成后保持夹紧并停顿"),
                ("move_down_to_place", "保持夹紧，原地下降放回"),
                ("open_gripper", "下降完成后松开右夹爪"),
                ("withdraw_after_release", "松手完成后沿 -X 分段抽回前伸起点"),
                ("first_reset", "抽回完成后右臂回到 first_reset"),
            ):
                next_task = dict(task)
                next_task.update(action=action, arm="right", target_position=None,
                                 target_rotation=None, description=description)
                if action == 'first_reset':
                    next_task['book_post_release_reset'] = True
                kept.append(next_task)
    print(
        "cart+ v2：先沿 Y 对准书脊中心，仅深度回退引起的中心偏移补偿基座 Z，再按整书深度沿 X 前进、合爪、抬升、停顿、分段下降，"
        "下降完成后松爪、分段抽回，最后右臂 first_reset；参数在各自动作前热加载。",
        flush=True,
    )
    return kept


def _install_right_arm_reset():
    """Reset the right arm only, then grasp cart+ books with a level motion."""
    import dual_v2_2_agent_pcl as agent
    import step5_execute as step5
    import book_grasp as book_geom
    from cart_book_grasp_v2_visualization import install_book_visualization

    visualization = install_book_visualization(
        agent, lambda robot: _right_tool_mm_euler(robot, step5), _load_cart_plus_parameters)
    _install_spine_center(book_geom, visualization)

    original = step5.execute_single_task
    original_main = agent.execute_main
    original_grounding = agent.grounding_main
    original_recovery = step5.recover_to_reset_after_failure
    retreat_context = {}

    def recover_after_failure(robot, failed_task, *, operator_requested=False):
        if failed_task and failed_task.get("book_grasp_mode") == "cart+":
            print("cart+ 已中止后续动作：保持夹爪当前指令，不自动松手或复位。")
            return False
        return original_recovery(
            robot, failed_task, operator_requested=operator_requested
        )

    step5.recover_to_reset_after_failure = recover_after_failure

    def grounding_main(*args, **kwargs):
        import book_grasp as book_geom

        robot = kwargs.get("robot", args[4] if len(args) > 4 else None)
        if visualization.logger is not None and robot is not None:
            visualization.attach_robot(robot)
        if book_geom.BOOK_GRASP_MODE == "cart+" and robot is not None:
            try:
                _reload_initial_right_j6(robot)
            except ValueError as error:
                print(f"{error}；取消本轮，未发送识别前复位。")
                raise RuntimeError("Grounding failed for all objects in the scene.") from error
            print(
                "识别前先把右臂送到 first_reset，避免手挡住书。",
                flush=True,
            )
            moved = robot.control_joint_position(
                left_pos=False,
                right_pos=True,
                action="first_reset",
            )
            if not moved:
                print("识别前的 first_reset 失败，取消本次识别。")
                raise RuntimeError("Grounding failed for all objects in the scene.")
        return original_grounding(*args, **kwargs)

    agent.grounding_main = grounding_main

    def execute_main(tasks, *args, **kwargs):
        retreat_context.clear()
        visualization.tracking = visualization.active
        try:
            return original_main(_keep_cart_plus_grasp_only(tasks), *args, **kwargs)
        finally:
            visualization.tracking = False
            visualization.safe('flush_book')

    agent.execute_main = execute_main
    step5.execute_main = execute_main

    def execute_and_visualize(robot, task, *args, **kwargs):
        if task.get('action') not in {'close_gripper', 'open_gripper'}:
            visualization.safe('log_book_target', task)
        result = original(robot, task, *args, **kwargs)
        if task.get('action') == 'open_gripper' and task.get('object') in retreat_context:
            retreat_context[task['object']]['released'] = bool(result[1])
        return result

    def execute_single_task(robot, task, task_index, object_id_to_type, **kwargs):
        action = task.get("action")
        if (task.get("book_grasp_mode") == "cart+"
                and task.get("arm") == "right" and action == "move_down_to_grasp"):
            return _execute_cart_plus_forward(
                robot, task, task_index, object_id_to_type, step5, execute_and_visualize,
                retreat_context=retreat_context, **kwargs)
        if (task.get('book_grasp_mode') == 'cart+' and task.get('arm') == 'right'
                and action == 'move_down_to_place'):
            return _execute_cart_plus_lower(robot, task, task_index, object_id_to_type,
                step5, execute_and_visualize, **kwargs)
        if (task.get('book_grasp_mode') == 'cart+' and task.get('arm') == 'right'
                and action == 'withdraw_after_release'):
            return _execute_cart_plus_withdraw(
                robot, task, task_index, object_id_to_type, step5, execute_and_visualize,
                retreat_context, **kwargs)
        if task.get("book_grasp_mode") == "cart+" and action == "wait_after_lift":
            try:
                parameters = _load_cart_plus_parameters()
            except ValueError as error:
                print(f"{error}；取消后续动作，保持夹爪当前指令。")
                return task.get("object"), False
            print(f"cart+ 抬升已完成，保持夹紧并停顿 {parameters['hold_seconds']:g} 秒。", flush=True)
            time.sleep(parameters["hold_seconds"])
            return task.get("object"), True
        if action in _RIGHT_ARM_RESET_ACTIONS:
            if (task.get('book_post_release_reset')
                    and not retreat_context.get(task.get('object'), {}).get('withdrawn')):
                print('cart+ 抽回尚未成功，禁止自动 first_reset。')
                return task.get('object'), False
            try:
                _reload_initial_right_j6(robot)
            except ValueError as error:
                print(f"{error}；取消复位。")
                return task.get("object"), False
            print(
                f"执行任务 {task_index + 1}: arm: right - action: {action} - "
                "reset 使用右臂准备位，左臂保持当前关节。"
            )
            result = robot.control_joint_position(
                left_pos=False,
                right_pos=True,
                action=action,
            )
            return task.get("object"), result
        rewritten = _rewrite_cart_plus_level_task(robot, task, step5)
        if rewritten is None:
            return task.get("object"), False
        rewritten = _rewrite_cart_plus_lift(robot, rewritten, step5)
        if rewritten is None:
            return task.get("object"), False
        return execute_and_visualize(
            robot, rewritten, task_index, object_id_to_type, **kwargs
        )

    step5.execute_single_task = execute_single_task


def main(*, task_types=None):
    import dual_v2_2_agent_pcl as agent

    book.install_maximum_pregrasp_height()
    _install_right_arm_reset()
    sys.argv = [sys.argv[0], *_local_camera_argv(sys.argv[1:])]
    agent.EXTRA_TASK_TYPES = list(task_types) if task_types is not None else ["shelf", "cart", "cart+"]
    agent.EXTRA_TASK_BUILDER = book.build_book_tasks
    agent.EXCLUSIVE_EXTENSION_MODE = True
    agent.GROUNDING_BOX_THRESHOLD_OVERRIDE = 0.18
    agent.GROUNDING_TEXT_THRESHOLD_OVERRIDE = 0.20
    agent.EXTRA_SCENE_OBJECTS = [
        {
            "name": "book",
            "color": "",
            "category": "book",
        }
    ]
    print(
        "本机直接打开 USB Orbbec，采集 RGB 和对齐深度，不从机器人拉流。\n"
        "抓书目标仍发给规划器，由规划器交给远程机器人执行。\n"
        "first_reset / reset_pos / reset_all 只把右臂送到右臂准备位，左臂不动。\n"
        + ("输入 shelf：右手从侧面抓架子上立着的书。\n"
        "输入 cart：车上立书。预抓高度取表内高度和软限位允许的最大高度中的较低者；"
        "夹爪竖直向下，两指沿点云薄边开合。提起后向左 6cm 放到左侧格子，不复位。\n"
           if task_types is None else "")
        + "cart+：每次识别前先把右臂送到 first_reset，避免手挡住书；"
        "识别书脊中心，然后 X 保持当前值，只沿 Y 对准书脊中心，按配置分段前进、合爪、原地抬升，"
        "停顿后原地下降，松爪完成后分段抽回，再右臂 first_reset；中途失败不自动松手或复位。\n"
        f"cart+ 热加载配置：{_CART_PLUS_CONFIG_PATH}；距离单位 mm，停顿单位秒，保存后下一动作生效。",
        flush=True,
    )
    agent.main()


if __name__ == "__main__":
    main()


def execute_depth_z_compensation(robot, task, step5, execute, geometry, *args, **kwargs):
    """After Y alignment, apply only the fallback-induced base-Z difference once."""
    delta = float(geometry.get('depth_recovery_delta_z_mm', 0.))
    if not np.isfinite(delta):
        raise ValueError('Non-finite depth-recovery Z compensation')
    if abs(delta) < 1e-8:
        return task.get('object'), True
    current = _right_tool_mm_euler(robot, step5)
    if current is None:
        return task.get('object'), False
    start, euler = current
    target = start + [0., 0., delta]
    waypoint = dict(task, action='book_depth_z_compensation',
        target_position=target.tolist(), target_rotation=euler.tolist(),
        description=f'Depth fallback base-Z compensation: {delta:+.1f} mm')
    print(f'深度回退高度补偿：基座 Z {delta:+.1f} mm；保持当前 X、Y 和姿态，完成后才前伸。', flush=True)
    return execute(robot, waypoint, *args, **kwargs)
