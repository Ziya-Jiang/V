#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import os
import json
import argparse
from utils.transform_utils import (
    angle2rotation,
    pos_to_mat,
    mat_to_pos,
    rotation2angle,
)
import numpy as np
from book_action_hyperparameters import load_book_action_hyperparameters
from book_grasp import spine_sample_offsets_mm
from step4_manual_planning import load_object_stage_offsets
sys.stdout.reconfigure(encoding='utf-8')


# Matches the right-bowl orientation validated from the robot's current tool
# pose in the position-only IK test.
RIGHT_BOWL_SAFE_ROTATION = [-80.14893, 1.72794, 80.14893]
LEFT_BOWL_ROTATION = [180, 70, 90]
RIGHT_BOWL_GRASP_YAW_CANDIDATES_DEG = (
    0, 90, -90, 180
)
# Cart books have their spine along base X.  The physical jaw-opening axis is
# gripper X, so only +/-90-degree yaw keeps the line between the two fingers
# along base Y and therefore perpendicular to the spine.
RIGHT_CART_BOOK_GRASP_YAW_CANDIDATES_DEG = (
    90, -90
)
RIGHT_BOWL_TILT_CANDIDATES_DEG = (
    (0, 0),
    (10, 0), (-10, 0), (20, 0), (-20, 0),
)
RIGHT_CART_BOOK_TILT_CANDIDATES_DEG = (
    (0, 0),
)
RIGHT_BOWL_MOTION_ACTIONS = {
    "move_to_safe_entry",
    "move_above_grasp",
    "move_down_to_grasp",
    "lift_object",
}
RIGHT_BOWL_PICK_ACTIONS = RIGHT_BOWL_MOTION_ACTIONS | {"close_gripper"}
RIGHT_FORK_PICK_MOTION_ACTIONS = {
    "move_above_grasp",
    "move_down_to_grasp",
    "lift_object",
}
RIGHT_FORK_POSE_ACTIONS = RIGHT_FORK_PICK_MOTION_ACTIONS | {
    "close_gripper",
    "move_to_target",
    "move_down_to_place",
    "open_gripper",
    "move_up_after_release",
}
RIGHT_CART_BOOK_MOTION_ACTIONS = {
    "move_to_safe_entry",
    "move_above_grasp",
    "move_down_to_grasp",
    "lift_object",
    "move_to_target",
    "move_down_to_place",
    "move_up_after_release",
}
RIGHT_CART_BOOK_POSE_ACTIONS = RIGHT_CART_BOOK_MOTION_ACTIONS | {
    "close_gripper",
    "open_gripper",
}
RIGHT_CART_BOOK_VERTICAL_ACTIONS = {
    "move_down_to_grasp",
    "lift_object",
    "move_down_to_place",
    "move_up_after_release",
}
# These candidates are diagnostic only. They are sent with the __preview__
# prefix, so planner_to_panel must not forward them to the robot controller.
FORK_DIAGNOSTIC_TILT_CANDIDATES_DEG = (
    (10.0, 0.0), (-10.0, 0.0), (20.0, 0.0), (-20.0, 0.0),
    (0.0, 10.0), (0.0, -10.0), (0.0, 20.0), (0.0, -20.0),
)
# Offsets are expressed in the physical gripper frame: X crosses the fingers,
# Y follows the fork's long axis, and Z is world vertical.
FORK_DIAGNOSTIC_POSITION_OFFSETS_MM = (
    (15.0, 0.0, 0.0), (-15.0, 0.0, 0.0),
    (0.0, 15.0, 0.0), (0.0, -15.0, 0.0),
    (0.0, 0.0, 20.0), (0.0, 0.0, -20.0),
)
# Keep candidates away from the active CuRobo URDF limits. Values here are
# degrees and follow J1...J7 ordering.
RIGHT_ARM_SOFT_LOWER_DEG = np.array(
    [-360.0, -60.0, -360.0, -40.0, -360.0, -90.0, -360.0]
)
RIGHT_ARM_SOFT_UPPER_DEG = np.array(
    [360.0, 90.0, 360.0, 145.0, 360.0, 90.0, 360.0]
)
MIN_RIGHT_JOINT_LIMIT_MARGIN_DEG = 5.0
CART_BOOK_PREVIEW_TIMEOUT_S = 10.0
CART_BOOK_MIN_JOINT_MARGIN_DEG = 5.0
GRIPPER_OPEN_SETTLE_TIME_S = 1.5
GRIPPER_CLOSE_SETTLE_TIME_S = 2.0
OPERATOR_RESET_REQUESTED = "__operator_safe_reset__"
# arm_end_effector_r -> physical right-gripper frame in the active URDF.
RIGHT_GRIPPER_FROM_TOOL_ROTATION = np.diag([1.0, -1.0, -1.0])


def same_rotation(a, b, tol=1e-6):
    if not a or len(a) < 3:
        return False
    try:
        return all(abs(float(a[i]) - float(b[i])) <= tol for i in range(3))
    except (TypeError, ValueError):
        return False


def apply_runtime_rotation_guard(arm, target_rotation, object_id, object_id_to_type):
    object_type = object_id_to_type.get(object_id, 'cup')
    if arm == 'right' and object_type == 'bowl' and same_rotation(target_rotation, LEFT_BOWL_ROTATION):
        print(f'提示: right臂bowl任务使用了left侧姿态 {target_rotation}，自动改为 {RIGHT_BOWL_SAFE_ROTATION}')
        return RIGHT_BOWL_SAFE_ROTATION.copy()
    return target_rotation


def _rotation_x(degrees):
    radians = np.deg2rad(float(degrees))
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(radians), -np.sin(radians)],
            [0.0, np.sin(radians), np.cos(radians)],
        ]
    )


def _rotation_y(degrees):
    radians = np.deg2rad(float(degrees))
    return np.array(
        [
            [np.cos(radians), 0.0, np.sin(radians)],
            [0.0, 1.0, 0.0],
            [-np.sin(radians), 0.0, np.cos(radians)],
        ]
    )


def right_bowl_vertical_pick_rotation(yaw_deg, roll_deg=0.0, pitch_deg=0.0):
    """Return a right-gripper-down tool pose, optionally tilted about local axes."""
    yaw_rad = np.deg2rad(float(yaw_deg))
    gripper_x = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0])
    gripper_z = np.array([0.0, 0.0, -1.0])
    gripper_y = np.cross(gripper_z, gripper_x)
    world_from_gripper = np.column_stack([gripper_x, gripper_y, gripper_z])
    tilted_world_from_gripper = (
        world_from_gripper
        @ _rotation_y(pitch_deg)
        @ _rotation_x(roll_deg)
    )
    world_from_tool = (
        tilted_world_from_gripper @ RIGHT_GRIPPER_FROM_TOOL_ROTATION.T
    )
    return rotation2angle(world_from_tool).tolist()


def right_bowl_rim_offset_for_yaw(yaw_deg, nominal_rim_offset_mm):
    """Return an evenly distributed XY offset on the nominal bowl rim."""
    nominal_rim_offset_mm = np.asarray(nominal_rim_offset_mm, dtype=float)
    radius_mm = float(np.linalg.norm(nominal_rim_offset_mm))
    base_angle_rad = float(
        np.arctan2(
            nominal_rim_offset_mm[1],
            nominal_rim_offset_mm[0],
        )
    )
    angle_rad = base_angle_rad + np.deg2rad(float(yaw_deg))
    return np.array(
        [radius_mm * np.cos(angle_rad), radius_mm * np.sin(angle_rad)]
    )


def right_joint_limit_risk(
    final_joint_waypoint,
    min_margin_deg=MIN_RIGHT_JOINT_LIMIT_MARGIN_DEG,
):
    """Return a finite joint-limit risk, or None for an unsafe preview result."""
    limit_details = right_joint_limit_details(final_joint_waypoint)
    if limit_details is None:
        return None
    right_joint_deg, margin_deg = limit_details
    min_margin_deg = float(min_margin_deg)
    if np.min(margin_deg) < min_margin_deg:
        return None

    # Penalize proximity to every limit. A lower score means more joint-limit
    # clearance across the complete approach, descend, and lift sequence.
    risk = float(np.sum((min_margin_deg / margin_deg) ** 2))
    return risk, right_joint_deg, margin_deg


def right_joint_limit_details(final_joint_waypoint):
    """Return right-arm angles and soft-limit margins for a preview waypoint."""
    final_joint_waypoint = np.asarray(final_joint_waypoint, dtype=float)
    if final_joint_waypoint.shape != (2, 7):
        return None

    right_joint_deg = np.rad2deg(final_joint_waypoint[1])
    margin_deg = np.minimum(
        right_joint_deg - RIGHT_ARM_SOFT_LOWER_DEG,
        RIGHT_ARM_SOFT_UPPER_DEG - right_joint_deg,
    )
    if not np.isfinite(margin_deg).all():
        return None
    return right_joint_deg, margin_deg


def _bowl_preview_cache_key(action, yaw_deg, target_position_mm, rotation):
    """Create a stable key for an identical collision/IK preview request."""
    return (
        str(action),
        float(yaw_deg),
        tuple(np.round(np.asarray(target_position_mm[:3], dtype=float), 6)),
        tuple(np.round(np.asarray(rotation[:3], dtype=float), 6)),
    )


def _preview_right_bowl_pose(
    robot,
    pose_mm_deg,
    description,
    *,
    preview_cache=None,
    cache_key=None,
    preview_stats=None,
):
    """Run one preview, reusing only an identical already-checked pose."""
    if preview_cache is not None and cache_key in preview_cache:
        if preview_stats is not None:
            preview_stats["cache_hits"] += 1
        success, detail, final_joint = preview_cache[cache_key]
        return (
            success,
            detail,
            None if final_joint is None else final_joint.copy(),
        )

    if preview_stats is not None:
        preview_stats["planner_requests"] += 1
    success, detail, final_joint = robot.preview_right_cartesian_motion(
        pose_mm_deg,
        description=description,
        timeout=3.0,
    )
    if preview_cache is not None:
        preview_cache[cache_key] = (
            success,
            detail,
            None if final_joint is None else np.asarray(final_joint, dtype=float).copy(),
        )
    return success, detail, final_joint


def _log_right_bowl_preview_stats(start_time, preview_stats):
    """Print candidate-search timing and the number of actual planner calls."""
    elapsed_s = time.perf_counter() - start_time
    print(
        "右臂 bowl 候选预检："
        f"耗时 {elapsed_s:.2f}s，"
        f"Planner preview {preview_stats['planner_requests']} 次，"
        f"阶段缓存命中 {preview_stats['cache_hits']} 次。"
    )


def set_right_bowl_approach_height(
    tasks, object_id, approach_z_mm, *, workflow_label="bowl"
):
    """Set the right-arm pre-grasp to an absolute robot-base Z height."""
    for task in tasks:
        if (
            task.get("object") != object_id
            or task.get("arm") != "right"
            or task.get("action") != "move_above_grasp"
        ):
            continue
        target_position = task.get("target_position")
        if not target_position or len(target_position) < 3:
            continue
        original_z_mm = float(target_position[2])
        updated_position = list(target_position)
        updated_position[2] = approach_z_mm
        task["target_position"] = updated_position
        print(
            f"设置 {workflow_label} 预抓取高度："
            f"Z={original_z_mm:.1f}mm -> {approach_z_mm:.1f}mm "
            f"(机器人基坐标系绝对高度 {approach_z_mm:.1f}mm)。"
        )
    return approach_z_mm


def select_safest_bowl_stage_rotation(
    robot,
    *,
    object_id,
    action,
    target_position_mm,
    yaw_deg,
    preferred_rotation,
    tilt_candidates_deg=RIGHT_BOWL_TILT_CANDIDATES_DEG,
    preview_cache=None,
    preview_stats=None,
):
    """Independently find a safe local orientation for approach or lift."""
    rotation_candidates = [list(preferred_rotation)]
    for roll_deg, pitch_deg in tilt_candidates_deg:
        rotation = right_bowl_vertical_pick_rotation(
            yaw_deg, roll_deg, pitch_deg
        )
        if not any(
            np.allclose(rotation, existing, atol=1e-6)
            for existing in rotation_candidates
        ):
            rotation_candidates.append(rotation)

    safe_rotations = []
    rejected_ik = 0
    rejected_limits = 0
    for rotation_index, rotation in enumerate(rotation_candidates):
        cache_key = _bowl_preview_cache_key(
            action, yaw_deg, target_position_mm, rotation
        )
        success, _, final_joint = _preview_right_bowl_pose(
            robot,
            list(target_position_mm[:3]) + list(rotation),
            (
                f"right_bowl_stage:{object_id}:{action}:"
                f"yaw={yaw_deg}:rotation={rotation_index}"
            ),
            preview_cache=preview_cache,
            cache_key=cache_key,
            preview_stats=preview_stats,
        )
        if not success:
            rejected_ik += 1
            continue
        risk_result = right_joint_limit_risk(final_joint)
        if risk_result is None:
            rejected_limits += 1
            continue
        risk, _, margin_deg = risk_result
        safe_rotations.append(
            (risk, -float(np.min(margin_deg)), list(rotation))
        )

    if not safe_rotations:
        return None, {
            "checked": len(rotation_candidates),
            "rejected_ik": rejected_ik,
            "rejected_limits": rejected_limits,
        }

    risk, negative_margin, rotation = min(safe_rotations)
    return {
        "rotation": rotation,
        "risk": risk,
        "min_margin_deg": -negative_margin,
    }, None


def select_safest_right_bowl_candidate(
    robot, tasks, object_id, *, rerun_logger=None
):
    """Search yaw, local tilt, and rim-offset candidates for bowl/cart-book."""
    search_start_time = time.perf_counter()
    is_cart_book = any(
        task.get("object") == object_id
        and task.get("book_grasp_mode") == "cart"
        for task in tasks
    )
    workflow_label = "cart book" if is_cart_book else "bowl"
    if is_cart_book:
        detected_yaw_deg = next(
            (
                float(task["book_grasp_yaw_deg"])
                for task in tasks
                if task.get("object") == object_id
                and "book_grasp_yaw_deg" in task
            ),
            None,
        )
        if detected_yaw_deg is None or not np.isfinite(detected_yaw_deg):
            yaw_candidates_deg = RIGHT_CART_BOOK_GRASP_YAW_CANDIDATES_DEG
        else:
            opposite_yaw_deg = (
                (detected_yaw_deg + 360.0) % 360.0
            ) - 180.0
            yaw_candidates_deg = (
                detected_yaw_deg,
                opposite_yaw_deg,
            )
    else:
        yaw_candidates_deg = RIGHT_BOWL_GRASP_YAW_CANDIDATES_DEG
    tilt_candidates_deg = (
        RIGHT_CART_BOOK_TILT_CANDIDATES_DEG
        if is_cart_book
        else RIGHT_BOWL_TILT_CANDIDATES_DEG
    )
    preview_stats = {"planner_requests": 0, "cache_hits": 0}
    stage_preview_cache = {}
    if is_cart_book:
        stage_offsets = load_book_action_hyperparameters()
        offset_key_prefix = "book_cart"
    else:
        stage_offsets = load_object_stage_offsets()
        offset_key_prefix = "bowl"
    grasp_offset = stage_offsets[(offset_key_prefix, "move_down_to_grasp")]
    nominal_rim_offset_mm = np.array(
        [
            grasp_offset["offset_x_mm"],
            grasp_offset["offset_y_mm"],
        ],
        dtype=float,
    )
    approach_z_mm = stage_offsets[
        (offset_key_prefix, "move_above_grasp")
    ]["target_z_base_mm"]

    motion_actions = (
        RIGHT_CART_BOOK_MOTION_ACTIONS
        if is_cart_book
        else RIGHT_BOWL_MOTION_ACTIONS
    )
    pose_actions = (
        RIGHT_CART_BOOK_POSE_ACTIONS
        if is_cart_book
        else RIGHT_BOWL_PICK_ACTIONS
    )
    probe_tasks = [
        task for task in tasks
        if task.get("object") == object_id
        and task.get("arm") == "right"
        and task.get("action") in motion_actions
    ]
    if not probe_tasks:
        return True

    grasp_task = next(
        (
            task for task in probe_tasks
            if task.get("action") == "move_down_to_grasp"
        ),
        probe_tasks[0],
    )
    grasp_position_mm = np.asarray(
        grasp_task["target_position"][:3], dtype=float
    )
    set_right_bowl_approach_height(
        tasks,
        object_id,
        approach_z_mm,
        workflow_label=workflow_label,
    )
    spine_offsets_mm = [np.zeros(2)]
    if is_cart_book:
        spine_xy = next(
            (
                task.get("book_spine_xy")
                for task in tasks
                if task.get("object") == object_id and task.get("book_spine_xy")
            ),
            None,
        )
        spine_length_mm = next(
            (
                task.get("book_length_mm")
                for task in tasks
                if task.get("object") == object_id
                and task.get("book_length_mm") is not None
            ),
            None,
        )
        if spine_xy is not None and spine_length_mm is not None:
            spine_offsets_mm = spine_sample_offsets_mm(spine_xy, spine_length_mm)
        print(
            f"沿书脊取样 {len(spine_offsets_mm)} 个位置，"
            f"每个位置 {len(yaw_candidates_deg)} 个偏航，"
            f"共 {len(spine_offsets_mm) * len(yaw_candidates_deg) * len(tilt_candidates_deg)} 个候选。"
        )
    visualization_candidates = []
    for spine_offset_mm in spine_offsets_mm:
        for yaw_deg in yaw_candidates_deg:
            for roll_deg, pitch_deg in tilt_candidates_deg:
                rotation = right_bowl_vertical_pick_rotation(
                    yaw_deg, roll_deg, pitch_deg
                )
                if is_cart_book:
                    # Slide along the detected spine. Book XY offsets stay in
                    # the base frame and are not rotated with yaw.
                    rim_delta_xy_mm = np.asarray(spine_offset_mm, dtype=float).reshape(2)
                    rim_offset_xy_mm = nominal_rim_offset_mm + rim_delta_xy_mm
                else:
                    rim_offset_xy_mm = right_bowl_rim_offset_for_yaw(
                        yaw_deg, nominal_rim_offset_mm
                    )
                    rim_delta_xy_mm = (
                        rim_offset_xy_mm - nominal_rim_offset_mm
                    )
                candidate_position_mm = grasp_position_mm.copy()
                candidate_position_mm[:2] += rim_delta_xy_mm
                visualization_candidates.append(
                    {
                        "yaw_deg": yaw_deg,
                        "roll_deg": roll_deg,
                        "pitch_deg": pitch_deg,
                        "rim_offset_xy_mm": rim_offset_xy_mm.tolist(),
                        "rim_delta_xy_mm": rim_delta_xy_mm.tolist(),
                        "pose_mm_deg": candidate_position_mm.tolist() + rotation,
                    }
                )
    if rerun_logger is not None:
        rerun_logger.log_grasp_candidates(
            object_id=object_id,
            arm="right",
            candidates=visualization_candidates,
        )

    candidates = []
    rejected_ik = 0
    rejected_limits = 0
    for candidate_index, candidate in enumerate(visualization_candidates):
        success, detail, final_joint = _preview_right_bowl_pose(
            robot,
            candidate["pose_mm_deg"],
            (
                f"right_bowl_candidate:{object_id}:"
                f"yaw={candidate['yaw_deg']}:roll={candidate['roll_deg']}:"
                f"pitch={candidate['pitch_deg']}:offset={candidate['rim_offset_xy_mm']}"
            ),
            preview_stats=preview_stats,
        )
        if not success:
            rejected_ik += 1
            continue

        risk_result = right_joint_limit_risk(final_joint)
        if risk_result is None:
            rejected_limits += 1
            continue
        risk, _, margin_deg = risk_result
        candidates.append(
            (
                risk,
                -float(np.min(margin_deg)),
                candidate_index,
                candidate,
            )
        )

    if not candidates:
        _log_right_bowl_preview_stats(search_start_time, preview_stats)
        print(
            f"未找到 object={object_id} 的安全右臂 {workflow_label} IK 候选；"
            f"共检查 {len(visualization_candidates)} 个，"
            f"IK/碰撞失败 {rejected_ik} 个，"
            f"软限位余量不足 {rejected_limits} 个。"
        )
        return False

    selected_candidate = None
    for grasp_risk, grasp_negative_margin, candidate_index, candidate in sorted(candidates):
        offset_x_mm, offset_y_mm = candidate["rim_delta_xy_mm"]
        grasp_rotation = candidate["pose_mm_deg"][3:]
        total_risk = grasp_risk
        min_margin_deg = -grasp_negative_margin
        stage_rotations = {}
        sequence_is_safe = True
        for task in probe_tasks:
            if task is grasp_task:
                continue
            task_position = np.asarray(task["target_position"][:3], dtype=float)
            if task.get("action") != "move_to_safe_entry":
                task_position[:2] += [offset_x_mm, offset_y_mm]
            stage_result, rejection = select_safest_bowl_stage_rotation(
                robot,
                object_id=object_id,
                action=task["action"],
                target_position_mm=task_position,
                yaw_deg=candidate["yaw_deg"],
                preferred_rotation=grasp_rotation,
                tilt_candidates_deg=tilt_candidates_deg,
                preview_cache=stage_preview_cache,
                preview_stats=preview_stats,
            )
            if stage_result is None:
                print(
                    f"{workflow_label} 候选 #{candidate_index} 的 "
                    f"{task['action']} "
                    f"独立姿态搜索失败：检查 {rejection['checked']} 个，"
                    f"IK/碰撞失败 {rejection['rejected_ik']} 个，"
                    f"软限位余量不足 {rejection['rejected_limits']} 个。"
                )
                sequence_is_safe = False
                break
            stage_rotations[task["action"]] = stage_result["rotation"]
            total_risk += stage_result["risk"]
            min_margin_deg = min(
                min_margin_deg, stage_result["min_margin_deg"]
            )
        if sequence_is_safe:
            selected_candidate = (
                total_risk,
                -min_margin_deg,
                candidate_index,
                candidate,
                stage_rotations,
            )
            break

    if selected_candidate is None:
        _log_right_bowl_preview_stats(search_start_time, preview_stats)
        print(
            f"所有 {len(candidates)} 个抓取点候选在靠近或抬升预检时失败；"
            "拒绝执行。"
        )
        return False

    (
        risk,
        negative_margin,
        candidate_index,
        candidate,
        stage_rotations,
    ) = selected_candidate
    grasp_rotation = candidate["pose_mm_deg"][3:]
    offset_x_mm, offset_y_mm = candidate["rim_delta_xy_mm"]
    for task in tasks:
        if (
            task.get("object") == object_id
            and task.get("arm") == "right"
            and task.get("action") in pose_actions
        ):
            action = task.get("action")
            task["target_rotation"] = list(
                stage_rotations.get(action, grasp_rotation)
            )
            if action != "move_to_safe_entry":
                task["target_position"][0] += offset_x_mm
                task["target_position"][1] += offset_y_mm
    if is_cart_book:
        print(
            f"选择 {workflow_label} 候选 #{candidate_index}："
            f"yaw={candidate['yaw_deg']}°，"
            f"roll={candidate['roll_deg']}°，pitch={candidate['pitch_deg']}°，"
            f"书脊偏移={np.round(candidate['rim_delta_xy_mm'], 1).tolist()}mm，"
            f"风险={risk:.4f}，"
            f"最小软限位余量={-negative_margin:.1f}°。"
        )
    else:
        print(
            f"选择 {workflow_label} 候选 #{candidate_index}："
            f"yaw={candidate['yaw_deg']}°，"
            f"roll={candidate['roll_deg']}°，pitch={candidate['pitch_deg']}°，"
            f"rim_offset={candidate['rim_offset_xy_mm']}mm，风险={risk:.4f}，"
            f"最小软限位余量={-negative_margin:.1f}°。"
        )
    for action, rotation in stage_rotations.items():
        print(f"  {action} 独立姿态: {np.round(rotation, 2).tolist()}°")
    _log_right_bowl_preview_stats(search_start_time, preview_stats)
    if rerun_logger is not None:
        rerun_logger.log_grasp_candidates(
            object_id=object_id,
            arm="right",
            candidates=visualization_candidates,
            selected_candidate_index=candidate_index,
        )
    return True


def right_fork_axis_equivalent_rotations(base_rotation):
    """Return both directions of the same fork/gripper-Y alignment."""
    if base_rotation is None or len(base_rotation) < 3:
        return []
    try:
        base_rotation = [float(value) for value in base_rotation[:3]]
        world_from_tool = angle2rotation(*base_rotation)
    except (TypeError, ValueError):
        return []
    if not np.isfinite(world_from_tool).all():
        return []

    # A fork has no preferred sign along its long axis. Rotating 180 degrees
    # around world Z reverses physical gripper X/Y while retaining the vertical
    # grasp axis, so the fork remains parallel to gripper Y and between the
    # fingers. One sign can have a much better IK branch than the other.
    world_z_half_turn = np.diag([-1.0, -1.0, 1.0])
    equivalent_rotation = rotation2angle(
        world_z_half_turn @ world_from_tool
    ).tolist()
    candidates = [base_rotation]
    if not np.allclose(
        angle2rotation(*equivalent_rotation),
        world_from_tool,
        atol=1e-6,
    ):
        candidates.append(equivalent_rotation)
    return candidates


def right_fork_tilted_tool_rotation(base_rotation, roll_deg, pitch_deg):
    """Tilt a fork pose about physical gripper axes, then return tool Euler XYZ."""
    world_from_tool = angle2rotation(*base_rotation[:3])
    world_from_gripper = world_from_tool @ RIGHT_GRIPPER_FROM_TOOL_ROTATION
    tilted_world_from_gripper = (
        world_from_gripper
        @ _rotation_y(pitch_deg)
        @ _rotation_x(roll_deg)
    )
    tilted_world_from_tool = (
        tilted_world_from_gripper @ RIGHT_GRIPPER_FROM_TOOL_ROTATION.T
    )
    return rotation2angle(tilted_world_from_tool).tolist()


def _print_right_fork_preview_limit_details(label, final_joint_waypoint):
    """Print every right-arm soft-limit margin returned by one preview."""
    limit_details = right_joint_limit_details(final_joint_waypoint)
    if limit_details is None:
        print(f"{label}: 预检未返回有效的右臂 7 关节终点。")
        return False

    right_joint_deg, margin_deg = limit_details
    min_margin_deg = float(np.min(margin_deg))
    safe = min_margin_deg >= MIN_RIGHT_JOINT_LIMIT_MARGIN_DEG
    print(
        f"{label}: {'通过' if safe else '余量不足'}；"
        f"J1..J7={np.round(right_joint_deg, 1).tolist()}°；"
        f"软限位余量={np.round(margin_deg, 1).tolist()}°；"
        f"最小余量={min_margin_deg:.1f}°。"
    )
    return safe


def diagnose_right_fork_pregrasp_candidates(
    robot, object_id, target_position_mm, base_rotations
):
    """Test nearby fork poses through planner previews only; never execute motion."""
    print(
        "开始叉子预抓取诊断：仅发送 __preview__ 规划请求，不会下发实机运动。"
    )
    target_position = np.asarray(target_position_mm[:3], dtype=float)
    checked = 0
    safe = 0

    for branch_index, base_rotation in enumerate(base_rotations):
        world_from_tool = angle2rotation(*base_rotation[:3])
        world_from_gripper = world_from_tool @ RIGHT_GRIPPER_FROM_TOOL_ROTATION

        for roll_deg, pitch_deg in FORK_DIAGNOSTIC_TILT_CANDIDATES_DEG:
            rotation = right_fork_tilted_tool_rotation(
                base_rotation, roll_deg, pitch_deg
            )
            label = (
                f"fork 诊断 branch={branch_index} "
                f"roll={roll_deg:+.0f}° pitch={pitch_deg:+.0f}°"
            )
            success, detail, final_joint = robot.preview_right_cartesian_motion(
                target_position.tolist() + rotation,
                description=(
                    f"right_fork_diagnostic:{object_id}:"
                    f"branch={branch_index}:roll={roll_deg}:pitch={pitch_deg}"
                ),
                timeout=3.0,
            )
            checked += 1
            if not success:
                print(f"{label}: 规划预检失败：{detail}。")
                continue
            safe += int(
                _print_right_fork_preview_limit_details(label, final_joint)
            )

        for offset_x_mm, offset_y_mm, offset_z_mm in (
            FORK_DIAGNOSTIC_POSITION_OFFSETS_MM
        ):
            position = target_position.copy()
            position += (
                offset_x_mm * world_from_gripper[:, 0]
                + offset_y_mm * world_from_gripper[:, 1]
            )
            position[2] += offset_z_mm
            label = (
                f"fork 诊断 branch={branch_index} "
                f"offset=({offset_x_mm:+.0f}, {offset_y_mm:+.0f}, "
                f"{offset_z_mm:+.0f})mm"
            )
            success, detail, final_joint = robot.preview_right_cartesian_motion(
                position.tolist() + list(base_rotation),
                description=(
                    f"right_fork_diagnostic:{object_id}:"
                    f"branch={branch_index}:offset="
                    f"{offset_x_mm},{offset_y_mm},{offset_z_mm}"
                ),
                timeout=3.0,
            )
            checked += 1
            if not success:
                print(f"{label}: 规划预检失败：{detail}。")
                continue
            safe += int(
                _print_right_fork_preview_limit_details(label, final_joint)
            )

    print(
        f"叉子预抓取诊断完成：检查 {checked} 个候选，"
        f"最小软限位余量≥{MIN_RIGHT_JOINT_LIMIT_MARGIN_DEG:.1f}° 的候选 {safe} 个。"
    )


def select_safest_right_fork_candidate(
    robot, tasks, object_id, *, rerun_logger=None
):
    """Validate the restored legacy fork pose without changing its orientation."""
    planning_blockers = {
        str(task["planning_blocker"])
        for task in tasks
        if task.get("object") == object_id
        and task.get("planning_blocker")
    }
    if planning_blockers:
        print(
            f"fork object={object_id} 的感知结果未通过安全检查："
            + "; ".join(sorted(planning_blockers))
            + "。请重新采集物体姿态。"
        )
        return False

    probe_tasks = [
        task for task in tasks
        if task.get("object") == object_id
        and task.get("arm") == "right"
        and task.get("action") in RIGHT_FORK_PICK_MOTION_ACTIONS
    ]
    if not probe_tasks:
        return True

    approach_task = next(
        (
            task for task in probe_tasks
            if task.get("action") == "move_above_grasp"
        ),
        probe_tasks[0],
    )
    try:
        legacy_rotation = [
            float(value) for value in approach_task["target_rotation"][:3]
        ]
        if not np.isfinite(angle2rotation(*legacy_rotation)).all():
            raise ValueError("non-finite rotation")
    except (KeyError, TypeError, ValueError):
        print(f"fork object={object_id} 的目标姿态无效，拒绝执行。")
        return False
    # Reversing physical gripper X/Y by 180 degrees keeps the two-finger axis
    # perpendicular to the fork, but can select a very different IK branch.
    rotations = right_fork_axis_equivalent_rotations(legacy_rotation)

    started_at = time.perf_counter()
    safe_candidates = []
    rejected_planning = 0
    rejected_limits = 0
    visualization_candidates = []
    for candidate_index, rotation in enumerate(rotations):
        total_risk = 0.0
        min_margin_deg = float("inf")
        sequence_is_safe = True
        failed_action = None
        failed_detail = None
        for task in probe_tasks:
            target_position = task.get("target_position")
            if target_position is None or len(target_position) < 3:
                sequence_is_safe = False
                failed_action = task.get("action")
                failed_detail = "invalid target_position"
                rejected_planning += 1
                break
            pose_mm_deg = list(target_position[:3]) + list(rotation)
            success, detail, final_joint = (
                robot.preview_right_cartesian_motion(
                    pose_mm_deg,
                    description=(
                        f"right_fork_candidate:{object_id}:"
                        f"branch={candidate_index}:"
                        f"action={task.get('action')}"
                    ),
                    timeout=3.0,
                )
            )
            if not success:
                sequence_is_safe = False
                failed_action = task.get("action")
                failed_detail = detail
                rejected_planning += 1
                break
            limit_safe = _print_right_fork_preview_limit_details(
                f"fork 候选 #{candidate_index} 的 {task.get('action')} 预检",
                final_joint,
            )
            risk_result = right_joint_limit_risk(final_joint)
            if risk_result is None:
                sequence_is_safe = False
                failed_action = task.get("action")
                failed_detail = (
                    "soft joint-limit margin is insufficient"
                    if not limit_safe
                    else "invalid right-arm joint-limit result"
                )
                rejected_limits += 1
                break
            risk, _, margin_deg = risk_result
            total_risk += risk
            min_margin_deg = min(
                min_margin_deg, float(np.min(margin_deg))
            )

        visualization_candidates.append(
            {
                "branch": candidate_index,
                "pose_mm_deg": (
                    list(approach_task["target_position"][:3])
                    + list(rotation)
                ),
                "safe": sequence_is_safe,
                "failed_action": failed_action,
                "detail": (
                    None if failed_detail is None else str(failed_detail)
                ),
            }
        )
        if sequence_is_safe:
            safe_candidates.append(
                (
                    total_risk,
                    -min_margin_deg,
                    candidate_index,
                    list(rotation),
                )
            )
        else:
            print(
                f"fork 候选 #{candidate_index} 在 {failed_action} 预检失败："
                f"{failed_detail}"
            )

    if rerun_logger is not None:
        rerun_logger.log_grasp_candidates(
            object_id=object_id,
            arm="right",
            candidates=visualization_candidates,
        )

    elapsed_s = time.perf_counter() - started_at
    if not safe_candidates:
        print(
            f"fork 抓取位姿未通过右臂安全预检；"
            f"检查 {len(rotations)} 个等效姿态，"
            f"规划/碰撞失败 {rejected_planning} 个，"
            f"软限位余量不足 {rejected_limits} 个，"
            f"耗时 {elapsed_s:.2f}s。"
        )
        return False

    risk, negative_margin, candidate_index, selected_rotation = min(
        safe_candidates
    )
    print(
        f"fork 等效抓取位姿预检通过 #{candidate_index}："
        f"姿态={np.round(selected_rotation, 2).tolist()}°，"
        f"风险={risk:.4f}，最小软限位余量={-negative_margin:.1f}°，"
        f"耗时 {elapsed_s:.2f}s。"
    )
    for task in tasks:
        if (
            task.get("object") == object_id
            and task.get("arm") == "right"
            and task.get("action") in RIGHT_FORK_POSE_ACTIONS
        ):
            task["target_rotation"] = list(selected_rotation)
    if rerun_logger is not None:
        rerun_logger.log_grasp_candidates(
            object_id=object_id,
            arm="right",
            candidates=visualization_candidates,
            selected_candidate_index=candidate_index,
        )
    return True


def select_safest_right_cart_book_candidate(
    robot, tasks, object_id, *, rerun_logger=None
):
    """Preflight one direct above/down/lift sequence without extra waypoints."""
    probe_tasks = [
        task for task in tasks
        if task.get("object") == object_id
        and task.get("arm") == "right"
        and task.get("action") in RIGHT_CART_BOOK_MOTION_ACTIONS
    ]
    if not probe_tasks:
        return True
    approach_task = next(
        (
            task for task in probe_tasks
            if task.get("action") == "move_above_grasp"
        ),
        probe_tasks[0],
    )
    # Cart book uses the same vertical-down tool pose as bowl (yaw=0).
    base_rotation = approach_task.get("target_rotation")
    try:
        rotation_is_valid = (
            base_rotation is not None
            and len(base_rotation) >= 3
            and np.isfinite(angle2rotation(*base_rotation[:3])).all()
        )
    except (TypeError, ValueError):
        rotation_is_valid = False
    if not rotation_is_valid:
        print("cart book 竖直向下抓取姿态无效，拒绝执行。")
        return False
    print(
        "cart book 与抓碗相同末端姿态（竖直向下）："
        f"{np.round(base_rotation[:3], 2).tolist()}°。"
    )
    rotation_candidates = [
        {
            "branch": 0,
            "roll_deg": float(base_rotation[0]),
            "rotation": [float(value) for value in base_rotation[:3]],
        }
    ]

    started_at = time.perf_counter()
    safe_candidates = []
    visualization_candidates = []
    for candidate_index, candidate in enumerate(rotation_candidates):
        rotation = candidate["rotation"]
        total_risk = 0.0
        min_margin_deg = float("inf")
        failed_action = None
        failed_detail = None
        for task in probe_tasks:
            action = task.get("action")
            target_position = task.get("target_position")
            if target_position is None or len(target_position) < 3:
                failed_action = action
                failed_detail = "invalid target_position"
                break
            preview_timeout_s = CART_BOOK_PREVIEW_TIMEOUT_S
            min_margin_deg_for_action = CART_BOOK_MIN_JOINT_MARGIN_DEG
            success, detail, final_joint = (
                robot.preview_right_cartesian_motion(
                    list(target_position[:3]) + list(rotation),
                    description=(
                        f"right_cart_book_candidate:{object_id}:"
                        f"branch={candidate['branch']}:"
                        f"roll={candidate['roll_deg']}:"
                        f"action={action}"
                    ),
                    timeout=preview_timeout_s,
                )
            )
            if not success:
                failed_action = action
                failed_detail = detail
                break
            risk_result = right_joint_limit_risk(
                final_joint,
                min_margin_deg=min_margin_deg_for_action,
            )
            if risk_result is None:
                failed_action = action
                failed_detail = (
                    "right-arm joint-limit margin is insufficient "
                    f"(<{min_margin_deg_for_action:.1f}deg)"
                )
                break
            risk, _, margin_deg = risk_result
            total_risk += risk
            min_margin_deg = min(
                min_margin_deg,
                float(np.min(margin_deg)),
            )

        candidate_safe = failed_action is None
        visualization_candidates.append(
            {
                "branch": candidate_index,
                "equivalent_branch": candidate["branch"],
                "roll_deg": candidate["roll_deg"],
                "pose_mm_deg": (
                    list(approach_task["target_position"][:3])
                    + list(rotation)
                ),
                "safe": candidate_safe,
                "failed_action": failed_action,
                "detail": failed_detail,
            }
        )
        if candidate_safe:
            safe_candidates.append(
                (
                    abs(candidate["roll_deg"]),
                    total_risk,
                    -min_margin_deg,
                    candidate_index,
                    list(rotation),
                )
            )
        else:
            print(
                f"cart book 候选 #{candidate_index} 在 {failed_action} "
                f"预检失败：{failed_detail}。"
            )

    if rerun_logger is not None:
        rerun_logger.log_grasp_candidates(
            object_id=object_id,
            arm="right",
            candidates=visualization_candidates,
        )
    if not safe_candidates:
        print(
            "cart book 的唯一抓取姿态未通过安全预检；"
            "未发送任何抓取运动。"
        )
        return False

    tilt_deg, risk, negative_margin, candidate_index, selected_rotation = min(
        safe_candidates
    )
    for task in tasks:
        if (
            task.get("object") == object_id
            and task.get("arm") == "right"
            and task.get("action") in RIGHT_CART_BOOK_POSE_ACTIONS
        ):
            task["target_rotation"] = list(selected_rotation)
    elapsed_s = time.perf_counter() - started_at
    print(
        "cart book 唯一抓取姿态预检通过："
        f"姿态={np.round(selected_rotation, 2).tolist()}°，"
        f"最小软限位余量={-negative_margin:.1f}°，"
        f"耗时 {elapsed_s:.2f}s。"
    )
    if rerun_logger is not None:
        rerun_logger.log_grasp_candidates(
            object_id=object_id,
            arm="right",
            candidates=visualization_candidates,
            selected_candidate_index=candidate_index,
        )
    return True


def current_tool_orientation_as_euler(robot, arm):
    """Return the current tool orientation as this executor's Euler convention."""
    current_state = robot.get_current_state()
    if current_state is None:
        return None
    arm_key = f"{arm}_arm"
    arm_state = current_state.get(arm_key)
    if arm_state is None:
        print(f"无法读取 {arm} 臂当前工具姿态。")
        return None
    orientation_wxyz = arm_state.get("orientation")
    if orientation_wxyz is None or len(orientation_wxyz) != 4:
        print(f"{arm} 臂当前工具四元数无效: {orientation_wxyz}")
        return None
    pose_mm_deg = robot._convert_quaternion_to_pos_rot(
        [0.0, 0.0, 0.0, *orientation_wxyz]
    )
    return pose_mm_deg[3:]


def expected_arm_for_target_y(y_value):
    # Verified from planner FK: left/index0 is on +Y, right/index1 on -Y.
    return 'left' if float(y_value) >= 0 else 'right'


def validate_arm_target_side(arm, target_position, action, *, book_grasp_mode=None):
    # cart+ deliberately uses the right arm, including targets across Y=0.
    # Y sign is not a reachability test. Keep the normal MotionGen execution
    # path (IK, joint limits and configured collision checks) authoritative.
    if book_grasp_mode == 'cart+' and arm == 'right':
        return True
    pickup_side_actions = {
        'move_to_safe_entry',
        'move_above_grasp',
        'move_down_to_grasp',
        'close_gripper',
        'lift_object',
    }
    if arm not in ('left', 'right') or action not in pickup_side_actions:
        return True
    if not target_position or len(target_position) < 2:
        return True

    try:
        expected_arm = expected_arm_for_target_y(target_position[1])
    except (TypeError, ValueError):
        return True

    if arm == expected_arm:
        return True

    print(
        f"错误: 任务左右臂分配与当前cuRobo坐标约定不一致: "
        f"action={action}, arm={arm}, target_y={target_position[1]}, expected_arm={expected_arm}. "
        "left/index0应执行正Y目标，right/index1应执行负Y目标；请重新生成detailed_subtasks_manual.json。"
    )
    return False


def load_detailed_tasks(tasks_file):
    """加载详细任务文件"""
    if not os.path.exists(tasks_file):
        print(f"错误: 任务文件不存在: {tasks_file}")
        return None
    with open(tasks_file, 'r', encoding='utf-8') as f:
        tasks = json.load(f)
    
    print(f"成功加载 {len(tasks)} 个详细任务")
    return tasks
# 物体偏移配置 - 从RobotArm类中复制
object_poses = {
    'bowl': {
        'tip2tool_distance': 0,#100,  # mm
        'rotation_y_deg': -0,    # 绕Y轴旋转角度
        'rotation_x_deg': 0,      # 绕X轴旋转角度
        'rotation_z_deg' : 0,
        'position_offset': [0, 0, 0]  # [x, y, z] 位置偏移
    },
    'cup': {
        'tip2tool_distance': 0, #40,#70,
        'rotation_y_deg': 0,
        'rotation_x_deg': 0,
        'rotation_z_deg' :-0,
        'position_offset': [0, 0, 0]
    },
    'plate': {
        'tip2tool_distance': 0,
        'rotation_y_deg': -0,
        'rotation_x_deg': 0,
        'rotation_z_deg' : 0,
        'position_offset': [0, 0, 0]
    },
    'brush': {
        'tip2tool_distance': 0,
        'rotation_y_deg': 0,
        'rotation_x_deg': 0,
        'rotation_z_deg' : 0,
        'position_offset': [0, 0, 0]
    },
    'spoon': {
        'tip2tool_distance': 0,
        'rotation_y_deg': 0,
        'rotation_x_deg': 0,
        'rotation_z_deg' : 0,
        'position_offset': [0, 0, 0]
    },
    'fork': {
        'tip2tool_distance': 0,
        'rotation_y_deg': 0,
        'rotation_x_deg': 0,
        'rotation_z_deg' : 0,
        'position_offset': [0, 0, 0]
    },
    'bread': {
        'tip2tool_distance': 0, #50,
        'rotation_y_deg': -0,
        'rotation_x_deg': 0,
        'rotation_z_deg' : 0,
        'position_offset': [-0, 0, 0]
    }
}

def load_object_categories(categories):
    """从vlm_categories.json加载物体分类信息"""

    
    object_id_to_type = {}
    for item in categories:
        object_id = f"object_{item['id']}"
        category = item['category']
        
        # 处理特殊类型映射
        if category == 'other':
            # 对于'other'类型，尝试从名称推断
            name = item['name'].lower()
            if 'fork' in name:
                category = 'fork'
            elif 'spoon' in name:
                category = 'spoon'
            elif 'brush' in name:
                category = 'brush'
            # 如果无法推断，保持为'other'，后续会使用默认配置
        elif category == 'plate':
            # 盘子使用bowl的配置
            category = 'bowl'
            
        object_id_to_type[object_id] = category
        print(f"物体映射: {object_id} -> {category} ({item['name']})")
    
    return object_id_to_type

def apply_object_offset(position, rotation, object_id, object_id_to_type):
    """根据物体类型应用偏移"""
    if not object_id or object_id not in object_id_to_type:
        return position[:], rotation[:]  # 返回副本，没有偏移
    
    object_type = object_id_to_type[object_id]
    if object_type not in object_poses:
        print(f"警告: 未找到物体类型 '{object_type}' 的配置，使用原始位置")
        return position[:], rotation[:]
    
    config = object_poses[object_type]
    
    # 复制原始位置和旋转
    pos_rot = position + rotation
    
    angle_rad = np.deg2rad(config['rotation_y_deg'])
    R_y = np.array([
        [np.cos(angle_rad),  0, np.sin(angle_rad), 0],
        [0,                  1, 0, 0],
        [-np.sin(angle_rad), 0, np.cos(angle_rad), 0],
        [0,                 0,                 0, 1]
    ])

    # 构建绕X轴旋转矩阵
    angle_rad = np.deg2rad(config['rotation_x_deg'])
    R_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(angle_rad), -np.sin(angle_rad), 0],
        [0, np.sin(angle_rad), np.cos(angle_rad), 0],
        [0,                 0,                 0, 1]
    ])
    angle_rad = np.deg2rad(config['rotation_z_deg'])
    R_z = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad), 0, 0],
        [np.sin(angle_rad),  np.cos(angle_rad), 0, 0],
        [0,                 0,                 1, 0],
        [0,                 0,                 0, 1]
    ])
    def tip2tool_transmat(tip2tool_dist):
        tip2tool = np.eye(4)
        tip2tool[0, 3] = tip2tool_dist
        return tip2tool
    # 计算工具位置
    tip2tool = tip2tool_transmat(config['tip2tool_distance'])
    tip2w = pos_to_mat(pos_rot)
    tool2tip = np.linalg.inv(tip2tool) 
    tool2w = tip2w @ tool2tip 
    
    # 应用旋转
    tool2w = tool2w @ R_x @ R_y @ R_z
    # 转换为位置姿态格式
    tool_pos_rot = mat_to_pos(tool2w)
    
    # 应用位置偏移
    position_offset = config['position_offset']
    tool_pos_rot[0] += position_offset[0]  # x
    tool_pos_rot[1] += position_offset[1]  # y
    tool_pos_rot[2] += position_offset[2]  # z

    
    # print(f"物体 {object_id} ({object_type}) 应用偏移:")
    # print(f"  pos_rot: {position} {rotation} -> new_pos_rot: {tool_pos_rot[:3]} {tool_pos_rot[3:]}")
    return tool_pos_rot[:3].tolist(), tool_pos_rot[3:].tolist()

def execute_cart_book_vertical_motion(
    robot,
    target_position,
    target_rotation,
    *,
    action,
    object_category,
    right_gripper,
    max_z_step_mm,
):
    """Execute a monotonic cart-book Z motion as short Cartesian segments."""
    current_state = robot.get_current_state()
    if current_state is None:
        print(f"{action} 无法读取当前右手位姿，拒绝分段垂直运动。")
        return False
    current_position_mm = (
        np.asarray(current_state["right_arm"]["position"], dtype=float) * 1000.0
    )
    target_position_mm = np.asarray(target_position[:3], dtype=float)
    delta_z_mm = float(target_position_mm[2] - current_position_mm[2])
    segment_count = max(
        1,
        int(np.ceil(abs(delta_z_mm) / max_z_step_mm)),
    )
    segment_z_values = np.linspace(
        current_position_mm[2],
        target_position_mm[2],
        segment_count + 1,
    )[1:]
    poses = [
        [
            float(target_position_mm[0]),
            float(target_position_mm[1]),
            float(z_mm),
            *[float(value) for value in target_rotation[:3]],
        ]
        for z_mm in segment_z_values
    ]

    for segment_index, pose in enumerate(poses, start=1):
        success, _, final_joint = _preview_right_bowl_pose(
            robot,
            pose,
            (
                f"cart_book_vertical:{action}:"
                f"{segment_index}/{segment_count}:z={pose[2]:.1f}"
            ),
        )
        if not success or right_joint_limit_risk(final_joint) is None:
            print(
                f"{action} 垂直分段 #{segment_index}/{segment_count} "
                f"Z={pose[2]:.1f}mm 未通过 IK/碰撞/软限位预检。"
            )
            return False

    print(
        f"{action} 使用 {segment_count} 个单调垂直分段："
        f"Z={current_position_mm[2]:.1f}mm -> "
        f"{target_position_mm[2]:.1f}mm，"
        f"每段≤{max_z_step_mm:.1f}mm。"
    )
    for segment_index, pose in enumerate(poses, start=1):
        print(
            f"  执行垂直分段 {segment_index}/{segment_count}: "
            f"Z={pose[2]:.1f}mm"
        )
        result = robot.master_control(
            None,
            pose,
            None,
            right_gripper,
            action=action,
            object_category=object_category,
        )
        if not result:
            return False
    return True


def execute_single_task(
    robot,
    task,
    task_index,
    object_id_to_type,
    *,
    preserve_current_orientation_for_approach=False,
):
    """执行单个任务"""
    action = task.get('action')
    arm = task.get('arm')
    object_id = task.get('object')
    description = task.get('description', 'N/A')
    
    print(f"执行任务 {task_index+1}: arm: {arm} - action: {action} - object: {object_id} - description: {description}")
    
    # 获取目标位置和旋转
    target_pos = task.get('target_position') or [0, 0, 0]
    target_rot = task.get('target_rotation') or [0, 0, 0]
    if (
        preserve_current_orientation_for_approach
        and action == "move_above_grasp"
    ):
        current_euler = current_tool_orientation_as_euler(robot, arm)
        if current_euler is None:
            print("位置 IK 验证已取消：无法读取当前工具方向。")
            return object_id, False
        print(
            "位置 IK 验证：保持当前工具方向，"
            f"忽略任务方向 {target_rot}，使用 {current_euler}"
        )
        target_rot = current_euler

    if not validate_arm_target_side(
        arm, target_pos, action, book_grasp_mode=task.get('book_grasp_mode')
    ):
        return object_id, False

    # 应用物体偏移
    # adjusted_pos, adjusted_rot = apply_object_offset(target_pos, target_rot, object_id, object_id_to_type)
    adjusted_pos, adjusted_rot = target_pos, target_rot
    # 合并为6DOF姿态 [x, y, z, rx, ry, rz]
    pose_6dof = adjusted_pos[:3] + adjusted_rot[:3]
    print(f"execute {arm} pose_6dof", pose_6dof)
    # 准备控制参数
    left_pos = None
    right_pos = None
    left_gripper = None
    right_gripper = None
    
    # 根据机械臂设置位置
    if arm == 'left':
        left_pos = pose_6dof
    elif arm == 'right':
        right_pos = pose_6dof
    
    # 根据动作类型设置夹爪状态
    if action == 'close_gripper':
        if arm == 'left':
            left_gripper = 1  # 关闭
        elif arm == 'right':
            right_gripper = 1  # 关闭

        print("object_id_to_type: ", object_id_to_type, object_id)
        # This task only changes the gripper. Do not resend the unchanged arm
        # pose as a second Cartesian motion command.
        left_pos = None
        right_pos = None

            
    elif action == 'open_gripper':
        if arm == 'left':
            left_gripper = 0  # 打开
        elif arm == 'right':
            right_gripper = 0  # 打开
        left_pos = None
        right_pos = None
            
    elif action in [
        'move_to_safe_entry', 'move_above_grasp', 'move_down_to_grasp'
    ]:
        # 这些动作确保夹爪是打开的
        if arm == 'left':
            left_gripper = 0  # 打开
        elif arm == 'right':
            right_gripper = 0  # 打开
    elif action in ["reset_all"]:
        # Use the verified joint reset path instead of stale Cartesian reset_pos.
        pass
    elif action in ["reset_for_fork"]:
        left_pos= [457.70407,589.74236,726.4593,162.26496827,-8.00807842,16.72914414]  
        right_pos=[416.47378,-563.9223,679.55184,-162.45669699,-7.94381229,-16.67909464]  
    # elif action in ['reset_pos']:
    #     if arm =='left':
    #         left_pos=[-999,-999,-999,0,0,0]
    #         left_gripper = 0  # 打开
    #     elif arm =="right":
    #         right_pos=[999,999,999,0,0,0]
    #         right_gripper = 0  # 打开
    # 调用机器人控制函数
    # print("控制参数:",left_gripper, right_gripper, action)
    if action =="reset_pos":
        if arm =="left":
            result = robot.control_joint_position(left_pos=True, right_pos=False, action=action)
        elif arm =="right":
            result = robot.control_joint_position(left_pos=False, right_pos=True, action=action)
    elif action =="first_reset":
        result = robot.control_joint_position(left_pos=True, right_pos=True, action=action)
        # result = True
    elif action =="reset_all":
        result = robot.control_joint_position(left_pos=True, right_pos=True, action=action)
    elif (
        arm == "right"
        and task.get("book_grasp_mode") == "cart"
        and action in RIGHT_CART_BOOK_VERTICAL_ACTIONS
    ):
        book_parameters = load_book_action_hyperparameters()
        max_z_step_mm = book_parameters[
            ("book_cart", action)
        ]["max_z_step_mm"]
        result = execute_cart_book_vertical_motion(
            robot,
            adjusted_pos,
            adjusted_rot,
            action=action,
            object_category=object_id_to_type.get(object_id, 'book'),
            right_gripper=right_gripper,
            max_z_step_mm=max_z_step_mm,
        )
    else:
        result = robot.master_control(left_pos, right_pos, left_gripper, right_gripper, action=action, object_category=object_id_to_type.get(object_id, 'cup'))
        if result and action in {'close_gripper', 'open_gripper'}:
            settle_time_s = (
                GRIPPER_CLOSE_SETTLE_TIME_S
                if action == 'close_gripper'
                else GRIPPER_OPEN_SETTLE_TIME_S
            )
            print(f"等待夹爪动作稳定 {settle_time_s:.1f}s。")
            getattr(robot, 'operator_sleep', time.sleep)(settle_time_s)
    # else:
    # robot.master_control(left_pos, right_pos, left_gripper, right_gripper)
    
    return object_id, result
        
def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description="执行机器人任务序列")
    parser.add_argument("--output", type=str, 
                       default="./output/scene_02", 
                       help="详细任务文件路径")
    parser.add_argument("--taskname", type=str, 
                       default="detailed_subtasks.json", 
                       help="详细任务文件路径")
    return parser.parse_args()


def recover_to_reset_after_failure(
    robot, failed_task, *, operator_requested=False
):
    """Open both grippers, then return once to the verified safe joint pose."""
    del failed_task
    if operator_requested:
        print(
            "操作员请求中止当前任务：先打开双夹爪，"
            "再将双臂调整到已验证的安全关节准备位。"
        )
        action_prefix = "operator_reset"
    else:
        print(
            "任务失败后的恢复：先发送双夹爪打开命令，"
            "然后尝试将双臂调整到已验证的安全关节准备位。"
        )
        action_prefix = "failure_recovery"
    try:
        robot.master_control(
            left_gripper=0,
            right_gripper=0,
            action=f"{action_prefix}_open_grippers",
            object_category="bowl",
        )
        time.sleep(1.5)
        recovered = robot.control_joint_position(
            left_pos=True,
            right_pos=True,
            action=f"{action_prefix}_safe_reset",
            force=True,
        )
    except Exception as error:
        context = "操作员复位" if operator_requested else "失败恢复"
        print(f"{context}时打开夹爪或安全复位异常: {error}")
        return False

    if recovered:
        if operator_requested:
            print("夹爪已打开，双臂已安全复位，即将返回模式选择。")
        else:
            print("失败后夹爪已打开，双臂已调整至安全关节准备位。")
    else:
        print(
            "夹爪打开命令已发送，但双臂安全复位未完成；"
            "请保持现场安全并检查 planner_to_panel 日志。"
        )
    return bool(recovered)


def confirm_step_execution(task_index, task):
    """Wait for an operator before sending one robot action."""
    action = task.get("action", "unknown")
    arm = task.get("arm", "unknown")
    object_id = task.get("object", "unknown")
    try:
        answer = input(
            f"\n[步骤执行] 下一步 {task_index + 1}: "
            f"action={action}, arm={arm}, object={object_id}。"
            "按 Enter 执行；输入 q 取消后续动作："
        ).strip().lower()
    except EOFError:
        print("未检测到交互输入，取消后续动作。")
        return False
    return answer not in {"q", "quit", "exit"}


def operator_requests_reset_after_step(task_index):
    """Pause at a completed step and let the operator request a safe reset."""
    while True:
        try:
            answer = input(
                f"任务 {task_index + 1} 已完成："
                "按 Enter 执行下一任务；输入 r 安全复位并返回模式选择："
            ).strip().lower()
        except EOFError:
            print("未检测到交互输入，将安全复位并返回模式选择。")
            return True
        if answer in {"", "c", "continue"}:
            return False
        if answer in {"r", "reset"}:
            return True
        print("无效输入：请按 Enter 继续，或输入 r 复位返回。")


def fully_open_gripper_before_pick(robot, arm, object_id, object_type):
    """Fully open the assigned gripper before an object's first pick motion."""
    if arm not in {"left", "right"}:
        print(
            f"无法在抓取前完全打开夹爪：object={object_id}, arm={arm}。"
        )
        return False

    print(
        f"{object_type} object={object_id} 首次动作前，"
        f"发送 {arm} 侧夹爪完全打开指令。"
    )
    result = robot.master_control(
        left_gripper=0 if arm == "left" else None,
        right_gripper=0 if arm == "right" else None,
        action=f"pregrasp_full_open:{object_type}:{object_id}",
        object_category=object_type,
    )
    if result is not True:
        print(f"{arm} 侧夹爪完全打开指令发送失败。")
        return False
    time.sleep(GRIPPER_OPEN_SETTLE_TIME_S)
    return True


def _log_rerun_action_transition(
    rerun_logger,
    task_index,
    task_total,
    task,
    status,
    *,
    result=None,
    error=None,
):
    """Keep visualization failures from interrupting hardware execution."""
    if rerun_logger is None:
        return
    try:
        rerun_logger.log_execution_action(
            task_index=task_index,
            task_total=task_total,
            task=task,
            status=status,
            result=result,
            error=error,
        )
    except Exception as log_error:  # noqa: BLE001
        print(f"Rerun 动作状态记录失败（不影响机器人执行）：{log_error}")


def drop_cart_book_reset_all_tasks(tasks):
    """Keep a grasped cart book at the left-grid place pose instead of resetting."""
    if not any(
        task.get("book_grasp_mode") in ("cart", "cart+") for task in tasks
    ):
        return tasks
    filtered = [task for task in tasks if task.get("action") != "reset_all"]
    dropped = len(tasks) - len(filtered)
    if dropped:
        print(
            f"{'cart+' if any(task.get('book_grasp_mode') == 'cart+' for task in tasks) else 'cart book'}"
            f" 提起后放到左侧格子，跳过 {dropped} 个 reset_all。"
        )
    return filtered


def insert_cart_book_safe_entry_tasks(tasks):
    """Insert a safe entry before approaching a cart book."""
    if not tasks:
        return tasks
    parameters = load_book_action_hyperparameters()
    config = parameters[("book_cart", "move_to_safe_entry")]
    if config["reference"] != "robot_base":
        raise ValueError(
            "book_cart/move_to_safe_entry 必须使用 robot_base reference"
        )
    entry_position = [
        float(config["offset_x_mm"]),
        float(config["offset_y_mm"]),
        float(config["target_z_base_mm"]),
    ]
    expanded = []
    for task in tasks:
        if (
            task.get("action") == "move_above_grasp"
            and task.get("arm") == "right"
            and task.get("book_grasp_mode") == "cart"
            and (
                not expanded
                or expanded[-1].get("action") != "move_to_safe_entry"
                or expanded[-1].get("object") != task.get("object")
            )
        ):
            entry = dict(task)
            entry["action"] = "move_to_safe_entry"
            entry["target_position"] = list(entry_position)
            entry["target_rotation"] = list(
                task.get("target_rotation") or [0.0, 0.0, 0.0]
            )
            entry["description"] = (
                "先在车外安全入口将夹爪转为向下，再水平移动到书上方"
            )
            expanded.append(entry)
        expanded.append(task)
    return expanded


def execute_main(
    tasks,
    object_id_to_type,
    robot,
    *,
    preserve_current_orientation_for_approach=False,
    rerun_logger=None,
    step_by_step=False,
    pause_between_tasks=False,
):
    
    if tasks is None:
        return None, False

    tasks = drop_cart_book_reset_all_tasks(
        insert_cart_book_safe_entry_tasks(tasks)
    )
    # 执行任务序列
    print(f"开始执行 {len(tasks)} 个任务...")
    preflighted_right_forks = set()
    preflighted_right_cart_books = set()
    preopened_objects = set()

    for i, task in enumerate(tasks):
        object_id = task.get("object")
        object_type = object_id_to_type.get(object_id)
        action = task.get("action")

        if (
            object_id not in preflighted_right_cart_books
            and object_type == "book"
            and task.get("arm") == "right"
            and action in {"move_to_safe_entry", "move_above_grasp"}
            and task.get("book_grasp_mode") == "cart"
        ):
            print(
                f"执行 cart book object={object_id} 前，"
                "位置与高度使用 book offset，并执行分阶段预检；"
                "禁用倾斜，沿整条书脊取样，每个位置使用点云薄边及其 180° 等效方向，"
                "确保夹爪竖直向下并沿书的实际薄边闭合。"
            )
            if not select_safest_right_bowl_candidate(
                robot,
                tasks,
                object_id,
                rerun_logger=rerun_logger,
            ):
                _log_rerun_action_transition(
                    rerun_logger,
                    i,
                    len(tasks),
                    task,
                    "failed",
                    error="cart book bowl-style candidates failed safety preflight",
                )
                recover_to_reset_after_failure(robot, task)
                return object_id, False
            preflighted_right_cart_books.add(object_id)

        if (
            object_id not in preflighted_right_forks
            and object_type == "fork"
            and task.get("arm") == "right"
            and task.get("action") in RIGHT_FORK_PICK_MOTION_ACTIONS
        ):
            print(
                f"执行 fork object={object_id} 前，预检原方向及其等效 180° 方向。"
            )
            if not select_safest_right_fork_candidate(
                robot,
                tasks,
                object_id,
                rerun_logger=rerun_logger,
            ):
                print(
                    f"fork object={object_id} 的两个等效方向均不可安全规划，"
                    "拒绝发送实机抓取命令。"
                )
                _log_rerun_action_transition(
                    rerun_logger,
                    i,
                    len(tasks),
                    task,
                    "failed",
                    error="fork grasp candidates failed safety preflight",
                )
                recover_to_reset_after_failure(robot, task)
                return object_id, False
            preflighted_right_forks.add(object_id)

        if step_by_step and not confirm_step_execution(i, task):
            print("操作员取消执行，未发送当前或后续机器人动作。")
            _log_rerun_action_transition(
                rerun_logger,
                i,
                len(tasks),
                task,
                "cancelled",
                error="operator cancelled before execution",
            )
            return object_id, False

        if (
            object_id not in preopened_objects
            and object_type in {"cup", "bread"}
            and action == "move_above_grasp"
        ):
            if not fully_open_gripper_before_pick(
                robot,
                task.get("arm"),
                object_id,
                object_type,
            ):
                _log_rerun_action_transition(
                    rerun_logger,
                    i,
                    len(tasks),
                    task,
                    "failed",
                    error="pre-grasp gripper opening failed",
                )
                return object_id, False
            preopened_objects.add(object_id)

        _log_rerun_action_transition(
            rerun_logger,
            i,
            len(tasks),
            task,
            "started",
        )
        try:
            object_id, result = execute_single_task(
                robot,
                task,
                i,
                object_id_to_type,
                preserve_current_orientation_for_approach=(
                    preserve_current_orientation_for_approach
                ),
            )
        except Exception as error:
            _log_rerun_action_transition(
                rerun_logger,
                i,
                len(tasks),
                task,
                "failed",
                error=str(error),
            )
            raise
        _log_rerun_action_transition(
            rerun_logger,
            i,
            len(tasks),
            task,
            "completed" if result else "failed",
            result=bool(result),
            error=None if result else "robot did not confirm action completion",
        )
        if not result:
            print(
                f"任务 {i + 1} 未确认完成，安全中止后续任务："
                f"action={task.get('action')}, arm={task.get('arm')}。"
            )
            recover_to_reset_after_failure(robot, task)
            return object_id, False

        if pause_between_tasks and i < len(tasks) - 1:
            if operator_requests_reset_after_step(i):
                recover_to_reset_after_failure(
                    robot,
                    task,
                    operator_requested=True,
                )
                return OPERATOR_RESET_REQUESTED, False

    return -1, True
