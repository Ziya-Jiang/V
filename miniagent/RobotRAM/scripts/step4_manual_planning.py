import numpy as np
from os.path import join as pjoin
import json
import csv
import os
from functools import lru_cache
from pathlib import Path
from step_llm_planning import (
    generate_pick_and_place_sequence,
    get_object_info_with_vlm_categories,
    refine_rough_tasks_with_precise_calculation,
)
from task_generator import generate_rough_tasks_batch
from utils.transform_utils import angle2rotation, rotation2angle
from typing import Dict, Tuple

from judge_fork_direction import get_fork_direction_vector, get_fork_direction_vector_from_image
from adjust_fork_bbox_direction import load_camera_params
from adjust_fork_bbox_direction import project_2d_direction_to_3d
from adjust_fork_bbox_direction import adjust_bbox_direction

DEFAULT_OBJECT_STAGE_OFFSETS_PATH = (
    Path(__file__).resolve().parents[2] / "object_stage_offsets.csv"
)
REQUIRED_OBJECT_STAGE_OFFSET_KEYS = {
    ("bowl", action)
    for action in (
        "move_above_grasp", "move_down_to_grasp", "close_gripper",
        "lift_object", "move_to_target", "move_down_to_place",
        "open_gripper", "move_up_after_release",
    )
} | {
    ("bread", action)
    for action in (
        "move_above_grasp", "move_down_to_grasp", "close_gripper",
        "lift_object", "move_to_target", "move_down_to_place",
        "open_gripper", "move_up_after_release",
    )
} | {
    ("cup", action)
    for action in (
        "move_above_grasp", "move_down_to_grasp",
        "close_to_object_center", "close_gripper", "lift_object",
        "move_to_target", "adjust_orientation_for_place",
        "move_down_to_place", "open_gripper", "move_up_after_release",
    )
}


@lru_cache(maxsize=4)
def _load_object_stage_offsets_cached(csv_path, mtime_ns, file_size):
    del mtime_ns, file_size
    offsets = {}
    with open(csv_path, newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required_columns = {
            "object", "stage", "reference",
            "offset_x_mm", "offset_y_mm", "target_z_base_mm",
        }
        missing_columns = required_columns.difference(reader.fieldnames or ())
        if missing_columns:
            raise ValueError(
                f"offset CSV 缺少列: {sorted(missing_columns)} ({csv_path})"
            )
        for row_number, row in enumerate(reader, start=2):
            key = (
                (row["object"] or "").strip().lower(),
                (row["stage"] or "").strip(),
            )
            if not all(key) or key in offsets:
                raise ValueError(
                    f"offset CSV 第 {row_number} 行对象/阶段为空或重复: {key}"
                )
            reference = (row["reference"] or "").strip()
            if reference not in {"object_center", "planned_place_target"}:
                raise ValueError(
                    f"offset CSV 第 {row_number} 行 reference 无效: {reference}"
                )
            values = np.asarray(
                [
                    float(row["offset_x_mm"]),
                    float(row["offset_y_mm"]),
                    float(row["target_z_base_mm"]),
                ],
                dtype=float,
            )
            if not np.isfinite(values).all():
                raise ValueError(
                    f"offset CSV 第 {row_number} 行包含非有限数值"
                )
            offsets[key] = {
                "reference": reference,
                "offset_x_mm": float(values[0]),
                "offset_y_mm": float(values[1]),
                "target_z_base_mm": float(values[2]),
            }

    missing_keys = REQUIRED_OBJECT_STAGE_OFFSET_KEYS.difference(offsets)
    if missing_keys:
        raise ValueError(
            "offset CSV 缺少必要对象阶段: "
            + ", ".join(f"{obj}/{stage}" for obj, stage in sorted(missing_keys))
        )
    return offsets


def load_object_stage_offsets(csv_path=None):
    configured_path = csv_path or os.environ.get(
        "ROBOTRAM_OBJECT_STAGE_OFFSETS_PATH",
        DEFAULT_OBJECT_STAGE_OFFSETS_PATH,
    )
    resolved_path = Path(configured_path).expanduser().resolve()
    stat = resolved_path.stat()
    return _load_object_stage_offsets_cached(
        str(resolved_path), stat.st_mtime_ns, stat.st_size
    )


def stage_target_position(stage_offsets, object_type, stage, object_center, target):
    offset = stage_offsets[(object_type, stage)]
    if offset["reference"] == "object_center":
        base_x = float(object_center["x"])
        base_y = float(object_center["y"])
    else:
        if not target or len(target) < 2:
            raise ValueError(f"{object_type}/{stage} 缺少原始放置目标")
        base_x = float(target[0])
        base_y = float(target[1])
    return [
        base_x + offset["offset_x_mm"],
        base_y + offset["offset_y_mm"],
        offset["target_z_base_mm"],
    ]


def _arm_for_target_y(y_value):
    # Verified from planner FK: left/index0 is on +Y, right/index1 on -Y.
    return 'left' if float(y_value) >= 0 else 'right'


def normalize_task_arm_by_target_y(detailed_task_list, logger=None):
    normalized_count = 0
    for task in detailed_task_list:
        target_position = task.get('target_position')
        arm = task.get('arm')
        if arm not in ('left', 'right') or not target_position or len(target_position) < 2:
            continue

        try:
            expected_arm = _arm_for_target_y(target_position[1])
        except (TypeError, ValueError):
            continue

        if arm == expected_arm:
            continue

        task['arm'] = expected_arm
        description = task.get('description')
        if isinstance(description, str):
            task['description'] = description.replace(f'使用{arm}臂', f'使用{expected_arm}臂')
        normalized_count += 1

    if normalized_count and logger:
        logger.info(f'根据目标Y坐标修正了 {normalized_count} 个子任务的左右臂分配')
    return detailed_task_list


LEFT_BOWL_ROTATION = [180, 70, 90]
# The legacy all-in-one planner targeted a left-hand mount with
# rpy=[90, 0, -90]. dual_v2_2 mounts the same physical hand at
# rpy=[0, 180, 0], so identical end-effector Euler angles point the hand in a
# different direction. Convert legacy commands so the physical hand keeps the
# same world orientation.
LEGACY_LEFT_HAND_MOUNT_ROTATION = angle2rotation(90, 0, -90)
CURRENT_LEFT_HAND_MOUNT_ROTATION = angle2rotation(0, 180, 0)
LEGACY_TO_CURRENT_LEFT_TOOL_ROTATION = (
    LEGACY_LEFT_HAND_MOUNT_ROTATION
    @ CURRENT_LEFT_HAND_MOUNT_ROTATION.T
)
# Bowl TCP height fallback in the robot base frame for left-arm planning.
BOWL_GRASP_Z_FALLBACK_MM = 80.0
BOWL_APPROACH_OFFSET_MM = 40.0
BOWL_LIFT_DELTA_MM = 100.0
# Euler XYZ angles converted from RobotArm.intermediate_reset_pos quaternions.
# These are safer approach candidates than the identity orientation [0, 0, 0].
LEFT_SAFE_APPROACH_ROTATION = [-75.66946, 23.33137, -9.89926]
RIGHT_SAFE_APPROACH_ROTATION = [155.93501, 26.74707, 8.70120]
BOWL_RIM_OFFSET_MM = np.array([-15.0, -40.0, 0.0])
BOWL_PICK_ACTIONS = {
    'move_above_grasp',
    'move_down_to_grasp',
    'close_gripper',
    'lift_object',
    'move_to_target',
    'move_down_to_place',
    'open_gripper',
    'move_up_after_release',
}
# Candidate ordering prefers a small change in the horizontal jaw direction.
BOWL_GRASP_YAW_CANDIDATES_DEG = (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180)
# arm_end_effector_r -> physical right-gripper frame in the URDF.
RIGHT_GRIPPER_FROM_TOOL_ROTATION = np.diag([1.0, -1.0, -1.0])
# The current scene calibration places tabletop fork centers near Z=0...30 mm.
# Reject a grossly inconsistent reconstruction instead of commanding its XY.
FORK_CENTER_Z_MIN_MM = -30.0
FORK_CENTER_Z_MAX_MM = 160.0


def legacy_rotation_for_arm(arm, legacy_euler):
    """Preserve legacy physical-hand orientation on the dual_v2_2 left arm."""
    legacy_euler = list(legacy_euler)
    if arm != 'left':
        return legacy_euler
    legacy_world_from_tool = angle2rotation(*legacy_euler)
    current_world_from_tool = (
        legacy_world_from_tool @ LEGACY_TO_CURRENT_LEFT_TOOL_ROTATION
    )
    return rotation2angle(current_world_from_tool).tolist()


def bowl_bbox_mean_z_mm(object_info):
    """Return the mean world Z of the bowl bounding-box height planes."""
    try:
        bounding_box = object_info["bounding_box"]
        z1 = float(bounding_box["bbox_plane_z1"]["z"])
        z2 = float(bounding_box["bbox_plane_z2"]["z"])
    except (KeyError, TypeError, ValueError):
        return BOWL_GRASP_Z_FALLBACK_MM, False

    mean_z = (z1 + z2) * 0.5
    if not np.isfinite(mean_z):
        return BOWL_GRASP_Z_FALLBACK_MM, False
    return mean_z, True


def _same_rotation(a, b, tol=1e-6):
    if not a or len(a) < 3:
        return False
    try:
        return all(abs(float(a[i]) - float(b[i])) <= tol for i in range(3))
    except (TypeError, ValueError):
        return False


def right_bowl_vertical_pick_rotation(yaw_deg):
    """Return a tool Euler pose with physical right-gripper +Z pointing down."""
    yaw_rad = np.deg2rad(float(yaw_deg))
    gripper_x = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0])
    gripper_z = np.array([0.0, 0.0, -1.0])
    gripper_y = np.cross(gripper_z, gripper_x)
    world_from_gripper = np.column_stack(
        [gripper_x, gripper_y, gripper_z]
    )
    world_from_tool = (
        world_from_gripper @ RIGHT_GRIPPER_FROM_TOOL_ROTATION.T
    )
    return rotation2angle(world_from_tool).tolist()


def apply_right_bowl_pick_yaw(
    detailed_task_list, yaw_deg, combined_objects, logger=None
):
    """Apply one vertical, rim-grasp candidate to all right-arm bowl pickup steps."""
    adjusted_count = 0
    rotation = right_bowl_vertical_pick_rotation(yaw_deg)
    for task in detailed_task_list:
        object_id = task.get('object')
        if (
            task.get('arm') != 'right'
            or task.get('action') not in BOWL_PICK_ACTIONS
            or object_id not in combined_objects
            or combined_objects[object_id].get('category') != 'bowl'
        ):
            continue
        if not task.get('target_position'):
            continue
        task['target_rotation'] = rotation.copy()
        adjusted_count += 1

    if adjusted_count and logger:
        logger.info(
            '将 %s 个 right 臂碗抓取任务设为夹爪 +Z 垂直向下，yaw=%s°，姿态=%s',
            adjusted_count,
            yaw_deg,
            np.round(rotation, 3).tolist(),
        )
    return detailed_task_list


def vlm_euler_to_robot_euler(euler):
    R0 = angle2rotation(*euler)
    R = np.array([
        [0, 0, 1], 
        [0, -1, 0], 
        [1, 0, 0]
    ])
    return rotation2angle(R0 @ R)

def calculate_gripper_rotation(bounding_box: Dict, object_size: Dict) -> Tuple[np.ndarray, np.ndarray]:
    """
    计算夹爪的旋转矩阵和欧拉角
    
    Args:
        bounding_box: 3D bounding box数据
        object_size: 物体尺寸，包含width, height, depth
        
    Returns:
        rotation_matrix: 3x3旋转矩阵
        euler_angles: (roll, pitch, yaw) 欧拉角（弧度）
    """
    
    # adjust_bbox_direction() has already canonicalized bbox X as the fork
    # long axis. Do not re-select an axis from RAM's width/height/depth labels:
    # those labels can swap between runs and rotate the gripper by 90 degrees.
    x1 = np.array([bounding_box['bbox_plane_x1']['x'], bounding_box['bbox_plane_x1']['y'], bounding_box['bbox_plane_x1']['z']])
    x2 = np.array([bounding_box['bbox_plane_x2']['x'], bounding_box['bbox_plane_x2']['y'], bounding_box['bbox_plane_x2']['z']])

    def normalized(vector, name):
        norm = float(np.linalg.norm(vector))
        if not np.isfinite(norm) or norm < 1e-6:
            raise ValueError(f"fork bounding-box {name} axis is invalid")
        normalized_vector = vector / norm
        if not np.isfinite(normalized_vector).all():
            raise ValueError(f"fork bounding-box {name} axis is non-finite")
        return normalized_vector

    long_axis_dir = normalized(x2 - x1, "X")
    
    # 夹爪坐标系定义：
    # z_axis: 垂直于桌面（世界坐标系-Z轴）
    # y_axis: 平行于叉子长轴，使叉子位于沿X轴分开的两指之间
    # x_axis: 两指开合/分离方向，由y_axis和z_axis叉积得到
    
    # 世界坐标系的Z轴（桌面法线，向下抓取）
    world_z = np.array([0, 0, -1])
    
    # 长边方向在水平面上的投影
    long_axis_horizontal = long_axis_dir.copy()
    long_axis_horizontal[2] = 0  # 置Z分量为0，投影到XY平面
    long_axis_horizontal_norm = normalized(
        long_axis_horizontal, "long-axis horizontal projection"
    )
    
    # 夹爪Y轴沿叉子长边；夹爪X轴因而横跨叉子宽度。
    gripper_y_norm = long_axis_horizontal_norm
    
    # 夹爪z轴：垂直于桌面（世界-Z）
    gripper_z = world_z
    
    # 夹爪x轴：由y轴和z轴叉积得到
    gripper_x = np.cross(gripper_y_norm, gripper_z)
    gripper_x_norm = gripper_x / np.linalg.norm(gripper_x)
    
    # 构建旋转矩阵 [x, y, z]
    rotation_matrix = np.column_stack([gripper_x_norm, gripper_y_norm, gripper_z])
    
    # 计算欧拉角（ZYX顺序，即yaw-pitch-roll）
    # 注意：这里使用谨慎的方法计算欧拉角，避免万向锁问题
    sy = np.sqrt(rotation_matrix[0,0] * rotation_matrix[0,0] + rotation_matrix[1,0] * rotation_matrix[1,0])
    
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2])
        pitch = np.arctan2(-rotation_matrix[2,0], sy)
        yaw = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])
    else:
        roll = np.arctan2(-rotation_matrix[1,2], rotation_matrix[1,1])
        pitch = np.arctan2(-rotation_matrix[2,0], sy)
        yaw = 0
    
    euler_angles = np.array([roll, pitch, yaw])
    
    return rotation_matrix, euler_angles


def legacy_fork_downward_tool_rotation(rot_mat):
    """Keep both fingers perpendicular to the fork while approaching downward."""
    # calculate_gripper_rotation() defines physical gripper X across the fork
    # and Y along its long axis. Rebuild the frame explicitly so no runtime
    # max-axis choice can rotate the two fingers by 90 degrees.
    gripper_x = np.asarray(rot_mat[:, 0], dtype=float).copy()
    gripper_x[2] = 0.0
    reference_norm = float(np.linalg.norm(gripper_x))
    if not np.isfinite(reference_norm) or reference_norm < 1e-6:
        raise ValueError("fork transverse gripper axis is invalid")
    gripper_x /= reference_norm
    gripper_z = np.array([0.0, 0.0, -1.0])
    gripper_y = np.cross(gripper_z, gripper_x)
    world_from_gripper = np.column_stack([gripper_x, gripper_y, gripper_z])
    world_from_tool = (
        world_from_gripper @ RIGHT_GRIPPER_FROM_TOOL_ROTATION.T
    )
    return rotation2angle(world_from_tool).tolist()


def fork_perception_blocker(object_info):
    """Return a reason when the reconstructed fork pose is unsafe to use."""
    try:
        center = object_info["center_position"]
        center_xyz = np.asarray(
            [center["x"], center["y"], center["z"]], dtype=float
        )
    except (KeyError, TypeError, ValueError):
        return "fork center_position is missing or invalid"
    if not np.isfinite(center_xyz).all():
        return "fork center_position contains a non-finite value"
    if not FORK_CENTER_Z_MIN_MM <= center_xyz[2] <= FORK_CENTER_Z_MAX_MM:
        return (
            f"fork center Z={center_xyz[2]:.1f}mm is outside the calibrated "
            f"range [{FORK_CENTER_Z_MIN_MM:.1f}, "
            f"{FORK_CENTER_Z_MAX_MM:.1f}]mm"
        )
    return None

def use_manual_rotation(detailed_task_list, combined_objects, stage_offsets):
    new_detailed_task_list = []
    new_detailed_task_list.append(  {
    "subtask_id": 9,
    "action": "first_reset",
    "target_position": None,
    "target_rotation": None,
    "description": "开场准备位；实际位姿由RobotArm.control_joint_position()中的intermediate_reset_pos决定",
    "object": None,
    "arm": "right"
  },)
    for task in detailed_task_list:
        # print(task)
        if task["object"] in combined_objects:
            if combined_objects[task["object"]]['category'] == "cup":
                objinfo = combined_objects[task["object"]]['object_info']
                # Restore the legacy three-stage cup pickup:
                # 1) move above a lateral approach point,
                # 2) descend at that point,
                # 3) move horizontally toward the final grasp point.
                cup_grasp_rotation = legacy_rotation_for_arm(
                    task["arm"], [-90, 0, -30]
                )
                # Keep the cup in the same hand orientation from grasp through
                # placement instead of rotating the wrist above the target.
                cup_place_rotation = cup_grasp_rotation
                if task["action"] == "move_above_grasp":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "cup", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = cup_grasp_rotation
                elif task["action"] == "move_down_to_grasp":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "cup", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = cup_grasp_rotation
                elif task["action"] == "close_gripper":

                    close_to =task.copy()
                    close_to["action"] = "close_to_object_center"
                    close_to["target_position"] = stage_target_position(
                        stage_offsets, "cup", close_to["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    close_to["target_rotation"] = cup_grasp_rotation
                    close_to["description"] = "低位水平靠近杯子后再抓取"
                    new_detailed_task_list.append(close_to)

                    task["target_position"] = stage_target_position(
                        stage_offsets, "cup", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = cup_grasp_rotation
                elif task["action"] == "lift_object":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "cup", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = cup_grasp_rotation
                elif task["action"] == "move_to_target":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "cup", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    # Keep the known-reachable grasp orientation during the
                    # long transport motion.
                    task["target_rotation"] = cup_grasp_rotation
                elif task["action"] == "move_down_to_place":
                    align_for_place = task.copy()
                    align_for_place["action"] = "adjust_orientation_for_place"
                    align_for_place["target_position"] = stage_target_position(
                        stage_offsets, "cup", align_for_place["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    align_for_place["target_rotation"] = cup_place_rotation
                    align_for_place["description"] = (
                        "到达杯子放置点上方后单独调整腕部方向"
                    )
                    new_detailed_task_list.append(align_for_place)

                    task["target_position"] = stage_target_position(
                        stage_offsets, "cup", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = cup_place_rotation
                elif task["action"] == "open_gripper":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "cup", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = cup_place_rotation
                elif task["action"] == "move_up_after_release":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "cup", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = cup_place_rotation
            elif combined_objects[task["object"]]['category'] == "bread":
                objinfo = combined_objects[task["object"]]['object_info']
                if task["action"] == "move_above_grasp":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "bread", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = legacy_rotation_for_arm(task["arm"], [0, 50, -45])
                elif task["action"] == "move_down_to_grasp":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "bread", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = legacy_rotation_for_arm(task["arm"], [0, 50, -45])
                elif task["action"] == "close_gripper":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "bread", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = legacy_rotation_for_arm(task["arm"], [0, 50, -45])
                elif task["action"] == "lift_object":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "bread", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = legacy_rotation_for_arm(task["arm"], [0, 50, -45])
                elif task["action"] == "move_to_target":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "bread", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = legacy_rotation_for_arm(task["arm"], [0, 50, -110])
                elif task["action"] == "move_down_to_place":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "bread", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = legacy_rotation_for_arm(task["arm"], [0, 50, -110])
                elif task["action"] == "open_gripper":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "bread", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = legacy_rotation_for_arm(task["arm"], [0, 50, -110])
                elif task["action"] == "move_up_after_release":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "bread", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = legacy_rotation_for_arm(task["arm"], [0, 50, -110])
            elif combined_objects[task["object"]]['category'] in ("bowl", "plate"):
                objinfo = combined_objects[task["object"]]['object_info']
                bowl_rotation = [-90, 90, 0]
                if combined_objects[task["object"]]['category'] == "bowl":
                    task["target_position"] = stage_target_position(
                        stage_offsets, "bowl", task["action"],
                        objinfo["center_position"], task["target_position"],
                    )
                    task["target_rotation"] = bowl_rotation
                elif task["action"] == "move_above_grasp":
                    task["target_position"] = [
                        objinfo['center_position']['x'] + BOWL_RIM_OFFSET_MM[0],
                        objinfo['center_position']['y'] + BOWL_RIM_OFFSET_MM[1],
                        120 + 100,
                    ]
                    task["target_rotation"] = bowl_rotation
                elif task["action"] == "move_down_to_grasp":
                    task["target_position"] = [
                        objinfo['center_position']['x'] + BOWL_RIM_OFFSET_MM[0],
                        objinfo['center_position']['y'] + BOWL_RIM_OFFSET_MM[1],
                        120 + 40,
                    ]
                    task["target_rotation"] = bowl_rotation
                elif task["action"] == "close_gripper":
                    task["target_position"] = [
                        objinfo['center_position']['x'] + BOWL_RIM_OFFSET_MM[0],
                        objinfo['center_position']['y'] + BOWL_RIM_OFFSET_MM[1],
                        120 + 40,
                    ]
                    task["target_rotation"] = bowl_rotation
                elif task["action"] == "lift_object":
                    task["target_position"] = [
                        objinfo['center_position']['x'] + BOWL_RIM_OFFSET_MM[0],
                        objinfo['center_position']['y'] + BOWL_RIM_OFFSET_MM[1],
                        120 + 200,
                    ]
                    task["target_rotation"] = bowl_rotation
                elif task["action"] == "move_to_target":
                    task["target_position"] = [
                        task['target_position'][0],
                        task['target_position'][1],
                        120 + 200,
                    ]
                    task["target_rotation"] = bowl_rotation
                elif task["action"] == "move_down_to_place":
                    task["target_position"] = [
                        task['target_position'][0],
                        task['target_position'][1],
                        120 + 80,
                    ]
                    task["target_rotation"] = bowl_rotation
                elif task["action"] == "open_gripper":
                    task["target_position"] = [
                        task['target_position'][0],
                        task['target_position'][1],
                        120 + 80,
                    ]
                    task["target_rotation"] = bowl_rotation
                elif task["action"] == "move_up_after_release":
                    task["target_position"] = [
                        task['target_position'][0],
                        task['target_position'][1],
                        120 + 200,
                    ]
                    task["target_rotation"] = bowl_rotation
            elif combined_objects[task["object"]]['category'] == "fork":   
                objinfo = combined_objects[task["object"]]['object_info']
                rot_mat, _ = calculate_gripper_rotation(
                    objinfo['bounding_box'], objinfo['object_size']
                )
                # Convert the legacy fork yaw into the current right-gripper
                # frame.  Keep this pose through pickup and placement so the
                # wrist does not flip after lift_object.
                target_rot = legacy_fork_downward_tool_rotation(rot_mat)

                fork_center = np.array([
                    objinfo['center_position']['x'],
                    objinfo['center_position']['y'],
                    objinfo['center_position']['z'],
                ])
                # Keep the fork center between the two fingers. The previous
                # local-X offset placed the target beside the fork and could
                # flip sides when the bounding-box axis changed sign.
                legacy_grasp_point = fork_center.copy()

                if task["action"] == "move_above_grasp":
                    task["target_position"] = [
                        legacy_grasp_point[0],
                        legacy_grasp_point[1],
                        200,
                    ]
                    task["target_rotation"] = target_rot
                elif task["action"] == "move_down_to_grasp":
                    task["target_position"] = [
                        legacy_grasp_point[0],
                        legacy_grasp_point[1],
                        10,
                    ]
                    task["target_rotation"] = target_rot
                elif task["action"] == "close_gripper":
                    task["target_position"] = [
                        legacy_grasp_point[0],
                        legacy_grasp_point[1],
                        10,
                    ]
                    task["target_rotation"] = target_rot
                elif task["action"] == "lift_object":
                    task["target_position"] = [
                        legacy_grasp_point[0],
                        legacy_grasp_point[1],
                        120,
                    ]
                    task["target_rotation"] = target_rot
                elif task["action"] == "move_to_target":
                    task["target_position"] = [
                        task["target_position"][0] - 20,
                        task["target_position"][1],
                        120,
                    ]
                    task["target_rotation"] = target_rot
                elif task["action"] == "move_down_to_place":
                    task["target_position"] = [
                        task["target_position"][0] - 20,
                        task["target_position"][1],
                        120 + 40,
                    ]
                    task["target_rotation"] = target_rot
                elif task["action"] == "open_gripper":
                    task["target_position"] = [
                        task["target_position"][0] - 20,
                        task["target_position"][1],
                        120 + 40,
                    ]
                    task["target_rotation"] = target_rot
                elif task["action"] == "move_up_after_release":
                    task["target_position"] = [
                        task["target_position"][0] - 20,
                        task["target_position"][1],
                        120,
                    ]
                    task["target_rotation"] = target_rot
            elif combined_objects[task["object"]]['category'] == "wineglass" and combined_objects[task["object"]]['object_info']['object_id'] == 2: #Golden
                objinfo = combined_objects[task["object"]]['object_info']
                if task["action"] == "move_above_grasp":
                    # task["target_position"] = [objinfo['center_position']['x'], objinfo['center_position']['y']-20,  320]
                    # task["target_rotation"] = [70, 0, 90]

                    close_to =task.copy()
                    close_to["action"] = "close_to_object_center"
                    close_to["target_position"] = [objinfo['center_position']['x'], objinfo['center_position']['y']-220,  420]
                    # close_to["target_rotation"] = [70, 0, 90]

                    close_to["target_rotation"] = [0, 0, 0]
                    close_to["description"] = "在xy平面靠近物体"
                    new_detailed_task_list.append(close_to)

                    task["target_position"] = [objinfo['center_position']['x'], objinfo['center_position']['y']-10,  380]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "move_down_to_grasp":
                    # task["target_position"] = [objinfo['center_position']['x'], objinfo['center_position']['y']-20,  270]
                    # task["target_rotation"] = [70, 0, 90]
                    task["target_position"] = [objinfo['center_position']['x'], objinfo['center_position']['y']-10,  330]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "close_gripper":

                    # close_to =task.copy()
                    # close_to["action"] = "close_to_object_center"
                    # close_to["target_position"] = [objinfo['center_position']['x'], objinfo['center_position']['y']+30,  270]
                    # # close_to["target_rotation"] = [70, 0, 90]
                    # close_to["target_rotation"] = [0, 45, 0]
                    # close_to["description"] = "在xy平面靠近物体"
                    # new_detailed_task_list.append(close_to)

                    task["target_position"] = [objinfo['center_position']['x'], objinfo['center_position']['y']-10,  330]
                    # task["target_rotation"] = [70, 0, 90]
                    task["target_rotation"] = [0, 45, 0]
                    
                elif task["action"] == "lift_object":
                    task["target_position"] = [objinfo['center_position']['x'], objinfo['center_position']['y']-10,  380]
                    # task["target_rotation"] = [70, 0, 90]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "move_to_target":
                    task["target_position"] = [task['target_position'][0], task['target_position'][1],  380]
                    # task["target_rotation"] = [70, 0, 90]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "move_down_to_place":
                    task["target_position"] = [task['target_position'][0], task['target_position'][1],  355]
                    # task["target_rotation"] = [70, 0, 90]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "open_gripper":
                    task["target_position"] = [task['target_position'][0], task['target_position'][1],  355]
                    # task["target_rotation"] = [70, 0, 90]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "move_up_after_release":
                    task["target_position"] = [task['target_position'][0], task['target_position'][1],  400]
                    # task["target_rotation"] = [70, 0, 90]
                    task["target_rotation"] = [0, 45, 0]
            elif combined_objects[task["object"]]['category'] == "wineglass" and combined_objects[task["object"]]['object_info']['object_id'] == 0: #Black
                objinfo = combined_objects[task["object"]]['object_info']
                if task["action"] == "move_above_grasp":
                    # task["target_position"] = [objinfo['center_position']['x']+35, objinfo['center_position']['y']+50,  320]
                    # task["target_rotation"] = [-70, 0, -90]

                    close_to =task.copy()
                    close_to["action"] = "close_to_object_center"
                    close_to["target_position"] = [objinfo['center_position']['x']-20, objinfo['center_position']['y']+220,  450]
                    # close_to["target_rotation"] = [70, 0, 90]
                    close_to["target_rotation"] = [0, 0, 0]
                    close_to["description"] = "在xy平面靠近物体"
                    new_detailed_task_list.append(close_to)

                    task["target_position"] = [objinfo['center_position']['x']-20, objinfo['center_position']['y']+20,  430]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "move_down_to_grasp":
                    # task["target_position"] = [objinfo['center_position']['x']+35, objinfo['center_position']['y']+50,  270]
                    # task["target_rotation"] = [-70, 0, -90]
                    task["target_position"] = [objinfo['center_position']['x']-20, objinfo['center_position']['y']+20,  290]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "close_gripper":

                    # close_to =task.copy()
                    # close_to["action"] = "close_to_object_center"
                    # close_to["target_position"] = [objinfo['center_position']['x']+35, objinfo['center_position']['y']+20,  270]
                    # # close_to["target_rotation"] = [-70, 0, -90]
                    # close_to["target_rotation"] = [0, 45, 0]
                    # close_to["description"] = "在xy平面靠近物体"
                    # new_detailed_task_list.append(close_to)

                    task["target_position"] = [objinfo['center_position']['x']-20, objinfo['center_position']['y']+20,  290]
                    # task["target_rotation"] = [-70, 0, -90]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "lift_object":
                    task["target_position"] = [objinfo['center_position']['x']-20, objinfo['center_position']['y']+20,  345]
                    # task["target_rotation"] = [-70, 0, -90]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "move_to_target":
                    task["target_position"] = [task['target_position'][0], task['target_position'][1],  345]
                    # task["target_rotation"] = [-70, 0, -90]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "move_down_to_place":
                    task["target_position"] = [task['target_position'][0], task['target_position'][1],  325]
                    # task["target_rotation"] = [-70, 0, -90]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "open_gripper":
                    task["target_position"] = [task['target_position'][0], task['target_position'][1],  325]
                    # task["target_rotation"] = [-70, 0, -90]
                    task["target_rotation"] = [0, 45, 0]
                elif task["action"] == "move_up_after_release":
                    task["target_position"] = [task['target_position'][0], task['target_position'][1]+100,  410]
                    # task["target_rotation"] = [-70, 0, -90]
                    task["target_rotation"] = [0, 45, 0]
            elif combined_objects[task["object"]]['category'] == "dustpan":
                objinfo = combined_objects[task["object"]]['object_info']
                if task["action"] == "move_above_grasp":
                    task["target_position"] = [objinfo['center_position']['x']-30, objinfo['center_position']['y']-50,  260]
                    task["target_rotation"] = [0, 30, 30]
                elif task["action"] == "move_down_to_grasp":
                    task["target_position"] = [objinfo['center_position']['x']-30, objinfo['center_position']['y']-50,  180]
                    task["target_rotation"] = [0, 30, 30]
                elif task["action"] == "close_gripper":
                    task["target_position"] = [objinfo['center_position']['x']-30, objinfo['center_position']['y']-50,  180]
                    task["target_rotation"] = [0, 30, 30]
                elif task["action"] == "lift_object":
                    task["target_position"] = [objinfo['center_position']['x']-30, objinfo['center_position']['y']-50,  190]
                    task["target_rotation"] = [0, 30, 30]
                elif task["action"] == "move_to_target":
                    task["target_position"] = [task['target_position'][0]-30, task['target_position'][1]-50,  190]
                    task["target_rotation"] = [0, 30, 0]
                elif task["action"] == "move_down_to_place":
                    task["target_position"] = [task['target_position'][0]-30, task['target_position'][1]-50,  190]
                    task["target_rotation"] = [0, 30, 0]
                elif task["action"] == "open_gripper":
                    task["target_position"] = [task['target_position'][0]-30, task['target_position'][1]-50,  190]
                    task["target_rotation"] = [0, 30, 0]
                elif task["action"] == "move_up_after_release":
                    task["target_position"] = [task['target_position'][0]-30, task['target_position'][1]-50,  190]
                    task["target_rotation"] = [0, 30, 0]
        new_detailed_task_list.append(task)
    
    return new_detailed_task_list

def planning_main(ext_image, output_dir, objects_info, vlm_categories_dict, masks,
                  camera_params_K, camera_params_R, camera_params_T, logger=None, predefined_tasks=None):
    stage_offsets = load_object_stage_offsets()
    if logger:
        logger.info(
            "已加载对象阶段 offset: %s",
            os.environ.get(
                "ROBOTRAM_OBJECT_STAGE_OFFSETS_PATH",
                str(DEFAULT_OBJECT_STAGE_OFFSETS_PATH),
            ),
        )
    # output_dir = os.path.join(args.output)
    # objects_info = load_objects_info(output_dir)
    # if objects_info is None:
    #     if logger: logger.error("无法加载物体信息，退出程序")
    #     exit(1)
    
    # # 2. 加载VLM类别信息
    # vlm_categories = load_vlm_categories(output_dir)
    # if vlm_categories is None:
    #     if logger: logger.error("无法加载VLM类别信息，退出程序")
    #     exit(1)
    
    # 3. 结合物体信息和VLM类别
    combined_objects = get_object_info_with_vlm_categories(objects_info, vlm_categories_dict)

    # print(objects_info)
    # print(combined_objects)
    fork_object_id = 4


    
    # 使用task_generator生成粗略任务
    # objects_info_path = os.path.join(output_dir, "objects_info_world.json")
    rough_tasks = generate_rough_tasks_batch(predefined_tasks, objects_info)
    # vlm_subtasks_json = pjoin(output_dir, 'vlm_rough_tasks.json')
    # with open(vlm_subtasks_json, 'w', encoding='utf-8') as f:
    #     json.dump(rough_tasks, f, indent=2, ensure_ascii=False)
    # if logger: logger.info(f"粗略任务列表已保存到: {vlm_subtasks_json}")


    # 使用精确计算来优化粗略任务位置
    precise_tasks = refine_rough_tasks_with_precise_calculation(rough_tasks, combined_objects)
    # precise_tasks_json = pjoin(output_dir, 'precise_pick_and_place_tasks.json')
    # with open(precise_tasks_json, 'w', encoding='utf-8') as f:
    #     json.dump(precise_tasks, f, indent=2, ensure_ascii=False)
    # if logger: logger.info(f"精确任务列表已保存到: {precise_tasks_json}")
    
    # 生成完整的8步流程任务序列
    detailed_task_list = generate_pick_and_place_sequence(precise_tasks, combined_objects)
    if logger: logger.info(f"生成了 {len(detailed_task_list)} 个详细子任务，包含完整6DOF位姿信息")
    
    detailed_subtasks_json = pjoin(output_dir, 'detailed_subtasks.json')
    # with open(detailed_subtasks_json, 'w', encoding='utf-8') as f:
    #     json.dump(detailed_task_list, f, indent=2, ensure_ascii=False)
    # if logger: logger.info(f"详细任务列表已保存到: {detailed_subtasks_json}")

    # 使用手动旋转
    # step1 翻转叉子方向使得叉子头始终为x轴正向


    # fork_object_id=4
    if hasattr(combined_objects, f'object_{fork_object_id}') and combined_objects[f'object_{fork_object_id}']['category'] == "fork":
        ext_image[~masks[fork_object_id]]=0
        direction_2d = get_fork_direction_vector_from_image(ext_image)
        fork_depth = combined_objects[f'object_{fork_object_id}']['object_info']['center_position_camera']['z']
        fork_direction_3d = project_2d_direction_to_3d(direction_2d, fork_depth, camera_params_K, camera_params_R, camera_params_T)
        updated_objects_info = adjust_bbox_direction(combined_objects, fork_object_id, fork_direction_3d)
    else:
        updated_objects_info = combined_objects

    detailed_task_list = use_manual_rotation(
        detailed_task_list, updated_objects_info, stage_offsets
    )
    detailed_task_list = apply_right_bowl_pick_yaw(
        detailed_task_list,
        BOWL_GRASP_YAW_CANDIDATES_DEG[0],
        updated_objects_info,
        logger=logger,
    )
    with open(detailed_subtasks_json.replace('.json', '_manual.json'), 'w', encoding='utf-8') as f:
        json.dump(detailed_task_list, f, indent=2, ensure_ascii=False)
    if logger: logger.info(f"重定制详细任务列表已保存到: {detailed_subtasks_json.replace('.json', '_manual.json')}")
    return detailed_task_list