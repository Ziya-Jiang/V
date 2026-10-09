"""Validate an eye-to-hand calibration using unseen armR chessboard poses.

The script replays only the right-arm group (index 1) from a user-provided
target file, holds armL at its measured position, and captures a new image
plus armR end-effector pose at every reached target.

For every validation sample it solves the chessboard pose in the camera frame.
It then checks whether the implied chessboard mounting transform relative to
armR is consistent with the calibration dataset.  This verifies the saved
external transform without requiring an independently measured board mount.

Target files use the same one-line format as record_data_on_right_hand.py:
    [left_arm_7_values], [right_arm_7_values]
Only the second group is used.  By default it is interpreted in degrees.
"""

import argparse
import ast
import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

repo_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.insert(0, os.path.join(repo_path, "modules/cuarm_configuration/share/python/scripts"))
sys.path.insert(0, os.path.join(repo_path, "modules/curi_udp/python"))
sys.path.insert(0, os.path.join(repo_path, "../cuarm_panel_control/robot_platform_utils/python/scripts"))

from scripts.camera_manager import CameraManager
from scripts.chessboard import detect_chessboard
from scripts.utils import load_config, show_image
from cuarm_state import ControlType, PanelRemoteState, PanelTargetMode
from cuarm_state_initializers import get_initialized_remote_panel_command
from cuarm_udp import CuarmUdp, CuarmUdpThread
from cuarm_utils import same_array


RIGHT_ARM_INDEX = 1
ZERO_VEL_THRESHOLD = 0.005  # rad/s
POS_THRESHOLD = 0.00174533  # rad (~0.1 deg)
MOTION_TIMEOUT_SEC = 60.0
CMD_INTERVAL_SEC = 0.02  # ~50 Hz command rate
INTERPOLATION_TIME_SEC = 5.0


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Validate a right-hand eye-to-hand calibration."
    )
    parser.add_argument(
        "--targets",
        required=True,
        help=(
            "Path to unseen validation poses. Each line must contain left and "
            "right joint groups; only the right group is commanded."
        ),
    )
    parser.add_argument(
        "--right-target-unit",
        choices=("deg", "rad"),
        default="deg",
        help="Unit of the right-arm target group (default: deg).",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually command the right arm. Omit this flag for a safe dry run.",
    )
    parser.add_argument(
        "--max-targets",
        type=int,
        default=None,
        help="Replay only the first N validation targets.",
    )
    parser.add_argument(
        "--max-translation-error-mm",
        type=float,
        default=20.0,
        help="Pass threshold for median board-mount translation error (default: 20 mm).",
    )
    parser.add_argument(
        "--max-rotation-error-deg",
        type=float,
        default=5.0,
        help="Pass threshold for median board-mount rotation error (default: 5 deg).",
    )
    return parser.parse_args()


def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= np.finfo(float).eps:
        raise ValueError("End-effector pose contains a zero quaternion.")
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=float,
    )


def rotation_matrix_to_quaternion(rotation):
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ]
        )
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
            ]
        )
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        quaternion = np.array(
            [
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
            ]
        )
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        quaternion = np.array(
            [
                (rotation[1, 0] - rotation[0, 1]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
            ]
        )
    return quaternion / np.linalg.norm(quaternion)


def average_rotation(rotations):
    if not rotations:
        raise ValueError("Cannot average an empty rotation list.")

    reference_quaternion = rotation_matrix_to_quaternion(rotations[0])
    accumulator = np.zeros((4, 4), dtype=float)
    for rotation in rotations:
        quaternion = rotation_matrix_to_quaternion(rotation)
        if np.dot(reference_quaternion, quaternion) < 0:
            quaternion = -quaternion
        accumulator += np.outer(quaternion, quaternion)

    _, eigenvectors = np.linalg.eigh(accumulator)
    quaternion = eigenvectors[:, -1]
    if quaternion[0] < 0:
        quaternion = -quaternion
    qw, qx, qy, qz = quaternion
    return quaternion_to_rotation_matrix(qw, qx, qy, qz)


def pose_to_transform(pose):
    if len(pose) != 7:
        raise ValueError(f"Expected a 7-value pose, received {len(pose)} values.")
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = quaternion_to_rotation_matrix(*pose[3:])
    transform[:3, 3] = np.array(pose[:3], dtype=float)
    return transform


def transform_to_dict(transform):
    return {
        "rotation": transform[:3, :3].tolist(),
        "translation_m": transform[:3, 3].tolist(),
    }


def rotation_angle_deg(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(float(cosine)))


def transform_error(reference, measured):
    delta = np.linalg.inv(reference) @ measured
    translation_error_m = float(np.linalg.norm(delta[:3, 3]))
    rotation_error_deg = rotation_angle_deg(delta[:3, :3])
    return translation_error_m, rotation_error_deg


def average_transform(transforms):
    if not transforms:
        raise ValueError("Cannot average an empty transform list.")
    average = np.eye(4, dtype=float)
    average[:3, 3] = np.median(
        np.array([transform[:3, 3] for transform in transforms]), axis=0
    )
    average[:3, :3] = average_rotation(
        [transform[:3, :3] for transform in transforms]
    )
    return average


def summarize_transform_errors(reference, transforms):
    errors = []
    for transform in transforms:
        translation_error_m, rotation_error_deg = transform_error(reference, transform)
        errors.append(
            {
                "translation_error_mm": translation_error_m * 1000.0,
                "rotation_error_deg": rotation_error_deg,
            }
        )
    if not errors:
        raise ValueError("No transforms are available for error analysis.")

    translation_errors = np.array(
        [error["translation_error_mm"] for error in errors], dtype=float
    )
    rotation_errors = np.array(
        [error["rotation_error_deg"] for error in errors], dtype=float
    )
    return {
        "per_sample": errors,
        "median_translation_error_mm": float(np.median(translation_errors)),
        "mean_translation_error_mm": float(np.mean(translation_errors)),
        "max_translation_error_mm": float(np.max(translation_errors)),
        "median_rotation_error_deg": float(np.median(rotation_errors)),
        "mean_rotation_error_deg": float(np.mean(rotation_errors)),
        "max_rotation_error_deg": float(np.max(rotation_errors)),
    }


def build_chessboard_object_points(chessboard_dimension, square_size_m):
    rows, columns = chessboard_dimension
    object_points = np.zeros((rows * columns, 3), np.float32)
    object_points[:, :2] = (
        np.mgrid[0:rows, 0:columns].T.reshape(-1, 2) * square_size_m
    )
    return object_points


def solve_board_pose(image, chessboard_dimension, object_points, camera_matrix, distortion):
    preview, found, corners = detect_chessboard(image, chessboard_dimension)
    if not found:
        return preview, None, None

    success, rotation_vector, translation_vector = cv2.solvePnP(
        object_points,
        corners,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return preview, None, None

    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    transform_camera_from_board = np.eye(4, dtype=float)
    transform_camera_from_board[:3, :3] = rotation_matrix
    transform_camera_from_board[:3, 3] = translation_vector.reshape(3)

    projected, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation_vector,
        camera_matrix,
        distortion,
    )
    corner_error_px = float(
        np.sqrt(np.mean(np.sum((corners.reshape(-1, 2) - projected.reshape(-1, 2)) ** 2, axis=1)))
    )
    return preview, transform_camera_from_board, corner_error_px


def parse_target_file(target_path):
    targets = []
    with open(target_path, "r") as target_stream:
        for line_number, line in enumerate(target_stream, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                target = ast.literal_eval(f"({line})")
            except (SyntaxError, ValueError) as error:
                raise ValueError(
                    f"Cannot parse target at {target_path}:{line_number}: {error}"
                ) from error
            if len(target) <= RIGHT_ARM_INDEX:
                raise ValueError(
                    f"Target at {target_path}:{line_number} has no right-arm group."
                )
            targets.append(target)
    if not targets:
        raise ValueError(f"No targets found in {target_path}.")
    return targets


def read_pose_csv(pose_path):
    poses = {}
    with open(pose_path, newline="") as pose_stream:
        for sample_id, row in enumerate(csv.DictReader(pose_stream), start=1):
            poses[sample_id] = [
                float(row[key]) for key in ("x", "y", "z", "qw", "qx", "qy", "qz")
            ]
    return poses


def sample_id_from_image_path(image_path):
    digits = "".join(character for character in image_path.stem if character.isdigit())
    return int(digits) if digits else None


def resolve_session_directory(raw_calibration_path, config_path):
    raw_path = Path(raw_calibration_path).expanduser()
    if raw_path.is_absolute():
        return raw_path

    candidates = [
        Path.cwd() / raw_path,
        config_path.parent / raw_path,
    ]
    for candidate in candidates:
        if (candidate / "base_to_camera" / "results" / "extrinsics.yaml").is_file():
            return candidate
    return candidates[0]


def load_calibration_artifacts(session_directory, calibration_type):
    result_name = "ee_to_camera" if calibration_type == "eye_in_hand" else "base_to_camera"
    results_directory = session_directory / result_name / "results"
    intrinsics_path = results_directory / "intrinsics.yaml"
    extrinsics_path = results_directory / "extrinsics.yaml"
    report_path = results_directory / "report.json"

    for required_path in (intrinsics_path, extrinsics_path, report_path):
        if not required_path.is_file():
            raise FileNotFoundError(f"Missing calibration artifact: {required_path}")

    with open(intrinsics_path, "r") as intrinsics_stream:
        intrinsics_data = yaml.safe_load(intrinsics_stream)
    with open(extrinsics_path, "r") as extrinsics_stream:
        extrinsics_data = yaml.safe_load(extrinsics_stream)
    with open(report_path, "r") as report_stream:
        report_data = json.load(report_stream)

    saved_transform = np.eye(4, dtype=float)
    saved_transform[:3, :3] = np.asarray(extrinsics_data["rotation"], dtype=float)
    saved_transform[:3, 3] = np.asarray(extrinsics_data["translation"], dtype=float).reshape(3)
    return (
        np.asarray(intrinsics_data["camera_matrix"], dtype=float),
        np.asarray(intrinsics_data["distortion"], dtype=float),
        saved_transform,
        report_data,
        results_directory,
    )


def collect_calibration_observations(
    session_directory,
    report_data,
    chessboard_dimension,
    object_points,
    camera_matrix,
    distortion,
):
    image_directory = session_directory / "results"
    pose_path = image_directory / "ee_pose.csv"
    if not pose_path.is_file():
        raise FileNotFoundError(f"Missing calibration pose file: {pose_path}")

    poses_by_id = read_pose_csv(pose_path)
    used_sample_ids = set(report_data.get("used_sample_ids", poses_by_id.keys()))
    observations = []
    for image_path in sorted(image_directory.glob("*.png")):
        sample_id = sample_id_from_image_path(image_path)
        if sample_id not in used_sample_ids or sample_id not in poses_by_id:
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        _, transform_camera_from_board, reprojection_error_px = solve_board_pose(
            image,
            chessboard_dimension,
            object_points,
            camera_matrix,
            distortion,
        )
        if transform_camera_from_board is None:
            continue
        observations.append(
            {
                "sample_id": sample_id,
                "transform_base_from_ee": pose_to_transform(poses_by_id[sample_id]),
                "transform_camera_from_board": transform_camera_from_board,
                "reprojection_error_px": reprojection_error_px,
            }
        )
    if len(observations) < 3:
        raise RuntimeError(
            "Fewer than three usable calibration observations were recovered."
        )
    return observations


def board_mount_transforms(
    observations,
    transform_base_from_camera,
    invert_ee_pose=False,
):
    transforms = []
    for observation in observations:
        transform_base_from_ee = observation["transform_base_from_ee"]
        if invert_ee_pose:
            transform_base_from_ee = np.linalg.inv(transform_base_from_ee)
        transform_ee_from_board = (
            np.linalg.inv(transform_base_from_ee)
            @ transform_base_from_camera
            @ observation["transform_camera_from_board"]
        )
        transforms.append(transform_ee_from_board)
    return transforms


def choose_transform_convention(saved_transform, observations):
    """Resolve historical transform naming ambiguity by calibration consistency."""
    candidates = []
    for invert_saved_transform in (False, True):
        transform_base_from_camera = (
            np.linalg.inv(saved_transform)
            if invert_saved_transform
            else saved_transform
        )
        for invert_ee_pose in (False, True):
            transforms = board_mount_transforms(
                observations,
                transform_base_from_camera,
                invert_ee_pose=invert_ee_pose,
            )
            reference = average_transform(transforms)
            summary = summarize_transform_errors(reference, transforms)
            score = (
                summary["median_translation_error_mm"] / 1000.0
                + 0.1 * math.radians(summary["median_rotation_error_deg"])
            )
            candidates.append(
                {
                    "transform_base_from_camera": transform_base_from_camera,
                    "invert_saved_transform": invert_saved_transform,
                    "invert_ee_pose": invert_ee_pose,
                    "board_mount_reference": reference,
                    "calibration_consistency": summary,
                    "score": score,
                }
            )

    selected = min(candidates, key=lambda candidate: candidate["score"])
    selected["candidate_scores"] = [
        {
            "invert_saved_transform": candidate["invert_saved_transform"],
            "invert_ee_pose": candidate["invert_ee_pose"],
            "score": candidate["score"],
            "median_translation_error_mm": candidate["calibration_consistency"][
                "median_translation_error_mm"
            ],
            "median_rotation_error_deg": candidate["calibration_consistency"][
                "median_rotation_error_deg"
            ],
        }
        for candidate in candidates
    ]
    return selected


def wait_for_panel_state(panel_udp, timeout_sec=30.0):
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        panel_state = panel_udp.receive()
        if panel_state:
            return panel_state
        time.sleep(0.05)
    return None


def make_right_arm_command(target, panel_state, arm_size, right_target_unit):
    if RIGHT_ARM_INDEX >= arm_size:
        raise ValueError(f"Right arm index {RIGHT_ARM_INDEX} is not available.")
    commands = [
        list(panel_state.arm_joint_position[arm_index])
        for arm_index in range(arm_size)
    ]
    right_target = np.asarray(target[RIGHT_ARM_INDEX], dtype=float)
    if right_target.size != len(commands[RIGHT_ARM_INDEX]):
        raise ValueError(
            "Right-arm target joint count does not match the current robot state."
        )
    if right_target_unit == "deg":
        right_target = np.radians(right_target)
    commands[RIGHT_ARM_INDEX] = right_target.tolist()
    return commands


def hold_non_right_arms(remote_command, panel_state):
    for arm_index in range(remote_command.arm_size):
        if arm_index != RIGHT_ARM_INDEX:
            remote_command.arm_joint_command[arm_index] = list(
                panel_state.arm_joint_position[arm_index]
            )


def sync_gripper_command(remote_command, panel_state):
    for gripper_index in range(panel_state.gripper_size):
        remote_command.gripper_joint_command[gripper_index] = list(
            panel_state.gripper_joint_position[gripper_index]
        )


def right_target_reached(panel_state, remote_command):
    joint_size = panel_state.arm_joint_size[RIGHT_ARM_INDEX]
    velocity_is_zero = same_array(
        panel_state.arm_joint_velocity[RIGHT_ARM_INDEX],
        np.zeros(joint_size),
        atol=ZERO_VEL_THRESHOLD,
    )
    target_is_reached = same_array(
        panel_state.arm_interpolated_target[RIGHT_ARM_INDEX],
        remote_command.arm_joint_command[RIGHT_ARM_INDEX],
        atol=POS_THRESHOLD,
    )
    return velocity_is_zero and target_is_reached


def save_validation_sample(
    output_directory,
    sample_id,
    image,
    right_pose,
    transform_camera_from_board,
    reprojection_error_px,
):
    image_path = output_directory / f"{sample_id:02d}.png"
    cv2.imwrite(str(image_path), image)
    return {
        "sample_id": sample_id,
        "image_path": str(image_path),
        "ee_pose": right_pose,
        "transform_camera_from_board": transform_to_dict(transform_camera_from_board),
        "pnp_reprojection_error_px": reprojection_error_px,
    }


def run_capture(
    config,
    targets,
    right_target_unit,
    output_directory,
    chessboard_dimension,
    object_points,
    camera_matrix,
    distortion,
):
    receive_config = config["receive_robot_state"]
    camera_manager = CameraManager(
        config["camera_type"], camera_id=config.get("camera_id")
    )
    panel_udp = CuarmUdpThread(
        receive_config["local_ip"],
        receive_config["local_port"],
        receive_config["remote_ip"],
        receive_config["remote_port"],
        CuarmUdp.unpack_panel_state,
        CuarmUdp.pack_remote_panel_command,
        recv_delay_ms=10,
    )

    try:
        print("[INFO] Waiting for panel UDP state...")
        panel_state = wait_for_panel_state(panel_udp)
        if not panel_state:
            raise RuntimeError("No panel state received within 30 seconds.")
        if panel_state.arm_size <= RIGHT_ARM_INDEX:
            raise RuntimeError("The panel state does not contain armR.")

        arm_joint_size = list(panel_state.arm_joint_size)
        remote_command = get_initialized_remote_panel_command(
            target_type=ControlType.POSITION,
            actuator_mode=ControlType.POSITION,
            arm_target_mode=PanelTargetMode.SINGLE_POINT,
            arm_size=panel_state.arm_size,
            arm_joint_size=arm_joint_size,
            gripper_size=panel_state.gripper_size,
            gripper_joint_size=list(panel_state.gripper_joint_size),
        )
        remote_command.interpolation_acceleration_time = INTERPOLATION_TIME_SEC
        remote_command.gripper_joint_command = [
            list(panel_state.gripper_joint_position[gripper_index])
            for gripper_index in range(panel_state.gripper_size)
        ]

        print("[INFO] armL is held at its current position; only armR will move.")
        print(
            "[INFO] On panel Remote page: Connect -> Start "
            f"(panel local={receive_config['remote_ip']}:{receive_config['remote_port']}, "
            f"remote={receive_config['local_ip']}:{receive_config['local_port']})"
        )
        if panel_state.state != PanelRemoteState.START:
            raise RuntimeError("Panel Remote is STOP. Click Start before validation.")

        captured_observations = []
        for target_index, target in enumerate(targets, start=1):
            remote_command.arm_joint_command = make_right_arm_command(
                target,
                panel_state,
                remote_command.arm_size,
                right_target_unit,
            )
            target_label = target[RIGHT_ARM_INDEX]
            print(
                f"[INFO] Moving armR to validation target "
                f"{target_index}/{len(targets)}: {target_label}"
            )

            start_time = time.time()
            last_debug_time = start_time
            arrived = False
            quit_requested = False
            while time.time() - start_time < MOTION_TIMEOUT_SEC:
                panel_state = panel_udp.receive()
                if panel_state and panel_state.state == PanelRemoteState.START:
                    hold_non_right_arms(remote_command, panel_state)
                    sync_gripper_command(remote_command, panel_state)
                    panel_udp.send(remote_command)
                    if right_target_reached(panel_state, remote_command):
                        arrived = True
                        break

                now = time.time()
                if panel_state and now - last_debug_time >= 2.0:
                    joint_error_deg = np.degrees(
                        np.asarray(panel_state.arm_joint_position[RIGHT_ARM_INDEX])
                        - np.asarray(remote_command.arm_joint_command[RIGHT_ARM_INDEX])
                    )
                    print(
                        "[DEBUG] armR joint err(deg)="
                        f"{np.round(joint_error_deg, 2).tolist()}"
                    )
                    last_debug_time = now

                camera_manager.update()
                image = camera_manager.get_color_image()
                if image is not None:
                    preview, _, _ = detect_chessboard(image, chessboard_dimension)
                    if show_image(preview, wait_time_ms=1) == "q":
                        quit_requested = True
                        break
                time.sleep(CMD_INTERVAL_SEC)

            if quit_requested:
                print("[INFO] Validation stopped by user.")
                break
            if not arrived:
                print(
                    f"[WARN] Validation target {target_index} timed out; "
                    "no sample will be saved."
                )
                continue

            panel_state = panel_udp.receive()
            camera_manager.update()
            image = camera_manager.get_color_image()
            if panel_state is None or image is None:
                print(
                    f"[WARN] Validation target {target_index} lacks a synchronized "
                    "state or image; skipping."
                )
                continue

            preview, transform_camera_from_board, reprojection_error_px = solve_board_pose(
                image,
                chessboard_dimension,
                object_points,
                camera_matrix,
                distortion,
            )
            show_image(preview, wait_time_ms=1)
            if transform_camera_from_board is None:
                print(
                    f"[WARN] Chessboard was not detected at validation target "
                    f"{target_index}; skipping."
                )
                continue

            right_pose = list(panel_state.ee_pose[RIGHT_ARM_INDEX])
            sample_id = len(captured_observations) + 1
            record = save_validation_sample(
                output_directory,
                sample_id,
                image,
                right_pose,
                transform_camera_from_board,
                reprojection_error_px,
            )
            record["target_index"] = target_index
            captured_observations.append(record)
            print(
                f"[SAVED] Validation sample {sample_id}: "
                f"PnP reprojection error={reprojection_error_px:.3f}px"
            )

        return captured_observations
    finally:
        camera_manager.release()
        panel_udp.terminate()


def validation_records_to_observations(records):
    observations = []
    for record in records:
        transform_camera_from_board = np.eye(4, dtype=float)
        transform_camera_from_board[:3, :3] = np.asarray(
            record["transform_camera_from_board"]["rotation"], dtype=float
        )
        transform_camera_from_board[:3, 3] = np.asarray(
            record["transform_camera_from_board"]["translation_m"], dtype=float
        )
        observations.append(
            {
                "sample_id": record["sample_id"],
                "transform_base_from_ee": pose_to_transform(record["ee_pose"]),
                "transform_camera_from_board": transform_camera_from_board,
                "reprojection_error_px": record["pnp_reprojection_error_px"],
            }
        )
    return observations


def main():
    args = parse_arguments()
    config_path = Path(__file__).resolve().parent.parent / "config.yaml"
    config = load_config(str(config_path))
    if config.get("calibration_type") != "eye_to_hand":
        raise ValueError(
            "This validator currently supports only eye_to_hand calibration."
        )

    target_path = Path(args.targets).expanduser().resolve()
    targets = parse_target_file(target_path)
    if args.max_targets is not None:
        targets = targets[: args.max_targets]
    if len(targets) < 3:
        raise ValueError("At least three unseen validation targets are required.")

    session_directory = resolve_session_directory(
        config["calibration_path"], config_path
    )
    camera_matrix, distortion, saved_transform, report_data, results_directory = (
        load_calibration_artifacts(session_directory, config["calibration_type"])
    )
    chessboard_dimension = tuple(config["chessboard_dimension"])
    object_points = build_chessboard_object_points(
        chessboard_dimension,
        config["chessboard_square_edge_size_m"],
    )
    calibration_observations = collect_calibration_observations(
        session_directory,
        report_data,
        chessboard_dimension,
        object_points,
        camera_matrix,
        distortion,
    )
    convention = choose_transform_convention(
        saved_transform, calibration_observations
    )

    print(f"[INFO] Calibration session: {session_directory}")
    print(f"[INFO] Calibration artifacts: {results_directory}")
    print(
        "[INFO] Selected transform convention: "
        f"invert_saved_transform={convention['invert_saved_transform']}, "
        f"invert_ee_pose={convention['invert_ee_pose']}"
    )
    print(
        "[INFO] Calibration board-mount consistency: "
        f"{convention['calibration_consistency']['median_translation_error_mm']:.2f} mm, "
        f"{convention['calibration_consistency']['median_rotation_error_deg']:.2f} deg "
        "(median)."
    )

    if not args.execute:
        print("[INFO] Dry run completed. Add --execute to command the robot.")
        return

    output_directory = (
        session_directory
        / "validation"
        / datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    output_directory.mkdir(parents=True, exist_ok=False)
    records = run_capture(
        config,
        targets,
        args.right_target_unit,
        output_directory,
        chessboard_dimension,
        object_points,
        camera_matrix,
        distortion,
    )
    validation_observations = validation_records_to_observations(records)
    if len(validation_observations) < 3:
        raise RuntimeError(
            f"Only {len(validation_observations)} validation samples were captured; "
            "at least three are required."
        )

    validation_board_mounts = board_mount_transforms(
        validation_observations,
        convention["transform_base_from_camera"],
        invert_ee_pose=convention["invert_ee_pose"],
    )
    validation_summary = summarize_transform_errors(
        convention["board_mount_reference"],
        validation_board_mounts,
    )
    passed = (
        validation_summary["median_translation_error_mm"]
        <= args.max_translation_error_mm
        and validation_summary["median_rotation_error_deg"]
        <= args.max_rotation_error_deg
    )

    report = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "status": "PASS" if passed else "CHECK_REQUIRED",
        "calibration_session": str(session_directory),
        "validation_target_file": str(target_path),
        "right_target_unit": args.right_target_unit,
        "captured_sample_count": len(records),
        "thresholds": {
            "median_translation_error_mm": args.max_translation_error_mm,
            "median_rotation_error_deg": args.max_rotation_error_deg,
        },
        "selected_transform_convention": {
            "invert_saved_transform": convention["invert_saved_transform"],
            "invert_ee_pose": convention["invert_ee_pose"],
            "candidate_scores": convention["candidate_scores"],
        },
        "saved_transform_base_from_camera": transform_to_dict(
            convention["transform_base_from_camera"]
        ),
        "estimated_board_mount_ee_from_board": transform_to_dict(
            convention["board_mount_reference"]
        ),
        "calibration_consistency": convention["calibration_consistency"],
        "validation_consistency": validation_summary,
        "validation_samples": records,
    }
    report_path = output_directory / "validation_report.json"
    with open(report_path, "w") as report_stream:
        json.dump(report, report_stream, indent=2)

    print(f"[RESULT] {report['status']}")
    print(
        "[RESULT] Validation median board-mount error: "
        f"{validation_summary['median_translation_error_mm']:.2f} mm, "
        f"{validation_summary['median_rotation_error_deg']:.2f} deg"
    )
    print(f"[RESULT] Report saved to: {report_path}")


if __name__ == "__main__":
    main()
