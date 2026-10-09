"""Replay right-arm calibration poses and capture synchronized camera data.

The calibration board is mounted on armR (index 1).  The first target group
in 0804-calib.txt is deliberately ignored: armL is held at its measured
position throughout the replay.  The second target group is interpreted as
right-arm joint angles in degrees.
"""

import ast
import os
import shutil
import sys
import time

import cv2
import numpy as np

repo_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.insert(0, os.path.join(repo_path, "modules/cuarm_configuration/share/python/scripts"))
sys.path.insert(0, os.path.join(repo_path, "modules/curi_udp/python"))
sys.path.insert(0, os.path.join(repo_path, "../cuarm_panel_control/robot_platform_utils/python/scripts"))

from scripts.utils import load_config, show_image
from scripts.camera_manager import CameraManager
from scripts.chessboard import detect_chessboard
from cuarm_udp import CuarmUdp, CuarmUdpThread
from cuarm_state import ControlType, PanelRemoteState, PanelTargetMode
from cuarm_state_initializers import get_initialized_remote_panel_command
from cuarm_utils import same_array


RIGHT_ARM_INDEX = 1
TARGET_FILE_NAME = "calib_demo.txt"
ZERO_VEL_THRESHOLD = 0.005  # rad/s
POS_THRESHOLD = 0.00174533  # rad (~0.1 deg)
MOTION_TIMEOUT_SEC = 60.0
CMD_INTERVAL_SEC = 0.02  # ~50 Hz command rate
INTERPOLATION_TIME_SEC = 5.0


def wait_for_panel_state(panel_udp, timeout_sec=30.0):
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        panel_state = panel_udp.receive()
        if panel_state:
            return panel_state
        time.sleep(0.05)
    return None


def load_targets(target_file):
    targets = []
    with open(target_file, "r") as target_stream:
        for line_number, line in enumerate(target_stream, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                target = ast.literal_eval(f"({line})")
            except (SyntaxError, ValueError) as error:
                raise ValueError(
                    f"Cannot parse target at {target_file}:{line_number}: {error}"
                ) from error

            if len(target) <= RIGHT_ARM_INDEX:
                raise ValueError(
                    f"Target at {target_file}:{line_number} has no right-arm group."
                )
            targets.append(target)

    if not targets:
        raise ValueError(f"No targets found in {target_file}.")
    return targets


def build_right_arm_joint_commands(target, panel_state, arm_size):
    """Hold non-right arms at their measured positions and move only armR."""
    if RIGHT_ARM_INDEX >= arm_size:
        raise ValueError(
            f"Right arm index {RIGHT_ARM_INDEX} is unavailable; panel reports "
            f"{arm_size} arm(s)."
        )
    if len(panel_state.arm_joint_position) < arm_size:
        raise ValueError("Panel state does not contain positions for every arm.")

    commands = [
        list(panel_state.arm_joint_position[arm_index])
        for arm_index in range(arm_size)
    ]
    right_target_deg = list(target[RIGHT_ARM_INDEX])
    if len(right_target_deg) != len(commands[RIGHT_ARM_INDEX]):
        raise ValueError(
            "Right-arm target joint count does not match the panel state: "
            f"{len(right_target_deg)} != {len(commands[RIGHT_ARM_INDEX])}."
        )

    # The second group in 0804-calib.txt contains armR joint angles in degrees.
    commands[RIGHT_ARM_INDEX] = np.radians(right_target_deg).tolist()
    return commands


def hold_non_right_arms(remote_panel_cmd, panel_state):
    """Refresh held-arm targets so armL never follows the trajectory file."""
    for arm_index in range(remote_panel_cmd.arm_size):
        if arm_index != RIGHT_ARM_INDEX:
            remote_panel_cmd.arm_joint_command[arm_index] = list(
                panel_state.arm_joint_position[arm_index]
            )


def right_target_reached(panel_state, remote_panel_cmd):
    joint_size = panel_state.arm_joint_size[RIGHT_ARM_INDEX]
    zero_velocity = same_array(
        panel_state.arm_joint_velocity[RIGHT_ARM_INDEX],
        np.zeros(joint_size),
        atol=ZERO_VEL_THRESHOLD,
    )
    position_reached = same_array(
        panel_state.arm_interpolated_target[RIGHT_ARM_INDEX],
        remote_panel_cmd.arm_joint_command[RIGHT_ARM_INDEX],
        atol=POS_THRESHOLD,
    )
    return zero_velocity and position_reached


def sync_gripper_command(remote_panel_cmd, panel_state):
    for gripper_index in range(panel_state.gripper_size):
        remote_panel_cmd.gripper_joint_command[gripper_index] = list(
            panel_state.gripper_joint_position[gripper_index]
        )


def format_joint_error_deg(panel_state, target_cmd_rad):
    error = np.degrees(
        np.array(panel_state.arm_joint_position[RIGHT_ARM_INDEX], dtype=float)
        - np.array(target_cmd_rad, dtype=float)
    )
    return np.round(error, 2).tolist()


def clear_session_results(save_dir):
    if os.path.exists(save_dir):
        print(f"[WARN] Removing previous calibration data: {save_dir}")
        shutil.rmtree(save_dir)
    os.makedirs(save_dir, exist_ok=True)


def main():
    config_path = os.path.join(os.path.dirname(__file__), "..", "config.yaml")
    config = load_config(config_path)
    save_dir = os.path.join(config.get("calibration_path", "."), "results")
    target_file = os.path.join(os.path.dirname(__file__), TARGET_FILE_NAME)

    targets = load_targets(target_file)
    arm_joint_size = [len(targets[0][0]), len(targets[0][1])]
    if len(targets[0]) >= 4:
        gripper_joint_size = [len(targets[0][2]), len(targets[0][3])]
    else:
        gripper_joint_size = [1, 1]

    clear_session_results(save_dir)
    joint_csv = os.path.join(save_dir, "joint_position.csv")
    ee_csv = os.path.join(save_dir, "ee_pose.csv")
    records_csv = os.path.join(save_dir, "records.csv")
    with open(joint_csv, "w") as joint_stream:
        joint_stream.write("j0,j1,j2,j3,j4,j5,j6\n")
    with open(ee_csv, "w") as pose_stream:
        pose_stream.write("x,y,z,qw,qx,qy,qz\n")
    with open(records_csv, "w") as record_stream:
        record_stream.write(
            "sample_id,quality_flag,image_path,joint_csv_path,ee_csv_path\n"
        )

    camera_manager = CameraManager(
        config["camera_type"], camera_id=config.get("camera_id")
    )
    chessboard_dimension = tuple(config.get("chessboard_dimension", [8, 6]))
    receive_config = config["receive_robot_state"]
    panel_udp = CuarmUdpThread(
        receive_config["local_ip"],
        receive_config["local_port"],
        receive_config["remote_ip"],
        receive_config["remote_port"],
        CuarmUdp.unpack_panel_state,
        CuarmUdp.pack_remote_panel_command,
        recv_delay_ms=10,
    )

    remote_panel_cmd = get_initialized_remote_panel_command(
        target_type=ControlType.POSITION,
        actuator_mode=ControlType.POSITION,
        arm_target_mode=PanelTargetMode.SINGLE_POINT,
        arm_size=len(arm_joint_size),
        arm_joint_size=arm_joint_size,
        gripper_size=len(gripper_joint_size),
        gripper_joint_size=gripper_joint_size,
    )
    remote_panel_cmd.interpolation_acceleration_time = INTERPOLATION_TIME_SEC

    print(f"[INFO] Loaded {len(targets)} right-arm targets from {target_file}.")
    print("[INFO] armL will be held at its current position; only armR will move.")
    print("[INFO] Waiting for panel UDP state...")
    print(
        "[INFO] On panel Remote page: Connect -> Start "
        f"(panel local={receive_config['remote_ip']}:{receive_config['remote_port']}, "
        f"remote={receive_config['local_ip']}:{receive_config['local_port']})"
    )

    try:
        panel_state = wait_for_panel_state(panel_udp, timeout_sec=30.0)
        if not panel_state:
            print("[ERROR] No panel state received within 30s.")
            print("[ERROR] Check config.yaml ports and panel Remote page Connect/Start.")
            return
        if panel_state.arm_size <= RIGHT_ARM_INDEX:
            print(
                f"[ERROR] Panel reports {panel_state.arm_size} arm(s); "
                "cannot collect from armR."
            )
            return
        if not config.get("receive_robot_state", {}).get("enabled", False):
            print("[ERROR] receive_robot_state.enabled must be true.")
            return

        remote_panel_cmd.gripper_size = panel_state.gripper_size
        remote_panel_cmd.gripper_joint_size = list(panel_state.gripper_joint_size)
        remote_panel_cmd.gripper_joint_command = [
            list(panel_state.gripper_joint_position[gripper_index])
            for gripper_index in range(panel_state.gripper_size)
        ]
        if panel_state.state != PanelRemoteState.START:
            print("[WARN] Panel remote is STOP. Click Start before motion.")

        print("[INFO] Starting right-arm calibration replay. Press 'q' to quit.")
        counter = 1
        remote_panel_running = panel_state.state == PanelRemoteState.START

        for target_index, target in enumerate(targets, start=1):
            remote_panel_cmd.arm_joint_command = build_right_arm_joint_commands(
                target, panel_state, remote_panel_cmd.arm_size
            )
            right_target_deg = target[RIGHT_ARM_INDEX]
            print(
                f"[INFO] Moving armR to target {target_index}/{len(targets)}: "
                f"{right_target_deg}"
            )

            start_time = time.time()
            last_debug_time = start_time
            arrived = False
            quit_requested = False
            warned_no_motion = False

            while time.time() - start_time < MOTION_TIMEOUT_SEC:
                panel_state = panel_udp.receive()
                if panel_state:
                    if panel_state.state == PanelRemoteState.START:
                        remote_panel_running = True
                        hold_non_right_arms(remote_panel_cmd, panel_state)
                        sync_gripper_command(remote_panel_cmd, panel_state)
                        panel_udp.send(remote_panel_cmd)
                        if right_target_reached(panel_state, remote_panel_cmd):
                            arrived = True
                            break
                    elif remote_panel_running and not warned_no_motion:
                        print("[WARN] Panel remote switched to STOP - click Start to resume.")
                        warned_no_motion = True
                elif remote_panel_running and not warned_no_motion:
                    print("[WARN] No panel state - commands are not being sent.")
                    warned_no_motion = True

                now = time.time()
                if panel_state and now - last_debug_time >= 2.0:
                    error = format_joint_error_deg(
                        panel_state,
                        remote_panel_cmd.arm_joint_command[RIGHT_ARM_INDEX],
                    )
                    velocity = np.round(
                        np.degrees(panel_state.arm_joint_velocity[RIGHT_ARM_INDEX]), 2
                    ).tolist()
                    print(
                        f"[DEBUG] target {target_index}: "
                        f"right joint err(deg)={error}, vel(deg/s)={velocity}"
                    )
                    last_debug_time = now

                camera_manager.update()
                rgb_image = camera_manager.get_color_image()
                if rgb_image is not None:
                    processed_image, _, _ = detect_chessboard(
                        rgb_image, chessboard_dimension
                    )
                    if show_image(processed_image, wait_time_ms=1) == "q":
                        quit_requested = True
                        break
                time.sleep(CMD_INTERVAL_SEC)

            if quit_requested:
                print("[INFO] Quit requested by user.")
                break
            if not arrived:
                print(
                    f"[WARN] Target {target_index} was not reached within "
                    f"{MOTION_TIMEOUT_SEC:.0f}s; skipping capture."
                )
                continue

            print(f"[INFO] Arrived at target {target_index}. Capturing data...")
            panel_state = panel_udp.receive()
            camera_manager.update()
            rgb_image = camera_manager.get_color_image()
            if rgb_image is None:
                print("[WARN] No RGB image captured; skipping target.")
                continue
            if not panel_state:
                print("[WARN] No panel state available; skipping target.")
                continue

            processed_image, success, _ = detect_chessboard(
                rgb_image, chessboard_dimension
            )
            show_image(processed_image, wait_time_ms=1)

            joints = np.array(
                panel_state.arm_joint_position[RIGHT_ARM_INDEX]
            ).flatten()
            ee_pose = np.array(panel_state.ee_pose[RIGHT_ARM_INDEX]).flatten()
            filename = f"{counter:02d}.png"
            image_path = os.path.join(save_dir, filename)
            cv2.imwrite(image_path, rgb_image)

            with open(joint_csv, "a") as joint_stream:
                joint_stream.write(",".join(map(str, joints)) + "\n")
            with open(ee_csv, "a") as pose_stream:
                pose_stream.write(",".join(map(str, ee_pose)) + "\n")
            with open(records_csv, "a") as record_stream:
                record_stream.write(
                    f"{counter},{int(success)},{image_path},"
                    f"{os.path.abspath(joint_csv)},"
                    f"{os.path.abspath(ee_csv)}\n"
                )
            print(
                f"[SAVED] {filename} with synchronized armR joint and ee pose."
            )
            counter += 1

        print("[INFO] Finished processing right-arm targets.")
    finally:
        camera_manager.release()
        panel_udp.terminate()


if __name__ == "__main__":
    main()
