import os
import cv2
import sys
import numpy as np
import csv
import ast
import time

repo_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.join(repo_path, "modules/cuarm_configuration/share/python/scripts"))
sys.path.insert(0, os.path.join(repo_path, "modules/curi_udp/python"))
sys.path.insert(0, os.path.join(repo_path, "../cuarm_panel_control/robot_platform_utils/python/scripts"))
from scripts.utils import load_config, show_image
from scripts.camera_manager import CameraManager
from scripts.chessboard import detect_chessboard
from cuarm_udp import CuarmUdp, CuarmUdpThread

from cuarm_state import ControlType, RemotePanelCommand, PanelRemoteState, PanelTargetMode
from cuarm_state_initializers import get_initialized_remote_panel_command
from cuarm_utils import same_array

ZERO_VEL_THRESHOLD = 0.005   # rad/s
POS_THRESHOLD = 0.00174533   # rad (~0.1 deg)
MOTION_TIMEOUT_SEC = 60.0
CMD_INTERVAL_SEC = 0.02    # ~50 Hz command rate

def wait_for_panel_state(panel_udp, timeout_sec=30.0):
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        panel_state = panel_udp.receive()
        if panel_state:
            return panel_state
        time.sleep(0.05)
    return None

def build_arm_joint_commands(target, panel_state, arm_size):
    commands = []
    for arm_idx in range(arm_size):
        if arm_idx < len(target):
            commands.append(np.radians(list(target[arm_idx])).tolist())
        elif panel_state is not None:
            commands.append(list(panel_state.arm_joint_position[arm_idx]))
        else:
            commands.append([])
    return commands

def targets_reached(panel_state, remote_panel_cmd, arm_indices):
    for arm_idx in arm_indices:
        joint_size = panel_state.arm_joint_size[arm_idx]
        zero_vel = same_array(
            panel_state.arm_joint_velocity[arm_idx],
            np.zeros(joint_size),
            atol=ZERO_VEL_THRESHOLD,
        )
        position_reached = same_array(
            panel_state.arm_interpolated_target[arm_idx],
            remote_panel_cmd.arm_joint_command[arm_idx],
            atol=POS_THRESHOLD,
        )
        if not (zero_vel and position_reached):
            return False
    return True

def sync_gripper_command(remote_panel_cmd, panel_state):
    for grip_idx in range(panel_state.gripper_size):
        remote_panel_cmd.gripper_joint_command[grip_idx] = list(
            panel_state.gripper_joint_position[grip_idx]
        )

def format_joint_error_deg(panel_state, target_cmd_rad, arm_idx):
    err = np.degrees(
        np.array(panel_state.arm_joint_position[arm_idx], dtype=float)
        - np.array(target_cmd_rad, dtype=float)
    )
    return np.round(err, 2).tolist()

def main():
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config.yaml')
    config = load_config(config_path)
    save_dir = os.path.join(config.get("calibration_path", "."), "results")
    if os.path.exists(save_dir):
        for root, dirs, files in os.walk(save_dir):
            for f in files:
                try:
                    os.unlink(os.path.join(root, f))
                except Exception as e:
                    print(f"Warning: failed to remove file {os.path.join(root,f)}: {e}")
            for d in dirs:
                try:
                    import shutil
                    shutil.rmtree(os.path.join(root, d))
                except Exception as e:
                    print(f"Warning: failed to remove dir {os.path.join(root,d)}: {e}")
    else:
        os.makedirs(save_dir, exist_ok=True)
    
    camera_manager = CameraManager(config["camera_type"], camera_id=config.get("camera_id"))
    cb_dimension = tuple(config.get("chessboard_dimension", [8, 6]))
    arm_index = 0 
    
    panel_udp = CuarmUdpThread(config["receive_robot_state"]["local_ip"], config["receive_robot_state"]["local_port"], config["receive_robot_state"]["remote_ip"], config["receive_robot_state"]["remote_port"], 
                               CuarmUdp.unpack_panel_state, CuarmUdp.pack_remote_panel_command, recv_delay_ms=10)
    
    joint_csv = os.path.join(save_dir, "joint_position.csv")
    ee_csv = os.path.join(save_dir, "ee_pose.csv")
    records_csv = os.path.join(save_dir, "records.csv")
    if not os.path.exists(joint_csv):
        with open(joint_csv, 'w') as f: f.write("j0,j1,j2,j3,j4,j5,j6\n")
        with open(ee_csv, 'w') as f: f.write("x,y,z,qw,qx,qy,qz\n")
        with open(records_csv, 'w') as f: f.write("sample_id,quality_flag,image_path,joint_csv_path,ee_csv_path\n")

    counter = 1


    calibrate_file = os.path.join(os.path.dirname(__file__), '0804-calib.txt')
    targets = []
    if os.path.exists(calibrate_file):
        with open(calibrate_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                try:
                  
                    parsed = ast.literal_eval(f"({line})")
                    targets.append(parsed)
                except Exception as e:
                    print(f"[WARN] Failed to parse line in calibrate.txt: {line}, Error: {e}")
    else:
        print(f"[ERROR] calibrate.txt not found at {calibrate_file}")
        camera_manager.release()
        panel_udp.terminate()
        return

    print(f"[INFO] Loaded {len(targets)} targets from calibrate.txt.")

   
    if targets:
        arm_joint_size = [len(targets[0][0]), len(targets[0][1])]
        if len(targets[0]) >= 4:
            gripper_joint_size = [len(targets[0][2]), len(targets[0][3])]
        else:
            gripper_joint_size = [1, 1]
    else:
        arm_joint_size = [7, 7]
        gripper_joint_size = [1, 1]

  
    remote_panel_cmd = get_initialized_remote_panel_command(
        target_type = ControlType.POSITION,
        actuator_mode = ControlType.POSITION,
        arm_target_mode = PanelTargetMode.SINGLE_POINT,
        arm_size = len(arm_joint_size),
        arm_joint_size = arm_joint_size,
        gripper_size = len(gripper_joint_size),
        gripper_joint_size = gripper_joint_size
    )
    remote_panel_cmd.interpolation_acceleration_time = 5.0
    

    print("[INFO] Waiting for panel UDP state...")
    print("[INFO] On panel Remote page: Connect -> Start (panel local=30140, remote=30141)")
    panel_state = wait_for_panel_state(panel_udp, timeout_sec=30.0)
    if not panel_state:
        print("[ERROR] No panel state received within 30s.")
        print("[ERROR] Check config.yaml ports and panel Remote page Connect/Start.")
        camera_manager.release()
        panel_udp.terminate()
        return
    remote_panel_cmd.gripper_size = panel_state.gripper_size
    remote_panel_cmd.gripper_joint_size = list(panel_state.gripper_joint_size)
    remote_panel_cmd.gripper_joint_command = [
        list(panel_state.gripper_joint_position[grip_idx])
        for grip_idx in range(panel_state.gripper_size)
    ]
    if panel_state.state != PanelRemoteState.START:
        print("[WARN] Panel remote is STOP. Click Start on panel Remote page before motion.")

    print("[INFO] Starting automatic calibration procedure. Press 'q' to quit.")

    try:
        if not config.get("receive_robot_state", {}).get("enabled", False):
            print("[WARN] receive_robot_state is not enabled in config. Exiting...")
            return

        remote_panel_running = panel_state.state == PanelRemoteState.START
        for i, target in enumerate(targets):
            arm_commands = build_arm_joint_commands(
                target, panel_state, remote_panel_cmd.arm_size
            )
            remote_panel_cmd.arm_joint_command = arm_commands
            target_joints_deg = target[arm_index] if arm_index < len(target) else target[0]

            print(f"[INFO] Moving to target {i+1}/{len(targets)} (arm {arm_index}): {target_joints_deg}")

            start_time = time.time()
            last_debug_time = start_time
            quit_requested = False
            arrived = False
            warned_no_motion = False

            while time.time() - start_time < MOTION_TIMEOUT_SEC:
                panel_state = panel_udp.receive()
                if panel_state:
                    if panel_state.state == PanelRemoteState.START:
                        remote_panel_running = True
                        sync_gripper_command(remote_panel_cmd, panel_state)
                        panel_udp.send(remote_panel_cmd)
                        if targets_reached(
                            panel_state,
                            remote_panel_cmd,
                            [arm_index],
                        ):
                            arrived = True
                            break
                    elif remote_panel_running and not warned_no_motion:
                        print("[WARN] Panel remote switched to STOP - click Start to resume motion.")
                        warned_no_motion = True
                elif remote_panel_running and not warned_no_motion:
                    print("[WARN] No panel state - commands not sent. Check UDP ports.")
                    warned_no_motion = True

                now = time.time()
                if panel_state and now - last_debug_time >= 2.0:
                    err = format_joint_error_deg(
                        panel_state, arm_commands[arm_index], arm_index
                    )
                    vel = np.round(
                        np.degrees(panel_state.arm_joint_velocity[arm_index]), 2
                    ).tolist()
                    print(
                        f"[DEBUG] target {i+1}: joint err(deg)={err}, vel(deg/s)={vel}"
                    )
                    last_debug_time = now

                camera_manager.update()
                rgb_image = camera_manager.get_color_image()
                if rgb_image is not None:
                    processed_image, _, _ = detect_chessboard(rgb_image, cb_dimension)
                    key = show_image(processed_image, wait_time_ms=1)
                    if key == 'q':
                        quit_requested = True
                        break
                time.sleep(CMD_INTERVAL_SEC)

            if quit_requested:
                print("[INFO] Quit requested by user.")
                break

            if not arrived:
                print(
                    f"[WARN] Target {i+1} not reached within {MOTION_TIMEOUT_SEC:.0f}s. "
                    "Capturing anyway..."
                )
            else:
                print(f"[INFO] Arrived at target {i+1}. Capturing data...")
            panel_state = panel_udp.receive()
            camera_manager.update()
            rgb_image = camera_manager.get_color_image()
            
            if rgb_image is None:
                print("[WARN] No RGB image captured at target. Skipping...")
                continue
                
            processed_image, success, _ = detect_chessboard(rgb_image, cb_dimension)
            show_image(processed_image, wait_time_ms=1)
            
            if panel_state:
                joints = np.array(panel_state.arm_joint_position)[arm_index].flatten()
                ee = np.array(panel_state.ee_pose)[arm_index].flatten()
                
                filename = f"{counter:02d}.png"
                image_full_path = os.path.join(save_dir, filename)
                cv2.imwrite(image_full_path, rgb_image)
                
                with open(joint_csv, 'a') as f: f.write(",".join(map(str, joints)) + "\n")
                with open(ee_csv, 'a') as f: f.write(",".join(map(str, ee)) + "\n")
                with open(records_csv, 'a') as f: f.write(f"{counter},{int(success)},{image_full_path},{os.path.abspath(joint_csv)},{os.path.abspath(ee_csv)}\n")
                print(f"[SAVED] {filename} and corresponding joint/ee data for target {i+1}.")
                counter += 1
            else:
                print(f"[WARN] No panel state received at target {i+1}. Cannot save.")
                
        print("[INFO] Finished processing all targets.")
        
    finally:
        camera_manager.release()
        panel_udp.terminate()

if __name__ == "__main__":
    main()