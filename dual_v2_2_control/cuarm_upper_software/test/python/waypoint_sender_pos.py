import sys
import os 
import time
import numpy as np

#Add Other Repository to path
repo_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
parent_path = os.path.dirname(repo_path)
sys.path.extend([repo_path, parent_path])

#Import from cuarm_configuration repository
udp_repo_path = parent_path+"/curi_udp/python"
config_repo_share_path = parent_path+"/cuarm_configuration/share/python/scripts"
sys.path.extend([udp_repo_path, config_repo_share_path])
from cuarm_state import RemotePanelCommand, PanelState, ControlType, \
PanelRemoteState, PanelTargetMode
from cuarm_udp import CuarmUdp, CuarmUdpThread
from cuarm_state_initializers import get_initialized_remote_panel_command

udp = CuarmUdpThread("127.0.0.1", 30031, "127.0.0.1", 30030, CuarmUdp.unpack_panel_state, CuarmUdp.pack_remote_panel_command, recv_delay_ms=10)

arm_waypoints1 = [
    [[5, 5, 5, 5, 5, 5, 5], [5, 5, 5, 5, 5, 5, 5]],
    [[10, 10, 10, 10, 10, 10, 10], [10, 10, 10, 10, 10, 10, 10]],
    [[15, 15, 15, 15, 15, 15, 15], [15, 15, 15, 15, 15, 15, 15]],
]

arm_waypoints2 = [
    [[10, 10, 10, 10, 10, 10, 10], [10, 10, 10, 10, 10, 10, 10]],
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]],
    [[-10, -10, -10, -10, -10, -10, -10], [-10, -10, -10, -10, -10, -10, -10]],
]

arm_waypoints3 = [
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]
]

all_waypoints = [arm_waypoints1, arm_waypoints2, arm_waypoints3]

target_gripper_cmd = [
    [[0, 0, 0, 0, 0, 0], [0]],
    [[0.5, 0.5, 0.5, 0.5, 0.5, 0.5], [1]],
    [[0, 0, 0, 0, 0, 0], [0]],
]

cnt = 0
delay_ms = 50
target_i = -1
remote_panel_cmd: RemotePanelCommand = get_initialized_remote_panel_command(
    target_type = ControlType.POSITION, # joint_cmd type
    actuator_mode = ControlType.VELOCITY, # Underlying motors' running mode
    arm_target_mode = PanelTargetMode.WAYPOINT,
    arm_size = 2,
    arm_joint_size = [7, 7],
    gripper_size = 2,
    gripper_joint_size = [6, 1]
)

while target_i < len(all_waypoints)-1:
    try:
        panel_state: PanelState = udp.receive()
        if panel_state and panel_state.state == PanelRemoteState.START:
            # Check if last waypoint is finished
            update_target= True
            for arm_idx in range(remote_panel_cmd.arm_size):
                zero_vel = all(np.isclose(panel_state.joint_vel[arm_idx], 0.0, atol=1e-1))
                interpolation_finished = target_i < 0 or all(np.isclose(panel_state.interpolated_target[arm_idx],
                    remote_panel_cmd.arm_waypoint_cmd[-1][arm_idx], atol=1e-3))
                if not interpolation_finished or not zero_vel:
                    update_target = False
                    break
            
            # If last waypoint is finished, send next waypoint
            if update_target:
                target_i += 1
                remote_panel_cmd.arm_waypoint_size = len(all_waypoints[target_i])
                remote_panel_cmd.arm_waypoint_dt = 2.0
                remote_panel_cmd.arm_waypoint_cmd = all_waypoints[target_i]
                remote_panel_cmd.gripper_joint_cmd = target_gripper_cmd[target_i]
                print(f"###################### Send waypoints {target_i} ########################")
                print("Last point: ", remote_panel_cmd.arm_waypoint_cmd[-1])
                print("Current Joint Pos: ")
                print("    Left: ", panel_state.joint_pos[0])
                print("    Right: ", panel_state.joint_pos[1])
                print("Current Joint Vel: ")
                print("    Left: ", panel_state.joint_vel[0])
                print("    Right: ", panel_state.joint_vel[1])

            #Send Command
            udp.send(remote_panel_cmd)
        time.sleep(delay_ms/1000)
    except KeyboardInterrupt:
        print("Ctrl+C detected. Exiting...")
        break
udp.terminate()
    