import sys
import os 
import time

#Add Other Repository to path
repo_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
parent_path = os.path.dirname(repo_path)
sys.path.extend([repo_path, parent_path])

#Import from cuarm_configuration repository
udp_repo_path = parent_path+"/curi_udp/python"
config_repo_share_path = parent_path+"/cuarm_configuration/share/python/scripts"
sys.path.extend([udp_repo_path, config_repo_share_path])
from cuarm_state import RemotePanelCommand, PanelState, ControlType, PanelRemoteState, PanelTargetMode
from cuarm_udp import CuarmUdp, CuarmUdpThread
from cuarm_state_initializers import get_initialized_remote_panel_command


udp = CuarmUdpThread("127.0.0.1", 30031, "127.0.0.1", 30030, CuarmUdp.unpack_panel_state, CuarmUdp.pack_remote_panel_command, recv_delay_ms=10)

target_joint_cmd = [
    [[5, 5, 5, 5, 5, 5, 5], [5, 5, 5, 5, 5, 5, 5]],
    [[15, 15, 15, 15, 15, 15, 15], [15, 15, 15, 15, 15, 15, 15]],
    [[7, 7, 7, 7, 7, 7, 7], [7, 7, 7, 7, 7, 7, 7]],
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]],
]

target_gripper_cmd = [
    [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]],
    [[0.5, 0.5, 0.5, 0.5, 0.5, 0.5], [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]],
    [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]],
    [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]],
]

cnt = 0
delay_ms = 50
target_i = -1
remote_panel_cmd: RemotePanelCommand = get_initialized_remote_panel_command(
    target_type = ControlType.POSITION, # joint_cmd type
    actuator_mode = ControlType.VELOCITY, # Underlying motors' running mode
    arm_target_mode = PanelTargetMode.RAW_SINGLE_POINT,
    arm_size = 2,
    arm_joint_size = [7, 7],
    gripper_size = 2,
    gripper_joint_size = [6, 6]
)

while target_i < len(target_joint_cmd)-1:
    panel_state: PanelState = udp.receive()
    if panel_state and panel_state.state == PanelRemoteState.START:
        if cnt % 20 == 0:
            target_i += 1
            remote_panel_cmd.arm_joint_cmd = target_joint_cmd[target_i]
            remote_panel_cmd.gripper_joint_cmd = target_gripper_cmd[target_i]
            print("Current Joint Pos: ")
            print("    Left: ", panel_state.joint_pos[0])
            print("    Right: ", panel_state.joint_pos[1])
            print(f"######################Send Cmd {target_i}: ########################")
            print("Joint Cmd: ")
            print("    Left: ", remote_panel_cmd.arm_joint_cmd[0])
            print("    Right: ", remote_panel_cmd.arm_joint_cmd[1])
            print("Gripper Cmd: ")
            print("    Left: ", remote_panel_cmd.gripper_joint_cmd[0])
            print("    Right: ", remote_panel_cmd.gripper_joint_cmd[1])
        
        udp.send(remote_panel_cmd)
        cnt += 1
    time.sleep(delay_ms/1000)
udp.terminate()
    