import sys
import os 
import time

#Add Other Repository to path
repo_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
parent_path = os.path.dirname(repo_path)
sys.path.extend([repo_path, parent_path])
print("Parent: ", parent_path)

#Import from cuarm_configuration repository
udp_repo_path = parent_path+"/curi_udp/python"
config_repo_share_path = parent_path+"/cuarm_configuration/share/python/scripts"
sys.path.extend([udp_repo_path, config_repo_share_path])
from cuarm_state import RemotePanelCommand, PanelState, ControlType, PanelRemoteState, PanelTargetMode
from cuarm_udp import CuarmUdp, CuarmUdpThread
from cuarm_state_initializers import get_initialized_remote_panel_command

udp = CuarmUdpThread("127.0.0.1", 30031, "127.0.0.1", 30030, CuarmUdp.unpack_panel_state, CuarmUdp.pack_remote_panel_command, recv_delay_ms=10)

target_joint_cmd = [
    [[2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0], [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]],
    [[-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0], [-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0]],
]

target_gripper_cmd = [
    [[0], [0]],
    [[0.5], [1]],
]

cnt = 0
delay_ms = 50
target_i = -1
remote_panel_cmd: RemotePanelCommand = get_initialized_remote_panel_command(
    target_type = ControlType.VELOCITY, # joint_cmd type
    actuator_mode = ControlType.VELOCITY, # Underlying motors' running mode
    arm_target_mode = PanelTargetMode.SINGLE_POINT,
    arm_size = 2,
    arm_joint_size = [7, 7],
    gripper_size = 2,
    gripper_joint_size = [1, 1]
)

while True:
    try:
        state: PanelState = udp.receive()
        if state and state.state == PanelRemoteState.START:
            if cnt % 100 == 0: #Update Target_i
                target_i = (target_i + 1)%len(target_gripper_cmd)
                remote_panel_cmd.arm_joint_cmd = target_joint_cmd[target_i]
                remote_panel_cmd.gripper_joint_cmd = target_gripper_cmd[target_i]
                print(f"######################Send Cmd {target_i}: ########################")
                print("Joint Cmd: ")
                print("    Left: ", remote_panel_cmd.arm_joint_cmd[0])
                print("    Right: ", remote_panel_cmd.arm_joint_cmd[1])
                print("Gripper Cmd: ")
                print("    Left: ", remote_panel_cmd.gripper_joint_cmd[0])
                print("    Right: ", remote_panel_cmd.gripper_joint_cmd[1])
            
            if cnt%20 == 0:
                udp.send(remote_panel_cmd)
                print("Current Joint Vel: ")
                print("    Left: ", state.joint_vel[0])
                print("    Right: ", state.joint_vel[1])
            
            udp.send(remote_panel_cmd)
            cnt += 1
        time.sleep(delay_ms/1000)
    except KeyboardInterrupt:
        print("Ctrl+C detected. Exiting...")
        break
udp.terminate()
    