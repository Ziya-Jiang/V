from .base_arm import BaseArm
import time
import copy
# import Robot
import logging
import os 
import sys
import json
import yaml
import numpy as np
from .utils import *
from scipy.spatial.transform import Rotation as Rotation
# from controllor.hkclr_robot.cuarm_configuration.share.python.cuarm_state import INSPIRE_DEXTEROUS_HAND_COMMAND, INSPIRE_DEXTEROUS_ID2HANDCOMMAND, RobotStatus
# import pinocchio as pin    

# Reuse the canonical protocol/configuration shared with the planner and panel.
_CONFIG_ROOT = os.path.abspath(
    os.path.expanduser(
        os.environ.get(
            "CUARM_CONFIG_ROOT",
            "/mnt/ssd/user/V/dual_v2_2_configuration",
        )
    )
)
_UDP_REPO_PATH = os.path.join(_CONFIG_ROOT, "curi_udp", "python")
_ROBOT_UTILS_PATH = os.path.join(
    _CONFIG_ROOT,
    "robot_platform_utils",
    "python",
    "scripts",
)
sys.path[:0] = [_ROBOT_UTILS_PATH, _UDP_REPO_PATH]

from cuarm_udp import CuarmUdp, CuarmUdpThread
from cuarm_state import (
    TargetCommand,
    TargetCommandArmCommandType,
    TargetCommandGripperCommandType,
    TargetCommandPlannerMode,
    GripperMode,
    CuroboState,
    RobotStatus,
    CuroboStatus,
)
# from controllor.hkclr_robot.cuarm_state import TeleopCommand
CUARM_AVAILABLE = True
# except ImportError:
#     CUARM_AVAILABLE = False
#     print("cuarm_configuration not available, using fallback methods")

# Cartesian error-reporting thresholds are deliberately stricter near the
# object and slightly looser during collision-clear transport motions. They do
# not decide motion completion.
_GRASP_POSITION_TOLERANCE_M = {
    "fork": 0.015,
    "cup": 0.020,
    "wineglass": 0.020,
    "bowl": 0.025,
    "book": 0.030,
    "plate": 0.025,
    "bread": 0.025,
}
_STRICT_POSITION_ACTIONS = {
    "move_down_to_grasp",
    "close_to_object_center",
}
_APPROACH_POSITION_ACTIONS = {
    "move_to_cart_entry",
    "move_above_grasp",
    "move_down_to_place",
}
_TRANSPORT_POSITION_ACTIONS = {
    "lift_object",
    "move_to_target",
    "move_up_after_release",
}
_DEFAULT_POSITION_TOLERANCE_M = 0.030
_TASK_POSITION_TOLERANCE_M = {
    ("book", "move_down_to_grasp"): 0.008,
}


def _position_tolerance_for_task(object_category, action):
    """Return the Cartesian completion tolerance for one object/action."""
    category = str(object_category or "").strip().lower()
    action = str(action or "").strip().lower()
    task_tolerance = _TASK_POSITION_TOLERANCE_M.get((category, action))
    if task_tolerance is not None:
        return task_tolerance
    grasp_tolerance = _GRASP_POSITION_TOLERANCE_M.get(
        category, _DEFAULT_POSITION_TOLERANCE_M
    )

    if action in _STRICT_POSITION_ACTIONS:
        tolerance = grasp_tolerance
    elif action in _APPROACH_POSITION_ACTIONS:
        tolerance = grasp_tolerance + 0.005
    elif action in _TRANSPORT_POSITION_ACTIONS:
        tolerance = grasp_tolerance + 0.010
    else:
        tolerance = _DEFAULT_POSITION_TOLERANCE_M

    # Never restore the old, overly permissive 50 mm completion threshold.
    return min(tolerance, _DEFAULT_POSITION_TOLERANCE_M)


def _default_dual_v2_2_curobo_config_path():
    return os.path.join(_CONFIG_ROOT, "config.yaml")

def _default_dual_v2_2_config_root():
    return _CONFIG_ROOT

def _load_dual_v2_2_gripper_joint_size():
    config_root = _default_dual_v2_2_config_root()
    try:
        with open(os.path.join(config_root, "config.yaml"), "r") as file:
            full_config = yaml.safe_load(file)

        assembly_rel = full_config["robot"]["import_yaml"]
        with open(os.path.join(config_root, assembly_rel), "r") as file:
            assembly_config = yaml.safe_load(file)

        gripper_joint_size = []
        for gripper_entry in assembly_config.get("gripper", []):
            gripper_rel = gripper_entry["import_yaml"]
            with open(os.path.join(config_root, gripper_rel), "r") as file:
                gripper_config = yaml.safe_load(file)
            gripper_joint_size.append(int(gripper_config["joint_size"]))

        return gripper_joint_size
    except Exception as exc:
        raise RuntimeError(
            f"Failed to load gripper joint sizes from dual_v2_2 config: {exc}"
        ) from exc

def are_equivalent_quaternion(q1_arr, q2_arr):
    if len(q1_arr) != len(q2_arr):
        return False
    for (q1, q2) in zip(q1_arr, q2_arr):
        if not (np.allclose(q1, q2, atol=1e-3) or np.allclose(q1, -q2, atol=1e-3)):
            return False
    return True

def check_if_robot_stopped(prev_arm_cartesian_position, prev_arm_cartesian_orientation, arm_cartesian_position, arm_cartesian_orientation, cnt, max_cnt):
    if len(prev_arm_cartesian_position) != len(arm_cartesian_position) or len(prev_arm_cartesian_orientation) != len(arm_cartesian_orientation):
        return False, 0
    
    return_cnt = 0
    all_equal = True
    for arm_i in range(len(arm_cartesian_position)):
        all_equal &= np.allclose(arm_cartesian_position[arm_i], prev_arm_cartesian_position[arm_i], atol=1e-3)
        all_equal &= np.allclose(arm_cartesian_orientation[arm_i], prev_arm_cartesian_orientation[arm_i], atol=1e-3)

    if all_equal:
        return_cnt = cnt + 1

    if cnt == max_cnt:
        return True, 0
    return False, return_cnt


def matrix4x4_to_pos_quat_list(matrix):
    position = matrix[:3, 3].flatten().tolist() # [x, y, z]
    rotation_matrix = matrix[:3, :3]
    rot = Rotation.from_matrix(rotation_matrix)
    quat = rot.as_quat() # [x, y, z, w]
    quat_reordered = [quat[3], quat[0], quat[1], quat[2]] # [w, x, y, z]
    pos_quat_list = position + quat_reordered # [x, y, z, w, x, y, z]
    return pos_quat_list

class RobotArm(BaseArm):
    def __init__(self, config_file=None, target_host="127.0.0.1", target_local_port=10165, target_remote_port=10166):
        """
        Robot Arm class for controlling the robot arm

        使用特定物体类型移动:
        robot.tip_moveto(target_pos, object_type='bowl')
        
        配置参数说明:
        - tip2tool_distance: 末端执行器到工具的距离(mm)
        - rotation_y_deg: 绕Y轴旋转角度(度)
        - rotation_x_deg: 绕X轴旋转角度(度)
        - position_offset: 位置偏移 [x, y, z] (mm)
        """
        super().__init__()

        # default parameters of the robot arm
        #倒水姿势
        # self.reset_pos = [[442.651, 275.714, 355.8131, 0.715581, -0.59793, 0.336348, -0.131506], [441.108, -276.737, 364.485, 0.714965, 0.598737, 0.336355, 0.131168]]
        # self.reset_pos = [[457.70407, 589.74236, 726.4593, 0.14210181, 0.9767168, 0.13273239, 0.09062798], [416.47378, -563.9223, 679.55184, 0.14059298, -0.97703695, 0.13254714, -0.0897982]]
        self.reset_pos = [[260.90686, 847.93127, 613.90411, 0.79162, -0.18351, -0.34275, 0.47137], [215.33098, -722.95459, 753.57861, 0.03061, -0.8128, 0.45228, -0.36587]]  # 可能要改 #目前不用
        #others
        self.intermediate_reset_pos = [[577.7265625, 387.6640930175781, 255.3731231689453, 0.7812899947166443, -0.5846999883651733, 0.2109300047159195, 0.05683000013232231],
                                       [531.023193359375, -419.52587890625, 266.6437683105469, 0.21939000487327576, 0.945110023021698, 0.12026000022888184, -0.21017999947071075] ] # 可能要改 #这个跟下面一样
        # Left reset is the previous home. Right reset is the book-grasp ready pose.
        self.left_safe_joint_deg = np.array([10.0, 20.0, 0.0, 40.0, 0.0, 20.0, 0.0])
        self.right_safe_joint_deg = np.array(
            [115.0, 85.0, -65.0, 112.0, 100.0, -15.0, 0.0]
        )
        #wineglass
        # self.intermediate_reset_pos = [[577.7265625, 387.6640930175781, 355.3731231689453, 0.7812899947166443, -0.5846999883651733, 0.2109300047159195, 0.05683000013232231],
                                    #    [531.023193359375, -419.52587890625, 366.6437683105469, 0.21939000487327576, 0.945110023021698, 0.12026000022888184, -0.21017999947071075] ]

        
        # 手心朝下拿碗
        # self.reset_pos = [[442.651, 275.714, 255.8131, 0.928853, 0.083073, 0.330899, 0.144924], [441.108, -276.737, 264.485, 0.928853, -0.083073, 0.330899, -0.144924]]
        self.workspace = [385, -330, -130, 900, 300, 290] # 可能要改
        
        # Dual arm configuration
        self.arm_size = 2
        self.udp = None
        self.config = None
        self.target_host = target_host
        self.target_local_port = target_local_port
        self.target_remote_port = target_remote_port
        self.config_file = config_file or _default_dual_v2_2_curobo_config_path()
        self.target_command = None
        self.teleop_command = None
        self.prev_arm_tool_command = None
        
        # Initialize UDP communication if available
        self.category = 'cup'
        self.cate2gripper = {'bowl': {'open': GripperMode.OPEN, 'close':GripperMode.CLOSE},
                             'cup': {'open': GripperMode.OPEN, 'close':GripperMode.VLM_CLOSE},
                             'bread': {'open': GripperMode.OPEN, 'close':GripperMode.VLM_CLOSE},
                             'fork': {'open': GripperMode.OPEN, 'close':GripperMode.VLM_CLOSE},
                             'brush': {'open': GripperMode.OPEN, 'close':GripperMode.CLOSE},
                             'dustpan': {'open': GripperMode.DUSTPAN_OPEN, 'close':GripperMode.DUSTPAN_CLOSE},
                             'wineglass':{'open': GripperMode.WINEGLASS_OPEN, 'close':GripperMode.WINEGLASS_CLOSE},
                             'book': {'open': GripperMode.OPEN, 'close': GripperMode.CLOSE}}
        
        # 物体姿势配置系统
        # 使用示例:
        # 1. 设置物体配置: self.set_object_pose('knife', tip2tool_distance=40, rotation_y_deg=-45)
        # 2. 切换物体类型: self.set_current_object('cup')
        # 3. 使用特定物体移动: self.tip_moveto(target_pos, object_type='bowl')
        # 4. 保存配置: self.save_object_poses('config.json')
        # 5. 加载配置: self.load_object_poses('config.json')
        self.ik_lib = 'curobo' #  'pinocchio'
        # self.ik_lib = 'pinocchio'
        self.object_poses = {
            'bowl': {
                'tip2tool_distance': 45,  # mm # 可能要改
                'rotation_y_deg': -30,    # 绕Y轴旋转角度
                'rotation_x_deg': 0,      # 绕X轴旋转角度
                'position_offset': [0, 0, 0]  # [x, y, z] 位置偏移
            },
            'cup': {
                'tip2tool_distance': 75,
                'rotation_y_deg': -15,
                'rotation_x_deg': 0,
                'position_offset': [0, 0, 0]
            },
            'brush': {
                'tip2tool_distance': 70,
                'rotation_y_deg': 0,
                'rotation_x_deg': 0,
                'position_offset': [-50, 0, 0]
            },
            'spoon': {
                'tip2tool_distance': 60,
                'rotation_y_deg': -20,
                'rotation_x_deg': 10,
                'position_offset': [0, 0, 0]
            },
            'fork': {
                'tip2tool_distance': 55,
                'rotation_y_deg': -25,
                'rotation_x_deg': 5,
                'position_offset': [0, 0, 0]
            }
        }
        if CUARM_AVAILABLE:
            self._init_udp_communication()
            self._load_configuration()
        self.use_gripper_claw=False
        # self.use_gripper_claw=True
        if self.use_gripper_claw:
            self.init_gripper_claw()
        
        print("==> Robot relink complete")
        time.sleep(1)

        # self.reset()
    
    def get_reset_pos(self, arm_index, eular=False):
        if eular:
            pos = self._convert_quaternion_to_pos_rot(self.reset_pos[arm_index])
        else:
            pos = self.reset_pos[arm_index]
        return pos

    def init_gripper_claw(self):
        import serial
        from controllor.gripper.uart.src.uart_servo import UartServoManager

        SERVO_PORT_NAME =  '/dev/ttyUSB0' 	# 舵机串口号
        SERVO_BAUDRATE = 115200 	# 舵机的波特率
        uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,\
                    parity=serial.PARITY_NONE, stopbits=1,\
                    bytesize=8,timeout=0)
        self.uservo = UartServoManager(uart, servo_id_list=[1])
        # uservo.set_position_time(SERVO_ID,OPEN_STATE, 1000)


    def get_cat_gripper(self, status='open', object_category='cup'):
        temp = self.cate2gripper[object_category][status].value
        return temp

    def _make_arm_tool_command(self, position, orientation):
        """Build [x, y, z, qw, qx, qy, qz] tool command for new cuarm UDP."""
        return np.concatenate([
            np.asarray(position, dtype=float),
            np.asarray(orientation, dtype=float),
        ])

    def _set_arm_tool_command(self, arm_index, position, orientation):
        self.target_command.arm_tool_command[arm_index] = self._make_arm_tool_command(position, orientation)

    def _get_arm_tool_position_orientation(self, arm_index):
        arm_tool_command = np.asarray(self.target_command.arm_tool_command[arm_index], dtype=float)
        return arm_tool_command[:3], arm_tool_command[3:]

    def _set_arm_joint_command_placeholders(self):
        self.target_command.arm_joint_size = [7] * self.arm_size
        self.target_command.arm_joint_command = [[0.0] * 7 for _ in range(self.arm_size)]

    def _init_udp_communication(self):
        """Initialize UDP communication with curobo planner"""
        try:
            # UDP configuration - adjust IP addresses and ports as needed
            if self.ik_lib == 'curobo':
                self.udp = CuarmUdpThread(
                    self.target_host, self.target_local_port,
                    self.target_host, self.target_remote_port,
                    CuarmUdp.unpack_curobo_state, 
                    CuarmUdp.pack_target_command, 
                    recv_delay_ms=10
                )
            elif self.ik_lib == 'pinocchio':
                raise NotImplementedError("pinocchio teleop mode is not migrated to the new cuarm_state/cuarm_udp protocol")
            print("UDP communication initialized successfully")
        except Exception as e:
            print(f"Failed to initialize UDP communication: {e}")
            self.udp = None


    def _convert_target_command_to_teleop_command(self):
        raise NotImplementedError("pinocchio teleop command conversion is not migrated to the new cuarm_state protocol")

    def _load_configuration(self):
        """Load robot configuration from YAML file"""
        try:
            with open(self.config_file, 'r') as file:
                self.config = yaml.safe_load(file)
            
            # Initialize target command
            self.target_command = TargetCommand()
            self.target_command.planner_mode = TargetCommandPlannerMode.PLANNING
            self.target_command.arm_command_type = TargetCommandArmCommandType.CARTESIAN
            self.target_command.arm_size = self.arm_size
            self._set_arm_joint_command_placeholders()
            self.target_command.arm_tool_command = np.zeros((self.arm_size, 7))
            self.target_command.arm_tool_command[:, 3] = 1.0
            self.target_command.gripper_command_type = TargetCommandGripperCommandType.MODE
            gripper_joint_size = _load_dual_v2_2_gripper_joint_size()
            self.target_command.gripper_size = len(gripper_joint_size)
            self.target_command.gripper_joint_size = gripper_joint_size
            self.target_command.gripper_mode_command = [self.get_cat_gripper('open')] * self.target_command.gripper_size
            self.target_command.gripper_joint_command = [[0.0] * n for n in gripper_joint_size]
            print("Robot configuration loaded successfully")
        except Exception as e:
            print(f"Failed to load configuration: {e}")
            self.config = None
            self.target_command = None

    def initialize_target_command(self):
        """
        Reset the robot arm to its original status (position and gripper)
        @return: 0 if successful, -1 if error occurs
        """
        if self.target_command:
            # Set to original location
            for arm_i in range(self.arm_size):
                # Convert reset_pos to meters and set orientation to identity quaternion
                position = np.array(self.intermediate_reset_pos[arm_i][:3]) / 1000.0
                orientation = np.array(self.intermediate_reset_pos[arm_i][3:])
                self._set_arm_tool_command(arm_i, position, orientation)
            
            # # Reset gripper to open
            for j in range(self.target_command.gripper_size):
                self.target_command.gripper_mode_command[j] = self.get_cat_gripper('open')

            
        
    def reset(self):
        """
        Reset the robot arm to its original status (position and gripper)
        @return: 0 if successful, -1 if error occurs
        """
        try:
            if self.udp and self.target_command:
                # Reset to original location
                for arm_i in range(self.arm_size):
                    # Convert reset_pos to meters and set orientation to identity quaternion
                    position = np.array(self.reset_pos[arm_i][:3]) / 1000.0
                    orientation = np.array(self.reset_pos[arm_i][3:])
                    self._set_arm_tool_command(arm_i, position, orientation)
                
                # Reset gripper to open
                for j in range(self.target_command.gripper_size):
                    self.target_command.gripper_mode_command[j] = self.get_cat_gripper('open')
                if self.ik_lib == 'curobo':
                    self.udp.send(self.target_command)
                elif self.ik_lib == 'pinocchio':
                    self._convert_target_command_to_teleop_command()
                    self.udp.send(self.teleop_command)                   
                
                # Send reset command
                self._wait_for_robot_stop()
                print("Reset command sent successfully")
                return 0
            else:
                print("UDP communication not available, using fallback reset")
                # Fallback reset logic
                return 0
        except Exception as e:
            print(f"Reset error: {e}")
            return -1

    def moveto(self, pos_rot, arm_ind=0):
        """
        Move robot arms based on arm_ind parameter
        @param pos_rot: target position and rotation or list of targets
        @param arm_ind: 0 for left arm, 1 for right arm, 3 for dual arm
        @return: 0 if successful, -1 if error occurs
        """
        if arm_ind == 0:
            return self.master_control(left_arm=pos_rot)
        elif arm_ind == 1:
            return self.master_control(right_arm=pos_rot)
        elif arm_ind == 3:
            if isinstance(pos_rot, list) and len(pos_rot) == 2:
                return self.master_control(left_arm=pos_rot[0], right_arm=pos_rot[1])
            else:
                print("For dual arm movement, pos_rot must be a list of [left_arm, right_arm]")
                return -1
        else:
            print(f"Invalid arm_ind: {arm_ind}")
            return -1

    def moveto1(self, pos_rot):
        """
        Move the left robot arm to the target position
        @param pos_rot: target position and rotation for left arm
            [x, y, z, rx, ry, rz]
            x, y, z: position in mm
            rx is pitch, ry is yaw, rz is roll, eular angle: rotation in degree
        @return: 0 if successful, -1 if error occurs
        """
        return self.master_control(left_arm=pos_rot)

    def moveto2(self, pos_rot):
        """
        Move the right robot arm to the target position
        @param pos_rot: target position and rotation for right arm
            [x, y, z, rx, ry, rz]
            x, y, z: position in mm
            rx is pitch, ry is yaw, rz is roll: rotation in degree
        @return: 0 if successful, -1 if error occurs
        """
        return self.master_control(right_arm=pos_rot)

    def _get_current_arm_state(self, arm_index):
        """
        Get current position and orientation of the specified arm
        @param arm_index: 0 for left arm, 1 for right arm
        @return: tuple (position, orientation) in meters and quaternion, or None if error
        """
        try:
            # Get current state once
            current_state = self.get_current_state()
            if current_state is None:
                return None
            
            # Extract the specified arm's data
            if arm_index == 0:
                arm_data = current_state['left_arm']
            elif arm_index == 1:
                arm_data = current_state['right_arm']
            else:
                print(f"Invalid arm index: {arm_index}")
                return None
            
            position = np.array(arm_data['position'])
            orientation = np.array(arm_data['orientation'])
            return position, orientation
                
        except Exception as e:
            print(f"Error getting current arm state: {e}")
            return None

    def _get_current_gripper_state(self, arm_index):
        """
        Get current gripper state of the specified arm
        @param arm_index: 0 for left arm, 1 for right arm
        @return: GripperMode value, or None if error
        """
        try:
            # Get current state once
            current_state = self.get_current_state()
            if current_state is None:
                return None
            
            # Extract the specified gripper's data
            if arm_index == 0:
                gripper_state = current_state['left_gripper']
            elif arm_index == 1:
                gripper_state = current_state['right_gripper']
            else:
                print(f"Invalid gripper index: {arm_index}")
                return None
            
            return gripper_state
                
        except Exception as e:
            print(f"Error getting current gripper state: {e}")
            return None

    def get_current_state(self):
        """
        Get current state of both arms and grippers in one UDP receive
        @return: dict with current states, or None if error
        {
            'left_arm': {'position': [x, y, z], 'orientation': [qw, qx, qy, qz]},
            'right_arm': {'position': [x, y, z], 'orientation': [qw, qx, qy, qz]},
            'left_gripper': gripper_mode_value,
            'right_gripper': gripper_mode_value,
            'timestamp': timestamp
        }
        """
        try:
            if not self.udp:
                print("UDP communication not available")
                return None
            
            # Receive current robot state once
            curobo_state = self.udp.receive()
            if curobo_state is None:
                print("No robot state received")
                return None
            
            # New CuroboState uses arm_tool_pose: [x, y, z, qw, qx, qy, qz].
            arm_tool_pose = np.asarray(curobo_state.arm_tool_pose, dtype=float)
            gripper_mode_command = getattr(curobo_state, "processed_gripper_mode_command", [])
            processed_arm_joint_command = getattr(
                curobo_state, "processed_arm_joint_command", []
            )
            processed_arm_tool_command = getattr(
                curobo_state, "processed_arm_tool_command", []
            )
            joint_velocity = np.asarray(
                getattr(curobo_state, "arm_joint_velocity", []),
                dtype=float,
            ).reshape(-1)
            
            # Check if we have data for both arms
            if len(arm_tool_pose) < 2: #or don't have gripper satus
                # len(gripper_mode_command) < 2):
                print("Insufficient data received for both arms")
                return None
            
            # Build state dictionary
            current_state = {
                'left_arm': {
                    'position': arm_tool_pose[0, :3].tolist(),  # Already in meters
                    'orientation': arm_tool_pose[0, 3:].tolist()
                },
                'right_arm': {
                    'position': arm_tool_pose[1, :3].tolist(),
                    'orientation': arm_tool_pose[1, 3:].tolist()
                },
                # 'left_gripper': int(gripper_mode_command[0]),
                # 'right_gripper': int(gripper_mode_command[1]),
                'timestamp': time.time(),
                "joint_state": curobo_state.arm_joint_position,
                "joint_velocity": joint_velocity.tolist(),
                "processed_arm_joint_command": processed_arm_joint_command,
                "processed_arm_tool_command": processed_arm_tool_command,
                'robot_status': curobo_state.robot_status,
                'curobo_status': curobo_state.curobo_status
            }
            if len(gripper_mode_command) >= 2:
                current_state['left_gripper'] = gripper_mode_command[0]
                current_state['right_gripper'] = gripper_mode_command[1]
            
            logging.debug("Current state retrieved successfully")
            return current_state
            
        except Exception as e:
            print(f"Error getting current state: {e}")
            return None

    def get_current_pos_rot(self, arm_index):
        """
        Get current position and rotation of the specified arm in pos_rot format
        @param arm_index: 0 for left arm, 1 for right arm
        @return: pos_rot [x, y, z, rx, ry, rz] in mm and degrees, or None if error
        """
        try:
            # Get current state once
            current_state = self.get_current_state()
            if current_state is None:
                return None
            
            # Extract the specified arm's data
            if arm_index == 0:
                arm_data = current_state['left_arm']
            elif arm_index == 1:
                arm_data = current_state['right_arm']
            else:
                print(f"Invalid arm index: {arm_index}")
                return None
            
            # Convert position from meters to millimeters and quaternion to euler angles
            position = np.array(arm_data['position']) * 1000.0  # Convert to mm
            orientation = np.array(arm_data['orientation'])
            
            # Convert to pos_rot format
            pos_quat = list(position) + list(orientation)  # [x, y, z, qx, qy, qz, qw]
            pos_rot = self._convert_quaternion_to_pos_rot(pos_quat)
            
            return pos_rot
            
        except Exception as e:
            print(f"Error getting current pos_rot: {e}")
            return None

    def get_both_arms_pos_rot(self):
        """
        Get current position and rotation of both arms in pos_rot format
        @return: list of [left_pos_rot, right_pos_rot] or None if error
        """
        try:
            # Get current state once
            current_state = self.get_current_state()
            if current_state is None:
                return None
            
            # Convert both arms to pos_rot format
            left_pos_rot = self.get_current_pos_rot(0)
            right_pos_rot = self.get_current_pos_rot(1)
            
            if left_pos_rot is not None and right_pos_rot is not None:
                return [left_pos_rot, right_pos_rot]
            else:
                return None
                
        except Exception as e:
            print(f"Error getting both arms pos_rot: {e}")
            return None

    def _wait_for_robot_stop_legacy(self, max_wait_time=10.0, check_interval=0.1):
        """
        Wait for the entire robot (both arms) to stop moving
        @param max_wait_time: maximum time to wait in seconds
        @param check_interval: interval between checks in seconds
        @return: 0 if successful, -1 if timeout or error
        """
        # try:
        start_time = time.time()
        # prev_arm_positions = None
        # prev_arm_orientations = None
        # cnt = 0
        # max_cnt = 50  # Number of consecutive readings that must be the same
        curobo_started_planning = False
        robot_started_running = False
        same_target_wait_cycle = 15
        
        #If control gripper only, wait for n cycles
        if self.prev_arm_tool_command is not None and \
            np.allclose(self.prev_arm_tool_command, self.target_command.arm_tool_command, atol=1e-3):
            print(f"Control_gripper_only, sleep for {same_target_wait_cycle} cycles")
            getattr(self, 'operator_sleep', time.sleep)(check_interval * same_target_wait_cycle)
            return 0
        
        while time.time() - start_time < max_wait_time:
            # Get current state once
            current_state = self.get_current_state()
            if current_state is None:
                time.sleep(check_interval)
                continue
            
            # print(f"curobo_status: {current_state['curobo_status']}, robot_status: {current_state['robot_status']}")
            if curobo_started_planning:
                if current_state['curobo_status'].value == CuroboStatus.PLANNING_FAILED.value:
                    print("Curobo planning failed")
                    return 0
                elif current_state['curobo_status'].value == CuroboStatus.PLANNING_SUCCEEDED.value:
                    if robot_started_running and current_state['robot_status'].value == RobotStatus.AVAILABLE.value:
                        print("Robot finished moving")
                        return 0
                    elif not robot_started_running and current_state['robot_status'].value == RobotStatus.BUSY.value:
                        robot_started_running = True
                        print("Robot started moving")
            elif current_state['curobo_status'].value == CuroboStatus.PLANNING.value:
                curobo_started_planning = True
                print("Curobo started planning")

            # if prev_arm_positions is not None and prev_arm_orientations is not None:
            #     # Check if the entire robot has stopped using check_if_robot_stopped
            #     # print("prev_arm_positions: ", prev_arm_positions)
            #     # print("current_arm_positions: ", current_arm_positions)
            #     # print("prev_arm_orientations: ", prev_arm_orientations)
            #     # print("current_arm_orientations: ", current_arm_orientations)
            #     robot_stopped, new_cnt = check_if_robot_stopped(
            #         prev_arm_positions, prev_arm_orientations,
            #         current_arm_positions, current_arm_orientations,
            #         cnt, max_cnt
            #     )

            #     cnt = new_cnt
                
            #     if robot_stopped:
            #         print("Robot stopped moving")
            #         return 0
            
            # # Update previous positions and orientations
            # prev_arm_positions = current_arm_positions
            # prev_arm_orientations = current_arm_orientations
            
            time.sleep(check_interval)
        
        # Timeout occurred
        print("Timeout waiting for robot to stop moving")
        return -1
            
        # except Exception as e:
        #     print(f"Error waiting for robot stop: {e}")
        #     return -1

    @staticmethod
    def _status_value(status):
        """Return a comparable enum value without assuming an enum implementation."""
        return getattr(status, "value", status)

    def _target_pose_diagnostics(self, current_state):
        """Return Cartesian error between current tools and commanded tools."""
        target = np.asarray(self.target_command.arm_tool_command, dtype=float)
        current = np.asarray(
            [
                current_state["left_arm"]["position"]
                + current_state["left_arm"]["orientation"],
                current_state["right_arm"]["position"]
                + current_state["right_arm"]["orientation"],
            ],
            dtype=float,
        )
        if target.shape != current.shape or target.shape != (2, 7):
            return None

        position_error_m = np.linalg.norm(
            current[:, :3] - target[:, :3], axis=1
        )
        current_quat = current[:, 3:]
        target_quat = target[:, 3:]
        current_norm = np.linalg.norm(current_quat, axis=1)
        target_norm = np.linalg.norm(target_quat, axis=1)
        if np.any(current_norm < 1e-9) or np.any(target_norm < 1e-9):
            return None
        dot = np.abs(
            np.sum(
                current_quat / current_norm[:, None]
                * target_quat / target_norm[:, None],
                axis=1,
            )
        )
        orientation_error_rad = 2.0 * np.arccos(np.clip(dot, 0.0, 1.0))
        arm_names = ("left", "right")
        per_arm = {}
        for arm_index, arm_name in enumerate(arm_names):
            target_position_mm = target[arm_index, :3] * 1000.0
            current_position_mm = current[arm_index, :3] * 1000.0
            per_arm[arm_name] = {
                "target_position_mm": target_position_mm.tolist(),
                "current_position_mm": current_position_mm.tolist(),
                "position_delta_mm": (
                    current_position_mm - target_position_mm
                ).tolist(),
                "position_error_m": float(position_error_m[arm_index]),
                "orientation_error_rad": float(
                    orientation_error_rad[arm_index]
                ),
            }
        return {
            "max_position_error_m": float(np.max(position_error_m)),
            "max_orientation_error_rad": float(np.max(orientation_error_rad)),
            "per_arm": per_arm,
        }

    def _target_joint_diagnostics(self, current_state):
        """Return arm-joint error for a joint-space command."""
        target_joint = np.asarray(
            self.target_command.arm_joint_command, dtype=float
        ).reshape(-1)
        current_joint = np.asarray(
            current_state.get("joint_state", []), dtype=float
        ).reshape(-1)
        if not target_joint.size or current_joint.size < target_joint.size:
            return None
        joint_error_rad = np.abs(
            current_joint[:target_joint.size] - target_joint
        )
        return {
            "max_joint_error_rad": float(np.max(joint_error_rad)),
            "joint_count": int(target_joint.size),
        }

    def _wait_for_robot_stop(
        self,
        max_wait_time=40.0,
        check_interval=0.1,
        position_tolerance_m=0.020,
        orientation_tolerance_rad=0.10,
        joint_tolerance_rad=0.0175,
        max_missing_state_cycles=20,
        completion_settle_timeout=10.0,
    ):
        """
        Wait for a commanded motion to complete.

        Completion follows the legacy controller-state logic: Curobo reports
        motion finished, or planning succeeds and the panel transitions from
        BUSY back to AVAILABLE. Cartesian/joint tolerances decide whether
        tracking succeeded once the controller reports motion complete.
        """
        start_time = time.time()
        curobo_started_planning = False
        curobo_succeeded = False
        robot_started_running = False
        missing_state_cycles = 0
        last_diagnostics = None
        last_curobo_status = None
        last_robot_status = None
        same_target_wait_cycle = 15
        command_type = self._status_value(
            self.target_command.arm_command_type
        )
        joint_command_value = self._status_value(
            TargetCommandArmCommandType.JOINT
        )
        is_joint_command = command_type == joint_command_value
        if is_joint_command:
            expected_processed_command = np.asarray(
                self.target_command.arm_joint_command, dtype=float
            ).copy()
        else:
            expected_processed_command = np.asarray(
                self.target_command.arm_tool_command, dtype=float
            ).copy()
        command_acknowledged = False
        command_acknowledged_at = None
        completion_detected_at = None

        def _print_tcp_diagnostics(diagnostics):
            for arm_name in ("left", "right"):
                arm_diagnostics = diagnostics["per_arm"][arm_name]
                print(
                    f"{arm_name} TCP: "
                    f"target_mm={np.round(arm_diagnostics['target_position_mm'], 1).tolist()}, "
                    f"actual_mm={np.round(arm_diagnostics['current_position_mm'], 1).tolist()}, "
                    f"delta_mm(actual-target)="
                    f"{np.round(arm_diagnostics['position_delta_mm'], 1).tolist()}, "
                    f"position_error="
                    f"{arm_diagnostics['position_error_m'] * 1000.0:.1f}mm, "
                    f"orientation_error="
                    f"{arm_diagnostics['orientation_error_rad']:.3f}rad"
                )

        def report_completion_error(current_state, *, verbose=False):
            """Return True when commanded tracking is within tolerance."""
            if is_joint_command:
                diagnostics = self._target_joint_diagnostics(current_state)
                if diagnostics is None:
                    return True
                if diagnostics["max_joint_error_rad"] > joint_tolerance_rad:
                    if verbose:
                        print(
                            "动作跟踪失败：最大关节误差 "
                            f"{diagnostics['max_joint_error_rad']:.4f}rad "
                            f"超过阈值 {joint_tolerance_rad:.4f}rad。"
                        )
                    return False
                return True

            diagnostics = self._target_pose_diagnostics(current_state)
            if diagnostics is None:
                return True

            commanded_arms = getattr(
                self, "_cartesian_commanded_arms", (0, 1)
            )
            tracking_ok = True
            for arm_index in commanded_arms:
                arm_name = ("left", "right")[arm_index]
                arm_diagnostics = diagnostics["per_arm"][arm_name]
                position_error_m = arm_diagnostics["position_error_m"]
                orientation_error_rad = arm_diagnostics["orientation_error_rad"]
                if position_error_m > position_tolerance_m:
                    tracking_ok = False
                    if verbose:
                        print(
                            f"动作跟踪失败：{arm_name} 臂 TCP 位置误差 "
                            f"{position_error_m * 1000.0:.1f}mm 超过阈值 "
                            f"{position_tolerance_m * 1000.0:.1f}mm。"
                        )
                if orientation_error_rad > orientation_tolerance_rad:
                    tracking_ok = False
                    if verbose:
                        print(
                            f"动作跟踪失败：{arm_name} 臂 TCP 姿态误差 "
                            f"{orientation_error_rad:.3f}rad 超过阈值 "
                            f"{orientation_tolerance_rad:.3f}rad。"
                        )
            if not tracking_ok and verbose:
                _print_tcp_diagnostics(diagnostics)
            return tracking_ok

        def completion_result(current_state, success_message):
            """Wait for measured state to catch up after an early done signal."""
            nonlocal completion_detected_at
            if report_completion_error(current_state):
                print(success_message)
                return True

            now = time.time()
            if completion_detected_at is None:
                completion_detected_at = now
                print(
                    "控制器已报告运动结束，实机反馈尚未到位；"
                    f"继续等待最多 {completion_settle_timeout:.1f}s。"
                )
            if now - completion_detected_at < completion_settle_timeout:
                return None

            report_completion_error(current_state, verbose=True)
            print(
                "控制器报告结束后实机反馈仍未收敛，"
                "本步判为失败。"
            )
            return False

        if not is_joint_command and self.prev_arm_tool_command is not None and \
            np.allclose(
                self.prev_arm_tool_command,
                self.target_command.arm_tool_command,
                atol=1e-3,
            ):
            print(f"Control_gripper_only, sleep for {same_target_wait_cycle} cycles")
            getattr(self, 'operator_sleep', time.sleep)(check_interval * same_target_wait_cycle)
            return True

        failure_values = {
            self._status_value(getattr(CuroboStatus, name))
            for name in ("PLANNING_FAILED", "IK_FAILED", "COLLISION_CHECK_FAILED")
            if hasattr(CuroboStatus, name)
        }
        planning_value = self._status_value(CuroboStatus.PLANNING)
        succeeded_value = self._status_value(CuroboStatus.PLANNING_SUCCEEDED)
        finished_value = self._status_value(CuroboStatus.MOTION_FINISHED)
        available_value = self._status_value(RobotStatus.AVAILABLE)
        busy_value = self._status_value(RobotStatus.BUSY)

        while time.time() - start_time < max_wait_time:
            current_state = self.get_current_state()
            if current_state is None:
                missing_state_cycles += 1
                if missing_state_cycles >= max_missing_state_cycles:
                    print(
                        "Motion completion failed: no Curobo state received for "
                        f"{missing_state_cycles * check_interval:.1f}s."
                    )
                    return False
                time.sleep(check_interval)
                continue

            missing_state_cycles = 0
            curobo_status = self._status_value(current_state["curobo_status"])
            robot_status = self._status_value(current_state["robot_status"])
            last_curobo_status = curobo_status
            last_robot_status = robot_status

            if is_joint_command:
                processed_command = np.asarray(
                    current_state.get("processed_arm_joint_command", []),
                    dtype=float,
                )
                response_matches_command = (
                    processed_command.shape == expected_processed_command.shape
                    and np.allclose(
                        processed_command,
                        expected_processed_command,
                        atol=1e-5,
                    )
                )
            else:
                processed_command = np.asarray(
                    current_state.get("processed_arm_tool_command", []),
                    dtype=float,
                )
                response_matches_command = (
                    processed_command.shape == expected_processed_command.shape
                    and np.allclose(
                        processed_command[:, :3],
                        expected_processed_command[:, :3],
                        atol=1e-5,
                    )
                    and np.all(
                        np.abs(
                            np.sum(
                                processed_command[:, 3:]
                                * expected_processed_command[:, 3:],
                                axis=1,
                            )
                        )
                        >= 1.0 - 1e-5
                    )
                )

            if not response_matches_command:
                time.sleep(check_interval)
                continue
            if not command_acknowledged:
                command_acknowledged = True
                command_acknowledged_at = time.time()
                print("Curobo acknowledged current command")

            if curobo_status in failure_values:
                if (
                    command_acknowledged_at is not None
                    and time.time() - command_acknowledged_at < 3.0
                ):
                    # The state packet can still contain the previous
                    # command's terminal failure while the newly acknowledged
                    # command is entering MotionGen.
                    time.sleep(check_interval)
                    continue
                print(f"Curobo planning failed: status={curobo_status}")
                return False
            if curobo_status == finished_value:
                result = completion_result(
                    current_state,
                    "Robot finished moving (Curobo motion-finished status)",
                )
                if result is not None:
                    return result
                time.sleep(check_interval)
                continue
            if curobo_status == planning_value and not curobo_started_planning:
                curobo_started_planning = True
                print("Curobo started planning")
            elif curobo_status == succeeded_value:
                curobo_started_planning = True
                curobo_succeeded = True

            if curobo_succeeded and robot_status == busy_value and not robot_started_running:
                robot_started_running = True
                print("Robot started moving")

            if curobo_succeeded:
                if is_joint_command:
                    last_diagnostics = self._target_joint_diagnostics(current_state)
                else:
                    last_diagnostics = self._target_pose_diagnostics(current_state)
                if robot_started_running and robot_status == available_value:
                    result = completion_result(
                        current_state,
                        "Robot finished moving "
                        "(planner succeeded and panel returned AVAILABLE)",
                    )
                    if result is not None:
                        return result
            time.sleep(check_interval)

        diagnostic_text = "unavailable"
        if last_diagnostics is not None:
            if is_joint_command:
                diagnostic_text = (
                    f"max_joint_error={last_diagnostics['max_joint_error_rad']:.4f}rad"
                )
            else:
                diagnostic_text = (
                    f"max_position_error={last_diagnostics['max_position_error_m']:.4f}m, "
                    f"max_orientation_error={last_diagnostics['max_orientation_error_rad']:.3f}rad"
                )
        print(
            "Motion completion timeout: "
            f"curobo_status={last_curobo_status}, robot_status={last_robot_status}, "
            f"command_acknowledged={command_acknowledged}, "
            f"{diagnostic_text}."
        )
        if not is_joint_command and last_diagnostics is not None:
            for arm_name in ("left", "right"):
                arm_diagnostics = last_diagnostics["per_arm"][arm_name]
                target_mm = np.round(
                    arm_diagnostics["target_position_mm"], 1
                ).tolist()
                current_mm = np.round(
                    arm_diagnostics["current_position_mm"], 1
                ).tolist()
                delta_mm = np.round(
                    arm_diagnostics["position_delta_mm"], 1
                ).tolist()
                print(
                    f"{arm_name} TCP: target_mm={target_mm}, "
                    f"actual_mm={current_mm}, "
                    f"delta_mm(actual-target)={delta_mm}, "
                    f"position_error="
                    f"{arm_diagnostics['position_error_m'] * 1000.0:.1f}mm, "
                    f"orientation_error="
                    f"{arm_diagnostics['orientation_error_rad']:.3f}rad"
                )
        return False

    def moveto_dual_arm(self, left_pos_rot, right_pos_rot):
        """
        Move both robot arms to different target positions
        @param left_pos_rot: left arm target position and rotation [x, y, z, rx, ry, rz]
        @param right_pos_rot: right arm target position and rotation [x, y, z, rx, ry, rz]
        @return: 0 if successful, -1 if error occurs
        """
        return self.master_control(left_arm=left_pos_rot, right_arm=right_pos_rot)

    def _convert_pos_rot_to_quaternion(self, pos_rot):
        """Convert position (mm) and euler angles (deg, ZYX) to position (m) and quaternion"""
        x, y, z, rx, ry, rz = pos_rot  # rx=X roll, ry=Y pitch, rz=Z yaw

        # 保持 mm，不转换为米
        position = np.array([x, y, z])/1000.0
        # R_mat = Rotation.from_euler('XYZ', pos_rot[3:], degrees=True).as_matrix()
        R_mat = angle2rotation(rx,ry,rz)
        quat = Rotation.from_matrix(R_mat).as_quat(scalar_first=True)
        orientation=quat
        return position, orientation


    def _convert_quaternion_to_pos_rot(self, pos_quat):
        """Convert position (mm) and quaternion (scalar-first [w,x,y,z]) to position (mm) and euler angles (deg, XYZ)"""
        position = np.array(pos_quat[:3], dtype=float)
        orientation = np.array(pos_quat[3:], dtype=float)  # [w, x, y, z]

        # 将 scalar-first 四元数 [w, x, y, z] 转为 SciPy 期望的 [x, y, z, w]
        # quat_xyzw = np.array([orientation[1], orientation[2], orientation[3], orientation[0]], dtype=float)

        # # 与 _convert_pos_rot_to_quaternion 使用的欧拉顺序保持一致（'XYZ'，角度制）
        # euler_deg = Rotation.from_quat(quat_xyzw).as_euler('XYZ', degrees=True)
        rotmat = quaternion_to_matrix(orientation)
        euler_deg = rotation2angle(rotmat)
        rx, ry, rz = euler_deg.tolist()
        return [position[0], position[1], position[2], rx, ry, rz]


    def tip_moveto(self, pos_rot, object_type=None):
        """
        将机器人末端执行器移动到指定位置和姿态
        根据物体类型自动应用相应的变换参数
        @param pos_rot: 目标位置和姿态 [x, y, z, rx, ry, rz]
        @param object_type: 物体类型，如果为None则使用当前category
        """
        if object_type is None:
            object_type = self.category
            
        # 获取物体配置
        if object_type in self.object_poses:
            config = self.object_poses[object_type]
            rotation_y_deg = config['rotation_y_deg']
            rotation_x_deg = config['rotation_x_deg']
            position_offset = config['position_offset']
        else:
            print(f"警告: 未找到物体类型 '{object_type}' 的配置，使用默认参数")
            rotation_y_deg = 0
            rotation_x_deg = 0
            position_offset = [0, 0, 0]

        # 构建绕Y轴旋转矩阵
        angle_rad = np.deg2rad(rotation_y_deg)
        R_y = np.array([
            [np.cos(angle_rad),  0, np.sin(angle_rad), 0],
            [0,                  1, 0, 0],
            [-np.sin(angle_rad), 0, np.cos(angle_rad), 0],
            [0,                 0,                 0, 1]
        ])

        # 构建绕X轴旋转矩阵
        angle_rad = np.deg2rad(rotation_x_deg)
        R_x = np.array([
            [1, 0, 0, 0],
            [0, np.cos(angle_rad), -np.sin(angle_rad), 0],
            [0, np.sin(angle_rad), np.cos(angle_rad), 0],
            [0,                 0,                 0, 1]
        ])

        # 计算工具位置
        tip2tool = self.tip2tool_transmat(object_type)
        tip2w = pos_to_mat(pos_rot)
        tool2tip = np.linalg.inv(tip2tool) 
        tool2w = tip2w @ tool2tip 
        
        # 应用旋转
        tool2w = tool2w @ R_x @ R_y

        # 转换为位置姿态格式
        tool_pos_rot = mat_to_pos(tool2w)
        
        # 应用位置偏移
        tool_pos_rot[0] += position_offset[0]  # x
        tool_pos_rot[1] += position_offset[1]  # y
        tool_pos_rot[2] += position_offset[2]  # z
        
        print(f"物体类型: {object_type}, 目标位置: {tool_pos_rot}")
        self.moveto(tool_pos_rot)
        return

    def tip_moveto2(self, pos_rot):
        """
        Move the robot's end effector (tip) to the specified position and orientation.
        actually use the tip pose to compuate the tool pose in world coordinate
        """
        tip2tool = self.tip2tool_transmat()
        tip2w = pos_to_mat(pos_rot)
        tool2tip = np.linalg.inv(tip2tool) 
        tool2w = tip2w @ tool2tip 
       
        tool_pos_rot = mat_to_pos(tool2w)
        self.moveto(tool_pos_rot)
        return

    def restore_config(self, file):
        """
        Restore the robot arm configuration from a JSON file
        """
        config_paras = {
            "reset_pos": self.reset_pos,
            "workspace": self.workspace,
        }
        # write config_paras to a json file
        with open(file, 'w') as f:
            json.dump(config_paras, f, indent=4)
        # print("Configuration saved to %s", file)

    def read_config(self, file):
        """
        Configure the robot arm form a JSON file
        """
        with open(file, 'r') as f:
            config_paras = json.load(f)
        self.reset_pos = config_paras["reset_pos"]
        self.workspace = config_paras["workspace"]
    
    def act_gripper(self, status, object_category='cup'):
        """
        Activate both grippers with the same status
        @param status: 0 for open, 1 for close
        @return: status: [fault,status]: 
            fault (0 - no error,1-error), 
            status (0 -non finishing moving, 1- finishing moving)
        """
        if self.use_gripper_claw:
            if status[1]==1:
                result = self.uservo.set_position_time(1, 500, 1000)
            else:
                result = self.uservo.set_position_time(1, 3000, 1000)
        else:
            print("act_gripper: ", object_category)
            result = self.master_control(left_gripper=status[0], right_gripper=status[1], object_category=object_category)
        if result == 0:
            return [0, 1]  # [fault, status]
        else:
            return [1, 0]  # [fault, status]
    
    def act_gripper1(self, status):
        """
        Control the left arm gripper
        @param status: 1: close gripper, 0: open gripper
        @return: status: [fault,status]: 
            fault (0 - no error,1-error), 
            status (0 -non finishing moving, 1- finishing moving)
        """
        result = self.master_control(left_gripper=status)
        if result == 0:
            return [0, 1]  # [fault, status]
        else:
            return [1, 0]  # [fault, status]
    
    def act_gripper2(self, status):
        """
        Control the right arm gripper
        @param status: 1: close gripper, 0: open gripper
        @return: status: [fault,status]: 
            fault (0 - no error,1-error), 
            status (0 -non finishing moving, 1- finishing moving)
        """
        result = self.master_control(right_gripper=status)
        if result == 0:
            return [1, 0]  # [fault, status]
        else:
            return [1, 0]  # [fault, status]
    
    def get_tool_pose(self, flag=1):
        """
        Get the status of the robot arm TCP (Tool (centroid of hand) Current Pose)
        :param flag: 0 for blocking, 1 for non-blocking
        :return: [fault, tcp_pose]: 
            fault (0 - no error, 1 - error), 
            tcp_pose (x, y, z, rx, ry, rz) - position and rotation
        """
        # flag：0-阻塞，1-非阻塞 默认1
        # return 错误码, tcp_pose=[x,y,z,rx,ry,rz]
        # pos6d = self.get_current_pos_rot(0)  
        curr_state = self.get_current_pos_rot(0)  # Ensure we have the latest state
        curr_state = np.zeros(6).tolist() if curr_state is None else curr_state  # Fallback to zero if None

        # pos6d = self._convert_quaternion_to_pos_rot(self.reset_pos[0])  # Use the reset position for left arm as an example
        ret = [0, curr_state]
        
        #TODO: receive message as the list
        return ret[0], ret[1]  # [fault, tcp_pose]
    
    def get_workspace(self):
        """
        Get the workspace of the robot arm
        :return: workspace [xmin, ymin, zmin, xmax, ymax, zmax]
        """
        return self.workspace
    
    def tip2tool_transmat(self, object_type=None):
        """
        创建从末端执行器到工具的变换矩阵
        @param object_type: 物体类型，如果为None则使用当前category
        @return: 4x4变换矩阵
        """
        if object_type is None:
            object_type = self.category
            
        # 获取物体配置，如果不存在则使用默认配置
        if object_type in self.object_poses:
            config = self.object_poses[object_type]
            tip2tool_dist = config['tip2tool_distance']
        else:
            print(f"警告: 未找到物体类型 '{object_type}' 的配置，使用默认距离50mm")
            tip2tool_dist = 50
            
        tip2tool = np.eye(4)
        tip2tool[0, 3] = tip2tool_dist

        return tip2tool
    
    def set_object_pose(self, object_type, tip2tool_distance=None, rotation_y_deg=None, 
                       rotation_x_deg=None, position_offset=None):
        """
        设置或更新物体姿势配置
        @param object_type: 物体类型
        @param tip2tool_distance: 末端执行器到工具的距离(mm)
        @param rotation_y_deg: 绕Y轴旋转角度(度)
        @param rotation_x_deg: 绕X轴旋转角度(度)
        @param position_offset: 位置偏移 [x, y, z] (mm)
        """
        if object_type not in self.object_poses:
            self.object_poses[object_type] = {
                'tip2tool_distance': 50,
                'rotation_y_deg': 0,
                'rotation_x_deg': 0,
                'position_offset': [0, 0, 0]
            }
        
        config = self.object_poses[object_type]
        if tip2tool_distance is not None:
            config['tip2tool_distance'] = tip2tool_distance
        if rotation_y_deg is not None:
            config['rotation_y_deg'] = rotation_y_deg
        if rotation_x_deg is not None:
            config['rotation_x_deg'] = rotation_x_deg
        if position_offset is not None:
            config['position_offset'] = position_offset
            
        print(f"已更新物体 '{object_type}' 的姿势配置: {config}")
    
    def get_object_pose(self, object_type):
        """
        获取物体姿势配置
        @param object_type: 物体类型
        @return: 物体配置字典，如果不存在则返回None
        """
        return self.object_poses.get(object_type, None)
    
    # TODO: @ckli
    def tip2tool(self, tip_pose):
        """
        get the tool world coord based on the tip pose world coordinate
        """
        tip2tool = self.tip2tool_transmat()   
        tip2w = pos_to_mat(tip_pose)
        tool2tip = np.linalg.inv(tip2tool) 
        tool2w = tip2w @ tool2tip 
       
        tool_pos = mat_to_pos(tool2w)
        return tool_pos
    
    def cleanup(self):
        """Cleanup UDP communication"""
        if self.udp:
            self.udp.terminate()
            print("UDP communication terminated")

    def shutdown(self):
        """
        Terminate UDP thread and clean up resources
        """
        try:
            if self.udp:
                print("Terminating UDP thread...")
                self.udp.terminate()
                self.udp = None
                print("UDP thread terminated")
            return 0
        except Exception as e:
            print(f"Error terminating UDP: {e}")
            return -1

    def __del__(self):
        """
        Destructor to ensure cleanup when object is destroyed
        """
        try:
            if hasattr(self, 'udp') and self.udp:
                self.shutdown()
        except:
            pass  # Ignore errors during destruction
    
    def record_left_safe_joint_pose(self):
        """Print the current left-arm joints without sending any robot command."""
        current_state = self.get_current_state()
        if current_state is None or 'joint_state' not in current_state:
            print("无法记录左臂安全姿态：未收到 planner_to_panel 关节状态。")
            return None

        joint_state = np.asarray(current_state['joint_state'], dtype=float).reshape(-1)
        if joint_state.size < 7 or not np.isfinite(joint_state[:7]).all():
            print("无法记录左臂安全姿态：左臂关节状态无效。")
            return None

        left_joint_rad = joint_state[:7].copy()
        left_joint_deg = np.rad2deg(left_joint_rad)
        print(
            "已记录左臂当前关节姿态（未发送任何控制命令）：\n"
            f"  LEFT_SAFE_JOINT_RAD = {np.round(left_joint_rad, 6).tolist()}\n"
            f"  LEFT_SAFE_JOINT_DEG = {np.round(left_joint_deg, 3).tolist()}"
        )
        return left_joint_rad

    def control_joint_position(
        self, left_pos, right_pos, action, *, force=False
    ):
        current_state = self.get_current_state()
        if current_state is None or 'joint_state' not in current_state:
            print("No robot state available for joint reset; check planner_to_panel UDP link")
            return False
        measured_joint_state = np.asarray(
            current_state['joint_state'], dtype=float
        ).reshape(-1)
        joint_state = measured_joint_state.copy()
        # print(joint_state)
        
        self.target_command.arm_command_type = TargetCommandArmCommandType.JOINT
        self.target_command.arm_joint_size = [7] * self.arm_size
        if left_pos==True:
            joint_state[:7] = np.deg2rad(self.left_safe_joint_deg)
            left_tool_position = np.array(self.intermediate_reset_pos[0][:3]) / 1000.
            left_tool_orientation = np.array(self.intermediate_reset_pos[0][3:])
            self._set_arm_tool_command(0, left_tool_position, left_tool_orientation)
        if right_pos==True:
            joint_state[7:] = np.deg2rad(self.right_safe_joint_deg)
            right_tool_position = np.array(self.intermediate_reset_pos[1][:3]) / 1000.
            right_tool_orientation = np.array(self.intermediate_reset_pos[1][3:])
            self._set_arm_tool_command(1, right_tool_position, right_tool_orientation)
        self.target_command.arm_joint_command = joint_state.reshape(self.arm_size, 7)
        max_reset_error = float(
            np.max(np.abs(joint_state - measured_joint_state))
        )
        reset_tolerance = np.deg2rad(1.0)
        if not force and max_reset_error <= reset_tolerance:
            print(
                "Joint reset already satisfied; skipping duplicate planning: "
                f"action={action}, max_joint_error="
                f"{np.rad2deg(max_reset_error):.3f}deg"
            )
            return True
        # The planner normally de-duplicates identical joint targets. A reset
        # must be a fresh request every time, including across task rounds.
        reset_nonce = time.time_ns()
        self.target_command.description = f"{action}:joint_reset:{reset_nonce}"
        print(
            f"Sending fresh joint reset command: action={action}, nonce={reset_nonce}"
        )

        self.udp.send(self.target_command)

        return self._wait_for_robot_stop()

    def control_zero_position(self, action="zero_position"):
        """Move both arms to the URDF joint-space zero configuration."""
        current_state = self.get_current_state()
        if current_state is None or 'joint_state' not in current_state:
            print("No robot state available for zero-position command")
            return False

        self.target_command.arm_command_type = TargetCommandArmCommandType.JOINT
        self.target_command.arm_joint_size = [7] * self.arm_size
        self.target_command.arm_joint_command = np.zeros(
            (self.arm_size, 7), dtype=float
        )
        zero_nonce = time.time_ns()
        self.target_command.description = f"{action}:joint_zero:{zero_nonce}"
        print(
            "Sending fresh joint zero command: "
            f"action={action}, nonce={zero_nonce}"
        )
        self.udp.send(self.target_command)
        return self._wait_for_robot_stop()
        
    def _target_position_is_reasonable(self, pos_rot, arm_name, max_abs_mm=5000):
        if pos_rot is None:
            return True
        pos = np.asarray(pos_rot[:3], dtype=float)
        if np.any(np.abs(pos) > max_abs_mm):
            print(f"Reject {arm_name} target outside sanity range: {pos.tolist()} mm")
            return False
        return True

    def preview_right_cartesian_motion(self, pos_rot, description, timeout=12.0):
        """Ask the planner to validate a right-arm pose without panel motion.

        Requires the planner_to_panel process from this workspace, which
        recognizes the ``__preview__:`` description prefix and deliberately
        withholds the resulting panel command.  On success, also return the
        planner's final, limit-clipped dual-arm joint waypoint.
        """
        if not self.udp or not self.target_command:
            return False, "UDP communication not available", None
        if not self._target_position_is_reasonable(pos_rot, "right_arm_preview"):
            return False, "target outside sanity range", None

        current_state = self.get_current_state()
        if current_state is None:
            return False, "no current Curobo state", None

        preview = copy.deepcopy(self.target_command)
        preview.arm_command_type = TargetCommandArmCommandType.CARTESIAN
        preview.arm_joint_size = [7] * self.arm_size
        preview.arm_tool_command = np.asarray(
            self.target_command.arm_tool_command, dtype=float
        ).copy()
        if preview.arm_tool_command.shape != (self.arm_size, 7):
            return False, "invalid target command shape", None

        left_state = current_state["left_arm"]
        preview.arm_tool_command[0] = np.asarray(
            left_state["position"] + left_state["orientation"], dtype=float
        )
        right_position, right_orientation = self._convert_pos_rot_to_quaternion(
            pos_rot
        )
        preview.arm_tool_command[1] = np.concatenate(
            [right_position, right_orientation]
        )
        preview.description = f"__preview__:{description}:{time.time_ns()}"
        self.udp.send(preview)

        failure_values = {
            self._status_value(getattr(CuroboStatus, name))
            for name in ("PLANNING_FAILED", "IK_FAILED", "COLLISION_CHECK_FAILED")
            if hasattr(CuroboStatus, name)
        }
        success_values = {
            self._status_value(CuroboStatus.PLANNING_SUCCEEDED),
            self._status_value(CuroboStatus.MOTION_FINISHED),
        }
        deadline = time.time() + timeout
        while time.time() < deadline:
            state = self.get_current_state()
            if state is not None:
                status = self._status_value(state["curobo_status"])
                processed_tool = np.asarray(
                    state.get("processed_arm_tool_command", []), dtype=float
                )
                response_matches_preview = (
                    processed_tool.shape == (self.arm_size, 7)
                    and np.allclose(
                        processed_tool[1, :3],
                        preview.arm_tool_command[1, :3],
                        atol=1e-5,
                    )
                    and abs(
                        float(
                            np.dot(
                                processed_tool[1, 3:],
                                preview.arm_tool_command[1, 3:],
                            )
                        )
                    ) >= 1.0 - 1e-5
                )
                if not response_matches_preview:
                    time.sleep(0.05)
                    continue
                if status in success_values:
                    final_joint = np.asarray(
                        state.get("processed_arm_joint_command", []), dtype=float
                    )
                    if final_joint.shape != (self.arm_size, 7):
                        return False, "preview returned no final joint waypoint", None
                    return True, f"status={status}", final_joint.copy()
                if status in failure_values:
                    return False, f"status={status}", None
            time.sleep(0.05)
        return False, "planner preview timeout", None

    def master_control(self, left_arm=None, right_arm=None, left_gripper=None, right_gripper=None, action="", object_category='cup'):
        """
        Master control function that intelligently controls arms and grippers
        Only moves/controls what has values, keeps unchanged what doesn't have values
        
        @param left_arm: [x, y, z, rx, ry, rz] for left arm position and rotation (mm, degrees), None to keep unchanged
        @param right_arm: [x, y, z, rx, ry, rz] for right arm position and rotation (mm, degrees), None to keep unchanged
        @param left_gripper: 0 for open, 1 for close, None to keep unchanged
        @param right_gripper: 0 for open, 1 for close, None to keep unchanged
        @return: 0 if successful, -1 if error occurs
        """
        # try:
        if not self.udp or not self.target_command:
            print("UDP communication not available for master control")
            return -1
        
        # Get current states of both arms and grippers in one call
        # current_state = self.get_current_state()
        # if current_state is None:
        #     print("Failed to get current state")
        #     return -1
        
        # Extract current arm states
        # left_arm_current = (np.array(current_state['left_arm']['position']), 
        #                     np.array(current_state['left_arm']['orientation']))
        # right_arm_current = (np.array(current_state['right_arm']['position']), 
        #                     np.array(current_state['right_arm']['orientation']))
        
        if not self._target_position_is_reasonable(left_arm, "left_arm"):
            return False
        if not self._target_position_is_reasonable(right_arm, "right_arm"):
            return False

        arm_motion_requested = left_arm is not None or right_arm is not None
        current_state = None
        if arm_motion_requested and (left_arm is None or right_arm is None):
            current_state = self.get_current_state()
            if current_state is None:
                print("Failed to get current state for unchanged arm")
                return False

        # A gripper-only update must preserve the entire arm request, including
        # its type and identity. After a joint reset, switching to CARTESIAN
        # would activate stale tool targets; changing the reset description
        # would also force the planner to plan the joint motion again.
        if arm_motion_requested:
            self.target_command.arm_command_type = TargetCommandArmCommandType.CARTESIAN
            self._set_arm_joint_command_placeholders()
            self.target_command.description = action
            self.prev_arm_tool_command = np.asarray(self.target_command.arm_tool_command).copy()
            self._cartesian_commanded_arms = tuple(
                arm_index
                for arm_index, arm_pose in ((0, left_arm), (1, right_arm))
                if arm_pose is not None
            )

        configured_gripper_size = self.target_command.gripper_size
        if configured_gripper_size == 0 and (
            left_gripper is not None or right_gripper is not None
        ):
            print("No gripper is configured; skipping gripper command for arm-only validation")
            left_gripper = None
            right_gripper = None

        # Process left arm
        if left_arm is not None:
            # Convert new position to quaternion
            left_pos, left_ori = self._convert_pos_rot_to_quaternion(left_arm)
            self._set_arm_tool_command(0, left_pos, left_ori)
            # print(left_pos, left_ori)
            # print(f"Left arm will move to: {left_arm}")
        elif current_state is not None:
            left_state = current_state["left_arm"]
            self._set_arm_tool_command(0, left_state["position"], left_state["orientation"])
            # print("Left arm position unchanged")
        # For a gripper-only command, preserve the previous arm target exactly.
        # Replacing it with delayed measured feedback creates a new Cartesian
        # arm target and makes the hand jump while the gripper closes.
        
        # Process right arm
        if right_arm is not None:
            # Convert new position to quaternion
            right_pos, right_ori = self._convert_pos_rot_to_quaternion(right_arm)
            self._set_arm_tool_command(1, right_pos, right_ori)
            # print(f"Right arm will move to: {right_arm}")
        elif current_state is not None:
            right_state = current_state["right_arm"]
            self._set_arm_tool_command(1, right_state["position"], right_state["orientation"])
            # print("Right arm position unchanged")
        # With neither arm requested, leave target_command.arm_tool_command
        # untouched so the planner receives only a gripper-state change.
        
        # Process left gripper
        if left_gripper is not None and configured_gripper_size >= 1:
            left_gripper_mode = self.get_cat_gripper('close', object_category) if left_gripper == 1 else self.get_cat_gripper('open', object_category)
            # print(f"Left gripper will {'close' if left_gripper == 1 else 'open'}")
            self.target_command.gripper_mode_command[0] = left_gripper_mode
        # else:
        #     left_gripper_mode = left_gripper_current
            # print("Left gripper state unchanged")
        
        # Process right gripper
        if right_gripper is not None and configured_gripper_size >= 2:
            right_gripper_mode = self.get_cat_gripper('close', object_category) if right_gripper == 1 else self.get_cat_gripper('open', object_category)
            self.target_command.gripper_mode_command[1] = right_gripper_mode
            # print(f"Right gripper will {'close' if right_gripper == 1 else 'open'}")
        # else:
        #     right_gripper_mode = right_gripper_current
            # print("Right gripper state unchanged")
        
        # Send the complete command
        # print(self.target_command)
        if self.ik_lib == 'curobo':
            self.udp.send(self.target_command)
        elif self.ik_lib == 'pinocchio':
            self._convert_target_command_to_teleop_command()
            self.udp.send(self.teleop_command)
        # print("Master control command sent successfully")
        
        # Wait for movement to complete if any arm moved
        if left_arm is not None or right_arm is not None:
            # Wait for the entire robot to stop moving
            position_tolerance_m = _position_tolerance_for_task(
                object_category, action
            )
            print(
                "Motion error diagnostic threshold: "
                f"object={object_category}, action={action}, "
                f"position={position_tolerance_m * 1000.0:.1f}mm"
            )
            result = self._wait_for_robot_stop(
                position_tolerance_m=position_tolerance_m
            )
            return result
          
        # time.sleep(10)  
        return True    
            
        # except Exception as e:
        #     print(f"Master control error: {e}")
        #     return -1

    def quick_move(self, target_dict):
        """
        Quick move function using a dictionary to specify what to move
        
        @param target_dict: Dictionary with keys:
            - 'left_arm': [x, y, z, rx, ry, rz] or None
            - 'right_arm': [x, y, z, rx, ry, rz] or None
            - 'left_gripper': 0/1 or None
            - 'right_gripper': 0/1 or None
        @return: 0 if successful, -1 if error occurs
        """
        left_arm = target_dict.get('left_arm')
        right_arm = target_dict.get('right_arm')
        left_gripper = target_dict.get('left_gripper')
        right_gripper = target_dict.get('right_gripper')
        
        return self.master_control(left_arm, right_arm, left_gripper, right_gripper) 
    
