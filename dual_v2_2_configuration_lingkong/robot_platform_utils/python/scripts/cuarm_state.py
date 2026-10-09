from dataclasses import dataclass, field
from typing import List
import numpy as np
from enum import Enum
from typing import Union

class RobotStatus(Enum):
    AVAILABLE = 0
    BUSY = 1

class CuroboStatus(Enum):
    UNINITIALIZED = 0
    INITIALIZED = 1
    PLANNING = 2
    PLANNING_FAILED = 3
    PLANNING_SUCCEEDED = 4
    SOLVING_IK = 5
    IK_SUCCEEDED = 6
    IK_FAILED = 7
    COLLISION_CHECK_FAILED = 8
    MOTION_FINISHED = 9

class GripperMode(Enum):
    OPEN = 0
    CLOSE = 1
    VISION_PRO_BUTTON_A_TRUE = 2
    VISION_PRO_BUTTON_A_FALSE = 3
    VLM_CLOSE = 4
    WINEGLASS_OPEN = 5
    WINEGLASS_CLOSE = 6
    DUSTPAN_OPEN = 7
    DUSTPAN_CLOSE = 8

class PlannerCuroboSolverType(Enum):
    MOTION_GEN = 0  # Support all TargetCommandPlannerMode
    IK_SOLVER = 1   # Support INTERPOLATION_ONLY and RAW only. Inverse Kinematics can reach 120Hz on A6000 while MotionGen max at 60Hz.

class PlannerConnectionMethod(Enum):
    """
    Enumeration of different communication use cases for the CuroboPlanner.
    
    Defines how the planner communicates with the robot and target systems.
    
    Attributes:
        BOTH_LOCAL: Both robot state and target commands via local API
            robot_state(API) -> [Curobo] <- target_pose(API)
            joint_command(API) <- [Curobo] -> curobo_state(API)
            
        BOTH_REMOTE: Both robot state and target commands via UDP
            robot_state(UDP) -> [Curobo] <- target_command(UDP)
            joint_command(UDP) <- [Curobo] -> curobo_state(UDP)
            
        LOCAL_TARGET_AND_REMOTE_ROBOT: Target via API, robot via UDP
            robot_state(UDP) -> [Curobo] <- target_pose(API)
            joint_command(UDP) <- [Curobo] -> curobo_state(API)
            
        REMOTE_TARGET_AND_LOCAL_ROBOT: Target via UDP, robot via API
            robot_state(API) -> [Curobo] <- target_command(UDP)
            joint_command(API) <- [Curobo] -> curobo_state(UDP)
            
    """
    BOTH_LOCAL = 0
    BOTH_REMOTE = 1
    LOCAL_TARGET_AND_REMOTE_ROBOT = 2
    REMOTE_TARGET_AND_LOCAL_ROBOT = 3
    
class PlannerInputStateType(Enum):
    """
    Enumeration of input state types for motion generation.
    
    Defines what type of state information to retrieve for planning.
    
    Attributes:
        JOINT_NAME: Retrieve joint names as strings
        JOINT_POSITION: Retrieve current joint positions
        JOINT_VELOCITY: Retrieve current joint velocities
        TARGET_JOINT_COMMAND: Retrieve target joint commands
    """
    JOINT_NAME = 0
    JOINT_POSITION = 1
    JOINT_VELOCITY = 2
    TARGET_JOINT_COMMAND = 3
    LAST_WAYPOINT_POSITION = 4
    LAST_LIMITED_WAYPOINT_POSITION = 5
    
class TargetCommandArmCommandType(Enum):
    CARTESIAN = 0
    JOINT = 1
    
class TargetCommandGripperCommandType(Enum):
    MODE = 0
    JOINT = 1

class TargetCommandPlannerMode(Enum):     
    PLANNING = 0                    # planning with collision check and interpolation
    NO_PLANNING = 1                 # no planning, but collision check + interpolation.
    COLLISION_ONLY = 2              # no planning, no interpolation
    INTERPOLATION_ONLY = 3          # no planning, no collision check
    RAW = 4                         # no planning, no collision check, no interpolation

class PanelRemoteState(Enum):
    STOP = 0    # stop button pressed on panel
    START = 1   # start button pressed on panel
    
class PanelTargetMode(Enum):
    """
    Attributes:
        WAYPOINT        : Send a list of points, and go to each point with interpolation. Use interpolation_acceleration_time
        SINGLE_POINT    : Send a point, and go to the point with interpolation. Use interpolation_acceleration_time
        RAW_SINGLE_POINT: Send a point, and go to point directly without interpolation. Use no_interpolation_max_velocity_ratio
    """
    SINGLE_POINT = 0
    WAYPOINT = 1
    RAW_SINGLE_POINT = 2    
    
class ControlType(Enum):
    INVALID  = -1
    POSITION = 0
    VELOCITY = 1
    TORQUE   = 2

INSPIRE_DEXTEROUS_HAND_COMMAND = {
    # GripperMode.OPEN: [0.0, 1.0, 1.0, 1.0, 1.0, 1.0],  # J1: thumb lateral rotation, J2 - J6: bending
    # GripperMode.CLOSE: [0.0, 0.3, 0.3, 0.3, 0.7, 0.7], # J1: thumb lateral rotation, J2 - J6: bending
    # GripperMode.VISION_PRO_BUTTON_A_TRUE: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    # GripperMode.VISION_PRO_BUTTON_A_FALSE: [0.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    # GripperMode.VLM_CLOSE: [0.0, 0.3, 0.3, 0.3, 0.9, 0.9],
    # GripperMode.WINEGLASS_OPEN: [0.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    # GripperMode.WINEGLASS_CLOSE: [0.0, 0.3, 0.3, 0.3, 0.3, 0.3],
    # GripperMode.DUSTPAN_OPEN: [0.3, 1.0, 1.0, 1.0, 1.0, 1.0],
    # GripperMode.DUSTPAN_CLOSE: [0.3, 0.0, 0.0, 0.0, 0.3, 0.3],
    GripperMode.OPEN: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # J1: thumb lateral rotation, J2 - J6: bending
    GripperMode.CLOSE: [1.0, 0.7, 0.7, 0.7, 0.3, 0.3], # J1: thumb lateral rotation, J2 - J6: bending
    GripperMode.VISION_PRO_BUTTON_A_TRUE: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    GripperMode.VISION_PRO_BUTTON_A_FALSE: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    GripperMode.VLM_CLOSE: [1.0, 0.7, 0.7, 0.7, 0.1, 0.1],
    GripperMode.WINEGLASS_OPEN: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    GripperMode.WINEGLASS_CLOSE: [1.0, 0.7, 0.7, 0.7, 0.7, 0.7],
    GripperMode.DUSTPAN_OPEN: [0.7, 0.0, 0.0, 0.0, 0.0, 0.0],
    GripperMode.DUSTPAN_CLOSE: [0.7, 1.0, 1.0, 1.0, 0.7, 0.7],
}

TWO_FINGER_GRIPPER_COMMAND = {
    # GripperMode.OPEN: [1.0],  
    # GripperMode.CLOSE: [0.0],
    # GripperMode.VISION_PRO_BUTTON_A_TRUE: [0.0],
    # GripperMode.VISION_PRO_BUTTON_A_FALSE: [1.0],
    # GripperMode.VLM_CLOSE: [0.0],
    GripperMode.OPEN: [0.0],  
    GripperMode.CLOSE: [1.0],
    GripperMode.VISION_PRO_BUTTON_A_TRUE: [1.0],
    GripperMode.VISION_PRO_BUTTON_A_FALSE: [0.0],
    GripperMode.VLM_CLOSE: [1.0],
}

@dataclass
class CuroboCommand:
    #Arm
    arm_size: int = field(default=0)
    arm_joint_size: int = field(default=0)                                                  #1D array: [arm_size]
    arm_joint_command: Union[list[float], np.ndarray] = field(default_factory=list)         #2D array: [arm_size, arm_joint_size], unit: radian
    cmd_plan_size: int = field(default=0)                                                   #Total planned arm_joint_command count

    #Waypoints of arm
    waypoints_size: int = field(default=0)
    waypoints_dt: float = field(default=0.0)
    waypoints: Union[list[float], np.ndarray] = field(default_factory=list)                 #3D array: [waypoints_size, arm_size, arm_joint_size], unit: radian
    
    #Gripper
    gripper_size : int = field(default=0)
    gripper_joint_size: int = field(default=0)                                              #1D array: [gripper_size]
    gripper_joint_command: Union[list[float], np.ndarray] = field(default_factory=list)     #2D array: [gripper_size, gripper_joint_size], range: [0.0, 1.0]
    

@dataclass
class RobotState:
    #Arm
    arm_size: int = field(default=0)
    arm_joint_size: int = field(default=0)                                                  #1D array: [arm_size]
    arm_joint_position: Union[list[float], np.ndarray] = field(default_factory=list)        #2D array: [arm_size, arm_joint_size], unit: radian
    arm_joint_velocity: Union[list[float], np.ndarray] = field(default_factory=list)        #2D array: [arm_size, arm_joint_size], unit: radian/second
    
    #Gripper
    gripper_size : int = field(default=0)
    gripper_joint_size: int = field(default=0)                                              #1D array: [gripper_size]
    gripper_joint_position: Union[list[float], np.ndarray] = field(default_factory=list)    #2D array: [gripper_size, gripper_joint_size], range: [0.0, 1.0]

    status: RobotStatus = field(default=RobotStatus.AVAILABLE)

@dataclass
class TargetCommand:
    planner_mode: TargetCommandPlannerMode = TargetCommandPlannerMode.PLANNING
    interpolation_acceleration_time: float = 5.0
    no_interpolation_max_velocity_ratio: float = 0.1
    candidate_count: int = 1  # Goalset: 候选目标数. 普通单目标=1. arm_size = candidate_count × target_link_size
    
    #Arm
    arm_command_type: TargetCommandArmCommandType = TargetCommandArmCommandType.CARTESIAN
    arm_size: int = field(default=0)
    arm_joint_size: int = field(default=0)                                                  #1D array: [arm_size]
    arm_joint_command: Union[list[float], np.ndarray] = field(default_factory=list)         #2D array: [arm_size, arm_joint_size], unit: radian
    arm_tool_command: Union[list[float], np.ndarray] = field(default_factory=list)          #2D array: [arm_size, 7], order in [x, y, z, qw, qx, qy, qz], unit: m and quaternion

    #Gripper
    gripper_command_type: TargetCommandGripperCommandType = TargetCommandGripperCommandType.MODE
    gripper_size: int = field(default=0)
    gripper_joint_size: Union[list[float], np.ndarray] = field(default_factory=list) 
    gripper_mode_command: Union[list[GripperMode], np.ndarray] = field(default_factory=list)    #1D array: [gripper_size], use GripperMode Enum
    gripper_joint_command: Union[list[float], np.ndarray] = field(default_factory=list)         #2D array: [gripper_size, gripper_joint_size], range: [0.0, 1.0]
    
    description: str= field(default="")

@dataclass
class CuroboState:
    arm_size: int = field(default=0)
    arm_joint_size: int = field(default=0)                                              #1D array: [arm_size]
    arm_joint_position: Union[list[float], np.ndarray] = field(default_factory=list)    #2D array: [arm_size, arm_joint_size], unit: radian
    arm_joint_velocity: Union[list[float], np.ndarray] = field(default_factory=list)    #2D array: [arm_size, arm_joint_size], unit: radian/second
    arm_tool_pose: Union[list[float], np.ndarray] = field(default_factory=list)         #2D array: [arm_size, 7], order in [x, y, z, qw, qx, qy, qz], unit: m and quaternion
    processed_arm_joint_command: Union[list[float], np.ndarray] = field(default_factory=list)   #2D array: [arm_size, arm_joint_size], unit: radian
    processed_arm_tool_command: Union[list[float], np.ndarray] = field(default_factory=list)    #2D array: [arm_size, 7], order in [x, y, z, qw, qx, qy, qz], unit: m and quaternion
        
    #Gripper
    gripper_size: int = field(default=0)
    gripper_joint_size: int = field(default=0)                                                          #1D array: [gripper_size]
    gripper_joint_position: Union[list[float], np.ndarray] = field(default_factory=list)                #2D array: [gripper_size, gripper_joint_size], range: [0.0, 1.0]
    processed_gripper_mode_command: Union[list[GripperMode], np.ndarray] = field(default_factory=list)        #1D array: [gripper_size], use GripperMode Enum
    processed_gripper_joint_command: Union[list[float], np.ndarray] = field(default_factory=list)       #2D array: [gripper_size, gripper_joint_size], range: [0.0, 1.0]
        
    robot_status: RobotStatus = field(default=RobotStatus.AVAILABLE)
    curobo_status: CuroboStatus = field(default=CuroboStatus.UNINITIALIZED)

@dataclass
class WorkspaceSpheres:
    size: int = field(default=0)
    radius: float = field(default=0.02)
    center: Union[list[float], np.ndarray] = field(default_factory=list)
    meshgrid_offset_param: List[int] = field(default_factory=lambda: [0]*9)             #n_x, n_y, n_z, min_x, min_y, min_z, max_x, max_y, max_z
    visualize_size: int = field(default=-1)                                             # -1 = all
    visualize_index: Union[list[float], np.ndarray] = field(default_factory=list)
    type: Union[list[float], np.ndarray] = field(default_factory=list)                  #0: unreachable
    
    #for visual the orientation
    arm_size: int = field(default=0)
    arm_joint_size: int = field(default=0)                                              #1D array: [arm_size]
    arm_joint_command: Union[list[float], np.ndarray] = field(default_factory=list)     #2D array: [arm_size, arm_joint_size], unit: radian
    
@dataclass
class PanelState:
    state: PanelRemoteState = PanelRemoteState.STOP
    
    #Arm related
    arm_size:   int = field(default=0)
    arm_joint_size: Union[list[float], np.ndarray] = field(default_factory=list)                #1D array: [arm_size]
    arm_joint_position: Union[list[float], np.ndarray] = field(default_factory=list)            #2D array: [arm_size, arm_joint_size], unit: radian
    arm_joint_velocity: Union[list[float], np.ndarray] = field(default_factory=list)            #2D array: [arm_size, arm_joint_size], unit: radian/second
    arm_joint_torque: Union[list[float], np.ndarray] = field(default_factory=list)              #2D array: [arm_size, arm_joint_size]
    ee_pose: Union[list[float], np.ndarray] = field(default_factory=list)                       #2D array: [arm_size, 7], order in [x, y, z, qw, qx, qy, qz], unit: m and quaternion
    tool_pose: Union[list[float], np.ndarray] = field(default_factory=list)                     #2D array: [arm_size, 7], order in [x, y, z, qw, qx, qy, qz], unit: m and quaternion
    arm_interpolated_target: Union[list[float], np.ndarray] = field(default_factory=list)       #2D array: [arm_size, arm_joint_size], unit: radian
    
    #Gripper related
    gripper_size:       int = field(default=0)
    gripper_joint_size: Union[list[float], np.ndarray] = field(default_factory=list)            #1D array: [gripper_size]
    gripper_joint_position:  Union[list[float], np.ndarray] = field(default_factory=list)       #2D array: [gripper_size, gripper_joint_size], range: [0.0, 1.0]
    
@dataclass
class RemotePanelCommand:
    interpolation_acceleration_time: float = 5.0
    no_interpolation_max_velocity_ratio: float = 0.1
    target_type:    ControlType = ControlType.INVALID                                           #Arm command type (position/velocity/torque)
    actuator_mode:   ControlType = ControlType.INVALID                                           #Underlying motor command type (position/velocity/torque)
    arm_target_mode:    PanelTargetMode = PanelTargetMode.SINGLE_POINT
    
    #Arm related
    arm_size:   int = field(default=0)
    arm_joint_size: Union[list[float], np.ndarray] = field(default_factory=list)                #1D array: [arm_size]
    arm_joint_command:  Union[list[float], np.ndarray] = field(default_factory=list)            #2D array: [arm_size, arm_joint_size], unit: radian
    
    #Path related
    arm_waypoint_size:  int = field(default=0)
    arm_waypoint_dt:    float = field(default=0.0)
    arm_waypoint_command:   Union[list[float], np.ndarray] = field(default_factory=list)        #3D array: [arm_waypoint_size, arm_size, arm_joint_size], unit: radian

    #Gripper related
    gripper_size:       int = field(default=0)
    gripper_joint_size: Union[list[float], np.ndarray] = field(default_factory=list)
    gripper_joint_command:  Union[list[float], np.ndarray] = field(default_factory=list)        #2D array: [gripper_size, gripper_joint_size], range: [0.0, 1.0]

#For alternative rt_control panel
class RtMotionControl(Enum):
    JOINT = 0
    NULL = 1
    TASK = 2
    MOTOR = 3

class RtOrientControl(Enum):
    END = 0
    BASE = 1

class RtInterpolationMethod(Enum):
    LINEAR = 0
    COS = 1
    CUBIC = 2
    QUINTIC = 3
    NONE = 4
    QUINTIC_PATH = 5

class RtControlAlgorithm(Enum):
    NONE = 0
    GRAVITY = 1
    GRAVITY_WITH_BOUNDARY = 2
    TASK_ADMITTANCE = 3
    JOINT_ADMITTANCE = 4
    TASK_IMPEDANCE = 5
    JOINT_IMPEDANCE = 6
    NULL_ADMITTANCE = 7

class RtConnectionState(Enum):
    WAITING = 0
    REMOTE = 1
    SHUT_DOWN = 2
    
@dataclass
class RtPanelCommand:
    send_timestamp: int = 0
    connection_state: RtConnectionState = RtConnectionState.WAITING

    # Settings
    need_setting_update: bool = False
    simulation: bool = False
    motion_type: RtMotionControl = RtMotionControl.JOINT
    task_orien_type: RtOrientControl = RtOrientControl.END
    reset_control_mem: bool = False
    target_type: ControlType = ControlType.INVALID
    actuator_mode: ControlType = ControlType.INVALID
    interpolation_type: RtInterpolationMethod = RtInterpolationMethod.COS
    interpolation_acc_time: float = 0.0
    interpolation_const_vel_time: float = 0.0
    none_interpolation_saturation_ratio: float = 0.0
    reset_interpolation: bool = False
    control_algorithm: RtControlAlgorithm = RtControlAlgorithm.NONE

    # Arm related
    arm_target_mode: PanelTargetMode = PanelTargetMode.SINGLE_POINT
    arm_size: int = 0
    joint_size: Union[list[int], np.ndarray] = field(default_factory=list)      # 1D array: [arm_size]
    robot_name: Union[list[str], np.ndarray] = field(default_factory=list)      # List of strings
    enable_joint: Union[list[bool], np.ndarray] = field(default_factory=list)    # 2D array: [arm_size, joint_size]
    
    # Arm single point mode
    motor_cmd: Union[list[float], np.ndarray] = field(default_factory=list)     # 2D array
    joint_cmd: Union[list[float], np.ndarray] = field(default_factory=list)     # 2D array
    task_cmd: Union[list[float], np.ndarray] = field(default_factory=list)      # 2D array: [arm_size, 6]

    # Arm waypoint mode
    arm_waypoint_size: int = 0
    arm_waypoint_dt: float = 0.0
    arm_waypoint_command: Union[list[float], np.ndarray] = field(default_factory=list)     # 3D array

    # Gripper related
    gripper_size: int = 0
    gripper_joint_size: Union[list[int], np.ndarray] = field(default_factory=list)
    gripper_joint_command: Union[list[float], np.ndarray] = field(default_factory=list)

    
@dataclass
class RtPlannerState:
    send_timestamp: int = 0
    received_panel_command_timestamp: int = 0
    setting_update_finished: bool = False

    # Arm State
    arm_size: int = 0
    joint_size: Union[list[int], np.ndarray] = field(default_factory=list)      # 1D array: [arm_size]
    joint_pos: Union[list[float], np.ndarray] = field(default_factory=list)     # 2D array: [arm_size, joint_size]
    joint_vel: Union[list[float], np.ndarray] = field(default_factory=list)
    joint_tor: Union[list[float], np.ndarray] = field(default_factory=list)
    
    motor_pos: Union[list[float], np.ndarray] = field(default_factory=list)     # 2D array: [arm_size, motor_size]
    motor_vel: Union[list[float], np.ndarray] = field(default_factory=list)
    motor_cur: Union[list[float], np.ndarray] = field(default_factory=list)
    motor_tor: Union[list[float], np.ndarray] = field(default_factory=list)
    
    joint_pose: Union[list[float], np.ndarray] = field(default_factory=list)    # 3D array: [arm_size, joint_size, 7]
    ee_pose: Union[list[float], np.ndarray] = field(default_factory=list)       # 2D array: [arm_size, 7]
    tool_pose: Union[list[float], np.ndarray] = field(default_factory=list)     # 2D array: [arm_size, 7]

    interpolated_target: Union[list[float], np.ndarray] = field(default_factory=list)

    # Gripper related
    gripper_size: int = 0
    gripper_joint_size: Union[list[int], np.ndarray] = field(default_factory=list)
    gripper_joint_pos: Union[list[float], np.ndarray] = field(default_factory=list)