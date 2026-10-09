from enum import Enum
import numpy as np
from scipy.spatial.transform import Rotation

from cuarm_state import PanelTargetMode, PanelState, RemotePanelCommand, RobotState, \
    TargetCommandPlannerMode, PlannerCuroboSolverType, RobotStatus, CuroboCommand, TargetCommand

def same_quaternion(q1_arr, q2_arr, atol=1e-3):
    """
    Check if two quaternion arrays are equivalent (considering quaternion double cover).
    
    Args:
        q1_arr: First quaternion array or list of quaternions.
        q2_arr: Second quaternion array or list of quaternions.
    
    Returns:
        bool: True if quaternions are equivalent (q ≡ -q), False otherwise.
    """
    if len(q1_arr) != len(q2_arr):
        return False
    for (q1, q2) in zip(q1_arr, q2_arr):
        if not (np.allclose(q1, q2, atol=atol) or np.allclose(q1, -q2, atol=atol)):
            return False
    return True

def same_array(a, b, atol=1e-3):
    if a is None or b is None: return False

    #For scalar
    if np.isscalar(a):
        if not np.isscalar(b):
            return False
        return np.allclose(a, b, atol=atol)
    
    #For Enum
    if isinstance(a, Enum):
        if not isinstance(b, Enum):
            return False
        return str(a) == str(b) #Avoid fail due to double definition 
    
    #For array
    if not isinstance(a, (list, tuple, np.ndarray)) or not isinstance(b, (list, tuple, np.ndarray)) or len(a) != len(b):
        return False
    for i in range(len(a)):
        if not same_array(a[i], b[i], atol):
            return False
    return True

def is_empty(x):
    """
    Check if a variable is empty.
    
    Args:
        x: Variable to check (list, numpy array, or any object).
    
    Returns:
        bool: True if the variable is empty, False otherwise.
    """
    if isinstance(x, list):
        return len(x) == 0
    elif isinstance(x, np.ndarray):
        return x.size == 0
    else:
        return x is None
    
def create_uneven_lists(shape: list, val=0.0):
    """
    Create a nested list of zeros where each level uses the next size in shape.
    Example: [2, 3] -> [[0, 0], [0, 0, 0]]
    """
    if not any(shape):
        return []
    
    out = []
    shape_copy = list(shape)
    while len(shape_copy):
        n = int(shape_copy.pop(0))
        out.append([val] * n)
    return out

def add_tool_offset(pos, quat, offset):
    """
    Add tool offset to a position considering the tool orientation.
    
    Args:
        pos: Base position as numpy array [x, y, z].
        quat: Tool orientation as quaternion numpy array [w, x, y, z].
        offset: Tool offset vector as numpy array [dx, dy, dz].
    
    Returns:
        np.ndarray: Transformed position with tool offset applied.
    """
    R = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix()
    return pos + R@offset

def remove_tool_offset(pos, quat, offset):
    """
    Remove tool offset from a position considering the tool orientation.
    
    Args:
        pos: Position with tool offset as numpy array [x, y, z].
        quat: Tool orientation as quaternion numpy array [w, x, y, z].
        offset: Tool offset vector as numpy array [dx, dy, dz].
    
    Returns:
        np.ndarray: Base position without tool offset.
    """
    R = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix()
    return pos - R@offset

def convert_curobo_command_to_remote_panel_command(remote_panel_command:RemotePanelCommand, curobo_command:CuroboCommand, target_command:TargetCommand, arm_target_mode:PanelTargetMode=None):
    if not curobo_command: return None
    
    #Update interpolation parameters
    if target_command:
        remote_panel_command.interpolation_acceleration_time = target_command.interpolation_acceleration_time
        remote_panel_command.no_interpolation_max_velocity_ratio = target_command.no_interpolation_max_velocity_ratio
        
    if arm_target_mode: remote_panel_command.arm_target_mode = arm_target_mode
    if remote_panel_command.arm_target_mode != PanelTargetMode.WAYPOINT:
        remote_panel_command.arm_joint_command = curobo_command.arm_joint_command
    else:
        remote_panel_command.arm_waypoint_size = curobo_command.waypoints_size
        remote_panel_command.arm_waypoint_dt = curobo_command.waypoints_dt
        remote_panel_command.arm_waypoint_command = curobo_command.waypoints
            
    #Gripper related
    remote_panel_command.gripper_joint_command = curobo_command.gripper_joint_command
    
def convert_panel_state_to_robot_state(robot_state:RobotState, panel_state:PanelState, remote_panel_command:RemotePanelCommand, 
    zero_vel_threshold=0.3, pos_threshold=1e-1):
    if not panel_state or not remote_panel_command: return None
    
    robot_state.arm_size = panel_state.arm_size
    robot_state.arm_joint_size = panel_state.arm_joint_size
    robot_state.arm_joint_position = panel_state.arm_joint_position
    robot_state.arm_joint_velocity = panel_state.arm_joint_velocity

    robot_state.gripper_size = panel_state.gripper_size
    robot_state.gripper_joint_size = panel_state.gripper_joint_size
    robot_state.gripper_joint_position = panel_state.gripper_joint_position
    
    target_reached = [False]*panel_state.arm_size
    for arm_i in range(panel_state.arm_size):
        zero_vel = same_array(panel_state.arm_joint_velocity[arm_i], np.zeros([panel_state.arm_joint_size[arm_i]]), atol=zero_vel_threshold)
        
        position_reached = True
        if remote_panel_command.arm_target_mode != PanelTargetMode.WAYPOINT:
            position_reached = same_array(panel_state.arm_interpolated_target[arm_i], \
                remote_panel_command.arm_joint_command[arm_i], atol=pos_threshold)
        else:
            position_reached = remote_panel_command.arm_waypoint_size <= 0 or \
                same_array(panel_state.arm_interpolated_target[arm_i], \
                remote_panel_command.arm_waypoint_command[-1][arm_i], atol=pos_threshold)
        target_reached[arm_i] = (zero_vel and position_reached)
    robot_state.status = RobotStatus.AVAILABLE if all(target_reached) else RobotStatus.BUSY
    
def get_arm_target_mode(solver_type: PlannerCuroboSolverType, planner_mode: TargetCommandPlannerMode):
    if solver_type == PlannerCuroboSolverType.IK_SOLVER and planner_mode == TargetCommandPlannerMode.PLANNING:
        raise ValueError("IK Solver don't support TargetCommandPlannerMode.PLANNING")
    
    if planner_mode == TargetCommandPlannerMode.PLANNING:
        arm_target_mode = PanelTargetMode.WAYPOINT
    elif planner_mode == TargetCommandPlannerMode.NO_PLANNING or planner_mode == TargetCommandPlannerMode.INTERPOLATION_ONLY:
        arm_target_mode = PanelTargetMode.SINGLE_POINT
    elif planner_mode == TargetCommandPlannerMode.RAW or planner_mode == TargetCommandPlannerMode.COLLISION_ONLY:
        arm_target_mode = PanelTargetMode.RAW_SINGLE_POINT
    return arm_target_mode
