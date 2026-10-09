from cuarm_state import *
from cuarm_utils import create_uneven_lists

def _set_property(kwargs: dict, obj, property_name: str, default_val):
    """
    Assigns obj.property_name = kwargs.get(property_name, default_val).

    :param kwargs: Dictionary of keyword arguments.
    :param obj: The object whose property will be set.
    :param property_name: The property name as a string.
    :param default_val: Default value if property_name is not in kwargs.
    """
    setattr(obj, property_name, kwargs.get(str(property_name), default_val))
def get_initialized_target_command(**kwargs) -> TargetCommand: 
    out = TargetCommand()
    
    _set_property(kwargs, out, "planner_mode", TargetCommandPlannerMode.PLANNING)
    _set_property(kwargs, out, "interpolation_acceleration_time", 5.0)
    _set_property(kwargs, out, "no_interpolation_max_velocity_ratio", 0.1)
    _set_property(kwargs, out, "candidate_count", 1)
    
    #Arm related
    _set_property(kwargs, out, "arm_command_type", TargetCommandArmCommandType.CARTESIAN)
    _set_property(kwargs, out, "arm_size", 0)
    if out.arm_size > 0:
        _set_property(kwargs, out, "arm_joint_size", [0]*out.arm_size)
        _set_property(kwargs, out, "arm_joint_command", create_uneven_lists(out.arm_joint_size))
        _set_property(kwargs, out, "arm_tool_command", [[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] for _ in range(out.arm_size)])
    
    #Gripper related
    _set_property(kwargs, out, "gripper_command_type", TargetCommandGripperCommandType.MODE)
    _set_property(kwargs, out, "gripper_size", 0)
    if out.gripper_size > 0:
        _set_property(kwargs, out, "gripper_joint_size", [0]*out.gripper_size)
        _set_property(kwargs, out, "gripper_mode_command", [GripperMode.OPEN]*out.gripper_size)  #Gripper Mode Command
        _set_property(kwargs, out, "gripper_joint_command", create_uneven_lists(out.gripper_joint_size)) #Gripper Joint Command
    
    _set_property(kwargs, out, "description", "")
    return out
    

def get_initialized_remote_panel_command(**kwargs) -> RemotePanelCommand: 
    out = RemotePanelCommand()
    
    _set_property(kwargs, out, "target_type", ControlType.INVALID)
    _set_property(kwargs, out, "actuator_mode", ControlType.INVALID)
    _set_property(kwargs, out, "arm_target_mode", PanelTargetMode.SINGLE_POINT)
    
    # #Arm related
    _set_property(kwargs, out, "arm_size", 0)
    if out.arm_size > 0:
        _set_property(kwargs, out, "arm_joint_size", [0]*out.arm_size)
        _set_property(kwargs, out, "arm_joint_command", create_uneven_lists(out.arm_joint_size))
        
    # #Path related
    _set_property(kwargs, out, "arm_waypoint_size", 0)
    _set_property(kwargs, out, "arm_waypoint_dt", 0)
    if out.arm_waypoint_size > 0:
        _set_property(kwargs, out, "arm_waypoint_command", [create_uneven_lists(out.arm_joint_size) for _ in range(out.arm_waypoint_size)])
    
    # #Gripper related
    _set_property(kwargs, out, "gripper_size", 0)
    if out.gripper_size > 0:
        _set_property(kwargs, out, "gripper_joint_size", [0]*out.gripper_size)
        _set_property(kwargs, out, "gripper_joint_command", create_uneven_lists(out.gripper_joint_size))
    
    return out
    
def get_initialized_curobo_command(**kwargs) -> CuroboCommand: 
    out = CuroboCommand()
    
    #Arm related
    _set_property(kwargs, out, "arm_size", 0)
    if out.arm_size > 0:
        _set_property(kwargs, out, "arm_joint_size", [0]*out.arm_size)
        _set_property(kwargs, out, "arm_joint_command", create_uneven_lists(out.arm_joint_size))
        
    #Gripper related
    _set_property(kwargs, out, "gripper_size", 0)
    if out.gripper_size > 0:
        _set_property(kwargs, out, "gripper_joint_size", [0]*out.gripper_size)
        _set_property(kwargs, out, "gripper_joint_command", create_uneven_lists(out.gripper_joint_size))
    
    return out
def get_initialized_curobo_state(**kwargs) -> CuroboState: 
    out = CuroboState()
    
    #Arm related
    _set_property(kwargs, out, "arm_size", 0)
    if out.arm_size > 0:
        _set_property(kwargs, out, "arm_joint_size", [0]*out.arm_size)
        _set_property(kwargs, out, "arm_joint_position", create_uneven_lists(out.arm_joint_size, val=-999))
        _set_property(kwargs, out, "arm_joint_velocity", create_uneven_lists(out.arm_joint_size, val=-999))
        _set_property(kwargs, out, "arm_tool_pose", [[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] for _ in range(out.arm_size)])
        _set_property(kwargs, out, "processed_arm_joint_command", create_uneven_lists(out.arm_joint_size, val=-999))
        _set_property(kwargs, out, "processed_arm_tool_command", [[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] for _ in range(out.arm_size)])
        
    #Gripper related
    _set_property(kwargs, out, "gripper_size", 0)
    if out.gripper_size > 0:
        _set_property(kwargs, out, "gripper_joint_size", [0]*out.gripper_size)
        _set_property(kwargs, out, "gripper_joint_position", create_uneven_lists(out.gripper_joint_size, val=-999))
        _set_property(kwargs, out, "processed_gripper_mode_command", [GripperMode.OPEN]*out.gripper_size)           
        _set_property(kwargs, out, "processed_gripper_joint_command", create_uneven_lists(out.gripper_joint_size, val=-999))
    
    return out
