
import sys
import time
import math
import signal
import json
from functools import partial
import threading
import traceback
from collections import deque
from typing import Callable, Any, Optional

import numpy as np
from udp_socket import UdpSocket
from cuarm_state import *

class CuarmUdp(UdpSocket):
    def __init__(self, local_ip, local_port, remote_ip, remote_port, buffer_size):
        super().__init__(local_ip, local_port, remote_ip, remote_port, buffer_size=buffer_size)
    
    def _get_data(self, deq:deque, cast_type, shape, is_dimension=True) -> deque:
        enum_class = None
        res = []
        if issubclass(cast_type, Enum):
            enum_class = cast_type
            cast_type = int
            
        if is_dimension:
            # shape is a dimension. e.g. [2, 3] = 2x3 array
            n = math.prod(shape)
            if n <= 0:
                deq.popleft()
                return []
            res = np.array([cast_type(deq.popleft()) for _ in range(n)]).reshape(shape).tolist()
        else:
            # shape is a list of sizes. e.g. [2, 3] = [[0, 0], [0, 0, 0]]
            n = sum(shape)
            if n <= 0:
                deq.popleft()
                return []
            for s in shape:
                res.append([cast_type(deq.popleft()) for _ in range(s)])
        if enum_class:
            res = self._wrap_enum_array(res, enum_class)
        return res
        
    def _wrap_enum_array(self, arr: Any, enum_class: type) -> Any:
        if isinstance(arr, list):
            return list(map(enum_class, arr))
        elif isinstance(arr, np.ndarray):
            return np.vectorize(enum_class)(arr)
        else:
            raise TypeError("Unsupported type for _wrap_enum_array")
        
    def _to_string(self, value: Any, sep='#') -> str:
        if isinstance(value, str):
            return value + sep
        
        # Handle enums explicitly
        if isinstance(value, Enum):
            return str(value.value) + sep
        
        if isinstance(value, bool):
            return str(int(value)) + sep
        
        if isinstance(value, (int, float)):
            return str(value) + sep
        
        # arrays (any dimension)
        if isinstance(value, (np.ndarray, list, tuple, set)):
            out = ""
            for x in value:
                out += self._to_string(x)
            if not out: #Empty array
                return sep
            return out

        # Fallback: just cast to string
        return str(value) + sep

    def pack_robot_state(self, data: RobotState) -> str:
        message = ""
        message += self._to_string(data.arm_size)
        message += self._to_string(data.arm_joint_size)
        message += self._to_string(data.arm_joint_position)
        message += self._to_string(data.arm_joint_velocity)
        message += self._to_string(data.gripper_size)
        message += self._to_string(data.gripper_joint_size)
        message += self._to_string(data.gripper_joint_position)
        message += self._to_string(data.status)
        return message
        
    def unpack_robot_state(self, raw: str) -> RobotState:
        data = None
        if raw:
            temp = RobotState()
            splited = deque(raw.split('#')[:-1])
            temp.arm_size           = int(splited.popleft())
            temp.arm_joint_size     = self._get_data(splited, int,   [temp.arm_size])
            temp.arm_joint_position = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.arm_joint_velocity = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.gripper_size           = int(splited.popleft())
            temp.gripper_joint_size     = self._get_data(splited, int,   [temp.gripper_size])
            temp.gripper_joint_position = self._get_data(splited, float, temp.gripper_joint_size, is_dimension=False)
            temp.status = RobotStatus(int(splited.popleft()))
            data = temp
        return data
        
    def pack_curobo_command(self, data: CuroboCommand):
        message = ""
        message += self._to_string(data.arm_size)
        message += self._to_string(data.arm_joint_size)
        message += self._to_string(data.arm_joint_command)
        message += self._to_string(data.gripper_size)
        message += self._to_string(data.gripper_joint_size)
        message += self._to_string(data.gripper_joint_command)
        message += self._to_string(data.cmd_plan_size)
        message += self._to_string(data.waypoints_size)
        message += self._to_string(data.waypoints_dt)
        message += self._to_string(data.waypoints)
        return message
        
    def unpack_curobo_command(self, raw: str) -> CuroboCommand:
        data = None
        if raw:
            temp = CuroboCommand()
            splited = deque(raw.split('#')[:-1])
            #Arm related
            temp.arm_size               = int(splited.popleft())
            temp.arm_joint_size         = self._get_data(splited, int,   [temp.arm_size])
            temp.arm_joint_command      = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.cmd_plan_size          = int(splited.popleft())
            
            temp.waypoints_size = int(splited.popleft())
            temp.waypoints_dt   = float(splited.popleft())
            temp.waypoints      = []
            for _ in range(temp.waypoints_size):
                temp.waypoints.append(self._get_data(splited, float, temp.arm_joint_size, is_dimension=False))
            
            #Gripper related
            temp.gripper_size           = int(splited.popleft())
            temp.gripper_joint_size     = self._get_data(splited, int,   [temp.gripper_size])
            temp.gripper_joint_command  = self._get_data(splited, float, temp.gripper_joint_size, is_dimension=False)
            
            data = temp
        return data
    
    def pack_target_command(self, data: TargetCommand):
        message = ""
        message += self._to_string(data.planner_mode)
        message += self._to_string(data.interpolation_acceleration_time)
        message += self._to_string(data.no_interpolation_max_velocity_ratio)
        
        #Arm related
        message += self._to_string(data.arm_command_type)
        message += self._to_string(data.arm_size)
        message += self._to_string(data.arm_joint_size)
        message += self._to_string(data.arm_joint_command)
        message += self._to_string(data.arm_tool_command)
        
        #Gripper related
        message += self._to_string(data.gripper_command_type)
        message += self._to_string(data.gripper_size)
        message += self._to_string(data.gripper_joint_size)
        message += self._to_string(data.gripper_mode_command)
        message += self._to_string(data.gripper_joint_command)
        message += data.description + '#'
        return message
    
    def pack_goalset_target_command(self, data: TargetCommand) -> str:
        """Pack TargetCommand in goalset format: Add candidate_count """
        tool_cmd = np.asarray(data.arm_tool_command)
        if tool_cmd.ndim == 3:
            tool_cmd = tool_cmd.reshape(-1, 7)
        if data.arm_size <= 0 or data.arm_size != tool_cmd.shape[0]:
            raise ValueError("pack_goalset_target_command: invalid arm_size or arm_tool_command shape")


        message = ""
        message += self._to_string(getattr(data, "candidate_count", 1))
        message += self._to_string(data.planner_mode)
        message += self._to_string(data.interpolation_acceleration_time)
        message += self._to_string(data.no_interpolation_max_velocity_ratio)

        #Arm related
        message += self._to_string(data.arm_command_type)
        message += self._to_string(data.arm_size)
        message += self._to_string(data.arm_joint_size)
        message += self._to_string(data.arm_joint_command)
        message += self._to_string(tool_cmd)

        #Gripper related
        message += self._to_string(data.gripper_command_type)
        message += self._to_string(data.gripper_size)
        message += self._to_string(data.gripper_joint_size)
        message += self._to_string(data.gripper_mode_command)
        message += self._to_string(data.gripper_joint_command)
        message += data.description + '#'
        return message

    def unpack_goalset_target_command(self, raw: str) -> TargetCommand:
        """Unpack goalset format. Add candidate_count."""
        data = None
        if raw:
            temp = TargetCommand()
            splited = deque(raw.split('#')[:-1])
            temp.candidate_count = int(splited.popleft())
            if temp.candidate_count < 1 or temp.candidate_count > 10:
                temp.candidate_count = 1
            temp.planner_mode = TargetCommandPlannerMode(int(splited.popleft()))
            temp.interpolation_acceleration_time = float(splited.popleft())
            temp.no_interpolation_max_velocity_ratio = float(splited.popleft())

            #Arm related
            temp.arm_command_type = TargetCommandArmCommandType(int(splited.popleft()))
            temp.arm_size = int(splited.popleft())
            temp.arm_joint_size = self._get_data(splited, int, [temp.arm_size])
            temp.arm_joint_command = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.arm_tool_command = self._get_data(splited, float, [temp.arm_size, 7])
            
            #Gripper related
            temp.gripper_command_type = TargetCommandGripperCommandType(int(splited.popleft()))
            temp.gripper_size = int(splited.popleft())
            temp.gripper_joint_size = self._get_data(splited, int, [temp.gripper_size])
            temp.gripper_mode_command = self._get_data(splited, GripperMode, [temp.gripper_size])
            temp.gripper_joint_command = self._get_data(splited, float, temp.gripper_joint_size, is_dimension=False)
            temp.description = splited.popleft()
            data = temp
        return data

    def unpack_target_command(self, raw: str) -> TargetCommand:
        data = None
        if raw:
            temp = TargetCommand()
            splited = deque(raw.split('#')[:-1])
            temp.planner_mode = TargetCommandPlannerMode(int(splited.popleft()))
            temp.interpolation_acceleration_time = float(splited.popleft())
            temp.no_interpolation_max_velocity_ratio = float(splited.popleft())
            
            #Arm related
            temp.arm_command_type = TargetCommandArmCommandType(int(splited.popleft()))
            temp.arm_size               = int(splited.popleft())
            temp.arm_joint_size         = self._get_data(splited, int,   [temp.arm_size])
            temp.arm_joint_command      = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.arm_tool_command = self._get_data(splited, float, [temp.arm_size, 7])
            
            #Gripper related
            temp.gripper_command_type = TargetCommandGripperCommandType(int(splited.popleft()))
            temp.gripper_size           = int(splited.popleft())
            temp.gripper_joint_size     = self._get_data(splited, int,   [temp.gripper_size])
            temp.gripper_mode_command   = self._get_data(splited, GripperMode,   [temp.gripper_size])
            temp.gripper_joint_command  = self._get_data(splited, float, temp.gripper_joint_size, is_dimension=False)
            
            temp.description = splited.popleft()
            data = temp
        return data
    
    def pack_curobo_state(self, data: CuroboState):
        message = ""
        
        #Arm 
        message += self._to_string(data.arm_size)
        message += self._to_string(data.arm_joint_size)
        message += self._to_string(data.arm_joint_position)
        message += self._to_string(data.arm_joint_velocity)
        message += self._to_string(data.arm_tool_pose)
        message += self._to_string(data.processed_arm_joint_command)
        message += self._to_string(data.processed_arm_tool_command)
        
        #Gripper
        message += self._to_string(data.gripper_size)
        message += self._to_string(data.gripper_joint_size)
        message += self._to_string(data.gripper_joint_position)
        message += self._to_string(data.processed_gripper_mode_command)
        message += self._to_string(data.processed_gripper_joint_command)
        
        #Status
        message += self._to_string(data.robot_status)
        message += self._to_string(data.curobo_status)
        return message
    
    def unpack_curobo_state(self, raw: str) -> CuroboState:
        data = None
        if raw:
            temp = CuroboState()
            splited = deque(raw.split('#')[:-1])
            #Arm related
            temp.arm_size           = int(splited.popleft())
            temp.arm_joint_size     = self._get_data(splited, int,   [temp.arm_size])
            temp.arm_joint_position = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.arm_joint_velocity = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.arm_tool_pose      = self._get_data(splited, float, [temp.arm_size, 7])
            temp.processed_arm_joint_command = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.processed_arm_tool_command  = self._get_data(splited, float, [temp.arm_size, 7])
            
            #Gripper related
            temp.gripper_size           = int(splited.popleft())
            temp.gripper_joint_size     = self._get_data(splited, int,   [temp.gripper_size])
            temp.gripper_joint_position = self._get_data(splited, float, temp.gripper_joint_size, is_dimension=False)
            temp.processed_gripper_mode_command     = self._get_data(splited, GripperMode,   [temp.gripper_size])
            temp.processed_gripper_joint_command    = self._get_data(splited, float, temp.gripper_joint_size, is_dimension=False)
            
            #Status
            temp.robot_status = RobotStatus(int(splited.popleft()))
            temp.curobo_status = CuroboStatus(int(splited.popleft()))
            data = temp
        return data

    def pack_workspace_spheres(self, data: WorkspaceSpheres) -> str:
        message = ""
        message += self._to_string(data.size)
        message += self._to_string(data.radius)
        message += self._to_string(data.center)
        message += self._to_string(data.meshgrid_offset_param)
        message += self._to_string(data.visualize_size)
        message += self._to_string(data.visualize_index)
        message += self._to_string(data.type)
        
        message += self._to_string(data.arm_size)
        message += self._to_string(data.arm_joint_size)
        message += self._to_string(data.arm_joint_command)
        return message
    
    def unpack_workspace_spheres(self, raw: str) -> WorkspaceSpheres:
        data = None
        if raw:
            temp = WorkspaceSpheres()
            splited = deque(raw.split('#')[:-1])
            temp.size                   = int(splited.popleft())
            temp.radius                 = float(splited.popleft())
            temp.center                 = self._get_data(splited, float, [3])
            temp.meshgrid_offset_param  = self._get_data(splited, int, [9])
            temp.visualize_size         = int(splited.popleft())
            temp.visualize_index        = self._get_data(splited, int, [temp.visualize_size])
            temp.type                   = self._get_data(splited, int, [temp.visualize_size])
            
            #Arm related
            temp.arm_size               = int(splited.popleft())
            temp.arm_joint_size         = self._get_data(splited, int, [temp.arm_size])
            temp.arm_joint_command      = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            data = temp
        return data
    
    def unpack_panel_state(self, raw: str) -> PanelState:
        data = None
        if raw:
            temp = PanelState()
            splited = deque(raw.split('#')[:-1])
            temp.state = PanelRemoteState(int(splited.popleft()))
            
            #Arm related
            temp.arm_size   = int(splited.popleft())
            temp.arm_joint_size = self._get_data(splited, int, [temp.arm_size])
            temp.arm_joint_position  = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.arm_joint_velocity  = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.arm_joint_torque  = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            temp.ee_pose        = self._get_data(splited, float, [temp.arm_size, 7])
            temp.tool_pose      = self._get_data(splited, float, [temp.arm_size, 7])
            temp.arm_interpolated_target = self._get_data(splited, float, temp.arm_joint_size, is_dimension=False)
            
            #Gripper related
            temp.gripper_size       = int(splited.popleft())
            temp.gripper_joint_size = self._get_data(splited, int, [temp.gripper_size])
            temp.gripper_joint_position  = self._get_data(splited, float, temp.gripper_joint_size, is_dimension=False)

            data = temp
        return data
        
    def pack_remote_panel_command(self, data: RemotePanelCommand) -> str:
        message = ""
        message += self._to_string(data.interpolation_acceleration_time)
        message += self._to_string(data.no_interpolation_max_velocity_ratio)
        message += self._to_string(data.target_type)
        message += self._to_string(data.actuator_mode)
        
        #Arm related
        message += self._to_string(data.arm_target_mode)
        message += self._to_string(data.arm_size)
        message += self._to_string(data.arm_joint_size)
        message += self._to_string(data.arm_joint_command)
        
        #Waypoint for arm
        message += self._to_string(data.arm_waypoint_size)
        message += self._to_string(data.arm_waypoint_dt)
        message += self._to_string(data.arm_waypoint_command)
        
        #Gripper related
        message += self._to_string(data.gripper_size)
        message += self._to_string(data.gripper_joint_size)
        message += self._to_string(data.gripper_joint_command)
        return message
        
    def pack_rt_panel_command(self, data: RtPanelCommand) -> str:
        message = ""
        message += self._to_string(data.send_timestamp)
        message += self._to_string(data.connection_state)

        # Settings
        message += self._to_string(data.need_setting_update)
        message += self._to_string(data.simulation)
        message += self._to_string(data.motion_type)
        message += self._to_string(data.task_orien_type)
        message += self._to_string(data.reset_control_mem)
        message += self._to_string(data.target_type)
        message += self._to_string(data.actuator_mode)
        message += self._to_string(data.interpolation_type)
        message += self._to_string(data.interpolation_acc_time)
        message += self._to_string(data.interpolation_const_vel_time)
        message += self._to_string(data.none_interpolation_saturation_ratio)
        message += self._to_string(data.reset_interpolation)
        message += self._to_string(data.control_algorithm)
        
        # Arm related
        message += self._to_string(data.arm_target_mode.value)
        message += self._to_string(data.arm_size)
        
        for arm_i in range(data.arm_size):
            message += self._to_string(data.joint_size[arm_i])
            message += self._to_string(data.robot_name[arm_i])
            message += self._to_string(data.enable_joint[arm_i])
            message += self._to_string(data.motor_cmd[arm_i])
            message += self._to_string(data.joint_cmd[arm_i])
            message += self._to_string(data.task_cmd[arm_i])

        # Arm waypoint mode
        message += self._to_string(data.arm_waypoint_size)
        message += self._to_string(data.arm_waypoint_dt)
        message += self._to_string(data.arm_waypoint_command)

        # Gripper related
        message += self._to_string(data.gripper_size)
        for gri_i in range(data.gripper_size):
            message += self._to_string(data.gripper_joint_size[gri_i])
            message += self._to_string(data.gripper_joint_command[gri_i])

        return message
    
    def unpack_rt_planner_state(self, raw: str) -> RtPlannerState:
        data = None
        if raw:
            temp = RtPlannerState()
            splited = deque(raw.split('#')[:-1])
            
            temp.received_panel_command_timestamp = int(splited.popleft())
            temp.send_timestamp = int(splited.popleft())
            temp.setting_update_finished = splited.popleft() == '1'

            # Arm State
            temp.arm_size = int(splited.popleft())
            for _ in range(temp.arm_size):
                temp.joint_size.append(int(splited.popleft()))
                temp.joint_pos.append(self._get_data(splited, float, [temp.joint_size[-1]]))
                temp.joint_vel.append(self._get_data(splited, float, [temp.joint_size[-1]]))
                temp.joint_tor.append(self._get_data(splited, float, [temp.joint_size[-1]]))
            
                # Assuming motor_size follows joint_size for these buffers
                temp.motor_pos.append(self._get_data(splited, float, [temp.joint_size[-1]]))
                temp.motor_vel.append(self._get_data(splited, float, [temp.joint_size[-1]]))
                temp.motor_cur.append(self._get_data(splited, float, [temp.joint_size[-1]]))
                temp.motor_tor.append(self._get_data(splited, float, [temp.joint_size[-1]]))

                temp.joint_pose.append(self._get_data(splited, float, [temp.joint_size[-1], 7]))
                temp.ee_pose.append(self._get_data(splited, float, [7]))
                temp.tool_pose.append(self._get_data(splited, float, [7]))
                temp.interpolated_target.append(self._get_data(splited, float, [temp.joint_size[-1]]))

            # Gripper related
            temp.gripper_size = int(splited.popleft())
            for _ in range(temp.gripper_size):
                temp.gripper_joint_size.append(int(splited.popleft()))
                temp.gripper_joint_pos.append(self._get_data(splited, float, [temp.gripper_joint_size[-1]]))

            data = temp
        return data

class CuarmUdpThread:
    def __init__(self, local_ip, local_port, remote_ip, remote_port, unpack_message_func: Optional[Callable[[CuarmUdp, str], Any]], pack_message_func: Optional[Callable[[CuarmUdp, Any], str]], recv_delay_ms=10, buffer_size=4096):
        self._terminate_thread = False
        self._closed = False
        self._udp = CuarmUdp(local_ip, local_port, remote_ip, remote_port, buffer_size)
        try:
            self._udp.open()
        except Exception:
            self._udp.close()
            self._closed = True
            raise
        
        self._unpack_message_func = unpack_message_func
        self._pack_message_func = pack_message_func
        
        self._recv_delay_ms = recv_delay_ms
        # self._data = self._unpack_message_func(self._udp, "") if self._unpack_message_func is not None else None
        self._data = None
        self._last_receive_time = None
        self._data_lock = threading.Lock()
        self._receive_thread = threading.Thread(target=self._receive_thread_task, args=(self._recv_delay_ms,))
        self._receive_thread.start()
        # signal.signal(signal.SIGINT, self._thread_signal_handler)
        
    def _thread_signal_handler(self, sig, frame):
        print('You pressed Ctrl+C!')
        self.terminate()
        sys.exit(0)
        
    def _receive_thread_task(self, delay_ms):
        if self._unpack_message_func is None: return
        while not self._terminate_thread:
            raw = self._udp.receive()
            if raw:
                try:
                    data = self._unpack_message_func(self._udp, raw)
                except Exception:
                    traceback.print_exc()
                    print("Catch receive exception. raw: ", raw)
                    data = None
                    
                with self._data_lock: 
                    self._data = data
                    self._last_receive_time = (
                        time.monotonic() if data is not None else None
                    )
            time.sleep(delay_ms/1000)
    
    def __del__(self):
        try:
            self.terminate()
        except Exception:
            pass
    
    def terminate(self):
        if not self._terminate_thread:
            self._terminate_thread = True
            receive_thread = getattr(self, "_receive_thread", None)
            if (
                receive_thread is not None
                and receive_thread.is_alive()
                and receive_thread is not threading.current_thread()
            ):
                receive_thread.join()
        if not self._closed:
            self._udp.close()
            self._closed = True
        
    def receive(self, max_age_s=None):
        if max_age_s is not None and max_age_s <= 0:
            raise ValueError("max_age_s must be positive")
        with self._data_lock:
            if (
                max_age_s is not None
                and self._last_receive_time is not None
                and time.monotonic() - self._last_receive_time > max_age_s
            ):
                return None
            return self._data

    def last_receive_age(self):
        with self._data_lock:
            if self._last_receive_time is None:
                return None
            return time.monotonic() - self._last_receive_time
    
    def send(self, data: Any):
        if self._closed or self._pack_message_func is None:
            return
        try:
            message = self._pack_message_func(self._udp, data)
        except Exception:
            traceback.print_exc()
            print("Catch send exception. data: ", data)
            message = ""
        self._udp.send(message)
    
    def clear(self):
        with self._data_lock: 
            self._data = None
            self._last_receive_time = None
