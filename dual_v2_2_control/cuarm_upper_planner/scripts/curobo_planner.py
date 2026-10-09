import os
import signal
from functools import partial
from enum import Enum
from typing import List, Tuple
import time
import threading
import copy
from typing import Union


#3rd Party
import torch
import numpy as np
from scipy.spatial.transform import Rotation as RotationR

#Curobo
from curobo.geom.sdf.world import CollisionCheckerType, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig, MotionGenResult, PoseCostMetric
from curobo.wrap.reacher.ik_solver import IKSolver, IKResult
from curobo.util_file import load_yaml
from curobo.util.trajectory import InterpolateType
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.evaluator import TrajEvaluatorConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig

#Cuarm configuration
from cuarm_udp import CuarmUdp, CuarmUdpThread
from cuarm_state import CuroboCommand, RobotState, TargetCommand, \
    GripperMode, INSPIRE_DEXTEROUS_HAND_COMMAND, TWO_FINGER_GRIPPER_COMMAND, CuroboState, \
    TargetCommandArmCommandType, RobotStatus, TargetCommandPlannerMode, \
    CuroboStatus, TargetCommandGripperCommandType, PlannerConnectionMethod, PlannerCuroboSolverType, PlannerInputStateType
from cuarm_state_initializers import get_initialized_curobo_state
from cuarm_utils import same_quaternion, same_array, add_tool_offset, remove_tool_offset
    

from curobo_config import CuroboConfig
from curobo_ik_solver import create_ik_solver

class CuroboPlanner(CuroboConfig):
    def __init__(self, config: CuroboConfig, connection_method: PlannerConnectionMethod, solver_type:PlannerCuroboSolverType=PlannerCuroboSolverType.MOTION_GEN, catch_SIGINT=True, **kwargs):
        super().__init__(**vars(config))
        
        #Catch Ctrl+C
        if (catch_SIGINT):
            signal.signal(signal.SIGINT, self._sigint_signal_handler)
        
        self.target_cartesian_position_precision = kwargs.get("target_cartesian_position_precision", 1e-3)
        self.target_cartesian_orientation_precision = kwargs.get("target_cartesian_orientation_precision", 1e-3)
        self.target_joint_position_precision = kwargs.get("target_joint_position_precision", 1e-3)
        self.soft_limit_k = kwargs.get("soft_limit_k", 0.95)
        
        #gripper related
        self.gripper_input_urdf_order  = kwargs.get("gripper_input_urdf_order", True)
        self.gripper_input_normalized  = kwargs.get("gripper_input_normalized", True)
        self.gripper_input_zero_open   = kwargs.get("gripper_input_zero_open", True)
        self.gripper_output_urdf_order = kwargs.get("gripper_output_urdf_order", True)
        self.gripper_output_normalized = kwargs.get("gripper_output_normalized", True)
        self.gripper_output_zero_open  = kwargs.get("gripper_output_zero_open", True)
        self.curobo_used_gripper_type       = TargetCommandGripperCommandType.JOINT
        self.curobo_used_gripper_urdf_order = True
        self.curobo_used_gripper_normalized = False
        self.curobo_used_gripper_zero_open  = True
        self.robot_sent_gripper_type        = TargetCommandGripperCommandType.JOINT
        self.robot_sent_gripper_urdf_order  = self.gripper_output_urdf_order
        self.robot_sent_gripper_normalized  = self.gripper_output_normalized
        self.robot_sent_gripper_zero_open   = self.gripper_output_zero_open
        
        #kwargs for MotionGen
        interpolation_dt = kwargs.get("interpolation_dt", 0.02)
        trajopt_tsteps = kwargs.get("trajopt_tsteps", 16)
        assert trajopt_tsteps > 5, "trajopt_tsteps must be larger than 5."
        minimum_trajectory_dt = kwargs.get("minimum_trajectory_dt", trajopt_tsteps * interpolation_dt)
        self.trim_steps = kwargs.get("trim_steps", [0, trajopt_tsteps])
        warmup_js_trajopt = kwargs.get("warmup_js_trajopt", False)
        time_dilation_factor = kwargs.get("time_dilation_factor", 1.0)
        solver_position_threshold = kwargs.get("solver_position_threshold", 0.005)
        solver_rotation_threshold = kwargs.get("solver_rotation_threshold", 0.05)
        solver_cspace_threshold = kwargs.get("solver_cspace_threshold", 0.05)
        maximum_trajectory_time = 7.0
        ik_opt_iters = kwargs.get("ik_opt_iters", 200)
        self_collision_check = kwargs.get("self_collision_check", True)
        num_ik_seeds = kwargs.get("num_ik_seeds", 12)
        self_collision_opt = kwargs.get("self_collision_opt", True)
        ik_particle_opt = kwargs.get("ik_particle_opt", True)
        use_cuda_graph = kwargs.get("use_cuda_graph", False)
        num_trajopt_seeds = kwargs.get("num_trajopt_seeds", 12)
        
        self._terminate_flag = False
        self._connection_method = connection_method
        self._solver_type = solver_type
        self._robot_udp = None
        self._target_udp = None
        self.status = CuroboStatus.UNINITIALIZED
        self._curobo_planning_status_planning_count = 999
        self._curobo_planning_status_planning_threshold = 3
        self._first_robot_state_received = False
        self._first_target_command_received = False
        self._gpu_lock = threading.Lock()
        self._use_cuda_graph = use_cuda_graph
        
        #Received target command
        self.last_received_target_command = None
        self.planner_mode = TargetCommandPlannerMode.RAW
        self.target_arm_type = TargetCommandArmCommandType.JOINT
        self.target_arm_tool_position = np.array([])       #[number of arm size x 3] shape
        self.target_arm_tool_orientation = np.array([])    #[number of arm size x 4] shape, quaternion in scalar-first (w, x, y, z)
        self.target_arm_joint = np.array([-1]*self.total_arm_joint_size)
        self.target_gripper_type = TargetCommandGripperCommandType.MODE
        self.target_gripper_mode = [GripperMode.OPEN]*self.gripper_size
        self.target_gripper_joint_raw = [-1]*self.total_gripper_joint_size
        self.target_gripper_joint = [-1]*self.total_gripper_joint_size
        self.target_description = ""
        
        #Received robot state
        self.last_received_robot_state = None
        self.arm_joint_position = np.array([0.0 for _ in range(self.total_arm_joint_size)])
        self.arm_joint_velocity = np.array([0.0 for _ in range(self.total_arm_joint_size)])
        self.gripper_joint_position = np.array([0.0 for _ in range(self.total_gripper_joint_size)])
        self.arm_tool_position = np.zeros((self.arm_size, 3))
        self.arm_tool_orientation = np.array([[1.0, 0.0, 0.0, 0.0]]*self.arm_size)
        self.robot_status = RobotStatus.AVAILABLE

        self._prev_target_tool_position = np.zeros((self.arm_size, 3))
        self._prev_target_tool_orientation = np.array([[1.0, 0.0, 0.0, 0.0]]*self.arm_size)
        self._prev_target_joint = np.array([-999 for _ in range(self.total_arm_joint_size+self.total_gripper_joint_size)])
        self._prev_target_arm_joint = np.array([-999 for _ in range(self.total_arm_joint_size)])
        self._prev_target_gripper_mode = [GripperMode.OPEN]*self.gripper_size
        self._prev_target_gripper_joint = [-999]*self.total_gripper_joint_size
        self.tensor_args = TensorDeviceType()
        self.arm_joint_limits = np.zeros((2, self.total_arm_joint_size))
        self.arm_soft_joint_limits = np.zeros((2, self.total_arm_joint_size))
        self.gripper_joint_limits = np.zeros((2, self.total_gripper_joint_size))
        self.gripper_soft_joint_limits = np.zeros((2, self.total_gripper_joint_size))
        self._move_arm = [True]*self.arm_size
        self._udp_initialized = False
        self._prev_arm_cmd_pos = None
        
        self._cmd_plan = None
        self._cmd_idx = 0
        self.waypoints = [] #Number equal to trajopt_tsteps
        self.limited_waypoints = [] #Number equal to trajopt_tsteps
        self.waypoints_velocity     = [] 
        self.waypoints_acceleration = []
        self.waypoints_jerk         = []
        self.waypoints_dt = 0
        
        self.curobo_cmd = None
        self.curobo_state = get_initialized_curobo_state(
            arm_size=self.arm_size,
            arm_joint_size = self.arm_joint_size,
            gripper_size=self.gripper_size,
            gripper_joint_size=self.gripper_joint_size
        )
        
        self.curobo_input_joint_names = []
        self.curobo_input_arm_joint_names = []
        robot_cfg_instance = RobotConfig.from_dict(self.robot_cfg, self.tensor_args)

        joint_limits_struc = robot_cfg_instance.kinematics.get_joint_limits()
        joint_limits_names = joint_limits_struc.joint_names
        joint_position_limits = joint_limits_struc.position.cpu().numpy()

        #Get input names
        self.curobo_input_joint_names = joint_limits_names
        self.curobo_input_arm_joint_names = [name for name in self.curobo_input_joint_names if name in self.total_arm_joint_names]

        #Get arm joint limits
        for name in self.total_arm_joint_names:
            idx = joint_limits_names.index(name)
            self.arm_joint_limits[:, idx] = joint_position_limits[:, idx]
            self.arm_soft_joint_limits[:, idx] = joint_position_limits[:, idx]*self.soft_limit_k
        
        #Get gripper joint limits
        for name in self.total_gripper_joint_names:
            idx = joint_limits_names.index(name)
            lower, upper = joint_position_limits[:, idx]
            threshold = (upper - lower) * 0.05
            self.gripper_joint_limits[:, idx-self.total_arm_joint_size] = joint_position_limits[:, idx]
            self.gripper_soft_joint_limits[0, idx-self.total_arm_joint_size] = lower + threshold
            self.gripper_soft_joint_limits[1, idx-self.total_arm_joint_size] = upper - threshold

        # TrajEvaluatorConfig
        if maximum_trajectory_time is not None:
            max_dt = maximum_trajectory_time / trajopt_tsteps
        traj_evaluator_config = TrajEvaluatorConfig.from_basic(
            min_dt=minimum_trajectory_dt, max_dt=max_dt, dof=robot_cfg_instance.kinematics.dof
        )
        # traj_evaluator_config.max_acc = joint_limits_struc.acceleration[1]
        # traj_evaluator_config.max_jerk = joint_limits_struc.jerk[1]
        # traj_evaluator_config.max_acc[joint_limits_names.index("arm_J1_l")] = 2.0
        # traj_evaluator_config.max_acc[joint_limits_names.index("arm_J3_l")] = 2.0
        # traj_evaluator_config.max_acc[joint_limits_names.index("arm_J1_r")] = 2.0
        # traj_evaluator_config.max_acc[joint_limits_names.index("arm_J3_r")] = 2.0
        
        self._motion_gen = None
        self._ik_solver = None
        if solver_type == PlannerCuroboSolverType.MOTION_GEN:
            print("Setting up motion generator")
            motion_gen_config = MotionGenConfig.load_from_robot_config(
                # self.robot_cfg,
                robot_cfg_instance,
                self.world_cfg,
                self.tensor_args,
                trajopt_tsteps = trajopt_tsteps, #Must be > 5
                # trajopt_tsteps=32,
                collision_checker_type=CollisionCheckerType.MESH,
                collision_activation_distance=0.00,
                # collision_activation_distance=0.005,
                ik_opt_iters=ik_opt_iters,
                use_cuda_graph=use_cuda_graph,
                ik_particle_opt = ik_particle_opt,
                num_ik_seeds = num_ik_seeds,
                num_trajopt_seeds=num_trajopt_seeds,
                num_graph_seeds=1,
                interpolation_dt = interpolation_dt,
                # interpolation_dt=0.002,
                # interpolation_type = InterpolateType.CUBIC,
                self_collision_check=self_collision_check,
                self_collision_opt = self_collision_opt,
                maximum_trajectory_time = maximum_trajectory_time,
                minimum_trajectory_dt = minimum_trajectory_dt, #Min traj time = minimum_trajectory_dt * trajopt_tsteps
                trim_steps=self.trim_steps, 
                position_threshold = solver_position_threshold,
                rotation_threshold = solver_rotation_threshold,
                cspace_threshold = solver_cspace_threshold,
                traj_evaluator_config = traj_evaluator_config
            )
            self._motion_gen = MotionGen(motion_gen_config)
            if not kwargs.get("skip_warmup", False):
                self._motion_gen.warmup(warmup_js_trajopt=warmup_js_trajopt)
            
            self._plan_config = MotionGenPlanConfig(
                # enable_graph=True,
                # enable_graph_attempt=1,
                # max_attempts=2,
                # enable_finetune_trajopt=False, 
                check_start_validity=False,
                time_dilation_factor = time_dilation_factor,
            )
        elif solver_type == PlannerCuroboSolverType.IK_SOLVER:
            print("Setting up ik solver")
            self._ik_solver = create_ik_solver(config,
                                        num_seeds = num_ik_seeds,
                                        self_collision_check = self_collision_check,
                                        self_collision_opt = self_collision_opt,
                                        use_particle_opt = ik_particle_opt,
                                        use_cuda_graph = use_cuda_graph,
                                        position_threshold = solver_position_threshold,
                                        rotation_threshold = solver_rotation_threshold)
        else:
            raise NotImplementedError("Unknown planning method")

        #Thread for sending curobo state
        self._curobo_state_delay_ms = 20
        self._curobo_state_sending_thread = threading.Thread(target=self._curobo_state_sending_task, args=(self._curobo_state_delay_ms,), daemon=True)
        self._curobo_state_sending_thread.start()
        # self._curobo_state_lock = threading.Lock()

    def __del__(self):
        self.terminate()
            
    def terminate(self):
        # self._curobo_state_lock.release()
        self._terminate_flag = True
        self._curobo_state_sending_thread.join()
        if self._robot_udp:
            self._robot_udp.terminate()
        if self._target_udp:
            self._target_udp.terminate()
        
    def clear_curobo_command(self):
        #Clear curobo command
        self.curobo_cmd.arm_joint_command = self._reshape_list(self.arm_joint_position, self.arm_joint_size)
        gripper_joint_command = self.gripper_convertion(
                self.gripper_joint_position, self.curobo_used_gripper_type, self.curobo_used_gripper_urdf_order,
                self.curobo_used_gripper_normalized, self.curobo_used_gripper_zero_open,
                TargetCommandGripperCommandType.JOINT, self.gripper_output_urdf_order,
                self.gripper_output_normalized, self.gripper_output_zero_open)
        self.curobo_cmd.gripper_joint_command = self._reshape_list(gripper_joint_command, self.gripper_joint_size)
        self.curobo_cmd.waypoints_size = 0
        self.curobo_cmd.waypoints = []
        
        self.target_arm_joint = self.arm_joint_position
        self.target_arm_tool_position = self._prev_target_tool_position = self.arm_tool_position
        self.target_arm_tool_orientation = self._prev_target_tool_orientation = self.arm_tool_orientation
        self.target_gripper_joint = self.gripper_joint_position
        self._prev_target_joint = self.get_input_state(PlannerInputStateType.TARGET_JOINT_COMMAND) #Arm + gripper cmd
        if self._target_udp: self._target_udp.clear()
        self.last_received_target_command = None
    
    def get_curobo_command(self):
        return copy.deepcopy(self.curobo_cmd)
    
    def connect_udp_to_robot(self, local_ip, local_port, remote_ip, remote_port, recv_delay_ms=10):
        """
        Connect to robot via UDP for receiving robot state and sending commands.
        
        Args:
            local_ip (str): Local IP address to bind to
            local_port (int): Local port to bind to
            remote_ip (str): Remote IP address of the robot
            remote_port (int): Remote port of the robot
            recv_delay_ms (int, optional): Receive delay in milliseconds. Default: 10
            
        Note:
            Only connects for use cases that require UDP communication with the robot.
            Prints a message if connection is not needed for the current use case.
        """
        if self._connection_method == PlannerConnectionMethod.BOTH_LOCAL or \
            self._connection_method == PlannerConnectionMethod.REMOTE_TARGET_AND_LOCAL_ROBOT:
            print("You don't need to connect to robot.")
            return
        
        if self._connection_method == PlannerConnectionMethod.BOTH_REMOTE or \
            self._connection_method == PlannerConnectionMethod.LOCAL_TARGET_AND_REMOTE_ROBOT:
            self._robot_udp = CuarmUdpThread(local_ip, local_port, remote_ip, remote_port, CuarmUdp.unpack_robot_state, CuarmUdp.pack_curobo_command, recv_delay_ms=recv_delay_ms)
        print("Connect to robot udp")
    
    def connect_udp_to_target(self, local_ip, local_port, remote_ip, remote_port, recv_delay_ms=10):
        """
        Connect to target via UDP for receiving target commands and sending state.
        
        Args:
            local_ip (str): Local IP address to bind to
            local_port (int): Local port to bind to
            remote_ip (str): Remote IP address of the target system
            remote_port (int): Remote port of the target system
            recv_delay_ms (int, optional): Receive delay in milliseconds. Default: 10
            
        Note:
            Only connects for use cases that require UDP communication with the target.
            Prints a message if connection is not needed for the current use case.
        """
        if self._connection_method == PlannerConnectionMethod.BOTH_LOCAL or \
            self._connection_method == PlannerConnectionMethod.LOCAL_TARGET_AND_REMOTE_ROBOT:
            print("You don't need to connect to target.")
            return
        
        if self._connection_method == PlannerConnectionMethod.BOTH_REMOTE or \
            self._connection_method == PlannerConnectionMethod.REMOTE_TARGET_AND_LOCAL_ROBOT:
            self._target_udp = CuarmUdpThread(local_ip, local_port, remote_ip, remote_port, CuarmUdp.unpack_target_command, CuarmUdp.pack_curobo_state, recv_delay_ms=recv_delay_ms)
        self._curobo_planning_status_planning_threshold *= int(recv_delay_ms / self._curobo_state_delay_ms)
        print("Connect to target udp")
    
    def receive_state_from_udp_connection(self) -> bool:
        ret = True
        #Receive target command
        if self._connection_method == PlannerConnectionMethod.BOTH_REMOTE or \
            self._connection_method == PlannerConnectionMethod.REMOTE_TARGET_AND_LOCAL_ROBOT:
            self.last_received_target_command = self._target_udp.receive()
            if not self._retrieve_target_command(self.last_received_target_command):
                ret = False
        return ret

    def check_if_udp_initialized(self):
        if self._udp_initialized: return True
        if (self._connection_method == PlannerConnectionMethod.REMOTE_TARGET_AND_LOCAL_ROBOT or \
            self._connection_method == PlannerConnectionMethod.BOTH_REMOTE) and self._target_udp is None:
            print("Target UDP is not connected, you need to call connect_udp_to_target() first.")
            return False
        if (self._connection_method == PlannerConnectionMethod.LOCAL_TARGET_AND_REMOTE_ROBOT or \
            self._connection_method == PlannerConnectionMethod.BOTH_REMOTE) and self._robot_udp is None:
            print("Robot UDP is not connected, you need to call connect_udp_to_robot() first.")
            return False
        self._udp_initialized = True
        self.status = CuroboStatus.INITIALIZED
        return True
    
    def compute_fk(self, arm_joint_position):
        with self._gpu_lock:
            return self._compute_fk_without_lock(arm_joint_position)
    
    def compute_ik(self, tool_position, tool_orientation, current_joint_positon) -> IKResult:
        """
        :param tool_position: 2D array of shape (arm_size, 3), ordered as [x, y, z], unit in meters
        :param tool_orientation: 2D array of shape (arm_size, 4), ordered as [w, x, y, z]
        :param current_joint_positon: 2D array of shape (arm_size, arm_joint_size), unit in radians
        :return: IKResult
        """
        #Remove tool offset
        ee_position = tool_position.copy()
        ee_orientation = tool_orientation.copy()
        for i in range(len(ee_position)):
            ee_position[i] = remove_tool_offset(tool_position[i], tool_orientation[i], self.end_effector_offset[i])
            
        goal_pose = Pose(
            position=self.tensor_args.to_device(ee_position[self.ee_link_idx_in_target_link_names]),
            quaternion=self.tensor_args.to_device(ee_orientation[self.ee_link_idx_in_target_link_names]),
        )
        
        link_poses = {}
        if len(ee_position) == self.target_size and self.target_link_names:
            for i, name in enumerate(self.target_link_names):
                c_p, c_rot = ee_position[i], ee_orientation[i]
                link_poses[name] = Pose(
                    position=self.tensor_args.to_device(c_p),
                    quaternion=self.tensor_args.to_device(c_rot),
                )
        start_pos = self.tensor_args.to_device([current_joint_positon])
        
        success, solution = False, None
        result = None
        with self._gpu_lock:
            if self._solver_type == PlannerCuroboSolverType.MOTION_GEN:
                result = self._motion_gen.ik_solver.solve_single(goal_pose, seed_config=start_pos, link_poses=link_poses)
            elif self._solver_type == PlannerCuroboSolverType.IK_SOLVER:
                result = self._ik_solver.solve_single(goal_pose, seed_config=start_pos, link_poses=link_poses)
            else:
                raise NotImplementedError("IK solver not implemented for method: " + self._solver_type.name)
            success = result.success.item()
            solution = self._reshape_list(result.solution.cpu().numpy().squeeze(), self.arm_joint_size)
            print("IK success: {}, \tTime: {}s".format(success, round(result.solve_time, 5)))
        return success, solution
    
    def receive(self, robot_state: RobotState=None, target_command: TargetCommand=None) -> bool:
        ret = True
        #Ensure UDP connections set up
        assert self.check_if_udp_initialized()
        
        #Get robot and target state for UDP connection
        ret &= self.receive_state_from_udp_connection()
        
        #Update robot and target state for API connection
        ret &= self._update_state_from_local_connection(robot_state=robot_state, target_command=target_command)

        return ret
    
    def compute(self) -> Tuple[CuroboCommand, CuroboState]:
        if self._first_robot_state_received and self._first_target_command_received:
            if self.planner_mode == TargetCommandPlannerMode.PLANNING:
                if self._solver_type == PlannerCuroboSolverType.MOTION_GEN and self.target_arm_type == TargetCommandArmCommandType.CARTESIAN:
                    self._motion_gen_plan_single()
                elif self._solver_type == PlannerCuroboSolverType.MOTION_GEN and self.target_arm_type == TargetCommandArmCommandType.JOINT:
                    self._motion_gen_plan_single_js()
            else:
                self._clear_planned_result()
                if self.target_arm_type == TargetCommandArmCommandType.CARTESIAN:   #Included collision check
                    joint_input_pos = self.get_input_state(PlannerInputStateType.JOINT_POSITION)
                    self._solve_ik()
                else:
                    joint_cmd = self.target_arm_joint
                    if(self.planner_mode == TargetCommandPlannerMode.NO_PLANNING or \
                    self.planner_mode == TargetCommandPlannerMode.COLLISION_ONLY):  #Do collision check explicitly
                        joint_input_pos = self.get_input_state(PlannerInputStateType.TARGET_JOINT_COMMAND)
                        if not self.check_collision(joint_input_pos):
                            joint_cmd = None
                            self.status = CuroboStatus.COLLISION_CHECK_FAILED
                        else:
                            self.status = CuroboStatus.INITIALIZED
                    else:
                        self.status = CuroboStatus.INITIALIZED
                
                    if joint_cmd is not None:
                        cliped_joint_cmd = self._clip_arm_joint_command(joint_cmd)
                        self.curobo_cmd.arm_joint_command = self._reshape_list(cliped_joint_cmd, self.arm_joint_size)
            self.set_gripper_command()
            if self.target_arm_type == TargetCommandArmCommandType.CARTESIAN:
                self._prev_target_tool_position = self.target_arm_tool_position
                self._prev_target_tool_orientation = self.target_arm_tool_orientation
                self._prev_target_arm_joint = self.arm_joint_position
            else:
                self._prev_target_tool_position = self.arm_tool_position
                self._prev_target_tool_orientation = self.arm_tool_orientation
                self._prev_target_arm_joint = self.target_arm_joint
        return copy.deepcopy(self.curobo_cmd), copy.deepcopy(self.curobo_state)
    
    def receive_and_compute(self, robot_state: RobotState=None, target_command: TargetCommand=None) -> Tuple[CuroboCommand, CuroboState]:
        """
        Receive robot and target states, compute motion plan, and update commands and state.
        
        Args:
            robot_state (RobotState, optional): Current robot state from API. Defaults to None.
            target_command (TargetCommand, optional): Current target command from API. Defaults to None.
        
        Returns:
            Tuple[CuroboCommand, CuroboState]: Computed command and state objects for the robot.
        
        Raises:
            AssertionError: If UDP connections are not initialized.
        """
        ret = self.receive(robot_state=robot_state, target_command=target_command)
        
        #Compute curobo command
        self.compute()
        
        return copy.deepcopy(self.curobo_cmd), copy.deepcopy(self.curobo_state)

    def receive_compute_send(self, robot_state: RobotState=None, target_command: TargetCommand=None) -> Tuple[CuroboCommand, CuroboState]:
        """
        Perform receive, compute, and send cycle for robot commands and state.
        
        Args:
            robot_state (RobotState, optional): Current robot state from API. Defaults to None.
            target_command (TargetCommand, optional): Current target command from API. Defaults to None.
        
        Returns:
            Tuple[CuroboCommand, CuroboState]: Command and state objects sent to the robot.
        """
        self.receive_and_compute(robot_state=robot_state, target_command=target_command)
        
        #Send curobo command
        self.send_curobo_command()
        return copy.deepcopy(self.curobo_cmd), copy.deepcopy(self.curobo_state)
    
    def get_input_state(self, input_state_type:PlannerInputStateType):
        if input_state_type == PlannerInputStateType.JOINT_NAME:
            return self.reorder_input(self.total_arm_joint_names, self.total_gripper_joint_names, as_curobo=True)
        elif input_state_type == PlannerInputStateType.JOINT_POSITION:
            return self.reorder_input(self.arm_joint_position, self.gripper_joint_position, as_curobo=True)
        elif input_state_type == PlannerInputStateType.JOINT_VELOCITY:
            return self.reorder_input(self.arm_joint_velocity, [0]*self.total_gripper_joint_size, as_curobo=True)
        elif input_state_type == PlannerInputStateType.TARGET_JOINT_COMMAND:
            return self.reorder_input(self.target_arm_joint, self.target_gripper_joint, as_curobo=True)
        elif input_state_type == PlannerInputStateType.LAST_WAYPOINT_POSITION:
            if len(self.waypoints) == 0: return [] 
            return self.reorder_input(self.waypoints[-1], self.target_gripper_joint, as_curobo=True)
        elif input_state_type == PlannerInputStateType.LAST_LIMITED_WAYPOINT_POSITION:
            if len(self.limited_waypoints) == 0: return [] 
            return self.reorder_input(self.limited_waypoints[-1], self.target_gripper_joint, as_curobo=True)
        else:
            raise ValueError("Invalid input state type")
    
    def reorder_input(self, arm_joint_value, gripper_joint_value=None, as_curobo=True):
        config_order = self.total_arm_joint_names
        if gripper_joint_value:
            config_order = self.total_arm_joint_names + self.total_gripper_joint_names
        curobo_order = self.curobo_input_joint_names if gripper_joint_value else self.curobo_input_arm_joint_names
        cur_order, final_order = (config_order, curobo_order) if as_curobo else (curobo_order, config_order)
        cur_input = np.concatenate((arm_joint_value, gripper_joint_value)) if gripper_joint_value else arm_joint_value
        assert len(cur_input) == len(cur_order), (
            "Input length does not match order length, "
            "current input: {}, \ncurrent order: {}".format(cur_input, cur_order)
        )

        out = []
        for name in final_order:
            if name in cur_order:
                out.append(cur_input[cur_order.index(name)])
            else:
                raise ValueError(f"Joint name {name} not found in current order: {cur_order}")
        if isinstance(arm_joint_value, np.ndarray):
            return np.array(out)
        return out
        
    def check_collision(self, joint_position) -> bool:
        """
        Check if a joint position would result in collision.
        
        Args:
            joint_position (np.ndarray): Joint positions to check
            
        Returns:
            bool: True if no collision, False if collision detected
            
        Raises:
            NotImplementedError: If planning method doesn't support collision checking
        """
        valid = False
        if (self._solver_type == PlannerCuroboSolverType.MOTION_GEN):
            js = JointState(
                position=self.tensor_args.to_device(joint_position),
            )
            valid, status = self._motion_gen.check_start_state(js)

            if not valid:
                print("check_collision() invalid status: ", status)
        else:
            raise NotImplementedError("check_collision() has not been implemented for: " + self._solver_type.name)
        return valid
    
    def _check_target_arm_collision(self, arm_joint_position, label) -> bool:
        if self._solver_type != PlannerCuroboSolverType.MOTION_GEN:
            print(f"{label} collision check skipped for solver: {self._solver_type.name}")
            return True

        gripper_joint = self.target_gripper_joint if self.total_gripper_joint_size > 0 else None
        joint_position = self.reorder_input(arm_joint_position, gripper_joint, as_curobo=True)
        valid = self.check_collision(joint_position)
        print(f"{label} collision free: ", valid)
        if not valid:
            print(f"{label} target arm joint: ", arm_joint_position)
            print(f"{label} target gripper joint: ", gripper_joint)
            print(f"{label} collision check input: ", joint_position)
        return valid
    
    def _hold_current_arm_command(self):
        self._cmd_plan = None
        self._cmd_idx = 0
        if self.curobo_cmd is None:
            return
        self.curobo_cmd.arm_joint_command = self._reshape_list(self.arm_joint_position, self.arm_joint_size)
        self.curobo_cmd.cmd_plan_size = 0
        self.curobo_cmd.waypoints_size = 0
        self.curobo_cmd.waypoints = []
    
    def gripper_convertion(self, cmd, 
                        input_type: TargetCommandGripperCommandType, input_urdf_order: bool, 
                        input_normalized: bool, input_zero_open: bool,
                        output_type: TargetCommandGripperCommandType, output_urdf_order: bool, 
                        output_normalized: bool, output_zero_open:bool):
        if (input_type == output_type and input_urdf_order == output_urdf_order and input_normalized == output_normalized):
            return cmd
        if (output_type == TargetCommandGripperCommandType.MODE):
            raise NotImplementedError("output gripper command as mode is not supported")
        
        #Convert mode to joint command
        input_joint_cmd = cmd 
        if input_type is TargetCommandGripperCommandType.MODE:
            input_joint_cmd = self._gripper_mode_to_joint(cmd)
        
        #Normalization
        norm_cmd = self._gripper_normalization(input_joint_cmd, input_normalized, output_normalized)
        
        #Reorder joint
        reorder_cmd = self._gripper_order_convertion(norm_cmd, input_urdf_order, output_urdf_order)
        
        #Zero may represent open or close
        comp_cmd = self._gripper_complement(reorder_cmd, output_normalized, input_zero_open, output_zero_open)
        return comp_cmd

    def set_gripper_command(self):
        if self.target_gripper_joint is not None:
            gripper_joint_command = self.gripper_convertion(
                self.target_gripper_joint, self.curobo_used_gripper_type, self.curobo_used_gripper_urdf_order,
                self.curobo_used_gripper_normalized, self.curobo_used_gripper_zero_open,
                TargetCommandGripperCommandType.JOINT, self.gripper_output_urdf_order,
                self.gripper_output_normalized, self.gripper_output_zero_open)
            self.curobo_cmd.gripper_joint_command = self._reshape_list(gripper_joint_command, self.gripper_joint_size)
            # self.curobo_cmd.gripper_joint_size = self.total_gripper_joint_size
        self._prev_target_gripper_mode = self.target_gripper_mode
        self._prev_target_gripper_joint = self.target_gripper_joint_raw

    def send_curobo_command(self):
        if self._robot_udp is not None and self._first_target_command_received and self._first_robot_state_received:
            self._robot_udp.send(self.curobo_cmd)
   
    def _curobo_state_sending_task(self, delay_ms = 20):
        if self._connection_method == PlannerConnectionMethod.BOTH_LOCAL: return

        while not self._terminate_flag:
            #Receive robot state from remote robot
            if self._robot_udp is not None and (self._connection_method == PlannerConnectionMethod.BOTH_REMOTE or \
                self._connection_method == PlannerConnectionMethod.LOCAL_TARGET_AND_REMOTE_ROBOT):
                robot_state = self._robot_udp.receive()
                self._retrieve_robot_state(robot_state)
                
            self._update_curobo_state()
                    
            #Send curobo state to remote target
            self._send_curobo_state() 
            time.sleep(delay_ms / 1000.0)

    def _initialize_curobo_command(self):
        self.curobo_cmd = CuroboCommand()
        self.curobo_cmd.arm_size = self.arm_size
        self.curobo_cmd.arm_joint_size = self.arm_joint_size.copy()
        self.curobo_cmd.gripper_size = self.gripper_size
        self.curobo_cmd.gripper_joint_size = self.gripper_joint_size.copy()
        self.clear_curobo_command()
        
    def _retrieve_robot_state(self, robot_state: RobotState) -> bool:
        """
        Retrieve and validate robot state from RobotState object.
        
        Args:
            robot_state (RobotState): Robot state object containing joint positions and velocities
            
        Returns:
            bool: True if robot state was successfully retrieved and validated, False otherwise
            
        Raises:
            AssertionError: If joint sizes don't match expected configuration
        """
        self.last_received_robot_state = robot_state
        if robot_state is None: return False
        
        if robot_state.arm_size == 0: return False
        assert robot_state.arm_size == self.arm_size, "RobotState arm size does not match"
        for i in range(robot_state.arm_size):
            if robot_state.arm_joint_size[i] != self.arm_joint_size[i]:
                raise ValueError(f"RobotState arm {i} joint size do not match: "
                                f"got {robot_state.arm_joint_size[i]}, expected {self.arm_joint_size[i]}")
            
        assert robot_state.gripper_size == 0 or robot_state.gripper_size == self.gripper_size, "RobotState gripper size does not match"
        for i in range(robot_state.gripper_size):
            if robot_state.gripper_joint_size[i] != self.gripper_joint_size[i]:
                raise ValueError(f"RobotState gripper {i} joint size do not match: "
                                f"got {robot_state.gripper_joint_size[i]}, expected {self.gripper_joint_size[i]}")
            
        self.arm_joint_position = np.array(self._flatten_list(robot_state.arm_joint_position))
        self.arm_joint_velocity = np.array(self._flatten_list(robot_state.arm_joint_velocity))
        flatten_gripper_joint_position = self._flatten_list(robot_state.gripper_joint_position)
        self.gripper_joint_position = self.gripper_convertion(
            flatten_gripper_joint_position, self.robot_sent_gripper_type, self.robot_sent_gripper_urdf_order,
            self.robot_sent_gripper_normalized, self.robot_sent_gripper_zero_open, self.curobo_used_gripper_type, 
            self.curobo_used_gripper_urdf_order, self.curobo_used_gripper_normalized, self.curobo_used_gripper_zero_open)

        self.robot_status = robot_state.status
        if not self._first_robot_state_received:
            self._initialize_curobo_command()
            self._first_robot_state_received = True
        return True
        
    def _retrieve_target_command(self, target_command: TargetCommand) -> bool:
        """
        Retrieve and validate target command from TargetCommand object.
        
        Args:
            target_command (TargetCommand): Target command object containing target information
            
        Returns:
            bool: True if target command was successfully retrieved and validated, False otherwise
            
        Raises:
            AssertionError: If target sizes don't match expected configuration
        """
        if target_command is None:
            return False
        
        if target_command.arm_size == 0: return False
        assert target_command.arm_size == self.arm_size, "TargetCommand arm size do not match"
        for i in range(target_command.arm_size):
            if target_command.arm_joint_size[i] != self.arm_joint_size[i]:
                raise ValueError(f"TargetCommand arm {i} joint size do not match: "
                                f"got {target_command.arm_joint_size[i]}, expected {self.arm_joint_size[i]}")
        
        assert target_command.gripper_size == 0 or target_command.gripper_size == self.gripper_size, "TargetCommand gripper size do not match"
        for i in range(target_command.gripper_size):
            if target_command.gripper_joint_size[i] != self.gripper_joint_size[i]:
                raise ValueError(f"TargetCommand gripper {i} joint size do not match: "
                                f"got {target_command.gripper_joint_size[i]}, expected {self.gripper_joint_size[i]}")
        
        self.planner_mode = TargetCommandPlannerMode(target_command.planner_mode.value) #Prevent protobuf enum issue
        
        #Arm related
        self.target_arm_type = TargetCommandArmCommandType(target_command.arm_command_type.value) #Prevent protobuf enum issue
        if (self.target_arm_type == TargetCommandArmCommandType.CARTESIAN):
            arm_tool_command = np.array(target_command.arm_tool_command)
            self.target_arm_tool_position = arm_tool_command[:, :3]
            self.target_arm_tool_orientation = arm_tool_command[:, 3:]
        elif (self.target_arm_type == TargetCommandArmCommandType.JOINT):
            self.target_arm_joint = np.array(self._flatten_list(target_command.arm_joint_command))
        
        #Gripper related
        self.target_gripper_type = TargetCommandGripperCommandType(target_command.gripper_command_type.value) #Prevent protobuf enum issue
        if (self.target_gripper_type == TargetCommandGripperCommandType.MODE):
            self.target_gripper_mode = target_command.gripper_mode_command.copy()
            self.target_gripper_joint = self.gripper_convertion(
                self.target_gripper_mode, self.target_gripper_type, True, True, True, 
                self.curobo_used_gripper_type, self.curobo_used_gripper_urdf_order, self.curobo_used_gripper_normalized, 
                self.curobo_used_gripper_zero_open)
        elif (self.target_gripper_type == TargetCommandGripperCommandType.JOINT):
            self.target_gripper_joint_raw = self._flatten_list(target_command.gripper_joint_command)
            self.target_gripper_joint = self.gripper_convertion(
                self.target_gripper_joint_raw, self.target_gripper_type, self.gripper_input_urdf_order, 
                self.gripper_input_normalized, self.gripper_input_zero_open, 
                self.curobo_used_gripper_type, self.curobo_used_gripper_urdf_order, self.curobo_used_gripper_normalized, 
                self.curobo_used_gripper_zero_open)
        
        #Description
        self.target_description = target_command.description
        self._first_target_command_received = True
        return True
    
    def _update_state_from_local_connection(self, robot_state: RobotState, target_command: TargetCommand) -> bool:
        ret = True
        if self._connection_method == PlannerConnectionMethod.BOTH_LOCAL or \
            self._connection_method == PlannerConnectionMethod.LOCAL_TARGET_AND_REMOTE_ROBOT:
            if not self._retrieve_target_command(target_command):
                ret &= False
        if self._connection_method == PlannerConnectionMethod.BOTH_LOCAL or \
            self._connection_method == PlannerConnectionMethod.REMOTE_TARGET_AND_LOCAL_ROBOT:
            if not self._retrieve_robot_state(robot_state):
                ret &= False
            # self._update_curobo_state() //Updated in thread
        return ret
  
    def _compute_fk_without_lock(self, arm_joint_position):
        #Calculate Forward kinematics
        if (len(arm_joint_position) == self.total_arm_joint_size):
            arm_joint_position = self.reorder_input(arm_joint_position, as_curobo=True)
            q_tensor = self.tensor_args.to_device([arm_joint_position])
            if self._solver_type == PlannerCuroboSolverType.MOTION_GEN:
                out = self._motion_gen.kinematics.get_link_poses(q_tensor, link_names=self.target_link_names)
            elif self._solver_type == PlannerCuroboSolverType.IK_SOLVER:
                out = self._ik_solver.kinematics.get_link_poses(q_tensor, link_names=self.target_link_names)
            arm_ee_position = out.position.cpu().numpy().squeeze()
            arm_ee_orientation = out.quaternion.cpu().numpy().squeeze()

            arm_tool_position = np.zeros(arm_ee_position.shape)
            arm_tool_orientation = arm_ee_orientation.copy()
            for i in range(self.arm_size):
                arm_tool_position[i] = add_tool_offset(arm_ee_position[i], arm_ee_orientation[i], self.end_effector_offset[i])
            return arm_tool_position, arm_tool_orientation
        
    def _update_curobo_state(self): 
        #Update arm poses only when graph capturing is not used
        if not self._use_cuda_graph or self._gpu_lock.acquire(blocking=False):
            arm_tool_position, arm_tool_orientation = self._compute_fk_without_lock(self.arm_joint_position)
            if self._use_cuda_graph: self._gpu_lock.release()
            
            self.arm_tool_position = arm_tool_position.copy()
            self.arm_tool_orientation = arm_tool_orientation.copy()
            
            if self._first_robot_state_received:
                self.curobo_state.arm_joint_position = self.last_received_robot_state.arm_joint_position
                self.curobo_state.arm_joint_velocity = self.last_received_robot_state.arm_joint_velocity
                self.curobo_state.arm_tool_pose = np.concatenate((self.arm_tool_position, self.arm_tool_orientation), axis=1).tolist()
        
        if self._first_robot_state_received:
            self.curobo_state.gripper_joint_position = self.last_received_robot_state.gripper_joint_position
            self.curobo_state.robot_status = self.last_received_robot_state.status
            
            #Send CuroboStatus.PLANNING for n times to ensure the receiver get the PLANNING status
            if self._curobo_planning_status_planning_count < self._curobo_planning_status_planning_threshold:
                # self.curobo_state.curobo_status = CuroboStatus.PLANNING
                self._curobo_planning_status_planning_count += 1
            else:
                self.curobo_state.curobo_status = self.status
                
                #Check if arm reached
                if self.last_received_target_command:
                    target_cmd = self.last_received_target_command
                    self.curobo_state.processed_arm_joint_command = self._reshape_list(self._prev_target_arm_joint, self.arm_joint_size)
                    self.curobo_state.processed_arm_tool_command  = np.concatenate((self._prev_target_tool_position, self._prev_target_tool_orientation), axis=1)
                    # A preview is never sent to the panel. Return its actual
                    # final, limit-clipped joint waypoint through the existing
                    # state field so the caller can rank safe IK candidates.
                    # Normal motion state remains unchanged.
                    if (
                        str(getattr(target_cmd, "description", "")).startswith("__preview__:")
                        and getattr(self, "_preview_result_description", None)
                        == getattr(target_cmd, "description", "")
                        and getattr(self, "_preview_final_arm_joint", None) is not None
                    ):
                        self.curobo_state.processed_arm_joint_command = self._reshape_list(
                            self._preview_final_arm_joint, self.arm_joint_size
                        )
                    self.curobo_state.processed_gripper_mode_command = self._prev_target_gripper_mode
                    self.curobo_state.processed_gripper_joint_command = self._reshape_list(self._prev_target_gripper_joint, self.gripper_joint_size)
                    if (self.status == CuroboStatus.PLANNING_FAILED or \
                        self.status == CuroboStatus.IK_FAILED or self.status == CuroboStatus.COLLISION_CHECK_FAILED):
                        self.curobo_state.curobo_status = self.status
                    else:
                        arm_reached = (target_cmd.arm_command_type == TargetCommandArmCommandType.JOINT and \
                            same_array(target_cmd.arm_joint_command, self.curobo_state.arm_joint_position, atol=self.target_joint_position_precision)) or \
                            (target_cmd.arm_command_type == TargetCommandArmCommandType.CARTESIAN and \
                            same_array(self.target_arm_tool_position, self.arm_tool_position, atol=self.target_cartesian_position_precision) and \
                            same_quaternion(self.target_arm_tool_orientation, self.arm_tool_orientation, atol=self.target_cartesian_orientation_precision))
                        gripper_reached = self.gripper_size == 0 or (
                            (target_cmd.gripper_command_type == TargetCommandGripperCommandType.JOINT and \
                            same_array(target_cmd.gripper_joint_command, self.curobo_state.gripper_joint_position, atol=1e-2)) or \
                            target_cmd.gripper_command_type == TargetCommandGripperCommandType.MODE)
                        if arm_reached and gripper_reached:
                            self.curobo_state.curobo_status = CuroboStatus.MOTION_FINISHED
    def _send_curobo_state(self):
        if self._target_udp is not None and self._first_robot_state_received:
            self._target_udp.send(self.curobo_state)
    
    def _update_move_arm_cartesian(self, target_tool_position, target_tool_orientation, arm_tool_position, arm_tool_orientation):
        # Hold an arm already at its goal, including the first command and a
        # JOINT -> CARTESIAN transition. Requiring a previous identical target
        # allowed the unrequested arm to change redundant joint configuration.
        self._move_arm = []
        for i in range(self.arm_size):
            target_reached = same_array(arm_tool_position[i], target_tool_position[i], atol=1e-3) and \
                same_quaternion(arm_tool_orientation[i], target_tool_orientation[i], atol=self.target_cartesian_orientation_precision)
            self._move_arm.append(not target_reached)
    
    def _update_move_arm_joint(self, target_joint_position):
        self._move_arm = []
        start_ind = 0
        for size in self.arm_joint_size:
            end_ind = start_ind + size
            target_reached = same_array(
                self.arm_joint_position[start_ind:end_ind],
                target_joint_position[start_ind:end_ind],
                atol=self.target_joint_position_precision)
            self._move_arm.append(not target_reached)
            start_ind = end_ind
    
    def _retrieve_motion_gen_result(self, result:MotionGenResult) -> bool:
        succ = result.success.item()
        if succ:
            #Cmd plan
            cmd_plan = result.get_interpolated_plan()
            cmd_plan = self._motion_gen.get_full_js(cmd_plan)
            common_js_names = []
            for x in self.total_arm_joint_names:
                if x in cmd_plan.joint_names:
                    common_js_names.append(x)
            self._cmd_plan = cmd_plan.get_ordered_joint_state(common_js_names)
            self._cmd_idx = 0
            
            #Waypoints
            optimized_plan = result.optimized_plan
            optimized_dt = result.optimized_dt
            optimized_plan = optimized_plan.get_ordered_joint_state(common_js_names)
            self.waypoints = optimized_plan.position.cpu().numpy()
            self.waypoints_velocity     = optimized_plan.velocity.cpu().numpy()
            self.waypoints_acceleration = optimized_plan.acceleration.cpu().numpy()
            self.waypoints_jerk         = optimized_plan.jerk.cpu().numpy()
            self.waypoints_dt = optimized_dt.cpu().numpy()
            if self.waypoints_dt.shape:
                self.waypoints_dt = self.waypoints_dt[0]
                
            self.limited_waypoints = self.waypoints.copy()
            limited_waypoints_reshapped = []
            for i in range(len(self.limited_waypoints)):
                #Fix non-moving arm 
                fixed_waypoint = self._fix_non_moving_arm(self.limited_waypoints[i], self.arm_joint_position)
                
                #Clip by joint limit
                cliped_waypoint = self._clip_arm_joint_command(fixed_waypoint)
                cliped_waypoint_list = cliped_waypoint.tolist()
                self.limited_waypoints[i] = cliped_waypoint_list
                limited_waypoints_reshapped.append(self._reshape_list(cliped_waypoint_list, self.arm_joint_size))
                
            if len(self.limited_waypoints) > 0:
                final_waypoint = self.limited_waypoints[-1]
                if not self._check_target_arm_collision(final_waypoint, "MotionGen final waypoint"):
                    print("MotionGen final waypoint collision check failed before sending joint command.")
                    print("target tool position: ", self.target_arm_tool_position)
                    print("target tool orientation: ", self.target_arm_tool_orientation)
                    print("current arm joint position: ", self.arm_joint_position)
                    print("final limited waypoint: ", final_waypoint)
                    self.status = CuroboStatus.COLLISION_CHECK_FAILED
                    self._hold_current_arm_command()
                    return False
                if str(self.target_description).startswith("__preview__:"):
                    self._preview_result_description = self.target_description
                    self._preview_final_arm_joint = final_waypoint.copy()

            #Get only joint names that are in both:
            self.curobo_cmd.cmd_plan_size = len(self._cmd_plan)
            self.curobo_cmd.waypoints_size = len(limited_waypoints_reshapped)
            self.curobo_cmd.waypoints_dt = self.waypoints_dt
            self.curobo_cmd.waypoints = limited_waypoints_reshapped
            self.status = CuroboStatus.PLANNING_SUCCEEDED
            print("self._move_arm: ", self._move_arm)
            print("planned commands size:", len(self._cmd_plan))
            print("waypoints_size: ", len(limited_waypoints_reshapped))
            print("waypoints_dt: ", self.waypoints_dt)
            print("last waypoint: ", limited_waypoints_reshapped[-1])
        else:
            self.status = CuroboStatus.PLANNING_FAILED
            self._hold_current_arm_command()
            print("result.status:", result.status)
        print(f"time ik: {result.ik_time:.3f}, graph: {result.graph_time:.3f}, trajopt: {result.trajopt_time:.3f}, finetune: {result.finetune_time:.3f}, solve: {result.solve_time:.3f}, total: {result.total_time:.3f}")
        print("============================================================================")
        return succ
        
    def _motion_gen_plan_single_js(self) -> bool:
        """
        Generate motion plan for joint space target using Curobo motion generation.
        
        Returns:
            bool: True if a new plan was generated, False if using previous plan
            
        Note:
            Only generates a new plan if the target joint position has changed significantly
            from the previous target, or if no previous plan exists.
        """
        if not self._solver_type == PlannerCuroboSolverType.MOTION_GEN:
            raise NotImplementedError("Only PlannerCuroboSolverType.MOTION_GEN is supported for _motion_gen_plan_single_js")
        
        joint_names = self.get_input_state(PlannerInputStateType.JOINT_NAME)
        joint_position = self.get_input_state(PlannerInputStateType.JOINT_POSITION)
        joint_velocity = self.get_input_state(PlannerInputStateType.JOINT_VELOCITY)
        target_joint_position =  self.get_input_state(PlannerInputStateType.TARGET_JOINT_COMMAND)
        target_description = self.target_description

        replan = False
        # Replan when either the joint goal or the target-command identity
        # changes. Joint reset requests intentionally use a unique description
        # even when their numerical joint target is unchanged.
        if (self._prev_target_joint is None or \
            not same_array(target_joint_position, self._prev_target_joint, atol=self.target_joint_position_precision) or \
            target_description != getattr(self, "_prev_target_description", None)
            ):
            self.status = CuroboStatus.PLANNING
            self._curobo_planning_status_planning_count = 0
            if target_description != getattr(
                self, "_prev_target_description", None
            ):
                print(
                    "Joint target command identity changed; forcing replan: "
                    f"{target_description}"
                )
            
            cu_js = JointState(
                position=self.tensor_args.to_device(joint_position),
                velocity=self.tensor_args.to_device(joint_velocity) * 0.0,
                acceleration=self.tensor_args.to_device(joint_velocity) * 0.0,
                jerk=self.tensor_args.to_device(joint_velocity) * 0.0,
                joint_names=joint_names,
            )
            
            goal_js = JointState(
                position=self.tensor_args.to_device(target_joint_position),
                velocity=self.tensor_args.to_device(target_joint_position) * 0.0,
                acceleration=self.tensor_args.to_device(target_joint_position) * 0.0,
                jerk=self.tensor_args.to_device(target_joint_position) * 0.0,
                joint_names=joint_names,
            )
            self._update_move_arm_joint(target_joint_position)
            
            with self._gpu_lock:
                result = self._motion_gen.plan_single_js(cu_js.unsqueeze(0), goal_js.unsqueeze(0), self._plan_config)
            succ = self._retrieve_motion_gen_result(result)
            
            #trim_steps is not used in plan_single_js, we need to trim the steps manually
            if succ:
                start = self.trim_steps[0]
                end = len(self.waypoints) + self.trim_steps[1] if self.trim_steps[1] < 0 else self.trim_steps[1]
                self.waypoints = np.array([self.waypoints[i] for i in range(start, end)])
                waypoints_reshaped = [self._reshape_list(waypoint, self.arm_joint_size) for waypoint in self.waypoints]
                self.curobo_cmd.waypoints = waypoints_reshaped
                self.curobo_cmd.waypoints_size = len(waypoints_reshaped)
            replan = True
        self._get_next_planned_arm_cmd()
        self._prev_target_joint = target_joint_position
        self._prev_target_description = target_description
        return replan

    def _motion_gen_plan_single(self) -> bool:
        """
        Generate motion plan for Cartesian space target using Curobo motion generation.
        
        Returns:
            bool: True if a new plan was generated, False if using previous plan
            
        Note:
            Only generates a new plan if the target Cartesian position or orientation
            has changed significantly from the previous target, or if no previous plan exists.
            Handles end effector offset compensation.
        """
        if not self._solver_type == PlannerCuroboSolverType.MOTION_GEN:
            raise NotImplementedError("Only PlannerCuroboSolverType.MOTION_GEN is supported for _motion_gen_plan_single")
        
        joint_names = self.get_input_state(PlannerInputStateType.JOINT_NAME)
        joint_position = self.get_input_state(PlannerInputStateType.JOINT_POSITION)
        joint_velocity = self.get_input_state(PlannerInputStateType.JOINT_VELOCITY)
        arm_tool_position = self.arm_tool_position
        arm_tool_orientation = self.arm_tool_orientation
        target_tool_orientation = self.target_arm_tool_orientation
        target_tool_position = self.target_arm_tool_position

        replan = False
        #Replan if a new target is given and previous target reached
        if (self._prev_target_tool_position is None or \
            self._prev_target_tool_orientation is None or \
            not same_array(target_tool_position, self._prev_target_tool_position, atol=self.target_cartesian_position_precision) or \
            not same_quaternion(target_tool_orientation, self._prev_target_tool_orientation, atol=self.target_cartesian_orientation_precision)
        ):
            print("current tool_position: ",    arm_tool_position) 
            print("current tool_orientation: ", arm_tool_orientation) 
            print("target tool position: ",     target_tool_position) 
            print("target tool orientation: ",  target_tool_orientation) 
            print("current joint_position: ",   joint_position)

            self.status = CuroboStatus.PLANNING
            self._curobo_planning_status_planning_count = 0

            #Remove tool offset
            target_ee_position = np.zeros(target_tool_position.shape)
            target_ee_orientation = target_tool_orientation
            for i in range(self.target_size):
                target_ee_position[i] = remove_tool_offset(target_tool_position[i], target_tool_orientation[i], self.end_effector_offset[i])
            # print("target_ee_position: ", target_ee_position)

            #Move the arms whose targets have been changed, avoiding irrelevant arms from moving.
            self._update_move_arm_cartesian(target_tool_position, target_tool_orientation, arm_tool_position, arm_tool_orientation)

            cu_js = JointState(
                position=self.tensor_args.to_device(joint_position),
                velocity=self.tensor_args.to_device(joint_velocity) * 0.0,
                acceleration=self.tensor_args.to_device(joint_velocity) * 0.0,
                jerk=self.tensor_args.to_device(joint_velocity) * 0.0,
                joint_names=joint_names,
            )
            
            # define link poses
            link_poses = {}
            if self.target_link_names:
                for i, name in enumerate(self.target_link_names):
                    c_p, c_rot = target_ee_position[i], target_ee_orientation[i]
                    link_poses[name] = Pose(
                        position=self.tensor_args.to_device(c_p),
                        quaternion=self.tensor_args.to_device(c_rot),
                    )
                    
            # define ee goal pose
            ik_goal = Pose(
                position=self.tensor_args.to_device(target_ee_position[self.ee_link_idx_in_target_link_names]),
                quaternion=self.tensor_args.to_device(target_ee_orientation[self.ee_link_idx_in_target_link_names]),
            )
            
            with self._gpu_lock:
                result = self._motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self._plan_config, link_poses=link_poses)
            self._retrieve_motion_gen_result(result)
            replan = True
        self._get_next_planned_arm_cmd()
        return replan
                    
    def _clear_planned_result(self):
        self._cmd_plan = None
        self._cmd_idx = 0
        self.waypoints              = []
        self.limited_waypoints      = []
        self.waypoints_velocity     = [] 
        self.waypoints_acceleration = []
        self.waypoints_jerk         = []
        self.waypoints_dt = 0
 
    def _solve_ik(self):
        joint_position = self.get_input_state(PlannerInputStateType.JOINT_POSITION)
        target_tool_position, target_tool_orientation = self.target_arm_tool_position, self.target_arm_tool_orientation
        
        if (self._prev_target_tool_position is None or \
            self._prev_target_tool_orientation is None or \
            not same_array(target_tool_position, self._prev_target_tool_position, atol=self.target_cartesian_position_precision) or \
            not same_quaternion(target_tool_orientation, self._prev_target_tool_orientation, atol=self.target_cartesian_orientation_precision)
        ):
            self.status = CuroboStatus.SOLVING_IK
            self._curobo_planning_status_planning_count = 0
            
            self._update_move_arm_cartesian(target_tool_position, target_tool_orientation, self.arm_tool_position, self.arm_tool_orientation)
            success, solution = self.compute_ik(self.target_arm_tool_position, self.target_arm_tool_orientation, joint_position)
            self.status = CuroboStatus.IK_SUCCEEDED if success else CuroboStatus.IK_FAILED
            if success:
                flatten_solution = self._flatten_list(solution)
                fixed_solution = self._fix_non_moving_arm(flatten_solution, self.arm_joint_position)
                cliped_solution = self._clip_arm_joint_command(fixed_solution)
                if not self._check_target_arm_collision(cliped_solution, "IK target"):
                    print("IK target collision check failed before sending joint command.")
                    print("target tool position: ", target_tool_position)
                    print("target tool orientation: ", target_tool_orientation)
                    print("current arm joint position: ", self.arm_joint_position)
                    print("raw IK solution: ", flatten_solution)
                    print("fixed IK solution: ", fixed_solution)
                    print("clipped IK solution: ", cliped_solution)
                    self.status = CuroboStatus.COLLISION_CHECK_FAILED
                    self._hold_current_arm_command()
                    return
                self.curobo_cmd.arm_joint_command = self._reshape_list(cliped_solution, self.arm_joint_size)
    
    def _get_next_planned_arm_cmd(self):
        """
        Get the next arm command from the current motion plan.
        
        Updates the curobo_cmd with the next joint position command, applying
        joint limit constraints and handling arms that should not move.
        """
        #Get next planned command
        if self._cmd_plan is not None:
            cmd_state = self._cmd_plan[self._cmd_idx]
            cmd_pos = cmd_state.position.cpu().numpy()
            fixed_cmd = self._fix_non_moving_arm(cmd_pos, self._prev_arm_cmd_pos)
            cliped_cmd = self._clip_arm_joint_command(fixed_cmd)
            
            # self.curobo_cmd.arm_joint_size = self.total_arm_joint_size
            self.curobo_cmd.arm_joint_command = self._reshape_list(cliped_cmd, self.arm_joint_size)
            self._prev_arm_cmd_pos = cliped_cmd   
            if self._cmd_idx < len(self._cmd_plan.position) - 1:
                self._cmd_idx += 1
            else:
                self.curobo_cmd.cmd_plan_size = 0
               
    def _gripper_mode_to_joint(self, mode_cmd):
        joint_cmd = [0]*self.total_gripper_joint_size
        current_ind = 0
        for gri_i in range(self.gripper_size):
            n = self.gripper_joint_size[gri_i]
            mode = mode_cmd[gri_i]
            for i in range(n):
                if self.gripper_types[gri_i] == "InspireDexterousHand":
                    joint_cmd[current_ind] = INSPIRE_DEXTEROUS_HAND_COMMAND[mode][i]
                elif self.gripper_types[gri_i] == "UartServoTwoFingerGripper":
                    joint_cmd[current_ind] = TWO_FINGER_GRIPPER_COMMAND[mode][i]
                else:
                    raise ValueError(f"Unknown gripper type: {self.gripper_types[gri_i]}")
                current_ind += 1
        return joint_cmd
    
    def _gripper_normalization(self, joint_cmd, input_normalized:bool, output_normalized:bool):
        if len(joint_cmd) != self.total_gripper_joint_size:
            raise ValueError(f"Expected {self.total_gripper_joint_size} joints, got {joint_cmd}")
        if input_normalized == output_normalized:
            return joint_cmd
        
        out = [0] * self.total_gripper_joint_size
        if not input_normalized:    #normalize cmd
            for i in range(self.total_gripper_joint_size):          
                a, b = self.gripper_soft_joint_limits[:, i]
                out[i] = (joint_cmd[i] - a) / (b - a)
        else:                       #unnormalized cmd
            for i in range(self.total_gripper_joint_size):
                x = np.clip(joint_cmd[i], 0, 1)
                a, b = self.gripper_soft_joint_limits[:, i] 
                out[i] = a + (b - a) * x
        return out
        
    def _gripper_order_convertion(self, joint_cmd, input_urdf_order: bool, output_urdf_order: bool):
        if len(joint_cmd) != self.total_gripper_joint_size:
            raise ValueError(f"Expected {self.total_gripper_joint_size} joints, got {joint_cmd}")
        if input_urdf_order == output_urdf_order:
            return joint_cmd
        
        out = [0] * self.total_gripper_joint_size
        start_ind = 0
        for gri_i in range(self.gripper_size):
            n = self.gripper_joint_size[gri_i]
            if self.gripper_types[gri_i] == "InspireDexterousHand":
                # URDF order: "thumbRotate", "thumbFlex", "indexFlex", "middleFlex", "ringFlex", "littleFlex"
                # Hardware order: "littleFlex", "ringFlex", "middleFlex", "indexFlex", "thumbFlex", "thumbRotate" 
                out[start_ind : start_ind+n] = np.flip(out[start_ind : start_ind+n])
            start_ind += n
        return out
    
    def _gripper_complement(self, joint_cmd, input_normalized: bool, input_zero_open: bool, output_zero_open: bool):
        if len(joint_cmd) != self.total_gripper_joint_size:
            raise ValueError(f"Expected {self.total_gripper_joint_size} joints, got {joint_cmd}")
        if input_zero_open == output_zero_open:
            return joint_cmd
        
        if input_normalized:
            for i in range(self.total_gripper_joint_size):
                joint_cmd[i] = 1.0 - joint_cmd[i]
        else:
            for i in range(self.total_gripper_joint_size):      
                a, b = self.gripper_soft_joint_limits[:, i]
                joint_cmd[i] = b + a - joint_cmd[i]
        return joint_cmd
        
    def _sigint_signal_handler(self, sig, frame):
        """
        Handle SIGINT signal (Ctrl+C) for clean termination.
        
        Args:
            sig: Signal number
            frame: Current stack frame
        """
        print('You pressed Ctrl+C!')
        self.terminate()
        exit(0)

    def _flatten_list(self, array_2d):
        return [x for sublist in array_2d for x in sublist]
    
    def _reshape_list(self, array_1d, shape):
        reshaped = []
        current_index = 0
        for length in shape:
            reshaped.append(array_1d[current_index:current_index + length])
            current_index += length
        return reshaped
    
    def _fix_non_moving_arm(self, cmd, prev_cmd) -> Union[List[float], np.ndarray]:
        if self._move_arm is None or prev_cmd is None:
            return cmd
        
        fixed_cmd = cmd.copy()
        start_idx = 0
        for i in range(self.target_size):
            l = self.arm_joint_size[i]
            if (self._move_arm[i] == False):
                fixed_cmd[start_idx:start_idx+l] = prev_cmd[start_idx:start_idx+l]
            start_idx += l
        return fixed_cmd
    
    def _clip_arm_joint_command(self, cmd) -> Union[List[float], np.ndarray]:
        if self.arm_soft_joint_limits is None:
            return cmd
        
        # Joint position need to be smaller and not equal to the joint_limit for passing the joint limit test,
        # however the planner would generate commands that are exactly at the joint limit.
        clipped_cmd = cmd.copy()
        for i in range(len(clipped_cmd)):
            clipped_cmd[i] = np.clip(clipped_cmd[i], self.arm_soft_joint_limits[0][i], self.arm_soft_joint_limits[1][i])
        return clipped_cmd
