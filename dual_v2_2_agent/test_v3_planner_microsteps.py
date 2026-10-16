"""Exercise actual planner methods without importing CUDA or opening UDP."""
import ast
import contextlib
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import numpy as np
from RobotRAM.controllor import hkstp_arm  # Registers canonical protocol paths only.
from cuarm_state import (CuroboStatus, RobotStatus, TargetCommandArmCommandType,
    TargetCommandGripperCommandType, PlannerCuroboSolverType, PlannerInputStateType)
from cuarm_utils import same_array, same_quaternion, remove_tool_offset


SCRIPTS=Path(__file__).resolve().parents[1]/'dual_v2_2_control/cuarm_upper_planner/scripts'
spec=importlib.util.spec_from_file_location('servo_policy_test',SCRIPTS/'servo_cartesian_policy.py')
policy=importlib.util.module_from_spec(spec);spec.loader.exec_module(policy)
tree=ast.parse((SCRIPTS/'curobo_planner.py').read_text())
original=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CuroboPlanner')
names={'_cartesian_target_changed','_update_move_arm_cartesian','_motion_gen_plan_single','_update_curobo_state'}
cls=ast.ClassDef(name='Planner',bases=[],keywords=[],body=[n for n in original.body if isinstance(n,ast.FunctionDef) and n.name in names],decorator_list=[])
ns=dict(globals(),is_book_servo=policy.is_book_servo,poses_close=policy.poses_close)
exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(SCRIPTS/'curobo_planner.py'),'exec'),ns)
Planner=ns['Planner']


def planner(delta=.002,action='book_v3_xyz_servo'):
    p=Planner();p.target_description=action
    p.arm_tool_position=np.array([[.03,1.17,.38],[.518,.041,-.011]])
    p.arm_tool_orientation=np.array([[1.,0,0,0],[1.,0,0,0]])
    p.target_arm_tool_position=p.arm_tool_position.copy();p.target_arm_tool_position[1,1]-=delta
    p.target_arm_tool_orientation=p.arm_tool_orientation.copy()
    p._prev_target_tool_position=p.arm_tool_position.copy()
    p._prev_target_tool_orientation=p.arm_tool_orientation.copy()
    p.target_cartesian_position_precision=.005;p.target_cartesian_orientation_precision=.1
    p.arm_size=p.target_size=2
    return p


class PlannerTests(unittest.TestCase):
    def test_logged_two_mm_and_sub_mm_corrections_trigger_replan(self):
        for delta in (.002,.0005,.00003):
            p=planner(delta);self.assertTrue(p._cartesian_target_changed())
            p._update_move_arm_cartesian(p.target_arm_tool_position,p.target_arm_tool_orientation,
                                        p.arm_tool_position,p.arm_tool_orientation)
            self.assertEqual(p._move_arm,[False,True])

    def test_duplicate_and_quaternion_sign_do_not_replan(self):
        p=planner(0.);p.target_arm_tool_orientation*=-1
        self.assertFalse(p._cartesian_target_changed())

    def test_ordinary_action_preserves_original_five_mm_policy(self):
        self.assertFalse(planner(.002,'move_down_to_grasp')._cartesian_target_changed())
        self.assertTrue(planner(.006,'move_down_to_grasp')._cartesian_target_changed())

    def test_unrequested_left_arm_remains_fixed(self):
        p=planner();p.target_arm_tool_position[0,0]+=.0005
        p._update_move_arm_cartesian(p.target_arm_tool_position,p.target_arm_tool_orientation,
                                    p.arm_tool_position,p.arm_tool_orientation)
        self.assertEqual(p._move_arm,[False,True])

    def test_motiongen_branch_is_entered_instead_of_reusing_old_plan(self):
        p=planner();p._solver_type=PlannerCuroboSolverType.MOTION_GEN
        p.get_input_state=Mock(return_value=np.zeros(14));p.end_effector_offset=np.zeros((2,3))
        p._update_move_arm_cartesian=Mock(side_effect=RuntimeError('entered new planning branch'))
        p._get_next_planned_arm_cmd=Mock()
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError,'entered new planning'):
            p._motion_gen_plan_single()
        p._get_next_planned_arm_cmd.assert_not_called()

    def status(self,p,status=CuroboStatus.PLANNING_SUCCEEDED):
        p._use_cuda_graph=False;p.arm_joint_position=np.zeros(14)
        p._compute_fk_without_lock=lambda q:(p.arm_tool_position,p.arm_tool_orientation)
        p._first_robot_state_received=True;p.curobo_state=SimpleNamespace()
        p.last_received_robot_state=SimpleNamespace(arm_joint_position=np.zeros((2,7)),
            arm_joint_velocity=np.zeros((2,7)),gripper_joint_position=[],status=RobotStatus.AVAILABLE)
        p._curobo_planning_status_planning_count=3;p._curobo_planning_status_planning_threshold=3
        p.status=status;p.last_received_target_command=SimpleNamespace(
            arm_command_type=TargetCommandArmCommandType.CARTESIAN,description=p.target_description,
            gripper_command_type=TargetCommandGripperCommandType.MODE)
        p._prev_target_arm_joint=np.zeros(14);p.arm_joint_size=[7,7]
        p._reshape_list=lambda a,b:np.asarray(a).tolist()
        p._prev_target_gripper_mode=[];p._prev_target_gripper_joint=[];p.gripper_joint_size=[];p.gripper_size=0
        p._update_curobo_state();return p.curobo_state.curobo_status

    def test_over_two_mm_not_finished_but_real_arrival_can_finish(self):
        self.assertEqual(self.status(planner(.0021)),CuroboStatus.PLANNING_SUCCEEDED)
        self.assertEqual(self.status(planner(.0015)),CuroboStatus.MOTION_FINISHED)
        self.assertEqual(self.status(planner(.002,'move_down_to_grasp')),CuroboStatus.MOTION_FINISHED)

    def test_failure_is_never_overwritten_by_arrival(self):
        self.assertEqual(self.status(planner(0.),CuroboStatus.PLANNING_FAILED),CuroboStatus.PLANNING_FAILED)


if __name__=='__main__':unittest.main()
