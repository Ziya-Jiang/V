"""Actual planner method regression without importing CUDA or opening sockets."""
import ast
import copy
import contextlib
import io
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation as RotationR
from RobotRAM.controllor import hkstp_arm
from cuarm_utils import same_array,same_quaternion,remove_tool_offset
from cuarm_state import CuroboStatus,RobotStatus,TargetCommandArmCommandType,TargetCommandGripperCommandType

SCRIPTS=Path(__file__).resolve().parents[1]/'dual_v2_2_control/cuarm_upper_planner/scripts'
sys.path.insert(0,str(SCRIPTS))
from v4_2_cartesian_precision import ACTION as V42_ACTION,is_precision_action,arrival_position_m,solver_precision,endpoint_report,configure_plan
from v4_2_x_path import right_x_objective, validate_path as validate_v42_x_path
from servo_cartesian_policy_v3_1 import is_book_servo,poses_close
from v4_1_planner_direct_servo import VISUAL_ACTIONS

tree=ast.parse((SCRIPTS/'curobo_planner_v4_1.py').read_text())
original=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CuroboPlanner')
names={'_cartesian_target_changed','_update_move_arm_cartesian','_update_curobo_state','_plan_cartesian_candidate','_check_v42_endpoint','_check_v42_x_path'}
cls=ast.ClassDef(name='Planner',bases=[],keywords=[],body=[n for n in original.body if isinstance(n,ast.FunctionDef) and n.name in names],decorator_list=[])
ns=dict(globals(),Pose=NS)
exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'<V4.1 actual methods>','exec'),ns)
Planner=ns['Planner']

def planner(action=V42_ACTION,angle=2.,distance=0.):
    p=Planner();p.target_description=action;p.arm_size=2
    p.arm_tool_position=np.array([[.03,1.17,.38],[.411,-.228,-.011]])
    p.arm_tool_orientation=np.array([[1.,0,0,0],[1.,0,0,0]])
    p.target_arm_tool_position=p.arm_tool_position.copy();p.target_arm_tool_position[1,1]+=distance
    p.target_arm_tool_orientation=p.arm_tool_orientation.copy()
    p.target_arm_tool_orientation[1]=RotationR.from_euler('z',angle,degrees=True).as_quat()[[3,0,1,2]]
    p._prev_target_tool_position=p.arm_tool_position.copy();p._prev_target_tool_orientation=p.arm_tool_orientation.copy()
    p.target_cartesian_position_precision=.005;p.target_cartesian_orientation_precision=.1
    return p

class Tests(unittest.TestCase):
    def test_two_degree_probe_requires_plan_and_right_arm_motion(self):
        p=planner();self.assertTrue(p._cartesian_target_changed())
        p._update_move_arm_cartesian(p.target_arm_tool_position,p.target_arm_tool_orientation,p.arm_tool_position,p.arm_tool_orientation)
        self.assertEqual(p._move_arm,[False,True])
        self.assertFalse(planner('move_down_to_grasp')._cartesian_target_changed())
    def test_small_translation_replans_and_duplicate_does_not(self):
        self.assertTrue(planner(angle=0,distance=.0001)._cartesian_target_changed())
        p=planner(angle=0);p.target_arm_tool_orientation*=-1
        self.assertFalse(p._cartesian_target_changed())
    def test_not_routed_to_fixed_z_direct_servo(self):
        self.assertFalse(is_book_servo(V42_ACTION));self.assertNotIn(V42_ACTION,VISUAL_ACTIONS)
        method=next(n for n in original.body if isinstance(n,ast.FunctionDef) and n.name=='_motion_gen_plan_single')
        source=ast.unparse(method)
        self.assertIn("self.target_description == 'book_v3_1_stream'",source)
        self.assertNotIn('is_precision_action',source)
    def status(self,p,status=CuroboStatus.PLANNING_SUCCEEDED,age=0.):
        p._refresh_feedback_pose=Mock();p._first_robot_state_received=True
        p.last_received_robot_state=NS(status=RobotStatus.AVAILABLE)
        p.curobo_state=NS(arm_joint_position=np.zeros((2,7)));p.status=status
        p._curobo_planning_status_planning_count=3;p._curobo_planning_status_planning_threshold=3
        p.last_received_target_command=NS(arm_command_type=TargetCommandArmCommandType.CARTESIAN,
            description=p.target_description,gripper_command_type=TargetCommandGripperCommandType.MODE)
        p._prev_target_arm_joint=np.zeros(14);p.arm_joint_size=[7,7]
        p._reshape_list=lambda a,b:np.asarray(a).tolist()
        p._prev_target_gripper_mode=[];p._prev_target_gripper_joint=[];p.gripper_joint_size=[];p.gripper_size=0
        p._pose_sample_monotonic=time.monotonic()-age;p._panel_feedback_max_age=.25
        p._update_curobo_state();return p.curobo_state.curobo_status
    def test_measured_completion_requires_one_degree(self):
        self.assertEqual(self.status(planner(angle=1.955,distance=.00095)),CuroboStatus.PLANNING_SUCCEEDED)
        self.assertEqual(self.status(planner(angle=.8,distance=.0029)),CuroboStatus.MOTION_FINISHED)
        self.assertEqual(self.status(planner(angle=.8,distance=.0031)),CuroboStatus.PLANNING_SUCCEEDED)
    def test_failure_and_stale_feedback_cannot_become_complete(self):
        self.assertEqual(self.status(planner(angle=0),CuroboStatus.PLANNING_FAILED),CuroboStatus.PLANNING_FAILED)
        self.assertEqual(self.status(planner(angle=0),age=1),CuroboStatus.PLANNING_SUCCEEDED)
    def test_legacy_servo_completion_stays_two_mm(self):
        self.assertEqual(self.status(planner('book_v3_xyz_servo',0,.0025)),CuroboStatus.PLANNING_SUCCEEDED)
        self.assertEqual(self.status(planner('move_down_to_grasp',2,.002)),CuroboStatus.MOTION_FINISHED)
    def motiongen(self):
        return NS(**{name:NS(position_threshold=.005,rotation_threshold=.025) for name in ('ik_solver','trajopt_solver','finetune_trajopt_solver')})
    def test_solver_precision_scoped_and_exception_restored(self):
        mg=self.motiongen()
        with self.assertRaisesRegex(RuntimeError,'solve failed'):
            with solver_precision(mg,V42_ACTION):
                for s in vars(mg).values():self.assertEqual((s.position_threshold,s.rotation_threshold),(.00025,.001))
                raise RuntimeError('solve failed')
        for s in vars(mg).values():self.assertEqual((s.position_threshold,s.rotation_threshold),(.005,.025))
        with solver_precision(mg,'move_down_to_grasp'):self.assertEqual(mg.ik_solver.rotation_threshold,.025)
    def test_v42_time_scale_without_mutating_ordinary_config(self):
        c=NS(time_dilation_factor=.4)
        self.assertEqual(configure_plan(c,V42_ACTION).time_dilation_factor,.9)
        self.assertEqual(configure_plan(c,'move_down_to_grasp').time_dilation_factor,.4)
        self.assertEqual(c.time_dilation_factor,.4)

    def test_actual_plan_call_receives_scoped_precision(self):
        p=planner();p._motion_gen=self.motiongen();p._gpu_lock=contextlib.nullcontext()
        p.end_effector_offset=np.zeros((2,3));p.target_size=2;p.target_link_names=['left','right'];p.ee_link_idx_in_target_link_names=0
        p.tensor_args=NS(to_device=lambda a:a);p._log_plan_wall_time=Mock()
        def solve(*args,**kwargs):
            self.assertEqual(p._motion_gen.ik_solver.rotation_threshold,.001)
            self.assertEqual(p._motion_gen.finetune_trajopt_solver.rotation_threshold,.001)
            return 'result'
        import torch
        cost=NS(run_vec_weight=torch.zeros(6),project_distance_tensor=torch.ones(1))
        p._motion_gen.get_all_pose_rollout_instances=lambda:[NS(_link_pose_costs={'arm_end_effector_r':cost})]
        p._motion_gen.plan_single=solve
        result=p._plan_cartesian_candidate(NS(unsqueeze=lambda i:None),p.target_arm_tool_position,p.target_arm_tool_orientation,NS())
        self.assertEqual(result,'result');self.assertEqual(p._motion_gen.ik_solver.rotation_threshold,.025)
    def test_endpoint_rejects_unchanged_two_degree_probe(self):
        p=planner();r=endpoint_report(p.arm_tool_position[1:],p.arm_tool_orientation[1:],p.target_arm_tool_position[1:],p.target_arm_tool_orientation[1:])
        self.assertFalse(r['accepted']);self.assertAlmostEqual(r['orientation_error_deg'],2.)
        p=planner(angle=.1,distance=.0002)
        self.assertTrue(endpoint_report(p.arm_tool_position[1:],p.arm_tool_orientation[1:],p.target_arm_tool_position[1:],p.target_arm_tool_orientation[1:])['accepted'])
    def test_endpoint_gate_includes_tool_offset(self):
        p=planner(angle=0);p.target_link_names=['left','arm_end_effector_r'];p._gpu_lock=contextlib.nullcontext()
        p.reorder_input=lambda q,**kw:q;p.tensor_args=NS(to_device=lambda a:a)
        p.end_effector_offset=np.array([[0,0,0],[0,0,.05]])
        def tensor(a):return NS(cpu=lambda:NS(numpy=lambda:np.array(a)))
        out=NS(position=tensor(p.target_arm_tool_position[1:]-[0,0,.05]),quaternion=tensor([[1,0,0,0]]))
        p._path_kinematics=NS(get_link_poses=lambda *a,**kw:out)
        with contextlib.redirect_stdout(io.StringIO()):self.assertTrue(p._check_v42_endpoint(np.zeros(14)))
        p.target_arm_tool_position[1,0]+=.002
        with contextlib.redirect_stdout(io.StringIO()):self.assertFalse(p._check_v42_endpoint(np.zeros(14)))
    def test_endpoint_rejection_precedes_publication(self):
        method=next(n for n in original.body if isinstance(n,ast.FunctionDef) and n.name=='_retrieve_motion_gen_result')
        source=ast.unparse(method)
        self.assertLess(source.index('self._check_v42_endpoint(final_waypoint)'),source.index('self.curobo_cmd.waypoints ='))
        self.assertIn('self._hold_current_arm_command()',source[source.index('self._check_v42_endpoint(final_waypoint)'):])
    def test_diagnostic_threshold_matches_runtime(self):
        self.assertEqual(hkstp_arm._position_tolerance_for_task('book',V42_ACTION),.003)
        self.assertEqual(hkstp_arm._position_tolerance_for_task('book','book_v3_xyz_servo'),.002)

if __name__=='__main__':unittest.main()
