"""Offline execution-limit regressions. No UDP, camera or robot commands."""
import ast
import contextlib
import copy
import io
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
import torch

SCRIPTS = Path(__file__).resolve().parents[1] / 'dual_v2_2_control/cuarm_upper_planner/scripts'
sys.path.insert(0, str(SCRIPTS))
from v4_1_joint_limits import constrain_robot_config, checked_joint_command, execution_limits


def fixture():
    # Deliberately interleave gripper and reverse arm order.
    names = ['J7', 'finger', 'J4']
    bounds = torch.tensor(np.deg2rad([[-360., 0., -40.], [360., 10., 145.]]))
    limits = SimpleNamespace(joint_names=names, position=bounds,
                            velocity=torch.tensor([[-4.] * 3, [4.] * 3]),
                            acceleration=torch.tensor([[-10.] * 3, [10.] * 3]))
    kin = SimpleNamespace(get_joint_limits=lambda: limits,
                          cspace=SimpleNamespace(max_acceleration=torch.full((3,), 10.)))
    soft = dict(position=[[-40., -300.], [140., 140.]], velocity=[150., 150.], acceleration=[150., 150.])
    config = dict(robot=dict(arm=[dict(joint_size=2, urdf_joints=['J4', 'J7'],
        safety=dict(joint_soft_limit=soft, joint_hard_limit=dict(position=[[-45.,-365.],[150.,365.]],
        velocity=[180.,180.]), joint_jump=dict(position=[150.,150.], velocity=[150.,150.])))]))
    return SimpleNamespace(kinematics=kin), limits, config


class ExecutionLimitTests(unittest.TestCase):
    def test_name_mapping_intersection_and_dynamic_limits(self):
        robot, limits, config = fixture()
        old = limits.position.clone()
        raw, effective, report = constrain_robot_config(robot, config, ['J4','J7'], .99)
        np.testing.assert_allclose(np.rad2deg(effective), [[-39.6,-300.],[140.,140.]], atol=1e-4)
        np.testing.assert_allclose(raw, old[:, [2,0]].numpy())
        np.testing.assert_allclose(limits.position[:,1], old[:,1])
        self.assertAlmostEqual(limits.velocity[1,2].item(), np.deg2rad(150), places=6)
        self.assertAlmostEqual(limits.acceleration[1,0].item(), np.deg2rad(150), places=6)
        self.assertEqual([r['joint'] for r in report], ['J4','J7'])
        np.testing.assert_allclose(robot.kinematics.cspace.max_acceleration, limits.acceleration[1])

    def test_logged_141_degree_goal_rejected_never_clipped(self):
        robot, limits, config = fixture()
        _, bounds, _ = constrain_robot_config(robot, config, ['J4','J7'], .99)
        q = np.array([2.475858211517334, .7537801265716553]); original=q.copy()
        with self.assertRaisesRegex(ValueError, 'J4.*141.856.*not clipped'):
            checked_joint_command(q, bounds, ['J4','J7'])
        np.testing.assert_array_equal(q, original)
        q[0]=np.deg2rad(140.)
        np.testing.assert_array_equal(checked_joint_command(q,bounds,['J4','J7']),q)

    def test_intermediate_violation_not_only_endpoint(self):
        robot, _, config = fixture()
        _, bounds, _ = constrain_robot_config(robot, config, ['J4','J7'], .99)
        q=np.deg2rad([[130.,0.],[141.,0.],[139.,0.]])
        with self.assertRaisesRegex(ValueError,'J4'):
            checked_joint_command(q,bounds,['J4','J7'])

    def test_invalid_configs_fail_before_tensor_mutation(self):
        for kind in ('missing','duplicate','nan','empty','units_shape'):
            robot, limits, config=fixture(); old=limits.position.clone()
            arm=config['robot']['arm'][0]
            if kind=='missing': arm['urdf_joints'][1]='absent'
            if kind=='duplicate': arm['urdf_joints'][1]='J4'
            if kind=='nan': arm['safety']['joint_soft_limit']['position'][0][0]=float('nan')
            if kind=='empty': arm['safety']['joint_soft_limit']['position'][0][0]=160.
            if kind=='units_shape': arm['safety']['joint_soft_limit']['position']=[1,2]
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                constrain_robot_config(robot,config,['J4','J7'],.99)
            np.testing.assert_array_equal(limits.position,old)

    def test_raw_nan_and_wrong_order_shape_rejected(self):
        for q in ([float('nan'),0.],[1.],[float('inf'),0.]):
            with self.assertRaises(ValueError): checked_joint_command(q,[[-1,-1],[1,1]],['a','b'])

    def test_ik_factory_uses_constrained_config(self):
        tree=ast.parse((SCRIPTS/'curobo_ik_solver_v4_1.py').read_text())
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef))
        loader=Mock(return_value='ik_config'); constructor=Mock()
        ns=dict(CuroboConfig=object,IKSolverConfig=SimpleNamespace(load_from_robot_config=loader),
                IKSolver=constructor,TensorDeviceType=lambda:None,CollisionCheckerType=SimpleNamespace(MESH=1))
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'ik_factory','exec'),ns)
        constrained=object(); cfg=SimpleNamespace(robot_cfg={'old':'model'},world_cfg={})
        ns['create_ik_solver'](cfg,robot_config=constrained)
        self.assertIs(loader.call_args.args[0],constrained)
        ns['create_ik_solver'](cfg)
        self.assertIs(loader.call_args.args[0],cfg.robot_cfg)  # Old planner unchanged.

    def test_actual_motiongen_result_rejected_and_command_held(self):
        from test_v3_1_stream import StreamTests
        p,result=StreamTests().interval_result(.2)
        p.arm_soft_joint_limits=np.deg2rad(np.array([[-360.]*14,[360.]*14]))
        p.arm_soft_joint_limits[1,10]=np.deg2rad(140.)
        # Reuse only the fixture; bind the new implementation under test.
        from types import MethodType
        tree=ast.parse((SCRIPTS/'curobo_planner_v4_1.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        names={'_retrieve_motion_gen_result','_clip_arm_joint_command','_accept_arm_joint_path'}
        nodes=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
        ns=dict(p._retrieve_motion_gen_result.__func__.__globals__,checked_joint_command=checked_joint_command)
        ns.update(Union=__import__('typing').Union,List=list)
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'v41_actual_result','exec'),ns)
        for name in names:setattr(p,name,MethodType(ns[name],p))
        result.optimized_plan.position[2,10]=2.475858211517334
        with contextlib.redirect_stdout(io.StringIO()): self.assertFalse(p._retrieve_motion_gen_result(result))
        p._hold_current_arm_command.assert_called_once()
        p._check_target_arm_collision.assert_not_called()
        from cuarm_state import CuroboStatus
        self.assertEqual(p.status,CuroboStatus.PLANNING_FAILED)

    def test_ik_result_reordered_by_joint_name_excludes_gripper(self):
        import test_v3_planner_microsteps  # Register protocol module paths, no sockets.
        from cuarm_state import PlannerCuroboSolverType
        from curobo.types.state import JointState
        tree=ast.parse((SCRIPTS/'curobo_planner_v4_1.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='compute_ik')
        ns=dict(IKResult=object,Pose=SimpleNamespace,PlannerCuroboSolverType=PlannerCuroboSolverType,
                remove_tool_offset=lambda p,q,o:p)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'compute_ik','exec'),ns)
        result=SimpleNamespace(success=torch.tensor(True),solve_time=.01,
            js_solution=JointState(position=torch.tensor([[.2,.9,.1]]),joint_names=['right','finger','left']))
        p=SimpleNamespace(end_effector_offset=np.zeros((2,3)),tensor_args=SimpleNamespace(to_device=torch.as_tensor),
            ee_link_idx_in_target_link_names=0,target_size=2,target_link_names=['ee_l','ee_r'],
            _gpu_lock=threading.Lock(),_solver_type=PlannerCuroboSolverType.IK_SOLVER,
            _ik_solver=SimpleNamespace(solve_single=lambda *a,**k:result),
            total_arm_joint_names=['left','right'],arm_joint_size=[1,1],
            _reshape_list=lambda q,s:np.asarray(q).reshape(2,1))
        with contextlib.redirect_stdout(io.StringIO()):
            success,solution=ns['compute_ik'](p,np.zeros((2,3)),np.array([[1.,0,0,0]]*2),[0.,0.,0.])
        self.assertTrue(success)
        np.testing.assert_allclose(solution,[[.1],[.2]])

    def test_direct_servo_also_inherits_execution_dynamic_caps(self):
        from v4_1_planner_direct_servo import DirectVisualServo, load_settings
        settings=load_settings(); before=copy.deepcopy(settings)
        p=SimpleNamespace(_v31_bridge=SimpleNamespace(tcp=lambda q:None),
            arm_joint_size=[7,7],arm_soft_joint_limits=np.array([[-3.]*14,[3.]*14]),
            execution_limit_report=[dict(velocity_rad_s=.1,acceleration_rad_s2=.2) for _ in range(14)])
        servo=DirectVisualServo(p,None,settings)
        self.assertEqual(servo.c['joint_speed_rad_s'],.1)
        self.assertEqual(servo.c['joint_acceleration_rad_s2'],.2)
        self.assertEqual(settings,before)

    def test_actual_config_all_fourteen_execution_constraints(self):
        from curobo_config import CuroboConfig
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import RobotConfig
        path=Path(__file__).resolve().parents[1]/'dual_v2_2_configuration/config.yaml'
        c=CuroboConfig.load_from_yaml_path(path)
        r=RobotConfig.from_dict(c.robot_cfg,TensorDeviceType(device=torch.device('cpu')))
        _,bounds,rows=constrain_robot_config(r,c.config,c.total_arm_joint_names,.99)
        execution=execution_limits(c.config)
        for i,name in enumerate(c.total_arm_joint_names):
            self.assertGreaterEqual(bounds[0,i]+1e-6,execution[name][0][0])
            self.assertLessEqual(bounds[1,i]-1e-6,execution[name][0][1])
        self.assertEqual(len(rows),14)
        for name in ['arm_J4_l','arm_J4_r','arm_J7_l','arm_J7_r']:
            self.assertAlmostEqual(np.rad2deg(bounds[1,c.total_arm_joint_names.index(name)]),140.,places=3)


if __name__ == '__main__': unittest.main()
