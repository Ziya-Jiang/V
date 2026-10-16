"""Offline timing-policy and watchdog regressions; no GPU or sockets."""
import ast
import copy
import contextlib
import io
from pathlib import Path
import sys
import threading
from dataclasses import dataclass
from importlib.util import find_spec
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import numpy as np

SCRIPTS=Path(__file__).resolve().parents[1]/'dual_v2_2_control/cuarm_upper_planner/scripts'
sys.path.insert(0,str(SCRIPTS))
from v4_1_planning_time import (trajectory_timing, SOLVER_TIMEOUT_SECONDS,
    SOLVER_MAX_ATTEMPTS, PLANNING_HOLD_SECONDS)
import test_v4_1_panel_transport as transport_tests
from v4_2_cartesian_precision import solver_precision,configure_plan


def installed_plan_config():
    # motion_gen imports a CUDA Pose as a function default. Extract its real
    # pure configuration class so these tests run on a CPU-only workstation.
    path=Path(find_spec('curobo.wrap.reacher.motion_gen').origin)
    tree=ast.parse(path.read_text())
    node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MotionGenPlanConfig')
    future=ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0)
    ns=dict(__name__=__name__,dataclass=dataclass)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[future,node],type_ignores=[])),str(path),'exec'),ns)
    return ns['MotionGenPlanConfig']


class PlanningTimeTests(unittest.TestCase):
    def test_stricter_acceleration_gets_longer_seed_inside_evaluator_range(self):
        row=dict(model_velocity_rad_s=3.14,velocity_rad_s=np.deg2rad(100),
                 model_acceleration_rad_s2=15.,acceleration_rad_s2=np.deg2rad(100))
        timing=trajectory_timing([row],10,.02,.2,7.)
        self.assertAlmostEqual(timing['trajopt_dt'],.15*np.sqrt(15/np.deg2rad(100)))
        self.assertEqual(timing['trajopt_dt'],timing['js_trajopt_dt'])
        self.assertGreaterEqual(timing['trajopt_dt'],timing['minimum_trajectory_dt'])
        self.assertLessEqual(timing['trajopt_dt'],timing['maximum_trajectory_dt'])
        self.assertAlmostEqual(timing['maximum_trajectory_dt'],.7)

    def test_initial_dt_never_below_minimum_or_above_maximum(self):
        self.assertEqual(trajectory_timing([],10,.02,.2,7.)['trajopt_dt'],.2)
        row=dict(model_velocity_rad_s=100.,velocity_rad_s=.1,
                 model_acceleration_rad_s2=100.,acceleration_rad_s2=.1)
        self.assertEqual(trajectory_timing([row],10,.02,.2,7.)['trajopt_dt'],.7)
        for args in [(10,.02,.01,7.),(10,.02,.8,7.),(10,.02,.2,float('nan')),(5,.02,.2,7.)]:
            with self.subTest(args=args),self.assertRaises(ValueError):trajectory_timing([],*args)

    def test_production_constructor_passes_same_dt_to_optimizer_and_evaluator(self):
        tree=ast.parse((SCRIPTS/'curobo_planner_v4_1.py').read_text())
        init=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='__init__')
        timing=trajectory_timing([],10,.02,.2,7.)
        assignments={n.targets[0].id:n for n in ast.walk(init) if isinstance(n,ast.Assign)
                     and isinstance(n.targets[0],ast.Name)}
        captured=[]
        ns={name.id:None for name in ast.walk(init) if isinstance(name,ast.Name)}
        ns.update(timing=timing, robot_cfg_instance=NS(kinematics=NS(dof=21)),
            self=NS(world_cfg={},tensor_args=None,trim_steps=[2,-2]),
            CollisionCheckerType=NS(MESH=1),
            TrajEvaluatorConfig=NS(from_basic=lambda **k:captured.append(k) or k),
            MotionGenConfig=NS(load_from_robot_config=lambda *a,**k:captured.append(k) or k))
        nodes=[assignments['traj_evaluator_config'],assignments['motion_gen_config']]
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'production_timing','exec'),ns)
        evaluator,optimizer=captured
        self.assertEqual(evaluator['min_dt'],optimizer['minimum_trajectory_dt'])
        self.assertEqual(evaluator['max_dt'],optimizer['maximum_trajectory_dt'])
        self.assertEqual(optimizer['trajopt_dt'],optimizer['js_trajopt_dt'])

    def test_production_plan_config_preserves_finetune_and_has_timeout_margin(self):
        MotionGenPlanConfig=installed_plan_config()
        tree=ast.parse((SCRIPTS/'curobo_planner_v4_1.py').read_text())
        node=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and
                  isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr=='_plan_config')
        ns=dict(self=NS(),MotionGenPlanConfig=MotionGenPlanConfig,time_dilation_factor=.4,
                SOLVER_TIMEOUT_SECONDS=SOLVER_TIMEOUT_SECONDS,SOLVER_MAX_ATTEMPTS=SOLVER_MAX_ATTEMPTS)
        exec(compile(ast.Module(body=[node],type_ignores=[]),'production_plan_config','exec'),ns)
        cfg=ns['self']._plan_config
        self.assertEqual(cfg.timeout,6.)
        self.assertEqual(cfg.max_attempts,4)
        self.assertTrue(cfg.enable_finetune_trajopt)
        self.assertEqual(cfg.finetune_dt_scale,1.)
        self.assertGreaterEqual(PLANNING_HOLD_SECONDS-cfg.timeout,4.)

    def test_cartesian_call_cannot_mutate_next_requests_config(self):
        MotionGenPlanConfig=installed_plan_config()
        tree=ast.parse((SCRIPTS/'curobo_planner_v4_1.py').read_text())
        node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_plan_cartesian_candidate')
        import time
        from v4_2_x_path import right_x_objective
        ns=dict(copy=copy,np=np,time=time,Pose=NS,remove_tool_offset=lambda p,q,o:p,solver_precision=solver_precision,configure_plan=configure_plan,right_x_objective=right_x_objective)
        exec(compile(ast.Module(body=[node],type_ignores=[]),'cartesian_plan','exec'),ns)
        seen=[]
        def solve(start,goal,cfg,**kw):
            self.assertFalse(cfg.check_start_validity);seen.append(cfg.finetune_dt_scale);cfg.finetune_dt_scale=9.;return NS(solve_time=.1,status='FAIL')
        p=NS(target_description='move_down_to_grasp',target_size=2,target_link_names=['left','right'],end_effector_offset=np.zeros((2,3)),
             tensor_args=NS(to_device=lambda x:x),ee_link_idx_in_target_link_names=0,
             _gpu_lock=threading.Lock(),_motion_gen=NS(plan_single=solve),_log_plan_wall_time=Mock())
        cfg=MotionGenPlanConfig(finetune_dt_scale=1.,timeout=6.,check_start_validity=False)
        for _ in range(2):ns['_plan_cartesian_candidate'](p,NS(unsqueeze=lambda n:None),np.zeros((2,3)),np.ones((2,4)),cfg)
        self.assertEqual(seen,[1.,1.]);self.assertEqual(cfg.finetune_dt_scale,1.)
        self.assertEqual(p._log_plan_wall_time.call_count,2)

    def test_failure_return_within_budget_transitions_to_live_hold(self):
        harness=transport_tests.TransportTests();harness.setUp();t=harness.t
        t.begin_planning()
        for _ in range(32):harness.advance(.2)  # Slow failure returns at 6.4s.
        self.assertIsNone(t.fault)
        t.idle_hold()
        self.assertEqual(t.mode,'idle_hold')
        self.assertEqual(t.command.arm_waypoint_size,0)
        self.assertEqual(t.deadline,harness.now+t.PRODUCER_LEASE)

    def test_single_call_hang_still_times_out_and_reports_wall_time(self):
        harness=transport_tests.TransportTests();harness.setUp();t=harness.t
        t.begin_planning()
        for _ in range(51):harness.advance(.2)
        self.assertIn('planning_wall_seconds=',t.fault)
        self.assertIn('mode=planning_hold',t.fault)
        self.assertIsNone(t.command)
        with self.assertRaises(RuntimeError):t.idle_hold()


if __name__=='__main__':unittest.main()
