"""Offline direct-control limits, routing and fail-closed transition tests."""
import ast
import copy
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation

SCRIPTS=Path(__file__).resolve().parents[1]/'dual_v2_2_control/cuarm_upper_planner/scripts'
sys.path.insert(0,str(SCRIPTS))
sys.path.insert(0,str(SCRIPTS.parents[2]/'dual_v2_2_configuration/robot_platform_utils/python/scripts'))
from v3_1_direct_servo import DifferentialStep,DirectVisualServo,CpuCollision,load_settings
from v3_1_execution import ExecutionGuard
from cuarm_state import CuroboStatus,TargetCommandArmCommandType,TargetCommandPlannerMode,PlannerCuroboSolverType


def tcp(q):
    q=np.atleast_2d(q);p=np.zeros((len(q),2,3));p[:,1]=q[:,7:10]*.3
    r=np.tile([1.,0,0,0],(len(q),2,1));r[:,1]=Rotation.from_rotvec(q[:,10:13]).as_quat()[:,[3,0,1,2]]
    return p,r


class DirectTests(unittest.TestCase):
    def fixture(self):
        self.t=10.;self.q=np.zeros(14);self.sent=[]
        self.g=ExecutionGuard(self.sent.append,clock=lambda:self.t,log=lambda _:None)
        def sample(**kw):
            p,r=tcp(self.q)
            return dict(joints=self.q.copy(),velocity=np.zeros(14),position=p[0],orientation=r[0],stamp=self.t)
        self.b=NS(tcp=tcp,guard=self.g,snapshot=sample,serial=0)
        self.p=NS(_v31_bridge=self.b,arm_soft_joint_limits=np.array([[-2.]*14,[2.]*14]),
            target_arm_tool_position=np.array([[0.,0,0],[.02,0,0]]),
            target_arm_tool_orientation=np.array([[1.,0,0,0]]*2),
            target_description='book_v3_1_stream',gripper_joint_position=[],
            reorder_input=lambda q,*args,**kw:q,curobo_cmd=NS(),set_gripper_command=Mock())
        self.collision=NS(check=Mock(return_value=True))
        self.d=DirectVisualServo(self.p,self.collision)

    def tick(self,receipt=None,accept=True):
        with patch('time.monotonic',return_value=self.t):self.d.compute(self.t if receipt is None else receipt)
        if accept:
            c=self.p.curobo_cmd
            c.arm_target_mode=2;c.gripper_joint_command=[[1.],[0.]]
            self.assertTrue(self.g.publish(c,self.b.checked_start,self.b.serial,requested=self.b.requested_target))
        self.d.dispatched(accept)

    def test_limits_acceleration_convergence_and_left_arm_hold(self):
        self.fixture();vel=np.zeros(7)
        for _ in range(250):
            self.tick()
            np.testing.assert_array_less(np.abs(self.d.stepper.velocity),self.d.c['joint_speed_rad_s']+1e-9)
            self.assertLessEqual(np.max(np.abs(self.d.stepper.velocity-vel)),.020001)
            vel=self.d.stepper.velocity.copy();self.q=self.d.stepper.command.copy();self.t+=.02
        self.assertLess(np.linalg.norm(tcp(self.q)[0][0,1]-self.p.target_arm_tool_position[1]),.002)
        np.testing.assert_array_equal(self.q[:7],0.)
        self.assertEqual(self.p.curobo_cmd.waypoints_size,0)

    def test_visual_lease_holds_then_fresh_target_resumes(self):
        self.fixture();self.tick();self.q=self.d.stepper.command.copy()
        for _ in range(22):self.t+=.02;self.tick(receipt=10.)
        self.assertTrue(self.d.paused)
        held=self.d.stepper.command.copy();self.q[7]+=.001
        self.t+=.02;self.tick(receipt=10.)
        np.testing.assert_array_equal(self.d.stepper.command,held)
        self.t+=.02;self.tick();self.assertFalse(self.d.paused)

    def test_collision_and_height_failure_produce_no_new_output(self):
        for kind in ('collision','height','deadline','plane'):
            with self.subTest(kind=kind):
                self.fixture();self.tick();count=len(self.sent);self.t+=.02
                if kind=='collision':self.collision.check.return_value=False
                if kind=='height':self.q[9]=.02
                if kind=='deadline':self.t+=.2
                if kind=='plane':self.p.target_arm_tool_position[1,2]=.004
                with self.assertRaises(ValueError):self.tick()
                self.assertFalse(self.d.output_ready)
                # A height fault may send the measured hold, never a new target.
                if kind=='height':self.assertIsNotNone(self.g.fault)
                else:self.assertEqual(len(self.sent),count)

    def test_rejected_dispatch_does_not_advance_integrator(self):
        self.fixture();self.tick(accept=False)
        self.assertIsNone(self.d.stepper.command)
        np.testing.assert_array_equal(self.d.stepper.velocity,0.)

    def test_z_backoff_reduces_accumulated_lead_and_checks_selected_path(self):
        self.fixture();self.tick();self.t+=.02
        proposal=self.q.copy();proposal[7]=.02;proposal[9]=.016
        self.d.stepper.step=Mock(return_value=(proposal,np.ones(7)*.1,None))
        self.tick()
        # 4.8 mm -> 2.4 mm rejected -> 1.2 mm accepted, including old lead.
        np.testing.assert_allclose(self.d.stepper.command[7:],proposal[7:]*.25)
        np.testing.assert_array_equal(self.d.stepper.velocity,0.)
        path=self.collision.check.call_args.args[0]
        self.assertLessEqual(np.max(np.abs(tcp(path)[0][:,1,2])),.002)
        np.testing.assert_array_equal(path[-1],self.d.stepper.command)
        self.assertIsNone(self.g.fault)

    def test_z_backoff_does_not_bypass_collision(self):
        self.fixture();self.tick();self.t+=.02;count=len(self.sent)
        proposal=self.q.copy();proposal[9]=.016
        self.d.stepper.step=Mock(return_value=(proposal,np.zeros(7),None))
        self.collision.check.return_value=False
        with self.assertRaisesRegex(ValueError,'collision'):self.tick()
        self.assertEqual(len(self.sent),count)

    def test_no_feasible_z_step_holds_then_times_out_with_details(self):
        self.fixture();self.q[9]=.0025/.3
        snapshot=self.b.snapshot();proposal=self.q.copy();proposal[9]+=.025
        result,velocity,_=self.d.bounded_z_step(snapshot,proposal,np.ones(7),0.,self.t)
        np.testing.assert_array_equal(result,self.q);np.testing.assert_array_equal(velocity,0.)
        self.q[7]+=.0001;snapshot=self.b.snapshot()
        held,_,_=self.d.bounded_z_step(snapshot,proposal,np.ones(7),0.,self.t+.5)
        np.testing.assert_array_equal(held,result)
        with self.assertRaisesRegex(ValueError,'Z backoff exhausted:.*predicted_mm=.*limit_mm='):
            self.d.bounded_z_step(snapshot,proposal,np.ones(7),0.,self.t+1.01)

    def test_joint_lead_limit_preserves_coordinated_direction(self):
        c=load_settings();step=DifferentialStep(tcp,np.array([[-2.]*14,[2.]*14]),c)
        q=np.zeros(14);step.command=q.copy();step.command[7:9]=[.1,.04]
        out,_,_=step.step(q,np.array([0.,0,0,1.,0,0,0]),.02)
        np.testing.assert_allclose(out[7:9],[.025,.01])

    def test_height_recovery_precedes_forward_and_keeps_original_request(self):
        self.fixture();self.tick();self.t+=.02
        old_command=self.d.stepper.command.copy()
        self.q[9]=.00204/.3  # reproduce the 2.04 mm boundary in the field log
        self.p.target_arm_tool_position[1,2]=.0009  # old post-loss rebase is ignored for control
        original=self.p.target_arm_tool_position.copy()
        self.tick()
        self.assertTrue(self.d.recovering_z)
        np.testing.assert_array_equal(self.d.z_recovery_goal[:2],tcp(self.q)[0][0,1,:2])
        self.assertEqual(self.d.z_recovery_goal[2],0.)
        np.testing.assert_array_equal(self.b.requested_target[:,:3],original)
        first_x=self.d.stepper.command[7]
        self.assertAlmostEqual(first_x,old_command[7])
        for _ in range(80):
            self.q=self.d.stepper.command.copy();self.t+=.02;self.tick()
        self.assertFalse(self.d.recovering_z)
        self.assertLess(abs(tcp(self.q)[0][0,1,2]),.00075)
        self.assertGreater(self.d.stepper.command[7],first_x)
        self.assertIsNone(self.g.fault)

    def test_recovery_timeout_does_not_turn_into_indefinite_tiny_steps(self):
        self.fixture();self.tick();self.q[9]=.00204/.3
        # Feedback does not move although requests/ACKs keep arriving.
        with self.assertRaisesRegex(ValueError,'Z recovery timed out'):
            for _ in range(110):self.t+=.02;self.tick()
        self.assertFalse(self.d.output_ready)

    def test_recovery_does_not_override_visual_lease_hold(self):
        self.fixture();self.tick();self.t+=.02;self.q[9]=.0016/.3
        self.tick(receipt=self.t-.45)
        self.assertTrue(self.d.paused)
        self.assertFalse(self.d.recovering_z)
        np.testing.assert_array_equal(self.d.stepper.command,self.q)

    def test_probe_timeout_is_fatal_not_unbounded_motion(self):
        self.fixture();self.p.target_description='book_v3_xyz_servo'
        with self.assertRaisesRegex(ValueError,'timed out'):self.tick(receipt=1.)
        self.assertEqual(self.sent,[])

    def test_singular_jacobian_and_joint_lead_are_bounded(self):
        c=load_settings();step=DifferentialStep(tcp,np.array([[-2.]*14,[2.]*14]),c)
        q=np.zeros(14);goal=np.array([10.,10.,10.,1.,0,0,0])
        for _ in range(200):
            out,vel,_=step.step(q,goal,.02);step.command=out;step.velocity=vel
            self.assertLessEqual(np.max(np.abs(out-q)),c['joint_lead_rad']+1e-12)
        blocked=DifferentialStep(lambda q:(np.zeros((len(np.atleast_2d(q)),2,3)),
            np.tile([1.,0,0,0],(len(np.atleast_2d(q)),2,1))),step.bounds,c)
        out,_,_=blocked.step(q,goal,.02)
        np.testing.assert_array_equal(out,q)

    def test_only_visual_actions_bypass_motiongen(self):
        tree=ast.parse((SCRIPTS/'curobo_planner_v3_1.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CuroboPlanner')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='compute')
        fn.returns=None
        ns=dict(copy=copy,CuroboStatus=CuroboStatus,PlannerCuroboSolverType=PlannerCuroboSolverType,
                TargetCommandArmCommandType=TargetCommandArmCommandType,TargetCommandPlannerMode=TargetCommandPlannerMode)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'compute','exec'),ns)
        for action in ('book_v3_xyz_servo','book_v3_1_stream','first_reset','move_down_to_grasp','lift'):
            p=NS(_first_robot_state_received=True,_first_target_command_received=True,target_description=action,
                 _v31_direct=Mock(),_v31_bridge=NS(guard=Mock()),_target_udp=NS(receive_with_timestamp=lambda:(None,10.)),
                 planner_mode=TargetCommandPlannerMode.PLANNING,_solver_type=PlannerCuroboSolverType.MOTION_GEN,
                 target_arm_type=TargetCommandArmCommandType.CARTESIAN,_motion_gen_plan_single=Mock(),
                 set_gripper_command=Mock(),curobo_cmd=NS(),curobo_state=NS(),target_arm_tool_position=[],
                 target_arm_tool_orientation=[],arm_joint_position=[])
            ns['compute'](p)
            if action.startswith('book_v3'):
                p._v31_direct.compute.assert_called_once_with(10.);p._motion_gen_plan_single.assert_not_called()
            else:p._motion_gen_plan_single.assert_called_once();p._v31_direct.reset.assert_called_once()


if __name__=='__main__':unittest.main()
