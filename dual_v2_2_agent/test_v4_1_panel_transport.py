"""Offline publisher regressions. No robot, socket or CUDA initialization."""
import ast
import copy
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[1]
SCRIPTS=ROOT/'dual_v2_2_control/cuarm_upper_planner/scripts'
sys.path.insert(0,str(SCRIPTS))
from v4_1_panel_transport import PanelTransport, StalePlanStart

def command():
    return NS(arm_size=2,arm_joint_size=[7,7],arm_target_mode=1,
        arm_joint_command=[[0.]*7,[0.]*7],arm_waypoint_size=2,arm_waypoint_dt=.5,
        arm_waypoint_command=[[[.3]*7]*2]*2,gripper_joint_command=[[.7],[.4]])

class TransportTests(unittest.TestCase):
    def setUp(self):
        self.now=self.stamp=1.
        self.panel=NS(state=1,arm_joint_position=[[.1]*7,[.2]*7],gripper_joint_position=[[.8],[.6]])
        self.sent=[]
        self.t=PanelTransport(lambda c:self.sent.append(copy.deepcopy(c)),lambda:(self.panel,self.stamp),
            command(),0,0,clock=lambda:self.now,log=lambda _:None)
        self.t.begin_cycle()
    def advance(self,dt,fresh=True):
        self.now+=dt
        if fresh:self.stamp=self.now
        self.t.tick()
    def test_blocked_plan_repeats_immutable_hold_not_pending_goal(self):
        self.t.send(command());self.t.begin_planning();original=copy.deepcopy(self.sent[-1])
        self.assertEqual(original.arm_target_mode,0)
        self.assertEqual(original.arm_joint_command,self.panel.arm_joint_position)
        self.assertEqual(original.gripper_joint_command,command().gripper_joint_command)
        for _ in range(30):self.advance(.05)
        self.assertIsNone(self.t.fault)
        self.assertTrue(all(vars(c)==vars(original) for c in self.sent[1:]))
        self.t.send(command());self.assertEqual(self.sent[-1].arm_target_mode,1)
    def test_raw_to_hold_uses_positive_interpolation_time(self):
        c=command();c.arm_target_mode=2;c.interpolation_acceleration_time=0.
        self.t.send(c);self.t.begin_planning()
        self.assertEqual(self.sent[-1].arm_target_mode,0)
        self.assertGreater(self.sent[-1].interpolation_acceleration_time,0.)
    def test_initial_hold_uses_measured_gripper(self):
        self.t.begin_planning()
        self.assertEqual(self.sent[-1].gripper_joint_command,self.panel.gripper_joint_position)
        self.assertEqual(self.sent[-1].arm_waypoint_size,0)
    def test_hold_not_dragged_by_measurement(self):
        self.t.begin_planning();self.panel.arm_joint_position[1][0]+=.001;self.advance(.05)
        self.assertEqual(self.sent[-1].arm_joint_command[1][0],.2)
    def test_active_hang_holds_once_then_silence(self):
        self.t.send(command());self.advance(.2);self.advance(.06)
        self.assertIn('producer',self.t.fault);self.assertEqual(self.sent[-1].arm_target_mode,0)
        n=len(self.sent);self.advance(1.);self.assertEqual(len(self.sent),n)
        with self.assertRaises(RuntimeError):self.t.send(command())
    def test_planning_deadline_rejects_late_result(self):
        self.t.begin_planning()
        for _ in range(51):self.advance(.2)
        self.assertIsNotNone(self.t.fault)
        with self.assertRaises(RuntimeError):self.t.send(command())
    def test_deadline_even_without_worker_tick(self):
        self.t.begin_planning();self.now+=11.;self.stamp=self.now
        with self.assertRaises(RuntimeError):self.t.send(command())
        self.assertEqual(self.sent[-1].arm_target_mode,0)
    def test_stale_feedback_no_replay(self):
        self.t.begin_planning();n=len(self.sent);self.advance(.3,False)
        self.assertIn('stale',self.t.fault);self.assertEqual(len(self.sent),n)
        self.advance(.1)
        with self.assertRaises(RuntimeError):self.t.begin_cycle()
    def test_stop_restart_rejects_inflight_result(self):
        self.t.begin_planning();self.panel.state=0;self.advance(.05);n=len(self.sent)
        self.panel.state=1;self.advance(.05);self.assertEqual(len(self.sent),n)
        with self.assertRaisesRegex(RuntimeError,'session changed'):self.t.send(command())
    def test_stopped_startup_no_commands(self):
        self.panel.state=0;self.t.begin_cycle();self.advance(.1);self.assertEqual(self.sent,[])
        with self.assertRaises(RuntimeError):self.t.begin_planning()
    def test_moved_hold_rejects_stale_plan(self):
        self.t.begin_planning();hold=copy.deepcopy(self.sent[-1])
        self.panel.arm_joint_position[1][0]+=.003236
        with self.assertRaises(StalePlanStart):self.t.send(command())
        self.assertIsNone(self.t.fault)
        self.assertEqual(vars(self.sent[-1]),vars(hold))
        self.assertFalse(self.t.ready_to_plan())
        # Same sample cannot satisfy settlement, even after wall time advances.
        self.now+=.16
        self.assertFalse(self.t.ready_to_plan())
        self.t.idle_hold();self.advance(.01)
        self.assertTrue(self.t.ready_to_plan())
        self.t.begin_planning(copy.deepcopy(self.panel.arm_joint_position))
        self.assertEqual(vars(self.sent[-1]),vars(hold))
        self.advance(.1);self.t.send(command())
        self.assertEqual(self.t.mode,'active')
        self.assertIsNone(self.t.recovery_started)
    def test_gate_compares_actual_planning_input(self):
        start=copy.deepcopy(self.panel.arm_joint_position)
        self.panel.arm_joint_position[1][0]+=.003236
        self.t.begin_planning(start)
        with self.assertRaises(StalePlanStart):self.t.send(command())
    def test_recovery_requires_stability_not_just_elapsed_time(self):
        self.t.begin_planning();self.panel.arm_joint_position[1][0]+=.003236
        with self.assertRaises(StalePlanStart):self.t.send(command())
        self.assertFalse(self.t.ready_to_plan())
        for _ in range(4):
            self.t.idle_hold();self.panel.arm_joint_position[1][0]+=.001
            self.advance(.1);self.assertFalse(self.t.ready_to_plan())
        self.t.idle_hold();self.advance(.16)
        self.assertTrue(self.t.ready_to_plan())
    def test_repeated_start_rejections_fail_closed(self):
        for attempt in range(3):
            self.t.begin_planning()
            self.panel.arm_joint_position[1][0]+=.003236
            exception=StalePlanStart if attempt<2 else RuntimeError
            with self.assertRaises(exception):self.t.send(command())
        self.assertIn('3 rejections',self.t.fault)
        n=len(self.sent);self.advance(.1);self.assertEqual(len(self.sent),n)
    def test_recovery_timeout_even_with_live_main_and_fresh_feedback(self):
        self.t.begin_planning();self.panel.arm_joint_position[1][0]+=.003236
        with self.assertRaises(StalePlanStart):self.t.send(command())
        with self.assertRaisesRegex(RuntimeError,'recovery timed out'):
            for _ in range(102):
                self.t.idle_hold();self.advance(.1)
                self.panel.arm_joint_position[1][0]+=.001
                self.t.ready_to_plan()
        self.assertIsNotNone(self.t.fault)
    def test_recovery_does_not_hide_dead_producer_or_feedback_loss(self):
        self.t.begin_planning();self.panel.arm_joint_position[1][0]+=.003236
        with self.assertRaises(StalePlanStart):self.t.send(command())
        self.advance(.26)
        self.assertIn('producer',self.t.fault)
    def test_client_cancel_before_visual_start_maintains_hold(self):
        from v3_1_execution import ExecutionGuard
        guard=ExecutionGuard(self.t.send,clock=lambda:self.now,log=lambda _:None)
        self.t.send(command())
        guard.trip('client requested hold')
        self.assertIsNone(guard.hold_command)
        # Exercise the adapter's actual invalid-plan branch without CUDA/UDP.
        tree=ast.parse((SCRIPTS.parent/'dual_v2_2_upper_planner_v3_1.py').read_text())
        dispatch=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
                      and ast.unparse(n.test)=='not is_preview and valid_motion_plan')
        branch=compile(ast.Module(body=dispatch.orelse,type_ignores=[]),'<adapter hold>','exec')
        context={'planner':NS(_v31_bridge=NS(guard=guard)),'transport':self.t}
        for _ in range(30):
            exec(branch,context);self.advance(.05)
        self.assertIsNone(self.t.fault)
        self.assertEqual(self.sent[-1].arm_target_mode,0)
        self.assertEqual(self.sent[-1].arm_waypoint_size,0)
        self.assertTrue(all(c.arm_target_mode==0 for c in self.sent[1:]))
        self.advance(.26);self.assertIsNotNone(self.t.fault)
    def test_discard_plan_keeps_goal_and_forces_recompute(self):
        tree=ast.parse((SCRIPTS/'curobo_planner_v3_1.py').read_text())
        methods={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
        names=('discard_panel_plan','_clear_planned_result','_hold_current_arm_command',
               '_cartesian_target_changed')
        context={'CuroboStatus':NS(PLANNING=2)}
        exec(compile(ast.Module(body=[methods[n] for n in names],type_ignores=[]),
                     '<planner recovery>','exec'),context)
        class Fake:pass
        for n in names:setattr(Fake,n,context[n])
        p=Fake();p.arm_joint_position=[0.]*14;p.arm_joint_size=[7,7]
        p._reshape_list=lambda q,s:[q[:7],q[7:]]
        p.curobo_cmd=NS(waypoints_size=6,waypoints=[1]*6)
        p.target_arm_tool_position=[[.4,0,0],[.5,-.2,-.01]]
        goal=copy.deepcopy(p.target_arm_tool_position)
        p.discard_panel_plan()
        self.assertEqual(p.curobo_cmd.waypoints_size,0)
        self.assertIsNone(p._cmd_plan)
        self.assertTrue(p._cartesian_target_changed())
        self.assertEqual(p.target_arm_tool_position,goal)
    def test_packet_copy_and_shutdown(self):
        c=command();self.t.send(c);c.arm_joint_command[0][0]=99.;self.advance(.05)
        self.assertNotEqual(self.sent[-1].arm_joint_command[0][0],99.)
        self.t.terminate();n=len(self.sent);self.advance(.05);self.assertEqual(len(self.sent),n)
        self.assertEqual(self.sent[-1].arm_target_mode,0)
    def test_failed_planning_hold_has_producer_lease(self):
        self.t.begin_planning();self.advance(.6);self.t.idle_hold();self.advance(.26)
        self.assertIsNotNone(self.t.fault)
    def test_worker_keeps_sending_during_800ms_block(self):
        sent=[]
        t=PanelTransport(lambda c:sent.append((time.monotonic(),copy.deepcopy(c))),
            lambda:(self.panel,time.monotonic()),command(),0,0,log=lambda _:None)
        try:
            t.begin_cycle();t.begin_planning();t.start();time.sleep(.8)
            self.assertIsNone(t.fault);self.assertGreater(len(sent),8)
            self.assertLess(max(b[0]-a[0] for a,b in zip(sent,sent[1:])),.5)
            self.assertTrue(all(c.arm_target_mode==0 for _,c in sent))
            t.send(command());self.assertEqual(sent[-1][1].arm_target_mode,1)
        finally:t.terminate()
    def test_no_publication_after_shutdown(self):
        self.t.send(command());self.t.terminate();n=len(self.sent)
        with self.assertRaises(RuntimeError):self.t.send(command())
        self.assertEqual(len(self.sent),n)
    def test_guard_hold_replaces_cached_motion(self):
        self.t.send(command())
        hold=command();hold.arm_target_mode=0;hold.arm_joint_command=[[.1]*7,[.2]*7]
        self.t.send(hold);self.advance(.05)
        self.assertEqual(vars(self.sent[-1]),vars(hold))
    def test_hold_real_serializer(self):
        sys.path.insert(0,str(ROOT/'dual_v2_2_configuration/robot_platform_utils/python/scripts'))
        sys.path.insert(0,str(ROOT/'dual_v2_2_configuration/curi_udp/python'))
        from cuarm_udp import CuarmUdp
        from cuarm_state import ControlType,PanelTargetMode
        from cuarm_state_initializers import get_initialized_remote_panel_command
        self.t.template=get_initialized_remote_panel_command(target_type=ControlType.POSITION,
            actuator_mode=ControlType.POSITION,arm_size=2,arm_joint_size=[7,7],gripper_size=2,gripper_joint_size=[1,1])
        self.t.hold_mode=PanelTargetMode.SINGLE_POINT;self.t.begin_planning()
        raw=CuarmUdp.pack_remote_panel_command(object.__new__(CuarmUdp),self.sent[-1])
        self.assertGreater(len(raw),60);self.assertNotIn('nan',raw)
    def test_routing_and_syntax(self):
        source=(SCRIPTS.parent/'dual_v2_2_upper_planner_v3_1.py').read_text();ast.parse(source)
        self.assertNotIn('panel_udp.send(',source);self.assertIn('ExecutionGuard(transport.send)',source)
        self.assertIn('planner._panel_begin_planning = transport.begin_planning',source)
        self.assertIn('transport.begin_cycle()',source)
        planner=(SCRIPTS/'curobo_planner_v3_1.py').read_text();ast.parse(planner)
        self.assertEqual(planner.count("getattr(self, '_panel_begin_planning', None)"),2)
if __name__=='__main__':unittest.main()
