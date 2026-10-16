"""Offline regressions: closure is gated by post-hold visual confirmation."""
import unittest
from unittest.mock import patch
from unittest.mock import Mock
from types import SimpleNamespace
import numpy as np
import tempfile
from pathlib import Path
from contextlib import nullcontext
from test_cart_grasp_alignment import FakeLoop
from cart_grasp_alignment import load_config
from cart_grasp_depth import load_approach
from cart_grap_v1_1_alignment import align_loop
import cart_grap_v1_1 as cycle

class FinalAlignmentTests(unittest.TestCase):
    def test_repeat_success_waits_five_seconds_and_separates_outputs(self):
        from cart_book_grasp_v1_1 import AutoCartLoop
        loop=AutoCartLoop();waits=[];calls=[]
        def delay(seconds):waits.append(seconds);return False
        def run(args,c):
            calls.append(args.out)
            if len(calls)==2:loop.request_stop()
            return 0
        with tempfile.TemporaryDirectory() as folder:
            args=SimpleNamespace(execute=True,once=False,out=Path(folder)/'runs')
            with patch('cart_book_grasp_v1_1.AutoCartLoop',return_value=loop),patch('cart_book_grasp_v1_1.stop_key_listener',return_value=nullcontext()),patch.object(loop.stop,'wait',side_effect=delay),patch.object(cycle,'run',side_effect=run):
                self.assertEqual(cycle.run_repeated(args,{}),0)
            self.assertEqual(calls,[args.out/'round_0001',args.out/'round_0002'])
            self.assertEqual(waits,[5.])

    def test_repeat_failure_waits_for_retry_not_success_timer(self):
        from cart_book_grasp_v1_1 import AutoCartLoop
        loop=AutoCartLoop()
        with patch('cart_book_grasp_v1_1.AutoCartLoop',return_value=loop),patch('cart_book_grasp_v1_1.stop_key_listener',return_value=nullcontext()),patch.object(loop,'wait_for_retry',return_value=False) as retry,patch.object(loop.stop,'wait') as delay,patch.object(cycle,'run',return_value=1) as run:
            self.assertEqual(cycle.run_repeated(SimpleNamespace(out=None),{}),1)
            retry.assert_called_once();run.assert_called_once();delay.assert_not_called()

    def test_interrupt_exits_and_once_bypasses_loop(self):
        from cart_book_grasp_v1_1 import AutoCartLoop
        loop=AutoCartLoop()
        with patch('cart_book_grasp_v1_1.AutoCartLoop',return_value=loop),patch('cart_book_grasp_v1_1.stop_key_listener',return_value=nullcontext()),patch.object(cycle,'run',return_value=130) as run:
            self.assertEqual(cycle.run_repeated(SimpleNamespace(out=None),{}),130)
            run.assert_called_once()
        with patch.object(cycle,'run',return_value=0) as run,patch.object(cycle,'run_repeated',return_value=0) as repeat:
            cycle.main(['--execute','--once']);run.assert_called_once();repeat.assert_not_called()
            cycle.main(['--execute']);repeat.assert_called_once()

    def settle_fixture(self,position=lambda t:min(t,2.)*.001,velocity=lambda t:0.):
        clock=SimpleNamespace(now=100.);events=[]
        def read():
            return dict(pose_sample_monotonic=clock.now,
                right_arm=dict(position=[position(clock.now-100),0.,0.],orientation=[1.,0.,0.,0.]),
                joint_velocity=[velocity(clock.now-100)]*14)
        return clock,events,read

    def test_tcp_waits_for_residual_motion_then_records_final_pose(self):
        clock,events,read=self.settle_fixture()
        with patch.object(cycle.time,'monotonic',lambda:clock.now),patch.object(cycle.time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt)):
            result=cycle.wait_tcp_stable(read,events.append)
        self.assertGreater(clock.now,102.)
        self.assertEqual(result['right_arm']['position'][0],.002)
        self.assertEqual(events[-1]['event'],'tcp_settle_confirmed')
        self.assertGreaterEqual(events[-1]['stable_seconds'],1.)

    def test_tcp_slow_drift_does_not_pass_adjacent_sample_check(self):
        clock,events,read=self.settle_fixture(position=lambda t:t*.001)
        with patch.object(cycle.time,'monotonic',lambda:clock.now),patch.object(cycle.time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt)):
            with self.assertRaisesRegex(ValueError,'position_delta=.*orientation_delta=.*max_joint_speed='):
                cycle.wait_tcp_stable(read,events.append,timeout=3.)
        self.assertEqual(events[-1]['event'],'tcp_settle_timeout')
        self.assertGreater(events[-1]['peak']['position_delta_mm'],.15)

    def test_tcp_velocity_timeout_reports_actual_speed(self):
        clock,events,read=self.settle_fixture(position=lambda t:0.,velocity=lambda t:.03)
        with patch.object(cycle.time,'monotonic',lambda:clock.now),patch.object(cycle.time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt)):
            with self.assertRaisesRegex(ValueError,'max_joint_speed=0.03000'):
                cycle.wait_tcp_stable(read,events.append,timeout=1.5)

    def test_acquisition_motion_reports_metrics_without_rebasing(self):
        clock,events,read=self.settle_fixture(position=lambda t:0.)
        io=object.__new__(cycle.CycleIO);io.initial=read();io.log=events.append
        later=read();later['right_arm']['position'][0]=.0007;io.state=lambda:later
        with self.assertRaisesRegex(ValueError,'position_delta=0.700mm'):
            io.stationary()
        self.assertEqual(io.initial['right_arm']['position'][0],0.)
        self.assertEqual(events[-1]['event'],'acquisition_motion_rejected')

    def test_cycle_origin_and_orientation_use_confirmed_sample(self):
        robot,clock,_=self.reset_fixture()
        settled=robot.get_current_state()
        settled['right_arm']['position']=[.412,-.059,-.012]
        settled['right_arm']['orientation']=[np.cos(.1),0.,0.,np.sin(.1)]
        with patch.object(cycle,'wait_tcp_stable',return_value=settled) as wait:
            io=cycle.CycleIO(robot,[0,0,0],cycle.load_config(),load_config(),load_approach(),None,lambda e:None)
        wait.assert_called_once()
        np.testing.assert_allclose(io.initial_q,[412,-59,-12])
        self.assertAlmostEqual(io.reference_z,-12.+cycle.load_config()['alignment_raise_mm'])
        self.assertAlmostEqual(io.base[2],np.degrees(.2))
        np.testing.assert_allclose(io.transport.base,io.base)
        self.assertIsNotNone(io.transport.last_state)
        robot.control_joint_position.assert_not_called()

    def test_coarse_failure_can_cancel_before_first_visual_read(self):
        from cart_book_grasp_v3_2_runtime import StreamTransport
        robot,clock,_=self.reset_fixture()
        state=robot.get_current_state()  # status=3 is allowed for cancellation
        io=object.__new__(cycle.CycleIO);io.robot=robot;io.commanded=True
        io.transport=StreamTransport(robot,[0,0,0],{}, {})
        with patch.object(cycle.time,'monotonic',lambda:clock.now):io.abort()
        self.assertEqual(io.transport.last_state['curobo_status'],3)
        self.assertEqual(robot.master_control.call_args.kwargs['action'],'book_v3_1_hold')
        self.assertFalse(robot.master_control.call_args.kwargs['wait_for_completion'])
        self.assertNotIn('right_gripper',robot.master_control.call_args.kwargs)

    def test_cancel_keeps_valid_envelope_when_feedback_disappears(self):
        from cart_book_grasp_v3_2_runtime import StreamTransport
        robot,clock,_=self.reset_fixture();cached=robot.get_current_state()
        io=object.__new__(cycle.CycleIO);io.robot=robot;io.commanded=True
        io.transport=StreamTransport(robot,[0,0,0],{}, {});io.transport.last_state=cached
        robot.get_current_state=lambda:None
        io.abort()
        self.assertIs(io.transport.last_state,cached)
        self.assertEqual(robot.master_control.call_args.kwargs['action'],'book_v3_1_hold')

    def reset_fixture(self,success=True,grip=0.):
        clock=SimpleNamespace(now=100.)
        robot=SimpleNamespace(right_safe_joint_deg=[0.]*7,_wait_for_robot_stop=Mock(return_value=True),
            get_cat_gripper=Mock(return_value=0),master_control=Mock(return_value=True))
        sent=[]
        def state():
            return dict(received_monotonic=clock.now,pose_sample_monotonic=clock.now,
                joint_state=np.zeros((2,7)),joint_velocity=np.zeros((2,7)),
                robot_status=0,curobo_status=9 if sent else 3,
                right_arm=dict(position=[.411,-.058,-.011],orientation=[1.,0.,0.,0.]))
        robot.get_current_state=state
        robot.udp=SimpleNamespace(receive_with_timestamp=lambda:(SimpleNamespace(
            pose_sample_monotonic=clock.now,gripper_joint_position=[[0.],[grip]],
            processed_gripper_mode_command=[0,0]),clock.now))
        def reset(*args,**kwargs):
            sent.append((args,kwargs));clock.now+=.1
            robot._wait_for_robot_stop()
            return success
        robot.control_joint_position=Mock(side_effect=reset)
        return robot,clock,sent

    def test_first_reset_replaces_old_fault_and_confirms_fresh_home(self):
        robot,clock,sent=self.reset_fixture();old_wait=robot._wait_for_robot_stop;events=[]
        with patch.object(cycle.time,'monotonic',lambda:clock.now),patch.object(cycle.time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt)):
            base=cycle.first_reset(robot,cycle.load_config(),events.append)
        self.assertEqual(sent,[((False,True,'first_reset'),{'force':True})])
        self.assertIs(robot._wait_for_robot_stop,old_wait)
        self.assertAlmostEqual(old_wait.call_args.kwargs['joint_tolerance_rad'],np.deg2rad(.3))
        self.assertGreaterEqual(clock.now,100.3)
        np.testing.assert_allclose(base,0)
        self.assertEqual(events[-1]['event'],'first_reset_confirmed')
        robot.master_control.assert_not_called()

    def test_first_reset_waits_for_controller_idle_and_planner_finished(self):
        robot,clock,_=self.reset_fixture();read=robot.get_current_state
        def state():
            result=read()
            if clock.now<100.5:result['robot_status']=1
            if clock.now<100.8:result['curobo_status']=4
            return result
        robot.get_current_state=state
        with patch.object(cycle.time,'monotonic',lambda:clock.now),patch.object(cycle.time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt)):
            cycle.first_reset(robot,cycle.load_config(),lambda e:None)
        self.assertGreaterEqual(clock.now,101.)

    def test_first_reset_near_target_but_busy_times_out(self):
        robot,clock,_=self.reset_fixture();read=robot.get_current_state
        def state():
            result=read();result['robot_status']=1;return result
        robot.get_current_state=state
        with patch.object(cycle.time,'monotonic',lambda:clock.now),patch.object(cycle.time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt)):
            with self.assertRaisesRegex(ValueError,'stable feedback timed out'):
                cycle.first_reset(robot,cycle.load_config(),lambda e:None)
        self.assertEqual(robot.master_control.call_args.kwargs['action'],'book_v3_1_hold')

    def test_first_reset_failure_cancels_without_grasp(self):
        robot,clock,_=self.reset_fixture(success=False);old_wait=robot._wait_for_robot_stop
        with patch.object(cycle.time,'monotonic',lambda:clock.now):
            with self.assertRaisesRegex(ValueError,'first_reset failed'):
                cycle.first_reset(robot,cycle.load_config(),lambda e:None)
        self.assertIs(robot._wait_for_robot_stop,old_wait)
        self.assertEqual(robot.master_control.call_args.kwargs['action'],'book_v3_1_hold')
        self.assertNotIn('right_gripper',robot.master_control.call_args.kwargs)

    def test_first_reset_requires_measured_open_gripper(self):
        robot,clock,_=self.reset_fixture(grip=.5)
        with patch.object(cycle.time,'monotonic',lambda:clock.now):
            with self.assertRaisesRegex(ValueError,'physically open'):
                cycle.first_reset(robot,cycle.load_config(),lambda e:None)
        robot.control_joint_position.assert_not_called()

    def run_loop(self,f,finalize,c=None):
        return align_loop(f.read,f.observe,f.send,lambda:True,f.hold,
            np.array([500.,0.,-10.]),c or load_config(),f.events.append,lambda:f.now,
            goal_x=510.,approach=load_approach(),finalize=finalize)

    def test_post_hold_shift_realigns_without_reprobe_or_extra_final_hold(self):
        f=FakeLoop();attempts=[]
        def finish(checkpoint):
            f.hold();attempts.append(f.now)
            if len(attempts)==1:f.q[1]+=1.3
            for _ in range(3):
                e,p=f.observe(checkpoint)
                if abs(e[0])>3 or abs(e[1])>1.5:return None
            return e,p
        result=self.run_loop(f,finish)
        self.assertEqual(len(attempts),2)
        self.assertEqual(f.holds,2)  # no third hold after final confirmation
        self.assertEqual(sum(b for q,r,b in f.sent),2)  # probes only once
        self.assertEqual(sum(e['event']=='cart_aligned' for e in f.events),1)
        self.assertLessEqual(abs(result['error_px_deg'][0]),3)
        self.assertEqual(f.events[-1]['event'],'cart_aligned')

    def test_persistent_rejection_uses_original_time_budget(self):
        f=FakeLoop();c=load_config();c['max_seconds']=8
        def finish(checkpoint):
            f.hold();f.now+=.3;checkpoint();return None
        with self.assertRaisesRegex(ValueError,'time budget'):
            self.run_loop(f,finish,c)
        self.assertFalse(any(e['event']=='cart_aligned' for e in f.events))
        self.assertEqual(sum(b for q,r,b in f.sent),2)
        self.assertLess(f.now,9)

    def test_feedback_fault_in_final_check_propagates(self):
        f=FakeLoop()
        def finish(checkpoint):
            f.q[2]+=4;checkpoint()
        with self.assertRaisesRegex(ValueError,'fixed X/Z'):
            self.run_loop(f,finish)
        self.assertFalse(any(e['event']=='cart_aligned' for e in f.events))

    def test_entry_has_offline_preview_default_and_existing_config(self):
        with patch.object(cycle,'run',return_value=0) as run:
            self.assertEqual(cycle.main([]),0)
            self.assertFalse(run.call_args.args[0].execute)
        with patch.object(cycle,'run',side_effect=AssertionError('hardware')):
            self.assertEqual(cycle.main(['--check-config']),0)

if __name__=='__main__':unittest.main()
