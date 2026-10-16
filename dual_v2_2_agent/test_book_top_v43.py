"""Device-free V4.3 pacing and reset order/integration regression."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
from book_top_v43_pacing import LiftPacing
from book_top_v43_servo import load_config,run_raise
from book_top_v42_servo import load_config as old_config,run_raise as old_raise
from book_top_v43_reset import installed,load_config as reset_config
from cart_book_grasp_v1_3_pause import open_and_reset,operator_robot


class PacingTests(unittest.TestCase):
    def test_hysteresis_and_slow_progress(self):
        p=LiftPacing(load_config()['controller']);tol=np.array([3.,1.5])
        self.assertFalse(p.step([4,0],tol,80)[0])
        self.assertTrue(p.step([2,0],tol,80)[0])
        allowed,cap,_=p.step([3.3,0],tol,80)
        self.assertTrue(allowed);self.assertLess(cap,10)
        self.assertFalse(p.step([6,0],tol,80)[0])
        self.assertFalse(p.step([4,0],tol,80)[0])
        self.assertTrue(p.step([2,0],tol,80)[0])
        p.reset();self.assertFalse(p.step([4,0],tol,80)[0])

    def test_fast_far_near_and_history_caps(self):
        p=LiftPacing(load_config()['controller']);tol=np.array([3.,1.5])
        self.assertEqual(p.step([0,0],tol,80)[1],10.)
        self.assertEqual(p.step([0,0],tol,70)[1],15.)
        self.assertEqual(p.step([0,0],tol,19)[1],5.)
        self.assertEqual(p.history_step(80),10.)
        self.assertEqual(p.history_step(19),5.)

    def simulate(self,runner,config,goal=120.,biased=False):
        q=np.array([410.,-100.,0.]);roll=0.;clock=[100.];commands=[]
        def observe():
            clock[0]+=.02
            error=np.array([[3.,.2],[-.1,1.]])@np.array([q[1]+100,roll])
            if biased:error+=np.array([12+q[2]*.1,3.])
            return dict(uv=[639.5+error[0],359.5],angle_deg=error[1],xyz_mm=q.copy(),
                        roll_deg=roll,scale=1.,captured_at=clock[0])
        def move(target,r):
            nonlocal q,roll
            commands.append((q.copy(),target.copy()));q=target.copy();roll=r
            return True
        result=runner(observe,move,639.5,goal,config,path_available=500,clock=lambda:clock[0])
        return result,commands

    def test_fewer_commands_far_and_small_steps_near_final(self):
        new,commands=self.simulate(run_raise,load_config())
        old,old_commands=self.simulate(old_raise,old_config())
        self.assertLess(len(commands),len(old_commands))
        self.assertLessEqual(abs(new['actual_z_mm']-120),2.)
        for start,end in commands:
            self.assertEqual(end[0],410.)
            self.assertLessEqual(np.linalg.norm(end-start),15.+1e-6)
            if 120-start[2]<=20:self.assertLessEqual(end[2]-start[2],5.)

    def test_coupled_controller_final_tolerance_not_relaxed(self):
        result,commands=self.simulate(run_raise,load_config(),goal=60,biased=True)
        self.assertLessEqual(result['error_px'],3.)
        self.assertLessEqual(result['angle_error_deg'],1.5)
        self.assertTrue(any(b[2]>a[2] and abs(b[1]-a[1])>.01 for a,b in commands))

    def test_version_configs_separate(self):
        self.assertEqual(old_config()['controller']['max_step_mm'],10.)
        self.assertEqual(load_config()['controller']['max_step_mm'],15.)
        self.assertEqual(load_config()['controller']['gain'],.5)


class ResetTests(unittest.TestCase):
    def fixture(self,*,fail=False,stale=False,already=False,no_motion=False):
        events=[]
        class Robot:
            right_safe_joint_deg=[0]*7
            def __init__(self):
                self.q=np.array([500.,-100.,20.]);self.joints=np.r_[np.zeros(7),np.ones(7)*(.001 if already else .1)]
                self._wait_for_robot_stop=Mock(return_value=True)
            def get_current_state(self):
                stamp=time.monotonic()-(1 if stale else 0)
                return dict(received_monotonic=stamp,pose_sample_monotonic=stamp,joint_state=self.joints,
                            right_arm=dict(position=self.q/1000,orientation=[1,0,0,0]))
            def control_joint_position(self,left_pos,right_pos,action,*,force=False):
                events.append(action)
                if right_pos:self.joints[7:]=0
                return True
            def master_control(self,**kwargs):
                if 'right_arm' in kwargs:
                    events.append(('lift',list(kwargs['right_arm'])))
                    if fail:return False
                    if not no_motion:self.q=np.array(kwargs['right_arm'][:3])
                    self._wait_for_robot_stop()
                else:events.append('open')
                return True
            def operator_sleep(self,seconds):pass
        return NS(RobotArm=Robot),NS(open_and_reset=open_and_reset),events

    def test_round_reset_lifts_then_resets_with_same_xy_orientation(self):
        agent,loop,events=self.fixture();original=agent.RobotArm
        with installed(agent,loop):
            robot=agent.RobotArm();old_wait=robot._wait_for_robot_stop
            self.assertTrue(robot.control_joint_position(False,True,'first_reset'))
            self.assertEqual(events[0],'book_operator_hold')
            self.assertEqual([e[1][2] for e in events if isinstance(e,tuple)],[45.,70.])
            for e in events:
                if isinstance(e,tuple):self.assertEqual(e[1][:2],[500.,-100.]);self.assertEqual(e[1][3:],[0.,0.,0.])
            self.assertEqual(events[-1],'first_reset')
            self.assertIs(robot._wait_for_robot_stop,old_wait)
            self.assertEqual(old_wait.call_args.kwargs['position_tolerance_m'],.003)
        self.assertIs(agent.RobotArm,original);self.assertIs(loop.open_and_reset,open_and_reset)

    def test_p_lifts_before_open_and_only_once_through_operator_wrapper(self):
        import threading
        agent,loop,events=self.fixture()
        state=NS(armed=True,recovering=True,pause=threading.Event())
        with installed(agent,loop),operator_robot(agent,state):
            robot=agent.RobotArm()
            self.assertTrue(loop.open_and_reset(robot,lambda r:None))
            lifts=[i for i,e in enumerate(events) if isinstance(e,tuple)]
            self.assertEqual(len(lifts),2)
            self.assertLess(lifts[-1],events.index('open'))
            self.assertEqual(events[-1],'first_reset')
            self.assertFalse(robot._v43_reset_lift_done)

    def test_failed_lift_cancels_open_and_reset_requests_hold(self):
        agent,loop,events=self.fixture(fail=True)
        with installed(agent,loop):
            robot=agent.RobotArm()
            self.assertFalse(loop.open_and_reset(robot,lambda r:None))
            self.assertNotIn('open',events);self.assertNotIn('first_reset',events)
            self.assertEqual(events[-1],'book_operator_hold')

    def test_stale_feedback_no_motion_and_already_ready_no_lift(self):
        agent,loop,events=self.fixture(stale=True)
        with installed(agent,loop):
            with self.assertRaisesRegex(ValueError,'stale'):agent.RobotArm().control_joint_position(False,True,'first_reset')
            self.assertEqual(events,[])
        agent,loop,events=self.fixture(already=True)
        with installed(agent,loop):
            self.assertTrue(agent.RobotArm().control_joint_position(False,True,'first_reset'))
            self.assertEqual(events,['first_reset'])

    def test_reported_success_without_measured_lift_cannot_reset(self):
        agent,loop,events=self.fixture(no_motion=True)
        with installed(agent,loop):
            with self.assertRaisesRegex(ValueError,'not reached'):
                loop.open_and_reset(agent.RobotArm(),lambda r:None)
            self.assertNotIn('open',events);self.assertNotIn('first_reset',events)
            self.assertEqual(events[-1],'book_operator_hold')

    def test_reset_configuration_hot_reloads(self):
        agent,loop,events=self.fixture()
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'reset.json'
            path.write_text(json.dumps(dict(lift_mm=50,step_mm=25)))
            with installed(agent,loop,config_path=path):
                r=agent.RobotArm();r.control_joint_position(False,True,'first_reset')
                path.write_text(json.dumps(dict(lift_mm=25,step_mm=25)))
                r=agent.RobotArm();events.clear();r.control_joint_position(False,True,'first_reset')
                self.assertEqual(len([e for e in events if isinstance(e,tuple)]),1)
            path.write_text(json.dumps(dict(lift_mm=500,step_mm=25)))
            with self.assertRaises(ValueError):reset_config(path)


class RuntimeTests(unittest.TestCase):
    def test_v43_reuses_real_operator_wrapper_and_cleanup(self):
        from unittest.mock import patch
        import test_book_top_v42 as previous
        import book_top_v43_runtime as current
        with patch.object(previous,'runtime',current), \
             patch('cart_book_grasp_v3_gpt_recovery.make_lock',return_value=Mock()), \
             patch('book_top_v43_gpt_recovery.verify_tracking',return_value=(None,None)):
            previous.RuntimeTests().run_runtime(operator_wrapped=True)
            previous.RuntimeTests().run_runtime(x_drift=True)

    def test_entry_version_and_reset_wrapper(self):
        import ast
        import book_top_v43_runtime as current
        # Entrypoint must wrap the inherited execution, never the check-only path.
        source=Path(current.__file__).read_text()
        node=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='main')
        text=ast.unparse(node)
        self.assertIn('with reset_installed(agent, loop_module)',text)
        self.assertIn('cuart_book_grasp_v4_3_config.json',text)
        self.assertNotIn('cuart_book_grasp_v4_2_config.json',text)

if __name__=='__main__':unittest.main()
