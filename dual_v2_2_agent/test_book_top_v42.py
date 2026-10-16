"""Offline V4.2 axis, failure, frame ownership, and lifecycle regression tests."""
import json
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_servo import ServoError
import book_top_v42_servo as servo
import book_top_v42_runtime as runtime
from book_top_v42_tracking import TrackingStream


class LiftTests(unittest.TestCase):
    def simulation(self,*,loss=False):
        q=np.array([410.,-100.,0.]);r=0.;clock=[100.];commands=[];events=[]
        def observe():
            clock[0]+=.02
            if loss and len(commands)>=3:raise ServoError('lost points')
            # Tilting spine: vertical camera translation changes its centre column.
            f=np.array([[3.,.2],[-.1,1.]])@np.array([q[1]+100,r])+[12+q[2]*.1,3]
            return dict(uv=[639.5+f[0],359.5],angle_deg=f[1],xyz_mm=q.copy(),roll_deg=r,scale=1.,captured_at=clock[0])
        def move(goal,roll):
            nonlocal q,r
            commands.append((goal.copy(),roll));q=goal.copy();r=roll;return True
        return observe,move,clock,commands,events
    def test_lift_and_correct_without_advancing_x(self):
        observe,move,clock,commands,events=self.simulation()
        result=servo.run_raise(observe,move,639.5,30.,servo.load_config(),path_available=500,
            clock=lambda:clock[0],log=events.append)
        self.assertTrue(result['converged']);self.assertLessEqual(abs(result['actual_z_mm']-30),2)
        self.assertTrue(all(abs(q[0]-410)<1e-9 for q,r in commands))
        self.assertTrue(any(abs(q[1]+100)>1 for q,r in commands))
        self.assertTrue(any(abs(r)>1 for q,r in commands))
        self.assertTrue(all(e['advance_axis']=='base_Z' for e in events))
    def test_track_loss_does_not_raise_blindly(self):
        observe,move,clock,commands,events=self.simulation(loss=True)
        with self.assertRaisesRegex(ServoError,'lost points'):
            servo.run_raise(observe,move,639.5,30.,servo.load_config(),path_available=500,clock=lambda:clock[0])
        self.assertEqual(len(commands),3)
    def test_partial_raise_is_not_pure_y_roll_model_failure(self):
        q=np.array([410.,-100.,0.]);roll=0.;now=[100.];events=[]
        def observe():
            now[0]+=.02
            f=np.array([[3.,.2],[-.1,1.]])@np.array([q[1]+100,roll])+[12-2.4*q[2],3]
            return dict(uv=[639.5+f[0],359.5],angle_deg=f[1],xyz_mm=q.copy(),roll_deg=roll,scale=1.,captured_at=now[0])
        def move(goal,r):
            nonlocal q,roll
            z=q[2]+.56*(goal[2]-q[2]) if goal[2]>q[2] else goal[2]
            q=goal.copy();q[2]=z;roll=r;return True
        c=servo.load_config();c['controller']['max_step_mm']=5.
        result=servo.run_raise(observe,move,639.5,15.,c,path_available=500,clock=lambda:now[0],log=events.append)
        self.assertTrue(result['converged'])
        self.assertTrue(any(e['event']=='lift_response' and 2<e['measured_progress_mm']<3 for e in events))

    def test_small_ineffective_corrections_stop_without_140_repeats(self):
        q=np.array([410.,-100.,0.]);roll=0.;now=[100.];calls=[]
        def observe():
            now[0]+=.02
            f=np.array([3*(q[1]+100)-9,roll-2])
            return dict(uv=[639.5+f[0],359.5],angle_deg=f[1],xyz_mm=q.copy(),roll_deg=roll,scale=1.,captured_at=now[0])
        def move(goal,r):
            nonlocal q,roll
            calls.append(1)
            if len(calls)<=2:q=goal.copy();roll=r
            return True
        with self.assertRaisesRegex(ServoError,'no measured correction progress'):
            servo.run_raise(observe,move,639.5,30.,servo.load_config(),path_available=500,clock=lambda:now[0])
        self.assertLessEqual(len(calls),6)

    def test_empty_budget_sends_nothing(self):
        o,m,t,commands,_=self.simulation()
        with self.assertRaisesRegex(ServoError,'budget'):servo.run_raise(o,m,640,30,servo.load_config(),path_available=0)
        self.assertEqual(commands,[])
    def test_lateral_limit(self):
        o,m,t,commands,_=self.simulation();c=servo.load_config();c['max_lateral_mm']=1
        with self.assertRaisesRegex(ServoError,'Y correction'):servo.run_raise(o,m,639.5,30,c,path_available=500,clock=lambda:t[0])
        self.assertEqual(commands,[])
    def test_bad_config(self):
        c=servo.load_config();c['controller']['probe_mm']=3
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'bad.json';p.write_text(json.dumps(c))
            with self.assertRaises(ServoError):servo.load_config(p)


class StreamTests(unittest.TestCase):
    def test_background_tracking_and_fresh_after(self):
        def capture(check):
            time.sleep(.005);check();return dict(rgb=np.zeros((2,2,3)),captured_at=time.monotonic())
        lock=NS(observe=Mock(return_value={'uv':[1,1]}));camera=NS(capture=capture)
        stream=TrackingStream(camera,lock).start()
        try:
            first=stream.after(time.monotonic(),1)
            time.sleep(.03)
            second=stream.after(first[0]['captured_at'],1)
            self.assertGreater(second[2],first[2]);self.assertGreater(lock.observe.call_count,1)
        finally:stream.close()
        self.assertFalse(stream.thread.is_alive())
    def test_failure_propagates(self):
        stream=TrackingStream(NS(capture=lambda check:dict(rgb=0,captured_at=time.monotonic())),
                              NS(observe=Mock(side_effect=ServoError('lost')))).start()
        try:
            with self.assertRaisesRegex(ServoError,'lost'):stream.after(time.monotonic(),1)
        finally:stream.close()
    def test_stale_frame_rejected(self):
        stream=TrackingStream(NS(capture=lambda check:dict(rgb=0,captured_at=time.monotonic()-10)),NS(observe=Mock())).start()
        try:
            with self.assertRaisesRegex(ServoError,'stale'):stream.after(0,1)
        finally:stream.close()


class RuntimeTests(unittest.TestCase):

    def test_raise_hook_is_opt_in_and_restored(self):
        import sys
        import cart_book_grasp_v1_3_motion as motion
        import book_top_v34_inherited as inherited
        ordinary=Mock(return_value=('object_0',True));handler=Mock(return_value=('object_0',True))
        step5=NS(execute_single_task=ordinary,execute_main=Mock(),recover_to_reset_after_failure=Mock())
        agent=NS(grounding_main=Mock(),execute_main=Mock())
        config=inherited.load_config(Path(__file__).with_name('cuart_book_grasp_v4_2_config.json'))
        with patch.dict(sys.modules,{'dual_v2_2_agent_pcl':agent,'step5_execute':step5}),patch.object(motion,'_install_right_arm_reset'):
            with inherited.installed(config,raise_servo=handler):
                motion._install_right_arm_reset()
                task={'action':'book_v41_top_raise_z','object':'object_0'}
                self.assertTrue(step5.execute_single_task(None,task,0,{})[1])
                handler.assert_called_once();ordinary.assert_not_called()
            self.assertIs(step5.execute_single_task,ordinary)

    def test_real_executor_routes_raise_as_ordinary_cartesian(self):
        import ast
        tree=ast.parse((Path(__file__).parent/'RobotRAM/scripts/step5_execute.py').read_text())
        nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('execute_single_task','validate_arm_target_side')]
        env=dict(RIGHT_CART_BOOK_VERTICAL_ACTIONS=set(),GRIPPER_CLOSE_SETTLE_TIME_S=0.,time=NS(sleep=Mock()))
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'<step5>','exec'),env)
        robot=NS(master_control=Mock(return_value=True),operator_sleep=Mock())
        task=dict(arm='right',object='book0',action='book_v42_raise_servo',target_position=[410,-100,5],target_rotation=[-90,0,90])
        self.assertTrue(env['execute_single_task'](robot,task,0,{'book0':'book'})[1])
        self.assertEqual(robot.master_control.call_args.args,(None,[410,-100,5,-90,0,90],None,None))

    def test_feedback_receipt_does_not_hide_old_pose(self):
        s=dict(received_monotonic=time.monotonic(),pose_sample_monotonic=time.monotonic()-1)
        with self.assertRaisesRegex(ServoError,'pose_sample'):runtime.pose(NS(get_current_state=lambda:s))
    def run_runtime(self,failed=False,operator_wrapped=False,point_loss=False,gpt_recovery=False,x_drift=False):
        from contextlib import ExitStack
        import book_top_v34_inherited as inherited
        config=inherited.load_config(Path(__file__).with_name('cuart_book_grasp_v4_2_config.json'))
        q=np.array([410.,-100.,0.]);angles=np.zeros(3)
        def read():
            quat=Rotation.from_euler('xyz',angles,degrees=True).as_quat()
            return dict(received_monotonic=time.monotonic(),pose_sample_monotonic=time.monotonic(),curobo_status=0,
                        right_arm=dict(position=q/1000,orientation=quat[[3,0,1,2]]))
        robot=NS(get_current_state=read,operator_checkpoint=Mock(),_wait_for_robot_stop=Mock(),master_control=Mock())
        if operator_wrapped:
            import threading
            from cart_book_grasp_v1_3_pause import operator_robot
            class Base:
                def get_current_state(self):return read()
                def master_control(self,**kw):return True
                def _wait_for_robot_stop(self,*a,**kw):return True
            agent=NS(RobotArm=Base)
            loop=NS(armed=True,recovering=False,pause=threading.Event())
            with operator_robot(agent,loop):robot=agent.RobotArm()
            robot.master_control=Mock(wraps=robot.master_control)
        old_wait=robot._wait_for_robot_stop;old_check=robot.operator_checkpoint
        camera=NS(calibration_hash='test',close=Mock(),capture=lambda check:dict(rgb=np.zeros((20,20,3),np.uint8),captured_at=time.monotonic()))
        stream=NS(start=lambda:stream,check=lambda:None,close=Mock(),after=lambda *a: (camera.capture(None),dict(uv=[9.5,9.5],angle_deg=0,scale=1,inliers=8,line=[[9,0],[9,19]]),201))
        if point_loss:stream.after=lambda *a:(camera.capture(None),dict(point_loss=True,tracking_valid=False,remaining_points=5),202)
        state=dict(geometry={'top_base_mm':[500,-100,0]},path=0.,wrist_detector=Mock(),display=None,angles=angles.copy())
        commands=[]
        def execute(robot,task,index,objects):
            nonlocal q,angles
            robot.operator_checkpoint();q=np.array(task['target_position']);angles=np.array(task['target_rotation'])
            robot._wait_for_robot_stop()
            commands.append(task);return 'object_0',True
        def fake_servo(observe,move,target,goal_z,c,**kwargs):
            observation=observe();self.assertEqual(observation['xyz_mm'][0],410.)
            if point_loss:
                self.assertTrue(observation['point_loss']);self.assertNotIn('uv',observation)
            if gpt_recovery:
                kwargs['recover']('test response drift');observe()
            move(np.array([410.1,-101.,0.]),1.)
            if not operator_wrapped:
                self.assertEqual(old_wait.call_args.kwargs['position_tolerance_m'],.0005)
                self.assertAlmostEqual(old_wait.call_args.kwargs['orientation_tolerance_rad'],np.deg2rad(.25))
            move(np.array([410.,-101.,5.]),1.)
            if not operator_wrapped:self.assertEqual(old_wait.call_args.kwargs['position_tolerance_m'],.003)
            if x_drift:
                q[0]+=3.2;kwargs['checkpoint']()
            if failed:raise ServoError('lost points')
            return dict(converged=True,actual_z_mm=goal_z,path_mm=5.1)
        with tempfile.TemporaryDirectory() as td,ExitStack() as stack:
            stack.enter_context(patch.object(runtime,'__file__',str(Path(td)/'runtime.py')))
            stack.enter_context(patch('cart_book_grasp_v3_camera.WristCamera',return_value=camera))
            stack.enter_context(patch('cart_book_grasp_v3_2_selection.select_wrist_book',return_value=({'mask':np.ones((20,20))},{})))
            stack.enter_context(patch('cart_book_grasp_v3_gpt_recovery.initialize_tracking',return_value=Mock()))
            stack.enter_context(patch.object(runtime,'TrackingStream',return_value=stream))
            stack.enter_context(patch.object(runtime,'run_raise',side_effect=fake_servo))
            def reanchor(*a,**kw):
                stream.close.assert_called_once()
                self.assertEqual(kw['reason'],'test response drift')
                return NS(last_u=9.5,last_angle=0.)
            recovery=stack.enter_context(patch('book_top_v42_gpt_recovery.reanchor',side_effect=reanchor))
            call=lambda:runtime.execute_raise(robot,{'action':'book_v41_top_raise_z','object':'object_0'},0,{},execute,None,state,config)
            if failed or x_drift:
                with self.assertRaisesRegex(ServoError,'delta=3.200mm, limit=3.000mm' if x_drift else 'lost points'):call()
                if x_drift:
                    import json
                    events=[json.loads(line) for f in Path(td).rglob('events.jsonl') for line in f.read_text().splitlines()]
                    event=next(e for e in events if e['event']=='fixed_x_drift')
                    self.assertEqual(event['reference_x_mm'],410.)
                    self.assertAlmostEqual(event['actual_tcp_mm'][0],413.2)
                robot.master_control.assert_called_once();self.assertEqual(robot.master_control.call_args.kwargs['action'],'book_v3_1_hold')
                self.assertEqual(state['path'],0.)
            else:
                self.assertTrue(call()[1]);self.assertAlmostEqual(state['path'],5.1)
                np.testing.assert_allclose(state['angles'],angles)
                robot.master_control.assert_not_called()
            self.assertEqual(commands[0]['action'],'book_v42_raise_servo')
            self.assertEqual(commands[0]['target_position'][0],410.)
        self.assertEqual(robot._wait_for_robot_stop,old_wait);self.assertEqual(robot.operator_checkpoint,old_check)
        camera.close.assert_called_once();self.assertEqual(stream.close.call_count,2 if gpt_recovery else 1)
        self.assertEqual(recovery.call_count,1 if gpt_recovery else 0)
    def test_fixed_x_drift_logs_actual_values_and_requests_hold(self):self.run_runtime(x_drift=True)
    def test_runtime_reanchor_closes_camera_worker_before_gpt(self):self.run_runtime(gpt_recovery=True)
    def test_runtime_point_loss_uses_fresh_pose_without_old_pixels(self):self.run_runtime(point_loss=True)
    def test_real_operator_wrapper_does_not_recurse(self):self.run_runtime(operator_wrapped=True)
    def test_real_operator_wrapper_failure_still_holds(self):self.run_runtime(failed=True,operator_wrapped=True)
    def test_reentrant_guard_retains_operator_interrupt(self):
        from cart_book_grasp_v1_3_pause import PauseRequested
        calls=[]
        def operator():
            calls.append(1)
            if len(calls)==2:raise PauseRequested()
        check=runtime.guarded_checkpoint(operator,lambda:check())
        with self.assertRaises(PauseRequested):check()
        self.assertEqual(len(calls),2)
    def test_guard_resets_after_failed_feedback_check(self):
        check=runtime.guarded_checkpoint(lambda:None,Mock(side_effect=ServoError('stale feedback')))
        for _ in range(2):
            with self.assertRaisesRegex(ServoError,'stale feedback'):check()
    def test_success_preserves_new_orientation(self):self.run_runtime()
    def test_failure_hold_and_cleanup(self):self.run_runtime(failed=True)


if __name__=='__main__':unittest.main()

class HistoricalLiftTests(unittest.TestCase):
    def run_history(self,*,loss_at=30.,goal=60.,path=500.,lateral=60.):
        q=np.array([410.,-100.,0.]);clock=[100.];commands=[];events=[]
        def observe():
            clock[0]+=.02
            common=dict(xyz_mm=q.copy(),roll_deg=0.,captured_at=clock[0])
            if q[2]>=loss_at:return dict(common,point_loss=True,tracking_valid=False)
            return dict(common,uv=[639.5,359.5],angle_deg=0.,scale=1.)
        def move(target,roll):
            nonlocal q
            commands.append((target.copy(),roll));q=target.copy()
            if q[2]<=loss_at:q[1]=-100.+.1*q[2]
            return True
        c=servo.load_config();c['max_lateral_mm']=lateral
        result=servo.run_raise(observe,move,639.5,goal,c,path_available=path,
            clock=lambda:clock[0],log=events.append)
        return result,commands,events
    def test_point_loss_follows_recorded_direction_to_height(self):
        result,commands,events=self.run_history()
        self.assertEqual(result['mode'],'historical_direction')
        self.assertFalse(result['visual_converged'])
        self.assertLessEqual(abs(result['actual_z_mm']-60),2.)
        switch=next(e for e in events if e['event']=='point_loss_history_switch')
        self.assertAlmostEqual(switch['slope_dy_dz'],.1)
        self.assertEqual(sum(e['event']=='point_loss_history_switch' for e in events),1)
        for q,r in commands:
            self.assertAlmostEqual(q[0],410.);self.assertEqual(r,0.)
            self.assertLessEqual(q[2],60.)
            if q[2]>30:self.assertAlmostEqual(q[1],-100.+.1*q[2])
        k=next(i for i,e in enumerate(events) if e['event']=='point_loss_history_switch')
        self.assertFalse(any(e['event']=='centerline_observation' for e in events[k:]))
    def test_insufficient_history_stops(self):
        with self.assertRaisesRegex(ServoError,'sufficient aligned lift history'):
            self.run_history(loss_at=10.)
    def test_history_keeps_budget(self):
        with self.assertRaisesRegex(ServoError,'path budget'):self.run_history(path=35.)
    def test_history_rejects_lateral_endpoint(self):
        with self.assertRaisesRegex(ServoError,'endpoint exceeds Y bound'):self.run_history(lateral=4.)
    def test_only_explicit_point_exhaustion_is_recoverable(self):
        from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted,CenterlineGeometryError
        def capture(check):
            time.sleep(.005);check();return dict(rgb=0,captured_at=time.monotonic())
        for error in (FeaturePointsExhausted(5),CenterlineGeometryError('bad geometry')):
            lock=NS(observe=Mock(side_effect=error))
            stream=TrackingStream(NS(capture=capture),lock,allow_point_loss=True).start()
            try:
                if isinstance(error,FeaturePointsExhausted):
                    a=stream.after(time.monotonic(),1)
                    b=stream.after(a[0]['captured_at'],1)
                    self.assertTrue(b[1]['point_loss']);self.assertNotIn('uv',b[1])
                    self.assertGreater(b[0]['captured_at'],a[0]['captured_at'])
                    self.assertEqual(lock.observe.call_count,1)
                else:
                    with self.assertRaisesRegex(ServoError,'bad geometry'):stream.after(time.monotonic(),1)
            finally:stream.close()
    def test_camera_staleness_remains_fatal_after_point_loss(self):
        from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted
        count=[0]
        def capture(check):
            count[0]+=1;time.sleep(.005);check()
            return dict(rgb=0,captured_at=time.monotonic()-(10 if count[0]>1 else 0))
        stream=TrackingStream(NS(capture=capture),NS(observe=Mock(side_effect=FeaturePointsExhausted(5))),allow_point_loss=True).start()
        try:
            with self.assertRaisesRegex(ServoError,'stale'):
                stream.after(time.monotonic()+.1,1)
        finally:stream.close()
