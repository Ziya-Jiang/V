"""No hardware: independent image alignment and measured-X stopping."""
import unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace
import tempfile
import time
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_distance_servo import load_config,run_distance_servo,update_jacobian
from cart_book_grasp_v3_cotracker_points import PointLock


class Plant:
    def __init__(self,goal=18.,initial_error=(18.,-10.)):
        self.c=load_config();self.q=np.array([500.,0.,0.]);self.goal=500.+goal;self.t=10.;self.moves=[]
        self.bias=np.array(initial_error);self.j=np.array([[-3.,.3],[.2,2.5]])
    def read(self):
        self.t+=.01
        uv=np.array([640.,360.])+self.bias+self.j@self.q[1:]+np.array([.25,-.12])*(self.q[0]-500.)
        return dict(uv=uv,xyz_mm=self.q.copy(),captured_at=self.t)
    def move(self,q):
        self.moves.append(np.array(q));self.q=np.array(q);return True
    def run(self,**kwargs):
        return run_distance_servo(self.read,self.move,[640,360],self.goal,self.c,clock=lambda:self.t,**kwargs)


class DistanceTests(unittest.TestCase):
    def test_centering_alone_cannot_grasp_and_depth_alone_cannot_grasp(self):
        for goal,bias in [(12.,(0,0)),(0.,(15,-10)),(18.,(18,-10))]:
            p=Plant(goal,bias);result=p.run()
            self.assertTrue(result['converged']);self.assertLessEqual(abs(p.q[0]-p.goal),1.)
            self.assertLessEqual(result['error_px'],3.)
            self.assertTrue(all(q[0]<=p.goal for q in p.moves))
            self.assertTrue(all(np.linalg.norm(b-a)<=p.c['max_step_mm']+1e-6 for a,b in zip([np.array([500.,0.,0.]),*p.moves],p.moves)))
    def test_already_at_goal_requires_three_fresh_frames_without_motion(self):
        p=Plant(0,(0,0));self.assertTrue(p.run()['converged']);self.assertEqual(p.moves,[]);self.assertAlmostEqual(p.t,10.03)
    def test_remainder_shorter_than_step_is_clamped_to_endpoint(self):
        p=Plant(5.4,(0,0));p.c['x_tolerance_mm']=.1;p.run();self.assertAlmostEqual(p.q[0],505.4)
    def test_yz_calibration_never_probes_x(self):
        p=Plant();p.run();self.assertTrue(all(q[0]==500. for q in p.moves[:8]))
    def test_probe_return_holds_measured_x_after_logged_submicron_drift(self):
        p=Plant(12,(0,0));p.q=np.array([518.4003018032647,0.,0.])
        p.goal=p.q[0]+12;read=p.read;move=p.move;reads=[0];actuals=[]
        def measured():
            reads[0]+=1
            if reads[0]==2:p.q[0]=518.3923516178457
            return read()
        def execute(goal):
            move(goal)
            if len(p.moves)==1:
                p.q+=np.array([518.3932100678295-goal[0],.226435,.085773])
            actuals.append(p.q.copy());return True
        p.read=measured;p.move=execute
        self.assertTrue(p.run()['converged'])
        self.assertAlmostEqual(p.moves[1][0],518.3932100678295)
        for i in range(1,8):self.assertEqual(p.moves[i][0],actuals[i-1][0])

    def test_probe_actual_x_outside_tolerance_still_stops(self):
        p=Plant();move=p.move
        def execute(goal):
            move(goal);p.q[0]-=1.1;return True
        p.move=execute
        with self.assertRaisesRegex(ServoError,'X outside approach interval'):p.run()
        self.assertEqual(len(p.moves),1)
    def test_logged_remaining_distance_with_yz_corrections(self):
        p=Plant();p.q=np.array([518.552,41.134,-11.871]);origin=p.q.copy()
        p.goal=731.4125265263374
        def read():
            p.t+=.01;delta=p.q-origin
            uv=np.array([640.,360.])+[12.,-8.]+p.j@delta[1:]+np.array([.08,-.04])*delta[0]
            return dict(uv=uv,xyz_mm=p.q.copy(),captured_at=p.t)
        p.read=read;result=p.run()
        self.assertTrue(result['converged'])
        self.assertLessEqual(abs(p.q[0]-p.goal),1.)
        self.assertLessEqual(result['error_px'],3.)
        lengths=[np.linalg.norm(b-a) for a,b in zip([origin,*p.moves],p.moves)]
        self.assertLessEqual(max(lengths),p.c['max_step_mm']+1e-6)
        self.assertAlmostEqual(sum(lengths),result['path_mm'])
        self.assertLessEqual(result['path_mm'],p.c['max_path_mm'])
        self.assertTrue(all(q[0]<=p.goal for q in p.moves))
    def test_in_tolerance_x_overshoot_can_finish_yz_without_advancing_x(self):
        p=Plant(-.3,(15,0));result=p.run()
        self.assertTrue(result['converged']);self.assertTrue(all(q[0]==500. for q in p.moves))
    def test_lost_tracking_stops_before_next_move(self):
        p=Plant();original=p.read
        def read():
            if len(p.moves)>=9:raise ServoError('lost points')
            return original()
        p.read=read
        with self.assertRaisesRegex(ServoError,'lost points'):p.run()
        self.assertEqual(len(p.moves),9)
    def test_stale_frame_and_failed_motion_rejected(self):
        p=Plant();ob=p.read();ob['captured_at']-=10;p.read=lambda:ob
        with self.assertRaisesRegex(ServoError,'Stale'):p.run()
        self.assertEqual(p.moves,[])
        p=Plant();p.move=Mock(return_value=False)
        with self.assertRaisesRegex(ServoError,'not confirmed'):p.run()
        p.move.assert_called_once()
    def test_wrong_waypoint_feedback_and_overshoot_do_not_grasp(self):
        p=Plant()
        def wrong(q):p.q=np.array(q)+[0,3,0];return True
        p.move=wrong
        with self.assertRaises(ServoError):p.run()
        p=Plant(-3,(0,0))
        with self.assertRaises(ServoError):p.run()
        self.assertEqual(p.moves,[])
    def test_singular_jacobian_and_large_depth_rejected(self):
        p=Plant();p.j[:]=0
        with self.assertRaisesRegex(ServoError,'ill-conditioned'):p.run()
        p=Plant(300)
        with self.assertRaisesRegex(ServoError,'budget'):p.run()
        self.assertEqual(p.moves,[])
    def test_lateral_path_and_time_budgets(self):
        p=Plant(initial_error=(200,0));p.c['max_lateral_mm']=4
        with self.assertRaisesRegex(ServoError,'travel bound'):p.run()
        p=Plant();p.c['max_path_mm']=1
        with self.assertRaisesRegex(ServoError,'path budget'):p.run()
        p=Plant();p.c['max_seconds']=.005
        with self.assertRaisesRegex(ServoError,'time budget'):p.run()

    def test_disabled_lateral_bound_allows_large_alignment_but_keeps_step_and_path_limits(self):
        p=Plant(0,(90,-60));p.c['max_lateral_mm']=None
        result=p.run()
        self.assertTrue(result['converged'])
        self.assertGreater(np.linalg.norm(p.q[1:]),20.)
        self.assertTrue(all(np.linalg.norm(b-a)<=5.000001 for a,b in zip([np.array([500.,0.,0.]),*p.moves],p.moves)))
        self.assertTrue(all(q[0]==500. for q in p.moves))
        p=Plant(0,(90,-60));p.c['max_lateral_mm']=None;p.c['max_path_mm']=20.
        with self.assertRaisesRegex(ServoError,'path budget'):p.run()

    def test_nullable_lateral_config_rejects_other_invalid_limits(self):
        import json
        for value,valid in ((None,True),(20.,True),(0,False),(-1,False),('none',False),(float('inf'),False)):
            c=load_config();c['max_lateral_mm']=value
            with tempfile.TemporaryDirectory() as directory:
                from pathlib import Path
                path=Path(directory)/'config.json';path.write_text(json.dumps(c))
                if valid:self.assertEqual(load_config(path)['max_lateral_mm'],value)
                else:
                    with self.assertRaises(ServoError):load_config(path)


class JacobianUpdateTests(unittest.TestCase):
    def test_logged_small_motion_cannot_corrupt_model_or_reverse_correction(self):
        c=load_config();j=np.array([[1.2940574989393987,-.23614664199041074],[.47746973027822764,2.3446839221545095]])
        actual=np.array([-.2737270758921859,.0717873917669074])
        delta=np.array([.4016706710743847,-.1728523706373153])
        old_candidate=j+.5*np.outer(delta-j@actual,actual)/(actual@actual)
        self.assertGreater(np.linalg.cond(old_candidate),30.)
        events=[];new=update_jacobian(j,actual,delta,4.6314330839393705,5.057935192967389,c,events.append)
        np.testing.assert_array_equal(new,j)
        self.assertEqual(events[-1]['reason'],'insufficient_motion')
        error=np.array([643.6190317542632,356.5646489437157])-[639.5,359.5]
        dq=-c['gain']*np.linalg.solve(new.T@new+c['damping']**2*np.eye(2),new.T@error)
        self.assertLess(dq[0],0.);self.assertGreater(dq[1],0.)

    def test_near_target_freezes_even_with_large_resolved_motion(self):
        j=np.eye(2)*2;events=[]
        new=update_jacobian(j,[2,0],[4.4,.1],12.,9.,load_config(),events.append)
        np.testing.assert_array_equal(new,j);self.assertEqual(events[-1]['reason'],'near_target')

    def test_condition_degradation_keeps_last_model(self):
        j=np.diag([1.,5.]);events=[]
        new=update_jacobian(j,[2,0],[.2,0],40.,39.,load_config(),events.append)
        np.testing.assert_array_equal(new,j);self.assertEqual(events[-1]['reason'],'condition_worsened')

    def test_excessive_update_keeps_last_model(self):
        j=np.eye(2)*2;events=[]
        new=update_jacobian(j,[2,0],[7.,0],40.,35.,load_config(),events.append)
        np.testing.assert_array_equal(new,j);self.assertEqual(events[-1]['reason'],'update_too_large')

    def test_good_resolved_response_can_update(self):
        j=np.eye(2)*2;events=[]
        new=update_jacobian(j,[3,0],[6.6,.3],40.,34.,load_config(),events.append)
        self.assertTrue(events[-1]['accepted'])
        self.assertLess(np.linalg.norm(np.array([6.6,.3])-new@np.array([3,0])),np.linalg.norm([.6,.3]))
        np.testing.assert_array_equal(j,np.eye(2)*2)

    def test_response_mismatch_still_stops_when_update_would_be_skipped(self):
        for motion,before,after in (([.2,0],5.,6.),([2,0],5.,6.),([2,0],40.,35.)):
            events=[]
            with self.assertRaisesRegex(ServoError,'residual='):
                update_jacobian(np.eye(2)*2,motion,[12,0],before,after,load_config(),events.append)
            self.assertEqual(events[-1]['reason'],'response_mismatch')

    def test_controller_uses_frozen_model_near_target_and_converges(self):
        p=Plant(0,(15,-10));events=[]
        self.assertTrue(p.run(log=events.append)['converged'])
        updates=[e for e in events if e['event']=='jacobian_update']
        self.assertTrue(updates)
        self.assertTrue(any(e.get('reason') in ('near_target','insufficient_motion') for e in updates))
        self.assertTrue(all(q[0]==500. for q in p.moves))


class PointTests(unittest.TestCase):
    def lock(self):
        c=load_config();c['window_frames']=3
        points=np.float32([[30,30],[50,30],[70,30],[30,60],[50,60],[70,60]])
        image=np.zeros((100,120,3),np.uint8);model=Mock()
        with patch('cart_book_grasp_v3_cotracker_points.initialize_points',return_value=points):
            lock=PointLock(model,image,np.ones((100,120),bool),c)
        return lock,model,image,points
    def test_fixed_point_ids_survive_window_slide(self):
        lock,m,image,p=self.lock()
        m.predict.return_value=(np.stack([p,p+[2,1],p+[4,2]]),np.ones((3,len(p)),bool))
        lock.frames.append(image)
        result=lock.observe(image)
        np.testing.assert_allclose(result['uv'],p.mean(0)+[4,2],atol=.001)
        np.testing.assert_allclose(lock.queries,p+[2,1]);np.testing.assert_array_equal(lock.reference,p)
        self.assertEqual(len(lock.frames),2)
    def test_invisible_points_never_reinitialize_to_neighbor(self):
        lock,m,image,p=self.lock();m.predict.return_value=(np.stack([p,p]),np.zeros((2,len(p)),bool))
        with self.assertRaisesRegex(ServoError,'visibility lost'):lock.observe(image)
        np.testing.assert_array_equal(lock.reference,p)


class RuntimeTests(unittest.TestCase):
    def run_runtime(self,lose=False,*,start_x=400.,surface_x=600.,coarse_actual_x=None,column=False):
        from cart_book_grasp_v3_cotracker_runtime import execute_cotracker_approach
        import cart_book_grasp_v1_3_motion as motion
        q=np.array([start_x,0.,0.]);angles=np.array([0.,-90.,0.]);commands=[]
        robot=SimpleNamespace(operator_checkpoint=lambda:None,operator_sleep=lambda s:None,
            get_current_state=lambda:dict(joint_state=np.zeros(14),right_arm=dict(position=q/1000)))
        step5=SimpleNamespace(current_tool_orientation_as_euler=lambda *a:angles)
        rgb=np.zeros((720,1280,3),np.uint8);mask=np.zeros((720,1280),bool);mask[120:500,610:690]=True
        camera=SimpleNamespace(config={'max_joint_drift_deg':.5},calibration_hash='test',
                               capture=lambda *a:dict(rgb=rgb,captured_at=time.monotonic()))
        def execute(robot,task,*a,**k):
            commands.append(task);q[:]=task['target_position']
            if len(commands)==1 and coarse_actual_x is not None:q[0]=coarse_actual_x
            return 'book',True
        points=np.float32([[620,200],[650,200],[680,200],[620,300],[650,300],[680,300]])
        def observe(image):
            if lose:raise ServoError('lost points')
            return dict(uv=np.array([639.5,359.5]),points=points,visible=np.ones(6,bool),
                        inliers=6,scale=1.,inference_seconds=.01)
        # Pixel response must reflect the injected measured Y/Z motion.
        def responsive(image):
            o=observe(image);o['uv']=o['uv']+np.array([-3*q[1],2*q[2]]);return o
        tracker=SimpleNamespace(reference=points,anchor=points.mean(0),observe=responsive)
        c=load_config();context={}
        if column:
            from cart_book_grasp_v3_column import load_config as load_column
            c=load_column();c['mode']='column_distance'
        with tempfile.TemporaryDirectory() as out, \
             patch('cart_book_grasp_v3_cotracker_runtime.PointLock',return_value=tracker), \
             patch('cart_book_grasp_v3_column.ColumnLock',return_value=tracker), \
             patch('cart_book_grasp_v3_cotracker_runtime.cv2.imwrite',return_value=True), \
             patch.object(motion,'_load_cart_plus_parameters',return_value=dict(fingertip_offset_mm=50.,grasp_insertion_mm=100.,forward_step_mm=125.)), \
             patch('cart_book_grasp_v3_final_tracking.FinalImageTracker.from_config',side_effect=AssertionError('Old final image must not load')):
            try:
                result=execute_cotracker_approach(robot,dict(object='book',arm='right',action='move_down_to_grasp',book_surface_x_mm=surface_x),
                    0,{},step5,execute,c,model=object(),wrist_detector=lambda image:[(.95,mask,[])],
                    camera=camera,retreat_context=context,output_root=out,column_mode=column)
            except ServoError:
                if not lose:raise
                result=None
        return result,commands,context
    def test_depth_endpoint_and_handoff_without_head_or_final_image(self):
        result,commands,context=self.run_runtime()
        self.assertEqual(result,('book',True));self.assertTrue(context['book']['forward_complete'])
        self.assertAlmostEqual(commands[0]['target_position'][0],525.)
        self.assertLessEqual(abs(context['book']['visual_final_xyz_mm'][0]-650.),1.)
        self.assertTrue(all(t['target_position'][0]<=650. for t in commands))
        self.assertEqual(context['book']['start_x_mm'],400.)
    def test_lost_points_after_coarse_stage_blocks_grasp(self):
        result,commands,context=self.run_runtime(lose=True)
        self.assertIsNone(result);self.assertEqual(len(commands),1);self.assertFalse(context['book']['forward_complete'])
    def test_logged_endpoint_passes_runtime_budget_and_converges(self):
        goal=731.4125265263374
        # This fixture's tool orientation gives exactly 50 mm fingertip X offset.
        result,commands,context=self.run_runtime(start_x=goal-319.8635111389573,
            surface_x=goal-50.,coarse_actual_x=518.552)
        self.assertEqual(result,('book',True));self.assertTrue(context['book']['forward_complete'])
        self.assertLessEqual(abs(context['book']['visual_final_xyz_mm'][0]-goal),1.)
        self.assertTrue(all(t['target_position'][0]<=goal for t in commands))
    def test_default_entry_dispatches_to_new_mode(self):
        import V3
        with patch('cart_book_grasp_v3_column_main.main') as new:
            V3.main(['--check-config']);new.assert_called_once_with(['--check-config'])
    def test_installed_hook_uses_cotracker_not_fixed_forward(self):
        import cart_book_grasp_v1_3_motion as motion
        from cart_book_grasp_v3_motion import installed
        from cart_book_grasp_v3_distance_servo import CONFIG_PATH
        with patch.object(motion,'_execute_cart_plus_forward') as old, \
             patch('cart_book_grasp_v3_cotracker_runtime.execute_cotracker_approach',return_value=('book',True)) as new:
            with installed(None,cotracker_path=CONFIG_PATH,cotracker_model=object()):
                result=motion._execute_cart_plus_forward(None,{},0,{},None,None)
            self.assertEqual(result,('book',True));old.assert_not_called();new.assert_called_once()


if __name__=='__main__':unittest.main()
