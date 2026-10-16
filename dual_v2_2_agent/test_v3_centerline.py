import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch, Mock
from types import SimpleNamespace
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_centerline import CenterlineLock,visible_centerline,roll_orientation,measured_roll,run_centerline_servo
from cart_book_grasp_v3_column import load_config
from cart_book_grasp_v3_servo import ServoError


class CenterlineTests(unittest.TestCase):
    def mask(self,top=0,bottom=240):
        mask=np.zeros((240,320),np.uint8)
        for y in range(top,bottom):
            x=int(165+.23*(y-119.5));mask[y,x-20:x+21]=1
        return mask

    def test_tilt_and_vertical_clipping_preserve_centerline(self):
        for top,bottom in ((0,240),(0,120),(140,240)):
            u,angle=visible_centerline(self.mask(top,bottom))
            self.assertAlmostEqual(u,164.5,delta=.6)
            self.assertAlmostEqual(angle,np.degrees(np.arctan(.23)),delta=.3)
        with self.assertRaises(ServoError):visible_centerline(np.ones((240,320),np.uint8))

    def test_centerline_lock_tracks_texture_and_tilt(self):
        from cart_book_grasp_v3_feature_centerline import FeatureCenterlineLock,FeaturePointsExhausted
        self.assertIs(CenterlineLock,FeatureCenterlineLock)
        rgb=np.random.default_rng(42).integers(0,256,(240,320,3),dtype=np.uint8)
        lock=CenterlineLock(rgb,self.mask());u,angle=visible_centerline(self.mask())
        for degrees in (2,4,6):
            matrix=cv2.getRotationMatrix2D((159.5,119.5),degrees,1.)
            matrix[:,2]+=[3,2]
            out=lock.observe(cv2.warpAffine(rgb,matrix,(320,240)))
            self.assertAlmostEqual(out['angle_deg'],angle+degrees,delta=.6)
        with self.assertRaises(FeaturePointsExhausted):lock.observe(np.zeros_like(rgb))

    def test_tool_roll_preserves_grasp_axis_and_detects_off_axis(self):
        base=np.array([-88.7,2.6,94.8])
        for roll in (-30,-2,0,3,30):
            angles=roll_orientation(base,roll)
            np.testing.assert_allclose(Rotation.from_euler('xyz',angles,degrees=True).as_matrix()[:,2],
                                      Rotation.from_euler('xyz',base,degrees=True).as_matrix()[:,2],atol=1e-12)
            self.assertAlmostEqual(measured_roll(base,angles,1),roll,places=8)
        with self.assertRaises(ServoError):measured_roll(base,base+[5,0,0],1)

    def simulate(self, failure=None, reverse=False):
        c=load_config();q=np.array([500.,0.,17.]);roll=[0.];t=[10.];moves=[]
        jac=np.array([[2.,.8],[.05,-1. if reverse else 1.]])
        if failure=='singular':jac[:,1]=jac[:,0]
        def observe():
            t[0]+=.01;f=np.array([50.,12.])+jac@np.array([q[1],roll[0]])
            if failure=='loss' and len(moves)>2:raise ServoError('lost tracking')
            return dict(uv=[160+f[0],-9999],angle_deg=f[1],roll_deg=roll[0],scale=1.,xyz_mm=q.copy(),captured_at=t[0] if failure!='stale' else -10.)
        def move(goal,r):
            moves.append((goal.copy(),r));q[:]=goal;roll[0]=r
            if failure=='height':q[2]+=c['max_pose_error_mm']+.2
            if failure=='rotation':roll[0]+=1.2
            return failure!='motion'
        if failure=='bound':c['max_roll_deg']=3.
        if failure=='path':c['max_path_mm']=1.
        result=run_centerline_servo(observe,move,160.,520.,c,clock=lambda:t[0])
        return result,moves,q,roll,jac

    def test_coupled_y_roll_converges_both_rotation_directions_fixed_z(self):
        for reverse in (False,True):
            result,moves,q,r,jac=self.simulate(reverse=reverse)
            self.assertTrue(result['converged']);self.assertLessEqual(abs(q[0]-520),1)
            residual=np.array([50.,12.])+jac@np.array([q[1],r[0]])
            self.assertLessEqual(abs(residual[0]),3);self.assertLessEqual(abs(residual[1]),1.5)
            previous=np.array([500.,0.,17.]);angle=0
            for goal,roll in moves:
                self.assertEqual(goal[2],17.)
                self.assertLessEqual(np.linalg.norm(goal-previous),5.000001)
                self.assertLessEqual(abs(roll-angle),3.000001)
                previous=goal;angle=roll
            self.assertTrue(any(abs(b[1]-a[1])>.1 and abs(b[0][1]-a[0][1])>.1 for a,b in zip(moves[2:],moves[3:])))
            # Probe 2 does not waste a Y return-to-origin command.
            self.assertEqual(moves[0][0][1],moves[1][0][1])

    def test_loss_motion_height_rotation_stale_singular_and_budgets_stop(self):
        for failure in ('loss','motion','height','rotation','stale','singular','bound','path'):
            with self.subTest(failure=failure),self.assertRaises(ServoError):self.simulate(failure)

    def test_post_capture_borderline_arrival_reports_actual_error(self):
        c=load_config();c['max_pose_error_mm']=1.;t=[0.];q=np.array([518.625554,13.361353,-40.792580]);r=[0.];events=[]
        def observe():
            t[0]+=.01
            return dict(uv=[200,0],angle_deg=1,roll_deg=r[0],scale=1.,xyz_mm=q.copy(),captured_at=t[0])
        def move(goal,roll):
            q[:]=goal+[-.0163576476,.9450463992,-.3641234726];r[0]=roll;return True
        with self.assertRaisesRegex(ServoError,r'position=1.0129mm.*limit=1.0000mm.*actual_mm='):
            run_centerline_servo(observe,move,160,600,c,clock=lambda:t[0],log=events.append)
        error=[e for e in events if e['event']=='centerline_waypoint_error'][0]
        self.assertGreater(error['position_error_mm'],1.)
        self.assertLess(error['position_error_mm'],1.02)

    def test_two_mm_accepts_logged_error_and_rechecks_transient_excursion(self):
        # Start visually aligned so only X advances. Observations after a move
        # alternate just outside 2 mm and the recorded 1.0894 mm residual.
        c=load_config();q=np.array([500.,0.,17.]);t=[0.];moves=[];events=[];pending=[False]
        def observe():
            t[0]+=.01;actual=q.copy()
            if moves:
                actual[1]=2.1 if pending[0] else 1.0114311427
                actual[2]-=0.4048306692
                pending[0]=False
            return dict(uv=[160,0],angle_deg=0.,roll_deg=0.,scale=1.,xyz_mm=actual,captured_at=t[0])
        def move(goal,roll):
            moves.append(goal.copy());q[:]=goal;q[1]=0.;pending[0]=True;return True
        result=run_centerline_servo(observe,move,160,505,c,clock=lambda:t[0],log=events.append)
        self.assertTrue(result['converged']);self.assertEqual(len(moves),1)
        self.assertEqual(len([e for e in events if e['event']=='centerline_arrival_recheck']),1)
        self.assertFalse(any(e['event']=='jacobian_y_roll' for e in events))

    def test_actionable_step_respects_model_and_rotational_bounds(self):
        from cart_book_grasp_v3_centerline import actionable_correction
        c=load_config();j=np.array([[2.14,-2.41],[-.099,-1.026]])
        f=np.array([5.4939,-.2092]);tol=np.array([3.,1.5])
        correction=np.array([-1.1832,.0606])
        out=actionable_correction(correction,j,f,tol,5.,c)
        self.assertAlmostEqual(abs(out[0]),2.5)
        self.assertTrue((np.abs(f+j@out)<=tol).all())
        # A steep image response must not force a harmful minimum step.
        j=np.diag([20.,1.]);f=np.array([3.1,0.]);small=np.array([-.0775,0.])
        np.testing.assert_array_equal(actionable_correction(small,j,f,tol,5.,c),small)

    def test_closed_loop_with_one_mm_execution_residual_still_converges(self):
        c=load_config();q=np.array([500.,0.,17.]);r=[0.];t=[0.];moves=[]
        def observe():
            t[0]+=.01
            return dict(uv=[160+50+2.14*q[1]-.8*r[0],0],angle_deg=4-r[0],roll_deg=r[0],scale=1.,xyz_mm=q.copy(),captured_at=t[0])
        def move(goal,roll):
            delta=goal-q;length=np.linalg.norm(delta)
            moves.append(goal.copy());q[:]+=delta*max(0.,1-.9/max(length,1e-8));r[0]=roll;return True
        result=run_centerline_servo(observe,move,160,520,c,clock=lambda:t[0])
        self.assertTrue(result['converged']);self.assertLessEqual(result['error_px'],3.)
        self.assertLess(len(moves),30)

    def test_persistent_post_capture_error_over_two_mm_stops_without_more_motion(self):
        c=load_config();t=[0.];q=np.array([500.,0.,17.]);moves=[];events=[]
        def observe():
            t[0]+=.01;actual=q.copy()
            if moves:actual[1]+=2.1
            return dict(uv=[160,0],angle_deg=0.,roll_deg=0.,scale=1.,xyz_mm=actual,captured_at=t[0])
        def move(goal,roll):moves.append(goal);q[:]=goal;return True
        with self.assertRaisesRegex(ServoError,'position=2.1000mm'):
            run_centerline_servo(observe,move,160,510,c,clock=lambda:t[0],log=events.append)
        self.assertEqual(len(moves),1)
        self.assertEqual(len([e for e in events if e['event']=='centerline_arrival_recheck']),2)

    def test_already_done_needs_no_motion(self):
        t=[1.]
        def observe():
            t[0]+=.01
            return dict(uv=[160,9999],angle_deg=0.,roll_deg=0.,scale=1.,xyz_mm=[500,0,17],captured_at=t[0])
        def move(*args):self.fail('Unnecessary move')
        self.assertTrue(run_centerline_servo(observe,move,160,500,load_config(),clock=lambda:t[0])['converged'])

    def test_runtime_commands_rotated_orientation_and_fixed_tcp_height(self):
        from cart_book_grasp_v3_cotracker_runtime import execute_cotracker_approach
        q=np.array([400.,0.,17.]);base=np.array([-88.7,2.6,94.8]);angles=base.copy();commands=[]
        robot=SimpleNamespace(operator_checkpoint=lambda:None,operator_sleep=lambda x:None)
        frame=np.zeros((720,1280,3),np.uint8)
        import time
        camera=SimpleNamespace(config={'max_joint_drift_deg':.5},calibration_hash='test',capture=lambda f:dict(rgb=frame,captured_at=time.monotonic()))
        def execute(robot,task,*args,**kwargs):
            commands.append(task)
            if task.get('action')=='book_v3_xyz_servo':self.assertEqual(robot._v3_servo_position_tolerance_mm,2.)
            q[:]=task['target_position'];angles[:]=task['target_rotation'];return 'book',True
        def observe(rgb):
            roll=measured_roll(base,angles,1)
            return dict(uv=np.array([639.5+20+2*q[1]+.5*roll,359.5]),angle_deg=8+roll,
                        line=np.array([[639,0],[639,719]]),scale=1.,points=np.array([[639,350]]*8),
                        visible=np.ones(8,bool),inliers=8,inference_seconds=.001)
        tracker=SimpleNamespace(reference=np.array([[639,350]]*8),anchor=np.array([639,359]),observe=observe)
        mask=np.ones((720,1280),bool)
        c=load_config();context={};display=Mock()
        self.assertEqual(c['snapshot_every'],200)
        with tempfile.TemporaryDirectory() as directory, \
             patch('cart_book_grasp_v1_3_motion._right_tool_mm_euler',side_effect=lambda *a:(q.copy(),angles.copy())), \
             patch('cart_book_grasp_v1_3_motion._load_cart_plus_parameters',return_value={}), \
             patch('cart_book_grasp_v1_3_depth.forward_parameters',return_value={'grasp_tcp_x_mm':520.}), \
             patch('cart_book_grasp_v1_3_visualization.forward_targets',return_value=[np.array([500.,0.,17.]),np.array([520.,0.,17.])]), \
             patch('cart_book_grasp_v3_cotracker_runtime.wait_for_coarse_pose'), \
             patch('cart_book_grasp_v3_cotracker_runtime.stationary_guard',return_value=lambda:None), \
             patch('cart_book_grasp_v3_cotracker_runtime.select_center_book',return_value=({'mask':mask},{})), \
             patch('cart_book_grasp_v3_centerline.CenterlineLock',return_value=tracker):
            result=execute_cotracker_approach(robot,{'object':'book'},2,{},None,execute,c,model=None,wrist_detector=lambda rgb:[],column_mode=True,camera=camera,output_root=directory,retreat_context=context,display=display)
            self.assertEqual(result,('book',True));self.assertTrue(context['book']['forward_complete'])
            self.assertFalse(hasattr(robot,'_v3_servo_position_tolerance_mm'))
            self.assertEqual(len(list(Path(directory).rglob('frame_*.png'))),1)
            self.assertEqual([call.args[0] for call in display.safe.call_args_list],['log_image','flush_book'])
        self.assertTrue(all(t['target_position'][2]==17. for t in commands))
        self.assertGreater(abs(measured_roll(base,commands[-1]['target_rotation'],1)),4.)


if __name__=='__main__':unittest.main()
