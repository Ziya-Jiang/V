import time
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_column import load_config
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted
from cart_book_grasp_v3_direct_distance import run_direct_distance


class DirectTests(unittest.TestCase):
    def simulate(self,failure=None):
        c=load_config();q=np.array([704.,26.,-11.]);angles=np.array([-88.,9.,95.]);t=[1.];moves=[]
        def read():
            t[0]+=.02
            return dict(xyz_mm=q.copy(),angles_deg=angles.copy(),received_monotonic=t[0] if failure!='stale' else -1.)
        def move(goal,orientation):
            moves.append((goal.copy(),orientation.copy()));q[:]=goal
            if failure=='drift':q[1]+=2.1
            if failure=='rotation':angles[0]+=2.
            return failure!='motion'
        result=run_direct_distance(read,move,733.5,c,path_used=399. if failure=='path' else 216.,clock=lambda:t[0],sleep=lambda x:None)
        return result,moves

    def test_remaining_distance_keeps_y_z_orientation_and_five_mm_steps(self):
        result,moves=self.simulate()
        self.assertTrue(result['distance_complete']);self.assertFalse(result['visual_alignment_verified'])
        self.assertNotIn('converged',result)
        last=np.array([704.,26.,-11.])
        for goal,angles in moves:
            np.testing.assert_array_equal(goal[1:],[26.,-11.]);np.testing.assert_array_equal(angles,[-88.,9.,95.])
            self.assertLessEqual(np.linalg.norm(goal-last),5.000001);self.assertGreater(goal[0],last[0]);last=goal
        self.assertLessEqual(abs(last[0]-733.5),2.)
        self.assertLessEqual(result['path_mm'],400.)

    def test_other_failures_and_budget_still_stop(self):
        for failure in ('motion','drift','rotation','stale','path'):
            with self.subTest(failure=failure),self.assertRaises(ServoError):self.simulate(failure)

    def test_repeated_feedback_and_shared_timeout_stop(self):
        c=load_config();t=[1.];moves=[]
        def read():return dict(xyz_mm=[733.5,0,0],angles_deg=[0,0,0],received_monotonic=1.)
        def sleep(dt):t[0]+=dt
        with self.assertRaisesRegex(ServoError,'feedback'):
            run_direct_distance(read,lambda *a:moves.append(a),733.5,c,clock=lambda:t[0],sleep=sleep)
        with self.assertRaisesRegex(ServoError,'time budget'):
            run_direct_distance(read,lambda *a:moves.append(a),733.5,c,started=-1000,clock=lambda:t[0])
        self.assertEqual(moves,[])

    def runtime(self,failure):
        from cart_book_grasp_v3_cotracker_runtime import execute_cotracker_approach
        q=np.array([400.,0.,17.]);angles=np.array([-88.7,2.6,94.8]);commands=[];captures=[];observations=[];context={}
        def state():
            quat=Rotation.from_euler('xyz',angles,degrees=True).as_quat()
            return dict(right_arm=dict(position=(q/1000).tolist(),orientation=np.r_[quat[3],quat[:3]].tolist()),received_monotonic=time.monotonic())
        robot=SimpleNamespace(operator_checkpoint=lambda:None,operator_sleep=lambda x:None,get_current_state=state)
        rgb=np.zeros((720,1280,3),np.uint8)
        def capture(check):captures.append(1);return dict(rgb=rgb,captured_at=time.monotonic())
        camera=SimpleNamespace(config={'max_joint_drift_deg':.5},calibration_hash='test',capture=capture)
        def execute(robot,task,*args,**kwargs):
            commands.append(task)
            if failure=='direct_motion' and task['description'].startswith('Direct X'):return 'book',False
            q[:]=task['target_position'];angles[:]=task['target_rotation'];return 'book',True
        def observe(frame):
            observations.append(1)
            if len(observations)>=2:
                if failure=='geometry':raise ServoError('Geometry inconsistent')
                if failure=='camera':raise ValueError('Camera disconnected')
                raise FeaturePointsExhausted(5)
            return dict(uv=np.array([650.,359.5]),angle_deg=1.,line=np.array([[650,0],[650,719]]),
                scale=1.,points=np.array([[650,350]]*8),visible=np.ones(8,bool),inliers=8,inference_seconds=.001)
        tracker=SimpleNamespace(reference=np.array([[650,350]]*8),anchor=np.array([650,359]),observe=observe)
        with tempfile.TemporaryDirectory() as directory, \
             patch('cart_book_grasp_v1_3_motion._right_tool_mm_euler',side_effect=lambda *a:(q.copy(),angles.copy())), \
             patch('cart_book_grasp_v1_3_motion._load_cart_plus_parameters',return_value={}), \
             patch('cart_book_grasp_v1_3_depth.forward_parameters',return_value={'grasp_tcp_x_mm':520.}), \
             patch('cart_book_grasp_v1_3_visualization.forward_targets',return_value=[np.array([500.,0.,17.]),np.array([520.,0.,17.])]), \
             patch('cart_book_grasp_v3_cotracker_runtime.wait_for_coarse_pose'), \
             patch('cart_book_grasp_v3_cotracker_runtime.stationary_guard',return_value=lambda:None), \
             patch('cart_book_grasp_v3_cotracker_runtime.select_center_book',return_value=({'mask':np.ones((720,1280),bool)},{})), \
             patch('cart_book_grasp_v3_centerline.CenterlineLock',return_value=tracker):
            try:result=execute_cotracker_approach(robot,{'object':'book'},2,{},None,execute,load_config(),model=None,
                wrist_detector=lambda rgb:[],column_mode=True,camera=camera,output_root=directory,retreat_context=context)
            except (ServoError,ValueError) as error:result=error
        return result,commands,captures,context

    def test_runtime_switches_only_for_point_count_without_further_capture(self):
        result,commands,captures,context=self.runtime('points')
        self.assertEqual(result,('book',True));self.assertEqual(len(captures),3)
        direct=[x for x in commands if x['description'].startswith('Direct X')]
        self.assertGreater(len(direct),0)
        self.assertTrue(all(x['target_position'][1:]==[3.,17.] for x in direct))
        self.assertTrue(context['book']['forward_complete']);self.assertFalse(context['book']['visual_alignment_verified'])
        self.assertEqual(context['book']['completion_mode'],'direct_distance_after_point_loss')
        self.assertNotIn('visual_final_xyz_mm',context['book'])

    def test_runtime_never_falls_back_on_unrelated_error_or_finishes_failed_motion(self):
        for failure in ('geometry','camera','direct_motion'):
            result,commands,captures,context=self.runtime(failure)
            self.assertIsInstance(result,Exception);self.assertFalse(context['book']['forward_complete'])
            if failure!='direct_motion':self.assertFalse(any(x['description'].startswith('Direct X') for x in commands))


if __name__=='__main__':unittest.main()
