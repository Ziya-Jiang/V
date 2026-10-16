"""Offline XYZ convergence, final-image matching, and V1_3 stage handoff."""
import copy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError
import cart_book_grasp_v3_xyz_servo as control
import cart_book_grasp_v3_approach as runtime
from cart_book_grasp_v3_final_tracking import FinalImageTracker
from cart_book_grasp_v3_camera import WristCamera
from cart_book_grasp_v3_tracking import acquire_selected_spine


def config():return copy.deepcopy(control.load_approach())


class Plant:
    def __init__(self,nonlinear=False,goal=None):
        self.c=config();self.target=np.asarray(self.c['reference']['points'],float)
        self.goal=np.array([30.,2.,-1.] if goal is None else goal)
        self.q=np.zeros(3);self.moves=[];self.t=1.;self.nonlinear=nonlinear
        center=self.target.mean(axis=0)
        self.j=np.array([[.03*(p[0]-center[0]),-3.,.4] if a==0 else
                         [.03*(p[1]-center[1]),.2,2.5] for p in self.target for a in (0,1)])
    def read(self):
        self.t+=.01
        if self.nonlinear:
            depth=250.+self.goal[0]-self.q[0]
            xy=(self.target-[640.,360.])*250./680.
            xy=xy+[-(self.q[1]-self.goal[1]),self.q[2]-self.goal[2]]
            points=xy*680./depth+[640.,360.]
        else:points=self.target+(self.j@(self.q-self.goal)).reshape(4,2)
        return dict(points=points,xyz_mm=self.q.copy(),captured_at=self.t)
    def move(self,goal):
        self.moves.append(np.array(goal).copy());self.q=np.array(goal).copy();return True
    def run(self,**kw):return control.run_xyz(self.read,self.move,self.c,clock=lambda:self.t,**kw)


class XYZTests(unittest.TestCase):
    def test_linear_coupled_xyz_converges_with_fixed_feature_ids(self):
        p=Plant();r=p.run()
        self.assertTrue(r['converged']);self.assertLessEqual(r['error_px'],p.c['tolerance_px'])
        self.assertGreater(p.q[0],20.);self.assertLessEqual(r['path_mm'],p.c['max_path_mm'])
        self.assertLessEqual(max(np.linalg.norm(b-a) for a,b in zip([np.zeros(3),*p.moves],p.moves)),2.00001)
    def test_perspective_depth_changes_converge_with_online_update(self):
        p=Plant(nonlinear=True,goal=[60.,2.,-1.]);r=p.run()
        self.assertTrue(r['converged']);self.assertGreater(p.q[0],45.)
        self.assertLessEqual(r['error_px'],3.)
    def test_aligned_needs_three_fresh_frames_without_probing(self):
        p=Plant(goal=[0,0,0]);r=p.run();self.assertTrue(r['converged']);self.assertEqual(p.moves,[])
        self.assertAlmostEqual(p.t,1.03)
    def test_point_translation_without_scale_cannot_control_X(self):
        p=Plant();p.j[:,0]=0
        with self.assertRaisesRegex(ServoError,'ill-conditioned'):p.run()
        self.assertEqual(len(p.moves),12)
    def test_stale_and_failed_motion_do_not_continue(self):
        p=Plant();obs=p.read();obs['captured_at']-=10
        with self.assertRaises(ServoError):control.run_xyz(lambda:obs,p.move,p.c,clock=lambda:p.t)
        self.assertEqual(p.moves,[])
        calls=Mock(return_value=False)
        with self.assertRaisesRegex(ServoError,'not confirmed'):control.run_xyz(p.read,calls,p.c,clock=lambda:p.t)
        calls.assert_called_once()
    def test_motion_bounds_and_lost_target_abort(self):
        p=Plant(goal=[200,0,0]);p.c['max_forward_mm']=4
        with self.assertRaisesRegex(ServoError,'bound'):p.run()
        self.assertLessEqual(max(q[0] for q in p.moves),4.)
        p=Plant();read=p.read
        def lose():
            if len(p.moves)>12:raise ServoError('lost target')
            return read()
        with self.assertRaisesRegex(ServoError,'lost target'):
            control.run_xyz(lose,p.move,p.c,clock=lambda:p.t)
        self.assertEqual(len(p.moves),13)
    def test_invalid_shape_and_nonfinite_values_rejected(self):
        p=Plant()
        with self.assertRaises(ServoError):control.checked_jacobian(np.zeros((2,3)),p.c)
        with self.assertRaises(ServoError):control.fit_xyz(np.zeros((6,3)),np.ones((6,8)),p.c)
        obs=p.read();obs['points'][0,0]=np.nan
        with self.assertRaises(ServoError):control.run_xyz(lambda:obs,p.move,p.c,clock=lambda:p.t)
        self.assertEqual(p.moves,[])
    def test_real_final_and_observation_images_match_but_blank_does_not(self):
        c=config();camera=WristCamera();t=FinalImageTracker.from_config(c,camera,Path(__file__).parent)
        root=Path(__file__).parent
        reference=cv2.cvtColor(cv2.imread(str(root/c['reference']['image'])),cv2.COLOR_BGR2RGB)
        result=t.observe(reference)
        np.testing.assert_allclose(result['points'],t.target,atol=.5)
        observation=cv2.cvtColor(cv2.imread(str(root/'RobotRAM/output/v3_observation_before_stage2_20261009.png')),cv2.COLOR_BGR2RGB)
        result=t.observe(observation)
        self.assertGreaterEqual(result['inliers'],12);self.assertLess(result['reprojection_rms_px'],1.5)
        self.assertLess(result['area_ratio'],.1)
        with self.assertRaises(ServoError):t.observe(np.zeros_like(observation))
    def test_changed_final_image_hash_rejected(self):
        c=config();c['reference']['image_sha256']='0'*64
        with self.assertRaisesRegex(ServoError,'changed'):
            FinalImageTracker.from_config(c,WristCamera(),Path(__file__).parent)

    def test_tiny_head_spine_reports_new_backend_instead_of_sift_16(self):
        head=np.zeros((120,160,3),np.uint8)
        mask=np.zeros(head.shape[:2],bool);mask[20:95,70:80]=True
        backend=Mock()
        backend.correspondences.return_value=[('full',np.empty((0,2)),np.empty((0,2)),0)]
        with patch('cart_book_grasp_v3_superpoint.get_backend',return_value=backend), \
             self.assertRaisesRegex(ServoError,r'Head-to-wrist.*SuperPoint.*features=0.*matches=0'):
            acquire_selected_spine(dict(rgb=head,selected=dict(mask=mask,uv=[75,50])),head,backend='superpoint')


class CoarsePoseTests(unittest.TestCase):
    def wait(self, samples, *, checkpoint=lambda:None):
        c=config();now=[0.];index=[0]
        target=np.array([525.,-213.,-10.]);angles=np.array([0.,-90.,0.])
        def sleep(seconds):now[0]+=seconds
        def read():
            sample=samples[min(index[0],len(samples)-1)];index[0]+=1
            return None if sample is None else (target+np.array(sample[0]),angles+np.array(sample[1]))
        result=runtime.wait_for_coarse_pose(read,target,angles,c,sleep=sleep,
            checkpoint=checkpoint,clock=lambda:now[0])
        return result,now[0],index[0]

    def test_delayed_feedback_then_stable_coarse_residual_is_accepted(self):
        result,elapsed,reads=self.wait([None,([-20,0,0],[0,0,0]),([3,0,0],[0,0,0])])
        np.testing.assert_allclose(result[0],[528,-213,-10])
        self.assertGreaterEqual(elapsed,.7);self.assertEqual(reads,5)
        self.assertEqual(config()['max_pose_error_mm'],1.)

    def test_excessive_position_error_reports_actual_delta_and_threshold(self):
        with self.assertRaisesRegex(ServoError,r'actual_mm=.*position_error=9.000mm \(limit=8.000mm\)'):
            self.wait([([9,0,0],[0,0,0])])

    def test_coarse_orientation_limit_still_applies(self):
        with self.assertRaisesRegex(ServoError,'orientation_error=2.000deg'):
            self.wait([([0,0,0],[2,0,0])])

    def test_motion_within_tolerance_must_settle_before_handoff(self):
        with self.assertRaisesRegex(ServoError,'stable_samples=1/3'):
            self.wait([([(-1)**i*3,0,0],[0,0,0]) for i in range(100)])

    def test_interrupt_and_missing_feedback_stop_handoff(self):
        with self.assertRaisesRegex(ServoError,'No measured TCP feedback'):
            self.wait([None])
        def interrupt():raise RuntimeError('operator interrupt')
        with self.assertRaisesRegex(RuntimeError,'operator interrupt'):
            self.wait([([0,0,0],[0,0,0])],checkpoint=interrupt)


class HandoffTests(unittest.TestCase):
    def run_approach(self,failure=False,mismatch=False,coarse_failure=False,coarse_offset=0.,center_mode=False):
        import cart_book_grasp_v1_3_motion as motion
        xyz=np.array([400.,0.,0.]);angles=np.array([0.,-90.,0.]);commands=[]
        robot=SimpleNamespace(operator_checkpoint=lambda:None,operator_sleep=lambda s:None,
            get_current_state=lambda:dict(joint_state=np.zeros(14),right_arm=dict(position=xyz/1000)))
        step5=SimpleNamespace(current_tool_orientation_as_euler=lambda *a:angles)
        camera=SimpleNamespace(config={'max_joint_drift_deg':.5},capture=lambda *a:dict(
            rgb=np.zeros((720,1280,3),np.uint8),captured_at=time.monotonic()))
        points=np.array([[700,60],[730,60],[730,90],[700,90]],float)
        tracker=SimpleNamespace(target=points,observe=lambda rgb,**kw:dict(points=points,inliers=15,
            area_ratio=.03,reprojection_rms_px=.5))
        polygon=np.float32([[680,20],[760,20],[760,500],[680,500]])
        if mismatch:polygon+=200
        mask=np.zeros((720,1280),np.uint8);cv2.fillConvexPoly(mask,np.int32(polygon),1)
        selected=SimpleNamespace(observe=lambda rgb:dict(polygon=polygon,mask=mask))
        def execute(robot,task,*a,**k):
            commands.append(task)
            if coarse_failure:return 'book',False
            xyz[:]=task['target_position']
            if len(commands)==1:xyz[0]+=coarse_offset
            return 'book',True
        def servo(observe,move,c,**kwargs):
            observe()
            if failure:raise ServoError('target lost')
            self.assertTrue(move(xyz+[4,1,-1]))
            return dict(converged=True)
        params=dict(fingertip_offset_mm=50.,grasp_insertion_mm=100.,forward_step_mm=125.)
        context={}
        head_frame=dict(rgb=np.zeros((120,160,3),np.uint8),
                        selected=dict(mask=np.ones((120,160),bool),uv=[80,60]))
        c=config();c['target_selection']='wrist_center' if center_mode else 'head_identity'
        if center_mode:head_frame=None
        with tempfile.TemporaryDirectory() as directory, \
             patch('cart_book_grasp_v3_superpoint.get_backend') as backend, \
             patch.object(runtime.FinalImageTracker,'from_config',return_value=tracker), \
             patch.object(runtime,'acquire_selected_spine',return_value=selected) as acquire, \
             patch.object(runtime,'acquire_wrist_center',return_value=selected) as center, \
             patch.object(runtime,'reanchor_wrist_book',return_value=selected), \
             patch.object(runtime,'run_xyz',side_effect=servo) as run, \
             patch.object(motion,'_load_cart_plus_parameters',return_value=params):
            try:
                result=runtime.execute_visual_approach(robot,dict(object='book',arm='right',book_grasp_mode='cart+',
                    action='move_down_to_grasp',book_surface_x_mm=600.),0,{},step5,execute,head_frame,c,
                    camera=camera,retreat_context=context,output_root=directory,wrist_detector=Mock())
            except ServoError:
                if not (failure or mismatch):raise
                result=None
            self.assertEqual(run.call_count,0 if mismatch or coarse_failure else 1)
            if not coarse_failure:
                if center_mode:
                    acquire.assert_not_called();backend.assert_not_called();center.assert_called_once()
                else:self.assertEqual(acquire.call_args.kwargs['backend'],'superpoint')
                saved=list(Path(directory).glob('v3_xyz_*/acquisition.json'))
                self.assertEqual(len(saved),1)
                for name in (('wrist_acquisition.png',) if center_mode else ('head_selected_rgb.png','head_selected_mask.png','wrist_acquisition.png')):
                    self.assertTrue((saved[0].parent/name).is_file())
        return result,commands,context
    def test_center_selection_bypasses_head_matching_and_locks_once(self):
        result,commands,context=self.run_approach(center_mode=True)
        self.assertEqual(result,('book',True));self.assertTrue(context['book']['forward_complete'])
    def test_center_mode_rejects_reference_on_neighbor_without_servo(self):
        result,commands,context=self.run_approach(center_mode=True,mismatch=True)
        self.assertIsNone(result);self.assertEqual(len(commands),1)
        self.assertFalse(context['book']['forward_complete'])
    def test_first_segment_then_visual_only_preserves_retreat_origin(self):
        result,commands,context=self.run_approach()
        self.assertEqual(result,('book',True))
        self.assertEqual(len(commands),2)
        self.assertEqual(commands[0]['target_position'],[525.,0.,0.])
        self.assertEqual(commands[1]['target_position'],[529.,1.,-1.])
        self.assertEqual(commands[1]['action'],'book_v3_xyz_servo')
        self.assertEqual(context['book']['start_x_mm'],400.)
        self.assertTrue(context['book']['forward_complete'])
        self.assertEqual(context['book']['visual_final_xyz_mm'],[529.,1.,-1.])
    def test_failed_servo_and_wrong_book_cannot_mark_forward_complete(self):
        for kw in [dict(failure=True),dict(mismatch=True),dict(coarse_failure=True)]:
            result,commands,context=self.run_approach(**kw)
            self.assertEqual(len(commands),1)
            self.assertFalse(context['book']['forward_complete'])
    def test_coarse_residual_uses_measured_servo_origin_and_keeps_retreat_origin(self):
        result,commands,context=self.run_approach(coarse_offset=3.)
        self.assertEqual(result,('book',True))
        self.assertEqual(commands[1]['target_position'],[532.,1.,-1.])
        self.assertEqual(context['book']['start_x_mm'],400.)
    def test_new_hook_does_not_call_old_fixed_forward(self):
        import cart_book_grasp_v1_3_motion as motion
        from cart_book_grasp_v3_motion import installed
        with patch.object(motion,'_execute_cart_plus_forward') as original, \
             patch.object(runtime,'execute_visual_approach',return_value=('book',True)) as approach:
            with installed(None,control.CONFIG_PATH):
                result=motion._execute_cart_plus_forward(None,{},0,{},None,None,retreat_context={})
            self.assertEqual(result,('book',True));approach.assert_called_once();original.assert_not_called()


if __name__=='__main__':unittest.main()
