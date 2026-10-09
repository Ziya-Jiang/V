"""Offline whole-book median depth, base-X approach, detector and runtime wiring."""
import sys
import types
import unittest
from unittest.mock import Mock, patch
import numpy as np
import cart_book_grasp_v1_3 as cart
import cart_book_grasp_v1_3_motion as motion
import cart_book_grasp_v1_3_perception as perception
from cart_book_grasp_v1_3_depth import whole_book_depth, forward_parameters
from cart_book_grasp_v1_3_visualization import preview_targets


class DepthApproachTests(unittest.TestCase):
    def setUp(self):
        self.mask = np.zeros((40, 40), bool);self.mask[5:35, 5:35] = True
        self.depth = np.full((40, 40), 800.)
        self.k = np.array([[100.,0.,20.],[0.,100.,20.],[0.,0.,1.]])
        self.transform = np.eye(4)
        self.transform[:3,:3] = [[0,0,1],[1,0,0],[0,1,0]]
        self.params = dict(fingertip_offset_mm=50.,grasp_insertion_mm=50.,forward_step_mm=125.,
                          lift_mm=200.,lower_mm=200.,lower_step_mm=50.,hold_seconds=3.,initial_j6_deg=-20.3164)

    def test_full_mask_median_ignores_missing_depth_background_and_minority_outliers(self):
        self.depth[~self.mask] = 5000.
        self.depth[5:8,5:35] = 1700.
        self.depth[10,10] = 0.;self.depth[11,11] = np.nan;self.depth[12,12] = np.inf
        self.transform[:3,3] = [-100,10,20]  # millimetre calibration translation
        actual = whole_book_depth(self.mask,self.depth,self.k,self.transform)
        self.assertAlmostEqual(actual['whole_book_depth_median_mm'],800.)
        self.assertAlmostEqual(actual['book_surface_x_mm'],700.)
        self.assertEqual(actual['whole_book_depth_samples'],897)
        metric_transform=self.transform.copy();metric_transform[:3,3] *= .001
        metric = whole_book_depth(self.mask,self.depth*.001,self.k,metric_transform)
        self.assertEqual(actual,metric)

    def test_median_uses_base_x_not_camera_z(self):
        transform=np.eye(4);transform[0,3]=.3
        actual=whole_book_depth(self.mask,self.depth,self.k,transform)
        self.assertAlmostEqual(actual['book_surface_x_mm'],296.)
        self.assertNotEqual(actual['book_surface_x_mm'],800.)

    def test_invalid_or_insufficient_depth_never_becomes_a_fixed_distance(self):
        with self.assertRaises(ValueError):whole_book_depth(self.mask,np.zeros_like(self.depth),self.k,self.transform)
        self.depth[:25]=0
        with self.assertRaises(ValueError):whole_book_depth(self.mask,self.depth,self.k,self.transform)
        with self.assertRaises(ValueError):forward_parameters([300,0,100],[-90,0,90],None,self.params)
        with self.assertRaises(ValueError):forward_parameters([800,0,100],[-90,0,90],600,self.params)
        with self.assertRaises(ValueError):forward_parameters([300,0,100],[0,0,0],600,self.params)

    def test_fingertip_projection_changes_only_x_and_insertion_is_base_x(self):
        flat=forward_parameters([300,-200,100],[-90,0,90],600,self.params)
        self.assertAlmostEqual(flat['forward_mm'],300.)
        tilted=forward_parameters([300,-200,100],[-90,0,60],600,self.params)
        self.assertAlmostEqual(tilted['grasp_tcp_x_mm'],650-50*np.sqrt(3)/2)
        self.assertAlmostEqual(tilted['grasp_tcp_x_mm']+tilted['fingertip_x_offset_mm'],650)
        plan=preview_targets([300,-250,100],[-90,0,60],[700,-200,-180],self.params,-30,600)
        np.testing.assert_allclose(plan['grasp_mm'],[tilted['grasp_tcp_x_mm'],-200,70])
        self.assertEqual(len(plan['labels']),len(plan['points_mm']))
        forward=[p for p,label in zip(plan['points_mm'],plan['labels']) if label.startswith('forward')]
        np.testing.assert_allclose(np.array(forward)[:,1:],np.tile([-200,70],(len(forward),1)))

    def test_actual_motion_recomputes_distance_once_and_withdraws_to_saved_x(self):
        position=np.array([310.,-200.,70.]);commands=[];context={}
        robot=types.SimpleNamespace(get_current_state=lambda:{'right_arm':{'position':position/1000}})
        step5=types.SimpleNamespace(current_tool_orientation_as_euler=lambda *_:[-90.,0.,90.])
        params=dict(self.params)
        def execute(robot,task,*args,**kwargs):
            commands.append(dict(task));position[:]=task['target_position']
            params['grasp_insertion_mm']=100.  # Change only affects the next action.
            return 'b',True
        task=dict(object='b',arm='right',book_grasp_mode='cart+',action='move_down_to_grasp',book_surface_x_mm=610.)
        with patch.object(motion,'_load_cart_plus_parameters',side_effect=lambda:dict(params)):
            result=motion._execute_cart_plus_forward(robot,task,0,{},step5,execute,retreat_context=context)
            self.assertTrue(result[1]);self.assertEqual(len(commands),3)
            np.testing.assert_allclose([t['target_position'] for t in commands],[[410,-200,70],[510,-200,70],[610,-200,70]])
            context['b']['released']=True
            result=motion._execute_cart_plus_withdraw(robot,dict(task,action='withdraw_after_release'),1,{},step5,execute,context)
            self.assertTrue(result[1]);np.testing.assert_allclose(position,[310,-200,70])

    def test_bad_depth_and_failed_waypoint_stop_before_remaining_forward(self):
        robot=types.SimpleNamespace(get_current_state=lambda:{'right_arm':{'position':[.3,-.2,.1]}})
        step5=types.SimpleNamespace(current_tool_orientation_as_euler=lambda *_:[-90.,0.,90.])
        execute=Mock(return_value=('b',False));task=dict(object='b',action='move_down_to_grasp')
        with patch.object(motion,'_load_cart_plus_parameters',return_value=self.params):
            self.assertFalse(motion._execute_cart_plus_forward(robot,task,0,{},step5,execute)[1])
            execute.assert_not_called()
            self.assertFalse(motion._execute_cart_plus_forward(robot,dict(task,book_surface_x_mm=600),0,{},step5,execute)[1])
            self.assertEqual(execute.call_count,1)

    def test_detection_uses_full_book_even_without_rerun_and_clears_failed_cycle(self):
        image_cls=types.SimpleNamespace(fromarray=lambda a:a)
        calls=[]
        matched=[True]
        def detect(*args,prompt):
            calls.append(prompt)
            return [(1.,self.mask,np.array([5,5,35,35]))] if prompt=='book spine' or matched[0] else []
        geometry=types.SimpleNamespace(fill_book_meta=Mock(),_load_sam3_detector=lambda:(None,None,image_cls),
            _sam3_raw_books=detect,BOOK_GRASP_MODE='cart+')
        selected=dict(mask=self.mask,box=[5,5,35,35],point_camera_m=np.array([0,0,.8]),uv=[20,20],depth_m=.8)
        motion._install_spine_center(geometry)
        with patch.object(motion,'_spine_center_from_mask',side_effect=lambda *_:dict(selected)):
            result=geometry.segment_cart_book_with_sam3(np.zeros((40,40,3)),self.depth,self.k,self.transform)
            self.assertIsNotNone(result);self.assertEqual(calls,['book spine','book'])
            matched[0]=False
            self.assertIsNone(geometry.segment_cart_book_with_sam3(np.zeros((40,40,3)),self.depth,self.k,self.transform))
            with self.assertRaisesRegex(ValueError,'没有有效书脊中心'):
                geometry.fill_book_meta({},None,self.transform)

    def test_runtime_injects_current_whole_book_measurement_into_forward_task(self):
        seen=[]
        step5=types.SimpleNamespace(RIGHT_ARM_SOFT_LOWER_DEG=[-90]*7,RIGHT_ARM_SOFT_UPPER_DEG=[90]*7)
        step5.execute_single_task=lambda robot,task,*args,**kwargs:(seen.append(task) or ('b',True))
        selected=dict(spine_tilt_deg=0,book_surface_x_mm=612.,whole_book_depth_median_mm=800.,
            whole_book_depth_samples=900,whole_book_depth_valid_fraction=1.,whole_book_base_x_p10_p90_mm=[600,620])
        def install(geometry,visualization=None,on_geometry=None):on_geometry(selected,np.eye(4),{})
        def run(**kwargs):
            perception.previous._install_spine_center(object())
            perception.previous._install_right_arm_reset()
            step5.execute_single_task(object(),dict(action='move_down_to_grasp',arm='right',book_grasp_mode='cart+',object='b'),0,{})
        with patch.dict(sys.modules,{'step5_execute':step5}), \
             patch.object(perception.previous,'_install_spine_center',side_effect=install), \
             patch.object(perception.previous,'_install_right_arm_reset'), \
             patch.object(perception.previous,'main',side_effect=run), \
             patch.object(perception,'load_wrist_config',return_value={'enabled':False}):
            perception.main(task_types=['cart+'])
        self.assertEqual(seen[0]['book_surface_x_mm'],612.)

    def test_own_hot_config_has_no_fixed_distance(self):
        # Runtime configuration belongs to the operator and may be hot-edited.
        # Test loading with an isolated fixture, never assume their live values.
        import tempfile, json
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'
            path.write_text(json.dumps({'cart_plus':self.params}))
            with patch.object(motion,'_CART_PLUS_CONFIG_PATH',path):
                values=motion._load_cart_plus_parameters()
                self.assertNotIn('forward_mm',values)
                self.assertEqual(values['grasp_insertion_mm'],50.)
                self.assertEqual(values['fingertip_offset_mm'],50.)
                path.write_text(json.dumps({'cart_plus':dict(self.params,grasp_insertion_mm=85.)}))
                self.assertEqual(motion._load_cart_plus_parameters()['grasp_insertion_mm'],85.)

# Re-run the operator-interruption contract against the new entrypoint and
# pause module, including real joint-command extraction and pseudo-terminal p.
import test_cart_book_grasp_v1_2 as previous_tests
import cart_book_grasp_v1_3_pause as pause_module

class V13PauseTests(previous_tests.PauseTests):
    def setUp(self):
        self.scope = patch.multiple(previous_tests, cart=cart, motion=motion,
            PauseRequested=pause_module.PauseRequested,
            operator_robot=pause_module.operator_robot, open_and_reset=pause_module.open_and_reset)
        self.scope.start()
        self.addCleanup(self.scope.stop)

if __name__=='__main__':unittest.main()
