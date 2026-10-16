"""Offline head-tilt GPT gate; no network, camera or robot commands."""
import unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace
import numpy as np
from cart_book_grasp_v3_head_tilt_review import review,vertical_tilt
from cart_book_grasp_v3_2_head_fallback import build_payload,load_config


class TiltReviewTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.zeros((200,200,3),np.uint8);self.depth=np.full((200,200),800.)
        self.mask=np.zeros((200,200),bool);self.mask[20:180,80:110]=True
        self.config=dict(enabled=True,tilt_review_threshold_deg=45.)
        self.fallback=Mock(return_value=([(.9,self.mask,[])],[ (.9,self.mask,[])]))
        self.center=Mock(side_effect=lambda *args:self.selected(20.))
    def selected(self,angle):
        axis=[np.sin(np.radians(angle)),0.,np.cos(np.radians(angle))]
        return dict(mask=self.mask,box=[80,20,110,180],uv=[95.,100.],depth_m=.8,score=.8,
                    spine_tilt_deg=5.,spine_axis_camera=axis,spine_axis_residual_mm=1.,
                    spine_endpoints_camera_m=[[0,0,.7],[0,0,.9]])
    def run_review(self,selected,**kw):
        return review(selected,self.rgb,self.depth,np.eye(3),np.eye(4),config=self.config,
                      fallback=self.fallback,center=self.center,**kw)
    def test_threshold_uses_3d_axis_even_without_status_label(self):
        for a in (0.,44.999,45.):
            original=self.selected(a);selected,books=self.run_review(original)
            self.assertIs(selected,original);self.assertIsNone(books)
        self.fallback.assert_not_called()
        selected,books=self.run_review(self.selected(45.001))
        self.fallback.assert_called_once();self.assertIsNotNone(books)
        self.assertAlmostEqual(selected['head_tilt_review']['after_deg'],20.)
        self.assertNotIn('j7_skip_reason',selected)
    def test_base_transform_not_image_angle_controls_trigger(self):
        selected=self.selected(0.);selected['spine_tilt_deg']=80.
        self.run_review(selected);self.fallback.assert_not_called()
        transform=np.eye(4);transform[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]]
        self.assertAlmostEqual(vertical_tilt(selected,transform),90.)
    def test_recompute_uses_measured_depth_and_same_book_prompt(self):
        selected,books=self.run_review(self.selected(81.3))
        self.assertIs(self.center.call_args.args[1],self.depth)
        context=self.fallback.call_args.kwargs['review_context']
        self.assertAlmostEqual(context['base_vertical_tilt_deg'],81.3)
        payload=build_payload(self.rgb,'test',load_config(),context)
        self.assertIn('SAME physical book',payload['instructions'])
        self.assertIn('Do not infer robot-frame 3D angles',payload['instructions'])
        self.assertEqual(selected['selection_source'],'gpt6_polygon_fallback')
    def test_gpt_region_still_large_angle_skips_only_optional_rotation(self):
        self.center.side_effect=lambda *args:self.selected(80.)
        selected,_=self.run_review(self.selected(81.3))
        self.assertIn('j7_skip_reason',selected)
        from cart_book_grasp_v1_3_perception import _wrist_config_for_geometry
        from book_spine_wrist import align_j7
        geometry=dict(selected,spine_3d_status='valid')
        config=_wrist_config_for_geometry({'enabled':True},geometry)
        robot=Mock();record={}
        self.assertTrue(align_j7(robot,geometry,config,-170,170,record))
        robot.get_current_state.assert_not_called()
        self.assertTrue(record['complete']);self.assertEqual(record['delta_deg'],0.)
    def test_recomputed_invalid_axis_keeps_center_and_skips_j7(self):
        def no_axis(*args):
            result=self.selected(0.);result.pop('spine_axis_camera');return result
        self.center.side_effect=no_axis
        selected,_=self.run_review(self.selected(80.))
        self.assertIsNone(selected['head_tilt_review']['after_deg'])
        self.assertIn('j7_skip_reason',selected)
    def test_reviewed_unusable_projection_skips_without_sending_motion(self):
        from book_spine_wrist import align_j7
        from cart_book_grasp_v1_3_perception import _wrist_config_for_geometry
        geometry=dict(spine_3d_status='valid',spine_axis_base=[0,0,1],head_tilt_review={'after_deg':0.})
        config=_wrist_config_for_geometry(dict(enabled=True,max_delta_deg=60.,limit_margin_deg=5.),geometry)
        robot=Mock();robot.get_current_state.return_value=dict(joint_state=np.zeros((2,7)),right_arm=dict(orientation=[1,0,0,0]))
        record={};self.assertTrue(align_j7(robot,geometry,config,-170,170,record))
        self.assertTrue(record['complete']);self.assertEqual(record['delta_deg'],0.)
        self.assertEqual(len(robot.mock_calls),1) # state read only
        # Non-finite input is not an optional-geometry rejection.
        geometry['spine_axis_base']=[float('nan'),0,1]
        with self.assertRaisesRegex(ValueError,'Non-finite'):align_j7(robot,geometry,config,-170,170,{})
    def test_already_gpt_region_does_not_call_api_twice(self):
        selected=dict(self.selected(80.),selection_source='gpt6_polygon_fallback')
        reviewed,books=self.run_review(selected)
        self.fallback.assert_not_called();self.assertIsNone(books)
        self.assertIn('j7_skip_reason',reviewed)
    def test_no_3d_axis_uses_existing_unavailable_policy(self):
        selected=self.selected(80.);selected.pop('spine_axis_camera')
        self.run_review(selected);self.fallback.assert_not_called()
    def test_failed_gpt_does_not_reuse_suspect_sam(self):
        self.fallback.return_value=([],[])
        with self.assertRaisesRegex(ValueError,'did not identify'):self.run_review(self.selected(80.))
    def test_invalid_depth_cannot_be_replaced_by_gpt_angle(self):
        self.center.side_effect=ValueError('No valid depth')
        with self.assertRaisesRegex(ValueError,'No valid depth'):self.run_review(self.selected(80.))
    def test_gpt_region_not_rejected_by_old_mask_overlap(self):
        mask=np.zeros_like(self.mask);mask[20:180,140:170]=True
        self.fallback.return_value=([(.9,mask,[])],[])
        self.run_review(self.selected(80.))
        np.testing.assert_array_equal(self.center.call_args.args[0],mask)
    def test_disabled_review_stops_suspicious_selection(self):
        self.config['enabled']=False
        with self.assertRaisesRegex(ValueError,'disabled'):self.run_review(self.selected(80.))
        self.fallback.assert_not_called()
    def test_operator_cancel_prevents_request(self):
        def cancel():raise KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):self.run_review(self.selected(80.),checkpoint=cancel)
        self.fallback.assert_not_called()
    def test_selection_pipeline_uses_reviewed_book_for_depth_and_preview(self):
        import cart_book_grasp_v1_3_motion as motion
        book=self.mask.copy();book[20:180,110:130]=True
        self.fallback.return_value=([(.9,self.mask,[])],[ (.9,book,[])])
        geometry=SimpleNamespace(fill_book_meta=Mock(),BOOK_GRASP_MODE='cart+',
            _load_sam3_detector=lambda:(None,None,SimpleNamespace(fromarray=lambda a:a)),
            _sam3_raw_books=Mock(return_value=[(.8,self.mask,[])]))
        def hook(*a,**kw):return review(*a,config=self.config,fallback=self.fallback,center=self.center,**kw)
        visual=Mock()
        motion._install_spine_center(geometry,visualization=visual,spine_review=hook)
        with patch.object(motion,'_spine_center_from_mask',return_value=self.selected(80.)), \
             patch.object(motion,'whole_book_depth',return_value=dict(whole_book_depth_median_mm=800,book_surface_x_mm=600)) as depth_gate:
            result=geometry.segment_cart_book_with_sam3(self.rgb,self.depth,np.eye(3),np.eye(4))
        self.assertIsNotNone(result)
        np.testing.assert_array_equal(depth_gate.call_args.args[0],book)
        geometry._sam3_raw_books.assert_called_once() # no old SAM whole-book mask
        self.assertIn('head_tilt_review',visual.capture.call_args.args[4])


if __name__=='__main__':unittest.main()
