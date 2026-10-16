"""Offline distance reselection tests; model and robot calls are mocked."""
import unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace as NS
import numpy as np
from book_top_v34_inherited import load_config
from book_top_v41_selection import distance_report,review_distance
from cart_book_grasp_v3_2_head_fallback import build_payload


class Selection(unittest.TestCase):
    def setUp(self):
        self.c=load_config()
        self.c.update(forward_insertion_mm=100.,tcp_to_finger_tool_mm=[0,0,-50])
        self.pose=(np.array([410.,0.,0.]),np.array([0.,-90.,0.]))
        self.mask=np.ones((30,40),bool)
        self.rgb=np.zeros((30,40,3),np.uint8);self.depth=np.full((30,40),600.)
        self.k=np.array([[100.,0.,20.],[0.,100.,15.],[0.,0.,1.]])
        self.t=np.eye(4);self.t[0,3]=600.
        self.far=dict(mask=self.mask,point_camera_m=[0,-1.2,.6],book_surface_x_mm=600.,score=.22)
        self.near=dict(mask=self.mask,point_camera_m=[0,0,.6],uv=[20.,15.])
        self.detection=[(.9,self.mask,np.array([0,0,40,30]))]

    def run_review(self,selected,recover,pose_reader=None):
        return review_distance(selected,self.mask,self.rgb,self.depth,self.k,self.t,
            pose_reader=pose_reader or (lambda:self.pose),config=self.c,recover_fn=recover)

    def test_near_candidate_never_calls_gpt(self):
        near=dict(self.far,point_camera_m=[0,0,.6])
        recover=Mock(side_effect=AssertionError('No API expected'))
        got,mask=self.run_review(near,recover)
        self.assertIs(got,near);recover.assert_not_called()

    def test_far_candidate_reselects_and_recomputes_depth(self):
        recover=Mock(return_value=(self.detection,self.detection))
        with patch('cart_book_grasp_v3_2_head_fallback.center_from_mask',return_value=dict(self.near)):
            got,mask=self.run_review(self.far,recover)
        self.assertEqual(recover.call_count,1)
        context=recover.call_args.kwargs['review_context']
        self.assertEqual(context['mode'],'reselect_near_book')
        self.assertEqual(len(context['search_box_0_1000']),4)
        self.assertTrue(got['distance_reselection']['after']['accepted'])
        self.assertNotEqual(got['book_surface_x_mm'],self.far['book_surface_x_mm'])
        self.assertEqual(got['selection_source'],'gpt6_polygon_fallback')

    def test_gpt_far_again_rejected_without_loop(self):
        recover=Mock(return_value=(self.detection,self.detection))
        with patch('cart_book_grasp_v3_2_head_fallback.center_from_mask',return_value=dict(self.far)):
            with self.assertRaisesRegex(ValueError,'no depth-supported candidate'):
                self.run_review(self.far,recover)
        self.assertEqual(recover.call_count,1)

    def test_missing_depth_does_not_call_gpt_or_invent_pose(self):
        self.depth[:]=0;recover=Mock()
        with self.assertRaisesRegex(ValueError,'No valid depth'):self.run_review(self.far,recover)
        recover.assert_not_called()

    def test_current_pose_checked_again_after_model_wait(self):
        recover=Mock(return_value=(self.detection,self.detection))
        poses=Mock(side_effect=[self.pose,(np.array([410,1000,0]),self.pose[1])])
        with patch('cart_book_grasp_v3_2_head_fallback.center_from_mask',return_value=dict(self.near)):
            with self.assertRaisesRegex(ValueError,'no depth-supported candidate'):
                self.run_review(self.far,recover,poses)
        self.assertEqual(poses.call_count,2)

    def test_forward_range_and_units(self):
        near=dict(self.far,point_camera_m=[0,0,.6],book_surface_x_mm=1100.)
        a=distance_report(near,self.t,self.pose,self.c)
        t=self.t.copy();t[:3,3]/=1000
        b=distance_report(near,t,self.pose,self.c)
        self.assertEqual(a,b);self.assertIn('X forward exceeds range',a['reasons'])

    def test_reselection_prompt_allows_new_identity_only_in_new_mode(self):
        c=dict(model='gpt-6-astra')
        new=build_payload(self.rgb,'offline',c,dict(mode='reselect_near_book',search_box_0_1000=[0,0,800,1000]))
        old=build_payload(self.rgb,'offline',c,dict(reason='tilt'))
        self.assertIn('DIFFERENT nearby book',new['instructions'])
        self.assertNotIn('Reassess the SAME',new['instructions'])
        self.assertIn('Reassess the SAME',old['instructions'])

    def test_shared_selection_hook_runs_after_depth_before_capture(self):
        import cart_book_grasp_v1_3_motion as motion
        geometry=NS(fill_book_meta=Mock(),BOOK_GRASP_MODE='cart+',
            _load_sam3_detector=lambda:(None,None,NS(fromarray=lambda image:image)),
            _sam3_raw_books=lambda *a,**kw:self.detection)
        original=dict(self.far,uv=[20,15],depth_m=.6,box=[0,0,40,30])
        chosen=dict(original,point_camera_m=[0,0,.6],score=.9,
                    whole_book_depth_median_mm=600.,book_surface_x_mm=597.)
        display=Mock();robot=object()
        def review(selected,full,*args,**kwargs):
            self.assertIn('whole_book_depth_median_mm',selected)
            display.capture.assert_not_called()
            self.assertIs(kwargs['observation_robot'],robot)
            return chosen,self.mask
        hook=Mock(side_effect=review)
        motion._install_spine_center(geometry,display,selection_review=hook)
        with patch.object(motion,'_spine_center_from_mask',return_value=original):
            result=geometry.segment_cart_book_with_sam3(self.rgb,self.depth,self.k,self.t,observation_robot=robot)
        self.assertIsNotNone(result);hook.assert_called_once()
        self.assertIs(display.capture.call_args.args[4],chosen)


if __name__=='__main__':unittest.main()
