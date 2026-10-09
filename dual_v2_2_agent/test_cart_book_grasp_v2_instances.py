"""Synthetic instance masks only: no model, network or robot required."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from cart_book_grasp_v2_instances import clean_masks,detect_instances,load_config
from cart_book_grasp_v2_candidates import load_config as depth_config,CandidateError
from cart_book_grasp_v2_pause import PauseRequested


class InstancesTests(unittest.TestCase):
    def setUp(self):
        self.shape=(160,220);self.rgb=np.zeros((*self.shape,3),np.uint8)
        self.depth=np.full(self.shape,800.)
        self.k=np.array([[200.,0,110],[0,200.,80],[0,0,1.]])
        self.tf=np.eye(4);self.tf[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        self.config=dict(load_config(),crop_recall=False)
        self.limits=depth_config()

    def mask(self,x0=80,y0=50,x1=100,y1=120):
        m=np.zeros(self.shape,bool);m[y0:y1,x0:x1]=True;return m

    def run_detection(self,raw,config=None,checkpoint=lambda:None):
        detector=SimpleNamespace(_load_sam3_detector=lambda:(None,None,SimpleNamespace(fromarray=lambda x:x)),
                                 _sam3_raw_books=raw)
        return detect_instances(detector,self.rgb,self.depth,self.k,self.tf,
            self.config if config is None else config,self.limits,checkpoint)

    def test_largest_contour_fills_holes_and_removes_noise_without_axis_size_gate(self):
        mask=self.mask();mask[60:65,84:88]=False;mask[2:5,2:5]=True
        cleaned=clean_masks([(.8,mask,[])],self.shape,self.config)
        self.assertEqual(len(cleaned),1)
        np.testing.assert_equal(cleaned[0][1],self.mask())
        # Wide/horizontal objects remain for GPT; no fixed shelf-band/height gate.
        horizontal=self.mask(10,3,90,10)
        self.assertEqual(len(clean_masks([(.8,horizontal,[])],self.shape,self.config)),1)

    def test_mask_nms_deduplicates_but_preserves_neighbor_and_group(self):
        a=self.mask(70,50,90,120);b=self.mask(95,50,115,120)
        group=self.mask(70,50,115,120)
        items=clean_masks([(.99,group,[]),(.95,a,[]),(.9,a.copy(),[]),(.85,b,[])],self.shape,self.config)
        self.assertEqual(len(items),3)
        self.assertTrue(any(np.array_equal(m,a) for _,m,_ in items))
        self.assertTrue(any(np.array_equal(m,b) for _,m,_ in items))

    def test_actual_mask_depth_is_used_not_bbox_background(self):
        mask=self.mask();self.depth[:]=4000.;self.depth[mask]=800.
        raw=Mock(return_value=[(.9,mask,[])])
        candidates,diag=self.run_detection(raw)
        self.assertEqual(len(candidates),1)
        self.assertEqual(candidates[0]['depth_median_mm'],800.)
        np.testing.assert_equal(candidates[0]['book_mask'],mask)
        self.assertIsNone(candidates[0]['geometry'])
        self.assertEqual(diag['filter_method'],'sam3_mask_camera_depth')
        self.assertEqual(raw.call_args.kwargs,{'prompt':'book'})

    def test_missing_far_depth_skips_only_affected_candidate_and_orders_base_y(self):
        a=self.mask(30,50,50,120);b=self.mask(80,50,100,120);c=self.mask(150,50,170,120)
        self.depth[c]=3000.
        candidates,diag=self.run_detection(Mock(return_value=[(.9,c,[]),(.8,b,[]),(.7,a,[])]))
        self.assertEqual(len(candidates),2)
        self.assertGreater(candidates[0]['base_y_mm'],candidates[1]['base_y_mm'])
        self.assertEqual(candidates[0]['id'],'book_1')
        self.assertEqual(diag['boxes'][0]['status'],'outside_depth_range')
        self.depth[a]=0
        candidates,_=self.run_detection(Mock(return_value=[(.8,b,[]),(.7,a,[])]))
        self.assertEqual(len(candidates),1)

    def test_crop_recall_restores_coordinates_and_adds_missed_book(self):
        a=self.mask();b=self.mask(108,55,119,112)
        def raw(p,n,image,*,prompt):
            if image.shape==self.rgb.shape:return [(.9,a,[])]
            self.assertEqual(image.shape,(110,60,3))
            return [(.95,a[30:140,60:120],[]),(.8,b[30:140,60:120],[])]
        # b touches right crop edge -> deliberately exclude that partial mask.
        candidates,diag=self.run_detection(raw,dict(self.config,crop_recall=True,crop_padding_px=20))
        self.assertEqual(len(candidates),1)
        self.assertEqual(diag['depth_recall_roi'],[60,30,120,140])
        b=self.mask(106,55,116,112)
        candidates,_=self.run_detection(raw,dict(self.config,crop_recall=True,crop_padding_px=20))
        self.assertEqual(len(candidates),2)
        self.assertTrue(any(np.array_equal(c['book_mask'],b) for c in candidates))

    def test_empty_full_frame_can_recall_using_depth_region(self):
        a=self.mask();self.depth[:]=3000.;self.depth[a]=800.
        def raw(p,n,image,*,prompt):
            if image.shape==self.rgb.shape:return []
            return [(.9,a[30:140,60:120],[])]
        candidates,_=self.run_detection(raw,dict(self.config,crop_recall=True,crop_padding_px=20))
        self.assertEqual(len(candidates),1)

    def test_empty_or_cancelled_frame_cannot_proceed(self):
        raw=Mock(return_value=[])
        with self.assertRaises(CandidateError):self.run_detection(raw)
        raw.reset_mock()
        with self.assertRaises(PauseRequested):
            self.run_detection(raw,checkpoint=Mock(side_effect=PauseRequested))
        raw.assert_not_called()


if __name__=='__main__':unittest.main()
