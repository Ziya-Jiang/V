"""Offline depth-only gate regressions; no models, network or hardware."""
import ast
from pathlib import Path
import types
import unittest
from unittest.mock import Mock
import numpy as np
import torch
import cart_book_grasp_v2_candidates as gate
from cart_book_grasp_v2_target import restore_crop


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.zeros((200,300,3),np.uint8)
        self.depth=np.full((200,300),800.)
        self.k=np.array([[300.,0,150],[0,300.,100],[0,0,1]])
        self.tf=np.eye(4);self.tf[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        self.config=gate.load_config()
        self.boxes=np.array([[70,40,95,140],[135,40,160,140],[200,40,225,140]])
        self.scores=np.array([.8,.9,.85])
        self.detector=Mock()
        self.detector.detect_boxes_region=None
        def detect(prompt,**kwargs):
            self.assertEqual(prompt,'book.')  # querying cart is a regression
            return self.boxes,self.scores,['book']*len(self.boxes)
        self.detector.detect_boxes_prepared.side_effect=detect

    def run_gate(self):
        return gate.detect_candidates(self.detector,self.rgb,self.depth,self.k,self.tf,self.config)

    def test_depth_and_robot_left_order_without_cart_query(self):
        candidates,diag=self.run_gate()
        self.assertEqual([c['id'] for c in candidates],['book_1','book_2','book_3'])
        self.assertGreater(candidates[0]['base_y_mm'],candidates[2]['base_y_mm'])
        self.assertTrue(all(c['geometry'] is None for c in candidates))
        self.assertEqual(diag['filter_method'],'camera_depth_only')
        self.assertNotIn('cart_detections',diag)
        self.detector.detect_boxes_prepared.assert_called_once()

    def test_same_depth_outside_old_cart_region_is_kept(self):
        self.boxes=np.vstack([self.boxes,[260,30,295,100]]);self.scores=np.r_[self.scores,.95]
        candidates,_=self.run_gate();self.assertEqual(len(candidates),4)

    def test_far_and_too_close_candidates_are_rejected(self):
        self.depth[40:140,135:160]=1800
        self.depth[40:140,70:95]=200
        candidates,diag=self.run_gate()
        self.assertEqual(len(candidates),1)
        self.assertEqual(sum(b['status']=='outside_depth_range' for b in diag['boxes']),2)

    def test_missing_depth_skips_only_that_candidate(self):
        self.depth[40:140,135:160]=0
        candidates,diag=self.run_gate()
        self.assertEqual(len(candidates),2)
        self.assertEqual(sum(b['status']=='invalid_depth' for b in diag['boxes']),1)

    def test_empty_depth_reports_depth_failure_not_cart_ambiguity(self):
        self.depth[:]=0
        with self.assertRaisesRegex(gate.CandidateError,'深度.*没有有效书本候选'):self.run_gate()

    def test_depth_limits_are_configurable_and_inclusive(self):
        self.config['depth_range_mm']=[800,900]
        candidates,_=self.run_gate();self.assertEqual(len(candidates),3)
        self.config['depth_range_mm']=[801,900]
        with self.assertRaises(gate.CandidateError):self.run_gate()

    def test_background_depth_and_spread_do_not_impose_an_extra_cart_gate(self):
        self.depth[:]=4000
        for box in self.boxes:
            x0,y0,x1,y1=box
            self.depth[y0:y1,x0:x1]=np.linspace(600,1100,y1-y0)[:,None]
        candidates,_=self.run_gate();self.assertEqual(len(candidates),3)

    def test_meters_and_millimeters_have_same_result(self):
        first,_=self.run_gate();self.depth*=.001
        second,_=self.run_gate()
        np.testing.assert_allclose([c['base_xyz_mm'] for c in first],[c['base_xyz_mm'] for c in second])

    def test_duplicate_boxes_are_not_extra_books(self):
        self.boxes=np.vstack([self.boxes,self.boxes[1]]);self.scores=np.r_[self.scores,.7]
        candidates,_=self.run_gate();self.assertEqual(len(candidates),3)

    def test_depth_crop_recovers_missing_book_and_deduplicates(self):
        missing=self.boxes[1].copy()
        self.boxes=self.boxes[[0,2]];self.scores=self.scores[[0,2]]
        self.detector.detect_boxes_region=Mock(return_value=(np.array([missing,self.boxes[0]]),np.array([.9,.8]),['book']*2))
        candidates,diag=self.run_gate()
        self.assertEqual(len(candidates),3)
        self.assertEqual(diag['depth_crop_book_count'],2)
        self.assertNotIn('cart_box',diag)

    def test_crop_restores_exact_original_pixels(self):
        cropped=np.zeros((60,30),bool);cropped[3:50,7:14]=True
        restored=restore_crop([(.9,cropped,[0,0,30,60])],[40,70,70,130],(200,300))
        expected=np.zeros((200,300),bool);expected[73:120,47:54]=True
        np.testing.assert_equal(restored[0][1],expected)
        self.assertEqual(restored[0][2],[47,73,54,120])

    def test_dino_box_only_methods_do_not_touch_sam2(self):
        # Compile just these two methods to avoid importing model/CUDA packages.
        tree=ast.parse(Path('RobotRAM/visions/grounding/dino_grounding.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='DinoGrounding')
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('prepare_boxes_image','detect_boxes_prepared','detect_boxes_region')]
        load=Mock(return_value=(self.rgb,'tensor'))
        predict=Mock(return_value=(torch.tensor([[.5,.5,.2,.4]]),torch.tensor([.9]),['book']))
        def convert(boxes,**kwargs):
            return torch.cat([boxes[:,:2]-boxes[:,2:]/2,boxes[:,:2]+boxes[:,2:]/2],dim=1)
        ns=dict(load_data=load,predict=predict,torch=torch,box_convert=convert,np=np)
        exec(compile(ast.Module(body=methods,type_ignores=[]),'box_methods','exec'),ns)
        detector=types.SimpleNamespace(_prepared_dino_image=None,sam2_predictor=Mock(),
            grounding_model=object(),device='cpu',box_threshold=.3,text_threshold=.2)
        ns['prepare_boxes_image'](detector,self.rgb)
        boxes,scores,labels=ns['detect_boxes_prepared'](detector,'book.',box_threshold=.25)
        np.testing.assert_allclose(boxes,[[120,60,180,140]],atol=1e-5)
        self.assertEqual(predict.call_args.kwargs['device'],'cpu')
        detector.sam2_predictor.set_image.assert_not_called()
        detector.sam2_predictor.predict.assert_not_called()
        old_image=detector._prepared_dino_image
        boxes,_,_=ns['detect_boxes_region'](detector,self.rgb,[30,20,130,120],'book.',box_threshold=.25,text_threshold=.2)
        np.testing.assert_allclose(boxes,[[70,50,90,90]],atol=1e-5)
        self.assertIs(detector._prepared_dino_image,old_image)



if __name__=='__main__':unittest.main()
