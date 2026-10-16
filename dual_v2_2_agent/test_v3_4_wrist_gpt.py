"""Offline GPT routing, perspective and first-frame report tests. No API calls."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_2_selection import select_wrist_book
from cart_book_grasp_v3_4_edges import visible_spine_geometry
from cart_book_grasp_v3_4_wrist_gpt import recover,wrist_fallback,proposal_mask
from cart_book_grasp_wrist_preview import write_preview

class WristGPTTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.latest=patch('cart_book_grasp_wrist_preview.ROOT',self.root/'latest');self.latest.start();self.addCleanup(self.latest.stop)
        self.rgb=np.full((480,640,3),50,np.uint8)
        self.quad=[[440,0],[560,0],[560,1000],[440,1000]]
        self.mask=proposal_mask(self.quad,self.rgb.shape[:2])
        self.config=dict(enabled=True,model='gpt-6-astra',timeout_seconds=1,min_confidence=.7)
        self.predictor=NS(set_image=Mock(),predict=Mock(return_value=(self.mask[None],[.95],None)))
        self.detector=Mock(return_value=[]);self.detector.box_detector=NS(sam2_predictor=self.predictor)
        self.client=NS(select=self.response)
    def response(self,payload,*args):
        self.assertEqual(payload['model'],'gpt-6-astra')
        scene=payload['text']['format']['schema']['properties']['scene_id']['enum'][0]
        return dict(scene_id=scene,status='selected',confidence=.9,reason='visible central spine',spine_quad=self.quad)
    def test_gpt_direct_region_allows_partial_top_bottom(self):
        d={};r=recover(self.rgb,d,detector=self.detector,client=self.client,config=self.config,directory=self.root)
        self.assertEqual(d['status'],'selected');self.assertAlmostEqual(r['uv'][0],319.5,delta=2)
        self.predictor.set_image.assert_not_called();self.predictor.predict.assert_not_called()
        self.assertTrue((self.root/'wrist_gpt_proposal.npz').exists())
    def test_sam2_cover_result_not_used(self):
        self.predictor.predict.return_value=(np.ones_like(self.mask)[None],[.99],None)
        r=recover(self.rgb,{},detector=self.detector,client=self.client,config=self.config)
        np.testing.assert_array_equal(r['mask'],self.mask)
        self.predictor.predict.assert_not_called()
        r=recover(self.rgb,{},detector=None,client=self.client,config=self.config)
        np.testing.assert_array_equal(r['mask'],self.mask)
    def test_wrong_scene_low_confidence_and_uncertain_rejected(self):
        for change in [dict(scene_id='stale'),dict(confidence=.2),dict(status='uncertain')]:
            client=NS(select=lambda payload,*args:dict(self.response(payload),**change))
            with self.assertRaises(ServoError):recover(self.rgb,{},detector=self.detector,client=client,config=self.config)
        self.predictor.predict.assert_not_called()
    def test_selector_calls_gpt_once_and_never_boundaries(self):
        backend=NS(mask_geometry_validator=visible_spine_geometry)
        backend.recover=lambda rgb,d,**kw:recover(rgb,d,client=self.client,config=self.config,**kw)
        with patch('cart_book_grasp_v3_4_edges.select_boundary_pair',side_effect=AssertionError('edge fallback forbidden')):
            chosen,summary=select_wrist_book(self.rgb,self.detector,directory=self.root,boundary_selector=backend)
        self.assertEqual(summary['criterion'],'gpt6_polygon_wrist')
        text=(self.root/'first_detection.html').read_text()
        self.assertIn('GPT 候选区域',text);self.assertIn('成功',text)
        self.assertTrue((self.root/'latest/index.html').exists())
    def test_rejected_sam3_mask_also_routes_to_gpt(self):
        self.detector.return_value=[(.95,np.ones((480,640),bool),[0,0,640,480])]
        backend=NS(mask_geometry_validator=visible_spine_geometry)
        backend.recover=Mock(side_effect=lambda rgb,d,**kw:recover(rgb,d,client=self.client,config=self.config,**kw))
        _,summary=select_wrist_book(self.rgb,self.detector,directory=self.root,boundary_selector=backend)
        backend.recover.assert_called_once()
        self.assertEqual(summary['criterion'],'gpt6_polygon_wrist')
        self.assertNotIn('boundaries',summary['initial_detection'])

    def test_failed_gpt_still_produces_first_frame_report(self):
        backend=NS(mask_geometry_validator=visible_spine_geometry,recover=Mock(side_effect=ServoError('GPT unavailable')))
        with self.assertRaisesRegex(ServoError,'GPT unavailable'):
            select_wrist_book(self.rgb,self.detector,directory=self.root,boundary_selector=backend)
        backend.recover.assert_called_once()
        text=(self.root/'first_detection.html').read_text();self.assertIn('失败',text);self.assertIn('GPT unavailable',text)
    def test_sam3_success_does_not_call_gpt(self):
        raw=[(.9,self.mask,[281,0,359,480])];detector=Mock(return_value=raw)
        backend=NS(mask_geometry_validator=visible_spine_geometry,recover=Mock(side_effect=AssertionError('GPT not needed')))
        select_wrist_book(self.rgb,detector,directory=self.root,boundary_selector=backend)
        backend.recover.assert_not_called()
    def test_geometry_rejects_internal_step_not_just_total_width_ratio(self):
        mask=np.zeros((480,640),bool)
        for y in range(480):mask[y,240:400 if y<240 else 300]=True
        with self.assertRaises(ServoError):visible_spine_geometry(mask)
    def test_recorded_taper_recovered_with_observed_support(self):
        p=Path(__file__).parent/'RobotRAM/output/v3_column_aad30a72bd514ab8a308970ce473f997'
        if not p.exists():self.skipTest('Recorded frame unavailable')
        with np.load(p/'wrist_detection_book_spine.npz') as a:mask=a['mask_0'].astype(bool)
        g=visible_spine_geometry(mask)
        self.assertEqual(g['support_rows'],[108,608]);self.assertAlmostEqual(g['center_u'],690.5,delta=1)
        self.assertGreater(g['retained_height_fraction'],.6)
    def test_preview_failure_never_replaces_original_exception(self):
        with patch('cart_book_grasp_wrist_preview.write_preview',side_effect=OSError('disk full')):
            with self.assertRaisesRegex(RuntimeError,'detector failed'):
                select_wrist_book(self.rgb,Mock(side_effect=RuntimeError('detector failed')),directory=self.root)
    def test_latest_viewer_only_replaces_image_on_new_result(self):
        p=write_preview(self.rgb,dict(status='failed',reason='<script>bad()</script>'),self.root)
        text=p.read_text();self.assertIn('&lt;script&gt;',text)
        viewer=(self.root/'latest/index.html').read_text()
        self.assertIn('current===data.id',viewer);self.assertNotIn('location.reload',viewer)
if __name__=='__main__':unittest.main()
