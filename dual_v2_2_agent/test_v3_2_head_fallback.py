"""Offline contracts for GPT head fallback; no credentials/camera/robot."""
import json
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from cart_book_grasp_v3_2_head_fallback import recover,polygon_mask,build_payload,load_config
from cart_book_grasp_v2_gpt import GPTSelectionError

SPINE=[[400,100],[470,100],[470,900],[400,900]]
BOOK=[[380,90],[650,90],[650,910],[380,910]]

class FallbackTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.zeros((240,320,3),np.uint8)
        self.spine=polygon_mask(SPINE,self.rgb.shape[:2]);self.book=polygon_mask(BOOK,self.rgb.shape[:2])
        self.predictor=Mock();self.predictor.predict.side_effect=[(self.spine[None],[.9],None),(self.book[None],[.9],None)]
        self.detector=SimpleNamespace(sam2_predictor=self.predictor,_prepared_image_source=self.rgb.copy())
        self.client=Mock()
        def select(payload,*args):
            scene=json.loads(payload['input'][0]['content'][0]['text'])['scene_id']
            return dict(scene_id=scene,status='selected',confidence=.9,reason='visible',top_edge_visible=True,spine_quad=SPINE,book_quad=BOOK)
        self.client.select.side_effect=select

    def test_gpt_spine_and_same_book_polygons_used_directly(self):
        with tempfile.TemporaryDirectory() as d:
            spines,books=recover(self.rgb,self.detector,client=self.client,directory=d)
            np.testing.assert_array_equal(spines[0][1],self.spine)
            np.testing.assert_array_equal(books[0][1],self.book)
            self.predictor.predict.assert_not_called()
            from pathlib import Path
            record=json.loads(Path(d+'/result.json').read_text())
            self.assertEqual(record['status'],'segmented_pending_geometry_and_valid_depth')
            self.assertEqual(record['depth_consistency_check'],'skipped_gpt')
            self.assertEqual(self.client.select.call_args.args[0]['model'],'gpt-6-astra')

    def test_rejects_stale_scene_uncertain_or_low_confidence(self):
        for changed in [dict(scene_id='stale'),dict(status='uncertain'),dict(confidence=.4),dict(confidence=float('nan'))]:
            initial=self.client.select.side_effect
            client=Mock()
            def select(*args):return dict(initial(*args),**changed)
            client.select.side_effect=select
            with tempfile.TemporaryDirectory() as d,self.assertRaises(GPTSelectionError):
                recover(self.rgb,self.detector,client=client,directory=d)
        self.predictor.predict.assert_not_called()

    def test_gpt_book_pair_has_no_containment_veto(self):
        original=self.client.select.side_effect
        self.client.select.side_effect=lambda *args:dict(original(*args),book_quad=[[700,100],[800,100],[800,900],[700,900]])
        with tempfile.TemporaryDirectory() as d:
            spines,books=recover(self.rgb,None,client=self.client,directory=d)
            self.assertFalse(np.any(spines[0][1]&books[0][1]))

    def test_bad_polygon_rejected_and_predictor_not_required(self):
        for poly in ([[0,0]]*4,[[1001,0]]*4,[[float('nan'),0]]*4):
            with self.assertRaises(GPTSelectionError):polygon_mask(poly,self.rgb.shape[:2])
        with tempfile.TemporaryDirectory() as d:
            spines,books=recover(self.rgb,None,client=self.client,directory=d)
        self.assertEqual(len(spines),1);self.assertEqual(len(books),1)
        self.predictor.predict.assert_not_called()

    def test_bottom_clipped_region_allowed_but_top_visibility_required(self):
        original=self.client.select.side_effect
        partial=[[400,500],[470,500],[470,1000],[400,1000]]
        self.client.select.side_effect=lambda *a:dict(original(*a),spine_quad=partial)
        with tempfile.TemporaryDirectory() as d:
            spines,_=recover(self.rgb,None,client=self.client,directory=d)
        from cart_book_grasp_v3_2_head_fallback import center_from_mask
        from cart_book_grasp_v1_3_perception import _spine_center_from_mask
        depth=np.full(self.spine.shape,800.)
        result=center_from_mask(spines[0][1],depth,np.eye(3))
        self.assertEqual(result['spine_geometry_policy'],'direct_gpt_region')
        self.assertTrue(result['mask'][-1].any())
        with self.assertRaisesRegex(ValueError,'clipped'):
            _spine_center_from_mask(spines[0][1],depth,np.eye(3))
        for visible in (False,None):
            self.client.select.side_effect=lambda *a:dict(original(*a),spine_quad=partial,top_edge_visible=visible)
            with tempfile.TemporaryDirectory() as d,self.assertRaisesRegex(GPTSelectionError,'top edge'):
                recover(self.rgb,None,client=self.client,directory=d)
        prompt=build_payload(self.rgb,'test',load_config())
        self.assertIn('bottom of the book/spine may extend outside',prompt['instructions'])
        self.assertNotIn('complete spine',prompt['instructions'])

    def test_nonelongated_and_concave_regions_not_rejected(self):
        from cart_book_grasp_v3_2_head_fallback import center_from_mask
        for poly in ([[300,300],[600,300],[600,600],[300,600]],
                     [[300,300],[700,300],[450,450],[300,700]]):
            mask=polygon_mask(poly,self.rgb.shape[:2])
            result=center_from_mask(mask,np.full(mask.shape,800.),np.eye(3))
            np.testing.assert_array_equal(result['mask'],mask)
            self.assertTrue(np.isfinite(result['point_camera_m']).all())

    def test_saved_rejected_72_percent_pair_now_passes(self):
        # Real failed polygons; top visibility true is a fixture, not a new model call.
        original=self.client.select.side_effect
        self.client.select.side_effect=lambda *a:dict(original(*a),
            spine_quad=[[590,824],[606,816],[605,986],[591,989]],
            book_quad=[[566,704],[584,700],[606,817],[605,989]])
        rgb=np.zeros((800,1280,3),np.uint8)
        with tempfile.TemporaryDirectory() as d:
            spines,books=recover(rgb,None,client=self.client,directory=d)
        from cart_book_grasp_v3_2_head_fallback import direct_book_mask,center_from_mask
        selected=center_from_mask(spines[0][1],np.full((800,1280),800.),np.eye(3))
        full=direct_book_mask(books,selected['mask'].shape)
        self.assertAlmostEqual(np.count_nonzero(full&selected['mask'])/selected['mask'].sum(),.7226,delta=.001)

    def test_cancel_does_not_start_request(self):
        def checkpoint():raise KeyboardInterrupt()
        with tempfile.TemporaryDirectory() as d,self.assertRaises(KeyboardInterrupt):
            recover(self.rgb,self.detector,client=self.client,directory=d,checkpoint=checkpoint)
        self.client.select.assert_not_called()

    def test_sam_embedding_is_never_used(self):
        self.detector._prepared_image_source=np.ones_like(self.rgb)
        with tempfile.TemporaryDirectory() as d:
            recover(self.rgb,self.detector,client=self.client,directory=d)
        self.client.select.assert_called_once();self.predictor.predict.assert_not_called()

    def test_fallback_supplies_whole_book_to_existing_depth_checks(self):
        import cart_book_grasp_v1_3_motion as motion
        geometry=SimpleNamespace(fill_book_meta=Mock(),_load_sam3_detector=lambda:(None,None,SimpleNamespace(fromarray=lambda a:a)),
                                 _sam3_raw_books=lambda *a,**kw:[],BOOK_GRASP_MODE='cart+')
        # Deliberately disjoint book region: trust GPT pairing, not overlap.
        book=polygon_mask([[700,100],[800,100],[800,900],[700,900]],self.rgb.shape[:2])
        fallback=Mock(return_value=([(.9,self.spine,[])],[ (.9,book,[])]))
        motion._install_spine_center(geometry,spine_fallback=fallback)
        selected=dict(mask=self.spine,box=[128,24,151,216],uv=[139,120],depth_m=.8)
        with patch.object(motion,'_spine_center_from_mask',return_value=selected),patch.object(motion,'whole_book_depth',return_value=dict(whole_book_depth_median_mm=800,book_surface_x_mm=600)) as depth_gate:
            self.assertIsNotNone(geometry.segment_cart_book_with_sam3(self.rgb,np.ones((240,320)),np.eye(3),np.eye(4)))
        np.testing.assert_array_equal(depth_gate.call_args.args[0],book)
        fallback.assert_called_once()

    def test_api_error_stays_failure_without_synthetic_mask(self):
        self.client.select.side_effect=GPTSelectionError('missing key')
        with tempfile.TemporaryDirectory() as d,self.assertRaisesRegex(GPTSelectionError,'missing key'):
            recover(self.rgb,self.detector,client=self.client,directory=d)
        self.predictor.predict.assert_not_called()

    def test_original_geometry_depth_checks_remain_after_fallback(self):
        import cart_book_grasp_v1_3_motion as motion
        geometry=SimpleNamespace(fill_book_meta=Mock(),_load_sam3_detector=lambda:(None,None,SimpleNamespace(fromarray=lambda a:a)),
                                 _sam3_raw_books=lambda *a,**kw:[],BOOK_GRASP_MODE='cart+')
        fallback=Mock(return_value=([(.9,self.spine,[])],[ (.9,self.book,[])]))
        motion._install_spine_center(geometry,spine_fallback=fallback)
        with patch.object(motion,'_spine_center_from_mask',side_effect=ValueError('invalid depth')) as gate:
            self.assertIsNone(geometry.segment_cart_book_with_sam3(self.rgb,np.ones((240,320)),np.eye(3),np.eye(4)))
        gate.assert_called_once();self.assertIn('invalid depth',geometry.BOOK_SELECTION_NOTE)

    def test_successful_sam_does_not_call_gpt(self):
        import cart_book_grasp_v1_3_motion as motion
        geometry=SimpleNamespace(fill_book_meta=Mock(),_load_sam3_detector=lambda:(None,None,SimpleNamespace(fromarray=lambda a:a)),
            _sam3_raw_books=lambda *a,**kw:[(.8,self.spine,[])],BOOK_GRASP_MODE='cart+')
        fallback=Mock();motion._install_spine_center(geometry,spine_fallback=fallback)
        selected=dict(mask=self.spine,box=[128,24,151,216],uv=[139,120],depth_m=.8)
        with patch.object(motion,'_spine_center_from_mask',return_value=selected),patch.object(motion,'whole_book_depth',return_value=dict(whole_book_depth_median_mm=800,book_surface_x_mm=600)):
            self.assertIsNotNone(geometry.segment_cart_book_with_sam3(self.rgb,np.ones((240,320)),np.eye(3),np.eye(4)))
        fallback.assert_not_called()

    def test_gpt_mixed_depth_accepted_without_region_recovery(self):
        from cart_book_grasp_v3_2_head_fallback import center_from_mask
        import cart_book_grasp_v1_3_perception as perception
        depth=np.full(self.spine.shape,800.,float)
        depth[:,::2]=1100.
        k=np.array([[600,0,160],[0,600,120],[0,0,1.]])
        with patch.object(perception,'recover_stable_spine',side_effect=ValueError('no stable region')) as recovery:
            result=center_from_mask(self.spine,depth,k)
            recovery.assert_not_called()
            self.assertEqual(result['spine_depth_consistency_check'],'skipped_gpt')
            self.assertNotIn('spine_center_recovery',result)
            self.assertTrue(np.isfinite(result['point_camera_m']).all())
            self.assertGreater(np.ptp(result['depth_p10_p90_mm']),100)
            with self.assertRaisesRegex(ValueError,'mixed with background'):
                perception._spine_center_from_mask(self.spine,depth,k)
            recovery.assert_called_once()

    def test_gpt_still_requires_real_valid_depth(self):
        from cart_book_grasp_v3_2_head_fallback import center_from_mask
        for value in (0.,float('nan'),float('inf'),-1.):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'Too few valid depths'):
                center_from_mask(self.spine,np.full(self.spine.shape,value),np.eye(3))

    def test_skip_head_roll_without_3d_tilt_for_any_detector(self):
        from cart_book_grasp_v1_3_perception import _wrist_config_for_geometry
        from book_spine_wrist import align_j7
        config={'enabled':True}
        selected={'spine_depth_consistency_check':'skipped_gpt','spine_3d_status':'unavailable'}
        effective=_wrist_config_for_geometry(config,selected)
        self.assertFalse(effective['enabled']);self.assertTrue(config['enabled'])
        robot=Mock();record={}
        self.assertTrue(align_j7(robot,selected,effective,-170,170,record))
        robot.get_current_state.assert_not_called()
        self.assertTrue(record['complete']);self.assertEqual(record['delta_deg'],0.)
        self.assertTrue(_wrist_config_for_geometry(config,dict(selected,spine_3d_status='valid'))['enabled'])
        self.assertFalse(_wrist_config_for_geometry(config,{'spine_3d_status':'unavailable'})['enabled'])
        self.assertFalse(_wrist_config_for_geometry(config,{})['enabled'])
        # SAM3 selection has no GPT marker but must use the same zero-rotation
        # completion record, so the later undo stage does not demand a rotation.
        record={}
        self.assertTrue(align_j7(robot,{},_wrist_config_for_geometry(config,{}),-170,170,record))
        self.assertTrue(record['complete']);self.assertEqual(record['delta_deg'],0.)
        robot.get_current_state.assert_not_called()

    def test_fallback_center_policy_is_scoped_to_gpt_candidates(self):
        import cart_book_grasp_v1_3_motion as motion
        geometry=SimpleNamespace(fill_book_meta=Mock(),_load_sam3_detector=lambda:(None,None,SimpleNamespace(fromarray=lambda a:a)),
                                 _sam3_raw_books=lambda *a,**kw:[],BOOK_GRASP_MODE='cart+')
        fallback=Mock(return_value=([(.9,self.spine,[])],[ (.9,self.book,[])]))
        selected=dict(mask=self.spine,box=[128,24,151,216],uv=[139,120],depth_m=.8)
        center=Mock(return_value=selected)
        motion._install_spine_center(geometry,spine_fallback=fallback,spine_fallback_center=center)
        with patch.object(motion,'_spine_center_from_mask',side_effect=AssertionError('SAM3-only path')) as regular,patch.object(motion,'whole_book_depth',return_value=dict(whole_book_depth_median_mm=800,book_surface_x_mm=600)):
            self.assertIsNotNone(geometry.segment_cart_book_with_sam3(self.rgb,np.ones((240,320)),np.eye(3),np.eye(4)))
        regular.assert_not_called();center.assert_called_once()
        self.assertEqual(selected['selection_source'],'gpt6_polygon_fallback')

if __name__=='__main__':unittest.main()
