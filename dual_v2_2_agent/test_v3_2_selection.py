"""Offline initial-book recall gates; no camera, GPU or robot access."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock
import numpy as np
from cart_book_grasp_v3_2_selection import select_wrist_book,spine_geometry
from cart_book_grasp_v3_wrist_center import detector_from_geometry
from cart_book_grasp_v3_servo import ServoError


def scene(center=160,slope=.06):
    y,x=np.mgrid[:240,:320]
    mask=np.abs(x-(center+slope*(y-119.5)))<15
    return np.zeros((240,320,3),np.uint8),mask


class RecallTests(unittest.TestCase):
    def test_high_score_book_still_requires_spine_detection(self):
        rgb,mask=scene();detector=Mock(return_value=[(.8,mask,[])])
        chosen,report=select_wrist_book(rgb,detector)
        self.assertEqual(detector.call_count,2)
        self.assertEqual(report['initial_detection']['method'],'book_supported_spine')
        np.testing.assert_array_equal(chosen['mask'],mask)

    def test_cover_on_either_side_cannot_shift_spine_center(self):
        from cart_book_grasp_v3_feature_centerline import FeatureCenterlineLock
        for side in (-1,1):
            with self.subTest(cover_side=side):
                rgb,spine=scene(slope=.3);y,x=np.mgrid[:240,:320]
                center=160+.3*(y-119.5)
                cover=(side*(x-center)>0)&(side*(x-center)<70)
                book=spine|cover
                rng=np.random.default_rng(12)
                rgb[:]=rng.integers(0,256,rgb.shape,dtype=np.uint8)
                chosen,summary=select_wrist_book(rgb,Mock(side_effect=[[(.95,book,[])],[(.3,spine,[])]]))
                np.testing.assert_array_equal(chosen['mask'],spine)
                self.assertAlmostEqual(chosen['uv'][0],160,delta=1)
                lock=FeatureCenterlineLock(rgb,chosen['mask'],tracking_mask=chosen['tracking_mask'])
                pts=np.rint(lock.reference).astype(int)
                self.assertTrue(chosen['tracking_mask'][pts[:,1],pts[:,0]].all())
                self.assertTrue((abs(pts[:,0]-(160+.3*(pts[:,1]-119.5)))<10).all())

    def test_occluded_spine_uses_visible_component_without_cover(self):
        rgb,spine=scene(slope=.3);spine[180:200]=False
        book=np.ones_like(spine);book[:,:70]=False;book[:,250:]=False
        chosen,_=select_wrist_book(rgb,Mock(side_effect=[[(.9,book,[])],[(.3,spine,[])]]))
        self.assertAlmostEqual(chosen['uv'][0],160,delta=1)
        self.assertFalse(chosen['mask'][180:].any())
        self.assertFalse((chosen['tracking_mask'] & ~spine).any())

    def test_cover_only_high_score_book_is_not_a_spine(self):
        rgb,mask=scene()
        with self.assertRaises(ServoError):
            select_wrist_book(rgb,Mock(side_effect=[[(.99,mask,[])],[]]))

    def test_nested_broad_spine_prediction_does_not_override_narrow_surface(self):
        rgb,spine=scene();y,x=np.mgrid[:240,:320]
        broad=(x>145)&(x<205)
        chosen,_=select_wrist_book(rgb,Mock(side_effect=[[(.9,broad|spine,[])],[(.7,broad,[]),(.4,spine,[])]]))
        np.testing.assert_array_equal(chosen['mask'],spine)

    def test_low_score_clipped_book_requires_second_prompt_agreement(self):
        rgb,mask=scene();calls=[]
        def detect(image,prompt='book'):
            calls.append(prompt)
            return [(.173 if prompt=='book' else .207,mask,[])]
        chosen,report=select_wrist_book(rgb,detect)
        self.assertEqual(calls,['book','book spine'])
        self.assertTrue(chosen['mask'][0].any() and chosen['mask'][-1].any())
        self.assertEqual(report['initial_detection']['consensus'][0]['mask_iou'],1.)
        self.assertAlmostEqual(chosen['uv'][0],160,delta=1.)

    def test_disagreeing_spine_or_absent_spine_does_not_select(self):
        rgb,mask=scene();_,other=scene(190)
        for spine in ([],[(.2,other,[])]):
            with self.subTest(spine_count=len(spine)):
                detector=Mock(side_effect=[[(.2,mask,[])],spine])
                with self.assertRaisesRegex(ServoError,'lacks mask and centerline agreement'):
                    select_wrist_book(rgb,detector)

    def test_empty_low_score_and_background_do_not_fabricate_center(self):
        rgb,mask=scene()
        for raw in ([],[(.1,mask,[])],[(.2,np.ones_like(mask),[])],[(float('nan'),mask,[])]):
            with self.subTest(count=len(raw)):
                detector=Mock(return_value=raw)
                with self.assertRaises(ServoError):select_wrist_book(rgb,detector)
                self.assertEqual(detector.call_count,2)

    def test_nearest_book_is_selected_from_agreed_candidates(self):
        rgb,center=scene();_,side=scene(205)
        raw=[(.22,side,[]),(.19,center,[])]
        chosen,_=select_wrist_book(rgb,Mock(return_value=raw))
        np.testing.assert_array_equal(chosen['mask'],center)

    def test_sides_must_be_parallel_and_have_center_row_support(self):
        rgb,mask=scene();y,x=np.mgrid[:240,:320]
        wedge=(x>145-.2*y)&(x<170+.2*y)
        with self.assertRaisesRegex(ServoError,'perspective width'):spine_geometry(wedge)
        mask[100:]=False
        with self.assertRaisesRegex(ServoError,'center row'):spine_geometry(mask)

    def test_failure_diagnostics_include_raw_score_and_reason(self):
        rgb,mask=scene()
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ServoError):
                select_wrist_book(rgb,Mock(side_effect=[[(.17,mask,[])],[]]),directory=d)
            record=json.loads((Path(d)/'wrist_detection.json').read_text())
            self.assertEqual(record['status'],'failed')
            candidate=record['attempts'][0]['candidates'][0]
            self.assertEqual(candidate['score'],.17)
            self.assertEqual(candidate['strict_reason'],'score below original threshold 0.5')
            self.assertTrue(candidate['recall_geometry_valid'])
            self.assertTrue((Path(d)/'wrist_detection_book.npz').exists())

    def test_checkpoint_failure_is_preserved(self):
        rgb,mask=scene();detector=Mock()
        def cancel():raise RuntimeError('operator stopped')
        with self.assertRaisesRegex(RuntimeError,'operator stopped'):
            select_wrist_book(rgb,detector,checkpoint=cancel)
        detector.assert_not_called()

    def test_detector_factory_keeps_book_default_and_accepts_spine_prompt(self):
        rgb,_=scene();geometry=Mock();image_cls=Mock()
        geometry._load_sam3_detector.return_value=('processor','numpy',image_cls)
        detect=detector_from_geometry(geometry);detect(rgb);detect(rgb,prompt='book spine')
        self.assertEqual([c.kwargs['prompt'] for c in geometry._sam3_raw_books.call_args_list],['book','book spine'])

    def test_strong_spine_recovers_empty_book_detection(self):
        rgb,mask=scene();detector=Mock(side_effect=[[],[(.7,mask,[])]])
        chosen,report=select_wrist_book(rgb,detector)
        self.assertEqual(report['initial_detection']['method'],'high_confidence_spine')

    def test_edges_recover_both_empty_detectors_for_tilted_clipped_spine(self):
        rgb,mask=scene(slope=.15);rgb[:]=25;rgb[mask]=180
        chosen,report=select_wrist_book(rgb,Mock(return_value=[]))
        self.assertEqual(report['initial_detection']['method'],'paired_image_boundaries')
        self.assertFalse(report['initial_detection']['semantic_identity_verified'])
        self.assertAlmostEqual(chosen['uv'][0],160,delta=2.)
        from cart_book_grasp_v3_centerline import visible_centerline
        _,angle=visible_centerline(chosen['mask'])
        self.assertAlmostEqual(angle,np.degrees(np.arctan(.15)),delta=1.)

    def test_only_one_edge_does_not_form_a_book(self):
        rgb,_=scene();rgb[:]=25;rgb[:,160:]=180
        with self.assertRaisesRegex(ServoError,'boundary fallback failed'):
            select_wrist_book(rgb,Mock(return_value=[]))

    def test_two_equally_central_edge_pairs_are_rejected(self):
        from cart_book_grasp_v3_2_edges import select_boundary_pair
        from unittest.mock import patch
        rgb=np.full((240,320,3),25,np.uint8);rgb[:,109:140]=180;rgb[:,179:210]=180
        lines=np.array([[[109,0,109,239]],[[139,0,139,239]],[[179,0,179,239]],[[209,0,209,239]]])
        # Cross-book pairs exceed the conservative max width of 20% image.
        with patch('cv2.HoughLinesP',return_value=lines):
            with self.assertRaisesRegex(ServoError,'ambiguous'):
                select_boundary_pair(rgb,{})


if __name__=='__main__':unittest.main()
