"""Offline V3.4 perception tests. No robot, camera, network or model calls."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_4_edges import select_boundary_pair,join_segments,color_boundary_support,deduplicate_pairs
from cart_book_grasp_v3_2_selection import select_wrist_book,spine_tracking_interior
from cart_book_grasp_v3_feature_centerline import FeatureCenterlineLock


def scene():
    y,x=np.mgrid[:480,:640]
    mask=abs(x-(320+.06*(y-239.5)))<(80-.035*y)/2
    rgb=np.full((480,640,3),35,np.uint8);rgb[mask]=[200,170,130]
    return rgb,mask


class V34Tests(unittest.TestCase):
    def test_tilted_perspective_spine_with_internal_text(self):
        rgb,mask=scene()
        for y in range(40,450,32):cv2.putText(rgb,'ABC',(300,y),cv2.FONT_HERSHEY_SIMPLEX,.4,(20,20,20),1)
        chosen=select_boundary_pair(rgb,{})
        self.assertAlmostEqual(chosen['uv'][0],320,delta=2)
        # In particular the fitted band must not shrink to the text column.
        self.assertGreater(chosen['mask'][240].sum(),65)

    def test_short_collinear_segments_join_without_absorbing_text_column(self):
        segments=[dict(u=100.,slope=.06,top=float(y),bottom=float(y+40)) for y in range(40,341,50)]
        segments += [dict(u=125.,slope=.06,top=float(y),bottom=float(y+40)) for y in range(40,341,50)]
        lines=join_segments(segments,480)
        self.assertEqual(len(lines),2)
        self.assertTrue(all(l['segment_count']==7 for l in lines))
        self.assertAlmostEqual(lines[0]['u'],100,delta=.01)
        self.assertAlmostEqual(lines[1]['u'],125,delta=.01)

    def test_only_one_boundary_and_blank_images_rejected(self):
        rgb=np.full((480,640,3),35,np.uint8)
        with self.assertRaises(ServoError):select_boundary_pair(rgb,{})
        rgb[:,320:]=200
        with self.assertRaises(ServoError):select_boundary_pair(rgb,{})

    def test_text_on_uniform_paper_not_a_spine(self):
        rgb=np.full((480,640,3),180,np.uint8)
        for y in range(40,450,30):cv2.putText(rgb,'BOOK TEXT',(255,y),cv2.FONT_HERSHEY_SIMPLEX,.5,(20,20,20),1)
        with self.assertRaises(ServoError):select_boundary_pair(rgb,{})

    def test_narrow_stroke_with_paper_both_sides_rejected(self):
        lab=np.full((300,160,3),[180,128,128],np.uint8);lab[:,78:81,0]=20
        edge=np.zeros((300,160),np.uint8);edge[:,77]=255
        with self.assertRaisesRegex(ServoError,'internal text'):
            color_boundary_support(edge,lab,np.arange(300),np.full(300,77))

    def test_brightness_matching_background_recovers_using_chroma(self):
        lab=np.full((300,160,3),[150,128,128],np.uint8);lab[:,80:,2]=155
        edge=cv2.Canny(lab[:,:,2],15,35)
        support=color_boundary_support(edge,lab,np.arange(300),np.full(300,79))
        self.assertEqual(support['color_channel'],'b')
        self.assertEqual(support['broad_color_support'],1.)

    def test_interrupted_long_edge_not_filled_with_invented_support(self):
        lab=np.full((300,160,3),[50,128,128],np.uint8);lab[:,80:,0]=190
        edge=cv2.Canny(lab[:,:,0],50,130);edge[120:155]=0
        with self.assertRaisesRegex(ServoError,'continuous pixel'):
            color_boundary_support(edge,lab,np.arange(300),np.full(300,79))

    def test_two_equally_central_bands_rejected(self):
        rgb=np.full((480,640,3),35,np.uint8);rgb[:,200:260]=180;rgb[:,380:440]=180
        with self.assertRaisesRegex(ServoError,'ambiguous'):select_boundary_pair(rgb,{})

    @staticmethod
    def band(left=200,right=280,top=0,bottom=480,slope=0.,ids=None):
        y,x=np.mgrid[:480,:640]
        mask=(y>=top)&(y<bottom)&(x>=left+slope*(y-239.5))&(x<=right+slope*(y-239.5))
        return (abs((left+right)/2-319.5),mask,{},set(ids or [left,right]))

    def test_partial_duplicate_retains_longest_original_supported_fit(self):
        long=self.band(top=100,bottom=460,ids=[0,1])
        short=self.band(left=202,right=281,top=20,bottom=340,ids=[2,1])
        report={};result=deduplicate_pairs([short,long],report)
        self.assertEqual(len(result),1)
        self.assertIs(result[0][1],long[1])
        self.assertEqual(result[0][3],{0,1,2})
        self.assertEqual(report['candidate_groups'][0]['representative_pair_index'],1)

    def test_adjacent_or_nested_bands_not_duplicates(self):
        base=self.band()
        for other in [self.band(left=280,right=360),self.band(left=220,right=260),
                      self.band(left=180,right=300)]:
            with self.subTest(other=other[3]):
                self.assertEqual(len(deduplicate_pairs([base,other],{})),2)

    def test_same_center_but_crossing_sides_not_duplicates(self):
        a=self.band(slope=.1);b=self.band(slope=-.1)
        self.assertEqual(len(deduplicate_pairs([a,b],{})),2)

    def test_insufficient_common_rows_not_duplicates(self):
        a=self.band(top=0,bottom=300);b=self.band(top=280,bottom=480)
        self.assertEqual(len(deduplicate_pairs([a,b],{})),2)

    def test_duplicate_chain_cannot_merge_different_bands(self):
        bands=[self.band(left=200+i,right=300+i) for i in (0,3,6)]
        self.assertEqual(len(deduplicate_pairs(bands,{})),2)

    def test_actual_duplicate_failure_and_perturbations(self):
        p=Path(__file__).parent/'RobotRAM/output/v3_column_204b15f9d16046c79f5ef5969b65eb60/initial_wrist.png'
        if not p.exists():self.skipTest('Local recorded frame unavailable')
        rgb=cv2.cvtColor(cv2.imread(str(p)),cv2.COLOR_BGR2RGB)
        report={};chosen=select_boundary_pair(rgb,report)
        self.assertEqual(len(report['pairs']),2)
        self.assertEqual(report['distinct_candidate_count'],1)
        self.assertEqual(len(report['candidate_groups'][0]['pair_indices']),2)
        self.assertAlmostEqual(chosen['uv'][0],744.50,delta=1.)
        lock=FeatureCenterlineLock(rgb,chosen['mask'],tracking_mask=spine_tracking_interior(chosen['mask']))
        self.assertGreaterEqual(len(lock.reference),6)
        darker=np.clip(rgb.astype(float)*.85,0,255).astype(np.uint8)
        r=select_boundary_pair(darker,{})
        self.assertAlmostEqual(r['uv'][0],744.5,delta=4.)
        # Translation changes LSD's segment hypotheses on this image. The
        # existing perspective/support filter then has no accepted pair.
        # Deduplication must not invent one to make a perturbation pass.
        shifted=cv2.warpAffine(rgb,np.float32([[1,0,12],[0,1,0]]),(1280,720))
        report={}
        with self.assertRaisesRegex(ServoError,'No supported'):
            select_boundary_pair(shifted,report)
        self.assertEqual(report['pairs'],[])

    def test_explicit_backend_only_applies_to_v34_call(self):
        rgb,mask=scene()
        with tempfile.TemporaryDirectory() as folder:
            chosen,summary=select_wrist_book(rgb,Mock(return_value=[]),directory=folder,boundary_selector=select_boundary_pair)
            report=json.loads((Path(folder)/'wrist_detection.json').read_text())
            self.assertEqual(report['method'],'joined_color_boundaries_v3_4')
            self.assertFalse((chosen['tracking_mask'] & ~chosen['mask']).any())
        _,original=select_wrist_book(rgb,Mock(return_value=[]))
        self.assertEqual(original['criterion'],'paired_image_boundaries')

    def test_real_saved_failures_and_tracker_initialization(self):
        root=Path(__file__).parent/'RobotRAM/output'
        cases=[('8793d1e3ca844918af1cd9f0551b9672',711.5),('cc566140166e48ff9416319882ed53de',656.)]
        if not all((root/f'v3_column_{name}/initial_wrist.png').exists() for name,_ in cases):
            self.skipTest('Local recorded frames unavailable')
        for name,expected in cases:
            with self.subTest(frame=name):
                rgb=cv2.cvtColor(cv2.imread(str(root/f'v3_column_{name}/initial_wrist.png')),cv2.COLOR_BGR2RGB)
                report={};chosen=select_boundary_pair(rgb,report)
                self.assertEqual(len(report['pairs']),1)
                self.assertAlmostEqual(chosen['uv'][0],expected,delta=3)
                interior=spine_tracking_interior(chosen['mask'])
                lock=FeatureCenterlineLock(rgb,chosen['mask'],tracking_mask=interior)
                self.assertGreaterEqual(len(lock.reference),6)
                # Modest image perturbations exercise selection stability;
                # they are not a replacement for real motion/video testing.
                for image,shift in [(np.clip(rgb.astype(float)*.85,0,255).astype(np.uint8),0),
                                    (cv2.warpAffine(rgb,np.float32([[1,0,12],[0,1,0]]),(rgb.shape[1],rgb.shape[0])),12)]:
                    result=select_boundary_pair(image,{})
                    self.assertAlmostEqual(result['uv'][0],expected+shift,delta=4)


if __name__=='__main__':unittest.main()
