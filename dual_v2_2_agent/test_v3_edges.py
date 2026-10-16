"""Offline direct-edge tests: no camera, robot, model or texture tracker."""
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from cart_book_grasp_v3_edges import EdgeCenterlineLock
from cart_book_grasp_v3_servo import ServoError


def scene(center=180.,width=48.,slope=.12,top=0,bottom=240,color=(205,190,160)):
    h,w=240,360;y,x=np.mgrid[:h,:w];mid=center+slope*(y-(h-1)/2)
    mask=(np.abs(x-mid)<=width/2)&(y>=top)&(y<bottom)
    rgb=np.full((h,w,3),(35,45,55),np.uint8);rgb[mask]=color
    # Adjacent book outside the locked book's two local edge corridors.
    rgb[:,280:325]=[150,30,45]
    return rgb,mask


class EdgeTests(unittest.TestCase):
    def test_textureless_books_of_different_colors_need_no_corners_or_flow(self):
        for color in ((205,190,160),(180,25,40),(20,170,90)):
            rgb,mask=scene(color=color)
            with patch.object(cv2,'goodFeaturesToTrack',side_effect=AssertionError('corners called')), \
                 patch.object(cv2,'calcOpticalFlowPyrLK',side_effect=AssertionError('LK called')):
                tracker=EdgeCenterlineLock(rgb,mask)
                for i in range(6):
                    rgb,_=scene(center=180+2*i,slope=.12-.015*i,width=48+2*i,color=color)
                    o=tracker.observe(rgb)
                    self.assertAlmostEqual(o['uv'][0],180+2*i,delta=1.)
                    self.assertAlmostEqual(o['angle_deg'],np.degrees(np.arctan(.12-.015*i)),delta=.5)
                    self.assertNotIn('points',o)

    def test_up_down_clipping_and_scale_growth_do_not_deplete_features(self):
        rgb,mask=scene(top=0,bottom=150);tracker=EdgeCenterlineLock(rgb,mask)
        for i in range(40):
            rgb,_=scene(center=180-.4*i,width=48+1.2*i,top=0,bottom=150+i*3)
            o=tracker.observe(rgb)
            self.assertAlmostEqual(o['uv'][0],180-.4*i,delta=1.2)
        self.assertGreater(o['scale'],1.8)
        self.assertGreaterEqual(min(o['edge_support']),32)

    def test_combined_y_and_roll_step_can_shift_edges_eighteen_pixels(self):
        rgb,mask=scene();tracker=EdgeCenterlineLock(rgb,mask)
        rgb,_=scene(center=162.,slope=.07,width=50.)
        o=tracker.observe(rgb)
        self.assertAlmostEqual(o['uv'][0],162,delta=1.2)
        self.assertAlmostEqual(o['angle_deg'],np.degrees(np.arctan(.07)),delta=.6)

    def test_perspective_sides_need_not_be_exactly_parallel(self):
        h,w=240,360;y,x=np.mgrid[:h,:w]
        left=155+.1*(y-119.5);right=205+.02*(y-119.5)
        mask=(x>=left)&(x<=right)
        rgb=np.full((h,w,3),35,np.uint8);rgb[mask]=200
        tracker=EdgeCenterlineLock(rgb,mask);o=tracker.observe(rgb)
        self.assertAlmostEqual(o['uv'][0],180,delta=1.)
        self.assertAlmostEqual(o['angle_deg'],np.degrees(np.arctan(.06)),delta=.6)

    def test_same_image_does_not_walk_to_another_edge(self):
        rgb,mask=scene();tracker=EdgeCenterlineLock(rgb,mask)
        values=[tracker.observe(rgb)['uv'][0] for _ in range(20)]
        self.assertLess(np.ptp(values),.2)

    def test_blank_missing_side_and_neighbor_cannot_be_used_as_book(self):
        for failure in ('blank','one_side','neighbor'):
            rgb,mask=scene();tracker=EdgeCenterlineLock(rgb,mask)
            if failure=='blank':rgb[:]=35
            elif failure=='one_side':
                y,x=np.mgrid[:240,:360];rgb[x>180+.12*(y-119.5)]=[205,190,160]
            else:rgb,_=scene(center=280)
            with self.subTest(failure=failure),self.assertRaises(ServoError):tracker.observe(rgb)

    def test_limited_text_and_occlusion_do_not_define_the_center(self):
        rgb,mask=scene();tracker=EdgeCenterlineLock(rgb,mask)
        for y in range(20,230,35):cv2.putText(rgb,'B',(173,y),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),2)
        rgb[180:220,140:230]=[100,130,180]
        o=tracker.observe(rgb)
        self.assertAlmostEqual(o['uv'][0],180,delta=2)
        self.assertAlmostEqual(o['angle_deg'],np.degrees(np.arctan(.12)),delta=1)


if __name__=='__main__':unittest.main()
