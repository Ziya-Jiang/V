"""Offline perspective-spine acceptance and false-edge rejection."""
import unittest
import cv2
import numpy as np
from cart_book_grasp_v3_2_selection import spine_geometry
from cart_book_grasp_v3_2_edges import select_boundary_pair
from cart_book_grasp_v3_spine_geometry import perspective_width,boundary_support
from cart_book_grasp_v3_servo import ServoError

class PerspectiveTests(unittest.TestCase):
 def test_converging_and_diverging_spine_both_accepted(self):
  h,w=480,640;y,x=np.mgrid[:h,:w]
  for reverse in (False,True):
   widths=90-.05*y
   mask=abs(x-(320+.04*(y-239.5)))<widths/2
   if reverse:mask=mask[::-1].copy()
   rgb=np.full((h,w,3),30,np.uint8);rgb[mask]=190
   d={};chosen=select_boundary_pair(rgb,d)
   self.assertAlmostEqual(chosen['uv'][0],320,delta=2)
   self.assertGreater(d['selected_geometry']['width_ratio'],1.15)
 def test_more_than_five_degrees_can_be_valid(self):
  y,x=np.mgrid[:720,:1280]
  left=592+.083*(y-205);right=709-.008*(y-205)
  mask=(y>=205)&(y<=621)&(x>=left)&(x<=right)
  g=spine_geometry(mask)
  self.assertGreater(g['parallel_error_deg'],5)
  self.assertLess(g['width_ratio'],1.8)
 def test_crossing_or_extreme_taper_rejected(self):
  rows=np.arange(300)
  for right in (160-.3*rows,130+.6*rows):
   with self.assertRaises(ServoError):perspective_width(rows,np.full(300,100),right)
 def test_width_jump_not_interpreted_as_perspective(self):
  rows=np.arange(300);right=np.full(300,160.);right[150:]+=24
  with self.assertRaisesRegex(ServoError,'discontinuously'):
   perspective_width(rows,np.full(300,100),right)
 def test_merged_long_gap_rejected_even_with_high_overall_support(self):
  gray=np.zeros((300,150),np.uint8);gray[:,75:]=200
  edges=cv2.Canny(gray,50,130);edges[120:150]=0
  with self.assertRaisesRegex(ServoError,'continuous'):
   boundary_support(edges,gray,np.arange(300),np.full(300,74))
 def test_alternating_text_contrast_rejected(self):
  gray=np.zeros((300,150),np.uint8)
  for y in range(300):gray[y,75:] = 200 if (y//10)%2 else 0;gray[y,:75]=0 if (y//10)%2 else 200
  edges=np.zeros((300,150),np.uint8);edges[:,74]=255
  with self.assertRaisesRegex(ServoError,'possible text'):
   boundary_support(edges,gray,np.arange(300),np.full(300,74))

if __name__=='__main__':unittest.main()
