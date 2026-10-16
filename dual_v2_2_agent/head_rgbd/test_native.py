import unittest,time
from types import SimpleNamespace
import numpy as np
from .native import highest_color_profile,scaled_intrinsics
from .protocol import encode,decode
from .client import CalibratedCamera
from .test_shared import META,CAL

class NativeTests(unittest.TestCase):
 def test_highest_resolution_not_largest_fps(self):
  def profile(w,h,fps,fmt):
   p=SimpleNamespace(get_width=lambda:w,get_height=lambda:h,get_fps=lambda:fps,get_format=lambda:fmt)
   p.as_video_stream_profile=lambda:p
   return p
  best=profile(1280,800,15,'rgb')
  self.assertIs(highest_color_profile([profile(848,530,30,'rgb'),best,profile(1280,800,5,'mjpg')],('rgb','mjpg')),best)
 def test_intrinsic_resize_ray_equivalence(self):
  k=np.array([[400,0,425],[0,399,262],[0,0,1.]])
  native=scaled_intrinsics(k,(848,530),(1280,800))
  p=np.array([600.,390.,1.]);q=np.r_[(p[:2]+.5)*np.array([1280/848,800/530])-.5,1.]
  np.testing.assert_allclose(np.linalg.solve(k,p),np.linalg.solve(native,q))
 def packet(self):
  native=np.full((8,12,3),60,np.uint8);native[:,:,0]=70
  meta=dict(META,native_width=12,native_height=8,native_is_maximum=True,_native_bgr=native,_native_depth=np.full((8,12),123.5,np.float32))
  return decode(encode(np.full((4,6,3),20,np.uint8),np.ones((4,6),np.float32),time.time(),meta,'test:1'))
 def test_native_pair_transport_and_camera(self):
  bgr,depth,header=self.packet()
  self.assertEqual(bgr.shape,(4,6,3));self.assertEqual(header['native_bgr'].shape,(8,12,3))
  subscriber=SimpleNamespace(read=lambda:(bgr,depth,header),close=lambda:None)
  native=CalibratedCamera(CAL,warmup_frames=1,subscriber=subscriber,prefer_native=True)
  self.assertEqual(native.rgb_image.shape,(8,12,3));self.assertEqual(native.depth_map[0,0],123.5)
  self.assertEqual(native.rgb_image[0,0].tolist(),[60,60,70]);self.assertEqual((native.w,native.h),(12,8))
  np.testing.assert_allclose(native.K,scaled_intrinsics(CAL['K'],(6,4),(12,8)))
  legacy=CalibratedCamera(CAL,warmup_frames=1,subscriber=subscriber)
  self.assertEqual(legacy.rgb_image.shape,(4,6,3));np.testing.assert_array_equal(legacy.K,CAL['K'])
  native.T[2,0]=15;native.capture_image();self.assertEqual(native.c2w[2,3],15)
 def test_camera_waits_through_incomplete_startup_frames(self):
  from hand_workspace_ar.camera import Camera
  from unittest.mock import Mock
  color=SimpleNamespace(get_timestamp_us=lambda:1000000)
  depth=SimpleNamespace(get_timestamp_us=lambda:1000000,get_data=lambda:np.ones((8,12),np.uint16).tobytes(),get_height=lambda:8,get_width=lambda:12,get_depth_scale=lambda:1.)
  incomplete=SimpleNamespace(get_color_frame=lambda:color,get_depth_frame=lambda:None)
  complete=SimpleNamespace(get_color_frame=lambda:color,get_depth_frame=lambda:depth)
  pipeline=SimpleNamespace(wait_for_frames=Mock(side_effect=[incomplete]*6+[complete]))
  obj=Camera.__new__(Camera);obj.camera=SimpleNamespace(pipeline=pipeline,w=6,h=4,_frame_to_rgb=lambda c:np.zeros((8,12,3),np.uint8))
  obj.last_stamp=obj.last_depth_stamp=obj.offset=None;obj.native_size=(12,8)
  bgr,d,_=obj.read()
  self.assertEqual(pipeline.wait_for_frames.call_count,7)
  self.assertEqual(bgr.shape,(4,6,3));self.assertEqual(obj.native_bgr.shape,(8,12,3))
  self.assertEqual(obj.native_depth.shape,(8,12))
 def test_no_upscaled_fallback_if_native_missing(self):
  bgr,depth,h=self.packet();h.pop('native_bgr');h.pop('native_depth')
  with self.assertRaisesRegex(ValueError,'最高原始'):
   CalibratedCamera(CAL,1,SimpleNamespace(read=lambda:(bgr,depth,h),close=lambda:None),prefer_native=True)

if __name__=='__main__':unittest.main()
