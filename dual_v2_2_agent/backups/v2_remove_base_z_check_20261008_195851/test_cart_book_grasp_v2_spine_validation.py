import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from cart_book_grasp_v2_spine_validation import validate_spine_base_depth,load_config
from cart_book_grasp_v2_target import prepare_instance_target
from cart_book_grasp_v2_agent import SelectionError


class SpineBaseDepthTests(unittest.TestCase):
    def test_recorded_top_face_is_rejected(self):
        geometry=dict(spine_endpoints_camera_m=[[.63269975,.03026948,-.00296239],
                                               [.53224109,.03061248,.01378434]])
        with self.assertRaisesRegex(ValueError,'疑似书顶面'):
            validate_spine_base_depth(geometry,np.eye(4),load_config())

    def test_sideways_tilt_is_allowed_but_forward_depth_change_is_not(self):
        c=load_config()
        # Lateral lean is not confused with an X-depth ramp.
        good=dict(spine_endpoints_camera_m=[[.6,0,0],[.61,.10,.10]])
        stats=validate_spine_base_depth(good,np.eye(4),c)
        self.assertAlmostEqual(stats['spine_base_x_change_mm'],10.)
        for end in ([.7,0,.10],[.62,0,.04],[.6,.10,.005]):
            with self.subTest(end=end),self.assertRaises(ValueError):
                validate_spine_base_depth(dict(spine_endpoints_camera_m=[[.6,0,0],end]),np.eye(4),c)

    def test_check_uses_base_coordinates_not_camera_depth(self):
        # Optical camera Z maps to robot-base Z here, and may vary considerably.
        rotation=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
        tf=np.eye(4);tf[:3,:3]=rotation;tf[:3,3]=[1000,2000,3000]
        endpoints=np.array([[0,0,.6],[.02,.005,.8]])
        stats=validate_spine_base_depth(dict(spine_endpoints_camera_m=endpoints),tf,load_config())
        self.assertAlmostEqual(stats['spine_base_x_change_mm'],5.)
        self.assertAlmostEqual(stats['spine_base_z_span_mm'],200.)

    def test_missing_or_nan_geometry_cannot_pass(self):
        for value in (None,[],[[0,0,0],[0,0,float('nan')]]):
            with self.subTest(value=value),self.assertRaises(ValueError):
                validate_spine_base_depth(dict(spine_endpoints_camera_m=value),np.eye(4),load_config())

    def run_target(self,second_valid):
        shape=(120,120);rgb=np.zeros((*shape,3),np.uint8);depth=np.ones(shape)*800
        book=np.zeros(shape,bool);book[20:100,40:80]=True
        top=np.zeros(shape,bool);top[25:85,50:60]=True
        spine=np.zeros(shape,bool);spine[25:85,52:62]=True
        candidate=dict(id='book_1',book_mask=book,mask=book,box=[40,20,80,100],score=.9,
                       recognition='book_proposal',geometry=None,geometry_status='not_checked',error='')
        detector=SimpleNamespace(_load_sam3_detector=lambda:(None,None,SimpleNamespace(fromarray=lambda x:x)),
                                 _sam3_raw_books=Mock(return_value=[]))
        tried=[]
        def prepare(trial,spines,*args):
            mask=spines[0][1];tried.append(mask)
            good=second_valid and np.array_equal(mask,spine)
            trial.update(geometry=dict(spine_endpoints_camera_m=[[0,0,0],[.005,0,.10] if good else [.10,0,.015]]),
                         mask=mask,geometry_status='ready')
        with patch('cart_book_grasp_v2_target.restore_crop',return_value=[(.99,top,[]),(.8,spine,[])]), \
             patch('cart_book_grasp_v2_target.prepare_selected_geometry',side_effect=prepare):
            if second_valid:
                prepare_instance_target(candidate,rgb,depth,np.eye(3),np.eye(4),detector,None,
                                        'cart_book_grasp_v2_agent_config.json')
                np.testing.assert_equal(candidate['mask'],spine)
                self.assertEqual(candidate['geometry']['spine_base_depth_status'],'valid')
            else:
                with self.assertRaises(SelectionError):
                    prepare_instance_target(candidate,rgb,depth,np.eye(3),np.eye(4),detector,None,
                                            'cart_book_grasp_v2_agent_config.json')
                self.assertIsNone(candidate['geometry'])
                self.assertEqual(candidate['geometry_status'],'failed')
        self.assertEqual(len(tried),2)  # Their >0.65 mask IoU must not hide the valid alternative.
        np.testing.assert_equal(candidate['book_mask'],book)
        self.assertEqual(candidate['id'],'book_1')
        self.assertEqual(candidate['spine_candidate_reviews'][0]['status'],'rejected')

    def test_reject_top_and_try_same_books_next_spine(self):self.run_target(True)
    def test_all_top_candidates_leave_no_stale_geometry(self):self.run_target(False)


if __name__=='__main__':unittest.main()
