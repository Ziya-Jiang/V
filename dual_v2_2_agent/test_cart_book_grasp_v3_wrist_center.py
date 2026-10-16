"""Center selection is independent of head image, score ordering and book color."""
import unittest
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_tracking import LockedBookTracker
from cart_book_grasp_v3_wrist_center import select_center_book, require_points_in_book


class WristCenterTests(unittest.TestCase):
    def test_nearest_centroid_wins_not_highest_score(self):
        rgb=np.zeros((200,300,3),np.uint8)
        far=np.zeros((200,300),bool);far[20:180,10:45]=True
        near=np.zeros_like(far);near[30:170,135:165]=True
        chosen,report=select_center_book(rgb,[(.99,far,[]),(.5,near,[])])
        np.testing.assert_array_equal(chosen['mask'],near)
        self.assertEqual(report['selected_index'],0)
        self.assertAlmostEqual(chosen['distance_px'],0)

    def test_no_valid_detection_does_not_choose_background(self):
        rgb=np.zeros((200,300,3),np.uint8)
        with self.assertRaises(ServoError):select_center_book(rgb,[])
        with self.assertRaises(ServoError):select_center_book(rgb,[(.1,np.ones((200,300),bool),[])])

    def test_reference_points_cannot_jump_to_neighbor_or_outside_image(self):
        mask=np.zeros((100,200),bool);mask[20:80,30:70]=True
        good=np.float32([[40,30],[60,30],[60,60],[40,60]])
        require_points_in_book({'points':good},{'mask':mask})
        for bad in [good+[80,0],good+[-250,0],good*np.nan]:
            with self.assertRaises(ServoError):require_points_in_book({'points':bad},{'mask':mask})

    def test_visible_book_clipped_at_top_can_be_locked_locally(self):
        rng=np.random.default_rng(15)
        rgb=rng.integers(0,256,(240,320,3),dtype=np.uint8)
        rgb=cv2.GaussianBlur(rgb,(3,3),.6)
        mask=np.zeros((240,320),np.uint8);mask[:210,80:240]=1
        tracker=LockedBookTracker(rgb,mask,point=[160,100],allow_clipped=True)
        observation=tracker.observe(rgb)
        np.testing.assert_allclose(observation['uv'],[160,100],atol=.1)
        self.assertGreater(observation['inliers'],12)


if __name__=='__main__':unittest.main()
