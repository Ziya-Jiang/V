"""No hardware: cross-view backend dispatch and retained geometry gates."""
import unittest
from unittest.mock import Mock, patch
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_tracking import LockedBookTracker


class SuperPointAcquisitionTests(unittest.TestCase):
    def tracker(self, variants, point=(70, 60)):
        image = np.zeros((120, 160, 3), np.uint8)
        mask = np.zeros((120, 160), np.uint8)
        mask[20:101, 20:121] = 1
        backend = Mock()
        backend.correspondences.return_value = variants
        with patch('cart_book_grasp_v3_superpoint.get_backend', return_value=backend):
            tracker = LockedBookTracker(image, mask, point=point, cross_view=True, cross_view_backend='superpoint')
        return tracker, image

    @staticmethod
    def points():
        return np.float32([[30,30],[70,30],[110,30],[30,90],[70,90],[110,90]])

    def test_six_points_can_acquire_without_sift_initialization(self):
        points = self.points()
        tracker, image = self.tracker([('full', points, points+[5,0], 6)])
        result = tracker.observe(image)
        self.assertEqual(result['inliers'], 6)
        np.testing.assert_allclose(result['uv'], [75,60], atol=.001)
        self.assertTrue(tracker.match_diagnostics[0]['accepted'])

    def test_five_points_do_not_acquire(self):
        points = self.points()[:5]
        tracker, image = self.tracker([('full', points, points, 5)])
        with self.assertRaisesRegex(ServoError, '5/6'):
            tracker.observe(image)

    def test_roi_fallback_and_conflicting_full_roi(self):
        points = self.points()
        empty = np.empty((0,2), np.float32)
        tracker, image = self.tracker([('full',empty,empty,0),('roi',points,points,6)])
        self.assertEqual(tracker.observe(image)['inliers'], 6)
        tracker, image = self.tracker([('full',points,points,6),('roi',points,points+[10,0],6)])
        with self.assertRaisesRegex(ServoError, 'disagree'):
            tracker.observe(image)

    def test_lower_identity_threshold_does_not_allow_control_extrapolation(self):
        points = self.points()
        tracker, image = self.tracker([('full',points,points,6)], point=(25,25))
        with self.assertRaisesRegex(ServoError, 'outside the matched texture'):
            tracker.observe(image)

    def test_invalid_mapped_patch_rejected(self):
        points = self.points()
        tracker, image = self.tracker([('full',points,points+[90,0],6)])
        with self.assertRaisesRegex(ServoError, 'clipped'):
            tracker.observe(image)


if __name__ == '__main__':
    unittest.main()
