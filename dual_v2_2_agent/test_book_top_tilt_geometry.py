import unittest
import numpy as np
from book_top_tilt_geometry import TiltReference

class TiltGeometryTests(unittest.TestCase):
    def setUp(self):
        self.ref=TiltReference([600,0,0],[0,-1,0],[600,0,200],[10,0,50],np.eye(3))
    def test_45_degree_book_tip_moves_forward_and_down(self):
        goal=self.ref.angle_towards([-1,0,0]);self.assertEqual(goal,45.)
        out=self.ref.target(goal)
        np.testing.assert_allclose(out['tip_base_mm'],[600-200/np.sqrt(2),0,200/np.sqrt(2)])
    def test_tcp_accounts_for_single_finger_offset(self):
        for angle in (0,1,15,45):
            out=self.ref.target(angle)
            np.testing.assert_allclose(out['tcp_base_mm']+out['tool_rotation']@self.ref.tip_offset_tcp_mm,out['tip_base_mm'])
    def test_horizontal_finger_orientation_stays_fixed(self):
        for angle in (0,15,45):
            np.testing.assert_allclose(self.ref.target(angle)['tool_rotation'],np.eye(3))
    def test_explicit_corotation_is_separate(self):
        self.ref.orientation_mode='co_rotate'
        self.assertFalse(np.allclose(self.ref.target(45)['tool_rotation'],np.eye(3)))
    def test_pivot_radius_preserved(self):
        for angle in np.linspace(-45,45,30):
            self.assertAlmostEqual(np.linalg.norm(self.ref.target(angle)['tip_base_mm']-self.ref.pivot_base_mm),200.)
    def test_estimate_uses_book_points(self):
        for angle in (-30,0,45):
            observed=self.ref.target(angle)['tip_base_mm']
            self.assertAlmostEqual(self.ref.measured_angle(observed,self.ref.pivot_base_mm),angle)
    def test_no_observed_progress_does_not_accumulate_command(self):
        a=self.ref.incremental_target(0,45,max_tip_step_mm=2.,max_angle_step_deg=1.)
        b=self.ref.incremental_target(0,45,max_tip_step_mm=2.,max_angle_step_deg=1.)
        np.testing.assert_allclose(a['tcp_base_mm'],b['tcp_base_mm'])
        self.assertLessEqual(np.linalg.norm(a['tip_base_mm']-self.ref.contact_base_mm),2.+1e-9)
    def test_goal_not_overshot(self):
        out=self.ref.incremental_target(44.9,45,max_tip_step_mm=2.,max_angle_step_deg=1.)
        self.assertAlmostEqual(out['angle_deg'],45.)
    def test_invalid_axis_offset_and_rotation_rejected(self):
        with self.assertRaises(ValueError):TiltReference([0,0,0],[0,0,0],[0,0,200],[0,0,0],np.eye(3))
        with self.assertRaises(ValueError):TiltReference([0,0,0],[0,1,0],[0,0,200],[np.nan,0,0],np.eye(3))
        with self.assertRaises(ValueError):TiltReference([0,0,0],[0,1,0],[0,0,200],[0,0,0],np.zeros((3,3)))


class FingerPadTests(unittest.TestCase):
    def setUp(self):
        from book_top_tilt_geometry import FingerPad
        self.pad=FingerPad([0,0,50],[0,0,-1],[1,0,0],60,12,2)
    def test_contact_region_is_not_a_single_point(self):
        for x in (-20,0,20):
            out=self.pad.observe_book_point([x,0,50],[0,0,0],np.eye(3))
            self.assertTrue(out['inside_pad']);self.assertEqual(out['signed_gap_mm'],0.)
            self.assertFalse(out['contact_confirmed'])
    def test_positive_gap_is_on_book_side(self):
        out=self.pad.observe_book_point([0,0,48],[0,0,0],np.eye(3))
        self.assertEqual(out['signed_gap_mm'],2.)
    def test_edge_departure_is_visible(self):
        self.assertFalse(self.pad.observe_book_point([29,0,50],[0,0,0],np.eye(3))['inside_pad'])
    def test_world_frame_rotation_preserves_local_gap(self):
        from scipy.spatial.transform import Rotation
        rotation=Rotation.from_euler('xyz',[15,30,45],degrees=True).as_matrix()
        tcp=np.array([100,200,300]);point=tcp+rotation@np.array([10,0,47])
        out=self.pad.observe_book_point(point,tcp,rotation)
        self.assertAlmostEqual(out['signed_gap_mm'],3.);self.assertAlmostEqual(out['along_mm'],10.)

if __name__=='__main__':unittest.main()
