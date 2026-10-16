"""Offline synthetic tests. Never import/connect robot or camera drivers."""
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from book_top_tilt_perception import FaceTracker, measure_face
from cart_book_grasp_v4_1 import CONFIG, TiltController, load_config, main, RobotIO, Vision


def configuration():
    c = load_config()
    # Synthetic fixtures, deliberately not written to real configuration.
    c.update(pad_visual_required=False, pad_calibrated=True, scene_setup_confirmed=True, calibrated_gripper_position=0.,
             pad=dict(center_tcp_mm=[0, 0, 50], normal_tcp=[0, 0, -1],
                      along_tcp=[1, 0, 0], length_mm=60, width_mm=12, edge_margin_mm=2))
    return c


def observation(top=(600, 0, 200), bottom=(600, 0, 0), stamp=1):
    return dict(top_base_mm=np.array(top, float), bottom_base_mm=np.array(bottom, float),
                axis_base=np.array([0, -1, 0.]), captured_unix_s=stamp)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.c = configuration()
        self.tcp = np.array([600., 0., 158.])
        self.ctrl = TiltController(self.c, observation(), self.tcp, np.eye(3))

    def contact_trial(self):
        self.ctrl.phase = 'APPROACH'
        self.tcp = np.array([600., 0., 150.5])
        return self.ctrl.decide(observation(), self.tcp, np.eye(3))

    def test_unmeasured_manual_config_cannot_execute(self):
        import tempfile, json
        from pathlib import Path
        c=load_config();c['pad_source']='measured'
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'config.json';p.write_text(json.dumps(c))
            with self.assertRaisesRegex(ValueError, 'calibration'):load_config(p, True)

    def test_cli_validation_no_hardware(self):
        with patch('cart_book_grasp_v4_1.run', side_effect=AssertionError('I/O')):
            self.assertEqual(main(['--check-config']), 0)
        # Auto mode defers actual geometry to fresh RGB-D; never test it by
        # opening devices. Manual uncalibrated execution is tested above.

    def test_alignment_is_horizontal_above_book(self):
        tcp = self.tcp + [10, 0, 0]
        ctrl = TiltController(self.c, observation(), tcp, np.eye(3))
        d = ctrl.decide(observation(), tcp, np.eye(3))
        self.assertEqual(d['phase'], 'ALIGN')
        self.assertEqual(d['target'][2], tcp[2])
        self.assertAlmostEqual(np.linalg.norm(d['target']-tcp), 2)

    def test_descent_is_bounded_and_no_force_claim(self):
        d = self.ctrl.decide(observation(), self.tcp, np.eye(3))
        self.assertEqual(d['phase'], 'APPROACH')
        np.testing.assert_allclose(d['target'], [600, 0, 156])
        self.assertFalse(d['contact_confirmed'])

    def test_first_trial_moves_towards_robot(self):
        d = self.contact_trial()
        self.assertEqual(d['phase'], 'PROBE')
        self.assertLess(d['target'][0], self.tcp[0])
        self.assertLess(d['target'][2], self.tcp[2])
        self.assertLessEqual(np.linalg.norm(d['target']-self.tcp), 2)

    def test_no_book_response_stops_without_second_target(self):
        d = self.contact_trial()
        with self.assertRaisesRegex(ValueError, 'No measured book response'):
            self.ctrl.decide(observation(stamp=2), d['target'], np.eye(3))

    def test_pivot_slip_stops(self):
        with self.assertRaisesRegex(ValueError, 'support edge moved'):
            self.ctrl.decide(observation(bottom=(604, 0, 0)), self.tcp, np.eye(3))

    def test_repeated_frame_rejected(self):
        self.ctrl.decide(observation(), self.tcp, np.eye(3))
        with self.assertRaisesRegex(ValueError, 'did not advance'):
            self.ctrl.decide(observation(), self.tcp, np.eye(3))

    def test_horizontal_orientation_required(self):
        from scipy.spatial.transform import Rotation
        with self.assertRaisesRegex(ValueError, 'already be horizontal'):
            TiltController(self.c, observation(), self.tcp, Rotation.from_euler('x', 10, degrees=True).as_matrix())

    def test_start_must_be_above_book(self):
        with self.assertRaisesRegex(ValueError, 'Start above'):
            TiltController(self.c, observation(), [600, 0, 148], np.eye(3))

    def test_pad_departure_stops(self):
        self.contact_trial()
        with self.assertRaisesRegex(ValueError, 'pad region'):
            self.ctrl.decide(observation(stamp=2), self.tcp+[0, 10, 0], np.eye(3))

    def test_path_budget_exact_boundary_allowed(self):
        self.ctrl.path_mm = self.c['max_path_mm']-2
        self.ctrl.command(self.tcp+[2, 0, 0], self.tcp, 0)
        self.ctrl.path_mm += .01
        with self.assertRaisesRegex(ValueError, 'path budget'):
            self.ctrl.command(self.tcp+[2, 0, 0], self.tcp, 0)

    def test_closed_loop_simulated_book_reaches_angle_and_keeps_tool_horizontal(self):
        obs = observation()
        tcp = self.tcp.copy()
        reached = False
        for i in range(250):
            obs['captured_unix_s'] = i+1
            d = self.ctrl.decide(obs, tcp, np.eye(3))
            if d['phase'] == 'HOLD':
                reached = True
                self.assertGreaterEqual(d['angle_deg'], 44)
                break
            if d['target'] is None:
                continue
            tcp = d['target'].copy()
            if d['phase'] in ('PROBE', 'TILT'):
                # Ideal synthetic no-slip book; not a physics/contact validation.
                top = tcp + [0, 0, 50] + self.ctrl.normal*self.c['contact_gap_mm']
                obs['top_base_mm'] = top
        self.assertTrue(reached)


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[300., 0, 160], [0, 300, 120], [0, 0, 1]])
        self.d = np.zeros(5)
        self.q = np.array([[100, 50], [220, 50], [220, 210], [100, 210]], np.float32)
        self.depth = np.full((240, 320), 600., np.float32)

    def test_measured_depth_reconstructs_edges(self):
        result = measure_face(self.q, self.depth, self.k, self.d, np.eye(4))
        np.testing.assert_allclose(result['top_base_mm'], [0, -140, 600], atol=1e-6)
        np.testing.assert_allclose(result['bottom_base_mm'], [0, 180, 600], atol=1e-6)

    def test_interior_ignores_edge_background(self):
        mask = np.zeros(self.depth.shape, np.uint8)
        cv2.fillConvexPoly(mask, np.int32(self.q.mean(0)+.75*(self.q-self.q.mean(0))), 1)
        self.depth[mask == 0] = 1500
        result = measure_face(self.q, self.depth, self.k, self.d, np.eye(4))
        self.assertAlmostEqual(result['top_base_mm'][2], 600.)

    def test_mixed_background_depth_is_not_a_book_plane(self):
        rng = np.random.default_rng(2)
        self.depth = rng.uniform(500, 900, (240, 320)).astype(np.float32)
        with self.assertRaisesRegex(ValueError, 'dominant plane'):
            measure_face(self.q, self.depth, self.k, self.d, np.eye(4))

    def test_depth_holes_rejected(self):
        with self.assertRaisesRegex(ValueError, 'insufficient'):
            measure_face(self.q, np.zeros_like(self.depth), self.k, self.d, np.eye(4))

    def test_face_tracking_translation_preserves_identity(self):
        rng = np.random.default_rng(4)
        image = rng.integers(0, 256, (240, 320, 3), dtype=np.uint8)
        tracker = FaceTracker(image, self.q)
        shifted = cv2.warpAffine(image, np.float32([[1, 0, 2], [0, 1, 1]]), (320, 240))
        q = tracker.update(shifted)
        np.testing.assert_allclose(q, self.q+[2, 1], atol=.2)

    def test_textureless_book_rejected(self):
        with self.assertRaisesRegex(ValueError, 'little texture'):
            FaceTracker(np.zeros((240, 320, 3), np.uint8), self.q)

    def test_perspective_transports_fixed_midpoint_not_corner_average(self):
        rng = np.random.default_rng(9)
        image = rng.integers(0, 256, (240, 320, 3), dtype=np.uint8)
        # Band-limited texture permits same-template SIFT recovery at BOTH
        # semantic edges. Unfiltered white noise loses bottom-edge support
        # under resampling and must no longer pass through long extrapolation.
        image = cv2.GaussianBlur(image, (3, 3), 0)
        tracker = FaceTracker(image, self.q)
        h = np.array([[1., 0., 0.], [0., 1., 0.], [.0004, 0., 1.]])
        warped = cv2.warpPerspective(image, h, (320, 240))
        tracker.update(warped)
        expected = cv2.perspectiveTransform(tracker.reference_midpoints[None], h)[0]
        # LK resampling of synthetic high-frequency texture has subpixel
        # interpolation error; the runtime RANSAC threshold remains 1.5 px.
        np.testing.assert_allclose(tracker.edge_midpoints, expected, atol=.5)
        self.assertGreater(np.linalg.norm(expected[0]-(tracker.quad[0]+tracker.quad[1])/2), .5)

    def test_pre_settle_frames_do_not_enter_decision_window(self):
        vision = Vision.__new__(Vision)
        vision.c = configuration()
        # Old frames can contain motion; they must not poison a stationary
        # three-frame window or be paired with the later robot pose.
        samples = iter([observation(top=(900, 0, 200), stamp=1),
                        observation(stamp=2), observation(stamp=3), observation(stamp=4)])
        vision.observe = lambda: next(samples)
        result = vision.stable(min_stamp=2)
        self.assertEqual(result['captured_unix_s'], 4)



if __name__ == '__main__':
    unittest.main()
