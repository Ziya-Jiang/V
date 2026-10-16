"""Offline servo, coordinate convention, target lock, and V1_3 handoff tests."""
import copy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

import cart_book_grasp_v3_servo as servo
import cart_book_grasp_v3_motion as runtime
from cart_book_grasp_v3_tracking import LockedBookTracker
from cart_book_grasp_v1_3_pause import PauseRequested


def config():
    c = servo.load_config(require_reference=False)
    c['reference'] = dict(uv=[300., 240.], surface_gap_mm=200., tool_euler_deg=[0., 0., 0.],
                          device='/dev/test', width=640, height=480,
                          coordinate_system='undistorted_K', calibration_sha256='0'*64)
    return c


class Plant:
    def __init__(self, j=None, error=None):
        self.j = np.array([[-3., .8], [.4, 2.]]) if j is None else np.array(j, float)
        self.error = np.array([16., -11.]) if error is None else np.array(error, float)
        self.q = np.zeros(2)
        self.moves = []
        self.t = 10.
        self.observations = 0
        self.fraction = 1.

    def read(self):
        self.t += .01
        self.observations += 1
        return dict(uv=np.array([300., 240.])+self.error+self.j@self.q,
                    yz_mm=self.q.copy(), captured_at=self.t)

    def move(self, q):
        self.moves.append(np.array(q).copy())
        self.q += self.fraction*(q-self.q)
        return True

    def run(self, c=None, **kwargs):
        return servo.run_servo(self.read, self.move, c or config(), clock=lambda: self.t, **kwargs)


class ServoTests(unittest.TestCase):
    def test_coupled_negative_camera_axes_converge(self):
        p = Plant()
        result = p.run()
        self.assertLessEqual(result['error_px'], 3.)
        self.assertGreater(p.q[0], 0)  # A negative u response requires positive base Y.
        self.assertGreater(p.q[1], 0)
        self.assertLessEqual(np.max(np.linalg.norm(p.moves, axis=1)), 20.)
        self.assertLessEqual(result['path_mm'], 80.)

    def test_actual_positions_not_commanded_probes(self):
        p = Plant(); p.fraction = .9
        events = []
        p.run(log=events.append)
        fitted = next(e['pixels_per_base_mm'] for e in events if e['event'] == 'jacobian')
        np.testing.assert_allclose(fitted, p.j, atol=1e-8)

    def test_already_aligned_needs_fresh_confirmations_not_probes(self):
        p = Plant(error=[0, 0]); p.run()
        self.assertFalse(p.moves)
        self.assertEqual(p.observations, 2)

    def test_singular_and_inconsistent_response_fail(self):
        with self.assertRaisesRegex(servo.ServoError, 'ill-conditioned'):
            Plant(j=[[1, 1], [2, 2]]).run()
        q = [[-2, 0], [2, 0], [0, -2], [0, 2]]
        with self.assertRaisesRegex(servo.ServoError, 'Inconsistent'):
            servo.fit_jacobian(q, [[1, 0], [9, 0], [0, -4], [0, 4]], config())

    def test_stale_and_repeated_frames_cannot_command(self):
        p = Plant()
        def stale():
            obs = p.read(); obs['captured_at'] -= 5; return obs
        with self.assertRaisesRegex(servo.ServoError, 'Stale'):
            servo.run_servo(stale, p.move, config(), clock=lambda: p.t)
        self.assertFalse(p.moves)
        p = Plant(error=[0, 0]); observation = p.read()
        with self.assertRaisesRegex(servo.ServoError, 'Repeated'):
            servo.run_servo(lambda: observation, p.move, config(), clock=lambda: p.t)
        self.assertFalse(p.moves)

    def test_motion_failure_and_pause_do_not_continue(self):
        p = Plant(); failed = Mock(return_value=False)
        with self.assertRaisesRegex(servo.ServoError, 'not confirmed'):
            servo.run_servo(p.read, failed, config(), clock=lambda: p.t)
        failed.assert_called_once()
        p = Plant()
        def checkpoint():
            if p.moves: raise PauseRequested()
        with self.assertRaises(PauseRequested): p.run(checkpoint=checkpoint)
        self.assertEqual(len(p.moves), 1)

    def test_offset_budget_stops_large_image_error(self):
        c = config(); c['max_offset_mm'] = 3
        with self.assertRaisesRegex(servo.ServoError, 'offset limit'):
            Plant(error=[200, 200]).run(c)

    def test_measured_waypoint_error_stops(self):
        p = Plant(); p.fraction = .1
        with self.assertRaisesRegex(servo.ServoError, 'did not reach'):
            p.run()
        self.assertEqual(len(p.moves), 1)

    def test_diverging_response_stops(self):
        p = Plant(); original = p.read
        def read():
            obs = original()
            if len(p.moves) > 8:
                obs['uv'] = np.array([300., 240.])+p.error-p.j@p.q
            return obs
        with self.assertRaisesRegex(servo.ServoError, 'increased twice'):
            servo.run_servo(read, p.move, config(), clock=lambda: p.t)

    def test_analytic_jacobian_matches_camera_motion_projection(self):
        k = np.array([[800., 0, 640], [0, 780., 360], [0, 0, 1.]])
        point = np.array([.08, -.04, .6])
        def project(p): return (k@p)[:2]/p[2]
        j = servo.point_image_jacobian(project(point), k, point[2])
        for axis in range(6):
            velocity = np.eye(6)[axis]
            # Static world point in a moving camera: Pdot = -v - omega x P.
            p_dot = -velocity[:3]-np.cross(velocity[3:], point)
            numerical = (project(point+1e-7*p_dot)-project(point))/1e-7
            np.testing.assert_allclose(j[:, axis], numerical, rtol=1e-5, atol=1e-4)

    def test_reference_missing_rejects_before_robot_import(self):
        import cart_book_grasp_v3 as entry
        with patch('cart_book_grasp_v1_3.main') as main:
            with self.assertRaisesRegex(servo.ServoError, '尚未设置'):
                entry.main(['--check-config'])
            main.assert_not_called()

    def test_reference_registration_and_validation(self):
        import cart_book_grasp_v3 as entry
        metadata = {k: v for k, v in config()['reference'].items()
                    if k not in ('uv', 'surface_gap_mm', 'tool_euler_deg')}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'servo.json'
            path.write_text(json.dumps(servo.load_config(require_reference=False)))
            entry.save_reference(path, [300, 240], 200, [0, 0, 0], metadata)
            self.assertEqual(servo.load_config(path)['reference']['uv'], [300, 240])
            before = path.read_bytes()
            with self.assertRaises(servo.ServoError):
                entry.save_reference(path, [-1, 240], 200, [0, 0, 0], metadata)
            self.assertEqual(path.read_bytes(), before)


class IntegrationTests(unittest.TestCase):
    def test_runtime_sends_base_yz_planner_waypoints_with_fixed_x_rotation(self):
        c = config(); p = Plant()
        robot = SimpleNamespace(get_current_state=lambda: dict(
            joint_state=np.zeros(14), right_arm=dict(position=np.r_[400., p.q]/1000)),
            operator_checkpoint=lambda: None, operator_sleep=lambda _: None)
        step5 = SimpleNamespace(current_tool_orientation_as_euler=lambda *a: [0., 0., 0.])
        tracker = SimpleNamespace(center=np.array([300., 240.]), observe=lambda rgb: dict(uv=np.array([300., 240.])+p.error+p.j@p.q,
            polygon=np.float32([[200, 100], [400, 100], [400, 380], [200, 380]])))
        head_frame = dict(selected=dict(uv=[300., 240.]))
        camera = dict(device='/dev/test', width=640, height=480, enabled=True, max_joint_drift_deg=.5)
        calls = []
        def execute(robot, task, index, objects, **kwargs):
            calls.append(copy.deepcopy(task))
            self.assertEqual(task['target_position'][0], 400.)
            self.assertEqual(task['target_rotation'], [0., 0., 0.])
            self.assertEqual(task['action'], 'book_v3_visual_servo')
            p.move(np.array(task['target_position'][1:]))
            return 'object_0', True
        def capture(*args):
            return dict(rgb=np.zeros((480, 640, 3), np.uint8), captured_at=time.monotonic())
        camera_object = SimpleNamespace(config=camera, check_reference=lambda r: None, capture=capture)
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(runtime, 'acquire_selected_spine', return_value=tracker):
            result = runtime.execute_servo(robot, dict(object='object_0', arm='right', book_grasp_mode='cart+',
                book_surface_x_mm=600), 0, {}, step5, execute, head_frame, c,
                output_root=directory, camera=camera_object)
            self.assertLessEqual(result['error_px'], 3)
            self.assertGreater(len(calls), 8)
            self.assertTrue(list(Path(directory).glob('*/events.jsonl')))

    def test_reference_checks_depth_pose_and_camera(self):
        c = config(); camera = dict(device='/dev/test', width=640, height=480)
        runtime.check_reference(c, camera, [400, 0, 0], [0, 0, 0], 600)
        cases = [(dict(camera, width=848), [400, 0, 0], [0, 0, 0], 600),
                 (camera, [500, 0, 0], [0, 0, 0], 600),
                 (camera, [400, 0, 0], [0, 0, 20], 600),
                 (camera, [550, 0, 0], [0, 0, 0], 600)]
        for args in cases:
            with self.assertRaises(servo.ServoError): runtime.check_reference(c, *args)

    def test_acquisition_transports_selected_point_across_resolutions(self):
        from cart_book_grasp_v3_tracking import acquire_selected_spine
        rng = np.random.default_rng(7)
        head = np.zeros((480, 640, 3), np.uint8)
        head[90:390, 210:390] = rng.integers(0, 256, (300, 180, 3), dtype=np.uint8)
        mask = np.zeros(head.shape[:2], np.uint8); mask[90:390, 210:390] = 1
        transform = np.float32([[1.2, 0, 200], [0, 1.2, 30]])
        wrist = cv2.warpAffine(head, transform, (1280, 720))
        point = np.array([280., 210.])  # deliberately not the mask center
        tracker = acquire_selected_spine(dict(rgb=head, selected=dict(mask=mask, uv=point)), wrist)
        expected = transform @ np.r_[point, 1]
        np.testing.assert_allclose(tracker.center, expected, atol=.7)
        moved = cv2.warpAffine(wrist, np.float32([[1, 0, -8], [0, 1, 5]]), (1280, 720))
        np.testing.assert_allclose(tracker.observe(moved)['uv'], expected+[-8, 5], atol=.7)
        with self.assertRaises(servo.ServoError):
            acquire_selected_spine(dict(rgb=head, selected=dict(mask=mask, uv=point)), np.zeros_like(wrist))

    def test_v13_forward_runs_only_after_servo_success_and_hooks_restore(self):
        import cart_book_grasp_v1_3_motion as v2
        original_forward, original_spine = v2._execute_cart_plus_forward, v2._install_spine_center
        task = dict(object='object_0', book_surface_x_mm=600)
        context = {}; execute = Mock(); order = []
        with patch.object(v2, '_execute_cart_plus_forward', side_effect=lambda *a, **k: order.append('forward') or ('object_0', True)) as forward:
            with patch.object(runtime, 'load_config', return_value=config()), patch.object(runtime, 'execute_servo', side_effect=lambda *a, **k: order.append('servo')) as run:
                with runtime.installed('unused'):
                    result = v2._execute_cart_plus_forward(None, task, 0, {}, None, execute,
                        retreat_context=context, preserve_current_orientation_for_approach=True)
                    self.assertEqual(result, ('object_0', True))
                    self.assertEqual(order, ['servo', 'forward'])
                    self.assertNotIn('retreat_context', run.call_args.kwargs)
                    self.assertIs(forward.call_args.kwargs['retreat_context'], context)
                    run.side_effect = servo.ServoError('target lost')
                    with self.assertRaises(servo.ServoError):
                        v2._execute_cart_plus_forward(None, task, 0, {}, None, execute)
                    self.assertEqual(forward.call_count, 1)
        self.assertIs(v2._execute_cart_plus_forward, original_forward)
        self.assertIs(v2._install_spine_center, original_spine)

    def test_tracker_translation_and_occlusion(self):
        rng = np.random.default_rng(42)
        image = np.zeros((480, 640, 3), np.uint8)
        image[90:390, 240:360] = rng.integers(0, 256, (300, 120, 3), dtype=np.uint8)
        mask = np.zeros(image.shape[:2], bool); mask[90:390, 240:360] = True
        tracker = LockedBookTracker(image, mask)
        moved = cv2.warpAffine(image, np.float32([[1, 0, 12], [0, 1, -8]]), (640, 480))
        observation = tracker.observe(moved)
        np.testing.assert_allclose(observation['uv'], tracker.center+[12, -8], atol=.5)
        with self.assertRaises(servo.ServoError): tracker.observe(np.zeros_like(image))


class CalibrationAndEntryTests(unittest.TestCase):
    def test_rectification_uses_K_and_reference_rejects_changed_calibration(self):
        from cart_book_grasp_v3_camera import WristCamera
        import io, tarfile, yaml
        c = dict(device='/dev/test', width=640, height=480)
        k = np.array([[400., 0, 310.], [0, 410., 250.], [0, 0, 1.]])
        d = np.array([-.28, .06, -.003, .002, 0.])
        doc = dict(image_width=640, image_height=480, distortion_model='plumb_bob',
                   camera_matrix=dict(data=k.ravel().tolist()),
                   distortion_coefficients=dict(data=d.tolist()))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'calibration.tar.gz'
            raw = yaml.safe_dump(doc).encode()
            with tarfile.open(path, 'w:gz') as archive:
                info = tarfile.TarInfo('ost.yaml'); info.size = len(raw)
                archive.addfile(info, io.BytesIO(raw))
            camera = WristCamera(c, path)
            # A rectified pixel must sample its forward-distorted raw location.
            pixel = np.array([440, 300])
            ray = np.linalg.solve(k, [*pixel, 1.])
            projected, _ = cv2.projectPoints(ray.reshape(1, 3), np.zeros(3), np.zeros(3), k, d)
            np.testing.assert_allclose([camera.mapx[pixel[1], pixel[0]], camera.mapy[pixel[1], pixel[0]]],
                                       projected.ravel(), atol=1e-4)
            ref = dict(config()['reference'], calibration_sha256=camera.calibration_hash)
            camera.check_reference(ref)
            with self.assertRaises(servo.ServoError):
                camera.check_reference(dict(ref, calibration_sha256='f'*64))
            with self.assertRaises(servo.ServoError):
                camera.check_reference(dict(ref, coordinate_system='raw'))
            with self.assertRaises(servo.ServoError):
                WristCamera(dict(c, width=1280), path)

    def test_entry_dispatches_v13_with_own_config_and_restores_argv(self):
        import cart_book_grasp_v3 as entry
        import sys
        original = sys.argv
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'servo.json'; path.write_text(json.dumps(config()))
            with patch('cart_book_grasp_v3_camera.WristCamera') as camera, \
                 patch('cart_book_grasp_v1_3.main') as v13, \
                 patch('cart_book_grasp_v3_motion.installed') as installed:
                def run(**kwargs):
                    self.assertIn('--assume-head-camera-calibrated', sys.argv)
                    self.assertEqual(kwargs['config_path'].name, 'cart_book_grasp_v3_config.json')
                    self.assertEqual(kwargs['wrist_config_path'].name, 'cart_book_grasp_v3_wrist_config.json')
                v13.side_effect = run
                entry.main(['--servo-config', str(path), '--assume-head-camera-calibrated'])
                v13.assert_called_once()
                installed.assert_called_once_with(path)
                camera.return_value.capture.assert_not_called()
        self.assertIs(sys.argv, original)

    def test_missing_reference_rejects_before_camera_or_robot(self):
        import cart_book_grasp_v3 as entry
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'servo.json'; path.write_text(json.dumps(dict(config(), reference=None)))
            with patch('cart_book_grasp_v3_camera.WristCamera') as camera, \
                 patch('cart_book_grasp_v1_3.main') as v13:
                with self.assertRaises(servo.ServoError):
                    entry.main(['--servo-config', str(path)])
                camera.assert_not_called(); v13.assert_not_called()

    def test_capture_hook_uses_current_v13_frame_and_clears_failed_cycle(self):
        import cart_book_grasp_v1_3_motion as motion
        geometry = SimpleNamespace()
        head = np.zeros((100, 100, 3), np.uint8)
        selected = dict(uv=[50., 40.], mask=np.ones((100, 100), bool))
        enabled = [True]
        def install(geometry, visualization=None, **kwargs):
            def segment(*args):
                if enabled[0]:
                    visualization.capture(head, None, None, None, selected, None)
                    return selected['mask'], [0, 0, 100, 100]
                return None
            geometry.segment_cart_book_with_sam3 = segment
        with patch.object(motion, '_install_spine_center', side_effect=install), \
             patch.object(motion, '_execute_cart_plus_forward', return_value=('book', True)), \
             patch.object(runtime, 'load_config', return_value=config()), \
             patch.object(runtime, 'execute_servo') as run:
            with runtime.installed('unused'):
                motion._install_spine_center(geometry)
                geometry.segment_cart_book_with_sam3()
                motion._execute_cart_plus_forward(None, {}, 0, {}, None, None)
                frame = run.call_args.args[6]
                np.testing.assert_array_equal(frame['rgb'], head)
                self.assertEqual(frame['selected']['uv'], selected['uv'])
                enabled[0] = False
                geometry.segment_cart_book_with_sam3()
                motion._execute_cart_plus_forward(None, {}, 0, {}, None, None)
                self.assertIsNone(run.call_args.args[6])

    def test_timeout_and_tracking_failure_send_no_motion(self):
        p = Plant()
        def lost(): raise servo.ServoError('lost target')
        with self.assertRaises(servo.ServoError):
            servo.run_servo(lost, p.move, config(), clock=lambda: p.t)
        self.assertEqual(p.moves, [])
        def slow():
            p.t += 200
            return p.read()
        with self.assertRaisesRegex(servo.ServoError, 'time budget'):
            servo.run_servo(slow, p.move, config(), clock=lambda: p.t)
        self.assertEqual(p.moves, [])


if __name__ == '__main__':
    unittest.main()
