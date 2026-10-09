"""Offline servo, coordinate convention, target lock, and V2 handoff tests."""
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
from cart_book_grasp_v2_pause import PauseRequested


def config():
    c = servo.load_config(require_reference=False)
    c['reference'] = dict(uv=[300., 240.], surface_gap_mm=200., tool_euler_deg=[0., 0., 0.],
                          device='/dev/test', width=640, height=480)
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
        with patch('cart_book_grasp_v2.main') as main:
            with self.assertRaisesRegex(servo.ServoError, '尚未设置'):
                entry.main(['--check-config'])
            main.assert_not_called()

    def test_reference_registration_and_validation(self):
        import cart_book_grasp_v3 as entry
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'servo.json'
            path.write_text(json.dumps(servo.load_config(require_reference=False)))
            entry.main(['--servo-config', str(path), '--set-reference', '300', '240',
                        '--surface-gap-mm', '200', '--tool-euler-deg', '0', '0', '0'])
            self.assertEqual(servo.load_config(path)['reference']['uv'], [300, 240])
            before = path.read_bytes()
            with self.assertRaises(servo.ServoError):
                entry.main(['--servo-config', str(path), '--set-reference', '-1', '240',
                            '--surface-gap-mm', '200', '--tool-euler-deg', '0', '0', '0'])
            self.assertEqual(path.read_bytes(), before)


class IntegrationTests(unittest.TestCase):
    def test_runtime_sends_base_yz_planner_waypoints_with_fixed_x_rotation(self):
        c = config(); p = Plant()
        robot = SimpleNamespace(get_current_state=lambda: dict(
            joint_state=np.zeros(14), right_arm=dict(position=np.r_[400., p.q]/1000)),
            operator_checkpoint=lambda: None, operator_sleep=lambda _: None)
        step5 = SimpleNamespace(current_tool_orientation_as_euler=lambda *a: [0., 0., 0.])
        tracker = SimpleNamespace(observe=lambda rgb: dict(uv=np.array([300., 240.])+p.error+p.j@p.q,
            polygon=np.float32([[200, 100], [400, 100], [400, 380], [200, 380]])))
        selector = SimpleNamespace(last_decision={'selected_id': 'book_2'})
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
        with tempfile.TemporaryDirectory() as directory, \
                patch('cart_book_grasp_v2_dual_view.load_config', return_value=camera), \
                patch('cart_book_grasp_v2_dual_view.capture_wrist', side_effect=capture), \
                patch.object(runtime, 'selected_tracker', return_value=tracker):
            result = runtime.execute_servo(robot, dict(object='object_0', arm='right', book_grasp_mode='cart+',
                book_surface_x_mm=600), 0, {}, step5, execute, selector, c, output_root=directory)
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

    def test_cross_view_uses_matched_id_not_position_or_id_equality(self):
        image = np.zeros((10, 10, 3), np.uint8)
        mask1, mask2 = np.zeros((10, 10), bool), np.ones((10, 10), bool)
        selector = SimpleNamespace(last_decision=dict(status='selected', selected_id='book_1',
            cross_view_matches=[dict(head_id='book_1', wrist_id='wrist_2', status='matched', confidence=.99)]),
            last_wrist_view=dict(rgb=image, min_match_confidence=.85, candidates=[
                dict(id='wrist_1', book_mask=mask1, verified_book=True),
                dict(id='wrist_2', book_mask=mask2, verified_book=True)]))
        with patch.object(runtime, 'LockedBookTracker') as tracker:
            runtime.selected_tracker(selector)
            self.assertIs(tracker.call_args.args[1], mask2)
            selector.last_decision['cross_view_matches'][0]['confidence'] = .3
            with self.assertRaises(servo.ServoError): runtime.selected_tracker(selector)
            self.assertEqual(tracker.call_count, 1)

    def test_v2_forward_runs_only_after_servo_success_and_hooks_restore(self):
        import cart_book_grasp_v2_motion as v2
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


if __name__ == '__main__':
    unittest.main()
