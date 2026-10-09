"""Offline regression tests. No camera, planner, or motor commands."""
import copy
import ast
from dataclasses import replace
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

import book_grap_v2 as v2


ROTATION = np.array([[0., 0., -1.], [1., 0., 0.], [0., -1., 0.]])


def book_cloud(width=30):
    x, y, z = np.meshgrid(np.linspace(500, 700, 31),
                          np.linspace(-165-width/2, -165+width/2, 9),
                          np.linspace(-180, -80, 17), indexing='ij')
    return np.column_stack([x.ravel(), y.ravel(), z.ravel()])


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.geometry = v2.estimate_geometry(book_cloud())

    def test_fingertip_stops_at_front_plus_insertion(self):
        plan = v2.plan_approach(self.geometry, [350, -230, -130], ROTATION, v2.Settings())
        self.assertAlmostEqual(plan['grasp'][0] + 50, plan['front_projection_mm'] + 10)
        self.assertAlmostEqual(plan['pregrasp'][0] + 50, plan['front_projection_mm'] - 40)
        self.assertAlmostEqual(plan['grasp'][1], -165)
        self.assertAlmostEqual(plan['grasp'][2], -130)
        self.assertNotAlmostEqual(plan['grasp'][0], 550)  # Old fixed +200 mm.
        points = np.array([plan['pregrasp'], *plan['approach_steps']])
        self.assertTrue(np.all(np.linalg.norm(np.diff(points, axis=0), axis=1) <= 5.00001))

    def test_distance_tracks_book_translation(self):
        original = v2.plan_approach(self.geometry, [350, -230, -130], ROTATION, v2.Settings())
        moved = v2.estimate_geometry(book_cloud() + [35, 0, 0])
        updated = v2.plan_approach(moved, [350, -230, -130], ROTATION, v2.Settings())
        self.assertAlmostEqual(updated['grasp'][0] - original['grasp'][0], 35)

    def test_mid_bounds_do_not_follow_surface_density(self):
        cloud = book_cloud()
        biased = np.concatenate([cloud, np.repeat(cloud[cloud[:, 1] == -180], 20, axis=0)])
        geometry = v2.estimate_geometry(biased)
        self.assertLess(abs(geometry['center_base_mm'][1] + 165), 2)
        self.assertGreater(abs(np.median(biased[:, 1]) + 165), 10)

    def test_reject_wide_book(self):
        geometry = v2.estimate_geometry(book_cloud(50))
        with self.assertRaisesRegex(ValueError, '超过夹爪'):
            v2.plan_approach(geometry, [350, -230, -130], ROTATION, v2.Settings())

    def test_ready_pose_yaw_tolerance_preserves_width_checks(self):
        def yaw_rotation(degrees):
            c, s = np.cos(np.deg2rad(degrees)), np.sin(np.deg2rad(degrees))
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ ROTATION

        # Regression: a small 11.8-degree mismatch must not hit the old 10-degree gate.
        plan = v2.plan_approach(self.geometry, [350, -230, -130], yaw_rotation(11.8), v2.Settings())
        self.assertAlmostEqual(plan['yaw_error_deg'], 11.8)
        self.assertGreaterEqual(plan['side_clearance_mm'], 8)
        with self.assertRaisesRegex(ValueError, '超过 15'):
            v2.plan_approach(self.geometry, [350, -230, -130], yaw_rotation(16), v2.Settings())
        with self.assertRaisesRegex(ValueError, '超过夹爪'):
            v2.plan_approach(v2.estimate_geometry(book_cloud(50)), [350, -230, -130],
                             yaw_rotation(11.8), v2.Settings())

    def test_reject_unknown_front_or_bad_pose(self):
        for pose, rotation in [([350, -230, 0], ROTATION),
                               ([470, -230, -130], ROTATION),
                               ([350, -230, -130], np.eye(3)),
                               ([np.nan, -230, -130], ROTATION)]:
            with self.subTest(pose=pose):
                with self.assertRaises(ValueError):
                    v2.plan_approach(self.geometry, pose, rotation, v2.Settings())
        one_cover = copy.deepcopy(self.geometry)
        one_cover['points_base_mm'] = book_cloud()[book_cloud()[:, 1] == -180]
        with self.assertRaisesRegex(ValueError, '局部表面'):
            v2.plan_approach(one_cover, [350, -230, -130], ROTATION, v2.Settings())

    def test_reject_small_false_detection(self):
        with self.assertRaises(ValueError):
            v2.estimate_geometry(book_cloud() * 0.1)

    def test_tasks_finish_approach_before_closing(self):
        plan = v2.plan_approach(self.geometry, [350, -230, -130], ROTATION, v2.Settings())
        tasks = v2.build_tasks(plan, [90, 0, 90], 'object_0')
        close = next(i for i, task in enumerate(tasks) if task['action'] == 'close_gripper')
        self.assertEqual(tasks[close-1]['target_position'], plan['grasp'])
        self.assertEqual([t['action'] for t in tasks[close:]],
                         ['close_gripper', 'lift_object', 'open_gripper', 'first_reset'])

    def test_cli_defaults_and_nonfinite_values(self):
        options, rest = v2.parse_options(['--assume-head-camera-calibrated'])
        self.assertEqual(options.fingertip_offset_mm, 50)
        self.assertEqual(options.open_gap_mm, 60)
        self.assertEqual(options.min_score, 0.20)
        self.assertEqual(rest, ['--assume-head-camera-calibrated'])
        with self.assertRaises(SystemExit):
            v2.parse_options(['--book-insertion-mm', 'nan'])


class IntegrationTests(unittest.TestCase):
    def install(self, preview=False, open_success=True):
        events = []
        agent = types.SimpleNamespace(
            grounding_main=lambda *a, **k: None,
            planning_main=lambda *a, **k: [],
            execute_main=lambda tasks, *a, **k: events.append(('execute', tasks)) or ('object_0', True))
        book = types.SimpleNamespace(install_maximum_pregrasp_height=lambda: None)
        geom = types.ModuleType('book_grasp')
        geom.BOOK_GRASP_MODE = 'cart+'
        geom.fill_book_meta = lambda *a: None
        geom._component_labels = lambda points, **k: np.zeros(len(points), dtype=int)
        geom._camera_to_base = lambda pts, c2w: (pts + [0, 0, -1], np.eye(3), np.array([0, 0, -1.]))

        def fill(meta, *args):
            for key in ('centroid', 'bbox_plane_x1', 'bbox_plane_x2', 'bbox_plane_y1',
                        'bbox_plane_y2', 'bbox_plane_z1', 'bbox_plane_z2'):
                meta[key] = np.eye(4)
            meta['grasp_0'] = meta['centroid']
            return {}

        geom._fill_mask_depth_center = fill
        step = types.ModuleType('step5_execute')
        step.execute_single_task = lambda *a, **k: (None, True)
        step.fully_open_gripper_before_pick = lambda *a: events.append(('open', None)) or open_success
        transform = types.ModuleType('utils.transform_utils')
        transform.angle2rotation = lambda *a: ROTATION
        robot = types.SimpleNamespace(
            control_joint_position=lambda **k: events.append(('reset', None)) or True,
            get_current_state=lambda: {'right_arm': {'position': [.35, -.23, -.13], 'orientation': [1, 0, 0, 0]}},
            _convert_quaternion_to_pos_rot=lambda p: [0, 0, 0, 90, 0, 90])
        with patch.dict('sys.modules', {'book_grasp': geom, 'step5_execute': step,
                                        'utils.transform_utils': transform}):
            v2.install_v2(agent, book, replace(v2.Settings(), preview_only=preview))
        agent.test_step = step
        return agent, geom, robot, events

    def run_pipeline(self, preview=False, open_success=True):
        agent, geom, robot, events = self.install(preview, open_success)
        with tempfile.TemporaryDirectory() as output:
            agent.grounding_main('prompt', output, 'debug', [], robot)
            meta = {}
            geom.fill_book_meta(meta, book_cloud()/1000 + [0, 0, 1], np.eye(4))
            # The aliased grasp and centroid must not receive the offset twice.
            np.testing.assert_allclose(meta['centroid'][:3, 3], [.6, -.165, .87], atol=.002)
            tasks = [{'object': 'object_0', 'book_grasp_mode': 'cart+'}]
            result = agent.execute_main(tasks, {'object_0': 'book'}, robot)
            self.assertTrue((v2.Path(output) / 'book_grap_v2_plan.json').is_file())
        return result, events

    def test_preview_never_sends_reset_gripper_or_motion(self):
        result, events = self.run_pipeline(preview=True)
        self.assertFalse(result[1])
        self.assertEqual(events, [])

    def test_open_precedes_motion_and_failure_cancels(self):
        result, events = self.run_pipeline()
        self.assertTrue(result[1])
        self.assertEqual([e[0] for e in events], ['reset', 'open', 'execute'])
        result, events = self.run_pipeline(open_success=False)
        self.assertFalse(result[1])
        self.assertEqual([e[0] for e in events], ['reset', 'open'])

    def test_new_capture_clears_old_geometry(self):
        agent, geom, robot, events = self.install(preview=True)
        geom.fill_book_meta({}, book_cloud()/1000 + [0, 0, 1], np.eye(4))
        agent.grounding_main('prompt', '.', 'debug', [], robot)
        result = agent.execute_main([{'object': 'object_0', 'book_grasp_mode': 'cart+'}], {}, robot)
        self.assertFalse(result[1])
        self.assertEqual(events, [])

    def test_failure_and_operator_recovery_only_reset_right_arm(self):
        for operator in (False, True):
            agent, _, robot, events = self.install()
            resets, opens = [], []
            robot.control_joint_position = lambda **k: resets.append(k) or True
            agent.test_step.fully_open_gripper_before_pick = lambda *a: opens.append(a[1]) or True
            self.assertTrue(agent.test_step.recover_to_reset_after_failure(
                robot, {'object': 'object_0'}, operator_requested=operator))
            self.assertEqual(opens, ['right'])
            self.assertEqual(len(resets), 1)
            self.assertIs(resets[0]['left_pos'], False)
            self.assertIs(resets[0]['right_pos'], True)

    def test_recovery_cancels_reset_if_gripper_open_fails(self):
        agent, _, robot, events = self.install(open_success=False)
        self.assertFalse(agent.test_step.recover_to_reset_after_failure(robot, {}))
        self.assertEqual([e[0] for e in events], ['open'])

    def test_executor_rejects_left_arm_tasks(self):
        agent, _, robot, _ = self.install()
        self.assertEqual(agent.test_step.execute_single_task(
            robot, {'arm': 'left', 'action': 'move_above_grasp', 'object': 'object_0'}, 0, {}),
            ('object_0', False))


class ArmPacketTests(unittest.TestCase):
    """Execute the real method AST without loading GPU/robot dependencies."""
    @staticmethod
    def master_control():
        path = v2.Path(__file__).parent / 'RobotRAM/controllor/hkstp_arm.py'
        tree = ast.parse(path.read_text())
        robot_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RobotArm')
        method = next(n for n in robot_class.body if isinstance(n, ast.FunctionDef) and n.name == 'master_control')
        module = ast.Module(body=[method], type_ignores=[])
        namespace = {'np': np, 'TargetCommandArmCommandType': types.SimpleNamespace(CARTESIAN='cartesian')}
        exec(compile(module, str(path), 'exec'), namespace)
        return namespace['master_control']

    def test_gripper_only_preserves_joint_or_cartesian_targets_and_identity(self):
        for kind in ('joint', 'cartesian'):
            command = types.SimpleNamespace(
                arm_command_type=kind, description='first_reset:joint_reset:123',
                arm_joint_command=np.arange(14).reshape(2, 7).tolist(),
                arm_joint_size=[7, 7], arm_tool_command=np.arange(14).reshape(2, 7),
                gripper_size=2, gripper_mode_command=[7, 1])
            before = copy.deepcopy(command)
            sent = []
            robot = types.SimpleNamespace(
                target_command=command, udp=types.SimpleNamespace(send=lambda x: sent.append(copy.deepcopy(x))),
                ik_lib='curobo', _target_position_is_reasonable=lambda *a: True,
                get_cat_gripper=lambda status, category: 0 if status == 'open' else 1)
            # get_current_state and _set_arm_joint_command_placeholders are intentionally absent.
            self.assertTrue(self.master_control()(robot, right_gripper=0, action='pregrasp_full_open', object_category='book'))
            self.assertEqual(len(sent), 1)
            self.assertEqual(command.arm_command_type, before.arm_command_type)
            self.assertEqual(command.description, before.description)
            self.assertEqual(command.arm_joint_command, before.arm_joint_command)
            np.testing.assert_array_equal(command.arm_tool_command, before.arm_tool_command)
            self.assertEqual(command.gripper_mode_command, [7, 0])


class PlannerArmSelectionTests(unittest.TestCase):
    def method(self, name):
        path = v2.Path(__file__).parent.parent / 'dual_v2_2_control/cuarm_upper_planner/scripts/curobo_planner.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CuroboPlanner')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
        namespace = {
            'same_array': lambda a, b, atol: bool(np.allclose(a, b, atol=atol, rtol=0)),
            'same_quaternion': lambda a, b, atol: bool(np.allclose(a, b, atol=atol, rtol=0)
                                                    or np.allclose(a, -b, atol=atol, rtol=0)),
        }
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        return namespace[name]

    def test_first_cartesian_command_holds_left_measured_pose(self):
        planner = types.SimpleNamespace(arm_size=2, target_cartesian_orientation_precision=.01,
                                        _prev_target_tool_position=None, _prev_target_tool_orientation=None)
        measured = np.array([[.4, .2, .1], [.4, -.2, .1]])
        target = measured.copy()
        target[1, 0] += .1
        orientation = np.array([[1., 0, 0, 0], [1., 0, 0, 0]])
        method = self.method('_update_move_arm_cartesian')
        method(planner, target, orientation, measured, orientation)
        self.assertEqual(planner._move_arm, [False, True])
        # Intentionally changing both goals must still allow both arms.
        target[0, 0] += .1
        method(planner, target, orientation, measured, orientation)
        self.assertEqual(planner._move_arm, [True, True])
        target = measured.copy()
        rotated = orientation.copy()
        rotated[1] = [np.cos(.2), 0, 0, np.sin(.2)]
        method(planner, target, rotated, measured, orientation)
        self.assertEqual(planner._move_arm, [False, True])

    def test_first_right_joint_reset_holds_left_joints(self):
        planner = types.SimpleNamespace(arm_size=2, arm_joint_size=[7, 7],
                                        arm_joint_position=np.linspace(0, 1, 14),
                                        target_joint_position_precision=1e-4, _prev_target_joint=None)
        target = planner.arm_joint_position.copy()
        target[7:] += .1
        self.method('_update_move_arm_joint')(planner, target)
        self.assertEqual(planner._move_arm, [False, True])


class SegmentationTests(unittest.TestCase):
    def detector(self, detections):
        return types.SimpleNamespace(
            _load_sam3_detector=lambda: (None, None, types.SimpleNamespace(fromarray=lambda x: x)),
            _sam3_raw_books=lambda *a: detections(a[-1]),
            _mask_median_depth_m=lambda mask, depth: float(np.median(depth[mask])) / 1000,
        )

    def test_reject_low_confidence_nearby_false_book_and_refine_roi(self):
        rgb = np.zeros((120, 160, 3), dtype=np.uint8)
        depth = np.full((120, 160), 800.)
        main = np.zeros((120, 160), dtype=bool)
        main[30:100, 65:95] = True
        false = np.zeros_like(main)
        false[100:119, 0:20] = True
        depth[false] = 500

        def detections(image):
            if image.shape[:2] == main.shape:
                return [(0.13, false, None), (0.6, main, None)]
            local = np.zeros(image.shape[:2], dtype=bool)
            # The 24-pixel padded ROI starts at (41, 6).
            local[24:94, 25:53] = True
            return [(0.7, local, None)]

        geom = self.detector(detections)
        mask, box = v2.refine_segmentation(geom, rgb, depth, v2.Settings())
        self.assertEqual(box.tolist(), [66, 30, 94, 100])
        self.assertFalse(np.any(mask & false))
        self.assertIn('ROI匹配', geom.BOOK_SELECTION_NOTE)

    def test_component_cleanup_and_unmatched_crop_fallback(self):
        rgb = np.zeros((120, 160, 3), dtype=np.uint8)
        main = np.zeros((120, 160), dtype=bool)
        main[30:100, 65:95] = True
        noisy = main.copy()
        noisy[0:3, 0:3] = True
        geom = self.detector(lambda image: [(0.5, noisy, None)] if image.shape[:2] == main.shape else [])
        mask, box = v2.refine_segmentation(geom, rgb, np.full(main.shape, 800), v2.Settings())
        np.testing.assert_array_equal(mask, main)
        self.assertIn('保留原mask', geom.BOOK_SELECTION_NOTE)

    def test_accept_recorded_book_score_below_old_threshold(self):
        rgb = np.zeros((120, 160, 3), dtype=np.uint8)
        mask = np.zeros((120, 160), dtype=bool)
        mask[30:100, 65:95] = True
        geom = self.detector(lambda image: [(0.26, mask, None)] if image.shape[:2] == mask.shape else [])
        result = v2.refine_segmentation(geom, rgb, np.full(mask.shape, 723), v2.Settings())
        self.assertIsNotNone(result)
        result = v2.refine_segmentation(geom, rgb, np.full(mask.shape, 723), replace(v2.Settings(), min_score=.30))
        self.assertIsNone(result)
        self.assertIn('score=0.260', geom.BOOK_SELECTION_NOTE)
        self.assertIn('低于阈值', geom.BOOK_SELECTION_NOTE)

    def test_depth_and_empty_detection_have_distinct_diagnostics(self):
        rgb = np.zeros((120, 160, 3), dtype=np.uint8)
        mask = np.zeros((120, 160), dtype=bool)
        mask[30:100, 65:95] = True
        geom = self.detector(lambda image: [(0.5, mask, None)])
        geom._mask_median_depth_m = lambda *a: None
        self.assertIsNone(v2.refine_segmentation(geom, rgb, np.zeros(mask.shape), v2.Settings()))
        self.assertIn('有效深度不足', geom.BOOK_SELECTION_NOTE)
        geom._sam3_raw_books = lambda *a: []
        self.assertIsNone(v2.refine_segmentation(geom, rgb, np.zeros(mask.shape), v2.Settings()))
        self.assertIn('未返回 book 候选', geom.BOOK_SELECTION_NOTE)


if __name__ == '__main__':
    unittest.main()
