"""Offline Rerun SDK tests. Writes temporary recordings; never spawns a viewer."""
import sys
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).parent / 'RobotRAM'))
import rerun as rr
from utils.rerun_pipeline_logger import RerunPipelineLogger
from book_grasp_visualization import (install_book_visualization, match_book_mask,
                                      mask_cloud, forward_targets, preview_targets, english_task)


class BookVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.mask = np.zeros((120, 160), dtype=bool)
        self.mask[20:100, 70:81] = True
        self.depth = np.full(self.mask.shape, 800., dtype=float)
        self.rgb = np.full((*self.mask.shape, 3), 100, dtype=np.uint8)
        self.k = np.array([[200., 0, 80], [0, 200., 60], [0, 0, 1]])
        self.parameters = dict(forward_step_mm=125., lift_mm=200.,
                               grasp_insertion_mm=50., fingertip_offset_mm=50., pregrasp_clearance_mm=50., align_step_mm=50., lower_mm=200., hold_seconds=3.)

    def test_book_association_rejects_unrelated_high_score(self):
        wrong = np.roll(self.mask, 40, axis=1)
        actual = self.mask.copy()
        actual[20:100, 81:90] = True
        selected = match_book_mask([(.99, wrong, None), (.3, actual, None)], self.mask)
        np.testing.assert_array_equal(selected, actual)
        self.assertIsNone(match_book_mask([(.99, wrong, None)], self.mask))

    def test_cloud_uses_only_mask_and_current_extrinsics(self):
        transform = np.eye(4)
        transform[:3, 3] = [100, 200, 300]
        points, yy, xx = mask_cloud(self.mask, self.depth, self.k, transform, 100)
        self.assertLessEqual(len(points), 100)
        self.assertTrue(self.mask[yy, xx].all())
        np.testing.assert_allclose(points[:, 2], 1.1)
        np.testing.assert_allclose(points[:, 0], (xx-80)*.8/200+.1)

    def test_preview_includes_same_waypoints_and_place_height(self):
        plan = preview_targets([300, -200, 100], [0, -90, 0], [550, -200, 130], self.parameters)
        np.testing.assert_allclose(plan['points_mm'][1:5],
            [[300, -200, 130], [375, -200, 130], [450, -200, 130], [550, -200, 130]])
        np.testing.assert_allclose(plan['grasp_mm'], [550, -200, 130])
        np.testing.assert_allclose(plan['fingertip_grasp_mm'], [600, -200, 130])
        np.testing.assert_allclose(plan['points_mm'][5:10],
            [[550, -200, 330], [550, -200, 280], [550, -200, 230], [550, -200, 180], [550, -200, 130]])
        self.assertEqual(plan['labels'][6:10], ['lower 1/4', 'lower 2/4', 'lower 3/4', 'lower 4/4 / release'])
        np.testing.assert_allclose(plan['points_mm'][-2:], [[425, -200, 130], [300, -200, 130]])

    def test_waypoint_label_is_english_without_changing_control_task(self):
        task = dict(action='move_down_to_grasp', description='分段前伸',
                    book_forward_index=1, book_forward_count=2)
        display = english_task(task)
        self.assertEqual(display['description'], 'Forward waypoint 1/2')
        self.assertEqual(task['description'], '分段前伸')

    def test_flush_supports_old_and_new_sdk_signatures_and_missing_stream(self):
        agent = types.SimpleNamespace(RerunPipelineLogger=RerunPipelineLogger)
        install_book_visualization(agent, lambda _: None, lambda: self.parameters)
        logger = agent.RerunPipelineLogger.__new__(agent.RerunPipelineLogger)
        calls = []

        class OldRecording:
            def flush(self):
                calls.append('old')

        class NewRecording:
            def flush(self, blocking=True):
                calls.append(('new', blocking))

        for recording in (OldRecording(), NewRecording(), None):
            with patch.object(rr, 'get_global_data_recording', return_value=recording):
                logger.flush_book()
        self.assertEqual(calls, ['old', ('new', False)])

    def test_flush_failure_does_not_abort_begin_run(self):
        agent = types.SimpleNamespace(RerunPipelineLogger=RerunPipelineLogger)
        install_book_visualization(agent, lambda _: None, lambda: self.parameters)
        logger = agent.RerunPipelineLogger.__new__(agent.RerunPipelineLogger)
        logger._stage_sequence = logger._run_sequence = 0
        logger.send_default_blueprint = Mock()
        logger.log_text_document = Mock()

        class BrokenRecording:
            def flush(self):
                raise RuntimeError('disconnected viewer')

        with (patch.object(rr, 'get_global_data_recording', return_value=BrokenRecording()),
              patch.object(rr, 'log'), patch.object(rr, 'set_time'), patch('builtins.print') as warn):
            self.assertEqual(logger.begin_run('cart+'), 1)
            self.assertEqual(logger.begin_run('cart+'), 2)
            self.assertEqual(warn.call_count, 1)

    def test_real_sdk_perception_before_execution_no_stale_files_or_boxes(self):
        agent = types.SimpleNamespace(RerunPipelineLogger=RerunPipelineLogger)
        session = install_book_visualization(agent, lambda _: (np.array([300, -200, 100]), np.array([0., -90., 0.])), lambda: self.parameters)
        robot = types.SimpleNamespace(get_current_state=lambda: {'right_arm': {'position': [.3, -.2, .1]}})
        selected = dict(mask=self.mask, uv=[75., 59.5], point_camera_m=np.array([-.02, -.002, .8]),
                        depth_m=.8, score=.8, depth_samples=85, spine_tilt_deg=30.,
                        spine_axis_uv=[.5, np.sqrt(.75)], spine_length_px=80.,
                        spine_original_center_uv=[73., 55.], spine_stable_region_fraction=.45,
                        spine_stable_region_uv=[[70., 45.], [80., 74.]],
                        spine_center_recovery='stable_region',
                        spine_full_fit_failure='3D spine line is too short or inconsistent')
        with tempfile.TemporaryDirectory() as directory:
            logger = agent.RerunPipelineLogger(recording_path=Path(directory)/'test.rrd', spawn_viewer=False)
            try:
                session.attach_robot(robot)
                with patch.object(rr, 'log', wraps=rr.log) as log:
                    logger.begin_run('cart+')
                    session.capture_candidates(self.rgb,
                        {'spine': [dict(index=0, mask=self.mask, score=.8)], 'book': [], 'cover': [], 'pages': []},
                        {'candidates': [dict(index=0, reasons=[])], 'accepted_count': 1})
                    self.assertIsNone(session.frame)  # Candidates must render even before geometry succeeds.
                    session.capture(self.rgb, self.depth, self.k, np.eye(4), selected, self.mask)
                    session.update_geometry(dict(center_base_mm=[550, -200, 130],
                        spine_vertical_tilt_deg=30., spine_image_tilt_deg=30.,
                        spine_axis_base=[0, -.5, np.sqrt(.75)],
                        spine_endpoints_base_m=[[-.02, .05, .7], [-.02, -.05, .9]],
                        spine_axis_residual_mm=1., wrist_enabled=False,
                        j7_current_deg=5., j7_delta_deg=30., j7_target_deg=35.), np.eye(4))
                    # Neither method may look at the directory, even if old artifacts exist.
                    with patch.object(Path, 'glob', side_effect=AssertionError('stale glob')):
                        logger.log_detection_artifacts(Path('/nonexistent'))
                        logger.log_ram_artifacts(Path('/nonexistent'))
                    paths = [c.args[0] for c in log.call_args_list]
                    for path in ('book/images/book', 'book/images/spine', 'book/depth', 'world/environment/cloud',
                                 'world/book/spine_center', 'world/book/preview/grasp_tcp',
                                 'world/book/preview/points', 'world/book/tilt/arc',
                                 'world/book/tilt/label', 'book/wrist', 'book/candidates/spine',
                                 'book/candidates/cover', 'book/candidates/pages', 'book/semantic_report'):
                        self.assertIn(path, paths)
                    self.assertFalse(session.tracking)
                    documents = [text for call in log.call_args_list if isinstance(call.args[1], rr.TextDocument)
                                 for text in call.args[1].text.as_arrow_array().to_pylist()]
                    self.assertTrue(any('Stable region: 45% of spine length' in text for text in documents))
                    self.assertFalse(any(isinstance(c.args[1], (rr.Boxes2D, rr.Boxes3D)) for c in log.call_args_list))
                    for path, mask in (('world/environment/cloud', ~self.mask),
                                       ('world/book/cloud/book', self.mask),
                                       ('world/book/cloud/spine', self.mask)):
                        clouds = [c.args[1] for c in log.call_args_list
                                  if c.args[0] == path and isinstance(c.args[1], rr.Points3D)]
                        points, yy, xx = mask_cloud(mask, self.depth, self.k, np.eye(4), logger.max_points)
                        expected = rr.Points3D(points, colors=self.rgb[yy, xx])
                        self.assertEqual(clouds[-1].positions.as_arrow_array().to_pylist(),
                                         expected.positions.as_arrow_array().to_pylist())
                        self.assertEqual(clouds[-1].colors.as_arrow_array().to_pylist(),
                                         expected.colors.as_arrow_array().to_pylist())
                    logger.log_book_target(dict(action='move_down_to_grasp', target_position=[550, -200, 100],
                                                book_forward_index=2, book_forward_count=2, description='分段前伸'))
                    for call in log.call_args_list:
                        archetype = call.args[1]
                        if isinstance(archetype, rr.TextDocument):
                            self.assertTrue(all(value.isascii() for value in archetype.text.as_arrow_array().to_pylist()))
                        if isinstance(archetype, rr.Points3D) and archetype.labels is not None:
                            self.assertTrue(all(value.isascii() for value in archetype.labels.as_arrow_array().to_pylist()))
                    session.tracking = True
                    self.assertEqual(robot.get_current_state()['right_arm']['position'], [.3, -.2, .1])
                    self.assertEqual(len(session.samples), 1)
                    logger.log_execution_action(task_index=0, task_total=1, task=dict(action='move_down_to_grasp', arm='right'), status='started')
                    logger.begin_run('cart+')
                    self.assertIsNone(session.frame)
                    self.assertIsNone(session.geometry)
                    self.assertIsNone(session.semantic)
                    self.assertEqual(session.samples, [])
                    self.assertEqual(logger.sent_targets, [])
                    self.assertIn('world/environment', [c.args[0] for c in log.call_args_list
                                                       if isinstance(c.args[1], rr.Clear)])
                    rr.get_global_data_recording().flush()
                    self.assertGreater((Path(directory)/'test.rrd').stat().st_size, 100)
            finally:
                logger.close()


if __name__ == '__main__':
    unittest.main()
