import unittest
from unittest.mock import Mock, patch

import numpy as np

from utils import rerun_pipeline_logger as module


class RerunPipelineLoggerTests(unittest.TestCase):
    def make_logger(self):
        logger = module.RerunPipelineLogger.__new__(module.RerunPipelineLogger)
        logger.max_points = 100
        logger._stage_sequence = 0
        logger._run_sequence = 0
        logger._recording_path = None
        logger._closed = False
        logger._workspace_cache = None
        return logger

    def test_begin_run_advances_time_and_clears_dynamic_roots(self):
        logger = self.make_logger()
        logger.log_json_document = Mock()

        with (
            patch.object(module.rr, "set_time") as set_time,
            patch.object(module.rr, "log") as log,
        ):
            run_id = logger.begin_run("bowl")

        self.assertEqual(run_id, 1)
        self.assertEqual(logger._stage_sequence, 1)
        set_time.assert_called_once_with("pipeline_step", sequence=1)
        cleared_paths = [call.args[0] for call in log.call_args_list]
        self.assertIn("step1/camera", cleared_paths)
        self.assertIn("world/manual_plan/targets", cleared_paths)
        self.assertIn("world/execution", cleared_paths)
        logger.log_json_document.assert_called_once_with(
            "pipeline/current_run",
            {
                "run_id": 1,
                "task_type": "bowl",
                "pipeline_step": 1,
            },
        )

    def test_execution_action_advances_time_and_logs_target(self):
        logger = self.make_logger()
        logger._run_sequence = 3
        logger.log_text_document = Mock()
        logger.log_json_document = Mock()
        logger._log_transform_mm = Mock()
        logger._log_right_gripper_frame = Mock()
        logger._log_tool_pose_axes = Mock()
        logger._log_right_gripper_glyph = Mock()
        task = {
            "action": "move_above_grasp",
            "arm": "right",
            "object": "object_0",
            "description": "approach",
            "target_position": [400.0, -300.0, 120.0],
            "target_rotation": [0.0, 0.0, 0.0],
        }

        with (
            patch.object(module.rr, "set_time") as set_time,
            patch.object(module.rr, "log") as log,
        ):
            logger.log_execution_action(
                task_index=0,
                task_total=4,
                task=task,
                status="started",
            )

        self.assertEqual(logger._stage_sequence, 1)
        set_time.assert_called_once_with("pipeline_step", sequence=1)
        logger.log_text_document.assert_called_once()
        action_path, payload = logger.log_json_document.call_args.args
        self.assertEqual(
            action_path,
            "step5/execution/actions/000_move_above_grasp_object_0",
        )
        self.assertEqual(payload["run_id"], 3)
        self.assertEqual(payload["status"], "started")
        logger._log_transform_mm.assert_called_once()
        np.testing.assert_allclose(
            logger._log_transform_mm.call_args.args[1][:3, 3],
            [400.0, -300.0, 120.0],
        )
        self.assertIn(
            "world/execution/current_target",
            [call.args[0] for call in log.call_args_list],
        )


    def test_book_bounds_follow_thin_axis_instead_of_world_xyz(self):
        geometry = {
            "center_base_mm": [500.0, -275.0, -130.0],
            "thickness_mm": 38.0,
            "length_mm": 230.0,
            "height_mm": 161.0,
            "approach_xy": [-0.0698239, 0.9975593],
        }

        bounds = module.book_bounds_from_geometry(geometry)

        self.assertIsNotNone(bounds)
        np.testing.assert_allclose(bounds["center_m"], [0.5, -0.275, -0.13])
        np.testing.assert_allclose(bounds["sizes_m"], [0.038, 0.230, 0.161])
        expected_yaw = np.arctan2(0.9975593, -0.0698239)
        self.assertAlmostEqual(bounds["yaw_rad"], expected_yaw, places=6)
        thin = bounds["thin_xy"]
        long = bounds["long_xy"]
        rotated_thin = np.array([np.cos(bounds["yaw_rad"]), np.sin(bounds["yaw_rad"])])
        np.testing.assert_allclose(rotated_thin, thin, atol=1e-6)
        self.assertGreater(abs(thin[1]), abs(thin[0]))
        self.assertGreater(abs(long[0]), abs(long[1]))
        self.assertAlmostEqual(float(np.dot(thin, long)), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
