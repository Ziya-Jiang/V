"""Reusable Rerun logging helpers for the RobotRAM stage pipeline.

The legacy RobotRAM stages exchange files and use millimeters in their robot
world frame.  This module provides a single place to translate those artifacts
into a local Rerun session, whose world coordinates are expressed in meters.
"""

from __future__ import annotations

import atexit
import json
import uuid
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

try:
    import rerun as rr
    import rerun.blueprint as rrb
except ImportError as error:  # pragma: no cover - gives a useful runtime error
    raise RuntimeError(
        "Rerun SDK is required. Install RobotRAM/requirements.txt first."
    ) from error


MM_PER_METER = 1000.0
LEFT_ARM_COLOR = np.array([0, 190, 255], dtype=np.uint8)
RIGHT_ARM_COLOR = np.array([255, 168, 0], dtype=np.uint8)


def book_bounds_from_geometry(book_geometry: dict[str, Any]) -> dict[str, Any] | None:
    """Return an oriented standing-book box in meters.

    Point-cloud PCA stores thickness/length/height in the book's own frame:
    local X is the thin (jaw) axis, local Y is the long axis, local Z is up.
    Rerun ``Boxes3D`` sizes are those local extents; ``yaw_rad`` rotates local
    X onto ``approach_xy`` / ``thin_xy`` in the robot base. Drawing the same
    numbers as world XYZ with no rotation makes the box look 90° off the book.
    """
    center_mm = np.asarray(book_geometry.get("center_base_mm", []), dtype=float)
    if center_mm.shape != (3,) or not np.isfinite(center_mm).all():
        return None

    thickness_mm = book_geometry.get("thickness_mm")
    length_mm = book_geometry.get("length_mm")
    height_mm = book_geometry.get("height_mm")
    size_mm = np.asarray(
        [thickness_mm, length_mm, height_mm],
        dtype=float,
    )
    if (
        size_mm.shape != (3,)
        or not np.isfinite(size_mm).all()
        or np.any(size_mm <= 0.0)
    ):
        size_mm = np.asarray(
            book_geometry.get("axis_size_mm", []),
            dtype=float,
        )
        if (
            size_mm.shape != (3,)
            or not np.isfinite(size_mm).all()
            or np.any(size_mm <= 0.0)
        ):
            return None

    thin_xy = np.asarray(
        book_geometry.get("thin_xy", book_geometry.get("approach_xy", [0.0, 0.0])),
        dtype=float,
    ).reshape(-1)
    yaw_rad = 0.0
    if thin_xy.size >= 2 and np.isfinite(thin_xy[:2]).all():
        thin_norm = float(np.linalg.norm(thin_xy[:2]))
        if thin_norm >= 1e-6:
            thin_xy = thin_xy[:2] / thin_norm
            yaw_rad = float(np.arctan2(thin_xy[1], thin_xy[0]))
        else:
            thin_xy = np.array([1.0, 0.0], dtype=float)
    else:
        thin_xy = np.array([1.0, 0.0], dtype=float)
    long_xy = np.array([-thin_xy[1], thin_xy[0]], dtype=float)
    stored_long = np.asarray(book_geometry.get("long_xy", []), dtype=float).reshape(-1)
    if (
        stored_long.size >= 2
        and np.isfinite(stored_long[:2]).all()
        and float(np.linalg.norm(stored_long[:2])) >= 1e-6
    ):
        stored_long = stored_long[:2] / float(np.linalg.norm(stored_long[:2]))
        if float(np.dot(stored_long, long_xy)) < 0.0:
            stored_long = -stored_long
        long_xy = stored_long

    return {
        "center_m": center_mm / MM_PER_METER,
        "sizes_m": size_mm / MM_PER_METER,
        "yaw_rad": yaw_rad,
        "thin_xy": thin_xy,
        "long_xy": long_xy,
    }


def _read_json(path: Path, default: Any = None) -> Any:
    if not path.is_file():
        return default
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def _read_text(path: Path, default: str = "") -> str:
    if not path.is_file():
        return default
    return path.read_text(encoding="utf-8")


def _read_image_rgb(path: Path) -> np.ndarray | None:
    if not path.is_file():
        return None
    image_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        return None
    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def _safe_component_name(value: Any) -> str:
    return str(value).replace("/", "_").replace(" ", "_")


class RerunPipelineLogger:
    """Logs stage artifacts into one local Rerun recording/viewer session."""

    def __init__(
        self,
        *,
        application_id: str = "robotram_pipeline",
        recording_path: Path | None = None,
        spawn_viewer: bool = True,
        max_points: int = 100_000,
    ) -> None:
        self.max_points = max_points
        self._stage_sequence = 0
        self._run_sequence = 0
        self._recording_path = recording_path
        self._closed = False
        self._workspace_cache: dict[str, Any] | None = None

        rr.init(
            application_id,
            recording_id=uuid.uuid4(),
            spawn=False,
        )
        if recording_path is not None:
            recording_path.parent.mkdir(parents=True, exist_ok=True)
            rr.save(recording_path)
        if spawn_viewer:
            # rr.save installs a file sink. Spawn/connect afterwards so the
            # same recording is sent to both the .rrd file and live Viewer.
            rr.spawn(connect=True)

        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log(
            "pipeline/metadata",
            rr.TextDocument(
                "RobotRAM pipeline visualization. "
                "World-space coordinates are converted from millimeters to meters."
            ),
            static=True,
        )
        rr.log(
            "step4/manual_plan/legend",
            rr.TextDocument(
                "## TCP pose axes\n"
                "- Red: +X axis\n"
                "- Green: +Y axis\n"
                "- Blue: +Z axis\n"
                "- Each axis is 8 cm long.\n"
                "- Right-arm axes use the physical gripper-base frame from "
                "the URDF. The simplified gripper points along +Z and its "
                "jaw width is along X.\n"
                "- Right-arm targets include an orange simplified two-finger "
                "gripper in that physical frame."
            ),
            static=True,
        )
        rr.log(
            "workspace/planning_slices/legend",
            rr.TextDocument(
                "## Per-target reachability slices\n"
                "- Horizontal axis: physical Y (+Y points left)\n"
                "- Vertical axis: physical X (+X points up)\n"
                "- Blue: left arm\n"
                "- Orange: right arm\n"
                "- Green dot: target cell is reachable by its assigned arm in this cache\n"
                "- Red dot: target cell is outside its assigned arm's cached workspace\n"
                "- Each tile uses the planning task's own physical Z height.\n"
                "- Workspace cells are deliberately not rendered in the 3D scene."
                "\n- This is fixed-orientation cache membership, not a full runtime IK result."
            ),
            static=True,
        )
        atexit.register(self.close)

    @property
    def recording_path(self) -> Path | None:
        return self._recording_path

    def close(self) -> None:
        """Flush and detach the local Rerun recording stream once."""
        if not self._closed:
            self._closed = True
            atexit.unregister(self.close)
            # Disconnect before camera SDK teardown so the gRPC stream gets a
            # normal end-of-stream instead of observing an aborting process.
            rr.disconnect()

    def send_default_blueprint(self) -> None:
        """Open the pipeline with image, depth, world, and plan views visible."""
        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Vertical(
                    rrb.Spatial2DView(
                        origin="/step1/camera/rgb/main",
                        contents=["$origin"],
                        name="Grounding RGB",
                    ),
                    rrb.Spatial2DView(
                        origin="/step1/grounding/labeling_image",
                        contents=["$origin"],
                        name="Grounding Labels",
                    ),
                    rrb.Spatial2DView(
                        origin="/step2/vggt/depth",
                        contents=["$origin"],
                        name="VGGT Depth",
                    ),
                ),
                rrb.Vertical(
                    rrb.Spatial3DView(
                        origin="/world",
                        name="Calibrated Point Clouds & Plan",
                    ),
                    rrb.Horizontal(
                        rrb.Spatial2DView(
                            origin="/workspace/planning_slices/overview",
                            contents=["$origin"],
                            name="Reachability per Plan Target (+Y left, +X up)",
                        ),
                        rrb.Vertical(
                            rrb.TextDocumentView(
                                origin="/step5/execution/current_action",
                                name="Live Execution Action",
                            ),
                            rrb.TextDocumentView(
                                origin="/step4/manual_plan/tasks",
                                name="Manual Planning Tasks",
                            ),
                            rrb.TextDocumentView(
                                origin="/workspace/planning_slices/legend",
                                name="Reachability Legend",
                            ),
                            row_shares=[0.24, 0.56, 0.20],
                        ),
                        column_shares=[0.72, 0.28],
                    ),
                    row_shares=[0.60, 0.40],
                ),
                column_shares=[0.42, 0.58],
            ),
            auto_layout=False,
            auto_views=False,
            collapse_panels=False,
        )
        rr.send_blueprint(blueprint, make_active=True, make_default=True)

    def _advance_pipeline_step(self) -> None:
        self._stage_sequence += 1
        rr.set_time("pipeline_step", sequence=self._stage_sequence)

    def begin_run(self, task_type: str) -> int:
        """Start a loop iteration and remove the previous run's dynamic data."""
        self._run_sequence += 1
        self._advance_pipeline_step()
        dynamic_roots = (
            "step1/camera",
            "step1/grounding",
            "step1/vlm",
            "step2/vggt",
            "step2/objects",
            "step3/images",
            "step3/objects",
            "step3/subtasks",
            "step3/actions",
            "step4/manual_plan/tasks",
            "step5/execution",
            "world/camera/extern/objects",
            "world/camera/extern/point_clouds",
            "world/manual_plan/targets",
            "world/manual_plan/waypoint_paths",
            "world/manual_plan/waypoints",
            "world/grasp_candidates",
            "world/selected_book",
            "world/execution",
            "workspace/planning_slices/overview",
        )
        for entity_path in dynamic_roots:
            rr.log(entity_path, rr.Clear(recursive=True))
        self.log_json_document(
            "pipeline/current_run",
            {
                "run_id": self._run_sequence,
                "task_type": task_type,
                "pipeline_step": self._stage_sequence,
            },
        )
        return self._run_sequence

    def begin_stage(self, stage_name: str, description: str = "") -> None:
        self._advance_pipeline_step()
        rr.log(
            "pipeline/current_stage",
            rr.TextLog(f"{stage_name}: {description or 'completed'}"),
        )

    def log_text_document(self, entity_path: str, text: str) -> None:
        rr.log(entity_path, rr.TextDocument(text))

    def log_json_document(self, entity_path: str, data: Any) -> None:
        self.log_text_document(
            entity_path,
            json.dumps(data, indent=2, ensure_ascii=False, default=str),
        )

    def log_execution_action(
        self,
        *,
        task_index: int,
        task_total: int,
        task: dict[str, Any],
        status: str,
        result: Any = None,
        error: str | None = None,
    ) -> None:
        """Log one hardware action transition without changing its target pose."""
        self._advance_pipeline_step()
        action = _safe_component_name(task.get("action", "unknown"))
        object_id = _safe_component_name(task.get("object", "none"))
        arm = _safe_component_name(task.get("arm", "unknown"))
        payload = {
            "run_id": self._run_sequence,
            "task_index": task_index + 1,
            "task_total": task_total,
            "status": status,
            "action": task.get("action"),
            "arm": task.get("arm"),
            "object": task.get("object"),
            "description": task.get("description"),
            "target_position_mm": task.get("target_position"),
            "target_rotation_deg": task.get("target_rotation"),
            "result": result,
            "error": error,
        }
        summary = (
            f"## Action {task_index + 1}/{task_total}\n"
            f"**{status.upper()}** · `{arm}` · `{action}` · `{object_id}`"
        )
        if error:
            summary += f"\n\n{error}"
        self.log_text_document("step5/execution/current_action", summary)
        self.log_json_document(
            f"step5/execution/actions/{task_index:03d}_{action}_{object_id}",
            payload,
        )

        if status != "started":
            return
        rr.log("world/execution/current_target", rr.Clear(recursive=True))
        target_position = np.asarray(task.get("target_position", []), dtype=float)
        target_rotation = np.asarray(task.get("target_rotation", []), dtype=float)
        if target_position.size < 3 or target_rotation.size < 3:
            return
        pose_mm_deg = np.concatenate((target_position[:3], target_rotation[:3]))
        if not np.isfinite(pose_mm_deg).all():
            return
        tool_pose_path = "world/execution/current_target/tool_pose"
        self._log_transform_mm(
            tool_pose_path,
            _pose_mm_deg_to_matrix(pose_mm_deg),
        )
        if arm == "right":
            gripper_path = f"{tool_pose_path}/right_gripper"
            self._log_right_gripper_frame(gripper_path)
            self._log_tool_pose_axes(f"{gripper_path}/axes")
            self._log_right_gripper_glyph(f"{gripper_path}/model")
        else:
            gripper_path = f"{tool_pose_path}/left_gripper"
            self._log_tool_pose_axes(f"{gripper_path}/axes")
            self._log_gripper_glyph(
                f"{gripper_path}/model",
                color=LEFT_ARM_COLOR.tolist(),
                thin=False,
            )

    def log_workspace_slice_cache(
        self,
        cache_path: Path,
    ) -> dict[str, Any] | None:
        """Load workspace cache for later per-planning-target 2D slice rendering."""
        cache_path = Path(cache_path).expanduser().resolve()
        if not cache_path.is_file():
            self.log_text_document(
                "workspace/planning_slices/status",
                f"Workspace cache was not found: {cache_path}",
            )
            return None

        with np.load(cache_path, allow_pickle=False) as cache:
            required = {
                "grid_resolution_m",
                "default_slice_thickness_m",
                "left_cells_xyz",
                "right_cells_xyz",
            }
            missing = sorted(required.difference(cache.files))
            if missing:
                raise ValueError(
                    f"Workspace cache {cache_path} is missing keys: {missing}"
                )
            grid_resolution_m = float(
                np.asarray(cache["grid_resolution_m"]).item()
            )
            default_thickness_m = float(
                np.asarray(cache["default_slice_thickness_m"]).item()
            )
            cells_by_arm = {
                "left": np.asarray(cache["left_cells_xyz"], dtype=np.int32).reshape(
                    -1, 3
                ),
                "right": np.asarray(
                    cache["right_cells_xyz"], dtype=np.int32
                ).reshape(-1, 3),
            }

        if grid_resolution_m <= 0.0 or default_thickness_m <= 0.0:
            raise ValueError(
                "Workspace cache grid resolution and slice thickness must be positive."
            )
        self._workspace_cache = {
            "cache_path": cache_path,
            "grid_resolution_m": grid_resolution_m,
            "slice_thickness_m": default_thickness_m,
            "cells_by_arm": cells_by_arm,
        }

        summary = {
            "cache_path": str(cache_path),
            "grid_resolution_m": grid_resolution_m,
            "slice_thickness_m": default_thickness_m,
            "cell_counts": {
                arm: int(cells.shape[0]) for arm, cells in cells_by_arm.items()
            },
            "axis_mapping_2d": {
                "horizontal": "physical Y",
                "vertical": "physical X",
            },
        }
        self.log_json_document("workspace/planning_slices/cache_metadata", summary)
        return summary

    def _log_workspace_slices_for_tasks(
        self, tasks: Iterable[dict[str, Any]]
    ) -> None:
        """Render one raster reachability slice for every task with a TCP target."""
        if self._workspace_cache is None:
            return

        cache = self._workspace_cache
        cells_by_arm = cache["cells_by_arm"]
        grid_resolution_m = float(cache["grid_resolution_m"])
        slice_thickness_m = float(cache["slice_thickness_m"])
        all_cells = np.concatenate(
            [cells_by_arm["left"], cells_by_arm["right"]], axis=0
        )
        if not all_cells.size:
            return

        min_x, min_y = np.min(all_cells[:, :2], axis=0)
        max_x, max_y = np.max(all_cells[:, :2], axis=0)
        image_width = int(max_y - min_y + 1)
        image_height = int(max_x - min_x + 1)
        display_scale = max(
            1, min(4, int(360 / max(image_width, image_height, 1)))
        )
        header_height = 44
        arm_colors = {"left": LEFT_ARM_COLOR, "right": RIGHT_ARM_COLOR}
        tile_records: list[dict[str, Any]] = []
        tiles: list[np.ndarray] = []

        for index, task in enumerate(tasks):
            target_position_mm = np.asarray(
                task.get("target_position", []), dtype=float
            )
            if target_position_mm.size < 3 or not np.isfinite(
                target_position_mm[:3]
            ).all():
                continue

            target_position_m = target_position_mm[:3] / MM_PER_METER
            target_z_m = float(target_position_m[2])
            target_arm = _safe_component_name(task.get("arm", "unknown"))
            target_x_cell = int(
                np.floor(target_position_m[0] / grid_resolution_m)
            )
            target_y_cell = int(
                np.floor(target_position_m[1] / grid_resolution_m)
            )
            target_cell = np.array(
                [target_x_cell, target_y_cell], dtype=np.int32
            )
            target_is_in_assigned_workspace = False
            occupancy_rgb = np.full(
                (image_height, image_width, 3), [24, 27, 33], dtype=np.uint8
            )
            cell_counts: dict[str, int] = {}
            for arm, cells_xyz in cells_by_arm.items():
                z_centers_m = (
                    cells_xyz[:, 2].astype(np.float32) + 0.5
                ) * grid_resolution_m
                cells_at_z = cells_xyz[
                    np.abs(z_centers_m - target_z_m) <= slice_thickness_m * 0.5
                ]
                cell_counts[arm] = int(cells_at_z.shape[0])
                if arm == target_arm and cells_at_z.size:
                    target_is_in_assigned_workspace = bool(
                        np.any(
                            np.all(
                                cells_at_z[:, :2] == target_cell.reshape(1, 2),
                                axis=1,
                            )
                        )
                    )
                if not cells_at_z.size:
                    continue
                rows = max_x - cells_at_z[:, 0]
                # Image columns grow to the right. Flip physical Y so the
                # requested display convention is +Y left, +X up.
                cols = max_y - cells_at_z[:, 1]
                occupied = np.any(occupancy_rgb[rows, cols] != [24, 27, 33], axis=1)
                color = arm_colors[arm]
                occupancy_rgb[rows, cols] = np.where(
                    occupied[:, None],
                    (
                        occupancy_rgb[rows, cols].astype(np.uint16)
                        + color.astype(np.uint16)
                    )
                    // 2,
                    color,
                ).astype(np.uint8)

            enlarged = cv2.resize(
                occupancy_rgb,
                dsize=None,
                fx=display_scale,
                fy=display_scale,
                interpolation=cv2.INTER_NEAREST,
            )
            tile = np.full(
                (header_height + enlarged.shape[0], enlarged.shape[1], 3),
                [14, 16, 20],
                dtype=np.uint8,
            )
            tile[header_height:] = enlarged
            action = _safe_component_name(task.get("action", "target"))
            arm = target_arm
            subtask_id = task.get("subtask_id", index + 1)
            cv2.putText(
                tile,
                f"#{subtask_id} {arm} {action}",
                (7, 17),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (235, 235, 235),
                1,
                cv2.LINE_AA,
            )
            reachability_label = (
                "CACHE REACHABLE" if target_is_in_assigned_workspace else "OUTSIDE CACHE"
            )
            cv2.putText(
                tile,
                f"Z={target_z_m:.3f}m | {reachability_label}",
                (7, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (80, 235, 100)
                if target_is_in_assigned_workspace
                else (255, 90, 90),
                1,
                cv2.LINE_AA,
            )

            marker_row = int((max_x - target_x_cell) * display_scale + display_scale // 2)
            marker_col = int((max_y - target_y_cell) * display_scale + display_scale // 2)
            marker_y = header_height + marker_row
            if 0 <= marker_col < tile.shape[1] and header_height <= marker_y < tile.shape[0]:
                marker_color = (
                    (55, 225, 85)
                    if target_is_in_assigned_workspace
                    else (255, 48, 48)
                )
                cv2.circle(
                    tile,
                    (marker_col, marker_y),
                    max(4, display_scale * 2),
                    marker_color,
                    thickness=-1,
                    lineType=cv2.LINE_AA,
                )
                cv2.circle(
                    tile,
                    (marker_col, marker_y),
                    max(5, display_scale * 2 + 1),
                    (255, 255, 255),
                    thickness=1,
                    lineType=cv2.LINE_AA,
                )

            entity_name = (
                f"{index:02d}_{_safe_component_name(action)}_"
                f"{_safe_component_name(task.get('object', 'unknown'))}"
            )
            self.log_image(f"workspace/planning_slices/{entity_name}", tile)
            tiles.append(tile)
            tile_records.append(
                {
                    "index": index,
                    "subtask_id": subtask_id,
                    "action": action,
                    "arm": arm,
                    "target_position_m": target_position_m.tolist(),
                    "cell_counts": cell_counts,
                    "within_assigned_arm_cached_workspace": target_is_in_assigned_workspace,
                    "entity_path": f"workspace/planning_slices/{entity_name}",
                }
            )

        if not tiles:
            return
        columns = min(3, len(tiles))
        rows = (len(tiles) + columns - 1) // columns
        tile_height = max(tile.shape[0] for tile in tiles)
        tile_width = max(tile.shape[1] for tile in tiles)
        overview = np.full(
            (rows * tile_height, columns * tile_width, 3),
            [8, 10, 14],
            dtype=np.uint8,
        )
        for tile_index, tile in enumerate(tiles):
            row = tile_index // columns
            column = tile_index % columns
            y0 = row * tile_height
            x0 = column * tile_width
            overview[y0:y0 + tile.shape[0], x0:x0 + tile.shape[1]] = tile

        self.log_image("workspace/planning_slices/overview", overview)
        self.log_json_document("workspace/planning_slices/metadata", tile_records)

    def log_image(self, entity_path: str, image: np.ndarray | None) -> None:
        if image is not None and image.size:
            rr.log(entity_path, rr.Image(image))

    def log_image_file(self, entity_path: str, image_path: Path) -> None:
        self.log_image(entity_path, _read_image_rgb(image_path))

    def log_depth_file(
        self,
        entity_path: str,
        depth_path: Path,
        *,
        units_per_meter: float = MM_PER_METER,
    ) -> None:
        if not depth_path.is_file():
            return
        depth = np.load(depth_path, allow_pickle=False)
        if depth.ndim == 2:
            rr.log(entity_path, rr.DepthImage(depth, meter=units_per_meter))

    def log_camera(
        self,
        *,
        c2w_mm: np.ndarray | None,
        intrinsic: np.ndarray | None,
        resolution: tuple[int, int] | None,
    ) -> None:
        camera_path = "world/camera/extern"
        image_path = f"{camera_path}/image"
        if c2w_mm is not None:
            matrix = np.asarray(c2w_mm, dtype=float)
            if matrix.shape == (4, 4):
                rr.log(
                    camera_path,
                    rr.Transform3D(
                        translation=matrix[:3, 3] / MM_PER_METER,
                        mat3x3=matrix[:3, :3],
                        from_parent=False,
                    ),
                    static=True,
                )
        if intrinsic is not None and resolution is not None:
            intrinsic = np.asarray(intrinsic, dtype=float)
            if intrinsic.shape == (3, 3):
                rr.log(
                    image_path,
                    rr.Pinhole(
                        image_from_camera=intrinsic,
                        resolution=list(resolution),
                    ),
                    static=True,
                )

    def log_detection_artifacts(self, output_dir: Path) -> None:
        self.log_image_file(
            "step1/camera/rgb/main",
            output_dir / "rgb_ext.png",
        )
        self.log_image_file(
            "step1/camera/rgb/left",
            output_dir / "rgb_ext_left.png",
        )
        self.log_image_file(
            "step1/camera/rgb/right",
            output_dir / "rgb_ext_right.png",
        )
        self.log_image_file(
            "step1/grounding/labeling_image",
            output_dir / "rgb_ext_labeling.png",
        )
        self.log_depth_file(
            "step1/camera/depth",
            output_dir / "depth_ext.npy",
        )

        detections_path = output_dir / "detections.npy"
        if detections_path.is_file():
            detections = np.load(detections_path, allow_pickle=True).item()
            masks = np.asarray(detections.get("masks", []))
            boxes = np.asarray(detections.get("xyxys", []))
            class_ids = np.asarray(detections.get("class_ids", []))

            if masks.ndim == 3:
                segmentation = np.zeros(masks.shape[1:], dtype=np.uint16)
                for index, mask in enumerate(masks):
                    segmentation[np.asarray(mask, dtype=bool)] = index + 1
                rr.log(
                    "step1/grounding/segmentation",
                    rr.SegmentationImage(segmentation, opacity=0.45),
                )

            if boxes.ndim == 2 and boxes.shape[1] == 4:
                mins = boxes[:, :2]
                sizes = boxes[:, 2:] - boxes[:, :2]
                labels = [f"id={value}" for value in class_ids.tolist()]
                rr.log(
                    "step1/grounding/bboxes",
                    rr.Boxes2D(mins=mins, sizes=sizes, labels=labels),
                )

        self.log_json_document(
            "step1/vlm/objects",
            _read_json(output_dir / "vlm_objects.json", []),
        )
        self.log_json_document(
            "step1/vlm/categories",
            _read_json(output_dir / "vlm_categories.json", []),
        )

    def _log_transform_mm(self, entity_path: str, matrix: np.ndarray) -> None:
        self._log_transform(
            entity_path,
            matrix,
            units_per_meter=MM_PER_METER,
        )

    def _log_transform(
        self,
        entity_path: str,
        matrix: np.ndarray,
        *,
        units_per_meter: float,
    ) -> None:
        matrix = np.asarray(matrix, dtype=float)
        if matrix.shape != (4, 4):
            return
        rr.log(
            entity_path,
            rr.Transform3D(
                translation=matrix[:3, 3] / units_per_meter,
                mat3x3=matrix[:3, :3],
                from_parent=False,
            ),
        )

    def _log_oriented_book_bounds(self, book_geometry: dict[str, Any]) -> None:
        """Draw the PCA book box in the same frame the gripper yaw uses."""
        bounds = book_bounds_from_geometry(book_geometry)
        if bounds is None:
            return
        center_m = bounds["center_m"]
        sizes_m = bounds["sizes_m"]
        yaw_rad = bounds["yaw_rad"]
        thin_xy = bounds["thin_xy"]
        long_xy = bounds["long_xy"]
        mode = str(book_geometry.get("mode", "book"))
        rr.log(
            "world/selected_book/bounds",
            rr.Boxes3D(
                centers=[center_m],
                sizes=[sizes_m],
                rotation_axis_angles=[
                    rr.RotationAxisAngle(
                        axis=[0.0, 0.0, 1.0],
                        radians=yaw_rad,
                    )
                ],
                colors=[[255, 215, 0]],
                labels=[f"{mode} book"],
            ),
        )
        half_thin_m = 0.5 * float(sizes_m[0])
        half_long_m = 0.5 * float(sizes_m[1])
        rr.log(
            "world/selected_book/axes",
            rr.Arrows3D(
                origins=[center_m, center_m],
                vectors=[
                    [thin_xy[0] * half_thin_m, thin_xy[1] * half_thin_m, 0.0],
                    [long_xy[0] * half_long_m, long_xy[1] * half_long_m, 0.0],
                ],
                colors=[[255, 80, 80], [80, 200, 255]],
                radii=0.006,
                labels=["thickness / jaws", "length"],
            ),
        )

    def _log_tool_pose_axes(self, entity_path: str) -> None:
        """Draw the local TCP axes below a target-pose transform."""
        axis_length_m = 0.08
        rr.log(
            entity_path,
            rr.Arrows3D(
                origins=np.zeros((3, 3)),
                vectors=np.eye(3) * axis_length_m,
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                radii=0.004,
            ),
        )

    def _log_right_gripper_frame(self, entity_path: str) -> None:
        """Attach the right physical gripper-base frame below its TCP frame.

        The simulator's visual gripper frame is ``gripper_base_link_r``.
        Its fixed URDF joint relative to ``arm_end_effector_r`` has
        ``rpy=[0, -pi, pi]``, i.e. local X is retained while local Y and Z
        are reversed. Logging this transform makes the Rerun axes agree with
        the simulation instead of showing the upstream arm-end-effector axes.
        """
        rr.log(
            entity_path,
            rr.Transform3D(
                translation=[0.0, 0.0, -0.005],
                mat3x3=np.diag([1.0, -1.0, -1.0]),
                relation=rr.TransformRelation.ParentFromChild,
            ),
        )

    def _log_right_gripper_glyph(self, entity_path: str) -> None:
        """Draw a two-finger right-gripper glyph in the physical gripper frame.

        The geometry is intentionally schematic rather than an exact URDF
        mesh.  It makes the planned grasp orientation immediately readable:
        the fingers point along local +Z, while their separation/open-close
        direction is local X.
        """
        self._log_gripper_glyph(
            entity_path,
            color=[255, 168, 0],
            thin=False,
        )

    def _log_gripper_glyph(
        self,
        entity_path: str,
        *,
        color: list[int],
        thin: bool,
    ) -> None:
        """Draw a schematic gripper usable for either arm's candidate poses."""
        jaw_half_width_m = 0.030
        finger_length_m = 0.075 if thin else 0.090
        finger_width_m = 0.004 if thin else 0.008
        finger_depth_m = 0.005 if thin else 0.010
        bridge_size = (
            [0.060, 0.010, 0.008] if thin else [0.072, 0.020, 0.012]
        )

        # All coordinates are local to the glyph parent. Rerun composes this
        # with either the right physical-gripper frame or the left TCP frame.
        centers = np.array(
            [
                [0.0, 0.0, -0.012],             # wrist / bridge
                [-jaw_half_width_m, 0.0, 0.045], # -X finger
                [jaw_half_width_m, 0.0, 0.045],  # +X finger
                [-jaw_half_width_m, 0.0, 0.092], # -X fingertip pad
                [jaw_half_width_m, 0.0, 0.092],  # +X fingertip pad
            ],
            dtype=float,
        )
        sizes = np.array(
            [
                bridge_size,
                [finger_width_m, finger_depth_m, finger_length_m],
                [finger_width_m, finger_depth_m, finger_length_m],
                [finger_width_m * 1.15, finger_depth_m * 1.15, 0.007],
                [finger_width_m * 1.15, finger_depth_m * 1.15, 0.007],
            ],
            dtype=float,
        )
        rr.log(
            entity_path,
            rr.Boxes3D(
                centers=centers,
                sizes=sizes,
                colors=[
                    color,
                    color,
                    color,
                    color,
                    color,
                ],
            ),
        )

    def _log_xyzrgb_file(
        self,
        entity_path: str,
        npy_path: Path,
        *,
        units_per_meter: float,
    ) -> None:
        if not npy_path.is_file():
            return
        cloud = np.asarray(np.load(npy_path, allow_pickle=False))
        if cloud.ndim != 2 or cloud.shape[1] < 3:
            return
        points = cloud[:, :3]
        valid = np.isfinite(points).all(axis=1)
        colors = None
        if cloud.shape[1] >= 6:
            colors = cloud[:, 3:6]
            valid &= np.isfinite(colors).all(axis=1)
        points = points[valid]
        if colors is not None:
            colors = colors[valid]
        if len(points) == 0:
            return
        if len(points) > self.max_points:
            indices = np.linspace(0, len(points) - 1, self.max_points, dtype=int)
            points = points[indices]
            if colors is not None:
                colors = colors[indices]
        if colors is not None and np.max(colors) <= 1.0:
            colors = colors * 255.0
        rr.log(
            entity_path,
            rr.Points3D(
                positions=points / units_per_meter,
                colors=colors.astype(np.uint8) if colors is not None else None,
                radii=0.001,
            ),
        )

    def log_ram_artifacts(
        self,
        output_dir: Path,
        *,
        c2w_mm: np.ndarray | None = None,
        coordinate_unit: str = "mm",
    ) -> None:
        if coordinate_unit not in {"mm", "m"}:
            raise ValueError("coordinate_unit 必须是 'mm' 或 'm'。")
        units_per_meter = MM_PER_METER if coordinate_unit == "mm" else 1.0
        self.log_depth_file(
            "step2/vggt/depth",
            output_dir / "vggt_depth_ext.npy",
            units_per_meter=units_per_meter,
        )
        intrinsic_path = output_dir / "vggt_K.npy"
        intrinsic = (
            np.load(intrinsic_path, allow_pickle=False)
            if intrinsic_path.is_file()
            else None
        )
        main_image = _read_image_rgb(output_dir / "rgb_ext.png")
        if main_image is not None and intrinsic is not None:
            self.log_camera(
                c2w_mm=c2w_mm,
                intrinsic=intrinsic,
                resolution=(main_image.shape[1], main_image.shape[0]),
            )
        metadata_path = output_dir / "ram_objects_meta.npy"
        if metadata_path.is_file():
            metadata = np.load(metadata_path, allow_pickle=True).item()
            for object_id, object_meta in metadata.items():
                object_path = f"world/camera/extern/objects/{_safe_component_name(object_id)}"
                for primitive_name, primitive in object_meta.items():
                    primitive_array = np.asarray(primitive)
                    if primitive_array.shape == (4, 4):
                        self._log_transform(
                            f"{object_path}/{_safe_component_name(primitive_name)}",
                            primitive_array,
                            units_per_meter=units_per_meter,
                        )
                if "object_size" in object_meta:
                    self.log_json_document(
                        f"step2/objects/{_safe_component_name(object_id)}/metadata",
                        {
                            f"object_size_{coordinate_unit}": np.asarray(
                                object_meta["object_size"]
                            ).tolist(),
                            "primitive_names": list(object_meta.keys()),
                        },
                    )

        for cloud_path in sorted(output_dir.glob("*_point_cloud_rgb.npy")):
            entity_name = _safe_component_name(cloud_path.stem)
            self._log_xyzrgb_file(
                f"world/camera/extern/point_clouds/{entity_name}",
                cloud_path,
                units_per_meter=units_per_meter,
            )

        book_geometry = _read_json(
            output_dir / "book_grasp_geometry.json",
            None,
        )
        if isinstance(book_geometry, dict):
            self.log_json_document(
                "step2/objects/book/grasp_geometry",
                book_geometry,
            )
            center_mm = np.asarray(
                book_geometry.get("center_base_mm", []),
                dtype=float,
            )
            if center_mm.shape == (3,) and np.isfinite(center_mm).all():
                center_m = center_mm / MM_PER_METER
                rr.log(
                    "world/selected_book/center",
                    rr.Points3D(
                        [center_m],
                        colors=[[255, 215, 0]],
                        radii=0.018,
                        labels=["selected book"],
                    ),
                )
                self._log_oriented_book_bounds(book_geometry)

    def log_planning_artifacts(self, output_dir: Path) -> None:
        self.log_image_file(
            "step3/images/observation_labeling",
            output_dir / "rgb_ext_labeling.png",
        )
        self.log_image_file(
            "step3/images/concatenated_obs_ref",
            output_dir / "concatenated_obs_ref.png",
        )
        self.log_text_document(
            "step3/ram_description/with_primitives",
            _read_text(output_dir / "ram_objects_primitives_description.txt"),
        )
        self.log_text_document(
            "step3/ram_description/without_primitives",
            _read_text(output_dir / "ram_objects_description.txt"),
        )
        subtasks = _read_json(output_dir / "vlm_subtasks.json", [])
        actions = _read_json(output_dir / "vlm_actions.json", [])
        self.log_json_document("step3/subtasks/list", subtasks)
        self.log_json_document("step3/actions/list", actions)
        for action in actions:
            subtask_id = _safe_component_name(action.get("subtask_id", "unknown"))
            self.log_json_document(
                f"step3/actions/{subtask_id}/metadata",
                action,
            )

    def log_manual_planning_tasks(self, tasks: Iterable[dict[str, Any]]) -> None:
        """Visualize manual-planner target poses and their traversal order."""
        task_list = list(tasks)
        self.log_json_document("step4/manual_plan/tasks", task_list)
        target_positions_by_arm: dict[str, list[np.ndarray]] = {}
        arm_colors = {
            "right": RIGHT_ARM_COLOR,
            "left": LEFT_ARM_COLOR,
        }

        for index, task in enumerate(task_list):
            target_position = np.asarray(task.get("target_position", []), dtype=float)
            target_rotation = np.asarray(task.get("target_rotation", []), dtype=float)
            if target_position.size < 3 or target_rotation.size < 3:
                continue

            pose_mm_deg = np.concatenate(
                (target_position[:3], target_rotation[:3])
            )
            if not np.isfinite(pose_mm_deg).all():
                continue

            action = _safe_component_name(task.get("action", "target"))
            object_id = _safe_component_name(task.get("object", "unknown"))
            target_path = (
                f"world/manual_plan/targets/{index:02d}_{action}_{object_id}"
            )
            self._log_transform_mm(
                f"{target_path}/tool_pose",
                _pose_mm_deg_to_matrix(pose_mm_deg),
            )
            tool_pose_path = f"{target_path}/tool_pose"
            if task.get("arm") == "right":
                right_gripper_path = f"{tool_pose_path}/right_gripper"
                self._log_right_gripper_frame(right_gripper_path)
                self._log_tool_pose_axes(f"{right_gripper_path}/axes")
                self._log_right_gripper_glyph(
                    f"{right_gripper_path}/model"
                )
            else:
                left_gripper_path = f"{tool_pose_path}/left_gripper"
                self._log_tool_pose_axes(f"{left_gripper_path}/axes")
                self._log_gripper_glyph(
                    f"{left_gripper_path}/model",
                    color=LEFT_ARM_COLOR.tolist(),
                    thin=False,
                )
            self.log_json_document(f"{target_path}/metadata", task)
            arm = _safe_component_name(task.get("arm", "unknown"))
            arm_targets = target_positions_by_arm.setdefault(arm, [])
            if not arm_targets or not np.allclose(
                arm_targets[-1],
                target_position[:3],
            ):
                arm_targets.append(target_position[:3])

        for arm, target_positions in target_positions_by_arm.items():
            if len(target_positions) < 2:
                continue
            positions_m = np.asarray(target_positions) / MM_PER_METER
            color = arm_colors.get(arm, [180, 180, 180])
            rr.log(
                f"world/manual_plan/waypoint_paths/{arm}",
                rr.LineStrips3D(
                    [positions_m],
                    colors=color,
                    radii=0.003,
                ),
            )
            rr.log(
                f"world/manual_plan/waypoints/{arm}",
                rr.Points3D(
                    positions_m,
                    colors=color,
                    radii=0.012,
                    labels=[str(index + 1) for index in range(len(positions_m))],
                ),
            )
        self._log_workspace_slices_for_tasks(task_list)

    def log_grasp_candidates(
        self,
        *,
        object_id: str,
        arm: str,
        candidates: Iterable[dict[str, Any]],
        selected_candidate_index: int | None = None,
    ) -> None:
        """Render dense grasp candidates in green and the selected one in red.

        Candidate poses use the same millimeter/degree convention as manual
        planning. Both arms use a thin schematic gripper so the dense yaw
        alternatives remain readable; right-arm glyphs additionally include
        the fixed TCP-to-physical-gripper transform from the active URDF.
        """
        candidate_list = list(candidates)
        if arm not in {"left", "right"} or not candidate_list:
            return

        object_name = _safe_component_name(object_id)
        root_path = f"world/grasp_candidates/{object_name}/{arm}"

        def log_candidate(entity_path: str, candidate: dict[str, Any], color: list[int]) -> None:
            pose_mm_deg = np.asarray(candidate.get("pose_mm_deg", []), dtype=float)
            if pose_mm_deg.shape != (6,) or not np.isfinite(pose_mm_deg).all():
                return
            tool_pose_path = f"{entity_path}/tool_pose"
            self._log_transform_mm(
                tool_pose_path,
                _pose_mm_deg_to_matrix(pose_mm_deg),
            )
            gripper_path = f"{tool_pose_path}/gripper_model"
            if arm == "right":
                self._log_right_gripper_frame(gripper_path)
            self._log_gripper_glyph(
                f"{gripper_path}/thin_gripper",
                color=color,
                thin=True,
            )
            self.log_json_document(
                f"{entity_path}/metadata",
                {
                    **candidate,
                    "object": object_id,
                    "arm": arm,
                    "pose_mm_deg": pose_mm_deg.tolist(),
                },
            )

        for candidate_index, candidate in enumerate(candidate_list):
            yaw_deg = candidate.get("yaw_deg", candidate_index)
            log_candidate(
                f"{root_path}/all_candidates/{candidate_index:02d}_yaw_{yaw_deg}",
                candidate,
                [0, 220, 70],
            )

        if selected_candidate_index is None:
            return
        if 0 <= selected_candidate_index < len(candidate_list):
            candidate = candidate_list[selected_candidate_index]
            log_candidate(
                f"{root_path}/selected/{selected_candidate_index:03d}",
                candidate,
                [255, 0, 0],
            )

    def log_step4_plan(self, plan_path: Path) -> None:
        plan = _read_json(plan_path, None)
        if not plan:
            return
        self.log_json_document("step4/plan", plan)
        trajectory = np.asarray(plan.get("trajectory_mm_deg", []), dtype=float)
        if trajectory.ndim == 2 and trajectory.shape[1] >= 3:
            rr.log(
                "world/trajectory/planned",
                rr.LineStrips3D(
                    [trajectory[:, :3] / MM_PER_METER],
                    colors=[0, 150, 255],
                    radii=0.003,
                ),
            )

        for event in plan.get("constraint_events", []):
            event_id = _safe_component_name(event.get("event_id", "unknown"))
            pose = np.asarray(event.get("tip_pose_mm_deg", []), dtype=float)
            if pose.shape == (6,):
                matrix = _pose_mm_deg_to_matrix(pose)
                self._log_transform_mm(
                    f"step4/constraints/{event_id}/tip_pose",
                    matrix,
                )
                rr.log(
                    f"step4/constraints/{event_id}/jaw",
                    rr.Scalar(float(event.get("jaw", 0.0))),
                )
                rr.log(
                    f"step4/constraints/{event_id}/action",
                    rr.TextLog(str(event.get("action", ""))),
                )


def _pose_mm_deg_to_matrix(pose_mm_deg: np.ndarray) -> np.ndarray:
    """Convert RobotRAM xyz+Euler-deg pose to a homogeneous matrix."""
    from utils.transform_utils import pos_to_mat

    return pos_to_mat(np.asarray(pose_mm_deg, dtype=float))
