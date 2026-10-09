"""Runtime pitch correction for a movable head-mounted RGB camera.

The hand-eye calibration used by RobotRAM is a camera-to-base transform with
translation in millimetres.  This module estimates the one remaining head
pitch degree of freedom from a metric depth map of the horizontal table.  It
does not overwrite the original calibration file.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence
import uuid

import numpy as np


class HeadCameraCorrectionError(RuntimeError):
    """Raised when a safe runtime camera correction cannot be estimated."""


def _unit_vector(vector: Sequence[float], name: str) -> np.ndarray:
    value = np.asarray(vector, dtype=float).reshape(3)
    norm = float(np.linalg.norm(value))
    if not np.isfinite(value).all() or norm < 1e-9:
        raise HeadCameraCorrectionError(f"{name} must be a finite non-zero vector")
    return value / norm


def rotation_about_axis(axis: Sequence[float], angle_rad: float) -> np.ndarray:
    """Return an active right-handed rotation about an axis."""
    x, y, z = _unit_vector(axis, "rotation axis")
    skew = np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )
    identity = np.eye(3, dtype=float)
    return (
        identity
        + math.sin(angle_rad) * skew
        + (1.0 - math.cos(angle_rad)) * (skew @ skew)
    )


def _angle_degrees(first: Sequence[float], second: Sequence[float]) -> float:
    first_unit = _unit_vector(first, "first angle vector")
    second_unit = _unit_vector(second, "second angle vector")
    cosine = float(np.clip(np.dot(first_unit, second_unit), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _validate_roi(
    roi: Sequence[float],
) -> tuple[float, float, float, float]:
    if len(roi) != 4:
        raise HeadCameraCorrectionError(
            "table ROI must contain x_min x_max y_min y_max"
        )
    x_min, x_max, y_min, y_max = (float(value) for value in roi)
    if not (
        0.0 <= x_min < x_max <= 1.0
        and 0.0 <= y_min < y_max <= 1.0
    ):
        raise HeadCameraCorrectionError(
            "table ROI values must satisfy 0<=min<max<=1"
        )
    return x_min, x_max, y_min, y_max


def backproject_depth_roi(
    depth_m: np.ndarray,
    intrinsic: np.ndarray,
    valid_mask: np.ndarray | None = None,
    roi: Sequence[float] = (0.08, 0.92, 0.30, 0.95),
    depth_range_m: Sequence[float] = (0.25, 2.0),
    max_points: int = 60_000,
    random_seed: int = 7,
) -> np.ndarray:
    """Back-project valid metric depth pixels to OpenCV camera coordinates."""
    depth = np.asarray(depth_m, dtype=float)
    camera_matrix = np.asarray(intrinsic, dtype=float)
    if depth.ndim != 2:
        raise HeadCameraCorrectionError(
            f"depth map must be HxW, got shape {depth.shape}"
        )
    if camera_matrix.shape != (3, 3):
        raise HeadCameraCorrectionError(
            f"intrinsic matrix must be 3x3, got {camera_matrix.shape}"
        )
    if not np.isfinite(camera_matrix).all():
        raise HeadCameraCorrectionError("intrinsic matrix contains non-finite values")

    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise HeadCameraCorrectionError("camera focal lengths must be positive")

    min_depth, max_depth = (float(value) for value in depth_range_m)
    if not 0.0 < min_depth < max_depth:
        raise HeadCameraCorrectionError("invalid depth range")
    if max_points < 3:
        raise HeadCameraCorrectionError("max_points must be at least 3")

    height, width = depth.shape
    x_min, x_max, y_min, y_max = _validate_roi(roi)
    left = int(math.floor(x_min * width))
    right = int(math.ceil(x_max * width))
    top = int(math.floor(y_min * height))
    bottom = int(math.ceil(y_max * height))

    selected = np.zeros(depth.shape, dtype=bool)
    selected[top:bottom, left:right] = True
    selected &= np.isfinite(depth)
    selected &= depth >= min_depth
    selected &= depth <= max_depth
    if valid_mask is not None:
        mask = np.asarray(valid_mask, dtype=bool)
        if mask.shape != depth.shape:
            raise HeadCameraCorrectionError(
                f"depth mask shape {mask.shape} does not match {depth.shape}"
            )
        selected &= mask

    rows, columns = np.nonzero(selected)
    if len(rows) < 500:
        raise HeadCameraCorrectionError(
            f"only {len(rows)} valid depth pixels remain in the table ROI"
        )

    if len(rows) > max_points:
        rng = np.random.default_rng(random_seed)
        indices = rng.choice(len(rows), size=max_points, replace=False)
        rows = rows[indices]
        columns = columns[indices]

    z = depth[rows, columns]
    x = (columns.astype(float) - cx) * z / fx
    y = (rows.astype(float) - cy) * z / fy
    points = np.column_stack((x, y, z))
    if not np.isfinite(points).all():
        raise HeadCameraCorrectionError("back-projected points are not finite")
    return points


def fit_plane_ransac(
    points: np.ndarray,
    distance_threshold_m: float = 0.008,
    iterations: int = 240,
    min_inlier_ratio: float = 0.25,
    random_seed: int = 11,
) -> dict[str, Any]:
    """Fit the dominant plane and refine it with SVD."""
    point_array = np.asarray(points, dtype=float)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise HeadCameraCorrectionError(
            f"plane points must be Nx3, got {point_array.shape}"
        )
    if len(point_array) < 500:
        raise HeadCameraCorrectionError(
            f"at least 500 points are required, got {len(point_array)}"
        )
    if distance_threshold_m <= 0.0:
        raise HeadCameraCorrectionError("plane threshold must be positive")
    if iterations <= 0:
        raise HeadCameraCorrectionError("RANSAC iterations must be positive")
    if not 0.0 < min_inlier_ratio <= 1.0:
        raise HeadCameraCorrectionError("min_inlier_ratio must be in (0, 1]")

    rng = np.random.default_rng(random_seed)
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        sample = point_array[rng.choice(len(point_array), size=3, replace=False)]
        candidate_normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        normal_norm = float(np.linalg.norm(candidate_normal))
        if normal_norm < 1e-8:
            continue
        candidate_normal /= normal_norm
        candidate_offset = -float(np.dot(candidate_normal, sample[0]))
        distances = np.abs(point_array @ candidate_normal + candidate_offset)
        inlier_mask = distances <= distance_threshold_m
        inlier_count = int(np.count_nonzero(inlier_mask))
        if inlier_count > best_count:
            best_count = inlier_count
            best_mask = inlier_mask

    if best_mask is None:
        raise HeadCameraCorrectionError("RANSAC did not find a valid plane")
    inlier_ratio = best_count / float(len(point_array))
    if inlier_ratio < min_inlier_ratio:
        raise HeadCameraCorrectionError(
            f"dominant-plane inlier ratio {inlier_ratio:.3f} is below "
            f"{min_inlier_ratio:.3f}"
        )

    inliers = point_array[best_mask]
    centroid = np.mean(inliers, axis=0)
    _, singular_values, right_vectors = np.linalg.svd(
        inliers - centroid,
        full_matrices=False,
    )
    normal = _unit_vector(right_vectors[-1], "fitted plane normal")
    offset = -float(np.dot(normal, centroid))
    distances = np.abs(point_array @ normal + offset)
    refined_mask = distances <= distance_threshold_m
    refined_inliers = point_array[refined_mask]
    if len(refined_inliers) >= 500:
        centroid = np.mean(refined_inliers, axis=0)
        _, singular_values, right_vectors = np.linalg.svd(
            refined_inliers - centroid,
            full_matrices=False,
        )
        normal = _unit_vector(right_vectors[-1], "refined plane normal")
        offset = -float(np.dot(normal, centroid))
        inliers = refined_inliers

    residuals = inliers @ normal + offset
    return {
        "normal": normal,
        "centroid_m": centroid,
        "offset_m": offset,
        "inlier_count": int(len(inliers)),
        "sample_count": int(len(point_array)),
        "inlier_ratio": float(len(inliers) / len(point_array)),
        "rms_residual_mm": float(np.sqrt(np.mean(residuals**2)) * 1000.0),
        "singular_values": singular_values,
    }


def estimate_pitch_delta(
    rotation_base_from_camera: np.ndarray,
    table_normal_camera: Sequence[float],
    pitch_axis_base: Sequence[float] | None = None,
    pitch_limit_deg: float = 20.0,
    search_step_deg: float = 0.01,
) -> dict[str, Any]:
    """Estimate the base-frame rotation that makes the table horizontal.

    When the mechanical pitch axis is known, the estimate is constrained to
    that axis. Otherwise, use the shortest leveling rotation. The latter is
    preferable to assuming the optical X axis is the mechanical pivot axis,
    because the camera may be installed with a small roll offset.
    """
    rotation = np.asarray(rotation_base_from_camera, dtype=float)
    if rotation.shape != (3, 3):
        raise HeadCameraCorrectionError(
            f"camera rotation must be 3x3, got {rotation.shape}"
        )
    if pitch_limit_deg <= 0.0 or search_step_deg <= 0.0:
        raise HeadCameraCorrectionError("pitch search limits must be positive")

    normal_camera = _unit_vector(table_normal_camera, "table normal")
    normal_base_before = rotation @ normal_camera
    if normal_base_before[2] < 0.0:
        normal_camera = -normal_camera
        normal_base_before = -normal_base_before

    world_up = np.array([0.0, 0.0, 1.0])
    if pitch_axis_base is None:
        cross_product = np.cross(normal_base_before, world_up)
        sine = float(np.linalg.norm(cross_product))
        cosine = float(np.clip(np.dot(normal_base_before, world_up), -1.0, 1.0))
        best_delta_deg = math.degrees(math.atan2(sine, cosine))
        if best_delta_deg > pitch_limit_deg:
            raise HeadCameraCorrectionError(
                f"required leveling rotation {best_delta_deg:.2f} deg exceeds "
                f"the {pitch_limit_deg:.2f} deg limit"
            )
        if sine < 1e-9:
            axis = _unit_vector(rotation[:, 0], "fallback pitch axis")
            best_rotation = np.eye(3, dtype=float)
        else:
            axis = cross_product / sine
            best_rotation = rotation_about_axis(
                axis,
                math.radians(best_delta_deg),
            )
        best_normal = best_rotation @ normal_base_before
        best_angle_deg = _angle_degrees(best_normal, world_up)
        axis_source = "table_leveling"
    else:
        axis = _unit_vector(pitch_axis_base, "pitch axis")
        sample_count = max(
            3,
            int(math.ceil(2.0 * pitch_limit_deg / search_step_deg)) + 1,
        )
        candidates_deg = np.linspace(
            -pitch_limit_deg,
            pitch_limit_deg,
            sample_count,
        )
        best_angle_deg = None
        best_delta_deg = None
        best_rotation = None
        best_normal = None
        for delta_deg in candidates_deg:
            delta_rotation = rotation_about_axis(
                axis,
                math.radians(float(delta_deg)),
            )
            corrected_normal = delta_rotation @ normal_base_before
            residual_angle = _angle_degrees(corrected_normal, world_up)
            if best_angle_deg is None or residual_angle < best_angle_deg:
                best_angle_deg = residual_angle
                best_delta_deg = float(delta_deg)
                best_rotation = delta_rotation
                best_normal = corrected_normal
        assert best_rotation is not None
        axis_source = "configured_pitch_axis"

    return {
        "delta_pitch_deg": best_delta_deg,
        "delta_rotation_base": best_rotation,
        "pitch_axis_base": axis,
        "pitch_axis_source": axis_source,
        "normal_camera": normal_camera,
        "normal_base_before": normal_base_before,
        "normal_base_after": best_normal,
        "tilt_before_deg": _angle_degrees(normal_base_before, world_up),
        "tilt_after_deg": float(best_angle_deg),
    }


def _sample_anchor_depth(
    depth_m: np.ndarray,
    pixel: Sequence[float],
    radius_px: int = 4,
) -> float:
    u, v = (int(round(float(value))) for value in pixel)
    height, width = depth_m.shape
    left = max(0, u - radius_px)
    right = min(width, u + radius_px + 1)
    top = max(0, v - radius_px)
    bottom = min(height, v + radius_px + 1)
    if left >= right or top >= bottom:
        raise HeadCameraCorrectionError(f"anchor pixel {(u, v)} is outside the image")
    patch = np.asarray(depth_m[top:bottom, left:right], dtype=float)
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        raise HeadCameraCorrectionError(
            f"anchor pixel {(u, v)} has no valid nearby depth"
        )
    return float(np.median(valid))


def _anchor_camera_point(
    depth_m: np.ndarray,
    intrinsic: np.ndarray,
    pixel: Sequence[float],
) -> np.ndarray:
    u, v = (float(value) for value in pixel)
    if not math.isfinite(u) or not math.isfinite(v):
        raise HeadCameraCorrectionError("anchor pixel coordinates must be finite")
    z = _sample_anchor_depth(depth_m, pixel)
    fx = float(intrinsic[0, 0])
    fy = float(intrinsic[1, 1])
    cx = float(intrinsic[0, 2])
    cy = float(intrinsic[1, 2])
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])


def correct_head_camera_calibration(
    calibration: Mapping[str, Any],
    depth_m: np.ndarray,
    intrinsic: np.ndarray,
    valid_mask: np.ndarray | None = None,
    *,
    table_z_mm: float = 0.0,
    roi: Sequence[float] = (0.08, 0.92, 0.30, 0.95),
    depth_range_m: Sequence[float] = (0.25, 2.0),
    pitch_axis_base: Sequence[float] | None = None,
    pitch_limit_deg: float = 20.0,
    pivot_base_mm: Sequence[float] | None = None,
    anchors: Iterable[Sequence[float]] | None = None,
    xy_offset_mm: Sequence[float] = (0.0, 0.0),
    max_tilt_residual_deg: float = 2.0,
    max_table_height_shift_mm: float = 150.0,
    max_independent_table_height_error_mm: float = 15.0,
    max_manual_xy_offset_mm: float = 30.0,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Estimate and apply a safe one-axis correction to a calibration dict.

    Each optional anchor is ``(pixel_u, pixel_v, base_x_mm, base_y_mm,
    base_z_mm)``. One anchor fixes all translation components after pitch has
    been estimated; multiple anchors are averaged and provide a residual check.
    """
    rotation = np.asarray(calibration["R"], dtype=float)
    translation = np.asarray(calibration["T"], dtype=float).reshape(3)
    table_z_mm = float(table_z_mm)
    if rotation.shape != (3, 3):
        raise HeadCameraCorrectionError("calibration R must be 3x3")
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise HeadCameraCorrectionError("calibration contains non-finite values")
    if not math.isfinite(table_z_mm):
        raise HeadCameraCorrectionError("table Z must be finite")

    points = backproject_depth_roi(
        depth_m,
        intrinsic,
        valid_mask=valid_mask,
        roi=roi,
        depth_range_m=depth_range_m,
    )
    plane = fit_plane_ransac(points)
    if plane["inlier_ratio"] < 0.45:
        raise HeadCameraCorrectionError(
            f"dominant-plane inlier ratio {plane['inlier_ratio']:.3f} is below "
            "the execution-quality threshold 0.450"
        )
    if plane["rms_residual_mm"] > 6.0:
        raise HeadCameraCorrectionError(
            f"table plane RMS residual {plane['rms_residual_mm']:.1f}mm exceeds 6mm"
        )
    pitch = estimate_pitch_delta(
        rotation,
        plane["normal"],
        pitch_axis_base=pitch_axis_base,
        pitch_limit_deg=pitch_limit_deg,
    )
    if pitch["tilt_after_deg"] > max_tilt_residual_deg:
        raise HeadCameraCorrectionError(
            f"table still tilts {pitch['tilt_after_deg']:.2f} deg after the "
            f"one-axis correction (limit {max_tilt_residual_deg:.2f} deg)"
        )
    if abs(pitch["delta_pitch_deg"]) >= pitch_limit_deg - 0.02:
        raise HeadCameraCorrectionError(
            "best pitch correction lies on the configured search boundary"
        )

    delta_rotation = pitch["delta_rotation_base"]
    corrected_rotation = delta_rotation @ rotation
    if pivot_base_mm is not None:
        pivot = np.asarray(pivot_base_mm, dtype=float).reshape(3)
        if not np.isfinite(pivot).all():
            raise HeadCameraCorrectionError("camera pivot contains non-finite values")
        corrected_translation = pivot + delta_rotation @ (translation - pivot)
        translation_method = "pitch_pivot"
    else:
        corrected_translation = translation.copy()
        translation_method = "table_height_only"

    parsed_anchors = list(anchors or [])
    anchor_residuals_mm: list[float] = []
    if parsed_anchors:
        translation_candidates = []
        anchor_camera_points = []
        anchor_base_points = []
        for anchor in parsed_anchors:
            if len(anchor) != 5:
                raise HeadCameraCorrectionError(
                    "each anchor must contain u v base_x base_y base_z"
                )
            camera_point_m = _anchor_camera_point(
                np.asarray(depth_m, dtype=float),
                np.asarray(intrinsic, dtype=float),
                anchor[:2],
            )
            base_point_mm = np.asarray(anchor[2:], dtype=float)
            if not np.isfinite(base_point_mm).all():
                raise HeadCameraCorrectionError(
                    "anchor base coordinates must be finite"
                )
            translation_candidates.append(
                base_point_mm - corrected_rotation @ (camera_point_m * 1000.0)
            )
            anchor_camera_points.append(camera_point_m)
            anchor_base_points.append(base_point_mm)
        if len(parsed_anchors) >= 2:
            camera_points = np.asarray(anchor_camera_points) * 1000.0
            base_points = np.asarray(anchor_base_points)
            camera_span_mm = float(
                np.max(
                    np.linalg.norm(
                        camera_points[:, None, :] - camera_points[None, :, :],
                        axis=2,
                    )
                )
            )
            base_span_mm = float(
                np.max(
                    np.linalg.norm(
                        base_points[:, None, :] - base_points[None, :, :],
                        axis=2,
                    )
                )
            )
            if camera_span_mm < 50.0 or base_span_mm < 50.0:
                raise HeadCameraCorrectionError(
                    "known-point anchors must include two distinct points at "
                    "least 50mm apart"
                )
        corrected_translation = np.median(
            np.asarray(translation_candidates),
            axis=0,
        )
        for camera_point_m, base_point_mm in zip(
            anchor_camera_points,
            anchor_base_points,
        ):
            estimated_base = (
                corrected_rotation @ (camera_point_m * 1000.0)
                + corrected_translation
            )
            anchor_residuals_mm.append(
                float(np.linalg.norm(estimated_base - base_point_mm))
            )
        if max(anchor_residuals_mm) > 20.0:
            raise HeadCameraCorrectionError(
                f"anchor residual {max(anchor_residuals_mm):.1f}mm exceeds 20mm"
            )
        translation_method = "known_point_anchors"
    elif pivot_base_mm is None:
        table_centroid_camera_mm = np.asarray(plane["centroid_m"]) * 1000.0
        estimated_table_z = float(
            (
                corrected_rotation @ table_centroid_camera_mm
                + corrected_translation
            )[2]
        )
        table_height_shift = float(table_z_mm - estimated_table_z)
        if abs(table_height_shift) > max_table_height_shift_mm:
            raise HeadCameraCorrectionError(
                f"required table-height shift {table_height_shift:.1f}mm exceeds "
                f"{max_table_height_shift_mm:.1f}mm"
            )
        corrected_translation[2] += table_height_shift

    xy_offset = np.asarray(xy_offset_mm, dtype=float).reshape(2)
    if not np.isfinite(xy_offset).all():
        raise HeadCameraCorrectionError("XY offset contains non-finite values")
    if float(np.linalg.norm(xy_offset)) > max_manual_xy_offset_mm:
        raise HeadCameraCorrectionError(
            f"manual XY offset norm {np.linalg.norm(xy_offset):.1f}mm exceeds "
            f"{max_manual_xy_offset_mm:.1f}mm"
        )
    corrected_translation[:2] += xy_offset

    table_centroid_camera_mm = np.asarray(plane["centroid_m"]) * 1000.0
    corrected_table_centroid_base = (
        corrected_rotation @ table_centroid_camera_mm + corrected_translation
    )
    table_height_error_mm = float(corrected_table_centroid_base[2] - table_z_mm)
    if (
        (parsed_anchors or pivot_base_mm is not None)
        and abs(table_height_error_mm) > max_independent_table_height_error_mm
    ):
        raise HeadCameraCorrectionError(
            f"independently derived transform places the table "
            f"{table_height_error_mm:.1f}mm "
            "away from its configured height "
            f"(limit {max_independent_table_height_error_mm:.1f}mm)"
        )

    corrected_transform = np.eye(4, dtype=float)
    corrected_transform[:3, :3] = corrected_rotation
    corrected_transform[:3, 3] = corrected_translation
    corrected = dict(calibration)
    corrected["R"] = corrected_rotation
    corrected["T"] = corrected_translation.reshape(3, 1)
    corrected["T_base_cam"] = corrected_transform

    diagnostics = {
        "status": "applied",
        "delta_pitch_deg": float(pitch["delta_pitch_deg"]),
        "pitch_axis_base": pitch["pitch_axis_base"].tolist(),
        "pitch_axis_source": pitch["pitch_axis_source"],
        "table_normal_camera": pitch["normal_camera"].tolist(),
        "table_normal_base_before": pitch["normal_base_before"].tolist(),
        "table_normal_base_after": pitch["normal_base_after"].tolist(),
        "table_tilt_before_deg": float(pitch["tilt_before_deg"]),
        "table_tilt_after_deg": float(pitch["tilt_after_deg"]),
        "table_centroid_camera_m": plane["centroid_m"].tolist(),
        "table_centroid_base_mm": corrected_table_centroid_base.tolist(),
        "table_height_target_mm": float(table_z_mm),
        "table_height_error_mm": table_height_error_mm,
        "plane_inlier_count": int(plane["inlier_count"]),
        "plane_sample_count": int(plane["sample_count"]),
        "plane_inlier_ratio": float(plane["inlier_ratio"]),
        "plane_rms_residual_mm": float(plane["rms_residual_mm"]),
        "translation_method": translation_method,
        "anchor_count": len(parsed_anchors),
        "anchor_residuals_mm": anchor_residuals_mm,
        "manual_xy_offset_mm": xy_offset.tolist(),
        "original_T_base_cam_mm": np.block(
            [
                [rotation, translation.reshape(3, 1)],
                [np.zeros((1, 3)), np.ones((1, 1))],
            ]
        ).tolist(),
        "corrected_T_base_cam_mm": corrected_transform.tolist(),
        "warning": (
            None
            if parsed_anchors or pivot_base_mm is not None
            else (
                "No pivot or known-point anchor was supplied; pitch and table "
                "height are corrected, but translation parallel to the table "
                "cannot be observed from one plane."
            )
        ),
    }
    return corrected, diagnostics


def save_correction_outputs(
    source_calibration_path: str | Path,
    corrected_calibration: Mapping[str, Any],
    diagnostics: Mapping[str, Any],
    report_path: str | Path,
    corrected_calibration_path: str | Path,
) -> None:
    """Save diagnostics and a compatible corrected calibration JSON."""
    report_file = Path(report_path)
    calibration_file = Path(corrected_calibration_path)
    with Path(source_calibration_path).open("r", encoding="utf-8") as source:
        payload = json.load(source)
    if "left" not in payload or not isinstance(payload["left"], dict):
        raise HeadCameraCorrectionError(
            "source calibration JSON does not contain a left camera section"
        )
    correction_id = uuid.uuid4().hex
    report_payload = dict(diagnostics)
    report_payload["correction_id"] = correction_id
    payload["left"]["R"] = np.asarray(
        corrected_calibration["R"],
        dtype=float,
    ).tolist()
    payload["left"]["T"] = np.asarray(
        corrected_calibration["T"],
        dtype=float,
    ).reshape(3).tolist()
    # The corrected calibration is self-describing even if writing the
    # convenience report is interrupted.
    payload["_head_camera_runtime_correction"] = report_payload

    # Serialize both complete payloads before touching either destination.
    # allow_nan=False prevents a malformed correction from becoming a file
    # that the normal calibration loader would later accept.
    report_text = json.dumps(
        report_payload,
        indent=2,
        ensure_ascii=False,
        allow_nan=False,
    )
    calibration_text = json.dumps(
        payload,
        indent=2,
        ensure_ascii=False,
        allow_nan=False,
    )

    report_file.parent.mkdir(parents=True, exist_ok=True)
    calibration_file.parent.mkdir(parents=True, exist_ok=True)
    report_temporary = report_file.with_name(
        report_file.name + f".tmp.{correction_id}"
    )
    calibration_temporary = calibration_file.with_name(
        calibration_file.name + f".tmp.{correction_id}"
    )
    try:
        calibration_temporary.write_text(calibration_text, encoding="utf-8")
        report_temporary.write_text(report_text, encoding="utf-8")
        os.replace(calibration_temporary, calibration_file)
        # The report is the commit marker and is replaced last.
        os.replace(report_temporary, report_file)
    finally:
        for temporary in (report_temporary, calibration_temporary):
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
