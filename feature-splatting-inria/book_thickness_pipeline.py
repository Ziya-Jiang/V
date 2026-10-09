#!/usr/bin/env python3
"""Measure book-spine thickness from a multi-frame Record3D RGB-D capture.

The high-resolution RGB masks supply the left/right spine boundaries. Record3D
LiDAR supplies metric scale, confidence, camera intrinsics, and camera poses.
Measurements from multiple viewpoints are associated in 3D and robustly fused.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import liblzfse
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation


DEFAULT_SAM3_REPO = Path("/mnt/ssd/user/wx/SAM3_Inference")
DEFAULT_SAM3_CHECKPOINT = Path(
    "/home/vlmdemo/.cache/huggingface/hub/models--facebook--sam3/"
    "snapshots/3c879f39826c281e95690f02c7821c4de09afae7/sam3.pt"
)
DEFAULT_FRAMES = (0, 12, 24, 36, 48, 60, 71)
SHELF_BANDS = ((0.18, 0.45), (0.43, 0.70), (0.66, 0.94))
SHELF_COLORS_RGB = ((0, 205, 235), (255, 151, 20), (30, 210, 80))
GRASP_COLORS_RGB = {
    "finger": (255, 214, 40),
    "body": (40, 200, 255),
    "contact": (255, 70, 140),
    "approach": (255, 130, 40),
}
GRASP_AXIS_COLORS_RGB = ((235, 70, 70), (60, 215, 95), (80, 150, 255))
GRIPPER_PAD_HEIGHT_MM = 60.0


@dataclass
class SpineGeometry:
    center_uv: np.ndarray
    long_axis_uv: np.ndarray
    transverse_axis_uv: np.ndarray
    left_uv: np.ndarray
    right_uv: np.ndarray
    left_edge_uv: np.ndarray
    right_edge_uv: np.ndarray
    width_px: float
    length_px: float
    roll_deg: float
    polygon_xy: list[list[int]]


@dataclass
class PlaneCandidate:
    normal: np.ndarray
    offset: float
    residual_m: float
    inlier_indices: np.ndarray
    inlier_ratio: float
    longitudinal_coverage: float


@dataclass
class Detection:
    frame: int
    shelf: int
    score: float
    mask: np.ndarray
    box_xyxy: np.ndarray
    geometry: SpineGeometry
    depth_m: float = math.nan
    depth_mad_m: float = math.nan
    depth_samples: int = 0
    high_confidence_samples: int = 0
    plane_normal: np.ndarray | None = None
    plane_offset: float = math.nan
    plane_residual_m: float = math.nan
    plane_tilt_deg: float = math.nan
    used_local_plane: bool = False
    thickness_mm: float = math.nan
    metric_per_pixel_mm: float = math.nan
    camera_point: np.ndarray | None = None
    world_point: np.ndarray | None = None
    surface_points: np.ndarray | None = None
    surface_pixels_uv: np.ndarray | None = None
    plane_candidates: list[PlaneCandidate] = field(default_factory=list)
    spine_plane_inlier_ratio: float = math.nan
    plane_consensus_angle_deg: float = math.nan
    side_face_detected: bool = False
    side_face_fraction: float = 0.0
    view_angle_deg: float = math.nan
    boundary_corrected: bool = False
    boundary_trim_left_px: float = 0.0
    boundary_trim_right_px: float = 0.0
    silhouette_thickness_mm: float = math.nan
    side_trim_mm: float = 0.0
    measurement_rejection_reason: str | None = None


@dataclass
class BookTrack:
    book_id: str
    shelf: int
    reference: Detection
    measurements: list[Detection] = field(default_factory=list)
    match_residual_px: list[float] = field(default_factory=list)


@dataclass
class GraspPose:
    """Parallel-jaw grasp for one book, expressed in the reference camera frame.

    The rotation columns are the gripper closing axis, the spine axis pointing
    to the top of the book, and the approach axis pointing into the shelf.
    """

    book_id: str
    shelf: int
    quality: str
    rotation: np.ndarray
    tcp: np.ndarray
    contact_center: np.ndarray
    pregrasp: np.ndarray
    left_contact: np.ndarray
    right_contact: np.ndarray
    opening_mm: float
    thickness_mm: float
    reference_thickness_mm: float
    spine_span_mm: float

    @property
    def close_axis(self) -> np.ndarray:
        return self.rotation[:, 0]

    @property
    def spine_axis(self) -> np.ndarray:
        return self.rotation[:, 1]

    @property
    def approach_axis(self) -> np.ndarray:
        return self.rotation[:, 2]


@dataclass
class GraspExport:
    poses: list[GraspPose]
    intrinsic: np.ndarray
    transform_wc: np.ndarray
    insertion_mm: float
    approach_mm: float
    spine_offset_mm: float
    finger_length_mm: float
    requested_book_id: str | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure each visible book spine and recommend gripper opening."
    )
    parser.add_argument(
        "capture",
        nargs="?",
        type=Path,
        help="Extracted Record3D capture (not needed with --rgb/--depth/--intrinsics)",
    )
    parser.add_argument("--rgb", type=Path, help="Single-frame RGB image path")
    parser.add_argument(
        "--depth",
        type=Path,
        help="Single-frame depth image path (.png/.tif/.npy/.npz/.exr)",
    )
    parser.add_argument(
        "--intrinsics",
        type=Path,
        help="Camera intrinsics path (.json/.txt/.npy/.npz/.yaml/.xml)",
    )
    parser.add_argument(
        "--depth-scale",
        type=float,
        help=(
            "Metres per depth value; default is 0.001 for integer depth "
            "images and 1.0 for floating-point depth"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help=(
            "Output directory (default: CAPTURE/book_thickness_analysis or "
            "RGB_STEM_book_thickness for generic input)"
        ),
    )
    parser.add_argument("--reference-frame", type=int, default=24)
    parser.add_argument(
        "--single-frame-rgbd",
        action="store_true",
        help=(
            "Use only the reference RGB/depth frame; do not read confidence "
            "or use camera poses"
        ),
    )
    parser.add_argument(
        "--frames",
        default=",".join(map(str, DEFAULT_FRAMES)),
        help="Comma-separated frames used for multi-view fusion",
    )
    parser.add_argument("--prompt", default="book")
    parser.add_argument(
        "--detector-threshold",
        type=float,
        default=0.10,
        help="Low SAM3 processor threshold before stricter filtering",
    )
    parser.add_argument(
        "--min-score",
        type=float,
        default=0.50,
        help="Minimum accepted SAM3 confidence",
    )
    parser.add_argument("--nms-iou", type=float, default=0.40)
    parser.add_argument(
        "--match-radius-px",
        type=float,
        default=60.0,
        help="Maximum reprojection residual when matching views",
    )
    parser.add_argument(
        "--gripper-clearance-mm",
        type=float,
        default=3.0,
        help="Extra opening beyond the 95%% upper confidence bound",
    )
    parser.add_argument(
        "--grasp-insertion-mm",
        type=float,
        default=20.0,
        help="Fingertip depth past the spine surface at the grasp pose",
    )
    parser.add_argument(
        "--grasp-approach-mm",
        type=float,
        default=70.0,
        help="Pre-grasp standoff in front of the spine surface",
    )
    parser.add_argument(
        "--grasp-spine-offset-mm",
        type=float,
        default=0.0,
        help=(
            "Grasp height along the spine measured from the thickness "
            "cross-section; positive moves towards the top of the book"
        ),
    )
    parser.add_argument(
        "--grasp-finger-mm",
        type=float,
        default=45.0,
        help="Finger length drawn in the grasp visualisations",
    )
    parser.add_argument(
        "--book-id",
        "--grasp-book-id",
        dest="book_id",
        help=(
            "Generate and visualise a grasp pose only for this reference book "
            "ID, for example S2-10"
        ),
    )
    parser.add_argument("--sam3-repo", type=Path, default=DEFAULT_SAM3_REPO)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_SAM3_CHECKPOINT)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()
    generic_inputs = (args.rgb, args.depth, args.intrinsics)
    if any(value is not None for value in generic_inputs) and not all(
        value is not None for value in generic_inputs
    ):
        parser.error("--rgb, --depth and --intrinsics must be provided together")
    if all(value is not None for value in generic_inputs) and args.capture is not None:
        parser.error("do not provide CAPTURE together with generic RGB-D paths")
    if not all(value is not None for value in generic_inputs) and args.capture is None:
        parser.error("provide either CAPTURE or --rgb/--depth/--intrinsics")
    if args.depth_scale is not None and args.depth_scale <= 0:
        parser.error("--depth-scale must be greater than zero")
    if args.grasp_insertion_mm < 0:
        parser.error("--grasp-insertion-mm must not be negative")
    if args.grasp_approach_mm <= 0:
        parser.error("--grasp-approach-mm must be greater than zero")
    if args.grasp_finger_mm <= 0:
        parser.error("--grasp-finger-mm must be greater than zero")
    return args


def parse_frame_indices(text: str, frame_count: int, reference: int) -> list[int]:
    indices = {int(value.strip()) for value in text.split(",") if value.strip()}
    indices.add(reference)
    invalid = sorted(index for index in indices if index < 0 or index >= frame_count)
    if invalid:
        raise ValueError(f"Frame indices outside [0, {frame_count - 1}]: {invalid}")
    return sorted(indices)


def to_numpy(value: Any) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def largest_filled_mask(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    if not contours:
        raise ValueError("Empty instance mask")
    contour = max(contours, key=cv2.contourArea)
    filled = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(filled, [contour], -1, 1, thickness=cv2.FILLED)
    return filled.astype(bool), contour[:, 0].astype(np.float64)


def contour_polygon(contour: np.ndarray) -> list[list[int]]:
    contour_cv = contour.reshape(-1, 1, 2).astype(np.float32)
    perimeter = cv2.arcLength(contour_cv, True)
    approximated = cv2.approxPolyDP(
        contour_cv, max(1.0, perimeter * 0.0012), True
    )
    return np.rint(approximated[:, 0]).astype(int).tolist()


def fit_spine_geometry(mask: np.ndarray) -> tuple[np.ndarray, SpineGeometry]:
    filled, contour = largest_filled_mask(mask)
    y, x = np.nonzero(filled)
    points = np.column_stack((x, y)).astype(np.float64)
    center = np.mean(points, axis=0)
    centered = points - center
    covariance = centered.T @ centered / max(len(points) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    long_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    if long_axis[1] < 0:
        long_axis *= -1
    transverse = np.array([long_axis[1], -long_axis[0]])
    if transverse[0] < 0:
        transverse *= -1

    longitudinal = centered @ long_axis
    transverse_coordinate = centered @ transverse
    s_min, s_max = np.percentile(longitudinal, (3, 97))
    usable_min = s_min + 0.13 * (s_max - s_min)
    usable_max = s_max - 0.13 * (s_max - s_min)
    bin_edges = np.linspace(usable_min, usable_max, 33)

    samples_s: list[float] = []
    left_t: list[float] = []
    right_t: list[float] = []
    for lower, upper in zip(bin_edges[:-1], bin_edges[1:]):
        selected = transverse_coordinate[
            (longitudinal >= lower) & (longitudinal < upper)
        ]
        if selected.size < 8:
            continue
        samples_s.append((lower + upper) * 0.5)
        left_t.append(float(np.percentile(selected, 1)))
        right_t.append(float(np.percentile(selected, 99)))

    if len(samples_s) < 8:
        raise ValueError("Insufficient spine cross-sections")

    samples_s_array = np.asarray(samples_s)
    left_array = np.asarray(left_t)
    right_array = np.asarray(right_t)
    left_fit = np.polyfit(samples_s_array, left_array, 1)
    right_fit = np.polyfit(samples_s_array, right_array, 1)
    s_mid = float(np.median(samples_s_array))
    left_mid = float(np.polyval(left_fit, s_mid))
    right_mid = float(np.polyval(right_fit, s_mid))
    widths = right_array - left_array
    width_px = float(np.median(widths))

    left_uv = center + long_axis * s_mid + transverse * left_mid
    right_uv = center + long_axis * s_mid + transverse * right_mid
    edge_s = np.asarray([usable_min, usable_max])
    left_edge = (
        center[None, :]
        + edge_s[:, None] * long_axis[None, :]
        + np.polyval(left_fit, edge_s)[:, None] * transverse[None, :]
    )
    right_edge = (
        center[None, :]
        + edge_s[:, None] * long_axis[None, :]
        + np.polyval(right_fit, edge_s)[:, None] * transverse[None, :]
    )
    roll_deg = math.degrees(math.atan2(long_axis[0], long_axis[1]))

    geometry = SpineGeometry(
        center_uv=center,
        long_axis_uv=long_axis,
        transverse_axis_uv=transverse,
        left_uv=left_uv,
        right_uv=right_uv,
        left_edge_uv=left_edge,
        right_edge_uv=right_edge,
        width_px=width_px,
        length_px=float(s_max - s_min),
        roll_deg=roll_deg,
        polygon_xy=contour_polygon(contour),
    )
    return filled, geometry


def shelf_index(box_xyxy: np.ndarray, image_height: int) -> int:
    center_y = float((box_xyxy[1] + box_xyxy[3]) * 0.5 / image_height)
    candidates = [
        (index, abs(center_y - (top + bottom) * 0.5))
        for index, (top, bottom) in enumerate(SHELF_BANDS, start=1)
        if top <= center_y <= bottom
    ]
    return min(candidates, key=lambda item: item[1])[0] if candidates else -1


def bbox_iou(first: np.ndarray, second: np.ndarray) -> float:
    x0 = max(first[0], second[0])
    y0 = max(first[1], second[1])
    x1 = min(first[2], second[2])
    y1 = min(first[3], second[3])
    intersection = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    area_first = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    area_second = max(0.0, second[2] - second[0]) * max(
        0.0, second[3] - second[1]
    )
    return intersection / max(area_first + area_second - intersection, 1.0)


def mask_iou(first: np.ndarray, second: np.ndarray) -> float:
    first_box = cv2.boundingRect(first.astype(np.uint8))
    second_box = cv2.boundingRect(second.astype(np.uint8))
    x0 = max(first_box[0], second_box[0])
    y0 = max(first_box[1], second_box[1])
    x1 = min(first_box[0] + first_box[2], second_box[0] + second_box[2])
    y1 = min(first_box[1] + first_box[3], second_box[1] + second_box[3])
    if x1 <= x0 or y1 <= y0:
        return 0.0
    first_crop = first[y0:y1, x0:x1]
    second_crop = second[y0:y1, x0:x1]
    intersection = int(np.logical_and(first_crop, second_crop).sum())
    union = int(first.sum() + second.sum() - intersection)
    return intersection / max(union, 1)


def deduplicate(detections: list[Detection], iou_threshold: float) -> list[Detection]:
    kept: list[Detection] = []
    for candidate in sorted(detections, key=lambda item: item.score, reverse=True):
        duplicate = False
        for accepted in kept:
            if candidate.shelf != accepted.shelf:
                continue
            if bbox_iou(candidate.box_xyxy, accepted.box_xyxy) < 0.15:
                continue
            if mask_iou(candidate.mask, accepted.mask) >= iou_threshold:
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)
    return kept


def load_sam3(
    repo: Path, checkpoint: Path, device: str, confidence: float
) -> Any:
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))
    from sam3.model.sam3_image_processor import Sam3Processor
    from sam3.model_builder import build_sam3_image_model

    print(f"Loading SAM3 checkpoint: {checkpoint}", flush=True)
    model = build_sam3_image_model(
        checkpoint_path=str(checkpoint),
        load_from_HF=False,
        device=device,
        eval_mode=True,
        compile=False,
    )
    return Sam3Processor(
        model=model,
        resolution=1008,
        device=device,
        confidence_threshold=confidence,
    )


def detect_frame(
    processor: Any,
    image: Image.Image,
    frame_index: int,
    prompt: str,
    min_score: float,
    nms_iou: float,
) -> list[Detection]:
    with torch.inference_mode():
        state = processor.set_image(image)
        state = processor.set_text_prompt(prompt=prompt, state=state)
    masks = to_numpy(state["masks"])
    if masks.ndim == 4 and masks.shape[1] == 1:
        masks = masks[:, 0]
    boxes = to_numpy(state["boxes"]).reshape(-1, 4)
    scores = to_numpy(state["scores"]).reshape(-1)

    detections: list[Detection] = []
    for raw_mask, raw_box, raw_score in zip(masks, boxes, scores):
        score = float(raw_score)
        if score < min_score:
            continue
        shelf = shelf_index(raw_box, image.height)
        if shelf < 0:
            continue
        width = float(raw_box[2] - raw_box[0])
        height = float(raw_box[3] - raw_box[1])
        if width < 6 or height < 100:
            continue
        if width > image.width * 0.18 or height > image.height * 0.65:
            continue
        try:
            mask, geometry = fit_spine_geometry(raw_mask.astype(bool))
        except ValueError:
            continue
        if geometry.width_px < 5 or geometry.length_px < 100:
            continue
        detections.append(
            Detection(
                frame=frame_index,
                shelf=shelf,
                score=score,
                mask=mask,
                box_xyxy=raw_box.astype(np.float64),
                geometry=geometry,
            )
        )
    detections = deduplicate(detections, nms_iou)
    return sorted(
        detections,
        key=lambda item: (item.shelf, float(item.geometry.center_uv[0])),
    )


def decode_record3d(
    capture: Path,
    frame: int,
    shape: tuple[int, int],
    use_confidence: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    rgbd = capture / "rgbd"
    depth = np.frombuffer(
        liblzfse.decompress((rgbd / f"{frame}.depth").read_bytes()),
        dtype=np.float32,
    ).reshape(shape)
    if use_confidence:
        confidence = np.frombuffer(
            liblzfse.decompress((rgbd / f"{frame}.conf").read_bytes()),
            dtype=np.uint8,
        ).reshape(shape)
    else:
        confidence = np.full(shape, 2, dtype=np.uint8)
    return depth, confidence


def load_depth_image(
    path: Path, depth_scale: float | None
) -> tuple[np.ndarray, float]:
    suffix = path.suffix.lower()
    if suffix == ".npy":
        raw = np.load(path, allow_pickle=False)
    elif suffix == ".npz":
        with np.load(path, allow_pickle=False) as archive:
            if "depth" in archive.files:
                raw = archive["depth"]
            elif len(archive.files) == 1:
                raw = archive[archive.files[0]]
            else:
                raise ValueError(
                    f"{path} contains multiple arrays; name the depth array 'depth'"
                )
    else:
        raw = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if raw is None:
            raise ValueError(f"Could not decode depth image: {path}")

    raw = np.asarray(raw)
    if raw.ndim == 3 and raw.shape[2] == 1:
        raw = raw[..., 0]
    if raw.ndim != 2:
        raise ValueError(f"Depth must be a 2-D single-channel array, got {raw.shape}")

    if depth_scale is None:
        if np.issubdtype(raw.dtype, np.floating):
            depth_scale = 1.0
        elif np.issubdtype(raw.dtype, np.integer) and raw.dtype.itemsize >= 2:
            depth_scale = 0.001
        else:
            raise ValueError(
                "8-bit or unsupported depth input has ambiguous units; "
                "provide --depth-scale in metres per value"
            )
    depth = raw.astype(np.float32) * float(depth_scale)
    return depth, float(depth_scale)


def _intrinsic_matrix_from_value(value: Any) -> np.ndarray:
    if isinstance(value, dict):
        if all(key in value for key in ("fx", "fy", "cx", "cy")):
            array = np.asarray(
                [value["fx"], value["fy"], value["cx"], value["cy"]],
                dtype=np.float64,
            )
        elif "data" in value:
            array = np.asarray(value["data"], dtype=np.float64)
        else:
            for key in ("K", "camera_matrix", "intrinsic_matrix", "intrinsics"):
                if key in value:
                    return _intrinsic_matrix_from_value(value[key])
            raise ValueError(
                "Intrinsics JSON must contain fx/fy/cx/cy or a K/camera_matrix"
            )
    else:
        array = np.asarray(value, dtype=np.float64)

    if array.size == 4:
        fx, fy, cx, cy = array.reshape(-1)
        matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    elif array.size == 9:
        matrix = array.reshape(3, 3)
    else:
        raise ValueError(
            f"Intrinsics must contain 4 coefficients or a 3x3 matrix, got {array.shape}"
        )

    if not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("Intrinsics contain invalid focal lengths or non-finite values")
    return matrix


def load_intrinsic_matrix(path: Path) -> np.ndarray:
    suffix = path.suffix.lower()
    if suffix == ".json":
        with path.open() as file:
            value = json.load(file)
        return _intrinsic_matrix_from_value(value)
    if suffix == ".npy":
        return _intrinsic_matrix_from_value(np.load(path, allow_pickle=False))
    if suffix == ".npz":
        with np.load(path, allow_pickle=False) as archive:
            for key in ("K", "camera_matrix", "intrinsics", "arr_0"):
                if key in archive.files:
                    return _intrinsic_matrix_from_value(archive[key])
            if len(archive.files) == 1:
                return _intrinsic_matrix_from_value(archive[archive.files[0]])
        raise ValueError(f"Could not find an intrinsic matrix in {path}")
    if suffix in {".yaml", ".yml", ".xml"}:
        storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        try:
            for key in ("K", "camera_matrix", "intrinsic_matrix"):
                node = storage.getNode(key)
                if not node.empty():
                    return _intrinsic_matrix_from_value(node.mat())
        finally:
            storage.release()
        raise ValueError(f"Could not find K/camera_matrix in {path}")

    values = np.fromstring(
        path.read_text().replace(",", " "), sep=" ", dtype=np.float64
    )
    return _intrinsic_matrix_from_value(values)


def intrinsic_matrix(metadata: dict[str, Any], frame: int) -> np.ndarray:
    coefficients = metadata.get("perFrameIntrinsicCoeffs")
    if coefficients and frame < len(coefficients):
        fx, fy, cx, cy = map(float, coefficients[frame])
    else:
        values = metadata["K"]
        fx, fy, cx, cy = values[0], values[4], values[6], values[7]
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


def camera_to_world(metadata: dict[str, Any], frame: int) -> np.ndarray:
    qx, qy, qz, qw, tx, ty, tz = map(float, metadata["poses"][frame])
    rotation_arkit = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    cv_to_arkit = np.diag([1.0, -1.0, -1.0])
    transform = np.eye(4)
    transform[:3, :3] = rotation_arkit @ cv_to_arkit
    transform[:3, 3] = [tx, ty, tz]
    return transform


def rays_from_pixels(pixels: np.ndarray, intrinsic: np.ndarray) -> np.ndarray:
    pixels = np.atleast_2d(pixels).astype(np.float64)
    rays = np.column_stack(
        (
            (pixels[:, 0] - intrinsic[0, 2]) / intrinsic[0, 0],
            (pixels[:, 1] - intrinsic[1, 2]) / intrinsic[1, 1],
            np.ones(len(pixels)),
        )
    )
    return rays


def sample_detection_depth(
    detection: Detection,
    depth: np.ndarray,
    confidence: np.ndarray,
    image_size: tuple[int, int],
    intrinsic: np.ndarray,
) -> np.ndarray:
    image_width, image_height = image_size
    coverage = cv2.resize(
        detection.mask.astype(np.float32),
        (depth.shape[1], depth.shape[0]),
        interpolation=cv2.INTER_AREA,
    )
    valid_base = np.isfinite(depth) & (depth > 0.1) & (depth < 5.0)
    high = (coverage >= 0.35) & valid_base & (confidence == 2)
    medium = (coverage >= 0.35) & valid_base & (confidence >= 1)
    selected_mask = high if int(high.sum()) >= 8 else medium
    if int(selected_mask.sum()) < 4:
        return np.empty((0, 3), dtype=np.float64)

    rows, columns = np.nonzero(selected_mask)
    surface_depth = depth[selected_mask].astype(np.float64)
    scale_x = image_width / depth.shape[1]
    scale_y = image_height / depth.shape[0]
    u = (columns + 0.5) * scale_x - 0.5
    v = (rows + 0.5) * scale_y - 0.5
    surface_pixels = np.column_stack((u, v))
    surface_rays = rays_from_pixels(surface_pixels, intrinsic)
    surface_points = surface_rays * surface_depth[:, None]
    detection.surface_pixels_uv = surface_pixels
    detection.surface_points = surface_points

    median = float(np.median(surface_depth))
    mad = float(np.median(np.abs(surface_depth - median)))
    tolerance = max(0.012, 4.5 * 1.4826 * mad)
    core = np.abs(surface_depth - median) <= tolerance
    core_depth = surface_depth[core]
    if core_depth.size < 4:
        return np.empty((0, 3), dtype=np.float64)

    detection.depth_m = float(np.median(core_depth))
    detection.depth_mad_m = float(
        1.4826 * np.median(np.abs(core_depth - detection.depth_m))
    )
    detection.depth_samples = int(core_depth.size)
    high_samples = high[selected_mask]
    detection.high_confidence_samples = int(np.count_nonzero(high_samples[core]))
    return surface_points


def fit_local_plane(points: np.ndarray) -> tuple[np.ndarray, float, float, float]:
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal *= -1
    offset = -float(normal @ centroid)
    residual = float(np.median(np.abs(points @ normal + offset)))
    tilt = math.degrees(math.acos(float(np.clip(normal[2], -1.0, 1.0))))
    return normal, offset, residual, tilt


def normal_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(np.clip(abs(first @ second), 0.0, 1.0))
    return math.degrees(math.acos(cosine))


def fit_local_plane_ransac(
    points: np.ndarray,
    threshold_m: float = 0.006,
    iterations: int = 96,
    seed: int = 0,
) -> tuple[np.ndarray, float, float, float, np.ndarray]:
    if len(points) < 3:
        raise ValueError("At least three points are required for a plane")

    rng = np.random.default_rng(seed)
    if len(points) > 2000:
        proposal_points = points[
            rng.choice(len(points), size=2000, replace=False)
        ]
    else:
        proposal_points = points

    best_normal: np.ndarray | None = None
    best_offset = math.nan
    best_count = -1
    best_residual = math.inf
    for _ in range(iterations):
        sample = proposal_points[
            rng.choice(len(proposal_points), size=3, replace=False)
        ]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-8:
            continue
        normal /= norm
        if normal[2] < 0:
            normal *= -1
        offset = -float(normal @ sample[0])
        residuals = np.abs(proposal_points @ normal + offset)
        inliers = residuals <= threshold_m
        count = int(np.count_nonzero(inliers))
        median_residual = (
            float(np.median(residuals[inliers])) if count else math.inf
        )
        if count > best_count or (
            count == best_count and median_residual < best_residual
        ):
            best_normal = normal
            best_offset = offset
            best_count = count
            best_residual = median_residual

    if best_normal is None:
        raise ValueError("Could not generate a valid plane hypothesis")

    inliers = np.abs(points @ best_normal + best_offset) <= threshold_m
    for _ in range(3):
        if int(np.count_nonzero(inliers)) < 3:
            break
        normal, offset, _, _ = fit_local_plane(points[inliers])
        refined = np.abs(points @ normal + offset) <= threshold_m
        if np.array_equal(refined, inliers):
            break
        best_normal, best_offset, inliers = normal, offset, refined

    if int(np.count_nonzero(inliers)) < 3:
        raise ValueError("RANSAC plane has too few inliers")
    best_normal, best_offset, residual, tilt = fit_local_plane(points[inliers])
    inliers = np.abs(points @ best_normal + best_offset) <= threshold_m
    return best_normal, best_offset, residual, tilt, inliers


def plane_longitudinal_coverage(
    detection: Detection, pixels: np.ndarray
) -> float:
    if len(pixels) < 2:
        return 0.0
    longitudinal = (
        pixels - detection.geometry.center_uv
    ) @ detection.geometry.long_axis_uv
    span = float(np.percentile(longitudinal, 95) - np.percentile(longitudinal, 5))
    return min(span / max(detection.geometry.length_px, 1.0), 1.0)


def extract_plane_candidates(
    detection: Detection,
    threshold_m: float = 0.006,
    maximum_planes: int = 3,
) -> list[PlaneCandidate]:
    if detection.surface_points is None or detection.surface_pixels_uv is None:
        return []
    points = detection.surface_points
    pixels = detection.surface_pixels_uv
    total = len(points)
    remaining = np.arange(total)
    candidates: list[PlaneCandidate] = []
    minimum_inliers = max(12, int(math.ceil(total * 0.04)))

    for plane_index in range(maximum_planes):
        if len(remaining) < minimum_inliers:
            break
        try:
            normal, offset, residual, _, local_inliers = fit_local_plane_ransac(
                points[remaining],
                threshold_m=threshold_m,
                seed=detection.frame * 1009 + plane_index * 97 + total,
            )
        except ValueError:
            break
        indices = remaining[local_inliers]
        if len(indices) < minimum_inliers:
            break
        coverage = plane_longitudinal_coverage(detection, pixels[indices])
        candidates.append(
            PlaneCandidate(
                normal=normal,
                offset=offset,
                residual_m=residual,
                inlier_indices=indices,
                inlier_ratio=len(indices) / total,
                longitudinal_coverage=coverage,
            )
        )
        remaining = remaining[~local_inliers]

    return candidates


def estimate_spine_normal(detections: list[Detection]) -> np.ndarray:
    entries: list[tuple[np.ndarray, float]] = []
    for detection in detections:
        for candidate in detection.plane_candidates:
            normal = candidate.normal
            view_angle = normal_angle_deg(normal, np.array([0.0, 0.0, 1.0]))
            if (
                candidate.longitudinal_coverage < 0.45
                or candidate.inlier_ratio < 0.06
                or abs(float(normal[1])) > 0.55
                or view_angle > 75.0
            ):
                continue
            weight = (
                len(candidate.inlier_indices)
                * candidate.longitudinal_coverage
                * (0.35 + 0.65 * abs(float(normal[2])))
            )
            entries.append((normal, weight))

    if not entries:
        return np.array([0.0, 0.0, 1.0])

    scores = []
    for seed_normal, _ in entries:
        score = sum(
            weight
            for normal, weight in entries
            if normal_angle_deg(seed_normal, normal) <= 22.0
        )
        scores.append(score)
    best_normal = entries[int(np.argmax(scores))][0]
    cluster = [
        (normal, weight)
        for normal, weight in entries
        if normal_angle_deg(best_normal, normal) <= 22.0
    ]
    normal = np.sum(
        [item_normal * weight for item_normal, weight in cluster], axis=0
    )
    normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal *= -1
    return normal


def fit_plane_supported_edges(
    detection: Detection, pixels: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
    geometry = detection.geometry
    centered = pixels - geometry.center_uv
    longitudinal = centered @ geometry.long_axis_uv
    transverse = centered @ geometry.transverse_axis_uv
    lower, upper = np.percentile(longitudinal, (8, 92))
    bin_edges = np.linspace(lower, upper, 17)
    samples_s: list[float] = []
    left_t: list[float] = []
    right_t: list[float] = []
    for start, stop in zip(bin_edges[:-1], bin_edges[1:]):
        selected = transverse[
            (longitudinal >= start) & (longitudinal < stop)
        ]
        if selected.size < 3:
            continue
        samples_s.append((start + stop) * 0.5)
        left_t.append(float(np.percentile(selected, 2)))
        right_t.append(float(np.percentile(selected, 98)))
    if len(samples_s) < 6:
        return None

    samples = np.asarray(samples_s)
    left_fit = np.polyfit(samples, np.asarray(left_t), 1)
    right_fit = np.polyfit(samples, np.asarray(right_t), 1)
    target_s = float(
        (
            (geometry.left_uv + geometry.right_uv) * 0.5
            - geometry.center_uv
        )
        @ geometry.long_axis_uv
    )
    edge_s = (
        geometry.left_edge_uv - geometry.center_uv
    ) @ geometry.long_axis_uv
    left_uv = (
        geometry.center_uv
        + geometry.long_axis_uv * target_s
        + geometry.transverse_axis_uv * np.polyval(left_fit, target_s)
    )
    right_uv = (
        geometry.center_uv
        + geometry.long_axis_uv * target_s
        + geometry.transverse_axis_uv * np.polyval(right_fit, target_s)
    )
    left_edge = (
        geometry.center_uv[None, :]
        + edge_s[:, None] * geometry.long_axis_uv[None, :]
        + np.polyval(left_fit, edge_s)[:, None]
        * geometry.transverse_axis_uv[None, :]
    )
    right_edge = (
        geometry.center_uv[None, :]
        + edge_s[:, None] * geometry.long_axis_uv[None, :]
        + np.polyval(right_fit, edge_s)[:, None]
        * geometry.transverse_axis_uv[None, :]
    )
    return left_uv, right_uv, left_edge, right_edge


def prepare_detection_surface(
    detection: Detection,
    depth: np.ndarray,
    confidence: np.ndarray,
    image_size: tuple[int, int],
    intrinsic: np.ndarray,
) -> bool:
    points = sample_detection_depth(
        detection, depth, confidence, image_size, intrinsic
    )
    if len(points) < 12 or not math.isfinite(detection.depth_m):
        detection.measurement_rejection_reason = "insufficient_depth_surface"
        return False
    detection.plane_candidates = extract_plane_candidates(detection)
    if not detection.plane_candidates:
        detection.measurement_rejection_reason = "no_depth_plane"
        return False
    return True


def ray_plane_intersection(
    pixels: np.ndarray, intrinsic: np.ndarray, normal: np.ndarray, offset: float
) -> np.ndarray:
    rays = rays_from_pixels(pixels, intrinsic)
    denominator = rays @ normal
    scale = -offset / denominator
    return rays * scale[:, None]


def reject_measurement(detection: Detection, reason: str) -> bool:
    detection.measurement_rejection_reason = reason
    return False


def measure_detection(
    detection: Detection,
    depth: np.ndarray,
    confidence: np.ndarray,
    image_size: tuple[int, int],
    intrinsic: np.ndarray,
    transform_wc: np.ndarray,
    expected_spine_normal: np.ndarray | None = None,
) -> bool:
    if detection.surface_points is None or not detection.plane_candidates:
        if not prepare_detection_surface(
            detection, depth, confidence, image_size, intrinsic
        ):
            return False
    if detection.surface_points is None or detection.surface_pixels_uv is None:
        return reject_measurement(detection, "missing_surface_samples")

    expected = (
        expected_spine_normal
        if expected_spine_normal is not None
        else np.array([0.0, 0.0, 1.0])
    )
    candidates = [
        candidate
        for candidate in detection.plane_candidates
        if candidate.longitudinal_coverage >= 0.45
    ]
    if not candidates:
        return reject_measurement(detection, "insufficient_plane_coverage")
    selected = min(
        candidates,
        key=lambda candidate: (
            normal_angle_deg(candidate.normal, expected),
            -len(candidate.inlier_indices),
        ),
    )
    consensus_angle = normal_angle_deg(selected.normal, expected)
    view_angle = normal_angle_deg(
        selected.normal, np.array([0.0, 0.0, 1.0])
    )
    detection.spine_plane_inlier_ratio = selected.inlier_ratio
    detection.plane_consensus_angle_deg = consensus_angle
    detection.view_angle_deg = view_angle
    if selected.inlier_ratio < 0.35:
        return reject_measurement(detection, "low_spine_plane_inlier_ratio")
    if selected.longitudinal_coverage < 0.55:
        return reject_measurement(detection, "low_spine_plane_coverage")
    if consensus_angle > 32.0:
        return reject_measurement(detection, "inconsistent_spine_plane_normal")
    if view_angle > 60.0:
        return reject_measurement(detection, "grazing_view_angle")

    normal = selected.normal
    offset = selected.offset
    detection.plane_residual_m = selected.residual_m
    detection.plane_tilt_deg = view_angle
    detection.used_local_plane = True

    selected_pixels = detection.surface_pixels_uv[selected.inlier_indices]
    selected_points = detection.surface_points[selected.inlier_indices]
    detection.depth_m = float(np.median(selected_points[:, 2]))
    supported_edges = fit_plane_supported_edges(detection, selected_pixels)

    geometry = detection.geometry
    original_left_uv = geometry.left_uv.copy()
    original_right_uv = geometry.right_uv.copy()
    mask_left_t = float(
        (geometry.left_uv - geometry.center_uv)
        @ geometry.transverse_axis_uv
    )
    mask_right_t = float(
        (geometry.right_uv - geometry.center_uv)
        @ geometry.transverse_axis_uv
    )
    side_candidates: list[tuple[PlaneCandidate, float]] = []
    for candidate in detection.plane_candidates:
        if candidate is selected:
            continue
        if (
            candidate.inlier_ratio < 0.04
            or candidate.longitudinal_coverage < 0.40
            or normal_angle_deg(candidate.normal, normal) < 35.0
        ):
            continue
        candidate_pixels = detection.surface_pixels_uv[
            candidate.inlier_indices
        ]
        median_t = float(
            np.median(
                (candidate_pixels - geometry.center_uv)
                @ geometry.transverse_axis_uv
            )
        )
        if abs(median_t) < 0.15 * max(geometry.width_px, 1.0):
            continue
        side_candidates.append((candidate, median_t))

    detection.side_face_fraction = float(
        sum(candidate.inlier_ratio for candidate, _ in side_candidates)
    )
    detection.side_face_detected = bool(side_candidates)
    if detection.side_face_fraction > 0.40:
        return reject_measurement(detection, "side_face_dominates_mask")

    if side_candidates:
        if supported_edges is None:
            return reject_measurement(
                detection, "side_face_boundary_unresolved"
            )
        support_left, support_right, support_left_edge, support_right_edge = (
            supported_edges
        )
        support_left_t = float(
            (support_left - geometry.center_uv)
            @ geometry.transverse_axis_uv
        )
        support_right_t = float(
            (support_right - geometry.center_uv)
            @ geometry.transverse_axis_uv
        )
        minimum_trim = max(1.5, 0.04 * geometry.width_px)
        has_left_side = any(median_t < 0 for _, median_t in side_candidates)
        has_right_side = any(median_t > 0 for _, median_t in side_candidates)
        if has_left_side:
            trim = support_left_t - mask_left_t
            if trim > minimum_trim:
                geometry.left_uv = support_left
                geometry.left_edge_uv = support_left_edge
                detection.boundary_trim_left_px = trim
                detection.boundary_corrected = True
        if has_right_side:
            trim = mask_right_t - support_right_t
            if trim > minimum_trim:
                geometry.right_uv = support_right
                geometry.right_edge_uv = support_right_edge
                detection.boundary_trim_right_px = trim
                detection.boundary_corrected = True
        if not detection.boundary_corrected:
            return reject_measurement(
                detection, "side_face_boundary_ambiguous"
            )

        corrected_width = float(
            (geometry.right_uv - geometry.left_uv)
            @ geometry.transverse_axis_uv
        )
        if (
            corrected_width < 0.35 * geometry.width_px
            or corrected_width > geometry.width_px
        ):
            return reject_measurement(
                detection, "invalid_corrected_spine_width"
            )
        geometry.width_px = corrected_width
    detection.plane_normal = normal
    detection.plane_offset = offset

    silhouette_points = ray_plane_intersection(
        np.vstack((original_left_uv, original_right_uv)),
        intrinsic,
        normal,
        offset,
    )
    detection.silhouette_thickness_mm = float(
        np.linalg.norm(silhouette_points[1] - silhouette_points[0]) * 1000
    )
    boundary_points = ray_plane_intersection(
        np.vstack((detection.geometry.left_uv, detection.geometry.right_uv)),
        intrinsic,
        normal,
        offset,
    )
    thickness_mm = float(np.linalg.norm(boundary_points[1] - boundary_points[0]) * 1000)
    if not 3.0 <= thickness_mm <= 100.0:
        return reject_measurement(detection, "thickness_out_of_range")
    detection.thickness_mm = thickness_mm
    detection.side_trim_mm = max(
        0.0, detection.silhouette_thickness_mm - thickness_mm
    )

    center_camera = ray_plane_intersection(
        detection.geometry.center_uv[None, :], intrinsic, normal, offset
    )[0]
    detection.camera_point = center_camera
    detection.world_point = (
        transform_wc[:3, :3] @ center_camera + transform_wc[:3, 3]
    )
    transverse = detection.geometry.transverse_axis_uv
    detection.metric_per_pixel_mm = float(
        detection.depth_m
        * math.sqrt(
            (transverse[0] / intrinsic[0, 0]) ** 2
            + (transverse[1] / intrinsic[1, 1]) ** 2
        )
        * 1000
    )
    detection.measurement_rejection_reason = None
    return True


def project_world_point(
    point_world: np.ndarray, intrinsic: np.ndarray, transform_wc: np.ndarray
) -> np.ndarray | None:
    point_camera = transform_wc[:3, :3].T @ (
        point_world - transform_wc[:3, 3]
    )
    if point_camera[2] <= 0:
        return None
    return np.array(
        [
            intrinsic[0, 0] * point_camera[0] / point_camera[2] + intrinsic[0, 2],
            intrinsic[1, 1] * point_camera[1] / point_camera[2] + intrinsic[1, 2],
        ]
    )


def associate_detections(
    tracks: list[BookTrack],
    detections: list[Detection],
    intrinsic: np.ndarray,
    transform_wc: np.ndarray,
    match_radius_px: float,
) -> int:
    matched_count = 0
    for shelf in (1, 2, 3):
        shelf_tracks = [track for track in tracks if track.shelf == shelf]
        shelf_detections = [item for item in detections if item.shelf == shelf]
        if not shelf_tracks or not shelf_detections:
            continue
        projected = [
            project_world_point(
                track.reference.world_point, intrinsic, transform_wc
            )
            for track in shelf_tracks
        ]
        costs = np.full(
            (len(shelf_tracks), len(shelf_detections)),
            match_radius_px * 10,
            dtype=np.float64,
        )
        for row, point in enumerate(projected):
            if point is None:
                continue
            for column, detection in enumerate(shelf_detections):
                pixel_distance = float(
                    np.linalg.norm(point - detection.geometry.center_uv)
                )
                world_distance_mm = float(
                    np.linalg.norm(
                        shelf_tracks[row].reference.world_point
                        - detection.world_point
                    )
                    * 1000
                )
                costs[row, column] = pixel_distance + min(world_distance_mm, 50) * 0.15

        rows, columns = linear_sum_assignment(costs)
        for row, column in zip(rows, columns):
            residual = float(costs[row, column])
            if residual > match_radius_px:
                continue
            shelf_tracks[row].measurements.append(shelf_detections[column])
            shelf_tracks[row].match_residual_px.append(residual)
            matched_count += 1
    return matched_count


def robust_summary(
    track: BookTrack, gripper_clearance_mm: float
) -> dict[str, Any]:
    valid = [
        measurement
        for measurement in track.measurements
        if math.isfinite(measurement.thickness_mm)
    ]
    values = np.asarray([item.thickness_mm for item in valid], dtype=np.float64)
    median = float(np.median(values))
    raw_mad = float(np.median(np.abs(values - median)))
    robust_sigma = 1.4826 * raw_mad
    keep = np.abs(values - median) <= max(3.0, 3.5 * robust_sigma)
    used = [item for item, accepted in zip(valid, keep) if accepted]
    values = values[keep]
    median = float(np.median(values))
    repeatability = (
        float(1.4826 * np.median(np.abs(values - median))) if len(values) > 1 else 0.0
    )
    pixel_floor = float(
        np.median([item.metric_per_pixel_mm for item in used]) * math.sqrt(2) * 0.85
    )
    view_angle_factor = float(
        np.median(
            [
                1.0
                / max(
                    math.cos(math.radians(item.view_angle_deg)),
                    0.5,
                )
                for item in used
            ]
        )
    )
    pixel_floor *= view_angle_factor
    side_face_floor = (
        1.5 if any(item.side_face_detected for item in used) else 0.0
    )
    calibration_floor = 0.8
    random_component = repeatability / math.sqrt(max(len(values), 1))
    uncertainty_95 = float(
        max(
            1.5,
            1.96
            * math.sqrt(
                random_component**2 + pixel_floor**2 + calibration_floor**2
                + side_face_floor**2
            ),
        )
    )
    recommended_opening = median + uncertainty_95 + gripper_clearance_mm
    match_residuals = np.asarray(track.match_residual_px[1:], dtype=np.float64)
    median_match_residual = (
        float(np.median(match_residuals)) if match_residuals.size else 0.0
    )
    quality = "high"
    if (
        len(values) < 4
        or uncertainty_95 > 3.0
        or median_match_residual > 25.0
        or any(item.side_face_detected for item in used)
        or any(item.view_angle_deg > 40.0 for item in used)
    ):
        quality = "medium"
    if (
        len(values) < 2
        or uncertainty_95 > 5.0
        or median_match_residual > 45.0
    ):
        quality = "low"

    reference = track.reference
    measurements = [
        {
            "frame": item.frame,
            "thickness_mm": item.thickness_mm,
            "depth_mm": item.depth_m * 1000,
            "depth_mad_mm": item.depth_mad_m * 1000,
            "score": item.score,
            "metric_per_pixel_mm": item.metric_per_pixel_mm,
            "depth_samples": item.depth_samples,
            "high_confidence_samples": item.high_confidence_samples,
            "plane_residual_mm": (
                item.plane_residual_m * 1000
                if math.isfinite(item.plane_residual_m)
                else None
            ),
            "plane_tilt_deg": (
                item.plane_tilt_deg if math.isfinite(item.plane_tilt_deg) else None
            ),
            "used_local_plane": item.used_local_plane,
            "view_angle_deg": item.view_angle_deg,
            "spine_plane_inlier_ratio": item.spine_plane_inlier_ratio,
            "plane_consensus_angle_deg": item.plane_consensus_angle_deg,
            "side_face_detected": item.side_face_detected,
            "side_face_fraction": item.side_face_fraction,
            "boundary_corrected": item.boundary_corrected,
            "boundary_trim_left_px": item.boundary_trim_left_px,
            "boundary_trim_right_px": item.boundary_trim_right_px,
            "silhouette_thickness_mm": item.silhouette_thickness_mm,
            "side_trim_mm": item.side_trim_mm,
        }
        for item in valid
    ]
    return {
        "book_id": track.book_id,
        "shelf": track.shelf,
        "order_on_shelf": int(track.book_id.split("-")[1]),
        "thickness_mm": median,
        "uncertainty_95_mm": uncertainty_95,
        "repeatability_mad_mm": repeatability,
        "recommended_gripper_opening_mm": recommended_opening,
        "recommended_gripper_opening_m": recommended_opening / 1000,
        "gripper_clearance_mm": gripper_clearance_mm,
        "quality": quality,
        "views_observed": len(valid),
        "views_used": len(values),
        "median_match_residual_px": median_match_residual,
        "reference_score": reference.score,
        "reference_depth_mm": reference.depth_m * 1000,
        "reference_width_px": reference.geometry.width_px,
        "reference_roll_deg": reference.geometry.roll_deg,
        "reference_view_angle_deg": reference.view_angle_deg,
        "reference_spine_plane_inlier_ratio": reference.spine_plane_inlier_ratio,
        "reference_plane_consensus_angle_deg": (
            reference.plane_consensus_angle_deg
        ),
        "reference_side_face_detected": reference.side_face_detected,
        "reference_side_face_fraction": reference.side_face_fraction,
        "reference_boundary_corrected": reference.boundary_corrected,
        "reference_boundary_trim_left_px": reference.boundary_trim_left_px,
        "reference_boundary_trim_right_px": reference.boundary_trim_right_px,
        "reference_silhouette_thickness_mm": (
            reference.silhouette_thickness_mm
        ),
        "reference_side_trim_mm": reference.side_trim_mm,
        "bbox_xyxy": reference.box_xyxy.tolist(),
        "boundary_polygon_xy": reference.geometry.polygon_xy,
        "left_edge_xy": reference.geometry.left_edge_uv.tolist(),
        "right_edge_xy": reference.geometry.right_edge_uv.tolist(),
        "measurements": measurements,
    }


def compute_grasp_pose(
    track: BookTrack,
    summary: dict[str, Any],
    intrinsic: np.ndarray,
    insertion_mm: float,
    approach_mm: float,
    spine_offset_mm: float,
) -> GraspPose | None:
    """Build the gripper frame that closes across the measured spine thickness."""
    detection = track.reference
    if detection.plane_normal is None or not math.isfinite(detection.plane_offset):
        return None

    geometry = detection.geometry
    contacts = ray_plane_intersection(
        np.vstack((geometry.left_uv, geometry.right_uv)),
        intrinsic,
        detection.plane_normal,
        detection.plane_offset,
    )
    close_axis = contacts[1] - contacts[0]
    thickness_m = float(np.linalg.norm(close_axis))
    top_uv = (geometry.left_edge_uv[0] + geometry.right_edge_uv[0]) * 0.5
    bottom_uv = (geometry.left_edge_uv[1] + geometry.right_edge_uv[1]) * 0.5
    spine_edges = ray_plane_intersection(
        np.vstack((top_uv, bottom_uv)),
        intrinsic,
        detection.plane_normal,
        detection.plane_offset,
    )
    spine_direction = spine_edges[0] - spine_edges[1]
    spine_span_m = float(np.linalg.norm(spine_direction))
    if thickness_m < 1e-4 or spine_span_m < 1e-3:
        return None

    approach_axis = detection.plane_normal / np.linalg.norm(detection.plane_normal)
    close_axis = close_axis / thickness_m
    spine_axis = np.cross(approach_axis, close_axis)
    if float(spine_axis @ spine_direction) < 0:
        close_axis = -close_axis
        spine_axis = -spine_axis

    contact_center = (contacts[0] + contacts[1]) * 0.5 + spine_axis * (
        spine_offset_mm / 1000
    )
    return GraspPose(
        book_id=track.book_id,
        shelf=track.shelf,
        quality=str(summary["quality"]),
        rotation=np.column_stack((close_axis, spine_axis, approach_axis)),
        tcp=contact_center + approach_axis * (insertion_mm / 1000),
        contact_center=contact_center,
        pregrasp=contact_center - approach_axis * (approach_mm / 1000),
        left_contact=contacts[0],
        right_contact=contacts[1],
        opening_mm=float(summary["recommended_gripper_opening_mm"]),
        thickness_mm=float(summary["thickness_mm"]),
        reference_thickness_mm=float(detection.thickness_mm),
        spine_span_mm=spine_span_m * 1000,
    )


def select_grasp_poses(
    poses: list[GraspPose], requested_book_id: str | None
) -> tuple[list[GraspPose], str | None]:
    """Select one grasp by its reference-frame ID, preserving all-mode by default."""
    if requested_book_id is None:
        return poses, None
    normalized = requested_book_id.strip().upper()
    if not normalized:
        raise ValueError("--book-id must not be empty")
    for pose in poses:
        if pose.book_id == normalized:
            return [pose], normalized
    available = ", ".join(pose.book_id for pose in poses)
    raise ValueError(
        f'Unknown book ID "{requested_book_id}". Available book IDs: {available}'
    )


def gripper_wireframe(
    grasp: GraspPose, finger_length_mm: float
) -> list[tuple[str, np.ndarray]]:
    """Camera-frame polylines of a parallel-jaw gripper at its grasp pose.

    Each finger is drawn as a rectangle in the approach/spine plane, so a front
    view still shows both pads even though the approach axis is almost parallel
    to the line of sight. The pad height along the spine is a drawing
    convention, not a measured quantity.
    """
    half_opening = grasp.opening_mm / 2000
    finger_length = finger_length_mm / 1000
    half_height = GRIPPER_PAD_HEIGHT_MM / 2000
    close_axis = grasp.close_axis
    spine_axis = grasp.spine_axis
    tip_center = grasp.tcp
    back_center = tip_center - grasp.approach_axis * finger_length

    paths: list[tuple[str, np.ndarray]] = []
    for side in (-1.0, 1.0):
        tip = tip_center + close_axis * (side * half_opening)
        back = back_center + close_axis * (side * half_opening)
        paths.append(
            (
                "finger",
                np.vstack(
                    (
                        tip + spine_axis * half_height,
                        tip - spine_axis * half_height,
                        back - spine_axis * half_height,
                        back + spine_axis * half_height,
                        tip + spine_axis * half_height,
                    )
                ),
            )
        )
    paths.append(
        (
            "body",
            np.vstack(
                (
                    back_center - close_axis * half_opening,
                    back_center + close_axis * half_opening,
                )
            ),
        )
    )
    paths.append(
        (
            "body",
            np.vstack(
                (
                    back_center,
                    back_center - grasp.approach_axis * (finger_length * 0.5),
                )
            ),
        )
    )
    paths.append(
        ("contact", np.vstack((grasp.left_contact, grasp.right_contact)))
    )
    return paths


def project_camera_points(
    points: np.ndarray, intrinsic: np.ndarray
) -> np.ndarray:
    """Project camera-frame points to pixels; points behind the camera give NaN."""
    points = np.atleast_2d(points).astype(np.float64)
    depth = points[:, 2]
    pixels = np.full((len(points), 2), np.nan)
    visible = depth > 1e-6
    pixels[visible, 0] = (
        intrinsic[0, 0] * points[visible, 0] / depth[visible] + intrinsic[0, 2]
    )
    pixels[visible, 1] = (
        intrinsic[1, 1] * points[visible, 1] / depth[visible] + intrinsic[1, 2]
    )
    return pixels


def grasp_pose_records(
    grasp: GraspExport, uses_camera_poses: bool
) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for pose in grasp.poses:
        record = {
            "book_id": pose.book_id,
            "shelf": pose.shelf,
            "quality": pose.quality,
            "gripper_opening_mm": pose.opening_mm,
            "thickness_mm": pose.thickness_mm,
            "reference_thickness_mm": pose.reference_thickness_mm,
            "spine_span_mm": pose.spine_span_mm,
            "camera": {
                "grasp_position_m": pose.tcp.tolist(),
                "spine_contact_center_m": pose.contact_center.tolist(),
                "pregrasp_position_m": pose.pregrasp.tolist(),
                "rotation_matrix": pose.rotation.tolist(),
                "quaternion_xyzw": Rotation.from_matrix(pose.rotation)
                .as_quat()
                .tolist(),
                "close_axis": pose.close_axis.tolist(),
                "spine_axis": pose.spine_axis.tolist(),
                "approach_axis": pose.approach_axis.tolist(),
                "left_contact_m": pose.left_contact.tolist(),
                "right_contact_m": pose.right_contact.tolist(),
            },
        }
        if uses_camera_poses:
            rotation_wc = grasp.transform_wc[:3, :3]
            translation_wc = grasp.transform_wc[:3, 3]
            world_rotation = rotation_wc @ pose.rotation
            record["world"] = {
                "grasp_position_m": (rotation_wc @ pose.tcp + translation_wc).tolist(),
                "spine_contact_center_m": (
                    rotation_wc @ pose.contact_center + translation_wc
                ).tolist(),
                "pregrasp_position_m": (
                    rotation_wc @ pose.pregrasp + translation_wc
                ).tolist(),
                "rotation_matrix": world_rotation.tolist(),
                "quaternion_xyzw": Rotation.from_matrix(world_rotation)
                .as_quat()
                .tolist(),
            }
        records.append(record)
    return records


def draw_text_box(
    image: Image.Image,
    position: tuple[int, int],
    text: str,
    color: tuple[int, int, int],
) -> None:
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=16)
    left, top, right, bottom = draw.textbbox(position, text, font=font, stroke_width=1)
    padding = 3
    draw.rectangle(
        (left - padding, top - padding, right + padding, bottom + padding),
        fill=(0, 0, 0, 190),
    )
    draw.text(position, text, fill=color, font=font, stroke_width=1)


def draw_vertical_label(
    image: Image.Image,
    center: np.ndarray,
    text: str,
    color: tuple[int, int, int],
) -> None:
    font = ImageFont.load_default(size=14)
    probe = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    left, top, right, bottom = probe.textbbox((0, 0), text, font=font)
    label = Image.new(
        "RGBA", (right - left + 8, bottom - top + 6), (0, 0, 0, 185)
    )
    ImageDraw.Draw(label).text((4 - left, 3 - top), text, fill=color, font=font)
    label = label.rotate(90, expand=True, resample=Image.Resampling.BICUBIC)
    x = int(round(center[0] - label.width * 0.5))
    y = int(round(center[1] - label.height * 0.5))
    image.paste(label, (x, y), label)


def render_reference_overlay(
    image: Image.Image,
    tracks: list[BookTrack],
    summaries: list[dict[str, Any]],
    output_path: Path,
) -> None:
    array = np.asarray(image).copy()
    summary_by_id = {item["book_id"]: item for item in summaries}
    for track in tracks:
        color = SHELF_COLORS_RGB[track.shelf - 1]
        contours, _ = cv2.findContours(
            track.reference.mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(array, contours, -1, color, thickness=3)
        for edge in (
            track.reference.geometry.left_edge_uv,
            track.reference.geometry.right_edge_uv,
        ):
            endpoints = np.rint(edge).astype(int)
            cv2.line(array, tuple(endpoints[0]), tuple(endpoints[1]), color, 3)

    rendered = Image.fromarray(array)
    for track in tracks:
        summary = summary_by_id[track.book_id]
        draw_vertical_label(
            rendered,
            track.reference.geometry.center_uv,
            (
                f"{track.book_id}{'*' if track.reference.boundary_corrected else ''} "
                f"{summary['thickness_mm']:.1f}mm"
            ),
            SHELF_COLORS_RGB[track.shelf - 1],
        )
    rendered.save(output_path)


def render_rejection_overlay(
    image: Image.Image,
    detections: list[Detection],
    output_path: Path,
) -> None:
    array = np.asarray(image).copy()
    for detection in detections:
        contours, _ = cv2.findContours(
            detection.mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(array, contours, -1, (235, 45, 45), thickness=3)
        center = tuple(np.rint(detection.geometry.center_uv).astype(int))
        cv2.line(
            array,
            (center[0] - 8, center[1] - 8),
            (center[0] + 8, center[1] + 8),
            (235, 45, 45),
            3,
        )
        cv2.line(
            array,
            (center[0] - 8, center[1] + 8),
            (center[0] + 8, center[1] - 8),
            (235, 45, 45),
            3,
        )

    rendered = Image.fromarray(array)
    for index, detection in enumerate(detections, start=1):
        center = detection.geometry.center_uv
        draw_text_box(
            rendered,
            (int(center[0] + 5), int(center[1])),
            f"R{index:02d}",
            (255, 90, 90),
        )

    panel_width = 430
    canvas_height = max(rendered.height, 42 + len(detections) * 48)
    canvas = Image.new(
        "RGB", (rendered.width + panel_width, canvas_height), (24, 24, 24)
    )
    canvas.paste(rendered, (0, 0))
    draw = ImageDraw.Draw(canvas)
    title_font = ImageFont.load_default(size=17)
    font = ImageFont.load_default(size=14)
    draw.text(
        (rendered.width + 14, 12),
        f"Rejected candidates: {len(detections)}",
        fill=(255, 110, 110),
        font=title_font,
    )
    for index, detection in enumerate(detections, start=1):
        reason = detection.measurement_rejection_reason or "rejected"
        draw.multiline_text(
            (rendered.width + 14, 42 + (index - 1) * 48),
            (
                f"R{index:02d}  shelf={detection.shelf}  "
                f"x={detection.geometry.center_uv[0]:.0f}\n"
                f"{reason}"
            ),
            fill=(245, 215, 215),
            font=font,
            spacing=2,
        )
    canvas.save(output_path)


def render_shelf_views(
    image: Image.Image,
    tracks: list[BookTrack],
    summaries: list[dict[str, Any]],
    output: Path,
) -> None:
    summary_by_id = {item["book_id"]: item for item in summaries}
    image_array = np.asarray(image)
    for shelf in (1, 2, 3):
        shelf_tracks = [track for track in tracks if track.shelf == shelf]
        if not shelf_tracks:
            continue
        y0 = max(
            0,
            int(
                min(
                    track.reference.geometry.left_edge_uv[:, 1].min()
                    for track in shelf_tracks
                )
                - 50
            ),
        )
        y1 = min(
            image.height,
            int(
                max(
                    track.reference.geometry.left_edge_uv[:, 1].max()
                    for track in shelf_tracks
                )
                + 50
            ),
        )
        crop = image_array[y0:y1].copy()
        color = SHELF_COLORS_RGB[shelf - 1]
        for track in shelf_tracks:
            shifted_mask = track.reference.mask[y0:y1]
            contours, _ = cv2.findContours(
                shifted_mask.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(crop, contours, -1, color, thickness=3)
        rendered = Image.fromarray(crop)
        for track in shelf_tracks:
            summary = summary_by_id[track.book_id]
            center = track.reference.geometry.center_uv.copy()
            center[1] -= y0
            draw_vertical_label(
                rendered,
                center,
                (
                    f"{track.book_id}"
                    f"{'*' if track.reference.boundary_corrected else ''} "
                    f"{summary['thickness_mm']:.1f}mm"
                ),
                color,
            )
        rendered.save(output / f"shelf_{shelf}_thickness.png")


def save_book_crops(
    image: Image.Image,
    tracks: list[BookTrack],
    summaries: list[dict[str, Any]],
    output: Path,
) -> None:
    crop_directory = output / "book_crops"
    crop_directory.mkdir(exist_ok=True)
    summary_by_id = {item["book_id"]: item for item in summaries}
    image_array = np.asarray(image)
    for track in tracks:
        summary = summary_by_id[track.book_id]
        x, y, width, height = cv2.boundingRect(
            track.reference.mask.astype(np.uint8)
        )
        desired_width = max(340, width + 40)
        center_x = x + width // 2
        left = max(0, center_x - desired_width // 2)
        right = min(image.width, left + desired_width)
        left = max(0, right - desired_width)
        top = max(0, y - 24)
        bottom = min(image.height, y + height + 24)
        crop = image_array[top:bottom, left:right].copy()
        mask_crop = track.reference.mask[top:bottom, left:right]
        contours, _ = cv2.findContours(
            mask_crop.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        color = SHELF_COLORS_RGB[track.shelf - 1]
        cv2.drawContours(crop, contours, -1, color, thickness=3)
        crop_image = Image.fromarray(crop)
        rendered = Image.new("RGB", (crop_image.width, crop_image.height + 72))
        rendered.paste(crop_image, (0, 72))
        draw = ImageDraw.Draw(rendered)
        font = ImageFont.load_default(size=15)
        draw.multiline_text(
            (7, 6),
            (
                f"{track.book_id}  T={summary['thickness_mm']:.1f} "
                f"+/- {summary['uncertainty_95_mm']:.1f} mm\n"
                f"OPEN >= {summary['recommended_gripper_opening_mm']:.1f} mm  "
                f"(clearance {summary['gripper_clearance_mm']:.1f})\n"
                f"SIDE={track.reference.side_face_detected}  "
                f"TRIM={track.reference.boundary_trim_left_px:.1f}/"
                f"{track.reference.boundary_trim_right_px:.1f}px "
                f"({track.reference.side_trim_mm:.1f}mm)"
            ),
            fill=color,
            font=font,
            spacing=3,
        )
        rendered.save(crop_directory / f"{track.book_id}.png")


def draw_projected_path(
    array: np.ndarray,
    pixels: np.ndarray,
    color: tuple[int, int, int],
    line_width: int,
    arrow: bool = False,
) -> None:
    if not np.all(np.isfinite(pixels)) or np.any(np.abs(pixels) > 1e5):
        return
    path = np.rint(pixels).astype(int)
    if arrow:
        cv2.arrowedLine(
            array,
            tuple(path[0]),
            tuple(path[-1]),
            color,
            line_width,
            cv2.LINE_AA,
            tipLength=0.25,
        )
        return
    cv2.polylines(
        array,
        [path.reshape(-1, 1, 2)],
        False,
        color,
        line_width,
        cv2.LINE_AA,
    )


def draw_camera_path(
    array: np.ndarray,
    points: np.ndarray,
    grasp: GraspExport,
    origin_xy: tuple[int, int],
    color: tuple[int, int, int],
    line_width: int,
    arrow: bool = False,
) -> None:
    pixels = project_camera_points(points, grasp.intrinsic) - np.asarray(
        origin_xy, dtype=np.float64
    )
    draw_projected_path(array, pixels, color, line_width, arrow=arrow)


def draw_grasp_closing_marker(
    array: np.ndarray,
    pose: GraspPose,
    grasp: GraspExport,
    origin_xy: tuple[int, int] = (0, 0),
    line_width: int = 2,
) -> None:
    """Draw a compact U-shaped gripper pose without width annotations."""
    half_opening = pose.opening_mm / 2000
    finger_length = grasp.finger_length_mm / 1000
    tick = pose.spine_axis * (GRIPPER_PAD_HEIGHT_MM / 4000)
    tip_center = pose.tcp
    wrist_center = tip_center - pose.approach_axis * finger_length
    wrist_points = []
    for side in (-1.0, 1.0):
        tip = tip_center + pose.close_axis * (side * half_opening)
        wrist = wrist_center + pose.close_axis * (side * half_opening)
        wrist_points.append(wrist)
        draw_camera_path(
            array,
            np.vstack((wrist, tip)),
            grasp,
            origin_xy,
            GRASP_COLORS_RGB["finger"],
            line_width,
        )
        draw_camera_path(
            array,
            np.vstack((tip + tick, tip - tick)),
            grasp,
            origin_xy,
            GRASP_COLORS_RGB["finger"],
            line_width,
        )
    draw_camera_path(
        array,
        np.vstack(wrist_points),
        grasp,
        origin_xy,
        GRASP_COLORS_RGB["body"],
        line_width + 1,
    )


def draw_grasp_glyph(
    array: np.ndarray,
    pose: GraspPose,
    grasp: GraspExport,
    origin_xy: tuple[int, int] = (0, 0),
    line_width: int = 2,
    with_approach: bool = False,
) -> None:
    origin = np.asarray(origin_xy, dtype=np.float64)
    for role, path in gripper_wireframe(pose, grasp.finger_length_mm):
        pixels = project_camera_points(path, grasp.intrinsic) - origin
        draw_projected_path(array, pixels, GRASP_COLORS_RGB[role], line_width)
    if not with_approach:
        return
    draw_camera_path(
        array,
        np.vstack((pose.pregrasp, pose.contact_center)),
        grasp,
        origin_xy,
        GRASP_COLORS_RGB["approach"],
        line_width,
        arrow=True,
    )


def draw_grasp_axes(
    array: np.ndarray,
    pose: GraspPose,
    grasp: GraspExport,
    axis_length_mm: float,
    origin_xy: tuple[int, int] = (0, 0),
    line_width: int = 2,
) -> None:
    origin = np.asarray(origin_xy, dtype=np.float64)
    for index, color in enumerate(GRASP_AXIS_COLORS_RGB):
        endpoint = pose.tcp + pose.rotation[:, index] * (axis_length_mm / 1000)
        pixels = (
            project_camera_points(
                np.vstack((pose.tcp, endpoint)), grasp.intrinsic
            )
            - origin
        )
        draw_projected_path(array, pixels, color, line_width, arrow=True)


def render_grasp_overlay(
    image: Image.Image, grasp: GraspExport, output_path: Path
) -> None:
    array = np.asarray(image).copy()
    for pose in grasp.poses:
        draw_grasp_closing_marker(array, pose, grasp)

    rendered = Image.fromarray(array)
    for pose in grasp.poses:
        label_point = pose.contact_center - pose.spine_axis * (
            pose.spine_span_mm * 0.45 / 1000
        )
        label_pixel = project_camera_points(
            label_point[None, :], grasp.intrinsic
        )[0]
        if not np.all(np.isfinite(label_pixel)):
            continue
        draw_vertical_label(
            rendered,
            label_pixel,
            pose.book_id,
            SHELF_COLORS_RGB[pose.shelf - 1],
        )
    draw_text_box(
        rendered,
        (12, 10),
        (
            f"{len(grasp.poses)} grasps   "
            "yellow = fingers   cyan = wrist/back bar"
        ),
        (245, 245, 245),
    )
    rendered.save(output_path)


def save_grasp_crops(
    image: Image.Image,
    tracks: list[BookTrack],
    grasp: GraspExport,
    output: Path,
) -> None:
    directory = output / "book_grasps"
    directory.mkdir(exist_ok=True)
    for stale_image in directory.glob("*.png"):
        stale_image.unlink()
    pose_by_id = {pose.book_id: pose for pose in grasp.poses}
    image_array = np.asarray(image)
    for track in tracks:
        pose = pose_by_id.get(track.book_id)
        if pose is None:
            continue
        x, y, width, height = cv2.boundingRect(
            track.reference.mask.astype(np.uint8)
        )
        desired_width = max(380, width + 120)
        center_x = x + width // 2
        left = max(0, center_x - desired_width // 2)
        right = min(image.width, left + desired_width)
        left = max(0, right - desired_width)
        top = max(0, y - 40)
        bottom = min(image.height, y + height + 40)
        crop = image_array[top:bottom, left:right].copy()
        contours, _ = cv2.findContours(
            track.reference.mask[top:bottom, left:right].astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(
            crop, contours, -1, SHELF_COLORS_RGB[pose.shelf - 1], thickness=2
        )
        draw_grasp_glyph(
            crop, pose, grasp, origin_xy=(left, top), with_approach=True
        )
        draw_grasp_axes(crop, pose, grasp, 55.0, origin_xy=(left, top))

        crop_image = Image.fromarray(crop)
        rendered = Image.new(
            "RGB", (crop_image.width, crop_image.height + 96), (18, 18, 18)
        )
        rendered.paste(crop_image, (0, 96))
        quaternion = Rotation.from_matrix(pose.rotation).as_quat()
        ImageDraw.Draw(rendered).multiline_text(
            (7, 6),
            (
                f"{pose.book_id}  OPEN {pose.opening_mm:.1f} mm  "
                f"T={pose.thickness_mm:.1f} mm  [{pose.quality}]\n"
                f"pos xyz  {pose.tcp[0]:+.3f} {pose.tcp[1]:+.3f} "
                f"{pose.tcp[2]:+.3f} m\n"
                f"quat xyzw  {quaternion[0]:+.3f} {quaternion[1]:+.3f} "
                f"{quaternion[2]:+.3f} {quaternion[3]:+.3f}\n"
                "axes: X close, Y spine up, Z approach   "
                f"span {pose.spine_span_mm:.0f} mm"
            ),
            fill=SHELF_COLORS_RGB[pose.shelf - 1],
            font=ImageFont.load_default(size=14),
            spacing=3,
        )
        rendered.save(directory / f"{pose.book_id}.png")


def shelf_topdown_basis(
    poses: list[GraspPose],
) -> tuple[np.ndarray, np.ndarray]:
    """Right and depth axes of a bird's-eye view shared by one shelf."""
    spine_axis = np.sum([pose.spine_axis for pose in poses], axis=0)
    spine_axis = spine_axis / np.linalg.norm(spine_axis)
    depth_axis = np.sum([pose.approach_axis for pose in poses], axis=0)
    depth_axis = depth_axis - spine_axis * float(depth_axis @ spine_axis)
    depth_axis = depth_axis / np.linalg.norm(depth_axis)
    right_axis = np.cross(depth_axis, spine_axis)
    return right_axis / np.linalg.norm(right_axis), depth_axis


def topdown_millimetres(
    points: np.ndarray,
    origin: np.ndarray,
    right_axis: np.ndarray,
    depth_axis: np.ndarray,
) -> np.ndarray:
    delta = np.atleast_2d(points) - origin
    return np.column_stack((delta @ right_axis, delta @ depth_axis)) * 1000


def topdown_pixels(
    points_mm: np.ndarray,
    lower_mm: np.ndarray,
    upper_mm: np.ndarray,
    scale_px_per_mm: float,
) -> np.ndarray:
    return np.column_stack(
        (
            (points_mm[:, 0] - lower_mm[0]) * scale_px_per_mm,
            (upper_mm[1] - points_mm[:, 1]) * scale_px_per_mm,
        )
    )


def render_grasp_topdown(
    grasp: GraspExport, output: Path, scale_px_per_mm: float = 2.4
) -> None:
    """Draw each shelf seen from above so approach and opening stay readable."""
    for stale_image in output.glob("shelf_*_grasp_topdown.png"):
        stale_image.unlink()
    for shelf in (1, 2, 3):
        poses = [pose for pose in grasp.poses if pose.shelf == shelf]
        if not poses:
            continue
        right_axis, depth_axis = shelf_topdown_basis(poses)
        origin = np.mean([pose.contact_center for pose in poses], axis=0)

        stride = max(1, round(len(poses) / 8))
        segments: list[tuple[str, np.ndarray]] = []
        for index, pose in enumerate(poses):
            paths = gripper_wireframe(pose, grasp.finger_length_mm)
            if index % stride:
                paths = [item for item in paths if item[0] == "contact"]
            else:
                paths.append(
                    ("approach", np.vstack((pose.pregrasp, pose.contact_center)))
                )
            for role, path in paths:
                segments.append(
                    (
                        role,
                        topdown_millimetres(
                            path, origin, right_axis, depth_axis
                        ),
                    )
                )

        extent = np.vstack([points for _, points in segments])
        lower = extent.min(axis=0) - np.array([22.0, 44.0])
        upper = extent.max(axis=0) + np.array([22.0, 30.0])
        width = int(round((upper[0] - lower[0]) * scale_px_per_mm))
        height = int(round((upper[1] - lower[1]) * scale_px_per_mm))
        canvas = np.full((height, width, 3), 22, dtype=np.uint8)
        for role, points in segments:
            draw_projected_path(
                canvas,
                topdown_pixels(points, lower, upper, scale_px_per_mm),
                GRASP_COLORS_RGB[role],
                1 if role == "approach" else 2,
                arrow=role == "approach",
            )

        rendered = Image.fromarray(canvas)
        color = SHELF_COLORS_RGB[shelf - 1]
        label_y = height - 30
        for pose in poses:
            center = topdown_pixels(
                topdown_millimetres(
                    pose.contact_center[None, :], origin, right_axis, depth_axis
                ),
                lower,
                upper,
                scale_px_per_mm,
            )[0]
            draw_vertical_label(
                rendered,
                np.array([center[0], label_y]),
                f"{pose.book_id} {pose.opening_mm:.0f}",
                color,
            )
        draw_text_box(
            rendered,
            (12, 8),
            (
                f"shelf {shelf} seen from above   {len(poses)} grasps   "
                "up = into the shelf, camera below   magenta = measured "
                "thickness of every book   "
                + (
                    "gripper drawn for every book"
                    if stride == 1
                    else f"gripper drawn every {stride} books"
                )
            ),
            (245, 245, 245),
        )
        bar_length = int(round(100 * scale_px_per_mm))
        bar_right = width - 16
        ImageDraw.Draw(rendered).line(
            (bar_right - bar_length, 34, bar_right, 34),
            fill=(245, 245, 245),
            width=2,
        )
        draw_text_box(
            rendered,
            (bar_right - bar_length, 40),
            "100 mm",
            (245, 245, 245),
        )
        rendered.save(output / f"shelf_{shelf}_grasp_topdown.png")


def render_depth_map(
    depth: np.ndarray,
    confidence: np.ndarray,
    output_size: tuple[int, int],
    output_path: Path,
) -> None:
    valid = np.isfinite(depth) & (depth > 0.1) & (confidence >= 1)
    lower, upper = np.percentile(depth[valid], (2, 98))
    normalized = np.zeros(depth.shape, dtype=np.uint8)
    normalized[valid] = np.clip(
        (upper - depth[valid]) / max(upper - lower, 1e-6) * 255, 0, 255
    ).astype(np.uint8)
    colored_bgr = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    colored_bgr[~valid] = 0
    colored_bgr = cv2.resize(
        colored_bgr, output_size, interpolation=cv2.INTER_NEAREST
    )
    cv2.imwrite(str(output_path), colored_bgr)


def write_outputs(
    capture: Path | None,
    output: Path,
    metadata: dict[str, Any],
    frames: list[int],
    reference_frame: int,
    image: Image.Image,
    tracks: list[BookTrack],
    summaries: list[dict[str, Any]],
    rejected_reference: list[Detection],
    frame_statistics: list[dict[str, Any]],
    depth: np.ndarray,
    confidence: np.ndarray,
    single_frame_rgbd: bool,
    grasp: GraspExport,
    input_info: dict[str, Any] | None = None,
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    render_reference_overlay(
        image, tracks, summaries, output / "books_thickness_overlay.png"
    )
    render_rejection_overlay(
        image,
        rejected_reference,
        output / "rejected_books_overlay.png",
    )
    render_shelf_views(image, tracks, summaries, output)
    save_book_crops(image, tracks, summaries, output)
    if grasp.poses:
        render_grasp_overlay(image, grasp, output / "books_grasp_overlay.png")
        save_grasp_crops(image, tracks, grasp, output)
        render_grasp_topdown(grasp, output)
    render_depth_map(
        depth, confidence, image.size, output / "reference_depth_map.png"
    )
    np.savez_compressed(
        output / "reference_book_masks.npz",
        **{track.book_id: track.reference.mask for track in tracks},
    )

    result = {
        "capture": str(capture) if capture is not None else None,
        "input_files": input_info,
        "mode": "single_frame_rgbd" if single_frame_rgbd else "multi_view_rgbd",
        "uses_confidence": not single_frame_rgbd,
        "uses_camera_poses": not single_frame_rgbd,
        "method": (
            (
                "SAM3 masks + robust spine/side plane decomposition + "
                "generic single-frame depth scale"
                if input_info is not None
                else (
                    "SAM3 masks + robust spine/side plane decomposition + "
                    "single-frame Record3D depth scale"
                )
            )
            if single_frame_rgbd
            else (
                "SAM3 masks + robust spine/side plane decomposition + "
                "Record3D LiDAR scale + multi-view pose association + "
                "robust median fusion"
            )
        ),
        "reference_frame": reference_frame,
        "frames_requested": frames,
        "image_size_px": [metadata["w"], metadata["h"]],
        "depth_size_px": [metadata["dw"], metadata["dh"]],
        "depth_unit": "metres",
        "thickness_unit": "millimetres",
        "book_count": len(summaries),
        "reference_rejection_count": len(rejected_reference),
        "frame_statistics": frame_statistics,
        "accuracy_note": (
            (
                "single-frame mode has no repeatability estimate and all "
                "measurements are marked low quality; uncertainty_95_mm only "
                "covers RGB boundary sampling and a 0.8 mm calibration floor"
            )
            if single_frame_rgbd
            else (
                "uncertainty_95_mm includes observed multi-view repeatability, "
                "RGB boundary sampling, and a 0.8 mm calibration floor; it is "
                "an engineering estimate, not metrology certification"
            )
        ),
        "books": summaries,
    }
    with (output / "book_thickness.json").open("w") as file:
        json.dump(result, file, indent=2, allow_nan=False)

    gripper_commands = {
        "unit": "millimetres",
        "definition": "thickness + uncertainty_95 + clearance",
        "commands": {
            item["book_id"]: {
                "opening_mm": item["recommended_gripper_opening_mm"],
                "opening_m": item["recommended_gripper_opening_m"],
                "measured_thickness_mm": item["thickness_mm"],
                "uncertainty_95_mm": item["uncertainty_95_mm"],
                "quality": item["quality"],
            }
            for item in summaries
        },
    }
    with (output / "gripper_openings.json").open("w") as file:
        json.dump(gripper_commands, file, indent=2, allow_nan=False)

    grasp_poses = {
        "reference_frame": reference_frame,
        "requested_book_id": grasp.requested_book_id,
        "position_unit": "metres",
        "opening_unit": "millimetres",
        "pose_frame": "reference camera",
        "world_frame_available": not single_frame_rgbd,
        "axis_definition": (
            "rotation columns are x=gripper closing axis across the book "
            "thickness, y=spine axis towards the top of the book, "
            "z=approach axis into the shelf"
        ),
        "position_definition": (
            "spine_contact_center_m lies on the fitted spine plane midway "
            "between the two measured thickness boundaries; grasp_position_m "
            "is that point moved grasp_insertion_mm along +z and "
            "pregrasp_position_m is it moved grasp_approach_mm along -z"
        ),
        "grasp_insertion_mm": grasp.insertion_mm,
        "grasp_approach_mm": grasp.approach_mm,
        "grasp_spine_offset_mm": grasp.spine_offset_mm,
        "finger_length_mm": grasp.finger_length_mm,
        "grasp_count": len(grasp.poses),
        "grasps": grasp_pose_records(grasp, not single_frame_rgbd),
    }
    with (output / "book_grasp_poses.json").open("w") as file:
        json.dump(grasp_poses, file, indent=2, allow_nan=False)
    single_grasp_path = output / "book_grasp_pose.json"
    if grasp.requested_book_id is not None:
        single_grasp = {
            key: value
            for key, value in grasp_poses.items()
            if key not in {"grasp_count", "grasps"}
        }
        single_grasp["grasp"] = grasp_poses["grasps"][0]
        with single_grasp_path.open("w") as file:
            json.dump(single_grasp, file, indent=2, allow_nan=False)
    elif single_grasp_path.exists():
        single_grasp_path.unlink()

    fields = [
        "book_id",
        "shelf",
        "order_on_shelf",
        "thickness_mm",
        "uncertainty_95_mm",
        "recommended_gripper_opening_mm",
        "recommended_gripper_opening_m",
        "gripper_clearance_mm",
        "quality",
        "views_observed",
        "views_used",
        "median_match_residual_px",
        "reference_score",
        "reference_depth_mm",
        "reference_width_px",
        "reference_roll_deg",
        "reference_view_angle_deg",
        "reference_spine_plane_inlier_ratio",
        "reference_plane_consensus_angle_deg",
        "reference_side_face_detected",
        "reference_side_face_fraction",
        "reference_boundary_corrected",
        "reference_boundary_trim_left_px",
        "reference_boundary_trim_right_px",
        "reference_silhouette_thickness_mm",
        "reference_side_trim_mm",
        "repeatability_mad_mm",
    ]
    with (output / "book_thickness.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(summaries)


def main() -> None:
    args = parse_args()
    generic_mode = args.rgb is not None
    single_frame_rgbd = args.single_frame_rgbd or generic_mode
    input_info: dict[str, Any] | None = None

    if generic_mode:
        rgb_path = args.rgb.resolve()
        depth_path = args.depth.resolve()
        intrinsics_path = args.intrinsics.resolve()
        generic_image = Image.open(rgb_path).convert("RGB")
        generic_depth, applied_depth_scale = load_depth_image(
            depth_path, args.depth_scale
        )
        generic_intrinsic = load_intrinsic_matrix(intrinsics_path)
        image_width, image_height = generic_image.size
        depth_height, depth_width = generic_depth.shape
        image_aspect = image_width / image_height
        depth_aspect = depth_width / depth_height
        if abs(image_aspect - depth_aspect) / image_aspect > 0.02:
            raise ValueError(
                "RGB and depth aspect ratios differ by more than 2%; provide "
                "depth already registered/aligned to the RGB image"
            )

        capture = None
        reference_frame = 0
        frames = [reference_frame]
        metadata = {
            "w": image_width,
            "h": image_height,
            "dw": depth_width,
            "dh": depth_height,
        }
        image_size = generic_image.size
        depth_shape = generic_depth.shape
        output = (
            args.output.resolve()
            if args.output
            else rgb_path.parent / f"{rgb_path.stem}_book_thickness"
        )
        input_info = {
            "rgb": str(rgb_path),
            "depth": str(depth_path),
            "intrinsics": str(intrinsics_path),
            "depth_scale_m_per_unit": applied_depth_scale,
        }
    else:
        capture = args.capture.resolve()
        reference_frame = args.reference_frame
        output = (
            args.output.resolve()
            if args.output
            else capture / "book_thickness_analysis"
        )
        with (capture / "metadata").open() as file:
            metadata = json.load(file)
        if single_frame_rgbd:
            frames = [reference_frame]
        else:
            if "poses" not in metadata:
                raise ValueError(
                    "Multi-view mode requires metadata.poses; use "
                    "--single-frame-rgbd when poses are unavailable"
                )
            frame_count = len(metadata["poses"])
            frames = parse_frame_indices(
                args.frames, frame_count, reference_frame
            )
        depth_shape = (int(metadata["dh"]), int(metadata["dw"]))
        image_size = (int(metadata["w"]), int(metadata["h"]))

    processor = load_sam3(
        args.sam3_repo, args.checkpoint, args.device, args.detector_threshold
    )
    detections_by_frame: dict[int, list[Detection]] = {}
    rejected_by_frame: dict[int, list[Detection]] = {}
    frame_statistics: list[dict[str, Any]] = []
    frame_geometry: dict[int, tuple[np.ndarray, np.ndarray]] = {}

    ordered_frames = [reference_frame] + [
        frame for frame in frames if frame != reference_frame
    ]
    for frame in ordered_frames:
        print(f"Processing frame {frame}...", flush=True)
        if generic_mode:
            image = generic_image
            depth = generic_depth
            confidence = np.full(depth.shape, 2, dtype=np.uint8)
            intrinsic = generic_intrinsic
        else:
            image = Image.open(capture / "rgbd" / f"{frame}.jpg").convert("RGB")
            depth, confidence = decode_record3d(
                capture,
                frame,
                depth_shape,
                use_confidence=not single_frame_rgbd,
            )
            intrinsic = intrinsic_matrix(metadata, frame)
        detections = detect_frame(
            processor,
            image,
            frame,
            args.prompt,
            args.min_score,
            args.nms_iou,
        )
        transform_wc = (
            np.eye(4)
            if single_frame_rgbd
            else camera_to_world(metadata, frame)
        )
        prepared = [
            detection
            for detection in detections
            if prepare_detection_surface(
                detection,
                depth,
                confidence,
                image_size,
                intrinsic,
            )
        ]
        expected_spine_normal = estimate_spine_normal(prepared)
        measured = [
            detection
            for detection in prepared
            if measure_detection(
                detection,
                depth,
                confidence,
                image_size,
                intrinsic,
                transform_wc,
                expected_spine_normal,
            )
        ]
        measured_ids = {id(detection) for detection in measured}
        rejected_objects = [
            (index, detection)
            for index, detection in enumerate(detections)
            if id(detection) not in measured_ids
        ]
        rejected_detections = [
            {
                "candidate_index": index,
                "candidate_id": f"R{rejected_index:02d}",
                "shelf": detection.shelf,
                "score": detection.score,
                "bbox_xyxy": detection.box_xyxy.tolist(),
                "center_xy": detection.geometry.center_uv.tolist(),
                "reason": (
                    detection.measurement_rejection_reason
                    or "unknown_geometry_rejection"
                ),
                "side_face_detected": detection.side_face_detected,
                "side_face_fraction": detection.side_face_fraction,
                "spine_plane_inlier_ratio": (
                    detection.spine_plane_inlier_ratio
                    if math.isfinite(detection.spine_plane_inlier_ratio)
                    else None
                ),
                "view_angle_deg": (
                    detection.view_angle_deg
                    if math.isfinite(detection.view_angle_deg)
                    else None
                ),
            }
            for rejected_index, (index, detection) in enumerate(
                rejected_objects, start=1
            )
        ]
        detections_by_frame[frame] = measured
        rejected_by_frame[frame] = [
            detection for _, detection in rejected_objects
        ]
        frame_geometry[frame] = (intrinsic, transform_wc)
        frame_statistics.append(
            {
                "frame": frame,
                "detections": len(detections),
                "surface_models": len(prepared),
                "metric_measurements": len(measured),
                "geometry_rejections": len(detections) - len(measured),
                "rejected_detections": rejected_detections,
                "expected_spine_normal": expected_spine_normal.tolist(),
                "by_shelf": [
                    sum(item.shelf == shelf for item in measured)
                    for shelf in (1, 2, 3)
                ],
            }
        )
        print(
            f"  accepted {len(measured)} books "
            f"{frame_statistics[-1]['by_shelf']}",
            flush=True,
        )

    reference_detections = detections_by_frame[reference_frame]
    tracks: list[BookTrack] = []
    for shelf in (1, 2, 3):
        shelf_detections = sorted(
            (item for item in reference_detections if item.shelf == shelf),
            key=lambda item: float(item.geometry.center_uv[0]),
        )
        for index, detection in enumerate(shelf_detections, start=1):
            tracks.append(
                BookTrack(
                    book_id=f"S{shelf}-{index:02d}",
                    shelf=shelf,
                    reference=detection,
                    measurements=[detection],
                    match_residual_px=[0.0],
                )
            )

    for frame in frames:
        if frame == reference_frame:
            continue
        intrinsic, transform_wc = frame_geometry[frame]
        matched = associate_detections(
            tracks,
            detections_by_frame[frame],
            intrinsic,
            transform_wc,
            args.match_radius_px,
        )
        print(f"  matched frame {frame}: {matched}/{len(tracks)}", flush=True)

    summaries = [
        robust_summary(track, args.gripper_clearance_mm) for track in tracks
    ]
    reference_intrinsic, reference_transform = frame_geometry[reference_frame]
    grasp = GraspExport(
        poses=[],
        intrinsic=reference_intrinsic,
        transform_wc=reference_transform,
        insertion_mm=args.grasp_insertion_mm,
        approach_mm=args.grasp_approach_mm,
        spine_offset_mm=args.grasp_spine_offset_mm,
        finger_length_mm=args.grasp_finger_mm,
    )
    all_grasp_poses = [
        pose
        for pose in (
            compute_grasp_pose(
                track,
                summary,
                reference_intrinsic,
                args.grasp_insertion_mm,
                args.grasp_approach_mm,
                args.grasp_spine_offset_mm,
            )
            for track, summary in zip(tracks, summaries)
        )
        if pose is not None
    ]
    grasp.poses, grasp.requested_book_id = select_grasp_poses(
        all_grasp_poses, args.book_id
    )
    if generic_mode:
        reference_image = generic_image
        reference_depth = generic_depth
        reference_confidence = np.full(
            reference_depth.shape, 2, dtype=np.uint8
        )
    else:
        reference_image = Image.open(
            capture / "rgbd" / f"{reference_frame}.jpg"
        ).convert("RGB")
        reference_depth, reference_confidence = decode_record3d(
            capture,
            reference_frame,
            depth_shape,
            use_confidence=not single_frame_rgbd,
        )
    write_outputs(
        capture,
        output,
        metadata,
        frames,
        reference_frame,
        reference_image,
        tracks,
        summaries,
        rejected_by_frame[reference_frame],
        sorted(frame_statistics, key=lambda item: item["frame"]),
        reference_depth,
        reference_confidence,
        single_frame_rgbd,
        grasp,
        input_info,
    )

    quality_counts = {
        quality: sum(item["quality"] == quality for item in summaries)
        for quality in ("high", "medium", "low")
    }
    print(f"Measured {len(summaries)} books", flush=True)
    print(
        "Per shelf: "
        + ", ".join(
            f"S{shelf}={sum(item['shelf'] == shelf for item in summaries)}"
            for shelf in (1, 2, 3)
        ),
        flush=True,
    )
    print(f"Quality: {quality_counts}", flush=True)
    print(
        f"Grasp poses: {len(grasp.poses)}/{len(summaries)} books "
        f"(insertion {args.grasp_insertion_mm:g} mm, "
        f"standoff {args.grasp_approach_mm:g} mm)",
        flush=True,
    )
    if grasp.requested_book_id is not None:
        print(f"Selected book: {grasp.requested_book_id}", flush=True)
    print(f"Results: {output}", flush=True)


if __name__ == "__main__":
    main()
