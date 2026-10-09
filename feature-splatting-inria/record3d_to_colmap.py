#!/usr/bin/env python3
"""Convert an extracted Record3D capture to a 3DGS-ready COLMAP dataset.

The output contains:

    OUTPUT/
    ├── images/
    └── sparse/0/
        ├── cameras.txt
        ├── images.txt
        ├── points3D.txt
        └── points3D.ply

Record3D camera poses and intrinsics are written directly to a COLMAP text
model. The LZFSE-compressed LiDAR depth maps are fused into a metric RGB point
cloud for Gaussian initialization, so running COLMAP feature matching is not
required.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


@dataclass(frozen=True)
class FrameCamera:
    frame: int
    image_id: int
    camera_id: int
    source_image: Path
    output_name: str
    width: int
    height: int
    intrinsic: np.ndarray
    camera_to_world: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert an extracted Record3D capture into a COLMAP text model "
            "and RGB-D initial point cloud accepted by 3D Gaussian Splatting."
        )
    )
    parser.add_argument(
        "capture",
        type=Path,
        help="Record3D capture containing metadata and rgbd/",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output dataset directory (default: CAPTURE/colmap)",
    )
    parser.add_argument(
        "--frames",
        help="Explicit comma-separated frame indices; overrides frame range",
    )
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument(
        "--end-frame",
        type=int,
        help="Last frame index, inclusive (default: last available frame)",
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="Keep every Nth RGB frame (default: 1)",
    )
    parser.add_argument(
        "--point-frame-step",
        type=int,
        default=1,
        help="Use every Nth selected frame to build the point cloud (default: 1)",
    )
    parser.add_argument(
        "--depth-pixel-step",
        type=int,
        default=4,
        help="Sample every Nth depth pixel in each direction (default: 4)",
    )
    parser.add_argument(
        "--min-confidence",
        type=int,
        choices=(0, 1, 2),
        default=1,
        help="Minimum Record3D depth confidence when .conf exists (default: 1)",
    )
    parser.add_argument(
        "--min-depth",
        type=float,
        default=0.1,
        help="Minimum accepted depth in metres (default: 0.1)",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=5.0,
        help="Maximum accepted depth in metres (default: 5.0)",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.01,
        help="Point-cloud voxel size in metres; 0 disables fusion (default: 0.01)",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=1_000_000,
        help="Maximum output point count after voxel fusion; 0 is unlimited",
    )
    parser.add_argument(
        "--image-scale",
        type=float,
        default=1.0,
        help="RGB output scale; intrinsics are adjusted accordingly (default: 1)",
    )
    parser.add_argument(
        "--image-mode",
        choices=("copy", "symlink"),
        default="copy",
        help="How to export full-resolution RGB images (default: copy)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing output directory",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Random seed used only by --max-points (default: 0)",
    )
    return parser.parse_args()


def load_metadata(capture: Path) -> dict[str, Any]:
    path = capture / "metadata"
    if not path.is_file():
        raise ValueError(f"Record3D metadata not found: {path}")
    try:
        metadata = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"Could not read Record3D metadata: {path}") from exc

    for key in ("w", "h", "dw", "dh", "poses"):
        if key not in metadata:
            raise ValueError(f"Record3D metadata is missing '{key}'")
    if not metadata["poses"]:
        raise ValueError("Record3D metadata contains no camera poses")
    if "perFrameIntrinsicCoeffs" not in metadata and "K" not in metadata:
        raise ValueError(
            "Record3D metadata contains neither perFrameIntrinsicCoeffs nor K"
        )
    return metadata


def discover_images(capture: Path) -> dict[int, Path]:
    rgbd = capture / "rgbd"
    if not rgbd.is_dir():
        raise ValueError(f"Record3D rgbd directory not found: {rgbd}")

    images: dict[int, Path] = {}
    for path in rgbd.iterdir():
        if path.suffix.lower() not in {".jpg", ".jpeg", ".png"}:
            continue
        try:
            frame = int(path.stem)
        except ValueError:
            continue
        if frame in images:
            raise ValueError(f"Multiple RGB images found for frame {frame}")
        images[frame] = path
    if not images:
        raise ValueError(f"No numerically named RGB images found in {rgbd}")
    return images


def parse_frame_list(value: str) -> list[int]:
    try:
        frames = [int(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as exc:
        raise ValueError("--frames must be a comma-separated list of integers") from exc
    if not frames:
        raise ValueError("--frames did not contain any frame indices")
    if len(set(frames)) != len(frames):
        raise ValueError("--frames contains duplicate frame indices")
    return frames


def select_frames(
    images: dict[int, Path],
    pose_count: int,
    explicit_frames: str | None,
    start_frame: int,
    end_frame: int | None,
    frame_step: int,
) -> list[int]:
    if frame_step < 1:
        raise ValueError("--frame-step must be at least 1")

    if explicit_frames:
        frames = parse_frame_list(explicit_frames)
    else:
        last = max(images) if end_frame is None else end_frame
        if start_frame < 0 or last < start_frame:
            raise ValueError("Invalid --start-frame/--end-frame range")
        frames = [
            frame
            for frame in sorted(images)
            if start_frame <= frame <= last
        ][::frame_step]

    missing_images = [frame for frame in frames if frame not in images]
    if missing_images:
        raise ValueError(f"RGB images are missing for frames: {missing_images}")
    invalid_poses = [frame for frame in frames if frame < 0 or frame >= pose_count]
    if invalid_poses:
        raise ValueError(f"Camera poses are missing for frames: {invalid_poses}")
    if not frames:
        raise ValueError("No frames were selected")
    return frames


def intrinsic_from_metadata(metadata: dict[str, Any], frame: int) -> np.ndarray:
    coefficients = metadata.get("perFrameIntrinsicCoeffs")
    if coefficients is not None and frame < len(coefficients):
        values = coefficients[frame]
        if len(values) != 4:
            raise ValueError(
                f"Frame {frame} intrinsic coefficients must contain fx, fy, cx, cy"
            )
        fx, fy, cx, cy = map(float, values)
    else:
        values = metadata.get("K")
        if values is None or len(values) != 9:
            raise ValueError(f"No valid camera intrinsics for frame {frame}")
        # Record3D serializes this matrix in column-major order.
        fx, fy, cx, cy = map(float, (values[0], values[4], values[6], values[7]))

    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError(f"Frame {frame} has non-finite camera intrinsics")
    if fx <= 0 or fy <= 0:
        raise ValueError(f"Frame {frame} has invalid focal lengths")
    return np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def quaternion_xyzw_to_matrix(quaternion: list[float] | np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Camera pose contains an invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def camera_to_world_from_metadata(
    metadata: dict[str, Any], frame: int
) -> np.ndarray:
    pose = metadata["poses"][frame]
    if len(pose) != 7:
        raise ValueError(
            f"Frame {frame} pose must contain qx, qy, qz, qw, tx, ty, tz"
        )
    qx, qy, qz, qw, tx, ty, tz = map(float, pose)
    if not all(math.isfinite(value) for value in (tx, ty, tz)):
        raise ValueError(f"Frame {frame} pose contains non-finite translation")

    # ARKit camera axes are +X right, +Y up, -Z forward. COLMAP/OpenCV uses
    # +X right, +Y down, +Z forward.
    cv_to_arkit = np.diag([1.0, -1.0, -1.0])
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = (
        quaternion_xyzw_to_matrix([qx, qy, qz, qw]) @ cv_to_arkit
    )
    transform[:3, 3] = [tx, ty, tz]
    return transform


def rotation_matrix_to_qvec(rotation: np.ndarray) -> np.ndarray:
    """Return a normalized COLMAP quaternion in (qw, qx, qy, qz) order."""
    r = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(r))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r[2, 1] - r[1, 2]) / scale
        qy = (r[0, 2] - r[2, 0]) / scale
        qz = (r[1, 0] - r[0, 1]) / scale
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        scale = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / scale
        qx = 0.25 * scale
        qy = (r[0, 1] + r[1, 0]) / scale
        qz = (r[0, 2] + r[2, 0]) / scale
    elif r[1, 1] > r[2, 2]:
        scale = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / scale
        qx = (r[0, 1] + r[1, 0]) / scale
        qy = 0.25 * scale
        qz = (r[1, 2] + r[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / scale
        qx = (r[0, 2] + r[2, 0]) / scale
        qy = (r[1, 2] + r[2, 1]) / scale
        qz = 0.25 * scale
    quaternion = np.array([qw, qx, qy, qz], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[0] < 0:
        quaternion *= -1
    return quaternion


def prepare_frame_cameras(
    metadata: dict[str, Any],
    images: dict[int, Path],
    frames: list[int],
    images_dir: Path,
    image_scale: float,
    image_mode: str,
) -> list[FrameCamera]:
    if not math.isfinite(image_scale) or image_scale <= 0:
        raise ValueError("--image-scale must be positive")

    metadata_width = int(metadata["w"])
    metadata_height = int(metadata["h"])
    if metadata_width <= 0 or metadata_height <= 0:
        raise ValueError("Record3D metadata has invalid RGB dimensions")

    cameras: list[FrameCamera] = []
    for ordinal, frame in enumerate(frames, start=1):
        source = images[frame]
        with Image.open(source) as image:
            source_width, source_height = image.size
            output_width = max(1, round(source_width * image_scale))
            output_height = max(1, round(source_height * image_scale))
            output_name = f"{frame:06d}{source.suffix.lower()}"
            destination = images_dir / output_name

            if image_scale == 1.0:
                if image_mode == "copy":
                    shutil.copy2(source, destination)
                else:
                    destination.symlink_to(
                        os.path.relpath(source.resolve(), destination.parent)
                    )
            else:
                resized = image.convert("RGB").resize(
                    (output_width, output_height), Image.Resampling.LANCZOS
                )
                save_options = {"quality": 95} if destination.suffix in {".jpg", ".jpeg"} else {}
                resized.save(destination, **save_options)

        intrinsic = intrinsic_from_metadata(metadata, frame)
        intrinsic[0, :] *= output_width / metadata_width
        intrinsic[1, :] *= output_height / metadata_height
        intrinsic[2, :] = [0.0, 0.0, 1.0]
        cameras.append(
            FrameCamera(
                frame=frame,
                image_id=ordinal,
                camera_id=ordinal,
                source_image=source,
                output_name=output_name,
                width=output_width,
                height=output_height,
                intrinsic=intrinsic,
                camera_to_world=camera_to_world_from_metadata(metadata, frame),
            )
        )
    return cameras


def decode_lzfse_array(path: Path, dtype: np.dtype[Any], shape: tuple[int, int]) -> np.ndarray:
    try:
        import liblzfse
    except ImportError as exc:
        raise RuntimeError(
            "Record3D depth decoding requires pyliblzfse. Install it with: "
            "python -m pip install pyliblzfse"
        ) from exc

    if not path.is_file():
        raise ValueError(f"Record3D file not found: {path}")
    try:
        decoded = liblzfse.decompress(path.read_bytes())
    except Exception as exc:
        raise ValueError(f"Could not decompress Record3D file: {path}") from exc
    expected_bytes = int(np.prod(shape)) * np.dtype(dtype).itemsize
    if len(decoded) != expected_bytes:
        raise ValueError(
            f"{path} decoded to {len(decoded)} bytes; expected {expected_bytes}"
        )
    return np.frombuffer(decoded, dtype=dtype).reshape(shape)


def points_from_frame(
    capture: Path,
    camera: FrameCamera,
    depth_shape: tuple[int, int],
    pixel_step: int,
    min_confidence: int,
    min_depth: float,
    max_depth: float,
    exported_image: Path,
) -> tuple[np.ndarray, np.ndarray]:
    depth_path = capture / "rgbd" / f"{camera.frame}.depth"
    confidence_path = capture / "rgbd" / f"{camera.frame}.conf"
    depth = decode_lzfse_array(depth_path, np.dtype(np.float32), depth_shape)
    confidence = (
        decode_lzfse_array(confidence_path, np.dtype(np.uint8), depth_shape)
        if confidence_path.is_file()
        else None
    )

    rows = np.arange(0, depth_shape[0], pixel_step, dtype=np.int64)
    columns = np.arange(0, depth_shape[1], pixel_step, dtype=np.int64)
    grid_columns, grid_rows = np.meshgrid(columns, rows)
    sampled_depth = depth[grid_rows, grid_columns]
    valid = (
        np.isfinite(sampled_depth)
        & (sampled_depth >= min_depth)
        & (sampled_depth <= max_depth)
    )
    if confidence is not None:
        valid &= confidence[grid_rows, grid_columns] >= min_confidence
    if not np.any(valid):
        return (
            np.empty((0, 3), dtype=np.float64),
            np.empty((0, 3), dtype=np.uint8),
        )

    sampled_rows = grid_rows[valid].astype(np.float64)
    sampled_columns = grid_columns[valid].astype(np.float64)
    z = sampled_depth[valid].astype(np.float64)
    u = (sampled_columns + 0.5) * camera.width / depth_shape[1] - 0.5
    v = (sampled_rows + 0.5) * camera.height / depth_shape[0] - 0.5
    fx, fy = camera.intrinsic[0, 0], camera.intrinsic[1, 1]
    cx, cy = camera.intrinsic[0, 2], camera.intrinsic[1, 2]
    points_camera = np.column_stack(
        ((u - cx) * z / fx, (v - cy) * z / fy, z)
    )
    rotation = camera.camera_to_world[:3, :3]
    translation = camera.camera_to_world[:3, 3]
    points_world = points_camera @ rotation.T + translation

    with Image.open(exported_image) as image:
        rgb = np.asarray(image.convert("RGB"))
    color_u = np.clip(np.rint(u).astype(np.int64), 0, camera.width - 1)
    color_v = np.clip(np.rint(v).astype(np.int64), 0, camera.height - 1)
    colors = rgb[color_v, color_u]
    return points_world, colors


def voxel_fuse(
    points: np.ndarray, colors: np.ndarray, voxel_size: float
) -> tuple[np.ndarray, np.ndarray]:
    if voxel_size <= 0 or len(points) == 0:
        return points, colors
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    count = np.bincount(inverse).astype(np.float64)
    fused_points = np.column_stack(
        [np.bincount(inverse, weights=points[:, axis]) / count for axis in range(3)]
    )
    fused_colors = np.column_stack(
        [np.bincount(inverse, weights=colors[:, axis]) / count for axis in range(3)]
    )
    return fused_points, np.clip(np.rint(fused_colors), 0, 255).astype(np.uint8)


def build_point_cloud(
    capture: Path,
    cameras: list[FrameCamera],
    images_dir: Path,
    depth_shape: tuple[int, int],
    point_frame_step: int,
    pixel_step: int,
    min_confidence: int,
    min_depth: float,
    max_depth: float,
    voxel_size: float,
    max_points: int,
    seed: int,
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    if point_frame_step < 1:
        raise ValueError("--point-frame-step must be at least 1")
    if pixel_step < 1:
        raise ValueError("--depth-pixel-step must be at least 1")
    if min_depth < 0 or max_depth <= min_depth:
        raise ValueError("--max-depth must be greater than --min-depth")
    if not math.isfinite(voxel_size) or voxel_size < 0:
        raise ValueError("--voxel-size must be finite and non-negative")
    if max_points < 0:
        raise ValueError("--max-points must be non-negative")

    all_points: list[np.ndarray] = []
    all_colors: list[np.ndarray] = []
    point_frames: list[int] = []
    for camera in cameras[::point_frame_step]:
        points, colors = points_from_frame(
            capture=capture,
            camera=camera,
            depth_shape=depth_shape,
            pixel_step=pixel_step,
            min_confidence=min_confidence,
            min_depth=min_depth,
            max_depth=max_depth,
            exported_image=images_dir / camera.output_name,
        )
        if len(points):
            all_points.append(points)
            all_colors.append(colors)
            point_frames.append(camera.frame)

    if not all_points:
        raise ValueError(
            "No valid depth points were produced; check depth range, confidence, "
            "and .depth files"
        )
    points = np.concatenate(all_points, axis=0)
    colors = np.concatenate(all_colors, axis=0)
    points, colors = voxel_fuse(points, colors, voxel_size)

    if max_points and len(points) > max_points:
        rng = np.random.default_rng(seed)
        indices = np.sort(rng.choice(len(points), size=max_points, replace=False))
        points = points[indices]
        colors = colors[indices]
    return points, colors, point_frames


def write_cameras_txt(path: Path, cameras: list[FrameCamera]) -> None:
    with path.open("w") as file:
        file.write("# Camera list with one line of data per camera:\n")
        file.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        file.write(f"# Number of cameras: {len(cameras)}\n")
        for camera in cameras:
            fx = camera.intrinsic[0, 0]
            fy = camera.intrinsic[1, 1]
            cx = camera.intrinsic[0, 2]
            cy = camera.intrinsic[1, 2]
            file.write(
                f"{camera.camera_id} PINHOLE {camera.width} {camera.height} "
                f"{fx:.17g} {fy:.17g} {cx:.17g} {cy:.17g}\n"
            )


def write_images_txt(path: Path, cameras: list[FrameCamera]) -> None:
    with path.open("w") as file:
        file.write("# Image list with two lines of data per image:\n")
        file.write(
            "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, IMAGE_NAME\n"
        )
        file.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
        file.write(f"# Number of images: {len(cameras)}, mean observations per image: 0\n")
        for camera in cameras:
            world_to_camera = np.linalg.inv(camera.camera_to_world)
            qvec = rotation_matrix_to_qvec(world_to_camera[:3, :3])
            tvec = world_to_camera[:3, 3]
            pose_values = " ".join(
                f"{value:.17g}" for value in np.concatenate((qvec, tvec))
            )
            file.write(
                f"{camera.image_id} {pose_values} {camera.camera_id} "
                f"{camera.output_name}\n\n"
            )


def write_points3d_txt(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    with path.open("w") as file:
        file.write("# 3D point list with one line of data per point:\n")
        file.write("#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[]\n")
        file.write(f"# Number of points: {len(points)}, mean track length: 0\n")
        for point_id, (point, color) in enumerate(zip(points, colors), start=1):
            file.write(
                f"{point_id} {point[0]:.9g} {point[1]:.9g} {point[2]:.9g} "
                f"{int(color[0])} {int(color[1])} {int(color[2])} 0\n"
            )


def write_points3d_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    with path.open("w") as file:
        file.write("ply\n")
        file.write("format ascii 1.0\n")
        file.write(f"element vertex {len(points)}\n")
        file.write("property float x\nproperty float y\nproperty float z\n")
        file.write("property float nx\nproperty float ny\nproperty float nz\n")
        file.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        file.write("end_header\n")
        for point, color in zip(points, colors):
            file.write(
                f"{point[0]:.9g} {point[1]:.9g} {point[2]:.9g} 0 0 0 "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def ensure_output_is_safe(capture: Path, output: Path, overwrite: bool) -> None:
    if output.resolve() == capture.resolve():
        raise ValueError("--output must not be the Record3D capture directory itself")
    if output.exists():
        if not output.is_dir() or output.is_symlink():
            raise ValueError(f"Output exists and is not a normal directory: {output}")
        if any(output.iterdir()) and not overwrite:
            raise ValueError(
                f"Output directory is not empty: {output}; use --overwrite to replace it"
            )


def convert(args: argparse.Namespace) -> Path:
    capture = args.capture.expanduser().resolve()
    output = (
        args.output.expanduser().resolve()
        if args.output
        else capture / "colmap"
    )
    ensure_output_is_safe(capture, output, args.overwrite)
    metadata = load_metadata(capture)
    source_images = discover_images(capture)
    frames = select_frames(
        images=source_images,
        pose_count=len(metadata["poses"]),
        explicit_frames=args.frames,
        start_frame=args.start_frame,
        end_frame=args.end_frame,
        frame_step=args.frame_step,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.parent / f".{output.name}.tmp-{uuid.uuid4().hex}"
    images_dir = temporary / "images"
    sparse_dir = temporary / "sparse" / "0"
    images_dir.mkdir(parents=True)
    sparse_dir.mkdir(parents=True)
    try:
        cameras = prepare_frame_cameras(
            metadata=metadata,
            images=source_images,
            frames=frames,
            images_dir=images_dir,
            image_scale=args.image_scale,
            image_mode=args.image_mode,
        )
        depth_shape = (int(metadata["dh"]), int(metadata["dw"]))
        if depth_shape[0] <= 0 or depth_shape[1] <= 0:
            raise ValueError("Record3D metadata has invalid depth dimensions")
        points, colors, point_frames = build_point_cloud(
            capture=capture,
            cameras=cameras,
            images_dir=images_dir,
            depth_shape=depth_shape,
            point_frame_step=args.point_frame_step,
            pixel_step=args.depth_pixel_step,
            min_confidence=args.min_confidence,
            min_depth=args.min_depth,
            max_depth=args.max_depth,
            voxel_size=args.voxel_size,
            max_points=args.max_points,
            seed=args.seed,
        )
        write_cameras_txt(sparse_dir / "cameras.txt", cameras)
        write_images_txt(sparse_dir / "images.txt", cameras)
        write_points3d_txt(sparse_dir / "points3D.txt", points, colors)
        write_points3d_ply(sparse_dir / "points3D.ply", points, colors)
        conversion_info = {
            "source_capture": str(capture),
            "frames": frames,
            "point_cloud_frames": point_frames,
            "num_images": len(cameras),
            "num_points": len(points),
            "depth_shape": list(depth_shape),
            "image_scale": args.image_scale,
            "depth_pixel_step": args.depth_pixel_step,
            "min_confidence": args.min_confidence,
            "depth_range_metres": [args.min_depth, args.max_depth],
            "voxel_size_metres": args.voxel_size,
            "coordinate_system": (
                "Record3D/ARKit world; COLMAP cameras use +X right, +Y down, +Z forward"
            ),
        }
        (temporary / "conversion_info.json").write_text(
            json.dumps(conversion_info, indent=2) + "\n"
        )

        if output.exists():
            shutil.rmtree(output)
        temporary.replace(output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    print(
        f"Converted {len(cameras)} images and {len(points)} RGB-D points to {output}"
    )
    print(f"3DGS source path: {output}")
    return output


def main() -> int:
    args = parse_args()
    try:
        convert(args)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
