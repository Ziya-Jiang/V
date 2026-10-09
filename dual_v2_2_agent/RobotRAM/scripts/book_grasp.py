"""Side grasp for a book standing upright, from its point cloud.

The book has no RAM template. The masked depth points are turned into a
center, a mid-height grasp, and a horizontal approach along the thinner axis.
"""

import cv2
import numpy as np

from utils.transform_utils import rotation2angle

# Physical right-gripper frame relative to the dual_v2_2 tool frame.
# Jaw opening is physical X; the fingers point along physical +Z.
_RIGHT_GRIPPER_FROM_TOOL = np.diag([1.0, -1.0, -1.0])
_APPROACH_CLEARANCE_MM = 80.0
_RETREAT_LIFT_MM = 30.0
_PLACE_DISTANCE_MM = 150.0
_PLACE_LIFT_MM = 80.0
# Cart books lie with the spine along base X. Stay this far above the
# highest book before any horizontal motion so the hand clears the cart.
_CART_HOVER_CLEARANCE_MM = 120.0
_CART_MIN_SAFE_Z_MM = 180.0
_CART_ENTRY_X_MM = 580.0
_CART_ENTRY_Y_MM = -450.0
_CART_ENTRY_Z_MM = 250.0
# Prefer a clean single-book mask, but tolerate some depth/mask spill so a
# slightly noisy detection does not cancel the whole task.
_CART_STRICT_MAX_BOOK_HEIGHT_MM = 80.0
_CART_MAX_SINGLE_BOOK_HEIGHT_MM = 170.0
_CART_EFFECTIVE_BOOK_HEIGHT_MM = 80.0
_CART_MIN_DISTANCE_MM = 200.0
_CART_MAX_DISTANCE_MM = 1800.0
_CART_RETREAT_MM = 180.0
_CART_NEIGHBOR_MARGIN_MM = 40.0
_CART_PLUS_PLACE_Y_MM = 60.0
_CART_PLUS_INTO_BOOK_XY = np.array([1.0, 0.0])

# "shelf": standing book, side grasp. "cart": standing book, top grasp like bowl.
# "cart+": standing cart book, front-to-back spine grasp along base +X.
BOOK_GRASP_MODE = "shelf"
CART_NEIGHBOR_TOP_MM = None
CART_OTHER_BOOKS_MM = []
BOOK_SELECTION_NOTE = ""


def _pose(translation_m):
    pose = np.eye(4, dtype=float)
    pose[:3, 3] = np.asarray(translation_m, dtype=float).reshape(3)
    return pose


def _percentile_span(values, low=5.0, high=95.0):
    low_value = float(np.percentile(values, low))
    high_value = float(np.percentile(values, high))
    return low_value, high_value


def fill_standing_book_meta(obj_meta, pts_cam_m, c2w):
    """Fill one object meta entry with a standing-book pose.

    ``pts_cam_m`` is an Nx3 cloud in the camera frame, in meters. ``c2w`` is
    the camera-to-base transform used by the rest of the pipeline; its
    translation may be millimeters.
    """
    raw_points = np.asarray(pts_cam_m, dtype=float).reshape(-1, 3)
    finite = np.isfinite(raw_points).all(axis=1)
    points = raw_points[finite]
    if len(points) < 30:
        raise ValueError(f"book point cloud has only {len(points)} points")
    depth_mask = _nearest_book_depth_indices(points[:, 2])
    points = points[depth_mask]
    if len(points) < 30:
        raise ValueError("书的深度去噪后点太少")

    transform = np.asarray(c2w, dtype=float)
    rotation = transform[:3, :3]
    translation = transform[:3, 3].reshape(3)
    translation_scale = 0.001 if float(np.linalg.norm(translation)) > 10.0 else 1.0
    points_base = (rotation @ points.T).T + translation * translation_scale
    body_mask = _standing_book_body_mask(points_base)
    points = points[body_mask]
    points_base = points_base[body_mask]
    kept_indices = np.flatnonzero(finite)[depth_mask][body_mask]
    point_mask = np.zeros(len(raw_points), dtype=bool)
    point_mask[kept_indices] = True

    center = np.median(points_base, axis=0)
    z_low, z_high = _percentile_span(points_base[:, 2])
    grasp_z = 0.5 * (z_low + z_high)
    center[2] = grasp_z

    horizontal = points_base[:, :2] - center[:2]
    covariance = horizontal.T @ horizontal / max(len(horizontal), 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    thin_xy = eigenvectors[:, int(np.argmin(eigenvalues))]
    long_xy = eigenvectors[:, int(np.argmax(eigenvalues))]
    if float(np.dot(thin_xy, -center[:2])) < 0.0:
        thin_xy = -thin_xy
    if float(np.dot(long_xy, np.array([-thin_xy[1], thin_xy[0]]))) < 0.0:
        long_xy = -long_xy

    thin_low, thin_high = _percentile_span(horizontal @ thin_xy)
    long_low, long_high = _percentile_span(horizontal @ long_xy)
    half_thin = 0.5 * (thin_high - thin_low)
    half_long = 0.5 * (long_high - long_low)
    half_height = 0.5 * (z_high - z_low)
    if half_thin < 0.005 or half_long < 0.01 or half_height < 0.02:
        raise ValueError(
            "book cloud is too flat to recover a standing side grasp: "
            f"thickness={2 * half_thin * 1000:.1f}mm, "
            f"length={2 * half_long * 1000:.1f}mm, "
            f"height={2 * half_height * 1000:.1f}mm"
        )

    thin_base = np.array([thin_xy[0], thin_xy[1], 0.0])
    long_base = np.array([long_xy[0], long_xy[1], 0.0])
    up_base = np.array([0.0, 0.0, 1.0])

    def face(offset):
        return center + offset

    faces_base = {
        "bbox_plane_x1": face(-thin_base * half_thin),
        "bbox_plane_x2": face(thin_base * half_thin),
        "bbox_plane_y1": face(-long_base * half_long),
        "bbox_plane_y2": face(long_base * half_long),
        "bbox_plane_z1": face(-up_base * half_height),
        "bbox_plane_z2": face(up_base * half_height),
    }
    camera_from_base = rotation.T

    def to_camera(point_base):
        return camera_from_base @ (point_base - translation * translation_scale)

    obj_meta["centroid"] = _pose(to_camera(center))
    obj_meta["object_size"] = np.array(
        [2.0 * half_thin, 2.0 * half_long, 2.0 * half_height],
        dtype=float,
    )
    for name, point_base in faces_base.items():
        obj_meta[name] = _pose(to_camera(point_base))
    obj_meta["grasp_0"] = _pose(
        to_camera(center + thin_base * (half_thin + _APPROACH_CLEARANCE_MM / 1000.0))
    )
    return {
        "center_base_mm": (center * 1000.0).tolist(),
        "thickness_mm": 2.0 * half_thin * 1000.0,
        "length_mm": 2.0 * half_long * 1000.0,
        "height_mm": 2.0 * half_height * 1000.0,
        "thin_xy": thin_xy.tolist(),
        "long_xy": long_xy.tolist(),
        "approach_xy": thin_xy.tolist(),
        "input_points": int(len(raw_points)),
        "denoised_points": int(np.count_nonzero(point_mask)),
        "point_mask": point_mask,
    }


_CART_SPINE_INSET_MM = 15.0
_CART_SPINE_SPACING_MM = 30.0
_CART_SPINE_MAX_SAMPLES = 9


def spine_sample_offsets_mm(spine_xy, length_mm):
    """Sample grasp points along the whole spine, including the center.

    Offsets are millimeters in the robot base. Each end keeps a small inset so
    the gripper center stays on the book. Spacing stays near 30 mm, and a long
    book is still covered from end to end with at most nine samples.
    """
    direction = np.asarray(spine_xy, dtype=float).reshape(-1)
    if direction.size < 2 or not np.isfinite(direction[:2]).all():
        return [np.zeros(2)]
    direction = direction[:2]
    norm = float(np.linalg.norm(direction))
    length = float(length_mm)
    if norm < 1e-6 or not np.isfinite(length):
        return [np.zeros(2)]
    direction = direction / norm
    reach = 0.5 * length - _CART_SPINE_INSET_MM
    if reach <= 5.0:
        return [np.zeros(2)]
    count = int(np.floor((2.0 * reach) / _CART_SPINE_SPACING_MM)) + 1
    count = int(np.clip(count, 3, _CART_SPINE_MAX_SAMPLES))
    samples = np.linspace(-reach, reach, count)
    nearest = int(np.argmin(np.abs(samples)))
    samples[nearest] = 0.0
    return [direction * float(sample) for sample in samples]


def book_side_gripper_euler(into_book_xy):
    """Tool Euler angles whose physical right gripper points into the book."""
    direction = np.asarray(into_book_xy, dtype=float).reshape(2)
    gripper_z = np.array([direction[0], direction[1], 0.0], dtype=float)
    norm = float(np.linalg.norm(gripper_z))
    if norm < 1e-6:
        raise ValueError("book approach direction is zero")
    gripper_z /= norm
    gripper_x = np.cross(np.array([0.0, 0.0, 1.0]), gripper_z)
    gripper_x /= np.linalg.norm(gripper_x)
    gripper_y = np.cross(gripper_z, gripper_x)
    world_from_gripper = np.column_stack([gripper_x, gripper_y, gripper_z])
    world_from_tool = world_from_gripper @ _RIGHT_GRIPPER_FROM_TOOL.T
    return rotation2angle(world_from_tool).tolist()


def _point(position):
    return np.array(
        [float(position["x"]), float(position["y"]), float(position["z"])],
        dtype=float,
    )


def set_book_grasp_mode(mode):
    """Select shelf side-grasp, cart top-grasp, or cart+ X-spine grasp."""
    global BOOK_GRASP_MODE, CART_NEIGHBOR_TOP_MM, CART_OTHER_BOOKS_MM, BOOK_SELECTION_NOTE
    if mode not in ("shelf", "cart", "cart+"):
        raise ValueError(f"unknown book grasp mode: {mode}")
    BOOK_GRASP_MODE = mode
    CART_NEIGHBOR_TOP_MM = None
    CART_OTHER_BOOKS_MM = []
    BOOK_SELECTION_NOTE = ""


def _camera_to_base(points_cam_m, c2w):
    transform = np.asarray(c2w, dtype=float)
    rotation = transform[:3, :3]
    translation = transform[:3, 3].reshape(3)
    translation_scale = 0.001 if float(np.linalg.norm(translation)) > 10.0 else 1.0
    points_base = (rotation @ np.asarray(points_cam_m, dtype=float).T).T
    points_base = points_base + translation * translation_scale
    return points_base, rotation, translation * translation_scale


def _standing_extents(points):
    """Return PCA thickness/length and vertical height for one cloud."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(points) < 30 or not np.isfinite(points).all():
        return None
    center_xy = np.median(points[:, :2], axis=0)
    horizontal = points[:, :2] - center_xy
    covariance = horizontal.T @ horizontal / len(horizontal)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    thin = eigenvectors[:, int(np.argmin(eigenvalues))]
    long = eigenvectors[:, int(np.argmax(eigenvalues))]
    thin_low, thin_high = _percentile_span(horizontal @ thin)
    long_low, long_high = _percentile_span(horizontal @ long)
    z_low, z_high = _percentile_span(points[:, 2])
    return {
        "thickness_mm": (thin_high - thin_low) * 1000.0,
        "length_mm": (long_high - long_low) * 1000.0,
        "height_mm": (z_high - z_low) * 1000.0,
        "distance_mm": float(np.linalg.norm(center_xy) * 1000.0),
        "center_xy": center_xy,
        "thin": thin,
        "long": long,
    }


def _extents_look_like_one_book(extents):
    if extents is None:
        return False
    return (
        8.0 <= extents["thickness_mm"] <= 95.0
        and 70.0 <= extents["length_mm"] <= 450.0
        and 50.0 <= extents["height_mm"] <= 250.0
    )


def _drop_sparse_mask(points, voxel_m=0.020, min_count=8):
    """Drop isolated depth leaks while keeping a solid book surface."""
    if len(points) < 30:
        return np.ones(len(points), dtype=bool)
    origin = np.min(points, axis=0)
    coords = np.floor((points - origin) / voxel_m).astype(np.int64)
    unique_coords, inverse, counts = np.unique(
        coords, axis=0, return_inverse=True, return_counts=True
    )
    lookup = {
        (int(row[0]), int(row[1]), int(row[2])): index
        for index, row in enumerate(unique_coords)
    }
    neighbor_counts = np.zeros(len(unique_coords), dtype=np.int32)
    for index, row in enumerate(unique_coords):
        total = 0
        base = (int(row[0]), int(row[1]), int(row[2]))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    other = lookup.get((base[0] + dx, base[1] + dy, base[2] + dz))
                    if other is not None:
                        total += int(counts[other])
        neighbor_counts[index] = total
    kept = neighbor_counts[inverse] >= min_count
    if np.count_nonzero(kept) < max(30, int(0.25 * len(points))):
        return np.ones(len(points), dtype=bool)
    return kept


def _component_labels(points, voxel_m=0.025):
    """Label 26-connected voxels. Sparse bridges between book and cart break."""
    origin = np.min(points, axis=0)
    coords = np.floor((points - origin) / voxel_m).astype(np.int64)
    unique_coords, inverse = np.unique(coords, axis=0, return_inverse=True)
    lookup = {
        (int(row[0]), int(row[1]), int(row[2])): index
        for index, row in enumerate(unique_coords)
    }
    labels = -np.ones(len(unique_coords), dtype=np.int32)
    label_id = 0
    for start in range(len(unique_coords)):
        if labels[start] >= 0:
            continue
        stack = [start]
        labels[start] = label_id
        while stack:
            node = stack.pop()
            base_row = unique_coords[node]
            base = (int(base_row[0]), int(base_row[1]), int(base_row[2]))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == dy == dz == 0:
                            continue
                        nxt = lookup.get((base[0] + dx, base[1] + dy, base[2] + dz))
                        if nxt is not None and labels[nxt] < 0:
                            labels[nxt] = label_id
                            stack.append(nxt)
        label_id += 1
    return labels[inverse]


def _densest_axis_mask(values, window_m):
    values = np.asarray(values, dtype=float).reshape(-1)
    low, high = np.percentile(values, [1.0, 99.0])
    if high - low <= window_m:
        return np.ones(len(values), dtype=bool)
    bin_width = 0.010
    edges = np.arange(low, high + bin_width, bin_width)
    if len(edges) < 3:
        return np.ones(len(values), dtype=bool)
    counts, edges = np.histogram(values, bins=edges)
    window_bins = max(1, int(round(window_m / bin_width)))
    if window_bins >= len(counts):
        return np.ones(len(values), dtype=bool)
    scores = np.convolve(counts, np.ones(window_bins), mode="valid")
    start = int(np.argmax(scores))
    return (values >= edges[start]) & (values <= edges[start + window_bins])


def _trim_book_slab_mask(points):
    """Keep the densest book-sized slab if a mask still contains extra surfaces."""
    mask = np.ones(len(points), dtype=bool)

    def apply(current_points, selector):
        nonlocal mask
        kept = selector(current_points)
        if np.count_nonzero(kept) < 30:
            return current_points
        chosen = np.flatnonzero(mask)
        mask[chosen[~kept]] = False
        return current_points[kept]

    current = points
    extents = _standing_extents(current)
    if extents is not None:
        horizontal = current[:, :2] - extents["center_xy"]
        thin_coord = horizontal @ extents["thin"]
        current = apply(current, lambda _points: _densest_axis_mask(thin_coord, 0.070))
    extents = _standing_extents(current)
    if extents is not None:
        horizontal = current[:, :2] - extents["center_xy"]
        long_coord = horizontal @ extents["long"]
        current = apply(current, lambda _points: _densest_axis_mask(long_coord, 0.360))
    current = apply(current, lambda cloud: _densest_axis_mask(cloud[:, 2], 0.220))
    return mask


def _standing_book_body_mask(points_base):
    """Return the points that belong to one standing book body."""
    points = np.asarray(points_base, dtype=float).reshape(-1, 3)
    if len(points) < 30:
        raise ValueError(f"book point cloud has only {len(points)} points")
    compact_mask = _drop_sparse_mask(points)
    compact_indices = np.flatnonzero(compact_mask)
    compact = points[compact_mask]
    labels = _component_labels(compact)
    candidates = []
    for label in np.unique(labels):
        local = labels == label
        if np.count_nonzero(local) < 30:
            continue
        component_indices = compact_indices[local]
        component = points[component_indices]
        trimmed = _trim_book_slab_mask(component)
        kept_indices = component_indices[trimmed]
        extents = _standing_extents(points[kept_indices])
        if _extents_look_like_one_book(extents):
            candidates.append(
                (extents["distance_mm"], -len(kept_indices), int(label), kept_indices)
            )
    if not candidates:
        largest = max(np.unique(labels), key=lambda label: np.count_nonzero(labels == label))
        kept_indices = compact_indices[labels == largest]
        trimmed = _trim_book_slab_mask(points[kept_indices])
        kept_indices = kept_indices[trimmed]
        extents = _standing_extents(points[kept_indices])
        if not _extents_look_like_one_book(extents):
            thickness = 0.0 if extents is None else extents["thickness_mm"]
            length = 0.0 if extents is None else extents["length_mm"]
            height = 0.0 if extents is None else extents["height_mm"]
            raise ValueError(
                "去噪后仍不是一本立着的书: "
                f"thickness={thickness:.1f}mm, length={length:.1f}mm, "
                f"height={height:.1f}mm"
            )
        candidates.append((0.0, -len(kept_indices), 0, kept_indices))
    chosen = min(candidates, key=lambda item: (item[0], item[1], item[2]))[3]
    mask = np.zeros(len(points), dtype=bool)
    mask[chosen] = True
    return mask


def _nearest_book_depth_indices(depth_m):
    """Keep the nearest dense depth surface instead of the largest background."""
    values = np.asarray(depth_m, dtype=float).reshape(-1)
    finite = np.isfinite(values) & (values > 1e-6)
    if np.count_nonzero(finite) < 30:
        raise ValueError("book mask has too few finite depth samples")
    valid = values[finite]
    low, high = np.percentile(valid, [1.0, 99.0])
    if high - low < 0.03:
        return finite
    bin_width = 0.020
    edges = np.arange(low, high + bin_width, bin_width)
    if len(edges) < 3:
        return finite
    counts, edges = np.histogram(valid, bins=edges)
    peak_count = int(np.max(counts))
    if peak_count < 25:
        return _dominant_depth_cluster(values)
    # A leaked mask can contain more cart pixels than book pixels. Keep the
    # nearest surface that is still a real mode, not the largest far blob.
    min_peak = max(25, int(0.10 * peak_count))
    peaks = []
    for index, count in enumerate(counts):
        left = int(counts[index - 1]) if index > 0 else -1
        right = int(counts[index + 1]) if index + 1 < len(counts) else -1
        if count >= min_peak and count >= left and count >= right:
            peaks.append(index)
    if not peaks:
        peaks = [int(np.argmax(counts))]
    start = int(peaks[0])
    chosen_peak = int(counts[start])
    keep_level = max(8, int(0.20 * chosen_peak))
    peak_depth = 0.5 * (edges[start] + edges[start + 1])
    begin = start
    end = start
    for index in range(start, -1, -1):
        center = 0.5 * (edges[index] + edges[index + 1])
        if center < peak_depth - 0.08 or int(counts[index]) < keep_level:
            break
        begin = index
    for index in range(start, len(counts)):
        center = 0.5 * (edges[index] + edges[index + 1])
        if center > peak_depth + 0.16 or int(counts[index]) < keep_level:
            break
        end = index
    chosen = finite & (values >= edges[begin]) & (values <= edges[end + 1])
    if np.count_nonzero(chosen) < 30:
        return _dominant_depth_cluster(values)
    return chosen


def _dominant_depth_cluster(depth_m, bin_width_m=0.020, half_width_m=0.050):
    """Keep the dominant local depth surface and reject mask-edge background."""
    values = np.asarray(depth_m, dtype=float).reshape(-1)
    finite = np.isfinite(values) & (values > 1e-6)
    if np.count_nonzero(finite) < 30:
        raise ValueError("book mask has too few finite depth samples")
    valid_values = values[finite]
    low, high = np.percentile(valid_values, [1.0, 99.0])
    if high <= low:
        selected_valid = np.ones(valid_values.shape, dtype=bool)
    else:
        edges = np.arange(low, high + bin_width_m, bin_width_m)
        if len(edges) < 2:
            edges = np.array([low, low + bin_width_m])
        counts, edges = np.histogram(valid_values, bins=edges)
        peak = int(np.argmax(counts))
        center = 0.5 * (edges[peak] + edges[peak + 1])
        selected_valid = np.abs(valid_values - center) <= half_width_m
    if (
        np.count_nonzero(selected_valid) < 30
        or np.count_nonzero(selected_valid) < 0.20 * len(valid_values)
    ):
        raise ValueError("book mask has no dominant depth surface")
    selected = np.zeros(values.shape, dtype=bool)
    selected[np.flatnonzero(finite)[selected_valid]] = True
    return selected


def _mask_points_base(mask, depth, intrinsic, c2w):
    """Back-project one mask into the base frame. Depth may be mm or meters."""
    depth_map = np.asarray(depth, dtype=float)
    region = np.asarray(mask, dtype=bool)
    if depth_map.shape[:2] != region.shape:
        raise ValueError(
            f"depth shape {depth_map.shape} does not match mask {region.shape}"
        )
    region = cv2.erode(
        region.astype(np.uint8),
        np.ones((3, 3), dtype=np.uint8),
        iterations=4,
    ).astype(bool)
    ys, xs = np.nonzero(region)
    samples = depth_map[ys, xs]
    valid = np.isfinite(samples) & (samples > 1e-6)
    xs, ys, samples = xs[valid], ys[valid], samples[valid]
    if len(samples) < 30:
        raise ValueError(f"book mask has only {len(samples)} depth points")
    if float(np.median(samples)) > 20.0:
        samples = samples * 0.001
    dominant = _nearest_book_depth_indices(samples)
    xs, ys, samples = xs[dominant], ys[dominant], samples[dominant]
    if len(xs) > 4000:
        step = int(np.ceil(len(xs) / 4000))
        xs = xs[::step]
        ys = ys[::step]
        samples = samples[::step]
    matrix = np.asarray(intrinsic, dtype=float).reshape(3, 3)
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    points_cam = np.column_stack([
        (xs - cx) * samples / fx,
        (ys - cy) * samples / fy,
        samples,
    ])
    points_base, _, _ = _camera_to_base(points_cam, c2w)
    return points_base[_standing_book_body_mask(points_base)]


def _book_span(points_base):
    center = np.median(points_base, axis=0)
    z_low, z_high = _percentile_span(points_base[:, 2])
    x_low, x_high = _percentile_span(points_base[:, 0])
    y_low, y_high = _percentile_span(points_base[:, 1])
    center[2] = 0.5 * (z_low + z_high)
    span = {
        "center": center,
        "x": (x_low, x_high),
        "y": (y_low, y_high),
        "z": (z_low, z_high),
        "distance_mm": float(np.linalg.norm(center[:2]) * 1000.0),
        "height_mm": (z_high - z_low) * 1000.0,
        "thickness_mm": 0.0,
        "length_mm": 0.0,
    }
    extents = _standing_extents(points_base)
    if extents is not None:
        span["thickness_mm"] = extents["thickness_mm"]
        span["length_mm"] = extents["length_mm"]
    return span


def _cart_candidate_rejection(item):
    """Return why a cart detection is not one standing book in front of the arm."""
    center_mm = np.asarray(item["center"], dtype=float) * 1000.0
    reasons = []
    if not 100.0 <= item["height_mm"] <= 250.0:
        reasons.append(f"height={item['height_mm']:.0f}mm")
    elif item["height_mm"] < 1.6 * max(item["thickness_mm"], 1.0):
        reasons.append(
            f"flat height={item['height_mm']:.0f}mm "
            f"thickness={item['thickness_mm']:.0f}mm"
        )
    if not 8.0 <= item["thickness_mm"] <= 95.0:
        reasons.append(f"thickness={item['thickness_mm']:.0f}mm")
    if not 70.0 <= item["length_mm"] <= 450.0:
        reasons.append(f"length={item['length_mm']:.0f}mm")
    x_span_mm = (item["x"][1] - item["x"][0]) * 1000.0
    y_span_mm = (item["y"][1] - item["y"][0]) * 1000.0
    if x_span_mm > 450.0 or y_span_mm > 350.0:
        reasons.append(f"xy_span={x_span_mm:.0f}x{y_span_mm:.0f}mm")
    if not (200.0 <= center_mm[0] <= 1000.0 and -600.0 <= center_mm[1] <= 400.0):
        reasons.append(
            f"center=({center_mm[0]:.0f},{center_mm[1]:.0f},{center_mm[2]:.0f})mm"
        )
    if not 200.0 <= item["distance_mm"] <= 1100.0:
        reasons.append(f"distance={item['distance_mm']:.0f}mm")
    return ", ".join(reasons)


_SAM3_DETECTOR = None


def _load_sam3_detector():
    """Load the book segmenter from feature-splatting-inria once."""
    global _SAM3_DETECTOR
    if _SAM3_DETECTOR is not None:
        return _SAM3_DETECTOR
    import sys
    from pathlib import Path

    import torch
    from PIL import Image

    pipeline_root = Path("/mnt/ssd/user/V/feature-splatting-inria")
    if str(pipeline_root) not in sys.path:
        sys.path.insert(0, str(pipeline_root))
    from book_thickness_pipeline import (
        DEFAULT_SAM3_CHECKPOINT,
        DEFAULT_SAM3_REPO,
        load_sam3,
        to_numpy,
    )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"cart+ 加载 feature-splatting-inria 的 SAM3 书分割，device={device}", flush=True)
    processor = load_sam3(
        DEFAULT_SAM3_REPO,
        DEFAULT_SAM3_CHECKPOINT,
        device,
        0.10,
    )
    _SAM3_DETECTOR = (processor, to_numpy, Image)
    return _SAM3_DETECTOR


def _mask_median_depth_m(mask, depth):
    depth_map = np.asarray(depth, dtype=float)
    region = np.asarray(mask, dtype=bool)
    if region.shape != depth_map.shape[:2]:
        region = cv2.resize(
            region.astype(np.uint8),
            (depth_map.shape[1], depth_map.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)
    samples = depth_map[region]
    samples = samples[np.isfinite(samples) & (samples > 1e-6)]
    if samples.size < 30:
        return None
    depth_m = float(np.median(samples))
    if depth_m > 20.0:
        depth_m *= 0.001
    return depth_m


def _sam3_raw_books(processor, to_numpy, image, prompt="book"):
    """SAM3 book masks without the shelf size and height-band gates."""
    import torch

    with torch.inference_mode():
        state = processor.set_image(image)
        state = processor.set_text_prompt(prompt=prompt, state=state)
    masks = to_numpy(state["masks"])
    if masks.size == 0:
        return []
    if masks.ndim == 4 and masks.shape[1] == 1:
        masks = masks[:, 0]
    boxes = to_numpy(state["boxes"]).reshape(-1, 4)
    scores = to_numpy(state["scores"]).reshape(-1)
    books = []
    for raw_mask, raw_box, raw_score in zip(masks, boxes, scores):
        books.append(
            (
                float(raw_score),
                np.asarray(raw_mask).astype(bool),
                np.asarray(raw_box, dtype=np.float32).reshape(4),
            )
        )
    return books


def segment_cart_book_with_sam3(rgb, depth, intrinsic, c2w):
    """Segment the nearest book with SAM3, then keep its depth center."""
    global BOOK_SELECTION_NOTE
    try:
        processor, to_numpy, image_cls = _load_sam3_detector()
    except Exception as error:
        BOOK_SELECTION_NOTE = f"SAM3 分割未能启动: {error}"
        return None
    image = image_cls.fromarray(np.asarray(rgb))
    books = _sam3_raw_books(processor, to_numpy, image)
    ranked = []
    notes = []
    for index, (score, raw_mask, box) in enumerate(books, start=1):
        depth_m = _mask_median_depth_m(raw_mask, depth)
        box_text = np.round(box, 0).astype(int).tolist()
        if depth_m is None:
            notes.append(f"#{index} score={score:.2f} 框={box_text} 深度点不足")
            continue
        notes.append(
            f"#{index} score={score:.2f} 框={box_text} 深度={depth_m * 1000:.0f}mm"
        )
        ranked.append((depth_m, score, raw_mask, box))
    if notes:
        print("SAM3 书分割: " + "; ".join(notes), flush=True)
    if not ranked:
        BOOK_SELECTION_NOTE = (
            "SAM3 没有检出 book"
            if not books
            else "SAM3 检出的书都没有足够的有效深度"
        )
        return None
    depth_m, score, raw_mask, box = min(ranked, key=lambda item: item[0])
    mask = np.asarray(raw_mask, dtype=bool)
    if mask.shape != np.asarray(depth).shape[:2]:
        mask = cv2.resize(
            mask.astype(np.uint8),
            (np.asarray(depth).shape[1], np.asarray(depth).shape[0]),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)
    BOOK_SELECTION_NOTE = (
        "cart+ 使用 SAM3 mask，取深度最近的一本: "
        f"score={score:.2f}, 中位深度={depth_m * 1000:.0f} mm, "
        f"框={np.round(box, 0).astype(int).tolist()}"
    )
    return mask, box


def select_book_candidate(category, masks, depth, intrinsic, c2w):
    """Pick one book detection for the active grasp mode.

    Cart / cart+ keep the nearest standing book. Shelf keeps the tallest
    standing book. Returns None to let grounding use its own score.
    """
    global CART_NEIGHBOR_TOP_MM, CART_OTHER_BOOKS_MM, BOOK_SELECTION_NOTE
    if str(category).lower() != "book" or intrinsic is None or c2w is None:
        return None
    if BOOK_GRASP_MODE == "cart+":
        BOOK_SELECTION_NOTE = (
            "cart+ 使用分割 mask 内深度点的中位数作为书的水平中心"
        )
        return None
    detections = np.asarray(masks)
    if detections.ndim != 3 or detections.shape[0] == 0:
        return None
    measured = []
    for index in range(detections.shape[0]):
        try:
            span = _book_span(_mask_points_base(detections[index], depth, intrinsic, c2w))
        except ValueError:
            continue
        span["index"] = index
        measured.append(span)
    if not measured:
        BOOK_SELECTION_NOTE = "所有书的检测框在去噪后都不是一本立着的书"
        return -1 if BOOK_GRASP_MODE in ("cart", "cart+") else None
    if BOOK_GRASP_MODE in ("cart", "cart+"):
        # Standing book in front of the arm. A flat book can show about
        # 50 mm of vertical span, so height must clearly exceed thickness.
        plausible = [
            item for item in measured if not _cart_candidate_rejection(item)
        ]
        if not plausible:
            BOOK_SELECTION_NOTE = (
                "检测框未通过立书过滤；"
                + ", ".join(
                    f"#{item['index'] + 1}: {_cart_candidate_rejection(item)}"
                    for item in measured
                )
            )
            return -1
        chosen = min(plausible, key=lambda item: item["distance_mm"])
        others = [item for item in plausible if item is not chosen]
        CART_OTHER_BOOKS_MM = [
            (
                item["x"][0] * 1000.0 - _CART_NEIGHBOR_MARGIN_MM,
                item["x"][1] * 1000.0 + _CART_NEIGHBOR_MARGIN_MM,
                item["y"][0] * 1000.0 - _CART_NEIGHBOR_MARGIN_MM,
                item["y"][1] * 1000.0 + _CART_NEIGHBOR_MARGIN_MM,
            )
            for item in others
        ]
        CART_NEIGHBOR_TOP_MM = (
            max(item["z"][1] for item in others) * 1000.0 if others else None
        )
        grasp_note = (
            "从前向后沿 +X 抓书脊"
            if BOOK_GRASP_MODE == "cart+"
            else "竖直向下抓取，与碗相同"
        )
        BOOK_SELECTION_NOTE = (
            f"车上选择最靠近身体的立书（{grasp_note}）: "
            + f"detection {chosen['index'] + 1}/{detections.shape[0]}, "
            f"distance={chosen['distance_mm']:.0f} mm, "
            f"center_xy=({chosen['center'][0] * 1000:.0f}, {chosen['center'][1] * 1000:.0f}) mm, "
            f"size={chosen['thickness_mm']:.0f}x{chosen['length_mm']:.0f}x{chosen['height_mm']:.0f} mm"
        )
    else:
        chosen = max(measured, key=lambda item: (item["height_mm"], item["distance_mm"]))
        CART_OTHER_BOOKS_MM = []
        CART_NEIGHBOR_TOP_MM = None
        BOOK_SELECTION_NOTE = (
            f"架子选择立着的书: detection {chosen['index'] + 1}/{detections.shape[0]}, "
            f"height={chosen['height_mm']:.0f} mm"
        )
    return int(chosen["index"])


def _segment_hits_book(start_xy, end_xy, bounds):
    xmin, xmax, ymin, ymax = bounds
    for scale in np.linspace(0.0, 1.0, 13):
        point = start_xy + (end_xy - start_xy) * scale
        if xmin <= point[0] <= xmax and ymin <= point[1] <= ymax:
            return True
    return False


def _cart_retreat_xy(center_xy):
    """Move toward the robot without crossing another book."""
    direction = -np.asarray(center_xy, dtype=float).reshape(2)
    norm = float(np.linalg.norm(direction))
    if norm < 1.0:
        direction = np.array([-1.0, 0.0])
    else:
        direction = direction / norm
    candidates = [
        center_xy + direction * _CART_RETREAT_MM,
        center_xy + np.array([-_CART_RETREAT_MM, 0.0]),
        center_xy,
    ]
    for goal in candidates:
        if not any(_segment_hits_book(center_xy, goal, bounds) for bounds in CART_OTHER_BOOKS_MM):
            return goal
    return center_xy


def fill_cart_book_meta(obj_meta, pts_cam_m, c2w):
    """Flat cart book: spine along base X, grasped from above."""
    points = np.asarray(pts_cam_m, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 30:
        raise ValueError(f"book point cloud has only {len(points)} points")
    points = points[_dominant_depth_cluster(points[:, 2])]
    points_base, rotation, translation_m = _camera_to_base(points, c2w)
    span = _book_span(points_base)
    center = span["center"]
    x_low, x_high = span["x"]
    y_low, y_high = span["y"]
    z_low, z_high = span["z"]
    observed_height_mm = (z_high - z_low) * 1000.0
    if observed_height_mm > _CART_MAX_SINGLE_BOOK_HEIGHT_MM:
        raise ValueError(
            "cart book mask includes more than one book or cart geometry: "
            f"z={observed_height_mm:.1f}mm"
        )
    # A noisy mask can include points below the selected book. Anchor the
    # effective volume at its top surface and never command the fingers more
    # than 80 mm below that surface.
    effective_z_low = max(
        z_low,
        z_high - _CART_EFFECTIVE_BOOK_HEIGHT_MM / 1000.0,
    )
    center[2] = 0.5 * (effective_z_low + z_high)
    z_low = effective_z_low
    half_x = 0.5 * (x_high - x_low)
    half_y = 0.5 * (y_high - y_low)
    half_z = 0.5 * (z_high - z_low)
    if half_x < 0.04 or half_z < 0.008 or half_y < 0.005:
        raise ValueError(
            "cart book cloud is too small: "
            f"x={2 * half_x * 1000:.1f}mm, "
            f"y={2 * half_y * 1000:.1f}mm, "
            f"z={2 * half_z * 1000:.1f}mm"
        )
    camera_from_base = rotation.T

    def to_camera(point_base):
        return camera_from_base @ (point_base - translation_m)

    obj_meta["centroid"] = _pose(to_camera(center))
    obj_meta["object_size"] = np.array(
        [2.0 * half_x, 2.0 * half_y, 2.0 * half_z], dtype=float
    )
    faces = {
        "bbox_plane_x1": center + np.array([-half_x, 0.0, 0.0]),
        "bbox_plane_x2": center + np.array([half_x, 0.0, 0.0]),
        "bbox_plane_y1": center + np.array([0.0, -half_y, 0.0]),
        "bbox_plane_y2": center + np.array([0.0, half_y, 0.0]),
        "bbox_plane_z1": center + np.array([0.0, 0.0, -half_z]),
        "bbox_plane_z2": center + np.array([0.0, 0.0, half_z]),
    }
    for name, point_base in faces.items():
        obj_meta[name] = _pose(to_camera(point_base))
    hover = center.copy()
    hover[2] = z_high + _CART_HOVER_CLEARANCE_MM / 1000.0
    obj_meta["grasp_0"] = _pose(to_camera(hover))
    return {
        "center_base_mm": (center * 1000.0).tolist(),
        "thickness_mm": 2.0 * half_y * 1000.0,
        "length_mm": 2.0 * half_x * 1000.0,
        "height_mm": 2.0 * half_z * 1000.0,
        "axis_size_mm": [
            2.0 * half_x * 1000.0,
            2.0 * half_y * 1000.0,
            2.0 * half_z * 1000.0,
        ],
        "spine_axis": "x",
        "approach_xy": [0.0, 0.0],
    }


def _fill_mask_depth_center(obj_meta, pts_cam_m, c2w):
    """Book center is the median of the segmented mask's depth points."""
    raw_points = np.asarray(pts_cam_m, dtype=float).reshape(-1, 3)
    finite = np.isfinite(raw_points).all(axis=1) & (raw_points[:, 2] > 1e-6)
    points = raw_points[finite]
    if len(points) < 30:
        raise ValueError(f"书的 mask 里只有 {len(points)} 个有效深度点")
    if float(np.median(points[:, 2])) > 20.0:
        points = points.copy()
        points[:, :] *= 0.001
    points_base, rotation, translation_m = _camera_to_base(points, c2w)
    center = np.median(points_base, axis=0)
    low = np.percentile(points_base, 5.0, axis=0)
    high = np.percentile(points_base, 95.0, axis=0)
    half = np.maximum(0.5 * (high - low), np.array([0.015, 0.015, 0.015]))
    camera_from_base = rotation.T

    def to_camera(point_base):
        return camera_from_base @ (point_base - translation_m)

    obj_meta["centroid"] = _pose(to_camera(center))
    obj_meta["object_size"] = 2.0 * half
    faces = {
        "bbox_plane_x1": center + np.array([-half[0], 0.0, 0.0]),
        "bbox_plane_x2": center + np.array([half[0], 0.0, 0.0]),
        "bbox_plane_y1": center + np.array([0.0, -half[1], 0.0]),
        "bbox_plane_y2": center + np.array([0.0, half[1], 0.0]),
        "bbox_plane_z1": center + np.array([0.0, 0.0, -half[2]]),
        "bbox_plane_z2": center + np.array([0.0, 0.0, half[2]]),
    }
    for name, point_base in faces.items():
        obj_meta[name] = _pose(to_camera(point_base))
    obj_meta["grasp_0"] = obj_meta["centroid"]
    point_mask = finite.copy()
    return {
        "center_base_mm": (center * 1000.0).tolist(),
        "thickness_mm": float(2.0 * half[0] * 1000.0),
        "length_mm": float(2.0 * half[1] * 1000.0),
        "height_mm": float(2.0 * half[2] * 1000.0),
        "approach_xy": [1.0, 0.0],
        "input_points": int(len(raw_points)),
        "denoised_points": int(len(points)),
        "point_mask": point_mask,
    }


def fill_book_meta(obj_meta, pts_cam_m, c2w):
    """Fill book geometry for the mode selected by ``set_book_grasp_mode``."""
    if BOOK_GRASP_MODE == "cart+":
        return _fill_mask_depth_center(obj_meta, pts_cam_m, c2w)
    return fill_standing_book_meta(obj_meta, pts_cam_m, c2w)


def book_top_gripper_euler():
    """Point down with jaws closing across Y, perpendicular to the X spine."""
    gripper_x = np.array([0.0, 1.0, 0.0])
    gripper_z = np.array([0.0, 0.0, -1.0])
    gripper_y = np.cross(gripper_z, gripper_x)
    world_from_gripper = np.column_stack([gripper_x, gripper_y, gripper_z])
    world_from_tool = world_from_gripper @ _RIGHT_GRIPPER_FROM_TOOL.T
    return rotation2angle(world_from_tool).tolist()


def book_cart_targets(object_info):
    """Top grasp: descend on the nearest flat book, lift clear, then retreat."""
    bounding_box = object_info["bounding_box"]
    z1 = _point(bounding_box["bbox_plane_z1"])
    z2 = _point(bounding_box["bbox_plane_z2"])
    center = _point(object_info["center_position"])
    grasp_z = 0.5 * (z1[2] + z2[2])
    top_z = max(z1[2], z2[2])
    safe_z = max(
        top_z + _CART_HOVER_CLEARANCE_MM,
        _CART_MIN_SAFE_Z_MM,
    )
    if CART_NEIGHBOR_TOP_MM is not None:
        safe_z = max(safe_z, CART_NEIGHBOR_TOP_MM + _CART_HOVER_CLEARANCE_MM)

    grasp = np.array([center[0], center[1], grasp_z], dtype=float)
    above = np.array([center[0], center[1], safe_z], dtype=float)
    entry = np.array(
        [
            _CART_ENTRY_X_MM,
            _CART_ENTRY_Y_MM,
            max(safe_z, _CART_ENTRY_Z_MM),
        ],
        dtype=float,
    )
    retreat_xy = _cart_retreat_xy(center[:2])
    carry = np.array([retreat_xy[0], retreat_xy[1], safe_z], dtype=float)
    release_up = carry.copy()
    release_up[2] = safe_z + _PLACE_LIFT_MM
    return {
        "entry": entry.tolist(),
        "pregrasp": above.tolist(),
        "grasp": grasp.tolist(),
        "retreat": above.tolist(),
        "carry": carry.tolist(),
        "place": carry.tolist(),
        "release_up": release_up.tolist(),
        "rotation": book_top_gripper_euler(),
    }


def book_cart_plus_targets(object_info):
    """Front-to-back spine grasp along base +X, then place 60 mm to the left."""
    bounding_box = object_info["bounding_box"]
    faces = [
        _point(bounding_box["bbox_plane_x1"]),
        _point(bounding_box["bbox_plane_x2"]),
        _point(bounding_box["bbox_plane_y1"]),
        _point(bounding_box["bbox_plane_y2"]),
        _point(bounding_box["bbox_plane_z1"]),
        _point(bounding_box["bbox_plane_z2"]),
    ]
    center = _point(object_info["center_position"])
    x_min = min(point[0] for point in faces)
    x_max = max(point[0] for point in faces)
    half_x = 0.5 * (x_max - x_min)
    if half_x < 4.0:
        raise ValueError(
            f"cart+ book is too thin along X to grasp the spine: {2.0 * half_x:.1f}mm"
        )
    z1 = _point(bounding_box["bbox_plane_z1"])
    z2 = _point(bounding_box["bbox_plane_z2"])
    grasp_z = 0.5 * (z1[2] + z2[2])
    grasp = np.array([center[0], center[1], grasp_z], dtype=float)
    pregrasp = grasp.copy()
    pregrasp[0] = x_min - _APPROACH_CLEARANCE_MM
    retreat = grasp.copy()
    retreat[2] = grasp_z + _RETREAT_LIFT_MM
    place = grasp.copy()
    place[1] += _CART_PLUS_PLACE_Y_MM
    place_high = place.copy()
    place_high[2] = grasp_z + _RETREAT_LIFT_MM
    release_up = place.copy()
    release_up[2] = grasp_z + _PLACE_LIFT_MM
    return {
        "pregrasp": pregrasp.tolist(),
        "grasp": grasp.tolist(),
        "retreat": retreat.tolist(),
        "carry": place_high.tolist(),
        "place": place.tolist(),
        "release_up": release_up.tolist(),
        "rotation": book_side_gripper_euler(_CART_PLUS_INTO_BOOK_XY),
    }


def book_gripper_targets(object_info):
    """Return right-gripper stage positions for the active book mode."""
    if BOOK_GRASP_MODE == "cart":
        return book_cart_targets(object_info)
    if BOOK_GRASP_MODE == "cart+":
        return book_cart_plus_targets(object_info)
    return book_shelf_targets(object_info)


def book_shelf_targets(object_info):
    """Return right-gripper stage positions for one standing book."""
    bounding_box = object_info["bounding_box"]
    x1 = _point(bounding_box["bbox_plane_x1"])
    x2 = _point(bounding_box["bbox_plane_x2"])
    z1 = _point(bounding_box["bbox_plane_z1"])
    z2 = _point(bounding_box["bbox_plane_z2"])
    center = _point(object_info["center_position"])
    thin = x2 - x1
    thin_norm = float(np.linalg.norm(thin[:2]))
    if thin_norm < 1.0:
        raise ValueError("book thickness axis is missing")
    toward_robot = thin[:2] / thin_norm
    half_thickness = 0.5 * thin_norm
    grasp_z = 0.5 * (z1[2] + z2[2])

    grasp = np.array([center[0], center[1], grasp_z], dtype=float)
    pregrasp = grasp.copy()
    pregrasp[:2] = grasp[:2] + toward_robot * (half_thickness + _APPROACH_CLEARANCE_MM)
    retreat = pregrasp.copy()
    retreat[2] = grasp_z + _RETREAT_LIFT_MM

    toward_base = -grasp[:2]
    base_norm = float(np.linalg.norm(toward_base))
    if base_norm < 1.0:
        toward_base = np.array([-1.0, 0.0])
    else:
        toward_base = toward_base / base_norm
    place = grasp.copy()
    place[:2] = grasp[:2] + toward_base * _PLACE_DISTANCE_MM
    place_high = place.copy()
    place_high[2] = grasp_z + _RETREAT_LIFT_MM
    release_up = place.copy()
    release_up[2] = grasp_z + _PLACE_LIFT_MM

    into_book = -toward_robot
    return {
        "pregrasp": pregrasp.tolist(),
        "grasp": grasp.tolist(),
        "retreat": retreat.tolist(),
        "carry": place_high.tolist(),
        "place": place.tolist(),
        "release_up": release_up.tolist(),
        "rotation": book_side_gripper_euler(into_book),
    }
