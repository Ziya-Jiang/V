import time
import json
import open3d as o3d
import cv2
import os 
import numpy as np
from easydict import EasyDict

from utils.image_utils import (
    save_rgb_png,
    save_depth_npy,
    turbo_cmap,
)
import book_grasp as book_geom
from utils.transform_utils import mat_to_pos


def normal_to_rotmat(n):
    n = n / np.linalg.norm(n)
    if abs(n[2]) < 0.99:
        a = np.array([0, 0, 1])
    else:
        a = np.array([0, 1, 0])
    x = np.cross(a, n)
    x = x / np.linalg.norm(x)
    y = np.cross(n, x)
    R = np.stack([x, y, n], axis=1)
    return R

def euler_to_matrix(roll, pitch, yaw):
    # 右手系，欧拉角xyz顺序（roll, pitch, yaw），单位度
    rx, ry, rz = np.radians([roll, pitch, yaw])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx),  np.cos(rx)]
    ])
    Ry = np.array([
        [ np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz),  np.cos(rz), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx

def get_sym_equivalent_pose(T_target, T_init):
    """
    返回与T_init最接近且抓取等价的夹爪位姿(仅用numpy实现)
    输入:
        T_target: [4,4] 目标抓取位姿
        T_init:   [4,4] 初始位姿
    返回:
        T_result: [4,4] 推荐位姿
    """
    # 1. 绕自身z轴旋转180°
    R_sym = np.array([
        [-1, 0, 0],
        [ 0,-1, 0],
        [ 0, 0, 1]
    ])
    T_sym = np.eye(4)
    T_sym[:3,:3] = R_sym

    # 2. 目标的对称等价位姿
    T_target_sym = np.dot(T_target, T_sym)

    # 3. 距离计算函数
    def pose_distance(T1, T2, rot_weight=1.0, trans_weight=0.0):
        # 平移距离
        d_trans = np.linalg.norm(T1[:3,3] - T2[:3,3])
        # 旋转距离
        R1 = T1[:3,:3]
        R2 = T2[:3,:3]
        R_rel = np.dot(R1.T, R2)
        trace = np.clip(np.trace(R_rel), -1.0, 3.0)
        theta = np.arccos(np.clip((trace - 1)/2, -1.0, 1.0))  # 弧度
        return rot_weight * theta + trans_weight * d_trans

    dist0 = pose_distance(T_init, T_target)
    dist1 = pose_distance(T_init, T_target_sym)

    return T_target if dist0 <= dist1 else T_target_sym

def get_sym_equivalent_pose_12samples(T_target, T_init):
    """
    返回与T_init最接近且抓取等价的夹爪位姿
    绕Z轴旋转360°，间隔30°采样12个位姿
    根据在左侧还是右侧返回对应的位姿
    
    输入:
        T_target: [4,4] 目标抓取位姿
        T_init:   [4,4] 初始位姿
    返回:
        T_result: [4,4] 推荐位姿
    """
    # 1. 生成12个绕Z轴旋转的位姿 (0°, 30°, 60°, ..., 330°)
    sym_poses = []
    angles = np.linspace(0, 2*np.pi, 20, endpoint=False)
    
    for angle in angles:
        # 创建绕Z轴的旋转矩阵
        R_z = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle),  np.cos(angle), 0],
            [0,             0,             1]
        ])
        
        # 创建齐次变换矩阵
        T_rot = np.eye(4)
        T_rot[:3, :3] = R_z
        
        # 应用旋转变换：绕物体自身Z轴旋转
        T_sym = np.dot(T_target, T_rot)
        sym_poses.append(T_sym)
    
    # 2. 根据位置判断选择位姿
    # 找到最左侧和最右侧的位姿
    leftmost_pose = None
    rightmost_pose = None
    min_x = float('inf')
    max_x = float('-inf')
    
    for pose in sym_poses:
        x_pos = pose[0, 3]  # X坐标
        # x_pos = pose[1, 3]
        if x_pos < min_x:
            min_x = x_pos
            leftmost_pose = pose
        if x_pos > max_x:
            max_x = x_pos
            rightmost_pose = pose
    
    # 3. 判断原始目标是在左侧还是右侧
    target_x = T_target[0, 3]
    # target_x = T_target[1, 3]
    print([mat_to_pos(s) for s in sym_poses])
    if target_x < 0:  # 原始目标在左侧
        # 返回最左侧的位姿（左下角）
        return leftmost_pose
    else:  # 原始目标在右侧
        # 返回最右侧的位姿（右下角）
        return rightmost_pose

def get_rotation_pose_20_samples(grasp2obj, obj2world):
    sym_poses = []
    angles = np.linspace(0, 2*np.pi, 20, endpoint=False)
    
    for angle in angles:
        # 创建绕Z轴的旋转矩阵
        R_z = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle),  np.cos(angle), 0],
            [0,             0,             1]
        ])
        
        # 创建齐次变换矩阵
        T_rot = np.eye(4)
        T_rot[:3, :3] = R_z
        
        # 应用旋转变换：绕物体自身Z轴旋转
        # T_sym = np.dot(grasp2obj, T_rot)
        T_sym = T_rot @ grasp2obj
        T_sym = obj2world @ T_sym
        sym_poses.append(T_sym)
    
    # x_coords = [pose[0, 3] for pose in sym_poses]
    # y_coords = [pose[1, 3] for pose in sym_poses]
    # import matplotlib.pyplot as plt

    # plt.figure(figsize=(8, 8))
    # plt.scatter(x_coords, y_coords)
    # plt.scatter(obj2world[0, 3], obj2world[1, 3], color='red')  # 物体中心
    # plt.axis('equal')
    # plt.savefig('rotation_poses.png')
    
    # 2. 根据位置判断选择位姿
    # 找到最左侧和最右侧的位姿
    leftmost_pose = None
    rightmost_pose = None
    min_y = float('inf')
    max_y = float('-inf')
    
    for pose in sym_poses:
        y_pos = pose[1, 3]  # X坐标
        # x_pos = pose[1, 3]
        if y_pos < min_y:
            min_y = y_pos
            leftmost_pose = pose
        if y_pos > max_y:
            max_y = y_pos
            rightmost_pose = pose
    
    # 3. 判断原始目标是在左侧还是右侧
    target_x = sym_poses[0][1, 3]
    # target_x = T_target[1, 3]
    # print([mat_to_pos(s) for s in sym_poses])
    if target_x < 0:  # 原始目标在左侧
        # 返回最左侧的位姿（左下角）
        return leftmost_pose
    else:  # 原始目标在右侧
        # 返回最右侧的位姿（右下角）
        return rightmost_pose

def get_bbox_plane_poses(size, sRT):
    """
    给定物体尺寸、位姿, 计算6个面在相机坐标系下的4x4位姿
    Args:
        size: (3,) 物体尺寸 (x, y, z)
        sRT: (4,4) 物体位姿
    Returns:
        planes_poses: (6, 4, 4) ndarray, 顺序为正、背、左、右、顶、底
    """
    pred_scale = np.cbrt(np.linalg.det(sRT[:3, :3]))

    x, y, z = size
    # 物体系下的面心和法向顺序
    centers = np.array([
        [ +x/2,   0,    0],   # 正面
        [ -x/2,   0,    0],   # 背面
        [   0,  +y/2,   0],   # 左面
        [   0,  -y/2,   0],   # 右面
        [   0,    0,  +z/2],  # 顶面
        [   0,    0,  -z/2],  # 底面
    ])
    normals = np.array([
        [ +1,  0,  0],  # 正面
        [ -1,  0,  0],  # 背面
        [  0, +1,  0],  # 左面
        [  0, -1,  0],  # 右面
        [  0,  0, +1],  # 顶面
        [  0,  0, -1],  # 底面
    ])
    planes_poses = []
    for i in range(6):
        R = normal_to_rotmat(normals[i])
        T = np.eye(4)
        T[:3,:3] = R
        T[:3, 3] = centers[i]
        # 变换到相机系
        T_cam = sRT @ T
        T_cam[:3, :3] = T_cam[:3, :3] / pred_scale
        planes_poses.append(T_cam)
    return np.stack(planes_poses)  # (6,4,4)


def filter_depth_consistency(
    depth_mm,
    valid_mask,
    median_kernel,
    outlier_threshold_mm,
):
    """Reject isolated depth pixels without smoothing object surfaces."""
    if median_kernel <= 1 or outlier_threshold_mm <= 0:
        return valid_mask.copy(), 0
    masked_depth = np.where(valid_mask, depth_mm, 0.0).astype(np.float32)
    local_median = cv2.medianBlur(masked_depth, median_kernel)
    support = cv2.boxFilter(
        valid_mask.astype(np.float32),
        ddepth=-1,
        ksize=(median_kernel, median_kernel),
        normalize=False,
        borderType=cv2.BORDER_REPLICATE,
    )
    minimum_support = max(3, int(np.ceil(median_kernel ** 2 * 0.5)))
    isolated = (
        valid_mask
        & (support >= minimum_support)
        & (local_median > 0)
        & (np.abs(depth_mm - local_median) > outlier_threshold_mm)
    )
    filtered_mask = valid_mask & ~isolated
    return filtered_mask, int(np.count_nonzero(isolated))


def save_orbbec_scene_point_cloud(
    output_dir,
    rgb,
    depth_m,
    depth_mask,
    intrinsic,
    distortion,
    voxel_size_m,
    statistical_neighbors,
    statistical_std_ratio,
):
    """Filter, voxelize, and save metric XYZRGB in the Orbbec camera frame."""
    rows, columns = np.nonzero(depth_mask)
    pixels = np.column_stack((columns, rows)).astype(np.float64)
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
    normalized = cv2.undistortPoints(
        pixels.reshape(-1, 1, 2),
        intrinsic,
        distortion,
    ).reshape(-1, 2)
    z = depth_m[rows, columns].astype(np.float64)
    points = np.column_stack(
        (normalized[:, 0] * z, normalized[:, 1] * z, z)
    )
    colors = np.asarray(rgb)[rows, columns, :3]
    raw_cloud = np.concatenate(
        (points.astype(np.float32), colors.astype(np.float32)),
        axis=1,
    )
    np.save(
        os.path.join(output_dir, "orbbec_scene_point_cloud_rgb_raw.npy"),
        raw_cloud,
    )
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(
        colors.astype(np.float64) / 255.0
    )
    raw_count = len(point_cloud.points)
    statistical_count = raw_count
    if (
        statistical_neighbors > 0
        and raw_count > statistical_neighbors
    ):
        point_cloud, _ = point_cloud.remove_statistical_outlier(
            nb_neighbors=statistical_neighbors,
            std_ratio=statistical_std_ratio,
        )
        statistical_count = len(point_cloud.points)
    if voxel_size_m > 0:
        point_cloud = point_cloud.voxel_down_sample(voxel_size_m)
    optimized_count = len(point_cloud.points)
    optimized_points = np.asarray(
        point_cloud.points,
        dtype=np.float32,
    )
    optimized_colors = np.rint(
        np.asarray(point_cloud.colors) * 255.0
    ).clip(0, 255).astype(np.float32)
    optimized_cloud = np.concatenate(
        (optimized_points, optimized_colors),
        axis=1,
    )
    np.save(
        os.path.join(output_dir, "orbbec_scene_point_cloud_rgb.npy"),
        optimized_cloud,
    )
    o3d.io.write_point_cloud(
        os.path.join(output_dir, "orbbec_scene_point_cloud.ply"),
        point_cloud,
    )
    diagnostics = {
        "coordinate_frame": "orbbec_camera",
        "coordinate_unit": "m",
        "raw_points": raw_count,
        "after_statistical_filter": statistical_count,
        "optimized_points": optimized_count,
        "statistical_neighbors": statistical_neighbors,
        "statistical_std_ratio": statistical_std_ratio,
        "voxel_size_m": voxel_size_m,
    }
    return diagnostics


def ram_main(prompt, output_dir, debug_dir, cur_dir, extern_camera_class,
            vlm_categories, masks, xyxys, class_ids, RAM_Model, ObjectTemplate, RAM_IO,
            logger=None, enable_debug=False,
            depth_median_kernel=5, depth_outlier_threshold_mm=50.0,
            voxel_size_m=0.003, statistical_neighbors=20,
            statistical_std_ratio=2.0,
            camera_pose_correction_callback=None, stop_after_camera_pose_correction=False):

    t0 = time.time()
    rgb_original = extern_camera_class.rgb_image
    depth_mm = np.asarray(extern_camera_class.depth_map, dtype=np.float32)
    expected_shape = (
        extern_camera_class.height,
        extern_camera_class.width,
    )
    if depth_mm.shape != expected_shape:
        raise RuntimeError(
            "Orbbec RGB-D depth shape does not match calibrated RGB: "
            f"depth={depth_mm.shape}, expected={expected_shape}"
        )
    minimum_depth_m = float(
        os.environ.get("ROBOTRAM_ORBBEC_PCL_MIN_DEPTH_M", "0.10")
    )
    maximum_depth_m = float(
        os.environ.get("ROBOTRAM_ORBBEC_PCL_MAX_DEPTH_M", "3.00")
    )
    rgbd_depth_map_ext = depth_mm * 0.001
    depth_mask = (
        np.isfinite(rgbd_depth_map_ext)
        & (rgbd_depth_map_ext >= minimum_depth_m)
        & (rgbd_depth_map_ext <= maximum_depth_m)
    )
    range_valid_count = int(np.count_nonzero(depth_mask))
    depth_mask, rejected_depth_count = filter_depth_consistency(
        depth_mm,
        depth_mask,
        depth_median_kernel,
        depth_outlier_threshold_mm,
    )
    rgbd_depth_map_ext = np.where(
        depth_mask,
        rgbd_depth_map_ext,
        0.0,
    ).astype(np.float32)
    K = np.asarray(extern_camera_class.K, dtype=np.float64)
    if K.shape != (3, 3):
        raise RuntimeError(f"Invalid calibrated Orbbec K shape: {K.shape}")
    valid_count = int(np.count_nonzero(depth_mask))
    if valid_count == 0:
        raise RuntimeError(
            "Orbbec RGB-D frame has no valid metric depth in configured range."
        )
    cloud_diagnostics = save_orbbec_scene_point_cloud(
        output_dir,
        extern_camera_class.rgb_image,
        rgbd_depth_map_ext,
        depth_mask,
        K,
        extern_camera_class.D,
        voxel_size_m,
        statistical_neighbors,
        statistical_std_ratio,
    )
    cloud_diagnostics.update(
        {
            "range_valid_depth_pixels": range_valid_count,
            "rejected_local_depth_outliers": rejected_depth_count,
            "retained_depth_pixels": valid_count,
            "depth_median_kernel": depth_median_kernel,
            "depth_outlier_threshold_mm": depth_outlier_threshold_mm,
        }
    )
    with open(
        os.path.join(output_dir, "orbbec_point_cloud_filter_report.json"),
        "w",
        encoding="utf-8",
    ) as report_file:
        json.dump(cloud_diagnostics, report_file, indent=2)

    # Preserve legacy artifact names so Rerun and Step3-5 remain unchanged.
    save_depth_npy(
        rgbd_depth_map_ext,
        os.path.join(output_dir, "vggt_depth_ext.npy"),
    )
    np.save(os.path.join(output_dir, "vggt_K.npy"), K)
    np.save(
        os.path.join(output_dir, "orbbec_rgbd_depth_m.npy"),
        rgbd_depth_map_ext,
    )
    np.save(os.path.join(output_dir, "orbbec_rgbd_K.npy"), K)
    if enable_debug:
        save_rgb_png(
            turbo_cmap(rgbd_depth_map_ext),
            os.path.join(output_dir, "orbbec_rgbd_depth.png"),
        )
    if logger:
        logger.info(
            "Orbbec RGB-D depth: range-valid=%d, local outliers=%d, "
            "retained=%d/%d (%.1f%%)",
            range_valid_count,
            rejected_depth_count,
            valid_count,
            depth_mask.size,
            valid_count * 100.0 / depth_mask.size,
        )
        logger.info(
            "Point-cloud filters: raw=%d, statistical=%d, voxelized=%d "
            "(voxel=%.1fmm)",
            cloud_diagnostics["raw_points"],
            cloud_diagnostics["after_statistical_filter"],
            cloud_diagnostics["optimized_points"],
            voxel_size_m * 1000.0,
        )

    # Run head-camera correction against hardware metric depth before object
    # poses are transformed through extern_camera_class.c2w.
    if camera_pose_correction_callback is not None:
        camera_pose_correction_callback(
            depth_m=rgbd_depth_map_ext,
            intrinsic=K,
            valid_mask=depth_mask,
        )
        if stop_after_camera_pose_correction:
            return {}, rgb_original

    t2 = time.time()
    # get primitive poses of all objects in the scene
    objects_meta = EasyDict({})
    for object in vlm_categories:
        t2_1 = time.time()
        obj_name = object['name']
        obj_id = int(object['id'])

        if obj_id < 0:
            if logger: logger.warning(f"Object ID {obj_id} is invalid, skipping...")
            continue

        if obj_id >= masks.shape[0] or not np.any(masks[obj_id]):
            if logger: logger.warning(
                f"No grounding mask for object {obj_id} ({obj_name}), skipping..."
            )
            continue

        obj_category = object['category']
        objects_meta[str(obj_id)] = EasyDict({})

        obj_mask = masks[obj_id, ...]
        obj_xyxy = xyxys[obj_id, ...]

        inst_mask = np.logical_and(obj_mask, depth_mask)
        # inst_mask = obj_mask
        # save_rgb_png((inst_mask * 255).astype(np.uint8), os.path.join(debug_dir, f'ram_{obj_category}_id{obj_id:02}.png'))

        inst_img = RAM_IO.extract_rgb_patch(extern_camera_class.rgb_image, obj_mask)

        # if 'table' in obj_category:
        #     choose, pts, pts2d = RAM_IO.extract_obj_pts(inst_mask, rgbd_depth_map_ext, 1.0, K, True)
        # else:
        choose, pts, pts2d = RAM_IO.extract_obj_pts(inst_mask, rgbd_depth_map_ext, 1.0, K, True)

        t2_2 = time.time()
        if logger: logger.info(f">>> Processing object: {obj_name}, ID: {obj_id}, Category: {obj_category}")
        if obj_category == "book":
            try:
                book_geometry = book_geom.fill_book_meta(
                    objects_meta[str(obj_id)],
                    pts,
                    extern_camera_class.c2w,
                )
                point_mask = np.asarray(
                    book_geometry.pop("point_mask", []),
                    dtype=bool,
                )
                if (
                    point_mask.shape == (len(pts),)
                    and np.count_nonzero(point_mask) >= 30
                ):
                    pts = pts[point_mask]
                    if len(pts2d) == len(point_mask):
                        pts2d = pts2d[point_mask]
            except ValueError as error:
                if logger:
                    logger.warning("跳过书 %s: %s", obj_name, error)
                del objects_meta[str(obj_id)]
                continue
            if logger:
                logger.info(
                    "书点云抓取(%s): center=%s mm, thickness=%.1f mm, height=%.1f mm, approach=%s, denoise=%d/%d",
                    book_geom.BOOK_GRASP_MODE,
                    np.round(book_geometry["center_base_mm"], 1).tolist(),
                    book_geometry["thickness_mm"],
                    book_geometry["height_mm"],
                    np.round(book_geometry["approach_xy"], 3).tolist(),
                    int(book_geometry.get("denoised_points", 0)),
                    int(book_geometry.get("input_points", 0)),
                )
            if enable_debug:
                # The book path bypasses RAM template estimation, so persist
                # its selected cloud here before the early ``continue``.
                # RerunPipelineLogger picks up *_point_cloud_rgb.npy and
                # renders it in the calibrated world view.
                np.save(
                    os.path.join(output_dir, f"{obj_name}_point_cloud_pts.npy"),
                    pts,
                )
                rgb_image = np.asarray(extern_camera_class.rgb_image)
                if rgb_image.ndim == 3 and rgb_image.shape[2] >= 3:
                    pixel_coordinates = np.rint(pts2d).astype(np.int64)
                    valid_pixels = (
                        (pixel_coordinates[:, 0] >= 0)
                        & (pixel_coordinates[:, 0] < rgb_image.shape[1])
                        & (pixel_coordinates[:, 1] >= 0)
                        & (pixel_coordinates[:, 1] < rgb_image.shape[0])
                    )
                    if np.any(valid_pixels):
                        valid_points = pts[valid_pixels]
                        valid_pixels_xy = pixel_coordinates[valid_pixels]
                        point_colors = rgb_image[
                            valid_pixels_xy[:, 1],
                            valid_pixels_xy[:, 0],
                            :3,
                        ]
                        colored_point_cloud = np.concatenate(
                            (valid_points, point_colors.astype(np.float32)),
                            axis=1,
                        )
                        np.save(
                            os.path.join(
                                output_dir,
                                f"{obj_name}_point_cloud_rgb.npy",
                            ),
                            colored_point_cloud,
                        )
                with open(
                    os.path.join(output_dir, "book_grasp_geometry.json"),
                    "w",
                    encoding="utf-8",
                ) as geometry_file:
                    json.dump(
                        {
                            "object_id": int(obj_id),
                            "object_name": obj_name,
                            "mode": book_geom.BOOK_GRASP_MODE,
                            "selection": book_geom.BOOK_SELECTION_NOTE,
                            **book_geometry,
                        },
                        geometry_file,
                        indent=2,
                        ensure_ascii=False,
                    )
            continue
        ObjectTemplate.load_template(obj_category)
        
        # _, zed_pts, _ = RAM_IO.extract_obj_pts(inst_mask, extern_camera_class.depth_map, 1.0, K)
        # pts = pts + (np.mean(zed_pts, axis=0) - np.mean(pts, axis=0))

        if enable_debug:
            detections_npy_path = os.path.join(output_dir, f"{obj_name}_point_cloud_pts.npy")
            np.save(detections_npy_path, pts)

            # pts2d stores the original external-camera pixel coordinates in
            # (x, y) order. Preserve the RGB value for every valid 3D point
            # so the web viewer can render an N×6 XYZRGB point cloud.
            pcd_points = pts
            pcd_colors = None
            rgb_image = extern_camera_class.rgb_image
            if rgb_image is not None:
                rgb_image = np.asarray(rgb_image)
            if rgb_image is not None and rgb_image.ndim == 3 and rgb_image.shape[2] >= 3:
                pixel_coordinates = np.rint(pts2d).astype(np.int64)
                valid_pixels = (
                    (pixel_coordinates[:, 0] >= 0)
                    & (pixel_coordinates[:, 0] < rgb_image.shape[1])
                    & (pixel_coordinates[:, 1] >= 0)
                    & (pixel_coordinates[:, 1] < rgb_image.shape[0])
                )
            else:
                valid_pixels = np.zeros(len(pts), dtype=bool)

            if np.any(valid_pixels):
                valid_points = pts[valid_pixels]
                valid_pixels_xy = pixel_coordinates[valid_pixels]
                point_colors = rgb_image[
                    valid_pixels_xy[:, 1],
                    valid_pixels_xy[:, 0],
                    :3,
                ]
                colored_point_cloud = np.concatenate(
                    (valid_points, point_colors.astype(np.float32)),
                    axis=1,
                )
                colored_npy_path = os.path.join(
                    output_dir,
                    f"{obj_name}_point_cloud_rgb.npy",
                )
                np.save(colored_npy_path, colored_point_cloud)
                pcd_points = valid_points
                pcd_colors = point_colors.astype(np.float64) / 255.0
                if logger:
                    logger.info(
                        "Saved RGB point cloud: %s (%d points)",
                        colored_npy_path,
                        len(colored_point_cloud),
                    )

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pcd_points)
            if pcd_colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(pcd_colors)
            o3d.io.write_point_cloud(os.path.join(debug_dir, f'ram_{obj_category}_id{obj_id:02}.ply'), pcd)

        RAM_Model.viewpoint_estimation(ObjectTemplate, inst_img, choose, pts)
        RAM_Model.ramk_nocs_estimation(ObjectTemplate, inst_img, choose, pts)

        pred_sRT = RAM_Model.best_sRT

        # unique_nocs = RAM_Model.best_nocs
        # unique_pts = RAM_Model.best_pts

        # if pred_sRT is not None:
        #     pts1 = transform_coordinates_3d(unique_pts.T, np.linalg.inv(pred_sRT)).T
        #     pts_dis = np.linalg.norm(pts1 - unique_nocs, axis=1)
        #     inlier_idx = list(np.where(pts_dis <= 0.1)[0])
        #     pred_size = 2 * np.max(abs(unique_nocs[inlier_idx]), axis=0)
        # else:
        #     pred_size = 2 * np.max(abs(unique_nocs), axis=0)

        vis_size = RAM_Model.best_size / np.cbrt(np.linalg.det(RAM_Model.best_sRT[:3, :3]))

        # debug
        dis_img = extern_camera_class.rgb_image.copy()
        dis_img = RAM_Model.vis_pose(dis_img, pred_sRT, vis_size, K)
        if enable_debug: save_rgb_png(dis_img, os.path.join(debug_dir, f'ram_{obj_category}_id{obj_id:02}_pose.png'))
        
        t2_3 = time.time()
        grasps = RAM_Model.grasp_estimation(ObjectTemplate, pts, pts2d, K)
        grasp_candidates = []
        dis_img = RAM_Model.vis_pose(dis_img, pred_sRT, vis_size, K)
        for grasp_name, grasp_meta in grasps.items():
            grasp_pixel = grasp_meta['grasp_pixel']
            grasp_pose = grasp_meta['grasp_pose']

            # debug
            dis_img = extern_camera_class.rgb_image.copy()
            dis_img = RAM_Model.vis_grasp_point(dis_img, grasp_pixel)
            
            

            # 4 * 4 pose matrix in camera coordinate system
            robot_init = np.eye(4)
            robot_init[:3,:3] = euler_to_matrix(-180, 0, -90)
            
            robot_optimal_grasp_pose = get_sym_equivalent_pose(extern_camera_class.c2w @ grasp_pose, robot_init)
            # robot_optimal_grasp_pose = get_sym_equivalent_pose_12samples(extern_camera_class.c2w @ grasp_pose, robot_init)
            
            objects_meta[str(obj_id)][grasp_name] = np.linalg.inv(extern_camera_class.c2w) @ robot_optimal_grasp_pose
            grasp_candidates.append(grasp_pose)
        
        # if enable_debug: save_rgb_png(dis_img, os.path.join(debug_dir, f'ram_{obj_category}_id{obj_id:02}_{grasp_name}.png'))
        if enable_debug: save_rgb_png(dis_img, os.path.join(debug_dir, f'ram_{obj_category}_id{obj_id:02}.png'))
        t2_4 = time.time()
        # obj2cam = objects_meta[str(obj_id)]['centroid']
        
        # TODO:
        # for dict.key == "grasp" and catorogy is symm
        # RT1: obj2cam  --> objects_meta[str(obj_id)]['centroid'], 
        # RT1: grasp2cam --> objects_meta[str(obj_id)]['grasp_o' or 'grasp_1'], 
        # cam2world --> extern_camera_class.c2w
        # grasp2obj = cam2obj @ grasp2cam -> inv(obj2cam) @ grasp2cam
        # Rz, grasp_new2obj = Rz @ grasp2obj
        # grasp_new2cam = obj2cam @ grasp_new2obj
        # grasp_new2world = cam2world @ grasp_new2cam
        # TODO: remove function plane to reduce time
        # planes = RAM_Model.function_plane_estimation(ObjectTemplate, pts)
        # for plane_name, plane_meta in planes.items():
        #     function_point = plane_meta['position']
        #     function_normal = plane_meta['orientation']

        #     # debug
        #     dis_img = extern_camera_class.rgb_image.copy()
        #     function_dis = RAM_Model.vis_function_plane(dis_img, function_point, function_normal, pred_sRT, K)
        #     save_rgb_png(function_dis, os.path.join(debug_dir, f'ram_{obj_category}_id{obj_id:02}_{plane_name}.png'))

        #     # 4 * 4 pose matrix in camera coordinate system
        #     objects_meta[str(obj_id)][plane_name] = np.eye(4)
        #     objects_meta[str(obj_id)][plane_name][:3, :3] = normal_to_rotmat(function_normal)
        #     objects_meta[str(obj_id)][plane_name][:3, 3] = function_point.flatten()
            
        #     pred_scale = np.cbrt(np.linalg.det(pred_sRT[:3, :3]))
        #     objects_meta[str(obj_id)][plane_name] = pred_sRT @ objects_meta[str(obj_id)][plane_name]
        #     objects_meta[str(obj_id)][plane_name][:3, :3] = objects_meta[str(obj_id)][plane_name][:3, :3] / pred_scale

            # print('plane position in world frame:', transform_coordinates_3d(objects_meta[str(obj_id)][plane_name][:3, 3][:, np.newaxis], extern_camera_class.c2w))

        # 4 * 4 pose matrix in camera coordinate system of the centroid of the bounding box
        
        # centroid = np.eye(4)
        # centroid[:3, :3] = pred_sRT[:3, :3] / np.cbrt(np.linalg.det(pred_sRT[:3, :3]))
        # centroid[:3, 3] = pred_sRT[:3, 3].flatten()
        # centroid[:3, 2] = np.array([0, 0, 1])
        # centroid[:3, 0] = np.cross(centroid[:3, 1], centroid[:3, 2])
        # centroid[:3, 1] = np.cross(centroid[:3, 2], centroid[:3, 0])
        
        objects_meta[str(obj_id)]['centroid'] = pred_sRT
        objects_meta[str(obj_id)]['object_size'] = RAM_Model.best_size
        

        # (前,后,左,右,上,下)
        bbox_planes = get_bbox_plane_poses(objects_meta[str(obj_id)]['object_size'] / np.cbrt(np.linalg.det(RAM_Model.best_sRT[:3, :3])), \
                                                    objects_meta[str(obj_id)]['centroid'])
        objects_meta[str(obj_id)]['bbox_plane_x1'] = bbox_planes[0]
        objects_meta[str(obj_id)]['bbox_plane_x2'] = bbox_planes[1]
        objects_meta[str(obj_id)]['bbox_plane_y1'] = bbox_planes[2]
        objects_meta[str(obj_id)]['bbox_plane_y2'] = bbox_planes[3]
        objects_meta[str(obj_id)]['bbox_plane_z1'] = bbox_planes[4]
        objects_meta[str(obj_id)]['bbox_plane_z2'] = bbox_planes[5]

        obj_template_info = ObjectTemplate.template_info[obj_category]
        if obj_template_info['is_symmetric'] == 'True':
            for grasp_name, grasp_meta in grasps.items():
                obj2cam = objects_meta[str(obj_id)]['centroid']
                grasp2cam = objects_meta[str(obj_id)][grasp_name]
                cam2world = extern_camera_class.c2w
                grasp2obj = np.linalg.inv(obj2cam) @ grasp2cam
                obj2world = extern_camera_class.c2w @ obj2cam 
                grasp2world = get_rotation_pose_20_samples(grasp2obj, obj2world)
                grasp2cam_optmal = np.linalg.inv(extern_camera_class.c2w)@grasp2world
                objects_meta[str(obj_id)][grasp_name] = grasp2cam_optmal

    #     t2_5 = time.time()
    #     print(f"{obj_name} point cloud time:          ", t2_2 - t2_1)
    #     print(f"{obj_name} RAM_Model estimation time: ", t2_3 - t2_2)
    #     print(f"{obj_name} grasp pose time:           ", t2_4 - t2_3)
    #     print(f"{obj_name} bbox time:                 ", t2_5 - t2_4)
    #     print(f"{obj_name} total time:                ", t2_5 - t2_1)
    # t3 = time.time()
    # print("Read realsense time:     ", t1 - t0)
    # print("RGB-D preparation time:  ", t2 - t0)
    # print("Process categories time: ", t3 - t2)

    # save the objects meta information a npy file
    # objects_meta_npy_path = os.path.join(output_dir, "ram_objects_meta.npy")
    # np.save(objects_meta_npy_path, objects_meta)
    # if logger: logger.info(f">>> Save objects meta information to {objects_meta_npy_path}.")
    # return objects_meta
    return objects_meta, rgb_original
