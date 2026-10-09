import time
import sys
import open3d as o3d
import cv2
import os 
import numpy as np
import torch  
from os.path import join as pjoin
import logging
from easydict import EasyDict

from utils.image_utils import (
    save_rgb_png,
    save_depth_npy,
    save_depth_png,
    turbo_cmap,
    prepare_third_camera_rgb_for_vggt,
)
from utils.transform_utils import mat_to_pos
from visions.vggt.vggt.utils.pose_enc import pose_encoding_to_extri_intri
from vggt_service_simple import VGGTClient
from PIL import Image
from torchvision import transforms as TF

# Gemini 305 dual-RGB factory baseline measured from the SDK extrinsics:
# 17.98018934 mm. It anchors the otherwise scale-ambiguous VGGT reconstruction
# in metric meters. Override only when the camera hardware configuration changes.
ORBBEC_DUAL_RGB_BASELINE_METERS = float(
    os.environ.get("ROBOTRAM_ORBBEC_DUAL_RGB_BASELINE_M", "0.01798018934")
)


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


def load_and_preprocess_images_from_array(image_array_list, mode="crop"):
    """
    A quick start function to load and preprocess images for model input.
    This assumes the images should have the same shape for easier batching, but our model can also work well with different shapes.

    Args:
        image_path_list (list): List of paths to image files
        mode (str, optional): Preprocessing mode, either "crop" or "pad".
                             - "crop" (default): Sets width to 518px and center crops height if needed.
                             - "pad": Preserves all pixels by making the largest dimension 518px
                               and padding the smaller dimension to reach a square shape.

    Returns:
        torch.Tensor: Batched tensor of preprocessed images with shape (N, 3, H, W)

    Raises:
        ValueError: If the input list is empty or if mode is invalid

    Notes:
        - Images with different dimensions will be padded with white (value=1.0)
        - A warning is printed when images have different shapes
        - When mode="crop": The function ensures width=518px while maintaining aspect ratio
          and height is center-cropped if larger than 518px
        - When mode="pad": The function ensures the largest dimension is 518px while maintaining aspect ratio
          and the smaller dimension is padded to reach a square shape (518x518)
        - Dimensions are adjusted to be divisible by 14 for compatibility with model requirements
    """
    # Check for empty list
    if len(image_array_list) == 0:
        raise ValueError("At least 1 image is required")

    # Validate mode
    if mode not in ["crop", "pad"]:
        raise ValueError("Mode must be either 'crop' or 'pad'")

    images = []
    shapes = set()
    to_tensor = TF.ToTensor()
    target_size = 518

    # First process all images and collect their shapes
    for image_array in image_array_list:
        # Open image
        # img = Image.open(image_path)
        img = Image.fromarray(image_array)

        # If there's an alpha channel, blend onto white background:
        if img.mode == "RGBA":
            # Create white background
            background = Image.new("RGBA", img.size, (255, 255, 255, 255))
            # Alpha composite onto the white background
            img = Image.alpha_composite(background, img)

        # Now convert to "RGB" (this step assigns white for transparent areas)
        img = img.convert("RGB")

        width, height = img.size

        if mode == "pad":
            # Make the largest dimension 518px while maintaining aspect ratio
            if width >= height:
                new_width = target_size
                new_height = round(height * (new_width / width) / 14) * 14  # Make divisible by 14
            else:
                new_height = target_size
                new_width = round(width * (new_height / height) / 14) * 14  # Make divisible by 14
        else:  # mode == "crop"
            # Original behavior: set width to 518px
            new_width = target_size
            # Calculate height maintaining aspect ratio, divisible by 14
            new_height = round(height * (new_width / width) / 14) * 14

        # Resize with new dimensions (width, height)
        img = img.resize((new_width, new_height), Image.Resampling.BICUBIC)
        img = to_tensor(img)  # Convert to tensor (0, 1)

        # Center crop height if it's larger than 518 (only in crop mode)
        if mode == "crop" and new_height > target_size:
            start_y = (new_height - target_size) // 2
            img = img[:, start_y : start_y + target_size, :]

        # For pad mode, pad to make a square of target_size x target_size
        if mode == "pad":
            h_padding = target_size - img.shape[1]
            w_padding = target_size - img.shape[2]

            if h_padding > 0 or w_padding > 0:
                pad_top = h_padding // 2
                pad_bottom = h_padding - pad_top
                pad_left = w_padding // 2
                pad_right = w_padding - pad_left

                # Pad with white (value=1.0)
                img = torch.nn.functional.pad(
                    img, (pad_left, pad_right, pad_top, pad_bottom), mode="constant", value=1.0
                )

        shapes.add((img.shape[1], img.shape[2]))
        images.append(img)

    # Check if we have different shapes
    # In theory our model can also work well with different shapes
    if len(shapes) > 1:
        print(f"Warning: Found images with different shapes: {shapes}")
        # Find maximum dimensions
        max_height = max(shape[0] for shape in shapes)
        max_width = max(shape[1] for shape in shapes)

        # Pad images if necessary
        padded_images = []
        for img in images:
            h_padding = max_height - img.shape[1]
            w_padding = max_width - img.shape[2]

            if h_padding > 0 or w_padding > 0:
                pad_top = h_padding // 2
                pad_bottom = h_padding - pad_top
                pad_left = w_padding // 2
                pad_right = w_padding - pad_left

                img = torch.nn.functional.pad(
                    img, (pad_left, pad_right, pad_top, pad_bottom), mode="constant", value=1.0
                )
            padded_images.append(img)
        images = padded_images

    images = torch.stack(images)  # concatenate images

    # Ensure correct shape when single image
    if len(image_array_list) == 1:
        # Verify shape is (1, C, H, W)
        if images.dim() == 3:
            images = images.unsqueeze(0)

    return images

def ram_main(prompt, output_dir, debug_dir, cur_dir, extern_camera2, extern_camera_class,
            vlm_categories, masks, xyxys, class_ids, RAM_Model, ObjectTemplate, RAM_IO, device, dtype, vggt_model,
            use_vggt_service=True, vggt_service_url="http://localhost:5001", logger=None, enable_debug=False,
            camera_pose_correction_callback=None, stop_after_camera_pose_correction=False):

    t0 = time.time()
    rgb_ext2, depth_ext2 = extern_camera2.capture_image()
    rgb_ext2_original = rgb_ext2
    rgb_ext2 = prepare_third_camera_rgb_for_vggt(
        rgb_ext2, extern_camera_class.width, extern_camera_class.height
    )
    if enable_debug:
        rgb_ext2_pth = pjoin(output_dir, 'rgb_ext2.png')
        save_rgb_png(rgb_ext2, rgb_ext2_pth)

    t1 = time.time()
    # VGGT推理 - 支持服务模式和直接推理模式
    if use_vggt_service:
        if logger: logger.info("使用VGGT服务进行推理...")
        
        # 使用VGGT服务
        vggt_client = VGGTClient(url=vggt_service_url)
        
        # 健康检查
        health = vggt_client.health_check()
        if health.get('status') != 'ok':
            if logger: logger.error(f"VGGT服务不可用: {health}")
            sys.exit(1)
        
        # 如果模型未加载，先加载
        if not health.get('model_loaded', False):
            if logger: logger.info("VGGT模型未加载，正在加载...")
            load_result = vggt_client.load_model()
            if not load_result.get('success', False):
                if logger: logger.error(f"VGGT模型加载失败: {load_result}")
                sys.exit(1)
        
        # 执行推理
        rgb_ext_left_pth = pjoin(output_dir, 'rgb_ext_left.png')
        rgb_ext_right_pth = pjoin(output_dir, 'rgb_ext_right.png')
        rgb_ext2_pth = pjoin(output_dir, 'rgb_ext2.png')
        image_names = [rgb_ext_left_pth, rgb_ext_right_pth, rgb_ext2_pth]
        
        inference_result = vggt_client.inference(
            image_paths=image_names,
            output_dir=output_dir
        )
        
        if not inference_result.get('success', False):
            if logger: logger.error(f"VGGT推理失败: {inference_result}")
            sys.exit(1)
        
        
        # 从服务结果中提取数据
        results = inference_result['results']
        vggt_depth_map_ext = np.array(results['depth_map'])
        depth_confidence_map = np.array(results['depth_confidence_map'])
        depth_mask = np.array(results['depth_mask'])
        K = np.array(results['intrinsic_matrix'])
        vggt_intrinsic = np.array(results['vggt_intrinsic'])
        
        if logger: logger.info("VGGT服务推理完成")
    else:
        # image_names = [rgb_ext_left_pth, rgb_ext_right_pth, rgb_ext2_pth]
        image_array_list = [extern_camera_class.left_image, extern_camera_class.right_image, rgb_ext2]
        images = load_and_preprocess_images_from_array(image_array_list).to(device)

        with torch.no_grad():
            with torch.cuda.amp.autocast(dtype=dtype):
                vggt_inference_start = time.time()
                # Predict attributes including cameras, depth maps, and point maps.
                predictions = vggt_model(images)
                vggt_inference_time = time.time() - vggt_inference_start
                if logger: logger.info(f"VGGT model inference time: {vggt_inference_time:.3f} seconds")
        
        extrinsic, intrinsic = pose_encoding_to_extri_intri(predictions["pose_enc"], images.shape[-2:])
        # extrinsic: (2, 3, 4), intrinsic: (2, 3, 3)
        extrinsic = extrinsic.squeeze(0).cpu().numpy()  # (2, 3, 4)
        # print('vggt intrinsic:', intrinsic.squeeze(0).cpu().numpy()[0])  # (2, 3, 3)
        vggt_intrinsic = intrinsic.squeeze(0).cpu().numpy()[0]
        vggt_baseline = np.linalg.norm(extrinsic[0, :, 3] - extrinsic[1, :, 3])  # 计算基线
        if vggt_baseline <= 0:
            raise RuntimeError("VGGT predicted a non-positive left/right baseline.")
        vggt_depth_rescale = ORBBEC_DUAL_RGB_BASELINE_METERS / vggt_baseline
        if logger:
            logger.info(
                "VGGT depth scale: predicted_baseline=%.6f, "
                "Orbbec_dual_rgb_baseline=%.3fm, scale=%.6f",
                vggt_baseline,
                ORBBEC_DUAL_RGB_BASELINE_METERS,
                vggt_depth_rescale,
            )

        depth_map = predictions["depth"].detach().cpu().numpy().squeeze(0)[0, ..., 0] * vggt_depth_rescale  # (H, W)
        depth_confidence_map = predictions["depth_conf"].detach().cpu().numpy().squeeze(0)[0]

        depth_confidence_map = cv2.resize(depth_confidence_map, (extern_camera_class.width, extern_camera_class.height), interpolation=cv2.INTER_NEAREST)
        vggt_depth_map_ext = cv2.resize(depth_map, (extern_camera_class.width, extern_camera_class.height), interpolation=cv2.INTER_NEAREST)
        if enable_debug:
            save_depth_npy(vggt_depth_map_ext, os.path.join(output_dir, "vggt_depth_ext.npy"))
            save_rgb_png(turbo_cmap(vggt_depth_map_ext), os.path.join(output_dir, "vggt_depth_ext.png"))

        confidence_threshold = np.percentile(depth_confidence_map.flatten(), 10)
        depth_mask = (depth_confidence_map.flatten() >= confidence_threshold) & (depth_confidence_map.flatten() > 0.1)
        depth_mask = depth_mask.reshape((extern_camera_class.height, extern_camera_class.width))
        
        _iw = float(extern_camera_class.width)
        K = np.array([
            [vggt_intrinsic[0, 0] * _iw / 518, 0.0, vggt_intrinsic[0, 2] * _iw / 518],
            [0.0, vggt_intrinsic[1, 1] * _iw / 518, vggt_intrinsic[1, 2] * _iw / 518],
            [0.0, 0.0, 1.0],
        ])
        if enable_debug:
            np.save(os.path.join(output_dir, "vggt_K.npy"), K)

    # The movable head-camera correction needs metric depth, which is only
    # available after VGGT in the dual-RGB preset. Run it before any object
    # pose is transformed through extern_camera_class.c2w.
    if camera_pose_correction_callback is not None:
        camera_pose_correction_callback(
            depth_m=vggt_depth_map_ext,
            intrinsic=K,
            valid_mask=depth_mask,
        )
        if stop_after_camera_pose_correction:
            return {}, rgb_ext2_original

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
        #     choose, pts, pts2d = RAM_IO.extract_obj_pts(inst_mask, vggt_depth_map_ext, 1.0, K, True)
        # else:
        choose, pts, pts2d = RAM_IO.extract_obj_pts(inst_mask, vggt_depth_map_ext, 1.0, K, True)

        t2_2 = time.time()
        if logger: logger.info(f">>> Processing object: {obj_name}, ID: {obj_id}, Category: {obj_category}")
        ObjectTemplate.load_template(obj_category)
        
        # _, zed_pts, _ = RAM_IO.extract_obj_pts(inst_mask, extern_camera_class.depth_map, 1.0, K)
        # vggt_displacememnt =  np.linalg.norm(np.mean(zed_pts, axis=0)) - np.linalg.norm(np.mean(pts, axis=0))
        # print('distance between vggt and zed pts:', vggt_displacememnt)
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
    # print("VGGT service time:       ", t2 - t1)
    # print("Process categories time: ", t3 - t2)

    # save the objects meta information a npy file
    # objects_meta_npy_path = os.path.join(output_dir, "ram_objects_meta.npy")
    # np.save(objects_meta_npy_path, objects_meta)
    # if logger: logger.info(f">>> Save objects meta information to {objects_meta_npy_path}.")
    # return objects_meta
    return objects_meta, rgb_ext2_original
