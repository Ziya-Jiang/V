import os
import _pickle as cPickle
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import cv2
from ram_utils import transform_coordinates_3d
from align import estimateSimilarityTransform

def grasp_point_to_volume(p, template_scale, resolution=0.1):
    """
    template_scale: (3,), use to normalize point coordinate to nunocs space, not nocs space
    """
    origin = np.array([-0.5, -0.5, -0.5])
    vol_length = 1.0
    dims = int(np.ceil(vol_length / resolution))
    shape = (dims, dims, dims)
    # 归一化
    norm_p = p / template_scale
    norm_p = np.clip(norm_p, -0.5, 0.5)
    # 体素中心点坐标
    grid_points_1d = origin[0] + resolution * (np.arange(dims) + 0.5)
    # 找到体素索引
    idx = ((norm_p - origin) / resolution).astype(int)
    idx = np.clip(idx, 0, dims-1)
    grasp_voxel_idx = tuple(idx)
    # 构建volume
    volume = np.zeros(shape, dtype=np.uint8)
    volume[grasp_voxel_idx] = 1
    # 所有体素中心坐标
    grid_coords = np.stack(np.meshgrid(grid_points_1d, grid_points_1d, grid_points_1d, indexing='ij'), axis=-1).reshape(-1, 3)
    return volume, grid_coords, grasp_voxel_idx, norm_p

def query_grasp_volume(points, volume, template_scale, origin = np.array([-0.5, -0.5, -0.5]), resolution=0.1):
    """
    对归一化点云中每个点, 查找其在grasp volume中的体素值
    输入:
        points: (N, 3) np.ndarray, 归一化物体坐标系下的点云，范围[-0.5,0.5]
        template_scale: (3,) np.ndarray, 用于归一化点坐标到nunocs空间
        volume: (D, H, W) np.ndarray, grasp volume
        origin: (3,) np.ndarray, 体素网格原点
        resolution: float, 体素分辨率
    输出:
        grasp_values: (N, 1) np.ndarray, 每个点对应volume中的体素值
    """
    dims = volume.shape

    nunocs_pts = points * np.linalg.norm(template_scale) / template_scale  # 将点云坐标转换为nunocs空间

    idxs = ((nunocs_pts - origin) / resolution).astype(int)  # (N,3)
    # 防止越界
    idxs = np.clip(idxs, 0, np.array(dims)-1)
    grasp_values = volume[idxs[:,0], idxs[:,1], idxs[:,2]][:, None]  # (N,1)
    return grasp_values

def propagate_grasp_value_to_pixels(grasp_value_src, pixels_tgt, matching_matrix):
    """
    将源点云的 grasp value 映射到目标像素点集，并返回最大 grasp value 的像素坐标

    Args:
        grasp_value_src: (N, 1) np.ndarray, 源 grasp value
        pixels_tgt:   (M, 2) np.ndarray, 目标像素点坐标
        matching_matrix: (M, N) np.ndarray, 映射矩阵

    Returns:
        target_grasp_value: (M, 1) np.ndarray, 目标 grasp value
        max_grasp_pixel:    (2,) np.ndarray, grasp value 最大的像素坐标
        max_index:          int, grasp value 最大的像素索引
    """
    target_grasp_value = matching_matrix @ grasp_value_src  # (M, 1)
    max_index = np.argmax(target_grasp_value)
    max_grasp_pixel = pixels_tgt[max_index]
    return target_grasp_value, max_grasp_pixel, max_index

def grasp_point_coord(template_grasp_point, template_scale, ref_points, matching_matrix, query_pts):
    grasp_point_org = template_grasp_point / np.linalg.norm(template_scale)
    ref_pts_in_grasp_frame = ref_points - grasp_point_org[np.newaxis, :]
    query_pts_in_grasp_frame = matching_matrix @ ref_pts_in_grasp_frame
    _, grasp_R, grasp_T, grasp_sRT, _, _ = estimateSimilarityTransform(query_pts_in_grasp_frame, query_pts)

    return grasp_T

def find_grasp_point_with_ram2(template_grasp_point, template_scale, ref_points, matching_matrix, query_pts):
    grasp_position = grasp_point_coord(template_grasp_point, template_scale, ref_points, matching_matrix, query_pts)
    return grasp_position

def find_grasp_point_with_ram(template_grasp_point, template_scale, ref_points, matching_matrix, query_pixels):
    
    grasp_volume, _, _, _ = grasp_point_to_volume(template_grasp_point, template_scale)

    ref_grasp_values = query_grasp_volume(ref_points, grasp_volume, template_scale)

    if np.max(ref_grasp_values) < 0.3:
        print("No grasp point found in the reference points.")
        return None, None, None

    query_grasp_values, query_grasp_pixel, max_index = \
        propagate_grasp_value_to_pixels(ref_grasp_values, query_pixels, matching_matrix)

    return query_grasp_pixel, query_grasp_values[max_index], max_index

def visualize_grasp_point_on_image(rgb_img, pixel, star_size=15, color='gold'):
    """
    用matplotlib在图片上可视化一个大五角星, 返回uint8 RGB图片
    Args:
        rgb_img: (H, W, 3) np.uint8
        pixel: (2,) [x, y]
        star_size: marker大小
        color: str 或 RGB色
    Returns:
        vis_img: (H, W, 3) np.uint8, 可直接cv2.imwrite
    """
    h, w = rgb_img.shape[:2]
    fig, ax = plt.subplots(figsize=(w/80, h/80), dpi=80)
    ax.imshow(rgb_img)
    # 注意scatter是(x, y)顺序
    ax.scatter([pixel[0]], [pixel[1]], marker='*', s=star_size**2, c=color, edgecolors='black', linewidths=2, zorder=10)
    ax.axis('off')
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    fig.canvas.draw()
    vis_img = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)
    vis_img = vis_img.reshape(fig.canvas.get_width_height()[::-1] + (4,))[:, :, 1:]  # 去掉alpha通道
    plt.close(fig)
    return vis_img

def euler_to_matrix(roll, pitch, yaw):
    """欧拉角转旋转矩阵, roll/pitch/yaw为弧度"""
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    return Rz @ Ry @ Rx

def get_gripper_lines(
    center, roll, pitch, yaw, object_sRT = np.eye(4),
    finger_length=0.10, finger_width=0.06,
    approaching_depth=0.03
):

    object_scale = np.cbrt(np.linalg.det(object_sRT[:3, :3]))
    finger_length = finger_length / object_scale
    finger_width = finger_width / object_scale
    approaching_depth = approaching_depth / object_scale

    T = np.eye(4)
    T[:3,:3] = euler_to_matrix(roll, pitch, yaw)
    T[:3,3] = np.array(center)
    T = object_sRT @ T

    # 坐标系下六个点
    pts = np.array([
        [0, 0, approaching_depth - 2 * finger_length], # root1
        [0, 0, approaching_depth - finger_length], # root2
        [-finger_width/2, 0, approaching_depth - finger_length], # P1 base left
        [ finger_width/2, 0, approaching_depth - finger_length], # P2 base right
        [-finger_width/2, 0, approaching_depth], # P3 tip left
        [ finger_width/2, 0, approaching_depth], # P4 tip right
    ])  # (6,3)
    
    # 齐次坐标
    pts_homo = np.concatenate([pts, np.ones((6,1))], axis=1)  # (6,4)
    pts_world = (T @ pts_homo.T).T[:,:3]  # (6,3)

    # 四根线段
    lines = [
        (pts_world[0], pts_world[1]),  # root
        (pts_world[2], pts_world[3]),  # base
        (pts_world[2], pts_world[4]),  # left finger
        (pts_world[3], pts_world[5]),  # left finger
    ]
    return lines, T

def plot_gripper(
    ax, center, roll, pitch, yaw,
    finger_length=0.08, finger_width=0.09, approaching_depth=0.04,
    color='mediumvioletred', lw=3
):
    lines = get_gripper_lines(center, roll, pitch, yaw, finger_length, finger_width, approaching_depth)
    for p1, p2 in lines:
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color=color, linewidth=lw, solid_capstyle='round')

    # 可选：画坐标轴
    R = euler_to_matrix(roll, pitch, yaw)
    origin = np.array(center)
    axes = R * 0.04  # 4cm轴
    ax.quiver(*origin, *axes[:,0], color='r', linewidth=2)  # x
    ax.quiver(*origin, *axes[:,1], color='g', linewidth=2)  # y
    ax.quiver(*origin, *axes[:,2], color='b', linewidth=2)  # z

def create_point_cloud_from_rgbd(rgb, depth, K, depth_scale=1.0, depth_trunc=3.0):
    """
    输入:
        rgb: (H,W,3) uint8
        depth: (H,W) float32, 单位米(若为毫米请设置合适的depth_scale)
        K: (3,3) 相机内参
    返回:
        o3d.geometry.PointCloud
    """
    if depth.dtype != np.float32:
        depth = depth.astype(np.float32)

    depth = depth / depth_scale
    rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb),
        o3d.geometry.Image(depth),
        depth_scale=1.0,
        depth_trunc=depth_trunc,
        convert_rgb_to_intensity=False
    )
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        width=rgb.shape[1], height=rgb.shape[0],
        fx=K[0,0], fy=K[1,1], cx=K[0,2], cy=K[1,2]
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_img, intrinsic
    )
    return pcd

def create_gripper_lineset(lines, color=[0.9, 0.2, 0.1]):
    """
    输入:
        lines: list of (3,), (3,), 4条线段
        color: list, 线颜色 (r,g,b)
    返回:
        o3d.geometry.LineSet
    """
    # 收集所有点（去重）
    pts = []
    for a, b in lines:
        pts.append(tuple(a))
        pts.append(tuple(b))
    # 去重并保留顺序
    unique_pts = []
    pt_idx_map = {}
    for pt in pts:
        if pt not in pt_idx_map:
            pt_idx_map[pt] = len(unique_pts)
            unique_pts.append(pt)
    points = np.array(unique_pts, dtype=np.float32)
    line_indices = []
    for a, b in lines:
        i = pt_idx_map[tuple(a)]
        j = pt_idx_map[tuple(b)]
        line_indices.append([i, j])
    colors = np.tile(np.array(color).reshape(1,3), (len(lines),1))
    lineset = o3d.geometry.LineSet()
    lineset.points = o3d.utility.Vector3dVector(points)
    lineset.lines = o3d.utility.Vector2iVector(line_indices)
    lineset.colors = o3d.utility.Vector3dVector(colors)
    return lineset

def visualize_gripper_in_scene(rgb, depth, K, lines, 
                               depth_scale=1000.0, depth_trunc=3.0,
                               gripper_color=[0.1, 0.8, 0.9]):
    """
    返回Open3D几何体列表(point cloud + gripper lineset)
    """
    pcd = create_point_cloud_from_rgbd(rgb, depth, K, depth_scale, depth_trunc)
    gripper_ls = create_gripper_lineset(lines, color=gripper_color)
    return [pcd, gripper_ls]