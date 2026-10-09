import os
import _pickle as cPickle
import matplotlib.pyplot as plt
import numpy as np
import cv2
from ram_utils import transform_coordinates_3d
from align import estimateSimilarityTransform

# def transform_coordinates_3d(coordinates, sRT):
#     """
#     Args:
#         coordinates: [3, N]
#         sRT: [4, 4]

#     Returns:
#         new_coordinates: [3, N]

#     """
#     assert coordinates.shape[0] == 3
#     coordinates = np.vstack([coordinates, np.ones((1, coordinates.shape[1]), dtype=np.float32)])
#     new_coordinates = sRT @ coordinates
#     new_coordinates = new_coordinates[:3, :] / new_coordinates[3, :]
#     return new_coordinates

def points_on_plane(points, template_scale, p, n, epsilon=0.01):
    """
    计算点云中与平面(由点p和法向量n定义)相交的点集
    即距离平面小于epsilon的点

    Args:
        points: (N,3) np.ndarray, 点云
        template_scale: (3,)
        p: (3,) np.ndarray, 平面上一点
        n: (3,) np.ndarray, 平面法向量（无需单位化）
        epsilon: float, 距离阈值

    Returns:
        on_plane_points: (M,3) np.ndarray, 相交点集
        mask: (N,) bool, 相交点的布尔掩码
        distances: (N,) float, 每个点到平面的有符号距离
    """
    n = n / np.linalg.norm(n)           # 法向量归一化
    vecs = points - p                   # (N,3)
    distances = np.dot(vecs, n)         # (N,)
    mask = (np.abs(distances) / np.linalg.norm(template_scale)) < epsilon  # 距离平面小于epsilon即认为相交
    on_plane_points = points[mask]

    # print('point on plane: ', on_plane_points.shape)
    return on_plane_points, mask, distances

def plane_point_to_volume(plane_points, template_scale, resolution=0.1):
    """
    将平面点生成volume
    Args:
        plane_points: (K, 3) np.ndarray, 平面上的点
        template_scale: float, 归一化尺度
        resolution: float, 体素分辨率
    Returns:
        volume: (dims, dims, dims) np.uint8
    """
    origin = np.array([-0.5, -0.5, -0.5])
    vol_length = 1.0
    dims = int(np.ceil(vol_length / resolution))
    volume = np.zeros((dims, dims, dims), dtype=np.uint8)
    if len(plane_points) == 0:
        return volume
    # 归一化
    norm_pts = plane_points / template_scale
    norm_pts = np.clip(norm_pts, -0.5, 0.5)
    idxs = ((norm_pts - origin) / resolution).astype(int)
    idxs = np.clip(idxs, 0, dims-1)
    # 填充volume
    for ix, iy, iz in idxs:
        volume[ix, iy, iz] = 1
    return volume

def query_plane_volume_from_points(points, plane_volume, template_scale, resolution=0.1):
    """
    对每个已归一化点（[-0.5, 0.5]), 查找其在plane volume中的体素值
    Args:
        points: (N, 3) np.ndarray, 已归一化点云（范围[-0.5, 0.5])
        plane_volume: (dims, dims, dims) np.ndarray, plane volume
        resolution: float, 体素分辨率
    Returns:
        plane_values: (N, 1) np.ndarray, 每个点对应的plane volume值
    """
    origin = np.array([-0.5, -0.5, -0.5])
    dims = plane_volume.shape[0]

    nunocs_pts = points * np.linalg.norm(template_scale) / template_scale  # 将点云坐标转换为nunocs空间

    idxs = ((nunocs_pts - origin) / resolution).astype(int)
    idxs = np.clip(idxs, 0, dims-1)
    plane_values = plane_volume[idxs[:, 0], idxs[:, 1], idxs[:, 2]][:, None]
    return plane_values

def propagate_plane_value(plane_value, matching_matrix, query_points, threshold=0.8):
    """
    将N*1的plane value通过matching_matrix映射到M*1, 并筛选高于阈值的点
    Args:
        plane_value: (N, 1) np.ndarray, 源点云的plane value
        matching_matrix: (M, N) np.ndarray, 映射矩阵
        query_points: (M, 3) np.ndarray, 查询点云
        threshold: float, 阈值
    Returns:
        mapped_plane_value: (M, 1) np.ndarray, 映射后的plane value
        idx: (K,) np.ndarray, plane value > threshold的点的索引
        selected_points: (K, 3) np.ndarray, plane value > threshold的点的坐标
    """
    # (M, 1) = (M, N) @ (N, 1)
    mapped_plane_value = matching_matrix @ plane_value  # (M, 1)
    idx = np.where(mapped_plane_value[:, 0] > threshold)[0]
    selected_points = query_points[idx]
    return mapped_plane_value, idx, selected_points

def fit_plane_with_fixed_normal(selected_points, normal):
    """
    给定点云和法向量, 拟合平面(法向量为normal), 使所有点到平面距离和最小。
    Args:
        selected_points: (N, 3) np.ndarray
        normal: (3,) np.ndarray, 已归一化
    Returns:
        plane_point: (3,) np.ndarray, 平面上的一个点
        normal: (3,) np.ndarray, 单位法向量
    """
    n = normal / np.linalg.norm(normal)
    # 投影到n方向上的系数
    alphas = selected_points @ n
    alpha_mean = np.mean(alphas)
    plane_point = alpha_mean * n  # 平面上点
    return plane_point, n

def fit_plane_and_project_origin(points):
    """
    输入:
        points: (N,3) numpy.ndarray, 归一化物体坐标系下的点云, 近似共面
    输出:
        proj_point: (3,) 原点在平面上的投影点坐标
        normal: (3,) 拟合平面的单位法向量
    """
    # 1. 拟合平面法向量
    centroid = points.mean(axis=0)  # 任取平面上一点
    uu, dd, vv = np.linalg.svd(points - centroid)
    normal = vv[2, :]
    normal = normal / np.linalg.norm(normal)
    
    # 2. 原点到平面距离: d = n·c
    d = normal.dot(-centroid)
    # 3. 原点到平面投影点
    proj_point = -d * normal  # 投影点向量公式
    
    return proj_point, normal

def find_support_plane_coord(template_support_point, template_scale, ref_points, matching_matrix, query_points):
    support_point_org = template_support_point / np.linalg.norm(template_scale)
    ref_pts_in_support_frame = ref_points - support_point_org[np.newaxis, :]
    query_pts_in_support_frame = matching_matrix @ ref_pts_in_support_frame
    _, support_R, support_T, support_sRT, _, _ = estimateSimilarityTransform(query_pts_in_support_frame, query_points)

    return support_T

def find_support_plane_with_ram2(template_support_point, template_support_normal, \
    template_model_pts, template_scale, ref_points, matching_matrix, query_points, sRT, align_with_axis = None):
    
    plane_points, _, _ = points_on_plane(template_model_pts, template_scale, template_support_point, template_support_normal)
    support_volume = plane_point_to_volume(plane_points, template_scale)
    
    ref_support_values = query_plane_volume_from_points(ref_points, support_volume, template_scale)

    _, _, selected_support_points = propagate_plane_value(ref_support_values, matching_matrix, query_points)

    if selected_support_points.shape[0] < 3:
        print('Not enough support points found, returning default plane.')
        return template_support_point / (np.linalg.norm(template_scale)), template_support_normal

    selected_support_points_in_object_frame = transform_coordinates_3d(selected_support_points.T, np.linalg.inv(sRT)).T

    _, support_normal = fit_plane_and_project_origin(selected_support_points_in_object_frame)

    if np.dot(support_normal, template_support_normal) < 0:
        support_normal = -support_normal

    if align_with_axis is not None:
        if align_with_axis == 'x':
            support_normal = np.array([1, 0, 0])
        elif align_with_axis == 'y':
            support_normal = np.array([0, 1, 0])
        elif align_with_axis == 'z':
            support_normal = np.array([0, 0, 1])
        
    support_point = find_support_plane_coord(template_support_point, template_scale, ref_points, matching_matrix, query_points)
    support_point = transform_coordinates_3d(support_point[:, np.newaxis], np.linalg.inv(sRT)).flatten()
            
    return support_point, support_normal

def find_support_plane_with_ram(template_support_point, template_support_normal, \
    template_model_pts, template_scale, ref_points, matching_matrix, query_points, sRT, align_with_axis = None):
    
    plane_points, _, _ = points_on_plane(template_model_pts, template_scale, template_support_point, template_support_normal)
    support_volume = plane_point_to_volume(plane_points, template_scale)
    
    ref_support_values = query_plane_volume_from_points(ref_points, support_volume, template_scale)

    _, _, selected_support_points = propagate_plane_value(ref_support_values, matching_matrix, query_points)

    if selected_support_points.shape[0] < 3:
        print('Not enough support points found, returning default plane.')
        return template_support_point / (np.linalg.norm(template_scale)), template_support_normal

    selected_support_points_in_object_frame = transform_coordinates_3d(selected_support_points.T, np.linalg.inv(sRT)).T

    support_point, support_normal = fit_plane_and_project_origin(selected_support_points_in_object_frame)

    if np.dot(support_normal, template_support_normal) < 0:
        support_normal = -support_normal

    if align_with_axis is not None:
        if align_with_axis == 'x':
            support_normal = np.array([1, 0, 0])
        elif align_with_axis == 'y':
            support_normal = np.array([0, 1, 0])
        elif align_with_axis == 'z':
            support_normal = np.array([0, 0, 1])
            
    return support_point, support_normal

def get_plane_square_points(plane_point, plane_normal, plane_size=1.2):
    """
    在物体坐标系下, 以plane_point为中心、plane_normal为法向量, 生成正方形平面(角点逆时针排列)
    """
    n = plane_normal / np.linalg.norm(plane_normal)
    # 找到与法向量不平行的向量
    v = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])
    basis1 = np.cross(n, v)
    basis1 /= np.linalg.norm(basis1)
    basis2 = np.cross(n, basis1)
    basis2 /= np.linalg.norm(basis2)
    half = plane_size / 2

    # 按逆时针顺序排列：左上，右上，右下，左下
    corners = [
        plane_point - half * basis1 + half * basis2,
        plane_point + half * basis1 + half * basis2,
        plane_point + half * basis1 - half * basis2,
        plane_point - half * basis1 - half * basis2,
    ]
    return np.stack(corners, axis=0)  # (4,3)

def clip_points_to_cube(points, cube_min=-0.5, cube_max=0.5):
    """
    将点裁剪到cube范围
    """
    return np.clip(points, cube_min, cube_max)

def apply_pose(points, pose):
    """
    points: (N,3), pose: (4,4)
    """
    N = points.shape[0]
    homo = np.concatenate([points, np.ones((N,1))], axis=1)  # (N,4)
    cam = (pose @ homo.T).T
    return cam[:, :3]

def project_points(points_cam, K):
    """
    points_cam: (N,3) 相机坐标系
    K: (3,3) 相机内参
    返回: (N,2) 图像像素坐标
    """
    x = points_cam[:,0]
    y = points_cam[:,1]
    z = points_cam[:,2]
    u = K[0,0]*x/z + K[0,2]
    v = K[1,1]*y/z + K[1,2]
    return np.stack([u, v], axis=1)

def draw_plane_on_image_obj_coords(
    rgb_img, plane_point, plane_normal, pose, K,
    color=(255,128,0), alpha=0.45, plane_size=1.0, cube_min=-0.5, cube_max=0.5
):
    """
    在rgb图像上可视化物体坐标系下的平面(半透明)
    plane_point, plane_normal: 归一化物体坐标系
    pose: 4x4, 从物体系到相机系
    K: 3x3, 相机内参
    color: BGR
    alpha: 透明度
    plane_size: 平面正方形边长
    """
    # 1. 生成物体坐标系下的平面四角
    poly_obj = get_plane_square_points(plane_point, plane_normal, plane_size)
    # 2. 裁剪到cube范围
    poly_obj = clip_points_to_cube(poly_obj, cube_min, cube_max)
    # 3. 变换到相机坐标系
    poly_cam = apply_pose(poly_obj, pose)
    # 4. 投影到图像
    poly_pix = project_points(poly_cam, K)
    poly_pix = np.round(poly_pix).astype(np.int32)
    # 5. 绘制半透明多边形
    vis_img = rgb_img.copy()
    overlay = vis_img.copy()
    cv2.fillPoly(overlay, [poly_pix], color)
    vis_img = cv2.addWeighted(overlay, alpha, vis_img, 1-alpha, 0)
    return vis_img

def draw_plane_grid_on_image(
    img,
    plane_center_obj,
    plane_normal_obj,
    obj_pose_cam,
    K,
    grid_size=4,
    grid_extent=0.5
):
    """
    在RGB图像上可视化一个物体坐标系下平面的4x4空间网格
    - img: (H,W,3) uint8
    - plane_center_obj: (3,) 平面中心（物体坐标系）
    - plane_normal_obj: (3,) 平面法向（物体坐标系，单位向量）
    - obj_pose_cam: (4,4) 物体到相机的变换矩阵
    - K: (3,3) 相机内参
    - grid_size: int, 网格行列数
    - grid_extent: float, 网格覆盖范围（中心±extent）
    返回: 可视化后图像
    """
    H, W = img.shape[:2]
    overlay = img.copy()

    # 1. 物体坐标系的平面局部正交基
    n_obj = plane_normal_obj / np.linalg.norm(plane_normal_obj)
    print(f"Plane normal in object coordinates: {n_obj}")
    # 任取不平行向量
    v = np.array([1,0,0]) if abs(n_obj[0])<0.9 else np.array([0,1,0])
    u_obj = np.cross(n_obj, v)
    u_obj = u_obj / np.linalg.norm(u_obj)
    v_obj = np.cross(n_obj, u_obj)

    # 2. 网格点（物体坐标系下）
    ticks = np.linspace(-grid_extent, grid_extent, grid_size+1)
    grid_points_obj = []
    for i in range(grid_size+1):
        for j in range(grid_size+1):
            pt = plane_center_obj + u_obj*ticks[i] + v_obj*ticks[j]
            grid_points_obj.append(pt)
    grid_points_obj = np.array(grid_points_obj).reshape((grid_size+1, grid_size+1, 3))

    # 3. 变换到相机坐标系
    def to_cam(pt_obj):
        pt_obj_h = np.concatenate([pt_obj, [1]])
        pt_cam_h = obj_pose_cam @ pt_obj_h
        return pt_cam_h[:3]
    
    grid_points_cam = np.zeros_like(grid_points_obj)
    for i in range(grid_size+1):
        for j in range(grid_size+1):
            grid_points_cam[i,j] = to_cam(grid_points_obj[i,j])

    # 4. 投影到2D像素
    # def proj(pt3d):
    #     if pt3d[2] <= 1e-6:
    #         return None
    #     pt = pt3d / pt3d[2]
    #     pix = K @ pt
    #     x, y = int(round(pix[0])), int(round(pix[1]))
    #     if 0 <= x < W and 0 <= y < H:
    #         return (x, y)
    #     else:
    #         return None

    # 4. 投影到2D像素
    def proj(pt3d):
        pt3d = np.asarray(pt3d, dtype=np.float64)
        # 检查z
        if not np.isfinite(pt3d[2]) or pt3d[2] <= 1e-8:
            return None
        pt = pt3d / pt3d[2]
        if not np.all(np.isfinite(pt)):
            return None
        pix = K @ pt
        if not np.all(np.isfinite(pix)):
            return None
        x, y = int(round(pix[0])), int(round(pix[1]))
        return (x, y)

    grid_points_2d = np.full((grid_size+1, grid_size+1, 2), -1, dtype=int)
    valid_mask = np.zeros((grid_size+1, grid_size+1), dtype=bool)
    for i in range(grid_size+1):
        for j in range(grid_size+1):
            pt2d = proj(grid_points_cam[i,j])
            if pt2d is not None:
                grid_points_2d[i,j] = pt2d
                valid_mask[i,j] = True

    # 5. 配色方案
    grid_color = (180, 210, 255)      # 浅橙蓝
    grid_thickness = 2
    grid_alpha = 0.7
    vertex_color = (60, 120, 255)     # 深橙
    vertex_radius = 7
    center_color = (0, 80, 255)       # 红橙
    center_radius = 10

    # 6. 画网格线
    for i in range(grid_size+1):
        pts = [tuple(grid_points_2d[i,j]) for j in range(grid_size+1) if valid_mask[i,j]]
        if len(pts) >= 2:
            cv2.polylines(overlay, [np.array(pts)], isClosed=False, color=grid_color, thickness=grid_thickness, lineType=cv2.LINE_AA)
    for j in range(grid_size+1):
        pts = [tuple(grid_points_2d[i,j]) for i in range(grid_size+1) if valid_mask[i,j]]
        if len(pts) >= 2:
            cv2.polylines(overlay, [np.array(pts)], isClosed=False, color=grid_color, thickness=grid_thickness, lineType=cv2.LINE_AA)

    # 7. 画顶点
    for i in range(grid_size+1):
        for j in range(grid_size+1):
            if valid_mask[i,j]:
                cv2.circle(overlay, tuple(grid_points_2d[i,j]), vertex_radius, vertex_color, -1, lineType=cv2.LINE_AA)

    # 8. 画中心点
    center_idx = grid_size // 2
    if valid_mask[center_idx,center_idx]:
        cv2.circle(overlay, tuple(grid_points_2d[center_idx,center_idx]), center_radius, center_color, -1, lineType=cv2.LINE_AA)

    # 9. 融合
    out = cv2.addWeighted(overlay, grid_alpha, img, 1-grid_alpha, 0)
    return out

def draw_3d_arrow_on_image(
    img, 
    point,       # (3,) 3D起点（物体坐标系）
    normal,      # (3,) 3D方向（物体坐标系）
    pose,        # (4,4) 物体到相机
    K,           # (3,3) 相机内参
    length=0.3,  # 箭头长度
    color_main=(255,180,60),   # **亮蓝色** BGR
    color_tip=(255,64,0),      # **深蓝** BGR
    thickness=5,
    tip_length=0.2,
    shadow=True
):
    """
    在RGB图像上绘制一个美观的蓝色系3D箭头，与橙色网格区分明显
    """
    # 归一化法向量
    if np.dot(normal, np.array([0, 0, 1])) < 0:
        normal = -normal
    normal = normal / np.linalg.norm(normal)

    # 变到相机坐标系
    pt3d_start = apply_pose(point[np.newaxis, :], pose).flatten()
    pt3d_end = apply_pose((point + length * normal)[np.newaxis, :], pose).flatten()

    # # 投影到2D
    # def proj(pt):
    #     pt = np.asarray(pt)
    #     if pt[2] <= 1e-8:
    #         return None
    #     pt = pt / pt[2]
    #     pix = K @ pt
    #     return (int(round(pix[0])), int(round(pix[1])))

    # 投影到2D
    def proj(pt):
        pt = np.asarray(pt, dtype=np.float64)
        # 检查z合法性
        if not np.isfinite(pt[2]) or pt[2] <= 1e-8:
            return None
        pt = pt / pt[2]
        if not np.all(np.isfinite(pt)):
            return None
        pix = K @ pt
        if not np.all(np.isfinite(pix)):
            return None
        return (int(round(pix[0])), int(round(pix[1])))

    pt2d_start = proj(pt3d_start)
    pt2d_end = proj(pt3d_end)
    if pt2d_start is None or pt2d_end is None:
        return img

    overlay = img.copy()

    # 画阴影
    if shadow:
        shadow_offset = (5, 5)
        cv2.arrowedLine(
            overlay, 
            (pt2d_start[0]+shadow_offset[0], pt2d_start[1]+shadow_offset[1]),
            (pt2d_end[0]+shadow_offset[0], pt2d_end[1]+shadow_offset[1]),
            (40, 40, 40), thickness=thickness+4, tipLength=tip_length, line_type=cv2.LINE_AA
        )
    
    # 渐变主箭体
    num_steps = 24
    for i in range(num_steps):
        alpha = i / (num_steps - 1)
        color = tuple([
            int((1-alpha)*color_main[c] + alpha*color_tip[c])
            for c in range(3)
        ])
        p0 = (
            int((1-alpha)*pt2d_start[0] + alpha*pt2d_end[0]),
            int((1-alpha)*pt2d_start[1] + alpha*pt2d_end[1])
        )
        p1 = (
            int((1-(alpha+1/num_steps))*pt2d_start[0] + (alpha+1/num_steps)*pt2d_end[0]),
            int((1-(alpha+1/num_steps))*pt2d_start[1] + (alpha+1/num_steps)*pt2d_end[1])
        )
        cv2.line(overlay, p0, p1, color, thickness, lineType=cv2.LINE_AA)
    # 箭头尖
    cv2.arrowedLine(overlay, pt2d_start, pt2d_end, color_tip, thickness, tipLength=tip_length, line_type=cv2.LINE_AA)
    
    # 起点大圆球，外白内主色，中心小高光
    cv2.circle(overlay, pt2d_start, radius=thickness*2+2, color=(255,255,255), thickness=-1, lineType=cv2.LINE_AA)
    cv2.circle(overlay, pt2d_start, radius=thickness*2, color=color_main, thickness=-1, lineType=cv2.LINE_AA)
    cv2.circle(overlay, pt2d_start, radius=thickness, color=color_tip, thickness=-1, lineType=cv2.LINE_AA)
    # cv2.circle(overlay, (pt2d_start[0]+thickness//2, pt2d_start[1]-thickness//2), radius=thickness//2, color=(240,240,240), thickness=-1, lineType=cv2.LINE_AA)

    # # 端点小球高光
    # cv2.circle(overlay, pt2d_end, radius=thickness+2, color=color_tip, thickness=-1, lineType=cv2.LINE_AA)
    # cv2.circle(overlay, (pt2d_end[0]+thickness//2, pt2d_end[1]-thickness//2), radius=thickness//2, color=(255,255,255), thickness=-1, lineType=cv2.LINE_AA)

    # 透明融合
    alpha = 0.84
    img_out = cv2.addWeighted(overlay, alpha, img, 1-alpha, 0)
    return img_out