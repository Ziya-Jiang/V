import numpy as np
import json
from pprint import pprint
import sys
import os
import argparse
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from controllor.extern_camera import ZEDCamera, OrbbecCamera


def _pose_unit_to_mm_scale(camera_pose_unit):
    if camera_pose_unit == "mm":
        return 1.0
    if camera_pose_unit == "m":
        return 1000.0
    raise ValueError("camera_pose_unit must be 'mm' or 'm'.")


def _pose_translation_in_mm(pose_matrix, camera_pose_unit):
    """Copy a camera-frame pose and express its translation in millimeters."""
    pose_mm = np.asarray(pose_matrix, dtype=float).copy()
    if pose_mm.shape != (4, 4):
        raise ValueError(f"Expected a 4x4 pose matrix, got {pose_mm.shape}.")
    pose_mm[:3, 3] *= _pose_unit_to_mm_scale(camera_pose_unit)
    return pose_mm


def transform_pose_to_world(pose_matrix, c2w_matrix):
    """
    将相机坐标系下的位姿转换为世界坐标系下的位姿
    
    Args:
        pose_matrix: 4x4相机坐标系下的位姿矩阵
        c2w_matrix: 4x4相机到世界坐标系的变换矩阵
    
    Returns:
        4x4世界坐标系下的位姿矩阵
    """
    return c2w_matrix @ pose_matrix

def extract_position_from_matrix(matrix):
    """
    从4x4变换矩阵中提取位置信息
    """
    return {
        'x': float(matrix[0, 3]),
        'y': float(matrix[1, 3]), 
        'z': float(matrix[2, 3])
    }

# @yiyao: add rotation calculation functions
def extract_rotation_from_matrix(matrix):
    """
    从4x4变换矩阵中提取旋转矩阵
    """
    return matrix[:3, :3].tolist()

def rotation_matrix_to_euler(rotation_matrix):
    """
    将旋转矩阵转换为欧拉角 (纯numpy实现)
    
    Args:
        rotation_matrix: 3x3旋转矩阵
    
    Returns:
        tuple: (roll, pitch, yaw) 弧度制欧拉角
    """
    # 使用ZYX欧拉角约定
    sy = np.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])
    
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])  # roll
        y = np.arctan2(-rotation_matrix[2, 0], sy)                   # pitch  
        z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])  # yaw
    else:
        x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])  # roll
        y = np.arctan2(-rotation_matrix[2, 0], sy)                    # pitch
        z = 0                                                        # yaw
    
    return x, y, z

def rotation_matrix_to_quaternion(rotation_matrix):
    """
    将旋转矩阵转换为四元数 (纯numpy实现)
    
    Args:
        rotation_matrix: 3x3旋转矩阵
    
    Returns:
        tuple: (x, y, z, w) 四元数
    """
    trace = np.trace(rotation_matrix)
    
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
        qw = 0.25 * s
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
    elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
        s = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2  # s = 4 * qx
        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        s = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2  # s = 4 * qy
        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2  # s = 4 * qz
        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        qz = 0.25 * s
    
    return qx, qy, qz, qw

def calculate_rotation_from_bbox_planes(bbox_data):
    """
    根据边界框平面点计算物体的旋转
    
    Args:
        bbox_data: 包含bbox_plane_x1, bbox_plane_x2等的字典
    
    Returns:
        包含多种旋转表示的字典:
        - rotation_matrix: 3x3旋转矩阵
        - euler_angles: 欧拉角 (roll, pitch, yaw) 弧度制
        - euler_angles_deg: 欧拉角 度数制
        - quaternion: 四元数 (x, y, z, w)
        - axis_vectors: 主要轴向量
    """
    
    # 检查是否包含所需的bbox数据
    required_keys = ['bbox_plane_x1', 'bbox_plane_x2', 'bbox_plane_y1', 'bbox_plane_y2', 'bbox_plane_z1', 'bbox_plane_z2']
    if not all(key in bbox_data for key in required_keys):
        return None
    
    # 提取六个平面点
    x1 = np.array([bbox_data['bbox_plane_x1']['x'], 
                   bbox_data['bbox_plane_x1']['y'], 
                   bbox_data['bbox_plane_x1']['z']])
    x2 = np.array([bbox_data['bbox_plane_x2']['x'], 
                   bbox_data['bbox_plane_x2']['y'], 
                   bbox_data['bbox_plane_x2']['z']])
    
    y1 = np.array([bbox_data['bbox_plane_y1']['x'], 
                   bbox_data['bbox_plane_y1']['y'], 
                   bbox_data['bbox_plane_y1']['z']])
    y2 = np.array([bbox_data['bbox_plane_y2']['x'], 
                   bbox_data['bbox_plane_y2']['y'], 
                   bbox_data['bbox_plane_y2']['z']])
    
    z1 = np.array([bbox_data['bbox_plane_z1']['x'], 
                   bbox_data['bbox_plane_z1']['y'], 
                   bbox_data['bbox_plane_z1']['z']])
    z2 = np.array([bbox_data['bbox_plane_z2']['x'], 
                   bbox_data['bbox_plane_z2']['y'], 
                   bbox_data['bbox_plane_z2']['z']])
    
    # 计算三个主要轴方向
    x_axis = x2 - x1  # X轴方向
    y_axis = y2 - y1  # Y轴方向
    z_axis = z2 - z1  # Z轴方向
    
    # 归一化轴向量
    x_axis_norm = x_axis / (np.linalg.norm(x_axis) + 1e-10)
    y_axis_norm = y_axis / (np.linalg.norm(y_axis) + 1e-10)
    z_axis_norm = z_axis / (np.linalg.norm(z_axis) + 1e-10)
    
    # 构建旋转矩阵（以列向量形式）
    rotation_matrix = np.column_stack([x_axis_norm, y_axis_norm, z_axis_norm])
    
    # 使用SVD确保旋转矩阵是正交的
    try:
        U, _, Vt = np.linalg.svd(rotation_matrix)
        rotation_matrix_orthogonal = U @ Vt
        
        # 确保行列式为正（右手坐标系）
        if np.linalg.det(rotation_matrix_orthogonal) < 0:
            rotation_matrix_orthogonal[:, -1] *= -1
    except:
        # 如果SVD失败，使用原始矩阵
        rotation_matrix_orthogonal = rotation_matrix
    
    # 计算欧拉角
    euler_angles_rad = rotation_matrix_to_euler(rotation_matrix_orthogonal)
    euler_angles_deg = tuple(np.degrees(angle) for angle in euler_angles_rad)
    
    # 计算四元数
    quaternion = rotation_matrix_to_quaternion(rotation_matrix_orthogonal)
    
    return {
        'rotation_matrix': rotation_matrix_orthogonal.tolist(),
        'euler_angles': {
            'roll': euler_angles_rad[0],
            'pitch': euler_angles_rad[1], 
            'yaw': euler_angles_rad[2]
        },
        'euler_angles_deg': {
            'roll': euler_angles_deg[0],
            'pitch': euler_angles_deg[1],
            'yaw': euler_angles_deg[2]
        },
        'quaternion': {
            'x': quaternion[0],
            'y': quaternion[1],
            'z': quaternion[2],
            'w': quaternion[3]
        },
        'axis_vectors': {
            'x_axis': x_axis_norm.tolist(),
            'y_axis': y_axis_norm.tolist(),
            'z_axis': z_axis_norm.tolist()
        },
        'bbox_center': {
            'x': float(np.mean([x1[0], x2[0]])),
            'y': float(np.mean([y1[1], y2[1]])),
            'z': float(np.mean([z1[2], z2[2]]))
        }
    }

def read_object_meta(
    file_path,
    convert_to_world=True,
    c2w_matrix=None,
    camera_pose_unit="mm",
):
    """
    读取并解析RAM对象元数据文件
    
    Args:
        file_path: npy文件路径
        convert_to_world: 是否转换到世界坐标系
        c2w_matrix: 可选的 4x4 相机到世界/机器人基座变换矩阵
        camera_pose_unit: RAM 相机位姿平移单位，支持 "mm" 或 "m"。
    """
    pose_unit_scale_to_mm = _pose_unit_to_mm_scale(camera_pose_unit)

    # 加载numpy文件
    data = file_path
    
    # 获取相机到世界坐标系的变换矩阵。调用方传入时优先使用，
    # 以便规划流程与最新标定结果使用同一份外参。
    if convert_to_world:
        if c2w_matrix is None:
            try:
                # extern_camera = ZEDCamera(is_warmup=False)
                extern_camera = OrbbecCamera(is_warmup=False)
                c2w_matrix = extern_camera.c2w
            except Exception as e:
                print(f"警告: 无法获取相机变换矩阵: {e}")
                print("将使用相机坐标系下的位姿")
                convert_to_world = False
        else:
            c2w_matrix = np.asarray(c2w_matrix, dtype=float)
            if c2w_matrix.shape != (4, 4):
                raise ValueError(
                    f"c2w_matrix must have shape (4, 4), got {c2w_matrix.shape}"
                )

        if convert_to_world:
            print("获取到相机到世界坐标系变换矩阵:")
            print(c2w_matrix)
    
    objects_info = {}
    
    # 遍历所有对象
    for obj_id, obj_data in data.items():
        if obj_id.isdigit():  # 只处理数字ID的对象
            obj_info = {
                'object_id': int(obj_id),
                'center_position': None,
                'center_position_camera': None,  # 保留相机坐标系位置
                'grasp_point': None,
                'grasp_point_camera': None,  # 保留相机坐标系抓取点
                'grasp_pose_matrix': None,  # 完整的抓取位姿矩阵
                'object_size': None,
                'bounding_box': {},
                'calculated_rotation': None,  # 从bbox计算的旋转信息
                'coordinate_frame': 'world' if convert_to_world else 'camera'
            }
            
            # 提取中心点位置
            if 'centroid' in obj_data:
                centroid_matrix = _pose_translation_in_mm(
                    obj_data['centroid'],
                    camera_pose_unit,
                )
                
                # 保存相机坐标系下的位置
                obj_info['center_position_camera'] = extract_position_from_matrix(centroid_matrix)
                # @yiyao
                obj_info['center_rotation_camera'] = extract_rotation_from_matrix(centroid_matrix)
                
                if convert_to_world and c2w_matrix is not None:
                    # 转换到世界坐标系
                    world_centroid = transform_pose_to_world(centroid_matrix, c2w_matrix)
                    obj_info['center_position'] = extract_position_from_matrix(world_centroid)
                    # @yiyao: add rotation here
                    obj_info['center_rotation'] = extract_rotation_from_matrix(world_centroid)
                    normalize_scale = np.cbrt(np.linalg.det(np.asarray(obj_info['center_rotation'])))
                    obj_info['center_rotation'] = (np.asarray(obj_info['center_rotation']) / normalize_scale).tolist()  # 确保旋转矩阵正交
                else:
                    # 使用相机坐标系
                    obj_info['center_position'] = obj_info['center_position_camera']
                    # @yiyao: add rotation here
                    obj_info['center_rotation'] = extract_rotation_from_matrix(centroid_matrix)
                    normalize_scale = np.cbrt(np.linalg.det(np.asarray(obj_info['center_rotation'])))
                    obj_info['center_rotation'] = (np.asarray(obj_info['center_rotation']) / normalize_scale).tolist()  # 确保旋转矩阵正交
            
            # 提取抓取点和抓取位姿
            if 'grasp_0' in obj_data:
                grasp_matrix = _pose_translation_in_mm(
                    obj_data['grasp_0'],
                    camera_pose_unit,
                )
                
                # 保存相机坐标系下的抓取点
                obj_info['grasp_point_camera'] = extract_position_from_matrix(grasp_matrix)
                
                if convert_to_world and c2w_matrix is not None:
                    # 转换到世界坐标系
                    world_grasp = transform_pose_to_world(grasp_matrix, c2w_matrix)
                    obj_info['grasp_point'] = extract_position_from_matrix(world_grasp)
                    obj_info['grasp_pose_matrix'] = world_grasp.tolist()  # 完整的4x4矩阵
                else:
                    # 使用相机坐标系
                    obj_info['grasp_point'] = obj_info['grasp_point_camera']
                    obj_info['grasp_pose_matrix'] = grasp_matrix.tolist()
            
            # 提取物体尺寸
            if 'object_size' in obj_data:
                size_data = np.asarray(obj_data['object_size'], dtype=float)
                obj_info['object_size'] = {
                    'width': float(size_data[0] * pose_unit_scale_to_mm),
                    'height': float(size_data[1] * pose_unit_scale_to_mm), 
                    'depth': float(size_data[2] * pose_unit_scale_to_mm)
                }
            
            # 提取边界框信息（也需要坐标转换）
            bbox_keys = ['bbox_plane_x1', 'bbox_plane_x2', 'bbox_plane_y1', 
                        'bbox_plane_y2', 'bbox_plane_z1', 'bbox_plane_z2']
            for bbox_key in bbox_keys:
                if bbox_key in obj_data:
                    bbox_matrix = _pose_translation_in_mm(
                        obj_data[bbox_key],
                        camera_pose_unit,
                    )
                    
                    if convert_to_world and c2w_matrix is not None:
                        world_bbox = transform_pose_to_world(bbox_matrix, c2w_matrix)
                        obj_info['bounding_box'][bbox_key] = extract_position_from_matrix(world_bbox)
                    else:
                        obj_info['bounding_box'][bbox_key] = extract_position_from_matrix(bbox_matrix)
            
            # 计算从边界框得到的旋转信息
            if len(obj_info['bounding_box']) == 6:  # 确保所有6个平面都存在
                try:
                    rotation_info = calculate_rotation_from_bbox_planes(obj_info['bounding_box'])
                    obj_info['calculated_rotation'] = rotation_info
                except Exception as e:
                    print(f"警告: 计算物体 {obj_id} 的旋转时出错: {e}")
                    obj_info['calculated_rotation'] = None
            
            objects_info[f'object_{obj_id}'] = obj_info
    
    return objects_info

def print_object_summary(objects_info):
    """
    打印物体信息摘要
    """
    print("=" * 80)
    print("物体信息摘要")
    print("=" * 80)
    
    # 获取坐标系信息
    coord_frame = "未知"
    if objects_info:
        first_obj = next(iter(objects_info.values()))
        coord_frame = first_obj.get('coordinate_frame', '未知')
    
    print(f"坐标系: {coord_frame}")
    print("-" * 80)
    
    for obj_name, obj_info in objects_info.items():
        print(f"\n{obj_name.upper()}:")
        print(f"  ID: {obj_info['object_id']}")
        
        if obj_info['center_position']:
            pos = obj_info['center_position']
            print(f"  中心位置 ({coord_frame}): ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})")
        
        if obj_info['center_position_camera']:
            pos_cam = obj_info['center_position_camera']
            print(f"  中心位置 (camera): ({pos_cam['x']:.3f}, {pos_cam['y']:.3f}, {pos_cam['z']:.3f})")
        
        if obj_info['grasp_point']:
            grasp = obj_info['grasp_point']
            print(f"  抓取点 ({coord_frame}): ({grasp['x']:.3f}, {grasp['y']:.3f}, {grasp['z']:.3f})")
            
        if obj_info['grasp_point_camera']:
            grasp_cam = obj_info['grasp_point_camera']
            print(f"  抓取点 (camera): ({grasp_cam['x']:.3f}, {grasp_cam['y']:.3f}, {grasp_cam['z']:.3f})")
        
        if obj_info['object_size']:
            size = obj_info['object_size']
            print(f"  尺寸: 宽={size['width']:.3f}, 高={size['height']:.3f}, 深={size['depth']:.3f}")
            
        if obj_info.get('grasp_pose_matrix'):
            print(f"  抓取位姿矩阵 ({coord_frame}): 已保存")
            
        if obj_info.get('calculated_rotation'):
            rotation = obj_info['calculated_rotation']
            euler_deg = rotation['euler_angles_deg']
            print(f"  计算的旋转 (欧拉角-度): Roll={euler_deg['roll']:.2f}°, Pitch={euler_deg['pitch']:.2f}°, Yaw={euler_deg['yaw']:.2f}°")
            quat = rotation['quaternion']
            print(f"  四元数: x={quat['x']:.3f}, y={quat['y']:.3f}, z={quat['z']:.3f}, w={quat['w']:.3f}")

def save_objects_json(objects_info, output_file):
    """
    将物体信息保存为JSON文件
    """
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(objects_info, f, indent=2, ensure_ascii=False)
    print(f"\n物体信息已保存到: {output_file}")

def read_object_main(
    meta_file,
    output_dir,
    enable_debug=False,
    c2w_matrix=None,
    camera_pose_unit="mm",
):
    
    # # 构建输入文件路径
    # meta_file = os.path.join(args.input, args.meta_file)
    
    # # 确保输出目录存在
    # os.makedirs(args.output, exist_ok=True)
    
    # # 构建输出文件路径
    world_output_file = os.path.join(output_dir, "objects_info_world.json")
    # camera_output_file = os.path.join(args.output, "objects_info_camera.json")
    
    # print(f"输入文件: {meta_file}")
    # print(f"输出目录: {args.output}")
    
    # 读取物体信息并转换到世界坐标系
    # print("正在读取物体信息并转换坐标系...")
    objects_info = read_object_meta(
        meta_file,
        convert_to_world=True,
        c2w_matrix=c2w_matrix,
        camera_pose_unit=camera_pose_unit,
    )
    
    # 打印摘要信息
    # print_object_summary(objects_info)
    
    # # 保存为JSON文件
    if enable_debug:
        save_objects_json(objects_info, world_output_file)
    
    # 也保存一个相机坐标系的版本用于对比
    # print("\n正在生成相机坐标系版本用于对比...")
    # objects_info_camera = read_object_meta(meta_file, convert_to_world=False)
    # save_objects_json(objects_info_camera, camera_output_file)
    
    # 打印完整的物体信息（只显示前两个对象避免输出过长）
    # print("\n" + "=" * 80)
    # print("完整物体信息 (前两个对象):")
    # print("=" * 80)
    # limited_info = dict(list(objects_info.items())[:2])
    # pprint(limited_info)
    
    # print(f"\n✅ 成功处理了 {len(objects_info)} 个物体")
    # print(f"📁 世界坐标系信息已保存到: {world_output_file}")
    # print(f"📁 相机坐标系信息已保存到: {camera_output_file}")
    return objects_info
