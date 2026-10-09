import cv2
import numpy as np
import json
import matplotlib.pyplot as plt
from judge_fork_direction import get_fork_direction_vector

def load_camera_params(camera_params_path):
    """加载相机参数"""
    with open(camera_params_path, 'r') as f:
        camera_params = json.load(f)
    
    # 提取相机内参矩阵K和旋转平移矩阵
    K = np.array(camera_params['left']['K'])
    R = np.array(camera_params['left']['R'])  # 相机坐标系到世界坐标系的旋转矩阵
    T = np.array(camera_params['left']['T']).reshape(3, 1)  # 平移向量
    
    return K, R, T

def project_2d_direction_to_3d(direction_2d, depth, K, R, T):
    """
    将2D方向向量投影到3D空间
    
    Args:
        direction_2d: 2D方向向量 [dx, dy]
        depth: 对应的深度值
        K: 相机内参矩阵
        R: 旋转矩阵 (相机到世界)
        T: 平移向量
    
    Returns:
        direction_3d_world: 在世界坐标系下的3D方向向量
    """
    # 在图像平面上创建两个点：起始点和终点。
    # 使用标定主点而不是写死的 1280x720 图像中心，以兼容 Orbbec
    # 的 848x530 等不同分辨率。
    center_2d = np.array([K[0, 2], K[1, 2]], dtype=float)
    end_2d = center_2d + direction_2d * 100  # 放大方向向量以便计算
    
    # 将2D点投影到3D相机坐标系
    # 使用相机内参矩阵的逆矩阵
    K_inv = np.linalg.inv(K)
    
    # 齐次坐标
    center_2d_homo = np.array([center_2d[0], center_2d[1], 1.0])
    end_2d_homo = np.array([end_2d[0], end_2d[1], 1.0])
    
    # 投影到相机坐标系（深度为depth）
    center_3d_cam = K_inv.dot(center_2d_homo) * depth
    end_3d_cam = K_inv.dot(end_2d_homo) * depth
    
    # 计算在相机坐标系下的方向向量
    direction_3d_cam = end_3d_cam - center_3d_cam
    direction_3d_cam = direction_3d_cam / np.linalg.norm(direction_3d_cam)
    
    # 保持原有叉子方向约定：使用 R 的转置将相机方向转换到世界方向。
    direction_3d_world = R.T.dot(direction_3d_cam)
    
    return direction_3d_world

def get_current_bbox_x_direction(bbox_info):
    """
    获取当前bbox的x轴方向
    
    Args:
        bbox_info: bbox信息字典
        
    Returns:
        x_direction: 当前bbox的x轴方向向量（从x1指向x2）
    """
    x1_point = np.array([
        bbox_info['bbox_plane_x1']['x'],
        bbox_info['bbox_plane_x1']['y'], 
        bbox_info['bbox_plane_x1']['z']
    ])
    
    x2_point = np.array([
        bbox_info['bbox_plane_x2']['x'],
        bbox_info['bbox_plane_x2']['y'],
        bbox_info['bbox_plane_x2']['z']
    ])
    
    x_direction = x2_point - x1_point
    x_direction = x_direction / np.linalg.norm(x_direction)
    
    return x_direction, x1_point, x2_point

def adjust_bbox_direction(objects_info, object_id, fork_direction_3d):
    """
    调整bbox的x轴方向使其指向叉头
    
    Args:
        objects_info: 对象信息字典
        object_id: 叉子的对象ID
        fork_direction_3d: 叉头在3D世界坐标系下的方向向量
        
    Returns:
        updated_objects_info: 更新后的对象信息
    """
    obj_info = objects_info[f'object_{object_id}']['object_info']
    
    # 获取当前bbox的x轴方向
    current_x_dir, x1_point, x2_point = get_current_bbox_x_direction(obj_info['bounding_box'])
    
    # 计算当前x轴方向与叉头方向的相似度
    similarity = np.dot(current_x_dir, fork_direction_3d)
    
    print(f"当前x轴方向: {current_x_dir}")
    print(f"叉头方向: {fork_direction_3d}")
    print(f"方向相似度: {similarity:.3f}")
    
    # 如果相似度为负，说明当前x轴指向叉尾，需要翻转
    if similarity >= 0:
        print("需要翻转bbox的x轴方向")
        
        # 翻转x1和x2
        obj_info['bounding_box']['bbox_plane_x1'], obj_info['bounding_box']['bbox_plane_x2'] = \
            obj_info['bounding_box']['bbox_plane_x2'], obj_info['bounding_box']['bbox_plane_x1']
        
        obj_info['bounding_box']['bbox_plane_y1'], obj_info['bounding_box']['bbox_plane_y2'] = \
            obj_info['bounding_box']['bbox_plane_y2'], obj_info['bounding_box']['bbox_plane_y1']
        
        # 同时需要翻转grasp_pose_matrix中的x轴方向
        grasp_matrix = np.array(obj_info['grasp_pose_matrix'])
        grasp_matrix[0:3, 0] *= -1  # 翻转旋转矩阵的第一列（x轴方向）
        obj_info['grasp_pose_matrix'] = grasp_matrix.tolist()
        
        print("bbox方向已调整为指向叉头")
    else:
        print("当前bbox已指向叉头，无需调整")
    
    return objects_info

def visualize_adjustment_result(mask_path, objects_info, object_id, K, R, T):
    """
    可视化调整结果
    """
    # 获取2D方向向量
    direction_2d = get_fork_direction_vector(mask_path)
    
    # 读取mask并转成彩色图用于显示
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # 获取调整后的bbox信息
    obj_info = objects_info[f'object_{object_id}']
    current_x_dir, x1_point, x2_point = get_current_bbox_x_direction(obj_info['bounding_box'])
    
    # 将3D bbox的x轴方向投影到2D图像上进行对比
    center_3d = np.array([
        obj_info['center_position_camera']['x'],
        obj_info['center_position_camera']['y'],
        obj_info['center_position_camera']['z']
    ])
    
    # 将世界坐标下的x轴方向转换到相机坐标系
    x_dir_cam = R.dot(current_x_dir)
    
    # 投影到图像平面
    center_3d_homo = np.array([center_3d[0], center_3d[1], center_3d[2]])
    end_3d_homo = center_3d_homo + x_dir_cam * 50  # 延长50个单位用于可视化
    
    center_2d_proj = K.dot(center_3d_homo / center_3d_homo[2])
    end_2d_proj = K.dot(end_3d_homo / end_3d_homo[2])
    
    # 绘制结果
    moments = cv2.moments(mask)
    if moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        
        x, y, w, h = cv2.boundingRect(mask)
        rect_center_x = x + w // 2
        rect_center_y = y + h // 2
        
        # 绘制2D检测的方向箭头（绿色）
        arrow_length = min(w, h) * 0.4
        end_x = int(rect_center_x + direction_2d[0] * arrow_length)
        end_y = int(rect_center_y + direction_2d[1] * arrow_length)
        
        cv2.arrowedLine(display_img, 
                       (rect_center_x, rect_center_y), 
                       (end_x, end_y), 
                       (0, 255, 0), 3)  # 绿色：2D检测方向
        
        # 绘制3D bbox投影的方向箭头（蓝色）
        if center_2d_proj[0] > 0 and center_2d_proj[1] > 0 and \
           end_2d_proj[0] > 0 and end_2d_proj[1] > 0:
            cv2.arrowedLine(display_img,
                           (int(center_2d_proj[0]), int(center_2d_proj[1])),
                           (int(end_2d_proj[0]), int(end_2d_proj[1])),
                           (255, 0, 0), 3)  # 蓝色：3D bbox方向
        
        cv2.circle(display_img, (cx, cy), 5, (0, 0, 255), -1)  # 红色质心
        cv2.circle(display_img, (rect_center_x, rect_center_y), 5, (255, 255, 0), -1)  # 黄色矩形中心
    
    plt.figure(figsize=(12, 8))
    plt.imshow(cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB))
    plt.title("叉子方向调整结果\n绿色箭头：2D检测方向 | 蓝色箭头：调整后3D bbox方向")
    plt.axis('off')
    plt.savefig(mask_path.replace(".png", "_adjustment_result.png"))
    plt.show()

def main():
    """主函数"""
    # 文件路径
    mask_path = "/mnt/ssd/user/ckli/RobotRAM/output/scene_37_forkdiagonalcam/grounding_dir/grounding_mask_brown_fork_04.png"
    objects_info_path = "/mnt/ssd/user/ckli/RobotRAM/output/scene_37_forkdiagonalcam/objects_info_world.json"
    camera_params_path = "/mnt/ssd/user/ckli/RobotRAM/controllor/assets/calibration/external_camera/zed_paras.json"
    
    # 叉子的对象ID
    fork_object_id = 4
    
    # 1. 获取叉子在2D图像中的方向向量
    direction_2d = get_fork_direction_vector(mask_path)
    if direction_2d is None:
        print("无法从mask中提取方向信息")
        return
    
    print(f"2D方向向量: {direction_2d}")
    
    # 2. 加载相机参数和对象信息
    K, R, T = load_camera_params(camera_params_path)
    
    with open(objects_info_path, 'r') as f:
        objects_info = json.load(f)
    
    # 3. 获取叉子的深度信息（使用center_position_camera的z值）
    fork_depth = objects_info[f'object_{fork_object_id}']['center_position_camera']['z']
    
    # 4. 将2D方向向量投影到3D空间
    fork_direction_3d = project_2d_direction_to_3d(direction_2d, fork_depth, K, R, T)
    print(f"3D方向向量（世界坐标系）: {fork_direction_3d}")
    
    # 5. 调整bbox方向
    updated_objects_info = adjust_bbox_direction(objects_info, fork_object_id, fork_direction_3d)
    
    # 6. 保存更新后的对象信息
    output_path = objects_info_path.replace(".json", "_adjusted.json")
    with open(output_path, 'w') as f:
        json.dump(updated_objects_info, f, indent=2)
    print(f"更新后的对象信息已保存到: {output_path}")
    
    # 7. 可视化结果
    visualize_adjustment_result(mask_path, updated_objects_info, fork_object_id, K, R, T)

if __name__ == "__main__":
    main()
