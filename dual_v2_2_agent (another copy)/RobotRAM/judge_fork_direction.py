import cv2
import numpy as np
import matplotlib.pyplot as plt

def get_fork_direction_vector(mask_path):
    # 读取mask
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    
    # 计算质心
    moments = cv2.moments(mask)
    if moments["m00"] == 0:
        return None
    
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    
    # 计算外接矩形中心
    x, y, w, h = cv2.boundingRect(mask)
    rect_center_x = x + w // 2
    rect_center_y = y + h // 2
    
    # 计算方向向量：从矩形中心指向质心
    # 因为质心偏向叉头，所以这个向量就是叉头方向
    direction_vector = np.array([cx - rect_center_x, cy - rect_center_y])
    
    # 归一化向量
    norm = np.linalg.norm(direction_vector)
    if norm > 0:
        direction_vector = direction_vector / norm
    
    return direction_vector

def get_fork_direction_vector_from_image(mask):
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    # 计算质心
    moments = cv2.moments(mask)
    if moments["m00"] == 0:
        return None
    
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    
    # 计算外接矩形中心
    x, y, w, h = cv2.boundingRect(mask)
    rect_center_x = x + w // 2
    rect_center_y = y + h // 2
    
    # 计算方向向量：从矩形中心指向质心
    # 因为质心偏向叉头，所以这个向量就是叉头方向
    direction_vector = np.array([cx - rect_center_x, cy - rect_center_y])
    
    # 归一化向量
    norm = np.linalg.norm(direction_vector)
    if norm > 0:
        direction_vector = direction_vector / norm
    
    return direction_vector


def visualize_fork_direction(mask_path):
    # 读取mask并转成彩色图用于显示
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # 计算方向向量
    vector = get_fork_direction_vector(mask_path)
    if vector is None:
        return
    
    # 计算质心和矩形中心
    moments = cv2.moments(mask)
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    
    x, y, w, h = cv2.boundingRect(mask)
    rect_center_x = x + w // 2
    rect_center_y = y + h // 2
    
    # 绘制箭头显示方向
    arrow_length = min(w, h) * 0.4
    end_x = int(rect_center_x + vector[0] * arrow_length)
    end_y = int(rect_center_y + vector[1] * arrow_length)
    
    # 在图像上绘制
    cv2.arrowedLine(display_img, 
                   (rect_center_x, rect_center_y), 
                   (end_x, end_y), 
                   (0, 255, 0), 3)  # 绿色箭头
    
    cv2.circle(display_img, (cx, cy), 5, (0, 0, 255), -1)  # 红色质心
    cv2.circle(display_img, (rect_center_x, rect_center_y), 5, (255, 0, 0), -1)  # 蓝色矩形中心
    
    # plt.imshow(cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB))
    # plt.title(f"方向向量: [{vector[0]:.2f}, {vector[1]:.2f}]")
    # plt.axis('off')
    # plt.savefig(mask_path.replace(".png", "_direction.png"))
    # plt.show()
    
    return vector

# 使用示例
# vector = visualize_fork_direction("/mnt/ssd/user/ckli/RobotRAM/output/scene_38_forkdiagonalcam/grounding_dir/grounding_mask_brown_fork_04.png")