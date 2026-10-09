import cv2
import numpy as np
import matplotlib.pyplot as plt

def depth_to_point_cloud(depth_image, intrinsics, x_offset=0, y_offset=0):
    """
    将深度图转换为点云。

    参数：
    depth_image: 深度图像（numpy 数组），应该是一个单通道图像，单位是米或毫米。
    intrinsics: 相机内参，格式为 [fx, fy, cx, cy]。
    x_offset: x-coordinate of the top-left corner if a cropped depth image is given.
    y_offset: y-coordinate of the top-left corner if a cropped depth image is given.

    返回：
    point_cloud: 点云，格式为 (N, 3) 的 numpy 数组。
    """
    # 获取深度图的尺寸
    height, width = depth_image.shape

    # 生成网格坐标
    x = np.arange(width)
    y = np.arange(height)
    x, y = np.meshgrid(x, y)

    # 获取相机内参
    fx = intrinsics[0, 0]  # 焦距 fx
    fy = intrinsics[1, 1]  # 焦距 fy
    cx = intrinsics[0, 2]  # 主点 cx
    cy = intrinsics[1, 2]  # 主点 cy

    # 将深度值转换为点云
    z = depth_image.flatten()  # 深度值
    x = (x.flatten() - cx + x_offset) * z / fx  # 计算 X 坐标
    y = (y.flatten() - cy + y_offset) * z / fy  # 计算 Y 坐标

    # 组合为点云
    point_cloud = np.vstack((x, y, z)).T  # 转置为 (N, 3) 的形状

    return point_cloud

def transform_point_cloud(point_cloud, transformation_matrix):
    """
    使用给定的转换矩阵转换点云。

    参数：
    point_cloud: 原始点云，格式为 (N, 3) 的 numpy 数组。
    transformation_matrix: 4x4 转换矩阵，格式为 (4, 4) 的 numpy 数组。

    返回：
    transformed_point_cloud: 转换后的点云，格式为 (N, 3) 的 numpy 数组。
    """
    # 将点云转换为齐次坐标 (N, 4)
    ones = np.ones((point_cloud.shape[0], 1))
    homogeneous_point_cloud = np.hstack((point_cloud, ones))  # 变为 (N, 4)

    # 应用转换矩阵
    transformed_homogeneous = homogeneous_point_cloud @ transformation_matrix.T  # 矩阵乘法

    # 提取转换后的点云坐标 (N, 3)
    transformed_point_cloud = transformed_homogeneous[:, :3]  # 只保留前3列

    return transformed_point_cloud

def apply_color_to_depth_image(depth_image):
    colored_depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    colored_depth_image = cv2.applyColorMap(colored_depth_image, cv2.COLORMAP_JET)
    return colored_depth_image

def image_to_animation(image_filename_array, save_name="images", duration=100):
    # Use pillow to save all frames as an animation in a gif file
    from PIL import Image
    images = [Image.open(filename) for filename in image_filename_array]
    images[0].save(f'{save_name}.gif', save_all=True, append_images=images[1:], duration=duration, loop=0)

def generate_gif(fig, ax, title, pts):
    #Plot point cloud
    plt.cla()
    ax.set_title(title, y=1.0, pad=-30)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(*[0,0,0], c='red', marker="X")
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    fig.tight_layout()
    ax.scatter(pts[:,0], pts[:,1], pts[:,2], c='pink', alpha=0.5)
    n = 0
    for angle in range(0, 90, 5):
        ax.view_init(elev=0, azim=180+angle)
        plt.draw()
        plt.savefig(f"{n}.jpg")
        n += 1
    for angle in range(0, 45, 5):
        ax.view_init(elev=angle, azim=180+90)
        plt.draw()
        plt.savefig(f"{n}.jpg")
        n += 1
    images = [f"{i}.jpg" for i in range(n)]
    image_to_animation(images, title, 300)