import numpy as np
import torch
import math
from scipy.spatial.transform import Rotation as R


def pos_to_mat(pose):
    """
    Convert a pose (6D) to a 4x4 transformation matrix.
    The pose is assumed to be in the format [tx, ty, tz, rx, ry, rz].
    :return: 4x4 transformation matrix
    """
    if not isinstance(pose, np.ndarray):
        pose = np.array(pose)
    assert pose.shape == (6,), "Pose must be of shape (6,)"

    # Extract translation and rotation
    translation = pose[:3]
    rotation = pose[3:]

    # get rotation matrix
    rotation_matrix = angle2rotation(rotation[0], rotation[1], rotation[2])

    # Create transformation matrix
    mat = np.eye(4)
    mat[:3, :3] = rotation_matrix
    mat[:3, 3] = translation

    return mat

def mat_to_pos(mat):
    """
    Convert a 4x4 transformation matrix to a pose (6D).
    :param mat: 4x4 transformation matrix
    :return: Pose in the format [tx, ty, tz, rx, ry, rz]
    """
    assert mat.shape == (4, 4), "Matrix must be of shape (4, 4)"

    # Extract translation and rotation
    translation = mat[:3, 3]
    rotation_matrix = mat[:3, :3]

    # get rotation angles
    rotation = rotation2angle(rotation_matrix)

    return np.concatenate((translation, rotation))

def angle2rotation(rx, ry, rz):
    x = rx / 180.0 * math.pi
    y = ry / 180.0 * math.pi
    z = rz / 180.0 * math.pi

    Rx = np.array([[1, 0, 0],
                   [0, math.cos(x), -math.sin(x)],
                   [0, math.sin(x), math.cos(x)]])
    Ry = np.array([[math.cos(y), 0, math.sin(y)],
                   [0, 1, 0],
                   [-math.sin(y), 0, math.cos(y)]])
    Rz = np.array([[math.cos(z), -math.sin(z), 0],
                   [math.sin(z), math.cos(z), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return R

def rotation2angle(R):
    """
    Convert a rotation matrix to Euler angles (roll, pitch, yaw).
    :param R: 3x3 rotation matrix
    :return: Euler angles (rx, ry, rz) in degrees
    """
    assert R.shape == (3, 3), "Rotation matrix must be of shape (3, 3)"

    # Calculate Euler angles
    sy = math.sqrt(R[0, 0] * R[0, 0] +  R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x * 180. / np.pi,
                     y * 180. / np.pi,
                     z * 180. / np.pi])

def quaternion_to_matrix(q):
    """
    将四元数 [w, x, y, z] 转换为旋转矩阵
    
    参数:
    q -- 四元数 [w, x, y, z]
    
    返回:
    R -- 3x3旋转矩阵
    """
    # 确保四元数是单位四元数
    q = np.array(q)
    q = q / np.linalg.norm(q)
    
    w, x, y, z = q
    
    # 计算旋转矩阵的元素
    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,   2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,       1 - 2*x*x - 2*y*y]
    ])
    
    return R
# input = [733.0838623046875, -90.1358871459961, 123.56146240234375, 179.99282836914062, -0.0005515184602700174, -90.00130462646484]
# mat = pos_to_mat(input)
# print(mat)
# input2 = mat_to_pos(mat)
# print(input2)
# print(input2-input)