import re
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
def extract_number(filename: str) -> int:
    name = Path(filename).stem
    match = re.search(r"(\d+)", name)
    return int(match.group(1)) if match else -1


def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    """Convert quaternion (qw, qx, qy, qz) to 3x3 rotation matrix."""
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
    
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])
    return R

def read_robot_poses(file_path: str) -> list:
    """Read robot end-effector poses from CSV file (x,y,z,qw,qx,qy,qz format).
    Returns list of 4x4 homogeneous transformation matrices.
    """
    transform_matrices_list = []
    with open(file_path, "r") as file:
        lines = file.readlines()
        start_idx = 1 if lines and 'x' in lines[0].lower() else 0
        
        for line in lines[start_idx:]:
            parts = re.split(r'[,\s]+', line.strip())
            data = [float(x) for x in filter(None, parts)]
            
            if len(data) < 7:
                continue
            
            position = np.array(data[0:3])
            qw, qx, qy, qz = data[3], data[4], data[5], data[6]
            
            R = quaternion_to_rotation_matrix(qw, qx, qy, qz)
            
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = position
            
            transform_matrices_list.append(T)
            
    return transform_matrices_list


def calculate_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist):
    total_error = 0
    errors = []

    for i in range(len(objpoints)):
        imgpoints_projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        imgpoints_projected = imgpoints_projected.reshape(-1, 1, 2)
        error = cv2.norm(imgpoints[i], imgpoints_projected, cv2.NORM_L2) / len(imgpoints_projected)
        errors.append(error)
        total_error += error

    mean_error = total_error / len(objpoints) if objpoints else 0
    return mean_error, errors

def plot_reprojection_errors(errors, img_indices):
    plt.figure()
    plt.bar(range(len(errors)), errors, tick_label=img_indices)
    plt.xlabel("Valid Image ID")
    plt.ylabel("Reprojection Error")
    plt.title(f"Reprojection Error for Each Image")
    plt.axhline(y=np.mean(errors), color='r', linestyle='-', label=f'Mean: {np.mean(errors):.4f}')
    plt.legend()
    plt.show()


def perform_hand_eye_calibration(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method_type="eye_to_hand"):
    R_g2b = np.array(R_gripper2base)
    t_g2b = np.array(t_gripper2base)
    R_t2c = np.array(R_target2cam)
    t_t2c = np.array(t_target2cam)

    if method_type == "eye_to_hand":
        R_b2g, t_b2g = [], []
        for R, t in zip(R_g2b, t_g2b):
            R_inv = R.T
            t_inv = -R_inv @ t
            R_b2g.append(R_inv)
            t_b2g.append(t_inv)
        R_arm = np.array(R_b2g)
        t_arm = np.array(t_b2g)
    else:
        R_arm = R_g2b
        t_arm = t_g2b

    methods = {
        0: "TSAI", 1: "PARK", 2: "HORAUD", 3: "ANDREFF", 4: "DANIILIDIS"
    }

    best_R, best_t = None, None
    np.set_printoptions(precision=4, suppress=True)
    print(f"\n--- Running Hand-Eye Calibration ({method_type}) ---")
    
    for method_id, method_name in methods.items():
        try:
            R, t = cv2.calibrateHandEye(
                R_gripper2base=R_arm,
                t_gripper2base=t_arm,
                R_target2cam=R_t2c,
                t_target2cam=t_t2c,
                method=method_id
            )
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t.reshape(-1)
            print(f"Method {method_id} ({method_name}):\n{T}\n")
            
            if method_id == 4: 
                best_R, best_t = R, t
        except Exception as e:
            print(f"Method {method_id} ({method_name}) failed: {e}")

    return best_R, best_t

