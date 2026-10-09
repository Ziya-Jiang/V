import os
import sys
import glob
import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import json

  
def main():
    file_type = "png"
    # error_threshold = 0.015  TOO STRICT      
    error_threshold = 0.03  # MORE LENIENT
    config_path = str(Path(__file__).resolve().parent.parent / "config.yaml")
    
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from scripts.utils import load_config
    from scripts.calibration import extract_number, read_robot_poses, calculate_reprojection_error, plot_reprojection_errors, perform_hand_eye_calibration
    import datetime
    from datetime import datetime as dt
    try:
        config = load_config(config_path)
    except Exception as e:
        print(f"Error loading config: {e}")
        return

    calibrate_type = config.get('calibration_type')
    data_folder = config.get('calibration_path') 
    pattern_size = tuple(config.get('chessboard_dimension'))
    square_size = config.get('chessboard_square_edge_size_m')    

    print(f"Loaded config -> Type: {calibrate_type}, Session: {data_folder}, Pattern: {pattern_size}, Square Size: {square_size}")

    image_folder = os.path.join(data_folder, "results")
    data_path = os.path.join(data_folder, "results", "ee_pose.csv")

    image_files = glob.glob(os.path.join(image_folder, f"*.{file_type}"))
    image_files_sorted = sorted(image_files, key=extract_number)
    
    if not os.path.exists(data_path):
        print(f"Error: {data_path} not found.")
        return

    robot_poses = read_robot_poses(data_path)
    if len(image_files_sorted) != len(robot_poses):
        print(f"Warning: Images count ({len(image_files_sorted)}) != Poses count ({len(robot_poses)})")

    print(f"Found {len(image_files_sorted)} images in {image_folder}")

    objp_standard = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp_standard[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

    valid_objpoints = []
    valid_imgpoints = []
    valid_img_indices = []
    valid_robot_poses = []
    img_shape = None

    print("\n<<<<<<<  Finding chessboard corners...  >>>>>>>")
    for idx, img_path in enumerate(image_files_sorted):
        img_num = extract_number(img_path)
        img = cv2.imread(img_path)
        if img is None:
            continue
            
        if img_shape is None:
            img_shape = img.shape[:2][::-1] # (width, height)
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size)
        
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_subpix = cv2.cornerSubPix(gray, corners, (10, 10), (-1, -1), criteria)
            
            valid_imgpoints.append(corners_subpix)
            valid_objpoints.append(objp_standard)
            valid_img_indices.append(img_num)
            if idx < len(robot_poses):
                valid_robot_poses.append(robot_poses[idx])
            else:
                print(f"Warning: No robot pose for image {img_num}")
        else:
            print(f"No chessboard found in image {img_num} ({img_path})")

    if not valid_imgpoints:
        print("Error: No valid chessboard corners found in any images.")
        return

    print("\n<<<<<<<  Initial Calibration & Filtering  >>>>>>>")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        valid_objpoints, valid_imgpoints, img_shape, None, None
    )
    mean_error, errors = calculate_reprojection_error(valid_objpoints, valid_imgpoints, rvecs, tvecs, mtx, dist)
    print(f"Initial Mean Reprojection Error: {mean_error:.6f}")

    filtered_objpoints, filtered_imgpoints, filtered_indices, filtered_poses = [], [], [], []
    excluded_indices = []
    
    for i, err in enumerate(errors):
        if err <= error_threshold:
            filtered_objpoints.append(valid_objpoints[i])
            filtered_imgpoints.append(valid_imgpoints[i])
            filtered_indices.append(valid_img_indices[i])
            filtered_poses.append(valid_robot_poses[i])
        else:
            excluded_indices.append(valid_img_indices[i])

    print(f"Error threshold: {error_threshold}")
    print(f"Excluded Image IDs due to high error: {excluded_indices}")
    print(f"Remaining valid images: {len(filtered_indices)}")

    if not filtered_imgpoints:
        print("Error: All images were filtered out due to high reprojection error.")
        return

    print("\n<<<<<<<  Final Intrinsic Calibration  >>>>>>>")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        filtered_objpoints, filtered_imgpoints, img_shape, None, None
    )
    final_mean_error, final_errors = calculate_reprojection_error(
        filtered_objpoints, filtered_imgpoints, rvecs, tvecs, mtx, dist
    )
    
    print("Final Camera Matrix (mtx):\n", repr(mtx))
    print("Final Distortion (dist):\n", repr(dist))
    print(f"Final Mean Reprojection Error: {final_mean_error:.6f}")
    
    

    plot_reprojection_errors(final_errors, filtered_indices)

    print("\n<<<<<<<  Extrinsic Parameters (Hand-Eye) Calibration  >>>>>>>")

    R_end2base = [pose[:3, :3] for pose in filtered_poses]
    t_end2base = [pose[:3, 3] for pose in filtered_poses]

    R_target2cam = [cv2.Rodrigues(r)[0] for r in rvecs]
    t_target2cam = tvecs 

    print(f"Aligned Data Count: {len(R_end2base)} poses matched with images.")
    
    target_folder_name = "ee_to_camera" if calibrate_type == "eye_in_hand" else "base_to_camera"
    results_dir = os.path.join(data_folder, target_folder_name, "results")
    os.makedirs(results_dir, exist_ok=True)

    with open(os.path.join(results_dir, "intrinsics.yaml"), 'w') as f:
        yaml.dump({"camera_matrix": mtx.tolist(), "distortion": dist.tolist()}, f)

    best_R, best_t = perform_hand_eye_calibration(
        R_gripper2base=R_end2base,
        t_gripper2base=t_end2base,
        R_target2cam=R_target2cam,
        t_target2cam=t_target2cam,
        method_type=calibrate_type
    )
    
    with open(os.path.join(results_dir, "extrinsics.yaml"), 'w') as f:
        yaml.dump({"rotation": best_R.tolist(), "translation": best_t.tolist()}, f)

    report_data = {
        "left": {
            "R": best_R.tolist(),
            "T": (best_t.flatten() * 1000).tolist(), 
            "K": mtx.tolist(),
            "D": dist.tolist(),
            "w": img_shape[0], # width
            "h": img_shape[1]  # height
        }
    }
    
    with open(os.path.join(results_dir, "machine-readable results.json"), "w") as f:
        json.dump(report_data, f, indent=4)
    
    now_str = dt.now().strftime("%Y-%m-%d %H:%M:%S")
    
    config_abs_path = os.path.abspath(config_path)
    try:
        with open(config_abs_path, 'r') as cfg_f:
            config_text = cfg_f.read()
    except Exception as e:
        config_text = f"# Failed to read config file: {e}"
        print(f"Warning: cannot read config file at {config_abs_path}")
    
    reproj_mean = final_mean_error
    reproj_max = max(final_errors) if final_errors else 0.0
    
    used_sample_ids = filtered_indices 
    
    report_data = {
        "date_and_time": now_str,
        "config_yaml_content": config_text,  
        "reprojection_error": {
            "mean": reproj_mean,
            "max": reproj_max
        },
        "used_sample_ids": used_sample_ids
    }
    
    report_path = os.path.join(results_dir, "report.json")
    with open(report_path, 'w') as f:
        json.dump(report_data, f, indent=4)

    print(f"\nResults successfully saved to: {results_dir}")
    print("<<<<<<<  Calibration Finished  >>>>>>>\n")

if __name__ == "__main__":
    main()