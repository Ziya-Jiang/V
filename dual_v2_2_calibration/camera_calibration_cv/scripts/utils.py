import os
import cv2

def load_config(config_path):
    '''
    parse the robot config using cuarm_configuration's config_loader(see cuarm_configuration/share/python/test/test_config_loader.cpp for example).
    Return the whole config as a dictionary.
    - Reject unsupported `calibration_type`.
    - Reject non-positive chessboard values.
    - Reject invalid/missing `port` when robot state is enabled.
    - Auto-create `calibration_data_path` if it does not exist, and use the title that comes last in alphabetical order for calibration.
    '''
    import yaml
    
    # Try using config_loader if available, fallback to yaml
    try:
        from cuarm_configuration import config_loader
        config = config_loader.load(config_path)
    except ImportError:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

    # Reject unsupported calibration_type
    calib_type = config.get('calibration_type')
    if calib_type not in ['eye_in_hand', 'eye_to_hand']:
        raise ValueError(f"Unsupported calibration_type: {calib_type}")

    # Reject non-positive chessboard values
    cb_dim = config.get('chessboard_dimension', [0, 0])
    cb_size_m = config.get('chessboard_square_edge_size_m', 0)
    if not (isinstance(cb_dim, list) and len(cb_dim) == 2 and cb_dim[0] > 0 and cb_dim[1] > 0):
        raise ValueError("Chessboard dimensions must be positive.")
    if cb_size_m <= 0:
        raise ValueError("Chessboard square edge size must be positive.")

    # Reject invalid/missing port when robot state is enabled
    recv_state = config.get('receive_robot_state', {})
    if recv_state.get('enabled', False):
        local_port = recv_state.get('local_port')
        remote_port = recv_state.get('remote_port')
        if not local_port or not remote_port or local_port <= 0 or remote_port <= 0:
            raise ValueError("Invalid or missing 'port' in receive_robot_state.")

    # Auto-create calibration_path and find the latest alphabetically
    calib_path = config.get('calibration_path', './calibration')
    base_dir = os.path.dirname(calib_path) if os.path.basename(calib_path) else calib_path
    
    if not os.path.exists(calib_path):
        os.makedirs(calib_path)
        
    if os.path.exists(base_dir):
        subdirs = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
        if subdirs:
            latest_session = sorted(subdirs)[-1]
            config['calibration_path'] = os.path.join(base_dir, latest_session)

    return config

def show_image(image, interrupt_keys=None, wait_time_ms=0):
    if not hasattr(show_image, "_prompt_printed"):
        print("[INFO] Press 'q' within the display window to quit the camera preview.")
        show_image._prompt_printed = True

    cv2.imshow("Camera Preview", image)
    key = cv2.waitKey(wait_time_ms) & 0xFF
    if key != 255: # 255 represents no key pressed
        char_key = chr(key)
        if char_key == 'q':
            cv2.destroyWindow("Camera Preview")
            return char_key
        if interrupt_keys is None or char_key in interrupt_keys:
            return char_key
    return None

import numpy as np

def apply_color_to_depth_image(depth_image):
    """
    Applies a color map to a depth image.
    """
    if depth_image is None or depth_image.size == 0:
        return None
    depth_image_8bit = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    return cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_JET)

def depth_to_point_cloud(depth_image, intrinsics=None):
    """
    Converts a depth image to a 3D point cloud.
    """
    if depth_image is None or depth_image.size == 0:
        return None
    h, w = depth_image.shape
    x, y = np.meshgrid(np.arange(w), np.arange(h))
    
    if intrinsics is not None and hasattr(intrinsics, 'fx'):
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy
    elif isinstance(intrinsics, np.ndarray) and intrinsics.shape == (3, 3):
        fx = intrinsics[0, 0]
        fy = intrinsics[1, 1]
        cx = intrinsics[0, 2]
        cy = intrinsics[1, 2]
    else:
        # Default or fallback values
        fx = fy = 500
        cx = w / 2
        cy = h / 2

    z = depth_image / 1000.0  # Convert to meters
    x_3d = (x - cx) * z / fx
    y_3d = (y - cy) * z / fy

    return np.dstack((x_3d, y_3d, z))
