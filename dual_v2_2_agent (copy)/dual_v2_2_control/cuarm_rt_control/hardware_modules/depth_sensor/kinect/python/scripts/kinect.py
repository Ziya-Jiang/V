import numpy as np
import pykinect_azure as pykinect
from pykinect_azure.k4a._k4atypes import K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR

from scripts.utils import apply_color_to_depth_image

class Kinect:
    #Tested on Auzure Kinect DK sensor
    #Unit in mm
    def __init__(self):
        # Initialize the library, if the library is not found, add the library path as argument
        pykinect.initialize_libraries()
        # Modify camera configuration
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED
        device_config.synchronized_images_only = True
        # device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
        
        # Start device
        self.device = pykinect.start_device(config=device_config)
        calibration = self.device.get_calibration(device_config.depth_mode, device_config.color_resolution)
        self.depth_intrinsics_matrix = calibration.get_matrix(K4A_CALIBRATION_TYPE_DEPTH)
        self.rgb_intrinsics_matrix = calibration.get_matrix(K4A_CALIBRATION_TYPE_COLOR)
        self.update()
	
    def update(self):
        self.capture = self.device.update()
        self.depth_image = None
        
    def get_depth_image(self, min_depth_mm=20, max_depth_mm=2000):
        ret, depth_image = self.capture.get_depth_image()
        if not ret :
            return None
        depth_image = np.where((depth_image > min_depth_mm) & (depth_image < max_depth_mm), depth_image, 0)
        self.depth_image = depth_image
        return depth_image
	
    def get_color_image(self):
        ret, color_image = self.capture.get_color_image()
        if not ret :
            return None
        return color_image
	
    def get_colored_depth_image1(self):
        ret, depth_image = self.capture.get_colored_depth_image()
        if not ret :
            return None
        return depth_image
    
    def get_colored_depth_image(self, min_depth_mm=20, max_depth_mm=10000):
        depth_image = self.depth_image if self.depth_image is not None else self.get_depth_image(min_depth_mm=min_depth_mm, max_depth_mm=max_depth_mm)
        if depth_image is None:
            return None
        return apply_color_to_depth_image(depth_image)
	
    def get_point_cloud(self):
        ret, points = self.capture.get_pointcloud()
        if not ret :
            return None
        return points
    
    def get_depth_intrinsics_matrix(self):
        return np.array(self.depth_intrinsics_matrix)

    def get_rgb_intrinsics_matrix(self):
        return np.array(self.rgb_intrinsics_matrix)