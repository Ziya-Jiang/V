import atexit

try:
    import pyzed.sl as sl
except ImportError:
    sl = None
import time
import math
import functools
import numpy as np
import cv2
from PIL import Image
import os
import logging
import json
import open3d as o3d
try:
    import pykinect_azure as pykinect
except ImportError:
    pykinect = None
import pyrealsense2 as rs

from .base_camera import BaseCamera


class ZEDCamera(BaseCamera):
    def __init__(self, config_file=None, is_warmup=True):
        super(ZEDCamera, self).__init__()
        cur_path = os.path.dirname(os.path.abspath(__file__))
        self.camparas_file = os.path.join(cur_path, 
            'assets/calibration/external_camera', 'zed_paras.json')
        self.max_depth = 5000  # mm
        self.fps = 30

        if config_file is not None:
            self.read_config(config_file)
            logging.info("==> Camera configuration loaded from %s", config_file)

        with open(self.camparas_file, 'r') as f:
            cam_paras_dict = json.load(f)
        left_cam_paras = cam_paras_dict["left"]

        self.R = np.array(left_cam_paras["R"])
        self.T = np.array(left_cam_paras["T"])
        self.K = np.array(left_cam_paras["K"])
        self.D = np.array(left_cam_paras["D"])
        self.w = left_cam_paras["w"]
        self.h = left_cam_paras["h"]
        self.type = "external"
        
        # zero loop camera
        self.rgb_image = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.rgb_right_image = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.depth_map = np.zeros((self.h, self.w), dtype=np.float32)

        if is_warmup:
            self.Cam = sl.Camera()
            self.CamRuntimeParameters = sl.RuntimeParameters()
            
            # 配置运行时参数以提高深度精度
            self.CamRuntimeParameters.confidence_threshold = 80  # 增加置信度阈值
            self.CamRuntimeParameters.texture_confidence_threshold = 100
            
            init_parames = sl.InitParameters()
            init_parames.set_from_camera_id(0)
            if self.w == 1280 and self.h == 720:
                init_parames.camera_resolution = sl.RESOLUTION.HD720
            elif self.w == 1920 and self.h == 1080:
                init_parames.camera_resolution = sl.RESOLUTION.HD1080
            
            # 更改深度模式为QUALITY以获得更准确的深度
            # init_parames.depth_mode = sl.DEPTH_MODE.QUALITY
            init_parames.depth_mode = sl.DEPTH_MODE.ULTRA
            init_parames.camera_fps = self.fps
            init_parames.coordinate_units = sl.UNIT.MILLIMETER  # 确保使用毫米单位
            init_parames.depth_minimum_distance = 200  # 最小深度200mm
            init_parames.depth_maximum_distance = self.max_depth  # 最大深度
            
            # 启用深度稳定化
            init_parames.depth_stabilization = 1
            
            if self.Cam.open(init_parames) != sl.ERROR_CODE.SUCCESS:
                print("Failed to open ZED camera")
                return
            
            # 调整相机设置
            self.Cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 60)  # 较低曝光
            self.Cam.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 4)
            self.Cam.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 4)
            self.Cam.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 4)

            for i in range(30):
                rgb_image, depth_map = self.capture_image()

    def restore_camparas(self, config_file):
        """
        Restore the camera parameters
        :return: None
        """
        left_cam_paras = {
            "R": self.R.tolist(),  # rotation corresponding to the robot arm GetActualTCPPose 
            "T": self.T.tolist(),  # translation corresponding to the robot arm GetActualTCPPose
            "K": self.K.tolist(),
            "D": self.D.tolist(),
            "w": self.w,
            "h": self.h,
        }
        cam_paras_dict = {
            "left": left_cam_paras,
        }
        # write cam_paras_dict to a json file
        with open(config_file, 'w') as f:
            json.dump(cam_paras_dict, f, indent=4)
        logging.info("Camera parameters saved to %s", config_file)
        
    def restore_config(self, config_file):
        """
        Restore the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        config_paras = {
            "cam_paras_file": self.camparas_file,
            "fps": self.fps,
            "max_depth": self.max_depth,
            "type": self.type,
        }
        # write config_paras to a json file
        with open(config_file, 'w') as f:
            json.dump(config_paras, f, indent=4)
        logging.info("Configuration saved to %s", config_file)

    def read_config(self, config_file):
        """
        Read the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        with open(config_file, 'r') as f:
            config_paras = json.load(f)
        self.cam_paras_file = config_paras["cam_paras_file"]
        self.max_depth = config_paras["max_depth"]
        self.fps = config_paras["fps"]
        self.type = config_paras["type"]

    def capture_image(self):
        """
        Capture an image from the camera
        :return: RGB image and depth image
        """
        raw_image = sl.Mat()
        raw_depth = sl.Mat()
        raw_dimg = sl.Mat()
        raw_pcd = sl.Mat()
        raw_right_image = sl.Mat()

        if self.Cam.grab(self.CamRuntimeParameters) == sl.ERROR_CODE.SUCCESS:
            self.Cam.retrieve_image(raw_image, sl.VIEW.LEFT)
            self.Cam.retrieve_measure(raw_depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
            self.Cam.retrieve_image(raw_dimg, sl.VIEW.DEPTH)
            self.Cam.retrieve_measure(raw_pcd, sl.MEASURE.XYZBGRA, sl.MEM.CPU)
            self.Cam.retrieve_image(raw_right_image, sl.VIEW.RIGHT)

            CamBGR = raw_image.get_data()
            CamD = raw_depth.get_data()
            CamDIMG = raw_dimg.get_data()
            CamPCD = raw_pcd.get_data()
            CamBGR_right = raw_right_image.get_data()

            CamBGR = np.array(CamBGR)[..., :3]
            CamRGB = cv2.cvtColor(CamBGR, cv2.COLOR_BGR2RGB)
            CamD = np.array(CamD)
            
            # 改进深度数据处理
            CamD[np.isnan(CamD)] = 0
            CamD[np.isinf(CamD)] = 0
            CamD[CamD > self.max_depth] = 0
            CamD[CamD < 200] = 0  # 过滤过近的点
            
            # 可选：应用中值滤波减少噪声
            CamD = cv2.medianBlur(CamD.astype(np.float32), 5)
            
            CamRGB_right = cv2.cvtColor(np.array(CamBGR_right)[..., :3], cv2.COLOR_BGR2RGB)

        else:
            CamRGB = None
            CamD = None
            CamDIMG = None
            CamPCD = None
            CamRGB_right = None
            print("External camera captures fail")

        self.rgb_image = CamRGB
        self.rgb_right_image = CamRGB_right
        self.depth_map = CamD        
        return self.rgb_image, self.depth_map
    
    def capture_two_images(self):
        """
        Capture an image from the camera
        :return: RGB image and depth image
        """
        raw_image = sl.Mat()
        raw_depth = sl.Mat()
        raw_dimg = sl.Mat()
        raw_pcd = sl.Mat()
        raw_right_image = sl.Mat()

        if self.Cam.grab(self.CamRuntimeParameters) == sl.ERROR_CODE.SUCCESS:
            self.Cam.retrieve_image(raw_image, sl.VIEW.LEFT)
            self.Cam.retrieve_measure(raw_depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
            self.Cam.retrieve_image(raw_dimg, sl.VIEW.DEPTH)
            self.Cam.retrieve_measure(raw_pcd, sl.MEASURE.XYZBGRA, sl.MEM.CPU)
            self.Cam.retrieve_image(raw_right_image, sl.VIEW.RIGHT)

            CamBGR = raw_image.get_data()
            CamD = raw_depth.get_data()
            CamDIMG = raw_dimg.get_data()
            CamPCD = raw_pcd.get_data()
            CamBGR_right = raw_right_image.get_data()

            CamBGR = np.array(CamBGR)[..., :3]
            CamRGB = cv2.cvtColor(CamBGR, cv2.COLOR_BGR2RGB)
            CamD = np.array(CamD)
            
            # 改进深度数据处理
            CamD[np.isnan(CamD)] = 0
            CamD[np.isinf(CamD)] = 0
            CamD[CamD > self.max_depth] = 0
            CamD[CamD < 200] = 0  # 过滤过近的点
            
            # 可选：应用中值滤波减少噪声
            CamD = cv2.medianBlur(CamD.astype(np.float32), 5)
            
            CamRGB_right = cv2.cvtColor(np.array(CamBGR_right)[..., :3], cv2.COLOR_BGR2RGB)

        else:
            CamRGB = None
            CamD = None
            CamDIMG = None
            CamPCD = None
            CamRGB_right = None
            print("External camera captures fail")

        self.rgb_image = CamRGB
        self.rgb_right_image = CamRGB_right
        self.depth_map = CamD        
        return self.rgb_image, self.rgb_right_image, self.depth_map

    @property
    def left_image(self):
        """
        Get the left RGB image
        :return: RGB image
        """
        return self.rgb_image
    
    @left_image.setter
    def left_image(self, image):
        """
        Set the left RGB image
        :param image: RGB image
        """
        self.rgb_image = image
    
    @property
    def right_image(self):
        """
        Get the right RGB image
        :return: RGB image
        """
        return self.rgb_right_image
    
    @right_image.setter
    def right_image(self, image):
        """
        Set the right RGB image
        :param image: RGB image
        """
        self.rgb_right_image = image


class AzureKinectCamera(BaseCamera):
    def __init__(self, config_file=None):
        super(AzureKinectCamera, self).__init__()
        cur_path = os.path.dirname(os.path.abspath(__file__))
        self.camparas_file = os.path.join(cur_path, 
            'assets/calibration/external_camera', 'azure_kinect_paras.json')
        self.max_depth = 5000  # mm
        self.fps = 30

        if config_file is not None:
            self.read_config(config_file)
            logging.info("==> Camera configuration loaded from %s", config_file)

        with open(self.camparas_file, 'r') as f:
            cam_paras_dict = json.load(f)
        cam_paras = cam_paras_dict["left"]

        self.R = np.array(cam_paras["R"])
        self.T = np.array(cam_paras["T"])
        self.K = np.array(cam_paras["K"])
        self.D = np.array(cam_paras["D"])
        self.w = cam_paras["w"]
        self.h = cam_paras["h"]
        self.type = "external"
        
        # Initialize Azure Kinect
        pykinect.initialize_libraries()
        
        # Device configuration
        device_config = pykinect.default_configuration
        device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
        device_config.camera_fps = pykinect.K4A_FRAMES_PER_SECOND_30
        
        # Adjust resolution based on config
        if self.w == 1280 and self.h == 720:
            device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
        elif self.w == 1920 and self.h == 1080:
            device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
        elif self.w == 2560 and self.h == 1440:
            device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1440P
        elif self.w == 3840 and self.h == 2160:
            device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_2160P
            
        # Start device
        self.device = pykinect.start_device(config=device_config)
        
        # zero loop camera
        self.rgb_image = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.depth_map = np.zeros((self.h, self.w), dtype=np.float32)
        for i in range(30):
            rgb_image, depth_map = self.capture_image()

    def restore_camparas(self, config_file):
        """
        Restore the camera parameters
        :return: None
        """
        cam_paras = {
            "R": self.R.tolist(),  # rotation corresponding to the robot arm GetActualTCPPose 
            "T": self.T.tolist(),  # translation corresponding to the robot arm GetActualTCPPose
            "K": self.K.tolist(),
            "D": self.D.tolist(),
            "w": self.w,
            "h": self.h,
        }
        cam_paras_dict = {
            "left": cam_paras,
        }
        # write cam_paras_dict to a json file
        with open(config_file, 'w') as f:
            json.dump(cam_paras_dict, f, indent=4)
        logging.info("Camera parameters saved to %s", config_file)
        
    def restore_config(self, config_file):
        """
        Restore the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        config_paras = {
            "cam_paras_file": self.camparas_file,
            "fps": self.fps,
            "max_depth": self.max_depth,
            "type": self.type,
        }
        # write config_paras to a json file
        with open(config_file, 'w') as f:
            json.dump(config_paras, f, indent=4)
        logging.info("Configuration saved to %s", config_file)

    def read_config(self, config_file):
        """
        Read the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        with open(config_file, 'r') as f:
            config_paras = json.load(f)
        self.cam_paras_file = config_paras["cam_paras_file"]
        self.max_depth = config_paras["max_depth"]
        self.fps = config_paras["fps"]
        self.type = config_paras["type"]

    def capture_image(self):
        """
        Capture an image from the camera
        :return: RGB image and depth image
        """
        try:
            # Get capture
            capture = self.device.update()

            # Get the color image
            ret_color, color_image = capture.get_color_image()
            
            # Get the depth image
            ret_depth, depth_image = capture.get_transformed_depth_image()

            if ret_color and ret_depth:
                # Convert BGRA to RGB
                CamBGRA = color_image
                CamRGB = cv2.cvtColor(CamBGRA, cv2.COLOR_BGRA2RGB)
                
                # Process depth image
                CamD = depth_image.astype(np.float32)
                CamD[np.isnan(CamD)] = 0
                CamD[CamD > self.max_depth] = 0
                
                # Resize if necessary to match expected dimensions
                if CamRGB.shape[:2] != (self.h, self.w):
                    CamRGB = cv2.resize(CamRGB, (self.w, self.h))
                if CamD.shape != (self.h, self.w):
                    CamD = cv2.resize(CamD, (self.w, self.h))
                    
            else:
                CamRGB = None
                CamD = None
                print("Azure Kinect camera capture fail")

        except Exception as e:
            CamRGB = None
            CamD = None
            print(f"Azure Kinect camera error: {e}")

        self.rgb_image = CamRGB
        self.depth_map = CamD
        return self.rgb_image, self.depth_map


class RealSense455Camera(BaseCamera):
    def __init__(self, config_file=None, is_warmup=True, target_serial=None, color_only=False, warmup_frames=30):
        super(RealSense455Camera, self).__init__()
        cur_path = os.path.dirname(os.path.abspath(__file__))
        self.camparas_file = os.path.join(cur_path, 
            'assets/calibration/external_camera', 'realsense455_paras.json')
        self.preset_file = os.path.join(cur_path, 
            'assets/calibration/external_camera', 'realsense455_HighResHighAccuracyPreset.json')
        self.max_depth = 10000  # mm (D455 has longer range than D435)
        self.fps = 30
        self.type = "external"
        self.target_serial = target_serial  # Store target serial number
        self.color_only = color_only
        self.pipeline = None
        self.profile = None
        self.ctx = None
        self.device = None
        self._closed = False
        # RealSense owns native streaming threads; ensure they are stopped
        # even when the caller exits through an exception.
        atexit.register(self.close)

        if config_file is not None:
            self.read_config(config_file)
            logging.info("==> Camera configuration loaded from %s", config_file)

        with open(self.camparas_file, 'r') as f:
            cam_paras_dict = json.load(f)
        left_cam_paras = cam_paras_dict["left"]

        self.R = np.array(left_cam_paras["R"])
        self.T = np.array(left_cam_paras["T"])
        self.K = np.array(left_cam_paras["K"])
        self.D = np.array(left_cam_paras["D"])
        self.w = left_cam_paras["w"]
        self.h = left_cam_paras["h"]

        # zero loop camera
        self.rgb_image = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.depth_map = np.zeros((self.h, self.w), dtype=np.float32)

        if is_warmup:
            self._initialize_camera()
            for _ in range(warmup_frames):
                self.capture_image()

    def close(self):
        """Stop RealSense streaming and release native SDK resources."""
        if self._closed:
            return
        self._closed = True
        atexit.unregister(self.close)
        pipeline, self.pipeline = self.pipeline, None
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                # The device may already have been released by the SDK.
                pass
        self.profile = None
        self.device = None
        self.ctx = None

    def _initialize_camera(self):
        """Initialize the RealSense D455 camera"""
        try:
            # Create a context object to manage device and sensor
            self.ctx = rs.context()
            devices = self.ctx.query_devices()
            
            if len(devices) == 0:
                raise RuntimeError("No RealSense devices found")
            
            # Determine which device to use
            if self.target_serial:
                # Use specific serial number if provided
                target_device = None
                for device in devices:
                    serial_number = device.get_info(rs.camera_info.serial_number)
                    if serial_number == self.target_serial:
                        device_name = device.get_info(rs.camera_info.name)
                        logging.info(f"Found target device with serial {self.target_serial}: {device_name}")
                        # Verify it's a D455 device
                        if "D455" in device_name or "455" in device_name:
                            target_device = device
                            break
                        else:
                            raise RuntimeError(f"Device with serial {self.target_serial} is not a D455 device: {device_name}")
                
                if target_device is None:
                    raise RuntimeError(f"No device found with serial number: {self.target_serial}")
                
                self.device = target_device
            else:
                # Find D455 device automatically
                d455_device = None
                for device in devices:
                    device_name = device.get_info(rs.camera_info.name)
                    product_id = device.get_info(rs.camera_info.product_id)
                    
                    # Check if it's a D455 device
                    if ("D455" in device_name or "455" in device_name or 
                        product_id in ['0x0B5C', '0x0B5B']):
                        d455_device = device
                        serial_number = device.get_info(rs.camera_info.serial_number)
                        logging.info(f"Found RealSense D455 device: {device_name}, Serial: {serial_number}")
                        break
                    else:
                        logging.warning(f"Found non-D455 device: {device_name}, skipping...")
                
                if d455_device is None:
                    raise RuntimeError("No RealSense D455 device found. Please ensure D455 is connected.")
                
                self.device = d455_device
            
            # Rest of the initialization code...
            self.pipeline = rs.pipeline(self.ctx)
            config = rs.config()
            
            # Enable device-specific stream configuration
            config.enable_device(self.device.get_info(rs.camera_info.serial_number))
            
            # Configure streams
            config.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, self.fps)
            if not self.color_only:
                config.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, self.fps)
            
            # Start streaming
            self.profile = self.pipeline.start(config)

            try:
                color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
                color_intrinsics = color_profile.get_intrinsics()
                logging.info("彩色相机内参 (Color Intrinsics):")
                logging.info(f"  Width: {color_intrinsics.width}")
                logging.info(f"  Height: {color_intrinsics.height}")
                logging.info(f"  Fx: {color_intrinsics.fx}")
                logging.info(f"  Fy: {color_intrinsics.fy}")
                logging.info(f"  Ppx: {color_intrinsics.ppx}")
                logging.info(f"  Ppy: {color_intrinsics.ppy}")
                logging.info(f"  Model: {color_intrinsics.model}")
                logging.info(f"  Coeffs (畸变系数): {color_intrinsics.coeffs}")

                if not self.color_only:
                    depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
                    depth_intrinsics = depth_profile.get_intrinsics()
                    logging.info("深度相机内参 (Depth Intrinsics):")
                    logging.info(f"  Width: {depth_intrinsics.width}")
                    logging.info(f"  Height: {depth_intrinsics.height}")
                    logging.info(f"  Fx: {depth_intrinsics.fx}")
                    logging.info(f"  Fy: {depth_intrinsics.fy}")
                    logging.info(f"  Ppx: {depth_intrinsics.ppx}")
                    logging.info(f"  Ppy: {depth_intrinsics.ppy}")
                    logging.info(f"  Model: {depth_intrinsics.model}")
                    logging.info(f"  Coeffs (畸变系数): {depth_intrinsics.coeffs}")

            except Exception as e:
                logging.error(f"获取相机内参时发生错误: {e}")
            
            if self.color_only:
                self.depth_scale = 0.001
                self.align = None
                logging.info("RealSense D455 color-only mode enabled (no depth stream)")
            else:
                self._configure_depth_sensor()
                depth_sensor = self.profile.get_device().first_depth_sensor()
                self.depth_scale = depth_sensor.get_depth_scale()
                logging.info(f"Depth Scale is: {self.depth_scale}")
                self.align = rs.align(rs.stream.color)

            for _ in range(10):
                try:
                    self.pipeline.wait_for_frames(timeout_ms=1000)
                except Exception:
                    break
            
        except Exception as e:
            logging.error(f"Failed to initialize RealSense D455 camera: {e}")
            raise

    def _configure_depth_sensor(self):
        """Configure depth sensor settings for optimal performance"""
        try:
            depth_sensor = self.profile.get_device().first_depth_sensor()
            
            # Disable auto exposure for consistent results
            depth_sensor.set_option(rs.option.enable_auto_exposure, 0)
            
            # Set exposure (microseconds) - adjust based on lighting conditions
            depth_sensor.set_option(rs.option.exposure, 8500)
            
            # Set laser power (0-360 for D455)
            depth_sensor.set_option(rs.option.laser_power, 240.0)
            
            # Set gain
            depth_sensor.set_option(rs.option.gain, 16.0)
            
            # Enable auto white balance for color
            color_sensor = self.profile.get_device().first_color_sensor()
            color_sensor.set_option(rs.option.enable_auto_white_balance, 1)
            
            logging.info("Depth sensor configured successfully")
            
        except Exception as e:
            logging.warning(f"Failed to configure depth sensor: {e}")

    def capture_image(self):
        """
        Capture an image from the camera
        :return: RGB image and depth image
        """
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=10000)
            if self.color_only:
                color_frame = frames.get_color_frame()
                if not color_frame:
                    logging.warning("Failed to get valid color frame")
                    return None, None
                CamBGR = np.array(color_frame.get_data())
                CamRGB = cv2.cvtColor(CamBGR, cv2.COLOR_BGR2RGB)
                CamD = np.zeros((self.h, self.w), dtype=np.float32)
            else:
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    logging.warning("Failed to get valid frames")
                    return None, None
                CamBGR = np.array(color_frame.get_data())
                CamRGB = cv2.cvtColor(CamBGR, cv2.COLOR_BGR2RGB)
                CamD = np.array(depth_frame.get_data()) * self.depth_scale * 1000
                CamD[CamD > self.max_depth] = 0
                CamD[np.isnan(CamD)] = 0
            
        except Exception as e:
            logging.error(f"RealSense D455 camera capture error: {e}")
            CamRGB = None
            CamD = None

        self.rgb_image = CamRGB
        self.depth_map = CamD
        return self.rgb_image, self.depth_map

    def restore_camparas(self, config_file):
        """
        Restore the camera parameters
        :return: None
        """
        left_cam_paras = {
            "R": self.R.tolist(),
            "T": self.T.tolist(),
            "K": self.K.tolist(),
            "D": self.D.tolist(),
            "w": self.w,
            "h": self.h,
        }
        cam_paras_dict = {
            "left": left_cam_paras,
        }
        # write cam_paras_dict to a json file
        with open(config_file, 'w') as f:
            json.dump(cam_paras_dict, f, indent=4)
        logging.info("Camera parameters saved to %s", config_file)

    def restore_config(self, config_file):
        """
        Restore the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        config_paras = {
            "cam_paras_file": self.camparas_file,
            "preset_file": self.preset_file,
            "fps": self.fps,
            "max_depth": self.max_depth,
            "type": self.type,
        }
        # write config_paras to a json file
        with open(config_file, 'w') as f:
            json.dump(config_paras, f, indent=4)
        logging.info("Configuration saved to %s", config_file)

    def read_config(self, config_file):
        """
        Read the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        with open(config_file, 'r') as f:
            config_paras = json.load(f)
        self.cam_paras_file = config_paras["cam_paras_file"]
        self.preset_file = config_paras.get("preset_file", self.preset_file)
        self.max_depth = config_paras["max_depth"]
        self.fps = config_paras["fps"]
        self.type = config_paras["type"]

    def get_intrinsics(self):
        """Get camera intrinsic parameters"""
        if hasattr(self, 'profile'):
            color_stream = self.profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            return {
                'width': intrinsics.width,
                'height': intrinsics.height,
                'fx': intrinsics.fx,
                'fy': intrinsics.fy,
                'ppx': intrinsics.ppx,
                'ppy': intrinsics.ppy,
                'coeffs': intrinsics.coeffs
            }
        return None


class RealSense515Camera(BaseCamera):
    def __init__(self, config_file=None, is_warmup=True, target_serial=None):
        super(RealSense515Camera, self).__init__()
        cur_path = os.path.dirname(os.path.abspath(__file__))
        self.camparas_file = os.path.join(cur_path, 
            'assets/calibration/external_camera', 'realsense515_paras.json')
        self.preset_file = os.path.join(cur_path, 
            'assets/calibration/external_camera', 'realsense515_HighResHighAccuracyPreset.json')
        self.max_depth = 9000  # mm (L515 has up to 9m range)
        self.fps = 30
        self.type = "external"
        self.target_serial = target_serial  # Store target serial number

        if config_file is not None:
            self.read_config(config_file)
            logging.info("==> Camera configuration loaded from %s", config_file)

        with open(self.camparas_file, 'r') as f:
            cam_paras_dict = json.load(f)
        left_cam_paras = cam_paras_dict["left"]

        self.R = np.array(left_cam_paras["R"])
        self.T = np.array(left_cam_paras["T"])
        self.K = np.array(left_cam_paras["K"])
        self.D = np.array(left_cam_paras["D"])
        self.w = left_cam_paras["w"]
        self.h = left_cam_paras["h"]

        # zero loop camera
        self.rgb_image = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.depth_map = np.zeros((self.h, self.w), dtype=np.float32)

        if is_warmup:
            self._initialize_camera()
            # Warm up the camera
            for i in range(30):
                rgb_image, depth_map = self.capture_image()

    def _initialize_camera(self):
        """Initialize the RealSense L515 camera"""
        try:
            # Create a context object to manage device and sensor
            self.ctx = rs.context()
            devices = self.ctx.query_devices()
            
            if len(devices) == 0:
                raise RuntimeError("No RealSense devices found")
            
            # Determine which device to use
            if self.target_serial:
                # Use specific serial number if provided
                target_device = None
                for device in devices:
                    serial_number = device.get_info(rs.camera_info.serial_number)
                    if serial_number == self.target_serial:
                        device_name = device.get_info(rs.camera_info.name)
                        logging.info(f"Found target device with serial {self.target_serial}: {device_name}")
                        # Verify it's a L515 device
                        if "L515" in device_name or "515" in device_name:
                            target_device = device
                            break
                        else:
                            raise RuntimeError(f"Device with serial {self.target_serial} is not a L515 device: {device_name}")
                
                if target_device is None:
                    raise RuntimeError(f"No device found with serial number: {self.target_serial}")
                
                self.device = target_device
            else:
                # Find L515 device automatically
                l515_device = None
                for device in devices:
                    device_name = device.get_info(rs.camera_info.name)
                    product_id = device.get_info(rs.camera_info.product_id)
                    
                    # Check if it's a L515 device
                    if ("L515" in device_name or "515" in device_name or 
                        product_id in ['0x0B64']):  # L515 product ID
                        l515_device = device
                        serial_number = device.get_info(rs.camera_info.serial_number)
                        logging.info(f"Found RealSense L515 device: {device_name}, Serial: {serial_number}")
                        break
                    else:
                        logging.warning(f"Found non-L515 device: {device_name}, skipping...")
                
                if l515_device is None:
                    raise RuntimeError("No RealSense L515 device found. Please ensure L515 is connected.")
                
                self.device = l515_device
            
            # Rest of the initialization code...
            self.pipeline = rs.pipeline(self.ctx)
            config = rs.config()
            
            # Enable device-specific stream configuration
            config.enable_device(self.device.get_info(rs.camera_info.serial_number))
            
            # Configure streams - L515 supports different resolutions
            config.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, self.fps)
            
            # Start streaming
            self.profile = self.pipeline.start(config)

            try:
                # Get depth sensor and color sensor
                depth_sensor = self.profile.get_device().first_depth_sensor()
                color_sensor = self.profile.get_device().first_color_sensor()

                # Get stream profiles
                depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
                color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))

                # Get intrinsics
                depth_intrinsics = depth_profile.get_intrinsics()
                color_intrinsics = color_profile.get_intrinsics()

                logging.info("深度相机内参 (Depth Intrinsics):")
                logging.info(f"  Width: {depth_intrinsics.width}")
                logging.info(f"  Height: {depth_intrinsics.height}")
                logging.info(f"  Fx: {depth_intrinsics.fx}")
                logging.info(f"  Fy: {depth_intrinsics.fy}")
                logging.info(f"  Ppx: {depth_intrinsics.ppx}")
                logging.info(f"  Ppy: {depth_intrinsics.ppy}")
                logging.info(f"  Model: {depth_intrinsics.model}")
                logging.info(f"  Coeffs (畸变系数): {depth_intrinsics.coeffs}")

                logging.info("彩色相机内参 (Color Intrinsics):")
                logging.info(f"  Width: {color_intrinsics.width}")
                logging.info(f"  Height: {color_intrinsics.height}")
                logging.info(f"  Fx: {color_intrinsics.fx}")
                logging.info(f"  Fy: {color_intrinsics.fy}")
                logging.info(f"  Ppx: {color_intrinsics.ppx}")
                logging.info(f"  Ppy: {color_intrinsics.ppy}")
                logging.info(f"  Model: {color_intrinsics.model}")
                logging.info(f"  Coeffs (畸变系数): {color_intrinsics.coeffs}")

            except Exception as e:
                logging.error(f"获取相机内参时发生错误: {e}")
            
            # Configure sensor settings (L515 has different settings than D455)
            self._configure_sensor()
            
            # Get depth scale
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            logging.info(f"Depth Scale is: {self.depth_scale}")
            
            # Create align object to align depth to color
            self.align = rs.align(rs.stream.color)
            
        except Exception as e:
            logging.error(f"Failed to initialize RealSense L515 camera: {e}")
            raise

    def _configure_sensor(self):
        """Configure sensor settings for optimal performance on L515"""
        try:
            depth_sensor = self.profile.get_device().first_depth_sensor()
            
            # L515 has different options than D455 - it uses LiDAR instead of stereo
            # Configure confidence threshold
            if depth_sensor.supports(rs.option.confidence_threshold):
                depth_sensor.set_option(rs.option.confidence_threshold, 1)
            
            # Configure invalidation bypass
            if depth_sensor.supports(rs.option.invalidation_bypass):
                depth_sensor.set_option(rs.option.invalidation_bypass, 0)
            
            # Configure laser power (if available)
            if depth_sensor.supports(rs.option.laser_power):
                depth_sensor.set_option(rs.option.laser_power, 100.0)
            
            # Configure color sensor
            color_sensor = self.profile.get_device().first_color_sensor()
            if color_sensor.supports(rs.option.enable_auto_white_balance):
                color_sensor.set_option(rs.option.enable_auto_white_balance, 1)
            
            # Set auto exposure for color
            if color_sensor.supports(rs.option.enable_auto_exposure):
                color_sensor.set_option(rs.option.enable_auto_exposure, 1)
            
            logging.info("L515 sensor configured successfully")
            
        except Exception as e:
            logging.warning(f"Failed to configure L515 sensor: {e}")

    def capture_image(self):
        """
        Capture an image from the camera
        :return: RGB image and depth image
        """
        try:
            # Wait for a coherent pair of frames
            frames = self.pipeline.wait_for_frames()
            
            # Align depth frame to color frame
            aligned_frames = self.align.process(frames)
            
            # Get aligned frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                logging.warning("Failed to get valid frames")
                return None, None
            
            # Convert to numpy arrays
            CamBGR = np.array(color_frame.get_data())
            CamRGB = cv2.cvtColor(CamBGR, cv2.COLOR_BGR2RGB)
            
            # Convert depth to millimeters and filter
            CamD = np.array(depth_frame.get_data()) * self.depth_scale * 1000
            CamD[CamD > self.max_depth] = 0
            CamD[CamD < 100] = 0  # L515 minimum distance is about 10cm
            CamD[np.isnan(CamD)] = 0
            CamD[np.isinf(CamD)] = 0
            
            # Apply light filtering for L515 noise reduction
            CamD = cv2.medianBlur(CamD.astype(np.float32), 3)
            
        except Exception as e:
            logging.error(f"RealSense L515 camera capture error: {e}")
            CamRGB = None
            CamD = None

        self.rgb_image = CamRGB
        self.depth_map = CamD
        return self.rgb_image, self.depth_map

    def restore_camparas(self, config_file):
        """
        Restore the camera parameters
        :return: None
        """
        left_cam_paras = {
            "R": self.R.tolist(),
            "T": self.T.tolist(),
            "K": self.K.tolist(),
            "D": self.D.tolist(),
            "w": self.w,
            "h": self.h,
        }
        cam_paras_dict = {
            "left": left_cam_paras,
        }
        # write cam_paras_dict to a json file
        with open(config_file, 'w') as f:
            json.dump(cam_paras_dict, f, indent=4)
        logging.info("Camera parameters saved to %s", config_file)

    def restore_config(self, config_file):
        """
        Restore the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        config_paras = {
            "cam_paras_file": self.camparas_file,
            "preset_file": self.preset_file,
            "fps": self.fps,
            "max_depth": self.max_depth,
            "type": self.type,
        }
        # write config_paras to a json file
        with open(config_file, 'w') as f:
            json.dump(config_paras, f, indent=4)
        logging.info("Configuration saved to %s", config_file)

    def read_config(self, config_file):
        """
        Read the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        with open(config_file, 'r') as f:
            config_paras = json.load(f)
        self.cam_paras_file = config_paras["cam_paras_file"]
        self.preset_file = config_paras.get("preset_file", self.preset_file)
        self.max_depth = config_paras["max_depth"]
        self.fps = config_paras["fps"]
        self.type = config_paras["type"]

    def get_intrinsics(self):
        """Get camera intrinsic parameters"""
        if hasattr(self, 'profile'):
            color_stream = self.profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            return {
                'width': intrinsics.width,
                'height': intrinsics.height,
                'fx': intrinsics.fx,
                'fy': intrinsics.fy,
                'ppx': intrinsics.ppx,
                'ppy': intrinsics.ppy,
                'coeffs': intrinsics.coeffs
            }
        return None




class OrbbecCamera(BaseCamera):
    def __init__(self, config_file=None, is_warmup=True, preset_name="Default", warmup_frames=30):
        super(OrbbecCamera, self).__init__()
        cur_path = os.path.dirname(os.path.abspath(__file__))
        self.camparas_file = os.path.join(cur_path,
            'assets/calibration/external_camera', 'orbbec_paras.json')
        self.max_depth = 5000  # mm
        self.fps = 30
        self.type = "external"
        self.pipeline = None
        self.depth_scale = 1.0
        self.orbbec_preset = preset_name
        self.use_dual_color = False
        self._orbbec_depth_enabled = True
        self._orbbec_align_mode = None
        self._ob_context = None
        self._ob_device = None
        self._closed = False
        # Ensure a native SDK stream is released even when the caller exits
        # through an uncaught exception.
        atexit.register(self.close)

        if config_file is not None:
            self.read_config(config_file)
            logging.info("==> Camera configuration loaded from %s", config_file)
        if preset_name is not None:
            self.orbbec_preset = preset_name

        with open(self.camparas_file, 'r') as f:
            cam_paras_dict = json.load(f)
        left_cam_paras = cam_paras_dict["left"]

        self.R = np.array(left_cam_paras["R"])
        self.T = np.array(left_cam_paras["T"])
        self.K = np.array(left_cam_paras["K"])
        self.D = np.array(left_cam_paras["D"])
        self.w = left_cam_paras["w"]
        self.h = left_cam_paras["h"]

        # zero loop camera
        self.rgb_image = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.rgb_right_image = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.depth_map = np.zeros((self.h, self.w), dtype=np.float32)

        if is_warmup:
            self._initialize_camera()
            for _ in range(warmup_frames):
                self.capture_image()

    def close(self):
        """Stop the Orbbec SDK pipeline and release the camera device."""
        if self._closed:
            return

        self._closed = True
        atexit.unregister(self.close)
        pipeline, self.pipeline = self.pipeline, None
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                # The SDK can already have released the device during shutdown.
                pass
        self._ob_device = None
        self._ob_context = None

    def _get_orbbec_presets(self, device):
        preset_list = device.get_available_preset_list()
        return [preset_list[i] for i in range(len(preset_list))]

    def _create_orbbec_pipeline(self, Context, Pipeline):
        self._ob_context = Context()
        device_list = self._ob_context.query_devices()
        if hasattr(device_list, "get_count") and device_list.get_count() == 0:
            raise RuntimeError("No Orbbec device found")
        self._ob_device = device_list.get_device_by_index(0)

        if self.orbbec_preset:
            available_presets = self._get_orbbec_presets(self._ob_device)
            if self.orbbec_preset not in available_presets:
                raise ValueError(
                    f"Orbbec preset '{self.orbbec_preset}' is unavailable. "
                    f"Available presets: {available_presets}"
                )
            self._ob_device.load_preset(self.orbbec_preset)
            logging.info("Orbbec preset loaded: %s", self.orbbec_preset)

        self.pipeline = Pipeline(self._ob_device)

    def _orbbec_get_video_profile(self, profiles, width, height, pixel_format, fps, format_alternates=()):
        try:
            return profiles.get_video_stream_profile(width, height, pixel_format, fps)
        except Exception:
            pass

        preferred_formats = (pixel_format,) + tuple(format_alternates)
        candidates = []
        for i in range(len(profiles)):
            profile = profiles[i]
            if not hasattr(profile, "as_video_stream_profile"):
                continue
            video_profile = profile.as_video_stream_profile()
            if video_profile.get_format() not in preferred_formats:
                continue
            candidates.append(video_profile)

        if not candidates:
            raise RuntimeError(
                f"No Orbbec stream profile found for {width}x{height} @ {fps}fps "
                f"(format={pixel_format})"
            )

        def _profile_score(video_profile):
            profile_w = video_profile.get_width()
            profile_h = video_profile.get_height()
            profile_fps = video_profile.get_fps()
            same_resolution = profile_w == width and profile_h == height
            resolution_diff = abs(profile_w - width) + abs(profile_h - height)
            fps_diff = abs(profile_fps - fps)
            return (0 if same_resolution else 1, resolution_diff, fps_diff, -profile_fps)

        best_profile = min(candidates, key=_profile_score)
        logging.info(
            "Orbbec stream profile fallback: requested %dx%d@%dfps, using %dx%d@%dfps",
            width, height, fps,
            best_profile.get_width(), best_profile.get_height(), best_profile.get_fps(),
        )
        return best_profile

    def _orbbec_video_profile_shape(self, video_profile):
        if hasattr(video_profile, "as_video_stream_profile"):
            video_profile = video_profile.as_video_stream_profile()
        return video_profile.get_width(), video_profile.get_height(), video_profile.get_fps()

    def _orbbec_get_color_profile(self, sensor_type, OBSensorType, OBFormat):
        color_profiles = self.pipeline.get_stream_profile_list(sensor_type)
        return self._orbbec_get_video_profile(
            color_profiles, self.w, self.h, OBFormat.RGB, self.fps,
            format_alternates=(OBFormat.MJPG, OBFormat.BGR))

    def _orbbec_build_config(self, Config, OBSensorType, OBFormat):
        config = Config()
        self.use_dual_color = False
        self._orbbec_depth_enabled = True

        if self.orbbec_preset == "Dual Color Streams":
            left_color_profile = self._orbbec_get_color_profile(
                OBSensorType.LEFT_COLOR_SENSOR, OBSensorType, OBFormat)
            right_color_profile = self._orbbec_get_color_profile(
                OBSensorType.RIGHT_COLOR_SENSOR, OBSensorType, OBFormat)
            config.enable_stream(left_color_profile)
            config.enable_stream(right_color_profile)
            self.use_dual_color = True
            self._orbbec_depth_enabled = False
            logging.info("Orbbec dual RGB streams enabled by preset")
            return config

        try:
            left_color_profile = self._orbbec_get_color_profile(
                OBSensorType.LEFT_COLOR_SENSOR, OBSensorType, OBFormat)
            right_color_profile = self._orbbec_get_color_profile(
                OBSensorType.RIGHT_COLOR_SENSOR, OBSensorType, OBFormat)
            config.enable_stream(left_color_profile)
            config.enable_stream(right_color_profile)
            self.use_dual_color = True
            color_profile = left_color_profile
            logging.info("Orbbec dual RGB streams enabled")
        except Exception as e:
            logging.warning("Orbbec dual RGB streams unavailable, fallback to single color stream: %s", e)
            color_profile = self._orbbec_get_color_profile(
                OBSensorType.COLOR_SENSOR, OBSensorType, OBFormat)
            config.enable_stream(color_profile)

        stream_w, stream_h, stream_fps = self._orbbec_video_profile_shape(color_profile)
        depth_profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_profile = self._orbbec_get_video_profile(
            depth_profiles, stream_w, stream_h, OBFormat.Y16, stream_fps)
        config.enable_stream(depth_profile)
        return config

    def _initialize_camera(self):
        """Initialize the Orbbec camera through pyorbbecsdk."""
        try:
            from pyorbbecsdk import (  # pylint: disable=import-outside-toplevel
                Config,
                Context,
                OBAlignMode,
                OBFrameType,
                OBFormat,
                OBSensorType,
                Pipeline,
            )
        except ImportError as e:
            raise ImportError(
                "pyorbbecsdk is required for OrbbecCamera. "
                "Install the Orbbec Python SDK before using is_warmup=True."
            ) from e

        self._ob_format = OBFormat
        self._ob_frame_type = OBFrameType
        self._ob_sensor_type = OBSensorType
        self._create_orbbec_pipeline(Context, Pipeline)

        if self.orbbec_preset == "Dual Color Streams":
            config = self._orbbec_build_config(Config, OBSensorType, OBFormat)
            self.pipeline.start(config)
            self._orbbec_align_mode = OBAlignMode.DISABLE
            return

        align_attempts = (
            (OBAlignMode.HW_MODE, "hardware D2C"),
            (OBAlignMode.SW_MODE, "software D2C"),
            (OBAlignMode.DISABLE, "alignment disabled"),
        )
        last_error = None
        for align_mode, label in align_attempts:
            config = self._orbbec_build_config(Config, OBSensorType, OBFormat)
            try:
                config.set_align_mode(align_mode)
                self.pipeline.start(config)
                self._orbbec_align_mode = align_mode
                logging.info("Orbbec pipeline started with %s", label)
                break
            except Exception as e:
                last_error = e
                logging.warning("Orbbec pipeline start failed (%s): %s", label, e)
                try:
                    self.pipeline.stop()
                except Exception:
                    pass
        else:
            raise last_error

    def _frame_to_rgb(self, frame):
        if frame is None:
            return None
        if hasattr(frame, "as_video_frame"):
            frame = frame.as_video_frame()
        color_data = np.frombuffer(frame.get_data(), dtype=np.uint8)
        color_format = frame.get_format()
        if color_format == self._ob_format.RGB:
            return color_data.reshape((frame.get_height(), frame.get_width(), 3))
        if color_format == self._ob_format.BGR:
            cam_bgr = color_data.reshape((frame.get_height(), frame.get_width(), 3))
            return cv2.cvtColor(cam_bgr, cv2.COLOR_BGR2RGB)
        cam_bgr = cv2.imdecode(color_data, cv2.IMREAD_COLOR)
        if cam_bgr is None:
            raise ValueError(f"Unsupported Orbbec color format: {color_format}")
        return cv2.cvtColor(cam_bgr, cv2.COLOR_BGR2RGB)

    def _get_frame_by_type(self, frames, frame_type):
        if hasattr(frames, "get_frame_by_type"):
            return frames.get_frame_by_type(frame_type)
        return frames.get_frame(frame_type)

    def restore_camparas(self, config_file):
        """
        Restore the camera parameters
        :return: None
        """
        left_cam_paras = {
            "R": self.R.tolist(),
            "T": self.T.tolist(),
            "K": self.K.tolist(),
            "D": self.D.tolist(),
            "w": self.w,
            "h": self.h,
        }
        cam_paras_dict = {
            "left": left_cam_paras,
        }
        with open(config_file, 'w') as f:
            json.dump(cam_paras_dict, f, indent=4)
        logging.info("Camera parameters saved to %s", config_file)

    def restore_config(self, config_file):
        """
        Restore the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        config_paras = {
            "cam_paras_file": self.camparas_file,
            "fps": self.fps,
            "max_depth": self.max_depth,
            "type": self.type,
            "orbbec_preset": self.orbbec_preset,
        }
        with open(config_file, 'w') as f:
            json.dump(config_paras, f, indent=4)
        logging.info("Configuration saved to %s", config_file)

    def read_config(self, config_file):
        """
        Read the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        with open(config_file, 'r') as f:
            config_paras = json.load(f)
        self.camparas_file = config_paras["cam_paras_file"]
        self.max_depth = config_paras["max_depth"]
        self.fps = config_paras["fps"]
        self.type = config_paras["type"]
        self.orbbec_preset = config_paras.get("orbbec_preset", self.orbbec_preset)

    def capture_image(self):
        """
        Capture an image from the camera
        :return: RGB image and depth image
        """
        if self.pipeline is None:
            self._initialize_camera()

        try:
            frames = self.pipeline.wait_for_frames(1000)
            if frames is None:
                logging.warning("Orbbec camera capture timeout")
                return None, None

            depth_frame = frames.get_depth_frame() if self._orbbec_depth_enabled else None
            if self.use_dual_color:
                left_color_frame = self._get_frame_by_type(frames, self._ob_frame_type.LEFT_COLOR_FRAME)
                right_color_frame = self._get_frame_by_type(frames, self._ob_frame_type.RIGHT_COLOR_FRAME)
            else:
                left_color_frame = frames.get_color_frame()
                right_color_frame = None

            if left_color_frame is None or (self._orbbec_depth_enabled and depth_frame is None):
                logging.warning("Failed to get valid Orbbec frames")
                return None, None

            CamRGB = self._frame_to_rgb(left_color_frame)
            CamRGBRight = self._frame_to_rgb(right_color_frame) if right_color_frame is not None else None

            if depth_frame is not None:
                frame_scale = self.depth_scale
                if hasattr(depth_frame, "get_depth_scale"):
                    try:
                        frame_scale = depth_frame.get_depth_scale()
                        self.depth_scale = frame_scale
                    except Exception:
                        pass
                depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                CamD = depth_data.reshape((depth_frame.get_height(), depth_frame.get_width()))
                CamD = CamD.astype(np.float32) * frame_scale
                CamD[CamD > self.max_depth] = 0
                CamD[np.isnan(CamD)] = 0
                CamD[np.isinf(CamD)] = 0
            else:
                CamD = np.zeros((self.h, self.w), dtype=np.float32)

            if CamRGB.shape[:2] != (self.h, self.w):
                CamRGB = cv2.resize(CamRGB, (self.w, self.h))
            if CamRGBRight is not None and CamRGBRight.shape[:2] != (self.h, self.w):
                CamRGBRight = cv2.resize(CamRGBRight, (self.w, self.h))
            if CamD.shape != (self.h, self.w):
                CamD = cv2.resize(CamD, (self.w, self.h), interpolation=cv2.INTER_NEAREST)

        except Exception as e:
            logging.error("Orbbec camera capture error: %s", e)
            CamRGB = None
            CamRGBRight = None
            CamD = None

        self.rgb_image = CamRGB
        self.rgb_right_image = CamRGBRight if CamRGBRight is not None else (CamRGB.copy() if CamRGB is not None else None)
        self.depth_map = CamD
        return self.rgb_image, self.depth_map

    @property
    def left_image(self):
        """
        Get the left RGB image
        :return: RGB image
        """
        return self.rgb_image

    @left_image.setter
    def left_image(self, image):
        """
        Set the left RGB image
        :param image: RGB image
        """
        self.rgb_image = image

    @property
    def right_image(self):
        """
        Get the right RGB image
        :return: RGB image
        """
        return self.rgb_right_image

    @right_image.setter
    def right_image(self, image):
        """
        Set the right RGB image
        :param image: RGB image
        """
        self.rgb_right_image = image


# Update the ExternCameraCallbacks dictionary
ExternCameraCallbacks = {
    'ZED': ZEDCamera,
    'AzureKinect': AzureKinectCamera,
    'RealSense455': RealSense455Camera,
    'RealSense515': RealSense515Camera,
    'Orbbec': OrbbecCamera,
}
