import time
import numpy as np
from pyorbbecsdk import *

from scripts.utils import apply_color_to_depth_image
from scripts.orbbec_utils import frame_to_bgr_image

class Orbbec:
    #Tested on Orbbec Gemini Pro sensor
    #Unit in mm
    def __init__(self):
        config = Config()
        pipeline = self._open_pipeline()
        device = pipeline.get_device()
        device_info = device.get_device_info()
        device_pid = device_info.get_pid()

        #Set depth sensor
        try:
            profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            assert profile_list is not None
            depth_profile = profile_list.get_default_video_stream_profile()
            assert depth_profile is not None
            # print("depth profile: ", depth_profile)
            config.enable_stream(depth_profile)
        except Exception as e:
            print("Depth sensor setup error:\n", e)
            return
        
        #Set aligned color sensor
        try:
            profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            if profile_list is not None:
                color_profile: VideoStreamProfile = profile_list.get_default_video_stream_profile()
                config.enable_stream(color_profile)
                if device_pid == 0x066B:
                    # Femto Mega does not support hardware D2C, and it is changed to software D2C
                    config.set_align_mode(OBAlignMode.SW_MODE)
                else:
                    config.set_align_mode(OBAlignMode.HW_MODE)
        except OBError as e:
            config.set_align_mode(OBAlignMode.DISABLE)
            print("Color sensor setup error:\n", e)

        self.pipeline = pipeline
        self.config = config
        self.depth_intrinsic = None
        self.rgb_intrinsic = None

    def _open_pipeline(self, retries=3, retry_delay_s=1.0):
        last_error = None
        for attempt in range(retries):
            try:
                return Pipeline()
            except RuntimeError as e:
                last_error = e
                if "uvc_open failed" not in str(e) or attempt == retries - 1:
                    break
                print(
                    f"[Warning] Orbbec camera is busy (attempt {attempt + 1}/{retries}). "
                    "Retrying after releasing other camera processes..."
                )
                time.sleep(retry_delay_s)
        raise RuntimeError(
            f"Failed to open Orbbec camera: {last_error}. "
            "Another process may still be using the camera; stop other scripts "
            "using the Orbbec device and try again."
        ) from last_error
        
    def __enter__(self):
        self.pipeline.start(self.config)
        camera_param = self.pipeline.get_camera_param()
        self.depth_intrinsic = camera_param.depth_intrinsic
        self.rgb_intrinsic = camera_param.rgb_intrinsic
        self.update()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()

    def release(self):
        if getattr(self, "pipeline", None) is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        
    def update(self, t=1000):
        self.frames = self.pipeline.wait_for_frames(t)
        self.depth_image = None
    
    def get_depth_image(self, min_depth_mm=20, max_depth_mm=10000):
        if self.frames is None:
            return None
        depth_frame = self.frames.get_depth_frame()
        if depth_frame is None:
            return None
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        scale = depth_frame.get_depth_scale()

        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
        depth_data = depth_data.reshape((height, width))

        depth_data = depth_data.astype(np.float32) * scale
        depth_data = np.where((depth_data > min_depth_mm) & (depth_data < max_depth_mm), depth_data, 0)
        depth_data = depth_data.astype(np.uint16)

        self.depth_image = depth_data
        return depth_data

    def get_color_image(self):
        if self.frames is None:
            return None
        color_frame = self.frames.get_color_frame()
        if color_frame is None:
            return None
        # covert to RGB format
        color_image = frame_to_bgr_image(color_frame)
        if color_image is None:
            print("failed to convert frame to image")
            return None
        return color_image

    def get_point_cloud(self):
        if self.frames is None:
            return None
        pts = self.frames.get_point_cloud(self.pipeline.get_camera_param())
        return pts

    def get_colored_depth_image(self, min_depth_mm=20, max_depth_mm=10000):
        depth_image = self.depth_image if self.depth_image is not None else self.get_depth_image(min_depth_mm=min_depth_mm, max_depth_mm=max_depth_mm)
        if depth_image is None:
            return None
        return apply_color_to_depth_image(depth_image)
    
    def get_intrinsics_matrix(self):
        if not self.depth_intrinsic: return None
        intrinsics = self.depth_intrinsic
        intrinsic_matrix = np.array(
            [
                [intrinsics.fx, 0, intrinsics.cx],
                [0, intrinsics.fy, intrinsics.cy],
                [0, 0, 1],
            ]
        )
        return intrinsic_matrix
