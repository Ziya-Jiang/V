import cv2
import zmq
import time
import struct
from collections import deque
import numpy as np
import pyrealsense2 as rs
import threading
from multiprocessing import shared_memory

from shared_memory_manager import SharedMemoryManager
from zmq_stream import pack_frame

try:
    import sapien.core as sapien
    SAPIEN_AVAILABLE = True
except ImportError:
    sapien = None
    SAPIEN_AVAILABLE = False
    print("Warning: sapien not available. SAPIEN camera functionality will be disabled.")

try:
    import pykinect_azure as pykinect
    from pykinect_azure.k4a._k4atypes import K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR
    KINECT_AVAILABLE = True
except ImportError:
    pykinect = None
    KINECT_AVAILABLE = False
    print("Warning: pykinect_azure not available. Kinect Azure camera functionality will be disabled.")

try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except ImportError:
    sl = None
    ZED_AVAILABLE = False
    print("Warning: pyzed.sl not available. ZED camera functionality will be disabled.")

def apply_color_to_depth_image(depth_image):
    colored_depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    colored_depth_image = cv2.applyColorMap(colored_depth_image, cv2.COLORMAP_JET)
    return colored_depth_image

class ZedCamera():
    def __init__(self, img_shape, fps, depth_shape=None) -> None:
        if not ZED_AVAILABLE:
            raise ImportError("ZED camera requires the 'pyzed' package.")
        self.img_shape = img_shape
        self.fps = fps
        self.depth_shape = depth_shape

        self.init_zed()
    
    def init_zed(self):
        # Create a Camera object
        self.zed = sl.Camera()
        
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720 
        init_params.camera_fps = self.fps 
        
        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : "+repr(err)+". Exit program.")
            exit()
        
    def get_frame(self):
        color_frame = sl.Mat()
        depth_frame = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(color_frame, sl.VIEW.SIDE_BY_SIDE)
            self.zed.retrieve_measure(depth_frame, sl.MEASURE.DEPTH)
            timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
            # print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(color_frame.get_width(), color_frame.get_height(),
            #       timestamp.get_milliseconds()))
        
            color_image = color_frame.get_data()

            depth_image = None
            if self.depth_shape is not None:
                depth_image = depth_frame.get_data().astype(np.float32).copy()  # meters, may contain nan/inf

            color_image = color_image[..., :3]
            
            width = color_frame.get_width()
            left_img = color_image[:, :width//2, :3]  # Left half (RGB)
            right_img = color_image[:, width//2:, :3]  # Right half (RGB)
            target_h, target_total_width = self.img_shape  # (h, w*2)
            target_w = target_total_width // 2  # Target width for each image
            left_img = cv2.resize(left_img, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
            right_img = cv2.resize(right_img, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
            color_image = np.concatenate([left_img, right_img], axis=1)
            
            # Resize depth if enabled
            if depth_image is not None:
                target_h, target_w = self.depth_shape  # (h, w)
                depth_image = cv2.resize(depth_image, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
        
            return color_image, depth_image
        else:
            print("Error: Unable to grab image from ZED camera.")
            return None, None
        
    def release(self):
        self.zed.close()    

class KinectAzureCamera():
    def __init__(self, img_shape, fps, enable_depth=False) -> None:
        if not KINECT_AVAILABLE:
            raise ImportError("Kinect Azure camera requires the 'pykinect_azure' package.")
        self.img_shape = img_shape
        self.fps = fps
        self.enable_depth = enable_depth

        pykinect.initialize_libraries()
        # Modify camera configuration
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED
        device_config.synchronized_images_only = True
        
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

    def get_frame(self):
        self.update()
        color_img = self.get_color_image() 
        depth_img = self.get_depth_image()
        resize_color_img = cv2.resize(color_img, (self.img_shape[1], self.img_shape[0]), interpolation = cv2.INTER_LINEAR)
        resize_depth_img = cv2.resize(depth_img, (self.img_shape[1], self.img_shape[0]), interpolation = cv2.INTER_LINEAR)
        
        return resize_color_img, resize_depth_img

    def release(self):
        pass

class RealSenseCamera():
    def __init__(self, img_shape, fps, serial_number=None, enable_depth=False,
                 depth_shape=None, no_resize=False) -> None:
        """
        img_shape: [height, width] for color
        depth_shape: [height, width] for depth (optional, separate from color)
        no_resize: keep native frame sizes without cv2.resize
        """
        self.img_shape = img_shape
        self.depth_shape = depth_shape
        self.fps = fps
        self.serial_number = serial_number
        self.enable_depth = enable_depth
        self.no_resize = no_resize

        self._dual_native = (
            enable_depth and no_resize and depth_shape is not None
            and (depth_shape[0] != img_shape[0] or depth_shape[1] != img_shape[1])
        )
        if self._dual_native:
            self.align = None
        else:
            align_to = rs.stream.depth if enable_depth and no_resize else rs.stream.color
            self.align = rs.align(align_to)

        self._product_line = self._query_product_line()
        self.init_realsense()

    def _query_product_line(self):
        if self.serial_number is None:
            return None
        for device in rs.context().query_devices():
            if device.get_info(rs.camera_info.serial_number) == self.serial_number:
                return device.get_info(rs.camera_info.product_line)
        return None

    def _resolution_fps_candidates(self, width, height, fps):
        candidates = [
            (width, height, fps),
            (width, height, 30),
            (width, height, 15),
            (640, 480, fps),
            (640, 480, 30),
            (1280, 720, fps),
            (1280, 720, 30),
            (960, 540, fps),
            (960, 540, 30),
            (848, 480, fps),
            (848, 480, 30),
        ]
        if self._product_line == "L500":
            candidates = [
                (width, height, fps),
                (width, height, 30),
                (1280, 720, fps),
                (1280, 720, 30),
                (960, 540, fps),
                (960, 540, 30),
                (1920, 1080, fps),
                (1920, 1080, 30),
                (640, 480, fps),
                (640, 480, 30),
            ]
        seen = set()
        ordered = []
        for item in candidates:
            if item not in seen:
                seen.add(item)
                ordered.append(item)
        return ordered

    def _start_pipeline(self, width, height, fps, depth_width=None, depth_height=None):
        config = rs.config()
        if self.serial_number is not None:
            config.enable_device(self.serial_number)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        if self.enable_depth:
            dw = depth_width if depth_width is not None else width
            dh = depth_height if depth_height is not None else height
            config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
        return self.pipeline.start(config)

    def _start_pipeline_l500_dual_native(self):
        """L515: color 1280x720 + depth 640x480, no align (separate native resolutions)."""
        color_w, color_h = self.img_shape[1], self.img_shape[0]
        depth_w, depth_h = self.depth_shape[1], self.depth_shape[0]
        for try_fps in dict.fromkeys([self.fps, 30, 15]):
            config = rs.config()
            if self.serial_number is not None:
                config.enable_device(self.serial_number)
            config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, try_fps)
            config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, try_fps)
            try:
                profile = self.pipeline.start(config)
                color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
                depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
                print(
                    f"[Image Server] L515 dual-native: color "
                    f"{color_profile.width()}x{color_profile.height()}@{color_profile.fps()}, "
                    f"depth {depth_profile.width()}x{depth_profile.height()}@{depth_profile.fps()} "
                    f"(no align)"
                )
                return profile
            except RuntimeError:
                continue
        raise RuntimeError(
            f"[Image Server] L515 {self.serial_number} failed dual-native pipeline "
            f"(color {color_w}x{color_h}, depth {depth_w}x{depth_h})"
        )

    def _start_pipeline_l500_depth_native(self):
        """L515: color 960x540 + depth 640x480, align to depth -> same 480x640."""
        config = rs.config()
        if self.serial_number is not None:
            config.enable_device(self.serial_number)
        for try_fps in dict.fromkeys([self.fps, 30, 15]):
            config = rs.config()
            if self.serial_number is not None:
                config.enable_device(self.serial_number)
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, try_fps)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, try_fps)
            try:
                profile = self.pipeline.start(config)
                color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
                depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
                print(
                    f"[Image Server] L515 depth-native mode: color "
                    f"{color_profile.width()}x{color_profile.height()}@{color_profile.fps()}, "
                    f"depth {depth_profile.width()}x{depth_profile.height()}@{depth_profile.fps()}, "
                    f"align=depth"
                )
                return profile
            except RuntimeError:
                continue
        raise RuntimeError(
            f"[Image Server] L515 {self.serial_number} failed to start depth-native pipeline"
        )

    def init_realsense(self):
        self.pipeline = rs.pipeline()
        target_w, target_h = self.img_shape[1], self.img_shape[0]
        profile = None
        used_fps = self.fps
        last_error = None

        if self._dual_native and self._product_line == "L500":
            profile = self._start_pipeline_l500_dual_native()
        elif self.enable_depth and self.no_resize and self._product_line == "L500":
            profile = self._start_pipeline_l500_depth_native()
        else:
            for color_w, color_h, try_fps in self._resolution_fps_candidates(target_w, target_h, self.fps):
                try:
                    profile = self._start_pipeline(color_w, color_h, try_fps)
                    used_fps = try_fps
                    break
                except RuntimeError as exc:
                    last_error = exc
                    continue

            if profile is None:
                config = rs.config()
                if self.serial_number is not None:
                    config.enable_device(self.serial_number)
                config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
                if self.enable_depth:
                    config.enable_stream(rs.stream.depth)
                try:
                    profile = self.pipeline.start(config)
                except RuntimeError as exc:
                    raise RuntimeError(
                        f"[Image Server] RealSense {self.serial_number} failed to start: {last_error or exc}"
                    ) from exc
                color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
                print(
                    f"[Image Server] RealSense {self.serial_number} does not support "
                    f"{target_w}x{target_h} @ {self.fps}fps, "
                    f"using {color_profile.width()}x{color_profile.height()} and resizing."
                )
            elif used_fps != self.fps:
                color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
                print(
                    f"[Image Server] RealSense {self.serial_number} does not support "
                    f"{target_w}x{target_h} @ {self.fps}fps, using {used_fps}fps "
                    f"({color_profile.width()}x{color_profile.height()})."
                )

        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self._color_format = color_profile.format()

        self._device = profile.get_device()
        if self._device is None:
            print('[Image Server] pipe_profile.get_device() is None .')
        if self.enable_depth:
            assert self._device is not None
            depth_sensor = self._device.first_depth_sensor()
            self.g_depth_scale = depth_sensor.get_depth_scale()

        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        if self.align is not None:
            frames = self.align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame() if self.enable_depth else None

        if not color_frame:
            return None, None

        color_image = np.asanyarray(color_frame.get_data())
        if self._color_format == rs.format.rgb8:
            color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        depth_image = np.asanyarray(depth_frame.get_data()) if depth_frame else None

        if self.no_resize:
            if color_image.shape[0] != self.img_shape[0] or color_image.shape[1] != self.img_shape[1]:
                raise RuntimeError(
                    f"[Image Server] Color frame {color_image.shape[1]}x{color_image.shape[0]} "
                    f"!= configured {self.img_shape[1]}x{self.img_shape[0]}"
                )
            if depth_image is not None and self.depth_shape is not None:
                if depth_image.shape[0] != self.depth_shape[0] or depth_image.shape[1] != self.depth_shape[1]:
                    raise RuntimeError(
                        f"[Image Server] Depth frame {depth_image.shape[1]}x{depth_image.shape[0]} "
                        f"!= configured {self.depth_shape[1]}x{self.depth_shape[0]}"
                    )
            return color_image, depth_image

        target_h, target_w = self.img_shape[0], self.img_shape[1]
        if color_image.shape[0] != target_h or color_image.shape[1] != target_w:
            color_image = cv2.resize(color_image, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        if depth_image is not None:
            depth_h, depth_w = (
                (self.depth_shape[0], self.depth_shape[1])
                if self.depth_shape is not None else (target_h, target_w)
            )
            if depth_image.shape[0] != depth_h or depth_image.shape[1] != depth_w:
                depth_image = cv2.resize(depth_image, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST)

        return color_image, depth_image

    def release(self):
        self.pipeline.stop()

class OpenCVCamera():
    def __init__(self, device_id, img_shape, fps):
        """
        decive_id: /dev/video* or *
        img_shape: [height, width]
        """
        self.id = device_id
        self.fps = fps
        self.img_shape = img_shape
        self.cap = cv2.VideoCapture(self.id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_shape[0])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.img_shape[1])
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Test if the camera can read frames
        if not self._can_read_frame():
            print(f"[Image Server] Camera {self.id} Error: Failed to initialize the camera or read frames. Exiting...")
            self.release()

    def _can_read_frame(self):
        success, _ = self.cap.read()
        return success

    def release(self):
        self.cap.release()

    def get_frame(self):
        ret, color_image = self.cap.read()
        if not ret:
            return None
        
        color_image = cv2.resize(color_image, (self.img_shape[1], self.img_shape[0]), interpolation = cv2.INTER_LINEAR)
        return color_image

def _require_sapien(feature="SAPIEN camera"):
    if not SAPIEN_AVAILABLE:
        raise ImportError(
            f"{feature} requires the 'sapien' package, which is not installed. "
            "Install it with: pip install sapien==3.0.0"
        )

class SapienCamera():
    def __init__(self, sapien_scene, sapien_camera):
        _require_sapien()
        self.scene = sapien_scene
        self.sapien_camera = sapien_camera
        self.img_shape = [self.sapien_camera.height, self.sapien_camera.width]

    def release(self):
        pass
    
    def get_frame(self):
        self.scene.update_render()
        self.sapien_camera.take_picture()
        camera_rgba = self.sapien_camera.get_picture("Color")
        color_image = (camera_rgba * 255).clip(0, 255).astype("uint8")[:, :, :3]
        return color_image
    
class ImageServer:
    def __init__(self, config, port = 5555, Unit_Test = False, sapien_scene=None, local_image_server=False):
        """
        config example1:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'opencv',                                     # opencv or realsense
            'head_camera_image_shape': [480, 1280],                           # Head camera resolution  [height, width]
            'head_camera_id_numbers': [0],                                    # '/dev/video0' (opencv)
            'wrist_camera_type': 'realsense', 
            'wrist_camera_image_shape': [480, 640],                           # Wrist camera resolution  [height, width]
            'wrist_camera_id_numbers': ["218622271789", "241222076627"],      # realsense camera's serial number
        }

        config example2:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'realsense',                                  # opencv or realsense or sapien_camera
            'head_camera_image_shape': [480, 640],                            # Head camera resolution  [height, width]
            'head_camera_id_numbers': ["218622271739"],                       # realsense camera's serial number
            'wrist_camera_type': 'opencv', 
            'wrist_camera_image_shape': [480, 640],                           # Wrist camera resolution  [height, width]
            'wrist_camera_id_numbers': [0,1],                                 # '/dev/video0' and '/dev/video1' (opencv)
        }
        
        config example4:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'realsense',                                  # opencv or realsense or sapien_camera
            'head_camera_image_shape': [480, 640],                            # Head camera resolution  [height, width]
            'head_camera_id_numbers': ["117222250634"],                       # realsense camera's serial number
            'wrist_camera_type': 'realsense', 
            'wrist_camera_image_shape': [480, 640],                           # Wrist camera resolution  [height, width]
            'wrist_camera_id_numbers': [],                                    # '/dev/video0' and '/dev/video1' (opencv)
        }
        
        config example3:
        {
            'fps':30                                                          # frame per second, depends on sapien rendering fps if type == sapien_camera
            'head_camera_type': 'sapien_camera',                              # opencv or realsense or sapien_camera
            'head_camera_image_shape': [480, 1280],                           # Head camera resolution  [height, width]
            'head_camera_id_numbers': [],                                     # camera name
            'wrist_camera_type': 'sapien_camera', 
            'wrist_camera_image_shape': [480, 640],                           # Wrist camera resolution  [height, width]
            'wrist_camera_id_numbers': [],                                    # camera name
        }

        If you are not using the wrist camera, you can comment out its configuration, like this below:
        config:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'opencv',                                     # opencv or realsense
            'head_camera_image_shape': [480, 1280],                           # Head camera resolution  [height, width]
            'head_camera_id_numbers': [0],                                    # '/dev/video0' (opencv)
            #'wrist_camera_type': 'realsense', 
            #'wrist_camera_image_shape': [480, 640],                           # Wrist camera resolution  [height, width]
            #'wrist_camera_id_numbers': ["218622271789", "241222076627"],      # serial number (realsense)
        }
        """
        print(config)
        self.fps = config.get('fps', 30)
        self.teleop_use_head_camera = config.get('teleop_use_head_camera', True)
        self.head_camera_type = config.get('head_camera_type', 'opencv')
        self.head_image_shape = config.get('head_camera_image_shape', [480, 640])      # (height, width)
        self.head_depth_shape = config.get('head_camera_depth_shape', None)      # (height, width)
        self.head_camera_id_numbers = config.get('head_camera_id_numbers', [0])
        self.head_enable_depth = config.get('head_camera_enable_depth', False)
        self.head_no_resize = config.get('head_camera_no_resize', False)

        self.wrist_camera_type = config.get('wrist_camera_type', None)
        self.wrist_image_shape = config.get('wrist_camera_image_shape', [480, 640])    # (height, width)
        self.wrist_camera_id_numbers = config.get('wrist_camera_id_numbers', None)

        self.agent_camera_type = config.get('agent_camera_type', None)
        self.agent_image_shape = config.get('agent_camera_image_shape', [480, 640])    # (height, width)
        self.agent_camera_id_numbers = config.get('agent_camera_id_numbers', None)
        
        self.port = port
        self.Unit_Test = Unit_Test
        
        # Initialize head cameras
        self.head_cameras = []
        if self.head_camera_type == 'opencv':
            for device_id in self.head_camera_id_numbers:
                camera = OpenCVCamera(device_id=device_id, img_shape=self.head_image_shape, fps=self.fps)
                self.head_cameras.append(camera)
        elif self.head_camera_type == 'realsense':
            for serial_number in self.head_camera_id_numbers:
                camera = RealSenseCamera(
                    img_shape=self.head_image_shape,
                    fps=self.fps,
                    serial_number=serial_number,
                    enable_depth=self.head_enable_depth,
                    depth_shape=self.head_depth_shape if self.head_enable_depth else None,
                    no_resize=self.head_no_resize,
                )
                self.head_cameras.append(camera)
        elif self.head_camera_type == 'kinect_azure':
            for serial_number in self.head_camera_id_numbers:
                camera = KinectAzureCamera(img_shape=self.head_image_shape, fps=self.fps)
                self.head_cameras.append(camera)
        elif self.head_camera_type == 'zed':
            for serial_number in self.head_camera_id_numbers:
                camera = ZedCamera(img_shape=self.head_image_shape, fps=self.fps, depth_shape=self.head_depth_shape)
                self.head_cameras.append(camera)
        elif self.head_camera_type == 'sapien_camera':
            _require_sapien("head_camera_type='sapien_camera'")
            if sapien_scene is not None:
                for cam in sapien_scene.get_cameras():
                    if cam.name in self.head_camera_id_numbers:
                        self.head_cameras.append(SapienCamera(sapien_scene, cam))
        else:
            print(f"[Image Server] Unsupported head_camera_type: {self.head_camera_type}")

        # Initialize teleop cameras(case for teleop and head cameras are different)
        if not self.teleop_use_head_camera:
            self.teleop_camera_type = config.get('teleop_camera_type', 'zed')
            self.teleop_image_shape = config.get('teleop_camera_image_shape', [480, 1280])
            self.teleop_camera_id_numbers = config.get('teleop_camera_id_numbers', [0])
            # Initialize teleop cameras 
            self.teleop_cameras = []
            if self.teleop_camera_type == 'opencv':
                for device_id in self.teleop_camera_id_numbers:
                    camera = OpenCVCamera(device_id=device_id, img_shape=self.teleop_image_shape, fps=self.fps)
                    self.teleop_cameras.append(camera)
            elif self.teleop_camera_type == 'realsense':
                for serial_number in self.teleop_camera_id_numbers:
                    camera = RealSenseCamera(img_shape=self.teleop_image_shape, fps=self.fps, serial_number=serial_number)
                    self.teleop_cameras.append(camera)
            elif self.teleop_camera_type == 'kinect_azure':
                for serial_number in self.teleop_camera_id_numbers:
                    camera = KinectAzureCamera(img_shape=self.teleop_image_shape, fps=self.fps)
                    self.teleop_cameras.append(camera)
            elif self.teleop_camera_type == 'zed':
                for serial_number in self.teleop_camera_id_numbers:
                    camera = ZedCamera(img_shape=self.teleop_image_shape, fps=self.fps)
                    self.teleop_cameras.append(camera)
            elif self.teleop_camera_type == 'sapien_camera':
                _require_sapien("teleop_camera_type='sapien_camera'")
                if sapien_scene is not None:
                    for cam in sapien_scene.get_cameras():
                        if cam.name in self.teleop_camera_id_numbers:
                            self.teleop_cameras.append(SapienCamera(sapien_scene, cam))
            else:
                print(f"[Image Server] Unsupported teleop_camera_type: {self.teleop_camera_type}")
        else:
            self.teleop_camera_type = self.head_camera_type
            self.teleop_image_shape = self.head_image_shape
            self.teleop_camera_id_numbers = self.head_camera_id_numbers
            self.teleop_cameras = self.head_cameras
        
        # Initialize wrist cameras if provided
        self.wrist_cameras = []
        if self.wrist_camera_type and self.wrist_camera_id_numbers:
            if self.wrist_camera_type == 'opencv':
                for device_id in self.wrist_camera_id_numbers:
                    camera = OpenCVCamera(device_id=device_id, img_shape=self.wrist_image_shape, fps=self.fps)
                    self.wrist_cameras.append(camera)
            elif self.wrist_camera_type == 'realsense':
                for serial_number in self.wrist_camera_id_numbers:
                    camera = RealSenseCamera(img_shape=self.wrist_image_shape, fps=self.fps, serial_number=serial_number)
                    self.wrist_cameras.append(camera)
            elif self.wrist_camera_type == 'kinect_azure':
                for serial_number in self.wrist_camera_id_numbers:
                    camera = KinectAzureCamera(img_shape=self.wrist_image_shape, fps=self.fps)
                    self.wrist_cameras.append(camera)
            elif self.wrist_camera_type == 'zed':
                for serial_number in self.wrist_camera_id_numbers:
                    camera = ZedCamera(img_shape=self.wrist_image_shape, fps=self.fps, enable_depth=True)
                    self.wrist_cameras.append(camera)
            elif self.wrist_camera_type == 'sapien_camera':
                _require_sapien("wrist_camera_type='sapien_camera'")
                if sapien_scene is not None:
                    for cam in sapien_scene.get_cameras():
                        if cam.name in self.wrist_camera_id_numbers:
                            self.wrist_cameras.append(SapienCamera(sapien_scene, cam))
            else:
                print(f"[Image Server] Unsupported wrist_camera_type: {self.wrist_camera_type}")

        # initialize agent cameras if provided
        self.agent_cameras = []
        if self.agent_camera_type and self.agent_camera_id_numbers:
            if self.agent_camera_type == 'opencv':
                for device_id in self.agent_camera_id_numbers:
                    camera = OpenCVCamera(device_id=device_id, img_shape=self.agent_image_shape, fps=self.fps)
                    self.agent_cameras.append(camera)
            elif self.agent_camera_type == 'realsense':
                for serial_number in self.agent_camera_id_numbers:
                    camera = RealSenseCamera(img_shape=self.agent_image_shape, fps=self.fps, serial_number=serial_number)
                    self.agent_cameras.append(camera)
            elif self.agent_camera_type == 'kinect_azure':
                for serial_number in self.agent_camera_id_numbers:
                    camera = KinectAzureCamera(img_shape=self.agent_image_shape, fps=self.fps)
                    self.agent_cameras.append(camera)
            elif self.agent_camera_type == 'zed':
                for serial_number in self.agent_camera_id_numbers:
                    camera = ZedCamera(img_shape=self.agent_image_shape, fps=self.fps, enable_depth=True)
                    self.agent_cameras.append(camera)
            elif self.agent_camera_type == 'sapien_camera':
                _require_sapien("agent_camera_type='sapien_camera'")
                if sapien_scene is not None:
                    for cam in sapien_scene.get_cameras():
                        if cam.name in self.agent_camera_id_numbers:
                            self.agent_cameras.append(SapienCamera(sapien_scene, cam))
            else:
                print(f"[Image Server] Unsupported agent_camera_type: {self.agent_camera_type}")
        
        if not local_image_server:
            # Set ZeroMQ context and socket
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.PUB)
            self.socket.bind(f"tcp://*:{self.port}")

        if self.Unit_Test:
            self._init_performance_metrics()

        for cam in self.head_cameras:
            if isinstance(cam, OpenCVCamera):
                print(f"[Image Server] Head camera {cam.id} resolution: {cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} x {cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
            elif isinstance(cam, RealSenseCamera):
                print(f"[Image Server] Head camera {cam.serial_number} resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, KinectAzureCamera):
                print(f"[Image Server] Head camera(Kinect Azure) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, ZedCamera):
                print(f"[Image Server] Head camera(ZED) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, SapienCamera):
                print(f"[Image Server] Head camera(Sapien) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            else:
                print("[Image Server] Unknown camera type in head_cameras.")
                
        for cam in self.teleop_cameras:
            if isinstance(cam, OpenCVCamera):
                print(f"[Image Server] Teleop camera {cam.id} resolution: {cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} x {cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
            elif isinstance(cam, RealSenseCamera):
                print(f"[Image Server] Teleop camera {cam.serial_number} resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, KinectAzureCamera):
                print(f"[Image Server] Teleop camera(Kinect Azure) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, ZedCamera):
                print(f"[Image Server] Teleop camera(ZED) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, SapienCamera):
                print(f"[Image Server] Teleop camera(Sapien) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            else:
                print("[Image Server] Unknown camera type in teleop_cameras.")

        for cam in self.wrist_cameras:
            if isinstance(cam, OpenCVCamera):
                print(f"[Image Server] Wrist camera {cam.id} resolution: {cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} x {cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
            elif isinstance(cam, RealSenseCamera):
                print(f"[Image Server] Wrist camera {cam.serial_number} resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, KinectAzureCamera):
                print(f"[Image Server] Wrist camera(Kinect Azure) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, ZedCamera):
                print(f"[Image Server] Wrist camera(ZED) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, SapienCamera):
                print(f"[Image Server] Wrist camera(Sapien) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            else:
                print("[Image Server] Unknown camera type in wrist_cameras.")
        
        for cam in self.agent_cameras:
            if isinstance(cam, OpenCVCamera):
                print(f"[Image Server] Agent camera {cam.id} resolution: {cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} x {cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
            elif isinstance(cam, RealSenseCamera):
                print(f"[Image Server] Agent camera {cam.serial_number} resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, KinectAzureCamera):
                print(f"[Image Server] Agent camera(Kinect Azure) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, ZedCamera):
                print(f"[Image Server] Agent camera(ZED) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            elif isinstance(cam, SapienCamera):
                print(f"[Image Server] Agent camera(Sapien) resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            else:
                print("[Image Server] Unknown camera type in agent_cameras.")

        print("[Image Server] Image server has started, waiting for client connections...")

    def _init_performance_metrics(self):
        self.frame_count = 0  # Total frames sent
        self.time_window = 1.0  # Time window for FPS calculation (in seconds)
        self.frame_times = deque()  # Timestamps of frames sent within the time window
        self.start_time = time.time()  # Start time of the streaming

    def _update_performance_metrics(self, current_time):
        # Add current time to frame times deque
        self.frame_times.append(current_time)
        # Remove timestamps outside the time window
        while self.frame_times and self.frame_times[0] < current_time - self.time_window:
            self.frame_times.popleft()
        # Increment frame count
        self.frame_count += 1

    def _print_performance_metrics(self, current_time):
        if self.frame_count % 30 == 0:
            elapsed_time = current_time - self.start_time
            real_time_fps = len(self.frame_times) / self.time_window
            print(f"[Image Server] Real-time FPS: {real_time_fps:.2f}, Total frames sent: {self.frame_count}, Elapsed time: {elapsed_time:.2f} sec")

    def _close(self):
        for cam in self.head_cameras:
            cam.release()
        for cam in self.wrist_cameras:
            cam.release()
        if hasattr(self, "socket"):
            self.socket.close()
        if hasattr(self, "context"):
            self.context.term()
        print("[Image Server] The server has been closed.")

    def set_image_shm(self, head_img_shape=None, head_image_shm: SharedMemoryManager=None, 
                            head_depth_shape=None, head_depth_shm: SharedMemoryManager=None,
                            tv_img_shape=None, tv_image_shm: SharedMemoryManager=None,
                            wrist_img_shape=None, wrist_image_shm: SharedMemoryManager=None,
                            agent_img_shape=None, agent_image_shm: SharedMemoryManager=None):
        """
        Set the shared memory for image server.
        This is used for unit testing.
        """
        self.head_image_shape = head_img_shape
        self.head_depth_shape = head_depth_shape
        self.wrist_image_shape = wrist_img_shape
        self.agent_image_shape = agent_img_shape
        self.tv_image_shape = tv_img_shape

        self.head_enable_shm = False
        if self.head_image_shape is not None:
            if head_image_shm is not None:
                self.head_image_shm = head_image_shm
                input_size = np.prod(self.head_image_shape) * np.uint8().itemsize
                assert input_size == self.head_image_shm.get_size(), "input tv img shape must be equal to that of given tv image shm !!!"
            else:
                self.head_image_shm = shared_memory.SharedMemory(shape=self.head_image_shape)
            self.head_enable_shm = True
            
        self.head_depth_enable_shm = False
        if head_depth_shm is not None:
            self.head_depth_shm = head_depth_shm
            input_size = np.prod(self.head_depth_shape) * np.float32().itemsize
            assert input_size == self.head_depth_shm.get_size(), "input tv depth shape must be equal to that of given tv depth shm !!!"
            self.head_depth_enable_shm = True
        
        self.tv_enable_shm = False
        if not self.teleop_use_head_camera and self.tv_image_shape is not None:
            if tv_image_shm is not None:
                self.tv_image_shm = tv_image_shm
                input_size = np.prod(self.tv_image_shape) * np.uint8().itemsize
                assert input_size == self.tv_image_shm.get_size(), "input tv img shape must be equal to that of given tv image shm !!!"
            else:
                self.tv_image_shm = shared_memory.SharedMemory(shape=self.tv_image_shape)
            self.tv_enable_shm = True
        else:
            self.tv_image_shm = self.head_image_shm
            self.tv_enable_shm = True
        
        self.wrist_enable_shm = False
        if self.wrist_image_shape is not None:
            if wrist_image_shm is not None:
                self.wrist_image_shm = wrist_image_shm
                input_size = np.prod(self.wrist_image_shape) * np.uint8().itemsize
                assert input_size == self.wrist_image_shm.get_size(), "input wrist img shape must be equal to that of given wrist image shm !!!"
            else:
                self.wrist_image_shm = shared_memory.SharedMemory(shape=self.wrist_image_shape)
            self.wrist_enable_shm = True
        
        self.agent_enable_shm = False
        if self.agent_image_shape is not None:
            if agent_image_shm is not None:
                self.agent_image_shm = agent_image_shm
                input_size = np.prod(self.agent_image_shape) * np.uint8().itemsize
                assert input_size == self.agent_image_shm.get_size(), "input agent img shape must be equal to that of given agent image shm !!!"
            else:
                self.agent_image_shm = shared_memory.SharedMemory(shape=self.agent_image_shape)
            self.agent_enable_shm = True
    
    def image_stream_process(self):
        try:
            while True:
                head_frames = []
                head_depths = []
                for cam in self.head_cameras:
                    depth_iamge = None
                    if self.head_camera_type == 'opencv':
                        color_image = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                    elif self.head_camera_type == 'realsense':
                        color_image, depth_iamge = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                    elif self.head_camera_type == 'kinect_azure':
                        color_image, depth_iamge = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                    elif self.head_camera_type == 'zed':
                        color_image, depth_iamge = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                    elif self.head_camera_type == 'sapien_camera':
                        color_image = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                    head_frames.append(color_image)
                    if depth_iamge is not None:
                        head_depths.append(depth_iamge)
                if len(head_frames) != len(self.head_cameras):
                    continue
                
                head_color = cv2.hconcat(head_frames)
                head_depth = cv2.hconcat(head_depths) if head_depths else None
                if self.head_enable_shm:
                    assert head_color.shape == self.head_image_shape, "[Image Server] Head camera image shape mismatch."
                    self.head_image_shm.write_numpy(np.array(head_color))
                
                if self.head_depth_enable_shm and head_depth is not None:
                    assert head_depth.shape == self.head_depth_shape, "[Image Server] Head camera depth shape mismatch."
                    self.head_depth_shm.write_numpy(np.array(head_depth))
                
                if not self.teleop_use_head_camera and self.teleop_cameras and self.tv_enable_shm:
                    tv_frames = []
                    for cam in self.teleop_cameras:
                        if self.teleop_camera_type == 'opencv':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        elif self.teleop_camera_type == 'realsense':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        elif self.teleop_camera_type == 'kinect_azure':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        elif self.teleop_camera_type == 'zed':
                            color_image, _ = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        elif self.teleop_camera_type == 'sapien_camera':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        tv_frames.append(color_image)
                    if len(tv_frames) != len(self.teleop_cameras):
                        continue
                    tv_color = cv2.hconcat(tv_frames)
                    assert tv_color.shape == self.tv_image_shape, "[Image Server] Teleop camera image shape mismatch."
                    self.tv_image_shm.write_numpy(np.array(tv_color))
                
                if self.wrist_cameras and self.wrist_enable_shm:
                    wrist_frames = []
                    for cam in self.wrist_cameras:
                        if self.wrist_camera_type == 'opencv':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        elif self.wrist_camera_type == 'realsense':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        elif self.wrist_camera_type == 'kinect_azure':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        elif self.wrist_camera_type == 'zed':
                            color_image, _ = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        elif self.wrist_camera_type == 'sapien_camera':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        wrist_frames.append(color_image)
                    if len(wrist_frames) != len(self.wrist_cameras):
                        continue
                    wrist_color = cv2.hconcat(wrist_frames)
                    assert wrist_color.shape == self.wrist_image_shape, "[Image Server] Wrist camera image shape mismatch."
                    self.wrist_image_shm.write_numpy(np.array(wrist_color))

                if self.agent_cameras and self.agent_enable_shm:
                    agent_frames = []
                    for cam in self.agent_cameras:
                        if self.agent_camera_type == 'opencv':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        elif self.agent_camera_type == 'realsense':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        elif self.agent_camera_type == 'kinect_azure':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        elif self.agent_camera_type == 'zed':
                            color_image, _ = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        elif self.agent_camera_type == 'sapien_camera':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        agent_frames.append(color_image)
                    if len(agent_frames) != len(self.agent_cameras):
                        continue
                    agent_color = cv2.hconcat(agent_frames)
                    assert agent_color.shape == self.agent_image_shape, "[Image Server] agent camera image shape mismatch."
                    self.agent_image_shm.write_numpy(np.array(agent_color))
                
                if self.Unit_Test:
                    current_time = time.time()
                    self._update_performance_metrics(current_time)
                    self._print_performance_metrics(current_time)

        except KeyboardInterrupt:
            print("[Image Server] Interrupted by user.")
        finally:
            self._close()
    
    def send_process(self):
        try:
            while True:
                head_frames = []
                head_depths = []
                for cam in self.head_cameras:
                    if self.head_camera_type == 'opencv':
                        color_image = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                    elif self.head_camera_type == 'realsense':
                        color_image, depth_image = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                        if depth_image is not None:
                            head_depths.append(depth_image)
                    elif self.head_camera_type == 'kinect_azure':
                        color_image, depth_image = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                        if depth_image is not None:
                            head_depths.append(depth_image)
                    elif self.head_camera_type == 'zed':
                        color_image, _ = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                    elif self.head_camera_type == 'sapien_camera':
                        color_image = cam.get_frame()
                        if color_image is None:
                            print("[Image Server] Head camera frame read is error.")
                            break
                    else:
                        continue
                    head_frames.append(color_image)
                if len(head_frames) != len(self.head_cameras):
                    break
                
                full_color = cv2.hconcat(head_frames)
                
                if not self.teleop_use_head_camera and self.teleop_cameras:
                    tv_frames = []
                    for cam in self.teleop_cameras:
                        if self.teleop_camera_type == 'opencv':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        elif self.teleop_camera_type == 'realsense':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        elif self.teleop_camera_type == 'kinect_azure':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        elif self.teleop_camera_type == 'zed':
                            color_image, _ = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        elif self.teleop_camera_type == 'sapien_camera':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Teleop camera frame read is error.")
                                break
                        tv_frames.append(color_image)
                    if len(tv_frames) != len(self.teleop_cameras):
                        break
                    tv_color = cv2.hconcat(tv_frames)
                    # Concatenate head and teleop frames
                    full_color = cv2.hconcat([full_color, tv_color])
                
                if self.wrist_cameras:
                    wrist_frames = []
                    for cam in self.wrist_cameras:
                        if self.wrist_camera_type == 'opencv':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        elif self.wrist_camera_type == 'realsense':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        elif self.wrist_camera_type == 'kinect_azure':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        elif self.wrist_camera_type == 'zed':
                            color_image, _ = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        elif self.wrist_camera_type == 'sapien_camera':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] Wrist camera frame read is error.")
                                break
                        wrist_frames.append(color_image)
                    wrist_color = cv2.hconcat(wrist_frames)

                    # Concatenate head and wrist frames
                    full_color = cv2.hconcat([full_color, wrist_color])
                
                if self.agent_cameras:
                    agent_frames = []
                    for cam in self.agent_cameras:
                        if self.agent_camera_type == 'opencv':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        elif self.agent_camera_type == 'realsense':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        elif self.agent_camera_type == 'kinect_azure':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        elif self.agent_camera_type == 'zed':
                            color_image, _ = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        elif self.agent_camera_type == 'sapien_camera':
                            color_image = cam.get_frame()
                            if color_image is None:
                                print("[Image Server] agent camera frame read is error.")
                                break
                        agent_frames.append(color_image)
                    agent_color = cv2.hconcat(agent_frames)

                    # Concatenate head and wrist and agent frames
                    full_color = cv2.hconcat([full_color, agent_color])

                ret, buffer = cv2.imencode('.jpg', full_color)
                if not ret:
                    print("[Image Server] Frame imencode is failed.")
                    continue

                jpg_bytes = buffer.tobytes()
                depth_jpg_bytes = None
                if head_depths:
                    full_depth = cv2.hconcat(head_depths)
                    # 发送 uint16 原始深度 PNG（Z16 * depth_scale = 米），勿发 JET 伪彩色
                    ret_depth, depth_buffer = cv2.imencode(".png", full_depth)
                    if ret_depth:
                        depth_jpg_bytes = depth_buffer.tobytes()

                if self.Unit_Test:
                    timestamp = time.time()
                    frame_id = self.frame_count
                    header = struct.pack('dI', timestamp, frame_id)  # 8-byte double, 4-byte unsigned int
                    payload = pack_frame(jpg_bytes, depth_jpg_bytes)
                    message = header + payload
                elif depth_jpg_bytes is not None:
                    message = pack_frame(jpg_bytes, depth_jpg_bytes)
                else:
                    message = jpg_bytes

                self.socket.send(message)

                if self.Unit_Test:
                    current_time = time.time()
                    self._update_performance_metrics(current_time)
                    self._print_performance_metrics(current_time)

        except KeyboardInterrupt:
            print("[Image Server] Interrupted by user.")
        finally:
            self._close()

def TestRealSenseCamera():
    config = {
        'fps':25,                                                         # frame per second
        'head_camera_type': 'realsense',                                  # opencv or realsense or sapien_camera
        'head_camera_image_shape': [480, 640],                            # Head camera resolution  [height, width]
        'head_camera_id_numbers': ["254622078165"],                       # realsense camera's serial number
    }
    
    server = ImageServer(config, Unit_Test=False)
    server.send_process()
    
def TestKinectAzureCamera():
    img_config = {
        'fps': 30,
        'head_camera_type': 'kinect_azure',  # 'kinect_azure' or 'realsense' or 'opencv' or 'sapien camera'
        'head_camera_image_shape': [480, 720],  # Head camera resolution
        'head_camera_id_numbers': [0],
    }
    
    server = ImageServer(img_config, Unit_Test=False)
    server.send_process()

def TestZEDCamera():
    img_config = {
        'fps':30,                                                           # frame per second
        'head_camera_type': 'zed',                                          # opencv or realsense or sapien_camera, zed
        'head_camera_image_shape': [2560, 720],                              # Head camera resolution  [height, width]
        'head_camera_id_numbers': [0],                                      # realsense camera's serial number
    }
    
    server = ImageServer(img_config, Unit_Test=False)
    server.send_process()

def TestOpenCVCamera():
    img_config = {
        'fps':30,                                                           # frame per second
        'head_camera_type': 'opencv',                                          # opencv or realsense or sapien_camera, zed
        'head_camera_image_shape': [480, 640],                              # Head camera resolution  [height, width]
        'head_camera_id_numbers': [3],                                      # realsense camera's serial number
    }
    
    server = ImageServer(img_config, Unit_Test=False)
    server.send_process()

def TestSapienCamera():
    engine = sapien.Engine()
    from sapien.render import set_global_config

    set_global_config(max_num_materials=50000, max_num_textures=50000)
    renderer = sapien.SapienRenderer()
    # give renderer to sapien sim
    engine.set_renderer(renderer)
    sapien.render.set_camera_shader_dir("rt")
    sapien.render.set_ray_tracing_samples_per_pixel(32)
    sapien.render.set_ray_tracing_path_depth(8)
    sapien.render.set_ray_tracing_denoiser("oidn")

    # declare sapien scene
    scene_config = sapien.SceneConfig()
    scene = engine.create_scene(scene_config)
    # set simulation timestep
    scene.set_timestep(1 / 250)
    # add ground to scene
    scene.add_ground(0)
    # set default physical material
    scene.default_physical_material = scene.create_physical_material(
        0.5,
        0.5,
        0,
    )
    scene.set_ambient_light([0.5, 0.5, 0.5])
    # default enable shadow unless specified otherwise
    shadow = True
    # default spotlight angle and intensity
    direction_lights = [[[0, 0.5, -1], [0.5, 0.5, 0.5]]]
    for direction_light in direction_lights:
        scene.add_directional_light(
            direction_light[0], direction_light[1], shadow=shadow
        )
    # default point lights position and intensity
    point_lights = [[[1, 0, 1.8], [1, 1, 1]], [[-1, 0, 1.8], [1, 1, 1]]]
    for point_light in point_lights:
        scene.add_point_light(point_light[0], point_light[1], shadow=shadow)

    # camera in world 
    head_height = 480
    head_width = 720
    wrist_height = 480
    wrist_width = 720
    
    head_camera = scene.add_camera(
        name="head_camera",
        width=head_width,
        height=head_height,
        fovy=np.deg2rad(60),
        near=0.1,
        far=100,
    )
    
    wrist_camera = scene.add_camera(
        name="wrist_camera",
        width=wrist_width,
        height=wrist_height,
        fovy=np.deg2rad(60),
        near=0.1,
        far=100,
    )
    
    head_camera.entity.set_pose(sapien.Pose([-0.6, 0, 1.6], [1, 0, 0, 0]))
    wrist_camera.entity.set_pose(sapien.Pose([0.6, 0, 1.6], [1, 0, 0, 0]))
    scene.step()  # run a physical step
    scene.update_render()  # sync pose from SAPIEN to renderer
    
    # start img server
    config = {
        'fps': 30,
        'head_camera_type': 'sapien_camera',
        'head_camera_image_shape': [head_height, head_width],  # Head camera resolution
        'head_camera_id_numbers': [head_camera.name],
        'wrist_camera_type': 'sapien_camera',
        'wrist_camera_image_shape': [wrist_height, wrist_width],  # Wrist camera resolution
        'wrist_camera_id_numbers': [wrist_camera.name],
    }
        
    img_server = ImageServer(config, Unit_Test=False, sapien_scene=scene)
    image_send_thread = threading.Thread(target = img_server.send_process, daemon = True)
    image_send_thread.daemon = True
    image_send_thread.start()
    
    # sapien 
    while True:
        scene.step()  

def list_realsense_devices():
    devices = []
    for device in rs.context().query_devices():
        devices.append({
            "name": device.get_info(rs.camera_info.name),
            "serial": device.get_info(rs.camera_info.serial_number),
            "product_line": device.get_info(rs.camera_info.product_line),
        })
    return devices


def resolve_serial(serial, devices=None):
    """Resolve user-provided serial: exact match, then suffix/prefix partial match."""
    if devices is None:
        devices = list_realsense_devices()
    serial = str(serial).strip().lower()
    for dev in devices:
        full = dev["serial"].lower()
        if full == serial:
            return dev["serial"]
    partial = [dev for dev in devices if dev["serial"].lower().endswith(serial) or serial.endswith(dev["serial"].lower())]
    if len(partial) == 1:
        print(f"序列号 '{serial}' 匹配到设备 {partial[0]['name']} [{partial[0]['serial']}]")
        return partial[0]["serial"]
    if len(partial) > 1:
        names = ", ".join(f"{d['serial']}" for d in partial)
        raise SystemExit(f"序列号 '{serial}' 匹配到多台设备: {names}，请指定完整序列号")
    return None


def build_realsense_config(
    serial,
    width=1280,
    height=720,
    fps=30,
    enable_depth=False,
    depth_width=640,
    depth_height=480,
    no_resize=False,
):
    """Build ImageServer config for a single RealSense camera."""
    config = {
        "fps": fps,
        "head_camera_type": "realsense",
        "head_camera_image_shape": [height, width],
        "head_camera_id_numbers": [serial],
    }
    if enable_depth:
        config["head_camera_enable_depth"] = True
        config["head_camera_depth_shape"] = [depth_height, depth_width]
    if no_resize:
        config["head_camera_no_resize"] = True
    return config


def parse_args():
    import argparse

    parser = argparse.ArgumentParser(description="RealSense Image Server (standalone)")
    parser.add_argument("--serial", default=None, help="RealSense serial number")
    parser.add_argument("--width", type=int, default=1280, help="RGB width (default: 1280)")
    parser.add_argument("--height", type=int, default=720, help="RGB height (default: 720)")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate (default: 30)")
    parser.add_argument("--depth", action="store_true", help="Enable depth stream")
    parser.add_argument("--depth-width", type=int, default=640, help="Depth width (default: 640)")
    parser.add_argument("--depth-height", type=int, default=480, help="Depth height (default: 480)")
    parser.add_argument("--no-resize", action="store_true", help="Keep native frame sizes without cv2.resize")
    parser.add_argument("--port", type=int, default=5555, help="ZMQ publish port (default: 5555)")
    parser.add_argument("--unit-test", action="store_true", help="Enable latency/frame metrics in stream")
    parser.add_argument("--list-devices", action="store_true", help="List connected RealSense devices and exit")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    if args.list_devices:
        devices = list_realsense_devices()
        if not devices:
            print("未发现 RealSense 设备")
        for dev in devices:
            print(f"  {dev['name']} | serial={dev['serial']} | product_line={dev['product_line']}")
        raise SystemExit(0)

    devices = list_realsense_devices()
    if not devices:
        print("未发现 RealSense 设备")
        raise SystemExit(1)

    if args.serial:
        serial = resolve_serial(args.serial, devices)
        if serial is None:
            print(f"未找到序列号为 {args.serial} 的设备")
            print("当前已连接设备:")
            for dev in devices:
                print(f"  {dev['name']} | serial={dev['serial']} | product_line={dev['product_line']}")
            raise SystemExit(1)
    else:
        serial = devices[0]["serial"]
    product_line = next((d["product_line"] for d in devices if d["serial"] == serial), None)

    if product_line == "L500":
        print(
            f"检测到 L515 [{serial}]，RGB {args.width}x{args.height}"
            + (f"，Depth {args.depth_width}x{args.depth_height}" if args.depth else "")
            + f" @ {args.fps}fps"
        )
        if args.depth and not args.no_resize:
            args.no_resize = True
            print("[Image Server] L515 + depth: 自动启用 --no-resize (原生分辨率，RGB/Depth 分开)")

    config = build_realsense_config(
        serial, args.width, args.height, args.fps,
        enable_depth=args.depth,
        depth_width=args.depth_width,
        depth_height=args.depth_height,
        no_resize=args.no_resize,
    )
    server = ImageServer(config, port=args.port, Unit_Test=args.unit_test)
    server.send_process()