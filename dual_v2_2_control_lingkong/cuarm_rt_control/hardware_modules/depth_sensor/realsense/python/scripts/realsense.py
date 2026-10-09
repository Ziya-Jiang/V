import numpy as np
import pyrealsense2 as rs

from scripts.utils import apply_color_to_depth_image, depth_to_point_cloud

class Realsense:
    #Tested on Realsense D455 sensor
    #Unit in mm
    def __init__(self, clipping_distance_mm=None):
        self.setup_realsense(clipping_distance_mm)

    def setup_realsense(self, clipping_distance_mm):
        # Create a pipeline
        pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == "RGB Camera":
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        if device_product_line == "L500":
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_m meters away
        clipping_distance = clipping_distance_mm / depth_scale if clipping_distance_mm else 1

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)

        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        intrinsics = color_profile.get_intrinsics()

        self.pipeline = pipeline
        self.align = align
        self.clipping_distance = clipping_distance
        self.depth_scale = depth_scale
        self.intrinsics = intrinsics
        self.update()
    
    def update(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # Align the depth frame to color frame
        self.aligned_frames = self.align.process(frames)
        self.depth_image = None
    
    def get_depth_image(self):
        # Get aligned frames
        aligned_depth_frame = self.aligned_frames.get_depth_frame()
        if not aligned_depth_frame:
            return None
        depth_image = np.asanyarray(aligned_depth_frame.get_data())*1000.0 #convert to mm
        depth_image[depth_image > self.clipping_distance] = 0.0
        depth_image = depth_image.astype(np.float32) * self.depth_scale
        self.depth_image = depth_image
        return depth_image

    def get_color_image(self, filter_color=False):
        # Get aligned frames
        aligned_color_frame = self.aligned_frames.get_color_frame()
        if not aligned_color_frame:
            return None
        color_image = np.asanyarray(aligned_color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        if filter_color and self.depth_image is not None:
            depth_image = self.depth_image
            depth_image_3d = np.dstack(
                (depth_image, depth_image, depth_image)
            )  # depth image is 1 channel, color is 3 channels

            color_image = np.where(
                (depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0),
                grey_color,
                color_image,
            )
        return color_image
    
    def get_colored_depth_image(self):
        depth_image = self.depth_image if self.depth_image is not None else self.get_depth_image()
        if depth_image is None:
            return None
        return apply_color_to_depth_image(depth_image)
        
    def get_point_cloud(self, intrinsics=None):
        intrinsics = intrinsics if intrinsics is not None else self.get_intrinsics_matrix()
        depth_image = self.depth_image if self.depth_image is not None else self.get_depth_image()
        if depth_image is None: return None
        return depth_to_point_cloud(depth_image, intrinsics)

    def get_intrinsics_matrix(self):
        intrinsics = self.intrinsics
        intrinsic_matrix = np.array(
            [
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1],
            ]
        )
        return intrinsic_matrix
