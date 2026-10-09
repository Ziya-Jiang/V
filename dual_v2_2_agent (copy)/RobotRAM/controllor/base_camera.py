import numpy as np
from abc import ABC, abstractmethod


class BaseCamera:
    def __init__(self):
        """
        Initialize the camera
        """
        self.R = np.eye(3)  # rotation matrix [3, 3]
        self.T = np.zeros((3, 1))
        self.K = np.eye(3)
        self.D = np.zeros((1, 5))
        self.w = 1280
        self.h = 720
        self.undistort = False
        self.max_depth = 2000  # mm
        self.type = "external"

        self.rgb_image = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.depth_map = np.zeros((self.h, self.w), dtype=np.float32)

    @abstractmethod
    def restore_camparas(self, config_file):
        """
        Restore the camera parameters
        :return: None
        """
        pass

    @abstractmethod
    def restore_config(self, config_file):
        """
        Restore the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        pass

    @abstractmethod
    def read_config(self, config_file):
        """
        Read the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        pass

    @abstractmethod
    def capture_image(self):
        """
        Capture an image from the camera
        :return: RGB image and depth image
        """
        RGB_image = self.rgb_image
        depth_map = self.depth_map
        return RGB_image, depth_map
    
    @property
    def w2c(self):
        c2w = np.eye(4)
        c2w[:3, :3] = self.R
        c2w[:3, 3] = self.T.reshape(3, )
        c2w[3, 3] = 1.0
        w2c = np.linalg.inv(c2w)
        return w2c

    @property
    def c2w(self):
        c2w = np.eye(4)
        c2w[:3, :3] = self.R
        c2w[:3, 3] = self.T.reshape(3, )
        c2w[3, 3] = 1.0
        return c2w
    
    @property
    def fx(self):
        return self.K[0, 0]
    
    @property
    def fy(self):
        return self.K[1, 1]
    
    @property
    def cx(self):
        return self.K[0, 2]
    
    @property
    def cy(self):
        return self.K[1, 2]

    @property
    def width(self):
        return self.w
    
    @property
    def height(self):
        return self.h
    
    
        