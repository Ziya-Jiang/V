import time
import math
import functools
import numpy as np
from abc import ABC, abstractmethod

class BaseArm():
    def __init__(self, config_file=None):
        pass
    
    @abstractmethod
    def reset(self):
        raise NotImplementedError("Subclasses should implement this method")

    @abstractmethod
    def restore_config(self, config_file: str):
        raise NotImplementedError("Subclasses should implement this method")

    @abstractmethod
    def read_config(self, config_file: str):
        raise NotImplementedError("Subclasses should implement this method")

    @abstractmethod
    def moveto(self):
        raise NotImplementedError("Subclasses should implement this method")
    
    @abstractmethod
    def get_status(self):
        """
        Get the status of the robot arm
        :return: status of the robot arm
        """
        raise NotImplementedError("Subclasses should implement this method")
    
    @abstractmethod
    def act_gripper(self, status: int):
        """
        Activate the gripper
        :param status: 0 for loose, 1 for tight
        """
        raise NotImplementedError("Subclasses should implement this method")


