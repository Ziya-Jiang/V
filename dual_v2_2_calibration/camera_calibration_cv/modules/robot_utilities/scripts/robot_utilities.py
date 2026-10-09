import json
from enum import Enum

import numpy
from scipy.spatial.transform import Rotation as ScipyR

from .robot import Robot
from .mdh_robot import MDHRobot

class RotationType(Enum):
    RPY = 0
    AXIS_ANGLE = 1
    ROTATION_MATRIX = 2
    QUATERNION_SCALAR_LAST = 2

def read_json_file(path):
    with open(path, 'r') as f:
        data = json.load(f)
    return data

def read_joint_cmd_file(path, isRad=False):
    """
    Return joint commands from a csv file
    Args:
        path: path to csv
        isRad: True if joint angle unit in the csv is in Radian
    Returns:
        numpy array of joint command
    """
    Q = []
    with open(path, 'r', encoding='utf-8-sig') as f:
        for line in f.readlines():
            q = list(map(float, line.split(',')))
            if not isRad: q = numpy.array(q)*numpy.pi/180
            Q.append(q)
    return numpy.array(Q)

def read_cartesian_cmd_file(path):
    """
    Return cartesian command from a csv file
    Args:
        path: path to csv
    Returns:
        numpy array of joint command
    """
    cmds = []
    with open(path, 'r', encoding='utf-8-sig') as f:
        for line in f.read().splitlines():
            cmd = numpy.array(list(map(float, line.split(','))))
            cmds.append(cmd)
    return numpy.array(cmds)

def convert_joint_position_to_cartesian(Q, robot: Robot, rot_type:RotationType=RotationType.QUATERNION_SCALAR_LAST, transpose=False):
    """
    Return cartesian pose from given joint position array
    Args:
        Q: joint position array
        robot: robot object
        rot_type: output rotation representation, support quaternion(x,y,z,w) and axis_angle
        transpose: True if rotation matrix need transpose before convertion
    Returns:
        numpy array of cartersian pose.
    """
    cmds = []
    rot = None
    for q in Q:
            T = robot.FK(q)
            R = T[:3, :3] if not transpose else T[:3, :3].transpose()
            if rot_type == RotationType.QUATERNION_SCALAR_LAST:
                rot = ScipyR.from_matrix(R).as_quat() #(x,y,z,w)
            elif rot_type == RotationType.AXIS_ANGLE:
                rot = ScipyR.from_matrix(R).as_rotvec()
            cmds.append(numpy.concatenate([T[0:3, 3],rot]))
    return numpy.array(cmds)

def load_MDH_robot(config_file:str='config/cuarm_mdh.json'):
    config = read_json_file(config_file)
    joint_size = config['robot']['joint_size']
    joint_type = numpy.array(config['robot']['joint_type'])
    # Modified DH parameters
    a          = numpy.array(config['robot']['DH_a'],dtype=numpy.float64)
    alpha      = numpy.array(config['robot']['DH_alpha'],dtype=numpy.float64)*numpy.pi/180
    d_base     = numpy.array(config['robot']['DH_d_base'],dtype=numpy.float64)
    q_base     = numpy.array(config['robot']['DH_q_base'],dtype=numpy.float64)*numpy.pi/180
    
    robot = MDHRobot(joint_size, joint_type, a, alpha, d_base, q_base)
    robot.JOINT_LIMIT_L = numpy.array(config['robot']['joint_limit_L'],dtype=numpy.float64) * numpy.pi/180 
    robot.JOINT_LIMIT_R = numpy.array(config['robot']['joint_limit_R'],dtype=numpy.float64) * numpy.pi/180
    return robot