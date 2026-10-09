#!/usr/bin/env python3
from os.path import dirname, realpath
import json

data = {}
data['robot'] = {
    'joint_size'    : 7,
    'joint_type'    : [0, 0, 0, 0, 0, 0, 0, 0],
    # DH parameter
    'DH_a'          : [0, 0, 0, 0, 0, 0, 0], # in meter
    'DH_alpha'      : [0, -90, +90, -90, +90, -90, +90], # in degree
    # 'DH_d_base'     : [0.33, 0, 0.26, 0, 0.25, 0, 0.08],  # 1.0v (Orange case)
    # 'DH_d_base'     : [0.43, 0, 0.269, 0, 0.214, 0, 0.0555],  # 2.0v
    # 'DH_d_base'     : [0.373, 0, 0.297, 0, 0.264, 0, 0.0555],  # 2.1v
    'DH_d_base'     : [0.537, 0, 0.322, 0, 0.280, 0, 0.0945],  # 2.2v (White case)
    'DH_q_base'     : [0, 0, 0, 0, 0, 0, 0], # in degree
    'joint_limit_L' : [-150, -65.8, -200,   -74,-200, -114.5, -360], # in degree
    'joint_limit_R' : [ 150,   111,  200, 114.5, 200,  122  ,  360], # in degree
    # 'joint_L_limit' : [-75, -30, -100, -35,-100, -30, -180], # in degree
    # 'joint_R_limit' : [ 75,   70,  100, 70, 100, 70,  180], # in degree
}

with open('cuarm_mdh.json', 'w') as outfile:
    json.dump(data, outfile)
