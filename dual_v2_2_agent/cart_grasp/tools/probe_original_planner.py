#!/usr/bin/env python3
"""Bounded GPU feasibility probe using the original upper-planner factory.

Inputs include historical arm feedback, so results are never executable. No UDP
connection is created; BOTH_LOCAL mode keeps all commands inside this process.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from seldon_cart_grasp.upper_target import protocol, preview_command, outward_stage
from seldon_cart_grasp.handle_pair import detect_handle_pair
from seldon_cart_grasp.rgbd_bridge import blue_points
from seldon_cart_grasp.planning import ArmModel
from seldon_cart_grasp.frame_graph import FrameGraph


def cart_world(urdf, obs, half_T_upper):
    axis = np.asarray(obs['bar_axis'], dtype=float)
    axis /= np.linalg.norm(axis)
    normal = np.asarray(obs['approach_normal'], dtype=float)
    normal -= np.dot(axis, normal)*axis
    normal /= np.linalg.norm(normal)
    t = np.eye(4)
    t[:3, :3] = np.column_stack((np.cross(axis, normal), axis, normal))
    t[:3, 3] = np.mean(obs['handles'], axis=0)-normal*1.0575
    base = next(x for x in ET.parse(urdf).getroot().findall('link') if x.get('name') == 'cart_base')
    boxes = {}
    for collision in base.findall('collision'):
        geometry = collision.find('geometry')
        box, cylinder = geometry.find('box'), geometry.find('cylinder')
        if box is not None:
            dimensions = np.fromstring(box.get('size'), sep=' ')
        elif cylinder is not None:
            dimensions = np.array([2*float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))])
        else:
            raise ValueError('Unsupported measured-cart primitive')
        pose = half_T_upper @ t @ ArmModel.origin(collision)
        quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
        boxes['cart_'+collision.get('name')] = {
            'dims': (dimensions+.04).tolist(),  # 2 cm inflated on each side
            'pose': np.r_[pose[:3, 3], quat[3], quat[:3]].tolist()}
    return {'cuboid': boxes}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--feedback', type=Path, required=True)
    ap.add_argument('--rgbd', type=Path, required=True)
    ap.add_argument('--cart-urdf', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    start = time.monotonic()
    report = {'hardware_executable': False, 'packets_sent': 0,
              'input_arm_state': 'historical_recording_not_live',
              'environment_coverage': 'inflated measured cart only; lower body and unknown obstacles not certified'}
    planner = None
    try:
        import torch
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        torch.cuda.set_per_process_memory_fraction(.08, 0)
        v = Path('/mnt/ssd/user/V')
        config_root = v/'dual_v2_2_configuration'
        os.environ['CUARM_CONFIG_ROOT'] = str(config_root)
        path = v/'dual_v2_2_control/cuarm_upper_planner/dual_v2_2_upper_planner.py'
        spec = importlib.util.spec_from_file_location('original_upper_entry', path)
        upper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(upper)
        config = upper.CuroboConfig.load_from_yaml_path(str(config_root/'config.yaml'))
        with np.load(args.rgbd, allow_pickle=False) as f:
            data = {key: f[key] for key in f.files}
        obs = detect_handle_pair(blue_points(data), float(data['stamp_s']))
        half = FrameGraph(config_root/'share/left_hand_right_gripper_half/dual_arm_half.urdf')
        config.world_cfg = cart_world(args.cart_urdf, obs, half.relative('dummy_world', 'SJ4Z'))
        snapshot = json.loads(args.feedback.read_text())
        if snapshot.get('coordinate_system') != 'V2 calibrated':
            raise ValueError('Recorded feedback is not V2 calibrated')
        q = np.array([a['position'] for a in snapshot['state']['arms']])
        limits = np.array([a['soft_position'] for a in snapshot['state']['arms']])
        report.update(cart_observation=obs, world_object_count=len(config.world_cfg['cuboid']), q_start=q.tolist())
        constructor = upper.CuroboPlanner
        def limited_constructor(*positional, **kw):
            kw.update(connection_method=upper.PlannerConnectionMethod.BOTH_LOCAL,
                      num_ik_seeds=1, num_trajopt_seeds=1, ik_particle_opt=False,
                      use_cuda_graph=False, skip_warmup=True)
            return constructor(*positional, **kw)
        upper.CuroboPlanner = limited_constructor
        print('Creating original planner in BOTH_LOCAL mode; no hardware sockets', flush=True)
        planner = upper.create_curobo_planner(config, upper.PlannerCuroboSolverType.MOTION_GEN)
        planner._plan_config.check_start_validity = True
        planner._plan_config.max_attempts = 1
        planner._plan_config.timeout = 8.
        types, _ = protocol(config_root)
        robot = types.RobotState()
        robot.arm_size, robot.arm_joint_size = 2, [7, 7]
        robot.arm_joint_position, robot.arm_joint_velocity = q.tolist(), np.zeros((2, 7)).tolist()
        robot.gripper_size, robot.gripper_joint_size = 2, [6, 1]
        robot.gripper_joint_position = [[1., 0., 0., 0., 0., 0.], [0.]]
        # Hypothetical open-hand geometry for offline screening, not feedback.
        report['gripper_state'] = 'assumed_open_for_offline_probe_only'
        positions, orientations = planner.compute_fk(q.reshape(-1))
        state = {'arm_size': 2, 'arm_joint_size': [7, 7], 'arm_joint_position': q.tolist(),
                 'arm_joint_velocity': np.zeros((2, 7)).tolist(),
                 'arm_tool_pose': np.concatenate([positions, orientations], axis=1).tolist(),
                 'gripper_size': 2, 'gripper_joint_size': [6, 1],
                 'gripper_joint_position': robot.gripper_joint_position}
        stage = outward_stage(q, limits)
        target = preview_command(types, state, stage)
        print('Computing one outward preview against measured-cart geometry', flush=True)
        command, _ = planner.receive_and_compute(robot_state=robot, target_command=target)
        report.update(stage=stage, planner_status=str(planner.status),
                      waypoint_count=int(command.waypoints_size),
                      waypoints=np.asarray(command.waypoints).tolist(),
                      torch_peak_allocated_bytes=int(torch.cuda.max_memory_allocated()))
        if command.waypoints_size:
            trajectory = np.asarray(command.waypoints)
            report['all_returned_samples_within_recorded_soft_limits'] = bool(
                np.all(trajectory >= limits[:, :, 0]) and np.all(trajectory <= limits[:, :, 1]))
        print(json.dumps({k: report[k] for k in ('planner_status', 'waypoint_count', 'hardware_executable')}), flush=True)
    except Exception as error:
        report['error'] = str(error)
        print(type(error).__name__+': '+str(error), flush=True)
    finally:
        if planner is not None:
            planner.terminate()
        report['elapsed_s'] = time.monotonic()-start
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    return 1 if 'error' in report else 0


if __name__ == '__main__':
    raise SystemExit(main())
