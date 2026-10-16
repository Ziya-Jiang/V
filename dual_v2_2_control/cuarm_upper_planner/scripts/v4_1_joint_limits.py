"""Execution constraints, applied before constructing CuRobo solvers.

The robot YAML uses degrees; CuRobo and panel joint commands use radians.
Never repair a solved Cartesian trajectory by independently clipping joints.
"""
import numpy as np


def execution_limits(config):
    result = {}
    for arm in config['robot']['arm']:
        names = arm['urdf_joints']
        n = arm['joint_size']
        if len(names) != n:
            raise ValueError('Execution joint names/size mismatch')
        safety = arm['safety']
        soft = safety['joint_soft_limit']
        hard = safety['joint_hard_limit']
        position = np.deg2rad(np.asarray(soft['position'], dtype=float))
        hard_position = np.deg2rad(np.asarray(hard['position'], dtype=float))
        if position.shape != (2, n) or hard_position.shape != (2, n):
            raise ValueError('Invalid execution position limit shape')
        position[0] = np.maximum(position[0], hard_position[0])
        position[1] = np.minimum(position[1], hard_position[1])
        # RT's per-cycle jump saturation also imposes speed/acceleration caps.
        velocity = np.minimum(soft['velocity'], hard['velocity'])
        velocity = np.minimum(velocity, safety['joint_jump']['position'])
        acceleration = np.minimum(soft['acceleration'], safety['joint_jump']['velocity'])
        velocity, acceleration = np.deg2rad(velocity), np.deg2rad(acceleration)
        if (velocity.shape != (n,) or acceleration.shape != (n,) or
                not np.isfinite(position).all() or not np.isfinite(velocity).all() or
                not np.isfinite(acceleration).all() or
                (position[0] >= position[1]).any() or
                (velocity <= 0).any() or (acceleration <= 0).any()):
            raise ValueError('Invalid execution joint limits')
        for i, name in enumerate(names):
            if name in result:
                raise ValueError('Duplicate execution joint: ' + name)
            result[name] = (position[:, i].copy(), velocity[i], acceleration[i])
    return result


def constrain_robot_config(robot, config, arm_names, soft_limit_k):
    """Update the tensor config shared by IK, graph, trajopt, FK and CPU servo."""
    if not np.isfinite(soft_limit_k) or not 0 < soft_limit_k <= 1:
        raise ValueError('soft_limit_k must be in (0, 1]')
    limits = robot.kinematics.get_joint_limits()
    names = list(limits.joint_names)
    if len(set(names)) != len(names) or len(set(arm_names)) != len(arm_names):
        raise ValueError('Duplicate model/arm joint names')
    execution = execution_limits(config)
    if set(execution) != set(arm_names) or not set(arm_names).issubset(names):
        raise ValueError('Execution/model arm joint names disagree')
    original = limits.position.detach().cpu().numpy().copy()
    position = original.copy()
    velocity = limits.velocity.detach().cpu().numpy().copy()
    acceleration = limits.acceleration.detach().cpu().numpy().copy()
    report = []
    for name in arm_names:
        i = names.index(name)
        rt_position, rt_velocity, rt_acceleration = execution[name]
        lower = max(original[0, i], original[0, i] * soft_limit_k, rt_position[0])
        upper = min(original[1, i], original[1, i] * soft_limit_k, rt_position[1])
        if not np.isfinite([lower, upper]).all() or lower >= upper:
            raise ValueError('Empty effective position range: ' + name)
        position[:, i] = [lower, upper]
        velocity[:, i] = [max(velocity[0, i], -rt_velocity), min(velocity[1, i], rt_velocity)]
        acceleration[:, i] = [max(acceleration[0, i], -rt_acceleration),
                              min(acceleration[1, i], rt_acceleration)]
        if velocity[0, i] >= 0 or velocity[1, i] <= 0 or acceleration[0, i] >= 0 or acceleration[1, i] <= 0:
            raise ValueError('Invalid effective dynamic limits: ' + name)
        report.append(dict(joint=name, model_deg=np.rad2deg(original[:, i]).tolist(),
                           model_velocity_rad_s=float(limits.velocity[1, i].item()),
                           model_acceleration_rad_s2=float(limits.acceleration[1, i].item()),
                           execution_deg=np.rad2deg(rt_position).tolist(),
                           effective_deg=np.rad2deg(position[:, i]).tolist(),
                           velocity_rad_s=float(velocity[1, i]),
                           acceleration_rad_s2=float(acceleration[1, i])))
    for field, values in [('position', position), ('velocity', velocity), ('acceleration', acceleration)]:
        tensor = getattr(limits, field)
        tensor.copy_(tensor.new_tensor(values))
    # Keep metadata coherent for consumers which clone/rebuild CSpaceConfig.
    robot.kinematics.cspace.max_acceleration.copy_(limits.acceleration[1])
    indices = [names.index(name) for name in arm_names]
    return original[:, indices], position[:, indices].copy(), report


def checked_joint_command(command, bounds, names, tolerance=1e-6):
    """Validate a waypoint/path without modifying it (epsilon is roundoff only)."""
    q = np.asarray(command, dtype=float)
    bounds = np.asarray(bounds, dtype=float)
    if q.ndim < 1 or q.size == 0 or q.shape[-1] != len(names) or bounds.shape != (2, len(names)):
        raise ValueError('Joint command/limit shape mismatch')
    if not np.isfinite(q).all():
        raise ValueError('Non-finite joint command')
    excess = np.maximum(bounds[0] - q, q - bounds[1])
    if np.max(excess) > tolerance:
        index = np.unravel_index(np.argmax(excess), excess.shape)
        j = index[-1]
        raise ValueError(f'{names[j]} command={np.rad2deg(q[index]):.6f}deg '
                         f'outside [{np.rad2deg(bounds[0,j]):.6f}, '
                         f'{np.rad2deg(bounds[1,j]):.6f}]deg; trajectory not clipped')
    return q.copy()
