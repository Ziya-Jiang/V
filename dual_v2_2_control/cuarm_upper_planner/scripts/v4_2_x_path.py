"""Scoped right-link X objective and actual TCP path gate for V4.2 only.

The local CuRobo primary EE is LEFT. Its public MotionGen pose metric would
constrain the wrong arm; use the installed rollout's right-link cost instead.
This soft objective is not a certificate: tool offsets and panel interpolation
are checked separately. No sockets or robot commands in this module.
"""
from contextlib import contextmanager
import numpy as np

ACTION = 'book_v42_raise_servo'
PLANNED_X_MM = 1.5  # Remaining 1.5 mm of the client's 3 mm is tracking reserve.


@contextmanager
def right_x_objective(motion_gen, description):
    """Caller holds GPU lock. Restore tensor contents even if planning raises.

Uses local CuRobo link-cost internals; fail closed if the installed API differs.
Do not change endpoint weights, left arm costs, or joint/collision constraints.
"""
    if description != ACTION:
        yield
        return
    costs = []
    other_costs = []
    for rollout in motion_gen.get_all_pose_rollout_instances():
        other_costs.extend(c for name, c in getattr(rollout, '_link_pose_costs', {}).items()
                           if name != 'arm_end_effector_r')
        if getattr(rollout, 'goal_cost', None) is not None:
            other_costs.append(rollout.goal_cost)
        cost = getattr(rollout, '_link_pose_costs', {}).get('arm_end_effector_r')
        if cost is not None and all(cost is not x for x in costs):
            costs.append(cost)
    if not costs:
        raise ValueError('V4.2 right-link X objective unavailable in installed CuRobo')
    # Snapshot all before writing: some rollouts share cost tensor storage.
    saved = [(c, c.run_vec_weight.clone(), c.project_distance_tensor.clone()) for c in costs]
    if any(c.run_vec_weight.data_ptr() == other.run_vec_weight.data_ptr()
           for c in costs for other in other_costs):
        raise ValueError('V4.2 right-link cost aliases another link; cannot isolate X objective')
    try:
        for cost in costs:
            weight, project = cost.run_vec_weight, cost.project_distance_tensor
            if tuple(weight.shape) != (6,):
                raise ValueError('V4.2 unexpected right-link cost shape')
            # [rotation XYZ, translation XYZ], base frame (not goal frame).
            weight.zero_(); weight[3] = 1.
            project.zero_()
        yield
    finally:
        for cost, weight, project in reversed(saved):
            cost.run_vec_weight.copy_(weight)
            cost.project_distance_tensor.copy_(project)


def x_report(positions, right, reference_m, *, source, delay=0.):
    p = np.asarray(positions, float)
    if (p.ndim != 3 or p.shape[2] != 3 or len(p) == 0 or
            not 0 <= right < p.shape[1] or not np.isfinite(p).all() or
            not np.isfinite(reference_m)):
        raise ValueError('Invalid V4.2 TCP path')
    delta = (p[:, right, 0]-reference_m)*1000.
    i = int(np.argmax(np.abs(delta)))
    return dict(source=source, delay_s=delay, samples=len(p), reference_x_mm=reference_m*1000.,
                peak_x_mm=float(p[i, right, 0]*1000.), signed_deviation_mm=float(delta[i]),
                max_deviation_mm=float(abs(delta[i])), planned_limit_mm=PLANNED_X_MM,
                tracking_reserve_mm=1.5, accepted=bool(abs(delta[i]) <= PLANNED_X_MM))


def validate_path(bridge, dense, points, interval, right, reference_m, log):
    """Inspect dense CuRobo path plus actual RT WAYPOINT interpolation.

The publisher still checks planning-start joints at send time. Here reject
feedback movement during validation, without a 40 ms wall-time deadline that
would repeatedly expire solely because FK/validation took time.
"""
    before = bridge.snapshot()
    dense = np.concatenate((before['joints'][None, :], np.asarray(dense), np.asarray(points)))
    report = x_report(bridge.tcp(dense)[0], right, reference_m, source='curobo')
    log(report)
    if not report['accepted']:
        return False
    for delay in (0., .02, .04):
        start = before['joints'] + delay*before['velocity']
        arms = []
        for arm in range(2):
            path, _ = bridge.panel_path.sample(start[arm*7:arm*7+7],
                points[:, arm*7:arm*7+7], interval, bridge.limits[arm])
            arms.append(path)
        n = max(map(len, arms))
        joined = np.concatenate([np.pad(a, ((0, n-len(a)), (0, 0)), mode='edge')
                                 for a in arms], axis=1)
        report = x_report(bridge.tcp(joined)[0], right, reference_m, source='panel', delay=delay)
        log(report)
        if not report['accepted']:
            return False
    after = bridge.snapshot()
    joint_delta = float(np.max(np.abs(after['joints']-before['joints'])))
    tcp_delta = float(np.max(np.linalg.norm(after['position']-before['position'], axis=1)))*1000.
    accepted = joint_delta <= .002 and tcp_delta <= .25
    log(dict(source='validation_start', joint_delta_rad=joint_delta,
             tcp_delta_mm=tcp_delta, accepted=accepted))
    return accepted
