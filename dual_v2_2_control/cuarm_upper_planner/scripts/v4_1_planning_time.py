"""Keep trajectory duration and computation deadlines separate (seconds)."""
import math

PLANNING_HOLD_SECONDS = 10.0
SOLVER_TIMEOUT_SECONDS = 6.0
SOLVER_MAX_ATTEMPTS = 4


def trajectory_timing(report, steps, interpolation_dt, minimum_dt, maximum_time):
    """Scale the initial seed duration, never the robot's physical limits.

    For an unchanged geometric path, velocity scales as 1/T and acceleration
    as 1/T**2. This is an initialization heuristic, not a feasibility proof;
    CuRobo's finetune and trajectory validation still decide acceptance.
    """
    values = (steps, interpolation_dt, minimum_dt, maximum_time)
    if not all(math.isfinite(v) for v in values) or steps <= 5 or int(steps) != steps:
        raise ValueError('Invalid trajectory timing parameters')
    maximum_dt = maximum_time / steps
    if not 0 < interpolation_dt <= minimum_dt <= maximum_dt:
        raise ValueError('Require interpolation_dt <= minimum_dt <= maximum_dt')
    scale = 1.0
    for row in report:
        old_v, new_v = row['model_velocity_rad_s'], row['velocity_rad_s']
        old_a, new_a = row['model_acceleration_rad_s2'], row['acceleration_rad_s2']
        if not all(math.isfinite(v) and v > 0 for v in (old_v, new_v, old_a, new_a)):
            raise ValueError('Invalid dynamic limits for trajectory timing')
        scale = max(scale, old_v / new_v, math.sqrt(old_a / new_a))
    initial_dt = min(maximum_dt, max(minimum_dt, .15 * scale))
    return dict(minimum_trajectory_dt=float(minimum_dt),
                maximum_trajectory_dt=float(maximum_dt),
                trajopt_dt=initial_dt, js_trajopt_dt=initial_dt)
