"""V3.1 panel-path validation and coherent planning-start snapshots."""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from v3_1_execution import PLANNED_Z_MM, HANDOFF_TCP_MM, HANDOFF_HORIZON_S, handoff_matches


class SnapshotUnavailable(ValueError):
    """A transient feedback gap: defer a target, never reuse its old start."""


class MotionBridge:
    def __init__(self, planner, cpu, panel_path):
        self.p = planner
        self.cpu = cpu
        self.panel_path = panel_path
        self.guard = None
        self.checked_start = None
        self.serial = 0
        self.requested_target = None
        self.duration = None
        self.failure = None
        self.limits = np.deg2rad([a['safety']['joint_soft_limit']['velocity']
                                 for a in planner.config['robot']['arm']])
        # Joint names/order are fixed. Resolve once instead of performing
        # thousands of Python name searches inside the 40ms handoff window.
        self.arm_order = np.asarray(planner.reorder_input(np.arange(14), as_curobo=True), dtype=int)

    def tcp(self, joints):
        ordered = np.atleast_2d(joints)[:, self.arm_order]
        position, quaternion = self.cpu.poses(ordered, self.p.target_link_names)
        for i, offset in enumerate(self.p.end_effector_offset):
            position[:, i] += Rotation.from_quat(quaternion[:, i, [1,2,3,0]]).apply(offset)
        return position, quaternion

    def snapshot(self, max_age=.1):
        panel, stamp = self.p._panel_feedback_source()
        if panel is None or stamp is None:
            raise SnapshotUnavailable('V3.1 waiting for panel feedback')
        if not np.isfinite(stamp) or stamp > time.monotonic():
            raise ValueError('Invalid V3.1 panel timestamp')
        if time.monotonic()-stamp > max_age:
            raise SnapshotUnavailable('V3.1 waiting for fresh planning feedback')
        joints = np.asarray(panel.arm_joint_position, float).reshape(-1).copy()
        velocity = np.asarray(panel.arm_joint_velocity, float).reshape(-1).copy()
        if joints.shape != (14,) or velocity.shape != (14,) or not np.isfinite([joints,velocity]).all():
            raise ValueError('Invalid V3.1 planning feedback')
        position, orientation = self.tcp(joints)
        return dict(joints=joints, velocity=velocity, position=position[0],
                    orientation=orientation[0], stamp=float(stamp), panel=panel)

    def validate(self, points, dt):
        """Validate the actual panel curve at current and 20/40ms projected starts.

        The projected starts cover nominal send delay, not an unbounded network
        delay. The independent monitor remains active throughout execution.
        """
        self.failure = 'handoff'
        began = time.monotonic()
        before = self.snapshot()
        future_position, _ = self.tcp(before['joints']+before['velocity']*HANDOFF_HORIZON_S)
        before['tcp_velocity']=(future_position[0,1]-before['position'][1])/HANDOFF_HORIZON_S
        if self.guard:
            self.guard.update(before)
            if self.guard.fault: raise ValueError(self.guard.fault)
        paths = []
        durations = []
        for delay in (0., .02, .04):
            start = before['joints'] + delay * before['velocity']
            arms = []
            for arm in range(2):
                path, duration = self.panel_path.sample(start[arm*7:arm*7+7],
                    points[:, arm*7:arm*7+7], dt, self.limits[arm])
                arms.append(path);durations.append(duration)
            n = max(map(len, arms))
            joined = np.concatenate([np.pad(a, ((0,n-len(a)),(0,0)), mode='edge') for a in arms], axis=1)
            position, _ = self.tcp(joined)
            z = self.guard.reference if self.guard else self.p.target_arm_tool_position[1,2]
            deviation = float(np.max(np.abs(position[:,1,2]-z)))*1000
            # Never allow a new correction to rewind through old X waypoints.
            x = position[:,1,0]
            retreat = float(np.max(np.maximum.accumulate(x)-x))*1000
            print(f'V3.1 panel path: delay={delay:.3f}s, z_deviation={deviation:.3f}mm, '
                  f'planned_limit={PLANNED_Z_MM:.1f}mm, x_retreat={retreat:.3f}mm, samples={n}')
            if deviation > PLANNED_Z_MM:
                self.failure = 'height_exceeded'
                return False
            if retreat > HANDOFF_TCP_MM:
                return False
            paths.append(joined)
        # Check the reconstructed path, including its transition, for collision
        # and limits, rather than only checking the CuRobo path/end point.
        samples = np.concatenate(paths)
        grip = list(self.p.target_gripper_joint)
        order = np.asarray(self.p.reorder_input(np.arange(14), list(range(14,14+len(grip))), as_curobo=True), dtype=int)
        values = np.ascontiguousarray(np.concatenate(
            (samples, np.broadcast_to(grip, (len(samples),len(grip)))), axis=1)[:,order])
        with self.p._gpu_lock:
            for i in range(0,len(values),1024):
                metrics = self.p._motion_gen.rollout_fn.rollout_constraint(
                    self.p.tensor_args.to_device(values[i:i+1024]).unsqueeze(1), use_batch_env=False)
                if not bool(metrics.feasible.all().item()):
                    self.failure = 'collision'
                    return False
        after = self.snapshot()
        if not handoff_matches(before, after, time.monotonic()):
            print(f'V3.1 panel path validation start expired/inconsistent; '
                  f'validation_ms={(time.monotonic()-began)*1000:.1f}, '
                  f'start_age_ms={(time.monotonic()-before["stamp"])*1000:.1f}, '
                  f'limit_ms={HANDOFF_HORIZON_S*1000:.1f}; discard and replan from fresh feedback')
            return False
        self.checked_start = before
        self.duration = max(durations)
        self.serial += 1
        self.failure = None
        return True
