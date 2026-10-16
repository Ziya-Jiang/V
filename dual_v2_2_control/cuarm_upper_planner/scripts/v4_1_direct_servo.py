"""V4.1 bounded XYZ servo. Independent of V3's fixed-height controller.

Reuses CPU kinematics, damped differential IK and collision geometry only.
All positions here are metres. A rolling target is limited to 3 mm ahead.
"""
import copy
import time
import numpy as np
from scipy.spatial.transform import Rotation
from v3_1_direct_servo import DifferentialStep, load_settings
from v3_1_execution import ExecutionGuard


def settings():
    c = load_settings()
    c.update(period_s=.02, linear_speed_m_s=.01, angular_speed_rad_s=.05,
             position_gain=2., rotation_gain=2., joint_speed_rad_s=.15,
             joint_acceleration_rad_s2=.5, joint_lead_rad=.01,
             stream_lease_s=.3, raw_velocity_ratio=.15)
    return c


def compute_closed_fingers(planner):
    """Pre-servo gripper-only command. Latch measured arms once; no IK/trajectory.

    Use the existing bounded PanelTransport publisher so STOP, stale feedback,
    producer timeout and the RT watchdog retain their normal semantics.
    """
    from cuarm_state import CuroboStatus, GripperMode
    p = planner
    try:
        if not getattr(p, '_v41_closed_dispatched', False):
            s = p._v31_bridge.snapshot(max_age=.1)
            if (np.linalg.norm(p.target_arm_tool_position-s['position'], axis=1).max() > .001):
                raise ValueError('Gripper-only command must preserve measured arm poses')
            if getattr(p.target_gripper_mode[1], 'value', p.target_gripper_mode[1]) != GripperMode.CLOSE.value:
                raise ValueError('V4.1 preparation only authorizes right CLOSE')
            now=time.monotonic()
            if not hasattr(p,'_v41_close_wait_started'):
                p._v41_close_wait_started=now
                p._v41_close_stable=[]
                p._v41_close_last_stamp=-float('inf')
                print('V4.1 close: waiting for stable arms; speed_limit=0.005rad/s, stable_seconds=0.2, timeout=2s',flush=True)
            speed=np.asarray(s['velocity'],float)
            if speed.shape!=(14,) or not np.isfinite(speed).all():
                raise ValueError('Invalid arm velocity for gripper preparation')
            maximum=float(np.max(abs(speed)));joint=int(np.argmax(abs(speed)))
            if s['stamp']>p._v41_close_last_stamp:
                p._v41_close_last_stamp=s['stamp']
                if maximum<=.005:p._v41_close_stable.append(s['stamp'])
                else:p._v41_close_stable=[]
            stable=p._v41_close_stable
            if len(stable)<3 or stable[-1]-stable[0]<.2:
                if now-p._v41_close_wait_started>=2.:
                    raise ValueError('Arms did not settle before close: '
                        f'max_speed={maximum:.6f}rad/s, arm={"left" if joint<7 else "right"}, '
                        f'joint=J{joint%7+1}, limit=0.005rad/s, stable_samples={len(stable)}')
                # Return immediately. The shared main publishes idle_hold and
                # services STOP/heartbeat while fresh feedback settles.
                p._hold_current_arm_command()
                p.status=CuroboStatus.PLANNING
                return copy.deepcopy(p.curobo_cmd),copy.deepcopy(p.curobo_state)
            # The shared transport installs this bound callback after creating
            # the planner. Do not open a second panel sender or change RT.
            transport = p._panel_begin_planning.__self__
            with transport.lock:
                panel = transport._eligible()
                command = transport._hold(panel)
                p.set_gripper_command()
                command.gripper_joint_command[1] = copy.deepcopy(p.curobo_cmd.gripper_joint_command[1])
                transport.send(command)
            p._v41_closed_dispatched = True
        p._hold_current_arm_command()
        p._prev_target_tool_position = p.target_arm_tool_position.copy()
        p._prev_target_tool_orientation = p.target_arm_tool_orientation.copy()
        p._prev_target_gripper_mode = copy.deepcopy(p.target_gripper_mode)
        p.status = CuroboStatus.PLANNING_SUCCEEDED
    except Exception as exc:
        p._v31_bridge.guard.trip(f'V4.1 gripper preparation rejected: {exc}')
        p.status = CuroboStatus.PLANNING_FAILED
        p._hold_current_arm_command()
    return copy.deepcopy(p.curobo_cmd), copy.deepcopy(p.curobo_state)


def compute_perception_pause(planner):
    """Private pause dispatch; shared transport publishes a latched idle hold."""
    from cuarm_state import CuroboStatus
    try:
        bridge = planner._v31_bridge
        bridge.guard.pause_visual(bridge.snapshot(max_age=.1))
        planner._v31_direct.reset()
        planner._hold_current_arm_command()
        # ACK only this pause command; no ordinary planning/cache dispatch.
        planner._prev_target_tool_position = planner.target_arm_tool_position.copy()
        planner._prev_target_tool_orientation = planner.target_arm_tool_orientation.copy()
        planner.status = CuroboStatus.PLANNING_SUCCEEDED
    except Exception as exc:
        planner._v31_bridge.guard.trip(f'V4.1 pause rejected: {exc}')
        planner.status = CuroboStatus.PLANNING_FAILED
        planner._hold_current_arm_command()
    return copy.deepcopy(planner.curobo_cmd), copy.deepcopy(planner.curobo_state)


class TiltExecutionGuard(ExecutionGuard):
    """Independent measured envelope + immutable orientation, no fixed Z."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.origin = None
        self.rotation = None
        self.left_joints = None
        self.last_target_time = None
        self.paused_at = None
        self.pause_command = None

    def reset(self):
        super().reset()
        self.origin = self.rotation = self.left_joints = self.last_target_time = None
        self.paused_at = self.pause_command = None

    def pause_visual(self, snapshot):
        """Explicit perception pause, not a fresh motion lease or fault reset."""
        with self.lock:
            if self.fault:
                raise ValueError(self.fault)
            self.begin_xyz(snapshot)
            if self.paused_at is None:
                self.paused_at = self.clock()
                self.last_target_time = None
                # The shared main enters PanelTransport.idle_hold once and
                # repeats that latched joint target. Do not create a competing
                # guard sender that would relatch measured joints every tick.
            self.update(snapshot)

    def resume_visual(self, snapshot):
        with self.lock:
            self.update(snapshot)
            if self.fault:
                raise ValueError(self.fault)
            if self.paused_at is not None and np.max(abs(snapshot['velocity'])) > .005:
                raise ValueError('Resume V4.1 only after stationary perception recovery')
            self.paused_at = self.pause_command = None

    def begin_xyz(self, snapshot):
        with self.lock:
            if self.fault:
                raise ValueError(self.fault)
            if self.origin is None:
                self.origin = snapshot['position'][1].copy()
                self.rotation = Rotation.from_quat(snapshot['orientation'][1, [1, 2, 3, 0]])
                self.left_joints = snapshot['joints'][:7].copy()
                self.reference = float(self.origin[2])  # active-session marker, not a fixed-Z plane
                self.snapshot = snapshot

    def geometry_ok(self, positions, orientations):
        p = np.asarray(positions)
        r = Rotation.from_quat(np.asarray(orientations)[..., [1, 2, 3, 0]])
        angle = (r*self.rotation.inv()).magnitude()
        # Session envelope accommodates upper edge tilting toward -X and down;
        # the client additionally constrains its actual book/pad geometry.
        delta = p-self.origin
        return bool(np.isfinite(delta).all() and
                    np.all(abs(delta) <= np.array([.25, .04, .20])) and
                    np.all(angle <= np.deg2rad(1.)))

    def update(self, snapshot):
        with self.lock:
            if snapshot is not None:
                self.snapshot = snapshot
            if self.origin is None:
                return
            now = self.clock()
            s = self.snapshot
            if s is None or not 0 <= now-s['stamp'] <= .25:
                self.trip('V4.1 independent monitor: stale feedback')
            elif not self.geometry_ok(s['position'][1], s['orientation'][1]):
                self.trip('V4.1 independent XYZ/orientation envelope exceeded')
            elif np.max(abs(s['joints'][:7]-self.left_joints)) > .005:
                self.trip('V4.1 independent monitor: left arm changed')
            if self.last_target_time is not None and now-self.last_target_time > 1.:
                self.trip('V4.1 visual stream absent for 1 second')
            if self.paused_at is not None:
                if now-self.paused_at > 75.:
                    self.trip('V4.1 perception pause exceeded 75 seconds')
            if self.fault and self.hold_command is not None and now-self.last_hold >= .05:
                self.send(self.hold_command)
                self.last_hold = now


class TiltDirectServo:
    def __init__(self, planner, collision):
        self.p, self.collision = planner, collision
        self.c = settings()
        self.stepper = DifferentialStep(planner._v31_bridge.tcp, planner.arm_soft_joint_limits, self.c)
        self.active = self.output_ready = self.paused = False
        self.recovering_z = False  # legacy ACK encoder compatibility; no Z recovery mode
        self.last_tick = None
        self.request = None

    def reset(self):
        self.active = self.output_ready = self.paused = False
        self.last_tick = None
        self.stepper.reset()

    def configure_panel(self, command):
        command.interpolation_acceleration_time = 0.
        command.no_interpolation_max_velocity_ratio = self.c['raw_velocity_ratio']

    def compute(self, receipt):
        from cuarm_state import CuroboStatus
        p, now = self.p, time.monotonic()
        b = p._v31_bridge
        g = b.guard
        self.output_ready = False
        if not isinstance(g, TiltExecutionGuard):
            raise ValueError('V4.1 requires its own XYZ guard')
        s = b.snapshot(max_age=.1)
        if not self.active:
            if np.max(abs(s['velocity'])) > .01:
                raise ValueError('Begin V4.1 session at a stationary observation pose')
            g.begin_xyz(s)
            g.resume_visual(s)
            self.active = True
            self.last_tick = now
            self.stepper.reset()
            print('V4.1 CPU XYZ differential servo: target 50 Hz, visual lease 0.3 s; MotionGen bypassed', flush=True)
        if receipt is None or not np.isfinite(receipt) or not 0 <= now-receipt:
            raise ValueError('Missing/invalid V4.1 target timestamp')
        g.last_target_time = receipt
        g.update(s)
        if g.fault:
            raise ValueError(g.fault)
        dt = now-self.last_tick
        if dt > .1:
            raise ValueError('V4.1 differential servo deadline missed')
        dt = np.clip(dt, .001, .04)
        self.last_tick = now
        target = np.c_[p.target_arm_tool_position, p.target_arm_tool_orientation].copy()
        if target.shape != (2, 7) or not np.isfinite(target).all():
            raise ValueError('Invalid V4.1 target')
        if not g.geometry_ok(target[1, :3], target[1, 3:]):
            raise ValueError('V4.1 target outside XYZ/orientation envelope')
        if np.linalg.norm(target[0, :3]-s['position'][0]) > .001:
            raise ValueError('V4.1 cannot command the left arm')
        stale = now-receipt > self.c['stream_lease_s']
        if stale:
            if not self.paused:
                self.stepper.command = s['joints'].copy()
                self.stepper.velocity[:] = 0
                print('V4.1 new visual target expired; holding measured joints', flush=True)
            self.paused = True
            command = self.stepper.command.copy()
            velocity = np.zeros(7)
        else:
            if np.linalg.norm(target[1, :3]-s['position'][1]) > .003:
                raise ValueError('V4.1 measured rolling target lead exceeds 3 mm')
            if self.paused:
                self.stepper.reset()
            self.paused = False
            command, velocity, _ = self.stepper.step(s['joints'], target[1], dt)
        command[:7] = g.left_joints
        samples = np.linspace(s['joints'], command, 5)
        positions, orientations = b.tcp(samples)
        if not g.geometry_ok(positions[:, 1], orientations[:, 1]):
            raise ValueError('V4.1 predicted XYZ/orientation path outside envelope')
        if np.max(np.linalg.norm(positions[:, 1]-s['position'][1], axis=1)) > .003:
            raise ValueError('V4.1 predicted TCP lead exceeds 3 mm')
        grip = list(p.gripper_joint_position)
        ordered = np.asarray([p.reorder_input(q, grip, as_curobo=True) for q in samples])
        if not self.collision.check(ordered):
            raise ValueError('V4.1 CPU collision/joint-limit check failed')
        ahead, _ = b.tcp(s['joints']+s['velocity']*.04)
        s['tcp_velocity'] = (ahead[0, 1]-s['position'][1])/.04
        b.checked_start, b.requested_target = s, target
        b.serial += 1
        self.request, self.proposed, self.proposed_velocity = target, command, velocity
        p.curobo_cmd.arm_joint_command = command.reshape(2, 7).tolist()
        p.curobo_cmd.waypoints_size = 0
        p.curobo_cmd.waypoints = []
        p.curobo_cmd.cmd_plan_size = 0
        p.status = CuroboStatus.PLANNING_SUCCEEDED
        p.set_gripper_command()
        self.output_ready = True

    def dispatched(self, accepted):
        if accepted:
            self.stepper.command = self.proposed.copy()
            self.stepper.velocity = self.proposed_velocity.copy()
        else:
            self.stepper.reset()
