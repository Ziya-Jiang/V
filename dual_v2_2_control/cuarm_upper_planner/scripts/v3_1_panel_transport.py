"""Bounded, independent Remote Panel publication; no CUDA or robot planning.

Planning sends one immutable measured-position hold, never the pending goal.
Normal/RAW commands have a short producer lease. A stuck planner, stale panel,
STOP, or transport error cannot be hidden by an endless background keepalive.
"""
import copy
import math
import threading
import time


class StalePlanStart(RuntimeError):
    """Only the computed trajectory is invalid; the fixed hold stays live."""


class PanelTransport:
    PERIOD = .05
    FEEDBACK_AGE = .25
    PRODUCER_LEASE = .25
    PLANNING_LEASE = 10.
    START_ERROR = .002
    RECOVERY_SECONDS = 10.
    MAX_START_REJECTIONS = 3
    SETTLE_SECONDS = .15
    SETTLE_ERROR = .0005

    def __init__(self, send, feedback, template, stop_state, hold_mode,
                 clock=time.monotonic, log=print):
        self._send = send
        self.feedback = feedback
        self.template = copy.deepcopy(template)
        self.stop_state, self.hold_mode = stop_state, hold_mode
        self.clock, self.log = clock, log
        self.lock = threading.RLock()
        self.command = None
        self.mode = 'idle'
        self.deadline = 0.
        self.fault = None
        self.epoch = 0
        self.cycle_epoch = 0
        self.running = False
        self.planning_joints = None
        self.panel_stamp = None
        self.recovery_started = None
        self.start_rejections = 0
        self.settle_sample = None
        self.last_sent = None
        self.last_gap = 0.
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name='v31-panel-publisher', daemon=True)
        self._thread.start()
        self.log('V3.1 panel publisher: 20Hz; planning=fixed measured hold; '
                 'producer lease=0.25s, planning deadline=10s; Upper watchdog unchanged (0.5s)')

    def _panel(self):
        panel, stamp = self.feedback()
        now = self.clock()
        fresh = (panel is not None and stamp is not None and math.isfinite(stamp)
                 and 0 <= now-stamp <= self.FEEDBACK_AGE)
        if not fresh:
            if self.running:
                self._fail('stale PanelState; command output disabled')
            return None
        if panel.state == self.stop_state:
            if self.running:
                self.epoch += 1
            self.running = False
            self.command = None
            self.planning_joints = None
            self._clear_recovery()
            self.mode = 'idle'
            return None
        self.running = True
        self.panel_stamp = stamp
        return panel

    def _check(self):
        if self._stop.is_set():
            raise RuntimeError('V3.1 panel publisher is closed')
        if self.fault:
            raise RuntimeError(self.fault)

    def begin_cycle(self):
        with self.lock:
            self._check()
            panel = self._panel()
            self._check()
            self.cycle_epoch = self.epoch
            return panel

    def _eligible(self):
        self._check()
        panel = self._panel()
        self._check()
        if panel is None or self.cycle_epoch != self.epoch:
            raise RuntimeError('V3.1 panel session changed during computation; discard result')
        if self.command is not None and self.clock() > self.deadline:
            self._fail('planner producer/deadline expired; discard late result', panel)
            self._check()
        return panel

    def _hold(self, panel):
        command = copy.deepcopy(self.command if self.command is not None else self.template)
        joints = [[float(v) for v in arm] for arm in panel.arm_joint_position]
        if (len(joints) != command.arm_size or
                any(len(q) != n for q, n in zip(joints, command.arm_joint_size)) or
                not all(math.isfinite(v) for q in joints for v in q)):
            raise ValueError('Invalid measured joints for panel hold')
        command.arm_target_mode = self.hold_mode
        # RAW visual packets use zero interpolation time. Do not carry that
        # value into a SINGLE_POINT hold interpolator.
        command.interpolation_acceleration_time = .2
        command.arm_joint_command = joints
        command.arm_waypoint_size = 0
        command.arm_waypoint_dt = .2
        command.arm_waypoint_command = []
        # Preserve the last dispatched gripper target. Before the first target,
        # use PanelState values in the same panel gripper coordinate convention.
        if self.command is None:
            command.gripper_joint_command = copy.deepcopy(panel.gripper_joint_position)
        return command

    def _clear_recovery(self):
        self.recovery_started = None
        self.start_rejections = 0
        self.settle_sample = None

    def ready_to_plan(self):
        """Bounded settling after a discarded start; requires new feedback time."""
        with self.lock:
            if self.recovery_started is None:
                return True
            panel = self._eligible()
            if self.clock()-self.recovery_started >= self.RECOVERY_SECONDS:
                self._fail('planning start recovery timed out; watchdog handoff', panel)
                self._check()
            stamp = self.panel_stamp
            # _eligible checked freshness; only advance settlement on a new sample.
            joints = copy.deepcopy(panel.arm_joint_position)
            if self.settle_sample is None:
                self.settle_sample = (stamp, joints)
                return False
            first_stamp, first_joints = self.settle_sample
            error = max(abs(float(a)-float(b)) for q, r in zip(joints, first_joints)
                        for a, b in zip(q, r))
            if error > self.SETTLE_ERROR:
                self.settle_sample = (stamp, joints)
                return False
            return stamp-first_stamp >= self.SETTLE_SECONDS

    def begin_planning(self, start_joints=None):
        with self.lock:
            panel = self._eligible()
            if self.recovery_started is None:
                self.command = self._hold(panel)
            # Check against the actual MotionGen input, not a later hold sample.
            self.planning_joints = copy.deepcopy(
                panel.arm_joint_position if start_joints is None else start_joints)
            self.mode = 'planning_hold'
            self.deadline = self.clock() + self.PLANNING_LEASE
            if self.recovery_started is not None:
                self.deadline = min(self.deadline, self.recovery_started+self.RECOVERY_SECONDS)
            self._emit()
            self.log('V3.1 planning hold: measured joints latched; pending goal not sent')

    def idle_hold(self, cancel_recovery=False):
        """Called by the live main loop only, never used to renew active motion."""
        with self.lock:
            panel = self._eligible()
            if self.mode not in ('idle_hold', 'replan_hold'):
                self.command = self._hold(panel)
            self.mode = 'idle_hold'
            self.planning_joints = None
            if cancel_recovery:
                self._clear_recovery()
            self.deadline = self.clock() + self.PRODUCER_LEASE
            self._emit()

    def send(self, command):
        # Synchronous commit preserves ExecutionGuard's checked-start timing.
        # Only repeats run in the worker; there is no delayed motion queue.
        with self.lock:
            panel = self._eligible()
            if self.mode == 'planning_hold':
                error = max(abs(float(a)-float(b)) for q, r in
                            zip(panel.arm_joint_position, self.planning_joints)
                            for a, b in zip(q, r))
                if error > self.START_ERROR:
                    self.start_rejections += 1
                    if self.recovery_started is None:
                        self.recovery_started = self.clock()
                    if (self.start_rejections >= self.MAX_START_REJECTIONS or
                            self.clock()-self.recovery_started >= self.RECOVERY_SECONDS):
                        self._fail(f'planning start remained unstable after {self.start_rejections} '
                                   f'rejections: {error:.6f}rad; watchdog handoff', panel)
                        self._check()
                    self.mode = 'replan_hold'
                    self.deadline = self.clock()+self.PRODUCER_LEASE
                    self.settle_sample = None
                    self._emit()  # Same latched hold; never publish the stale trajectory.
                    reason = (f'V3.1 discard stale-start plan: max_joint_error={error:.6f}rad, '
                              f'limit={self.START_ERROR:.6f}rad; hold, settle and replan '
                              f'({self.start_rejections}/{self.MAX_START_REJECTIONS})')
                    self.log(reason)
                    raise StalePlanStart(reason)
            self.command = copy.deepcopy(command)
            self.mode = 'active'
            self.planning_joints = None
            self._clear_recovery()
            self.deadline = self.clock() + self.PRODUCER_LEASE
            self._emit()

    def _emit(self):
        now = self.clock()
        if self.last_sent is not None:
            self.last_gap = now-self.last_sent
            if self.last_gap > .2:
                self.log(f'V3.1 panel send gap: {self.last_gap*1000:.1f}ms, mode={self.mode}')
        self._send(copy.deepcopy(self.command))
        self.last_sent = now

    def _fail(self, reason, panel=None):
        if self.fault:
            return
        self.fault = 'V3.1 panel transport: ' + reason
        self.log(self.fault)
        # One best-effort hold then silence: Upper's original watchdog still
        # latches STOP. Never extend liveness after a dead producer.
        try:
            if panel is not None and self.command is not None:
                self.command = self._hold(panel)
                self._emit()
        finally:
            self.command = None
            self.mode = 'fault'

    def tick(self):
        with self.lock:
            if self.fault or self._stop.is_set():
                return
            panel = self._panel()
            if panel is None or self.command is None or self.fault:
                return
            if self.clock() > self.deadline:
                self._fail('planner producer/deadline expired; watchdog handoff', panel)
                return
            self._emit()

    def _run(self):
        while not self._stop.wait(self.PERIOD):
            try:
                self.tick()
            except Exception as exc:
                with self.lock:
                    self._fail(f'publisher exception: {exc}')

    def terminate(self):
        with self.lock:
            panel = self._panel() if not self.fault else None
            if panel is not None and self.command is not None:
                try:
                    self.command = self._hold(panel)
                    self._emit()
                except Exception as exc:
                    self.log(f'V3.1 shutdown hold failed: {exc}')
            self.command = None
            self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.)
