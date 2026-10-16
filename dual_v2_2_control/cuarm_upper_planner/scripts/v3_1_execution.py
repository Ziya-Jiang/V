"""Independent height interlock and atomic panel-command arbitration for V3.1.

No vision or CUDA is used here. A latched fault replaces the last waypoint
command with a measured joint hold. This is NOT a hardware emergency stop.
"""
import copy
import threading
import time
import numpy as np

PLANNED_Z_MM = 2.0
MONITOR_Z_MM = 3.0
HARD_Z_MM = 4.0
HANDOFF_JOINT_RAD = .002
HANDOFF_TCP_MM = .5
HANDOFF_HORIZON_S = .04


def handoff_matches(checked, current, now):
    """Compare the splice prediction error, not normal in-flight displacement."""
    age = now-checked['stamp']
    dt = current['stamp']-checked['stamp']
    if not 0 <= dt <= age <= HANDOFF_HORIZON_S:
        return False
    predicted_joints = checked['joints'] + checked['velocity']*dt
    predicted_tcp = checked['position'][1] + checked.get('tcp_velocity', np.zeros(3))*dt
    return (np.max(np.abs(current['joints']-predicted_joints)) <= HANDOFF_JOINT_RAD
            and np.linalg.norm(current['position'][1]-predicted_tcp)*1000 <= HANDOFF_TCP_MM)


class ExecutionGuard:
    def __init__(self, send, clock=time.monotonic, log=print):
        self.send = send
        self.clock = clock
        self.log = log
        self.lock = threading.RLock()
        self.reference = None
        self.fault = None
        self.snapshot = None
        self.last_command = None
        self.hold_command = None
        self.published_key = None
        self.published_request = None
        self.last_hold = -np.inf

    def begin(self, z):
        with self.lock:
            if self.fault: raise ValueError(self.fault)
            if self.reference is None:
                self.reference = float(z)
            elif abs(z-self.reference)*1000 > .05:
                raise ValueError('V3.1 stream cannot change the fixed Z reference')

    def reset(self):
        with self.lock:
            self.reference = None
            self.fault = None
            self.last_command = self.hold_command = self.published_key = None
            self.published_request = None

    def acknowledgement(self):
        """One coherent dispatch identity, independent of planning caches."""
        with self.lock:
            return (self.published_key, copy.deepcopy(self.published_request), self.fault)

    def update(self, snapshot):
        with self.lock:
            if snapshot is not None:
                self.snapshot = snapshot
            if self.reference is None: return
            if self.snapshot is None or self.clock()-self.snapshot['stamp'] > .25:
                self.trip('V3.1 independent monitor: stale panel feedback')
            else:
                delta = (self.snapshot['position'][1,2]-self.reference)*1000
                if abs(delta) >= MONITOR_Z_MM:
                    self.trip(f'V3.1 independent Z monitor: delta={delta:+.3f}mm, '
                              f'hold_limit={MONITOR_Z_MM:.1f}mm, hard_limit={HARD_Z_MM:.1f}mm')
            if self.fault and self.hold_command is not None and self.clock()-self.last_hold >= .05:
                self.send(self.hold_command)
                self.last_hold = self.clock()

    def trip(self, reason):
        with self.lock:
            if self.fault: return
            self.fault = str(reason)
            self.log(self.fault)
            # Keep the first hold immutable: following a moving measured pose
            # every tick would itself create a sequence of new trajectories.
            if self.snapshot is not None and self.last_command is not None:
                self.hold_command = copy.deepcopy(self.last_command)
                q = self.snapshot['joints'].reshape(2,7).tolist()
                self.hold_command.arm_waypoint_size = 2
                self.hold_command.arm_waypoint_dt = .2
                self.hold_command.arm_waypoint_command = [q, copy.deepcopy(q)]
                self.hold_command.arm_joint_command = copy.deepcopy(q)
                self.send(self.hold_command)
                self.last_hold = self.clock()

    def publish(self, command, checked_start, key, requested=None):
        with self.lock:
            self.update(None)
            if self.fault: return False
            if key == self.published_key:
                self.send(self.last_command)  # identical keepalive; no re-splice
                return True
            now = self.snapshot
            if now is None or self.clock()-now['stamp'] > .1:
                return False
            if not handoff_matches(checked_start, now, self.clock()):
                self.log('V3.1 stale/inconsistent handoff discarded; replanning from fresh feedback')
                return False
            changed_request = (self.published_request is None or requested is None or
                               not np.array_equal(self.published_request,requested))
            self.send(command)
            self.last_command = copy.deepcopy(command)
            self.published_key = key
            self.published_request = copy.deepcopy(requested)
            if changed_request:
                self.log(f'V3.1 segment dispatched: key={key}, '
                         f'checked_start_age_ms={(self.clock()-checked_start["stamp"])*1000:.1f}')
            return True
