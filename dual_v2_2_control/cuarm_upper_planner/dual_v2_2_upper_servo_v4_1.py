#!/usr/bin/env python3
"""Independent V4.1 servo process. Shares transport/model loader, not V3 policy.

The legacy wire streaming verb is retained on private ports 10185/10186.
Process-local factories install the XYZ controller and guard; old files and
old processes remain unchanged. No MotionGen calls are used for V4.1 streams.
"""
import ast
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent


def install_landing_routing(planner):
    """Only pre-servo coarse motions may use MotionGen; faults never reset here."""
    original_compute = planner.compute
    visual_started = False
    def compute():
        nonlocal visual_started
        if not planner._first_target_command_received:
            return original_compute()
        verb = planner.target_description
        guard = planner._v31_bridge.guard
        if verb == 'book_v4_1_close' and not visual_started and not guard.fault:
            from v4_1_direct_servo import compute_closed_fingers
            return compute_closed_fingers(planner)
        if verb == 'book_v4_1_pause':
            from v4_1_direct_servo import compute_perception_pause
            return compute_perception_pause(planner)
        allowed = verb in ('book_v3_1_stream', 'book_v3_1_hold')
        if verb == 'book_v4_1_coarse':
            allowed = not visual_started and not guard.fault
        if verb.startswith('first_reset:joint_reset:'):
            from cuarm_state import TargetCommandArmCommandType
            allowed = (not visual_started and not guard.fault and
                       getattr(planner, 'target_arm_type', None) == TargetCommandArmCommandType.JOINT)
        if not allowed or (guard.fault and verb != 'book_v3_1_hold'):
            from cuarm_state import CuroboStatus
            import copy
            guard.trip('V4.1 rejected phase transition/action; coarse only before wrist servo, faults retained')
            planner.status = CuroboStatus.PLANNING_FAILED
            planner._hold_current_arm_command()
            return copy.deepcopy(planner.curobo_cmd), copy.deepcopy(planner.curobo_state)
        if verb == 'book_v3_1_stream':
            visual_started = True
        return original_compute()
    planner.compute = compute


def main():
    if sys.argv[1:]:
        if sys.argv[1:] != ['--check-config']:
            raise SystemExit('Usage: dual_v2_2_upper_servo_v4_1.py [--check-config]')
        for path in [Path(__file__), ROOT/'scripts/v4_1_direct_servo.py', ROOT/'scripts/v4_1_readiness.py']:
            ast.parse(path.read_text())
        print('V4.1 static syntax check passed; agent 10185 -> servo 10186; no CUDA, UDP or robot opened')
        return 0
    os.environ['CUARM_V31_TARGET_LOCAL_PORT'] = '10186'
    os.environ['CUARM_V31_TARGET_REMOTE_PORT'] = '10185'
    import dual_v2_2_upper_planner_v3_1 as transport_main
    import v3_1_execution
    import v3_1_readiness
    from v4_1_direct_servo import TiltDirectServo, TiltExecutionGuard
    from v4_1_readiness import publish_ready
    original = transport_main.create_curobo_planner

    def create(config, solver_type):
        planner = original(config, solver_type)
        planner._v31_direct = TiltDirectServo(planner, planner._v31_direct.collision)
        install_landing_routing(planner)
        return planner

    # The shared main imports these factories lazily. This affects this
    # dedicated process only; no edits to original planner/RT files.
    transport_main.create_curobo_planner = create
    v3_1_execution.ExecutionGuard = TiltExecutionGuard
    v3_1_readiness.publish_ready = publish_ready
    return transport_main.main()


if __name__ == '__main__':
    raise SystemExit(main())
