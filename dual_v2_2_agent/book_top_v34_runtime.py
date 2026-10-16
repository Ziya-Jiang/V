"""V4.1 task goals using V3.4's existing planner and ordinary RobotArm API.

No V4.1 planner hooks, visual stream, or new wire protocol. Import is inert.
"""
from contextlib import contextmanager
from types import SimpleNamespace
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from cart_book_grasp_v4_1 import ROOT,pose_mm,tool_rotation


def read_state(robot,*,check_backend=True):
    s=robot.get_current_state()
    if not s:raise ValueError('V3.4 planner feedback missing')
    now=time.monotonic()
    for key in ('received_monotonic','pose_sample_monotonic'):
        stamp=s.get(key)
        if stamp is None or not np.isfinite(stamp) or not 0<=now-stamp<=.25:
            raise ValueError('V3.4 planner feedback stale: '+key)
    if getattr(s['robot_status'],'value',s['robot_status']) not in (0,1):
        raise ValueError('Robot unavailable')
    status=getattr(s['curobo_status'],'value',s['curobo_status'])
    if check_backend and status in (3,7,8):
        raise ValueError(f'V3.4 planner rejected request: status={status}; inspect dual_v2_2_upper_planner_v3_1.py terminal')
    return s


@contextmanager
def connection():
    # These are the same readiness receipt, ports, feedback decoder and
    # RobotArm class used by cuart_book_grasp_v3_4.py.
    scripts=ROOT.parent/'dual_v2_2_control/cuarm_upper_planner/scripts'
    sys.path[:0]=[str(ROOT/'RobotRAM'),str(scripts)]
    from v3_1_readiness import require_ready
    from cart_book_grasp_v3_1_ports import dedicated_planner,AGENT_PORT,PLANNER_PORT
    from controllor.hkstp_arm import RobotArm
    require_ready(PLANNER_PORT)
    endpoints=SimpleNamespace(TARGET_LOCAL_PORT=AGENT_PORT,TARGET_REMOTE_PORT=PLANNER_PORT)
    robot=None
    with dedicated_planner(endpoints):
        try:
            robot=RobotArm(target_local_port=endpoints.TARGET_LOCAL_PORT,
                           target_remote_port=endpoints.TARGET_REMOTE_PORT)
            deadline=time.monotonic()+3
            while True:
                s=robot.get_current_state()
                if s and s.get('received_monotonic') is not None and 0<=time.monotonic()-s['received_monotonic']<.25:break
                if time.monotonic()>deadline:raise ValueError('No fresh V3.4 planner feedback on 10175/10176')
                time.sleep(.02)
            # As in V3.4, a fresh first_reset request is evaluated by the
            # existing planner even when its previous request failed. Do not
            # clear a fault or claim the new reset already succeeded here.
            s=read_state(robot,check_backend=False)
            robot.target_command.gripper_mode_command=[getattr(s[k],'value',s[k]) for k in ('left_gripper','right_gripper')]
            yield robot
        finally:
            if robot is not None:robot.cleanup()


def reset(robot,log):
    import cart_book_grasp_v1_3_motion as motion
    from book_top_model_coarse import confirmed_reset
    # Reuse the exact hot-reload helper used by V3.4 before first_reset.
    original=motion._CART_PLUS_CONFIG_PATH
    try:
        motion._CART_PLUS_CONFIG_PATH=ROOT/'cart_book_grasp_v3_config.json'
        motion._reload_initial_right_j6(robot)
    finally:motion._CART_PLUS_CONFIG_PATH=original
    first_read=True
    def reset_state(r):
        nonlocal first_read
        s=read_state(r,check_backend=not first_read)
        if first_read:
            log(dict(event='reset_previous_status',status=getattr(s['curobo_status'],'value',s['curobo_status']),
                     policy='fresh first_reset evaluated by existing V3.4 planner'))
        first_read=False
        return s
    confirmed_reset(robot,log,read_state=reset_state)


def close_fingers(robot,c,log):
    """Ordinary V3.4 gripper-only update preserves JOINT reset identity."""
    from book_top_landing_runtime import actual_aperture
    from book_top_coarse_alignment import settings
    before=read_state(robot)
    sent=time.monotonic()
    if robot.master_control(right_gripper=1,action='close_gripper',object_category='book') is not True:
        raise ValueError('V3.4 ordinary right gripper command failed')
    robot._v41_model_command_sent=True
    log(dict(event='close_fingers_requested',arm_motion=False,backend='v3_4',
             arm_request_preserved=True))
    expected=robot.get_cat_gripper('close','book')
    expected=getattr(expected,'value',expected)
    d=settings(c);deadline=sent+d['gripper_timeout_seconds'];stable=[];last=sent
    while time.monotonic()<deadline:
        s=read_state(robot)
        if any(np.linalg.norm(np.asarray(s[a]['position'])-before[a]['position'])>.001 for a in ('left_arm','right_arm')):
            raise ValueError('Arm moved during ordinary gripper close')
        opening=actual_aperture(robot);stamp=s['pose_sample_monotonic']
        if stamp>last:
            last=stamp
            if getattr(s['right_gripper'],'value',s['right_gripper'])==expected and opening>=d['closed_threshold']:
                stable.append((stamp,opening))
                if len(stable)>=3 and stamp-stable[0][0]>=.3 and np.ptp([v for _,v in stable])<=.01:
                    log(dict(event='closed_fingers_confirmed',measured_opening=opening,backend='v3_4'));return opening
            else:stable=[]
        time.sleep(.02)
    raise ValueError('V3.4 right gripper measured closure timeout')


STAGE_ACTIONS={'align_y':'move_above_grasp','raise_z':'lift_object',
               'forward_x':'move_down_to_grasp','lower_z':'move_down_to_place'}


def coarse_move(io,goal,stage):
    """Use the existing ordinary blocking Cartesian path, never direct servo."""
    robot=io.robot;action=STAGE_ACTIONS[stage]
    initial=io.guard();angles=Rotation.from_matrix(io.rotation).as_euler('xyz',degrees=True)
    old_wait=robot._wait_for_robot_stop;old_read=robot.get_current_state
    missing=object();old_initial=getattr(robot,'_servo_initial_state',missing)
    def wait(*a,**kw):
        kw.update(position_tolerance_m=io.c['landing']['coarse_position_tolerance_mm']/1000,
                  orientation_tolerance_rad=np.deg2rad(io.c['orientation_error_deg']),
                  check_interval=.02,require_fresh_stable=True,
                  max_wait_time=io.c['landing']['coarse_timeout_seconds'])
        print('V4.1本段实际到位标准：'
              f'{kw["position_tolerance_m"]*1000:g}mm / '
              f'{io.c["orientation_error_deg"]:g}deg，新鲜连续稳定反馈。',flush=True)
        return old_wait(*a,**kw)
    robot._wait_for_robot_stop=wait
    # RobotIO.old_read is the pre-wrapper reader, avoiding recursion. All
    # ordinary arrival reads now enforce the task's held-axis/gripper guards.
    robot.get_current_state=io.guard
    robot._servo_initial_state=initial
    io.log(dict(event='coarse_target',stage=stage,action=action,target_mm=goal,backend='v3_4'))
    try:
        result=robot.master_control(right_arm=[*np.asarray(goal,float),*angles],action=action,object_category='book')
        if result is not True:raise ValueError('V3.4 ordinary motion not confirmed: '+stage)
        io.log(dict(event='coarse_arrived',actual_mm=pose_mm(io.guard()),backend='v3_4'))
    finally:
        robot._wait_for_robot_stop=old_wait;robot.get_current_state=old_read
        if old_initial is missing:del robot._servo_initial_state
        else:robot._servo_initial_state=old_initial
