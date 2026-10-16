"""V4.3 only: lift out of the book before a requested right-arm reset.

No automatic reset on failure. Called by the original operator/round reset
entry points, on the execution thread. Settings reload at each request.
"""
from contextlib import contextmanager
from pathlib import Path
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation

CONFIG = Path(__file__).with_name('cart_book_grasp_v4_3_reset.json')


def load_config(path=CONFIG):
    c = json.loads(Path(path).read_text())
    limits = dict(lift_mm=(1,100),step_mm=(5,50))
    if set(c) != set(limits):
        raise ValueError('Invalid V4.3 reset settings')
    for k,(lo,hi) in limits.items():
        if type(c[k]) not in (int,float) or not np.isfinite(c[k]) or not lo<=c[k]<=hi:
            raise ValueError('Invalid V4.3 reset setting: '+k)
    return c


def feedback(robot):
    s = robot.get_current_state()
    now = time.monotonic()
    if not s:
        raise ValueError('V4.3 reset missing feedback')
    for k in ('received_monotonic','pose_sample_monotonic'):
        stamp = float(s.get(k,float('nan')))
        if not np.isfinite(stamp) or not 0<=now-stamp<=.25:
            raise ValueError('V4.3 reset stale feedback: '+k)
    q = np.asarray(s['right_arm']['position'],float)*1000.
    quat = np.asarray(s['right_arm']['orientation'],float)
    joints = np.asarray(s['joint_state'],float).reshape(-1)
    if q.shape!=(3,) or quat.shape!=(4,) or joints.shape!=(14,) or not np.isfinite(np.r_[q,quat,joints]).all():
        raise ValueError('V4.3 reset invalid feedback')
    rot = Rotation.from_quat(quat[[1,2,3,0]])
    return q,rot,joints


def lift_before_reset(robot, hold, *, config_path=CONFIG):
    c = load_config(config_path)
    q,rot,joints = feedback(robot)
    ready = np.deg2rad(np.asarray(robot.right_safe_joint_deg,float))
    if ready.shape!=(7,) or not np.isfinite(ready).all():
        raise ValueError('V4.3 reset invalid ready joints')
    if np.max(np.abs(joints[7:]-ready)) <= np.deg2rad(1.):
        print('V4.3：右臂已在准备位，跳过重复抬升。',flush=True)
        return True
    if hold() is not True:
        print('V4.3：复位前保持未确认，取消抬升和复位。',flush=True)
        return False
    q,rot,_ = feedback(robot)
    anchor = q.copy();angles=rot.as_euler('xyz',degrees=True)
    print('V4.3 复位前抬升：'+json.dumps(dict(start_tcp_mm=q.tolist(),**c)),flush=True)
    previous_wait = robot._wait_for_robot_stop
    def wait(*args,**kwargs):
        kwargs.update(position_tolerance_m=.003,orientation_tolerance_rad=np.deg2rad(1.),
                      require_fresh_stable=True,max_wait_time=60.,check_interval=.02)
        return previous_wait(*args,**kwargs)
    robot._wait_for_robot_stop=wait
    try:
        n=int(np.ceil(c['lift_mm']/c['step_mm']))
        for dz in np.linspace(c['lift_mm']/n,c['lift_mm'],n):
            target=anchor+[0,0,dz]
            # Reuse the backend's fixed-X Cartesian path policy. No vision or
            # new gripper command is needed for this reset clearance move.
            if robot.master_control(right_arm=[*target,*angles],action='book_v42_raise_servo',
                                    object_category='book',wait_for_completion=True) is not True:
                return False
            actual,rotation,_=feedback(robot)
            error=float(np.linalg.norm(actual-target))
            angle=float(np.rad2deg((rot.inv()*rotation).magnitude()))
            if error>3. or angle>1.:
                raise ValueError(f'V4.3 reset lift not reached: position={error:.3f}mm, angle={angle:.3f}deg')
        return True
    finally:
        robot._wait_for_robot_stop=previous_wait


@contextmanager
def installed(agent, loop_module, *, config_path=CONFIG):
    original=agent.RobotArm
    original_pause=loop_module.open_and_reset

    class LiftResetRobot(original):
        def _v43_lift_for_reset(self):
            def hold():
                feedback(self)
                return super(LiftResetRobot,self).control_joint_position(
                    left_pos=False,right_pos=False,action='book_operator_hold',force=True)
            complete=False
            try:
                complete=lift_before_reset(self,hold,config_path=config_path)
                return complete
            finally:
                if not complete:
                    try:confirmed=hold() is True
                    except BaseException as error:
                        confirmed=False
                        print(f'V4.3 复位抬升失败后的保持未确认：{error}',flush=True)
                    print(f'V4.3 复位已取消；保持请求 confirmed={confirmed}',flush=True)

        def control_joint_position(self,left_pos,right_pos,action,*,force=False):
            if right_pos is True and action in ('first_reset','reset_pos','reset_all') and not getattr(self,'_v43_reset_lift_done',False):
                if not self._v43_lift_for_reset():
                    print('V4.3：抬升未确认，停止复位，保持夹爪当前指令。',flush=True)
                    return False
            return super().control_joint_position(left_pos,right_pos,action,force=force)

    def pause_reset(robot,reload_ready):
        # p: lift while keeping the current gripper state, then run the old
        # open/reset sequence. Suppress a second clearance lift in that call.
        reload_ready(robot)
        if not robot._v43_lift_for_reset():
            print('V4.3 p复位：抬升未确认，取消松爪和复位。',flush=True)
            return False
        robot._v43_reset_lift_done=True
        try:return original_pause(robot,reload_ready)
        finally:robot._v43_reset_lift_done=False

    agent.RobotArm=LiftResetRobot;loop_module.open_and_reset=pause_reset
    try:yield
    finally:agent.RobotArm=original;loop_module.open_and_reset=original_pause
