"""Nonvisual X completion after explicit, typed loss of tracking points."""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_servo import ServoError,vector


def run_direct_distance(read_pose,move,goal_x,c,*,path_used=0.,started=None,
                        checkpoint=lambda:None,sleep=time.sleep,clock=time.monotonic,log=lambda e:None):
    started=clock() if started is None else started
    stamp=-np.inf;path=float(path_used);origin=None;orientation=None
    def check():
        checkpoint()
        if clock()-started>c['max_seconds']:raise ServoError('Direct approach time budget exceeded')
    def read():
        nonlocal stamp
        deadline=clock()+2.
        while True:
            check();s=read_pose();t=float(s['received_monotonic'])
            if not np.isfinite(t) or not 0<=clock()-t<=.5:raise ServoError('Stale direct-approach feedback')
            if t>stamp:break
            if clock()>=deadline:raise ServoError('No new direct-approach feedback')
            sleep(.02)
        stamp=t;q=vector(s['xyz_mm'],3,'direct TCP');angles=vector(s['angles_deg'],3,'direct orientation')
        if origin is not None:
            if not origin[0]-c['max_pose_error_mm']<=q[0]<=goal_x+c['x_tolerance_mm']:
                raise ServoError('Direct X outside remaining interval')
            if np.linalg.norm(q[1:]-origin[1:])>c['max_pose_error_mm']:raise ServoError('Direct Y/Z drift exceeded')
            error=(Rotation.from_euler('xyz',orientation,degrees=True).inv()*Rotation.from_euler('xyz',angles,degrees=True)).magnitude()
            if np.degrees(error)>c['max_orientation_error_deg']:raise ServoError('Direct tool orientation drift exceeded')
        return q,angles
    q,angles=read();origin=q.copy();orientation=angles.copy()
    if (not np.isfinite([goal_x,path]).all() or path<0 or path>c['max_path_mm']
            or not -c['x_tolerance_mm']<=goal_x-q[0]<=c['max_forward_mm']):
        raise ServoError('Direct approach budget/endpoint invalid')
    log(dict(event='direct_distance_start',start_mm=origin.tolist(),fixed_orientation_deg=orientation.tolist(),goal_x_mm=goal_x,path_used_mm=path))
    stable=0
    for iteration in range(c['max_iterations']+c['stable_frames']):
        check();remaining=goal_x-q[0]
        if abs(remaining)<=c['x_tolerance_mm']:
            stable+=1
            if stable>=c['stable_frames']:
                result=dict(distance_complete=True,visual_alignment_verified=False,actual_x_mm=float(q[0]),path_mm=path)
                log(dict(event='direct_distance_complete',**result));return result
            q,angles=read();continue
        stable=0
        if iteration>=c['max_iterations']:break
        goal=origin.copy();yz=float(np.linalg.norm(q[1:]-origin[1:]))
        budget=np.sqrt(max(0.,c['max_step_mm']**2-yz**2))
        if budget<.03:raise ServoError('Direct step budget exhausted by Y/Z drift')
        goal[0]=min(goal_x,q[0]+budget);length=float(np.linalg.norm(goal-q))
        if path+length>c['max_path_mm']:raise ServoError('Direct cumulative path budget exceeded')
        if not move(goal,orientation):raise ServoError('Direct waypoint not confirmed')
        path+=length;q,angles=read()
        if np.linalg.norm(q-goal)>c['max_pose_error_mm']:raise ServoError('Direct measured waypoint error')
    raise ServoError('Direct approach iteration budget exceeded')
