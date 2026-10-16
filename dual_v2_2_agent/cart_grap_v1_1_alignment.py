"""V1.1 final-hold revalidation within one bounded visual servo session."""
import time
import sys
import numpy as np
from cart_grasp_alignment import MarkerLost

def align_loop(read,observe,send,ready,hold,origin,c,log,clock=time.monotonic,*,goal_x=None,approach=None,finalize=None):
    """Injectable control loop. observe(checkpoint) returns error + paired pose."""
    from servor import fit_jacobian,correction,angle_difference
    from cart_grasp_depth import ForwardStep
    if (goal_x is None)!=(approach is None):raise ValueError('Approach requires an explicit depth endpoint and settings')
    if goal_x is not None and (not np.isfinite(goal_x) or goal_x<origin[0]-approach['x_tolerance_mm'] or goal_x-origin[0]>approach['max_forward_mm']):
        raise ValueError('Invalid cart forward endpoint')
    forward=ForwardStep(approach) if approach else None
    deadline=clock()+c['max_seconds'];previous=None;path=0.;stamp=None;completed=False
    def guard():
        nonlocal previous,path,stamp
        if clock()>=deadline:raise ValueError('Cart alignment time budget exceeded')
        p=read();q=p['q']
        if stamp!=p['pose_sample_monotonic']:
            if previous is not None:path+=np.linalg.norm(q-previous)
            previous=q.copy();stamp=p['pose_sample_monotonic']
        if path>c['max_path_mm']:raise ValueError('Cart measured path budget exceeded')
        if abs(q[2]-origin[2])>c['fixed_xz_mm'] or (goal_x is None and abs(q[0]-origin[0])>c['fixed_xz_mm']):
            raise ValueError('Cart fixed X/Z drift')
        if goal_x is not None and not origin[0]-c['fixed_xz_mm']<=q[0]<=goal_x+approach['x_tolerance_mm']:
            raise ValueError('Cart measured X outside approach interval')
        if abs(q[1]-origin[1])>c['max_servo_y_mm'] or abs(p['roll'])>c['max_roll_deg']:
            raise ValueError('Cart Y/roll travel bound exceeded')
        return p
    def move(q,roll,blocking=False):
        current=guard();q=np.asarray(q).copy();q[2]=origin[2]
        if goal_x is None:q[0]=origin[0]
        else:
            # Image-time/current samples can differ; never retreat to an older X.
            q[0]=max(current['q'][0],min(goal_x,q[0]))
        if goal_x is not None and (q[0]>max(goal_x,current['q'][0])+.0001 or
              q[0]<current['q'][0]-.0001 or q[0]-current['q'][0]>approach['max_x_step_mm']+.0001):
            raise ValueError('Cart requested X exceeds head endpoint/lookahead')
        if (abs(q[1]-origin[1])>c['max_servo_y_mm'] or abs(roll)>c['max_roll_deg'] or
                abs(q[1]-current['q'][1])>max(3.,c['max_y_step_mm'])+.01 or
                abs(roll-current['roll'])>2.01):
            raise ValueError('Cart requested correction exceeds bounds')
        send(q,roll,blocking)
    try:
        before,p0=observe(guard)
        errors=[];poses=[]
        for dy,dr in ((3.,0.),(0.,2.)):
            p=guard();q=p['q'].copy();q[1]+=dy
            move(q,p['roll']+dr,True)
            e,p=observe(guard);errors.append(e)
            poses.append([p['q'][1]-p0['q'][1],p['roll']-p0['roll']])
        jac=fit_jacobian(before,errors,poses)
        initial_scale=float(p.get('marker_scale_px',1.))
        log(dict(event='cart_jacobian_y_roll',matrix=jac.tolist()))
        holding=False;lost=None;stable_start=None;stable=0;last_update=-np.inf;last_stable=None
        settle_target=None
        while True:
            guard()
            try:e,paired=observe(guard)
            except MarkerLost as exc:
                settle_target=None
                if lost is None:hold();holding=True;lost=clock();log(dict(event='marker_lost_hold',reason=str(exc)))
                if forward:forward.update(clock(),0,False)
                if clock()-lost>=c['loss_timeout_seconds']:raise ValueError('ID 0 reacquisition timed out')
                stable=0;stable_start=None;last_stable=None
                continue
            if lost is not None:log(dict(event='marker_reacquired',marker_id=0));lost=None
            p=guard()
            if np.linalg.norm(p['q']-paired['q'])>3 or abs(p['roll']-paired['roll'])>1:
                settle_target=None
                if not holding:hold();holding=True
                if forward:forward.update(clock(),0,False)
                stable=0;stable_start=None;last_stable=None
                continue
            local_jac=jac.copy()
            # Approximate image translation sensitivity scales with apparent marker size.
            ratio=float(paired.get('marker_scale_px',initial_scale))/initial_scale
            if not .5<=ratio<=4:raise ValueError('Marker scale outside local servo model')
            local_jac[0]*=ratio
            predicted=e+local_jac@np.array([p['q'][1]-paired['q'][1],angle_difference(p['roll'],paired['roll'])])
            aligned=abs(e[0])<=c['tolerance_px'] and abs(e[1])<=c['tolerance_angle_deg']
            remaining=0. if goal_x is None else goal_x-p['q'][0]
            arrived=goal_x is None or abs(remaining)<=approach['x_tolerance_mm']
            # Once one bounded step can reach the endpoint, keep that endpoint
            # alive while settling. Freezing measured X at goal-tolerance caused
            # repeated hold/advance transitions as feedback crossed the boundary.
            near_endpoint=goal_x is None or -approach['x_tolerance_mm']<=remaining<=approach['max_x_step_mm']
            centered=abs(e[0])<=c['tolerance_px']*.5 and abs(e[1])<=c['tolerance_angle_deg']*.5
            if aligned and (settle_target is not None or (near_endpoint and centered)):
                if settle_target is None:
                    if not ready():continue
                    q=p['q'].copy();q[2]=origin[2]
                    if goal_x is not None:q[0]=max(q[0],goal_x)
                    settle_target=(q,p['roll'])
                    log(dict(event='cart_terminal_target',tcp_mm=q.tolist(),roll_deg=float(p['roll'])))
                    move(*settle_target);last_update=clock();holding=False
                    stable=0;stable_start=None;last_stable=None
                    continue  # Never count pre-command feedback as settled.
                if clock()-last_update>=1/c['update_hz'] and ready():
                    move(*settle_target);last_update=clock();holding=False
                if not arrived:
                    stable=0;stable_start=None;last_stable=None
                    continue
                if last_stable is not None and (np.linalg.norm(p['q']-last_stable['q'])>.3 or abs(p['roll']-last_stable['roll'])>.15):
                    stable=0;stable_start=None
                if stable_start is None:stable_start=clock()
                if last_stable is None or p['pose_sample_monotonic']!=last_stable['pose_sample_monotonic']:
                    stable+=1
                last_stable=p
                if stable>=5 and clock()-stable_start>=.3:
                    # Hold can move the image. Validate AFTER that motion, and
                    # resume the same Jacobian/session if it is no longer aligned.
                    if finalize is not None:
                        confirmed=finalize(guard)
                        if confirmed is None:
                            settle_target=None;stable=0;stable_start=None;last_stable=None
                            continue
                        e,p=confirmed
                    result=dict(event='cart_aligned',error_px_deg=e.tolist(),tcp_mm=p['q'].tolist(),
                                tool_roll_deg=p['roll'],path_mm=float(path),gripper_changed=False,goal_tcp_x_mm=goal_x)
                    completed=finalize is not None
                    log(result);return result
                continue
            settle_target=None
            stable=0;stable_start=None;last_stable=None
            if clock()-last_update<1/c['update_hz'] or not ready():continue
            delta=correction(predicted,local_jac,c['gain'])
            delta*=min(1.,c['max_y_step_mm']/max(abs(delta[0]),1e-12),
                       c['max_roll_step_deg']/max(abs(delta[1]),1e-12))
            q=p['q'].copy();q[1]+=delta[0]
            if forward:
                dx=forward.update(clock(),remaining,aligned and not arrived)
                q[0]+=min(dx,max(0,remaining))
                log(dict(event='cart_depth_approach',actual_x_mm=float(p['q'][0]),goal_x_mm=goal_x,
                         remaining_mm=float(remaining),step_mm=float(dx),aligned=bool(aligned)))
            move(q,p['roll']+delta[1]);last_update=clock();holding=False
    finally:
        if completed:
            pass  # No extra movement after post-hold visual confirmation.
        elif sys.exc_info()[0] is None:
            hold()
        else:
            # A failed cleanup must not replace the original fault diagnosis.
            try:hold()
            except Exception as exc:log(dict(event='cart_cleanup_hold_failed',reason=f'{type(exc).__name__}: {exc}'))
