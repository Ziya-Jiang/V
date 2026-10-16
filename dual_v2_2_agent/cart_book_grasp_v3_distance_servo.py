"""Y/Z image servo + fixed V1_3 TCP-X endpoint. No hardware imports."""
import json
import time
from pathlib import Path
import numpy as np
from cart_book_grasp_v3_servo import ServoError, vector

CONFIG_PATH=Path(__file__).with_name('cart_book_grasp_v3_cotracker.json')


def load_config(path=CONFIG_PATH):
    c=json.loads(Path(path).read_text())
    bounds=dict(probe_mm=(.5,3),max_step_mm=(.1,5),max_forward_mm=(1,250),max_lateral_mm=(1,30),
        max_path_mm=(20,500),tolerance_px=(.5,8),x_tolerance_mm=(.1,2),gain=(.05,.7),damping=(.01,5),
        max_condition=(1,100),min_response=(.01,10),max_fit_error_px=(.1,5),max_model_error_px=(.1,6),
        max_seconds=(10,900),max_frame_age_seconds=(.1,5),max_pose_error_mm=(.1,2),
        max_orientation_error_deg=(.1,3),coarse_pose_error_mm=(1,8),coarse_settle_seconds=(.1,1),
        coarse_feedback_timeout_seconds=(.5,5),settle_seconds=(0,2))
    ints=dict(stable_frames=(2,5),max_iterations=(1,400),point_count=(6,24),min_visible_points=(6,24),window_frames=(3,12))
    if set(c)!=set(bounds)|set(ints)|{'mode','target_uv','device','checkpoint'} or c['mode']!='cotracker_distance':
        raise ServoError('Invalid CoTracker distance configuration schema')
    for key,(lo,hi) in {**bounds,**ints}.items():
        if key=='max_lateral_mm' and c[key] is None:continue
        if type(c[key]) not in ((int,) if key in ints else (int,float)) or not np.isfinite(c[key]) or not lo<=c[key]<=hi:
            raise ServoError(f'Invalid CoTracker setting: {key}')
    if c['device'] not in ('auto','cpu','cuda') or not isinstance(c['checkpoint'],str) or not c['checkpoint']:
        raise ServoError('Invalid CoTracker model settings')
    if c['target_uv'] is not None:vector(c['target_uv'],2,'alignment pixel')
    if (c['min_visible_points']>c['point_count'] or
            (c['max_lateral_mm'] is not None and c['probe_mm']>c['max_lateral_mm']) or
            c['max_path_mm']<8*c['probe_mm']):
        raise ServoError('Invalid probe/point budget')
    return c


def checked_jacobian(j,c):
    if np.shape(j)!=(2,2) or not np.isfinite(j).all():raise ServoError('Invalid Y/Z Jacobian')
    s=np.linalg.svd(j,compute_uv=False)
    if s[-1]<c['min_response'] or s[0]/s[-1]>c['max_condition']:raise ServoError('Y/Z Jacobian is weak or ill-conditioned')
    return j


def update_jacobian(j,actual,delta,error_before,error_after,c,log=lambda e:None):
    """Validate response, then learn only from sufficiently resolved motion.

    Small measured displacements can mostly be pose/tracking noise. Reject an
    unreliable candidate without replacing the last accepted local model.
    """
    actual=vector(actual,2,'measured Y/Z displacement')
    delta=vector(delta,2,'observed pixel displacement')
    predicted=j@actual;residual=delta-predicted
    distance=float(np.linalg.norm(actual));mismatch=float(np.linalg.norm(residual))
    singular=np.linalg.svd(j,compute_uv=False);condition=float(singular[0]/singular[-1])
    detail=dict(event='jacobian_update',actual_yz_mm=actual.tolist(),observed_delta_px=delta.tolist(),
                predicted_delta_px=predicted.tolist(),model_error_px=mismatch,
                measured_step_mm=distance,condition_before=condition)
    def keep(reason,**extra):
        log(dict(detail,accepted=False,reason=reason,**extra));return j
    # Never treat a skipped model update as permission to ignore bad feedback.
    if mismatch>c['max_model_error_px']:
        keep('response_mismatch')
        raise ServoError(f'Y/Z response disagrees with measured model: residual={mismatch:.3f}px '
                         f'(limit={c["max_model_error_px"]:.3f}px), actual_yz_mm={actual.tolist()}')
    minimum=max(1.,2*c['max_pose_error_mm'])
    if distance<minimum:return keep('insufficient_motion',minimum_step_mm=minimum)
    freeze_radius=max(10.,3*c['tolerance_px'])
    if min(error_before,error_after)<=freeze_radius:
        return keep('near_target',freeze_radius_px=freeze_radius)
    candidate=j+.5*np.outer(residual,actual)/(actual@actual)
    candidate_s=np.linalg.svd(candidate,compute_uv=False)
    candidate_condition=float(candidate_s[0]/max(candidate_s[-1],1e-12))
    condition_limit=min(c['max_condition'],10.,max(3.,1.5*condition))
    if candidate_condition>condition_limit:
        return keep('condition_worsened',condition_after=candidate_condition,condition_limit=condition_limit)
    if (candidate_s[-1]<max(c['min_response'],.5*singular[-1]) or
            np.linalg.det(candidate)*np.linalg.det(j)<=0):
        return keep('response_degraded',condition_after=candidate_condition)
    relative_change=float(np.linalg.norm(candidate-j,2)/np.linalg.norm(j,2))
    if relative_change>.25:
        return keep('update_too_large',relative_change=relative_change,condition_after=candidate_condition)
    checked_jacobian(candidate,c)
    log(dict(detail,accepted=True,condition_after=candidate_condition,pixels_per_mm=candidate.tolist()))
    return candidate


def run_distance_servo(observe,move,target_uv,goal_x,c,*,checkpoint=lambda:None,log=lambda e:None,clock=time.monotonic):
    target=vector(target_uv,2,'alignment target');goal_x=float(goal_x)
    if not np.isfinite(goal_x):raise ServoError('Invalid V1_3 X endpoint')
    started=clock();last_stamp=-np.inf;origin=None;path=0.;scale=1.
    def check():
        checkpoint()
        if clock()-started>c['max_seconds']:raise ServoError('CoTracker approach time budget exceeded')
    def bounds(q,tol=0.,hold_x=None):
        # A Y/Z move may hold an in-tolerance measured X slightly beyond the
        # nominal interval, but must never extend that overshoot deliberately.
        lower=origin[0] if hold_x is None else min(origin[0],hold_x)
        upper=goal_x if hold_x is None else max(goal_x,hold_x)
        if q[0]<lower-tol or q[0]>upper+tol:
            raise ServoError(f'Measured/commanded X outside approach interval: x={q[0]:.6f}mm, '
                             f'allowed=[{lower-tol:.6f}, {upper+tol:.6f}]mm')
        if c['max_lateral_mm'] is not None and np.linalg.norm(q[1:]-origin[1:])>c['max_lateral_mm']+tol:
            raise ServoError('Y/Z travel bound exceeded')
    def read():
        nonlocal last_stamp,scale
        check();o=observe();check()
        uv=vector(o['uv'],2,'tracked anchor');q=vector(o['xyz_mm'],3,'measured TCP')
        scale=float(o.get('scale',1.))
        if not np.isfinite(scale) or not .1<=scale<=20:raise ServoError('Invalid tracked patch scale')
        stamp=float(o['captured_at']);age=clock()-stamp
        if not np.isfinite(stamp) or not 0<=age<=c['max_frame_age_seconds'] or stamp<=last_stamp:
            raise ServoError('Stale or repeated CoTracker observation')
        last_stamp=stamp
        if origin is not None:bounds(q,c['max_pose_error_mm'])
        log(dict(event='distance_observation',uv=uv.tolist(),xyz_mm=q.tolist(),remaining_x_mm=goal_x-q[0],error_px=float(np.linalg.norm(uv-target))))
        return uv,q
    def go(goal,q):
        nonlocal path
        check();bounds(goal,hold_x=q[0])
        length=float(np.linalg.norm(goal-q))
        if length>max(c['probe_mm'],c['max_step_mm'])+1e-6:raise ServoError('Approach step bound exceeded')
        if path+length>c['max_path_mm']:raise ServoError('Approach path budget exceeded')
        if not move(goal):raise ServoError('Approach waypoint not confirmed')
        path+=length
        uv,actual=read()
        if np.linalg.norm(actual-goal)>c['max_pose_error_mm']:raise ServoError('Measured approach waypoint error')
        return uv,actual
    uv,q=read();origin=q.copy()
    if not -c['x_tolerance_mm']<=goal_x-origin[0]<=c['max_forward_mm']:
        raise ServoError('V1_3 endpoint exceeds remaining forward budget')
    j=None;j_scale=scale;stable=0
    for iteration in range(c['max_iterations']+c['stable_frames']):
        check();error=uv-target;remaining=goal_x-q[0]
        if remaining < -c['x_tolerance_mm']:raise ServoError('Measured TCP overshot V1_3 endpoint')
        if abs(remaining)<=c['x_tolerance_mm'] and np.linalg.norm(error)<=c['tolerance_px']:
            stable+=1
            if stable>=c['stable_frames']:
                result=dict(converged=True,path_mm=path,iterations=iteration,goal_x_mm=goal_x,
                            actual_x_mm=float(q[0]),error_px=float(np.linalg.norm(error)))
                log(dict(event='converged_distance',**result));return result
            uv,q=read();continue
        stable=0
        if iteration>=c['max_iterations']:break
        # Learn Y/Z response before the first forward step, even when centered.
        if j is None:
            motions=[];pixels=[]
            for axis in (1,2):
                for sign in (-1,1):
                    before_uv,before_q=read();goal=before_q.copy();goal[axis]+=sign*c['probe_mm']
                    a,b=go(goal,before_q)
                    if np.linalg.norm(b[1:]-before_q[1:])<.5*c['probe_mm']:raise ServoError('Y/Z probe response too small')
                    motions.append(b[1:]-before_q[1:]);pixels.append(a-before_uv)
                    # Return Y/Z only. Holding the latest measured X avoids
                    # commanding tiny reverse-X corrections for feedback noise.
                    for return_step in range(5):
                        baseline=before_q.copy();baseline[0]=b[0]
                        delta=baseline-b;distance=float(np.linalg.norm(delta))
                        return_goal=baseline if distance<=c['max_step_mm'] else b+delta*c['max_step_mm']/distance
                        uv,q=go(return_goal,b);b=q
                        if np.linalg.norm(q-before_q)<=c['max_pose_error_mm']:break
                    else:raise ServoError('Y/Z probe did not return to baseline')
                    if np.linalg.norm(uv-before_uv)>c['max_fit_error_px']:raise ServoError('Point tracking drift during Y/Z probes')
            motions=np.asarray(motions);pixels=np.asarray(pixels)
            if np.linalg.matrix_rank(motions)<2:raise ServoError('Y/Z probes lack independent motion')
            j=checked_jacobian(np.linalg.lstsq(motions,pixels,rcond=None)[0].T,c)
            if np.max(np.linalg.norm(motions@j.T-pixels,axis=1))>c['max_fit_error_px']:raise ServoError('Inconsistent Y/Z probe fit')
            j_scale=scale
            log(dict(event='jacobian_yz',pixels_per_mm=j.tolist()));continue
        # With fixed tool orientation, observed patch expansion supplies a local
        # gain adjustment as X changes; no new probes near the grasp endpoint.
        j=checked_jacobian(j*(scale/j_scale),c);j_scale=scale
        if np.linalg.norm(error)>c['tolerance_px']:
            dq=-c['gain']*np.linalg.solve(j.T@j+c['damping']**2*np.eye(2),j.T@error)
            if np.linalg.norm(dq)<.03:raise ServoError('Y/Z correction stalled')
            dq*=min(1.,c['max_step_mm']/np.linalg.norm(dq))
            old_uv,old_q=uv.copy(),q.copy();goal=q.copy();goal[1:]+=dq
            uv,q=go(goal,q);actual=q[1:]-old_q[1:];delta=uv-old_uv
            j=update_jacobian(j,actual,delta,float(np.linalg.norm(old_uv-target)),
                              float(np.linalg.norm(uv-target)),c,log=log)
        else:
            goal=q.copy();goal[0]=min(goal_x,q[0]+c['max_step_mm']);uv,q=go(goal,q)
    raise ServoError('Distance/visual alignment did not converge; grasp cancelled')
