"""Bounded, measured-Jacobian XYZ image servo. No hardware imports."""
import json
from pathlib import Path
import time
import numpy as np
from cart_book_grasp_v3_servo import ServoError, vector

CONFIG_PATH = Path(__file__).with_name('cart_book_grasp_v3_approach.json')


def load_approach(path=CONFIG_PATH):
    c = json.loads(Path(path).read_text())
    c.setdefault('target_selection','head_identity')
    if c['target_selection'] not in ('wrist_center','head_identity'):
        raise ServoError('Invalid target selection mode')
    bounds = dict(probe_mm=(.5,5), max_step_mm=(.1,5), max_forward_mm=(1,250),
        max_backtrack_mm=(0,10), max_lateral_mm=(1,30), max_path_mm=(10,500),
        tolerance_px=(.5,5), gain=(.05,.7), damping_px_per_mm=(.01,5),
        max_condition=(1,100), min_response_px_per_mm=(.01,10), max_fit_error_px=(.1,5),
        max_model_error_px=(.1,10), max_seconds=(10,900), max_frame_age_seconds=(.1,5),
        max_pose_error_mm=(.1,2), max_orientation_error_deg=(.1,3), settle_seconds=(0,2),
        coarse_pose_error_mm=(1,8), coarse_settle_seconds=(.1,1),
        coarse_feedback_timeout_seconds=(.5,5))
    if set(c) != set(bounds)|{'mode','reference','stable_frames','max_iterations','target_selection'} or c['mode'] != 'stage2_xyz':
        raise ServoError('Invalid XYZ approach schema')
    for key,(lo,hi) in bounds.items():
        if type(c[key]) not in (int,float) or not np.isfinite(c[key]) or not lo <= c[key] <= hi:
            raise ServoError(f'Invalid XYZ setting: {key}')
    for key,lo,hi in [('stable_frames',2,5),('max_iterations',1,200)]:
        if type(c[key]) is not int or not lo <= c[key] <= hi:
            raise ServoError(f'Invalid XYZ setting: {key}')
    if min(c['max_backtrack_mm'],c['max_lateral_mm'],c['max_forward_mm']) < c['probe_mm'] or c['max_path_mm'] < 12*c['probe_mm']:
        raise ServoError('XYZ motion bounds do not allow the initial probes')
    r = c['reference']
    required = {'image','image_sha256','metadata','role','roi','points','mask_polygon'}
    if not isinstance(r,dict) or set(r) != required or r['role'] != 'final_grasp_before_close':
        raise ServoError('A final grasp image reference is required')
    if not isinstance(r['image'],str) or not r['image']:
        raise ServoError('Final reference image path is required')
    if (not isinstance(r['image_sha256'],str) or len(r['image_sha256']) != 64 or
            any(v not in '0123456789abcdef' for v in r['image_sha256'])):
        raise ServoError('Invalid final image fingerprint')
    vector(r['roi'],4,'final ROI')
    points = np.asarray(r['points'],float)
    if points.shape != (4,2) or not np.isfinite(points).all():
        raise ServoError('Invalid final image points')
    return c


def checked_jacobian(j, c):
    j = np.asarray(j,float)
    if j.shape != (8,3) or not np.isfinite(j).all():
        raise ServoError('Expected an 8x3 XYZ image Jacobian')
    singular = np.linalg.svd(j,compute_uv=False)
    if singular[-1] < c['min_response_px_per_mm'] or singular[0]/singular[-1] > c['max_condition']:
        raise ServoError('XYZ image Jacobian is weak or ill-conditioned')
    return j


def fit_xyz(q, ds, c):
    q,ds=np.asarray(q,float),np.asarray(ds,float)
    if q.shape!=(6,3) or ds.shape!=(6,8) or not np.isfinite(q).all() or not np.isfinite(ds).all():
        raise ServoError('Six measured XYZ probe responses are required')
    if np.linalg.matrix_rank(q)!=3 or np.linalg.cond(q)>c['max_condition']:
        raise ServoError('XYZ probes do not excite independent axes')
    j=checked_jacobian(np.linalg.lstsq(q,ds,rcond=None)[0].T,c)
    residual=(q@j.T-ds).reshape(-1,4,2)
    if np.max(np.linalg.norm(residual,axis=2))>c['max_fit_error_px']:
        raise ServoError('Inconsistent XYZ probe image responses')
    return j


def correction_xyz(j,e,c):
    j=checked_jacobian(j,c); e=vector(e,8,'feature error')
    dq=-c['gain']*np.linalg.solve(j.T@j+c['damping_px_per_mm']**2*np.eye(3),j.T@e)
    norm=np.linalg.norm(dq)
    if norm>c['max_step_mm']:dq*=c['max_step_mm']/norm
    return dq


def run_xyz(observe,move,config,*,checkpoint=lambda:None,log=lambda event:None,clock=time.monotonic):
    """observe -> points[4,2], xyz_mm[3], captured_at; move -> confirmed bool.

    All control features correspond to the SAME four points in the final image.
    Initial probes are at the observation pose. Only measured motion updates J.
    """
    c=config; target=np.asarray(c['reference']['points'],float).reshape(8)
    started=clock(); origin=None; last_stamp=-np.inf; path=0.
    def check():
        checkpoint()
        if clock()-started>c['max_seconds']:raise ServoError('XYZ time budget exceeded')
    def bounded(p,tolerance=0.):
        delta=p-origin
        if not (-c['max_backtrack_mm']-tolerance<=delta[0]<=c['max_forward_mm']+tolerance):
            raise ServoError('XYZ forward/backtrack bound exceeded')
        if np.linalg.norm(delta[1:])>c['max_lateral_mm']+tolerance:
            raise ServoError('XYZ lateral bound exceeded')
    def read():
        nonlocal last_stamp
        check(); o=observe(); check()
        points=np.asarray(o['points'],float)
        if points.shape!=(4,2) or not np.isfinite(points).all():raise ServoError('Invalid tracked final points')
        xyz=vector(o['xyz_mm'],3,'measured XYZ'); stamp=float(o['captured_at']); age=clock()-stamp
        if not np.isfinite(stamp) or not 0<=age<=c['max_frame_age_seconds'] or stamp<=last_stamp:
            raise ServoError('Stale or repeated XYZ observation')
        last_stamp=stamp
        if origin is not None:bounded(xyz,c['max_pose_error_mm'])
        features=points.reshape(8)
        log(dict(event='observation_xyz',points=points.tolist(),xyz_mm=xyz.tolist(),
                 max_error_px=float(np.max(np.linalg.norm(points-target.reshape(4,2),axis=1)))))
        return features,xyz
    def go(goal,actual):
        nonlocal path
        check(); bounded(goal)
        distance=float(np.linalg.norm(goal-actual))
        if distance>max(c['probe_mm'],c['max_step_mm'])+2*c['max_pose_error_mm']:
            raise ServoError('XYZ step bound exceeded')
        if path+distance>c['max_path_mm']:raise ServoError('XYZ path budget exceeded')
        log(dict(event='move_xyz',xyz_mm=goal.tolist()))
        if not move(goal):raise ServoError('XYZ waypoint not confirmed')
        path+=distance
        s,q=read()
        if np.linalg.norm(q-goal)>c['max_pose_error_mm']:raise ServoError('XYZ measured waypoint error')
        return s,q
    def error(s):return float(np.max(np.linalg.norm((s-target).reshape(4,2),axis=1)))
    s,origin=read();origin=origin.copy();q=origin.copy();stable=0
    while error(s)<=c['tolerance_px']:
        stable+=1
        if stable>=c['stable_frames']:
            return dict(converged=True,iterations=0,path_mm=path,error_px=error(s))
        s,q=read()
    motions=[];pixels=[]
    for axis in range(3):
        for sign in (-1,1):
            baseline_s,baseline_q=read()
            goal=origin.copy();goal[axis]+=sign*c['probe_mm']
            probe_s,probe_q=go(goal,baseline_q)
            if np.linalg.norm(probe_q-baseline_q)<.5*c['probe_mm']:
                raise ServoError('Measured XYZ probe is too small')
            motions.append(probe_q-baseline_q);pixels.append(probe_s-baseline_s)
            s,q=go(origin,probe_q)
            if np.max(np.linalg.norm((s-baseline_s).reshape(4,2),axis=1))>c['max_fit_error_px']:
                raise ServoError('Final patch moved or tracking drifted during XYZ probe')
    j=fit_xyz(motions,pixels,c);log(dict(event='jacobian_xyz',pixels_per_mm=j.tolist()))
    stable=0;increases=0
    for iteration in range(c['max_iterations']+c['stable_frames']):
        check(); current_error=error(s)
        if current_error<=c['tolerance_px']:
            stable+=1
            if stable>=c['stable_frames']:
                result=dict(converged=True,iterations=iteration,path_mm=path,error_px=current_error)
                log(dict(event='converged_xyz',**result));return result
            s,q=read();continue
        stable=0
        if iteration>=c['max_iterations']:break
        dq=correction_xyz(j,s-target,c)
        if np.linalg.norm(dq)<.05:raise ServoError('XYZ correction stalled before convergence')
        old_s,old_q=s.copy(),q.copy();s,q=go(q+dq,q)
        actual_dq=q-old_q;ds=s-old_s
        model_error=np.max(np.linalg.norm((ds-j@actual_dq).reshape(4,2),axis=1))
        if model_error>c['max_model_error_px']:
            raise ServoError('XYZ observed response disagrees with the motion model')
        increases=increases+1 if error(s)>current_error+c['max_fit_error_px'] else 0
        if increases>=2:raise ServoError('XYZ feature error increased twice')
        # Broyden update follows changing depth; never probe again near contact.
        denominator=float(actual_dq@actual_dq)
        if denominator>.01:
            j=checked_jacobian(j+.5*np.outer(ds-j@actual_dq,actual_dq)/denominator,c)
    raise ServoError('XYZ final image did not converge; grasp cancelled')
