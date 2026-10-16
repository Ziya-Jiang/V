"""Book centerline tracking and measured Y/tool-axis-roll visual servo."""
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_servo import ServoError, vector


def roll_orientation(base_euler, roll_deg):
    # Same Rz @ Ry @ Rx convention as the robot. Local Z is the grasp axis;
    # rotating around it preserves the fingertip-axis X offset used by V1_3.
    base = Rotation.from_euler('xyz', base_euler, degrees=True)
    return (base * Rotation.from_euler('z', roll_deg, degrees=True)).as_euler('xyz', degrees=True)


def measured_roll(base_euler, euler, tolerance_deg):
    relative = Rotation.from_euler('xyz', base_euler, degrees=True).inv() * Rotation.from_euler('xyz', euler, degrees=True)
    matrix = relative.as_matrix()
    angle = float(np.degrees(np.arctan2(matrix[1, 0], matrix[0, 0])))
    residual = (Rotation.from_euler('z', angle, degrees=True).inv() * relative).magnitude()
    if np.degrees(residual) > tolerance_deg:
        raise ServoError('Centerline rotation left the tool grasp axis')
    return angle


def visible_centerline(mask):
    """Robust midpoints of the two visible spine sides, at image center row.

    Top/bottom clipping is allowed. Rows clipped at left/right cannot locate
    both sides and are excluded, rather than silently using a false midpoint.
    """
    mask = np.asarray(mask, bool)
    rows = np.flatnonzero(mask.any(axis=1))
    left = mask[rows].argmax(axis=1)
    right = mask.shape[1] - 1 - mask[rows, ::-1].argmax(axis=1)
    valid = (right-left >= 4) & (left > 0) & (right < mask.shape[1]-1)
    rows, mid = rows[valid], (left[valid]+right[valid])*.5
    if len(rows) < 16 or np.ptp(rows) < 20:
        raise ServoError('Too few visible book sides to estimate centerline')
    points = np.column_stack([mid, rows]).astype(np.float32)
    dx, dy, x, y = cv2.fitLine(points, cv2.DIST_HUBER, 0, .01, .01).ravel()
    if abs(dy) < .5: raise ServoError('Book centerline is too close to horizontal')
    slope = float(dx/dy); cy = (mask.shape[0]-1)/2
    residual = mid-(x+slope*(rows-y))
    if np.median(np.abs(residual)) > 4:
        raise ServoError('Visible book centerline is not consistent')
    return float(x+slope*(cy-y)), float(np.degrees(np.arctan(slope)))


# Initial mask defines the centerline; texture points transport it between frames.
from cart_book_grasp_v3_feature_centerline import FeatureCenterlineLock as CenterlineLock


def actionable_correction(correction, jacobian, error, tolerance, budget, config):
    """Avoid tiny Y commands hidden inside the arrival ball, if the image
    model predicts improvement. Never force a large step across the image
    target; small necessary steps are protected by the real-progress gate.
    """
    correction=correction.copy()
    minimum=min(budget,config['max_pose_error_mm']+.5)
    dy=abs(correction[0])
    if abs(error[0])>tolerance[0] and 1e-6<dy<minimum:
        factor=minimum/dy
        candidate=correction*factor
        if abs(candidate[1])<=config['max_roll_step_deg']:
            predicted=error+jacobian@candidate
            baseline=error+jacobian@correction
            if (np.linalg.norm(predicted/tolerance)<=np.linalg.norm(baseline/tolerance)
                    or (np.abs(predicted)<=tolerance).all()):
                return candidate
    return correction


def run_centerline_servo(observe, move, target_u, goal_x, c, *, checkpoint=lambda:None,
                         log=lambda e:None, clock=time.monotonic):
    """2D image [u, line tilt] -> [base Y, local tool Z roll], fixed TCP Z."""
    start=clock(); stamp=-np.inf; origin=None; path=0.; stable=0
    def check():
        checkpoint()
        if clock()-start > c['max_seconds']: raise ServoError('Centerline time budget exceeded')
    def read():
        nonlocal stamp
        check(); o=observe(); check()
        q=vector(o['xyz_mm'],3,'measured TCP'); r=float(o['roll_deg'])
        f=np.array([o['uv'][0]-target_u, o['angle_deg']],float)
        s=float(o['scale']); t=float(o['captured_at'])
        if not np.isfinite([*f,r,s,t]).all() or not .1<=s<=20 or t<=stamp or not 0<=clock()-t<=c['max_frame_age_seconds']:
            raise ServoError('Stale or invalid centerline observation')
        stamp=t
        if abs(r)>c['max_roll_deg']+c['max_orientation_error_deg']: raise ServoError('Wrist roll bound exceeded')
        if origin is not None:
            if q[0]<origin[0]-c['max_pose_error_mm'] or q[0]>goal_x+c['x_tolerance_mm']: raise ServoError('Centerline X outside interval')
            if abs(q[2]-origin[2])>c['max_pose_error_mm']: raise ServoError('Centerline measured Z drift exceeded')
        log(dict(event='centerline_observation',error_px=float(f[0]),angle_deg=float(f[1]),roll_deg=r,xyz_mm=q.tolist()))
        return f,q,r,s
    def go(goal, roll, q, r):
        nonlocal path
        check(); goal=goal.copy(); goal[2]=origin[2]
        length=float(np.linalg.norm(goal-q))
        if length>max(c['max_step_mm'],c['probe_mm'])+1e-6: raise ServoError('Centerline translation step exceeded')
        if abs(roll-r)>max(c['max_roll_step_deg'],c['probe_roll_deg'])+1e-6 or abs(roll)>c['max_roll_deg']:
            raise ServoError('Centerline roll command bound exceeded')
        if path+length>c['max_path_mm']: raise ServoError('Centerline path budget exceeded')
        if not min(origin[0],q[0])<=goal[0]<=max(goal_x,q[0]): raise ServoError('Centerline command outside X interval')
        began=clock()
        if not move(goal,roll): raise ServoError('Centerline waypoint not confirmed')
        path+=length; result=read()
        # One sample just outside the gate after capture can be transient.
        # Reobserve the same locked book at most twice; never issue extra motion
        # or enlarge the tolerance to make a failing pose pass.
        for retry in range(2):
            position_error=float(np.linalg.norm(result[1]-goal));roll_error=abs(result[2]-roll)
            if position_error<=c['max_pose_error_mm'] and roll_error<=c['max_orientation_error_deg']:break
            log(dict(event='centerline_arrival_recheck',attempt=retry+1,position_error_mm=position_error,roll_error_deg=float(roll_error)))
            result=read()
        position_error=float(np.linalg.norm(result[1]-goal));roll_error=abs(result[2]-roll)
        if position_error>c['max_pose_error_mm'] or roll_error>c['max_orientation_error_deg']:
            diagnostic=dict(event='centerline_waypoint_error',target_mm=goal.tolist(),actual_mm=result[1].tolist(),
                delta_mm=(result[1]-goal).tolist(),position_error_mm=position_error,position_limit_mm=c['max_pose_error_mm'],
                target_roll_deg=float(roll),actual_roll_deg=float(result[2]),roll_error_deg=float(roll_error),
                roll_limit_deg=c['max_orientation_error_deg'])
            log(diagnostic)
            raise ServoError(f'Centerline measured waypoint error: position={position_error:.4f}mm '
                f'(limit={c["max_pose_error_mm"]:.4f}mm), roll={roll_error:.4f}deg '
                f'(limit={c["max_orientation_error_deg"]:.4f}deg), '
                f'target_mm={np.round(goal,4).tolist()}, actual_mm={np.round(result[1],4).tolist()}')
        log(dict(event='centerline_step',elapsed_seconds=clock()-began,path_mm=path))
        return result
    f,q,r,s=read(); origin=q.copy()
    if not np.isfinite([target_u,goal_x]).all() or not -c['x_tolerance_mm']<=goal_x-q[0]<=c['max_forward_mm']:
        raise ServoError('Centerline endpoint exceeds forward budget')
    tolerance=np.array([c['tolerance_px'],c['angle_tolerance_deg']]); jac=None
    for iteration in range(c['max_iterations']+c['stable_frames']):
        check(); aligned=bool((np.abs(f)<=tolerance).all()); remaining=goal_x-q[0]
        if aligned and abs(remaining)<=c['x_tolerance_mm']:
            stable+=1
            if stable>=c['stable_frames']:
                result=dict(converged=True,error_px=abs(float(f[0])),angle_error_deg=abs(float(f[1])),actual_x_mm=float(q[0]),path_mm=path)
                log(dict(event='converged_distance',**result)); return result
            f,q,r,s=read(); continue
        stable=0
        if iteration>=c['max_iterations']: break
        if jac is None and aligned:
            # Already aligned: no need to identify a correction model yet.
            goal=q.copy();budget=np.sqrt(max(0.,c['max_step_mm']**2-(origin[2]-q[2])**2))
            if budget<.03:raise ServoError('No translation budget after Z drift')
            goal[0]+=min(remaining,budget)
            f,q,r,s=go(goal,r,q,r);continue
        if jac is None:
            # Use actual displacement, including cross-axis arrival errors.
            # Two independent probes, no wasteful return-to-origin movement.
            features=[]; displacements=[]
            for axis in (0,1):
                old_f,old_q,old_r=f.copy(),q.copy(),r
                goal=q.copy(); roll=r
                if axis==0: goal[1]+=c['probe_mm']
                else: roll+=c['probe_roll_deg'] if r+c['probe_roll_deg']<=c['max_roll_deg'] else -c['probe_roll_deg']
                f,q,r,s=go(goal,roll,q,r)
                features.append(f-old_f); displacements.append([q[1]-old_q[1],r-old_r])
            delta=np.array(displacements).T
            if np.min(np.linalg.svd(delta,compute_uv=False))<.5: raise ServoError('Centerline probes did not move enough')
            jac=np.array(features).T @ np.linalg.inv(delta); reference_scale=s
            weighted=jac/tolerance[:,None]
            if not np.isfinite(jac).all() or np.linalg.cond(weighted)>40 or abs(jac[0,0])<.1 or abs(jac[1,1])<.1:
                raise ServoError('Centerline Y/roll response is not controllable')
            log(dict(event='jacobian_y_roll',matrix=jac.tolist())); continue
        # Translational sensitivity grows with image scale; angular sensitivity
        # is approximately scale-independent for this short approach.
        effective=jac.copy(); effective[:,0]*=s/reference_scale
        a=effective/tolerance[:,None]; b=f/tolerance
        correction=-c['gain']*np.linalg.solve(a.T@a+.02*np.eye(2),a.T@b)
        dz=origin[2]-q[2]; budget=np.sqrt(max(0.,c['max_step_mm']**2-dz**2))
        if budget<.03: raise ServoError('No translation budget after Z drift')
        factor=max(1.,abs(correction[0])/budget,abs(correction[1])/c['max_roll_step_deg'])
        correction/=factor
        correction=actionable_correction(correction,effective,f,tolerance,budget,c)
        goal=q.copy(); goal[1]+=correction[0]; roll=r+correction[1]
        if aligned and remaining>c['x_tolerance_mm']:
            goal[0]+=min(remaining,np.sqrt(max(0.,budget**2-correction[0]**2)))
        old_f,old_q,old_r=f.copy(),q.copy(),r
        f,q,r,s=go(goal,roll,q,r)
        if abs(q[0]-old_q[0])<c['max_pose_error_mm']:
            residual=f-old_f-effective@np.array([q[1]-old_q[1],r-old_r])
            if abs(residual[0])>6 or abs(residual[1])>2:
                raise ServoError('Centerline measured response disagrees with Y/roll model')
    raise ServoError('Centerline did not converge; grasp cancelled')
