"""V4.2-only planner-paced controller fork: preserve V3.4 behaviour.
Virtual coordinates are [physical Z, physical Y, physical X].
"""
import time
import numpy as np
from cart_book_grasp_v3_servo import ServoError,vector
from cart_book_grasp_v3_centerline import actionable_correction
from book_top_v42_bounded_correction import correction_at_roll_bound

def run_lift_centerline(observe, move, target_u, goal_x, c, *, checkpoint=lambda:None,
                         log=lambda e:None, clock=time.monotonic,recover=None):
    """2D image [u, line tilt] -> [base Y, local tool Z roll], fixed TCP Z."""
    start=clock(); stamp=-np.inf; origin=None; path=0.; stable=0; stagnant=0; bounded_stagnant=0
    history=[]; lost=False; continuation=None; reset_model=False
    tolerance=np.array([c['tolerance_px'],c['angle_tolerance_deg']])
    def check():
        checkpoint()
        if clock()-start > c['max_seconds']: raise ServoError('Centerline time budget exceeded')
    def reanchor(reason):
        nonlocal reset_model,lost,continuation,stable,stagnant,bounded_stagnant
        if recover is None:raise ServoError(reason)
        check();recover(reason);check()
        history.clear();lost=False;continuation=None;stable=0;stagnant=0;bounded_stagnant=0;reset_model=True
        log(dict(event='gpt_reference_reset',reason=reason,old_jacobian_discarded=True))
    def read():
        nonlocal stamp,lost
        check(); o=observe(); check()
        if o.get('geometry_failure',False):
            reanchor(o.get('reason','Centerline geometry failure'));o=observe();check()
            if o.get('geometry_failure',False) or o.get('point_loss',False):
                raise ServoError('V4.2 GPT reference failed immediately after recovery')
        q=vector(o['xyz_mm'],3,'measured TCP'); r=float(o['roll_deg'])
        point_loss=bool(o.get('point_loss',False))
        lost=lost or point_loss
        f=np.zeros(2) if lost else np.array([o['uv'][0]-target_u,o['angle_deg']],float)
        s=1. if lost else float(o['scale']); t=float(o['captured_at'])
        if not np.isfinite([*f,r,s,t]).all() or not .1<=s<=20 or t<=stamp or not 0<=clock()-t<=c['max_frame_age_seconds']:
            raise ServoError('Stale or invalid centerline observation')
        stamp=t
        if abs(r)>c['max_roll_deg']+c['max_orientation_error_deg']: raise ServoError('Wrist roll bound exceeded')
        if origin is not None:
            if q[0]<origin[0]-c['max_pose_error_mm'] or q[0]>goal_x+c['x_tolerance_mm']: raise ServoError('Centerline X outside interval')
            if abs(q[2]-origin[2])>c['max_pose_error_mm']: raise ServoError('Centerline measured Z drift exceeded')
        if lost:
            log(dict(event='history_pose_feedback',xyz_mm=q.tolist(),visual_valid=False))
            return f,q,r,s
        if (np.abs(f)<=tolerance).all():
            # One latest aligned pose per height band: pure corrections are
            # not new lift-direction evidence. Keep a short recent history.
            sample=(float(q[0]),float(q[1]))
            if history and abs(sample[0]-history[-1][0])<3.:history[-1]=sample
            else:history.append(sample)
            del history[:-8]
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
        check(); remaining=goal_x-q[0]
        if reset_model:jac=None;reset_model=False
        if lost:
            if continuation is None:
                slopes=[(b[1]-a[1])/(b[0]-a[0]) for i,a in enumerate(history)
                        for b in history[i+1:] if b[0]-a[0]>=5.]
                if not slopes or history[-1][0]-history[0][0]<10.:
                    raise ServoError('V4.2 point loss without sufficient aligned lift history (need 10mm span)')
                slope=float(np.median(slopes));end_y=q[1]+slope*max(0.,remaining)
                if not np.isfinite(slope):
                    raise ServoError('V4.2 historical direction is non-finite')
                if abs(end_y-origin[1])>c['history_max_lateral_mm']:
                    log(dict(event='history_direction_gpt_review_required',
                        slope_dy_dz=slope,remaining_z_mm=float(remaining),
                        predicted_y_mm=float(end_y),origin_y_mm=float(origin[1]),
                        predicted_y_offset_mm=float(end_y-origin[1]),
                        lateral_limit_mm=c['history_max_lateral_mm']))
                    reanchor('V4.2 historical direction endpoint exceeds Y bound: '
                        f'dY/dZ={slope:.6f}, remaining_Z={remaining:.3f}mm, '
                        f'predicted_Y={end_y:.3f}mm, origin_Y={origin[1]:.3f}mm, '
                        f'limit={c["history_max_lateral_mm"]:.3f}mm. '
                        'Recheck the same spine centerline; old direction may be biased. '
                        'The motion bound must remain unchanged.')
                    f,q,r,s=read()
                    if lost:raise ServoError('V4.2 GPT direction review did not restore visual tracking')
                    continue
                continuation=(q.copy(),r,slope)
                log(dict(event='point_loss_history_switch',slope_dy_dz=slope,
                         aligned_history_z_y_mm=history.copy(),anchor_mm=q.tolist(),
                         fixed_roll_deg=r,goal_z_mm=goal_x,visual_valid=False))
            anchor,fixed_roll,slope=continuation
            expected_y=anchor[1]+slope*(q[0]-anchor[0])
            if abs(q[1]-expected_y)>c['max_pose_error_mm'] or abs(r-fixed_roll)>c['max_orientation_error_deg']:
                raise ServoError('V4.2 historical direction pose drift')
            if abs(remaining)<=c['x_tolerance_mm']:
                stable+=1
                if stable>=c['stable_frames']:
                    result=dict(converged=True,visual_converged=False,mode='historical_direction',
                                actual_x_mm=float(q[0]),path_mm=path)
                    log(dict(event='completed_historical_lift',**result));return result
                f,q,r,s=read();continue
            if iteration>=c['max_iterations']:break
            stable=0
            # Reserve room for correcting measured drift back onto the stored
            # line, then spend the remaining Euclidean step budget along it.
            drift=np.hypot(expected_y-q[1],origin[2]-q[2])
            budget=c['max_step_mm']-drift
            if budget<=.2:raise ServoError('V4.2 no historical lift step budget')
            goal=q.copy();goal[0]+=min(remaining,budget/np.hypot(1.,slope))
            goal[1]=anchor[1]+slope*(goal[0]-anchor[0])
            previous=q.copy();f,q,r,s=go(goal,fixed_roll,q,r)
            stagnant=stagnant+1 if q[0]-previous[0]<.2 else 0
            if stagnant>=4:raise ServoError('V4.2 no measured historical lift progress')
            continue
        aligned=bool((np.abs(f)<=tolerance).all())
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
                if lost or reset_model:break
                features.append(f-old_f); displacements.append([q[1]-old_q[1],r-old_r])
            if lost or reset_model:continue
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
        factor=max(1.,abs(correction[0])/min(budget,c['probe_mm']),abs(correction[1])/c['max_roll_step_deg'])
        correction/=factor
        correction=actionable_correction(correction,effective,f,tolerance,budget,c)
        correction,correction_mode=correction_at_roll_bound(correction,effective,f,r,tolerance,budget,c)
        if correction_mode!='ordinary':
            predicted=f+effective@correction
            log(dict(event='roll_bounded_correction',mode=correction_mode,actual_roll_deg=r,
                     target_roll_deg=float(r+correction[1]),dy_mm=float(correction[0]),
                     error_px_deg=f.tolist(),predicted_error_px_deg=predicted.tolist()))
            if not aligned and np.linalg.norm(predicted/tolerance)>=np.linalg.norm(f/tolerance)-1e-6:
                reanchor('V4.2 bounded Y/roll correction cannot improve image error near roll limit')
                f,q,r,s=read();continue
        else:bounded_stagnant=0
        goal=q.copy(); goal[1]+=correction[0]; roll=r+correction[1]
        if correction_mode!='ordinary':roll=float(np.clip(roll,-c['max_roll_deg'],c['max_roll_deg']))
        if aligned and remaining>c['x_tolerance_mm']:
            goal[0]+=min(remaining,np.sqrt(max(0.,budget**2-correction[0]**2)))
        old_f,old_q,old_r=f.copy(),q.copy(),r
        commanded_progress=float(goal[0]-old_q[0])
        f,q,r,s=go(goal,roll,q,r)
        if lost or reset_model:continue
        # Lift displacement changes image geometry. A 2.8mm measured rise is
        # not a pure Y/roll correction merely because arrival tolerance is 3mm.
        pure_correction=(abs(commanded_progress)<1e-6 and abs(q[0]-old_q[0])<.5)
        if pure_correction:
            residual=f-old_f-effective@np.array([q[1]-old_q[1],r-old_r])
            if abs(residual[0])>6 or abs(residual[1])>2:
                log(dict(event='gpt_review_required',residual_px_deg=residual.tolist(),image_change=(f-old_f).tolist()))
                reanchor('Centerline measured response disagrees with Y/roll model')
                f,q,r,s=read();continue
        else:
            log(dict(event='lift_response',commanded_progress_mm=commanded_progress,
                     measured_progress_mm=float(q[0]-old_q[0]),
                     image_change=(f-old_f).tolist(),pure_y_roll_check=False))
        improvement=float(np.linalg.norm(old_f/tolerance)-np.linalg.norm(f/tolerance))
        if correction_mode!='ordinary' and pure_correction:
            bounded_stagnant=bounded_stagnant+1 if improvement<.05 else 0
            if bounded_stagnant>=4:
                reanchor('V4.2 bounded Y/roll correction did not converge; review same-book centerline')
                f,q,r,s=read();continue
        if abs(q[0]-old_q[0])<.2 and abs(q[1]-old_q[1])<.2 and abs(r-old_r)<.1 and improvement<.05:
            stagnant+=1
        else:stagnant=0
        if stagnant>=4:
            if correction_mode!='ordinary':
                reanchor('V4.2 bounded correction made no measured progress near roll limit')
                f,q,r,s=read();continue
            raise ServoError('V4.2 no measured correction progress; repeated near-identical targets stopped')
    raise ServoError('Centerline did not converge; grasp cancelled')
