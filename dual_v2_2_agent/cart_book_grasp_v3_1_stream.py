"""Bounded rolling Cartesian targets, with no intermediate arrival waits.

The dedicated adapter follows visual targets with CPU differential joint servo.
Only one unacknowledged target may exist; a target lease holds on lost updates.
"""
import json
import time
from pathlib import Path
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted

CONFIG=Path(__file__).with_name('cart_book_grasp_v3_1_stream.json')
DEFAULTS=dict(gain=.6,update_hz=10.,lookahead_mm=None,lookahead_roll_deg=2.,max_z_drift_mm=4.,pair_recovery_timeout_seconds=2.,feedback_max_age_seconds=.25,
              frame_max_age_seconds=.25,ack_timeout_seconds=3.,stall_seconds=8.,max_pose_pair_gap_seconds=.1,max_pose_pair_uncertainty_mm=3.,max_pose_pair_uncertainty_deg=1.)

def load_stream_config(path=CONFIG):
    c=json.loads(Path(path).read_text())
    limits=dict(gain=(.05,5.),update_hz=(1,20),lookahead_mm=(2.5,100.),lookahead_roll_deg=(.1,3),max_z_drift_mm=(2.1,4.),
                feedback_max_age_seconds=(.05,.5),frame_max_age_seconds=(.05,.5),
                ack_timeout_seconds=(.5,5),stall_seconds=(2,15),pair_recovery_timeout_seconds=(.2,3.),max_pose_pair_gap_seconds=(.02,.2),max_pose_pair_uncertainty_mm=(.1,3),max_pose_pair_uncertainty_deg=(.1,1))
    if set(c)!=set(DEFAULTS):raise ValueError('Invalid V3.1 stream config fields')
    for key,(lo,hi) in limits.items():
        if key=="lookahead_mm" and c[key] is None:continue
        if isinstance(c[key],bool) or not isinstance(c[key],(int,float)) or not lo<=c[key]<=hi:
            raise ValueError('Invalid V3.1 setting: '+key)
    return c


def translation_budget(c,s,path_mm=0.):
    """None removes the per-target cap; the remaining path budget stays finite."""
    remaining=float(c['max_path_mm'])-float(path_mm)
    configured=s['lookahead_mm']
    budget=remaining if configured is None else min(float(configured),remaining)
    if not np.isfinite(budget) or budget<=0:
        raise ServoError('V3.1 path budget exhausted')
    return budget


def run_stream(observe,read,blocking_move,send,ready,finish,goal_x,c,s,*,
               checkpoint=lambda:None,log=lambda e:None,clock=time.monotonic,sleep=time.sleep,started=None):
    """Injected callbacks allow a moving-arm simulator without hardware imports.

    State: q (mm), roll (deg). Observation additionally supplies error [px,deg],
    scale and fresh frame stamp. send is asynchronous; ready means planner ACK,
    not arrival. Blocking moves are restricted to two identification probes.
    """
    started=clock() if started is None else started
    origin=None;previous=None;path=0.;stamp=-np.inf;direct=False;fixed=None;jac=None;reference_scale=1.
    last_sent=-np.inf;last_progress=clock();progress_q=None;progress_r=0.;final_samples=0
    recovering_z=False;recovery_started=None
    advancing=False;aligned_samples=0;furthest_x=None
    tolerance=np.array([c['tolerance_px'],c['angle_tolerance_deg']])
    if s['max_z_drift_mm']<=c['max_pose_error_mm']:
        raise ServoError('V3.1 motion Z limit must exceed arrival tolerance')
    def sample(visual=True):
        nonlocal origin,previous,path,stamp,direct,fixed,progress_q,progress_r,last_progress
        nonlocal recovering_z,recovery_started
        checkpoint()
        if clock()-started>c['max_seconds']:raise ServoError('V3.1 total time budget exceeded')
        try:o=observe() if visual and not direct else read()
        except FeaturePointsExhausted as error:
            o=read()
            # Losing features must not hide an already out-of-bound height by
            # silently resetting the plane to the drifting pose.
            if origin is not None and abs(o['q'][2]-origin[2])>s['max_z_drift_mm']:
                raise ServoError('V3.1 fixed Z drift before point-loss hold')
            direct=True;fixed=(o['q'].copy(),float(o['roll']))
            if origin is None:origin=o['q'].copy()
            fixed[0][2]=origin[2]
            log(dict(event='point_loss_direct_switch',remaining_points=error.count,minimum_points=error.minimum,
                     fixed_xyz_mm=fixed[0].tolist(),actual_xyz_mm=o['q'].tolist(),
                     fixed_roll_deg=float(o['roll']),fixed_angles_deg=o['angles'].tolist()))
            if not finish(fixed[0],o['roll']):raise ServoError('V3.1 point-loss hold not confirmed')
            o=read();fixed=(o['q'].copy(),float(o['roll']))
            fixed[0][2]=origin[2]  # keep the phase's plane, never the settled drift
        q=np.asarray(o['q'],float);r=float(o['roll'])
        if q.shape!=(3,) or not np.isfinite([*q,r]).all():raise ServoError('Invalid V3.1 pose')
        if origin is None:origin=q.copy()
        if previous is not None:path+=float(np.linalg.norm(q-previous))
        previous=q.copy()
        if path>c['max_path_mm']:raise ServoError('V3.1 measured path budget exceeded')
        if not origin[0]-c['max_pose_error_mm']<=q[0]<=goal_x+c['x_tolerance_mm']:
            raise ServoError('V3.1 X outside endpoint interval')
        reference_z=origin[2]
        z_error=float(q[2]-reference_z)
        diagnostic=dict(reference_z_mm=float(reference_z),actual_z_mm=float(q[2]),
                        z_error_mm=z_error,motion_limit_mm=s['max_z_drift_mm'],
                        arrival_tolerance_mm=c['max_pose_error_mm'])
        if abs(z_error)>s['max_z_drift_mm']:
            log(dict(event='z_drift_stop',**diagnostic))
            raise ServoError(f"V3.1 fixed Z drift: reference={reference_z:.3f}mm, actual={q[2]:.3f}mm, "
                             f"delta={z_error:+.3f}mm, motion_limit={s['max_z_drift_mm']:.3f}mm")
        if not recovering_z and abs(z_error)>c['max_pose_error_mm']:
            recovering_z=True;recovery_started=clock()
            log(dict(event='z_recovery_start',**diagnostic))
        elif recovering_z and abs(z_error)<=.75*c['max_pose_error_mm']:
            recovering_z=False;recovery_started=None
            log(dict(event='z_recovery_complete',**diagnostic))
        if recovering_z and clock()-recovery_started>s['stall_seconds']:
            raise ServoError(f'V3.1 Z recovery timed out: delta={z_error:+.3f}mm')
        if abs(r)>c['max_roll_deg']+c['max_orientation_error_deg']:raise ServoError('V3.1 roll bound')
        if direct:
            if abs(q[1]-fixed[0][1])>c['max_pose_error_mm'] or abs(r-fixed[1])>c['max_orientation_error_deg']:
                raise ServoError('V3.1 direct-mode pose drift')
        else:
            t=float(o['stamp'])
            if t<=stamp or not 0<=clock()-t<=s['frame_max_age_seconds']:raise ServoError('V3.1 stale image')
            stamp=t
            if not np.isfinite([*o['error'],o['scale']]).all() or not .1<=o['scale']<=20:
                raise ServoError('V3.1 invalid visual measurement')
        if progress_q is None or np.linalg.norm(q-progress_q)>.3 or abs(r-progress_r)>.2:
            progress_q=q.copy();progress_r=r;last_progress=clock()
        if clock()-last_progress>s['stall_seconds']:
            raise ServoError(f'V3.1 no measured progress: elapsed_s={clock()-last_progress:.2f}, '
                             f'actual_mm={np.round(q,3).tolist()}, goal_x_mm={goal_x:.3f}, '
                             f'reference_z_mm={origin[2]:.3f}, direct={direct}')
        return o
    o=sample()
    if not np.isfinite(goal_x) or not -c['x_tolerance_mm']<=goal_x-o['q'][0]<=c['max_forward_mm']:
        raise ServoError('V3.1 endpoint outside approach budget')
    # Identify with actual settled displacements before permitting moving-camera feedback.
    if not direct:
        df=[];dq=[]
        for axis in (0,1):
            old=o;goal=old['q'].copy();roll=float(old['roll'])
            goal[2]=origin[2]
            if axis==0:goal[1]+=c['probe_mm']
            else:roll+=c['probe_roll_deg'] if roll+c['probe_roll_deg']<=c['max_roll_deg'] else -c['probe_roll_deg']
            if path+np.linalg.norm(goal-old['q'])>c['max_path_mm']:raise ServoError('V3.1 probe path budget')
            if not blocking_move(goal,roll):raise ServoError('V3.1 probe not confirmed')
            o=sample()
            if direct:break
            dq.append([o.get('image_q',o['q'])[1]-old.get('image_q',old['q'])[1],
                       o.get('image_roll',o['roll'])-old.get('image_roll',old['roll'])]);df.append(o['error']-old['error'])
        if not direct:
            d=np.array(dq).T
            if np.linalg.svd(d,compute_uv=False).min()<.5:raise ServoError('V3.1 probe displacement insufficient')
            jac=np.array(df).T@np.linalg.inv(d);reference_scale=o['scale']
            if not np.isfinite(jac).all() or np.linalg.cond(jac/tolerance[:,None])>40 or abs(jac[0,0])<.1 or abs(jac[1,1])<.1:
                raise ServoError('V3.1 invalid Y/roll response')
            log(dict(event='jacobian_y_roll',matrix=jac.tolist()))
    log(dict(event='stream_start',gain=s['gain'],update_hz=s['update_hz'],lookahead_mm=s['lookahead_mm']))
    # Time, travel, freshness and progress bound the loop; perception ticks are
    # not the old max_iterations waypoint count.
    while True:
        # Camera-time poses are for image pairing only. Rebase the command on
        # fresh feedback so a correction cannot ask to return to an old X.
        fresh=read();q=fresh['q'];r=fresh['roll'];remaining=goal_x-q[0]
        furthest_x=float(q[0]) if furthest_x is None else max(furthest_x,float(q[0]))
        lookahead=translation_budget(c,s,path)
        if not direct:
            effective=jac.copy();effective[:,0]*=o['scale']/reference_scale
            # Features belong to the paired image pose, not the later command
            # pose. Predict the small measured Y/roll change since that image.
            delta=np.array([q[1]-o.get('image_q',o['q'])[1],r-o.get('image_roll',o['roll'])])
            error=o['error']+effective@delta
        aligned=(not recovering_z) and (direct or bool((np.abs(error)<=tolerance).all()))
        aligned_samples=aligned_samples+1 if aligned else 0
        old_phase=advancing
        if recovering_z or (not direct and (np.abs(error)>1.5*tolerance).any()):
            advancing=False
        elif direct or aligned_samples>=c['stable_frames']:
            advancing=True
        if old_phase!=advancing:
            log(dict(event='stream_phase',phase='advance' if advancing else 'align',
                     measured_x_mm=float(q[0]),error=None if direct else error.tolist()))
        can_send=ready()
        if aligned and abs(remaining)<=c['x_tolerance_mm'] and can_send:
            final_samples+=1
            if final_samples>=c['stable_frames']:
                goal=(fixed[0].copy() if direct else q.copy());goal[0]=goal_x
                if not direct:goal[2]=origin[2]
                roll=fixed[1] if direct else r
                if np.linalg.norm(goal-q)>lookahead:raise ServoError('V3.1 final target beyond lookahead')
                if path+np.linalg.norm(goal-q)>c['max_path_mm']:raise ServoError('V3.1 final path budget')
                if not finish(goal,roll):raise ServoError('V3.1 final arrival not confirmed')
                # Validate a new image after final settling before allowing the gripper.
                o=sample()
                if recovering_z:
                    final_samples=0;continue
                if not direct and not (np.abs(o['error'])<=tolerance).all():
                    final_samples=0;continue
                return dict(converged=not direct,distance_complete=True,visual_alignment_verified=not direct,
                            completion_mode='direct_distance_after_point_loss' if direct else 'stream_visual',path_mm=path)
        else:final_samples=0
        if can_send and clock()-last_sent>=1/s['update_hz'] and not (aligned and abs(remaining)<=c['x_tolerance_mm']):
            goal=q.copy();goal[0]=min(goal_x,max(q[0],furthest_x))
            goal[2]=fixed[0][2] if direct else origin[2];roll=r
            if recovering_z:
                # Stop advancing X/Y/roll while returning toward the same TCP plane.
                goal[2]=q[2]+np.clip(goal[2]-q[2],-lookahead,lookahead)
            elif direct:
                goal[1]=fixed[0][1];roll=fixed[1]
            else:
                a=effective/tolerance[:,None]
                # Do not amplify already-acceptable residuals by gain=5; that
                # otherwise knocks alignment back out of tolerance every tick.
                b=np.where(np.abs(error)>tolerance,error,0.)/tolerance
                correction=-s['gain']*np.linalg.solve(a.T@a+.02*np.eye(2),a.T@b)
                correction/=max(1.,abs(correction[0])/lookahead,abs(correction[1])/s['lookahead_roll_deg'])
                goal[1]+=correction[0];roll+=correction[1]
            lateral=float(np.linalg.norm(goal[1:]-q[1:]))
            if lateral>lookahead:
                # Reserve the Z return first; scaling both axes changes the fixed plane.
                y_budget=np.sqrt(max(0.,lookahead**2-(goal[2]-q[2])**2))
                goal[1]=q[1]+np.clip(goal[1]-q[1],-y_budget,y_budget)
                lateral=float(np.linalg.norm(goal[1:]-q[1:]))
            if advancing and remaining>c['x_tolerance_mm']:
                goal[0]=q[0]+min(remaining,np.sqrt(max(0.,lookahead**2-lateral**2)))
            if abs(roll)>c['max_roll_deg']:raise ServoError('V3.1 commanded roll bound')
            if path+np.linalg.norm(goal-q)>c['max_path_mm']:
                raise ServoError(f'V3.1 path budget: used_mm={path:.1f}, '
                                 f'requested_mm={np.linalg.norm(goal-q):.1f}, limit_mm={c["max_path_mm"]:.1f}')
            if send(goal,roll) is not False:last_sent=clock()
        sleep(.005 if not direct else 1/s['update_hz'])
        o=sample()
