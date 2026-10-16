"""Right-handle ID 0: one head RGB-D Y move, then wrist Y/tool-roll alignment.

Optional forward to head RGB-D endpoint; no vertical/reset/gripper/lift. Hardware is
opened only by run(). Existing dedicated planner wire actions are reused.
"""
import json
import os
import sys
from pathlib import Path
import time
import cv2
import numpy as np
from cart_grasp import HandleMarker, annotate

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / 'cart_grasp_alignment.json'


def load_config(path=CONFIG):
    c = json.loads(Path(path).read_text())
    limits = dict(gain=(.05,.6), update_hz=(1,10), max_y_step_mm=(1,5),
        max_roll_step_deg=(.1,1), max_servo_y_mm=(5,60), max_roll_deg=(2,20),
        max_coarse_y_mm=(5,250), max_path_mm=(20,300), max_seconds=(10,300),
        tolerance_px=(1,5), tolerance_angle_deg=(.5,2), fixed_xz_mm=(1,3),
        loss_timeout_seconds=(.2,3))
    if set(c) != set(limits): raise ValueError('Unexpected/missing cart alignment settings')
    for k,(lo,hi) in limits.items():
        # null explicitly disables only the application-level coarse Y bound.
        if k=='max_coarse_y_mm' and c[k] is None:continue
        if isinstance(c[k],bool) or not isinstance(c[k],(int,float)) or not np.isfinite(c[k]) or not lo<=c[k]<=hi:
            raise ValueError('Invalid cart alignment setting: '+k)
    return c


def marker_error(result):
    """Canonical marker vertical center axis; center u only, no vertical goal."""
    if not result['tracking_valid']: raise MarkerLost(result['reason'])
    q = np.asarray(result['corners_px'],np.float32)
    H = cv2.getPerspectiveTransform(np.float32([[0,0],[1,0],[1,1],[0,1]]),q)
    ends = cv2.perspectiveTransform(np.float32([[[.5,0],[.5,1]]]),H)[0]
    axis = ends[1]-ends[0]
    angle = (np.degrees(np.arctan2(axis[0],axis[1]))+90)%180-90
    value = np.array([result['center_px'][0]-(result['image_size'][0]-1)/2,angle])
    if not np.isfinite(value).all(): raise ValueError('Invalid ArUco alignment measurement')
    return value


class MarkerLost(ValueError): pass


def head_point(result,depth,k,d,c2w):
    """Aligned depth is mm. Backproject raw RGB using its distortion model."""
    if not result['tracking_valid']: raise MarkerLost(result['reason'])
    if depth.ndim!=2 or list(depth.shape[::-1])!=result['image_size']:
        raise ValueError('Head RGB/depth dimensions differ')
    center=np.asarray(result['center_px']);q=np.asarray(result['corners_px'])
    mask=np.zeros(depth.shape,np.uint8)
    cv2.fillConvexPoly(mask,np.int32(np.rint(center+.45*(q-center))),255)
    z=np.asarray(depth[mask>0],float);z=z[np.isfinite(z)&(z>100)&(z<3000)]
    if len(z)<9 or np.percentile(z,90)-np.percentile(z,10)>30:
        raise ValueError('Marker interior depth missing or mixed with background')
    ray=cv2.undistortPoints(np.float64([[center]]),np.asarray(k,float),np.asarray(d,float))[0,0]
    camera=np.r_[ray,1.]*np.median(z)
    point=(np.asarray(c2w)@np.r_[camera,1.])[:3]
    if not np.isfinite(point).all(): raise ValueError('Invalid head-to-base point')
    return point


def coarse_target(origin,point,c):
    q=np.asarray(origin,float).copy();point=np.asarray(point,float)
    if q.shape!=(3,) or point.shape!=(3,) or not np.isfinite([*q,*point]).all():
        raise ValueError('Invalid coarse geometry')
    limit=c['max_coarse_y_mm']
    if limit is not None and abs(point[1]-q[1])>limit:
        raise ValueError('Coarse Y travel bound exceeded')
    q[1]=point[1]
    return q


def open_head_camera():
    from head_rgbd.client import CalibratedCamera
    path=os.environ.get('ROBOTRAM_ORBBEC_CALIBRATION_PATH',
        '/mnt/ssd/user/wx/vlm_demo/calibration/session_seed/base_to_camera/results/machine-readable results.json')
    source=json.loads(Path(path).read_text())['left']
    cal={k:np.asarray(source[k],float) for k in ('K','D','R','T')}
    cal.update(w=int(source['w']),h=int(source['h']),path=path)
    r=cal['R'];cal['T']=cal['T'].reshape(3)
    if (r.shape!=(3,3) or cal['K'].shape!=(3,3) or
        not all(np.isfinite(cal[k]).all() for k in ('K','D','R','T')) or
        not np.allclose(r.T@r,np.eye(3),atol=1e-3) or abs(np.linalg.det(r)-1)>1e-3):
        raise ValueError('Invalid calibrated head-to-base geometry')
    return CalibratedCamera(cal,warmup_frames=1,prefer_native=True)


def read_head(out):
    camera=open_head_camera();path=camera.camparas_file
    points=[];detector=HandleMarker(0,20.);deadline=time.monotonic()+4.;attempts=0;failure=None
    try:
        while len(points)<3 and time.monotonic()<deadline:
            rgb,depth=camera.capture_image();attempts+=1
            if not 0<=time.time()-camera.captured_unix_s<=.5:
                raise ValueError('Head frame stale')
            bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            result=detector.detect(bgr,camera.captured_unix_s)
            cv2.imwrite(str(out/'head_marker.png'),annotate(bgr,result))
            cv2.imwrite(str(out/'head_raw.png'),bgr)
            (out/'head_detection.json').write_text(json.dumps(dict(result,attempts=attempts),indent=2))
            try:
                point=head_point(result,depth,camera.K,camera.D,camera.c2w)
            except (MarkerLost,ValueError) as exc:
                failure=str(exc);points=[]
                continue
            points.append(point)
            if np.max(np.linalg.norm(np.asarray(points)-np.median(points,axis=0),axis=1))>5:
                failure='head marker depth/position unstable';points=[point]
        if len(points)<3:
            raise MarkerLost(f'Head camera ID 0 not reliable after {attempts} fresh frames; '
                             f'last_reason={failure}, detected_ids={result["detected_ids"]}; '
                             f'see {out}/head_raw.png and head_detection.json')
        points=np.asarray(points)
        if np.max(np.linalg.norm(points-np.median(points,axis=0),axis=1))>5:
            raise ValueError('Head marker depth/position not stable')
        cv2.imwrite(str(out/'head_marker.png'),annotate(bgr,result))
        point=np.median(points,axis=0)
        return dict(marker_id=0,black_square_mm=20.,point_base_mm=point.tolist(),
            samples_mm=points.tolist(),calibration=path,image_size=result['image_size'],
            captured_unix_s=camera.captured_unix_s)
    finally:camera.close()


def coarse_move(robot,target,base):
    """CuRobo once; scoped strict arrival, without changing the old planner."""
    old=robot._wait_for_robot_stop
    def wait(**kwargs):
        kwargs.update(position_tolerance_m=.003,orientation_tolerance_rad=np.deg2rad(1.),
                      require_fresh_stable=True,check_interval=.02)
        return old(**kwargs)
    robot._wait_for_robot_stop=wait
    try:
        if robot.master_control(right_arm=[*target,*base],action='cart_handle_coarse',
                                object_category='cart_handle') is not True:
            raise ValueError('Cart coarse move was not confirmed')
    finally:robot._wait_for_robot_stop=old


def align_loop(read,observe,send,ready,hold,origin,c,log,clock=time.monotonic,*,goal_x=None,approach=None):
    """Injectable control loop. observe(checkpoint) returns error + paired pose."""
    from servor import fit_jacobian,correction,angle_difference
    from cart_grasp_depth import ForwardStep
    if (goal_x is None)!=(approach is None):raise ValueError('Approach requires an explicit depth endpoint and settings')
    if goal_x is not None and (not np.isfinite(goal_x) or goal_x<origin[0]-approach['x_tolerance_mm'] or goal_x-origin[0]>approach['max_forward_mm']):
        raise ValueError('Invalid cart forward endpoint')
    forward=ForwardStep(approach) if approach else None
    deadline=clock()+c['max_seconds'];previous=None;path=0.;stamp=None
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
            if aligned and (settle_target is not None or near_endpoint):
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
                    result=dict(event='cart_aligned',error_px_deg=e.tolist(),tcp_mm=p['q'].tolist(),
                                tool_roll_deg=p['roll'],path_mm=float(path),gripper_changed=False,goal_tcp_x_mm=goal_x)
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
        if sys.exc_info()[0] is None:
            hold()
        else:
            # A failed cleanup must not replace the original fault diagnosis.
            try:hold()
            except Exception as exc:log(dict(event='cart_cleanup_hold_failed',reason=f'{type(exc).__name__}: {exc}'))


def run(a):
    from servor import robot_connection
    from cart_book_grasp_v3_camera import WristCamera
    from cart_book_grasp_v3_2_runtime import StreamTransport,capture_paired_frame
    from cart_book_grasp_v3_centerline import roll_orientation
    from cart_grasp_depth import load_approach,approach_goal
    wrist_only=getattr(a,'wrist_only',False)
    if wrist_only and getattr(a,'approach',False):raise ValueError('Wrist-only alignment cannot enable forward approach')
    approach=load_approach(a.approach_config) if getattr(a,'approach',False) else None
    c=load_config(a.config)
    if approach:c=dict(c,max_path_mm=approach['max_path_mm'])
    out=a.out or ROOT/'output'/('cart_align_'+str(time.time_ns()))
    out.mkdir(parents=True,exist_ok=False)
    with (out/'events.jsonl').open('w') as events:
        def log(e):
            events.write(json.dumps(e,ensure_ascii=False,allow_nan=False)+'\n');events.flush()
            if e['event']!='marker':print(json.dumps(e,ensure_ascii=False),flush=True)
        try:
            if wrist_only and not a.execute:
                camera=WristCamera()
                try:
                    frame=camera.capture();bgr=cv2.cvtColor(frame['rgb'],cv2.COLOR_RGB2BGR)
                    marker=HandleMarker(0,20.).detect(bgr,frame['captured_at'])
                    cv2.imwrite(str(out/'wrist_preview.png'),annotate(bgr,marker))
                    log(dict(event='marker',**marker))
                    error=marker_error(marker)
                    log(dict(event='wrist_alignment_preview',marker_id=0,error_px_deg=error.tolist(),
                             head_camera_used=False,robot_commands_sent=0,out=str(out)))
                    return 0
                finally:camera.close()
            with robot_connection() as (robot,base):
                settings=dict(feedback_max_age_seconds=.25,frame_max_age_seconds=.25,ack_timeout_seconds=3.,
                    max_pose_pair_gap_seconds=.1,max_pose_pair_uncertainty_mm=3.,max_pose_pair_uncertainty_deg=1.,
                    pair_recovery_timeout_seconds=2.)
                transport=StreamTransport(robot,base,dict(max_orientation_error_deg=1.),settings,log=log)
                initial=transport.read()['q'].copy()
                head=None
                if wrist_only:
                    target=initial.copy()
                else:
                    head=read_head(out);current=transport.read()
                    if np.linalg.norm(current['q']-initial)>.5 or abs(current['roll'])>.25:raise ValueError('Robot moved during head observation')
                    target=coarse_target(initial,head['point_base_mm'],c)
                endpoint=approach_goal(head,initial,base,approach) if approach else None
                plan=dict(event='cart_wrist_only_plan' if wrist_only else 'cart_coarse_plan',head=head,head_camera_used=not wrist_only,start_mm=initial.tolist(),target_mm=target.tolist(),
                    arm='right',marker_id=0,axes='Y/tool-roll' if wrist_only else ('Y then X/Y/tool-roll' if approach else 'Y then Y/tool-roll'),execute=a.execute,
                    coarse_arrival_mm=3.,approach=endpoint,no_forward=approach is None,no_closure=True)
                (out/'plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2))
                log(plan)
                if not a.execute:return 0
                camera=None
                try:
                    # Verify wrist calibration/device setup before the coarse command.
                    camera=WristCamera();camera.capture()
                    # The head measurement cannot silently age during wrist startup.
                    if not wrist_only and time.time()-head['captured_unix_s']>1.:
                        head=read_head(out);current=transport.read()
                        if np.linalg.norm(current['q']-initial)>.5 or abs(current['roll'])>.25:raise ValueError('Robot moved before coarse motion')
                        target=coarse_target(initial,head['point_base_mm'],c)
                        endpoint=approach_goal(head,initial,base,approach) if approach else None
                        plan.update(head=head,target_mm=target.tolist(),approach=endpoint)
                        (out/'plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2));log(plan)
                    if not wrist_only and abs(target[1]-initial[1])>1:coarse_move(robot,target,base)
                    coarse=transport.read()
                    if wrist_only:
                        if np.linalg.norm(coarse['q']-initial)>.5 or abs(coarse['roll'])>.25:
                            raise ValueError('Robot moved during wrist startup')
                        log(dict(event='head_and_coarse_skipped',actual_mm=coarse['q'].tolist()))
                    else:
                        if np.linalg.norm(coarse['q']-target)>3:raise ValueError('Measured coarse endpoint outside 3 mm')
                        log(dict(event='cart_coarse_confirmed',actual_mm=coarse['q'].tolist()))
                    # Keep original X/Z, not accumulated measured height drift.
                    origin=coarse['q'].copy();origin[[0,2]]=initial[[0,2]]
                    detector=HandleMarker(0,20.);last_frame=-np.inf;count=0
                    deadline=time.monotonic()+c['max_seconds']
                    def observe(checkpoint):
                        nonlocal last_frame,count
                        frame,paired,_,_,_=capture_paired_frame(transport,camera,checkpoint,log,last_frame,
                            clock=time.monotonic,sleep=time.sleep,total_deadline=deadline)
                        last_frame=frame['captured_at'];bgr=cv2.cvtColor(frame['rgb'],cv2.COLOR_RGB2BGR)
                        marker=detector.detect(bgr,last_frame)
                        checkpoint()
                        if time.monotonic()-last_frame>.25:raise ValueError('ArUco frame stale after detection')
                        log(dict(event='marker',**marker))
                        if count%200==0:cv2.imwrite(str(out/f'wrist_{count:06d}.png'),annotate(bgr,marker))
                        count+=1
                        if marker['tracking_valid']:paired['marker_scale_px']=float(np.sqrt(marker['area_px2']))
                        return marker_error(marker),paired
                    def send(q,roll,blocking):
                        robot._v3_servo_position_tolerance_mm=1. if blocking else 3.
                        action='book_v3_xyz_servo' if blocking else 'book_v3_1_stream'
                        if robot.master_control(right_arm=[*q,*roll_orientation(base,roll)],action=action,
                                object_category='book',wait_for_completion=blocking) is not True:
                            raise ValueError('Planner rejected cart alignment target')
                        if not blocking:
                            transport.sent_at=time.monotonic()
                            transport.pending=np.asarray(robot.target_command.arm_tool_command,float).copy()
                        log(dict(event='cart_control',tcp_mm=q.tolist(),roll_deg=float(roll),blocking=blocking))
                    align_loop(transport.read,observe,send,transport.ready,transport.abort,origin,c,log,
                               goal_x=endpoint['goal_tcp_x_mm'] if endpoint else None,approach=approach)
                finally:
                    try:transport.abort()
                    finally:
                        if camera:camera.close()
        except (Exception,KeyboardInterrupt) as exc:
            log(dict(event='stopped',reason=f'{type(exc).__name__}: {exc}'));return 1
    return 0
