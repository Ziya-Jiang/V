"""V4.3 planner-paced lift with continuously tracked V3.4 spine features.

Import is device-free. Only execute_raise opens the wrist camera/commands robot.
"""
import json
import time
import threading
import uuid
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_centerline import roll_orientation, measured_roll
from book_top_v43_servo import load_config, run_raise, CONFIG
from book_top_v42_tracking import TrackingStream


def pose(robot,max_age=.25,*,state=None):
    """Use one snapshot; receipt freshness and pose freshness are distinct."""
    state=robot.get_current_state() if state is None else state;now=time.monotonic()
    if not state:raise ServoError('V4.3 missing robot feedback')
    for key in ('received_monotonic','pose_sample_monotonic'):
        stamp=float(state.get(key,float('nan')))
        if not np.isfinite(stamp) or not 0<=now-stamp<=max_age:
            raise ServoError('V4.3 stale feedback: '+key)
    status=getattr(state.get('curobo_status'),'value',state.get('curobo_status'))
    if status in (3,7,8):raise ServoError(f'V4.3 backend failure status={status}')
    arm=state['right_arm'];q=np.asarray(arm['position'],float)*1000
    quat=np.asarray(arm['orientation'],float)
    if q.shape!=(3,) or quat.shape!=(4,) or not np.isfinite([*q,*quat]).all():
        raise ServoError('V4.3 invalid TCP feedback')
    angles=Rotation.from_quat(quat[[1,2,3,0]]).as_euler('xyz',degrees=True)
    return q,angles


def angle_distance(a,b):
    return float(np.rad2deg((Rotation.from_euler('xyz',a,degrees=True).inv()*
                            Rotation.from_euler('xyz',b,degrees=True)).magnitude()))


def guarded_checkpoint(operator_check, servo_check):
    """Feedback reads re-enter OperatorRobot.operator_checkpoint.

    Always honour the original operator interrupt. Run extra servo checks once
    per call chain, per thread; reset even when a safety check raises.
    """
    local=threading.local()
    def check():
        operator_check()
        if getattr(local,'active',False):return
        local.active=True
        try:servo_check()
        finally:local.active=False
    return check


def execute_raise(robot,task,index,objects,execute,step5,state,config,*,settings_path=CONFIG):
    from book_top_v34_inherited import reload_distances,stage_offset
    from cart_book_grasp_v3_camera import WristCamera
    from cart_book_grasp_v3_2_selection import select_wrist_book
    from cart_book_grasp_v3_4_wrist_gpt import wrist_fallback
    from cart_book_grasp_v3_gpt_recovery import make_lock
    from book_top_v43_gpt_recovery import verify_tracking,VISUAL_FAILURES
    from types import SimpleNamespace
    c=load_config(settings_path);ctl=c['controller'];config=reload_distances(config,'raise_z')
    geometry=state.get('geometry');detector=state.get('wrist_detector')
    if geometry is None or detector is None:raise ServoError('V4.3 missing current-cycle perception')
    start,base=pose(robot,c['feedback_max_age_seconds'])
    offset=stage_offset('raise_z',start,Rotation.from_euler('xyz',base,degrees=True).as_matrix(),geometry,config)
    goal_z=float(start[2]+offset[2]);available=config['max_path_mm']-state['path']
    if offset[2]>available:raise ServoError('V4.3 insufficient remaining path budget')
    directory=Path(__file__).parent/'RobotRAM/output'/('v43_raise_'+uuid.uuid4().hex)
    directory.mkdir(parents=True)
    def log(event):
        event=dict(event,time_monotonic=time.monotonic())
        with (directory/'events.jsonl').open('a') as f:
            f.write(json.dumps(event,ensure_ascii=False,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x)+'\n')
        if event['event'] not in ('tracks','centerline_observation'):
            print('V4.3 抬升伺服: '+json.dumps(event,ensure_ascii=False,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x),flush=True)
    original_checkpoint=getattr(robot,'operator_checkpoint',lambda:None)
    had_checkpoint=hasattr(robot,'operator_checkpoint');old_wait=robot._wait_for_robot_stop
    camera=None;stream=None;began=time.monotonic();last_stamp=-np.inf;last_saved=-200
    observed_xyz=start.copy();recovery_count=0
    def servo_check():
        if time.monotonic()-began>ctl['max_seconds']:raise ServoError('V4.3 lift time budget')
        q,_=pose(robot,c['feedback_max_age_seconds'])
        if abs(q[0]-start[0])>ctl['max_pose_error_mm']:
            delta=float(q[0]-start[0])
            log(dict(event='fixed_x_drift',reference_x_mm=float(start[0]),actual_tcp_mm=q,
                     delta_x_mm=delta,limit_mm=ctl['max_pose_error_mm']))
            raise ServoError(f"V4.3 fixed X drift: reference={start[0]:.3f}mm, actual={q[0]:.3f}mm, "
                             f"delta={delta:.3f}mm, limit={ctl['max_pose_error_mm']:.3f}mm")
        if not start[2]-ctl['max_pose_error_mm']<=q[2]<=goal_z+ctl['max_pose_error_mm']:
            raise ServoError('V4.3 measured Z outside lift interval')
        if abs(q[1]-start[1])>c['max_lateral_mm']+ctl['max_pose_error_mm']:
            raise ServoError('V4.3 measured Y correction bound')
        if stream is not None:stream.check()
    checkpoint=guarded_checkpoint(original_checkpoint,servo_check)
    def selection_check():
        checkpoint();q,a=pose(robot,c['feedback_max_age_seconds'])
        if np.linalg.norm(q-start)>c['selection_hold_mm'] or angle_distance(a,base)>c['selection_hold_deg']:
            raise ServoError('V4.3 arm moved during wrist selection')
    arrival=dict(position_mm=ctl['max_pose_error_mm'],angle_deg=ctl['max_orientation_error_deg'])
    def wait(*args,**kwargs):
        kwargs.update(position_tolerance_m=arrival['position_mm']/1000,
                      orientation_tolerance_rad=np.deg2rad(arrival['angle_deg']),
                      require_fresh_stable=True,max_wait_time=60.,check_interval=.02)
        return old_wait(*args,**kwargs)
    try:
        log(dict(event='start',start_tcp_mm=start,goal_z_mm=goal_z,offset_mm=offset,
                 control='planner_paced_Z_progress_Y_roll_correction',settings=c,
                 assumption='After head Y alignment, wrist centre spine is the selected book'))
        camera=WristCamera();selection_check()
        first=camera.capture(selection_check)
        chosen,selection=select_wrist_book(first['rgb'],detector,directory=directory,
            checkpoint=selection_check,log=log,boundary_selector=wrist_fallback)
        selection_check()
        def select(rgb,diagnostics,**kw):return wrist_fallback.recover(rgb,diagnostics,detector=detector,**kw)
        # Reference identity survives even when no initial texture can be built.
        lock=SimpleNamespace(reference_rgb=first['rgb'],reference_mask=chosen['mask'])
        h,w=first['rgb'].shape[:2];target=(w-1)/2 if ctl['target_u'] is None else ctl['target_u']
        if not 0<=target<w:raise ServoError('V4.3 target column outside image')
        (directory/'selection.json').write_text(json.dumps(dict(selection,target_u=target,goal_z_mm=goal_z,
            calibration_sha256=camera.calibration_hash),ensure_ascii=False,indent=2))
        # One producer tracks every delivered frame, including during movements.
        robot.operator_checkpoint=checkpoint;robot._wait_for_robot_stop=wait
        def observe():
            nonlocal last_stamp,last_saved,observed_xyz
            checkpoint();before,angles_before=pose(robot,c['feedback_max_age_seconds'])
            # Controller observations are taken after arrival, using a new frame.
            frame,o,count=stream.after(max(last_stamp,time.monotonic()),c['frame_wait_seconds'],checkpoint)
            after,angles=pose(robot,c['feedback_max_age_seconds']);checkpoint()
            if np.linalg.norm(after-before)>ctl['max_pose_error_mm'] or angle_distance(angles_before,angles)>ctl['max_orientation_error_deg']:
                raise ServoError('V4.3 motion during settled image/pose observation')
            last_stamp=frame['captured_at'];observed_xyz=after.copy()
            if o.get('point_loss',False) or o.get('geometry_failure',False):
                return dict(o,xyz_mm=after,roll_deg=measured_roll(base,angles,ctl['max_orientation_error_deg']),captured_at=last_stamp)
            log(dict(event='tracks',count=count,inliers=int(o['inliers']),uv=o['uv'],
                     angle_deg=float(o['angle_deg']),frame_age_seconds=time.monotonic()-last_stamp))
            if count-last_saved>=ctl['snapshot_every']:
                image=frame['rgb'].copy();ends=np.rint(o['line']).astype(int)
                cv2.line(image,tuple(ends[0]),tuple(ends[1]),(0,220,100),2)
                cv2.line(image,(int(target),0),(int(target),h-1),(255,210,0),1)
                cv2.imwrite(str(directory/f'frame_{count:06d}.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
                if state.get('display') is not None:state['display'].safe('log_image','book/images/spine',image)
                last_saved=count
            return dict(o,xyz_mm=after,roll_deg=measured_roll(base,angles,ctl['max_orientation_error_deg']),captured_at=last_stamp)
        def recover_centerline(reason):
            nonlocal stream,lock,recovery_count,last_saved
            from book_top_v43_gpt_recovery import reanchor,TrackingReacquisitionFailed
            checkpoint()
            # Called only at a settled observation boundary. Stop the sole
            # camera reader, then confirm a stationary feedback window before
            # GPT. No new arm motion targets are sent during reacquisition.
            if stream is not None:
                stream.close();stream=None
            def read_feedback():
                raw=robot.get_current_state()
                q,a=pose(robot,c['feedback_max_age_seconds'],state=raw)
                return dict(q=q,angles=a,pose_sample_monotonic=raw['pose_sample_monotonic'])
            while recovery_count<3:
                checkpoint();recovery_count+=1
                log(dict(event='gpt_centerline_recovery_started',attempt=recovery_count,
                         reason=reason,new_motion_targets=False))
                try:
                    candidate=reanchor(lock,camera.capture,select,read_feedback,checkpoint,
                        directory/f'gpt_centerline_recovery_{recovery_count:02d}',began+ctl['max_seconds'],
                        reason=reason,max_age=ctl['max_frame_age_seconds'])
                except TrackingReacquisitionFailed as error:
                    log(dict(event='gpt_centerline_recovery_retry',attempt=recovery_count,reason=str(error)))
                    continue
                checkpoint();lock=candidate
                stream=TrackingStream(camera,lock,max_age_seconds=ctl['max_frame_age_seconds'],
                    allow_point_loss=True,allow_geometry_recovery=True).start()
                last_saved=-200
                log(dict(event='gpt_centerline_recovery_complete',attempt=recovery_count,verified_frames=3,
                         center_u=float(lock.last_u),angle_deg=float(lock.last_angle)))
                return
            raise ServoError('V4.3 GPT centerline recovery limit (3); tracking not restored')
        def move(q,roll):
            checkpoint();a=roll_orientation(base,roll)
            # Same physical X anchor in commands and the live drift monitor.
            q=np.asarray(q,float).copy();q[0]=start[0]
            correction_only=bool(abs(q[2]-observed_xyz[2])<1e-6)
            short_progress=bool(abs(q[2]-observed_xyz[2])<=ctl['max_pose_error_mm'])
            arrival.update(position_mm=c['correction_position_tolerance_mm'] if short_progress else ctl['max_pose_error_mm'],
                           angle_deg=c['correction_orientation_tolerance_deg'] if correction_only else ctl['max_orientation_error_deg'])
            log(dict(event='command',target_mm=q.tolist(),target_roll_deg=float(roll),
                     correction_only=correction_only,short_progress=short_progress,arrival=dict(arrival)))
            waypoint=dict(task,action='book_v42_raise_servo',target_position=q.tolist(),
                          target_rotation=np.asarray(a).tolist(),description='V4.3 lift Z; wrist centreline Y/roll alignment')
            if state.get('display') is not None:state['display'].safe('log_book_target',waypoint)
            return execute(robot,waypoint,index,objects)[1] is True
        try:
            candidate=make_lock(first['rgb'],chosen)
            verify_tracking(candidate,camera.capture,selection_check,first['captured_at'],
                            max_age=ctl['max_frame_age_seconds'])
        except VISUAL_FAILURES as error:
            log(dict(event='initial_tracking_recovery_required',reason=str(error)))
            recover_centerline(str(error))
        else:
            lock=candidate
            log(dict(event='initial_tracking_verified',verified_frames=3,
                     reference_age_seconds=time.monotonic()-first['captured_at']))
            stream=TrackingStream(camera,lock,max_age_seconds=ctl['max_frame_age_seconds'],
                allow_point_loss=True,allow_geometry_recovery=True).start()
        result=run_raise(observe,move,target,goal_z,c,path_available=available,checkpoint=checkpoint,log=log,recover=recover_centerline)
        _,angles=pose(robot,c['feedback_max_age_seconds'])
        state['angles']=angles.copy();state['path']+=result['path_mm'];state['distance_snapshot']=dict(config)
        log(dict(event='complete',**result))
        return task.get('object'),True
    except BaseException as error:
        log(dict(event='stopped',reason=f'{type(error).__name__}: {error}'))
        # Do not invoke the tracking checkpoint while sending a hold for its failure.
        robot.operator_checkpoint=original_checkpoint
        try:
            q,a=pose(robot,c['feedback_max_age_seconds'])
            robot.master_control(right_arm=[*q,*a],action='book_v3_1_hold',object_category='book',wait_for_completion=False)
            log(dict(event='hold_requested',confirmed=False))
        except Exception as hold_error:log(dict(event='hold_request_failed',reason=str(hold_error)))
        raise
    finally:
        robot._wait_for_robot_stop=old_wait
        if had_checkpoint:robot.operator_checkpoint=original_checkpoint
        elif hasattr(robot,'operator_checkpoint'):del robot.operator_checkpoint
        if stream is not None:
            try:stream.close()
            except Exception as error:log(dict(event='cleanup_warning',reason=str(error)))
        if camera is not None:
            try:camera.close()
            except Exception as error:log(dict(event='cleanup_warning',reason=str(error)))


def main(argv=None):
    import argparse
    import sys
    from functools import partial
    from book_top_v34_inherited import main as inherited_main
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--raise-servo-config',type=Path,default=CONFIG)
    args,rest=parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    c=load_config(args.raise_servo_config)
    if '--config' not in rest and not any(x.startswith('--config=') for x in rest):
        rest+=['--config',str(Path(__file__).with_name('cuart_book_grasp_v4_3_config.json'))]
    print(f"V4.3：抬升视觉闭环，X固定、Z每步最多{c['controller']['max_step_mm']:g}mm、Y/手腕旋转纠偏；gain={c['controller']['gain']:g}。分段到位等待；点数不足且历史充分时沿已记录方向接续；历史不足先停稳恢复同书跟踪，最多3次。",flush=True)
    from book_top_v43_reset import installed as reset_installed,load_config as reset_config
    reset=reset_config()
    print(f"V4.3：复位前沿+Z抬升{reset['lift_mm']:g}mm（每段≤{reset['step_mm']:g}mm），确认到位后再first_reset；已在准备位时跳过重复抬升。",flush=True)
    if '--execute' not in rest:
        return inherited_main(rest,raise_servo=partial(execute_raise,settings_path=args.raise_servo_config))
    import dual_v2_2_agent_pcl as agent
    import cart_book_grasp_v1_3 as loop_module
    with reset_installed(agent,loop_module):
        return inherited_main(rest,raise_servo=partial(execute_raise,settings_path=args.raise_servo_config))
