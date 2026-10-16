"""Planner-paced CoTracker approach with V1_3's depth-derived X endpoint."""
import json
import time
import uuid
from pathlib import Path
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted
from cart_book_grasp_v3_camera import WristCamera,stationary_guard
from cart_book_grasp_v3_approach import wait_for_coarse_pose
from cart_book_grasp_v3_distance_servo import run_distance_servo
from cart_book_grasp_v3_cotracker_points import PointLock,draw_points
from cart_book_grasp_v3_wrist_center import select_center_book


def execute_cotracker_approach(robot,task,index,objects,step5,execute,config,*,model,wrist_detector,
        retreat_context=None,display=None,camera=None,output_root=None,column_mode=False,stream_config=None,wrist_boundary_selector=None,**kwargs):
    from cart_book_grasp_v1_3_motion import _right_tool_mm_euler,_load_cart_plus_parameters
    from cart_book_grasp_v1_3_depth import forward_parameters
    from cart_book_grasp_v1_3_visualization import forward_targets
    from cart_book_grasp_v3_motion import orientation_error
    if (model is None and not column_mode) or wrist_detector is None:raise ServoError('Tracker/model or wrist book detector unavailable')
    line_mode=column_mode and config['mode']=='centerline_distance'
    if line_mode:
        from cart_book_grasp_v3_centerline import CenterlineLock,run_centerline_servo,roll_orientation,measured_roll
    label='V3 中心线' if line_mode else 'V3 中心列' if column_mode else 'V3 CoTracker'
    owns_camera=camera is None
    camera=WristCamera() if owns_camera else camera
    checkpoint=getattr(robot,'operator_checkpoint',lambda:None)
    sleep=getattr(robot,'operator_sleep',time.sleep)
    initial=_right_tool_mm_euler(robot,step5)
    if initial is None:raise ServoError('Missing measured TCP before CoTracker approach')
    start,rotation=initial
    commanded_rotation=rotation.copy(); measured_roll_deg=0.
    parameters=forward_parameters(start,rotation,task.get('book_surface_x_mm'),_load_cart_plus_parameters())
    goal_x=parameters['grasp_tcp_x_mm']  # Frozen once; same endpoint as V1_3.
    context=dict(start_x_mm=float(start[0]),forward_complete=False,released=False,withdrawn=False)
    if retreat_context is not None:retreat_context[task.get('object')]=context
    targets=forward_targets(start,parameters)
    if len(targets)>1:
        checkpoint()
        waypoint=dict(task,target_position=targets[0].tolist(),target_rotation=rotation.tolist(),
            book_forward_index=1,book_forward_count=len(targets),description=('Coarse forward; book centerline alignment, fixed Z, V1_3 X endpoint follows' if column_mode else 'Coarse forward; CoTracker Y/Z + V1_3 X endpoint follows'))
        result=execute(robot,waypoint,index,objects,**kwargs)
        if not result[1]:return result
        wait_for_coarse_pose(lambda:_right_tool_mm_euler(robot,step5),targets[0],rotation,config,sleep=sleep,checkpoint=checkpoint)
    directory=(Path(output_root) if output_root else Path(__file__).parent/'RobotRAM/output')/(('v3_column_' if column_mode else 'v3_cotracker_')+uuid.uuid4().hex)
    directory.mkdir(parents=True);frame_index=0
    approach_started=time.monotonic();command_path=0.
    def log(event):
        with (directory/'events.jsonl').open('a') as f:f.write(json.dumps(dict(event,time_monotonic=time.monotonic()))+'\n')
        if event['event'] in ('start','jacobian_y','jacobian_yz','jacobian_y_roll','converged_distance','point_loss_direct_switch','direct_distance_start','direct_distance_complete','wrist_selection','stopped'):
            print(label+': '+json.dumps(event,ensure_ascii=False),flush=True)
    def pose():
        nonlocal measured_roll_deg
        value=_right_tool_mm_euler(robot,step5)
        if value is None:raise ServoError('Lost measured TCP during CoTracker approach')
        q,angles=value
        if orientation_error(angles,commanded_rotation)>config['max_orientation_error_deg']:raise ServoError('Unexpected tool rotation during CoTracker approach')
        if line_mode: measured_roll_deg=measured_roll(rotation,angles,config['max_orientation_error_deg'])
        return q
    def capture():
        checkpoint();sleep(config['settle_seconds'])
        guard=stationary_guard(robot,camera.config['max_joint_drift_deg'])
        frame=camera.capture(checkpoint);guard();pose()
        return frame,guard
    log(dict(event='start',goal_tcp_x_mm=goal_x,parameters=parameters,
        mode=config['mode'] if column_mode else 'cotracker_distance',
        tracker='feature_centerline_with_distance_fallback' if line_mode else 'legacy_points',
        assumption='After V1_3 Y alignment, wrist center book is the head-depth target',head_wrist_matching=False,final_reference_used=False))
    try:
        first,guard=capture()
        q=pose()
        if not -config['x_tolerance_mm']<=goal_x-q[0]<=config['max_forward_mm']:
            raise ServoError('V1_3 endpoint exceeds remaining CoTracker travel budget')
        cv2.imwrite(str(directory/'initial_wrist.png'),cv2.cvtColor(first['rgb'],cv2.COLOR_RGB2BGR))
        if line_mode and stream_config is not None and stream_config.get('runtime_version')=='3.2':
            from cart_book_grasp_v3_2_selection import select_wrist_book
            chosen,selection=select_wrist_book(first['rgb'],wrist_detector,
                                               directory=directory,checkpoint=checkpoint,log=log,
                                               boundary_selector=wrist_boundary_selector)
        else:
            chosen,selection=select_center_book(first['rgb'],wrist_detector(first['rgb']),horizontal_only=column_mode,centerline=line_mode)
        guard();checkpoint()
        if line_mode:
            from cart_book_grasp_v3_gpt_recovery import initialize_tracking
            selector=getattr(wrist_boundary_selector,'recover',None)
            def initial_select(rgb,diagnostics,**kw):
                return selector(rgb,diagnostics,detector=wrist_detector,**kw)
            def initial_check():
                guard();checkpoint()
                if time.monotonic()-approach_started>config['max_seconds']:
                    raise ServoError('Initial GPT recovery exceeded approach time budget')
            lock=initialize_tracking(first,chosen,initial_select if callable(selector) else None,
                lambda:capture()[0],initial_check,directory/'initial_gpt_recovery',factory=CenterlineLock,
                max_frame_age_seconds=(stream_config or {}).get('frame_max_age_seconds',config['max_frame_age_seconds']))
            if isinstance(getattr(lock,'selection_override',None),dict):
                chosen=lock.selection_override
                selection['initialization_recovery']='initial_gpt_recovery/initialization.json'
        elif column_mode:
            from cart_book_grasp_v3_column import ColumnLock,run_column_servo
            lock=ColumnLock(first['rgb'],chosen['mask'])
        else:lock=PointLock(model,first['rgb'],chosen['mask'],config)
        h,w=first['rgb'].shape[:2]
        if column_mode:target=np.array([config['target_u'] if config['target_u'] is not None else (w-1)/2,(h-1)/2],float)
        else:target=np.array(config['target_uv'] if config['target_uv'] is not None else [(w-1)/2,(h-1)/2],float)
        if not ((target>=[0,0]).all() and (target<[w,h]).all()):raise ServoError('Alignment target pixel outside wrist image')
        selection.update(initial_anchor=lock.anchor.tolist(),target_uv=target.tolist(),
                         goal_tcp_x_mm=goal_x,coordinate_system='undistorted_K',calibration_sha256=camera.calibration_hash)
        selection.update(initial_points=lock.reference.tolist())
        if line_mode:selection.update(tracking_method='feature_centerline',minimum_points=6,on_point_loss='direct_distance')
        (directory/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
        cv2.imwrite(str(directory/'selected_mask.png'),np.uint8(chosen['mask'])*255)
        tracking_label=f'{len(lock.reference)} 个特征点；少于 6 点后保持姿态直接前伸' if line_mode else f'{len(lock.reference)} 点'
        print(f'{label} 已锁定中心书：{tracking_label}；目标 TCP X={goal_x:.3f} mm；对准像素={target.tolist()}；输出 {directory}',flush=True)
        def observe():
            nonlocal frame_index
            observe_started=time.monotonic()
            frame,guard=capture();capture_seconds=time.monotonic()-observe_started
            try:observation=lock.observe(frame['rgb'])
            except BaseException as error:
                cv2.imwrite(str(directory/'failed_wrist.png'),cv2.cvtColor(frame['rgb'],cv2.COLOR_RGB2BGR))
                if isinstance(error,FeaturePointsExhausted):
                    # Do not turn camera/pose/scene-motion failures into fallback.
                    guard();pose();checkpoint()
                    if not 0<=time.monotonic()-frame['captured_at']<=config['max_frame_age_seconds']:
                        raise ServoError('Stale wrist frame at point-loss transition') from error
                raise
            guard();xyz=pose();checkpoint()
            snapshot=not line_mode or frame_index % config['snapshot_every']==0
            if snapshot:
                image=draw_points(frame['rgb'],observation,target)
                if column_mode:
                    cv2.line(image,(int(target[0]),0),(int(target[0]),h-1),(255,210,0),1)
                    if line_mode:
                        ends=np.rint(observation['line']).astype(int)
                        cv2.line(image,tuple(ends[0]),tuple(ends[1]),(0,220,100),2)
                    else:cv2.line(image,(int(observation['uv'][0]),0),(int(observation['uv'][0]),h-1),(0,220,100),1)
                if snapshot:cv2.imwrite(str(directory/f'frame_{frame_index:03d}.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
                if display is not None:
                    display.safe('log_image','book/images/spine',image)
                    display.safe('flush_book')
            frame_index+=1
            log(dict(event='tracks',inliers=observation['inliers'],scale=observation['scale'],
                     inference_seconds=observation['inference_seconds'],capture_seconds=capture_seconds,
                     observation_seconds=time.monotonic()-observe_started))
            result=dict(uv=observation['uv'],scale=observation['scale'],xyz_mm=xyz,captured_at=frame['captured_at'])
            if line_mode:result.update(angle_deg=observation['angle_deg'],roll_deg=measured_roll_deg)
            return result
        def execute_waypoint(q,angles,description):
            nonlocal commanded_rotation,command_path
            checkpoint();current=pose();length=float(np.linalg.norm(q-current))
            if line_mode and command_path+length>config['max_path_mm']:
                raise ServoError('Cumulative approach path budget exceeded')
            commanded_rotation=angles.copy()
            waypoint=dict(task,action='book_v3_xyz_servo',target_position=q.tolist(),target_rotation=angles.tolist(),description=description)
            previous=getattr(robot,'_v3_servo_position_tolerance_mm',None)
            robot._v3_servo_position_tolerance_mm=float(config['max_pose_error_mm'])
            try:success=bool(execute(robot,waypoint,index,objects,**kwargs)[1])
            finally:
                if previous is None:del robot._v3_servo_position_tolerance_mm
                else:robot._v3_servo_position_tolerance_mm=previous
            if success:command_path+=length
            return success
        def move(q,roll=None):
            angles=roll_orientation(rotation,roll) if line_mode else rotation
            return execute_waypoint(q,angles,'Feature centerline alignment with V1_3 X stop' if line_mode else 'Visual approach with V1_3 X stop')
        def direct_pose():
            from scipy.spatial.transform import Rotation
            state=robot.get_current_state()
            try:
                arm=state['right_arm'];quaternion=np.asarray(arm['orientation'],float)
                return dict(xyz_mm=np.asarray(arm['position'],float)*1000,
                    angles_deg=Rotation.from_quat(np.r_[quaternion[1:],quaternion[0]]).as_euler('xyz',degrees=True),
                    received_monotonic=state['received_monotonic'])
            except (KeyError,TypeError,ValueError,IndexError) as error:
                raise ServoError('Invalid feedback for direct-distance mode') from error
        if stream_config is not None:
            if not line_mode:raise ServoError('V3.1 requires centerline mode')
            recovery_kwargs={}
            if stream_config.get('runtime_version')=='3.2':
                from cart_book_grasp_v3_2_runtime import run_stream_session
                if callable(getattr(wrist_boundary_selector,'recover',None)):
                    def recover_geometry(rgb,diagnostics,**kw):
                        return wrist_boundary_selector.recover(rgb,diagnostics,detector=wrist_detector,**kw)
                    recovery_kwargs['geometry_recovery']=recover_geometry
            else:
                from cart_book_grasp_v3_1_runtime import run_stream_session
            result=run_stream_session(robot,camera,lock,target,goal_x,rotation,config,stream_config,
                move,checkpoint,log,directory,display,started=approach_started,**recovery_kwargs)
            commanded_rotation=np.asarray(result['final_angles_deg'])
            context['completion_mode']=result['completion_mode']
        elif line_mode:
            try:result=run_centerline_servo(observe,move,target[0],goal_x,config,checkpoint=checkpoint,log=log)
            except FeaturePointsExhausted as error:
                from cart_book_grasp_v3_direct_distance import run_direct_distance
                log(dict(event='point_loss_direct_switch',remaining_points=error.count,minimum_points=error.minimum,
                    goal_x_mm=goal_x,visual_alignment_verified=False))
                context['completion_mode']='direct_distance_after_point_loss'
                result=run_direct_distance(direct_pose,
                    lambda q,angles:execute_waypoint(q,angles,'Direct X to V1_3 endpoint after point loss; hold Y/Z/orientation'),
                    goal_x,config,path_used=command_path,started=approach_started,checkpoint=checkpoint,sleep=sleep,log=log)
        elif column_mode:result=run_column_servo(observe,move,target[0],goal_x,config,checkpoint=checkpoint,log=log)
        else:result=run_distance_servo(observe,move,target,goal_x,config,checkpoint=checkpoint,log=log)
        if result.get('converged') is not True and result.get('distance_complete') is not True:raise ServoError('Approach completion not verified')
        context.update(forward_complete=True,final_xyz_mm=pose().tolist(),visual_alignment_verified=result.get('converged') is True)
        if result.get('converged') is True:context['visual_final_xyz_mm']=context['final_xyz_mm']
        return task.get('object'),True
    except BaseException as error:
        log(dict(event='stopped',reason=f'{type(error).__name__}: {error}'));raise
    finally:
        if owns_camera:camera.close()
