"""V1_3 first forward waypoint, then final-image XYZ servo before grasp."""
from pathlib import Path
import json
import time
import uuid
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_camera import WristCamera, stationary_guard
from cart_book_grasp_v3_final_tracking import FinalImageTracker, draw_final_tracking
from cart_book_grasp_v3_tracking import acquire_selected_spine
from cart_book_grasp_v3_wrist_center import acquire_wrist_center, require_points_in_book, draw_selection, reanchor_wrist_book
from cart_book_grasp_v3_xyz_servo import run_xyz


def wait_for_coarse_pose(read_pose, target, rotation, config, *, sleep=time.sleep,
                         checkpoint=lambda:None, clock=time.monotonic):
    """Read-only settling after coarse motion; never command a correction.

    Require three in-tolerance samples within 0.5 mm / 0.2 degrees of the
    first sample in the window. Return the measured pose as the servo origin.
    """
    from cart_book_grasp_v3_motion import orientation_error
    checkpoint()
    sleep(config['coarse_settle_seconds'])
    deadline=clock()+config['coarse_feedback_timeout_seconds']
    anchor=None;stable=0;detail='No measured TCP feedback'
    while True:
        checkpoint()
        measured=read_pose()
        if measured is None:
            anchor=None;stable=0;detail='No measured TCP feedback'
        else:
            xyz,euler=measured
            delta=xyz-target
            error=float(np.linalg.norm(delta))
            angle=orientation_error(euler,rotation)
            inside=error<=config['coarse_pose_error_mm'] and angle<=config['max_orientation_error_deg']
            if not inside:
                anchor=None;stable=0
            elif (anchor is None or np.linalg.norm(xyz-anchor[0])>.5 or
                  orientation_error(euler,anchor[1])>.2):
                anchor=(xyz.copy(),euler.copy());stable=1
            else:
                stable+=1
            detail=(f'target_mm={np.round(target,3).tolist()}, '
                    f'actual_mm={np.round(xyz,3).tolist()}, '
                    f'delta_mm(actual-target)={np.round(delta,3).tolist()}, '
                    f'position_error={error:.3f}mm (limit={config["coarse_pose_error_mm"]:.3f}mm), '
                    f'orientation_error={angle:.3f}deg (limit={config["max_orientation_error_deg"]:.3f}deg), '
                    f'stable_samples={stable}/3')
            if stable>=3 and clock()<=deadline:
                print('V3 粗前伸到位检查通过：'+detail,flush=True)
                return xyz,euler
        if clock()>=deadline:
            raise ServoError('First-stage waypoint did not settle within tolerance: '+detail)
        sleep(min(.1,max(0.,deadline-clock())))


def execute_visual_approach(robot, task, index, objects, step5, execute, head_frame, config,
                            *, retreat_context=None, display=None, camera=None, output_root=None, wrist_detector=None, **kwargs):
    from cart_book_grasp_v1_3_motion import _right_tool_mm_euler, _load_cart_plus_parameters
    from cart_book_grasp_v1_3_depth import forward_parameters
    from cart_book_grasp_v1_3_visualization import forward_targets
    from cart_book_grasp_v3_motion import orientation_error
    center_mode=config.get('target_selection','head_identity')=='wrist_center'
    if head_frame is None and not center_mode:
        raise ServoError('No current V1_3 selected book; approach cancelled')
    if center_mode:
        if wrist_detector is None:raise ServoError('Wrist book detector is unavailable')
    else:
        from cart_book_grasp_v3_superpoint import get_backend
        get_backend()  # Verify dependencies/weights before the coarse movement.
    camera=WristCamera() if camera is None else camera
    tracker=FinalImageTracker.from_config(config,camera,Path(__file__).parent)
    initial=_right_tool_mm_euler(robot,step5)
    if initial is None:raise ServoError('No measured TCP before first forward stage')
    initial_xyz,rotation=initial
    parameters=forward_parameters(initial_xyz,rotation,task.get('book_surface_x_mm'),_load_cart_plus_parameters())
    targets=forward_targets(initial_xyz,parameters)
    context=dict(start_x_mm=float(initial_xyz[0]),forward_complete=False,released=False,withdrawn=False)
    if retreat_context is not None:retreat_context[task.get('object')]=context
    checkpoint=getattr(robot,'operator_checkpoint',lambda:None)
    # Keep only the first ORIGINAL waypoint. Never execute the remaining
    # distance-based waypoints after visual convergence.
    if len(targets)>1:
        checkpoint()
        waypoint=dict(task,target_position=targets[0].tolist(),target_rotation=rotation.tolist(),
                      book_forward_index=1,book_forward_count=len(targets),description='First coarse forward stage; visual stage follows')
        result=execute(robot,waypoint,index,objects,**kwargs)
        if not result[1]:return result
        stage2=wait_for_coarse_pose(lambda:_right_tool_mm_euler(robot,step5),targets[0],rotation,config,
            sleep=getattr(robot,'operator_sleep',time.sleep),checkpoint=checkpoint)
    else:
        stage2=_right_tool_mm_euler(robot,step5)
    if stage2 is None:raise ServoError('No TCP at second-stage observation pose')
    start,euler=stage2
    if orientation_error(euler,rotation)>config['max_orientation_error_deg']:
        raise ServoError('Tool orientation changed before the visual approach')
    c=dict(config)
    # Head-depth endpoint is a travel guard, not the visual stopping criterion.
    nominal_remaining=parameters['grasp_tcp_x_mm']-start[0]
    c['max_forward_mm']=min(c['max_forward_mm'],max(0.,nominal_remaining)+c['max_pose_error_mm'])
    if c['max_forward_mm']<c['probe_mm']:
        raise ServoError('Insufficient forward travel budget for initial visual probes')
    directory=(Path(output_root) if output_root else Path(__file__).parent/'RobotRAM'/'output')/('v3_xyz_'+uuid.uuid4().hex)
    directory.mkdir(parents=True);frame_index=0
    def log(event):
        event=dict(event,time_monotonic=time.monotonic())
        with (directory/'events.jsonl').open('a') as f:f.write(json.dumps(event)+'\n')
        if event['event'] in ('jacobian_xyz','converged_xyz','stopped'):
            print('V3 XYZ: '+json.dumps(event),flush=True)
    def pose():
        measured=_right_tool_mm_euler(robot,step5)
        if measured is None:raise ServoError('Lost measured TCP during visual approach')
        xyz,angles=measured
        if orientation_error(angles,euler)>c['max_orientation_error_deg']:
            raise ServoError('Unexpected rotation during XYZ servo')
        return xyz
    def capture():
        checkpoint();getattr(robot,'operator_sleep',time.sleep)(c['settle_seconds'])
        guard=stationary_guard(robot,camera.config['max_joint_drift_deg'])
        frame=camera.capture(checkpoint);guard();pose()
        return frame,guard
    locked_book=None
    def tracked_observation(rgb):
        nonlocal locked_book
        if center_mode:
            selection=locked_book.observe(rgb)
            observation=tracker.observe(rgb,search_mask=selection['mask'])
            require_points_in_book(observation,selection)
            locked_book=reanchor_wrist_book(rgb,selection)
            return observation
        return tracker.observe(rgb)
    def observe():
        nonlocal frame_index
        frame,guard=capture(); observation=tracked_observation(frame['rgb']);guard();xyz=pose()
        annotated=draw_final_tracking(frame['rgb'],observation,tracker.target)
        if not cv2.imwrite(str(directory/f'frame_{frame_index:03d}.png'),cv2.cvtColor(annotated,cv2.COLOR_RGB2BGR)):
            raise ServoError('Cannot write visual approach evidence')
        frame_index+=1
        log(dict(event='match',inliers=observation['inliers'],area_ratio=observation['area_ratio'],
                 reprojection_rms_px=observation['reprojection_rms_px']))
        if display is not None:
            display.safe('log_image','book/images/spine',annotated);display.safe('flush_book')
        return dict(points=observation['points'],xyz_mm=xyz,captured_at=frame['captured_at'])
    def move(xyz):
        checkpoint();pose()
        waypoint=dict(task,action='book_v3_xyz_servo',target_position=xyz.tolist(),target_rotation=euler.tolist(),
                      description='Second-stage XYZ final-image correction')
        return bool(execute(robot,waypoint,index,objects,**kwargs)[1])
    def match_head_book(first,guard):
        diagnostics=[]
        try:
            selected=acquire_selected_spine(head_frame,first['rgb'],diagnostics=diagnostics,backend='superpoint').observe(first['rgb'])
        finally:
            (directory/'head_wrist_matching.json').write_text(json.dumps(diagnostics,indent=2)+'\n')
            print('V3 头腕匹配：SuperPoint+LightGlue，至少 6 个内点；'+json.dumps(diagnostics,ensure_ascii=False),flush=True)
        final_match=tracker.observe(first['rgb']);guard()
        if any(cv2.pointPolygonTest(np.float32(selected['polygon']),tuple(map(float,p)),False)<0
               for p in final_match['points']):
            raise ServoError('Final taught book does not match the V1_3-selected book')
        return selected
    log(dict(event='start',observation_xyz_mm=start.tolist(),fixed_euler_deg=euler.tolist(),config=c))
    try:
        first,guard=capture()
        # Save acquisition inputs before matching: a failed acquisition used to
        # leave only an error string, without the images needed to reproduce it.
        inputs={'wrist_acquisition.png':cv2.cvtColor(first['rgb'],cv2.COLOR_RGB2BGR)}
        if head_frame is not None:
            inputs.update({'head_selected_rgb.png':cv2.cvtColor(head_frame['rgb'],cv2.COLOR_RGB2BGR),
                'head_selected_mask.png':(np.asarray(head_frame['selected']['mask'])!=0).astype(np.uint8)*255})
        for filename,pixels in inputs.items():
            if not cv2.imwrite(str(directory/filename),pixels):
                raise ServoError(f'Cannot save identity-check input: {filename}')
        acquisition=dict(selected_uv=list(map(float,head_frame['selected']['uv'])) if head_frame is not None else None,
                         target_selection='wrist_center' if center_mode else 'head_identity',
                         wrist_captured_at=float(first['captured_at']),
                         coordinate_system='undistorted_K')
        (directory/'acquisition.json').write_text(json.dumps(acquisition,indent=2)+'\n')
        print(f'V3 视觉入口输入已保存：{directory}',flush=True)
        if center_mode:
            selection_details={}
            try:
                locked_book=acquire_wrist_center(first['rgb'],wrist_detector,diagnostics=selection_details)
            finally:
                (directory/'wrist_center_selection.json').write_text(json.dumps(selection_details,indent=2)+'\n')
            selected=locked_book.observe(first['rgb']);guard()
            annotated=draw_selection(first['rgb'],selected)
            cv2.imwrite(str(directory/'wrist_center_selected.png'),cv2.cvtColor(annotated,cv2.COLOR_RGB2BGR))
            print('V3 腕部中心选书：已锁定离图像中心最近的书，不执行头腕匹配。'+json.dumps(selection_details,ensure_ascii=False),flush=True)
            if display is not None:display.safe('log_image','book/images/spine',annotated)
            final_match=tracker.observe(first['rgb'],search_mask=selected['mask']);guard()
            require_points_in_book(final_match,selected)
        else:
            selected=match_head_book(first,guard)
        result=run_xyz(observe,move,c,checkpoint=checkpoint,log=log)
        if result.get('converged') is not True:raise ServoError('No verified final-image convergence')
        context['forward_complete']=True
        context['visual_final_xyz_mm']=pose().tolist()
        return task.get('object'),True
    except BaseException as error:
        log(dict(event='stopped',reason=f'{type(error).__name__}: {error}'))
        raise
