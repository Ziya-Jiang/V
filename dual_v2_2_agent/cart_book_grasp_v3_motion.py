"""Scoped V1_3 pre-insertion Y/Z servo; commands retain the existing planner."""
from contextlib import contextmanager
import json
from pathlib import Path
import time
import uuid

import cv2
import numpy as np

from cart_book_grasp_v3_servo import ServoError, load_config, run_servo
from cart_book_grasp_v3_tracking import acquire_selected_spine, overlay
from cart_book_grasp_v3_camera import WristCamera, stationary_guard


def orientation_error(euler, reference):
    from cart_book_grasp_v1_3_depth import tool_rotation
    relative = tool_rotation(euler) @ tool_rotation(reference).T
    return float(np.rad2deg(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1))))


def check_reference(config, camera, position, euler, surface_x_mm):
    ref = config['reference']
    if ref is None:
        raise ServoError('V3 requires a taught alignment point')
    if any(ref[k] != camera[k] for k in ('device', 'width', 'height')):
        raise ServoError('Wrist camera/resolution differs from the taught reference')
    if surface_x_mm is None:
        raise ServoError('V1_3 did not provide the current book surface X')
    gap = float(surface_x_mm)-float(position[0])
    if not np.isfinite(gap) or gap < config['min_surface_gap_mm']:
        raise ServoError('Insufficient base-X clearance for visual servo probes')
    if abs(gap-ref['surface_gap_mm']) > config['max_reference_gap_error_mm']:
        raise ServoError(f'Reference distance mismatch: current TCP-to-surface X gap {gap:.1f} mm')
    if orientation_error(euler, ref['tool_euler_deg']) > config['max_reference_orientation_error_deg']:
        raise ServoError('Tool orientation differs from teaching; re-teach at this orientation')


def execute_servo(robot, task, task_index, object_id_to_type, step5, execute,
                  head_frame, config, *, display=None, output_root=None, camera=None, **kwargs):
    from cart_book_grasp_v1_3_motion import _right_tool_mm_euler
    camera = WristCamera() if camera is None else camera
    camera.check_reference(config['reference'])
    pose = _right_tool_mm_euler(robot, step5)
    if pose is None:
        raise ServoError('Cannot read the actual right TCP pose')
    start, rotation = pose
    check_reference(config, camera.config, start, rotation, task.get('book_surface_x_mm'))
    checkpoint = getattr(robot, 'operator_checkpoint', lambda: None)
    root = Path(output_root) if output_root is not None else Path(__file__).with_name('RobotRAM')/'output'
    directory = root/('v3_servo_'+uuid.uuid4().hex)
    directory.mkdir(parents=True)
    frame_index = 0

    def log(event):
        event = dict(event, time_monotonic=time.monotonic())
        with (directory/'events.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(event, ensure_ascii=True)+'\n')
        if display is not None:
            display.safe('log_json_document', 'book/v3_servo', event)
        if event['event'] in ('iteration', 'jacobian', 'converged'):
            print('V3 servo: '+json.dumps(event, ensure_ascii=True), flush=True)

    def actual_pose():
        current = _right_tool_mm_euler(robot, step5)
        if current is None:
            raise ServoError('Lost actual TCP state')
        position, euler = current
        if abs(position[0]-start[0]) > config['max_pose_error_mm']:
            raise ServoError('Unexpected base-X drift during Y/Z servo')
        if orientation_error(euler, rotation) > config['max_orientation_error_deg']:
            raise ServoError('Unexpected tool rotation during Y/Z servo')
        return position

    def fresh_frame():
        checkpoint()
        getattr(robot, 'operator_sleep', time.sleep)(config['settle_seconds'])
        guard = stationary_guard(robot, camera.config['max_joint_drift_deg'])
        frame = camera.capture(checkpoint)
        guard()
        actual_pose()
        return frame, guard

    def observe():
        nonlocal frame_index
        frame, guard = fresh_frame()
        observation = tracker.observe(frame['rgb'])
        guard()
        position = actual_pose()
        annotated = overlay(frame['rgb'], observation, config['reference']['uv'])
        if not cv2.imwrite(str(directory/f'frame_{frame_index:03d}.png'), cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)):
            raise ServoError('Cannot save servo observation')
        frame_index += 1
        if display is not None:
            # Reuse the existing visible V1_3 Spine + center panel.
            display.safe('log_image', 'book/images/spine', annotated)
            display.safe('flush_book')
        return dict(uv=observation['uv'], yz_mm=position[1:], captured_at=frame['captured_at'])

    def move(yz):
        checkpoint()
        actual_pose()
        target = [float(start[0]), *np.asarray(yz, float).tolist()]
        waypoint = dict(task, action='book_v3_visual_servo', target_position=target,
                        target_rotation=rotation.tolist(), description='V3 base-Y/Z visual correction')
        # The V1_3 original task executor still plans and checks the motion.
        result = execute(robot, waypoint, task_index, object_id_to_type, **kwargs)
        return bool(result[1])

    log(dict(event='start', start_mm=start.tolist(), tool_euler_deg=rotation.tolist(), config=config))
    try:
        frame, guard = fresh_frame()
        tracker = acquire_selected_spine(head_frame, frame['rgb'])
        guard()
        actual_pose()
        log(dict(event='target_locked', head_uv=list(map(float, head_frame['selected']['uv'])),
                 wrist_uv=tracker.center.tolist()))
        return run_servo(observe, move, config, checkpoint=checkpoint, log=log)
    except BaseException as error:
        log(dict(event='stopped', error=f'{type(error).__name__}: {error}'))
        raise


@contextmanager
def installed(config_path, approach_path=None, *, cotracker_path=None, cotracker_model=None,column_path=None,stream_config=None,wrist_boundary_selector=None):
    """Install before V1_3.main so its perception wrapper retains these hooks."""
    import cart_book_grasp_v1_3_motion as motion
    original_forward = motion._execute_cart_plus_forward
    original_spine = motion._install_spine_center
    state = dict(frame=None, display=None)

    class CaptureProxy:
        def __init__(self, visualization):
            self.visualization = visualization

        def __getattr__(self, name):
            return getattr(self.visualization, name)

        def capture(self, rgb, depth, intrinsic, c2w, selected, book_mask):
            # This callback is independent of whether a Rerun viewer is active.
            state['frame'] = dict(rgb=np.asarray(rgb).copy(), selected=dict(
                uv=list(selected['uv']), mask=np.asarray(selected['mask']).copy()))
            if self.visualization is not None:
                self.visualization.capture(rgb, depth, intrinsic, c2w, selected, book_mask)

        def safe(self, *args, **kwargs):
            if self.visualization is not None:
                return self.visualization.safe(*args, **kwargs)

        def update_geometry(self, *args):
            if self.visualization is not None:
                self.visualization.update_geometry(*args)

    def spine(geometry, visualization=None, **kwargs):
        state['display'] = visualization
        from cart_book_grasp_v3_wrist_center import detector_from_geometry
        state['wrist_detector'] = detector_from_geometry(geometry)
        use_head_fallback=stream_config is not None and stream_config.get('runtime_version')=='3.2'
        if use_head_fallback:
            from cart_book_grasp_v3_2_head_fallback import recover, center_from_mask
            from cart_book_grasp_v3_head_tilt_review import review
            kwargs['spine_fallback']=recover
            kwargs['spine_fallback_center']=center_from_mask
            kwargs['spine_review']=review
        original_spine(geometry, CaptureProxy(visualization), **kwargs)
        if callable(getattr(wrist_boundary_selector,'recover',None)):
            head_segment=geometry.segment_cart_book_with_sam3
            def remember_backend(*args,**segment_kwargs):
                state['wrist_detector'].box_detector=segment_kwargs.get('box_detector')
                return head_segment(*args,**segment_kwargs)
            geometry.segment_cart_book_with_sam3=remember_backend
        segment = geometry.segment_cart_book_with_sam3
        def current_segment(*args, **kw):
            state['frame'] = None  # Never reuse the previous cycle's target.
            return segment(*args, **kw)
        current_segment.gpt_spine_fallback=use_head_fallback
        current_segment.gpt_polygon_fallback=use_head_fallback
        geometry.segment_cart_book_with_sam3 = current_segment

    def forward(robot, task, task_index, object_id_to_type, step5, execute, **kwargs):
        if column_path is not None:
            from cart_book_grasp_v3_cotracker_runtime import execute_cotracker_approach
            from cart_book_grasp_v3_column import load_config as load_column
            return execute_cotracker_approach(robot,task,task_index,object_id_to_type,step5,execute,
                load_column(column_path),model=None,wrist_detector=state.get('wrist_detector'),
                display=state['display'],column_mode=True,stream_config=stream_config,
                wrist_boundary_selector=wrist_boundary_selector,**kwargs)
        if cotracker_path is not None:
            from cart_book_grasp_v3_cotracker_runtime import execute_cotracker_approach
            from cart_book_grasp_v3_distance_servo import load_config as load_cotracker
            return execute_cotracker_approach(robot,task,task_index,object_id_to_type,step5,execute,
                load_cotracker(cotracker_path),model=cotracker_model,wrist_detector=state.get('wrist_detector'),
                display=state['display'],**kwargs)
        if approach_path is not None:
            from cart_book_grasp_v3_approach import execute_visual_approach
            from cart_book_grasp_v3_xyz_servo import load_approach
            return execute_visual_approach(robot, task, task_index, object_id_to_type, step5, execute,
                state['frame'], load_approach(approach_path), display=state['display'],
                wrist_detector=state.get('wrist_detector'), **kwargs)
        execute_kwargs = {k: v for k, v in kwargs.items() if k != 'retreat_context'}
        config = load_config(config_path)
        print('V3：前伸前闭环对准示教点，固定 X 和姿态，仅修正基座 Y/Z。', flush=True)
        execute_servo(robot, task, task_index, object_id_to_type, step5, execute,
                      state['frame'], config, display=state['display'], **execute_kwargs)
        # Read the corrected TCP in the original forward method; all following
        # grasp, lift, measured J7 undo, release and retreat remain V1_3 behavior.
        return original_forward(robot, task, task_index, object_id_to_type, step5, execute, **kwargs)

    motion._execute_cart_plus_forward = forward
    motion._install_spine_center = spine
    try:
        yield
    finally:
        motion._execute_cart_plus_forward = original_forward
        motion._install_spine_center = original_spine
