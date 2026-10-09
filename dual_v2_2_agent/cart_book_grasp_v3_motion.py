"""Scoped V3 extension of V2; all commands still use its checked planner."""
from contextlib import contextmanager
import json
from pathlib import Path
import time
import uuid

import cv2
import numpy as np

from cart_book_grasp_v3_servo import ServoError, load_config, run_servo
from cart_book_grasp_v3_tracking import LockedBookTracker, overlay


def orientation_error(euler, reference):
    from cart_book_grasp_v2_depth import tool_rotation
    relative = tool_rotation(euler) @ tool_rotation(reference).T
    return float(np.rad2deg(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1))))


def selected_tracker(selector):
    if selector is None or selector.last_wrist_view is None:
        raise ServoError('V3 requires the current dual-view selected target')
    decision, view = selector.last_decision, selector.last_wrist_view
    selected = decision.get('selected_id')
    if decision.get('status') != 'selected' or not selected:
        raise ServoError('Current GPT decision is not a confirmed selection')
    rows = [row for row in decision.get('cross_view_matches', [])
            if row['head_id'] == selected and row['status'] == 'matched'
            and row['confidence'] >= view['min_match_confidence']]
    if len(rows) != 1:
        raise ServoError('No unique wrist counterpart for the selected head book')
    candidates = [item for item in view['candidates'] if item['id'] == rows[0]['wrist_id'] and item.get('verified_book')]
    if len(candidates) != 1:
        raise ServoError('Selected wrist candidate is missing or unverified')
    return LockedBookTracker(view['rgb'], candidates[0]['book_mask'])


def check_reference(config, camera, position, euler, surface_x_mm):
    ref = config['reference']
    if ref is None:
        raise ServoError('V3 requires a measured target-pixel reference')
    if any(ref[k] != camera[k] for k in ('device', 'width', 'height')):
        raise ServoError('Wrist camera/resolution differs from the taught reference')
    gap = float(surface_x_mm)-float(position[0])
    if not np.isfinite(gap) or gap < config['min_surface_gap_mm']:
        raise ServoError('Insufficient base-X clearance for visual servo probes')
    if abs(gap-ref['surface_gap_mm']) > config['max_reference_gap_error_mm']:
        raise ServoError(f'V3 reference distance mismatch: current TCP-to-surface X gap {gap:.1f} mm')
    if orientation_error(euler, ref['tool_euler_deg']) > config['max_reference_orientation_error_deg']:
        raise ServoError('V3 tool orientation differs from taught reference; reference pixel would be invalid')


def execute_servo(robot, task, task_index, object_id_to_type, step5, execute,
                  selector, config, *, display=None, output_root=None, **kwargs):
    from cart_book_grasp_v2_dual_view import load_config as camera_config, capture_wrist, stationary_guard
    from cart_book_grasp_v2_motion import _right_tool_mm_euler
    camera = camera_config()
    if not camera['enabled']:
        raise ServoError('V3 needs dual-view selection enabled')
    pose = _right_tool_mm_euler(robot, step5)
    if pose is None:
        raise ServoError('V3 cannot read the actual right TCP pose')
    start, rotation = pose
    check_reference(config, camera, start, rotation, task.get('book_surface_x_mm'))
    tracker = selected_tracker(selector)
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
            raise ServoError('V3 lost actual TCP state')
        position, euler = current
        if abs(position[0]-start[0]) > config['max_pose_error_mm']:
            raise ServoError('Unexpected base-X drift during Y/Z servo')
        if orientation_error(euler, rotation) > config['max_orientation_error_deg']:
            raise ServoError('Unexpected tool rotation during Y/Z servo')
        return position

    def observe():
        nonlocal frame_index
        checkpoint()
        sleeper = getattr(robot, 'operator_sleep', time.sleep)
        sleeper(config['settle_seconds'])
        guard = stationary_guard(robot, camera['max_joint_drift_deg'])
        frame = capture_wrist(camera, time.monotonic(), checkpoint)
        observation = tracker.observe(frame['rgb'])
        guard()
        position = actual_pose()
        annotated = overlay(frame['rgb'], observation, config['reference']['uv'])
        cv2.imwrite(str(directory/f'frame_{frame_index:03d}.png'), cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
        frame_index += 1
        if display is not None:
            # Existing V2 blueprint already displays this entity.
            display.safe('log_image', 'book/images/wrist_rgb', annotated)
            display.safe('flush_book')
        return dict(uv=observation['uv'], yz_mm=position[1:], captured_at=frame['captured_at'])

    def move(yz):
        checkpoint()
        actual_pose()
        target = [float(start[0]), *np.asarray(yz, float).tolist()]
        waypoint = dict(task, action='book_v3_visual_servo', target_position=target,
                        target_rotation=rotation.tolist(), description='V3 base-Y/Z visual correction')
        # This is the same low-level task executor V2 uses for forward waypoints,
        # bypassing only V2's coarse Y-only task rewrite, not its planner.
        result = execute(robot, waypoint, task_index, object_id_to_type, **kwargs)
        return bool(result[1])

    log(dict(event='start', selected_id=selector.last_decision['selected_id'],
             start_mm=start.tolist(), tool_euler_deg=rotation.tolist(), config=config))
    try:
        return run_servo(observe, move, config, checkpoint=checkpoint, log=log)
    except BaseException as error:
        log(dict(event='stopped', error=f'{type(error).__name__}: {error}'))
        raise


@contextmanager
def installed(config_path):
    """Only this entrypoint intercepts the forward action; restore on exit."""
    import cart_book_grasp_v2_motion as motion
    original_forward = motion._execute_cart_plus_forward
    original_spine = motion._install_spine_center
    display = [None]

    def spine(geometry, visualization=None, **kwargs):
        display[0] = visualization
        return original_spine(geometry, visualization, **kwargs)

    def forward(robot, task, task_index, object_id_to_type, step5, execute, **kwargs):
        # The retreat context belongs to V2, not the step5 task API.
        execute_kwargs = {k: v for k, v in kwargs.items() if k != 'retreat_context'}
        config = load_config(config_path)
        print('V3：前伸前进行手腕视觉闭环，固定基座 X 和姿态，仅修正基座 Y/Z。', flush=True)
        execute_servo(robot, task, task_index, object_id_to_type, step5, execute,
                      motion._SELECTION_AGENT, config, display=display[0], **execute_kwargs)
        # Only verified convergence reaches this call. Forward still uses head
        # depth, measured corrected pose and the V2 retreat/release bookkeeping.
        return original_forward(robot, task, task_index, object_id_to_type, step5, execute, **kwargs)

    motion._execute_cart_plus_forward = forward
    motion._install_spine_center = spine
    try:
        yield
    finally:
        motion._execute_cart_plus_forward = original_forward
        motion._install_spine_center = original_spine
