"""Live book and scene Rerun data. No output-directory globbing or robot commands."""
from __future__ import annotations

import inspect
import json
import time
import cv2
import numpy as np
from cart_book_grasp_v1_3_depth import forward_parameters


def match_book_mask(detections, spine):
    """Select a full-book detection containing this spine, not a nearby book."""
    candidates = []
    for score, mask, _ in detections:
        mask = np.asarray(mask, dtype=np.uint8)
        if mask.ndim != 2 or not np.isfinite(score) or score < .1:
            continue
        if mask.shape != spine.shape:
            mask = cv2.resize(mask, (spine.shape[1], spine.shape[0]), interpolation=cv2.INTER_NEAREST)
        mask = mask.astype(bool)
        coverage = np.count_nonzero(mask & spine) / max(1, np.count_nonzero(spine))
        if coverage >= .7:
            candidates.append((float(score) * coverage, mask))
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


def forward_targets(start_mm, parameters):
    distance = parameters['forward_mm']
    if distance <= 0:
        return []
    count = int(np.ceil(distance / parameters['forward_step_mm']))
    return [np.asarray(start_mm, dtype=float) + [distance * i / count, 0, 0]
            for i in range(1, count + 1)]


def withdraw_targets(start_mm, original_x_mm, step_mm):
    start = np.asarray(start_mm, dtype=float)
    distance = max(0., float(start[0] - original_x_mm))
    if distance == 0:
        return []
    count = int(np.ceil(distance / step_mm))
    return [start - [distance * i / count, 0, 0] for i in range(1, count + 1)]


def preview_targets(start, rotation, center_mm, parameters, delta_z_mm=0., book_surface_x_mm=None):
    """Same base-X forward waypoints as execution; preview is not an IK path."""
    aligned = np.asarray(start, dtype=float).copy()
    aligned[1] = center_mm[1]
    compensated = aligned + [0., 0., delta_z_mm]
    parameters = forward_parameters(compensated, rotation, book_surface_x_mm, parameters)
    forward = forward_targets(compensated, parameters)
    grasp = forward[-1] if forward else compensated
    lift = grasp + [0, 0, parameters['lift_mm']]
    descent = (lower_targets(lift, parameters) if 'lower_step_mm' in parameters
               else [lift - [0, 0, parameters['lower_mm']]])
    lower = descent[-1] if descent else lift
    withdraw = withdraw_targets(lower, aligned[0], parameters['forward_step_mm'])
    points = [np.asarray(start), aligned, *([compensated] if abs(delta_z_mm) > 1e-8 else []), *forward, lift, *descent, *withdraw]
    lower_labels = ([f'lower {i+1}/{len(descent)}' + (' / release' if i == len(descent)-1 else '')
                     for i in range(len(descent))] if 'lower_step_mm' in parameters else ['lower / release'])
    labels = (['start', 'align spine'] + (['depth fallback Z shift'] if abs(delta_z_mm) > 1e-8 else []) + [f'forward {i+1}/{len(forward)}' for i in range(len(forward))]
              + ['lift'] + lower_labels + [f'withdraw {i+1}/{len(withdraw)}' for i in range(len(withdraw))])
    return dict(points_mm=np.asarray(points), labels=labels, grasp_mm=grasp,
                rotation_deg=np.asarray(rotation), parameters=dict(parameters))


def lower_targets(start_mm, parameters):
    """v1.1 descent endpoints along base -Z, all from one measured origin."""
    distance, step = parameters['lower_mm'], parameters['lower_step_mm']
    if not np.isfinite([distance, step]).all() or distance < 0 or step < 1:
        raise ValueError('Invalid descent distance or segment length')
    if distance == 0:
        return []
    count = int(np.ceil(distance/step))
    return [np.asarray(start_mm, dtype=float)-[0, 0, distance*i/count] for i in range(1, count+1)]


def mask_cloud(mask, depth, intrinsic, c2w, max_points):
    valid = np.asarray(mask, dtype=bool) & np.isfinite(depth) & (depth > 0)
    yy, xx = np.nonzero(valid)
    if not len(xx):
        return np.empty((0, 3)), yy, xx
    stride = max(1, int(np.ceil(len(xx) / max_points)))
    yy, xx = yy[::stride], xx[::stride]
    z = depth[yy, xx].astype(float)
    if np.median(z) > 20:
        z *= .001
    rays = np.linalg.solve(intrinsic, np.array([xx, yy, np.ones(len(xx))]))
    points = (rays / rays[2] * z).T
    transform = np.asarray(c2w, dtype=float)
    translation = transform[:3, 3]
    if np.linalg.norm(translation) > 10:
        translation = translation * .001
    return points @ transform[:3, :3].T + translation, yy, xx


def english_task(task):
    """Display runtime tasks without their Chinese console descriptions."""
    display = dict(task)
    label = str(task.get('action', 'target')).replace('_', ' ')
    if task.get('book_forward_index') is not None:
        label = f"Forward waypoint {task['book_forward_index']}/{task['book_forward_count']}"
    if task.get('book_withdraw_index') is not None:
        label = f"Withdraw waypoint {task['book_withdraw_index']}/{task['book_withdraw_count']}"
    if task.get('book_lower_index') is not None:
        label = f"Lower waypoint {task['book_lower_index']}/{task['book_lower_count']}"
    display['description'] = label
    return display


class BookVisualization:
    def __init__(self, pose_reader, parameter_reader):
        self.pose_reader = pose_reader
        self.parameter_reader = parameter_reader
        self.logger = None
        self.robot = None
        self.frame = None
        self.geometry = None
        self.active = False
        self.tracking = False
        self.samples = []
        self.last_sample = 0.

    def safe(self, operation, *args, **kwargs):
        if self.logger is None or not self.active:
            return
        try:
            return getattr(self.logger, operation)(*args, **kwargs)
        except Exception as error:
            print(f'书本可视化 {operation} 失败（不影响控制）：{error}', flush=True)

    def capture(self, rgb, depth, intrinsic, c2w, selected, book_mask):
        self.frame = dict(rgb=np.asarray(rgb).copy(), depth=np.asarray(depth).copy(),
                          intrinsic=np.asarray(intrinsic).copy(), c2w=np.asarray(c2w).copy(),
                          selected=selected, book_mask=book_mask)
        self.safe('log_book_perception')

    def update_geometry(self, geometry, c2w):
        self.geometry = dict(geometry)
        if self.frame is not None:
            self.frame['c2w'] = np.asarray(c2w).copy()
        self.safe('log_book_perception')
        self.safe('log_book_preview')

    def attach_robot(self, robot):
        if robot is self.robot:
            return
        self.robot = robot
        original = robot.get_current_state

        def observed_state(*args, **kwargs):
            # Observe existing reads only: never compete for UDP receives.
            state = original(*args, **kwargs)
            now = time.monotonic()
            if self.active and self.tracking and now - self.last_sample >= .1:
                self.last_sample = now
                try:
                    point = np.asarray(state['right_arm']['position'], dtype=float)
                    if point.shape == (3,) and np.isfinite(point).all():
                        self.samples.append(point.copy())
                        self.safe('log_book_measured', self.samples)
                except (TypeError, KeyError, ValueError):
                    pass
            return state

        robot.get_current_state = observed_state


def install_book_visualization(agent, pose_reader, parameter_reader):
    """Replace the logger only for the local entrypoint; cart+ uses live data."""
    import rerun as rr
    import rerun.blueprint as rrb

    session = BookVisualization(pose_reader, parameter_reader)
    base = agent.RerunPipelineLogger

    class BookLogger(base):
        def __init__(self, **kwargs):
            kwargs['application_id'] = 'book_grasp_local'
            super().__init__(**kwargs)
            session.logger = self

        def log_text_document(self, entity_path, text):
            # Keep external diagnostics readable even when they contain text
            # unsupported by the viewer font. Our own UI strings are English.
            super().log_text_document(entity_path, str(text).encode('ascii', 'backslashreplace').decode('ascii'))

        def log_json_document(self, entity_path, data):
            self.log_text_document(entity_path, json.dumps(data, indent=2, ensure_ascii=True, default=str))

        def flush_book(self):
            try:
                recording = rr.get_global_data_recording()
                if recording is None:
                    return
                # Older SDKs expose flush() with no arguments. Do not pass
                # False positionally: other releases interpret it as a timeout.
                parameters = inspect.signature(recording.flush).parameters
                if 'blocking' in parameters:
                    recording.flush(blocking=False)
                else:
                    recording.flush()
            except Exception as error:
                # begin_run calls this directly, outside session.safe(). A
                # viewer/recording error must not abort the robot pipeline.
                if not getattr(self, '_book_flush_warning_emitted', False):
                    self._book_flush_warning_emitted = True
                    print(f'Rerun 刷新失败（不影响控制，后续重复提示省略）：{error}', flush=True)

        def send_default_blueprint(self):
            rr.send_blueprint(rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Vertical(
                        rrb.Spatial2DView(origin='/book/images/book', name='Book segmentation'),
                        rrb.Spatial2DView(origin='/book/images/spine', name='Spine + center'),
                        rrb.Spatial2DView(origin='/book/depth', name='Aligned depth (m)'),
                    ),
                    rrb.Vertical(
                        rrb.Spatial3DView(origin='/world', contents=['/world/book/**', '/world/environment/**'], name='Scene / book / targets / measured TCP'),
                        rrb.TextDocumentView(origin='/book/status', name='Spine center & depth'),
                        rrb.Tabs(
                            rrb.TextDocumentView(origin='/book/motion', name='Trajectory legend'),
                            rrb.TextDocumentView(origin='/step5/execution/current_action', name='Current action'),
                            rrb.TextDocumentView(origin='/book/current_command', name='Actual waypoint'),
                            rrb.TextDocumentView(origin='/book/wrist', name='Spine tilt / J7'),
                        ),
                        row_shares=[.65, .2, .15],
                    ), column_shares=[.4, .6]),
                auto_layout=False, auto_views=False), make_active=True, make_default=True)

        def begin_run(self, task_type):
            session.active = task_type == 'cart+'
            session.frame = session.geometry = None
            session.samples = []
            session.last_sample = 0.
            session.tracking = False
            self.sent_targets = []
            result = super().begin_run(task_type)
            for path in ('book', 'world/book', 'world/environment', 'world/trajectory', 'world/selected_book',
                         'world/camera/extern/objects', 'world/camera/extern/point_clouds'):
                rr.log(path, rr.Clear(recursive=True))
            if session.active:
                self.send_default_blueprint()
                self.log_text_document('book/status', 'Waiting for this run\'s spine detection. Previous data cleared.')
                self.flush_book()
            else:
                super().send_default_blueprint()
            return result

        def log_detection_artifacts(self, output_dir):
            if session.active:
                session.safe('log_book_perception')
                return
            super().log_detection_artifacts(output_dir)

        def log_ram_artifacts(self, output_dir, **kwargs):
            if session.active:
                session.safe('log_book_perception')
                session.safe('log_book_preview')
                return
            super().log_ram_artifacts(output_dir, **kwargs)

        def log_manual_planning_tasks(self, tasks):
            if session.active:
                session.safe('log_book_preview')
                return
            super().log_manual_planning_tasks(tasks)

        def log_book_perception(self):
            self._advance_pipeline_step()
            frame = session.frame
            if frame is None:
                return
            rgb, depth = frame['rgb'], frame['depth']
            selected = frame['selected']
            if rgb.shape[:2] != depth.shape:
                rgb = cv2.resize(rgb, (depth.shape[1], depth.shape[0]))
            # Reconstruct this capture's background only, using the same
            # calibration and RGB as the book. Never load historical clouds.
            foreground = selected['mask'].copy()
            if frame['book_mask'] is not None:
                foreground |= frame['book_mask']
            points, yy, xx = mask_cloud(~foreground, depth, frame['intrinsic'], frame['c2w'], self.max_points)
            rr.log('world/environment', rr.Clear(recursive=True))
            rr.log('world/environment/cloud', rr.Points3D(points, colors=rgb[yy, xx], radii=.0015))
            for name, mask, color in (('book', frame['book_mask'], [60, 160, 255]),
                                      ('spine', selected['mask'], [40, 240, 100])):
                rr.log(f'world/book/cloud/{name}', rr.Clear(recursive=True))
                overlay = rgb.copy()
                if mask is not None:
                    overlay[mask] = (.55 * overlay[mask] + .45 * np.array(color)).astype(np.uint8)
                    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(overlay, contours, -1, color, 1)
                    points, yy, xx = mask_cloud(mask, depth, frame['intrinsic'], frame['c2w'], self.max_points)
                    rr.log(f'world/book/cloud/{name}', rr.Points3D(
                        points, colors=rgb[yy, xx], radii=.0015))
                if name == 'spine':
                    u, v = np.rint(selected['uv']).astype(int)
                    cv2.drawMarker(overlay, (u, v), (255, 50, 50), cv2.MARKER_CROSS, 18, 2)
                    if selected.get('spine_center_recovery') == 'stable_region':
                        ends = np.rint(selected['spine_stable_region_uv']).astype(int)
                        cv2.line(overlay, tuple(ends[0]), tuple(ends[1]), (80, 255, 255), 3)
                        original = tuple(np.rint(selected['spine_original_center_uv']).astype(int))
                        cv2.circle(overlay, original, 5, (255, 230, 0), 2)
                        cv2.arrowedLine(overlay, original, (u, v), (255, 230, 0), 2)
                        cv2.putText(overlay, 'Depth fallback: stable region', (10, 75),
                                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 230, 0), 2)
                    cv2.putText(overlay, f"center depth {selected['depth_m']*1000:.1f} mm", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 70, 70), 2)
                    if 'spine_tilt_deg' in selected:
                        center_uv = np.asarray(selected['uv'])
                        axis_uv = np.asarray(selected['spine_axis_uv'])
                        half = .4*selected['spine_length_px']
                        for direction, color in ((axis_uv, (255, 60, 210)), (np.array([0, 1]), (255, 230, 0))):
                            cv2.line(overlay, tuple(np.rint(center_uv-half*direction).astype(int)),
                                     tuple(np.rint(center_uv+half*direction).astype(int)), color, 2)
                        cv2.putText(overlay, f"Image tilt {abs(selected['spine_tilt_deg']):.1f} deg", (10, 50),
                                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 230, 0), 2)
                self.log_image(f'book/images/{name}', overlay)
            depth_m = depth.astype(float)
            positive = depth_m[np.isfinite(depth_m) & (depth_m > 0)]
            if positive.size and np.median(positive) > 20:
                depth_m *= .001
            rr.log('book/depth', rr.DepthImage(depth_m.astype(np.float32), meter=1.))
            rr.log('book/depth/center', rr.Points2D([selected['uv']], colors=[255, 50, 50], radii=5))
            rr.log('book/depth/approach_median', rr.Clear(recursive=True))
            if 'whole_book_median_uv' in selected:
                rr.log('book/depth/approach_median', rr.Points2D(
                    [selected['whole_book_median_uv']], colors=[255, 230, 0], radii=7,
                    labels=[f"Approach X median: {selected['book_surface_x_mm']:.1f} mm | "
                            f"Camera Z median: {selected['whole_book_depth_median_mm']:.1f} mm"],
                    show_labels=True))
            transform = frame['c2w']
            translation = transform[:3, 3].copy()
            if np.linalg.norm(translation) > 10:
                translation *= .001
            center = transform[:3, :3] @ selected['point_camera_m'] + translation
            rr.log('world/book/spine_center', rr.Points3D([center], colors=[255, 40, 60], radii=.006,
                labels=[f"spine center / Zcam {selected['depth_m']*1000:.1f} mm"]))
            rr.log('world/book/depth_ray', rr.LineStrips3D([[translation, center]], colors=[130, 130, 130], radii=.0008))
            text = (f"Spine center pixel: {np.round(selected['uv'], 1).tolist()}\n\n"
                    f"Local median depth (camera Z): {selected['depth_m']*1000:.1f} mm\n\n"
                    f"Robot-base XYZ: {np.round(center*1000, 1).tolist()} mm\n\n"
                    f"Spine score: {selected.get('score', 0):.3f}; valid depth samples: {selected.get('depth_samples', 'N/A')}\n\n"
                    f"Center depth P10-P90: {selected.get('depth_p10_p90_mm', 'N/A')} mm\n\n"
                    f"{'Full-book mask matched to this spine' if frame['book_mask'] is not None else 'Full-book mask not matched; no guessed result shown'}. "
                    "2D overlays: green = spine, blue = book. All 3D clouds use original camera RGB. No estimated bounding boxes.")
            if 'spine_center_recovery' in selected:
                text += (f"\n\nDepth-only fallback: {selected['spine_center_recovery']}"
                         f"\n\nOriginal check: {selected.get('spine_full_fit_failure', 'N/A')}"
                         f"\n\nStable region fraction: {selected.get('spine_stable_region_fraction', 0):.0%}. "
                         'Center and tilt use the same region. Only the fallback-induced base-Z shift is applied before approach.')
            if session.geometry and 'depth_recovery_delta_z_mm' in session.geometry:
                text += f"\n\nPre-approach base-Z compensation: {session.geometry['depth_recovery_delta_z_mm']:+.1f} mm. X/Y are held during this separate step."
            if 'book_surface_x_mm' in selected:
                text += (f"\n\nWhole-book camera-Z median: {selected['whole_book_depth_median_mm']:.1f} mm. "
                         f"Base-X median: {selected['book_surface_x_mm']:.1f} mm. "
                         f"Valid depth pixels: {selected['whole_book_depth_samples']}. "
                         'Approach distance is recomputed from measured TCP pose before forward motion; Y/Z remain fixed.')
            self.log_text_document('book/status', text)
            self.log_book_tilt(center)
            self.flush_book()

        def log_book_tilt(self, center):
            geometry = session.geometry or {}
            rr.log('world/book/tilt', rr.Clear(recursive=True))
            if 'spine_vertical_tilt_deg' not in geometry:
                if session.frame and 'spine_tilt_deg' in session.frame['selected']:
                    self.log_text_document('book/wrist', 'Image tilt is shown in the RGB panel.\n\n3D vertical tilt / J7: ' +
                                           geometry.get('spine_3d_status', 'Waiting for calibrated 3D geometry.'))
                return
            angle = geometry['spine_vertical_tilt_deg']
            axis = np.asarray(geometry['spine_axis_base'])
            rr.log('world/book/tilt/spine_axis', rr.LineStrips3D([geometry['spine_endpoints_base_m']], colors=[255, 60, 210], radii=.002))
            rr.log('world/book/tilt/vertical', rr.LineStrips3D([[center-[0, 0, .1], center+[0, 0, .1]]], colors=[255, 230, 0], radii=.0015))
            horizontal = axis-np.array([0., 0., axis[2]])
            if np.linalg.norm(horizontal) > 1e-6:
                horizontal /= np.linalg.norm(horizontal)
                phi = np.linspace(0, np.radians(angle), 24)
                arc = center+.05*(np.sin(phi)[:, None]*horizontal + np.cos(phi)[:, None]*[0, 0, 1])
                rr.log('world/book/tilt/arc', rr.LineStrips3D([arc], colors=[255, 230, 0], radii=.001))
            rr.log('world/book/tilt/label', rr.Points3D([center+[0, 0, .12]], radii=.001,
                labels=[f'Spine vs base vertical: {angle:.1f} deg'], colors=[255, 230, 0]))
            text = (f"Image tilt: {abs(geometry['spine_image_tilt_deg']):.1f} deg\n\n"
                    f"Spine vs robot-base +Z: {angle:.1f} deg\n\n"
                    f"3D line residual P90: {geometry['spine_axis_residual_mm']:.1f} mm\n\n"
                    f"Automatic J7 alignment: {'ON' if geometry.get('wrist_enabled') else 'OFF (preview only)'}\n\n")
            if 'j7_target_deg' in geometry:
                text += (f"J7 current: {geometry['j7_current_deg']:.1f} deg\n\n"
                         f"J7 suggested delta: {geometry['j7_delta_deg']:+.1f} deg\n\n"
                         f"J7 suggested target: {geometry['j7_target_deg']:.1f} deg\n\n")
            else:
                text += f"J7 suggestion unavailable: {geometry.get('wrist_status', 'No robot state')}\n\n"
            self.log_text_document('book/wrist', text + 'J7 is computed in the tool roll plane, not copied from the image angle. Yellow: base vertical; magenta: measured spine axis.')

        def log_book_preview(self):
            if session.geometry is None or session.robot is None:
                return
            current = session.pose_reader(session.robot)
            if current is None:
                return
            plan = preview_targets(*current, session.geometry['center_base_mm'], session.parameter_reader(), session.geometry.get('depth_recovery_delta_z_mm', 0.), session.geometry.get('book_surface_x_mm'))
            self._advance_pipeline_step()
            rr.log('world/book/preview', rr.Clear(recursive=True))
            rr.log('world/book/preview/line', rr.LineStrips3D([plan['points_mm']/1000], colors=[255, 185, 50], radii=.002))
            rr.log('world/book/preview/points', rr.Points3D(plan['points_mm']/1000, labels=plan['labels'], colors=[255, 185, 50], radii=.004))
            rr.log('world/book/preview/grasp_tcp', rr.Points3D([plan['grasp_mm']/1000], labels=['grasp TCP'], colors=[255, 50, 160], radii=.006))
            self.log_text_document('book/motion', 'Yellow: target preview after perception (TCP point connections, not the planner\'s actual curve).\n\nOrange: issued targets, including forward and withdraw waypoints. Cyan: measured TCP trajectory.\n\nAfter release: withdraw to the original approach X, then right-arm first_reset (joint motion; shown by the measured trail).\n\nParameters reload at each action start; issued targets show the values actually used.')
            self.log_json_document('book/preview_parameters', plan['parameters'])
            self.flush_book()

        def log_book_target(self, task):
            task = english_task(task)
            target = np.asarray(task.get('target_position'), dtype=float)
            if target.shape != (3,) or not np.isfinite(target).all():
                return
            self._advance_pipeline_step()
            self.sent_targets.append(target.copy()/1000)
            rr.log('world/book/sent/points', rr.Points3D(self.sent_targets, colors=[255, 110, 0], radii=.004))
            if len(self.sent_targets) > 1:
                rr.log('world/book/sent/line', rr.LineStrips3D([self.sent_targets], colors=[255, 110, 0], radii=.0015))
            rr.log('world/book/current_target', rr.Points3D([target/1000], labels=[task.get('description', task['action'])], colors=[255, 110, 0], radii=.007))
            if (task.get('action') == 'move_down_to_grasp'
                    and task.get('book_forward_index') == task.get('book_forward_count')):
                rr.log('world/book/grasp_tcp', rr.Points3D([target/1000], labels=['commanded grasp TCP'], colors=[255, 50, 160], radii=.006))
            self.log_json_document('book/current_command', task)

        def log_book_measured(self, samples):
            self._advance_pipeline_step()
            rr.log('world/book/measured/tcp', rr.Points3D([samples[-1]], colors=[0, 240, 240], radii=.004))
            if len(samples) > 1:
                rr.log('world/book/measured/path', rr.LineStrips3D([np.asarray(samples)], colors=[0, 240, 240], radii=.0015))

        def log_execution_action(self, **kwargs):
            if session.active:
                # Generic task targets precede the local runtime rewrites.
                payload = dict(kwargs)
                payload['task'] = {**english_task(payload['task']), 'target_position': None, 'target_rotation': None}
                return super().log_execution_action(**payload)
            return super().log_execution_action(**kwargs)

    agent.RerunPipelineLogger = BookLogger
    return session
