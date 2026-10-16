"""Head coarse approach -> wrist local visual servo -> stationary visual landing.

No camera, GPT, transport or movement on import. Original V3/RT stay unchanged.
"""
import copy
from contextlib import nullcontext
from pathlib import Path
import time
import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v4_1 import ROOT, write_json, jsonable, pose_mm, tool_rotation
from book_top_landing import (settings, coarse_plan, head_geometry, fit_jacobian,
    LandingController, contact_reference, pad_fingerprint, horizontal_pad)
from book_top_landing_vision import LandingHeadVision, LandingWristVision, LandingSelector, measured_pad


def coarse_move(io,goal):
    """Private V4.1 planning verb; ordinary CuRobo only before visual session."""
    from cuarm_state import TargetCommandArmCommandType,TargetCommandPlannerMode
    state=io.guard();command=copy.deepcopy(io.robot.target_command)
    command.arm_command_type=TargetCommandArmCommandType.CARTESIAN
    command.planner_mode=TargetCommandPlannerMode.PLANNING
    command.description='book_v4_1_coarse'
    command.arm_tool_command=[[*state[a]['position'],*state[a]['orientation']] for a in ('left_arm','right_arm')]
    command.arm_tool_command[1][:3]=(np.asarray(goal)/1000).tolist()
    io.robot.target_command=command;io.robot.udp.send(command)
    target=np.asarray(command.arm_tool_command,float);sent=time.monotonic()
    deadline=sent+settings(io.c)['coarse_timeout_seconds'];stable=[];last=-np.inf
    io.log(dict(event='coarse_target',target_mm=goal))
    while time.monotonic()<deadline:
        s=io.guard();stamp=s['pose_sample_monotonic']
        echoed=np.asarray(s.get('processed_arm_tool_command',[]),float)
        ack=s['received_monotonic']>sent and echoed.shape==target.shape and np.allclose(echoed,target,atol=1e-6,rtol=0)
        if stamp>last:
            last=stamp;v=np.asarray(s.get('joint_velocity',[]),float)
            arrived=np.linalg.norm(pose_mm(s)-goal)<=settings(io.c)['coarse_position_tolerance_mm']
            if ack and arrived and v.shape==(14,) and np.isfinite(v).all() and max(abs(v))<=.005:
                stable.append(stamp)
                if len(stable)>=3 and stamp-stable[0]>=.3:
                    io.log(dict(event='coarse_arrived',actual_mm=pose_mm(s)));return
            else:stable=[]
        time.sleep(.02)
    raise ValueError('Coarse target not acknowledged and stably reached')


def actual_aperture(robot):
    raw,stamp=robot.udp.receive_with_timestamp()
    if raw is None or stamp is None or not 0<=time.monotonic()-stamp<=.25:
        raise ValueError('No fresh gripper opening for automatic pad geometry')
    a=np.asarray(raw.gripper_joint_position[1],float).reshape(-1)
    if a.shape!=(1,) or not np.isfinite(a).all() or not 0<=a[0]<=1:raise ValueError('Invalid gripper opening')
    return float(a[0])


def observe_pair(head,wrist,io,c):
    # Each camera has its own timestamp. Wrist pose is paired, not the pose
    # preceding a potentially blocking capture. Head supplies base geometry.
    io.guard();h=head.observe();w=wrist.observe(io.guard)
    if abs(h['captured_unix_s']-w['captured_unix_s'])>settings(c)['max_head_wrist_gap_seconds']:
        raise ValueError('Head/wrist acquisition gap too large')
    if time.time()-h['captured_unix_s']>c['frame_max_age_seconds']:
        raise ValueError('Head geometry expired during wrist capture')
    obs=dict(h,**w);p=io.pair(w);state=io.guard()
    return obs,p,state


def probe_jacobian(head,wrist,io,c,log):
    """Three positive probes and returns, above book; actual XYZ/UV regression."""
    d=settings(c);moves=[];responses=[]
    def feature(o):return o['wrist_contact_uv']-o.get('pad_contact_uv',np.zeros(2))
    with io.perception_pause():
        obs,p,state=observe_pair(head,wrist,io,c)
    origin=pose_mm(state);top0=obs['top_base_mm'].copy()
    pad=horizontal_pad(c,io.rotation)
    for axis in range(3):
        for offset in (np.eye(3)[axis]*d['probe_mm'],np.zeros(3)):
            goal=origin+offset;startp=p['q'].copy();startf=feature(obs).copy()
            deadline=time.monotonic()+c['motion_timeout_seconds']
            while time.monotonic()<deadline:
                obs,p,state=observe_pair(head,wrist,io,c);now=pose_mm(state)
                gap=pad.observe_book_point(obs['top_base_mm'],p['q'],io.rotation)['signed_gap_mm']
                if gap<5 or np.linalg.norm(obs['top_base_mm']-top0)>d['max_book_motion_mm']:
                    raise ValueError('Unsafe/stale probe geometry; no contact probe')
                delta=goal-now
                if np.linalg.norm(delta)<.35:break
                delta*=min(1.,c['step_mm']/max(np.linalg.norm(delta),1e-12))
                io.send(now+delta,obs['captured_unix_s']);time.sleep(1/c['visual_hz'])
            else:raise ValueError('Actual Jacobian probe displacement not reached')
            with io.perception_pause():
                obs,p,state=observe_pair(head,wrist,io,c)
            moves.append(p['q']-startp);responses.append(feature(obs)-startf)
    jac,error=fit_jacobian(moves,responses,c)
    log(dict(event='jacobian_xyz',pixels_per_mm=jac,fit_error_px=error,
             actual_displacements_mm=moves,pixel_responses=responses))
    return jac,obs,p,state


def teach_contact(camera,c,path):
    # Human interaction explicitly names visual contact, not a generated target.
    f=camera.capture();rgb=f['rgb'];points=[]
    title='Real contact only: click book upper edge at pad contact; Enter save, Esc cancel'
    cv2.namedWindow(title,cv2.WINDOW_NORMAL)
    def click(event,x,y,flags,data):
        if event==cv2.EVENT_LBUTTONDOWN:points[:]=[[x,y]]
    cv2.setMouseCallback(title,click)
    try:
        while True:
            im=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            if points:cv2.circle(im,tuple(points[0]),5,(0,0,255),-1)
            cv2.imshow(title,im);key=cv2.waitKey(30)&255
            if key==27:raise ValueError('Contact teaching cancelled')
            if key in (10,13) and points:break
        value=dict(kind='human_taught_visual_contact',coordinate_system='undistorted_K',contact_uv=points[0],
            calibration_sha256=camera.calibration_hash,pad_fingerprint=pad_fingerprint(c),
            captured_unix_s=time.time(),physical_contact_confirmed=False,
            **{k:camera.config[k] for k in ('device','width','height')})
        write_json(path,value);cv2.imwrite(str(path.with_suffix('.jpg')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        print('V4.1 可选视觉搭接参考已保存：',path)
    finally:cv2.destroyWindow(title)


def run_landing(args,config):
    from cart_grasp_alignment import open_head_camera
    from cart_book_grasp_v3_camera import WristCamera
    from book_top_tilt_servo import ContinuousRobotIO,servo_connection
    c=copy.deepcopy(config);d=settings(c)
    out=args.output or ROOT/'RobotRAM/output'/('v4_1_landing_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=True);print('V4.1 输出：',out)
    def log(record):
        record=dict(time_unix_s=time.time(),**record)
        with (out/'events.jsonl').open('a') as f:f.write(json.dumps(record,default=jsonable,ensure_ascii=False,allow_nan=False)+'\n')
        print('V4.1 搭接:',json.dumps(record,default=jsonable,ensure_ascii=False))
    write_json(out/'config_requested.json',c)
    wrist_camera=None;head_camera=None;io=None;head=None;wrist=None
    try:
        if getattr(args,'teach_contact',False):
            wrist_camera=WristCamera();ref=Path(d['contact_reference'])
            teach_contact(wrist_camera,c,ref if ref.is_absolute() else ROOT/ref);return
        head_camera=open_head_camera()
        if args.select:
            from cart_book_grasp_v4_1 import select_face
            select_face(head_camera,args.selection or ROOT/'v4_1_book_selection.npz');return
        if args.selection:raise ValueError('Old tilt selection lacks landing/pad semantics; use current-frame automatic selection')
        with (servo_connection() if args.execute else nullcontext()) as robot:
            try:
                auto=c.get('pad_source','auto_rgbd')=='auto_rgbd'
                if robot:
                    if auto:c['calibrated_gripper_position']=actual_aperture(robot)
                    io=ContinuousRobotIO(robot,c,log)
                    wrist_camera=WristCamera();wrist_camera.capture(io.guard)  # warm up before locking imagery
                selector=LandingSelector(args.target or c['target_description'],'head',require_pad=auto and args.execute)
                with (io.perception_pause() if io else nullcontext()):
                    head=LandingHeadVision.automatic(head_camera,selector,c,out/'head',io.guard if io else lambda:None)
                    h=head.stable(io.guard if io else lambda:None,include_pad=auto and args.execute)
                    head.save('first_verified')
                    log(dict(event='head_offsets',**head_geometry(h)))
                    if io:
                        state=io.guard()
                        if auto:
                            c['pad']=measured_pad(h['pad_corners_base_mm'],pose_mm(state),tool_rotation(state))
                            c['pad_calibrated']=True
                            log(dict(event='pad_auto_estimated',pad=c['pad'],source='visible_underside_RGBD',force_contact=False))
                        horizontal_pad(c,tool_rotation(state))
                        plan=coarse_plan(h,pose_mm(state),tool_rotation(state),c)
                        write_json(out/'coarse_plan.json',plan);write_json(out/'config_runtime.json',c)
                if not io:
                    for _ in range(0 if args.auto_select else args.frames):head.observe()
                    head.save('final');return
                # No use of the full tilt controller or bottom-pivot state machine.
                for goal in plan['waypoints_mm']:coarse_move(io,goal)
                with io.perception_pause():
                    h=head.stable(io.guard)
                    if np.linalg.norm(h['top_base_mm']-plan['top_base_mm'])>d['max_book_motion_mm']:
                        raise ValueError('Target book moved during coarse approach')
                    state=io.guard();pad=horizontal_pad(c,tool_rotation(state))
                    gap=pad.observe_book_point(h['top_base_mm'],pose_mm(state),tool_rotation(state))['signed_gap_mm']
                    if not 5<=gap<=d['coarse_clearance_mm']+4:raise ValueError('Coarse pad clearance not reached')
                    wrist=LandingWristVision(wrist_camera,c,out/'wrist',io)
                    # Auto mode must observe finger. Manual mode may use a
                    # validated taught contact pixel if actual pad is occluded.
                    wrist.initialize(LandingSelector(c['target_description'],'wrist',require_pad=auto),head.reference,io.guard)
                    obs,p,state=observe_pair(head,wrist,io,c)
                    if 'pad_contact_uv' in obs:goal_uv=obs['pad_contact_uv']
                    else:
                        ref=Path(d['contact_reference']);ref=ref if ref.is_absolute() else ROOT/ref
                        goal_uv=contact_reference(json.loads(ref.read_text()),wrist_camera,c)
                    wrist.save('first_verified')
                j,obs,p,state=probe_jacobian(head,wrist,io,c,log)
                controller=LandingController(c,obs['top_base_mm'],pose_mm(state),tool_rotation(state),j,goal_uv)
                log(dict(event='servo_start',mode='wrist_uv_head_gap',goal_uv=goal_uv,physical_contact_confirmed=False))
                for index in range(15000):
                    tick=time.monotonic();obs,p,state=observe_pair(head,wrist,io,c)
                    paired_r=Rotation.from_euler('xyz',p['angles'],degrees=True).as_matrix()
                    decision=controller.decide(obs,p['q'],paired_r,pose_mm(state),tool_rotation(state))
                    log(dict(event='servo',decision=decision,wrist_points=obs['wrist_points']))
                    if index%200==0:head.save();wrist.save()
                    if decision['phase']=='PAUSE_VERIFY':
                        landed = False
                        with io.perception_pause():
                            deadline=time.monotonic()+d['stable_seconds']+3.
                            while time.monotonic()<deadline:
                                obs,p,state=observe_pair(head,wrist,io,c)
                                decision=controller.decide(obs,p['q'],Rotation.from_euler('xyz',p['angles'],degrees=True).as_matrix(),
                                    pose_mm(state),tool_rotation(state),stationary=True)
                                if decision['phase']=='VERIFY_FAILED':raise ValueError('Visual landing not stable after hold; no further press')
                                if decision['phase']=='LANDED_VISUAL':
                                    report=dict(event='landed_visual',**decision,automatic_tilt=False,
                                        evidence='wrist relative landmark + head RGBD/pad geometry + stationary new frames')
                                    write_json(out/'landing_result.json',report);log(report);head.save('landed');wrist.save('landed')
                                    # Explicit terminal hold, no keepalive of an old motion goal.
                                    landed = True
                                    break
                                time.sleep(max(0.,1/c['visual_hz']))
                            if not landed:raise ValueError('Stationary landing verification timeout')
                        if landed:
                            io.cancel()
                            return
                    io.send(decision['target'],obs['captured_unix_s'])
                    time.sleep(max(0.,1/c['visual_hz']-(time.monotonic()-tick)))
                raise ValueError('Landing iteration budget')
            except BaseException as exc:
                report=dict(event='stopped',reason=f'{type(exc).__name__}: {exc}',physical_contact_confirmed=False)
                log(report);write_json(out/'landing_result.json',report)
                if io:
                    try:io.cancel()
                    except Exception as hold_error:log(dict(event='hold_unconfirmed',reason=str(hold_error)))
                raise
            finally:
                if io:io.close()
    finally:
        for vision in (head,wrist):
            if vision:
                try:vision.save('final')
                except Exception:pass
        if wrist_camera:wrist_camera.close()
        if head_camera:head_camera.close()
