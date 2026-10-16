"""V4.1 closed fingers -> head Y/raise/X -> wrist image-centre Y alignment.
No device, network, or motion is opened on import. No descent/contact/pull route.
"""
import copy
import json
from contextlib import nullcontext
from pathlib import Path
import time
import numpy as np
from cart_book_grasp_v4_1 import ROOT, load_config as base_config, pose_mm, tool_rotation, write_json, jsonable
from book_top_coarse_alignment import settings, coarse_plan, segments, centerline_u, fit_response, Alignment


def runtime_config(c):
    out=base_config();out['coarse_alignment']=settings(c)
    out.update(target_description=c['target_description'],pad_visual_required=False,
        step_mm=3.,orientation_error_deg=1.,max_seconds=600.)
    out['landing']['coarse_timeout_seconds']=60.
    return out


def read_state(robot):
    s=robot.get_current_state()
    if not s:raise ValueError('No V4.1 feedback')
    for k in ('received_monotonic','pose_sample_monotonic'):
        if not 0<=time.monotonic()-s[k]<=.25:raise ValueError('Stale V4.1 feedback')
    if getattr(s['robot_status'],'value',s['robot_status']) not in (0,1):raise ValueError('Robot unavailable')
    status=getattr(s['curobo_status'],'value',s['curobo_status'])
    if status in (3,7,8):
        name={3:'PLANNING_FAILED',7:'IK_FAILED',8:'COLLISION_CHECK_FAILED'}[status]
        sent=getattr(robot,'_v41_prepare_command_sent',False)
        phase=('本轮准备指令发出后' if sent
               else '本轮闭指/运动指令尚未发送')
        detail=('请查看后端本次 gripper preparation rejected 等具体拒绝原因。' if sent else
                '后端可能仍处于上一轮终止保持/故障锁存；查看后端最早的停止原因。')
        raise ValueError(f'V4.1 backend status={status} ({name}); {phase}。'
                         +detail+
                         '确认现场后重启独立V4.1后端再运行；程序不会自动清除故障。')
    return s


def close_fingers(robot,c,log):
    from cuarm_state import TargetCommandArmCommandType, TargetCommandPlannerMode
    from book_top_landing_runtime import actual_aperture
    d=settings(c);s=read_state(robot);initial=copy.deepcopy(s)
    velocity=np.asarray(s.get('joint_velocity',[]),float)
    if velocity.shape!=(14,) or not np.isfinite(velocity).all() or max(abs(velocity))>.005:
        raise ValueError('Close fingers only with arms stationary')
    cmd=copy.deepcopy(robot.target_command)
    cmd.arm_command_type=TargetCommandArmCommandType.CARTESIAN
    cmd.planner_mode=TargetCommandPlannerMode.PLANNING
    cmd.arm_tool_command=[[*s[a]['position'],*s[a]['orientation']] for a in ('left_arm','right_arm')]
    cmd.gripper_mode_command[1]=robot.get_cat_gripper('close','book')
    cmd.description='book_v4_1_close';robot.target_command=cmd
    sent=time.monotonic();robot.udp.send(cmd);robot._v41_prepare_command_sent=True;stable=[];last=-np.inf
    log(dict(event='close_fingers_requested',arm_motion=False))
    while time.monotonic()-sent<d['gripper_timeout_seconds']:
        s=read_state(robot)
        if any(np.linalg.norm(np.array(s[a]['position'])-initial[a]['position'])>.001 for a in ('left_arm','right_arm')):
            raise ValueError('Arm moved during gripper-only preparation')
        opening=actual_aperture(robot);stamp=s['pose_sample_monotonic']
        mode=getattr(s['right_gripper'],'value',s['right_gripper'])
        expected=getattr(cmd.gripper_mode_command[1],'value',cmd.gripper_mode_command[1])
        if stamp>last and stamp>sent:
            last=stamp
            if mode==expected and opening>=d['closed_threshold']:
                stable.append((stamp,opening))
                if len(stable)>=3 and stamp-stable[0][0]>=.3 and np.ptp([x[1] for x in stable])<=.01:
                    log(dict(event='closed_fingers_confirmed',measured_opening=opening));return opening
            else:stable=[]
        time.sleep(.02)
    raise ValueError('Closed fingers not confirmed by actual opening; no arm movement started')


def wrist_observation(wrist,io):
    o=wrist.observe(io.guard)
    o['u']=centerline_u(wrist.tracker.edge_midpoints,wrist.camera.size)
    paired=io.pair(o);s=io.guard()
    return o,paired,pose_mm(s)


def servo_align(wrist,io,c,log):
    d=settings(c)
    with io.perception_pause():o,p,q=wrist_observation(wrist,io)
    origin=q.copy();ys=[p['q'][1]];us=[o['u']]
    # Symmetric return checks response consistency, using measured displacement.
    for goal_y in (origin[1]+d['probe_mm'],origin[1]):
        until=time.monotonic()+c['motion_timeout_seconds']
        while time.monotonic()<until:
            tick=time.monotonic();o,p,q=wrist_observation(wrist,io)
            if max(abs(q[[0,2]]-origin[[0,2]]))>d['fixed_xz_mm']:
                raise ValueError('Probe fixed X/Z drift')
            if abs(q[1]-origin[1])>d['max_servo_y_mm']:raise ValueError('Probe Y envelope')
            if abs(q[1]-goal_y)<.4:break
            goal=origin.copy();goal[1]=goal_y;delta=goal-q
            delta*=min(1.,d['max_y_step_mm']/max(np.linalg.norm(delta),1e-12))
            io.send(q+delta,o['captured_unix_s'])
            time.sleep(max(0,1/d['update_hz']-(time.monotonic()-tick)))
        else:raise ValueError('Measured Y probe displacement timeout')
        with io.perception_pause():o,p,q=wrist_observation(wrist,io)
        ys.append(p['q'][1]);us.append(o['u'])
    jac,err=fit_response(ys,us,c);target_u=(wrist.camera.size[0]-1)/2
    log(dict(event='jacobian_y',pixels_per_mm=jac,max_fit_error_px=err,actual_y_mm=ys,u_px=us,target_u=target_u))
    ctrl=Alignment(c,origin,jac,target_u);until=time.monotonic()+d['servo_timeout_seconds']
    index=0
    while time.monotonic()<until:
        tick=time.monotonic();o,p,q=wrist_observation(wrist,io);decision=ctrl.decide(o,p['q'],q)
        log(dict(event='wrist_alignment',**decision,points=o['wrist_points']))
        if index%200==0:wrist.save('alignment_'+str(index))
        index+=1
        if decision['done']:
            # Stop the rolling target, then verify on new stationary images.
            with io.perception_pause():
                verified=[];until_verify=time.monotonic()+3.
                while time.monotonic()<until_verify:
                    o,p,q=wrist_observation(wrist,io);v=ctrl.decide(o,p['q'],q)
                    if abs(v['error_px'])>d['tolerance_px']:break
                    verified.append(o['captured_unix_s'])
                    if len(verified)>=d['stable_frames'] and verified[-1]-verified[0]>=d['stable_seconds']:
                        wrist.save('aligned');return dict(status='aligned',target_u=target_u,
                            error_px=v['error_px'],tcp_mm=q,contact_confirmed=False)
                    time.sleep(1/d['update_hz'])
            ctrl.stamps=[]
            continue
        delta=decision['target']-q
        delta*=min(1.,d['max_y_step_mm']/max(np.linalg.norm(delta),1e-12))
        io.send(q+delta,o['captured_unix_s'])
        time.sleep(max(0,1/d['update_hz']-(time.monotonic()-tick)))
    raise ValueError('Wrist centreline alignment timeout')


def execute_sequence(io,c,detect_head,move,align,log):
    """Hardware-independent orchestration; injected in offline sequence tests."""
    head,h=detect_head()
    plan=coarse_plan(h,pose_mm(io.guard()),io.rotation,c)
    log(dict(event='head_coarse_plan',**plan))
    for stage,goal in plan['stages']:
        start=pose_mm(io.guard())
        log(dict(event='stage',name=stage,target_mm=goal))
        original_guard=io.guard
        fixed_axes={'align_y':[0,2],'raise_z':[0,1],'forward_x':[1,2]}[stage]
        def stage_guard():
            state=original_guard()
            if max(abs(pose_mm(state)[fixed_axes]-goal[fixed_axes]))>settings(c)['coarse_axis_tolerance_mm']:
                raise ValueError('Coarse '+stage+' departed from held axes')
            return state
        io.guard=stage_guard
        try:
            for target in segments(start,goal,settings(c)['coarse_step_mm']):
                if np.linalg.norm(target-pose_mm(io.guard()))>.05:move(io,target)
        finally:io.guard=original_guard
        # Head checks use new images of the originally selected book, not a reselection.
        current=head.stable(io.guard)
        if np.linalg.norm(current['top_base_mm']-h['top_base_mm'])>c['landing']['max_book_motion_mm']:
            raise ValueError('Selected book moved during coarse preparation')
    return align(head)


def run(args,config):
    if args.execute and config.get('head_camera_review_required',False):
        raise ValueError('当前共享Orbbec视角已被确认不是目标头部视角；'
                         '需先核对正确图像源和对应手眼标定。可用--observe-head查看，不发送闭指或运动命令。')
    from cart_grasp_alignment import open_head_camera
    from cart_book_grasp_v3_camera import WristCamera
    from book_top_landing_vision import LandingSelector,LandingHeadVision,LandingWristVision
    from book_top_landing_runtime import coarse_move
    from book_top_tilt_servo import servo_connection,ContinuousRobotIO
    c=runtime_config(config);out=args.output or ROOT/'RobotRAM/output'/('v4_1_prepare_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    def log(r):
        r=dict(unix_s=time.time(),**r)
        with (out/'events.jsonl').open('a') as f:f.write(json.dumps(r,default=jsonable,ensure_ascii=False,allow_nan=False)+'\n')
        print('V4.1 准备:',json.dumps(r,default=jsonable,ensure_ascii=False),flush=True)
    write_json(out/'config.json',c);io=None;head_camera=None;wrist_camera=None;head=None;wrist=None
    try:
        with (servo_connection() if args.execute else nullcontext()) as robot:
            try:
                if robot:
                    aperture=close_fingers(robot,c,log)
                    c['calibrated_gripper_position']=aperture
                    io=ContinuousRobotIO(robot,c,log)
                head_camera=open_head_camera()
                def detect():
                    nonlocal head
                    guard=io.guard if io else lambda:None
                    # Ordinary stationary hold is still active; do NOT arm the
                    # local XYZ visual guard before long coarse translations.
                    if io:io.observation_anchor=pose_mm(io.guard())
                    try:
                        selector=LandingSelector(c['target_description'],'head',require_pad=False,book_only=True,upper_edge_only=True)
                        head=LandingHeadVision.automatic(head_camera,selector,c,out/'head',guard)
                        h=head.stable(guard);head.save('head_upper_edge')
                    finally:
                        if io:io.observation_anchor=None
                    return head,h
                if not robot:
                    from book_top_landing import head_geometry
                    head,h=detect();write_json(out/'head_geometry.json',head_geometry(h))
                    print('只感知头部上沿，未连接机器人控制；结果：',out);return
                def align(head):
                    nonlocal wrist,wrist_camera
                    wrist_camera=WristCamera();wrist_camera.capture(io.guard)
                    with io.perception_pause():
                        wrist=LandingWristVision(wrist_camera,c,out/'wrist',io)
                        wrist.initialize(LandingSelector(c['target_description'],'wrist',require_pad=False,book_only=True),head.reference,io.guard)
                        wrist.save('first_verified')
                    return servo_align(wrist,io,c,log)
                result=execute_sequence(io,c,detect,coarse_move,align,log)
                log(dict(event='completed',**result));write_json(out/'result.json',result)
                io.cancel()  # explicit terminal hold, leaves fingers closed
            except BaseException as exc:
                log(dict(event='stopped',reason=f'{type(exc).__name__}: {exc}'))
                write_json(out/'result.json',dict(status='stopped',reason=str(exc),contact_confirmed=False))
                if io:
                    try:io.cancel()
                    except Exception as e:log(dict(event='hold_unconfirmed',reason=str(e)))
                elif robot and getattr(robot,'_v41_prepare_command_sent',False):
                    # A gripper failure must still stop this private session.
                    try:
                        from scipy.spatial.transform import Rotation
                        s=robot.get_current_state()
                        if s:robot.master_control(right_arm=[*pose_mm(s),*Rotation.from_matrix(tool_rotation(s)).as_euler('xyz',degrees=True)],
                            action='book_v3_1_hold',object_category='book',wait_for_completion=False)
                    except Exception as e:log(dict(event='hold_unconfirmed',reason=str(e)))
                elif robot:
                    log(dict(event='startup_rejected_no_command_sent',hold_sent=False))
                raise
            finally:
                if io:io.close()
    finally:
        for camera in (wrist_camera,head_camera):
            if camera:camera.close()
