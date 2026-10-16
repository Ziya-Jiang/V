#!/usr/bin/env python3
"""Right cart handle: GPT head ROI -> RGB-D coarse -> image Jacobian approach
-> close -> hold 3 s -> open -> -X 100 mm -> recorded initial right joints.

No motion unless --execute. Existing cart_grasp.py tools remain available.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
# Compatibility for the original marker tests/tools.
from cart_grasp import HandleMarker,make_marker,dictionary,select_reference_sample,annotate
from cart_grasp_alignment import load_config as load_alignment,coarse_target,coarse_move,align_loop,marker_error
from cart_grasp_depth import load_approach
from cart_book_grasp_v1_3_depth import forward_parameters

ROOT=Path(__file__).resolve().parent
CONFIG=ROOT/'cart_grap_v1_config.json'


def load_config(path=CONFIG):
    c=json.loads(Path(path).read_text())
    numbers=dict(gpt_timeout_seconds=(5,120),min_confidence=(.7,1),fingertip_offset_mm=(0,150),
        grasp_insertion_mm=(0,100),alignment_raise_mm=(0,50),coarse_forward_fraction=(0,.5),coarse_forward_max_mm=(0,125),
        hold_seconds=(3,3),withdraw_mm=(100,100),gripper_timeout_seconds=(2,10),open_threshold=(.01,.1))
    strings={'model','api_key_file','alignment_config','approach_config'}
    if set(c)!=set(numbers)|strings|{'gpt_depth_review_enabled'}:raise ValueError('Missing/unexpected cart cycle settings')
    if type(c['gpt_depth_review_enabled']) is not bool:raise ValueError('Invalid GPT depth review setting')
    for key,(lo,hi) in numbers.items():
        value=c[key]
        if type(value) not in (int,float) or not np.isfinite(value) or not lo<=value<=hi:
            raise ValueError('Invalid cart cycle setting: '+key)
    if c['model']!='gpt-6-astra' or any(not isinstance(c[k],str) or not c[k] for k in strings):
        raise ValueError('Invalid GPT/config settings')
    return c


def grasp_plan(head,initial,angles,c,a):
    """The user explicitly selected the existing book 100 mm insertion geometry."""
    point=np.asarray(head['point_base_mm'],float)
    if point.shape!=(3,) or not np.isfinite(point).all():raise ValueError('Invalid head marker base point')
    f=forward_parameters(initial,angles,point[0],dict(fingertip_offset_mm=c['fingertip_offset_mm'],
                                                   grasp_insertion_mm=c['grasp_insertion_mm']))
    if f['forward_mm']>a['max_forward_mm']:raise ValueError('Head-depth grasp endpoint exceeds forward budget')
    raised=np.asarray(initial,float).copy();raised[2]+=c['alignment_raise_mm']
    return dict(marker_base_xyz_mm=point.tolist(),goal_tcp_x_mm=f['grasp_tcp_x_mm'],
        alignment_raise_mm=c['alignment_raise_mm'],raise_target_mm=raised.tolist(),fixed_z_mm=float(raised[2]),
        fingertip_x_offset_mm=f['fingertip_x_offset_mm'],grasp_insertion_mm=c['grasp_insertion_mm'],
        forward_mm=f['forward_mm'],coarse_forward_mm=min(c['coarse_forward_max_mm'],c['coarse_forward_fraction']*f['forward_mm']),
        hold_seconds=c['hold_seconds'],withdraw_mm=c['withdraw_mm'])


def coarse_approach(io,plan,angles):
    """Raise first, then align Y and approach X; each stage must arrive."""
    if plan['alignment_raise_mm']>0:
        io.move(np.asarray(plan['raise_target_mm']),angles,'raise_alignment_height')
    io.move(np.asarray(plan['coarse_y_target_mm']),angles,'coarse_y')
    q=io.pose6()[:3].copy();q[0]+=plan['coarse_forward_mm'];q[2]=io.reference_z
    if plan['coarse_forward_mm']>0:io.move(q,angles,'coarse_forward')


def finish_cycle(io,c):
    """No automatic cleanup motion after an error; order is externally testable."""
    io.gripper(True)
    io.hold_seconds(c['hold_seconds'])
    io.gripper(False)  # Requires fresh measured OPEN before any withdrawal.
    io.withdraw(c['withdraw_mm'])
    io.return_initial()


class CycleIO:
    def __init__(self,robot,base,c,alignment,approach,out,log):
        from cart_book_grasp_v3_2_runtime import StreamTransport
        self.robot=robot;self.base=np.asarray(base);self.c=c;self.alignment=alignment
        self.approach=approach;self.out=out;self.log=log;self.commanded=False;self.last_image=None
        self.settings=dict(feedback_max_age_seconds=.25,frame_max_age_seconds=.25,ack_timeout_seconds=3.,
            max_pose_pair_gap_seconds=.1,max_pose_pair_uncertainty_mm=3.,max_pose_pair_uncertainty_deg=1.,pair_recovery_timeout_seconds=2.)
        self.transport=StreamTransport(robot,base,dict(max_orientation_error_deg=1.),self.settings,log=log)
        self.initial=copy.deepcopy(self.state());self.initial_q=np.asarray(self.initial['right_arm']['position'])*1000
        self.initial_joints=np.asarray(self.initial['joint_state'],float).reshape(2,7).copy()
        if not np.isfinite(self.initial_joints).all():raise ValueError('Invalid initial joints')
        # The return pose remains the original pre-lift pose, not this working plane.
        self.reference_z=self.initial_q[2]+c['alignment_raise_mm'];self.anchor=None

    def state(self):
        s=self.robot.get_current_state();now=time.monotonic()
        if s is None:raise ValueError('No robot feedback')
        for key in ('received_monotonic','pose_sample_monotonic'):
            if not np.isfinite(s[key]) or not 0<=now-s[key]<=.25:raise ValueError('Stale robot feedback')
        if getattr(s['robot_status'],'value',s['robot_status']) not in (0,1):raise ValueError('Robot unavailable')
        if getattr(s['curobo_status'],'value',s['curobo_status']) in (3,7,8):raise ValueError('Planner fault')
        return s

    def stationary(self):
        s=self.state();q=np.asarray(s['right_arm']['position'])*1000
        v=np.asarray(s['joint_velocity']).reshape(-1)
        quat=np.asarray(s['right_arm']['orientation']);initial=np.asarray(self.initial['right_arm']['orientation'])
        error=np.degrees((Rotation.from_quat(initial[[1,2,3,0]]).inv()*Rotation.from_quat(quat[[1,2,3,0]])).magnitude())
        if not np.isfinite(v).all() or max(abs(v))>.02 or np.linalg.norm(q-self.initial_q)>.5 or error>.25:
            raise ValueError('Robot moved while acquiring GPT/head geometry')

    def gripper_sample(self):
        """Existing CuroboState carries actual normalized PanelState gripper positions."""
        self.state()
        raw,stamp=self.robot.udp.receive_with_timestamp();now=time.monotonic()
        if raw is None or not 0<=now-stamp<=.25 or not 0<=now-raw.pose_sample_monotonic<=.25:
            raise ValueError('No fresh measured gripper feedback')
        q=np.asarray(raw.gripper_joint_position[1],float).reshape(-1)
        if q.size!=1 or not np.isfinite(q).all() or not -.01<=q[0]<=1.01:
            raise ValueError('Expected one normalized right gripper position')
        mode=raw.processed_gripper_mode_command[1]
        return float(q[0]),getattr(mode,'value',mode),float(raw.pose_sample_monotonic)

    def require_open(self):
        q,mode,_=self.gripper_sample()
        expected=self.robot.get_cat_gripper('open','book')
        if mode!=expected or q>self.c['open_threshold']:
            raise ValueError('Start with the right gripper physically open; no automatic opening of a held object')

    def move(self,q,angles,label):
        self.state();self.commanded=True
        self.log(dict(event=label,target_mm=np.asarray(q).tolist(),angles_deg=np.asarray(angles).tolist()))
        coarse_move(self.robot,q,angles)
        s=self.state();actual=np.asarray(s['right_arm']['position'])*1000
        if np.linalg.norm(actual-q)>3:raise ValueError(label+' actual position not within 3 mm')

    def send_visual(self,q,roll,blocking):
        from cart_book_grasp_v3_centerline import roll_orientation
        self.state();self.commanded=True
        self.robot._v3_servo_position_tolerance_mm=1. if blocking else 3.
        if self.robot.master_control(right_arm=[*q,*roll_orientation(self.base,roll)],
                action='book_v3_xyz_servo' if blocking else 'book_v3_1_stream',
                object_category='book',wait_for_completion=blocking) is not True:
            raise ValueError('Visual target rejected')
        if not blocking:
            self.transport.sent_at=time.monotonic()
            self.transport.pending=np.asarray(self.robot.target_command.arm_tool_command,float).copy()
        self.log(dict(event='visual_target',tcp_mm=np.asarray(q).tolist(),roll_deg=float(roll),blocking=blocking))

    def soft_hold(self):
        # Normal stop must not call book_v3_1_hold, which deliberately latches a fault.
        if not self.commanded:return
        p=self.transport.read();q=p['q'].copy();q[2]=self.reference_z
        if abs(p['q'][2]-self.reference_z)>self.alignment['fixed_xz_mm']:
            raise ValueError('Fixed Z exceeded; abort required instead of normal hold')
        self.send_visual(q,p['roll'],True);self.transport.pending=None
        # A completed finite probe still has a 5 s deadline in the adapter.
        # Transfer the same target to leased streaming before observing again.
        # If observations stall, the stream lease holds joints; it does not
        # renew a motion goal indefinitely or disable any controller guard.
        self.send_visual(q,p['roll'],False)

    def servo(self,camera,goal):
        from cart_book_grasp_v3_2_runtime import capture_paired_frame
        detector=HandleMarker(0,20.);last=-np.inf;count=0;deadline=time.monotonic()+self.alignment['max_seconds']
        origin=self.transport.read()['q'].copy();origin[2]=self.reference_z
        def observe(checkpoint):
            nonlocal last,count
            frame,p,_,_,_=capture_paired_frame(self.transport,camera,checkpoint,self.log,last,
                clock=time.monotonic,sleep=time.sleep,total_deadline=deadline)
            last=frame['captured_at'];bgr=cv2.cvtColor(frame['rgb'],cv2.COLOR_RGB2BGR)
            marker=detector.detect(bgr,last);checkpoint()
            if time.monotonic()-last>.25:raise ValueError('Wrist image stale after ArUco detection')
            self.log(dict(event='wrist_marker',**marker))
            if count%200==0:cv2.imwrite(str(self.out/f'wrist_{count:06d}.png'),annotate(bgr,marker))
            count+=1
            e=marker_error(marker);p['marker_scale_px']=float(np.sqrt(marker['area_px2']))
            self.last_image=dict(stamp=last,error=e.copy())
            return e,p
        result=align_loop(self.transport.read,observe,self.send_visual,self.transport.ready,self.soft_hold,
            origin,dict(self.alignment,max_path_mm=self.approach['max_path_mm']),self.log,
            goal_x=goal,approach=self.approach)
        # Fresh validation after the final hold, before closing. A held frame is not sufficient.
        for _ in range(3):
            e,p=observe(lambda:self.state())
            if abs(e[0])>self.alignment['tolerance_px'] or abs(e[1])>self.alignment['tolerance_angle_deg']:
                raise ValueError('Final wrist alignment changed before closure')
            if abs(p['q'][0]-goal)>self.approach['x_tolerance_mm'] or abs(p['q'][2]-self.reference_z)>3:
                raise ValueError('Final measured grasp endpoint not confirmed')
        self.anchor=self.pose6();self.anchor[2]=self.reference_z
        return result

    def pose6(self):
        s=self.state();a=s['right_arm'];quat=np.asarray(a['orientation'])
        angles=Rotation.from_quat(quat[[1,2,3,0]]).as_euler('xyz',degrees=True)
        return np.r_[np.asarray(a['position'])*1000,angles]

    def keep_anchor(self):
        p=self.pose6()
        if np.linalg.norm(p[:3]-self.anchor[:3])>3 or np.degrees((Rotation.from_euler('xyz',p[3:],degrees=True).inv()*Rotation.from_euler('xyz',self.anchor[3:],degrees=True)).magnitude())>1:
            raise ValueError('TCP drift while holding gripper')
        if self.robot.master_control(right_arm=self.anchor.tolist(),action='book_v3_1_stream',
                object_category='book',wait_for_completion=False) is not True:raise ValueError('Anchor hold rejected')

    def gripper(self,close):
        self.keep_anchor();before,_,_=self.gripper_sample();sent=time.monotonic()
        # Send grip + SAME anchored pose as a blocking visual command, never retain a cancellation action.
        self.robot._v3_servo_position_tolerance_mm=3.
        if self.robot.master_control(right_arm=self.anchor.tolist(),right_gripper=int(close),
                action='book_v3_xyz_servo',object_category='book') is not True:raise ValueError('Gripper/hold send failed')
        expected=self.robot.get_cat_gripper('close' if close else 'open','book')
        deadline=time.monotonic()+self.c['gripper_timeout_seconds'];stable=None;last=None;last_stamp=-np.inf
        while time.monotonic()<deadline:
            self.keep_anchor();q,mode,stamp=self.gripper_sample()
            if stamp>last_stamp and stamp>sent:
                last_stamp=stamp
                reached=(q>=before+.05) if close else q<=self.c['open_threshold']
                if mode==expected and reached:
                    if last is None or abs(q-last)>.01:stable=time.monotonic()
                    elif stable is not None and time.monotonic()-stable>=.3 and time.monotonic()-sent>=.8:
                        self.log(dict(event='close_feedback_settled' if close else 'open_measured',position=q,
                            grasp_force_verified=False));return
                    last=q
                else:stable=None;last=None
            time.sleep(.05)
        raise ValueError('Right gripper '+('closure' if close else 'opening')+' feedback timeout; no retreat')

    def hold_seconds(self,seconds):
        self.log(dict(event='hold_start',seconds=seconds));until=time.monotonic()+seconds
        while time.monotonic()<until:self.keep_anchor();time.sleep(min(.05,max(0,until-time.monotonic())))
        self.log(dict(event='hold_complete',seconds=seconds))

    def withdraw(self,mm):
        self.require_open();pose=self.pose6();target=pose[:3].copy();target[0]-=mm
        self.move(target,pose[3:],'withdraw_minus_x')

    def return_initial(self):
        self.require_open();robot=self.robot
        old_joint=robot.right_safe_joint_deg;old_pose=copy.deepcopy(robot.intermediate_reset_pos[1])
        old_wait=robot._wait_for_robot_stop
        def wait(**kwargs):
            kwargs['joint_tolerance_rad']=np.deg2rad(.3);return old_wait(**kwargs)
        try:
            robot.right_safe_joint_deg=np.rad2deg(self.initial_joints[1])
            robot.intermediate_reset_pos[1]=[*self.initial_q,*self.initial['right_arm']['orientation']]
            robot._wait_for_robot_stop=wait
            if robot.control_joint_position(False,True,'cart_return_initial',force=True) is not True:
                raise ValueError('Return to initial right joints failed')
        finally:
            robot.right_safe_joint_deg=old_joint;robot.intermediate_reset_pos[1]=old_pose;robot._wait_for_robot_stop=old_wait
        s=self.state();j=np.asarray(s['joint_state']).reshape(2,7)
        if max(abs(j[1]-self.initial_joints[1]))>np.deg2rad(.3) or np.linalg.norm(np.asarray(s['right_arm']['position'])*1000-self.initial_q)>3:
            raise ValueError('Measured return to initial position not confirmed')
        self.log(dict(event='returned_initial',right_joint_rad=j[1].tolist()))

    def abort(self):
        if self.commanded:self.transport.abort()


def run(args,c):
    from servor import robot_connection
    from cart_book_grasp_v3_camera import WristCamera
    from cart_grap_v1_perception import locate
    alignment=load_alignment(ROOT/c['alignment_config']);approach=load_approach(ROOT/c['approach_config'])
    out=args.out or ROOT/'output'/('cart_grap_v1_'+str(time.time_ns()));out.mkdir(parents=True,exist_ok=False)
    with (out/'events.jsonl').open('w') as file:
        def log(e):
            file.write(json.dumps(e,ensure_ascii=False,allow_nan=False)+'\n');file.flush()
            if e['event']!='wrist_marker':print(json.dumps(e,ensure_ascii=False),flush=True)
        io=None;camera=None
        try:
            with robot_connection() as (robot,base):
                io=CycleIO(robot,base,c,alignment,approach,out,log)
                try:
                    io.require_open()
                    # Warm the wrist stream before acquiring a time-sensitive head target.
                    camera=WristCamera();camera.capture(io.stationary)
                    head=locate(out,c,checkpoint=io.stationary)
                    plan=grasp_plan(head,io.initial_q,base,c,approach)
                    target=coarse_target(plan['raise_target_mm'],head['point_base_mm'],alignment)
                    plan.update(initial_xyz_mm=io.initial_q.tolist(),initial_right_joints_rad=io.initial_joints[1].tolist(),
                        coarse_y_target_mm=target.tolist(),execute=args.execute)
                    (out/'plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2));log(dict(event='plan',**plan))
                    if not args.execute:return 0
                    io.stationary()
                    if not 0<=time.time()-head['captured_unix_s']<=1:raise ValueError('Head target expired before first motion')
                    coarse_approach(io,plan,base)
                    io.servo(camera,plan['goal_tcp_x_mm'])
                    finish_cycle(io,c)
                    log(dict(event='cycle_complete',grasp_force_verified=False,out=str(out)))
                    return 0
                except BaseException:
                    try:io.abort()
                    except Exception as exc:log(dict(event='hold_unconfirmed',reason=str(exc)))
                    raise
                finally:
                    if camera:camera.close()
        except (Exception,KeyboardInterrupt) as exc:
            log(dict(event='stopped',reason=f'{type(exc).__name__}: {exc}',automatic_open_or_return=False));return 1


def main(argv=None):
    argv=sys.argv[1:] if argv is None else argv
    if argv and argv[0] in ('marker','observe','teach','align'):
        from cart_grasp import main as tools_main
        return tools_main(argv)
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true',help='执行完整单轮；默认仅 GPT/RGB-D 定位并保存方案')
    p.add_argument('--check-config',action='store_true',help='离线检查配置')
    p.add_argument('--config',type=Path,default=CONFIG);p.add_argument('--out',type=Path)
    a=p.parse_args(argv);c=load_config(a.config)
    if a.check_config:
        load_alignment(ROOT/c['alignment_config']);load_approach(ROOT/c['approach_config'])
        print(json.dumps(c,ensure_ascii=False,indent=2));return 0
    return run(a,c)


if __name__=='__main__':raise SystemExit(main())
