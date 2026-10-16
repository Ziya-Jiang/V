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
from cart_grasp_alignment import load_config as load_alignment,coarse_target,coarse_move,marker_error
from cart_grap_v1_1_alignment import align_loop
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


def first_reset(robot,c,log):
    """Explicit fresh right-arm reset through CuRobo, before recording origin."""
    def feedback():
        s=robot.get_current_state();now=time.monotonic()
        if s is None:raise ValueError('No feedback for first_reset')
        for key in ('received_monotonic','pose_sample_monotonic'):
            if not np.isfinite(s[key]) or not 0<=now-s[key]<=.25:
                raise ValueError('Stale feedback for first_reset')
        if getattr(s['robot_status'],'value',s['robot_status']) not in (0,1):
            raise ValueError('Robot unavailable for first_reset')
        q=np.asarray(s['joint_state'],float).reshape(2,7)
        v=np.asarray(s['joint_velocity'],float).reshape(2,7)
        if not np.isfinite([q,v]).all():raise ValueError('Invalid reset joint feedback')
        return s,q,v
    s,_,v=feedback()
    if np.max(abs(v))>.02:raise ValueError('Wait for stationary arms before first_reset')
    # Preserve gripper commands; do not reset with a held object or open it automatically.
    raw,stamp=robot.udp.receive_with_timestamp();now=time.monotonic()
    if raw is None or not 0<=now-stamp<=.25 or not 0<=now-raw.pose_sample_monotonic<=.25:
        raise ValueError('No fresh gripper feedback for first_reset')
    grip=np.asarray(raw.gripper_joint_position[1],float).reshape(-1)
    mode=raw.processed_gripper_mode_command[1]
    if (grip.size!=1 or not np.isfinite(grip).all() or not -.01<=grip[0]<=c['open_threshold'] or
            getattr(mode,'value',mode)!=robot.get_cat_gripper('open','book')):
        raise ValueError('Right gripper must be physically open before first_reset')
    target=np.deg2rad(np.asarray(robot.right_safe_joint_deg,float))
    if target.shape!=(7,) or not np.isfinite(target).all():raise ValueError('Invalid first_reset target')
    log(dict(event='first_reset_start',previous_planner_status=int(getattr(s['curobo_status'],'value',s['curobo_status'])),
             right_joint_target_rad=target.tolist()))
    old_wait=robot._wait_for_robot_stop
    def wait(**kwargs):
        kwargs['joint_tolerance_rad']=np.deg2rad(.3)
        return old_wait(**kwargs)
    sent=time.monotonic()
    try:
        robot._wait_for_robot_stop=wait
        # Force a new command even when already near home; never reuse a stale
        # cancelled command/status. CuRobo still plans/checks the reset normally.
        if robot.control_joint_position(False,True,'first_reset',force=True) is not True:
            raise ValueError('first_reset failed; grasp cancelled')
        deadline=time.monotonic()+2.;stable_since=None;count=0;last=-np.inf
        while time.monotonic()<deadline:
            s,q,v=feedback();source=s['pose_sample_monotonic']
            if getattr(s['curobo_status'],'value',s['curobo_status']) in (3,7,8):
                raise ValueError('Planner rejected first_reset')
            if source>max(sent,last):
                last=source
                if (getattr(s['robot_status'],'value',s['robot_status'])==0 and
                        getattr(s['curobo_status'],'value',s['curobo_status'])==9 and
                        np.max(abs(q[1]-target))<=np.deg2rad(.3) and np.max(abs(v))<=.02):
                    if stable_since is None:stable_since=source
                    count+=1
                    if count>=3 and source-stable_since>=.2:
                        quat=np.asarray(s['right_arm']['orientation'])
                        base=Rotation.from_quat(quat[[1,2,3,0]]).as_euler('xyz',degrees=True)
                        log(dict(event='first_reset_confirmed',right_joint_rad=q[1].tolist(),
                            tcp_mm=(np.asarray(s['right_arm']['position'])*1000).tolist()))
                        return base
                else:stable_since=None;count=0
            time.sleep(.02)
        raise ValueError('first_reset stable feedback timed out')
    except BaseException:
        try:
            s,_,_=feedback();arm=s['right_arm'];quat=np.asarray(arm['orientation'])
            angles=Rotation.from_quat(quat[[1,2,3,0]]).as_euler('xyz',degrees=True)
            robot.master_control(right_arm=[*(np.asarray(arm['position'])*1000),*angles],
                action='book_v3_1_hold',object_category='book',wait_for_completion=False)
        except Exception as exc:log(dict(event='reset_hold_unconfirmed',reason=str(exc)))
        raise
    finally:robot._wait_for_robot_stop=old_wait


def tcp_stability_metrics(state,reference):
    q=np.asarray(state['right_arm']['position'],float)*1000
    origin=np.asarray(reference['right_arm']['position'],float)*1000
    quat=np.asarray(state['right_arm']['orientation'],float)
    initial=np.asarray(reference['right_arm']['orientation'],float)
    v=np.asarray(state['joint_velocity'],float).reshape(-1)
    if (q.shape!=(3,) or origin.shape!=(3,) or quat.shape!=(4,) or initial.shape!=(4,) or
            v.shape!=(14,) or not np.isfinite(np.r_[q,origin,quat,initial,v]).all() or
            np.linalg.norm(quat)<1e-6 or np.linalg.norm(initial)<1e-6):
        raise ValueError('Invalid TCP stability feedback')
    angle=np.degrees((Rotation.from_quat(initial[[1,2,3,0]]).inv()*
                     Rotation.from_quat(quat[[1,2,3,0]])).magnitude())
    return dict(position_delta_mm=float(np.linalg.norm(q-origin)),orientation_delta_deg=float(angle),
                max_joint_speed_rad_s=float(np.max(abs(v))))


def tcp_is_stable(metrics):
    return (metrics['position_delta_mm']<=.5 and metrics['orientation_delta_deg']<=.25 and
            metrics['max_joint_speed_rad_s']<=.02)


def wait_tcp_stable(read,log,*,timeout=8.,stable_seconds=1.,position_limit_mm=.15,
                    orientation_limit_deg=.1,joint_speed_limit_rad_s=.005):
    """Observe only; reserve margin for acquisition instead of using its full tolerance."""
    started=time.monotonic();deadline=started+timeout
    reference=None;last=-np.inf;count=0;since=None;metrics=None;peak={}
    log(dict(event='tcp_settle_start',timeout_seconds=timeout,stable_seconds=stable_seconds,
        position_limit_mm=position_limit_mm,orientation_limit_deg=orientation_limit_deg,
        joint_speed_limit_rad_s=joint_speed_limit_rad_s))
    while time.monotonic()<deadline:
        state=read();now=time.monotonic();source=float(state['pose_sample_monotonic'])
        if not np.isfinite(source) or not 0<=now-source<=.25:
            raise ValueError('Stale pose during TCP settling')
        if source>last and source>=started:
            last=source
            if reference is None:reference=copy.deepcopy(state);since=source;count=0
            metrics=tcp_stability_metrics(state,reference)
            for key,value in metrics.items():peak[key]=max(peak.get(key,0.),value)
            if (metrics['position_delta_mm']<=position_limit_mm and
                    metrics['orientation_delta_deg']<=orientation_limit_deg and
                    metrics['max_joint_speed_rad_s']<=joint_speed_limit_rad_s and
                    getattr(state.get('robot_status',0),'value',state.get('robot_status',0))==0):
                count+=1
                if count>=3 and source-since>=stable_seconds:
                    log(dict(event='tcp_settle_confirmed',**metrics,samples=count,
                        stable_seconds=source-since,elapsed_seconds=now-started,
                        tcp_mm=(np.asarray(state['right_arm']['position'])*1000).tolist()))
                    return copy.deepcopy(state)
            else:
                reference=None;since=None;count=0
        time.sleep(.02)
    details=dict(metrics or {})
    log(dict(event='tcp_settle_timeout',**details,peak=peak,samples=count,
        stable_seconds=0. if since is None else max(0,last-since),timeout_seconds=timeout))
    if metrics is None:raise ValueError('TCP settling timed out: no new pose samples')
    raise ValueError('TCP settling timed out: '
        f'position_delta={metrics["position_delta_mm"]:.3f}mm (limit={position_limit_mm:.3f}mm), '
        f'orientation_delta={metrics["orientation_delta_deg"]:.3f}deg (limit={orientation_limit_deg:.3f}deg), '
        f'max_joint_speed={metrics["max_joint_speed_rad_s"]:.5f}rad/s (limit={joint_speed_limit_rad_s:.5f}rad/s); '
        f'peak=({peak["position_delta_mm"]:.3f}mm, {peak["orientation_delta_deg"]:.3f}deg, '
        f'{peak["max_joint_speed_rad_s"]:.5f}rad/s); '
        f'continuous stability required={stable_seconds:.1f}s')


class CycleIO:
    def __init__(self,robot,base,c,alignment,approach,out,log,*,reset_target=None,defer_acquisition=False):
        from cart_book_grasp_v3_2_runtime import StreamTransport
        self.robot=robot;self.base=np.asarray(base);self.c=c;self.alignment=alignment
        self.approach=approach;self.out=out;self.log=log;self.commanded=False;self.last_image=None
        self.reset_target=None if reset_target is None else np.asarray(reset_target,float).copy()
        self.acquisition_ready=False
        self.settings=dict(feedback_max_age_seconds=.25,frame_max_age_seconds=.25,ack_timeout_seconds=3.,
            max_pose_pair_gap_seconds=.1,max_pose_pair_uncertainty_mm=3.,max_pose_pair_uncertainty_deg=1.,pair_recovery_timeout_seconds=2.)
        # A provisional envelope supports cancellation, but cannot authorize acquisition.
        initial=self.pre_acquisition() if defer_acquisition else wait_tcp_stable(self.pre_acquisition,log)
        self.transport=StreamTransport(robot,self.base,dict(max_orientation_error_deg=1.),self.settings,log=log)
        self._record_origin(initial)
        self.acquisition_ready=not defer_acquisition

    def _record_origin(self,state):
        self.initial=copy.deepcopy(state)
        quat=np.asarray(self.initial['right_arm']['orientation'])
        self.base=Rotation.from_quat(quat[[1,2,3,0]]).as_euler('xyz',degrees=True)
        self.transport.base=self.base.copy()
        self.transport.last_angles=self.base.copy()
        self.transport.last_state=copy.deepcopy(self.initial)
        self.initial_q=np.asarray(self.initial['right_arm']['position'])*1000
        self.initial_joints=np.asarray(self.initial['joint_state'],float).reshape(2,7).copy()
        if not np.isfinite(self.initial_joints).all():raise ValueError('Invalid initial joints')
        self.reference_z=self.initial_q[2]+self.c['alignment_raise_mm'];self.anchor=None

    def pre_acquisition(self):
        state=self.state()
        target=getattr(self,'reset_target',None)
        if target is not None:
            q=np.asarray(state['joint_state'],float).reshape(2,7)[1]
            if target.shape!=(7,) or not np.isfinite(np.r_[q,target]).all():
                raise ValueError('Invalid acquisition reset joints')
            error=float(np.degrees(np.max(abs(q-target))))
            if error>.3:
                self.log(dict(event='acquisition_reset_drift',max_joint_error_deg=error,limit_deg=.3,
                    actual_joint_rad=q.tolist(),target_joint_rad=target.tolist()))
                raise ValueError(f'Robot left first_reset during acquisition: joint_error={error:.3f}deg (limit=0.300deg)')
            # first_reset already confirmed arrival. MOTION_FINISHED is derived
            # on every feedback frame, not latched; it can fall back to the
            # underlying PLANNING_SUCCEEDED while the same goal remains active.
            status=getattr(state['curobo_status'],'value',state['curobo_status'])
            if status not in (4,9):
                self.log(dict(event='acquisition_reset_status_rejected',planner_status=status,
                    max_joint_error_deg=error))
                raise ValueError(f'Unexpected planner state after first_reset: status={status}')
            echoed=np.asarray(state.get('processed_arm_joint_command',[]),float)
            if echoed.shape!=(2,7) or not np.isfinite(echoed).all():
                raise ValueError(f'Invalid first_reset target feedback: status={status}')
            target_error=float(np.max(abs(echoed[1]-target)))
            if target_error>1e-5:
                self.log(dict(event='acquisition_reset_target_changed',planner_status=status,
                    target_error_rad=target_error,processed_right_joint_rad=echoed[1].tolist(),
                    expected_right_joint_rad=target.tolist()))
                raise ValueError(f'first_reset target changed before acquisition: status={status}, '
                    f'target_error={target_error:.6f}rad')
            if status==4 and not getattr(self,'_reset_success_status_logged',False):
                self.log(dict(event='acquisition_reset_status',planner_status=status,
                    same_reset_target=True,max_joint_error_deg=error))
                self._reset_success_status_logged=True
        return state

    def prepare_acquisition(self,camera):
        # Called once, with the wrist already warm and the head camera open.
        # Discard warm-up RGB-D; the next frame becomes the GPT source image.
        if self.acquisition_ready:raise ValueError('Acquisition anchor already locked')
        self.pre_acquisition();camera.capture_image();self.pre_acquisition()
        self.log(dict(event='acquisition_cameras_warmed'))
        settled=wait_tcp_stable(self.pre_acquisition,self.log)
        self._record_origin(settled)
        self.acquisition_ready=True
        self.log(dict(event='acquisition_anchor_locked',tcp_mm=self.initial_q.tolist(),
            pose_sample_monotonic=float(settled['pose_sample_monotonic']),
            right_joint_rad=self.initial_joints[1].tolist()))

    def state(self):
        s=self.robot.get_current_state();now=time.monotonic()
        if s is None:raise ValueError('No robot feedback')
        for key in ('received_monotonic','pose_sample_monotonic'):
            if not np.isfinite(s[key]) or not 0<=now-s[key]<=.25:raise ValueError('Stale robot feedback')
        if getattr(s['robot_status'],'value',s['robot_status']) not in (0,1):raise ValueError('Robot unavailable')
        if getattr(s['curobo_status'],'value',s['curobo_status']) in (3,7,8):raise ValueError('Planner fault')
        return s

    def stationary(self):
        if not getattr(self,'acquisition_ready',True):raise ValueError('Acquisition anchor not ready')
        metrics=tcp_stability_metrics(self.pre_acquisition(),self.initial)
        if not tcp_is_stable(metrics):
            self.log(dict(event='acquisition_motion_rejected',**metrics))
            raise ValueError('Robot moved while acquiring GPT/head geometry: '
                f'position_delta={metrics["position_delta_mm"]:.3f}mm (limit=0.500mm), '
                f'orientation_delta={metrics["orientation_delta_deg"]:.3f}deg (limit=0.250deg), '
                f'max_joint_speed={metrics["max_joint_speed_rad_s"]:.5f}rad/s (limit=0.02000rad/s)')

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
        from cart_grasp_alignment import MarkerLost
        def finalize(checkpoint):
            self.soft_hold()
            for sample in range(3):
                try:e,p=observe(checkpoint)
                except MarkerLost as exc:
                    self.log(dict(event='final_alignment_retry',reason=str(exc),sample=sample))
                    return None
                current=checkpoint()
                valid=(abs(e[0])<=self.alignment['tolerance_px'] and
                    abs(e[1])<=self.alignment['tolerance_angle_deg'] and
                    abs(p['q'][0]-goal)<=self.approach['x_tolerance_mm'] and
                    abs(current['q'][0]-goal)<=self.approach['x_tolerance_mm'] and
                    np.linalg.norm(current['q']-p['q'])<=3 and abs(current['roll']-p['roll'])<=1)
                if not valid:
                    self.log(dict(event='final_alignment_retry',sample=sample,error_px_deg=e.tolist(),
                        actual_x_mm=float(current['q'][0]),goal_x_mm=float(goal)))
                    return None
            self.log(dict(event='final_alignment_confirmed',samples=3,error_px_deg=e.tolist()))
            return e,current
        result=align_loop(self.transport.read,observe,self.send_visual,self.transport.ready,self.soft_hold,
            origin,dict(self.alignment,max_path_mm=self.approach['max_path_mm']),self.log,
            goal_x=goal,approach=self.approach,finalize=finalize)
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
        if not self.commanded:return
        # A failure status must not prevent cancellation. Prefer fresh geometry,
        # but retain the last valid envelope if feedback is unavailable; the
        # planner's hold action cancels and latches its own current joints.
        try:
            s=self.robot.get_current_state();now=time.monotonic()
            tcp_stability_metrics(s,s)  # validate geometry, not motion/status
            if all(0<=now-float(s[k])<=.25 for k in ('received_monotonic','pose_sample_monotonic')):
                self.transport.last_state=copy.deepcopy(s)
        except (ValueError,TypeError,KeyError,AttributeError):
            pass
        self.transport.abort()


def run(args,c):
    from servor import robot_connection
    from cart_book_grasp_v3_camera import WristCamera
    from cart_grap_v1_perception import locate
    alignment=load_alignment(ROOT/c['alignment_config']);approach=load_approach(ROOT/c['approach_config'])
    out=args.out or ROOT/'output'/('cart_grap_v1_1_'+str(time.time_ns()));out.mkdir(parents=True,exist_ok=False)
    with (out/'events.jsonl').open('w') as file:
        def log(e):
            file.write(json.dumps(e,ensure_ascii=False,allow_nan=False)+'\n');file.flush()
            if e['event']!='wrist_marker':print(json.dumps(e,ensure_ascii=False),flush=True)
        io=None;camera=None
        try:
            with robot_connection() as (robot,base):
                if args.execute:base=first_reset(robot,c,log)
                io=CycleIO(robot,base,c,alignment,approach,out,log,
                    reset_target=np.deg2rad(robot.right_safe_joint_deg) if args.execute else None,
                    defer_acquisition=True)
                try:
                    io.require_open()
                    # Warm the wrist stream before acquiring a time-sensitive head target.
                    camera=WristCamera();camera.capture(io.pre_acquisition)
                    head=locate(out,c,checkpoint=io.stationary,prepare_camera=io.prepare_acquisition)
                    base=io.base  # all geometry uses the final post-warmup reference
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
            log(dict(event='stopped',reason=f'{type(exc).__name__}: {exc}',automatic_open_or_return=False))
            return 130 if isinstance(exc,KeyboardInterrupt) else 1


def run_repeated(args,c):
    """Successful full cycles repeat after 5 s; failures require operator retry."""
    from cart_book_grasp_v1_1 import AutoCartLoop,stop_key_listener
    loop=AutoCartLoop(delay_seconds=5.)
    result=0
    try:
        with stop_key_listener(loop):
            if args.out is not None:args.out.mkdir(parents=True,exist_ok=False)
            print('车把循环：每轮先 first_reset；完整成功后等待 5 秒。s 停止后续轮次；失败暂停，回车重试。',flush=True)
            while loop.next_task() is not None:
                cycle_args=copy.copy(args)
                if args.out is not None:cycle_args.out=args.out/f'round_{loop.cycles:04d}'
                result=run(cycle_args,c)
                if result==130:return result
                loop.last_success=result==0
        return result
    except KeyboardInterrupt:
        return 130


def main(argv=None):
    argv=sys.argv[1:] if argv is None else argv
    if argv and argv[0] in ('marker','observe','teach','align'):
        from cart_grasp import main as tools_main
        return tools_main(argv)
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true',help='循环执行；每轮成功后间隔5秒，失败暂停；默认仅保存定位方案')
    p.add_argument('--once',action='store_true',help='与 --execute 配合，仅执行一轮')
    p.add_argument('--check-config',action='store_true',help='离线检查配置')
    p.add_argument('--config',type=Path,default=CONFIG);p.add_argument('--out',type=Path)
    a=p.parse_args(argv);c=load_config(a.config)
    if a.check_config:
        load_alignment(ROOT/c['alignment_config']);load_approach(ROOT/c['approach_config'])
        print(json.dumps(c,ensure_ascii=False,indent=2));return 0
    return run_repeated(a,c) if a.execute and not a.once else run(a,c)


if __name__=='__main__':raise SystemExit(main())
