"""Real robot adapter for bounded V3.1 rolling targets. No devices on import."""
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_centerline import measured_roll,roll_orientation
from cart_book_grasp_v3_cotracker_points import draw_points
from cart_book_grasp_v3_1_stream import run_stream,translation_budget
from cart_book_grasp_v3_1_pose_time import PoseTimeline,PoseNotBracketed,PoseSupportUnavailable


def status_value(value):return getattr(value,'value',value)


class FeedbackStale(ServoError):
    """A well-formed feedback packet has aged out; never usable for control."""


class StreamTransport:
    def __init__(self,robot,base,c,s,clock=time.monotonic,log=lambda event:None):
        self.robot=robot;self.base=base;self.c=c;self.s=s;self.clock=clock
        self.log=log
        self.pending=None;self.sent_at=0.;self.fixed_angles=None;self.last_angles=np.asarray(base).copy()
        self.last_state=None;self.timeline=PoseTimeline()

    def read(self):
        state=self.robot.get_current_state();now=self.clock()
        try:
            stamp=float(state['received_monotonic']);arm=state['right_arm']
            q=np.asarray(arm['position'],float)*1000;quat=np.asarray(arm['orientation'],float)
            if q.shape!=(3,) or quat.shape!=(4,) or not np.isfinite([*q,*quat,stamp]).all():raise ValueError('pose')
            if stamp>now:raise ValueError('future feedback')
            angles=Rotation.from_quat(np.r_[quat[1:],quat[0]]).as_euler('xyz',degrees=True)
            planner_status=status_value(state['curobo_status'])
            failures={3:'PLANNING_FAILED',7:'IK_FAILED',8:'COLLISION_CHECK_FAILED'}
            if planner_status in failures:
                raise ServoError(f'V3.1 planner/IK/collision failure: status={planner_status} ({failures[planner_status]})')
            if status_value(state['robot_status']) not in (0,1):raise ServoError('V3.1 robot unavailable')
            if now-stamp>self.s['feedback_max_age_seconds']:
                raise FeedbackStale('V3.1 stale robot feedback')
            source=float(state['pose_sample_monotonic'])
            completed=float(state['pose_fk_completed_monotonic'])
            if not np.isfinite([source,completed]).all() or not 0<source<=completed<=stamp:
                raise ValueError('invalid pose provenance; restart V3.1 planner and agent')
            if now-source>self.s['feedback_max_age_seconds']:
                raise FeedbackStale(f'V3.1 stale pose sample: source_age={now-source:.3f}s, packet_age={now-stamp:.3f}s')
        except ServoError:
            # ServoError inherits ValueError; preserve recoverable stale packets
            # and explicit controller faults instead of erasing their types.
            raise
        except (TypeError,ValueError,KeyError,IndexError) as error:
            raise ServoError('V3.1 invalid or stale robot feedback') from error
        self.last_state=state
        result=dict(q=q,roll=measured_roll(self.base,angles,self.c['max_orientation_error_deg']),
                    angles=angles,received_monotonic=stamp,pose_sample_monotonic=source,
                    pose_fk_completed_monotonic=completed)
        self.timeline.add(result)
        return result

    def pair_frame(self,stamp,checkpoint=lambda:None,sleep=time.sleep):
        deadline=self.clock()+self.s['max_pose_pair_gap_seconds']
        while True:
            checkpoint();self.read()
            try:
                paired=self.timeline.pair(stamp,self.s)
                paired['roll']=measured_roll(self.base,paired['angles'],self.c['max_orientation_error_deg'])
                return paired
            except PoseNotBracketed:
                if self.clock()>=deadline:raise
                sleep(.005)


    def ready(self):
        try:self.read()
        except FeedbackStale:
            # Skip this update; the next observation enters the bounded
            # recapture/recovery loop. Never use a stale pose to acknowledge.
            return False
        if self.pending is None:return True
        state=self.last_state
        processed=np.asarray(state.get('processed_arm_tool_command',[]),float)
        same=processed.shape==self.pending.shape and np.allclose(processed,self.pending,atol=1e-5,rtol=0)
        if state['received_monotonic']>self.sent_at and same and status_value(state['curobo_status']) in (4,9):
            self.log(dict(event='stream_target_acknowledged',elapsed_seconds=self.clock()-self.sent_at))
            self.pending=None;return True
        if self.clock()-self.sent_at>self.s['ack_timeout_seconds']:
            status=status_value(state['curobo_status'])
            self.log(dict(event='stream_ack_timeout',planner_status=status,target_matches=bool(same),
                          elapsed_seconds=self.clock()-self.sent_at,
                          requested=self.pending.tolist(),processed=processed.tolist()))
            raise ServoError(f'V3.1 planner acknowledgement timeout: status={status}, target_matches={same}')
        return False

    def send(self,q,roll):
        try:current=self.read()
        except FeedbackStale:
            # A sample can expire between ready() and send(). Do not send a
            # target computed from it; resume only through a new observation.
            return False
        if self.pending is not None:raise ServoError('V3.1 pending target must be acknowledged first')
        q=np.asarray(q,float).copy()
        # read/ready/send are separated by perception and scheduling. X hold
        # always means hold at the latest pose, never an older frame's X.
        q[0]=max(q[0],current['q'][0])
        # Independent send-side bound: observations and planning can lag.
        if np.linalg.norm(q-current['q'])>translation_budget(self.c,self.s)+.05:
            raise ServoError('V3.1 target exceeds measured lookahead')
        angles=self.fixed_angles if self.fixed_angles is not None else roll_orientation(self.base,roll)
        if abs(roll-current['roll'])>self.s['lookahead_roll_deg']+.1:raise ServoError('V3.1 angular lookahead exceeded')
        self.robot._v3_servo_position_tolerance_mm=self.c['max_pose_error_mm']
        if self.robot.master_control(right_arm=[*q,*angles],action='book_v3_1_stream',object_category='book',wait_for_completion=False) is not True:
            raise ServoError('V3.1 target send failed')
        self.last_angles=np.asarray(angles).copy()
        self.sent_at=self.clock();self.pending=np.asarray(self.robot.target_command.arm_tool_command,float).copy()
        self.log(dict(event='stream_target_sent',requested=self.pending.tolist()))
        return True

    def finish(self,q,roll):
        self.read()
        angles=self.fixed_angles if self.fixed_angles is not None else roll_orientation(self.base,roll)
        self.robot._v3_servo_position_tolerance_mm=self.c['max_pose_error_mm']
        result=self.robot.master_control(right_arm=[*q,*angles],action='book_v3_xyz_servo',object_category='book')
        self.pending=None;self.last_angles=np.asarray(angles).copy()
        return result is True

    def abort(self):
        # Best effort planner-mediated hold. If feedback is stale, stop sending;
        # the last finite lookahead target remains the bounded fallback, not a
        # guaranteed emergency brake. Never open/reset the gripper here.
        # A separate cancel action lets the planner hold its newest measured
        # joints without planning another Cartesian path or changing Z plane.
        state=self.last_state
        if state is None:raise ServoError('V3.1 no state for hold request')
        arm=state['right_arm'];q=np.asarray(arm['position'])*1000
        quat=np.asarray(arm['orientation'])
        angles=Rotation.from_quat(quat[[1,2,3,0]]).as_euler('xyz',degrees=True)
        self.pending=None
        if self.robot.master_control(right_arm=[*q,*angles],action='book_v3_1_hold',
                object_category='book',wait_for_completion=False) is not True:
            raise ServoError('V3.1 hold request send failed')


def capture_paired_frame(transport,camera,checkpoint,log,last_frame,*,clock,sleep,total_deadline):
    """Suspend target generation until a newly captured frame has valid support.

    No send/hold/finish occurs here: the last finite target may finish moving.
    Missing/wide brackets and stale samples discard the frame and retry with
    the same precision limits. Invalid feedback, faults and camera failures
    still propagate immediately.
    """
    recovery_started=None;recovery_deadline=None;attempts=0;last_seen=last_frame
    def check_budget():
        checkpoint()
        if clock()>=total_deadline:raise ServoError('V3.1 total time budget exceeded')
        if recovery_deadline is not None and clock()>=recovery_deadline:
            log(dict(event='pose_pair_recovery_timeout',attempts=attempts,
                     elapsed_seconds=clock()-recovery_started))
            raise ServoError('V3.1 pose pairing recovery timed out; no new motion targets sent during recovery')
    def checked():
        check_budget();transport.read()
    while True:
        check_budget();began=clock();stamp=None;before=None;after=None
        try:
            before=transport.read()
            frame=camera.capture(checked);capture_wait=clock()-began
            after=transport.read();stamp=float(frame['captured_at'])
            check_budget()
            if not last_seen<stamp<=clock() or clock()-stamp>transport.s['frame_max_age_seconds']:
                raise ServoError('V3.1 frame age exceeded')
            last_seen=stamp
            paired=transport.pair_frame(stamp,check_budget,sleep)
            check_budget()
            if clock()-stamp>transport.s['frame_max_age_seconds']:
                raise ServoError('V3.1 frame age exceeded after pairing')
            if recovery_started is not None:
                log(dict(event='pose_pair_recovered',attempts=attempts,
                         elapsed_seconds=clock()-recovery_started,frame_stamp=stamp,
                         pair_gap_seconds=paired['pair_gap_seconds']))
            return frame,paired,before,after,capture_wait
        except (PoseNotBracketed,PoseSupportUnavailable,FeedbackStale) as error:
            attempts+=1
            history=transport.timeline.samples
            log(dict(event='pose_pair_failed',reason=str(error),frame_stamp=stamp,
                     pose_oldest_stamp=history[0][0] if history else None,
                     pose_latest_stamp=history[-1][0] if history else None,
                     capture_wait_seconds=clock()-began,
                     acquisition_motion_mm=float(np.linalg.norm(after['q']-before['q']))
                         if before is not None and after is not None else None,
                     support=getattr(error,'details',{})))
            if recovery_started is None:
                recovery_started=clock()
                recovery_deadline=recovery_started+transport.s['pair_recovery_timeout_seconds']
                log(dict(event='pose_pair_recovery_start',timeout_seconds=transport.s['pair_recovery_timeout_seconds']))
            # Discard this image. A genuinely newer packet must arrive before
            # requesting a new post-request frame, not reusing a cached packet.
            wait_after=clock()
            while True:
                check_budget()
                try:fresh=transport.read()
                except FeedbackStale:fresh=None
                if fresh is not None and fresh['pose_sample_monotonic']>wait_after:break
                sleep(.005)


def run_stream_session(robot,camera,lock,target,goal_x,base,c,s,blocking_move,checkpoint,log,directory,display,*,started):
    transport=StreamTransport(robot,base,c,s,clock=time.monotonic,log=log);count=0;last_frame=-np.inf
    old_tolerance=getattr(robot,'_v3_servo_position_tolerance_mm',None)
    def observe():
        nonlocal count,last_frame
        began=time.monotonic()
        frame,paired,before,after_capture,capture_wait=capture_paired_frame(
            transport,camera,checkpoint,log,last_frame,clock=time.monotonic,
            sleep=getattr(robot,'operator_sleep',time.sleep),total_deadline=started+c['max_seconds'])
        stamp=float(frame['captured_at'])
        last_frame=stamp
        try:observation=lock.observe(frame['rgb'])
        finally:
            # A point-loss exception must not bypass frame freshness and turn
            # a delayed/invalid observation into nonvisual forward motion.
            if time.monotonic()-stamp>s['frame_max_age_seconds']:
                raise ServoError('V3.1 perception latency exceeded')
        state=transport.read()
        # Increasing command lookahead must not widen the image latency horizon.
        if np.linalg.norm(state['q']-paired['q'])>min(5.,translation_budget(c,s)) or abs(state['roll']-paired['roll'])>s['lookahead_roll_deg']:
            raise ServoError('V3.1 frame-to-command motion exceeds prediction horizon')
        if count % c['snapshot_every']==0:
            image=draw_points(frame['rgb'],observation,target)
            ends=np.rint(observation['line']).astype(int)
            cv2.line(image,tuple(ends[0]),tuple(ends[1]),(0,220,100),2)
            cv2.imwrite(str(directory/f'stream_{count:05d}.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
            if display is not None:display.safe('log_image','book/images/spine',image)
        count+=1
        log(dict(event='stream_observation',inliers=observation['inliers'],xyz_mm=state['q'].tolist(),
                 image_xyz_mm=paired['q'].tolist(),image_roll_deg=paired['roll'],frame_stamp=stamp,
                 pose_before_stamp=paired['pose_before_stamp'],pose_after_stamp=paired['pose_after_stamp'],
                 pair_gap_seconds=paired['pair_gap_seconds'],pair_motion_bound_mm=paired['pair_motion_bound_mm'],
                 pair_angle_bound_deg=paired['pair_angle_bound_deg'],
                 timestamp_basis='host_read_completion_panel_receipt',
                 feedback_packet_stamp=state['received_monotonic'],
                 pose_sample_stamp=state['pose_sample_monotonic'],
                 pose_fk_completed_stamp=state['pose_fk_completed_monotonic'],
                 pose_age_seconds=time.monotonic()-state['pose_sample_monotonic'],
                 acquisition_motion_mm=float(np.linalg.norm(after_capture['q']-before['q'])),
                 capture_wait_seconds=capture_wait,
                 observation_seconds=time.monotonic()-began,
                 frame_to_command_mm=float(np.linalg.norm(state['q']-paired['q'])),
                 error_px=float(observation['uv'][0]-target[0]),angle_deg=float(observation['angle_deg'])))
        return dict(state,image_q=paired['q'],image_roll=paired['roll'],
                    error=np.array([observation['uv'][0]-target[0],observation['angle_deg']]),
                    scale=observation['scale'],stamp=stamp)
    def event(e):
        if e['event']=='point_loss_direct_switch':
            transport.fixed_angles=np.asarray(e['fixed_angles_deg'],float)
        log(e)
    try:
        result=run_stream(observe,transport.read,blocking_move,transport.send,transport.ready,transport.finish,
            goal_x,c,s,checkpoint=checkpoint,log=event,started=started)
        result['final_angles_deg']=transport.last_angles.tolist()
        return result
    except BaseException:
        try:transport.abort()
        except BaseException as hold_error:
            log(dict(event='stream_hold_unconfirmed',reason=str(hold_error)))
        raise
    finally:
        if old_tolerance is None:
            if hasattr(robot,'_v3_servo_position_tolerance_mm'):del robot._v3_servo_position_tolerance_mm
        else:robot._v3_servo_position_tolerance_mm=old_tolerance
