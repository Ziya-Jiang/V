"""Stopped V4.3 GPT reanchor; GPT geometry supersedes the drifted line."""
import json
import time
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_gpt_recovery import make_lock,wait_stationary
from cart_book_grasp_v3_feature_centerline import (InitialFeaturesUnavailable,
    FeaturePointsExhausted, CenterlineGeometryError)

VISUAL_FAILURES=(InitialFeaturesUnavailable,FeaturePointsExhausted,CenterlineGeometryError)


class TrackingReacquisitionFailed(ServoError):
    """Retryable visual verification failure; never a motion/feedback failure."""


def verify_tracking(candidate,capture,checkpoint,reference_stamp,*,max_age=2.,
                    frames=3,clock=time.monotonic):
    """Track across distinct fresh frames before exposing a lock to control.

    The API reference may be old. Never transplant its polygon onto a new
    image: use the tracker to establish cross-frame correspondence instead.
    """
    stamp=float(reference_stamp);observation=None;fresh=None
    for _ in range(frames):
        checkpoint();fresh=capture(checkpoint);checkpoint()
        t=float(fresh["captured_at"])
        if not np.isfinite(t) or t<=stamp or not 0<=clock()-t<=max_age:
            raise ServoError("V4.3 verification needs distinct fresh frames")
        observation=candidate.observe(fresh["rgb"]);checkpoint()
        if not 0<=clock()-t<=max_age:
            raise ServoError("V4.3 verification image expired")
        stamp=t
    return fresh,observation



def reanchor(old_lock,capture,select,read,checkpoint,directory,deadline,*,
             reason,max_age=2.,clock=time.monotonic,sleep=time.sleep,factory=make_lock):
    folder=Path(directory);folder.mkdir(parents=True,exist_ok=True)
    record=dict(status='pending',trigger='tracking_geometry_failure',reason=reason)
    anchor=None
    def checked():
        checkpoint()
        if clock()>=deadline:raise ServoError('V4.3 GPT recovery time budget')
        state=read();now=clock()
        if not 0<=now-state['pose_sample_monotonic']<=.25:
            raise ServoError('V4.3 GPT recovery stale feedback')
        if anchor is not None:
            distance=float(np.linalg.norm(np.asarray(state['q'])-anchor['q']))
            angle=float(np.degrees((Rotation.from_euler('xyz',state['angles'],degrees=True)*
                Rotation.from_euler('xyz',anchor['angles'],degrees=True).inv()).magnitude()))
            if distance>3. or angle>1.:
                raise ServoError('V4.3 arm moved during GPT centerline review')
        return state
    def frame():
        checked();f=capture(checked);checked()
        if not 0<=clock()-f['captured_at']<=max_age:
            raise ServoError('V4.3 GPT recovery stale image')
        return f
    try:
        initial=checked()
        settled=wait_stationary(read,checked,record,deadline,initial['q'][2],3.,clock=clock,sleep=sleep)
        anchor=dict(q=np.asarray(settled['q']).copy(),angles=np.asarray(settled['angles']).copy())
        reference=frame()
        cv2.imwrite(str(folder/'gpt_reference.png'),cv2.cvtColor(reference['rgb'],cv2.COLOR_RGB2BGR))
        chosen=select(reference['rgb'],record,directory=folder,checkpoint=checked,
            review_context=dict(reason=reason,request='Repair the centerline of the SAME physical book spine. The old tracked line may have drifted; use the real spine edges, not cover or printed text.'),
            reference_rgb=old_lock.reference_rgb,reference_mask=old_lock.reference_mask)
        checked()
        if chosen.get('identity_confirmed') is not True or chosen.get('centerline') is None:
            raise ServoError('V4.3 GPT must confirm same book and provide a centerline')
        # No comparison with old u/angle: that old estimate is the suspect one.
        try:
            candidate=factory(reference['rgb'],chosen)
            fresh,observation=verify_tracking(candidate,capture,checked,reference['captured_at'],
                max_age=max_age,clock=clock)
        except VISUAL_FAILURES as error:
            raise TrackingReacquisitionFailed(f'V4.3 same-book tracking verification failed: {error}') from error
        overlay=fresh['rgb'].copy();ends=np.rint(observation['line']).astype(int)
        cv2.line(overlay,tuple(ends[0]),tuple(ends[1]),(0,220,100),2)
        cv2.imwrite(str(folder/'gpt_centerline_verified.png'),cv2.cvtColor(overlay,cv2.COLOR_RGB2BGR))
        record.update(status='recovered',verified_frames=3,centerline=chosen['centerline'],inliers=int(observation['inliers']),
                      reference_stamp=float(reference['captured_at']),verification_stamp=float(fresh['captured_at']))
        return candidate
    except BaseException as error:
        record.update(status='failed',error=str(error));raise
    finally:
        (folder/'recovery.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
