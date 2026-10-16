"""Precision policy for planner-paced V4.2 lifts, NOT a direct-servo route."""
from contextlib import contextmanager
import numpy as np
from servo_cartesian_policy_v3_1 import is_book_servo, poses_close

ACTION='book_v42_raise_servo'


def is_precision_action(description):
    return description==ACTION or is_book_servo(description)


def arrival_position_m(description):
    return .003 if description==ACTION else .002


@contextmanager
def solver_precision(motion_gen,description):
    """Called under the planner GPU lock; restore old settings even on failure.

    CuRobo success tests read these scalar attributes on each solve. Rotation
    threshold is CuRobo's own metric, not the quaternion-component comparator.
    A separate FK endpoint gate below measures the actual angle in degrees.
    """
    if description!=ACTION:
        yield
        return
    saved=[]
    try:
        for name in ('ik_solver','trajopt_solver','finetune_trajopt_solver'):
            solver=getattr(motion_gen,name)
            for attr,limit in (('position_threshold',.00025),('rotation_threshold',.001)):
                old=getattr(solver,attr)
                if not np.isfinite(old) or old<=0:raise ValueError('Invalid solver precision: '+name+'.'+attr)
                saved.append((solver,attr,old));setattr(solver,attr,min(old,limit))
        yield
    finally:
        for solver,attr,old in reversed(saved):setattr(solver,attr,old)


def endpoint_report(position,quaternion,target_position,target_quaternion):
    """Right TCP gate with reserve for measured tracking error (3 mm / 1 deg)."""
    p,q,tp,tq=map(lambda x:np.asarray(x,float),(position,quaternion,target_position,target_quaternion))
    accepted=poses_close(p,q,tp,tq,position_m=.00025,angle_deg=.15)
    if p.shape!=(1,3) or tp.shape!=(1,3) or q.shape!=(1,4) or tq.shape!=(1,4):
        raise ValueError('Invalid V4.2 endpoint shape')
    norms=np.linalg.norm(q,axis=1)*np.linalg.norm(tq,axis=1)
    if not all(np.isfinite(x).all() for x in (p,q,tp,tq)) or np.any(norms<1e-12):
        raise ValueError('Invalid V4.2 endpoint pose')
    angle=np.rad2deg(2*np.arccos(np.clip(np.abs(np.sum(q*tq,axis=1))/norms,0,1)))
    return dict(accepted=bool(accepted),position_error_mm=float(np.linalg.norm(p-tp)*1000),
                orientation_error_deg=float(angle[0]),planned_position_limit_mm=.25,planned_orientation_limit_deg=.15)


def configure_plan(plan,description):
    """Fresh plan copy: V4.2 uses 0.9 instead of ordinary 0.4 dilation.

    This removes inherited slowdown; it does not raise joint velocity limits.
    """
    import copy
    result=copy.deepcopy(plan)
    if description==ACTION:result.time_dilation_factor=.9
    return result
