"""Fine Cartesian comparisons for the right-arm book visual servo only."""
import numpy as np


def is_book_servo(description):
    return description in ('book_v3_xyz_servo', 'book_v3_1_stream')


def poses_close(position_a, quaternion_a, position_b, quaternion_b, *, position_m, angle_deg):
    if any(v is None for v in (position_a,quaternion_a,position_b,quaternion_b)):
        return False
    a,b=np.asarray(position_a,float),np.asarray(position_b,float)
    qa,qb=np.asarray(quaternion_a,float),np.asarray(quaternion_b,float)
    if a.shape!=b.shape or qa.shape!=qb.shape or a.ndim!=2 or a.shape[1]!=3 or qa.shape!=(len(a),4):
        return False
    if not all(np.isfinite(v).all() for v in (a,b,qa,qb)):return False
    na,nb=np.linalg.norm(qa,axis=1),np.linalg.norm(qb,axis=1)
    if np.any(na<1e-6) or np.any(nb<1e-6):return False
    cosine=np.abs(np.sum((qa/na[:,None])*(qb/nb[:,None]),axis=1))
    angle=2*np.arccos(np.clip(cosine,0,1))
    return bool(np.all(np.linalg.norm(a-b,axis=1)<=position_m) and np.all(angle<=np.deg2rad(angle_deg)))
