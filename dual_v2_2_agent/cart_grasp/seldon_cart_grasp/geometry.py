"""SE(3) operations. Matrices parent_T_child always map child points to parent."""
import hashlib
import numpy as np
from scipy.spatial.transform import Rotation

class Rejected(ValueError):
    pass

def finite(value, shape=None):
    a=np.asarray(value,dtype=float)
    if (shape is not None and a.shape!=shape) or not np.isfinite(a).all():
        raise Rejected('Nonfinite or malformed numeric input')
    return a

def transform(value):
    t=finite(value,(4,4));r=t[:3,:3]
    if not np.allclose(t[3],[0,0,0,1],atol=1e-7) or not np.allclose(r.T@r,np.eye(3),atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5:
        raise Rejected('Expected a rigid, right-handed transform')
    return t

def pose(xyz=(0,0,0),rpy=(0,0,0)):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',finite(rpy,(3,))).as_matrix();t[:3,3]=finite(xyz,(3,));return t

def apply(t,points):
    t=transform(t);p=finite(points)
    if p.shape[-1]!=3:raise Rejected('Expected XYZ points')
    return p@t[:3,:3].T+t[:3,3]

def wrap(x):return float(np.arctan2(np.sin(x),np.cos(x)))

def register_points(source,target):
    """Measured correspondences, not guessed camera height; RMS must be reviewed."""
    a=finite(source);b=finite(target)
    if a.ndim!=2 or a.shape[1]!=3 or a.shape!=b.shape or len(a)<4:raise Rejected('Need >=4 paired XYZ calibration points')
    ac=a-a.mean(0);bc=b-b.mean(0)
    if np.linalg.matrix_rank(ac,tol=1e-5)<2 or np.linalg.matrix_rank(bc,tol=1e-5)<2:raise Rejected('Calibration points are collinear')
    u,_,vt=np.linalg.svd(ac.T@bc);r=vt.T@u.T
    if np.linalg.det(r)<0:vt[-1]*=-1;r=vt.T@u.T
    t=np.eye(4);t[:3,:3]=r;t[:3,3]=b.mean(0)-r@a.mean(0)
    residual=np.linalg.norm(apply(t,a)-b,axis=1)
    return t,{'rms_m':float(np.sqrt(np.mean(residual**2))),'maximum_m':float(residual.max()),'points':len(a)}

def calibrated_frames(c,model_path,session_id):
    if c.get('calibrated') is not True or c.get('waist_mode')!='fixed' or c.get('waist_locked') is not True:raise Rejected('Require measured calibration and locked fixed waist')
    if not session_id or c.get('session_id')!=session_id:raise Rejected('Calibration belongs to another waist setup/session')
    if c.get('robot_model_sha256')!=hashlib.sha256(open(model_path,'rb').read()).hexdigest():raise Rejected('Robot model changed since calibration')
    if c.get('upper_frame')!='SJ4Z' or not c.get('base_frame') or not c.get('camera_frame'):raise Rejected('Unexpected or missing calibration frames')
    residual=finite(c.get('calibration_residual_m'),())
    if residual<0 or residual>0.008 or c.get('camera_alignment_verified') is not True:raise Rejected('Calibration residual/alignment not accepted')
    finite(c.get('base_floor_z_m'),())
    return {k:transform(c[k]) for k in ['base_T_camera','base_T_upper','left_tip_T_contact','right_tip_T_contact']}
