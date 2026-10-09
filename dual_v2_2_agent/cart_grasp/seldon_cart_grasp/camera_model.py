"""Validate provided camera calibration without guessing frame or resize rules."""
import numpy as np
from .geometry import Rejected,finite,transform

def validate_camera_document(c):
    k=finite(c['K'],(3,3));d=finite(c['D'],(5,));t=transform(c['target_T_camera']);r=finite(c['R'],(3,3))
    if c['translation_units']!='m' or c['translation_source_units']!='mm':raise Rejected('Camera translation units must be explicit')
    if c['transform_direction']!='target_from_camera':raise Rejected('Unexpected extrinsic direction')
    if not np.allclose(t[:3,3],finite(c['translation_source'],(3,))*.001,atol=1e-9) or not np.allclose(t[:3,:3],r):raise Rejected('Camera extrinsic transcription/units mismatch')
    w,h=int(c['image_width']),int(c['image_height'])
    if w<=0 or h<=0 or k[0,0]<=0 or k[1,1]<=0 or not 0<=k[0,2]<w or not 0<=k[1,2]<h or not np.allclose(k[2],[0,0,1]):raise Rejected('Invalid image/intrinsics')
    if c.get('distortion_model')!='plumb_bob':raise Rejected('Only verified OpenCV Brown-Conrady coefficients supported')
    return {'numeric_valid':True,'resolution':[w,h],'rotation_determinant':float(np.linalg.det(r)),'rotation_orthogonality_error':float(np.linalg.norm(r.T@r-np.eye(3))),'translation_m':t[:3,3].tolist(),'ready_for_frame_binding':c.get('target_frame_confirmed') is True and bool(c.get('target_frame')),'depth_alignment_verified':c.get('depth_aligned_to_color') is True}

def image_intrinsics(c,width,height,rectified=False):
    validate_camera_document(c)
    if (width,height)!=(c['image_width'],c['image_height']):raise Rejected('Image mode differs from calibrated 848x530; do not guess resize/crop intrinsics')
    if rectified:raise Rejected('Rectified images require their own projection intrinsics; do not reuse raw K')
    return finite(c['K'],(3,3)),finite(c['D'],(5,))

def camera_in_base(c,base_frame,base_T_target=None,*,base_T_upper=None):
    validate_camera_document(c)
    if c.get('target_frame_confirmed') is not True or not c.get('target_frame'):raise Rejected('Extrinsic robot-base label not bound to physical frame')
    if c['target_frame']=='half_model::dummy_world':
        upper_T_camera=confirmed_upper_camera(c)
        if base_frame==c['source_upper_frame']:return upper_T_camera
        if base_T_upper is None:raise Rejected('Confirmed half-root camera still needs actual base_T_upper for chassis coordinates')
        return transform(base_T_upper)@upper_T_camera
    t=transform(c['target_T_camera'])
    if c['target_frame']==base_frame:return t
    if base_T_target is None:raise Rejected('Need measured base_T_target for upper-body/waist adaptation')
    return transform(base_T_target)@t


def confirmed_upper_camera(c):
    """Rebase the user-confirmed half-model calibration exactly once."""
    from .frame_graph import FrameGraph,camera_to_upper,verify_arm_frame_compatibility
    validate_camera_document(c)
    if c.get('target_frame_confirmed') is not True or c.get('target_frame')!='half_model::dummy_world':raise Rejected('Half-model calibration source not confirmed')
    source=FrameGraph(c['source_urdf_path']);destination=FrameGraph(c['destination_urdf_path'])
    if source.sha256!=c.get('source_urdf_sha256') or destination.sha256!=c.get('destination_urdf_sha256'):raise Rejected('Calibration source/destination URDF hash changed')
    if c.get('source_urdf_root')!=source.root or c.get('source_upper_frame')!='SJ4Z':raise Rejected('Unexpected half-model reference frames')
    verify_arm_frame_compatibility(source,destination)
    return camera_to_upper(source,source.root,c['target_T_camera'])


def compose_fixed_waist_camera(c,waist_calibration):
    """Combine the known upper camera mount with an explicitly measured waist setup.

    Other calibration fields (TCP, floor, alignment) remain unmodified and are
    checked separately by calibrated_frames before any observation/planning.
    """
    import copy
    if waist_calibration.get('waist_mode')!='fixed' or waist_calibration.get('waist_locked') is not True or not waist_calibration.get('session_id'):raise Rejected('Need a named, locked fixed-waist setup')
    if waist_calibration.get('upper_frame')!='SJ4Z':raise Rejected('Fixed waist must reference SJ4Z')
    if waist_calibration.get('robot_model_sha256')!=c.get('destination_urdf_sha256'):raise Rejected('Fixed-waist and camera robot models differ')
    upper=confirmed_upper_camera(c);base=transform(waist_calibration['base_T_upper']);result=copy.deepcopy(waist_calibration)
    result['upper_T_camera']=upper.tolist();result['base_T_camera']=(base@upper).tolist();result['camera_extrinsic_source']='user_confirmed_half_model_root_rebased_to_SJ4Z';result['camera_source_urdf_sha256']=c['source_urdf_sha256']
    return result
