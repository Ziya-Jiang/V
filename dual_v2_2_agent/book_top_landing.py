"""Pure V4.1 head coarse geometry and wrist Image-Jacobian landing controller.

mm in robot base; rectified-K wrist pixels. No force-contact claim and no I/O.
"""
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation
from book_top_tilt_geometry import FingerPad, vector

DEFAULTS = dict(coarse_clearance_mm=12., coarse_position_tolerance_mm=2.,
    coarse_max_travel_mm=500., coarse_timeout_seconds=60., probe_mm=4.,
    pixel_tolerance=2., alignment_tolerance_mm=1.5, gap_tolerance_mm=.8,
    penetration_limit_mm=.5, stable_seconds=1., stable_frames=5,
    max_book_motion_mm=2., max_head_wrist_gap_seconds=.2,
    max_condition=30., min_response_px_per_mm=.1, max_fit_error_px=2.,
    max_servo_travel_mm=35., descent_step_mm=.5,
    contact_reference='v4_1_wrist_contact.json')


def settings(c):
    d = dict(DEFAULTS, **c.get('landing', {}))
    bounds = dict(coarse_clearance_mm=(8,30), coarse_position_tolerance_mm=(.5,3),
        coarse_max_travel_mm=(10,600), coarse_timeout_seconds=(5,120), probe_mm=(2,5),
        pixel_tolerance=(.5,4), alignment_tolerance_mm=(.3,3), gap_tolerance_mm=(.2,1),
        penetration_limit_mm=(.1,.8), stable_seconds=(.5,3), stable_frames=(3,20),
        max_book_motion_mm=(.5,3), max_head_wrist_gap_seconds=(.03,.3),
        max_condition=(2,50), min_response_px_per_mm=(.02,1), max_fit_error_px=(.5,3),
        max_servo_travel_mm=(5,50), descent_step_mm=(.1,.5))
    if set(d) != set(DEFAULTS): raise ValueError('Unknown V4.1 landing setting')
    for k,(lo,hi) in bounds.items():
        v=d[k]
        if type(v) not in (int,float) or not np.isfinite(v) or not lo<=v<=hi:
            raise ValueError('Invalid landing setting: '+k)
    if type(d['stable_frames']) is not int: raise ValueError('stable_frames must be integer')
    if not isinstance(d['contact_reference'],str) or not d['contact_reference']:
        raise ValueError('Missing wrist contact reference path')
    return d


def pad_fingerprint(c):
    return hashlib.sha256(json.dumps(dict(pad=c['pad'], aperture=c['calibrated_gripper_position']),
        sort_keys=True,allow_nan=False).encode()).hexdigest()


def contact_reference(value, camera, c):
    if value.get('kind')!='human_taught_visual_contact' or value.get('coordinate_system')!='undistorted_K':
        raise ValueError('Teach a real wrist contact reference; image center is not a contact calibration')
    camera.check_reference(value)
    if value.get('pad_fingerprint')!=pad_fingerprint(c):
        raise ValueError('Pad geometry/aperture changed: re-teach wrist contact reference')
    uv=np.asarray(value.get('contact_uv'),float)
    if uv.shape!=(2,) or not np.isfinite(uv).all() or (uv<2).any() or (uv>=np.array(camera.size)-2).any():
        raise ValueError('Invalid taught wrist contact pixel')
    return uv


def horizontal_pad(c, rotation):
    pad=FingerPad(**c['pad'])
    if np.dot(rotation@pad.normal_tcp,[0,0,-1])<np.cos(np.deg2rad(c['horizontal_tolerance_deg'])):
        raise ValueError('Set the designated finger underside horizontal before coarse approach')
    return pad


def head_geometry(obs):
    corners=np.asarray(obs['corners_base_mm'],float)
    if corners.shape!=(4,3) or not np.isfinite(corners).all(): raise ValueError('Invalid measured spine corners')
    top=(corners[0]+corners[1])/2
    center=np.mean(corners,axis=0)
    if obs.get('geometry_scope')=='upper_edge_patch':
        return dict(visible_patch_center_base_mm=center,top_base_mm=top,
                    top_offset_from_patch_mm=top-center,geometry_scope='upper_edge_patch')
    return dict(spine_center_base_mm=center, top_base_mm=top, offset_xyz_mm=top-center)


def coarse_plan(obs, tcp, rotation, c):
    d=settings(c); pad=horizontal_pad(c,rotation); geo=head_geometry(obs)
    hover=geo['top_base_mm']+[0,0,d['coarse_clearance_mm']]-rotation@pad.center_tcp_mm
    # Raise at the starting XY before moving over the book, then approach hover.
    high=max(float(tcp[2]),float(hover[2]))
    goals=[np.array([tcp[0],tcp[1],high]),np.array([hover[0],hover[1],high]),hover]
    path=[];p=np.asarray(tcp,float)
    for goal in goals:
        if np.linalg.norm(goal-p)>.05:path.append(goal.copy());p=goal
    length=sum(np.linalg.norm(b-a) for a,b in zip([tcp]+path,path))
    if length>d['coarse_max_travel_mm']:raise ValueError('V4.1 coarse path exceeds configured reach budget')
    return dict(**geo,tcp_hover_mm=hover,waypoints_mm=path,requested_path_mm=float(length))


def fit_jacobian(displacements, responses, c):
    d=settings(c); q=np.asarray(displacements,float);u=np.asarray(responses,float)
    if q.ndim!=2 or q.shape[1]!=3 or u.shape!=(len(q),2) or len(q)<3 or not np.isfinite(q).all() or not np.isfinite(u).all():
        raise ValueError('Invalid measured Image Jacobian samples')
    if np.linalg.matrix_rank(q)<3 or np.linalg.cond(q)>10:
        raise ValueError('Probe motion did not excite XYZ independently')
    j=np.linalg.lstsq(q,u,rcond=None)[0].T
    error=float(np.max(np.linalg.norm(q@j.T-u,axis=1)))
    if error>d['max_fit_error_px']:raise ValueError('Image Jacobian fit inconsistent')
    sv=np.linalg.svd(j[:,:2],compute_uv=False)
    if sv[-1]<d['min_response_px_per_mm'] or sv[0]/sv[-1]>d['max_condition']:
        raise ValueError('Wrist image cannot distinguish base X/Y at this view')
    return j,error


class LandingController:
    """Wrist UV + measured base-Z gap; pause and re-observe before success.

    The empirical 2x3 Jacobian includes eye-in-hand sign and camera mounting.
    It is fitted from actual probe motion, not commanded steps. Z is resolved
    by current head RGB-D + pad FK; monocular pixel overlap alone never wins.
    """
    def __init__(self,c,top,tcp,rotation,jacobian,goal_uv):
        self.c,self.d=c,settings(c);self.pad=horizontal_pad(c,rotation)
        self.rotation=np.asarray(rotation).copy();self.top0=vector(top,'top')
        self.origin=vector(tcp,'TCP');self.previous=self.origin.copy()
        self.j=np.asarray(jacobian,float);self.goal=np.asarray(goal_uv,float)
        if self.j.shape!=(2,3) or self.goal.shape!=(2,) or not np.isfinite(self.j).all() or not np.isfinite(self.goal).all():
            raise ValueError('Invalid landing Jacobian/reference')
        fit_jacobian(np.eye(3),self.j.T,c)
        self.path=0.;self.last_stamp=-np.inf;self.window=[];self.last_phase='ALIGN'

    def evidence(self, obs, paired_tcp, paired_rotation):
        top=vector(obs['top_base_mm'],'top')
        if np.linalg.norm(top-self.top0)>self.d['max_book_motion_mm']:
            raise ValueError('Book moved before landing; hold, no automatic tilt')
        surface=self.pad.observe_book_point(top,paired_tcp,paired_rotation)
        # Base-Z gap (normal is near vertical); retain true pad-plane gap too.
        pad_center=paired_tcp+paired_rotation@self.pad.center_tcp_mm
        gap=float(pad_center[2]-top[2]);uv=np.asarray(obs['wrist_contact_uv'],float)
        if uv.shape!=(2,) or not np.isfinite(uv).all():raise ValueError('Missing current wrist top-edge measurement')
        goal=np.asarray(obs.get('pad_contact_uv',self.goal),float)
        if goal.shape!=(2,) or not np.isfinite(goal).all():raise ValueError('Missing current pad landmark')
        normalized=uv-goal-self.j[:,2]*gap
        lateral=float(np.linalg.norm([surface['along_mm'],surface['across_mm']]))
        pixel=float(np.linalg.norm(uv-goal))
        aligned=bool(surface['inside_pad'] and lateral<=self.d['alignment_tolerance_mm'] and
                     np.linalg.norm(normalized)<=self.d['pixel_tolerance'])
        valid=bool(aligned and pixel<=self.d['pixel_tolerance'] and
                   0<=gap<=self.d['gap_tolerance_mm'] and
                   abs(surface['signed_gap_mm'])<=self.d['gap_tolerance_mm'])
        return dict(gap_mm=gap,pad_plane_gap_mm=surface['signed_gap_mm'],pixel_error=pixel,
            aligned=aligned,inside_pad=surface['inside_pad'],lateral_mm=lateral,
            landing_evidence=valid,physical_contact_confirmed=False),normalized,uv

    def decide(self, obs, paired_tcp, paired_rotation, latest_tcp, latest_rotation, *, stationary=False):
        stamp=float(obs['captured_unix_s'])
        if not np.isfinite(stamp) or stamp<=self.last_stamp:raise ValueError('Landing frame stale/repeated')
        self.last_stamp=stamp
        if not obs.get('tracking_valid',False):raise ValueError('Upper-edge tracking unavailable: hold')
        if np.rad2deg(Rotation.from_matrix(self.rotation.T@latest_rotation).magnitude())>self.c['orientation_error_deg']:
            raise ValueError('Finger orientation changed during landing')
        self.path+=float(np.linalg.norm(latest_tcp-self.previous));self.previous=np.asarray(latest_tcp).copy()
        if self.path>self.c['max_path_mm'] or np.linalg.norm(latest_tcp-self.origin)>self.d['max_servo_travel_mm']:
            raise ValueError('V4.1 landing measured travel budget')
        ev,normalized,uv=self.evidence(obs,paired_tcp,paired_rotation)
        if ev['gap_mm'] < -self.d['penetration_limit_mm'] or ev['pad_plane_gap_mm'] < -self.d['penetration_limit_mm']:
            raise ValueError('Predicted pad penetration: hold without further descent')
        if ev['landing_evidence']:
            if stationary:
                self.window.append((stamp,np.asarray(latest_tcp).copy(),np.asarray(obs['top_base_mm']).copy(),uv.copy()))
                a=self.window[0]
                if (np.linalg.norm(latest_tcp-a[1])>.3 or np.linalg.norm(obs['top_base_mm']-a[2])>.5 or np.linalg.norm(uv-a[3])>self.d['pixel_tolerance']):
                    self.window=self.window[-1:]
                done=len(self.window)>=self.d['stable_frames'] and stamp-self.window[0][0]>=self.d['stable_seconds']
                return dict(phase='LANDED_VISUAL' if done else 'VERIFY',target=None,**ev)
            self.window=[]
            return dict(phase='PAUSE_VERIFY',target=None,**ev)
        self.window=[]
        if stationary:return dict(phase='VERIFY_FAILED',target=None,**ev)
        # Compensate measured motion between wrist exposure and dispatch.
        moved=np.asarray(latest_tcp)-paired_tcp
        uv_now=uv+self.j@moved
        gap_now=ev['gap_mm']+moved[2]
        if gap_now<0:raise ValueError('Negative current gap: no blind downward correction')
        goal=np.asarray(obs.get('pad_contact_uv',self.goal),float)
        planar_error=uv_now-goal-self.j[:,2]*gap_now
        xy=-self.c['visual_gain']*np.linalg.solve(self.j[:,:2],planar_error)
        dz=0.
        phase='ALIGN'
        if ev['aligned']:
            phase='DESCEND'
            # Absolute floor from observed book height: no repeated press trial.
            dz=-min(self.d['descent_step_mm'],max(0.,gap_now-self.c['contact_gap_mm']))
        delta=np.r_[xy,dz]
        limit=min(self.c['step_mm'],self.d['descent_step_mm'] if phase=='DESCEND' else self.c['step_mm'])
        delta*=min(1.,limit/max(np.linalg.norm(delta),1e-12))
        if self.path+np.linalg.norm(delta)>self.c['max_path_mm']+1e-6:raise ValueError('V4.1 landing path budget')
        if np.linalg.norm(delta)<.01:
            raise ValueError('Landing evidence incomplete and no resolvable correction; hold')
        self.last_phase=phase
        return dict(phase=phase,target=np.asarray(latest_tcp)+delta,**ev)
