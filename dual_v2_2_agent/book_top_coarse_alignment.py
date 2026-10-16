"""V4.1 non-contact preparation: closed fingers, Y, raise, X, wrist Y servo.

Pure geometry/controller. Units: base mm and undistorted wrist pixels.
The inherited 50 mm tool -Z offset is a coarse fingertip estimate, not contact calibration.
"""
import numpy as np
from book_top_landing import head_geometry

DEFAULTS = dict(fingertip_offset_mm=50., top_clearance_mm=40., front_standoff_mm=60.,
    max_coarse_y_mm=300., max_raise_mm=350., max_forward_mm=350.,
    max_coarse_path_mm=800., coarse_step_mm=50., coarse_axis_tolerance_mm=5., gripper_timeout_seconds=10.,
    closed_threshold=.95, gain=.5, probe_mm=4., max_y_step_mm=2.,
    max_servo_y_mm=30., max_servo_path_mm=100., servo_timeout_seconds=90.,
    tolerance_px=3., fixed_xz_mm=2., stable_seconds=.5, stable_frames=5,
    min_response_px_per_mm=.2, max_response_px_per_mm=30., max_fit_error_px=2.,
    update_hz=10.)


def settings(c):
    d=dict(DEFAULTS, **c.get('coarse_alignment', {}))
    if set(d)!=set(DEFAULTS):raise ValueError('Unknown coarse_alignment key')
    bounds=dict(fingertip_offset_mm=(0,150),top_clearance_mm=(20,100),front_standoff_mm=(30,150),
        max_coarse_y_mm=(10,400),max_raise_mm=(10,400),max_forward_mm=(10,400),
        max_coarse_path_mm=(30,1200),coarse_step_mm=(10,50),coarse_axis_tolerance_mm=(1,8),gripper_timeout_seconds=(2,20),
        closed_threshold=(.9,1),gain=(.05,.8),probe_mm=(3,5),max_y_step_mm=(.5,3),
        max_servo_y_mm=(5,35),max_servo_path_mm=(10,150),servo_timeout_seconds=(10,180),
        tolerance_px=(1,5),fixed_xz_mm=(.5,3),stable_seconds=(.3,2),stable_frames=(3,20),
        min_response_px_per_mm=(.05,2),max_response_px_per_mm=(5,100),max_fit_error_px=(.5,3),
        update_hz=(2,15))
    for k,(lo,hi) in bounds.items():
        if type(d[k]) not in (int,float) or not np.isfinite(d[k]) or not lo<=d[k]<=hi:
            raise ValueError('Invalid coarse_alignment.'+k)
    if type(d['stable_frames']) is not int:raise ValueError('stable_frames must be integer')
    return d


def coarse_plan(obs, tcp, rotation, c):
    d=settings(c);q=np.asarray(tcp,float);r=np.asarray(rotation,float)
    if q.shape!=(3,) or r.shape!=(3,3) or not np.isfinite(q).all() or not np.isfinite(r).all():
        raise ValueError('Invalid TCP geometry')
    if not np.allclose(r.T@r,np.eye(3),atol=1e-4) or np.linalg.det(r)<.99:raise ValueError('Invalid rotation')
    forward=-r[:,2]
    if forward[0]<.8 or abs(forward[2])>np.sin(np.deg2rad(5)):
        raise ValueError('Start with closed fingers horizontal and pointing base +X (within 5 deg vertically)')
    h=head_geometry(obs);top=h['top_base_mm'];offset=forward*d['fingertip_offset_mm']
    # Align the closed-finger centre rather than the wrist's displaced origin.
    y=float(top[1]-offset[1]);z=max(float(q[2]),float(top[2]+d['top_clearance_mm']-offset[2]))
    x=float(top[0]-d['front_standoff_mm']-offset[0])
    if x<q[0]-1e-6:raise ValueError('Already beyond head-derived front observation plane; no automatic retreat')
    stages=[('align_y',np.array([q[0],y,q[2]])),('raise_z',np.array([q[0],y,z])),('forward_x',np.array([x,y,z]))]
    if abs(y-q[1])>d['max_coarse_y_mm'] or z-q[2]>d['max_raise_mm'] or x-q[0]>d['max_forward_mm']:
        raise ValueError('Head-derived coarse travel exceeds configured envelope')
    path=abs(y-q[1])+z-q[2]+x-q[0]
    if path>d['max_coarse_path_mm']:raise ValueError('Coarse path budget exceeded')
    return dict(**h,fingertip_offset_base_mm=offset,hover_tcp_mm=stages[-1][1],
        stages=stages,estimated_top_clearance_mm=z+offset[2]-top[2],
        front_standoff_mm=d['front_standoff_mm'],path_mm=path,contact_confirmed=False)


def segments(start, goal, limit):
    start,goal=np.asarray(start,float),np.asarray(goal,float)
    n=max(1,int(np.ceil(np.linalg.norm(goal-start)/limit)))
    return [start+(goal-start)*i/n for i in range(1,n+1)]


def centerline_u(landmarks, image_size):
    """Fixed spine landmarks propagated by homography, evaluated at centre row.

    Reject unsupported distant extrapolation; never use texture centroid.
    """
    p=np.asarray(landmarks,float);w,h=image_size
    if p.shape!=(2,2) or not np.isfinite(p).all():raise ValueError('Invalid spine centreline landmarks')
    v=p[1]-p[0];row=(h-1)/2
    if abs(v[1])<20:raise ValueError('Spine centreline has insufficient visible vertical extent')
    if row<min(p[:,1])-40 or row>max(p[:,1])+40:
        raise ValueError('Spine centreline not supported near wrist centre row')
    u=float(p[0,0]+(row-p[0,1])*v[0]/v[1])
    if not 2<u<w-2:raise ValueError('Spine centreline outside wrist image')
    return u


def fit_response(ys, us, c):
    d=settings(c);ys=np.asarray(ys,float);us=np.asarray(us,float)
    if ys.ndim!=1 or ys.shape!=us.shape or len(ys)<3 or not np.isfinite([ys,us]).all():
        raise ValueError('Invalid image-Jacobian samples')
    if np.ptp(ys)<d['probe_mm']*.6:raise ValueError('Y probe did not produce enough measured displacement')
    a=np.c_[ys-ys[0],np.ones(len(ys))];j,b=np.linalg.lstsq(a,us,rcond=None)[0]
    err=float(np.max(abs(a@np.array([j,b])-us)))
    if not d['min_response_px_per_mm']<=abs(j)<=d['max_response_px_per_mm'] or err>d['max_fit_error_px']:
        raise ValueError('Measured Y image response is weak/inconsistent')
    return float(j),err


class Alignment:
    def __init__(self,c,origin,j,target_u):
        self.d=settings(c);self.origin=np.asarray(origin,float).copy();self.previous=self.origin.copy()
        self.j=float(j);self.target_u=float(target_u);self.path=0.;self.stamps=[];self.last=-np.inf
        if not self.d['min_response_px_per_mm']<=abs(self.j)<=self.d['max_response_px_per_mm']:
            raise ValueError('Invalid Y Jacobian')
    def decide(self,obs,paired_tcp,current_tcp):
        d=self.d;p=np.asarray(paired_tcp,float);q=np.asarray(current_tcp,float);stamp=float(obs['captured_unix_s'])
        if not np.isfinite([*p,*q,stamp,obs['u']]).all() or stamp<=self.last or not obs.get('tracking_valid'):
            raise ValueError('Stale/repeated/invalid wrist observation')
        self.last=stamp;self.path+=float(np.linalg.norm(q-self.previous));self.previous=q.copy()
        if self.path>d['max_servo_path_mm']:raise ValueError('Wrist alignment path budget exceeded')
        if max(abs(q[[0,2]]-self.origin[[0,2]]))>d['fixed_xz_mm']:raise ValueError('Alignment fixed X/Z drift')
        if abs(q[1]-self.origin[1])>d['max_servo_y_mm']:raise ValueError('Alignment Y travel exceeded')
        # Account only for measured Y motion between exposure and dispatch.
        error=float(obs['u']-self.target_u+self.j*(q[1]-p[1]))
        if abs(error)<=d['tolerance_px']:
            self.stamps.append(stamp)
        else:self.stamps=[]
        done=len(self.stamps)>=d['stable_frames'] and self.stamps[-1]-self.stamps[0]>=d['stable_seconds']
        target=q.copy();target[[0,2]]=self.origin[[0,2]]
        if abs(error)>d['tolerance_px']:
            target[1]+=np.clip(-d['gain']*error/self.j,-d['max_y_step_mm'],d['max_y_step_mm'])
        if abs(target[1]-self.origin[1])>d['max_servo_y_mm']:raise ValueError('Alignment Y target outside envelope')
        return dict(target=target,error_px=error,done=done,path_mm=self.path)
