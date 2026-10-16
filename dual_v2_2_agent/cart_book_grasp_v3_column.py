"""Fast visible-book column tracking and one-dimensional Y alignment."""
import json
import time
from pathlib import Path
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError,vector

CONFIG_PATH=Path(__file__).with_suffix('.json')


def load_config(path=CONFIG_PATH):
    c=json.loads(Path(path).read_text())
    bounds=dict(probe_mm=(1,3),max_step_mm=(.1,5),max_forward_mm=(1,250),max_path_mm=(20,500),
        gain=(.05,.7),tolerance_px=(.5,8),x_tolerance_mm=(.1,2),max_seconds=(10,900),
        max_frame_age_seconds=(.1,5),max_pose_error_mm=(.1,3),max_orientation_error_deg=(.1,3),
        coarse_pose_error_mm=(1,8),coarse_settle_seconds=(.1,1),coarse_feedback_timeout_seconds=(.5,5),settle_seconds=(0,2))
    ints=dict(stable_frames=(2,5),max_iterations=(1,400))
    if c.get('mode')=='centerline_distance':
        bounds.update(probe_roll_deg=(1,3),max_roll_step_deg=(.5,3),max_roll_deg=(3,45),angle_tolerance_deg=(.5,3))
        ints.update(snapshot_every=(1,1000))
    if set(c)!=set(bounds)|set(ints)|{'mode','target_u'} or c['mode'] not in ('column_distance','centerline_distance'):
        raise ServoError('Invalid column configuration schema')
    for k,(lo,hi) in {**bounds,**ints}.items():
        if type(c[k]) not in ((int,) if k in ints else (int,float)) or not np.isfinite(c[k]) or not lo<=c[k]<=hi:
            raise ServoError(f'Invalid column setting: {k}')
    if c['target_u'] is not None and (type(c['target_u']) not in (int,float) or not np.isfinite(c['target_u'])):
        raise ServoError('Invalid target column')
    return c


def visible_center_column(mask):
    """Median midpoint of occupied rows; accepts a vertically clipped book."""
    mask=np.asarray(mask,bool);rows=np.flatnonzero(mask.any(axis=1))
    if len(rows)<8:raise ServoError('Too few visible book rows')
    left=mask[rows].argmax(axis=1);right=mask.shape[1]-1-mask[rows,::-1].argmax(axis=1)
    valid=right-left>=4
    if valid.sum()<8:raise ServoError('Visible book is too narrow')
    return float(np.median((left[valid]+right[valid])*.5))


class ColumnLock:
    """Sparse pyramidal LK + forward/backward check; only horizontal geometry.

    Track the same pixels for a cycle. Fit u=a*u0+b to transport the original
    visible-mask center, independent of the points' vertical centroid.
    """
    def __init__(self,rgb,mask):
        self.gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);self.shape=rgb.shape
        self.center=visible_center_column(mask);self.last_u=self.center
        interior=cv2.erode(np.uint8(mask),np.ones((3,3),np.uint8))
        p=cv2.goodFeaturesToTrack(self.gray,80,.005,5,mask=interior*255,blockSize=5)
        if p is None or len(p)<6:raise ServoError('Visible book lacks reliable points for fast column tracking')
        self.reference=p[:,0].copy();self.points=self.reference.copy();self.ids=np.arange(len(p))
        self.anchor=np.array([self.center,(rgb.shape[0]-1)/2])

    def observe(self,rgb):
        started=time.monotonic()
        if rgb.shape!=self.shape:raise ServoError('Wrist resolution changed')
        gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
        args=dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,20,.01))
        p,ok,_=cv2.calcOpticalFlowPyrLK(self.gray,gray,self.points[:,None],None,**args)
        if p is None:raise ServoError('Column optical flow failed')
        back,back_ok,_=cv2.calcOpticalFlowPyrLK(gray,self.gray,p,None,**args)
        if back is None:raise ServoError('Column reverse optical flow failed')
        p=p[:,0];h,w=gray.shape
        keep=ok.ravel().astype(bool)&back_ok.ravel().astype(bool)&np.isfinite(p).all(axis=1)
        keep&=np.linalg.norm(back[:,0]-self.points,axis=1)<=1.5
        keep&=(p>=[2,2]).all(axis=1)&(p<[w-2,h-2]).all(axis=1)
        points=p[keep];ids=self.ids[keep]
        if len(points)<6:raise ServoError(f'Column tracking lost: {len(points)} reliable points')
        x=self.reference[ids,0];y=points[:,0]
        candidates=[(1.,float(np.median(y-x)))]
        pairs=np.column_stack(np.triu_indices(len(x),1));pairs=pairs[np.abs(x[pairs[:,1]]-x[pairs[:,0]])>=8]
        for i,j in pairs[np.linspace(0,len(pairs)-1,min(96,len(pairs)),dtype=int)] if len(pairs) else []:
            a=float((y[j]-y[i])/(x[j]-x[i]))
            if .25<=a<=4:candidates.append((a,float(y[i]-a*x[i])))
        def rank(ab):
            residual=np.abs(y-(ab[0]*x+ab[1]));return (int((residual<=2.5).sum()),-float(np.median(residual)))
        a,b=max(candidates,key=rank);inliers=np.abs(y-(a*x+b))<=2.5
        if inliers.sum()<6 or inliers.mean()<.6:raise ServoError('Column horizontal geometry inconsistent')
        if np.ptp(x[inliers])>=8:
            a,b=np.linalg.lstsq(np.column_stack([x[inliers],np.ones(inliers.sum())]),y[inliers],rcond=None)[0]
        inliers=np.abs(y-(a*x+b))<=2.5
        if inliers.sum()<6 or inliers.mean()<.6:raise ServoError('Column horizontal fit lost consensus')
        u=float(a*self.center+b)
        if not .25<=a<=4 or not 2<=u<w-2 or abs(u-self.last_u)>80:
            raise ServoError('Column scale/jump outside valid range')
        self.gray=gray;self.points=points[inliers].copy();self.ids=ids[inliers];self.last_u=u
        return dict(uv=np.array([u,(h-1)/2]),scale=float(a),points=self.points.copy(),
            visible=np.ones(len(self.points),bool),inliers=len(self.points),inference_seconds=time.monotonic()-started)


def run_column_servo(observe,move,target_u,goal_x,c,*,checkpoint=lambda:None,log=lambda e:None,clock=time.monotonic):
    """Probe Y once, align u only, then interleave X steps with Y correction."""
    started=clock();stamp=-np.inf;origin=None;path=0.;j=None;reference_scale=1.;stable=0
    def check():
        checkpoint()
        if clock()-started>c['max_seconds']:raise ServoError('Column approach time budget exceeded')
    def read():
        nonlocal stamp
        check();o=observe();check();q=vector(o['xyz_mm'],3,'measured TCP');u=float(o['uv'][0]);s=float(o['scale']);t=float(o['captured_at'])
        if not np.isfinite([u,s,t]).all() or not .1<=s<=20 or t<=stamp or not 0<=clock()-t<=c['max_frame_age_seconds']:
            raise ServoError('Stale or invalid column observation')
        stamp=t
        if origin is not None:
            if q[0]<origin[0]-c['max_pose_error_mm'] or q[0]>goal_x+c['x_tolerance_mm']:raise ServoError('Column X outside endpoint interval')
            if abs(q[2]-origin[2])>c['max_pose_error_mm']:raise ServoError('Column mode measured Z drift exceeded')
        log(dict(event='column_observation',u=u,error_px=u-target_u,xyz_mm=q.tolist(),remaining_x_mm=goal_x-q[0]))
        return u,q,s
    def go(goal,q):
        nonlocal path
        check();goal=goal.copy();goal[2]=origin[2]
        length=float(np.linalg.norm(goal-q))
        if length>max(c['max_step_mm'],c['probe_mm'])+1e-6:raise ServoError('Column step bound exceeded')
        if path+length>c['max_path_mm']:raise ServoError('Column path budget exceeded')
        if goal[0]>max(goal_x,q[0]) or goal[0]<min(origin[0],q[0]):raise ServoError('Column command outside X interval')
        if not move(goal):raise ServoError('Column waypoint not confirmed')
        path+=length;u,actual,s=read()
        if np.linalg.norm(actual-goal)>c['max_pose_error_mm']:raise ServoError('Column measured waypoint error')
        return u,actual,s
    u,q,s=read();origin=q.copy()
    if not np.isfinite([target_u,goal_x]).all() or not -c['x_tolerance_mm']<=goal_x-q[0]<=c['max_forward_mm']:
        raise ServoError('Column V1_3 endpoint exceeds forward budget')
    for iteration in range(c['max_iterations']+c['stable_frames']):
        check();error=u-target_u;remaining=goal_x-q[0]
        if abs(error)<=c['tolerance_px'] and abs(remaining)<=c['x_tolerance_mm']:
            stable+=1
            if stable>=c['stable_frames']:
                result=dict(converged=True,error_px=abs(error),actual_x_mm=float(q[0]),path_mm=path)
                log(dict(event='converged_distance',**result));return result
            u,q,s=read();continue
        stable=0
        if iteration>=c['max_iterations']:break
        if j is None:
            old_u,old_q=u,q.copy();goal=q.copy();goal[1]+=c['probe_mm']
            u,q,s=go(goal,q);dy=q[1]-old_q[1]
            if abs(dy)<.5*c['probe_mm']:raise ServoError('Y column probe response too small')
            j=(u-old_u)/dy
            if not .1<=abs(j)<=20:raise ServoError('Invalid horizontal pixel/Y response')
            reference_scale=s
            # Return Y only, keep measured X and the fixed Z target.
            for _ in range(5):
                goal=q.copy();dz=origin[2]-q[2];budget=np.sqrt(max(0.,c['max_step_mm']**2-dz**2))
                goal[1]+=np.clip(old_q[1]-q[1],-budget,budget)
                u,q,s=go(goal,q)
                if abs(q[1]-old_q[1])<=c['max_pose_error_mm']:break
            else:raise ServoError('Column Y probe did not return')
            if abs(u-old_u)>4:raise ServoError('Column tracking drift during probe return')
            log(dict(event='jacobian_y',pixels_per_mm=j));continue
        effective=j*s/reference_scale
        goal=q.copy();dz=origin[2]-q[2]
        budget=np.sqrt(max(0.,c['max_step_mm']**2-dz**2))
        if budget<.03:raise ServoError('No step budget after measured Z drift')
        if abs(error)>c['tolerance_px']:
            dy=float(np.clip(-c['gain']*error/effective,-budget,budget));goal[1]+=dy
            old_u,old_q=u,q.copy();u,q,s=go(goal,q)
            mismatch=abs((u-old_u)-effective*(q[1]-old_q[1]))
            if mismatch>4:raise ServoError(f'Column Y response disagrees with model: {mismatch:.3f}px')
        else:
            goal[0]=min(goal_x,q[0]+budget);u,q,s=go(goal,q)
    raise ServoError('Column alignment did not converge; grasp cancelled')
