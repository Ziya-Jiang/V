"""CPU blue RGB-D segmentation and measured-cart shape registration.

Color proposes a candidate; depth, shape residual and both observed handle bars
must support it. No color-only metric pose or unseen handle is accepted.
"""
from dataclasses import dataclass
import copy
import cv2
import numpy as np
from scipy.spatial import cKDTree
from .geometry import Rejected,finite,apply,pose,wrap

@dataclass
class CartObservation:
    stamp: float
    base_T_cart: np.ndarray
    handles: np.ndarray
    bar_axis: np.ndarray
    residual_m: float
    inlier_fraction: float
    handle_support: list
    points: int

    def to_dict(self):
        return {k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in self.__dict__.items()}

def model_points(c,step=0.025):
    """Blue trays, two end panels and exposed handle bars; units metres."""
    hx=c['handle_half_separation_m'];width=c['width_m'];points=[]
    xs=np.linspace(-c['length_m']/2,c['length_m']/2,max(3,int(c['length_m']/step)))
    ys=np.linspace(-width/2,width/2,max(3,int(width/step)))
    for z in c['shelf_heights_m']:
        points.extend((x,y,z) for x in xs for y in ys)
    panel_top=c['handle_height_m']-.08-c['bar_section_m']
    zs=np.arange(c['shelf_heights_m'][0],panel_top+step/2,step)
    for x in [-hx,hx]:
        points.extend((x,y,z) for y in ys for z in zs)
        points.extend((x,y,c['handle_height_m']) for y in ys)
        points.extend((x,y,z) for y in [-width/2,width/2] for z in np.arange(panel_top,c['handle_height_m'],step))
    return np.asarray(points,dtype=float)

def cloud_from_rgbd(bgr,depth,k,base_T_camera,cfg,depth_scale,rectified_aligned,distortion=None,distortion_model="plumb_bob"):
    if rectified_aligned is not True:raise Rejected('Need RGB and depth aligned into the same color optical image geometry')
    if bgr.dtype!=np.uint8 or bgr.ndim!=3 or bgr.shape[2]!=3 or depth.shape!=bgr.shape[:2]:raise Rejected('RGB/depth dimensions or encoding mismatch')
    k=finite(k,(3,3))
    if k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1]):raise Rejected('Invalid intrinsics')
    if not np.isfinite(depth_scale) or depth_scale<=0:raise Rejected('Invalid depth scale')
    h,w=depth.shape
    if not (0<=k[0,2]<w and 0<=k[1,2]<h):raise Rejected('Intrinsics do not match image')
    mask=cv2.inRange(cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV),np.array(cfg['hsv_lower']),np.array(cfg['hsv_upper']))
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask)
    ids=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=cfg['min_pixels']]
    if len(ids)!=1:raise Rejected('Need one unambiguous blue cart candidate; found '+str(len(ids)))
    v,u=np.nonzero(labels==ids[0]);z=depth[v,u].astype(float)*depth_scale
    ok=np.isfinite(z)&(z>=cfg['depth_min_m'])&(z<=cfg['depth_max_m']);u,v,z=u[ok],v[ok],z[ok]
    if len(z)<cfg['min_pixels']:raise Rejected('Insufficient valid blue-object depth')
    stride=max(1,int(np.ceil(len(z)/cfg['max_points'])));u,v,z=u[::stride],v[::stride],z[::stride]
    if distortion is None:normalized=np.column_stack(((u-k[0,2])/k[0,0],(v-k[1,2])/k[1,1]))
    else:
        d=finite(distortion)
        if distortion_model!='plumb_bob' or d.shape not in [(4,),(5,)]:raise Rejected('Unsupported distortion model/coefficient order')
        pixels=np.column_stack((u,v)).astype(np.float64).reshape(-1,1,2)
        normalized=cv2.undistortPoints(pixels,k,d).reshape(-1,2)
    points=np.column_stack((normalized[:,0]*z,normalized[:,1]*z,z))
    return apply(base_T_camera,points),mask

class CartDetector:
    def __init__(self,cfg):
        self.p=cfg['perception'];self.c=cfg['cart'];self.model=model_points(self.c);self.tree=cKDTree(self.model)
    def estimate(self,points,stamp):
        p=finite(points)
        if p.ndim!=2 or p.shape[1]!=3 or len(p)<self.p['min_pixels'] or not np.isfinite(stamp) or stamp<=0:raise Rejected('Invalid cloud/stamp')
        _,index=np.unique(np.floor(p/self.p['voxel_m']).astype(np.int64),axis=0,return_index=True);p=p[index]
        support_cloud=p.copy()
        if len(p)>self.p['max_points']:p=p[::int(np.ceil(len(p)/self.p['max_points']))]
        center=np.median(p[:,:2],axis=0);floor=self.c['base_floor_z_m'];solutions=[]
        # Both yaw signs are equivalent for this symmetric cart. Later assignment
        # uses robot-left/robot-right, never an invented front/back appearance cue.
        for yaw in np.linspace(-np.pi,np.pi,16,endpoint=False):
            t=pose([*center,floor],[0,0,yaw])
            for _ in range(25):
                local=apply(np.linalg.inv(t),p);distance,idx=self.tree.query(local)
                keep=distance<=max(.025,float(np.quantile(distance,.8)));a=self.model[idx[keep]];b=p[keep]
                if len(a)<40:break
                ac=a[:,:2]-a[:,:2].mean(0);bc=b[:,:2]-b[:,:2].mean(0)
                angle=np.arctan2(np.sum(ac[:,0]*bc[:,1]-ac[:,1]*bc[:,0]),np.sum(ac*bc))
                r=pose(rpy=[0,0,angle])[:3,:3];translation=b.mean(0)-r@a.mean(0);translation[2]=np.clip(translation[2],floor-.08,floor+.08)
                new=pose(translation,[0,0,angle]);change=np.linalg.norm(new-t);t=new
                if change<1e-6:break
            distance,_=self.tree.query(apply(np.linalg.inv(t),p));inside=distance<self.p['icp_max_residual_m'];fraction=float(inside.mean());rms=float(np.sqrt(np.mean(distance[np.argsort(distance)[:max(1,int(.8*len(distance)))]]**2)))
            solutions.append((rms+(1-fraction)*.05,t,fraction,rms))
        _,t,fraction,rms=min(solutions,key=lambda x:x[0])
        if rms>self.p['icp_max_residual_m'] or fraction<self.p['minimum_inlier_fraction']:raise Rejected('Cart shape does not match depth')
        if abs(t[2,3]-floor)>self.p['maximum_ground_offset_m']:raise Rejected('Cart ground offset inconsistent with calibrated floor')
        local=apply(np.linalg.inv(t),support_cloud);support=[];handles=[]
        for x in [-self.c['handle_half_separation_m'],self.c['handle_half_separation_m']]:
            near=(abs(local[:,0]-x)<self.p['handle_support_radius_m'])&(abs(local[:,2]-self.c['handle_height_m'])<self.p['handle_support_radius_m'])&(abs(local[:,1])<self.c['width_m']/2+.02)
            observed=local[near];span=float(np.ptp(observed[:,1])) if len(observed) else 0.
            if len(observed)<self.p['min_handle_points'] or span<self.p['handle_min_observed_span_m']:raise Rejected('Both handle bars need measured depth support')
            # Refine bar height/lateral offset with local median of the observed bar.
            h=np.array([np.median(observed[:,0]),0.,np.median(observed[:,2])]);world=apply(t,h)
            axis=t[:3,1];side_sign=1 if np.dot(-world[:2],axis[:2])>=0 else -1
            h[1]=side_sign*self.c['grasp_offset_along_bar_m'];handles.append(apply(t,h));support.append({'points':len(observed),'span_m':span})
        order=np.argsort([-h[1] for h in handles]);handles=np.asarray(handles)[order]
        if handles[0,1]-handles[1,1]<.15:raise Rejected('Rotate/view cart broadside: two-ended handles not separated left/right')
        if np.min(handles[:,0])<.15:raise Rejected('Cart handles must be in front of robot')
        return CartObservation(float(stamp),t,handles,t[:3,1],rms,fraction,[support[i] for i in order],len(p))

class StableTracker:
    def __init__(self,cfg):self.p=cfg;self.last=None;self.count=0
    def clear(self):self.last=None;self.count=0
    def update(self,observation):
        previous=self.last
        if previous is not None:
            dt=observation.stamp-previous.stamp
            if dt<=0 or dt>1.0:self.clear();raise Rejected('Duplicate, reordered or discontinuous observation')
            displacement=np.max(np.linalg.norm(observation.handles-previous.handles,axis=1))
            # pi-invariant registration orientation, but physical handle order stable.
            a=observation.bar_axis;b=previous.bar_axis;angle=np.arccos(np.clip(abs(np.dot(a,b)),0,1))
            if displacement>self.p['stable_translation_m'] or angle>self.p['stable_yaw_rad']:self.count=0
        self.last=copy.deepcopy(observation);self.count+=1
        return self.count>=self.p['stable_frames']
