"""Texture-assisted centerline tracking with an explicit point-exhaustion signal."""
import time
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError


class FeaturePointsExhausted(ServoError):
    def __init__(self,count,minimum=6):
        self.count=int(count);self.minimum=int(minimum)
        super().__init__(f'Centerline reliable points below threshold: {count}/{minimum}')


class InitialFeaturesUnavailable(ServoError):
    """Selected region has insufficient real texture before tracking starts."""


class CenterlineGeometryError(ServoError):
    """A fresh image could not support the tracked centerline geometry."""


class FeatureCenterlineLock:
    minimum_points=6

    def __init__(self,rgb,mask,*,tracking_mask=None,centerline=None):
        from cart_book_grasp_v3_centerline import visible_centerline
        self.shape=rgb.shape;self.gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
        u,theta=visible_centerline(mask) if centerline is None else (centerline['center_u'],centerline['angle_deg'])
        h,w=rgb.shape[:2];cy=(h-1)/2
        self.anchor=np.array([u,cy]);slope=np.tan(np.radians(theta))
        self.line=np.array([[u-slope*h/2,cy-h/2],[u+slope*h/2,cy+h/2]])
        if centerline is not None:self.line=np.asarray(centerline['line'],float)
        self.reference_rgb=rgb.copy();self.reference_mask=np.asarray(mask,bool).copy()
        self.gpt_geometry=centerline is not None
        interior=cv2.erode(np.uint8(mask),np.ones((3,3),np.uint8))*255
        if tracking_mask is not None:
            tracking_mask=np.asarray(tracking_mask,bool)
            if tracking_mask.shape!=mask.shape or np.any(tracking_mask & ~np.asarray(mask,bool)):
                raise ServoError('Tracking region must lie inside the selected spine')
            interior &= np.uint8(tracking_mask)*255
        self.tracking_mask=interior>0
        p=cv2.goodFeaturesToTrack(self.gray,80,.005,5,mask=interior,blockSize=5)
        # Failure to initialize is not loss of an already tracked target.
        if p is None or len(p)<self.minimum_points:raise InitialFeaturesUnavailable('Book lacks reliable initial centerline points')
        self.reference=p[:,0].copy();self.points=self.reference.copy();self.ids=np.arange(len(p))
        self.last_u=u;self.last_angle=theta
        self.last_scale=1.

    def observe(self,rgb):
        started=time.monotonic()
        if rgb.shape!=self.shape or rgb.dtype!=np.uint8:raise ServoError('Wrist RGB format changed')
        gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
        args=dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,20,.01))
        p,ok,_=cv2.calcOpticalFlowPyrLK(self.gray,gray,self.points[:,None],None,**args)
        if p is None:raise FeaturePointsExhausted(0)
        back,bok,_=cv2.calcOpticalFlowPyrLK(gray,self.gray,p,None,**args)
        if back is None:raise FeaturePointsExhausted(0)
        p=p[:,0];h,w=gray.shape
        keep=ok.ravel().astype(bool)&bok.ravel().astype(bool)&np.isfinite(p).all(axis=1)
        keep&=np.linalg.norm(back[:,0]-self.points,axis=1)<=1.5
        keep&=(p>=[2,2]).all(axis=1)&(p<[w-2,h-2]).all(axis=1)
        points,ids=p[keep],self.ids[keep]
        if len(points)<self.minimum_points:raise FeaturePointsExhausted(len(points))
        matrix,inliers=cv2.estimateAffinePartial2D(self.reference[ids],points,method=cv2.RANSAC,
            ransacReprojThreshold=2.5,maxIters=300,confidence=.99,refineIters=10)
        if matrix is None or inliers is None:raise CenterlineGeometryError('Centerline geometry fit failed')
        good=inliers.ravel().astype(bool)
        # Low consensus with many observations is an inconsistent model, not
        # simple attrition. It must never authorize a blind forward segment.
        if good.mean()<.6:raise CenterlineGeometryError(f'Centerline geometry lacks consensus: inliers={int(good.sum())}/{len(good)}, required=60%')
        if good.sum()<self.minimum_points:raise FeaturePointsExhausted(good.sum())
        span=np.linalg.norm(np.ptp(points[good],axis=0)) if self.gpt_geometry else np.ptp(points[good,1])
        if span<20:raise CenterlineGeometryError('Centerline geometry lacks spatial span')
        line=self.line@matrix[:,:2].T+matrix[:,2];d=line[1]-line[0]
        if not self.gpt_geometry and abs(d[1])<.5*np.linalg.norm(d):raise CenterlineGeometryError('Tracked centerline too horizontal')
        t=((h-1)/2-line[0,1])/d[1] if abs(d[1])>1e-8 else .5
        if self.gpt_geometry:t=np.clip(t,0,1)
        u=float((line[0]+t*d)[0]);theta=float(np.degrees(np.arctan2(d[0],d[1])))
        scale=float(np.linalg.norm(matrix[:,0]))
        if not .25<=scale<=4 or not 2<=u<w-2 or abs(u-self.last_u)>80 or abs(theta-self.last_angle)>12:
            raise CenterlineGeometryError('Centerline scale/position/angle jump exceeded')
        self.gray=gray;self.points=points[good].copy();self.ids=ids[good];self.last_u=u;self.last_angle=theta
        self.last_scale=scale
        return dict(uv=np.array([u,(h-1)/2]),angle_deg=theta,line=line,scale=scale,
            points=self.points.copy(),visible=np.ones(len(self.points),bool),inliers=len(self.points),
            inference_seconds=time.monotonic()-started)
