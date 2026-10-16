"""CoTracker3 on bounded windows of actual settled wrist frames.

Point identities and the original anchor stay fixed for a cycle. Sliding the
window transports existing queries; it never detects a replacement book/point.
"""
from pathlib import Path
import sys
import time
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError

ROOT=Path(__file__).resolve().parent


def initialize_points(rgb,mask,count,minimum):
    ys,xs=np.nonzero(mask)
    if not len(xs):raise ServoError('Empty center-book mask')
    interior=cv2.erode(np.uint8(mask),np.ones((5,5),np.uint8))
    # Central visible spine/face region, away from silhouette and gripper edges.
    low,high=np.percentile(ys,[30,70]);interior[:int(low)]=0;interior[int(high)+1:]=0
    corners=cv2.goodFeaturesToTrack(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY),
        maxCorners=count,qualityLevel=.01,minDistance=8,mask=interior*255,blockSize=5)
    points=[] if corners is None else list(corners[:,0])
    # Point trackers accept arbitrary pixels; fill sparse texture with separated
    # foreground pixels, then validate visibility and geometry on later frames.
    y,x=np.nonzero(interior)
    if len(x):
        order=np.linspace(0,len(x)-1,min(len(x),200),dtype=int)
        for i in order:
            p=np.float32([x[i],y[i]])
            if all(np.linalg.norm(p-q)>=8 for q in points):points.append(p)
            if len(points)>=count:break
    points=np.float32(points[:count]).reshape(-1,2)
    if len(points)<minimum or cv2.contourArea(cv2.convexHull(points))<40:
        raise ServoError('Center book has insufficient separated point support')
    return points


class CoTrackerModel:
    def __init__(self,c):
        path=Path(c['checkpoint']);path=path if path.is_absolute() else ROOT/path
        if not path.is_file():raise ServoError(f'CoTracker local checkpoint missing: {path}')
        sys.path.insert(0,str(ROOT/'third_party/co-tracker')) if str(ROOT/'third_party/co-tracker') not in sys.path else None
        try:
            import torch
            from cotracker.predictor import CoTrackerPredictor
            self.torch=torch
            self.device=('cuda' if torch.cuda.is_available() else 'cpu') if c['device']=='auto' else c['device']
            if self.device=='cuda' and not torch.cuda.is_available():raise ServoError('CoTracker CUDA unavailable')
            torch.set_num_threads(min(4,torch.get_num_threads()))
            self.model=CoTrackerPredictor(checkpoint=str(path),offline=True).eval().to(self.device)
        except Exception as error:raise ServoError(f'CoTracker initialization failed: {error}') from error

    def predict(self,frames,points):
        torch=self.torch
        video=torch.from_numpy(np.stack(frames).transpose(0,3,1,2).copy()).float()[None].to(self.device)
        queries=torch.from_numpy(np.column_stack([np.zeros(len(points)),points]).astype(np.float32))[None].to(self.device)
        with torch.inference_mode():tracks,visible=self.model(video,queries=queries,backward_tracking=False)
        return tracks[0].cpu().numpy(),visible[0].cpu().numpy()


class PointLock:
    def __init__(self,model,rgb,mask,c):
        self.model=model;self.c=c;self.shape=rgb.shape
        self.reference=initialize_points(rgb,mask,c['point_count'],c['min_visible_points'])
        self.anchor=self.reference.mean(axis=0);self.queries=self.reference.copy();self.frames=[rgb.copy()]
        self.last_uv=self.anchor.copy()

    def observe(self,rgb):
        started=time.monotonic()
        if rgb.shape!=self.shape:raise ServoError('CoTracker wrist resolution changed')
        self.frames.append(rgb.copy())
        tracks,visible=self.model.predict(self.frames,self.queries)
        points=tracks[-1];keep=visible[-1].astype(bool)&np.isfinite(points).all(axis=1)
        h,w=rgb.shape[:2];keep&=(points>=[3,3]).all(axis=1)&(points<[w-3,h-3]).all(axis=1)
        if keep.sum()<self.c['min_visible_points']:raise ServoError(f'CoTracker point visibility lost: {keep.sum()}/{len(keep)}')
        matrix,inliers=cv2.estimateAffinePartial2D(self.reference[keep],points[keep],method=cv2.RANSAC,ransacReprojThreshold=3)
        if matrix is None or inliers is None or not np.isfinite(matrix).all() or inliers.sum()<self.c['min_visible_points']:
            raise ServoError('CoTracker point geometry inconsistent')
        if inliers.mean()<.65:raise ServoError('CoTracker point consensus too low')
        uv=matrix@np.r_[self.anchor,1.]
        if np.linalg.norm(uv-self.last_uv)>80 or not ((uv>=[3,3]).all() and (uv<[w-3,h-3]).all()):
            raise ServoError('CoTracker anchor jumped or left view')
        self.last_uv=uv.copy()
        if len(self.frames)>=self.c['window_frames']:
            if not visible[1].all() or not np.isfinite(tracks[1]).all():raise ServoError('Cannot transport occluded queries into next window')
            self.frames.pop(0);self.queries=tracks[1].copy()
        return dict(uv=uv,points=points,visible=keep,inliers=int(inliers.sum()),
                    scale=float(np.sqrt(np.linalg.det(matrix[:,:2]))),inference_seconds=time.monotonic()-started)


def draw_points(rgb,observation,target):
    image=rgb.copy()
    for p,v in zip(observation['points'],observation['visible']):
        if np.isfinite(p).all():cv2.circle(image,tuple(np.rint(p).astype(int)),4,(0,220,100) if v else (255,60,60),-1)
    cv2.drawMarker(image,tuple(np.rint(target).astype(int)),(255,210,0),cv2.MARKER_CROSS,24,2)
    cv2.circle(image,tuple(np.rint(observation['uv']).astype(int)),7,(255,255,255),2)
    return image
