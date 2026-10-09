"""Local parallel-bar proposals for a partially occluded cart, never grasp confirmation.

Unlike full cart registration this works directly in SJ4Z coordinates. It does
not assume that a fixed waist makes SJ4Z vertical relative to the floor.
"""
import numpy as np
from .geometry import Rejected,finite

def fit_bar(points,seed):
    p=finite(points);_,idx=np.unique(np.floor(p/.005).astype(np.int64),axis=0,return_index=True);p=p[idx]
    if len(p)<60:raise Rejected('Insufficient blue depth on one side')
    rng=np.random.default_rng(seed);best=None
    for _ in range(180):
        a,b=p[rng.choice(len(p),2,replace=False)];axis=b-a;n=np.linalg.norm(axis)
        if n<.14:continue
        axis/=n
        if abs(axis[0])<.55:continue
        distance=np.linalg.norm(np.cross(p-a,axis),axis=1);inliers=p[distance<.018]
        if len(inliers)<40:continue
        projection=(inliers-a)@axis;span=np.quantile(projection,.95)-np.quantile(projection,.05)
        if not .18<span<.60:continue
        score=len(inliers)
        if best is None or score>best[0]:best=(score,inliers)
    if best is None:raise Rejected('No supported extended handle bar')
    inliers=best[1];center=np.mean(inliers,axis=0);_,_,vt=np.linalg.svd(inliers-center,full_matrices=False);axis=vt[0]
    if axis[0]<0:axis=-axis
    distance=np.linalg.norm(np.cross(p-center,axis),axis=1);inliers=p[distance<.018];center=np.mean(inliers,axis=0);t=(inliers-center)@axis;low,high=np.quantile(t,[.05,.95]);residual=float(np.sqrt(np.mean(np.linalg.norm(np.cross(inliers-center,axis),axis=1)**2)))
    if high-low<.18 or residual>.015:raise Rejected('Bar line support is weak')
    return {'center':center,'axis':axis,'points':len(inliers),'span_m':float(high-low),'rms_m':residual,'endpoints':np.array([center+low*axis,center+high*axis])}

def detect_handle_pair(points,stamp):
    p=finite(points)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<120:raise Rejected('Invalid local blue point cloud')
    # Proposal workspace only; it is NOT a certified reachability/clearance zone.
    keep=(p[:,0]>.20)&(p[:,0]<1.0)&(abs(p[:,1])<.55)&(p[:,2]>-.85)&(p[:,2]<.15);p=p[keep]
    bars=[fit_bar(p[p[:,1]*side>.1],42+i) for i,side in enumerate([1,-1])]
    if abs(bars[0]['axis']@bars[1]['axis'])<.985:raise Rejected('Observed handle bars are not parallel')
    axis=bars[0]['axis']+bars[1]['axis'];axis/=np.linalg.norm(axis);delta=bars[0]['center']-bars[1]['center'];separation=delta-(delta@axis)*axis;distance=np.linalg.norm(separation)
    if not .415<distance<.515:raise Rejected('Handle separation does not match measured cart')
    intervals=[np.sort(b['endpoints']@axis) for b in bars];low=max(i[0] for i in intervals)+.025;high=min(i[1] for i in intervals)-.025
    if high-low<.08:raise Rejected('No shared observed grasp span')
    along=(low+high)/2;targets=[b['center']+(along-b['center']@axis)*axis for b in bars];normal=np.cross(axis,separation);normal/=np.linalg.norm(normal)
    if normal[2]<0:normal=-normal
    return {'stamp':float(stamp),'frame':'SJ4Z','kind':'parallel_blue_bar_proposals','handles':[p.tolist() for p in targets],'observed_grasp_segments':[[ (b['center']+(bound-b['center']@axis)*axis).tolist() for bound in [low,high]] for b in bars],'bar_axis':axis.tolist(),'approach_normal':normal.tolist(),'separation_m':float(distance),'support':[{'points':b['points'],'span_m':b['span_m'],'rms_m':b['rms_m']} for b in bars],'hardware_executable':False,'requires':['temporal stability','empirical alignment validation','TCP/IK and whole-body collision checks','measured grasp contact']}
