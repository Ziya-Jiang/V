"""Offline geometry prototype, deliberately NOT connected to V3 motion.

All translations/depths are METRES. T_a_b maps coordinates from b to a.
Head inputs are raw/aligned pixels with the matching K,D; wrist output uses
the existing undistorted_K convention. Real use requires verified hand-eye,
pose/frame timestamps and held-out reprojection validation. This prototype
does not estimate a hand-eye transform or certify object identity by itself.
"""
import cv2
import numpy as np


def rigid(value,name):
    t=np.asarray(value,float)
    if (t.shape!=(4,4) or not np.isfinite(t).all() or
        not np.allclose(t[3],[0,0,0,1],atol=1e-8) or
        not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-5) or
        abs(np.linalg.det(t[:3,:3])-1)>1e-5 or np.linalg.norm(t[:3,3])>10):
        raise ValueError(f'{name}: expected rigid transform with translation in metres')
    return t


def intrinsics(value):
    k=np.asarray(value,float)
    if (k.shape!=(3,3) or not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0 or
        not np.allclose(k[2],[0,0,1]) or abs(k[0,1])+abs(k[1,0])>1e-8):
        raise ValueError('Invalid camera matrix')
    return k


def project_head_samples(pixels,depth_m,k_head,d_head,t_base_head,t_base_tool,t_tool_wrist,k_wrist):
    if t_tool_wrist is None:raise ValueError('Verified wrist-to-tool hand-eye transform is missing')
    uv=np.asarray(pixels,float);z=np.asarray(depth_m,float)
    if uv.ndim!=2 or uv.shape[1]!=2 or z.shape!=(len(uv),):raise ValueError('Invalid RGB-D sample shapes')
    if not np.isfinite(uv).all() or not np.isfinite(z).all() or np.any(z<=0) or np.any(z>10):
        raise ValueError('Samples need valid positive depth in metres')
    kh,kw=intrinsics(k_head),intrinsics(k_wrist)
    d=np.asarray(d_head,float).reshape(-1)
    if len(d) not in (4,5,8,12,14) or not np.isfinite(d).all():raise ValueError('Invalid head distortion')
    tbh=rigid(t_base_head,'T_base_head');tbt=rigid(t_base_tool,'T_base_tool')
    ttw=rigid(t_tool_wrist,'T_tool_wrist')
    # p_wrist = inverse(T_base_tool T_tool_wrist) T_base_head p_head
    twh=np.linalg.inv(tbt@ttw)@tbh
    rays=cv2.undistortPoints(uv.reshape(-1,1,2),kh,d).reshape(-1,2)
    head=np.column_stack((rays,np.ones(len(rays))))*z[:,None]
    wrist=head@twh[:3,:3].T+twh[:3,3]
    valid=wrist[:,2]>1e-5
    image=np.full((len(uv),2),np.nan)
    projected=wrist[valid]@kw.T
    image[valid]=projected[:,:2]/projected[:,2,None]
    return image,valid


def associate_projected_samples(projected_sets,wrist_masks,*,uncertainty_radius_px,
                                min_samples=20,min_coverage=.8,min_margin=.2):
    """Candidate association only. Radius must come from measured uncertainty.

    Many-to-one or overlapping explanations are rejected. A row represents a
    known head object and a column a wrist segmentation instance. Point cloud
    visibility/occlusion and scene motion still need to be checked by a caller.
    """
    if (type(uncertainty_radius_px) is not int or not 0<=uncertainty_radius_px<=50 or
        min_samples<4 or not 0<min_coverage<=1 or not 0<min_margin<=1):
        raise ValueError('Invalid association thresholds')
    scores=np.zeros((len(projected_sets),len(wrist_masks)))
    if not wrist_masks:return tuple(None for _ in projected_sets),scores
    shape=np.shape(wrist_masks[0])
    if len(shape)!=2 or any(np.shape(m)!=shape for m in wrist_masks):raise ValueError('Wrist mask shapes differ')
    radius=uncertainty_radius_px
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius+1,2*radius+1))
    masks=[cv2.dilate(np.asarray(m,np.uint8),kernel).astype(bool) for m in wrist_masks]
    for i,points in enumerate(projected_sets):
        points=np.asarray(points,float)
        if points.ndim!=2 or points.shape[1]!=2:raise ValueError('Invalid projected sample shape')
        points=points[np.isfinite(points).all(axis=1)]
        points=points[(points[:,0]>=0)&(points[:,0]<shape[1]-.5)&
                      (points[:,1]>=0)&(points[:,1]<shape[0]-.5)]
        p=np.unique(np.rint(points).astype(int),axis=0)
        if len(p)<min_samples:continue
        for j,mask in enumerate(masks):scores[i,j]=mask[p[:,1],p[:,0]].mean()
    mapping=[None]*len(scores)
    for i,row in enumerate(scores):
        j=int(np.argmax(row));alternatives=np.r_[np.delete(row,j),np.delete(scores[:,j],i)]
        runner=float(alternatives.max()) if len(alternatives) else 0.
        if row[j]>=min_coverage and row[j]-runner>=min_margin:mapping[i]=j
    return tuple(mapping),scores
