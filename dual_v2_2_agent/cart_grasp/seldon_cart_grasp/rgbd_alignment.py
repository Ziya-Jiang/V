"""CPU depth-to-color registration with explicit units and z-buffer occlusion."""
import cv2
import numpy as np
from .geometry import finite,transform,Rejected

def align_depth(depth,depth_scale,depth_k,depth_d,color_k,color_d,color_T_depth,width,height):
    raw=np.asarray(depth);dk=finite(depth_k,(3,3));ck=finite(color_k,(3,3));t=transform(color_T_depth)
    if raw.ndim!=2 or not 0<depth_scale<=.01 or width<=0 or height<=0:raise Rejected('Depth shape, scale or target size invalid')
    if min(dk[0,0],dk[1,1],ck[0,0],ck[1,1])<=0:raise Rejected('Invalid focal length')
    dd=finite(depth_d);cd=finite(color_d)
    if dd.shape not in [(4,),(5,),(8,)] or cd.shape not in [(4,),(5,),(8,)]:raise Rejected('Unsupported distortion coefficients')
    metres=raw.astype(np.float32)*depth_scale;v,u=np.nonzero(np.isfinite(metres)&(metres>.10)&(metres<5.))
    output=np.full((height,width),np.inf,np.float32)
    if not len(u):return np.zeros((height,width),np.float32)
    rays=cv2.undistortPoints(np.column_stack([u,v]).astype(float).reshape(-1,1,2),dk,dd).reshape(-1,2);z=metres[v,u];points=np.column_stack([rays[:,0]*z,rays[:,1]*z,z])@t[:3,:3].T+t[:3,3];points=points[points[:,2]>.1]
    if not len(points):return np.zeros((height,width),np.float32)
    pixels,_=cv2.projectPoints(points,np.zeros(3),np.zeros(3),ck,cd);pixels=pixels.reshape(-1,2);valid=np.isfinite(pixels).all(axis=1)&(pixels[:,0]>=-.5)&(pixels[:,0]<width-.5)&(pixels[:,1]>=-.5)&(pixels[:,1]<height-.5);pixels=np.rint(pixels[valid]).astype(int);points=points[valid];np.minimum.at(output,(pixels[:,1],pixels[:,0]),points[:,2]);output[~np.isfinite(output)]=0
    return output

def intrinsic_matrix(profile):
    i=profile.get_intrinsic();return np.array([[i.fx,0,i.cx],[0,i.fy,i.cy],[0,0,1.]],float)

def distortion_vector(profile):
    d=profile.get_distortion();model=str(d.model)
    if model.endswith('BROWN_CONRADY_K6'):return np.array([d.k1,d.k2,d.p1,d.p2,d.k3,d.k4,d.k5,d.k6],float)
    if model.endswith('BROWN_CONRADY'):return np.array([d.k1,d.k2,d.p1,d.p2,d.k3],float)
    raise Rejected('Unsupported SDK distortion: '+model)
