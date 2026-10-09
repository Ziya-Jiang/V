"""Occupied RGB-D voxels for rejection screening. Missing depth stays unknown."""
import cv2,numpy as np
from .geometry import Rejected,apply

class DepthObstacles:
    def __init__(self,data,voxel_m=.015):
        import fcl
        self.fcl=fcl;depth=data['depth'];k=data['k'];d=data['d'];t=data['upper_T_camera'];v,u=np.nonzero(np.isfinite(depth)&(depth>.1)&(depth<3.0));u=u[::3];v=v[::3];r=cv2.undistortPoints(np.column_stack([u,v]).astype(float).reshape(-1,1,2),k,d).reshape(-1,2);z=depth[v,u];points=apply(t,np.column_stack([r[:,0]*z,r[:,1]*z,z]));keep=(points[:,0]>-.2)&(points[:,0]<1.2)&(abs(points[:,1])<.8)&(points[:,2]>-1.3)&(points[:,2]<.4);points=points[keep]
        if len(points)<200:raise Rejected('No sufficient observed environment geometry')
        self.centers=(np.unique(np.floor(points/voxel_m).astype(np.int64),axis=0)+.5)*voxel_m;self.geometry=fcl.Box(voxel_m,voxel_m,voxel_m);self.objects=[fcl.CollisionObject(self.geometry,fcl.Transform(p)) for p in self.centers];self.manager=fcl.DynamicAABBTreeCollisionManager();self.manager.registerObjects(self.objects);self.manager.setup();self.stamp=float(data['stamp_s']);self.voxel_m=voxel_m
    def distance(self,obj):
        data=self.fcl.DistanceData(request=self.fcl.DistanceRequest());self.manager.distance(obj,data,self.fcl.defaultDistanceCallback);return float(data.result.min_distance)
