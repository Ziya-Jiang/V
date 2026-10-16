import unittest
import numpy as np
import cv2
from head_workspace_ar import project_base,depth_flags

class ProjectionChecks(unittest.TestCase):
    def setUp(self):
        self.c={'T_dummy_world_camera_mm':np.eye(4).tolist(),
                'K':[[100,0,50],[0,100,50],[0,0,1]],'D':[.1,-.02,.001,-.001,.003]}

    def test_extrinsic_direction_and_distortion(self):
        T=np.eye(4);T[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];T[:3,3]=[100,-30,500]
        self.c['T_dummy_world_camera_mm']=T.tolist()
        pc=np.array([[100,80,1000],[0,0,500]],float)
        pb=pc@T[:3,:3].T+T[:3,3]
        uv,z,valid=project_base(pb,self.c)
        expected=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),np.array(self.c['K'],float),np.array(self.c['D']))[0].reshape(-1,2)
        np.testing.assert_allclose(uv,expected,atol=1e-10);np.testing.assert_allclose(z,pc[:,2]);self.assertTrue(valid.all())
        rays=cv2.undistortPoints(uv.reshape(-1,1,2),np.array(self.c['K'],float),np.array(self.c['D'])).reshape(-1,2)
        np.testing.assert_allclose(np.c_[rays,np.ones(len(rays))]*z[:,None],pc,atol=1e-5)

    def test_negative_depth_and_image_bounds(self):
        uv,z,valid=project_base(np.array([[0,0,-500],[1000,0,100],[0,0,1000]],float),self.c)
        flags=depth_flags(uv,z,valid,np.full((100,100),1100.))
        np.testing.assert_array_equal(flags,[-2,-2,0])

    def test_occlusion_unknown_and_margin(self):
        uv=np.array([[20,20],[30,30],[40,40],[50,50]],float)
        depth=np.full((100,100),1000.);depth[40,40]=0
        flags=depth_flags(uv,np.array([900,1030,800,1010]),np.ones(4,bool),depth)
        np.testing.assert_array_equal(flags,[0,1,2,0])

if __name__=='__main__':unittest.main()
