import unittest
import numpy as np
import cv2
from research_head_wrist_geometry import project_head_samples,associate_projected_samples


class GeometryPrototypeTests(unittest.TestCase):
    def setUp(self):
        self.k=np.array([[100.,0,50],[0,100,50],[0,0,1]])
        self.i=np.eye(4)

    def test_transform_direction_and_metres(self):
        head=self.i.copy();head[0,3]=.1
        tool=self.i.copy();tool[0,3]=.05
        wrist=self.i.copy();wrist[0,3]=.02
        uv,valid=project_head_samples([[50,50]],[1.],self.k,np.zeros(5),head,tool,wrist,self.k)
        np.testing.assert_allclose(uv,[[53,50]],atol=1e-6);self.assertTrue(valid[0])

    def test_head_distortion_is_removed(self):
        point=np.array([[.1,.2,1.]])
        d=np.array([.1,-.02,.001,.002,0])
        pixels,_=cv2.projectPoints(point,np.zeros(3),np.zeros(3),self.k,d)
        uv,_=project_head_samples(pixels.reshape(-1,2),[1],self.k,d,self.i,self.i,self.i,self.k)
        np.testing.assert_allclose(uv,[[60,70]],atol=1e-5)

    def test_missing_handeye_and_wrong_units_rejected(self):
        for handeye,z in [(None,1),(self.i,1000)]:
            with self.assertRaises(ValueError):
                project_head_samples([[50,50]],[z],self.k,np.zeros(5),self.i,self.i,handeye,self.k)

    def test_behind_camera_is_not_projected(self):
        wrist=self.i.copy();wrist[2,3]=2
        uv,valid=project_head_samples([[50,50]],[1],self.k,np.zeros(5),self.i,self.i,wrist,self.k)
        self.assertFalse(valid[0]);self.assertTrue(np.isnan(uv).all())

    def test_distinct_books_match_and_overlapping_explanations_reject(self):
        a=np.zeros((100,100),np.uint8);a[10:40,10:30]=1
        b=np.zeros_like(a);b[10:40,60:80]=1
        pa=np.array([(x,y) for x in range(15,25) for y in range(15,30)],float)
        pb=pa+[50,0]
        m,_=associate_projected_samples([pa,pb],[a,b],uncertainty_radius_px=2)
        self.assertEqual(m,(0,1))
        m,_=associate_projected_samples([pa],[a,a],uncertainty_radius_px=2)
        self.assertEqual(m,(None,))
        m,_=associate_projected_samples([pa,pa],[a,b],uncertainty_radius_px=2)
        self.assertEqual(m,(None,None))

    def test_invalid_and_duplicate_samples_cannot_manufacture_evidence(self):
        mask=np.ones((100,100),np.uint8)
        m,_=associate_projected_samples([[[np.nan,0],[-5,2]]+[[20,20]]*100],[mask],uncertainty_radius_px=1)
        self.assertEqual(m,(None,))


if __name__=='__main__':unittest.main()
