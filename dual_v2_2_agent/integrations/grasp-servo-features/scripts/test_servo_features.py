"""Behavioral tests with independently known image transformations."""
import json
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from servo_features import Tracker, initialize, project, compare_results


class TrackerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        rng = np.random.default_rng(72)
        cls.image = np.full((480,640,3),170,np.uint8)
        for _ in range(300):
            p = tuple(rng.integers([155,105],[485,375]))
            color = tuple(int(x) for x in rng.integers(0,255,3))
            cv2.circle(cls.image,p,int(rng.integers(2,9)),color,-1)
        cv2.putText(cls.image,'SERVO 1972',(180,230),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
        cv2.imwrite(str(cls.root/'reference.png'),cls.image)
        initialize(cls.root/'reference.png',{'patch_polygon':[[145,95],[495,95],[495,385],[145,385]],'point_count':8},cls.root/'model')

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_identity_and_transforms(self):
        tracker = Tracker(self.root/'model')
        ids = [f'P{i+1}' for i in range(len(tracker.anchors))]
        methods = set()
        for i in range(12):
            H = np.vstack([cv2.getRotationMatrix2D((320,240),i*.6,1+i*.007),[0,0,1]])
            H[0,2] += i*.8
            H[2,0] = i*0.000005
            frame = cv2.warpPerspective(self.image,H,(640,480))
            frame = np.clip(frame.astype(float)*.93+6,0,255).astype(np.uint8)
            r = tracker.process(frame,timestamp=i/30)
            self.assertTrue(r['tracking_valid'],r)
            self.assertEqual([f['id'] for f in r['features']],ids)
            truth = project(tracker.anchors,H)
            errors = [np.linalg.norm(np.array(f['xy'])-p) for f,p in zip(r['features'],truth) if f['valid']]
            self.assertLess(max(errors),2.0)
            methods.add(r['method'])
            json.dumps(r,allow_nan=False)
        self.assertEqual(methods,{'lk','sift'})

    def test_occlusion_invalidates_anchor(self):
        tracker = Tracker(self.root/'model')
        tracker.process(self.image)
        frame = self.image.copy()
        x,y = np.round(tracker.anchors[0]).astype(int)
        cv2.rectangle(frame,(x-14,y-14),(x+14,y+14),(170,170,170),-1)
        result = tracker.process(frame)
        self.assertFalse(result['features'][0]['valid'])
        self.assertIsNone(result['features'][0]['xy'])

    def test_loss_then_recovery(self):
        tracker = Tracker(self.root/'model')
        first = tracker.process(self.image)
        lost = tracker.process(np.full_like(self.image,170))
        self.assertEqual(lost['status'],'lost')
        self.assertTrue(all(x is None for x in lost['s_px']))
        recovered = tracker.process(self.image)
        self.assertEqual(first['template_id'],recovered['template_id'])
        self.assertEqual(recovered['status'],'ok')
        self.assertEqual(recovered['method'],'sift')

    def test_unrelated_texture_rejected(self):
        other = np.random.default_rng(29).integers(0,256,self.image.shape,dtype=np.uint8)
        result = Tracker(self.root/'model').process(other)
        self.assertEqual(result['status'],'lost')

    def test_synthetic_never_certified(self):
        r = Tracker(self.root/'model').process(self.image)
        pair = compare_results(r,r,'synthetic')
        self.assertFalse(pair['control_ready'])
        self.assertFalse(pair['target_calibrated'])
        self.assertLess(max(abs(x) for x in pair['e_px'] if x is not None),1e-6)
        changed = dict(r,template_id='another')
        with self.assertRaises(ValueError):
            compare_results(r,changed,'real')
        changed = dict(r,image_size=[320,240])
        mismatch = compare_results(r,changed,'real')
        self.assertTrue(all(x is None for x in mismatch['e_px']))
        self.assertFalse(any(mismatch['error_valid_mask']))

    def test_invalid_or_textureless_seed(self):
        blank = self.root/'blank.png'
        cv2.imwrite(str(blank),np.zeros_like(self.image))
        with self.assertRaisesRegex(ValueError,'insufficient_features'):
            initialize(blank,{'patch_polygon':[[100,100],[400,100],[400,350],[100,350]]},self.root/'blank_model')
        with self.assertRaises(ValueError):
            initialize(blank,{'patch_polygon':[[100,100],[900,100],[900,350],[100,350]]},self.root/'bad_model')

    def test_clustered_inliers_cannot_validate_distant_anchor(self):
        tracker = Tracker(self.root/'model')
        anchors = tracker.anchors.copy()
        distant = int(np.argmax(anchors[:,0]))
        # Reference points cover only the left-hand region. NCC is deliberately
        # perfect, isolating the geometric-support requirement from photometry.
        hull = np.float32([[145,95],[300,95],[300,385],[145,385]])
        a = np.concatenate([hull,np.float32([[160,120],[180,200],[200,300],[280,150],[260,330],[290,240]])])
        tracker._detect = lambda frame:(a,a)
        tracker._correlations = lambda frame,H:[1.]*len(anchors)
        result = tracker.process(self.image)
        self.assertGreater(anchors[distant,0],301)
        self.assertFalse(result['features'][distant]['valid'])
        self.assertIsNone(result['features'][distant]['xy'])


if __name__=='__main__':
    unittest.main(verbosity=2)
