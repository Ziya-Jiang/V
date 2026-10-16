import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from cart_grap_v1 import HandleMarker,make_marker,dictionary,select_reference_sample

class MarkerTests(unittest.TestCase):
    def image(self):
        image=np.full((720,1280,3),255,np.uint8)
        marker=cv2.aruco.generateImageMarker(dictionary(),0,240)
        image[240:480,520:760]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
        return image
    def test_print_size_and_decode(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'marker';meta=make_marker(out,0,40,10)
            self.assertEqual(meta['black_square_mm'],40)
            self.assertEqual(meta['print_page_mm'],60)
            self.assertIn('width="60mm"',(out/'marker.svg').read_text())
            result=HandleMarker().detect(cv2.imread(str(out/'marker_preview.png')))
            self.assertTrue(result['tracking_valid'])
            self.assertFalse(result['control_ready'])
    def test_perspective_corner_identity_and_center(self):
        source=np.float32([[520,240],[759,240],[759,479],[520,479]])
        target=np.float32([[470,220],[780,260],[730,510],[500,465]])
        H=cv2.getPerspectiveTransform(source,target)
        im=cv2.warpPerspective(self.image(),H,(1280,720),borderValue=(255,255,255))
        result=HandleMarker().detect(im,12.3)
        self.assertTrue(result['tracking_valid'],result)
        self.assertLess(np.max(np.linalg.norm(np.asarray(result['corners_px'])-target,axis=1)),1.5)
        truth=cv2.perspectiveTransform(np.float32([[[639.5,359.5]]]),H)[0,0]
        self.assertLess(np.linalg.norm(np.asarray(result['center_px'])-truth),1)
    def test_loss_and_same_id_reacquisition(self):
        detector=HandleMarker();first=detector.detect(self.image())
        lost=detector.detect(np.zeros((720,1280,3),np.uint8))
        self.assertFalse(lost['tracking_valid']);self.assertIsNone(lost['corners_px'])
        self.assertTrue(detector.detect(self.image())['tracking_valid'])
        self.assertEqual(first['marker_id'],0)
    def test_other_id_or_duplicate_rejected(self):
        image=self.image()
        self.assertFalse(HandleMarker(1).detect(image)['tracking_valid'])
        image[240:480,120:360]=image[240:480,520:760]
        self.assertEqual(HandleMarker().detect(image)['reason'],'duplicate_target_id')
    def test_partial_marker_not_accepted(self):
        image=self.image();image[240:350,520:630]=255
        self.assertFalse(HandleMarker().detect(image)['tracking_valid'])
    def test_pose_units_mm(self):
        K=np.array([[720.,0,639.5],[0,720.,359.5],[0,0,1]])
        r=HandleMarker(0,40).detect(self.image(),k=K)
        self.assertAlmostEqual(r['marker_camera_translation_mm'][2],120.,delta=2.)
    def test_invalid_id(self):
        with self.assertRaises(ValueError):HandleMarker(50)
    def test_narrow_handle_marker(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'narrow';meta=make_marker(out)
            self.assertEqual(meta['black_square_mm'],20)
            self.assertEqual(meta['white_margin_mm'],4)
            self.assertEqual(meta['print_page_mm'],28)
            self.assertTrue(HandleMarker().detect(cv2.imread(str(out/'marker_preview.png')))['tracking_valid'])
            self.assertEqual(HandleMarker().marker_mm,20)
    def test_reference_is_real_sample_and_rejects_motion(self):
        samples=[dict(marker=dict(marker_id=0,tracking_valid=True,s_px=[100.+i*.1]*8),
                      pose=dict(q=[500.,0.,0.],angles=[0.,0.,0.])) for i in range(8)]
        self.assertIn(select_reference_sample(samples),(3,4))
        samples[-1]['pose']['q'][0]+=1
        with self.assertRaisesRegex(ValueError,'TCP moved'):select_reference_sample(samples)

if __name__=='__main__':unittest.main()
