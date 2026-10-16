"""Offline V4.1 automatic semantic initialization/recovery and pad features."""
import copy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from book_top_tilt_selection import decode, build_payload, SemanticSelector
from book_top_tilt_perception import FaceTracker, FingerTracker, TrackingLost, observed_pad_bias
from book_top_tilt_geometry import FingerPad
from cart_book_grasp_v4_1 import Vision, recovery_consistent, main
from test_cart_book_grasp_v4_1 import configuration, observation
from book_top_tilt_servo import ContinuousTiltController


SHAPE=(240,320,3)
QUAD=np.float32([[100,50],[220,50],[220,210],[100,210]])
K=np.array([[300.,0,160],[0,300,120],[0,0,1.]])

def response(scene='scene'):
    q=(QUAD/np.array([319,239])*1000).tolist()
    fq=(np.array([[125,20],[195,20],[195,43],[125,43]])/np.array([319,239])*1000).tolist()
    return dict(scene_id=scene,status='selected',confidence=.9,reason='synthetic test only',
        book_quad=q,top_visible=True,support_visible=True,same_book=True,
        finger_visible=True,same_finger=True,finger_quad=fq)


class SelectionTests(unittest.TestCase):
    def test_semantics_not_clipped_spine_box(self):
        result=decode(response(),'scene',SHAPE,.7,True,True)
        np.testing.assert_allclose(result['quad'],QUAD,atol=1e-5)

    def test_wrong_scene_rejected(self):
        with self.assertRaisesRegex(ValueError,'reliable'):
            decode(response('old'),'new',SHAPE,.7)

    def test_wrong_book_or_finger_rejected_on_recovery(self):
        for key in ('same_book','same_finger'):
            r=response();r[key]=False
            with self.assertRaisesRegex(ValueError,'same physical'):
                decode(r,'scene',SHAPE,.7,True,True)

    def test_hidden_support_not_invented(self):
        r=response();r['support_visible']=False
        with self.assertRaisesRegex(ValueError,'support edge'):
            decode(r,'scene',SHAPE,.7)

    def test_invalid_polygon_and_confidence_rejected(self):
        for changes in ({'confidence':float('nan')},{'confidence':True},
                        {'confidence':.6},{'book_quad':[[0,0]]*4},
                        {'book_quad':[[1001,20]]*4}):
            r=response();r.update(changes)
            with self.assertRaises(ValueError):decode(r,'scene',SHAPE,.7)

    def test_finger_visibility_required_only_for_execution(self):
        r=response();r['finger_visible']=False
        self.assertIsNone(decode(r,'scene',SHAPE,.7)['finger_quad'])
        with self.assertRaisesRegex(ValueError,'finger'):
            decode(r,'scene',SHAPE,.7,require_finger=True)

    def test_reference_in_request_and_no_robot_actions(self):
        rgb=np.zeros(SHAPE,np.uint8)
        p=build_payload(rgb,'s',{'model':'test'},'target',dict(rgb=rgb,quad=QUAD))
        self.assertFalse(p['store'])
        self.assertEqual(len([v for v in p['input'][0]['content'] if v['type']=='input_image']),2)
        self.assertIn('same_book',p['text']['format']['schema']['required'])
        self.assertIn('no robot pose',p['instructions'])

    def test_client_checkpoints_and_failure_artifact(self):
        calls=[]
        class Client:
            def select(self,payload,timeout,checkpoint):
                checkpoint();r=response(payload['text']['format']['schema']['properties']['scene_id']['enum'][0])
                r['support_visible']=False
                return r
        sel=SemanticSelector('book',client=Client(),config=dict(enabled=True,model='test',timeout_seconds=1,min_confidence=.7))
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):sel.select(np.zeros(SHAPE,np.uint8),folder,checkpoint=lambda:calls.append(1))
            record=json.loads((Path(folder)/'selection.json').read_text())
            self.assertEqual(record['status'],'rejected')
            self.assertTrue((Path(folder)/'input.jpg').exists())
        self.assertGreaterEqual(len(calls),2)


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.random.default_rng(44).integers(0,256,SHAPE,dtype=np.uint8)

    def test_local_template_recovers_without_changing_contact_anchor(self):
        t=FaceTracker(self.rgb,QUAD)
        changed=cv2.warpAffine(self.rgb,np.float32([[1,0,3],[0,1,2]]),(320,240))
        with patch.object(t,'_lk',side_effect=TrackingLost('occlusion')):
            result=t.update(changed)
        self.assertEqual(t.method,'sift_same_template')
        np.testing.assert_allclose(result,QUAD+[3,2],atol=.4)
        np.testing.assert_allclose(t.edge_midpoints,t.reference_midpoints+[3,2],atol=.4)

    def test_unrelated_texture_does_not_recover(self):
        t=FaceTracker(self.rgb,QUAD)
        with self.assertRaises(TrackingLost):
            t.update(np.random.default_rng(987).integers(0,256,SHAPE,dtype=np.uint8))
        np.testing.assert_array_equal(t.quad,QUAD)

    def test_central_text_alone_cannot_extrapolate_support_edge(self):
        t=FaceTracker(self.rgb,QUAD)
        xx,yy=np.meshgrid(np.linspace(110,210,5),np.linspace(100,160,4))
        src=np.float32(np.c_[xx.ravel(),yy.ravel()]).reshape(-1,1,2)
        with self.assertRaisesRegex(TrackingLost,'nearby feature support'):
            t._fit(src,src+np.float32([1,0]))

    def test_book_and_finger_have_independent_motion(self):
        bq=np.float32([[30,80],[140,80],[140,220],[30,220]])
        fq=np.float32([[190,35],[290,35],[290,65],[190,65]])
        book=FaceTracker(self.rgb,bq);finger=FingerTracker(self.rgb,fq)
        moving=self.rgb.copy()
        roi=self.rgb[20:75,175:305].copy()
        moving[20:75,175:305]=cv2.warpAffine(roi,np.float32([[1,0,2],[0,1,0]]),(130,55))
        book.update(moving);finger.update(moving)
        np.testing.assert_allclose(book.quad,bq,atol=.3)
        np.testing.assert_allclose(finger.tracker.quad,fq+[2,0],atol=.4)

    def test_finger_reference_bound_to_visible_calibrated_side(self):
        q=np.float32([[130,95],[190,95],[190,125],[130,125]])
        t=FingerTracker(self.rgb,q)
        pad=FingerPad([0,0,0],[0,0,-1],[1,0,0],80,20,2)
        t.bind(np.array([0,0,600.]),np.eye(3),pad,K,np.zeros(5),np.eye(4))
        obs=t.update(self.rgb)
        bias=observed_pad_bias(obs,np.array([0,0,600.]),np.eye(3),K,np.zeros(5),np.eye(4))
        np.testing.assert_allclose(bias,0,atol=.05)
        obs['finger_reference_uv']=obs['finger_reference_uv']+[1,0]
        np.testing.assert_allclose(observed_pad_bias(obs,np.array([0,0,600.]),np.eye(3),K,np.zeros(5),np.eye(4)),[2,0,0],atol=.05)
        obs['finger_reference_uv']+=np.array([5,0])
        with self.assertRaisesRegex(ValueError,'disagreement'):
            observed_pad_bias(obs,np.array([0,0,600.]),np.eye(3),K,np.zeros(5),np.eye(4))

    def test_semantic_recovery_cannot_jump_to_neighbor(self):
        c=configuration();a=observation();a['corners_base_mm']=np.array([[590,0,200],[610,0,200],[610,0,0],[590,0,0]])
        b=copy.deepcopy(a);recovery_consistent(a,b,c)
        b['top_base_mm']+=np.array([20,0,0])
        with self.assertRaisesRegex(ValueError,'jumped'):recovery_consistent(a,b,c)

    def test_actual_observed_pad_bias_changes_control_direction(self):
        c=configuration();c['pad_visual_required']=True
        tcp=np.array([600.,0,158.]);ctrl=ContinuousTiltController(c,observation(),tcp,np.eye(3),np.eye(4))
        obs=observation();obs.update(K=K,D=np.zeros(5),finger_reference_tcp_mm=np.array([0.,0,50.]))
        point=tcp+obs['finger_reference_tcp_mm']
        uv=cv2.projectPoints(point[None],np.zeros(3),np.zeros(3),K,np.zeros(5))[0][0,0]
        obs['finger_reference_uv']=uv+[1,0]
        d=ctrl.decide_stream(obs,tcp,np.eye(3),tcp,np.eye(3))
        self.assertEqual(d['phase'],'ALIGN')
        self.assertLess(d['target'][0],tcp[0])
        self.assertGreater(d['pad_visual_bias_mm'][0],.6)

    def test_required_finger_cannot_silently_fall_back_to_fk(self):
        c=configuration();c['pad_visual_required']=True
        tcp=np.array([600.,0,158.]);ctrl=ContinuousTiltController(c,observation(),tcp,np.eye(3),np.eye(4))
        with self.assertRaisesRegex(ValueError,'finger landmark'):
            ctrl.decide_stream(observation(),tcp,np.eye(3),tcp,np.eye(3))


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.random.default_rng(2).integers(0,256,SHAPE,dtype=np.uint8)
        self.camera=SimpleNamespace(K=K,D=np.zeros(5),c2w=np.eye(4))
        def capture():
            self.camera.captured_unix_s=time.time()
            return self.rgb.copy(),np.full(SHAPE[:2],600.,np.float32)
        self.camera.capture_image=capture

    def test_automatic_selection_new_frame_tracking_and_named_artifacts(self):
        selector=SimpleNamespace(select=lambda *a,**k:dict(quad=QUAD,finger_quad=None,method='synthetic'))
        with tempfile.TemporaryDirectory() as folder:
            v=Vision.automatic(self.camera,selector,configuration(),folder)
            obs=v.stable(min_stamp=time.time())
            self.assertGreater(obs['points'],12)
            v.save('final')
            self.assertTrue((Path(folder)/'final.jpg').exists())
            self.assertFalse((Path(folder)/'support.jpg').exists())
            self.assertTrue((Path(folder)/'first_detection.jpg').exists())

    def test_failed_automatic_initialization_always_saves_first_image(self):
        def reject(*a,**k):raise ValueError('no upper edge')
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError,'upper edge'):
                Vision.automatic(self.camera,SimpleNamespace(select=reject),configuration(),folder)
            self.assertTrue((Path(folder)/'first_detection.jpg').exists())
            self.assertEqual(json.loads((Path(folder)/'first_detection.json').read_text())['status'],'failed')

    def test_cli_default_auto_selection_no_silent_old_reference(self):
        calls=[]
        with patch('cart_book_grasp_v4_1.run',side_effect=lambda args,c:calls.append(args)):
            self.assertEqual(main(['--frames','2']),0)
            self.assertIsNone(calls[0].selection)
            self.assertEqual(main(['--auto-select']),0)
            self.assertTrue(calls[1].auto_select)




class EntryIntegrationTests(unittest.TestCase):
    def test_default_entry_auto_selects_and_tracks_without_robot_connection(self):
        import sys
        from cart_book_grasp_v4_1 import run
        rgb=np.random.default_rng(101).integers(0,256,SHAPE,dtype=np.uint8)
        cam=SimpleNamespace(K=K,D=np.zeros(5),c2w=np.eye(4),close=lambda:None)
        def capture():
            cam.captured_unix_s=time.time()
            return rgb.copy(),np.full(SHAPE[:2],600.,np.float32)
        cam.capture_image=capture
        def select(*args,**kwargs):return dict(quad=QUAD,finger_quad=None,method='mock_semantics')
        args=SimpleNamespace(execute=False,selection=None,select=False,auto_select=False,target='book',frames=2)
        with tempfile.TemporaryDirectory() as folder:
            args.output=Path(folder)
            with patch.dict(sys.modules,{'cart_grasp_alignment':SimpleNamespace(open_head_camera=lambda:cam)}), patch(
                    'book_top_landing_vision.LandingSelector.select',side_effect=select), patch(
                    'cart_book_grasp_v2_gpt.GPTBookClient._request',side_effect=AssertionError('Offline test forbids API')), patch('builtins.print'):
                run(args,configuration())
            events=[json.loads(line) for line in (Path(folder)/'events.jsonl').read_text().splitlines()]
            self.assertEqual(events[0]['event'],'head_offsets')
            self.assertTrue((Path(folder)/'head/final.jpg').exists())
            self.assertTrue((Path(folder)/'head/first_detection.jpg').exists())

if __name__ == '__main__':
    unittest.main()
