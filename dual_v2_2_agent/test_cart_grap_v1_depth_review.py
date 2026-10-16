import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import cart_grap_v1 as cycle
import cart_grap_v1_perception as perception
from cart_grap_v1_depth_review import validate_review,review_payload

Q=[[30,30],[50,30],[50,50],[30,50]]

class DepthReviewTests(unittest.TestCase):
    def run_case(self,mode='recover'):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        out=Path(self.folder.name);rng=np.random.default_rng(2)
        rgb=rng.integers(0,255,(200,200,3),dtype=np.uint8)
        camera=SimpleNamespace(K=np.array([[100.,0,40],[0,100.,40],[0,0,1]]),D=np.zeros(5),
            c2w=np.eye(4),w=200,h=200,camparas_file='test',captured_unix_s=99.8,close=Mock(),count=0)
        def capture():
            camera.count+=1;camera.captured_unix_s+=.02
            image=rgb.copy();depth=np.full((200,200),680.)
            if mode!='normal' and (camera.count<=3 or mode=='still_bad'):depth[:,40:]=900.
            if mode=='scene_changed' and camera.count>=3:image=np.roll(image,10,axis=1)
            return image,depth
        camera.capture_image=capture;client=Mock();calls=[]
        def select(request,*args):
            calls.append(request)
            scene=json.loads(request['input'][0]['content'][0]['text'])['scene_id']
            result=dict(scene_id=scene,status='selected',confidence=.95,reason='测试复核',marker_quad=Q)
            if len(calls)>1:
                if mode=='timeout':raise TimeoutError('API timeout')
                if mode=='robot_moved':raise ValueError('Robot moved while acquiring GPT/head geometry')
                result.update(decision='stop' if mode=='stop' else 'continue',same_target=mode!='stop')
                if mode=='wrong_scene':result['scene_id']='old'
                if mode=='wrong_target':result['marker_quad']=[[100,100],[120,100],[120,120],[100,120]]
            return result
        client.select.side_effect=select
        with patch.object(perception,'open_head_camera',return_value=camera),patch.object(perception.time,'time',return_value=100.):
            result=None;error=None
            try:result=perception.locate(out,cycle.load_config(),client=client)
            except Exception as exc:error=exc
        camera.close.assert_called_once()
        return out,camera,calls,result,error

    def test_continue_requires_three_new_depth_frames(self):
        out,camera,calls,r,error=self.run_case()
        self.assertIsNone(error);self.assertEqual(len(calls),2);self.assertEqual(camera.count,6)
        self.assertTrue(r['gpt_depth_review_used']);self.assertEqual(len(r['samples']),3)
        self.assertTrue(all(x['depth_median_mm']==680 for x in r['samples']))
        self.assertEqual(json.loads((out/'gpt_depth_review/validation.json').read_text())['status'],'accepted')
        self.assertTrue((out/'head_rejected_geometry.npz').exists())
        self.assertEqual(calls[1]['model'],'gpt-6-astra')
        self.assertEqual(calls[1]['text']['format']['name'],'cart_marker_depth_review')

    def test_continue_does_not_override_bad_depth(self):
        out,camera,calls,r,error=self.run_case('still_bad')
        self.assertIsNone(r);self.assertIsInstance(error,perception.DepthRegionError)
        self.assertEqual(len(calls),2)
        self.assertFalse((out/'head_geometry.json').exists())
        self.assertEqual(json.loads((out/'gpt_depth_review/validation.json').read_text())['status'],'rejected')

    def test_stop_timeout_stale_scene_changed_target_or_robot_stop(self):
        for mode in ['stop','timeout','wrong_scene','wrong_target','robot_moved']:
            with self.subTest(mode=mode):
                out,camera,calls,r,error=self.run_case(mode)
                self.assertIsNone(r);self.assertIsNotNone(error)
                self.assertEqual(len(calls),2);self.assertFalse((out/'head_geometry.json').exists())
                self.assertEqual(json.loads((out/'gpt_depth_review/decision.json').read_text())['decision'],'stop')

    def test_scene_movement_cannot_request_override(self):
        out,camera,calls,r,error=self.run_case('scene_changed')
        self.assertIsNone(r);self.assertIsNotNone(error)
        self.assertEqual(len(calls),1);self.assertFalse((out/'head_geometry.json').exists())

    def test_normal_path_makes_no_extra_api_call(self):
        out,camera,calls,r,error=self.run_case('normal')
        self.assertIsNone(error);self.assertEqual(len(calls),1)
        self.assertFalse(r['gpt_depth_review_used']);self.assertEqual(camera.count,4)

    def test_review_confidence_and_schema_evidence(self):
        q=np.float32(Q);c=cycle.load_config()
        result=dict(scene_id='s',status='selected',confidence=.2,reason='',marker_quad=Q,decision='continue',same_target=True)
        with self.assertRaises(ValueError):validate_review(result,(200,200,3),'s',c,q)
        payload,evidence,_=review_payload(np.zeros((200,200,3),np.uint8),np.zeros((200,200)),q,'missing',c,'s')
        self.assertTrue(all(v is None for row in evidence['depth_grid_mm'] for v in row))
        self.assertIn('same_target',payload['text']['format']['schema']['required'])
        self.assertFalse(payload['store'])

if __name__=='__main__':unittest.main()
