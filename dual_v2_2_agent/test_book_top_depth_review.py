"""Offline GPT review contract, measured depth and observation motion tests."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
import book_top_model_coarse as coarse
from book_top_local_depth import decode_depth_anchors,local_spine_geometry,LocalDepthError
from book_top_observation_motion import ObservationMotionGate


def response(sid='test'):
    return dict(scene_id=sid,status='selected',reason='same book',confidence=.9,
        top_visible=True,same_book=True,full_spine_visible=True,
        spine_quad=[[300,200],[700,200],[700,800],[300,800]],
        book_quad=[[200,100],[800,100],[800,900],[200,900]],
        top_depth_uv=[500,235],center_depth_uv=[500,500],top_center_uv=[500,200],spine_center_uv=[500,500])


def selection():
    r=response();s=coarse.decode_selection(r,'test',(100,100,3),True)
    return decode_depth_anchors(r,s,(100,100,3))


class LocalDepth(unittest.TestCase):
    def test_reported_regions_no_longer_fail_containment_gate(self):
        r=response()
        r['spine_quad']=[[504.7,886.3],[521.9,877.5],[551.6,1000],[527.3,1000]]
        r['book_quad']=[[485.9,743.8],[499.2,740],[551.6,1000],[527.3,1000]]
        decoded=coarse.decode_selection(r,'test',(800,1280,3))
        self.assertEqual(decoded['spine_quad'].shape,(4,2))

    def test_small_nonconvex_region_is_not_rejected_by_old_shape_rules(self):
        r=response();r['spine_quad']=[[400,200],[500,200],[450,250],[400,300]]
        decoded=coarse.decode_selection(r,'test',(100,100,3))
        self.assertEqual(decoded['spine_quad'].shape,(4,2))

    def test_explicit_top_point_not_polygon_top_average_drives_height(self):
        r=response();r['top_center_uv']=[550,150]
        s=coarse.decode_selection(r,'test',(100,100,3),require_landmarks=True)
        # Explicit point can be outside the approximate spine face.
        g=coarse.sample_geometry(None,np.full((100,100),600.),np.array([[100,0,50],[0,100,50],[0,0,1.]]),
                                np.zeros(5),np.eye(4),s)
        np.testing.assert_allclose(g['top_base_mm'],[26.7,-210.9,600.])

    def test_live_response_must_have_finite_semantic_points(self):
        for value in (None,[float('nan'),500],[1001,500]):
            r=response();r['top_center_uv']=value
            with self.assertRaisesRegex(ValueError,'Invalid model landmark'):
                coarse.decode_selection(r,'test',(100,100,3),require_landmarks=True)

    def test_empty_region_is_still_invalid_input(self):
        r=response();r['book_quad']=[[500,500]]*4
        with self.assertRaisesRegex(ValueError,'Empty model region'):coarse.decode_selection(r,'test',(100,100,3))

    def test_upper_edge_with_missing_background_half_retains_real_depth(self):
        s=selection();depth=np.zeros((100,100));depth[20:,:]=600.
        s.pop('top_depth_uv');s.pop('center_depth_uv')
        g=local_spine_geometry(s,depth,np.array([[100,0,50],[0,100,50],[0,0,1.]]),np.zeros(5),np.eye(4))
        self.assertEqual(g['top_base_mm'][2],600.)

    def test_model_anchors_use_measured_depth_without_plane(self):
        s=selection();depth=np.full((100,100),600.)
        k=np.array([[100,0,50],[0,100,50],[0,0,1.]])
        with patch('book_top_tilt_perception.measure_face',side_effect=AssertionError('no plane')):
            g=coarse.sample_geometry(None,depth,k,np.zeros(5),np.eye(4),s)
        np.testing.assert_allclose(g['top_base_mm'],[-3.,-181.2,600.])
        self.assertEqual(g['geometry_method'],'gpt_landmarks_local_rgbd')
        self.assertEqual(g['local_depth']['top']['depth_mm'],600.)

    def test_anchor_far_from_landmark_rejected(self):
        r=response();s=coarse.decode_selection(r,'test',(100,100,3),True)
        r['top_depth_uv']=[500,500]
        with self.assertRaisesRegex(ValueError,'too far'):decode_depth_anchors(r,s,(100,100,3))

    def test_anchor_outside_surface_rejected(self):
        r=response();s=coarse.decode_selection(r,'test',(100,100,3),True);r['center_depth_uv']=[100,500]
        with self.assertRaisesRegex(ValueError,'outside'):decode_depth_anchors(r,s,(100,100,3))

    def test_missing_depth_not_replaced_by_model(self):
        with self.assertRaisesRegex(ValueError,'insufficient measured depth'):
            local_spine_geometry(selection(),np.zeros((100,100)),np.eye(3),np.zeros(5),np.eye(4))

    def test_mixed_local_depth_rejected(self):
        rng=np.random.default_rng(0);depth=rng.uniform(500,800,(100,100))
        with self.assertRaisesRegex(ValueError,'mixed or unstable'):
            local_spine_geometry(selection(),depth,np.eye(3),np.zeros(5),np.eye(4))

    def test_head_entry_mixed_depth_warns_and_returns_measured_geometry(self):
        depth=np.random.default_rng(0).uniform(500,800,(100,100))
        with patch('builtins.print') as output:
            g=coarse.sample_geometry(None,depth,np.eye(3),np.zeros(5),np.eye(4),selection())
        self.assertTrue(output.called)
        for info in g['local_depth'].values():
            self.assertIn('local_depth_MAD_exceeds_6mm',info['quality_warnings'])
            self.assertTrue(500<=info['depth_mm']<=800)

    def test_warning_policy_sparse_depth_keeps_actual_samples(self):
        s=selection();depth=np.zeros((100,100))
        for key in ('top_depth_uv','center_depth_uv'):
            x,y=np.rint(s[key]).astype(int);depth[y,x]=612.
        g=local_spine_geometry(s,depth,np.eye(3),np.zeros(5),np.eye(4),quality_warning_only=True)
        for info in g['local_depth'].values():
            self.assertEqual(info['depth_mm'],612.)
            self.assertIn('fewer_than_9_valid_depth_points',info['quality_warnings'])

    def test_warning_policy_cannot_invent_missing_or_invalid_depth(self):
        for value in (0.,float('nan'),float('inf'),3000.):
            with self.assertRaises(LocalDepthError):
                local_spine_geometry(selection(),np.full((100,100),value),np.eye(3),
                                     np.zeros(5),np.eye(4),quality_warning_only=True)


class RecoveryFlow(unittest.TestCase):
    def run_flow(self,mode='ok'):
        clock=[10.];frames=[0];calls=[];metrics=[0]
        fake_time=NS(monotonic=lambda:clock[0],time=lambda:clock[0]+1000,
                     sleep=lambda dt:clock.__setitem__(0,clock[0]+dt))
        camera=NS(K=np.array([[100,0,50],[0,100,50],[0,0,1.]]),D=np.zeros(5),c2w=np.eye(4))
        def capture():
            frames[0]+=1;clock[0]+=.04;camera.captured_unix_s=clock[0]+1000
            depth=np.full((100,100),600.)
            if mode=='quality_warning':depth=np.random.default_rng(0).uniform(500,800,(100,100))
            if mode=='unstable' and len(calls)==2:depth+=frames[0]*10
            intensity=40 if mode=='image_changed' and calls else 0
            return np.full((100,100,3),intensity,np.uint8),depth
        camera.capture_image=capture
        def model(payload,timeout,guard):
            guard();calls.append(copy.deepcopy(payload));clock[0]+=5
            if mode=='calibration_changed':camera.c2w[0,3]+=1.
            sid=payload['text']['format']['schema']['properties']['scene_id']['enum'][0]
            r=response(sid)
            if len(calls)==2:
                if mode=='uncertain':r['status']='uncertain'
                if mode=='other_book':r['same_book']=False
                if mode=='moved':raise ValueError('Arm moved during book observation')
            return r
        real_sample=coarse.sample_geometry
        def sample(*a):
            metrics[0]+=1
            if metrics[0]==1 and mode not in ('primary','quality_warning'):
                if mode=='missing_depth':raise ValueError('Book face has insufficient measured depth')
                raise LocalDepthError('GPT-selected local patch has insufficient measured depth')
            return real_sample(*a)
        with tempfile.TemporaryDirectory() as td,patch.object(coarse,'time',fake_time),\
                patch('cart_book_grasp_v2_gpt.GPTBookClient',return_value=NS(select=model)),\
                patch('cart_book_grasp_v3_2_head_fallback.load_config',return_value=dict(
                    enabled=True,model='configured-model',timeout_seconds=30,min_confidence=.7)),\
                patch.object(coarse,'sample_geometry',side_effect=sample):
            g,ref=coarse.perceive(camera,coarse.load_config(),Path(td),Mock())
            self.assertEqual((Path(td)/'failed_depth_frame.npz').exists(),mode not in ('primary','quality_warning'))
            self.assertEqual((Path(td)/'depth_review_response.json').exists(),mode not in ('primary','quality_warning'))
            samples=json.loads((Path(td)/'depth_samples.json').read_text())
            self.assertEqual(len(samples),3)
            self.assertGreater(min(x['captured_unix_s'] for x in samples),1015. if mode in ('primary','quality_warning') else 1020.)
            if mode=='quality_warning':
                self.assertIn('local_depth_MAD_exceeds_6mm',samples[0]['local_depth']['top']['quality_warnings'])
            self.assertEqual(g['geometry_method'],'gpt_landmarks_local_rgbd')
            self.assertIn('top_center_uv',ref['selection'])
        return calls,metrics[0]

    def test_local_depth_failure_reviews_once_and_recollects_three_frames(self):
        calls,count=self.run_flow();self.assertEqual(len(calls),2);self.assertEqual(count,4)

    def test_primary_route_uses_one_model_call_and_direct_local_depth(self):
        calls,count=self.run_flow('primary');self.assertEqual(len(calls),1);self.assertEqual(count,3)
        self.assertIn('top_center_uv',calls[0]['text']['format']['schema']['required'])

    def test_quality_warning_does_not_request_gpt_review(self):
        with patch('builtins.print'):
            calls,count=self.run_flow('quality_warning')
        self.assertEqual(len(calls),1);self.assertEqual(count,3)

    def test_uncertain_review_stops(self):
        with self.assertRaisesRegex(ValueError,'did not select'):self.run_flow('uncertain')

    def test_other_book_cannot_replace_selected_book(self):
        with self.assertRaisesRegex(ValueError,'same-book'):self.run_flow('other_book')

    def test_motion_during_review_is_not_bypassed(self):
        with self.assertRaisesRegex(ValueError,'Arm moved'):self.run_flow('moved')

    def test_invalid_depth_is_not_a_plane_override(self):
        with self.assertRaisesRegex(ValueError,'insufficient measured'):self.run_flow('missing_depth')

    def test_local_depth_still_requires_temporal_stability(self):
        with self.assertRaisesRegex(ValueError,'depth unstable'):self.run_flow('unstable')

    def test_calibration_change_during_model_is_not_accepted(self):
        with self.assertRaisesRegex(ValueError,'calibration changed'):self.run_flow('calibration_changed')

    def test_scene_change_is_checked_independently_of_arm(self):
        with self.assertRaisesRegex(ValueError,'Book image changed'):self.run_flow('image_changed')


class MotionGate(unittest.TestCase):
    def state(self,t,speed=0.,angle=0.):
        q=Rotation.from_euler('z',angle,degrees=True).as_quat()[[3,0,1,2]]
        return dict(pose_sample_monotonic=t,joint_velocity=[0.]*8+[speed]+[0.]*5,
            left_arm=dict(orientation=[1.,0.,0.,0.]),right_arm=dict(orientation=q))

    def test_reported_single_spike_allowed_then_cleared(self):
        gate=ObservationMotionGate(self.state(1.))
        for t,v in [(1.,0.),(1.02,.006283),(1.04,0.)]:
            gate.check(self.state(t,v))
        self.assertIsNone(gate.fast_start)

    def test_continuous_speed_stops(self):
        gate=ObservationMotionGate(self.state(1.))
        with self.assertRaisesRegex(ValueError,'sustained'):
            for t in np.arange(1.,1.3,.02):gate.check(self.state(t,.006283))

    def test_large_speed_stops_immediately(self):
        gate=ObservationMotionGate(self.state(1.))
        with self.assertRaisesRegex(ValueError,'significant'):gate.check(self.state(1.,.03))

    def test_rotation_stops_even_with_zero_velocity(self):
        gate=ObservationMotionGate(self.state(1.))
        with self.assertRaisesRegex(ValueError,'rotated'):gate.check(self.state(1.01,angle=.2))

    def test_repeated_samples_do_not_count(self):
        gate=ObservationMotionGate(self.state(1.))
        for _ in range(100):gate.check(self.state(1.,.007))
        self.assertEqual(gate.fast_count,1)

    def test_missing_feedback_is_not_treated_as_settling(self):
        gate=ObservationMotionGate(self.state(1.));gate.check(self.state(1.))
        with self.assertRaisesRegex(ValueError,'gap'):gate.check(self.state(1.3))


if __name__=='__main__':unittest.main()
