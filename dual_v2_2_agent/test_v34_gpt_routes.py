"""Offline new GPT handoffs: no hardware, network, or credential reads."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import numpy as np
from cart_book_grasp_v3_4_wrist_gpt import recover,build_payload
from cart_book_grasp_v3_gpt_recovery import initialize_tracking,make_lock
from cart_book_grasp_v3_feature_centerline import InitialFeaturesUnavailable,FeaturePointsExhausted,FeatureCenterlineLock
from cart_book_grasp_v3_servo import ServoError

class Routes(unittest.TestCase):
    def setUp(self):
        self.rgb=np.zeros((120,160,3),np.uint8);self.mask=np.ones((120,160),bool)
        self.config=dict(enabled=True,model='gpt-6-astra',timeout_seconds=1,min_confidence=.7)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
    def client(self,quad,same=True):
        def select(payload,*args):
            scene=payload['text']['format']['schema']['properties']['scene_id']['enum'][0]
            return dict(scene_id=scene,status='selected',confidence=.95,reason='book',spine_quad=quad,same_book=same)
        return NS(select=select)
    def test_gpt_ignores_old_width_angle_center_and_clipping_gates(self):
        # Wide, short, near-horizontal, side-clipped and outside center row.
        quad=[[0,100],[1000,100],[850,400],[50,400]]
        with patch('cart_book_grasp_v3_4_wrist_gpt.visible_spine_geometry',side_effect=AssertionError('old gate')):
            chosen=recover(self.rgb,{},detector=None,client=self.client(quad),config=self.config)
        self.assertEqual(chosen['centerline']['geometry_rejection'],'disabled')
        rng=np.random.default_rng(3);rgb=rng.integers(0,255,self.rgb.shape,dtype=np.uint8)
        with patch('cart_book_grasp_v3_centerline.visible_centerline',side_effect=AssertionError('second old gate')):
            lock=make_lock(rgb,chosen)
        self.assertTrue(lock.gpt_geometry)
        self.assertTrue(np.isfinite(lock.observe(rgb)['uv']).all())
    def test_reference_is_sent_and_uncertain_identity_rejected(self):
        quad=[[400,0],[600,0],[600,1000],[400,1000]]
        payload=build_payload(self.rgb,'s',self.config,review_context={'reason':'jump'},reference_rgb=self.rgb,reference_mask=self.mask)
        self.assertEqual(sum(c['type']=='input_image' for c in payload['input'][0]['content']),2)
        self.assertIn('same_book',payload['text']['format']['schema']['required'])
        with self.assertRaisesRegex(ServoError,'same physical book'):
            recover(self.rgb,{},detector=None,client=self.client(quad,False),config=self.config,reference_rgb=self.rgb)
    def test_initial_texture_failure_calls_gpt_once_then_verifies(self):
        first=dict(rgb=self.rgb,captured_at=1.)
        candidate=NS(observe=Mock(return_value={'inliers':12}))
        factory=Mock(side_effect=[InitialFeaturesUnavailable('texture'),candidate])
        select=Mock(return_value=dict(mask=self.mask,identity_confirmed=True))
        capture=Mock(side_effect=[dict(first,captured_at=2.),dict(first,captured_at=3.)])
        result=initialize_tracking(first,dict(mask=self.mask),select,capture,lambda:None,self.root,factory=factory)
        self.assertIs(result,candidate);select.assert_called_once();candidate.observe.assert_called_once()
        self.assertIn('reference_rgb',select.call_args.kwargs)
    def test_failed_initial_recovery_never_becomes_point_loss_forward(self):
        first=dict(rgb=self.rgb,captured_at=1.)
        candidate=NS(observe=Mock(side_effect=FeaturePointsExhausted(2)))
        factory=Mock(side_effect=[InitialFeaturesUnavailable('texture'),candidate])
        select=Mock(return_value=dict(mask=self.mask,identity_confirmed=True))
        capture=Mock(side_effect=[dict(first,captured_at=2.),dict(first,captured_at=3.)])
        with self.assertRaises(ServoError) as e:
            initialize_tracking(first,dict(mask=self.mask),select,capture,lambda:None,self.root,factory=factory)
        self.assertNotIsInstance(e.exception,FeaturePointsExhausted)
    def test_successful_initialization_does_not_query_gpt(self):
        select=Mock();candidate=NS()
        self.assertIs(initialize_tracking(dict(rgb=self.rgb),dict(mask=self.mask),select,Mock(),lambda:None,self.root,
                                        factory=lambda *a,**kw:candidate),candidate)
        select.assert_not_called()

class HeadRoutes(unittest.TestCase):
    def test_j7_geometry_at_small_tilt_still_triggers_review(self):
        from test_v3_head_tilt_review import TiltReviewTests
        fixture=TiltReviewTests();fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        from book_spine_wrist import J7GeometryUnavailable
        with patch('book_spine_wrist.j7_proposal',side_effect=J7GeometryUnavailable('parallel')):
            selected,_=fixture.run_review(fixture.selected(20.),observation_robot=Mock())
        fixture.fallback.assert_called_once()
        self.assertEqual(selected['head_tilt_review']['trigger'],'parallel')
    def test_full_book_failure_retries_same_book_and_uses_new_depth_mask(self):
        import cart_book_grasp_v1_3_motion as motion
        mask=np.zeros((100,100),bool);mask[10:90,45:55]=True
        book=np.zeros_like(mask);book[5:95,40:70]=True
        selected=dict(mask=mask,box=[45,10,55,90],uv=[50,50],score=.9,depth_m=.8)
        geometry=NS(fill_book_meta=lambda *a:None,_load_sam3_detector=lambda:(None,None,NS(fromarray=lambda a:a)),
                    _sam3_raw_books=Mock(side_effect=[[(.9,mask,[])],[]]))
        revised=dict(selected,selection_source='gpt6_polygon_fallback',head_tilt_review={'trigger':'full-book'})
        review=Mock(side_effect=[(selected,None),(revised,[(.9,book,[])])])
        motion._install_spine_center(geometry,spine_review=review)
        with patch.object(motion,'_spine_center_from_mask',return_value=selected),patch.object(motion,'whole_book_depth',return_value=dict(whole_book_depth_median_mm=800.,book_surface_x_mm=600.)) as depth:
            out=geometry.segment_cart_book_with_sam3(np.zeros((100,100,3),np.uint8),np.ones((100,100)),np.eye(3),np.eye(4))
        self.assertIsNotNone(out);self.assertIn('full-book',review.call_args.kwargs['force_reason'])
        np.testing.assert_array_equal(depth.call_args.args[0],book)

class RecoveryReviewRoutes(unittest.TestCase):
    def fixture(self):
        from test_v3_gpt_recovery import RecoveryTests
        f=RecoveryTests();f.setUp();self.addCleanup(f.doCleanups)
        f.old.reference_rgb=np.zeros((100,100,3),np.uint8);f.old.reference_mask=np.ones((100,100),bool)
        return f
    def test_large_jump_gets_two_image_identity_review_then_fresh_validation(self):
        f=self.fixture();f.candidate.last_u=140.;calls=[]
        def select(*args,**kw):
            calls.append(kw)
            return dict(f.select(*args,**kw),identity_confirmed=len(calls)>1)
        result=f.run_recovery(select=select)
        self.assertIs(result[0],f.candidate);self.assertEqual(len(calls),2)
        self.assertIn('reference_rgb',calls[1]);self.assertIn('review_context',calls[1])
    def test_retry_cannot_approve_a_jump_without_identity(self):
        f=self.fixture();f.candidate.last_u=140.
        with self.assertRaisesRegex(ServoError,'disagrees'):f.run_recovery()
        self.assertEqual(f.events.count('gpt'),2)
    def test_new_frame_feature_failure_reselects_but_does_not_blind_forward(self):
        f=self.fixture();f.candidate.observe=Mock(side_effect=[FeaturePointsExhausted(2),dict(inliers=10)])
        result=f.run_recovery()
        self.assertEqual(result[2]['inliers'],10);self.assertEqual(f.events.count('gpt'),2)
    def test_repeated_verification_failure_is_bounded(self):
        f=self.fixture();f.candidate.observe=Mock(side_effect=FeaturePointsExhausted(2))
        with self.assertRaises(ServoError) as e:f.run_recovery()
        self.assertNotIsInstance(e.exception,FeaturePointsExhausted)
        self.assertEqual(f.events.count('gpt'),2)

if __name__=='__main__':unittest.main()
