"""Meaningful local-selection regressions: identity, ambiguity, pose, and no motion on failure."""
import builtins
import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import cv2
import numpy as np
from PIL import Image
import cart_book_grasp_v2_1_selection as local
from cart_book_grasp_v2_dual_view import load_config as camera_config
from cart_book_grasp_v2_pause import PauseRequested


class FakeSemantics:
    def encode(self,crops,texts,checkpoint=lambda:None):
        features=[]
        for crop in crops:
            vector=np.asarray(crop).reshape(-1,3).mean(axis=0).astype(float)
            features.append(vector/max(1e-8,np.linalg.norm(vector)))
        logits=np.full((len(crops),len(texts)),-5.)
        logits[:,0]=2.;logits[:,-1]=-2.
        return np.asarray(features),logits


class LocalTests(unittest.TestCase):
    def setUp(self):
        self.c = local.load_config()
        self.rgb = np.full((240, 360, 3), 220, np.uint8)
        self.depth = np.full((240, 360), 800., np.float32)
        self.k = np.array([[300.,0,180],[0,300,120],[0,0,1]])
        self.tf = np.eye(4); self.tf[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
        self.raw = []
        for i, color in enumerate([(230,30,30),(30,230,30),(30,30,230)]):
            m = np.zeros((240,360),bool);m[45:195,30+110*i:90+110*i] = True
            self.rgb[m] = color
            self.raw.append((.9,m,[30+110*i,45,90+110*i,195]))
        self.head = local.build_observations(self.raw,self.rgb,self.c,'head',self.depth,self.k,self.tf)
        self.wrist = local.build_observations(self.raw,self.rgb,self.c,'wrist')
        # The actual CuroboState protocol returns [left, right] x 7 radians.
        self.robot = SimpleNamespace(right_safe_joint_deg=np.zeros(7),get_current_state=lambda:{'joint_state':np.zeros((2,7))})

    def test_startup_does_not_import_gpt_or_require_credentials(self):
        original = builtins.__import__
        def guard(name,*args,**kwargs):
            if name in ('cart_book_grasp_v2_gpt','openai'):
                raise AssertionError('GPT dependency was imported')
            return original(name,*args,**kwargs)
        with patch('builtins.__import__',side_effect=guard),patch.dict('os.environ',{},clear=True):
            a=local.LocalBookSelectionAgent();a.check_ready()
            self.assertFalse(hasattr(a,'client'))

    def test_instruction_grammar(self):
        for text,spatial,rank,color in [('给我中间的书','middle',None,None),('右边红色的书','right',None,'red'),
                ('第2本',None,1,None),('第5本',None,4,None),('黄色',None,None,'yellow'),
                ('please give me the blue book',None,None,'blue'),('3',None,2,None),
                ('geiwozhongjianzongsedeshu','middle',None,'brown')]:
            with self.subTest(text=text):
                rule=local.parse_instruction(text)
                self.assertEqual((rule['spatial'],rule['rank'],rule['color']),(spatial,rank,color))
        self.assertEqual(local.parse_instruction('给我强化学习的书')['topic'],'reinforcement learning')
        for text in ('不要红色','左边第二本','红色或蓝色','最右边倒数第二本','解剖学','随便一本','右边2本'):
            with self.subTest(text=text),self.assertRaises(local.SelectionError):local.parse_instruction(text)

    def test_variable_count_and_partial_masks_remain_for_semantic_review(self):
        self.assertEqual(len(self.head),3)
        self.assertEqual(len(local.build_observations(self.raw[:2],self.rgb,self.c,'wrist')),2)
        raw=copy.deepcopy(self.raw);raw[0][1][:,0:40]=True
        items=local.build_observations(raw,self.rgb,self.c,'wrist')
        self.assertEqual(len(items),3);self.assertTrue(items[0]['clipped'])

    def test_four_candidates_are_not_truncated(self):
        m=np.zeros((240,360),bool);m[5:35,120:210]=True
        self.assertEqual(len(local.build_observations(self.raw+[(.85,m,[])],self.rgb,self.c,'wrist')),4)

    def test_head_scores_from_failed_capture_reach_semantic_review(self):
        raw=[(score,mask,box) for score,(_,mask,box) in
             zip((.4003885,.4196410,.4216110),self.raw)]
        old_report={}
        old=local.build_observations(raw,self.rgb,dict(self.c,book_min_score=.5),'head',
                                     self.depth,self.k,self.tf,diagnostics=old_report)
        self.assertEqual(old,[])
        self.assertEqual(old_report['raw_count'],3)
        self.assertEqual(old_report['after_score_count'],0)
        report={}
        items=local.build_observations(raw,self.rgb,self.c,'head',self.depth,self.k,self.tf,
                                       diagnostics=report)
        self.assertEqual(len(items),3)
        self.assertEqual(report['after_score_count'],3)
        self.assertEqual(report['after_mask_cleanup_count'],3)
        self.assertEqual(report['accepted_count'],3)
        # A retained SAM3 proposal is not automatically a confirmed book.
        self.assertEqual(local.verified_observations(items,self.c),[])
        self.assertTrue(all(v['geometry'] is None for v in items))

    def test_filter_report_identifies_workspace_rejections(self):
        report={};c=dict(self.c,head_base_bounds_mm=[[400,600],[-350,350],[-350,250]])
        items=local.build_observations(self.raw,self.rgb,c,'head',self.depth,self.k,self.tf,
                                       diagnostics=report)
        self.assertEqual(items,[])
        self.assertEqual(report['after_score_count'],3)
        self.assertEqual(report['after_mask_cleanup_count'],3)
        self.assertEqual(report['accepted_count'],0)
        self.assertEqual(len(report['rejected']),3)
        self.assertTrue(all(v['reason']=='outside_base_workspace' for v in report['rejected']))
        self.assertIn('outside_base_workspace',local.proposal_filter_summary('head',report))
        json.dumps(report,allow_nan=False)

    def test_lower_score_proposals_still_undergo_semantic_rejection(self):
        from cart_book_grasp_v2_1_semantics import annotate_views
        raw=[(.4,mask,box) for _,mask,box in self.raw]
        head=local.build_observations(raw,self.rgb,self.c,'head',self.depth,self.k,self.tf)
        def encode(crops,texts,checkpoint):
            logits=np.zeros((len(crops),len(texts)));logits[:,:3]=-5.;logits[:,3]=2.
            return np.ones((len(crops),1)),logits
        annotate_views(self.rgb,head,self.rgb,[],local.parse_instruction('red book'),self.c,
                       model=SimpleNamespace(encode=encode))
        self.assertEqual(local.verified_observations(head,self.c),[])

    def test_missing_depth_does_not_redefine_middle(self):
        self.depth[self.raw[0][1]]=0
        head=local.build_observations(self.raw,self.rgb,self.c,'head',self.depth,self.k,self.tf)
        self.assertEqual(len(head),3);self.assertTrue(any(v['base_y_mm'] is None for v in head))
        with self.assertRaisesRegex(local.SelectionError,'位置/实例'):
            local.choose_target(local.parse_instruction('middle'),head,self.wrist,(0,1,2),self.c)

    def test_fixed_workspace_and_pose_enforced(self):
        c=dict(self.c,head_base_bounds_mm=[[400,600],[-350,350],[-350,250]])
        self.assertFalse(local.build_observations(self.raw,self.rgb,c,'head',self.depth,self.k,self.tf))
        state=np.zeros(14);state[8]=np.deg2rad(2)
        self.robot.get_current_state=lambda:{'joint_state':state}
        with self.assertRaisesRegex(local.SelectionError,'first_reset'):
            local.check_ready_pose(self.robot,self.c)

    def test_first_reset_accepts_protocol_matrix_and_legacy_flat_state(self):
        ready=np.array([115.,85.,-65.,112.,100.,-20.3164,0.])
        measured=ready.copy();measured[1]+=.241
        # Deliberately different left joints: index 1, not index 0, is right.
        joints=np.deg2rad(np.array([[10.,20.,0.,40.,0.,20.,0.],measured]))
        expected=None
        for values in (joints,joints.tolist(),joints.reshape(14),joints.reshape(14).tolist()):
            with self.subTest(shape=np.shape(values)):
                self.robot.right_safe_joint_deg=ready
                self.robot.get_current_state=lambda:{'joint_state':values}
                result=local.check_ready_pose(self.robot,self.c)
                np.testing.assert_allclose(result['right_joint_deg'],measured)
                self.assertAlmostEqual(result['max_error_deg'],.241)
                if expected is None:expected=result
                else:self.assertEqual(result,expected)

    def test_first_reset_matrix_still_rejects_actual_right_arm_error(self):
        joints=np.zeros((2,7));joints[1,5]=np.deg2rad(2.)
        self.robot.get_current_state=lambda:{'joint_state':joints}
        with self.assertRaisesRegex(local.SelectionError,'最大误差 2.00'):
            local.check_ready_pose(self.robot,self.c)

    def test_first_reset_rejects_invalid_shapes_missing_and_nonfinite_states(self):
        bad_states=[None,{}, {'joint_state':np.zeros(7)}, {'joint_state':np.zeros(13)},
                    {'joint_state':np.zeros((7,2))}, {'joint_state':[[0]*7,[0]*6]},
                    {'joint_state':np.full((2,7),np.nan)}, {'joint_state':np.full(14,np.inf)},
                    {'joint_state':['invalid']*14}]
        for state in bad_states:
            with self.subTest(state=state),self.assertRaises(local.SelectionError):
                self.robot.get_current_state=lambda:state
                local.check_ready_pose(self.robot,self.c)
        self.robot.get_current_state=lambda:{'joint_state':np.zeros((2,7))}
        for ready in (np.zeros(6),np.zeros((1,7)),np.full(7,np.nan)):
            with self.subTest(ready=ready),self.assertRaises(local.SelectionError):
                self.robot.right_safe_joint_deg=ready
                local.check_ready_pose(self.robot,self.c)

    def test_same_and_reversed_mapping_by_appearance(self):
        mapping,_=local.match_views(self.rgb,self.head,self.rgb,self.wrist,self.c)
        self.assertEqual(mapping,(0,1,2))
        flipped=self.rgb[:,::-1].copy()
        raw=[(s,m[:,::-1].copy(),[]) for s,m,b in self.raw]
        wrist=local.build_observations(raw,flipped,self.c,'wrist')
        mapping,decision=local.match_views(self.rgb,self.head,flipped,wrist,self.c)
        self.assertEqual(mapping,(2,1,0));self.assertEqual(decision['order_direction'],'reversed')
        selected=local.choose_target(dict(rank=0,color='red'),self.head,wrist,mapping,self.c)
        self.assertIs(selected,self.head[0])

    def test_id_strings_cannot_determine_correspondence(self):
        for item,newid in zip(self.wrist,['arbitrary_C','arbitrary_A','arbitrary_B']):item['id']=newid
        mapping,decision=local.match_views(self.rgb,self.head,self.rgb,self.wrist,self.c)
        self.assertEqual(mapping,(0,1,2));self.assertEqual(decision['matches'][0]['wrist_id'],'arbitrary_C')

    def test_same_count_and_neutral_colors_are_not_identity(self):
        rgb=self.rgb.copy()
        for _,m,_ in self.raw:rgb[m]=100
        self.assertEqual(local.match_views(rgb,self.head,rgb,self.wrist,self.c)[0],(None,None,None))
        self.assertEqual(local.solve_assignment(np.ones((3,3))*.9,self.c)[0],(None,None,None))

    def test_arbitrary_order_and_rectangular_partial_assignment(self):
        matrix=np.zeros((3,3));matrix[np.arange(3),[1,0,2]]=.95
        self.assertEqual(local.solve_assignment(matrix,self.c)[0],(1,0,2))
        matrix=np.array([[.9,.2],[.3,.4],[.1,.92]])
        self.assertEqual(local.solve_assignment(matrix,self.c)[0],(0,None,1))

    def test_non_target_ambiguity_does_not_block_unique_target(self):
        mapping,_=local.solve_assignment(np.array([[.9,.9,0],[.9,.9,0],[0,0,.95]]),self.c)
        self.assertEqual(mapping,(None,None,2))
        selected=local.choose_target(local.parse_instruction('右边的书'),self.head,self.wrist,mapping,self.c)
        self.assertIs(selected,self.head[2])
        with self.assertRaises(local.SelectionError):
            local.choose_target(local.parse_instruction('左边的书'),self.head,self.wrist,mapping,self.c)

    def test_even_count_middle_and_extra_wrist_book_are_ambiguous(self):
        with self.assertRaisesRegex(local.SelectionError,'偶数'):
            local.choose_target(local.parse_instruction('middle'),self.head[:2],self.wrist[:2],(0,1),self.c)
        with self.assertRaisesRegex(local.SelectionError,'漏书'):
            local.choose_target(local.parse_instruction('left'),self.head[:2],self.wrist,(0,1),self.c)

    def test_duplicate_color_needs_position(self):
        for item in self.head+self.wrist:item['colors']={k:float(k=='gray') for k in local.COLORS}
        with self.assertRaisesRegex(local.SelectionError,'匹配 3'):
            local.choose_target(dict(rank=None,color='gray'),self.head,self.wrist,(0,1,2),self.c)
        self.assertIs(local.choose_target(dict(rank=1,color='gray'),self.head,self.wrist,(0,1,2),self.c),self.head[1])

    def test_texture_can_match_neutral_books_with_geometric_evidence(self):
        rng=np.random.default_rng(9);rgb=self.rgb.copy()
        for _,mask,_ in self.raw:
            gray=rng.integers(30,220,mask.sum(),dtype=np.uint8)
            rgb[mask]=np.repeat(gray[:,None],3,axis=1)
        rotated=cv2.warpAffine(rgb,cv2.getRotationMatrix2D((180,120),1.,1.),(360,240))
        raw=[(s,cv2.warpAffine(m.astype(np.uint8),cv2.getRotationMatrix2D((180,120),1.,1.),(360,240)).astype(bool),[]) for s,m,_ in self.raw]
        wrist=local.build_observations(raw,rotated,self.c,'wrist')
        mapping,report=local.match_views(rgb,self.head,rotated,wrist,self.c)
        self.assertEqual(mapping,(0,1,2))
        self.assertTrue(all(m['texture']['inliers']>=8 for m in report['matches']))

    def run_selector(self,directory,*,ambiguous=False,observe_only=False,geometry_error=False,stale=False,pause=False):
        selector=local.LocalBookSelectionAgent(output_root=directory,observe_only=observe_only,semantic_model=FakeSemantics())
        selector.set_instruction('中间的书');selector.bind_observation_robot(self.robot)
        rgb=self.rgb.copy()
        if ambiguous:
            for _,m,_ in self.raw:rgb[m]=100
        geometry=Mock()
        def prepare(c):
            self.assertEqual(c['id'],'book_2')
            self.assertEqual(c['recognition'],'book_proposal')
            np.testing.assert_array_equal(c['book_mask'],self.raw[1][1])
            if geometry_error:raise local.SelectionError('bad head geometry')
            c['geometry']={'head_only':True}
        geometry.side_effect=prepare
        geom=SimpleNamespace(_load_sam3_detector=lambda:(None,None,Image),_sam3_raw_books=lambda *a,**k:self.raw)
        captured=dict(rgb=rgb,captured_at=time.monotonic(),pair_gap_seconds=.1)
        if pause:selector.checkpoint=lambda:(_ for _ in ()).throw(PauseRequested())
        with patch('cart_book_grasp_v2_dual_view.capture_wrist',return_value=captured):
            error=None;selected=None
            try:
                selected=selector.select_instances(rgb,self.depth,self.k,self.tf,geom,geometry,
                    head_captured_at=time.monotonic()-(40 if stale else .1),dual_config=camera_config())
            except (Exception, PauseRequested) as e:error=e
        return selector,geometry,selected,error

    def test_live_adapter_calls_geometry_once_with_head_target(self):
        with tempfile.TemporaryDirectory() as path:
            selector,geometry,selected,error=self.run_selector(path)
            if error:raise error
            geometry.assert_called_once();self.assertTrue(selected['geometry']['head_only'])
            self.assertFalse(selector.last_decision['gpt_called'])
            self.assertTrue((Path(selector.last_decision['output'])/'decision.json').exists())
            self.assertTrue(all(c['geometry'] is None for c in selector.last_wrist_view['candidates']))

    def test_ambiguous_stale_pause_and_observe_only_never_call_geometry(self):
        for kwargs in [dict(ambiguous=True),dict(stale=True),dict(pause=True),dict(observe_only=True)]:
            with self.subTest(kwargs=kwargs),tempfile.TemporaryDirectory() as path:
                selector,geometry,selected,error=self.run_selector(path,**kwargs)
                self.assertIsNotNone(error);geometry.assert_not_called();self.assertIsNone(selected)

    def test_geometry_failure_does_not_substitute_another_book(self):
        with tempfile.TemporaryDirectory() as path:
            selector,geometry,selected,error=self.run_selector(path,geometry_error=True)
            self.assertIsInstance(error,local.SelectionError);geometry.assert_called_once()
            self.assertIsNone(selected);self.assertEqual(selector.last_decision['status'],'geometry_failed')

    def test_joint_drift_during_detection_blocks_geometry(self):
        with tempfile.TemporaryDirectory() as path:
            selector=local.LocalBookSelectionAgent(output_root=path,semantic_model=FakeSemantics());selector.set_instruction('middle')
            state=np.zeros(14)
            self.robot.get_current_state=lambda:{'joint_state':state}
            selector.bind_observation_robot(self.robot)
            def detect(*a,**k):state[8]=.1;return self.raw
            geom=SimpleNamespace(_load_sam3_detector=lambda:(None,None,Image),_sam3_raw_books=detect)
            geometry=Mock()
            with patch('cart_book_grasp_v2_dual_view.capture_wrist',return_value=dict(rgb=self.rgb,pair_gap_seconds=.1)):
                with self.assertRaisesRegex(local.SelectionError,'first_reset'):
                    selector.select_instances(self.rgb,self.depth,self.k,self.tf,geom,geometry,head_captured_at=time.monotonic())
            geometry.assert_not_called()


    def make_reference(self, path):
        import hashlib
        import cart_book_grasp_v2_1 as entry
        directory=Path(path)/'pair';directory.mkdir()
        pose=local.check_ready_pose(self.robot,self.c)
        meta=dict(K=self.k.tolist(),c2w=self.tf.tolist(),pair_gap_seconds=.2,
                  wrist_camera=camera_config(),pose_verified=True,pose=pose)
        (directory/'camera.json').write_text(json.dumps(meta))
        np.save(directory/'depth.npy',self.depth)
        for view in ('head','wrist'):
            Image.fromarray(self.rgb).save(directory/(view+'_rgb.png'))
            np.savez_compressed(directory/(view+'_raw.npz'),frame_sha256=hashlib.sha256(self.rgb.tobytes()).hexdigest(),
                scores=np.array([v[0] for v in self.raw]),masks=np.array([v[1] for v in self.raw]),boxes=np.array([v[2] for v in self.raw]))
        output=Path(path)/'reference'
        entry.register_reference(directory,local.CONFIG_PATH,'same',output)
        return output/'reference.json',meta,directory

    def test_registered_same_camera_identity_and_geometry_binding(self):
        with tempfile.TemporaryDirectory() as path:
            reference,meta,_=self.make_reference(path)
            mapping,report=local.match_reference(reference,self.rgb,self.head,self.rgb,self.wrist,self.c,
                                                 camera_metadata=meta,pose=meta['pose'])
            self.assertEqual(mapping,(0,1,2));self.assertEqual(report['method'],'same_camera_references')
            altered=copy.deepcopy(meta);altered['c2w'][0][3]=10
            with self.assertRaisesRegex(local.SelectionError,'标定'):
                local.match_reference(reference,self.rgb,self.head,self.rgb,self.wrist,self.c,camera_metadata=altered,pose=meta['pose'])
            bad_pose=copy.deepcopy(meta['pose']);bad_pose['right_joint_deg'][2]=10
            with self.assertRaisesRegex(local.SelectionError,'观察姿态'):
                local.match_reference(reference,self.rgb,self.head,self.rgb,self.wrist,self.c,camera_metadata=meta,pose=bad_pose)

    def test_reference_registration_requires_verified_pose_and_same_frame_masks(self):
        import cart_book_grasp_v2_1 as entry
        with tempfile.TemporaryDirectory() as path:
            _,meta,directory=self.make_reference(path)
            meta['pose_verified']=False;(directory/'camera.json').write_text(json.dumps(meta))
            with self.assertRaisesRegex(local.SelectionError,'first_reset'):
                entry.register_reference(directory,local.CONFIG_PATH,'same',Path(path)/'unverified')
            meta['pose_verified']=True;(directory/'camera.json').write_text(json.dumps(meta))
            Image.fromarray(self.rgb//2).save(directory/'head_rgb.png')
            with self.assertRaisesRegex(local.SelectionError,'同一帧'):
                entry.register_reference(directory,local.CONFIG_PATH,'same',Path(path)/'wrong_frame')

    def test_modified_reference_and_equal_reference_books_reject(self):
        with tempfile.TemporaryDirectory() as path:
            reference,meta,_=self.make_reference(path)
            Image.fromarray(self.rgb//2).save(reference.parent/'head_rgb.png')
            with self.assertRaisesRegex(local.SelectionError,'修改'):
                local.match_reference(reference,self.rgb,self.head,self.rgb,self.wrist,self.c,camera_metadata=meta)
            blank=np.full_like(self.rgb,100)
            with self.assertRaisesRegex(local.SelectionError,'不唯一'):
                local._reference_scores(blank,self.head,blank,self.wrist,self.c)

    def test_capture_uses_v2_calibration_and_aligned_camera_without_robot(self):
        import cart_book_grasp_v2_1 as entry
        camera=SimpleNamespace(capture_image=lambda:(self.rgb,self.depth),K=self.k,c2w=self.tf,
                               _orbbec_align_mode='HW_MODE',close=Mock())
        fake=SimpleNamespace(load_orbbec_calibration=Mock(return_value={'path':'verified.json'}),
                             start_calibrated_orbbec=Mock(return_value=camera),LATEST_ORBBEC_CALIBRATION_PATH='verified.json')
        with tempfile.TemporaryDirectory() as path,patch.dict('sys.modules',{'dual_v2_2_agent_pcl':fake}), \
             patch('cart_book_grasp_v2_dual_view.capture_wrist',return_value=dict(rgb=self.rgb,pair_gap_seconds=.1)):
            output=Path(path)/'capture';entry.capture_pair(output)
            meta=json.loads((output/'camera.json').read_text())
            self.assertFalse(meta['pose_verified']);self.assertFalse(meta['robot_connected'])
            self.assertEqual(meta['calibration_path'],'verified.json')
            self.assertEqual(meta['depth_alignment'],'HW_MODE')
            camera.close.assert_called_once()


    def test_head_geometry_accepts_local_config_without_gpt_fields(self):
        from cart_book_grasp_v2_target import prepare_instance_target
        from cart_book_grasp_v2_motion import _spine_center_from_mask
        selected=copy.deepcopy(self.head[1])
        def segment(processor,to_numpy,image,*,prompt):
            self.assertEqual(prompt,'book spine')
            rgb=np.asarray(image);mask=(rgb[:,:,1]>200)&(rgb[:,:,0]<50)
            return [(.99,mask,[])]
        geometry=SimpleNamespace(_load_sam3_detector=lambda:(None,None,Image),_sam3_raw_books=segment)
        prepare_instance_target(selected,self.rgb,self.depth,self.k,self.tf,geometry,_spine_center_from_mask,local.CONFIG_PATH)
        self.assertEqual(selected['geometry_status'],'ready')
        self.assertAlmostEqual(selected['geometry']['book_surface_x_mm'],800.)
        self.assertEqual(selected['id'],'book_2')

    def test_v21_entrypoint_injects_local_selector(self):
        import cart_book_grasp_v2_1 as entry
        from cart_book_grasp_v2_1_numbered import NumberedBookSelectionAgent,NumberedCartLoop
        with patch('cart_book_grasp_v2.main') as main, patch.object(local.LocalBookSelectionAgent,'warmup'):
            entry.main(['--observe-only','--assume-head-camera-calibrated'])
            selector=main.call_args.kwargs['selector']
            self.assertIsInstance(selector,NumberedBookSelectionAgent)
            self.assertIs(main.call_args.kwargs['loop_factory'],NumberedCartLoop)
            self.assertTrue(selector.observe_only)
            self.assertFalse(hasattr(selector,'client'))
            self.assertEqual(main.call_args.kwargs['book_entry'].__name__,'run_v13')
            self.assertEqual(main.call_args.kwargs['config_path'].name,'cart_book_grasp_v2_1_motion_config.json')

    def test_five_books_use_actual_count_for_right_and_middle(self):
        raw=[]
        for i in range(5):
            mask=np.zeros((240,360),bool);mask[45:195,40+60*i:70+60*i]=True
            raw.append((.9,mask,[]))
        head=local.build_observations(raw,self.rgb,self.c,'head',self.depth,self.k,self.tf)
        wrist=local.build_observations(raw,self.rgb,self.c,'wrist')
        self.assertEqual(len(head),5)
        mapping,_=local.solve_assignment(np.eye(5)*.99,self.c)
        self.assertIs(local.choose_target(local.parse_instruction('右边的书'),head,wrist,mapping,self.c),head[4])
        self.assertIs(local.choose_target(local.parse_instruction('中间的书'),head,wrist,mapping,self.c),head[2])

    def test_semantic_category_transfer_does_not_invent_identity(self):
        from cart_book_grasp_v2_1_semantics import annotate_views
        head=copy.deepcopy(self.head[:2]);wrist=copy.deepcopy(self.wrist[:1])
        def encode(crops,texts,checkpoint):
            logits=np.full((3,len(texts)),-5.)
            logits[0,0]=-1.;logits[0,3]=0.  # Partly seen head book: weak category.
            logits[1,0]=-5.;logits[1,3]=2.  # Strong cart/handle evidence.
            logits[2,0]=3.
            return np.array([[1.,0.],[1.,0.],[1.,0.]]),logits
        report=annotate_views(self.rgb,head,self.rgb,wrist,local.parse_instruction('middle'),self.c,
                              model=SimpleNamespace(encode=encode))
        self.assertTrue(head[0]['verified_book']);self.assertFalse(head[1]['verified_book'])
        self.assertEqual(report['candidates'][0]['category_evidence'],'opposite_view_book_category')
        self.assertNotIn('matched_id',head[0])

    def test_no_category_transfer_from_two_unconfirmed_views(self):
        from cart_book_grasp_v2_1_semantics import annotate_views
        head=copy.deepcopy(self.head[:1]);wrist=copy.deepcopy(self.wrist[:1])
        def encode(crops,texts,checkpoint):
            logits=np.zeros((2,len(texts)));logits[:,:3]=-1
            return np.ones((2,1)),logits
        annotate_views(self.rgb,head,self.rgb,wrist,local.parse_instruction('middle'),self.c,
                       model=SimpleNamespace(encode=encode))
        self.assertFalse(head[0]['verified_book']);self.assertFalse(wrist[0]['verified_book'])

    def test_weak_alternative_is_not_zeroed_into_false_uniqueness(self):
        mapping,_=local.solve_assignment([[.81,.79],[.1,.1]],self.c)
        self.assertEqual(mapping,(None,None))

    def test_multi_book_group_is_removed_without_losing_individuals(self):
        items=copy.deepcopy(self.head[:2])
        for item in items:item['verified_book']=True
        group=dict(items[0],id='book_group',mask=items[0]['mask']|items[1]['mask'])
        group['book_mask']=group['mask']
        self.assertEqual([v['id'] for v in local.verified_observations(items+[group],self.c)],['book_1','book_2'])
        self.assertFalse(group['verified_book'])

    def test_geometry_adapter_uses_v13_fit_and_selected_head_depth(self):
        import cart_book_grasp_v2_1_motion as motion
        candidate=copy.deepcopy(self.head[1])
        captured=[]
        def detect(processor,to_numpy,image,*,prompt):
            captured.append(np.asarray(image));self.assertEqual(prompt,'book spine')
            im=np.asarray(image);mask=(im[:,:,1]>200)&(im[:,:,0]<50)
            return [(.99,mask,[])]
        geom=SimpleNamespace(_load_sam3_detector=lambda:(None,None,Image),_sam3_raw_books=detect)
        def fit(mask,depth,k):
            np.testing.assert_array_equal(mask,self.raw[1][1]);self.assertIs(depth,self.depth)
            return dict(mask=mask,box=self.raw[1][2],uv=[170.,120.],point_camera_m=np.array([0.,0.,.8]),depth_m=.8)
        with patch.object(motion.v13,'_spine_center_from_mask',side_effect=fit) as fitter:
            motion.prepare_head_target(candidate,self.rgb,self.depth,self.k,self.tf,geom,local.CONFIG_PATH)
        fitter.assert_called_once();self.assertEqual(candidate['id'],'book_2')
        self.assertEqual(candidate['geometry']['book_surface_x_mm'],800.)
        self.assertLess(captured[0].shape[1],self.rgb.shape[1])

    def test_runtime_invokes_v13_and_restores_scoped_hooks(self):
        import cart_book_grasp_v2_1_motion as adapter
        import cart_book_grasp_v1_3_motion as motion
        import cart_book_grasp_v1_3_visualization as display
        import cart_book_grasp_v2_motion as shell
        original=motion._install_spine_center;original_display=display.install_book_visualization
        original_path=shell._CART_PLUS_CONFIG_PATH
        with patch.object(shell,'_SELECTION_AGENT',object()),patch.object(adapter.v13,'main',return_value='v13') as main:
            result=adapter.run_v13(task_types=['cart+'],config_path='local_motion.json',wrist_config_path='local_wrist.json')
            self.assertEqual(result,'v13');main.assert_called_once_with(task_types=['cart+'],config_path='local_motion.json',wrist_config_path='local_wrist.json')
        self.assertIs(motion._install_spine_center,original)
        self.assertIs(display.install_book_visualization,original_display)
        self.assertEqual(shell._CART_PLUS_CONFIG_PATH,original_path)


if __name__=='__main__':unittest.main()
