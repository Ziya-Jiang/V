"""Dual-view contract, capture cancellation and head-only geometry regressions."""
import copy
import json
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import cart_book_grasp_v2_agent as agent
import cart_book_grasp_v2_dual_view as dual
from cart_book_grasp_v2_dual_gpt import build_dual_payload,validate_dual_response
from cart_book_grasp_v2_pause import PauseRequested


class DualViewTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.zeros((160,200,3),np.uint8);self.depth=np.full((160,200),800.)
        self.k=np.array([[200,0,100],[0,200,80],[0,0,1.]])
        self.tf=np.eye(4);self.tf[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        self.head=[];self.wrist=[]
        for i in range(3):
            m=np.zeros((160,200),bool);m[20:140,20+i*55:40+i*55]=True
            self.rgb[m]=[255,50*(i+1),80]
            item=dict(id=f'book_{i+1}',book_mask=m,mask=m,box=[20+i*55,20,40+i*55,140],
                      uv=[30.+i*55,80.],base_y_mm=100.-100*i,score=.9,recognition='book_proposal',
                      proposal_method='sam3_instances',geometry=None,geometry_status='not_checked',error='')
            self.head.append(item)
            self.wrist.append(dict(item,id=f'wrist_{i+1}',recognition='wrist_book_proposal'))
        self.config=agent.load_config('cart_book_grasp_v2_agent_config.json')
        self.view=dict(rgb=self.rgb.copy(),candidates=self.wrist,min_match_confidence=.85,pair_gap_seconds=.3)

    def response(self,scene='scene'):
        def reviews(items):return [dict(id=c['id'],kind='book',confidence=.99,reason='A single book.',duplicate_of=None) for c in items]
        # Deliberately permuted IDs: pixel/order/ID equality is not correspondence.
        return dict(scene_id=scene,candidate_reviews=reviews(self.head),wrist_reviews=reviews(self.wrist),
            status='selected',selected_id='book_2',matching_ids=['book_2'],confidence=.98,reason='Matched by title.',
            cross_view_matches=[dict(head_id=f'book_{i+1}',wrist_id=f'wrist_{w}',status='matched',confidence=.97,
                                     reason='Same cover and title.') for i,w in enumerate([2,3,1])])

    def test_four_images_separate_ids_and_no_wrist_pose_metadata(self):
        p=build_dual_payload('给我中间的书',self.rgb,self.head,'scene',self.config,self.view)
        content=p['input'][0]['content'];data=json.loads(content[0]['text'])
        self.assertEqual([x['type'] for x in content],['input_text']+['input_image']*4)
        self.assertEqual(len(data['wrist_candidates']),3)
        self.assertTrue(all('base_y_mm' not in c and 'depth_median_mm' not in c for c in data['wrist_candidates']))
        schema=p['text']['format']['schema']
        self.assertEqual(schema['properties']['selected_id']['enum'],[None,'book_1','book_2','book_3'])
        self.assertIn('wrist_reviews',schema['required']);self.assertIn('cross_view_matches',schema['required'])
        self.assertIn('pixel positions across views are unrelated',p['instructions'])

    def test_permuted_ids_are_valid_and_only_head_geometry_is_used(self):
        client=Mock();selector=agent.BookSelectionAgent(client=client);selector.set_instruction('中间的书')
        client.select.side_effect=lambda p,*_:self.response(json.loads(p['input'][0]['content'][0]['text'])['scene_id'])
        def prepare(c):
            self.assertIs(c,self.head[1]);c['geometry']={'head_only':True}
        geometry=Mock(side_effect=prepare)
        selected=selector.select(self.rgb,self.depth,self.k,self.tf,[],[],None,candidate_data=self.head,
                                 prepare_geometry=geometry,wrist_view=self.view)
        self.assertIs(selected,self.head[1]);geometry.assert_called_once()
        self.assertTrue(selector.last_decision['dual_view'])
        self.assertTrue(all(c['geometry'] is None for c in self.wrist))
        self.assertEqual(selector.last_decision['cross_view_matches'][1]['wrist_id'],'wrist_3')

    def test_invalid_or_ambiguous_correspondence_rejects(self):
        cases=[]
        r=self.response();r['cross_view_matches'][1]['confidence']=.4;cases.append(r)
        r=self.response();r['cross_view_matches'][1].update(status='uncertain',wrist_id=None);cases.append(r)
        r=self.response();r['cross_view_matches'][1]['wrist_id']='wrist_2';cases.append(r)
        r=self.response();r['cross_view_matches']=r['cross_view_matches'][:-1];cases.append(r)
        r=self.response();r['wrist_reviews']=r['wrist_reviews'][:-1];cases.append(r)
        r=self.response();r['wrist_reviews'][2]['kind']='not_book';cases.append(r)
        r=self.response();r['selected_id']='wrist_3';r['matching_ids']=['wrist_3'];cases.append(r)
        r=self.response();r['scene_id']='old';cases.append(r)
        r=self.response();r['cross_view_matches'][1]['confidence']=float('nan');cases.append(r)
        r=self.response();r['cross_view_matches'][1]['head_id']='book_999';cases.append(r)
        for r in cases:
            with self.subTest(response=r),self.assertRaises(agent.SelectionError):
                validate_dual_response(r,self.head,self.view,'scene',.85)

    def test_ambiguous_response_never_calls_geometry(self):
        client=Mock();selector=agent.BookSelectionAgent(client=client);selector.set_instruction('red')
        def select(p,*_):
            r=self.response(json.loads(p['input'][0]['content'][0]['text'])['scene_id'])
            r['cross_view_matches'][1]['confidence']=.2
            return r
        client.select.side_effect=select;geometry=Mock()
        with self.assertRaisesRegex(agent.SelectionError,'对应关系不明确'):
            selector.select(self.rgb,self.depth,self.k,self.tf,[],[],None,candidate_data=self.head,
                            prepare_geometry=geometry,wrist_view=self.view)
        geometry.assert_not_called()

    def test_capture_precedes_both_segmenters_and_failure_clears_last_view(self):
        client=Mock();selector=agent.BookSelectionAgent(client=client);selector.set_instruction('red')
        events=[];cfg=dual.load_config()
        def capture(*_):events.append('capture');return {k:v for k,v in self.view.items() if k!='candidates'}
        def head(*_):events.append('head_sam');return self.head,{}
        def wrist(*_):events.append('wrist_sam');return self.wrist
        def select(p,*_):
            events.append('gpt');return self.response(json.loads(p['input'][0]['content'][0]['text'])['scene_id'])
        client.select.side_effect=select
        def prepare(c):events.append('geometry');c['geometry']={'ok':True}
        with patch.object(dual,'capture_wrist',side_effect=capture) as cap,patch.object(dual,'wrist_candidates',side_effect=wrist), \
             patch('cart_book_grasp_v2_instances.detect_instances',side_effect=head):
            selector.select_instances(self.rgb,self.depth,self.k,self.tf,None,prepare,
                                      head_captured_at=time.monotonic(),dual_config=cfg)
            self.assertEqual(events,['capture','head_sam','wrist_sam','gpt','geometry'])
            cap.side_effect=ValueError('camera unavailable');client.reset_mock()
            with self.assertRaisesRegex(ValueError,'camera unavailable'):
                selector.select_instances(self.rgb,self.depth,self.k,self.tf,None,prepare,
                                          head_captured_at=time.monotonic(),dual_config=cfg)
            client.select.assert_not_called();self.assertIsNone(selector.last_wrist_view)
            self.assertIsNone(selector.last_wrist_image)

    def test_unmatched_non_target_is_allowed_when_target_is_confirmed(self):
        r=self.response();r['cross_view_matches'][0].update(status='not_visible',wrist_id=None,confidence=0.)
        base,_,matches=validate_dual_response(r,self.head,self.view,'scene',.85)
        self.assertEqual(base['selected_id'],'book_2')
        self.assertEqual(matches['book_1']['status'],'not_visible')

    def test_joint_movement_before_result_blocks_geometry(self):
        state={'joint_state':np.zeros(14)};robot=SimpleNamespace(get_current_state=lambda:state)
        guard=dual.stationary_guard(robot,.5)
        state['joint_state'][8]=np.deg2rad(.6)
        with self.assertRaisesRegex(ValueError,'姿态变化'):guard()

    def test_movement_during_gpt_response_blocks_selected_head(self):
        state={'joint_state':np.zeros(14)};robot=SimpleNamespace(get_current_state=lambda:state)
        guard=dual.stationary_guard(robot,.5)
        client=Mock();selector=agent.BookSelectionAgent(client=client);selector.set_instruction('middle')
        def select(p,*_):
            state['joint_state'][10]=np.deg2rad(1.)
            return self.response(json.loads(p['input'][0]['content'][0]['text'])['scene_id'])
        client.select.side_effect=select;geometry=Mock()
        with self.assertRaisesRegex(ValueError,'姿态变化'):
            selector.select(self.rgb,self.depth,self.k,self.tf,[],[],None,candidate_data=self.head,
                            prepare_geometry=geometry,wrist_view=self.view,observation_guard=guard)
        geometry.assert_not_called()

    def test_motion_adapter_renders_both_views_and_drops_failed_previous_target(self):
        import cart_book_grasp_v2_motion as motion
        client=Mock();selector=agent.BookSelectionAgent(client=client);selector.set_instruction('middle')
        client.select.side_effect=lambda p,*_:self.response(json.loads(p['input'][0]['content'][0]['text'])['scene_id'])
        book_geom=SimpleNamespace(fill_book_meta=Mock(),BOOK_GRASP_MODE='cart+')
        display=SimpleNamespace(safe=Mock(),capture=Mock())
        robot=SimpleNamespace(get_current_state=lambda:{'joint_state':np.zeros(14)})
        captured={k:v for k,v in self.view.items() if k!='candidates'}
        with patch.object(motion,'_SELECTION_AGENT',selector),patch.object(dual,'capture_wrist',return_value=captured), \
             patch.object(dual,'wrist_candidates',return_value=self.wrist), \
             patch('cart_book_grasp_v2_instances.detect_instances',side_effect=lambda *args:(copy.deepcopy(self.head),{})), \
             patch('cart_book_grasp_v2_target.prepare_instance_target') as target:
            def prepare(c,*args):
                self.assertEqual(c['id'],'book_2')
                np.testing.assert_equal(c['book_mask'],self.head[1]['book_mask'])
                c['geometry']=dict(mask=c['book_mask'],box=c['box'],uv=c['uv'],book_surface_x_mm=600.)
            target.side_effect=prepare
            motion._install_spine_center(book_geom,display)
            mask,_=book_geom.segment_cart_book_with_sam3(self.rgb,self.depth,self.k,self.tf,
                head_captured_at=time.monotonic(),observation_robot=robot)
            np.testing.assert_equal(mask,self.head[1]['book_mask'])
            for path in ('book/images/wrist_rgb','book/images/wrist_candidates'):
                self.assertTrue(any(call.args[:2]==('log_image',path) for call in display.safe.call_args_list))
            def ambiguous(p,*_):
                r=self.response(json.loads(p['input'][0]['content'][0]['text'])['scene_id'])
                r['cross_view_matches'][1]['confidence']=.1;return r
            client.select.side_effect=ambiguous;target.reset_mock()
            with self.assertRaisesRegex(RuntimeError,'对应关系不明确'):
                book_geom.segment_cart_book_with_sam3(self.rgb,self.depth,self.k,self.tf,
                    head_captured_at=time.monotonic(),observation_robot=robot)
            target.assert_not_called()
            with self.assertRaisesRegex(ValueError,'没有有效书脊中心'):book_geom.fill_book_meta({},None,self.tf)

    def test_frame_size_timestamp_and_pair_gap_validation(self):
        c=dual.load_config();rgb=np.zeros((c['height'],c['width'],3),np.uint8)
        now=time.monotonic();self.assertAlmostEqual(dual.validate_frame(rgb,now-.1,now-.3,c),.2)
        cases=[(rgb,now-.1,now-5),(rgb,now+1,now),(rgb,now-.2,now-.1),
               (rgb,now-.1,float('nan')),(rgb[:10],now-.1,now-.3)]
        for args in cases:
            with self.subTest(args=args[1:]),self.assertRaises(ValueError):dual.validate_frame(*args,c)

    def test_capture_timeout_and_pause_terminate_child(self):
        for cancelled in (False,True):
            process=Mock();process.poll.return_value=None
            calls=0
            def checkpoint():
                nonlocal calls
                calls+=1
                if cancelled and calls>1:raise PauseRequested()
            with patch.object(dual.subprocess,'Popen',return_value=process), \
                 patch.object(dual.time,'monotonic',side_effect=[0.,10.]):
                with self.assertRaises(PauseRequested if cancelled else ValueError):
                    dual.capture_wrist(dual.load_config(),0.,checkpoint)
            process.terminate.assert_called_once();process.wait.assert_called_once()

    def test_actual_capture_worker_protocol_uses_fresh_output(self):
        cfg=dual.load_config();now=time.monotonic()
        def start(args,**kwargs):
            self.assertEqual(args[2],'--capture-worker')
            np.savez(args[-1],rgb=np.zeros((cfg['height'],cfg['width'],3),np.uint8),captured_at=now)
            return SimpleNamespace(poll=lambda:0,returncode=0)
        with patch.object(dual.subprocess,'Popen',side_effect=start):
            result=dual.capture_wrist(cfg,now-.2)
        self.assertAlmostEqual(result['pair_gap_seconds'],.2)


if __name__=='__main__':unittest.main()
