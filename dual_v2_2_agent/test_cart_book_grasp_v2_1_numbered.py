"""Number input, displayed-scene binding and head-only geometry; no hardware."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch

import numpy as np
from PIL import Image

import cart_book_grasp_v2_1_numbered as numbered
from cart_book_grasp_v2_1_selection import SelectionError,load_config
from cart_book_grasp_v2_dual_view import load_config as camera_config
from cart_book_grasp_v2_pause import PauseRequested


class NumberedTests(unittest.TestCase):
    def setUp(self):
        self.c=load_config(require_semantic_model=False)
        self.rgb=np.full((240,360,3),100,np.uint8)
        self.depth=np.full((240,360),800.,np.float32)
        self.k=np.array([[300.,0,180],[0,300,120],[0,0,1]])
        self.tf=np.eye(4);self.tf[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        self.raw=[]
        for i in range(3):
            mask=np.zeros((240,360),bool);mask[45:195,30+110*i:90+110*i]=True
            self.raw.append((.4,mask,[30+110*i,45,90+110*i,195]))
        self.robot=SimpleNamespace(right_safe_joint_deg=np.zeros(7),
            get_current_state=lambda:{'joint_state':np.zeros((2,7))})

    def catalog(self,raw=None,bookends=None):
        return numbered.build_catalog(self.raw if raw is None else raw,self.rgb,self.depth,self.k,self.tf,self.c,
            bookend_detector=lambda _:({'bookend':bookends or []},dict(enabled=True)))

    def test_bookend_is_removed_before_numbers_without_changing_book_masks(self):
        end=np.zeros(self.rgb.shape[:2],bool);end[45:195,8:20]=True
        items=self.catalog([(.9,end,[8,45,20,195])]+self.raw,[(.6,end,[8,45,20,195])])
        self.assertEqual([v['number'] for v in items],[1,2,3])
        for item,(_,mask,_) in zip(items,self.raw):np.testing.assert_array_equal(item['book_mask'],mask)

    def test_enabled_filter_cannot_be_silently_skipped(self):
        with self.assertRaisesRegex(ValueError,'bookend detector'):
            numbered.build_catalog(self.raw,self.rgb,self.depth,self.k,self.tf,self.c)

    def test_color_and_detector_order_do_not_affect_numbers(self):
        with patch('cart_book_grasp_v2_1_selection.color_fractions',side_effect=AssertionError('color gate')):
            first=self.catalog(list(reversed(self.raw)))
            self.rgb[:]=[230,10,100]
            second=self.catalog()
        self.assertEqual([v['number'] for v in first],[1,2,3])
        for a,b in zip(first,second):np.testing.assert_array_equal(a['book_mask'],b['book_mask'])
        np.testing.assert_array_equal(numbered.select_number('2',first)['book_mask'],self.raw[1][1])

    def test_robot_base_order_can_differ_from_image_order(self):
        self.tf[1,:3]*=-1;self.tf[2,:3]*=-1
        items=self.catalog()
        np.testing.assert_array_equal(items[0]['book_mask'],self.raw[2][1])
        self.assertGreater(items[0]['base_y_mm'],items[2]['base_y_mm'])

    def test_invalid_numbers_never_select_a_nearby_target(self):
        items=self.catalog()
        for value in ('','0','-1','4','2.0','red book','middle','第2本','2 3',None):
            with self.subTest(value=value),self.assertRaises(SelectionError):numbered.select_number(value,items)

    def test_missing_depth_does_not_shift_numbering(self):
        self.depth[self.raw[0][1]]=0
        with self.assertRaisesRegex(SelectionError,'左右编号'):self.catalog()

    def test_group_mask_removed_and_overlapping_target_blocked(self):
        group=np.logical_or.reduce([v[1] for v in self.raw]);group[60:180,30:310]=True
        items=self.catalog([(.95,group,[])]+self.raw)
        self.assertEqual(len(items),3)
        overlap=self.raw[0][1].copy();overlap[45:195,90:130]=True
        # Neither hypothesis is a two-book group, so don't silently choose one.
        items=self.catalog(self.raw+[(.9,overlap,[])])
        blocked=[v for v in items if v['selection_blocked']]
        self.assertTrue(blocked)
        with self.assertRaisesRegex(SelectionError,'暂不可选'):
            numbered.select_number(blocked[0]['number'],items)

    def run_selection(self,path,provider,*,observe_only=False,geometry_error=False,clock=None):
        selector=numbered.NumberedBookSelectionAgent(output_root=path,observe_only=observe_only)
        selector.bind_observation_robot(self.robot);selector.number_provider=provider
        geometry=Mock()
        def prepare(item):
            if geometry_error:raise ValueError('spine failed')
            item['geometry']={'head_only':True}
        geometry.side_effect=prepare
        detection=Mock(return_value=self.raw)
        geom=SimpleNamespace(_load_sam3_detector=lambda:(None,None,Image),_sam3_raw_books=detection)
        view=dict(rgb=self.rgb.copy(),pair_gap_seconds=.1)
        events=[]
        def publish():events.append(selector.last_decision['status'])
        with patch('cart_book_grasp_v2_dual_view.capture_wrist',return_value=view), \
             patch.object(numbered,'detect_bookends',return_value=({},dict(enabled=True,raw_counts={}))), \
             patch('cart_book_grasp_v2_1_selection.match_views',side_effect=AssertionError('identity gate')), \
             patch('cart_book_grasp_v2_1_selection.color_fractions',side_effect=AssertionError('color gate')), \
             patch('cart_book_grasp_v2_1_semantics.get_model',side_effect=AssertionError('semantic model')):
            error=None;selected=None
            try:
                selected=selector.select_instances(self.rgb,self.depth,self.k,self.tf,geom,geometry,publish,
                    head_captured_at=time.monotonic(),dual_config=camera_config())
            except (Exception,PauseRequested) as exc:error=exc
        return selector,selected,error,geometry,detection,events

    def test_preview_is_saved_before_input_and_only_chosen_head_reaches_geometry(self):
        with tempfile.TemporaryDirectory() as path:
            def provider(items,scene,checkpoint):
                output=Path(path)/scene
                self.assertTrue((output/'head_numbered.png').exists())
                doc=json.loads((output/'decision.json').read_text())
                self.assertEqual(doc['status'],'awaiting_number')
                self.assertEqual([v['number'] for v in doc['catalog']],[1,2,3])
                return 2
            selector,selected,error,geometry,detection,events=self.run_selection(path,provider)
            if error:raise error
            self.assertEqual(selected['number'],2)
            geometry.assert_called_once_with(selected)
            np.testing.assert_array_equal(selected['book_mask'],self.raw[1][1])
            detection.assert_called_once()
            self.assertEqual(events[0],'awaiting_number')
            self.assertIsNone(selector.last_decision.get('selected_wrist_id'))

    def test_observe_only_and_invalid_input_do_not_plan(self):
        for provider,observe in ((lambda *a:2,True),(lambda *a:7,False),(None,False)):
            with self.subTest(observe=observe),tempfile.TemporaryDirectory() as path:
                selector,selected,error,geometry,_,_=self.run_selection(path,provider,observe_only=observe)
                self.assertIsInstance(error,SelectionError);geometry.assert_not_called()

    def test_pause_and_stale_number_never_reach_geometry(self):
        with tempfile.TemporaryDirectory() as path:
            def pause(*args):raise PauseRequested()
            selector,_,error,geometry,_,_=self.run_selection(path,pause)
            self.assertIsInstance(error,PauseRequested);geometry.assert_not_called()
            self.assertEqual(selector.last_decision['status'],'interrupted')
        clock=[100.]
        with tempfile.TemporaryDirectory() as path,patch.object(numbered.time,'monotonic',side_effect=lambda:clock[0]):
            def stale(*args):clock[0]+=31;return 2
            _,_,error,geometry,_,_=self.run_selection(path,stale)
            self.assertIn('过期',str(error));geometry.assert_not_called()

    def test_geometry_failure_does_not_change_selected_number(self):
        with tempfile.TemporaryDirectory() as path:
            selector,_,error,geometry,_,_=self.run_selection(path,lambda *a:2,geometry_error=True)
            self.assertEqual(str(error),'spine failed')
            geometry.assert_called_once()
            self.assertEqual(selector.last_decision['selected_number'],2)
            self.assertEqual(selector.last_decision['status'],'geometry_failed')

    def test_numbered_startup_does_not_require_semantic_weights(self):
        with tempfile.TemporaryDirectory() as path:
            config=dict(self.c,semantic_model_path='/missing/not_required')
            filename=Path(path)/'config.json';filename.write_text(json.dumps(config))
            selector=numbered.NumberedBookSelectionAgent(filename)
            selector.check_ready();selector.warmup()
            self.assertIsNone(selector.semantic_model)

    def wait_prompt(self,loop):
        limit=time.monotonic()+2
        while not loop.accepting_instruction and time.monotonic()<limit:time.sleep(.005)
        self.assertTrue(loop.accepting_instruction)

    def test_loop_perceives_before_number_and_does_not_reuse_number(self):
        loop=numbered.NumberedCartLoop(SimpleNamespace())
        self.assertEqual(loop.next_task(),'cart+')
        self.assertFalse(loop.accepting_instruction)
        result=[]
        thread=threading.Thread(target=lambda:result.append(loop.request_number(self.catalog(),'scene',lambda:None)))
        thread.start();self.wait_prompt(loop)
        for char in '2\n':loop.feed_character(char)
        thread.join(timeout=2);self.assertFalse(thread.is_alive());self.assertEqual(result,[2])
        tasks=[];thread=threading.Thread(target=lambda:tasks.append(loop.next_task()))
        thread.start();self.wait_prompt(loop)
        self.assertEqual(tasks,[])
        loop.feed_character('\n');thread.join(timeout=2)
        self.assertFalse(thread.is_alive());self.assertEqual(tasks,['cart+'])
        self.assertIsNone(loop.pending_instruction)

    def test_stop_and_pause_while_waiting_for_number(self):
        for key,kind in (('s',SelectionError),('p',PauseRequested)):
            with self.subTest(key=key):
                loop=numbered.NumberedCartLoop(SimpleNamespace());errors=[]
                def run():
                    try:loop.request_number(self.catalog(),'scene',lambda:None)
                    except (Exception,PauseRequested) as error:errors.append(error)
                thread=threading.Thread(target=run);thread.start();self.wait_prompt(loop)
                loop.feed_character(key);thread.join(timeout=2)
                self.assertFalse(thread.is_alive());self.assertIsInstance(errors[0],kind)
                self.assertFalse(loop.accepting_instruction)


if __name__=='__main__':unittest.main()
