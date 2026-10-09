"""Mock GPT, synthetic books and pseudo-terminal only: no network or devices."""
import json
import os
import pty
import sys
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch
import numpy as np
import cart_book_grasp_v2 as entry
import cart_book_grasp_v2_agent as agent
import cart_book_grasp_v2_motion as motion
from cart_book_grasp_v2_gpt import GPTBookClient,GPTSelectionError,build_payload
from cart_book_grasp_v2_pause import PauseRequested


class GPTBooksTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.full((100,180,3),80,np.uint8)
        self.depth=np.full((100,180),800.)
        self.k=np.array([[200.,0,90],[0,200.,50],[0,0,1.]])
        self.c2w=np.eye(4);self.c2w[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        self.spines=[];self.books=[]
        for index,color in enumerate(((255,30,30),(255,235,10),(30,60,255))):
            x=20+index*55
            spine=np.zeros((100,180),bool);spine[20:80,x:x+10]=True
            book=np.zeros_like(spine);book[15:85,x-5:x+20]=True
            self.rgb[book]=color
            self.spines.append((.9,spine,[x,20,x+10,80]));self.books.append((.9,book,[x-5,15,x+20,85]))
        self.config=agent.load_config('cart_book_grasp_v2_agent_config.json')
        def fit(mask,depth,k):
            yy,xx=np.nonzero(mask)
            return dict(mask=mask,box=np.array([xx.min(),yy.min(),xx.max()+1,yy.max()+1]),uv=[float(np.median(xx)),float(np.median(yy))],
                point_camera_m=np.array([0,0,.8]),depth_m=.8,depth_samples=100,depth_p10_p90_mm=[800,800])
        self.fit=fit

    def candidates(self,spines=None,books=None):
        return agent.build_candidates(self.rgb,self.depth,self.k,self.c2w,self.spines if spines is None else spines,
                                      self.books if books is None else books,self.fit,self.config)

    def response(self,scene='scene',id='book_2',**kwargs):
        return dict(scene_id=scene,status='selected',selected_id=id,matching_ids=[id],confidence=.95,reason='The middle book is yellow.',**kwargs)

    def prepare(self,candidate,spines=None):
        return agent.prepare_selected_geometry(candidate,self.spines if spines is None else spines,
            self.depth,self.k,self.c2w,self.fit,self.config)

    def test_three_books_are_distinct_robot_left_to_right_and_geometry_is_deferred(self):
        candidates=self.candidates()
        self.assertEqual([c['id'] for c in candidates],['book_1','book_2','book_3'])
        self.assertEqual([c['color'] for c in candidates],['red','yellow','blue'])
        self.assertTrue(all(c['geometry'] is None for c in candidates))
        self.prepare(candidates[1])
        self.assertIsNotNone(candidates[1]['geometry'])
        self.assertIsNone(candidates[0]['geometry'])
        self.assertIsNone(candidates[2]['geometry'])
        self.assertGreater(candidates[0]['base_y_mm'],candidates[-1]['base_y_mm'])
        self.assertEqual(len(self.candidates(spines=self.spines+[self.spines[0]])),3)

    def test_merged_full_book_mask_is_not_used_for_selected_book_depth(self):
        merged=np.logical_or.reduce([d[1] for d in self.books])
        candidates=self.candidates(books=[(.99,merged,[0,0,180,100])])
        grouped=next(c for c in candidates if c['recognition']=='book_proposal')
        self.assertEqual(len(candidates),4)  # retain three spine-only proposals too
        with self.assertRaises(agent.SelectionError):self.prepare(grouped)

    def test_missing_spine_does_not_remove_middle_book_before_gpt(self):
        candidates=self.candidates(spines=[self.spines[0],self.spines[2]])
        self.assertEqual(len(candidates),3)
        self.assertEqual([c['color'] for c in candidates],['red','yellow','blue'])
        selected=agent.validate_selection(self.response(),candidates,'scene',.85)
        with self.assertRaisesRegex(agent.SelectionError,'不更换目标'):
            self.prepare(selected,spines=[self.spines[0],self.spines[2]])

    def test_recognized_book_with_bad_depth_is_not_replaced(self):
        self.depth[self.books[1][1]]=0
        candidates=self.candidates()
        self.assertEqual(len(candidates),3)
        yellow=next(c for c in candidates if c['color']=='yellow')
        self.assertIsNone(yellow['geometry'])
        selected=agent.validate_selection(self.response(id=yellow['id']),candidates,'scene',.85)
        with self.assertRaisesRegex(agent.SelectionError,'不更换目标'):self.prepare(selected)

    def test_proposals_include_clutter_low_score_and_unpaired_masks_for_gpt_adjudication(self):
        clutter=np.zeros_like(self.depth,bool);clutter[5:18,60:73]=True
        low=np.zeros_like(self.depth,bool);low[25:75,105:115]=True
        spines=self.spines+[(.95,clutter,[60,5,73,18]),(.15,low,[105,25,115,75])]
        books=self.books+[(.99,clutter,[60,5,73,18]),(.99,low,[105,25,115,75])]
        self.assertEqual(len(self.candidates(spines=spines,books=books)),5)
        spine_only=self.candidates(books=[])
        self.assertEqual(len(spine_only),3)
        with self.assertRaisesRegex(agent.SelectionError,'缺少独立整书'):self.prepare(spine_only[1])

    def test_recognition_is_published_before_gpt_and_payload_hides_background(self):
        import base64,cv2
        events=[];client=Mock();selector=agent.BookSelectionAgent(client=client)
        selector.set_instruction('给我黄色的书')
        self.rgb[0:8]=[0,255,0]  # conspicuous non-book object in the scene
        def select(payload,*args):
            events.append('gpt')
            self.assertEqual(events,['recognized','gpt'])
            content=payload['input'][0]['content']
            encoded=content[1]['image_url'].split(',',1)[1]
            rgb=cv2.cvtColor(cv2.imdecode(np.frombuffer(base64.b64decode(encoded),np.uint8),cv2.IMREAD_COLOR),cv2.COLOR_BGR2RGB)
            union=np.logical_or.reduce([d[1] for d in self.books])
            np.testing.assert_equal(rgb[~union],55)
            np.testing.assert_equal(rgb[union],self.rgb[union])
            self.assertIn('CANDIDATE MASKS, not verified books',payload['instructions'])
            self.assertTrue(all(c['geometry'] is None for c in selector.last_candidates))
            scene=json.loads(content[0]['text'])['scene_id']
            return self.response(scene)
        client.select.side_effect=select
        def recognized():
            status=selector.last_decision['status']
            events.append('recognized' if status=='candidates_ready' else status)
            self.assertIn(status,('candidates_ready','selected'))
            self.assertEqual(len(selector.last_candidates),3)
        def fit(*args):
            events.append('geometry')
            self.assertEqual(events,['recognized','gpt','selected','geometry'])
            return self.fit(*args)
        selector.select(self.rgb,self.depth,self.k,self.c2w,self.spines,self.books,fit,on_recognized=recognized)
        self.assertEqual(events,['recognized','gpt','selected','geometry'])

    def test_no_verified_book_does_not_call_gpt(self):
        client=Mock();selector=agent.BookSelectionAgent(client=client);selector.set_instruction('给我中间的书')
        with self.assertRaisesRegex(agent.SelectionError,'未调用 GPT'):
            selector.select(self.rgb,self.depth,self.k,self.c2w,[],[],self.fit)
        client.select.assert_not_called()

    def test_gpt_is_called_for_every_instruction_with_two_images_and_current_ids(self):
        client=Mock()
        def select(payload,timeout,checkpoint):
            checkpoint()
            content=payload['input'][0]['content'];data=json.loads(content[0]['text'])
            self.assertEqual(data['instruction'],'给我黄色的书')
            self.assertEqual([p['type'] for p in content],['input_text','input_image','input_image'])
            self.assertTrue(content[1]['image_url'].startswith('data:image/png;base64,'))
            self.assertFalse(payload['store'])
            self.assertTrue(payload['text']['format']['strict'])
            return self.response(data['scene_id'])
        client.select.side_effect=select
        selector=agent.BookSelectionAgent(client=client);selector.set_instruction('给我黄色的书')
        selected=selector.select(self.rgb,self.depth,self.k,self.c2w,self.spines,self.books,self.fit)
        self.assertEqual(selected['id'],'book_2');client.select.assert_called_once()
        self.assertEqual(selector.last_decision['model'],'gpt-6-astra')

    def test_unknown_stale_ambiguous_low_confidence_and_nonfinite_outputs_reject(self):
        candidates=self.candidates()
        mutations=[{'scene_id':'old'},{'selected_id':'book_999'},{'matching_ids':['book_1','book_2']},
                   {'confidence':.2},{'confidence':float('nan')},{'confidence':True},
                   {'status':'clarify','selected_id':None},{'status':'no_match','selected_id':None}]
        for mutation in mutations:
            response=self.response();response.update(mutation)
            with self.subTest(mutation=mutation),self.assertRaises(agent.SelectionError):
                agent.validate_selection(response,candidates,'scene',.85)

    def test_pinyin_and_appearance_are_adjudicated_by_gpt6_without_hsv_hints(self):
        payload=build_payload('geiwozhongjianzongsedeshu',self.rgb,self.rgb,self.candidates(),'scene',self.config)
        data=json.loads(payload['input'][0]['content'][0]['text'])
        self.assertEqual(data['instruction'],'geiwozhongjianzongsedeshu')
        self.assertEqual(payload['model'],'gpt-6-astra')
        self.assertEqual(payload['reasoning'],{'effort':'low'})
        self.assertGreaterEqual(payload['max_output_tokens'],4000)
        self.assertIn('zongse means brown',payload['instructions'])
        self.assertTrue(all('detected_color_hint' not in c and 'geometry_available' not in c
                            for c in data['candidates']))

    def test_gpt_sees_missing_spine_book_then_selected_geometry_failure_retains_target(self):
        client=Mock();selector=agent.BookSelectionAgent(client=client)
        selector.set_instruction('给我中间的书');fit=Mock(side_effect=self.fit)
        def select(payload,*args):
            fit.assert_not_called()
            data=json.loads(payload['input'][0]['content'][0]['text'])
            self.assertEqual(len(data['candidates']),3)
            return self.response(data['scene_id'])
        client.select.side_effect=select
        with self.assertRaisesRegex(agent.SelectionError,'不更换目标'):
            selector.select(self.rgb,self.depth,self.k,self.c2w,[self.spines[0],self.spines[2]],self.books,fit)
        client.select.assert_called_once();fit.assert_not_called()
        self.assertEqual(selector.last_decision['status'],'geometry_failed')
        self.assertEqual(selector.last_decision['selected_id'],'book_2')
        self.assertEqual(selector.last_candidates[1]['geometry_status'],'failed')

    def test_tilt_recall_continues_after_first_view_has_a_spine(self):
        import cart_book_grasp_v2_perception as perception
        raw=Mock(side_effect=[[self.spines[0]],[],[],[self.spines[1]]])
        geometry=types.SimpleNamespace(_sam3_raw_books=raw,
            _load_sam3_detector=lambda:(None,None,types.SimpleNamespace(fromarray=lambda x:x)))
        with patch.object(perception,'_restore_detections',side_effect=lambda ds,*_:ds):
            result=perception._TiltDetector(geometry)._sam3_raw_books(None,None,self.rgb,prompt='book spine')
        self.assertEqual(raw.call_count,4)
        self.assertEqual(len(result),2)
        np.testing.assert_equal(result[1][1],self.spines[1][1])

    def test_http_error_is_sanitized_and_missing_key_prevents_robot_initialization(self):
        import urllib.error
        with patch.dict(os.environ,{'OPENAI_API_KEY':'test-secret'}), \
             patch('urllib.request.build_opener') as opener:
            opener.return_value.open.side_effect=urllib.error.HTTPError('url',401,'test-secret',{},None)
            with self.assertRaises(GPTSelectionError) as error:GPTBookClient()._request({},1)
            self.assertNotIn('test-secret',str(error.exception))
        with patch.dict(os.environ,{},clear=True),patch.object(entry,'stop_key_listener') as terminal:
            with self.assertRaisesRegex(GPTSelectionError,'OPENAI_API_KEY'):entry.main()
            terminal.assert_not_called()

    def test_responses_http_contract_and_connection_check_have_no_robot_import(self):
        raw=dict(status='completed',output=[dict(type='message',content=[dict(type='output_text',text='{"ready":true}')])])
        with patch.dict(os.environ,{'OPENAI_API_KEY':'test-secret'}),patch('urllib.request.build_opener') as opener:
            opener.return_value.open.return_value.__enter__.return_value.read.return_value=json.dumps(raw).encode()
            self.assertEqual(GPTBookClient()._request({'model':'gpt-4.1'},3),{'ready':True})
            request=opener.return_value.open.call_args.args[0]
            self.assertEqual(request.full_url,'https://api.openai.com/v1/responses')
            self.assertEqual(request.get_method(),'POST')
            self.assertEqual(request.get_header('Authorization'),'Bearer test-secret')
            self.assertNotIn('test-secret',request.data.decode())
        with patch('cart_book_grasp_v2_gpt.GPTBookClient') as client:
            client.return_value.select.return_value={'ready':True}
            before=set(sys.modules)
            entry.check_gpt_connection()
            self.assertNotIn('dual_v2_2_agent_pcl',set(sys.modules)-before)
            self.assertFalse(client.return_value.select.call_args.args[0]['store'])

    def test_pending_p_cancels_gpt_wait_and_late_response_has_no_effect(self):
        client=GPTBookClient();release=threading.Event();called=threading.Event()
        def request(*args):called.set();release.wait(1);return {'late':True}
        def checkpoint():raise PauseRequested()
        with patch.dict(os.environ,{'OPENAI_API_KEY':'test-secret'}),patch.object(client,'_request',side_effect=request):
            with self.assertRaises(PauseRequested):client.select({},1,checkpoint)
            self.assertTrue(called.wait(1));release.set()

    def test_motion_adapter_uses_gpt_selected_mask_and_renders_even_on_ambiguity(self):
        client=Mock()
        client.select.side_effect=lambda payload,*_:self.response(json.loads(payload['input'][0]['content'][0]['text'])['scene_id'])
        selector=agent.BookSelectionAgent(client=client);selector.set_instruction('给我中间的书')
        geometry=types.SimpleNamespace(fill_book_meta=Mock(),BOOK_GRASP_MODE='cart+',
            _load_sam3_detector=lambda:(None,None,types.SimpleNamespace(fromarray=lambda x:x)),
            _sam3_raw_books=lambda *a,prompt:self.spines if prompt=='book spine' else self.books)
        display=types.SimpleNamespace(safe=Mock(),capture=Mock())
        with patch.object(motion,'_SELECTION_AGENT',selector),patch.object(motion,'_spine_center_from_mask',self.fit):
            motion._install_spine_center(geometry,display)
            mask,box=geometry.segment_cart_book_with_sam3(self.rgb,self.depth,self.k,self.c2w)
            np.testing.assert_equal(mask,self.spines[1][1])
            self.assertEqual(display.capture.call_args.args[4]['selection_id'],'book_2')
            self.assertTrue(any(c.args[:2]==('log_image','book/images/candidates') for c in display.safe.call_args_list))
            client.select.side_effect=lambda payload,*_:dict(self.response(json.loads(payload['input'][0]['content'][0]['text'])['scene_id']),status='clarify',selected_id=None)
            with self.assertRaisesRegex(RuntimeError,'选书或目标几何检查未完成'):
                geometry.segment_cart_book_with_sam3(self.rgb,self.depth,self.k,self.c2w)
            self.assertIn('唯一目标',geometry.BOOK_SELECTION_NOTE)
            with self.assertRaisesRegex(ValueError,'没有有效书脊中心'):geometry.fill_book_meta({},None,self.c2w)


class LanguageLoopTests(unittest.TestCase):
    def wait_prompt(self,loop):
        deadline=time.monotonic()+1
        while not loop.accepting_instruction and time.monotonic()<deadline:time.sleep(.005)
        self.assertTrue(loop.accepting_instruction)

    def test_one_instruction_one_cycle_and_success_does_not_autorepeat(self):
        selector=Mock();loop=entry.InstructionCartLoop(selector);output=[]
        thread=threading.Thread(target=lambda:output.append(loop.next_task()),daemon=True);thread.start();self.wait_prompt(loop)
        for ch in '给我黄色的书\r':loop.feed_character(ch)
        thread.join(1);self.assertEqual(output,['cart+']);selector.set_instruction.assert_called_once_with('给我黄色的书')
        loop.record_result(('book',True));output.clear()
        thread=threading.Thread(target=lambda:output.append(loop.next_task()),daemon=True);thread.start();self.wait_prompt(loop)
        loop.feed_character('\r');self.wait_prompt(loop)
        self.assertEqual(loop.cycles,1)
        for ch in 's\r':loop.feed_character(ch)
        thread.join(1);self.assertEqual(output,[None])

    def test_failure_enter_retries_same_instruction(self):
        selector=Mock();loop=entry.InstructionCartLoop(selector)
        loop.previous_instruction='给我中间的书';loop.cycles=1;loop.last_success=False
        output=[];thread=threading.Thread(target=lambda:output.append(loop.next_task()),daemon=True)
        thread.start();self.wait_prompt(loop);loop.feed_character('\r');thread.join(1)
        selector.set_instruction.assert_called_once_with('给我中间的书');self.assertEqual(output,['cart+'])

    def test_utf8_terminal_and_english_p_s_are_not_hotkeys_at_prompt(self):
        selector=Mock();loop=entry.InstructionCartLoop(selector);master,slave=pty.openpty();output=[]
        thread=threading.Thread(target=lambda:output.append(loop.next_task()),daemon=True)
        try:
            with entry.stop_key_listener(loop,types.SimpleNamespace(isatty=lambda:True,fileno=lambda:slave)):
                thread.start();self.wait_prompt(loop)
                command='please 给我黄色的书'
                encoded=command.encode('utf-8')
                os.write(master,encoded[:9]);os.write(master,encoded[9:]+b'\r')
                thread.join(1);self.assertEqual(output,['cart+'])
                selector.set_instruction.assert_called_once_with(command)
                self.assertFalse(loop.pause.is_set());self.assertFalse(loop.stop.is_set())
                os.write(master,b'p');self.assertTrue(loop.pause.wait(1))
        finally:
            loop.request_stop();thread.join(1);os.close(master);os.close(slave)

if __name__=='__main__':unittest.main()
