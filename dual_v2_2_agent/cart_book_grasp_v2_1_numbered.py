"""Perceive, display fresh head IDs, then accept one number. No color/identity gate."""
import hashlib
import json
import re
import time
import uuid

import cv2
import numpy as np
from PIL import Image

from cart_book_grasp_v2 import InstructionCartLoop
from cart_book_grasp_v2_1_selection import (
    CONFIG_PATH, LocalBookSelectionAgent, SelectionError, build_observations,
    check_ready_pose, load_config, proposal_filter_summary,
)
from cart_book_grasp_v2_pause import PauseRequested
from cart_book_grasp_v2_1_bookends import (
    filter_config, detect_bookends, exclude_bookends, bookend_overlay, filter_summary,
)


def build_catalog(raw, rgb, depth, intrinsic, c2w, config, diagnostics=None, *, bookend_detector=None):
    """SAM3 + measured workspace; numbered by robot-base +Y to -Y."""
    report={} if diagnostics is None else diagnostics
    items=build_observations(raw,rgb,config,'head',depth,intrinsic,c2w,
                             diagnostics=report,compute_appearance=False)
    # Suppress a merged group only when two separate contained masks explain it.
    groups=[]
    for a in items:
        children=[b for b in items if b is not a and b['mask'].sum()<.75*a['mask'].sum()
                  and np.count_nonzero(a['mask']&b['mask'])/b['mask'].sum()>.85]
        if any(np.count_nonzero(b['mask']&d['mask'])/min(b['mask'].sum(),d['mask'].sum())<.1
               for i,b in enumerate(children) for d in children[i+1:]):
            groups.append(a)
    report['group_masks_removed']=len(groups)
    group_ids={id(v) for v in groups}
    items=[v for v in items if id(v) not in group_ids]
    enabled=filter_config(config)['enabled']
    ends={};end_report=dict(enabled=enabled,raw_counts={})
    report['bookends']=end_report
    if enabled and items:
        if bookend_detector is None:raise ValueError('SAM3 bookend detector is required before numbering')
        ends,detail=bookend_detector(items)
        end_report.update(detail)
    items=exclude_bookends(items,ends,config,end_report)
    if not items: raise SelectionError('头部工作区没有可编号的书本候选，请重新识别')
    if any(v.get('base_y_mm') is None or not np.isfinite(v['base_y_mm']) for v in items):
        raise SelectionError('部分候选缺少有效头部深度，无法按机器人左右编号；请重新识别')
    items.sort(key=lambda v:-v['base_y_mm'])
    for number,item in enumerate(items,1):
        item.update(id=f'book_{number}',number=number,selection_blocked='')
    for i,a in enumerate(items):
        for b in items[i+1:]:
            if np.count_nonzero(a['mask']&b['mask'])/min(a['mask'].sum(),b['mask'].sum())>config['max_overlap_fraction']:
                a['selection_blocked']=b['selection_blocked']='候选掩码重叠，无法确认独立书本'
            if abs(a['base_y_mm']-b['base_y_mm'])<config['min_head_y_gap_mm']:
                a['selection_blocked']=b['selection_blocked']='候选在机器人左右方向过近，编号顺序不明确'
    report['numbered_count']=len(items)
    return items


def select_number(text, items):
    value=str(text).strip()
    if not re.fullmatch(r'[1-9][0-9]*',value) or len(value)>6:
        raise SelectionError(f'请输入本轮显示的整数编号 1–{len(items)}')
    number=int(value)
    if number>len(items): raise SelectionError(f'编号 {number} 不在本轮范围 1–{len(items)} 内')
    target=items[number-1]
    if target['number']!=number: raise SelectionError('本轮编号与候选列表不一致，请重新识别')
    if target.get('selection_blocked'):
        raise SelectionError(f"编号 {number} 暂不可选：{target['selection_blocked']}")
    return target


def numbered_overlay(rgb, items, selected_number=None):
    out=np.asarray(rgb).copy()
    cv2.rectangle(out,(0,0),(out.shape[1],32),(25,25,25),-1)
    cv2.putText(out,'HEAD IDs: robot left -> right | enter a number',(10,22),
                cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
    for item in items:
        chosen=item['number']==selected_number
        color=(255,225,0) if chosen else (255,90,70) if item.get('selection_blocked') else (60,240,120)
        contours,_=cv2.findContours(item['book_mask'].astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out,contours,-1,color,2)
        x,y=np.rint(item['uv']).astype(int)
        label=str(item['number']);size,_=cv2.getTextSize(label,cv2.FONT_HERSHEY_SIMPLEX,.8,2)
        cv2.rectangle(out,(x-size[0]//2-6,y-size[1]-6),(x+size[0]//2+6,y+7),(20,20,20),-1)
        cv2.putText(out,label,(x-size[0]//2,y),cv2.FONT_HERSHEY_SIMPLEX,.8,color,2,cv2.LINE_AA)
    return out


def catalog_summary(items):
    return [dict(number=v['number'],id=v['id'],box=v['box'],score=v['score'],
                 base_xyz_mm=v['base_xyz_mm'],selection_blocked=v['selection_blocked']) for v in items]


class NumberedCartLoop(InstructionCartLoop):
    startup_message='V2.1 编号抓取：先识别并显示编号，再输入编号；完成或失败后回车重新识别。'

    def __init__(self,selector):
        super().__init__(selector)
        selector.number_provider=self.request_number

    def check_selection_checkpoint(self):
        self.check_pause()
        if self.stop.is_set(): raise SelectionError('已按 s 取消本轮选书，不执行抓取')

    def feed_character(self,character):
        # p/s remain immediate even while waiting for a numeric answer.
        if character.lower() in ('p','s'):
            with self._retry_condition:
                if character.lower()=='p': self.request_pause()
                else: self.request_stop()
                self._retry_condition.notify_all()
            return
        super().feed_character(character)

    def read_line(self,prompt,checkpoint):
        with self._retry_condition:
            self.pending_instruction=None;self.input_buffer=''
            print(prompt,end='',flush=True)
            self.accepting_instruction=True
            try:
                while self.pending_instruction is None:
                    self.check_selection_checkpoint();checkpoint()
                    self._retry_condition.wait(timeout=.05)
                self.check_selection_checkpoint();checkpoint()
                return self.pending_instruction
            finally:
                self.accepting_instruction=False
                self.pending_instruction=None;self.input_buffer=''

    def request_number(self,items,scene_id,checkpoint):
        while True:
            text=self.read_line(f'本轮 {scene_id[:8]}：输入编号 1–{len(items)} 后回车；p 松爪复位，s 退出。\n> ',checkpoint)
            try: return select_number(text,items)['number']
            except SelectionError as error: print(str(error),flush=True)

    def next_task(self):
        self.armed=True
        if self.stop.is_set(): return None
        self.check_pause()
        if self.cycles:
            while True:
                try:
                    line=self.read_line('\n本轮结束。按回车重新识别并编号；s 退出。\n> ',self.check_pause)
                except SelectionError:
                    if self.stop.is_set(): return None
                    raise
                if not line: break
                print('请先按回车重新识别，看到新编号后再选择；不复用旧编号。',flush=True)
        self.cycles+=1;self.last_success=None
        print(f'第 {self.cycles} 轮：先感知书本并显示编号。',flush=True)
        return 'cart+'


class NumberedBookSelectionAgent(LocalBookSelectionAgent):
    label='头部编号'

    def __init__(self,config_path=CONFIG_PATH,*,observe_only=False,output_root=None):
        super().__init__(config_path,observe_only=observe_only,output_root=output_root)
        self.number_provider=None

    def check_ready(self):
        load_config(self.config_path,require_semantic_model=False)
        from cart_book_grasp_v2_dual_view import load_config as camera_config
        camera_config()

    def warmup(self):
        # Number selection needs SAM3 only; no SigLIP weights or text encoder.
        pass

    def bind_observation_robot(self,robot):
        check_ready_pose(robot,load_config(self.config_path,require_semantic_model=False))
        self.robot=robot

    def select_instances(self,rgb,depth,intrinsic,c2w,book_geom,prepare_geometry,
                         on_recognized=None,*,head_captured_at=None,dual_config=None,observation_guard=lambda:None):
        from cart_book_grasp_v2_dual_view import load_config as camera_config,capture_wrist
        self._reset();self.instruction=''
        c=load_config(self.config_path,require_semantic_model=False)
        dual=camera_config() if dual_config is None else dual_config
        pose=check_ready_pose(self.robot,c)
        if head_captured_at is None or not np.isfinite(head_captured_at):
            raise SelectionError('缺少本轮头部采集时间')
        scene_id=uuid.uuid4().hex;output=self.output_root/scene_id
        output.mkdir(parents=True,exist_ok=False)
        started=time.monotonic();timings={}
        self.last_decision=dict(scene_id=scene_id,status='observing',selection_mode='number',
            pose=pose,gpt_called=False,semantic_model_called=False,color_gate_used=False,
            cross_view_identity_required=False,wrist_role='context_only',output=str(output))
        def checkpoint():
            self.checkpoint();observation_guard();check_ready_pose(self.robot,c)
            age=time.monotonic()-head_captured_at
            if age<0 or age>c['max_observation_age_seconds']:
                raise SelectionError(f'本轮编号图像已过期（{age:.1f}s）；按回车重新识别后再选编号')
        def publish():
            if on_recognized is not None:on_recognized()
        def save_decision():
            self.last_decision.update(timings=timings,config=c)
            (output/'decision.json').write_text(json.dumps(self.last_decision,indent=2,ensure_ascii=False))
        try:
            checkpoint();view=None
            if dual['enabled']:
                view=capture_wrist(dual,head_captured_at,self.checkpoint)
                self.last_wrist_view=view;self.last_wrist_image=view['rgb']
                Image.fromarray(view['rgb']).save(output/'wrist_rgb.png')
            checkpoint();timings['capture_wrist_s']=time.monotonic()-started
            Image.fromarray(rgb).save(output/'head_rgb.png');np.save(output/'depth.npy',depth)
            (output/'camera.json').write_text(json.dumps(dict(K=np.asarray(intrinsic).tolist(),
                c2w=np.asarray(c2w).tolist(),pair_gap_seconds=view['pair_gap_seconds'] if view else None,
                pose_verified=True,pose=pose,wrist_camera=dual),indent=2))
            processor,to_numpy,image_cls=book_geom._load_sam3_detector()
            checkpoint();t=time.monotonic()
            raw=book_geom._sam3_raw_books(processor,to_numpy,image_cls.fromarray(rgb),prompt='book')
            checkpoint();timings['head_sam3_s']=time.monotonic()-t
            np.savez_compressed(output/'head_raw.npz',frame_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),
                scores=np.array([v[0] for v in raw]),masks=np.array([v[1] for v in raw]),boxes=np.array([v[2] for v in raw]))
            self.last_filter=dict(head={});self.last_decision['proposal_filter']=self.last_filter
            end_results={}
            def detect_ends(candidates):
                nonlocal end_results
                t=time.monotonic()
                end_results,detail=detect_bookends(rgb,candidates,c,
                    lambda:(processor,to_numpy,image_cls),checkpoint=checkpoint,cache_dir=output)
                timings['head_bookend_s']=time.monotonic()-t
                return end_results,detail
            try:
                items=build_catalog(raw,rgb,depth,intrinsic,c2w,c,self.last_filter['head'],
                                    bookend_detector=detect_ends)
            finally:
                end_report=self.last_filter['head'].get('bookends',{})
                self.last_review_image=bookend_overlay(rgb,[],end_results,end_report)
                Image.fromarray(self.last_review_image).save(output/'head_bookend_filter.png')
            self.last_candidates=items
            self.last_decision.update(status='awaiting_number',catalog=catalog_summary(items))
            self.last_image=numbered_overlay(rgb,items)
            self.last_gate_image=self.last_image
            self.last_review_image=bookend_overlay(rgb,items,end_results,end_report)
            Image.fromarray(self.last_review_image).save(output/'head_bookend_filter.png')
            Image.fromarray(self.last_image).save(output/'head_numbered.png')
            print(proposal_filter_summary('head',self.last_filter['head']),flush=True)
            print(filter_summary(end_report),flush=True)
            print(f'已按机器人视角从左到右编号 1–{len(items)}。查看 Rerun 或 {output}/head_numbered.png',flush=True)
            remaining=max(0.,c['max_observation_age_seconds']-(time.monotonic()-head_captured_at))
            print(f'本轮图像剩余有效时间约 {remaining:.0f} 秒；超时后回车重新识别，不复用旧编号。',flush=True)
            save_decision();publish();checkpoint()
            if self.number_provider is None: raise SelectionError('编号输入接口尚未连接；未执行抓取')
            t=time.monotonic();number=self.number_provider(items,scene_id,checkpoint)
            timings['number_input_s']=time.monotonic()-t
            checkpoint();selected=select_number(number,items)
            self.last_decision.update(status='selected',selected_id=selected['id'],selected_number=selected['number'])
            self.last_image=numbered_overlay(rgb,items,selected['number']);self.last_gate_image=self.last_image
            Image.fromarray(self.last_image).save(output/'head_selected.png')
            save_decision();publish()
            if self.observe_only:
                self.last_decision['status']='observed_only'
                raise SelectionError('仅观测选中编号完成；未计算抓取几何，未执行抓取')
            checkpoint();t=time.monotonic()
            try:
                prepare_geometry(selected)
                if selected.get('geometry') is None: raise SelectionError('头部目标几何未完成')
            except Exception:
                self.last_decision['status']='geometry_failed';raise
            timings['head_geometry_s']=time.monotonic()-t
            checkpoint();return selected
        except PauseRequested:
            self.last_decision['status']='interrupted';raise
        except Exception as error:
            if self.last_decision['status'] not in ('observed_only','geometry_failed'):
                self.last_decision['status']='rejected'
            self.last_decision['error']=str(error);raise
        finally:
            timings['total_s']=time.monotonic()-started
            save_decision();publish()
