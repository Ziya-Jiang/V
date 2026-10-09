"""Select exactly one observed book from language; never generates robot poses.

GPT adjudicates candidate masks, using original RGB crops and fixed IDs.
Spine/depth grasp checks run ONLY after the language target is selected.
No local keyword selector or fallback is used. OPENAI_API_KEY is required.
Left/right order is robot-base +Y to -Y. GPT resolves appearance and spatial references.
"""
from pathlib import Path
import json
import uuid
import cv2
import numpy as np

from cart_book_grasp_v2_depth import whole_book_depth

COLORS = {
    'yellow': ('黄色', '黄', 'yellow'), 'red': ('红色', '红', 'red'),
    'blue': ('蓝色', '蓝', 'blue'), 'green': ('绿色', '绿', 'green'),
    'orange': ('橙色', '橙', 'orange'), 'purple': ('紫色', '紫', 'purple'),
    'pink': ('粉红色', '粉色', '粉', 'pink'), 'white': ('白色', '白', 'white'),
    'black': ('黑色', '黑', 'black'), 'gray': ('灰色', '灰', 'grey', 'gray'),
}

class SelectionError(ValueError):
    """No unique, valid target; the current grasp must not execute."""


def load_config(path):
    value = json.loads(Path(path).read_text())
    expected = {'backend', 'min_detection_score', 'duplicate_iou', 'min_book_spine_coverage',
                'max_other_spine_coverage', 'model', 'request_timeout_seconds', 'min_llm_confidence'}
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError('Invalid book-agent configuration fields')
    if value['backend'] != 'gpt' or not isinstance(value['model'], str) or not value['model'].startswith('gpt-'):
        raise ValueError('Book agent requires a GPT vision model')
    for name in expected-{'backend','model'}:
        number = value[name]
        upper = 120 if name == 'request_timeout_seconds' else 1
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not np.isfinite(number) or not 0 < number <= upper:
            raise ValueError(f'Invalid agent threshold: {name}')
    return value


def color_fractions(rgb, mask):
    # HSV is computed from RGB, never from the depth colormap or tinted overlay.
    inside = cv2.erode(np.asarray(mask, np.uint8), np.ones((3,3), np.uint8)).astype(bool)
    if inside.sum() < 30:
        inside = np.asarray(mask, bool)
    hsv = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2HSV)[inside]
    if not len(hsv):
        return {color: 0. for color in COLORS}
    h, s, v = hsv.T
    chroma = (s >= 65) & (v >= 55)
    regions = {'red': chroma & ((h < 8) | (h >= 175)),
               'orange': chroma & (h >= 8) & (h < 20),
               'yellow': chroma & (h >= 20) & (h < 38),
               'green': chroma & (h >= 38) & (h < 85),
               'blue': chroma & (h >= 85) & (h < 135),
               'purple': chroma & (h >= 135) & (h < 160),
               'pink': chroma & (h >= 160) & (h < 175),
               'white': (s < 65) & (v >= 185), 'black': v < 55,
               'gray': (s < 65) & (v >= 55) & (v < 185)}
    return {key: float(region.mean()) for key, region in regions.items()}


def distinct_masks(detections, shape, config):
    kept = []
    for score, raw, _ in sorted(detections, key=lambda d: d[0], reverse=True):
        if not np.isfinite(score) or score < config['min_detection_score']:
            continue
        mask = np.asarray(raw, dtype=np.uint8)
        if mask.ndim != 2:
            continue
        if mask.shape != shape:
            mask = cv2.resize(mask, (shape[1],shape[0]), interpolation=cv2.INTER_NEAREST)
        mask = mask.astype(bool)
        if mask.sum() < 100:
            continue
        if any(np.count_nonzero(mask & other)/np.count_nonzero(mask | other) >= config['duplicate_iou'] for _, other in kept):
            continue
        kept.append((float(score), mask))
    return kept


def observed_base_y(mask, depth, intrinsic, c2w):
    valid = mask & np.isfinite(depth) & (depth > 0)
    yy, xx = np.nonzero(valid)
    if len(xx) < 12:
        return None
    z = np.asarray(depth[yy,xx], float)
    if np.median(z) > 20: z = z*.001
    rays = np.linalg.solve(intrinsic, np.array([xx, yy, np.ones(len(xx))]))
    points = (rays/rays[2]*z).T @ np.asarray(c2w)[:3,:3].T
    translation = np.asarray(c2w)[:3,3].astype(float)
    if np.linalg.norm(translation) > 10: translation *= .001
    return float(np.median(points[:,1]+translation[1])*1000)


def recognized_pairs(spines, books, shape, config, diagnostics=None):
    """Post-selection grasp association; never filters the GPT candidate pool."""
    from cart_book_grasp_v2_perception import _spine_frame
    def reject(reason):
        if diagnostics is not None:diagnostics.append(reason)
    valid_spines=[]
    for score,mask in distinct_masks(spines,shape,config):
        try:
            cleaned=_spine_frame(mask,shape)[0]
            if cleaned.sum() < .85*mask.sum():
                reject('书脊 mask 破碎：最大连通区域不足 85%');continue
        except (ValueError,np.linalg.LinAlgError) as error:
            reason=str(error)
            if reason=='Spine is clipped by the image border':reason='书脊接触原始画面边缘，被截断'
            reject('书脊形状检查失败：'+reason);continue
        valid_spines.append((score,cleaned))
    valid_books=[]
    for score,mask in distinct_masks(books,shape,config):
        count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
        if count<2:
            reject('整书 mask 为空');continue
        component=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]))
        cleaned=labels==component
        if cleaned.sum()<.9*mask.sum():
            reject('整书 mask 破碎：最大连通区域不足 90%');continue
        yy,xx=np.nonzero(cleaned)
        _,(width,height),_=cv2.minAreaRect(np.column_stack((xx,yy)).astype(np.float32))
        if min(width,height)<4 or max(width,height)<40 or cleaned.sum()/max(1.,width*height)<.5:
            reject(f'整书形状检查失败：边长 {width:.1f}×{height:.1f} px，填充率 {cleaned.sum()/max(1.,width*height):.2f}');continue
        valid_books.append((score,cleaned))
    pairs=[]
    for score,mask in valid_spines:
        matches=[]
        for book_score,book_mask in valid_books:
            coverage=np.count_nonzero(book_mask & mask)/mask.sum()
            if coverage < config['min_book_spine_coverage']:
                reject(f'书脊与所选整书 mask 重合不足：{coverage:.0%}');continue
            if any(other is not mask and np.count_nonzero(book_mask & other)/other.sum() > config['max_other_spine_coverage']
                   for _,other in valid_spines):
                reject('整书 mask 同时覆盖多个不同书脊候选');continue
            matches.append((int(book_mask.sum()),-book_score,book_mask))
        if matches:pairs.append((score,mask,min(matches,key=lambda m:m[:2])[2]))
    return pairs


def build_candidates(rgb, depth, intrinsic, c2w, spines, books, fit_spine, config):
    shape=np.asarray(depth).shape
    if len(shape)!=2 or np.asarray(rgb).shape!=(*shape,3):
        raise SelectionError('RGB 与对齐深度尺寸不一致。')
    candidates=[]
    # Only score/area/duplicate checks here. Neither missing spines nor bad
    # depth may remove a physical book and thereby redefine 'middle'.
    book_masks=distinct_masks(books,shape,config)
    spine_masks=distinct_masks(spines,shape,config)
    proposals=[(score,mask,'book_proposal') for score,mask in book_masks]
    single_spine_books=[whole for _,whole in book_masks if sum(
        np.count_nonzero(other & whole)/other.sum() >= config['min_book_spine_coverage']
        for _,other in spine_masks)==1]
    for score,mask in spine_masks:
        if not any(np.count_nonzero(mask & whole)/mask.sum() >= config['min_book_spine_coverage']
                   for whole in single_spine_books):
            proposals.append((score,mask,'spine_proposal'))
    for score,mask,source in proposals:
        yy,xx=np.nonzero(mask)
        item=dict(mask=mask,book_mask=mask,score=score,
            uv=[float(np.median(xx)),float(np.median(yy))],
            box=[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)],
            colors=color_fractions(rgb,mask),geometry=None,error='',recognition=source,
            geometry_status='not_checked')
        try:item['base_y_mm']=observed_base_y(mask,depth,intrinsic,c2w)
        except (ValueError,np.linalg.LinAlgError):item['base_y_mm']=None
        candidates.append(item)
    candidates.sort(key=lambda c:(c['base_y_mm'] is None,
        -c['base_y_mm'] if c['base_y_mm'] is not None else c['uv'][0]))
    for index,candidate in enumerate(candidates,1):
        candidate['id']=f'book_{index}'
        candidate['color']=max(candidate['colors'],key=candidate['colors'].get)
    return candidates


def prepare_selected_geometry(candidate,spines,depth,intrinsic,c2w,fit_spine,config):
    """Validate only the selected mask; never substitute another book."""
    try:
        if candidate['recognition']!='book_proposal':
            raise ValueError('仅检出了该书的书脊候选，缺少独立整书 mask，无法计算前伸深度')
        reasons=[]
        pairs=recognized_pairs(spines,[(candidate['score'],candidate['book_mask'],candidate['box'])],
                               np.asarray(depth).shape,config,reasons)
        if len(pairs)!=1:
            raise ValueError('; '.join(dict.fromkeys(reasons)) or f'有效书脊匹配数为 {len(pairs)}，需要恰好一个')
        score,mask,book_mask=pairs[0]
        geometry=fit_spine(mask,depth,intrinsic)
        geometry.update(whole_book_depth(book_mask,depth,intrinsic,c2w),score=score)
        candidate.update(mask=mask,book_mask=book_mask,geometry=geometry,geometry_status='ready',error='')
    except (ValueError,np.linalg.LinAlgError) as error:
        candidate.update(error=str(error),geometry_status='failed')
        raise SelectionError(f"GPT 选中 {candidate['id']}，但该书不能可靠抓取：{error}；不更换目标。") from error
    return candidate


def isolated_books(rgb,candidates):
    """Keep original RGB inside all candidate masks; hide other pixels."""
    rgb=np.asarray(rgb,dtype=np.uint8)
    output=np.full_like(rgb,55)
    for candidate in candidates:
        mask=candidate['book_mask']
        output[mask]=rgb[mask]
    return output


def annotate_candidates(rgb,candidates,selected_id=None):
    output=isolated_books(rgb,candidates)
    for candidate in candidates:
        selected=candidate['id']==selected_id
        review=candidate.get('gpt_review')
        label=''
        color=(80,220,255)
        if review is not None:
            if candidate.get('verified_book'):
                label=' BOOK';color=(70,240,110)
            elif review['kind']=='uncertain' or review['confidence']<candidate.get('review_min_confidence',1):
                label=' UNCERTAIN';color=(255,170,50)
            else:
                label=' REJECT';color=(255,100,90)
        if selected:color=(255,230,0)
        contours,_=cv2.findContours(candidate['book_mask'].astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(output,contours,-1,color,3 if selected else 1)
        u,v=np.rint(candidate['uv']).astype(int)
        text=candidate['id']+(' SELECTED' if selected else label)
        x=max(0,min(u-20,output.shape[1]-120));y=max(18,min(v,output.shape[0]-8))
        cv2.putText(output,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),4)
        cv2.putText(output,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.55,color,1)
    return output


def candidate_gallery(rgb,candidates):
    """One isolated crop per candidate mask, with large nonoverlapping IDs."""
    width,height=240,320
    columns=min(4,max(1,len(candidates)))
    rows=max(1,(len(candidates)+columns-1)//columns)
    sheet=np.full((rows*height,columns*width,3),55,np.uint8)
    for index,candidate in enumerate(candidates):
        mask=candidate['book_mask'];yy,xx=np.nonzero(mask)
        crop=np.full_like(rgb,55);crop[mask]=rgb[mask]
        crop=crop[yy.min():yy.max()+1,xx.min():xx.max()+1]
        scale=min((width-20)/crop.shape[1],(height-70)/crop.shape[0])
        crop=cv2.resize(crop,(max(1,round(crop.shape[1]*scale)),max(1,round(crop.shape[0]*scale))))
        row,column=divmod(index,columns);top=row*height+55;left=column*width+(width-crop.shape[1])//2
        sheet[top:top+crop.shape[0],left:left+crop.shape[1]]=crop
        cv2.putText(sheet,candidate['id'],(column*width+15,row*height+30),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
    return sheet


def validate_candidate_reviews(response,candidates,scene_id,minimum):
    """Require complete, current-frame classification before accepting selection."""
    fields={'scene_id','candidate_reviews','status','selected_id','matching_ids','confidence','reason'}
    if not isinstance(response,dict) or set(response)!=fields or response['scene_id']!=scene_id:
        raise SelectionError('GPT 返回了无效结果或旧场景编号，取消本轮。')
    items=response['candidate_reviews'];ids={c['id'] for c in candidates};reviews={}
    if not isinstance(items,list):raise SelectionError('GPT 未逐个审核全部候选。')
    for item in items:
        if not isinstance(item,dict) or set(item)!={'id','kind','confidence','reason','duplicate_of'}:
            raise SelectionError('GPT 候选审核格式无效。')
        id=item['id'];kind=item['kind'];confidence=item['confidence']
        if not isinstance(id,str) or id not in ids or id in reviews:
            raise SelectionError('GPT 候选审核编号重复或无效。')
        if kind not in ('book','not_book','duplicate','multiple_books','uncertain'):
            raise SelectionError('GPT 候选类别无效。')
        if type(confidence) not in (int,float) or not 0<=confidence<=1:
            raise SelectionError('GPT 候选审核置信度无效。')
        if not isinstance(item['reason'],str) or not item['reason'].strip():
            raise SelectionError('GPT 未提供候选审核依据。')
        if kind!='duplicate' and item['duplicate_of'] is not None:
            raise SelectionError('GPT 候选去重关系无效。')
        reviews[id]=item
    if set(reviews)!=ids:raise SelectionError('GPT 未逐个审核全部候选，取消本轮。')
    for id,item in reviews.items():
        if item['kind']=='duplicate':
            target=item['duplicate_of']
            if not isinstance(target,str) or target==id or target not in reviews or reviews[target]['kind']!='book':
                raise SelectionError('GPT 候选去重关系无效。')
    return reviews


def validate_selection(response,candidates,scene_id,minimum):
    reviews=validate_candidate_reviews(response,candidates,scene_id,minimum)
    allowed={id for id,r in reviews.items() if r['kind']=='book' and r['confidence']>=minimum}
    if response['status'] not in ('selected','clarify','no_match') or not isinstance(response['reason'],str):
        raise SelectionError('GPT 结果格式无效。')
    if response['status']!='selected':
        raise SelectionError('GPT 未确定唯一目标，请补充描述：'+response['reason'][:500])
    confidence=response['confidence']
    if type(confidence) not in (int,float) or not minimum<=confidence<=1:
        raise SelectionError('GPT 选书置信度不足，请补充描述。')
    ids={c['id']:c for c in candidates}
    selected=response['selected_id'];matches=response['matching_ids']
    if not isinstance(selected,str) or selected not in ids or matches != [selected]:
        raise SelectionError('GPT 选书结果不唯一或编号无效，取消抓取。')
    if selected not in allowed:
        raise SelectionError('GPT 选中了未通过书本身份审核的候选，取消抓取。')
    return ids[selected]


class BookSelectionAgent:
    def __init__(self,config_path=None,client=None,checkpoint=lambda:None):
        from cart_book_grasp_v2_gpt import GPTBookClient
        self.config_path=Path(config_path or Path(__file__).with_name('cart_book_grasp_v2_agent_config.json'))
        self.client=client if client is not None else GPTBookClient()
        self.checkpoint=checkpoint;self.instruction=''
        self.last_candidates=[];self.last_decision={};self.last_image=None
        self.last_filter={};self.last_gate_image=None
        self.last_review_image=None
        self.last_wrist_view=None;self.last_wrist_image=None;self.last_wrist_candidates_image=None

    def check_ready(self):
        load_config(self.config_path)
        from cart_book_grasp_v2_candidates import load_config as load_candidates_config
        load_candidates_config()
        from cart_book_grasp_v2_instances import load_config as load_instance_config
        load_instance_config()
        from cart_book_grasp_v2_dual_view import load_config as load_dual_config
        load_dual_config()
        self.client.check_ready()

    def set_instruction(self,text):
        if not isinstance(text,str) or not text.strip() or len(text)>500:
            raise SelectionError('指令不能为空，且不得超过 500 字。')
        self.instruction=text.strip();self.last_candidates=[];self.last_decision={};self.last_image=None
        self.last_filter={};self.last_gate_image=None
        self.last_review_image=None
        self.last_wrist_view=None;self.last_wrist_image=None;self.last_wrist_candidates_image=None

    def select_boxes(self,rgb,depth,intrinsic,c2w,detector,prepare_geometry,on_recognized=None):
        from cart_book_grasp_v2_candidates import detect_candidates,load_config,annotate_gate,CandidateError
        self.last_candidates=[];self.last_decision={};self.last_image=None
        self.last_filter={};self.last_gate_image=None
        self.last_review_image=None
        try:
            candidates,self.last_filter=detect_candidates(detector,rgb,depth,intrinsic,c2w,load_config(),self.checkpoint)
        except CandidateError as error:
            self.last_filter=error.diagnostics
            self.last_gate_image=annotate_gate(rgb,self.last_filter)
            self.last_decision=dict(status='candidate_filter_failed',message=str(error))
            if on_recognized is not None:on_recognized()
            raise SelectionError(str(error)) from error
        self.last_gate_image=annotate_gate(rgb,self.last_filter)
        return self.select(rgb,depth,intrinsic,c2w,[],[],None,on_recognized,
                           candidate_data=candidates,prepare_geometry=prepare_geometry)

    def select_instances(self,rgb,depth,intrinsic,c2w,book_geom,prepare_geometry,on_recognized=None,
                         *,head_captured_at=None,dual_config=None,observation_guard=lambda:None):
        from cart_book_grasp_v2_instances import detect_instances,load_config
        from cart_book_grasp_v2_candidates import load_config as load_depth_config,annotate_gate,CandidateError
        from cart_book_grasp_v2_dual_view import load_config as load_dual_config,capture_wrist,wrist_candidates
        self.last_candidates=[];self.last_decision={};self.last_image=None;self.last_review_image=None
        self.last_filter={};self.last_gate_image=None
        self.last_wrist_view=None;self.last_wrist_image=None;self.last_wrist_candidates_image=None
        dual=load_dual_config() if dual_config is None else dual_config
        wrist_view=None
        if dual['enabled']:
            try:
                if head_captured_at is None:raise SelectionError('双视角采集缺少本轮头部图像时间戳')
                observation_guard()
                wrist_view=capture_wrist(dual,head_captured_at,self.checkpoint)
                observation_guard()
                wrist_view['min_match_confidence']=dual['min_match_confidence']
                self.last_wrist_image=wrist_view['rgb']
                print(f"双视角采集完成，头部/手部相隔 {wrist_view['pair_gap_seconds']:.2f} 秒。",flush=True)
                if on_recognized is not None:on_recognized()
            except ValueError as error:
                self.last_decision=dict(status='dual_view_capture_failed',message=str(error))
                raise
        try:
            candidates,self.last_filter=detect_instances(book_geom,rgb,depth,intrinsic,c2w,
                load_config(),load_depth_config(),self.checkpoint)
        except CandidateError as error:
            self.last_filter=error.diagnostics
            self.last_gate_image=annotate_gate(rgb,self.last_filter)
            self.last_decision=dict(status='candidate_filter_failed',message=str(error))
            if on_recognized is not None:on_recognized()
            raise SelectionError(str(error)) from error
        self.last_gate_image=annotate_gate(rgb,self.last_filter)
        if wrist_view is not None:
            try:
                wrist_view['candidates']=wrist_candidates(book_geom,wrist_view['rgb'],dual,self.checkpoint)
                self.last_wrist_view=wrist_view
                self.last_wrist_candidates_image=candidate_gallery(wrist_view['rgb'],wrist_view['candidates'])
                observation_guard()
            except ValueError as error:
                self.last_decision=dict(status='wrist_candidates_failed',message=str(error))
                raise
        return self.select(rgb,depth,intrinsic,c2w,[],[],None,on_recognized,
                           candidate_data=candidates,prepare_geometry=prepare_geometry,wrist_view=wrist_view,
                           observation_guard=observation_guard)

    def select(self,rgb,depth,intrinsic,c2w,spines,books,fit_spine,on_recognized=None,
               *,candidate_data=None,prepare_geometry=None,wrist_view=None,observation_guard=lambda:None):
        from cart_book_grasp_v2_gpt import build_payload
        self.last_candidates=[];self.last_decision={};self.last_image=None
        self.last_review_image=None
        self.last_wrist_view=wrist_view
        if wrist_view is None:
            self.last_wrist_image=None;self.last_wrist_candidates_image=None
        else:
            self.last_wrist_image=wrist_view['rgb']
            self.last_wrist_candidates_image=candidate_gallery(wrist_view['rgb'],wrist_view['candidates'])
        config=load_config(self.config_path)
        candidates=(build_candidates(rgb,depth,intrinsic,c2w,spines,books,fit_spine,config)
                    if candidate_data is None else candidate_data)
        self.last_candidates=candidates
        self.last_image=annotate_candidates(rgb,candidates)
        scene_id=uuid.uuid4().hex
        self.last_decision=dict(status='candidates_ready',scene_id=scene_id,candidate_count=len(candidates),
                               raw_book_masks=len(books),raw_spine_masks=len(spines))
        if candidate_data is None:
            print(f'阶段 1：分割得到 {len(candidates)} 个候选 mask；尚未筛选抓取几何。',flush=True)
        elif all(c.get('proposal_method')=='sam3_instances' for c in candidates):
            self.last_decision['proposal_method']='sam3_instances'
            print(f'阶段 1：SAM3 已分出 {len(candidates)} 个独立实例候选；尚未检查书脊和抓取几何。',flush=True)
        else:
            self.last_decision['proposal_method']='grounding_dino_depth_only'
            print(f'阶段 1：GroundingDINO + 深度筛选，保留 {len(candidates)} 个书框；尚未运行 SAM3。',flush=True)
        if on_recognized is not None:on_recognized()
        try:
            if not candidates:raise SelectionError('没有可用候选 mask，未调用 GPT；请调整场景后重试。')
            if not self.instruction:raise SelectionError('本轮缺少文字指令。')
            self.checkpoint()
            observation_guard()
            print(f"阶段 2：{config['model']} 先逐个审核所有候选是否为书，再从真实书本中按指令选择。",flush=True)
            if wrist_view is None:
                payload=build_payload(self.instruction,isolated_books(rgb,candidates),
                    candidate_gallery(rgb,candidates),candidates,scene_id,config)
            else:
                from cart_book_grasp_v2_dual_gpt import build_dual_payload
                payload=build_dual_payload(self.instruction,rgb,candidates,scene_id,config,wrist_view)
                print(f"双视角 GPT：头部 {len(candidates)} 个候选，手部 {len(wrist_view['candidates'])} 个候选；只返回头部抓取编号。",flush=True)
            response=self.client.select(payload,config['request_timeout_seconds'],self.checkpoint)
            self.checkpoint()
            observation_guard()
            base_response=response
            if wrist_view is not None:
                from cart_book_grasp_v2_dual_gpt import validate_dual_response
                base_response,wrist_reviews,matches=validate_dual_response(response,candidates,wrist_view,scene_id,config['min_llm_confidence'])
                for item in wrist_view['candidates']:
                    review=wrist_reviews[item['id']]
                    item.update(gpt_review=review,verified_book=review['kind']=='book' and review['confidence']>=config['min_llm_confidence'],
                                review_min_confidence=config['min_llm_confidence'])
                    print(f"GPT 手部审核 {item['id']}：{review['kind']} ({review['confidence']:.2f}) — {review['reason']}",flush=True)
                for row in matches.values():
                    print(f"GPT 跨视角 {row['head_id']} → {row['wrist_id']}：{row['status']} ({row['confidence']:.2f}) — {row['reason']}",flush=True)
                self.last_wrist_image=annotate_candidates(wrist_view['rgb'],wrist_view['candidates'])
            reviews=validate_candidate_reviews(base_response,candidates,scene_id,config['min_llm_confidence'])
            for candidate in candidates:
                review=reviews[candidate['id']]
                verified=review['kind']=='book' and review['confidence']>=config['min_llm_confidence']
                candidate.update(gpt_review=review,verified_book=verified,review_min_confidence=config['min_llm_confidence'])
                outcome='保留为书本' if verified else '排除出可选书本'
                print(f"GPT 逐项审核 {candidate['id']}：{outcome} [{review['kind']}, {review['confidence']:.2f}] — {review['reason']}",flush=True)
            verified_candidates=[c for c in candidates if c['verified_book']]
            self.last_decision=dict(response,instruction=self.instruction,model=config['model'],
                                    verified_book_ids=[c['id'] for c in verified_candidates],dual_view=wrist_view is not None)
            if wrist_view is not None:self.last_decision['pair_gap_seconds']=wrist_view['pair_gap_seconds']
            self.last_review_image=annotate_candidates(rgb,candidates)
            self.last_image=annotate_candidates(rgb,verified_candidates)
            print(f'GPT 审核完成：{len(candidates)} 个候选 → {len(verified_candidates)} 本确认书籍。',flush=True)
            selected=validate_selection(base_response,candidates,scene_id,config['min_llm_confidence'])
        except ValueError as error:
            self.last_decision.update(status='clarify',instruction=self.instruction,scene_id=scene_id,message=str(error))
            if on_recognized is not None:on_recognized()
            raise
        self.last_image=annotate_candidates(rgb,verified_candidates,selected['id'])
        print(f"GPT 选中 {selected['id']}：{response['reason']}（confidence={response['confidence']:.2f}）",flush=True)
        if on_recognized is not None:on_recognized()
        self.checkpoint()
        observation_guard()
        print(f"阶段 3：仅对 {selected['id']} 检查书脊、深度和抓取几何。",flush=True)
        try:
            if prepare_geometry is None:
                prepare_selected_geometry(selected,spines,depth,intrinsic,c2w,fit_spine,config)
            else:
                prepare_geometry(selected)
                if selected['geometry'] is None:raise SelectionError('选中书的几何计算未完成')
        except Exception as error:
            selected.update(geometry_status='failed',error=str(error))
            self.last_decision.update(status='geometry_failed',message=str(error))
            raise
        self.checkpoint()
        return selected
