"""Select exactly one observed book from language; never generates robot poses.

GPT only selects among recognized books, using isolated crops and fixed IDs.
No local keyword selector or fallback is used. OPENAI_API_KEY is required.
Left/right order is robot-base +Y to -Y. Color filters precede positional rank.
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


def recognized_pairs(spines, books, shape, config):
    """Recognition stage: independent spine shape AND a separate whole book."""
    from cart_book_grasp_v2_perception import _spine_frame
    valid_spines=[]
    for score,mask in distinct_masks(spines,shape,config):
        try:
            cleaned=_spine_frame(mask,shape)[0]
            if cleaned.sum() < .85*mask.sum():continue
        except (ValueError,np.linalg.LinAlgError):continue
        valid_spines.append((score,cleaned))
    valid_books=[]
    for score,mask in distinct_masks(books,shape,config):
        count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
        if count<2:continue
        component=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]))
        cleaned=labels==component
        if cleaned.sum()<.9*mask.sum():continue
        yy,xx=np.nonzero(cleaned)
        _,(width,height),_=cv2.minAreaRect(np.column_stack((xx,yy)).astype(np.float32))
        if min(width,height)<4 or max(width,height)<40 or cleaned.sum()/max(1.,width*height)<.5:continue
        valid_books.append((score,cleaned))
    pairs=[]
    for score,mask in valid_spines:
        matches=[]
        for book_score,book_mask in valid_books:
            if np.count_nonzero(book_mask & mask)/mask.sum() < config['min_book_spine_coverage']:continue
            if any(other is not mask and np.count_nonzero(book_mask & other)/other.sum() > config['max_other_spine_coverage']
                   for _,other in valid_spines):continue
            matches.append((int(book_mask.sum()),-book_score,book_mask))
        if matches:pairs.append((score,mask,min(matches,key=lambda m:m[:2])[2]))
    return pairs


def build_candidates(rgb, depth, intrinsic, c2w, spines, books, fit_spine, config):
    shape=np.asarray(depth).shape
    if len(shape)!=2 or np.asarray(rgb).shape!=(*shape,3):
        raise SelectionError('RGB 与对齐深度尺寸不一致。')
    candidates=[]
    # No raw detection or unmatched whole-book fallback is numbered for GPT.
    for score,mask,book_mask in recognized_pairs(spines,books,shape,config):
        yy,xx=np.nonzero(mask)
        item=dict(mask=mask,book_mask=book_mask,score=score,
            uv=[float(np.median(xx)),float(np.median(yy))],
            box=[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)],
            colors=color_fractions(rgb,mask),geometry=None,error='',recognition='book_and_spine')
        try:item['base_y_mm']=observed_base_y(mask,depth,intrinsic,c2w)
        except (ValueError,np.linalg.LinAlgError):item['base_y_mm']=None
        # Recognition and graspability are different. Keep a recognized book
        # with bad depth in the scene so 'middle' does not silently change.
        try:
            geometry=fit_spine(mask,depth,intrinsic)
            geometry.update(whole_book_depth(book_mask,depth,intrinsic,c2w),score=score)
            item['geometry']=geometry
        except (ValueError,np.linalg.LinAlgError) as error:item['error']=str(error)
        candidates.append(item)
    candidates.sort(key=lambda c:(c['base_y_mm'] is None,
        -c['base_y_mm'] if c['base_y_mm'] is not None else c['uv'][0]))
    for index,candidate in enumerate(candidates,1):
        candidate['id']=f'book_{index}'
        candidate['color']=max(candidate['colors'],key=candidate['colors'].get)
    return candidates


def isolated_books(rgb,candidates):
    """Keep original RGB only inside recognized books; hide every other pixel."""
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
        color=(255,230,0) if selected else (80,220,255)
        contours,_=cv2.findContours(candidate['book_mask'].astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(output,contours,-1,color,3 if selected else 1)
        u,v=np.rint(candidate['uv']).astype(int)
        text=candidate['id']+(' SELECTED' if selected else '')
        x=max(0,min(u-20,output.shape[1]-120));y=max(18,min(v,output.shape[0]-8))
        cv2.putText(output,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),4)
        cv2.putText(output,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.55,color,1)
    return output


def candidate_gallery(rgb,candidates):
    """One isolated crop per recognized book, with large nonoverlapping IDs."""
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


def validate_selection(response,candidates,scene_id,minimum):
    fields={'scene_id','status','selected_id','matching_ids','confidence','reason'}
    if not isinstance(response,dict) or set(response)!=fields or response['scene_id']!=scene_id:
        raise SelectionError('GPT 返回了无效结果或旧场景编号，取消本轮。')
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
    candidate=ids[selected]
    if candidate['geometry'] is None:
        raise SelectionError(f"GPT 选中 {selected}，但该书不能可靠抓取：{candidate['error']}；不更换目标。")
    return candidate


class BookSelectionAgent:
    def __init__(self,config_path=None,client=None,checkpoint=lambda:None):
        from cart_book_grasp_v2_gpt import GPTBookClient
        self.config_path=Path(config_path or Path(__file__).with_name('cart_book_grasp_v2_agent_config.json'))
        self.client=client if client is not None else GPTBookClient()
        self.checkpoint=checkpoint;self.instruction=''
        self.last_candidates=[];self.last_decision={};self.last_image=None

    def check_ready(self):
        load_config(self.config_path)
        self.client.check_ready()

    def set_instruction(self,text):
        if not isinstance(text,str) or not text.strip() or len(text)>500:
            raise SelectionError('指令不能为空，且不得超过 500 字。')
        self.instruction=text.strip();self.last_candidates=[];self.last_decision={};self.last_image=None

    def select(self,rgb,depth,intrinsic,c2w,spines,books,fit_spine,on_recognized=None):
        from cart_book_grasp_v2_gpt import build_payload
        self.last_candidates=[];self.last_decision={};self.last_image=None
        config=load_config(self.config_path)
        candidates=build_candidates(rgb,depth,intrinsic,c2w,spines,books,fit_spine,config)
        self.last_candidates=candidates
        self.last_image=annotate_candidates(rgb,candidates)
        scene_id=uuid.uuid4().hex
        self.last_decision=dict(status='recognized',scene_id=scene_id,candidate_count=len(candidates))
        print(f'阶段 1：书本识别完成，确认 {len(candidates)} 本（整书与书脊匹配）；随后交给 GPT 选择。',flush=True)
        if on_recognized is not None:on_recognized()
        try:
            if not candidates:raise SelectionError('没有通过书本识别的候选，未调用 GPT；请调整场景后重试。')
            if not self.instruction:raise SelectionError('本轮缺少文字指令。')
            self.checkpoint()
            print('阶段 2：GPT 仅从已确认的书本编号中选取目标。',flush=True)
            response=self.client.select(build_payload(self.instruction,isolated_books(rgb,candidates),
                candidate_gallery(rgb,candidates),candidates,scene_id,config),
                config['request_timeout_seconds'],self.checkpoint)
            self.checkpoint()
            selected=validate_selection(response,candidates,scene_id,config['min_llm_confidence'])
        except ValueError as error:
            self.last_decision=dict(status='clarify',instruction=self.instruction,scene_id=scene_id,message=str(error))
            raise
        self.last_decision=dict(response,instruction=self.instruction,model=config['model'])
        self.last_image=annotate_candidates(rgb,candidates,selected['id'])
        print(f"GPT 选中 {selected['id']}：{response['reason']}（confidence={response['confidence']:.2f}）",flush=True)
        return selected
