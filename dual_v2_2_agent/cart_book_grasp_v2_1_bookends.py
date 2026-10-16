"""SAM3 bookend exclusion in an automatically enlarged candidate region.

The original full-frame book masks and head calibration remain unchanged.
No color, fixed pixel location, wrist-to-head transform or cross-prompt score
comparison is used to decide which candidate to exclude.
"""
import hashlib
from pathlib import Path

import cv2
import numpy as np

DEFAULTS = dict(enabled=True, prompts=['bookend'], min_score=.25,
                crop_padding_fraction=.25, min_book_coverage=.75,
                min_bookend_coverage=.75, max_other_book_coverage=.2)


def filter_config(config):
    supplied=config.get('bookend_filter',{})
    if not isinstance(supplied,dict):raise ValueError('bookend_filter must be an object')
    c={**DEFAULTS,**supplied}
    if type(c['enabled']) is not bool:raise ValueError('bookend_filter.enabled must be boolean')
    prompts=c['prompts']
    if (not isinstance(prompts,list) or not 1<=len(prompts)<=4
        or any(not isinstance(p,str) or not p.strip() or p!=p.strip() or p.lower()=='book' for p in prompts)
        or len(set(prompts))!=len(prompts)):
        raise ValueError('bookend_filter.prompts requires 1-4 unique nonempty bookend prompts')
    for key,lo,hi in [('min_score',.1,1),('crop_padding_fraction',.05,1),
                       ('min_book_coverage',.5,1),('min_bookend_coverage',.5,1),
                       ('max_other_book_coverage',0,.5)]:
        v=c[key]
        if type(v) not in (int,float) or not np.isfinite(v) or not lo<=v<=hi:
            raise ValueError('Invalid bookend_filter.'+key)
    return c


def candidate_crop(items, shape, padding):
    """Union of measured-workspace candidates with context; no fixed scene ROI."""
    if not items:return None
    boxes=np.asarray([v['box'] for v in items],float)
    lo=boxes[:,:2].min(axis=0);hi=boxes[:,2:].max(axis=0)
    pad=(hi-lo)*padding
    x0,y0=np.maximum(0,np.floor(lo-pad)).astype(int)
    x1,y1=np.minimum([shape[1],shape[0]],np.ceil(hi+pad)).astype(int)
    if x1<=x0 or y1<=y0:raise ValueError('Invalid bookend candidate crop')
    return [int(x0),int(y0),int(x1),int(y1)]


def detect_bookends(rgb,items,config,adapter_factory,*,checkpoint=lambda:None,cache_dir=None):
    """One extra image embedding for the crop, shared by all exclusion prompts.

    adapter_factory is lazy so a valid offline cache does not load SAM3.
    Masks are restored to original RGB pixel coordinates before comparison.
    """
    c=filter_config(config)
    detail=dict(enabled=c['enabled'],prompts=c['prompts'],crop_xyxy=None,cache_hit=False)
    if not c['enabled'] or not items:return {},detail
    roi=candidate_crop(items,rgb.shape,c['crop_padding_fraction'])
    x0,y0,x1,y1=roi;crop=np.ascontiguousarray(rgb[y0:y1,x0:x1])
    detail['crop_xyxy']=roi
    path=Path(cache_dir)/'head_bookend_crop.npz' if cache_dir is not None else None
    checkpoint()
    results=load_prompt_cache(path,crop,c['prompts']) if path is not None else None
    detail['cache_hit']=results is not None
    if results is None:
        processor,to_numpy,image_cls=adapter_factory()
        results=infer_prompts(processor,to_numpy,image_cls.fromarray(crop),c['prompts'],checkpoint)
        if path is not None:save_prompt_cache(path,crop,results)
    checkpoint()
    restored={}
    for prompt,raw in results.items():
        restored[prompt]=[]
        for score,mask,_ in raw:
            if mask.shape!=crop.shape[:2]:raise ValueError('Bookend mask does not match crop')
            if not mask.any():continue
            full=np.zeros(rgb.shape[:2],bool);full[y0:y1,x0:x1]=mask
            yy,xx=np.nonzero(full)
            box=[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)]
            restored[prompt].append((float(score),full,box))
    detail['raw_counts']={p:len(v) for p,v in restored.items()}
    return restored,detail


def exclude_bookends(items,results,config,report):
    """Exclude whole candidates only on an isolated, bidirectional mask match.

    Do not subtract pixels: that would shift the original depth/center used by
    V1_3. Broad masks covering several books cannot remove any of them.
    """
    c=filter_config(config)
    report.update(input_count=len(items),removed=[],evidence=[],min_score=c['min_score'])
    excluded=set()
    for prompt,raw in results.items():
        for score,mask,box in raw:
            if not np.isfinite(score) or score<c['min_score']:continue
            mask=np.asarray(mask,bool)
            if items and any(mask.shape!=v['book_mask'].shape for v in items):
                raise ValueError('Bookend mask does not match head image')
            area=int(mask.sum())
            if not area:continue
            intersections=[int(np.count_nonzero(mask&v['book_mask'])) for v in items]
            book_cover=[n/max(1,int(v['book_mask'].sum())) for n,v in zip(intersections,items)]
            end_cover=[n/area for n in intersections]
            matches=[i for i in range(len(items)) if book_cover[i]>=c['min_book_coverage']
                     and end_cover[i]>=c['min_bookend_coverage']]
            evidence=dict(prompt=prompt,score=float(score),box=list(box),
                          book_coverage=book_cover,bookend_coverage=end_cover)
            if len(matches)!=1:
                evidence['decision']='no_unique_mask_match'
            elif any(v>c['max_other_book_coverage'] for i,v in enumerate(book_cover) if i!=matches[0]):
                evidence['decision']='overlaps_other_book'
            else:
                i=matches[0];evidence.update(decision='exclude_bookend',candidate_index=i)
                if i not in excluded:
                    report['removed'].append(dict(candidate_index=i,box=items[i]['box'],
                        book_score=items[i]['score'],prompt=prompt,bookend_score=float(score),
                        book_coverage=book_cover[i],bookend_coverage=end_cover[i]))
                excluded.add(i)
            report['evidence'].append(evidence)
    kept=[v for i,v in enumerate(items) if i not in excluded]
    report.update(removed_count=len(excluded),kept_count=len(kept))
    return kept


def bookend_overlay(rgb,items,results,report):
    from cart_book_grasp_v2_1_numbered import numbered_overlay
    out=numbered_overlay(rgb,items)
    # English-only labels for the Rerun viewer; retain original image colors.
    for evidence in report.get('evidence',[]):
        prompt=evidence['prompt'];box=evidence['box']
        excluded=evidence['decision']=='exclude_bookend'
        color=(255,70,190) if excluded else (255,180,70)
        for score,mask,raw_box in results.get(prompt,[]):
            if list(raw_box)!=box or abs(float(score)-evidence['score'])>1e-6:continue
            contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(out,contours,-1,color,2)
        x,y=int(box[0]),max(46,int(box[1])-7)
        label='BOOKEND excluded' if excluded else 'BOOKEND proposal'
        cv2.putText(out,label,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1,cv2.LINE_AA)
    text=f"SAM3 bookend: {report.get('removed_count',0)} excluded; {len(items)} numbered"
    cv2.rectangle(out,(0,0),(out.shape[1],32),(25,25,25),-1)
    cv2.putText(out,text,(10,22),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
    return out


def filter_summary(report):
    if not report.get('enabled'):return 'SAM3 书立排除已关闭。'
    return (f"SAM3 局部书立排除：检出 {sum(report.get('raw_counts',{}).values())} 个原始 mask，"
            f"排除 {report.get('removed_count',0)} 个候选，保留 {report.get('kept_count',0)} 个编号。"
            +(' 未匹配到足够明确的书立；不据此认定所有候选都是真书。' if not report.get('removed_count') else ''))


def infer_prompts(processor,to_numpy,image,prompts,checkpoint=lambda:None):
    """Copy each result before the mutable SAM3 state is reused for a new prompt."""
    import torch
    result={}
    checkpoint()
    with torch.inference_mode():
        state=processor.set_image(image)
        checkpoint()
        for prompt in prompts:
            processor.reset_all_prompts(state)
            state=processor.set_text_prompt(prompt=prompt,state=state)
            masks=to_numpy(state['masks'])
            if masks.ndim==4 and masks.shape[1]==1:masks=masks[:,0]
            scores=to_numpy(state['scores']).reshape(-1)
            boxes=to_numpy(state['boxes']).reshape(-1,4)
            result[prompt]=[(float(s),np.array(m,dtype=bool,copy=True),np.array(b,copy=True))
                            for s,m,b in zip(scores,masks,boxes)]
            checkpoint()
    return result


def save_prompt_cache(path,rgb,results):
    values=dict(frame_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),prompts=np.array(list(results)))
    for i,raw in enumerate(results.values()):
        values[f'scores_{i}']=np.asarray([v[0] for v in raw],float)
        values[f'masks_{i}']=np.asarray([v[1] for v in raw],bool).reshape(-1,*rgb.shape[:2])
        values[f'boxes_{i}']=np.asarray([v[2] for v in raw],float).reshape(-1,4)
    np.savez_compressed(path,**values)


def load_prompt_cache(path,rgb,prompts):
    path=Path(path)
    if not path.is_file():return None
    try:
        with np.load(path,allow_pickle=False) as data:
            if str(data['frame_sha256'])!=hashlib.sha256(rgb.tobytes()).hexdigest():return None
            if data['prompts'].tolist()!=list(prompts):return None
            result={}
            for i,prompt in enumerate(prompts):
                scores,masks,boxes=data[f'scores_{i}'],data[f'masks_{i}'],data[f'boxes_{i}']
                if (scores.ndim!=1 or masks.shape!=(len(scores),*rgb.shape[:2])
                    or masks.dtype!=np.bool_ or boxes.shape!=(len(scores),4)
                    or not np.isfinite(scores).all() or not np.isfinite(boxes).all()):return None
                result[prompt]=list(zip(scores,masks,boxes))
            return result
    except (OSError,ValueError,KeyError):
        return None
