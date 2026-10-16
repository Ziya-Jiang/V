"""Bounded GPT semantic review of head marker depth-region failures.

A 'continue' decision only authorizes a fresh local depth validation, not motion.
"""
import json
import uuid
import cv2
import numpy as np
from cart_book_grasp_v2_gpt import image_url


def review_payload(rgb,depth,q,error,c,scene):
    from cart_grap_v1_perception import payload
    request=payload(rgb,scene,c)
    schema=request['text']['format']['schema']
    schema['properties']['decision']=dict(type='string',enum=['continue','stop'])
    schema['properties']['same_target']=dict(type='boolean')
    schema['required']+=['decision','same_target']
    request['text']['format']['name']='cart_marker_depth_review'
    request['instructions']+=(' You are reviewing a FAILED depth-region check before any robot motion. '
        'Decide stop, or continue with fresh depth verification of the SAME physical marker. '
        'Inspect whether the previous marker boundary includes background/occlusion. Correct the marker '
        'quadrilateral only if visually supported; do not switch targets, select a depth patch unrelated '
        'to the marker, invent depth, relax thresholds or output robot commands. Return stop for ambiguity '
        'or no clearly visible target. Continue never bypasses local depth or robot checks. '
        'All output coordinates refer to the original full image, NOT the enlarged crop. '
        'For stop use status ambiguous/missing and same_target false. Explain briefly in Chinese.')
    lo=np.maximum(np.floor(q.min(0)).astype(int)-20,0)
    hi=np.minimum(np.ceil(q.max(0)).astype(int)+21,[rgb.shape[1],rgb.shape[0]])
    crop=rgb[lo[1]:hi[1],lo[0]:hi[0]].copy()
    cv2.polylines(crop,[np.int32(np.rint(q-lo))],True,(255,190,0),1)
    region=depth[lo[1]:hi[1],lo[0]:hi[0]]
    # Actual measured per-bin medians, with missing depth explicit. Never inpaint.
    grid=[]
    for rows in np.array_split(region,8,axis=0):
        line=[]
        for cell in np.array_split(rows,8,axis=1):
            z=cell[np.isfinite(cell)&(cell>100)&(cell<3000)]
            line.append(None if not z.size else round(float(np.median(z)),2))
        grid.append(line)
    evidence=dict(scene_id=scene,original_size=[rgb.shape[1],rgb.shape[0]],
        failure=str(error),previous_marker_quad=q.tolist(),crop_xyxy=[*lo.tolist(),*hi.tolist()],
        crop_display_scale=4,depth_grid_mm=grid,depth_grid_note='8x8 spatial bins of crop; null means no valid depth',
        continue_means='reacquire three fresh RGB-D frames and rerun unchanged local checks; no motion approval')
    content=request['input'][0]['content']
    content.append(dict(type='input_text',text=json.dumps(evidence,ensure_ascii=False,allow_nan=False)))
    content.append(dict(type='input_image',image_url=image_url(cv2.resize(crop,None,fx=4,fy=4,
        interpolation=cv2.INTER_NEAREST)),detail='original'))
    return request,evidence,crop


def validate_review(result,shape,scene,c,previous):
    from cart_grap_v1_perception import validate_region
    if result.get('decision')!='continue' or result.get('same_target') is not True:
        raise ValueError('GPT depth review decided stop: '+str(result.get('reason')))
    q=validate_region(result,shape,scene,c)
    area=abs(cv2.contourArea(q));oldarea=abs(cv2.contourArea(previous))
    overlap,_=cv2.intersectConvexConvex(q,np.asarray(previous,np.float32))
    iou=overlap/(area+oldarea-overlap)
    max_shift=max(5.,.35*float(np.linalg.norm(np.ptp(previous,axis=0))))
    if not .4<=area/oldarea<=1.5 or iou<.4 or np.linalg.norm(q.mean(0)-previous.mean(0))>max_shift:
        raise ValueError('GPT depth review changed target region; continuation rejected')
    return q


def review_region(out,rgb,depth,q,error,c,client,checkpoint):
    out.mkdir(exist_ok=False)
    scene=uuid.uuid4().hex
    request,evidence,crop=review_payload(rgb,depth,q,error,c,scene)
    (out/'evidence.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2))
    cv2.imwrite(str(out/'input.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out/'crop.png'),cv2.cvtColor(crop,cv2.COLOR_RGB2BGR))
    try:
        checkpoint()
        print(json.dumps(dict(event='gpt_depth_review_start',model=c['model'],reason=str(error)),ensure_ascii=False),flush=True)
        result=client.select(request,c['gpt_timeout_seconds'],checkpoint)
        checkpoint()
        (out/'response.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        refined=validate_review(result,rgb.shape,scene,c,q)
        decision=dict(event='gpt_depth_review',decision='continue_pending_depth_validation',reason=result.get('reason'))
        (out/'decision.json').write_text(json.dumps(decision,ensure_ascii=False,indent=2))
        print(json.dumps(decision,ensure_ascii=False),flush=True)
        return refined
    except Exception as exc:
        (out/'decision.json').write_text(json.dumps(dict(event='gpt_depth_review',decision='stop',reason=str(exc)),ensure_ascii=False,indent=2))
        raise
