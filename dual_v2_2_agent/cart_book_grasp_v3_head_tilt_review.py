"""Recheck suspicious head 3D tilt using GPT regions and measured RGB-D only."""
import numpy as np
from book_spine_wrist import base_tilt
from cart_book_grasp_v3_2_head_fallback import recover,center_from_mask,load_config


def vertical_tilt(selected,c2w):
    # Successful depth fitting returns camera-axis data; the 'valid' base-frame
    # label is assigned later by base_tilt, not necessarily by the fitter.
    if 'spine_axis_camera' not in selected:return None
    geometry=base_tilt(selected,c2w)
    value=float(geometry['spine_vertical_tilt_deg'])
    if not np.isfinite(value):raise ValueError('Non-finite head 3D spine tilt')
    return value


def review(selected,rgb,depth,intrinsic,c2w,*,box_detector=None,checkpoint=lambda:None,
           config=None,fallback=None,center=None,observation_robot=None,force_reason=None):
    """Return selected region and optional replacement whole-book detections."""
    config=load_config() if config is None else config
    threshold=config.get('tilt_review_threshold_deg',45.)
    before=vertical_tilt(selected,c2w)
    reason=force_reason
    if reason is None and observation_robot is not None and 'spine_axis_camera' in selected:
        from book_spine_wrist import j7_proposal,load_wrist_config,J7GeometryUnavailable
        wrist=load_wrist_config()
        if wrist['enabled']:
            try:
                # Geometry preview only; actual joint margins remain checked at execution.
                j7_proposal(base_tilt(selected,c2w),observation_robot.get_current_state(),wrist,-1e6,1e6)
            except J7GeometryUnavailable as error:reason=str(error)
    if reason is None and (before is None or before<=threshold):return selected,None
    checkpoint()
    original=selected;books=None
    already_gpt=selected.get('selection_source')=='gpt6_polygon_fallback' and force_reason is None
    if not already_gpt:
        if not config['enabled']:raise ValueError('Head tilt exceeds threshold but GPT review is disabled')
        mask=np.asarray(selected['mask'],bool);yy,xx=np.nonzero(mask);h,w=mask.shape
        context=dict(reason=reason or 'base_vertical_tilt_exceeded',base_vertical_tilt_deg=before,
                     threshold_deg=threshold,spine_image_tilt_deg=selected.get('spine_tilt_deg'),
                     candidate_box_0_1000=[float(xx.min())/(w-1)*1000,float(yy.min())/(h-1)*1000,
                                          float(xx.max())/(w-1)*1000,float(yy.max())/(h-1)*1000])
        print(f'头部区域复核：{reason or str(before)+"° > "+str(threshold)+"°"}；调用 GPT 复核同一本书并重算深度几何。',flush=True)
        detections,books=(fallback or recover)(rgb,box_detector,checkpoint=checkpoint,
                                               config=config,review_context=context)
        candidates=[]
        for score,new_mask,_ in detections:
            new_mask=np.asarray(new_mask,bool)
            if new_mask.shape!=mask.shape:
                raise ValueError('GPT review mask dimensions do not match the image')
            result=(center or center_from_mask)(new_mask,depth,intrinsic)
            result.update(score=float(score),selection_source='gpt6_polygon_fallback')
            candidates.append(result)
        if not candidates:raise ValueError('GPT tilt review did not identify the selected spine')
        selected=max(candidates,key=lambda candidate:candidate['score'])
    after=vertical_tilt(selected,c2w)
    selected=dict(selected)
    selected['head_tilt_review']=dict(threshold_deg=threshold,before_deg=before,after_deg=after,
        trigger=reason or 'base_vertical_tilt_exceeded',source='existing_gpt_selection' if already_gpt else 'gpt_reselected',
        original_center_uv=original['uv'],reviewed_center_uv=selected['uv'])
    if after is None or after>threshold:
        selected['j7_skip_reason']='GPT 已确认图像区域，但实测三维倾角仍不可用于预对齐（无效或超过阈值）'
        print(selected['j7_skip_reason']+'；跳过可选 J7 预旋转，保留有效深度定位。',flush=True)
    else:
        print(f'GPT 复核后实测三维倾角 {after:.2f}°，继续原 J7 可行性检查。',flush=True)
    checkpoint()
    return selected,books
