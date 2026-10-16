"""Refine only the selected instance's spine; preserve GPT's exact book mask.

The legacy DINO-box refinement remains available for offline comparisons.
"""
import numpy as np
import cv2
from cart_book_grasp_v2_candidates import load_config as load_candidate_config,overlap
from cart_book_grasp_v2_agent import SelectionError,load_geometry_config as load_config,distinct_masks,prepare_selected_geometry


def prepare_instance_target(candidate,rgb,depth,k,c2w,book_geom,fit_spine,agent_config_path,checkpoint=lambda:None):
    config=load_config(agent_config_path);c=load_candidate_config()
    book_mask=np.asarray(candidate['book_mask'],bool)
    if book_mask.shape!=np.asarray(depth).shape or not book_mask.any():
        raise SelectionError('选中的实例 mask 无效')
    yy,xx=np.nonzero(book_mask)
    box=np.array([xx.min(),yy.min(),xx.max()+1,yy.max()+1])
    pad=(box[2:]-box[:2])*c['selected_crop_padding']
    crop=np.r_[np.floor(box[:2]-pad),np.ceil(box[2:]+pad)].astype(int)
    crop[[0,2]]=crop[[0,2]].clip(0,rgb.shape[1]);crop[[1,3]]=crop[[1,3]].clip(0,rgb.shape[0])
    x0,y0,x1,y1=crop
    checkpoint()
    processor,to_numpy,image_cls=book_geom._load_sam3_detector()
    spines=restore_crop(book_geom._sam3_raw_books(processor,to_numpy,
        image_cls.fromarray(np.asarray(rgb)[y0:y1,x0:x1]),prompt='book spine'),crop,depth.shape)
    checkpoint()
    errors=[]
    for score,mask in distinct_masks(spines,depth.shape,config):
        if np.count_nonzero(mask & book_mask)/mask.sum()<config['min_book_spine_coverage']:continue
        checkpoint()
        try:
            prepare_selected_geometry(candidate,[(score,mask,[])],depth,k,c2w,fit_spine,config)
        except SelectionError as error:
            errors.append(str(error));continue
        break
    else:
        raise SelectionError('选中实例没有通过几何检查的书脊，不更换目标：'+('; '.join(errors[-3:]) or '书脊不匹配'))
    candidate.update(selection_box=box.tolist(),segmentation_crop=crop.tolist())
    candidate['geometry'].update(selection_box_xyxy=box.tolist(),segmentation_crop_xyxy=crop.tolist())
    return candidate


def restore_crop(detections,crop_box,shape):
    x0,y0,x1,y1=crop_box
    result=[]
    for score,mask,_ in detections:
        mask=np.asarray(mask,np.uint8)
        if mask.ndim!=2:continue
        if mask.shape!=(y1-y0,x1-x0):mask=cv2.resize(mask,(x1-x0,y1-y0),interpolation=cv2.INTER_NEAREST)
        full=np.zeros(shape,bool);full[y0:y1,x0:x1]=mask.astype(bool)
        yy,xx=np.nonzero(full)
        if len(xx):result.append((score,full,[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)]))
    return result


def prepare_target(candidate,rgb,depth,k,c2w,book_geom,fit_spine,agent_config_path,checkpoint=lambda:None):
    config=load_config(agent_config_path);c=load_candidate_config()
    box=np.asarray(candidate['box'],int);size=box[2:]-box[:2]
    pad=size*c['selected_crop_padding']
    crop=np.r_[np.floor(box[:2]-pad),np.ceil(box[2:]+pad)].astype(int)
    crop[[0,2]]=np.clip(crop[[0,2]],0,rgb.shape[1]);crop[[1,3]]=np.clip(crop[[1,3]],0,rgb.shape[0])
    x0,y0,x1,y1=crop
    checkpoint()
    processor,to_numpy,image_cls=book_geom._load_sam3_detector()
    image=image_cls.fromarray(np.asarray(rgb)[y0:y1,x0:x1])
    books=restore_crop(book_geom._sam3_raw_books(processor,to_numpy,image,prompt='book'),crop,depth.shape)
    checkpoint()
    matches=[]
    for score,mask in distinct_masks(books,depth.shape,config):
        yy,xx=np.nonzero(mask);bounds=np.array([xx.min(),yy.min(),xx.max()+1,yy.max()+1])
        inside=float(mask[box[1]:box[3],box[0]:box[2]].sum()/mask.sum())
        intersection=overlap(bounds,box)
        iou=intersection/(np.prod(bounds[2:]-bounds[:2])+np.prod(size)-intersection)
        if inside>=.85 and iou>=.35:matches.append((iou,score,mask))
    if not matches:raise SelectionError('选中框内没有匹配的整书 mask；取消本轮，不更换目标')
    matches.sort(key=lambda m:(m[0]*m[1],m[0]),reverse=True)
    yy,xx=np.nonzero(matches[0][2])
    if xx.min()<=x0+1 or xx.max()>=x1-2 or yy.min()<=y0+1 or yy.max()>=y1-2:
        raise SelectionError('选中书在裁剪边缘被截断，请增大 selected_crop_padding 后重试')
    if len(matches)>1:
        a,b=matches[0],matches[1]
        mask_iou=np.count_nonzero(a[2]&b[2])/np.count_nonzero(a[2]|b[2])
        if b[0]*b[1]>=.85*a[0]*a[1] and mask_iou<.5:
            raise SelectionError('选中框内有多个不同书本 mask，无法唯一匹配；取消本轮')
    spines=restore_crop(book_geom._sam3_raw_books(processor,to_numpy,image,prompt='book spine'),crop,depth.shape)
    checkpoint()
    candidate.update(selection_box=box.tolist(),segmentation_crop=crop.tolist(),
                     recognition='book_proposal',book_mask=matches[0][2])
    # SAM3 can emit many overlapping fragments of the SAME spine, especially
    # after rotated recall. In the selected book crop use the v1.3 rule: the
    # highest-score spine that passes geometry and whole-book association.
    # Do not interpret every partial mask as another physical book.
    errors=[]
    for score,mask in distinct_masks(spines,depth.shape,config):
        if np.count_nonzero(mask & candidate['book_mask'])/mask.sum()<config['min_book_spine_coverage']:
            continue
        checkpoint()
        try:
            prepare_selected_geometry(candidate,[(score,mask,[])],depth,k,c2w,fit_spine,config)
        except SelectionError as error:
            errors.append(str(error));continue
        break
    else:
        raise SelectionError('选中书没有通过几何检查的书脊：'+('; '.join(errors[-3:]) or '书脊与整书 mask 不匹配'))
    candidate['geometry'].update(selection_box_xyxy=box.tolist(),segmentation_crop_xyxy=crop.tolist())
    return candidate
