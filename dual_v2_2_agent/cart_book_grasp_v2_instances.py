"""SAM3 instance proposals before GPT, adapted from feature-splatting-inria.

Reference: book_thickness_pipeline.py largest_filled_mask / deduplicate /
detect_frame. Reuse its SAM3 loader (resolution 1008), largest filled contour
and mask IoU NMS. Shelf bands, fixed image sizes and grasp geometry gates are
intentionally omitted for cart scenes. Nested group/single-book hypotheses
remain for GPT adjudication; only comparable-size duplicate masks are removed.
"""
from pathlib import Path
import json
import cv2
import numpy as np
from cart_book_grasp_v2_candidates import CandidateError, measured_points

CONFIG_PATH=Path(__file__).with_name('cart_book_grasp_v2_instances.json')


def load_config(path=CONFIG_PATH):
    c=json.loads(Path(path).read_text())
    if not isinstance(c['prompt'],str) or not c['prompt'].strip():raise ValueError('Invalid SAM3 prompt')
    for key in ('min_score','mask_nms_iou','nms_min_area_ratio'):
        if type(c[key]) not in (int,float) or not 0<c[key]<=1:raise ValueError(f'Invalid {key}')
    for key in ('min_area_px','crop_padding_px'):
        if type(c[key]) is not int or not 1<=c[key]<=10000:raise ValueError(f'Invalid {key}')
    if type(c['crop_recall']) is not bool:raise ValueError('Invalid crop_recall')
    return c


def clean_masks(detections,shape,config):
    kept=[]
    for score,raw,_ in sorted(detections,key=lambda d:d[0],reverse=True):
        if not np.isfinite(score) or score<config['min_score']:continue
        raw=np.asarray(raw,np.uint8)
        if raw.ndim!=2 or raw.shape!=shape:continue
        contours,_=cv2.findContours(raw,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        if not contours:continue
        mask=np.zeros(shape,np.uint8)
        cv2.drawContours(mask,[max(contours,key=cv2.contourArea)],-1,1,cv2.FILLED)
        mask=mask.astype(bool);area=int(mask.sum())
        if area<config['min_area_px']:continue
        duplicate=False
        for _,other,_ in kept:
            ratio=min(area,other.sum())/max(area,other.sum())
            iou=np.count_nonzero(mask & other)/np.count_nonzero(mask | other)
            if ratio>=config['nms_min_area_ratio'] and iou>=config['mask_nms_iou']:
                duplicate=True;break
        if duplicate:continue
        yy,xx=np.nonzero(mask)
        kept.append((float(score),mask,[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)]))
    return kept


def detect_instances(book_geom,rgb,depth,k,c2w,config,depth_config,checkpoint=lambda:None):
    rgb=np.asarray(rgb);depth=np.asarray(depth,float);k=np.asarray(k,float);c2w=np.asarray(c2w,float)
    dlo,dhi=depth_config['depth_range_mm']
    diagnostics=dict(stage='sam3_instances',filter_method='sam3_mask_camera_depth',
                     accepted_depth_mm=[dlo,dhi],boxes=[],prompt=config['prompt'])
    def fail(message):raise CandidateError(message,diagnostics)
    if depth.ndim!=2 or rgb.shape!=(*depth.shape,3):fail('RGB 与深度未对齐')
    if k.shape!=(3,3) or c2w.shape!=(4,4) or not np.isfinite(k).all() or not np.isfinite(c2w).all():
        fail('相机内参或机器人基座外参无效')
    checkpoint()
    processor,to_numpy,image_cls=book_geom._load_sam3_detector()
    def segment(image):
        checkpoint()
        result=book_geom._sam3_raw_books(processor,to_numpy,image_cls.fromarray(image),prompt=config['prompt'])
        checkpoint()
        return result
    raw=segment(rgb)
    masks=clean_masks(raw,depth.shape,config)
    diagnostics.update(full_frame_raw_count=len(raw),full_frame_instance_count=len(masks))
    def measure(mask):
        # Actual mask interior, not the rectangular background between books.
        core=cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
        if core.sum()<depth_config['min_depth_pixels']:core=mask
        points,stats=measured_points(core,depth,k,c2w,depth_config)
        center=np.median(points,axis=0)
        if not np.isfinite(center).all():raise ValueError('反投影结果无效')
        return center,stats
    seeds=[]
    for _,mask,box in masks:
        try:
            _,stats=measure(mask)
            if dlo<=stats['depth_median_mm']<=dhi:seeds.append(box)
        except (ValueError,np.linalg.LinAlgError):pass
    if not seeds:
        valid=np.isfinite(depth)&(depth>0)
        values=depth[valid]
        scale=1. if len(values) and np.median(values)>20 else 1000.
        yy,xx=np.nonzero(valid&(depth*scale>=dlo)&(depth*scale<=dhi))
        if len(xx)>=config['min_area_px']:
            seeds=[[xx.min(),yy.min(),xx.max()+1,yy.max()+1]]
    if config['crop_recall'] and seeds:
        boxes=np.asarray(seeds);pad=config['crop_padding_px']
        region=np.r_[boxes[:,:2].min(0)-pad,boxes[:,2:].max(0)+pad].astype(int)
        region[[0,2]]=region[[0,2]].clip(0,depth.shape[1]);region[[1,3]]=region[[1,3]].clip(0,depth.shape[0])
        x0,y0,x1,y1=region
        # Skip an identical full-frame query.
        if (x1-x0)*(y1-y0)<depth.size*.95:
            diagnostics['depth_recall_roi']=region.tolist()
            extra=segment(rgb[y0:y1,x0:x1]);restored=[]
            for score,mask,_ in clean_masks(extra,(y1-y0,x1-x0),config):
                # A crop-edge fragment cannot replace a complete book mask.
                yy,xx=np.nonzero(mask)
                if xx.min()<=1 or yy.min()<=1 or xx.max()>=x1-x0-2 or yy.max()>=y1-y0-2:continue
                full=np.zeros(depth.shape,bool);full[y0:y1,x0:x1]=mask
                restored.append((score,full,[]))
            diagnostics['crop_raw_count']=len(extra)
            masks=clean_masks(masks+restored,depth.shape,config)
    accepted=[]
    for score,mask,box in masks:
        checkpoint()
        entry=dict(box=box,score=score,status='invalid_depth',area_px=int(mask.sum()))
        diagnostics['boxes'].append(entry)
        try:center,stats=measure(mask)
        except (ValueError,np.linalg.LinAlgError) as error:
            entry['reason']=str(error);continue
        entry.update(stats,base_xyz_mm=center.tolist())
        if not dlo<=stats['depth_median_mm']<=dhi:
            entry['status']='outside_depth_range';continue
        entry['status']='accepted'
        yy,xx=np.nonzero(mask)
        accepted.append(dict(mask=mask,book_mask=mask,box=box,uv=[float(np.median(xx)),float(np.median(yy))],
            score=score,base_y_mm=float(center[1]),base_xyz_mm=center.tolist(),depth_median_mm=stats['depth_median_mm'],
            recognition='book_proposal',proposal_method='sam3_instances',geometry=None,
            geometry_status='not_checked',error='',color='unclassified'))
    accepted.sort(key=lambda c:-c['base_y_mm'])
    for i,item in enumerate(accepted,1):item['id']=f'book_{i}'
    diagnostics.update(stage='candidates_ready',candidate_count=len(accepted))
    print(f'SAM3 实例分割：全图 {len(raw)} 个原始 mask，去重和深度筛选后保留 {len(accepted)} 个候选，交给 GPT 逐个审核。',flush=True)
    if not accepted:fail(f'SAM3 在深度 {dlo:g}–{dhi:g} mm 范围内没有有效书本实例')
    return accepted,diagnostics
