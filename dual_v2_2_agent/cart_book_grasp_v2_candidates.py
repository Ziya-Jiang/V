"""DINO book boxes -> camera-depth range filter, without cart detection.

Crop recall uses in-range books. Base coordinates supply spatial ordering to
GPT only; they do not impose an additional region gate.
"""
from pathlib import Path
import json
import cv2
import numpy as np

CONFIG_PATH=Path(__file__).with_name('cart_book_grasp_v2_candidates.json')


class CandidateError(ValueError):
    def __init__(self,message,diagnostics):
        super().__init__(message)
        self.diagnostics=diagnostics


def load_config(path=CONFIG_PATH):
    c=json.loads(Path(path).read_text())
    if not isinstance(c['book_prompt'],str) or not c['book_prompt'].strip():raise ValueError('Invalid book_prompt')
    for key in ('book_score','text_score','nms_iou','min_depth_fraction'):
        if type(c[key]) not in (int,float) or not 0<c[key]<=1:raise ValueError(f'Invalid {key}')
    for key,limit in [('selected_crop_padding',.5),('depth_crop_padding_px',1000),('min_depth_pixels',10000)]:
        if type(c[key]) not in (int,float) or not np.isfinite(c[key]) or not 0<c[key]<=limit:
            raise ValueError(f'Invalid {key}')
    interval=np.asarray(c['depth_range_mm'],float)
    if interval.shape!=(2,) or not np.isfinite(interval).all() or not 0<interval[0]<interval[1]<=10000:
        raise ValueError('Invalid depth_range_mm')
    return c


def box_mask(box,shape):
    x0,y0,x1,y1=map(int,box)
    mask=np.zeros(shape,bool);mask[y0:y1,x0:x1]=True
    return mask


def overlap(a,b):
    lo=np.maximum(a[:2],b[:2]);hi=np.minimum(a[2:],b[2:])
    return float(np.prod(np.maximum(hi-lo,0)))


def clean_boxes(result,shape,score_min,iou):
    boxes,scores,_=result
    boxes=np.asarray(boxes,float).reshape(-1,4);scores=np.asarray(scores,float).reshape(-1)
    if len(boxes)!=len(scores):raise ValueError('Detector box/score count mismatch')
    out=[]
    for i in np.argsort(-scores):
        b=boxes[i].copy();score=float(scores[i])
        if not np.isfinite(b).all() or not np.isfinite(score) or score<score_min:continue
        b[[0,2]]=np.clip(b[[0,2]],0,shape[1]);b[[1,3]]=np.clip(b[[1,3]],0,shape[0])
        b=np.r_[np.floor(b[:2]),np.ceil(b[2:])].astype(int)
        if np.any(b[2:]-b[:2]<2):continue
        area=float(np.prod(b[2:]-b[:2]))
        if any(overlap(b,p)/(area+np.prod(p[2:]-p[:2])-overlap(b,p))>=iou for _,p in out):continue
        out.append((score,b))
    return out


def measured_points(mask,depth,k,c2w,config):
    valid=mask & np.isfinite(depth) & (depth>0)
    yy,xx=np.nonzero(valid)
    fraction=len(xx)/max(1,int(mask.sum()))
    if len(xx)<config['min_depth_pixels'] or fraction<config['min_depth_fraction']:
        raise ValueError('有效深度不足')
    z=depth[yy,xx].astype(float)
    # Same depth-unit convention as the existing book geometry implementation.
    if np.median(z)>20:z*=.001
    q=np.percentile(z,[10,90])*1000
    rays=np.linalg.solve(k,np.array([xx,yy,np.ones(len(xx))]))
    if not np.isfinite(rays).all() or np.any(rays[2]<=0):raise ValueError('相机射线无效')
    translation=c2w[:3,3].copy()
    if np.linalg.norm(translation)>10:translation*=.001
    points=((rays/rays[2]*z).T@c2w[:3,:3].T+translation)*1000
    return points,dict(depth_median_mm=float(np.median(z)*1000),depth_p10_p90_mm=q.tolist(),valid_fraction=fraction)


def detect_candidates(detector,rgb,depth,k,c2w,config,checkpoint=lambda:None):
    """Uses the current prepared RGB frame; queries only book, never cart."""
    depth=np.asarray(depth,float);k=np.asarray(k,float);c2w=np.asarray(c2w,float)
    dlo,dhi=config['depth_range_mm']
    diagnostics=dict(stage='depth_filter',filter_method='camera_depth_only',
                     accepted_depth_mm=[dlo,dhi],boxes=[])
    def fail(message):raise CandidateError(message,diagnostics)
    if depth.ndim!=2 or np.asarray(rgb).shape!=(*depth.shape,3):fail('RGB 与深度未对齐')
    if k.shape!=(3,3) or c2w.shape!=(4,4) or not np.isfinite(k).all() or not np.isfinite(c2w).all():
        fail('相机内参或机器人基座外参无效')
    checkpoint()
    books=clean_boxes(detector.detect_boxes_prepared(config['book_prompt'],box_threshold=config['book_score'],
        text_threshold=config['text_score']),depth.shape,config['book_score'],config['nms_iou'])
    checkpoint()
    diagnostics['full_frame_book_count']=len(books)

    def measure(box):
        inset=(box[2:]-box[:2])*.2
        centerbox=np.r_[np.floor(box[:2]+inset),np.ceil(box[2:]-inset)].astype(int)
        points,stats=measured_points(box_mask(centerbox,depth.shape),depth,k,c2w,config)
        center=np.median(points,axis=0)
        if not np.isfinite(center).all():raise ValueError('反投影结果无效')
        return center,stats

    # Zoom around in-range books. No cart label/count/reference patch gate.
    seeds=[]
    for _,box in books:
        try:
            _,stats=measure(box)
            if dlo<=stats['depth_median_mm']<=dhi:seeds.append(box)
        except (ValueError,np.linalg.LinAlgError):pass
    region_detector=getattr(detector,'detect_boxes_region',None)
    if seeds and callable(region_detector):
        pad=config['depth_crop_padding_px']
        region=np.r_[np.min(np.array(seeds)[:,:2],axis=0)-pad,np.max(np.array(seeds)[:,2:],axis=0)+pad].astype(int)
        region[[0,2]]=np.clip(region[[0,2]],0,depth.shape[1]);region[[1,3]]=np.clip(region[[1,3]],0,depth.shape[0])
        diagnostics['depth_recall_roi']=region.tolist()
        extra=clean_boxes(region_detector(rgb,region,config['book_prompt'],box_threshold=config['book_score'],
            text_threshold=config['text_score']),depth.shape,config['book_score'],config['nms_iou'])
        checkpoint()
        diagnostics['depth_crop_book_count']=len(extra)
        combined=books+extra
        books=clean_boxes((np.array([b for _,b in combined]),np.array([s for s,_ in combined]),[]),
                          depth.shape,config['book_score'],config['nms_iou'])
    accepted=[]
    for score,box in books:
        entry=dict(box=box.tolist(),score=score,status='invalid_depth')
        diagnostics['boxes'].append(entry)
        try:center,stats=measure(box)
        except (ValueError,np.linalg.LinAlgError) as e:
            entry['reason']=str(e);continue
        entry.update(stats,base_xyz_mm=center.tolist())
        if not dlo<=stats['depth_median_mm']<=dhi:
            entry['status']='outside_depth_range';continue
        entry['status']='accepted'
        mask=box_mask(box,depth.shape)
        accepted.append(dict(mask=mask,book_mask=mask,box=box.tolist(),uv=((box[:2]+box[2:])/2).tolist(),
            score=score,base_y_mm=float(center[1]),base_xyz_mm=center.tolist(),depth_median_mm=stats['depth_median_mm'],
            recognition='book_box',geometry=None,geometry_status='not_checked',error='',color='unclassified'))
    accepted.sort(key=lambda c:-c['base_y_mm'])
    for i,item in enumerate(accepted,1):item['id']=f'book_{i}'
    diagnostics.update(stage='candidates_ready',candidate_count=len(accepted))
    print(f'深度筛选：保留 {dlo:g}–{dhi:g} mm 范围内的 {len(accepted)} 个候选；不检测推车。',flush=True)
    for entry in diagnostics['boxes']:
        if entry['status']!='accepted':
            z=entry.get('depth_median_mm')
            detail=f'{z:.0f} mm' if z is not None else entry.get('reason','invalid depth')
            print(f"跳过候选 {entry['box']}：{entry['status']}，{detail}",flush=True)
    if not accepted:fail(f'深度 {dlo:g}–{dhi:g} mm 范围内没有有效书本候选')
    return accepted,diagnostics


def annotate_gate(rgb,diagnostics):
    image=np.asarray(rgb).copy()
    limits=diagnostics.get('accepted_depth_mm',[])
    cv2.putText(image,f'Depth only: {limits} mm',(12,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,230,70),2)
    if 'depth_recall_roi' in diagnostics:
        b=diagnostics['depth_recall_roi'];cv2.rectangle(image,tuple(b[:2]),tuple(b[2:]),(30,220,255),1)
    for item in diagnostics.get('boxes',[]):
        b=item['box'];status=item['status'];color=(50,240,80) if status=='accepted' else (255,100,80)
        cv2.rectangle(image,tuple(b[:2]),tuple(b[2:]),color,1)
        z=item.get('depth_median_mm')
        label=f'{status} {z:.0f}mm' if z is not None else status
        cv2.putText(image,label,(b[0],min(image.shape[0]-5,b[3]+14)),cv2.FONT_HERSHEY_SIMPLEX,.35,color,1)
    return image
