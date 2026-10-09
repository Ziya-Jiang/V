"""DINO boxes -> cart RGB-D gate. No segmenter, GPT, camera or robot imports.

Cart bounds are estimates from a visible reference patch, not a support-plane
fit. Ambiguous cart/depth results stop the round rather than change book order.
All coordinates returned to selection are in the robot base frame, in mm.
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
    for key in ('book_prompt','cart_prompt'):
        if not isinstance(c[key],str) or not c[key].strip():raise ValueError(f'Invalid {key}')
    for key in ('book_score','cart_score','text_score','nms_iou','min_box_cart_overlap','min_depth_fraction'):
        if type(c[key]) not in (int,float) or not 0<c[key]<=1:raise ValueError(f'Invalid {key}')
    for key,limit in [('cart_score_margin',1),('cart_top_extension',1),('selected_crop_padding',.5),
                      ('cart_max_depth_spread_mm',1000),('book_max_depth_spread_mm',1000),
                      ('depth_padding_mm',500),('base_xy_padding_mm',500),('min_depth_pixels',10000)]:
        if type(c[key]) not in (int,float) or not np.isfinite(c[key]) or not 0<c[key]<=limit:
            raise ValueError(f'Invalid {key}')
    patch=np.asarray(c['cart_reference_patch'],float)
    if patch.shape!=(4,) or not np.isfinite(patch).all() or not (0<=patch[0]<patch[2]<=1 and 0<=patch[1]<patch[3]<=1):
        raise ValueError('Invalid cart reference patch')
    z=np.asarray(c['base_z_offsets_mm'],float)
    if z.shape!=(2,) or not np.isfinite(z).all() or not (-500<=z[0]<z[1]<=1000):raise ValueError('Invalid cart height range')
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


def measured_points(mask,depth,k,c2w,config,max_spread):
    valid=mask & np.isfinite(depth) & (depth>0)
    yy,xx=np.nonzero(valid)
    fraction=len(xx)/max(1,int(mask.sum()))
    if len(xx)<config['min_depth_pixels'] or fraction<config['min_depth_fraction']:
        raise ValueError('有效深度不足')
    z=depth[yy,xx].astype(float)
    # Same depth-unit convention as the existing book geometry implementation.
    if np.median(z)>20:z*=.001
    q=np.percentile(z,[10,90])*1000
    if q[1]-q[0]>max_spread:raise ValueError('深度离散过大，前景与背景无法可靠区分')
    rays=np.linalg.solve(k,np.array([xx,yy,np.ones(len(xx))]))
    if not np.isfinite(rays).all() or np.any(rays[2]<=0):raise ValueError('相机射线无效')
    translation=c2w[:3,3].copy()
    if np.linalg.norm(translation)>10:translation*=.001
    points=((rays/rays[2]*z).T@c2w[:3,:3].T+translation)*1000
    return points,dict(depth_median_mm=float(np.median(z)*1000),depth_p10_p90_mm=q.tolist(),valid_fraction=fraction)


def detect_candidates(detector,rgb,depth,k,c2w,config,checkpoint=lambda:None):
    """Uses detector input already prepared for this exact RGB frame."""
    depth=np.asarray(depth,float);k=np.asarray(k,float);c2w=np.asarray(c2w,float)
    diagnostics=dict(stage='cart_detection',boxes=[])
    def fail(message):raise CandidateError(message,diagnostics)
    if depth.ndim!=2 or np.asarray(rgb).shape!=(*depth.shape,3):fail('RGB 与深度未对齐')
    if k.shape!=(3,3) or c2w.shape!=(4,4) or not np.isfinite(k).all() or not np.isfinite(c2w).all():
        fail('相机内参或机器人基座外参无效')
    checkpoint()
    carts=clean_boxes(detector.detect_boxes_prepared(config['cart_prompt'],box_threshold=config['cart_score'],
        text_threshold=config['text_score']),depth.shape,config['cart_score'],config['nms_iou'])
    checkpoint()
    diagnostics['cart_detections']=[dict(score=s,box=b.tolist()) for s,b in carts]
    if not carts:fail('没有检测到推车，本轮不选书')
    books=clean_boxes(detector.detect_boxes_prepared(config['book_prompt'],box_threshold=config['book_score'],
        text_threshold=config['text_score']),depth.shape,config['book_score'],config['nms_iou'])
    checkpoint()
    # DINO may call the background robot or a book stand a trolley. Require
    # book-box support, and suppress lower-score nested cart/stand boxes.
    supported=[]
    for score,box in carts:
        region=box.copy();region[1]=max(0,int(box[1]-(box[3]-box[1])*config['cart_top_extension']))
        count=sum(overlap(book,region)/np.prod(book[2:]-book[:2])>=config['min_box_cart_overlap'] for _,book in books)
        if count:supported.append((score,box))
    supported=[(s,b) for s,b in supported if not any(
        s2>s and overlap(b,b2)/np.prod(b[2:]-b[:2])>=.9
        for s2,b2 in supported)]
    diagnostics['supported_cart_boxes']=[dict(score=s,box=b.tolist()) for s,b in supported]
    if not supported:fail('没有检测到包含书本候选的推车，请重新感知')
    if len(supported)>1 and supported[0][0]-supported[1][0]<config['cart_score_margin']:
        fail('检测到多辆难以区分的推车，请调整画面后重试')
    cart=supported[0][1];diagnostics['cart_box']=cart.tolist()
    region=cart.copy();region[1]=max(0,int(cart[1]-(cart[3]-cart[1])*config['cart_top_extension']))
    diagnostics['cart_work_roi']=region.tolist()
    region_detector=getattr(detector,'detect_boxes_region',None)
    diagnostics['full_frame_book_count']=len(books)
    if callable(region_detector):
        extra=clean_boxes(region_detector(rgb,region,config['book_prompt'],box_threshold=config['book_score'],
            text_threshold=config['text_score']),depth.shape,config['book_score'],config['nms_iou'])
        checkpoint()
        diagnostics['cart_crop_book_count']=len(extra)
        combined=books+extra
        if combined:
            books=clean_boxes((np.array([b for _,b in combined]),np.array([s for s,_ in combined]),[]),
                              depth.shape,config['book_score'],config['nms_iou'])
    inside=[]
    for score,box in books:
        coverage=overlap(box,region)/np.prod(box[2:]-box[:2])
        entry=dict(box=box.tolist(),score=score,cart_overlap=float(coverage),status='outside_cart_roi')
        diagnostics['boxes'].append(entry)
        if coverage>=config['min_box_cart_overlap']:inside.append((score,box,entry))
    # Observe the cart independently of the candidate books; do not use book
    # depth itself as its own proof of being on the cart.
    f=np.asarray(config['cart_reference_patch']);size=cart[2:]-cart[:2]
    refbox=np.r_[cart[:2]+f[:2]*size,cart[:2]+f[2:]*size].astype(int)
    ref=box_mask(refbox,depth.shape)
    for _,box in books:ref &= ~box_mask(box,depth.shape)
    diagnostics['reference_box']=refbox.tolist()
    try:
        points,stats=measured_points(ref,depth,k,c2w,config,config['cart_max_depth_spread_mm'])
    except (ValueError,np.linalg.LinAlgError) as e:fail(f'推车参考区域不可靠：{e}')
    lo,hi=np.percentile(points,[5,95],axis=0)
    lo[:2]-=config['base_xy_padding_mm'];hi[:2]+=config['base_xy_padding_mm']
    z0=float(np.median(points[:,2]));lo[2]=z0+config['base_z_offsets_mm'][0];hi[2]=z0+config['base_z_offsets_mm'][1]
    dlo,dhi=np.array(stats['depth_p10_p90_mm'])+[-config['depth_padding_mm'],config['depth_padding_mm']]
    diagnostics.update(stage='cart_depth_filter',reference_depth=stats,base_bounds_mm=[lo.tolist(),hi.tolist()],
                       accepted_depth_mm=[float(dlo),float(dhi)])
    accepted=[];uncertain=[]
    for score,box,entry in inside:
        inset=(box[2:]-box[:2])*.2
        centerbox=np.r_[np.floor(box[:2]+inset),np.ceil(box[2:]-inset)].astype(int)
        try:
            points,stats=measured_points(box_mask(centerbox,depth.shape),depth,k,c2w,config,config['book_max_depth_spread_mm'])
            center=np.median(points,axis=0)
        except (ValueError,np.linalg.LinAlgError) as e:
            entry.update(status='uncertain_depth',reason=str(e));uncertain.append(entry);continue
        entry.update(stats,base_xyz_mm=center.tolist())
        if not dlo<=stats['depth_median_mm']<=dhi or np.any(center<lo) or np.any(center>hi):
            entry['status']='outside_cart_depth_volume';continue
        entry['status']='accepted'
        mask=box_mask(box,depth.shape)
        accepted.append(dict(mask=mask,book_mask=mask,box=box.tolist(),uv=((box[:2]+box[2:])/2).tolist(),
            score=score,base_y_mm=float(center[1]),base_xyz_mm=center.tolist(),depth_median_mm=stats['depth_median_mm'],
            recognition='book_box',geometry=None,geometry_status='not_checked',error='',color='unclassified'))
    if uncertain:fail('车内候选存在不可靠深度，无法确定完整选书顺序；请重新感知')
    accepted.sort(key=lambda c:-c['base_y_mm'])
    for i,item in enumerate(accepted,1):item['id']=f'book_{i}'
    diagnostics.update(stage='candidates_ready',candidate_count=len(accepted))
    if not accepted:fail('推车区域和深度筛选后，没有可用书本候选')
    return accepted,diagnostics


def annotate_gate(rgb,diagnostics):
    image=np.asarray(rgb).copy()
    for name,color in [('cart_box',(30,220,255)),('reference_box',(255,220,40))]:
        if name in diagnostics:
            b=diagnostics[name];cv2.rectangle(image,tuple(b[:2]),tuple(b[2:]),color,2)
            cv2.putText(image,name,tuple([b[0],max(18,b[1]-5)]),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
    for item in diagnostics.get('boxes',[]):
        b=item['box'];status=item['status'];color=(50,240,80) if status=='accepted' else (255,100,80)
        cv2.rectangle(image,tuple(b[:2]),tuple(b[2:]),color,1)
        cv2.putText(image,status,(b[0],min(image.shape[0]-5,b[3]+14)),cv2.FONT_HERSHEY_SIMPLEX,.35,color,1)
    return image
