"""V4.1 distance-aware GPT reselection; image regions only, no arm commands."""
import json
import numpy as np
from scipy.spatial.transform import Rotation


def base_transform(c2w):
    transform=np.asarray(c2w,float).copy()
    if transform.shape!=(4,4) or not np.isfinite(transform).all():
        raise ValueError('Invalid head transform for distance selection')
    if np.linalg.norm(transform[:3,3])<10:transform[:3,3]*=1000.
    return transform


def distance_report(selected,c2w,pose,config):
    if pose is None:raise ValueError('No measured right pose for book distance selection')
    tcp,angles=map(lambda value:np.asarray(value,float),pose)
    if tcp.shape!=(3,) or angles.shape!=(3,) or not np.isfinite([tcp,angles]).all():
        raise ValueError('Invalid right pose for book distance selection')
    r=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
    finger_offset=r@np.asarray(config['tcp_to_finger_tool_mm'],float)
    transform=base_transform(c2w)
    center=transform[:3,:3]@(np.asarray(selected['point_camera_m'])*1000.)+transform[:3,3]
    dy=float(center[1]-tcp[1]-finger_offset[1])
    dx=float(selected['book_surface_x_mm']+config['forward_insertion_mm']-finger_offset[0]-tcp[0])
    if not np.isfinite([*center,dx,dy]).all():raise ValueError('Non-finite selected-book distance')
    reasons=[]
    if abs(dy)>config['max_y_travel_mm']+1e-6:reasons.append('Y travel exceeds range')
    if dx>config['max_forward_mm']+1e-6:reasons.append('X forward exceeds range')
    if dx<-.01:reasons.append('X endpoint behind current TCP')
    return dict(accepted=not reasons,reasons=reasons,center_base_mm=center.tolist(),
                y_offset_mm=dy,forward_mm=dx,current_tcp_mm=tcp.tolist(),
                max_y_travel_mm=config['max_y_travel_mm'],max_forward_mm=config['max_forward_mm'])


def search_box(depth,intrinsic,c2w,pose,config):
    """Advisory image bbox from depth points inside the coarse XY budgets, not IK."""
    d=np.asarray(depth,float)
    yy,xx=np.nonzero(np.isfinite(d)&(d>0))
    if len(xx)==0:raise ValueError('No valid depth for near-book search')
    z=d[yy,xx]
    if np.median(z)<20:z=z*1000.
    rays=np.linalg.solve(np.asarray(intrinsic,float),np.array([xx,yy,np.ones(len(xx))]))
    transform=base_transform(c2w)
    base=(rays/rays[2]*z).T@transform[:3,:3].T+transform[:3,3]
    tcp,angles=pose
    offset=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()@np.asarray(config['tcp_to_finger_tool_mm'])
    dx=base[:,0]+config['forward_insertion_mm']-offset[0]-tcp[0]
    dy=base[:,1]-offset[1]-tcp[1]
    valid=np.isfinite(base).all(axis=1)&(abs(dy)<=config['max_y_travel_mm'])&(dx>=0)&(dx<=config['max_forward_mm'])
    if not valid.any():raise ValueError('No measured region inside coarse XY travel budgets')
    h,w=d.shape
    return [float(xx[valid].min())/(w-1)*1000,float(yy[valid].min())/(h-1)*1000,
            float(xx[valid].max())/(w-1)*1000,float(yy[valid].max())/(h-1)*1000]


def review_distance(selected,book_mask,rgb,depth,intrinsic,c2w,*,pose_reader,config,
                    box_detector=None,checkpoint=lambda:None,recover_fn=None):
    from cart_book_grasp_v3_2_head_fallback import recover,center_from_mask,direct_book_mask
    from cart_book_grasp_v1_3_depth import whole_book_depth
    pose=pose_reader()
    before=distance_report(selected,c2w,pose,config)
    if before['accepted']:return selected,book_mask
    mask=np.asarray(selected['mask'],bool);yy,xx=np.nonzero(mask);h,w=mask.shape
    context=dict(mode='reselect_near_book',reason='selected_book_outside_coarse_travel',
        distance_report=before,
        rejected_box_0_1000=[float(xx.min())/(w-1)*1000,float(yy.min())/(h-1)*1000,
                           float(xx.max())/(w-1)*1000,float(yy.max())/(h-1)*1000],
        search_box_0_1000=search_box(depth,intrinsic,c2w,pose,config))
    print('V4.1 选书超出粗定位距离，交给 GPT-6 重选近处书：'+json.dumps(context,ensure_ascii=False),flush=True)
    checkpoint()
    detections,books=(recover_fn or recover)(rgb,box_detector,checkpoint=checkpoint,review_context=context)
    pose=pose_reader()  # Re-evaluate with current feedback after the model wait.
    candidates=[];rejections=[]
    for score,new_mask,_ in detections:
        try:
            candidate=center_from_mask(new_mask,depth,intrinsic)
            full=direct_book_mask(books,candidate['mask'].shape)
            if full is None:raise ValueError('GPT reselected spine has no matching whole-book mask')
            candidate.update(whole_book_depth(full,depth,intrinsic,c2w))
            candidate.update(score=float(score),selection_source='gpt6_polygon_fallback')
            after=distance_report(candidate,c2w,pose,config)
            if not after['accepted']:
                rejections.append(after);continue
            candidate['distance_reselection']=dict(before=before,after=after,policy='one_gpt_reselection')
            candidates.append((candidate,full))
        except (ValueError,np.linalg.LinAlgError) as error:rejections.append(str(error))
    checkpoint()
    if not candidates:
        raise ValueError('GPT near-book reselection has no depth-supported candidate within travel limits: '+json.dumps(rejections,ensure_ascii=False))
    chosen,full=max(candidates,key=lambda item:item[0]['score'])
    print('V4.1 GPT 近处选书通过距离检查：'+json.dumps(chosen['distance_reselection'],ensure_ascii=False),flush=True)
    return chosen,full
