"""Non-planar V4.1 geometry: model-selected pixels, measured local RGB-D.

The model identifies surfaces; it never supplies metric depth or arm commands.
Only a nearby interior patch supplies depth for each semantic landmark.
"""
import cv2
import numpy as np
from book_top_tilt_perception import rays


class LocalDepthError(ValueError):
    """Selected image location has no usable measured local depth."""


def decode_depth_anchors(response,selection,shape):
    result=dict(selection)
    scale=np.array([shape[1]-1,shape[0]-1])/1000.
    for name,landmark,quad in (
            ('top_depth_uv',np.mean(selection['spine_quad'][:2],axis=0),selection['book_quad']),
            ('center_depth_uv',np.mean(selection['spine_quad'],axis=0),selection['spine_quad'])):
        uv=np.asarray(response.get(name),float)
        if uv.shape!=(2,) or not np.isfinite(uv).all() or (uv<0).any() or (uv>1000).any():
            raise ValueError('GPT depth review returned invalid '+name)
        uv=uv*scale
        if cv2.pointPolygonTest(np.asarray(quad,np.float32),tuple(uv),False)<0:
            raise ValueError('GPT depth anchor outside selected surface: '+name)
        if np.linalg.norm(uv-landmark)>8.:
            raise ValueError('GPT local depth anchor too far from landmark: '+name)
        result[name]=uv
    return result


def local_landmark(landmark,anchor,quad,depth,k,d,c2w,*,quality_warning_only=False):
    mask=np.zeros(depth.shape,np.uint8)
    cv2.fillPoly(mask,[np.rint(quad).astype(np.int32)],1)
    x,y=np.rint(anchor).astype(int);h,w=depth.shape
    x0,x1=max(0,x-3),min(w,x+4);y0,y1=max(0,y-3),min(h,y+4)
    patch=np.asarray(depth[y0:y1,x0:x1],float);region=mask[y0:y1,x0:x1]>0
    # Approximate regions guide sampling, but cannot veto an explicit model
    # landmark. Do not search beyond this local window for unrelated depth.
    values=patch[region] if region.sum()>=9 else patch.reshape(-1)
    good=np.isfinite(values)&(values>100)&(values<2500)
    # An upper-edge window can contain background/invalid pixels on one side.
    # Require measured support, not an arbitrary filled-area percentage.
    warnings=[]
    if not good.any():
        raise LocalDepthError('GPT-selected local patch has insufficient measured depth')
    if good.sum()<9:
        if not quality_warning_only:
            raise LocalDepthError('GPT-selected local patch has insufficient measured depth')
        warnings.append('fewer_than_9_valid_depth_points')
    z=values[good];median=float(np.median(z));mad=float(np.median(abs(z-median)))
    if mad>6.:
        if not quality_warning_only:
            raise LocalDepthError('GPT-selected local depth is mixed or unstable: MAD>6mm')
        warnings.append('local_depth_MAD_exceeds_6mm')
    keep=abs(z-median)<=max(2.,3.*mad)
    if keep.sum()<9 or keep.mean()<.5:
        if not quality_warning_only:
            raise LocalDepthError('GPT-selected local depth has insufficient consistent support')
        warnings.append('insufficient_consistent_support')
    # Sparse valid samples may straddle the median without any point surviving
    # the filter. Use only those original valid measurements in that case.
    if not keep.any():keep=np.ones(z.shape,dtype=bool)
    measured=float(np.median(z[keep]))
    point=rays(np.asarray(landmark)[None],k,d)[0]*measured
    base=point@np.asarray(c2w)[:3,:3].T+np.asarray(c2w)[:3,3]
    return base,dict(depth_mm=measured,mad_mm=mad,support_points=int(keep.sum()),
                     quality_warnings=warnings,quality_policy='warn_only' if quality_warning_only else 'strict',
                     valid_fraction=float(good.mean()),
                     landmark_uv=np.asarray(landmark),depth_anchor_uv=np.asarray(anchor),
                     patch_radius_px=3,depth_transfer_distance_px=float(np.linalg.norm(np.asarray(landmark)-anchor)))


def local_spine_geometry(selection,depth,k,d,c2w,*,quality_warning_only=False):
    q=selection['spine_quad']
    top_uv=np.asarray(selection.get('top_center_uv',np.mean(q[:2],axis=0)))
    center_uv=np.asarray(selection.get('spine_center_uv',np.mean(q,axis=0)))
    # Live selection requires explicit points; means allow old snapshot replay.
    top,top_info=local_landmark(top_uv,selection.get('top_depth_uv',top_uv),
                              selection['book_quad'],depth,k,d,c2w,quality_warning_only=quality_warning_only)
    center,center_info=local_landmark(center_uv,selection.get('center_depth_uv',center_uv),
                                    q,depth,k,d,c2w,quality_warning_only=quality_warning_only)
    return dict(top_base_mm=top,spine_center_base_mm=center,
                geometry_method='gpt_landmarks_local_rgbd',plane_residual_mm=None,
                local_depth=dict(top=top_info,center=center_info))
