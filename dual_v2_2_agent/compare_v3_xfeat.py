"""Offline XFeat/SIFT comparison on saved V3 images. No hardware imports/actions.

Run with a PyTorch environment, e.g. sam3_full/bin/python. The official XFeat
checkout and weights must already be present under third_party/accelerated_features.
No downloads, installation, camera acquisition or robot connection occur here.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault('CUDA_VISIBLE_DEVICES', '')
os.environ.setdefault('TORCH_DEVICE_BACKEND_AUTOLOAD', '0')
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent


def rgb(path):
    a=cv2.imread(str(path))
    if a is None:raise ValueError(f'Cannot read {path}')
    return cv2.cvtColor(a,cv2.COLOR_BGR2RGB)


def in_mask(points,mask):
    p=np.rint(points).astype(int)
    valid=(p[:,0]>=0)&(p[:,0]<mask.shape[1])&(p[:,1]>=0)&(p[:,1]<mask.shape[0])
    idx=np.flatnonzero(valid)
    valid[idx]=mask[p[idx,1],p[idx,0]]!=0
    return valid


def roi_image(source,mask):
    y,x=np.nonzero(mask)
    x0=max(0,int(x.min())-24);y0=max(0,int(y.min())-24)
    x1=min(source.shape[1],int(x.max())+25);y1=min(source.shape[0],int(y.max())+25)
    crop=source[y0:y1,x0:x1]
    # Interpolation ablation, not new visual information. Used for BOTH methods.
    scale=min(4.,max(1.,480/max(crop.shape[:2])))
    out=cv2.resize(crop,None,fx=scale,fy=scale,interpolation=cv2.INTER_LINEAR)
    sx=out.shape[1]/crop.shape[1];sy=out.shape[0]/crop.shape[0]
    return out,np.array([sx,sy]),np.array([x0,y0])


def sift_matches(source,dest,mask,kind,roi=False):
    detector=cv2.SIFT_create(nfeatures=6000 if kind=='final' or roi else 2500,
                             contrastThreshold=.008 if kind=='final' or roi else .02)
    if roi:
        image,scale,offset=roi_image(source,mask)
        k,d=detector.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_RGB2GRAY),None)
        points=np.array([p.pt for p in k]).reshape(-1,2)/scale+offset
        valid=in_mask(points,mask)
        groups=[(points[valid],None if d is None else d[valid])]
    else:
        groups=[]
        interior=cv2.erode(mask,np.ones((5,5),np.uint8)) if kind=='head' else mask
        gray=cv2.cvtColor(source,cv2.COLOR_RGB2GRAY)
        for scale in ((.15,.2,.25,.3,.4,.5,.65,1.) if kind=='final' else (1.,)):
            small=cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
            m=cv2.resize(interior,(small.shape[1],small.shape[0]),interpolation=cv2.INTER_NEAREST)
            k,d=detector.detectAndCompute(small,m*255)
            groups.append((np.array([p.pt for p in k]).reshape(-1,2)/scale,d))
    k,d=detector.detectAndCompute(cv2.cvtColor(dest,cv2.COLOR_RGB2GRAY),None)
    destpoints=np.array([p.pt for p in k]).reshape(-1,2)
    candidates=[];matcher=cv2.BFMatcher(cv2.NORM_L2)
    for points,desc in groups:
        if desc is None or d is None or min(len(desc),len(d))<2:continue
        rev=matcher.match(d,desc)
        for pair in matcher.knnMatch(desc,d,k=2):
            if len(pair)!=2:continue
            m,n=pair
            ratio=.7 if kind=='final' or roi else .65
            if n.distance>0 and m.distance<ratio*n.distance and rev[m.trainIdx].trainIdx==m.queryIdx:
                candidates.append((m.distance/n.distance,points[m.queryIdx],destpoints[m.trainIdx]))
    candidates.sort(key=lambda item:item[0])
    return [(a,b) for _,a,b in candidates],sum(len(p) for p,_ in groups)


def xfeat_matches(model,source,dest,mask,dense=False,roi=False,glue=False):
    if roi:image,scale,offset=roi_image(source,mask)
    else:image,scale,offset=source,np.ones(2),np.zeros(2)
    if glue:
        d0=model.detectAndCompute(model.parse_input(image))[0]
        d1=model.detectAndCompute(model.parse_input(dest))[0]
        if not len(d0['keypoints']) or not len(d1['keypoints']):return [],None
        d0['image_size']=(image.shape[1],image.shape[0])
        d1['image_size']=(dest.shape[1],dest.shape[0])
        a,b,_=model.match_lighterglue(d0,d1,min_conf=.1)
    elif dense:
        a,b=model.match_xfeat_star(image,dest,top_k=4096)
    else:
        a,b=model.match_xfeat(image,dest,top_k=4096,min_cossim=.82)
    a=a/scale+offset
    keep=in_mask(a,mask)
    return list(zip(a[keep],b[keep])),None


def xfeat_multiscale(model,source,dest,mask,dense=False):
    pairs=[]
    for scale in (.15,.2,.25,.3,.4,.5,.65,1.):
        small=cv2.resize(source,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
        m=cv2.resize(mask,(small.shape[1],small.shape[0]),interpolation=cv2.INTER_NEAREST)
        found,_=xfeat_matches(model,small,dest,m,dense=dense)
        ratio=np.array([small.shape[1]/source.shape[1],small.shape[0]/source.shape[0]])
        pairs.extend((a/ratio,b) for a,b in found)
    return pairs,None


def evaluate(pairs,mask,controls,kind,dest_shape):
    # Suppress repeated orientations / pyramid samples at the same location.
    unique=[]
    for a,b in pairs:
        if all(np.linalg.norm(a-u)>(5 if kind=='final' else 1) and np.linalg.norm(b-v)>2 for u,v in unique):
            unique.append((a,b))
    a=np.float32([p[0] for p in unique]).reshape(-1,2)
    b=np.float32([p[1] for p in unique]).reshape(-1,2)
    result=dict(matches=len(a),inliers=0,inlier_ratio=0.,rms_px=None,control_supported=False,
                screen_pass=False,reasons=[],source_points=a.tolist(),destination_points=b.tolist(),inlier_indices=[])
    if len(a)<4:
        result['reasons']=['fewer_than_4_correspondences'];return result
    cv2.setRNGSeed(7)
    H,keep=cv2.findHomography(a,b,cv2.RANSAC,2.,maxIters=10000,confidence=.999)
    if H is None or keep is None or not np.isfinite(H).all():
        result['reasons']=['homography_failed'];return result
    keep=keep.ravel().astype(bool)
    n=int(keep.sum());result.update(inliers=n,inlier_ratio=float(keep.mean()),homography=H.tolist(),
                                  inlier_indices=np.flatnonzero(keep).tolist())
    if n<4:result['reasons']=['fewer_than_4_inliers'];return result
    projected=cv2.perspectiveTransform(a[None],H)[0]
    result['rms_px']=float(np.sqrt(np.mean(np.sum((projected[keep]-b[keep])**2,axis=1))))
    hull=cv2.convexHull(a[keep])
    result['control_supported']=all(cv2.pointPolygonTest(hull,tuple(map(float,p)),False)>=0 for p in controls)
    result['source_hull_mask_fraction']=float(cv2.contourArea(hull)/np.count_nonzero(mask))
    transformed=cv2.perspectiveTransform(np.float32(controls)[None],H)[0]
    result['mapped_controls']=transformed.tolist()
    y,x=np.nonzero(mask)
    quad=np.float32(controls) if kind=='final' else np.float32([[x.min(),y.min()],[x.max(),y.min()],
                                                            [x.max(),y.max()],[x.min(),y.max()]])
    mapped=cv2.perspectiveTransform(quad[None],H)[0]
    ratio=cv2.contourArea(mapped,oriented=True)/cv2.contourArea(quad,oriented=True)
    result['area_ratio']=float(ratio)
    h,w=dest_shape[:2]
    checks={
        'too_few_inliers':n<12,
        'low_inlier_ratio':keep.mean()<(.65 if kind=='head' else .6),
        'high_reprojection_error':result['rms_px']>1.5,
        'unsupported_control_points':not result['control_supported'],
        'poor_spatial_coverage':np.any(np.ptp(a[keep],axis=0)<np.array([x.max()-x.min(),y.max()-y.min()])*[.25,.3]) if kind=='head' else False,
        'invalid_projected_patch':not np.isfinite(mapped).all() or not cv2.isContourConvex(mapped) or
                                  np.any(mapped<[3,3]) or np.any(mapped>[w-4,h-4]),
        'invalid_scale':not ((.05<=ratio<=20) if kind=='head' else (.005<=ratio<=2)),
    }
    result['reasons']=[key for key,bad in checks.items() if bad]
    result['screen_pass']=not result['reasons']
    return result


def overlay(source,dest,mask,result,title,path):
    # Original-coordinate evidence; scale only the composed display uniformly.
    h=max(source.shape[0],dest.shape[0]);w=source.shape[1]+dest.shape[1]
    out=np.full((h+54,w,3),25,np.uint8)
    out[54:54+source.shape[0],:source.shape[1]]=source
    out[54:54+dest.shape[0],source.shape[1]:]=dest
    contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out,contours,-1,(255,210,30),1,offset=(0,54))
    keep=set(result['inlier_indices'])
    for i,(a,b) in enumerate(zip(result['source_points'],result['destination_points'])):
        color=(30,225,140) if i in keep else (220,75,75)
        a=tuple(np.rint(np.array(a)+[0,54]).astype(int))
        b=tuple(np.rint(np.array(b)+[source.shape[1],54]).astype(int))
        cv2.line(out,a,b,color,1);cv2.circle(out,a,2,color,-1);cv2.circle(out,b,2,color,-1)
    line=f'{title} | inliers {result["inliers"]}/{result["matches"]} | screen={result["screen_pass"]}'
    cv2.putText(out,line,(12,23),cv2.FONT_HERSHEY_SIMPLEX,.6,(245,245,245),1)
    cv2.putText(out,'Green: RANSAC inliers, red: rejected. Geometric fit is NOT physical identity proof.',(12,44),cv2.FONT_HERSHEY_SIMPLEX,.5,(220,220,220),1)
    cv2.imwrite(str(path),cv2.cvtColor(out,cv2.COLOR_RGB2BGR))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--methods',nargs='+',default=['SIFT_current','SIFT_ROI','XFeat_sparse',
        'XFeat_semi_dense','XFeat_sparse_ROI','XFeat_semi_dense_ROI'],choices=['SIFT_current','SIFT_ROI',
        'XFeat_sparse','XFeat_semi_dense','XFeat_sparse_ROI','XFeat_semi_dense_ROI',
        'XFeat_lighterglue','XFeat_lighterglue_ROI','XFeat_sparse_multiscale','XFeat_semi_dense_multiscale'])
    parser.add_argument('--pairs',nargs='+')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    import torch
    torch.set_num_threads(4);cv2.setNumThreads(4)
    repo=ROOT/'third_party/accelerated_features'
    sys.path.insert(0,str(ROOT/'third_party/xfeat_deps'))
    sys.path.insert(0,str(repo))
    from modules.xfeat import XFeat
    weights=repo/'weights/xfeat.pt'
    started=time.perf_counter()
    model=XFeat(weights=torch.load(weights,map_location='cpu',weights_only=True),top_k=4096)
    model.dev=torch.device('cpu');model.net=model.net.cpu()
    model.detectAndCompute(np.random.default_rng(7).integers(0,256,(64,64,3),dtype=np.uint8))
    report=dict(motion_sent=False,device='cpu',torch_version=torch.__version__,opencv_version=cv2.__version__,
                threads=4,weight_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
                model_load_warmup_seconds=time.perf_counter()-started,
                note='Head scene 57 and diagnostic wrist were captured at different times. No synchronized identity ground truth. '
                     'ROI means source crop with 24px context, upsample up to 4x, same preprocessing for SIFT and XFeat. '
                     'Timing is one CPU call including source extraction; not a latency benchmark. Screen is geometric only.',
                rows=[])
    out=ROOT/'RobotRAM/output'
    config=json.loads((ROOT/'cart_book_grasp_v3_approach.json').read_text())
    ref=rgb(ROOT/config['reference']['image']);mask=np.zeros(ref.shape[:2],np.uint8)
    cv2.fillConvexPoly(mask,np.int32(config['reference']['mask_polygon']),1)
    current=rgb(out/'v3_texture_debug_20261009.png');old=rgb(out/'v3_observation_before_stage2_20261009.png')
    head=rgb(out/'scene_57_pcl/rgb_ext.png')
    hm=np.load(out/'scene_57_pcl/detections.npy',allow_pickle=True).item()['masks'][0].astype(np.uint8)
    headgeo=json.loads((out/'scene_57_pcl/book_grasp_geometry.json').read_text())
    jobs=[('head_to_current','head',head,current,hm,[headgeo['spine_center_uv']]),
          ('final_to_current','final',ref,current,mask,config['reference']['points']),
          ('final_to_old_observation','final',ref,old,mask,config['reference']['points']),
          ('negative_red_book','final',ref,old[60:610,790:930].copy(),mask,config['reference']['points'])]
    known=np.array([[.9,0,20],[0,.9,25],[0,0,1]],float)
    jobs.append(('positive_known_warp','final',ref,cv2.warpPerspective(ref,known,(1280,720)),mask,config['reference']['points']))
    if args.pairs:jobs=[j for j in jobs if j[0] in args.pairs]
    if not jobs:raise ValueError('No matching test pairs')
    for name,kind,source,dest,mask,controls in jobs:
        for method in args.methods:
            started=time.perf_counter()
            if method.startswith('SIFT'):
                pairs,count=sift_matches(source,dest,mask,kind,roi=method.endswith('ROI'))
            elif method.endswith('multiscale'):
                pairs,count=xfeat_multiscale(model,source,dest,mask,dense='semi_dense' in method)
            else:
                pairs,count=xfeat_matches(model,source,dest,mask,dense='semi_dense' in method,
                    roi=method.endswith('ROI'),glue='lighterglue' in method)
            result=evaluate(pairs,mask,controls,kind,dest.shape)
            result.update(pair=name,method=method,source_features=count,seconds=time.perf_counter()-started)
            if name=='positive_known_warp' and 'mapped_controls' in result:
                truth=cv2.perspectiveTransform(np.float32(controls)[None],known)[0]
                result['ground_truth_control_max_error_px']=float(np.linalg.norm(np.array(result['mapped_controls'])-truth,axis=1).max())
            if method=='SIFT_current':
                try:
                    if kind=='head':
                        from cart_book_grasp_v3_tracking import LockedBookTracker
                        tracker=LockedBookTracker(source,mask,controls[0],cross_view=True)
                    else:
                        from cart_book_grasp_v3_final_tracking import FinalImageTracker
                        tracker=FinalImageTracker(source,config['reference']['roi'],controls,config['reference']['mask_polygon'])
                    tracker.observe(dest);result['existing_v3_pass']=True
                except ValueError as error:
                    result.update(existing_v3_pass=False,existing_v3_error=str(error))
            image_name=name+'__'+method+'.png'
            overlay(source,dest,mask,result,name+' / '+method,args.output/image_name)
            result['image']=image_name;report['rows'].append(result)
            (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(name,method,result['matches'],result['inliers'],result['screen_pass'],result['reasons'],flush=True)
    print('Report:',args.output.resolve(),flush=True)


if __name__=='__main__':main()
