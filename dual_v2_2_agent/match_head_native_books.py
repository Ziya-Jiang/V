"""Offline same-frame head resolution comparison using explicit diagnostic book ROIs."""
import argparse,json,time
from pathlib import Path
from compare_v3_superpoint import Models
from compare_v3_xfeat import rgb,in_mask,evaluate,overlay,sift_matches
import cv2,numpy as np,torch

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('capture',type=Path);args=parser.parse_args()
    root=args.capture;out=root/'book_matching';out.mkdir(exist_ok=False)
    rois=json.loads((root/'diagnostic_rois.json').read_text());capture=json.loads((root/'capture.json').read_text())
    torch.set_num_threads(4);cv2.setNumThreads(4);model=Models()
    report=dict(motion_sent=False,robot_connected=False,camera_opened=False,native_profile=capture['selected_profile'],
        note='Offline same native frame versus 848x530 downsample, not a separate camera mode. Hand-outlined candidate ROIs for this observation only; not automatic segmentation or verified identity ground truth. Wrist matching uses undistorted_K. Screen thresholds unchanged from XFeat tests. Timing includes cache effects, not a benchmark.',
        source_rois=rois,rows=[],candidates=[])
    for pair in capture['pairs']:
        name=pair['name'];head=rgb(root/f'{name}_head_native.png');wrist=rgb(root/f'{name}_wrist_undistorted.png')
        targets=[]
        for book,poly in rois['wrist'].items():
            mask=np.zeros(wrist.shape[:2],np.uint8);cv2.fillConvexPoly(mask,np.int32(poly),1);targets.append((book,mask))
        for book,poly in rois['head'].items():
            mask=np.zeros(head.shape[:2],np.uint8);cv2.fillConvexPoly(mask,np.int32(poly),1)
            for resolution in ['1280x800','848x530']:
                if resolution=='1280x800':src,hm=head,mask
                else:src=rgb(root/f'{name}_head_848.png');hm=cv2.resize(mask,(848,530),interpolation=cv2.INTER_NEAREST)
                ys,xs=np.nonzero(hm);controls=[[float(np.median(xs)),float(np.median(ys))]]
                for method in ['SIFT_ROI','SuperPoint_LightGlue','SuperPoint_LightGlue_ROI']:
                    before=time.perf_counter()
                    if method=='SIFT_ROI':pairs,count=sift_matches(src,wrist,hm,'head',roi=True);info=dict(source_roi_features=count)
                    else:pairs,info=model.match(src,wrist,hm,method)
                    r=evaluate(pairs,hm,controls,'head',wrist.shape)
                    r.update(pair=name,book=book,resolution=resolution,method=method,seconds=time.perf_counter()-before,**info)
                    r['image']=f'{name}_{book}_{resolution}_{method}.png'
                    overlay(src,wrist,hm,r,f'{name} {book} {resolution} {method}',out/r['image'])
                    report['rows'].append(r)
                    for target,tm in targets:
                        p=np.array([b for a,b in pairs]).reshape(-1,2);keep=in_mask(p,tm)
                        sub=evaluate([v for v,k in zip(pairs,keep) if k],hm,controls,'head',wrist.shape)
                        sub.update(pair=name,book=book,resolution=resolution,method=method,wrist_candidate=target,
                            identity_texture_screen=bool(sub['inliers']>=12 and sub['inlier_ratio']>=.65 and sub['rms_px'] is not None and sub['rms_px']<=1.5 and sub.get('source_hull_mask_fraction',0)>=.03))
                        report['candidates'].append(sub)
                    print(name,book,resolution,method,r['inliers'],r['matches'],r['screen_pass'],flush=True)
                    (out/'report.json').write_text(json.dumps(report,indent=2))
    print(out/'report.json')

if __name__=='__main__':main()
