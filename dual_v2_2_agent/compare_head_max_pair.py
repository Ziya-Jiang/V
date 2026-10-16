"""Same-capture native-vs-downsampled matching. Offline, no robot/camera imports."""
import argparse,json,time,hashlib
from pathlib import Path
import cv2,numpy as np,torch
from compare_v3_superpoint import Models
from compare_v3_xfeat import rgb,in_mask,evaluate,overlay

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('capture',type=Path)
    args=parser.parse_args();out=args.capture/'matching';out.mkdir(exist_ok=True)
    meta=json.loads((args.capture/'capture.json').read_text());model=Models()
    torch.set_num_threads(4);cv2.setNumThreads(4)
    # Manually outlined visible cream/red/black books in this fixed capture only.
    # This is a diagnostic source region, NOT a target identity ground truth.
    polygons=[[[513,480],[530,476],[553,681],[537,684]],
              [[544,476],[562,471],[619,658],[603,668]],
              [[591,484],[643,470],[695,520],[650,657],[629,661]]]
    report=dict(motion_sent=False,camera_opened=False,capture_directory=str(args.capture.resolve()),
        native_profile=meta['selected_profile'],
        note='Current wrist views do not contain the target books. Global matches are background evidence only; book success cannot be evaluated. Low resolution is downsampled from the SAME native frame; no new scene acquisition for low images. Source book polygons are manually marked diagnostic ROIs.',rows=[])
    for pair in meta['pairs']:
        name=pair['name'];head=rgb(args.capture/f'{name}_head_native.png');wrist=rgb(args.capture/f'{name}_wrist_undistorted.png')
        hm=np.zeros(head.shape[:2],np.uint8)
        for poly in polygons:cv2.fillConvexPoly(hm,np.int32(poly),1)
        for resolution in ['native_1280x800','downsample_848x530']:
            src=head if resolution.startswith('native') else rgb(args.capture/f'{name}_head_848.png')
            mask=hm if resolution.startswith('native') else cv2.resize(hm,(848,530),interpolation=cv2.INTER_NEAREST)
            for method in ['SuperPoint_LightGlue','SuperPoint_LightGlue_ROI']:
                start=time.perf_counter()
                pairs,info=model.match(src,wrist,mask,method)
                ys,xs=np.nonzero(mask);controls=[[float(np.median(xs)),float(np.median(ys))]]
                result=evaluate(pairs,mask,controls,'head',wrist.shape)
                result.update(pair=name,resolution=resolution,method=method,seconds=time.perf_counter()-start,
                              interpretation='No books visible in wrist: not a positive identity test',**info)
                image=f'{name}_{resolution}_{method}.png';result['image']=image
                overlay(src,wrist,mask,result,name+' '+resolution+' '+method,out/image)
                report['rows'].append(result)
                print(name,resolution,method,'features',result['source_roi_features'],'matches',result['matches'],'inliers',result['inliers'],flush=True)
            # Global matches use only native coordinates and never authorize book identity.
            pairs,info=model.match(src,wrist,np.ones(src.shape[:2],np.uint8),'SuperPoint_LightGlue')
            report['rows'].append(dict(pair=name,resolution=resolution,method='SuperPoint_LightGlue_global',matches=len(pairs),interpretation='background matches only',**info))
        (out/'report.json').write_text(json.dumps(report,indent=2))
    print(out/'report.json')

if __name__=='__main__':main()
