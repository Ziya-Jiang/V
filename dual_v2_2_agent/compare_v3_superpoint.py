"""Offline SuperPoint head/wrist study. No camera, network calls or robot imports.

Run after provisioning official weights in third_party/superpoint_cache.
"""
import os
from pathlib import Path
ROOT=Path(__file__).resolve().parent
os.environ['CUDA_VISIBLE_DEVICES']=''
os.environ['TORCH_DEVICE_BACKEND_AUTOLOAD']='0'
os.environ['TORCH_HOME']=str(ROOT/'third_party/superpoint_cache')
import argparse,hashlib,json,sys,time,subprocess,html
sys.path[:0]=[str(ROOT/'third_party/LightGlue'),str(ROOT/'third_party/superpoint_deps')]
import cv2,numpy as np,torch
from lightglue import SuperPoint,LightGlue
from compare_v3_xfeat import rgb,in_mask,roi_image,sift_matches,evaluate,overlay
from cart_book_grasp_v2_1_selection import load_config,build_observations

def paired_scenes():
    cfg=load_config(require_semantic_model=False)
    for scene in ['ae051ffdadef49df914c05114cc8119f','0dbdfb74c2d34b2fb3e7aeb5adbe43f2']:
        path=ROOT/'RobotRAM/output/v2_1'/scene
        head,wrist=rgb(path/'head_rgb.png'),rgb(path/'wrist_rgb.png')
        meta=json.loads((path/'camera.json').read_text())
        decision=json.loads((path/'decision.json').read_text())
        with np.load(path/'head_raw.npz') as d:
            assert str(d['frame_sha256'])==hashlib.sha256(head.tobytes()).hexdigest()
            raw=list(zip(d['scores'],d['masks'],d['boxes']))
        allhead=build_observations(raw,head,cfg,'head',np.load(path/'depth.npy'),np.array(meta['K']),np.array(meta['c2w']),compute_appearance=False)
        selected=[]
        for item in decision['catalog']:
            choices=[v for v in allhead if np.max(np.abs(np.array(v['box'])-item['box']))<2]
            assert len(choices)==1
            v=choices[0];v['id']=item['id'];selected.append(v)
        with np.load(ROOT/'RobotRAM/output/v3_identity_research_20261009'/f'{scene}_wrist_raw.npz') as d:
            assert str(d['frame_sha256'])==hashlib.sha256(wrist.tobytes()).hexdigest()
            raw=list(zip(d['scores'],d['masks'],d['boxes']))
        targets=build_observations(raw,wrist,cfg,'wrist',compute_appearance=False)
        for item in selected:
            yield dict(name=scene[:8]+'_'+item['id'],kind='head',source=head,dest=wrist,
                mask=item['book_mask'].astype(np.uint8),controls=[item['uv']],targets=targets,pair_gap=meta['pair_gap_seconds'],scene=scene)

class Models:
    def __init__(self):
        self.extractor=SuperPoint(max_num_keypoints=2048).eval()
        self.matcher=LightGlue(features='superpoint',depth_confidence=-1,width_confidence=-1).eval()
        self.cache={}
    @torch.inference_mode()
    def extract(self,img):
        key=(img.shape,hashlib.sha256(img.tobytes()).hexdigest())
        if key not in self.cache:
            tensor=torch.from_numpy(img.transpose(2,0,1).copy()).float()/255.
            self.cache[key]=self.extractor.extract(tensor,resize=None)
        return self.cache[key]
    @torch.inference_mode()
    def match(self,source,dest,mask,method):
        image,scale,offset=roi_image(source,mask) if method.endswith('_ROI') else (source,np.ones(2),np.zeros(2))
        a,b=self.extract(image),self.extract(dest)
        pa,pb=a['keypoints'][0].numpy()/scale+offset,b['keypoints'][0].numpy()
        if min(len(pa),len(pb))<2:return [],dict(source_roi_features=int(in_mask(pa,mask).sum()))
        if 'LightGlue' in method:
            indices=self.matcher(dict(image0=a,image1=b))['matches'][0].numpy()
        else:
            da,db=a['descriptors'][0].numpy(),b['descriptors'][0].numpy()
            matcher=cv2.BFMatcher(cv2.NORM_L2)
            reverse=matcher.match(db,da)
            pairs=matcher.knnMatch(da,db,k=2)
            indices=np.array([(m.queryIdx,m.trainIdx) for m,n in pairs
                if m.distance<.8*n.distance and reverse[m.trainIdx].trainIdx==m.queryIdx],dtype=int).reshape(-1,2)
        keep=in_mask(pa[indices[:,0]],mask)
        indices=indices[keep]
        return list(zip(pa[indices[:,0]],pb[indices[:,1]])),dict(source_roi_features=int(in_mask(pa,mask).sum()))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(4);cv2.setNumThreads(4);torch.manual_seed(7)
    # Fail locally instead of silently downloading during the experiment.
    cache=ROOT/'third_party/superpoint_cache/hub/checkpoints'
    for name in ['superpoint_v1.pth','superpoint_lightglue_v0-1_arxiv.pth']:
        if not (cache/name).is_file():raise FileNotFoundError(cache/name)
    model=Models();out=ROOT/'RobotRAM/output'
    cfg=json.loads((ROOT/'cart_book_grasp_v3_approach.json').read_text())
    ref=rgb(ROOT/cfg['reference']['image']);fm=np.zeros(ref.shape[:2],np.uint8)
    cv2.fillConvexPoly(fm,np.int32(cfg['reference']['mask_polygon']),1)
    head=rgb(out/'scene_57_pcl/rgb_ext.png')
    hm=np.load(out/'scene_57_pcl/detections.npy',allow_pickle=True).item()['masks'][0].astype(np.uint8)
    hg=json.loads((out/'scene_57_pcl/book_grasp_geometry.json').read_text())
    current=rgb(out/'v3_texture_debug_20261009.png');old=rgb(out/'v3_observation_before_stage2_20261009.png')
    known=np.array([[.9,0,20],[0,.9,25],[0,0,1]],float)
    jobs=list(paired_scenes())
    for name,kind,src,dst,mask,controls in [
        ('head_to_current','head',head,current,hm,[hg['spine_center_uv']]),
        ('final_to_current','final',ref,current,fm,cfg['reference']['points']),
        ('final_to_old_observation','final',ref,old,fm,cfg['reference']['points']),
        ('negative_red_book','final',ref,old[60:610,790:930].copy(),fm,cfg['reference']['points']),
        ('positive_known_warp','final',ref,cv2.warpPerspective(ref,known,(1280,720)),fm,cfg['reference']['points'])]:
        jobs.append(dict(name=name,kind=kind,source=src,dest=dst,mask=mask,controls=controls,targets=[]))
    report=dict(source='https://github.com/cvg/LightGlue',commit=subprocess.check_output(['git','-C',str(ROOT/'third_party/LightGlue'),'rev-parse','HEAD'],text=True).strip(),
        motion_sent=False,robot_connected=False,camera_opened=False,device='cpu',threads=4,
        weights={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in cache.glob('*.pth')},
        parameters=dict(max_keypoints=2048,resize=None,nms_radius=4,detection_threshold=.0005,mnn_ratio=.8,lightglue_filter=.1,depth_confidence=-1,width_confidence=-1),
        note='Paired scenes have about 1.07s capture gaps, not hardware synchronization or labelled identity ground truth. Legacy head_to_current is captured at different times. Same geometry screening as prior XFeat; timing includes cached extraction and is not a benchmark. ROI upsampling adds no information. No motion authorization.',rows=[],candidates=[])
    methods=['SIFT_current','SIFT_ROI','SuperPoint_MNN','SuperPoint_MNN_ROI','SuperPoint_LightGlue','SuperPoint_LightGlue_ROI']
    for job in jobs:
        src,dst,mask=job['source'],job['dest'],job['mask']
        for method in methods:
            begin=time.perf_counter()
            if method.startswith('SIFT'):
                pairs,count=sift_matches(src,dst,mask,job['kind'],roi=method.endswith('_ROI'))
                info=dict(source_roi_features=count)
            else:pairs,info=model.match(src,dst,mask,method)
            result=evaluate(pairs,mask,job['controls'],job['kind'],dst.shape)
            result.update(pair=job['name'],method=method,seconds=time.perf_counter()-begin,**info,
                source_rgb_sha256=hashlib.sha256(src.tobytes()).hexdigest(),dest_rgb_sha256=hashlib.sha256(dst.tobytes()).hexdigest(),pair_gap_seconds=job.get('pair_gap'))
            if job['name']=='positive_known_warp' and result.get('mapped_controls'):
                truth=cv2.perspectiveTransform(np.float32(job['controls'])[None],known)[0]
                result['control_max_error_px']=float(np.max(np.linalg.norm(np.array(result['mapped_controls'])-truth,axis=1)))
            for target in job['targets']:
                points=np.array([b for a,b in pairs]).reshape(-1,2)
                chosen=in_mask(points,target['book_mask'])
                sub=evaluate([p for p,k in zip(pairs,chosen) if k],mask,job['controls'],'head',dst.shape)
                sub.update(pair=job['name'],method=method,wrist_id=target['id'],
                    identity_texture_screen=bool(sub['inliers']>=12 and sub['inlier_ratio']>=.65 and sub['rms_px'] is not None and sub['rms_px']<=1.5 and sub.get('source_hull_mask_fraction',0)>=.03))
                report['candidates'].append(sub)
            filename=job['name']+'__'+method+'.png'
            overlay(src,dst,mask,result,job['name']+' '+method,args.output/filename)
            result['image']=filename;report['rows'].append(result)
            (args.output/'report.json').write_text(json.dumps(report,indent=2))
            print(job['name'],method,result['inliers'],result['matches'],result['screen_pass'],flush=True)
    previous=json.loads((out/'v3_xfeat_comparison_20261009/combined_report.json').read_text())
    report['previous_xfeat']=[{k:r.get(k) for k in ['pair','method','inliers','matches','screen_pass','reasons']} for r in previous['rows'] if r['method'].startswith('XFeat')]
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    rows=[]
    for r in report['rows']:
        rows.append(f'<tr><td>{html.escape(r["pair"])}</td><td>{r["method"]}</td><td>{r["source_roi_features"]}</td><td>{r["inliers"]}/{r["matches"]}</td><td>{"通过" if r["screen_pass"] else "未通过"}</td><td>{html.escape(", ".join(r["reasons"]))}</td><td><a href="{r["image"]}">匹配图</a></td></tr>')
    (args.output/'index.html').write_text('''<!doctype html><meta charset="utf-8"><title>V3 SuperPoint 头腕匹配实验</title><style>body{background:#10161d;color:#dfe8ef;font:15px system-ui;margin:30px}h1{font-size:24px}a{color:#c9ad72}table{border-collapse:collapse;width:100%}td,th{border-bottom:1px solid #34414c;padding:9px;text-align:left}p{max-width:1000px;line-height:1.7}</style><h1>V3 SuperPoint 头腕匹配实验</h1><p>离线图像实验，无机器人运动。绿色线仅表示 RANSAC 内点，不等于已确认是同一本书。两组配对采集间隔约 1.07 秒；head_to_current 来自不同时刻。正对照是已知变换，负对照为红色书局部。ROI 为裁剪放大，不增加真实纹理。</p><p>同一几何门槛：至少 12 内点、头腕内点比例 ≥65%、重投影 RMS ≤1.5 px，并要求控制点覆盖、空间分布和合理映射范围。CPU 单次计时包含缓存差异，不作性能比较。</p><table><thead><tr><th>图像对</th><th>算法</th><th>源区域特征</th><th>内点/匹配</th><th>几何检查</th><th>失败原因</th><th>图像</th></tr></thead><tbody>'''+''.join(rows)+'</tbody></table>')
    print(args.output/'report.json',flush=True)

if __name__=='__main__':main()
