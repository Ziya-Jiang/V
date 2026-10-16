"""Offline whole-book association study on saved synchronized V2_1 pairs."""
import os
os.environ['CUDA_VISIBLE_DEVICES']=''
os.environ['TORCH_DEVICE_BACKEND_AUTOLOAD']='0'
from pathlib import Path
import sys,json,time,hashlib
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'third_party/xfeat_deps'))
sys.path.insert(0,str(ROOT/'third_party/accelerated_features'))
import torch
torch.set_num_threads(4);cv2.setNumThreads(4)
from cart_book_grasp_v2_1 import _sam3_adapter
from cart_book_grasp_v2_1_selection import load_config,build_observations,match_views
from cart_book_grasp_v2_1_semantics import annotate_views,LocalSemantics
from compare_v3_xfeat import rgb,in_mask,xfeat_matches,evaluate,overlay
from modules.xfeat import XFeat
out=ROOT/'RobotRAM/output/v3_identity_research_20261009'
out.mkdir(exist_ok=True)
c=load_config();c['semantic_device']='cpu'
report={'robot_connected':False,'motion_sent':False,'cases':[]}
sam=None
print('Loading local SigLIP',flush=True)
semantic=LocalSemantics(c)
print('Loading local XFeat',flush=True)
xfeat=XFeat(weights=torch.load(ROOT/'third_party/accelerated_features/weights/xfeat.pt',map_location='cpu',weights_only=True))
for scene in ['ae051ffdadef49df914c05114cc8119f','0dbdfb74c2d34b2fb3e7aeb5adbe43f2']:
 p=ROOT/'RobotRAM/output/v2_1'/scene
 h,w=rgb(p/'head_rgb.png'),rgb(p/'wrist_rgb.png')
 meta=json.loads((p/'camera.json').read_text());decision=json.loads((p/'decision.json').read_text())
 with np.load(p/'head_raw.npz') as d:
  assert str(d['frame_sha256'])==hashlib.sha256(h.tobytes()).hexdigest()
  raw=list(zip(d['scores'],d['masks'],d['boxes']))
 allhead=build_observations(raw,h,c,'head',np.load(p/'depth.npy'),np.array(meta['K']),np.array(meta['c2w']))
 head=[]
 for item in decision['catalog']:
  candidates=[v for v in allhead if np.max(np.abs(np.array(v['box'])-item['box']))<2]
  if len(candidates)!=1:raise ValueError('Cannot bind saved numbered mask uniquely')
  v=candidates[0];v['id']=item['id'];head.append(v)
 cache=out/(scene+'_wrist_raw.npz')
 if cache.exists():
  with np.load(cache) as d:
   assert str(d['frame_sha256'])==hashlib.sha256(w.tobytes()).hexdigest()
   wr=list(zip(d['scores'],d['masks'],d['boxes']))
 else:
  if sam is None:
   print('Loading SAM3 for saved wrist images',flush=True);sam=_sam3_adapter()
  print('Segmenting saved wrist',scene,flush=True);wr=sam.raw(w)
  np.savez_compressed(cache,frame_sha256=hashlib.sha256(w.tobytes()).hexdigest(),scores=np.array([v[0] for v in wr]),masks=np.array([v[1] for v in wr]),boxes=np.array([v[2] for v in wr]))
 wrist=build_observations(wr,w,c,'wrist')
 print('Candidates',len(head),len(wrist),flush=True)
 semantics=annotate_views(h,head,w,wrist,{'query':'a photo of a book.'},c,model=semantic)
 mapping,details=match_views(h,head,w,wrist,c)
 case={'scene':scene,'pair_gap_seconds':meta['pair_gap_seconds'],'head_boxes':[v['box'] for v in head],
       'wrist_boxes':[v['box'] for v in wrist],'semantics':semantics,'mapping':mapping,'appearance':details,'features':[]}
 # Keep all candidates: removing an ambiguous alternative can manufacture identity.
 for hi,item in enumerate(head):
  for method in ('XFeat_sparse_ROI','XFeat_semi_dense_ROI','XFeat_lighterglue_ROI'):
   pairs,_=xfeat_matches(xfeat,h,w,item['book_mask'],roi=True,dense='semi_dense' in method,glue='lighterglue' in method)
   for wi,target in enumerate(wrist):
    pts=np.array([b for a,b in pairs]).reshape(-1,2)
    chosen=in_mask(pts,target['book_mask']) if len(pts) else []
    subset=[pair for pair,keep in zip(pairs,chosen) if keep]
    r=evaluate(subset,item['book_mask'],[item['uv']],'head',w.shape)
    # Identity comparison needs shared texture, not transport of the whole-book
    # center. This relaxed diagnostic is reported separately, never authorizes motion.
    r['identity_texture_screen']=bool(r['inliers']>=12 and r['inlier_ratio']>=.65 and
        r['rms_px'] is not None and r['rms_px']<=1.5 and r.get('source_hull_mask_fraction',0)>=.03)
    r.update(head_id=item['id'],wrist_id=target['id'],method=method,total_source_matches=len(pairs))
    case['features'].append(r)
    print(scene[:8],item['id'],target['id'],method,r['inliers'],r['matches'],r['identity_texture_screen'],flush=True)
   result=evaluate(pairs,item['book_mask'],[item['uv']],'head',w.shape)
   overlay(h,w,item['book_mask'],result,scene[:8]+' '+item['id']+' '+method,out/(scene[:8]+'_'+item['id']+'_'+method+'.png'))
 report['cases'].append(case)
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 print('Mapping',mapping,'scores',details['score_matrix'],flush=True)
print(out/'report.json',flush=True)
