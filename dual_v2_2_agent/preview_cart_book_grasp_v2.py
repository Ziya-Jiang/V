"""Offline DINO/depth-only preview. No GPT, SAM, camera, or robot control."""
import argparse
import json
import os
from pathlib import Path
import sys
import numpy as np
from PIL import Image


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rgb',type=Path,required=True)
    p.add_argument('--depth',type=Path,required=True)
    p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--device',default='cpu')
    args=p.parse_args()
    os.environ.setdefault('HF_HUB_OFFLINE','1')
    os.environ.setdefault('TRANSFORMERS_OFFLINE','1')
    import torch
    torch.set_num_threads(4)
    from cart_book_grasp_v2_candidates import load_config,detect_candidates,annotate_gate,CandidateError
    from cart_book_grasp_v2_agent import annotate_candidates,candidate_gallery
    root=Path(__file__).resolve().parent
    dino=root/'RobotRAM/visions/grounding'
    sys.path.insert(0,str(dino))
    from grounding_dino.groundingdino.util.inference import load_model,load_data,predict
    rgb=np.asarray(Image.open(args.rgb).convert('RGB'));depth=np.load(args.depth,allow_pickle=False)
    cal=json.loads(args.calibration.read_text())['left'];k=np.asarray(cal['K'],float)
    if rgb.shape[:2]!=(cal['h'],cal['w']):raise ValueError('Image resolution differs from calibration')
    tf=np.eye(4);tf[:3,:3]=cal['R'];tf[:3,3]=np.asarray(cal['T']).reshape(3)
    model=load_model(str(dino/'grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py'),
        str(dino/'Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth'),device=args.device)
    _,image=load_data(rgb)
    raw=[]
    class Detector:
        def detect_boxes_prepared(self,prompt,**kwargs):
            boxes,scores,labels=predict(model,image,prompt,device=args.device,**kwargs)
            boxes=boxes.detach().cpu().numpy()*[rgb.shape[1],rgb.shape[0],rgb.shape[1],rgb.shape[0]]
            boxes=np.c_[boxes[:,:2]-boxes[:,2:]/2,boxes[:,:2]+boxes[:,2:]/2]
            scores=scores.detach().cpu().numpy()
            raw.append(dict(prompt=prompt,boxes=boxes.tolist(),scores=scores.tolist(),labels=labels))
            return boxes,scores,labels
        def detect_boxes_region(self,rgb,region,prompt,**kwargs):
            x0,y0,x1,y1=map(int,region)
            _,crop_image=load_data(rgb[y0:y1,x0:x1])
            boxes,scores,labels=predict(model,crop_image,prompt,device=args.device,**kwargs)
            boxes=boxes.detach().cpu().numpy()*[x1-x0,y1-y0,x1-x0,y1-y0]
            boxes=np.c_[boxes[:,:2]-boxes[:,2:]/2,boxes[:,:2]+boxes[:,2:]/2]+[x0,y0,x0,y0]
            scores=scores.detach().cpu().numpy()
            raw.append(dict(prompt=prompt,region_xyxy=list(map(int,region)),boxes=boxes.tolist(),scores=scores.tolist(),labels=labels))
            return boxes,scores,labels
    args.output.mkdir(parents=True,exist_ok=True)
    try:
        candidates,diagnostics=detect_candidates(Detector(),rgb,depth,k,tf,load_config())
        Image.fromarray(annotate_candidates(rgb,candidates)).save(args.output/'candidates.png')
        Image.fromarray(candidate_gallery(rgb,candidates)).save(args.output/'crops.png')
        status=f'{len(candidates)} candidates accepted'
    except CandidateError as error:
        diagnostics=error.diagnostics;status=str(error)
    Image.fromarray(annotate_gate(rgb,diagnostics)).save(args.output/'cart_filter.png')
    report=dict(status=status,rgb=str(args.rgb),depth=str(args.depth),calibration=str(args.calibration),
                raw_detections=raw,cart_filter=diagnostics)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print(status,flush=True)
    print(args.output/'report.json',flush=True)


if __name__=='__main__':main()
