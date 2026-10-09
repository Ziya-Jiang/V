"""Offline SAM3 instance preview on saved RGB-D. No GPT, camera or robot."""
import argparse
import json
from pathlib import Path
import sys
import types
import numpy as np
from PIL import Image


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rgb',type=Path,required=True)
    parser.add_argument('--depth',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--device',default='cuda')
    parser.add_argument('--check-instance',help='Offline spine diagnostics for this proposal ID; no GPT or motion')
    args=parser.parse_args()
    import torch
    from cart_book_grasp_v2_instances import detect_instances,load_config
    from cart_book_grasp_v2_candidates import load_config as depth_config,annotate_gate,CandidateError
    from cart_book_grasp_v2_agent import annotate_candidates,candidate_gallery
    # Same SAM3 settings as book_thickness_pipeline.load_sam3, without importing
    # its unrelated Record3D/liblzfse dependencies for a saved RGB-D preview.
    sys.path.insert(0,'/mnt/ssd/user/wx/SAM3_Inference')
    from sam3.model.sam3_image_processor import Sam3Processor
    from sam3.model_builder import build_sam3_image_model
    args.output.mkdir(parents=True,exist_ok=False)
    rgb=np.asarray(Image.open(args.rgb).convert('RGB'));depth=np.load(args.depth,allow_pickle=False)
    Image.fromarray(rgb).save(args.output/'source.png');np.save(args.output/'depth.npy',depth)
    cal=json.loads(args.calibration.read_text())['left'];k=np.asarray(cal['K'],float)
    if rgb.shape[:2]!=(cal['h'],cal['w']):raise ValueError('Image resolution differs from calibration')
    tf=np.eye(4);tf[:3,:3]=cal['R'];tf[:3,3]=np.asarray(cal['T']).reshape(3)
    checkpoint='/home/vlmdemo/.cache/huggingface/hub/models--facebook--sam3/snapshots/3c879f39826c281e95690f02c7821c4de09afae7/sam3.pt'
    model=build_sam3_image_model(checkpoint_path=checkpoint,load_from_HF=False,
        device=args.device,eval_mode=True,compile=False).to(args.device)
    processor=Sam3Processor(model=model,resolution=1008,device=args.device,confidence_threshold=.1)
    def to_numpy(value):return value.detach().cpu().numpy() if torch.is_tensor(value) else np.asarray(value)
    raw_calls=[]
    def raw(p,as_numpy,image,prompt):
        with torch.inference_mode():
            state=p.set_image(image);state=p.set_text_prompt(prompt=prompt,state=state)
        masks=as_numpy(state['masks']);masks=masks[:,0] if masks.ndim==4 else masks
        scores=as_numpy(state['scores']);boxes=as_numpy(state['boxes'])
        np.savez_compressed(args.output/f'raw_{len(raw_calls)}.npz',masks=masks,scores=scores,boxes=boxes)
        raw_calls.append(dict(prompt=prompt,shape=[image.height,image.width],count=len(scores)))
        return [(float(s),m.astype(bool),b) for s,m,b in zip(scores,masks,boxes)]
    geometry=types.SimpleNamespace(_load_sam3_detector=lambda:(processor,to_numpy,Image),_sam3_raw_books=raw)
    candidates=[]
    try:
        candidates,diagnostics=detect_instances(geometry,rgb,depth,k,tf,load_config(),depth_config())
        status='ok'
    except CandidateError as error:diagnostics=error.diagnostics;status=str(error)
    Image.fromarray(annotate_candidates(rgb,candidates)).save(args.output/'instances.png')
    Image.fromarray(candidate_gallery(rgb,candidates)).save(args.output/'crops.png')
    Image.fromarray(annotate_gate(rgb,diagnostics)).save(args.output/'depth_filter.png')
    np.savez_compressed(args.output/'instances.npz',**{c['id']:c['book_mask'] for c in candidates})
    entries=[{key:value for key,value in c.items() if key not in ('mask','book_mask','geometry')} for c in candidates]
    report=dict(status=status,rgb=str(args.rgb),calibration=str(args.calibration),raw_calls=raw_calls,
                candidates=entries,diagnostics=diagnostics,gpt_called=False,robot_executed=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print(json.dumps(dict(status=status,candidate_count=len(candidates),raw_calls=raw_calls),ensure_ascii=False),flush=True)
    if args.check_instance:
        from cart_book_grasp_v2_target import prepare_instance_target
        from cart_book_grasp_v2_perception import _TiltDetector,_spine_center_from_mask
        selected=next((c for c in candidates if c['id']==args.check_instance),None)
        if selected is None:raise ValueError('Requested diagnostic instance is absent')
        try:
            prepare_instance_target(selected,rgb,depth,k,tf,_TiltDetector(geometry),_spine_center_from_mask,
                                    Path(__file__).with_name('cart_book_grasp_v2_agent_config.json'))
            outcome=dict(status='geometry_ready')
        except ValueError as error:outcome=dict(status='geometry_failed',error=str(error))
        outcome.update(instance_id=selected['id'],reviews=selected.get('spine_candidate_reviews',[]),
                       gpt_called=False,robot_executed=False)
        (args.output/'spine_diagnostics.json').write_text(json.dumps(outcome,indent=2,ensure_ascii=False))
        print(json.dumps(outcome,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
