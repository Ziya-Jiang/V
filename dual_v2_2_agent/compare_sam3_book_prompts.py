"""Offline raw SAM3 prompt comparison. Never imports robot/camera control."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', default='cuda')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    # Snapshot once: the live grasp process may overwrite its scene directory.
    source_bytes = args.image.read_bytes()
    snapshot = args.output / 'source.png'
    snapshot.write_bytes(source_bytes)
    rgb = Image.open(snapshot).convert('RGB')
    arr = np.asarray(rgb)
    # Same loader settings/checkpoint as book_thickness_pipeline.load_sam3;
    # avoid importing its unrelated Record3D/liblzfse dependencies.
    sys.path.insert(0, '/mnt/ssd/user/wx/SAM3_Inference')
    from sam3.model.sam3_image_processor import Sam3Processor
    from sam3.model_builder import build_sam3_image_model
    checkpoint=Path('/home/vlmdemo/.cache/huggingface/hub/models--facebook--sam3/snapshots/3c879f39826c281e95690f02c7821c4de09afae7/sam3.pt')
    model=build_sam3_image_model(checkpoint_path=str(checkpoint),load_from_HF=False,
                                device=args.device,eval_mode=True,compile=False)
    model=model.to(args.device)
    processor=Sam3Processor(model=model,resolution=1008,device=args.device,confidence_threshold=.10)
    def to_numpy(value):
        return value.detach().cpu().numpy() if torch.is_tensor(value) else np.asarray(value)
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font = ImageFont.truetype(font_path, 22)
    small = ImageFont.truetype(font_path, 16)
    palette = [(255,205,35), (30,225,240), (250,105,190), (135,235,75), (255,145,50), (180,145,255)]
    report = dict(source=str(args.image.resolve()), source_sha256=hashlib.sha256(source_bytes).hexdigest(),
                  source_mtime=datetime.fromtimestamp(args.image.stat().st_mtime).isoformat(),
                  checkpoint=str(checkpoint), threshold=.1, rotated=False,
                  postprocessing='none; raw SAM3 outputs', results={})
    panels = []
    with torch.inference_mode():
        start = time.monotonic()
        state = processor.set_image(rgb)
        print(f'Image embedding ready: {time.monotonic()-start:.2f}s', flush=True)
        for prompt in ('book spine', 'book cover', 'book'):
            start = time.monotonic()
            state = processor.set_text_prompt(prompt=prompt, state=state)
            masks = to_numpy(state['masks'])
            if masks.ndim == 4:
                masks = masks[:, 0]
            masks = np.asarray(masks, dtype=bool).reshape((-1, rgb.height, rgb.width))
            boxes = to_numpy(state['boxes']).reshape(-1, 4)
            scores = to_numpy(state['scores']).reshape(-1)
            slug = prompt.replace(' ', '_')
            np.savez_compressed(args.output / f'{slug}_raw.npz', masks=masks, boxes=boxes, scores=scores)
            outline = arr.copy()
            isolated = np.full_like(arr, 45)
            entries = []
            for i, (mask, box, score) in enumerate(zip(masks, boxes, scores), 1):
                color = palette[(i-1) % len(palette)]
                contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(outline, contours, -1, color, 2)
                isolated[mask] = arr[mask]
                Image.fromarray(mask.astype(np.uint8)*255).save(args.output / f'{slug}_mask_{i:02d}.png')
                entries.append(dict(id=i, score=float(score), area_px=int(mask.sum()), box_xyxy=box.tolist()))
            outline_image = Image.fromarray(outline)
            draw = ImageDraw.Draw(outline_image)
            for entry in entries:
                i=entry['id'];x,y,_,_=entry['box_xyxy']
                label=f"#{i} {entry['score']:.3f}"
                x=max(0, min(int(x), rgb.width-120)); y=max(0,int(y)-24)
                draw.rectangle((x,y,x+120,y+24), fill=(15,20,27))
                draw.text((x+3,y+2), label, font=small, fill=palette[(i-1)%len(palette)])
            outline_image.save(args.output / f'{slug}_outlines.png')
            Image.fromarray(isolated).save(args.output / f'{slug}_isolated.png')
            panel=Image.new('RGB', (rgb.width, 2*rgb.height+130), (20,25,33))
            draw=ImageDraw.Draw(panel)
            draw.text((16,12), f'{prompt}  |  {len(entries)} masks', font=font, fill='white')
            draw.text((16,43), 'Original RGB + raw mask contours / score', font=small, fill=(185,196,210))
            panel.paste(outline_image,(0,72))
            draw.text((16,rgb.height+84), 'All masks isolated - original RGB colors', font=small, fill=(185,196,210))
            panel.paste(Image.fromarray(isolated),(0,rgb.height+130))
            panels.append(panel)
            report['results'][prompt]=dict(count=len(entries), seconds=time.monotonic()-start, detections=entries)
            print(prompt, json.dumps(report['results'][prompt]), flush=True)
    comparison=Image.new('RGB',(rgb.width*3+32,panels[0].height+56),(20,25,33))
    draw=ImageDraw.Draw(comparison)
    draw.text((16,12),'SAM3 raw prompt comparison | threshold 0.10 | same saved frame | no robot motion',font=font,fill='white')
    for i,panel in enumerate(panels):
        comparison.paste(panel,(i*(rgb.width+16),56))
    comparison.save(args.output/'comparison.png')
    (args.output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(f"Saved: {args.output/'comparison.png'}",flush=True)


if __name__=='__main__':
    main()
