"""Known-warp regression on a user's template; not real robot validation."""
import argparse
from pathlib import Path
import cv2
import numpy as np
from servo_features import Tracker,read_image,write_json,project,annotate


def run(model,out):
    out = Path(out)
    out.mkdir(parents=True,exist_ok=True)
    im = read_image(Path(model)/'reference.png')
    h,w = im.shape[:2]
    tracker = Tracker(model)
    center = tuple(tracker.polygon.mean(axis=0).astype(float))
    rows, errors = [],[]
    rng = np.random.default_rng(26)
    for i in range(20):
        H = np.vstack([cv2.getRotationMatrix2D(center,i*.4,1+i*.008),[0,0,1]])
        H[0,2] += i*.7
        H[1,2] -= i*.3
        frame = cv2.warpPerspective(im,H,(w,h))
        frame = np.clip(frame.astype(float)*(.98-i*.003)+4+rng.normal(0,.5,frame.shape),0,255).astype(np.uint8)
        r = tracker.process(frame,timestamp=i/30)
        expected = project(tracker.anchors,H)
        e = [float(np.linalg.norm(np.array(f['xy'])-p)) for f,p in zip(r['features'],expected) if f['valid']]
        errors.extend(e)
        rows.append(dict(frame=i,status=r['status'],method=r['method'],valid_points=len(e),
                         max_error_px=max(e) if e else None,processing_ms=r['processing_ms']))
        if i in (0,19):
            cv2.imwrite(str(out/f'frame_{i:02d}.png'),annotate(frame,r,'known warp test'))
            write_json(out/f'frame_{i:02d}.json',r)
    # Independent full loss and reacquisition, with original identities.
    loss = tracker.process(np.full_like(im,128))
    recovery = tracker.process(im)
    result = dict(test='synthetic known warps; not physical grasp validation',frames=len(rows),
                  valid_frames=sum(r['valid_points']>=4 for r in rows),
                  median_error_px=float(np.median(errors)) if errors else None,
                  p95_error_px=float(np.percentile(errors,95)) if errors else None,
                  max_error_px=max(errors) if errors else None,
                  median_processing_ms=float(np.median([r['processing_ms'] for r in rows])),
                  full_occlusion_status=loss['status'],recovery_status=recovery['status'],rows=rows)
    write_json(out/'benchmark.json',result)
    print({k:v for k,v in result.items() if k!='rows'})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',required=True)
    p.add_argument('--out',required=True)
    a=p.parse_args()
    run(a.model,a.out)
