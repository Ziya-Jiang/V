"""Offline standard YOLO26 book detection. No robot/camera imports or motion.

Run with the existing /mnt/ssd/robotarm/env/yolo/bin/python environment.
Weights must already exist; this script never downloads a replacement model.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights',type=Path,default=Path(__file__).parent/'models/yolo26x.pt')
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--imgsz',type=int,default=1024)
    parser.add_argument('--conf',type=float,default=.25)
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--head',choices=['nms','end2end'],default='nms',
                        help='YOLO26 one-to-many NMS head or one-to-one end-to-end head')
    args=parser.parse_args(argv)
    if not args.weights.is_file() or not args.image.is_file():parser.error('Local weights and image are required')
    if 'world' in args.weights.name.lower() or not args.weights.name.startswith('yolo26'):
        parser.error('Use standard YOLO26 weights')
    if args.imgsz<32 or args.imgsz%32 or not 0<args.conf<=1:parser.error('Invalid image size or confidence')
    import ultralytics
    from ultralytics import YOLO
    start=time.monotonic();model=YOLO(str(args.weights))
    if type(model).__name__!='YOLO' or model.task!='detect':
        raise ValueError('Expected standard YOLO detection model')
    book_ids=[int(i) for i,name in model.names.items() if name.strip().lower() in ('book','books')]
    if not book_ids:raise ValueError('The checkpoint has no book category')
    load_seconds=time.monotonic()-start
    args.output.mkdir(parents=True,exist_ok=False)
    start=time.monotonic()
    result=model.predict(source=str(args.image),device=args.device,imgsz=args.imgsz,
        conf=args.conf,iou=.5,classes=book_ids,end2end=args.head=='end2end',verbose=False,save=False)[0]
    predict_seconds=time.monotonic()-start
    boxes=[dict(box_xyxy=b,score=s,class_id=int(c),class_name=model.names[int(c)])
           for b,s,c in zip(result.boxes.xyxy.tolist(),result.boxes.conf.tolist(),result.boxes.cls.tolist())]
    result.save(filename=str(args.output/'detections.jpg'))
    report=dict(weights=str(args.weights.resolve()),weights_sha256=hashlib.sha256(args.weights.read_bytes()).hexdigest(),
        ultralytics_version=ultralytics.__version__,model_type=type(model).__name__,
        image=str(args.image.resolve()),image_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),
        imgsz=args.imgsz,confidence=args.conf,device=args.device,head=args.head,book_class_ids=book_ids,
        detections=boxes,load_seconds=load_seconds,predict_seconds=predict_seconds,
        robot_executed=False,sam3_executed=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
