"""Default V3: current-book point tracking and V1_3 distance endpoint."""
import argparse
import json
import sys
import uuid
from pathlib import Path
import cv2
import numpy as np
from cart_book_grasp_v3_distance_servo import CONFIG_PATH,load_config
from cart_book_grasp_v3_cotracker_points import CoTrackerModel,PointLock,draw_points
from cart_book_grasp_v3_camera import WristCamera
from cart_book_grasp_v3_servo import ServoError

ROOT=Path(__file__).resolve().parent


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cotracker-config',type=Path,default=CONFIG_PATH)
    modes=p.add_mutually_exclusive_group()
    modes.add_argument('--check-config',action='store_true')
    modes.add_argument('--check-setup',action='store_true')
    modes.add_argument('--replay-cotracker',type=Path,nargs='+',metavar='WRIST_PNG')
    p.add_argument('--mask',type=Path,help='Initial book mask for offline replay; otherwise detect center book')
    args,remaining=p.parse_known_args(argv)
    if (args.check_config or args.check_setup or args.replay_cotracker) and remaining:p.error('Unknown utility arguments: '+' '.join(remaining))
    if args.mask and not args.replay_cotracker:p.error('--mask requires --replay-cotracker')
    c=load_config(args.cotracker_config);camera=WristCamera();model=CoTrackerModel(c)
    print(f'V3 模式：CoTracker 当前书点跟踪 + Y/Z 视觉对准 + V1_3 目标 X 停止；device={model.device}。不使用头腕匹配或最终示教图。',flush=True)
    if args.check_config or args.check_setup:
        print('配置、腕部标定、本地 CoTracker 模型加载通过；未打开相机或连接机器人。');return
    if args.replay_cotracker:
        frames=[]
        for path in args.replay_cotracker:
            bgr=cv2.imread(str(path))
            if bgr is None or bgr.shape[:2]!=(camera.size[1],camera.size[0]):raise ServoError(f'Invalid calibrated-size replay image: {path}')
            frames.append(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB))
        if len(frames)<2:p.error('Replay requires at least two real or explicitly synthetic frames')
        if args.mask:
            mask=cv2.imread(str(args.mask),0)
            if mask is None or mask.shape!=frames[0].shape[:2]:raise ServoError('Invalid initial replay mask')
            mask=mask>0
        else:
            from cart_book_grasp_v2_1 import _sam3_adapter
            from cart_book_grasp_v3_wrist_center import select_center_book
            chosen,_=select_center_book(frames[0],_sam3_adapter().raw(frames[0]));mask=chosen['mask']
        lock=PointLock(model,frames[0],mask,c);h,w=frames[0].shape[:2]
        target=np.array(c['target_uv'] if c['target_uv'] is not None else [(w-1)/2,(h-1)/2])
        output=ROOT/'RobotRAM/output'/('v3_cotracker_replay_'+uuid.uuid4().hex);output.mkdir()
        report=dict(motion_sent=False,camera_opened=False,inputs=[str(x.resolve()) for x in args.replay_cotracker],
            device=model.device,initial_points=lock.reference.tolist(),rows=[],passed=False)
        try:
            for i,rgb in enumerate(frames[1:],1):
                o=lock.observe(rgb);report['rows'].append(dict(frame=i,uv=o['uv'].tolist(),points=o['points'].tolist(),visible=o['visible'].tolist(),inliers=o['inliers'],
                    scale=o['scale'],inference_seconds=o['inference_seconds']))
                cv2.imwrite(str(output/f'frame_{i:03d}.png'),cv2.cvtColor(draw_points(rgb,o,target),cv2.COLOR_RGB2BGR))
            report['passed']=True
        except ServoError as error:report['reason']=str(error)
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(f'Report: {output}; passed={report["passed"]}')
        if not report['passed']:raise SystemExit(2)
        return
    from cart_book_grasp_v1_3 import main as v13_main
    from cart_book_grasp_v3_motion import installed
    saved=sys.argv;sys.argv=[saved[0],*remaining]
    try:
        with installed(None,cotracker_path=args.cotracker_config,cotracker_model=model):
            v13_main(config_path=ROOT/'cart_book_grasp_v3_config.json',wrist_config_path=ROOT/'cart_book_grasp_v3_wrist_config.json')
    finally:sys.argv=saved
