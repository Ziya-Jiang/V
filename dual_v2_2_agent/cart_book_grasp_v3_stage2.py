"""V3: V1_3 first forward waypoint, then XYZ final-image servo.

Normal execution starts a full V1_3 cycle (including first_reset); it does not
resume an already-paused robot. Check and image-matching utilities send no motion.
"""
import argparse
import json
from pathlib import Path
import sys
import uuid
import cv2
import numpy as np
from cart_book_grasp_v3_camera import WristCamera
from cart_book_grasp_v3_final_tracking import FinalImageTracker, draw_final_tracking
from cart_book_grasp_v3_xyz_servo import CONFIG_PATH, load_approach
from cart_book_grasp_v3_servo import ServoError

ROOT=Path(__file__).resolve().parent


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--approach-config',type=Path,default=CONFIG_PATH)
    group=parser.add_mutually_exclusive_group()
    group.add_argument('--check-setup',action='store_true')
    group.add_argument('--check-config',action='store_true')
    group.add_argument('--match-reference',type=Path,metavar='OBSERVATION_PNG',help='Offline final-image matching; no camera or robot')
    group.add_argument('--match-head-wrist',type=Path,metavar='SAVED_INPUT_DIR',
                       help='Replay saved V3 head/wrist acquisition and final-image checks; no camera or robot')
    group.add_argument('--preview-wrist-center',type=Path,metavar='WRIST_PNG',
                       help='Offline center-book detection/locking and final-reference check; no camera or robot')
    args,remaining=parser.parse_known_args(argv)
    utility=args.check_setup or args.check_config or args.match_reference is not None or args.match_head_wrist is not None or args.preview_wrist_center is not None
    if utility and remaining:parser.error('Unknown arguments: '+' '.join(remaining))
    config=load_approach(args.approach_config)
    camera=WristCamera()
    tracker=FinalImageTracker.from_config(config,camera,ROOT)
    if args.match_head_wrist is not None or (config['target_selection']=='head_identity' and args.match_reference is None and args.preview_wrist_center is None):
        from cart_book_grasp_v3_superpoint import get_backend
        get_backend()
        print('V3 头腕匹配后端：SuperPoint+LightGlue (CPU)，内点 >=6，内点率 >=0.65，RMS <=1.5 px；控制点与投影检查保留。',flush=True)
    elif config['target_selection']=='wrist_center':
        print('V3 选书模式：腕部图像中心最近的书；不执行头腕匹配，选中后锁定，不自动换书。',flush=True)
    if args.check_setup or args.check_config:
        print('V3 第二段 XYZ 配置通过：最终抓取图、标定和参考点已绑定。未打开相机或连接机器人。')
        return
    if args.preview_wrist_center:
        from cart_book_grasp_v2_1 import _sam3_adapter
        from cart_book_grasp_v3_wrist_center import acquire_wrist_center, require_points_in_book, draw_selection
        bgr=cv2.imread(str(args.preview_wrist_center))
        if bgr is None:raise ServoError('Cannot read wrist preview input')
        rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        if rgb.shape[:2]!=(camera.size[1],camera.size[0]):raise ServoError('Wrist preview size mismatch')
        adapter=_sam3_adapter()
        raw=adapter.raw(rgb)
        output=ROOT/'RobotRAM/output'/('v3_wrist_center_'+uuid.uuid4().hex)
        output.mkdir()
        np.savez_compressed(output/'detections.npz',scores=np.array([x[0] for x in raw]),
                            masks=np.array([x[1] for x in raw]),boxes=np.array([x[2] for x in raw]))
        report=dict(motion_sent=False,camera_opened=False,head_wrist_matching=False,
                    source=str(args.preview_wrist_center.resolve()),selection={},lock_passed=False,final_reference_passed=False)
        try:
            lock=acquire_wrist_center(rgb,lambda image:raw,diagnostics=report['selection'])
            selected=lock.observe(rgb);report['lock_passed']=True
            cv2.imwrite(str(output/'selected.png'),cv2.cvtColor(draw_selection(rgb,selected),cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(output/'selected_mask.png'),np.uint8(selected['mask'])*255)
            match=tracker.observe(rgb,search_mask=selected['mask'])
            require_points_in_book(match,selected)
            report['final_reference_passed']=True
        except ServoError as error:report['reason']=str(error)
        (output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        print(json.dumps(report,indent=2,ensure_ascii=False));print(f'Report: {output}')
        if not report['final_reference_passed']:raise SystemExit(2)
        return
    if args.match_head_wrist:
        from cart_book_grasp_v3_tracking import acquire_selected_spine
        folder=args.match_head_wrist
        metadata=json.loads((folder/'acquisition.json').read_text())
        if metadata.get('coordinate_system') != 'undistorted_K':
            raise ServoError('Replay requires undistorted_K wrist acquisition')
        def read(name,gray=False):
            pixels=cv2.imread(str(folder/name),cv2.IMREAD_GRAYSCALE if gray else cv2.IMREAD_COLOR)
            if pixels is None:raise ServoError(f'Missing replay input: {folder/name}')
            return pixels if gray else cv2.cvtColor(pixels,cv2.COLOR_BGR2RGB)
        head=dict(rgb=read('head_selected_rgb.png'),
                  selected=dict(mask=read('head_selected_mask.png',True)>0,uv=metadata['selected_uv']))
        wrist=read('wrist_acquisition.png')
        diagnostics=[];selected=None;observation=None
        report=dict(backend='SuperPoint+LightGlue',motion_sent=False,camera_opened=False,
                    input_directory=str(folder.resolve()),head_wrist=dict(passed=False),final_reference=dict(passed=False))
        try:
            selected=acquire_selected_spine(head,wrist,diagnostics=diagnostics,backend='superpoint').observe(wrist)
            report['head_wrist']=dict(passed=True,inliers=selected['inliers'],polygon=selected['polygon'].tolist())
        except ServoError as error:report['head_wrist']['reason']=str(error)
        report['head_wrist']['attempts']=diagnostics
        try:
            observation=tracker.observe(wrist)
            report['final_reference']=dict(passed=True,inliers=observation['inliers'],
                                           points=observation['points'].tolist())
        except ServoError as error:report['final_reference']['reason']=str(error)
        report['same_taught_book']=bool(selected is not None and observation is not None and all(
            cv2.pointPolygonTest(np.float32(selected['polygon']),tuple(map(float,p)),False)>=0
            for p in observation['points']))
        report['acquisition_passed']=report['same_taught_book']
        output=folder/('superpoint_replay_'+uuid.uuid4().hex)
        output.mkdir()
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2,ensure_ascii=False));print(f'Report: {output}')
        if not report['acquisition_passed']:raise SystemExit(2)
        return
    if args.match_reference:
        metadata=json.loads(args.match_reference.with_suffix('.json').read_text())
        camera.check_reference(metadata)
        bgr=cv2.imread(str(args.match_reference))
        if bgr is None:raise ServoError('Cannot read observation image')
        rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        observation=tracker.observe(rgb)
        output=ROOT/'RobotRAM'/'output'/('v3_match_'+uuid.uuid4().hex)
        output.mkdir(parents=True)
        report={k:v for k,v in observation.items() if k not in ('points','homography')}
        report.update(points=observation['points'].tolist(),target=tracker.target.tolist(),
                      point_errors_px=np.linalg.norm(observation['points']-tracker.target,axis=1).tolist(),
                      observation_image=str(args.match_reference.resolve()),
                      final_reference=config['reference']['image'],motion_sent=False)
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        cv2.imwrite(str(output/'matching.png'),cv2.cvtColor(draw_final_tracking(rgb,observation,tracker.target),cv2.COLOR_RGB2BGR))
        print(json.dumps(report,indent=2));print(f'Report: {output}')
        return
    from cart_book_grasp_v1_3 import main as v13_main
    from cart_book_grasp_v3_motion import installed
    saved=sys.argv
    sys.argv=[saved[0],*remaining]
    try:
        with installed(None,approach_path=args.approach_config):
            v13_main(config_path=ROOT/'cart_book_grasp_v3_config.json',
                     wrist_config_path=ROOT/'cart_book_grasp_v3_wrist_config.json')
    finally:sys.argv=saved


def cli():
    try:main()
    except (ValueError,OSError) as error:
        print(f'V3: {error}',file=sys.stderr)
        raise SystemExit(2)
