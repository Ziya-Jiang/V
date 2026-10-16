"""V2.1: display fresh head book numbers, select a number, use V1_3 geometry.

--check-config: no hardware. --capture-pair: cameras only, no robot connection.
--preview-pair: offline. --observe-only: uses existing first_reset, no grasp.
Without a utility flag, perceive first and then wait for a numeric selection.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time

from cart_book_grasp_v2_1_selection import CONFIG_PATH, SelectionError, load_config

ROOT = Path(__file__).resolve().parent


def capture_pair(output):
    """Acquire paired images only. Never constructs a robot or invokes the motion entrypoint."""
    import numpy as np
    from PIL import Image
    from cart_book_grasp_v2_dual_view import capture_wrist, load_config as camera_config
    from dual_v2_2_agent_pcl import (load_orbbec_calibration, start_calibrated_orbbec,
                                      LATEST_ORBBEC_CALIBRATION_PATH)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    calibration = load_orbbec_calibration(LATEST_ORBBEC_CALIBRATION_PATH)
    camera = start_calibrated_orbbec(calibration)
    try:
        rgb = depth = None
        for _ in range(12):
            rgb, depth = camera.capture_image()
        if rgb is None or depth is None:
            raise SelectionError('头部相机没有返回有效 RGB-D')
        captured_at = time.monotonic()
        dual = camera_config()
        view = capture_wrist(dual, captured_at)
        Image.fromarray(rgb).save(output/'head_rgb.png')
        Image.fromarray(view['rgb']).save(output/'wrist_rgb.png')
        np.save(output/'depth.npy', depth)
        metadata = dict(K=camera.K.tolist(), c2w=camera.c2w.tolist(),
                        captured_at=datetime.now().astimezone().isoformat(),
                        pair_gap_seconds=view['pair_gap_seconds'], wrist_camera=dual,
                        calibration_path=calibration['path'], depth_alignment=str(camera._orbbec_align_mode),
                        pose_verified=False, robot_connected=False,
                        note='Camera-only capture at current pose; not a first_reset commissioning result.')
        (output/'camera.json').write_text(json.dumps(metadata, indent=2))
    finally:
        camera.close()
    print(f'双视角已保存：{output.resolve()}；未连接机器人，未验证 first_reset 姿态。')


def _sam3_adapter():
    """Same local checkpoint/resolution as the existing offline SAM3 preview."""
    import numpy as np
    import torch
    from PIL import Image
    from types import SimpleNamespace
    sys.path.insert(0, '/mnt/ssd/user/wx/SAM3_Inference')
    from sam3.model.sam3_image_processor import Sam3Processor
    from sam3.model_builder import build_sam3_image_model
    checkpoint = '/home/vlmdemo/.cache/huggingface/hub/models--facebook--sam3/snapshots/3c879f39826c281e95690f02c7821c4de09afae7/sam3.pt'
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = build_sam3_image_model(checkpoint_path=checkpoint, load_from_HF=False,
                                  device=device, eval_mode=True, compile=False).to(device)
    processor = Sam3Processor(model=model, resolution=1008, device=device, confidence_threshold=.1)
    def numpy(value):
        return value.detach().cpu().numpy() if torch.is_tensor(value) else np.asarray(value)
    def raw(rgb):
        with torch.inference_mode():
            state = processor.set_image(Image.fromarray(rgb))
            state = processor.set_text_prompt(prompt='book', state=state)
        masks = numpy(state['masks'])
        if masks.ndim == 4:
            masks = masks[:, 0]
        return [(float(s), np.asarray(m, bool), b) for s, m, b in zip(numpy(state['scores']), masks, numpy(state['boxes']))]
    return SimpleNamespace(raw=raw,processor=processor,to_numpy=numpy,image_cls=Image)


def preview_pair(directory, config_path, instruction, output=None):
    import numpy as np
    from PIL import Image
    from cart_book_grasp_v2_1_numbered import build_catalog,select_number,numbered_overlay,catalog_summary
    from cart_book_grasp_v2_1_bookends import detect_bookends,bookend_overlay,filter_summary
    directory = Path(directory)
    c = load_config(config_path,require_semantic_model=False)
    metadata = json.loads((directory/'camera.json').read_text())
    gap = metadata.get('pair_gap_seconds')
    if type(gap) not in (int, float) or not np.isfinite(gap) or not 0 <= gap <= 3:
        raise SelectionError('离线验证需要真实配对采集的时间间隔，不能混用不同时刻的图片')
    head_rgb = np.asarray(Image.open(directory/'head_rgb.png').convert('RGB'))
    wrist_rgb = np.asarray(Image.open(directory/'wrist_rgb.png').convert('RGB'))
    depth = np.load(directory/'depth.npy', allow_pickle=False)
    k, c2w = np.asarray(metadata['K'], float), np.asarray(metadata['c2w'], float)
    output = Path(output) if output else directory/('preview_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True, exist_ok=False)
    report = dict(status='started', source=str(directory.resolve()), instruction=instruction, config=c,
                  pair_gap_seconds=gap, pose_verified=metadata.get('pose_verified') is True,
                  robot_executed=False,gpt_called=False,semantic_model_called=False,
                  selection_mode='number',color_gate_used=False,cross_view_identity_required=False,timings={})
    adapter = None
    def get_adapter():
        nonlocal adapter
        if adapter is None:
            start=time.monotonic();adapter=_sam3_adapter()
            report['timings']['model_load_s']=time.monotonic()-start
        return adapter
    def raw(rgb, name):
        cache = directory/(name+'_raw.npz')
        fingerprint = hashlib.sha256(rgb.tobytes()).hexdigest()
        if cache.exists():
            with np.load(cache, allow_pickle=False) as d:
                if 'frame_sha256' in d and str(d['frame_sha256']) == fingerprint:
                    return list(zip(d['scores'], d['masks'], d['boxes']))
        sam=get_adapter()
        start = time.monotonic(); values = sam.raw(rgb)
        report['timings'][name+'_sam3_s'] = time.monotonic()-start
        np.savez_compressed(cache, frame_sha256=fingerprint, masks=np.array([v[1] for v in values]),
                            scores=np.array([v[0] for v in values]), boxes=np.array([v[2] for v in values]))
        return values
    end_results={}
    def detect_ends(items):
        nonlocal end_results
        def factory():
            sam=get_adapter()
            return sam.processor,sam.to_numpy,sam.image_cls
        start=time.monotonic()
        end_results,detail=detect_bookends(head_rgb,items,c,factory,cache_dir=directory)
        report['timings']['head_bookend_s']=time.monotonic()-start
        return end_results,detail
    head=[]
    try:
        head_raw=raw(head_rgb,'head')
        report['proposal_filter']=dict(head={})
        head=build_catalog(head_raw,head_rgb,depth,k,c2w,c,report['proposal_filter']['head'],
                           bookend_detector=detect_ends)
        report.update(catalog=catalog_summary(head),status='catalog_ready')
        Image.fromarray(numbered_overlay(head_rgb,head)).save(output/'head_numbered.png')
        Image.fromarray(wrist_rgb).save(output/'wrist_context.png')
        if instruction is not None:
            selected=select_number(instruction,head)
            report.update(selected_id=selected['id'],selected_number=selected['number'],status='selected_offline')
            Image.fromarray(numbered_overlay(head_rgb,head,selected['number'])).save(output/'head_selected.png')
    except (ValueError, np.linalg.LinAlgError) as error:
        report.update(status='rejected', error=str(error))
        report.update(getattr(error, 'diagnostics', {}))
    finally:
        end_report=report.get('proposal_filter',{}).get('head',{}).get('bookends',{})
        Image.fromarray(bookend_overlay(head_rgb,head,end_results,end_report)).save(output/'head_bookend_filter.png')
        (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(filter_summary(end_report))
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f'离线结果：{output.resolve()}；未计算或执行抓取动作。')
    return 0 if report['status'] in ('selected_offline','catalog_ready') else 2


def register_reference(directory, config_path, direction, output):
    """Operator-confirmed direction; validates complete paired first_reset data."""
    import shutil
    import numpy as np
    from PIL import Image
    from cart_book_grasp_v2_1_selection import build_observations
    directory=Path(directory);output=Path(output)
    c=load_config(config_path)
    metadata=json.loads((directory/'camera.json').read_text())
    if metadata.get('pose_verified') is not True:
        raise SelectionError('参考必须来自 --observe-only 的 first_reset 实测采集；普通 --capture-pair 不能登记')
    pose=metadata.get('pose',{})
    actual=np.asarray(pose.get('right_joint_deg'),float)
    expected=np.asarray(pose.get('expected_right_joint_deg'),float)
    if actual.shape!=(7,) or expected.shape!=(7,) or not np.isfinite(actual).all() or not np.isfinite(expected).all():
        raise SelectionError('参考缺少有效关节证据')
    if np.max(np.abs(actual-expected))>c['pose_tolerance_deg']:
        raise SelectionError('参考未到 first_reset 姿态')
    gap=metadata.get('pair_gap_seconds')
    if type(gap) not in (int,float) or not np.isfinite(gap) or not 0<=gap<=3:
        raise SelectionError('参考双视角采集不同步')
    depth=np.load(directory/'depth.npy',allow_pickle=False)
    k=np.asarray(metadata['K'],float);c2w=np.asarray(metadata['c2w'],float)
    views={}
    for view in ('head','wrist'):
        rgb=np.asarray(Image.open(directory/(view+'_rgb.png')).convert('RGB'))
        with np.load(directory/(view+'_raw.npz'),allow_pickle=False) as data:
            if 'frame_sha256' not in data or str(data['frame_sha256'])!=hashlib.sha256(rgb.tobytes()).hexdigest():
                raise SelectionError('参考 SAM3 mask 与 RGB 图像不属于同一帧')
            raw=list(zip(data['scores'],data['masks'],data['boxes']))
            views[view]=build_observations(raw,rgb,c,view,depth,k,c2w)
    output.mkdir(parents=True,exist_ok=False)
    hashes={}
    for view,items in views.items():
        name=view+'_rgb.png';shutil.copy2(directory/name,output/name)
        maskname=view+'_reference_masks.npz'
        np.savez_compressed(output/maskname,masks=np.array([v['book_mask'] for v in items]))
        for filename in (name,maskname):hashes[filename]=hashlib.sha256((output/filename).read_bytes()).hexdigest()
    shutil.copy2(directory/'camera.json',output/'camera.json')
    hashes['camera.json']=hashlib.sha256((output/'camera.json').read_bytes()).hexdigest()
    if len(views['head'])!=len(views['wrist']) or not views['head']:
        raise SelectionError('登记同向/反向参考需要两侧数量一致且逐一人工确认')
    count=len(views['head'])
    reference=dict(schema=1,verification='operator_confirmed',pose_verified=True,
        direction=direction,head_to_wrist=list(range(count)) if direction=='same' else list(reversed(range(count))),
        pair_directory='.',source=str(directory.resolve()),sha256=hashes,
        note='Direction explicitly confirmed by the operator; not inferred from candidate counts.')
    target=output/'reference.json';target.write_text(json.dumps(reference,indent=2))
    print(f'已登记参考：{target.resolve()}；请将 selector 配置的 reference_path 设置为此路径。')
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--selector-config', type=Path, default=CONFIG_PATH)
    modes = p.add_mutually_exclusive_group()
    modes.add_argument('--check-config', action='store_true')
    modes.add_argument('--capture-pair', type=Path, metavar='NEW_DIR')
    modes.add_argument('--preview-pair', type=Path, metavar='PAIR_DIR')
    modes.add_argument('--register-reference', type=Path, metavar='PAIR_DIR', help='Register operator-verified paired first_reset images')
    p.add_argument('--direction', choices=['same','reversed'], help='Operator-confirmed head +Y-to-Y order vs wrist image left-to-right')
    modes.add_argument('--observe-only', action='store_true', help='Existing first_reset motion, then observation only; no grasp')
    p.add_argument('--book-number','--instruction',dest='instruction',default=None,
                   help='Offline head book number; omit to preview numbered candidates without selection')
    p.add_argument('--output', type=Path, help='New offline preview directory')
    args, remaining = p.parse_known_args(argv)
    if (args.check_config or args.capture_pair or args.preview_pair or args.register_reference) and remaining:
        p.error('Unknown utility arguments: '+' '.join(remaining))
    load_config(args.selector_config,require_semantic_model=False)
    if args.direction and not args.register_reference:
        p.error('--direction is only valid with --register-reference')
    if args.register_reference:
        if not args.direction or not args.output:
            p.error('--register-reference requires --direction and --output NEW_DIR')
        return register_reference(args.register_reference, args.selector_config, args.direction, args.output)
    if args.check_config:
        from cart_book_grasp_v2_1_numbered import NumberedBookSelectionAgent
        NumberedBookSelectionAgent(args.selector_config).check_ready()
        print('V2.1 配置通过；先识别再编号选书，无颜色/跨视角身份门槛；未初始化机器人或相机。')
        return 0
    if args.capture_pair:
        capture_pair(args.capture_pair)
        return 0
    if args.preview_pair:
        return preview_pair(args.preview_pair, args.selector_config, args.instruction, args.output)
    from cart_book_grasp_v2_1_numbered import NumberedBookSelectionAgent,NumberedCartLoop
    from cart_book_grasp_v2 import main as run_v2
    from cart_book_grasp_v2_1_motion import run_v13
    selector=NumberedBookSelectionAgent(args.selector_config, observe_only=args.observe_only)
    selector.check_ready()
    selector.warmup()
    saved_argv = sys.argv
    sys.argv = [saved_argv[0], *remaining]
    try:
        run_v2(selector=selector,book_entry=run_v13,loop_factory=NumberedCartLoop,
            config_path=ROOT/'cart_book_grasp_v2_1_motion_config.json',
            wrist_config_path=ROOT/'cart_book_grasp_v2_1_wrist_config.json')
    finally:
        sys.argv = saved_argv
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError) as error:
        print(f'V2.1: {error}', file=sys.stderr)
        raise SystemExit(2)
