"""V3.4: direct visual servo; ordinary robot actions retain CuRobo planning."""
import argparse
import sys
from pathlib import Path
from cart_book_grasp_v3_column import load_config
CONFIG_PATH=Path(__file__).with_name('cart_book_grasp_v3_4_column.json')
from cart_book_grasp_v3_4_policy import load_stream_config
CONFIG=Path(__file__).with_name('cart_book_grasp_v3_4_stream.json')
from cart_book_grasp_v3_4_wrist_gpt import wrist_fallback
from cart_book_grasp_v3_camera import WristCamera


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--column-config',type=Path,default=CONFIG_PATH)
    parser.add_argument('--stream-config',type=Path,default=CONFIG)
    parser.add_argument('--check-config',action='store_true')
    args,remaining=parser.parse_known_args(argv)
    c=load_config(args.column_config);s=load_stream_config(args.stream_config)
    print('V3.4 头部分割：使用共享源最高原始分辨率 RGB-D，不使用缩略图。',flush=True)
    print('V3.4 腕部备用：SAM3 拒绝后直接使用 GPT-6 圈定区域；不使用 SAM2 或边界算法。',flush=True)
    from cart_book_grasp_v3_2_head_fallback import load_config as load_head_fallback
    head_fallback=load_head_fallback()
    print(f"V3.4 头部三维书脊倾角超过 {head_fallback.get('tilt_review_threshold_deg',45.):g}°：GPT 复核同一本书，再以实测深度重算。",flush=True)
    if c['mode']!='centerline_distance':parser.error('V3.4 requires centerline_distance')
    # V3.4 translation/gain are independently bounded by load_stream_config.
    # The shared column max_step_mm remains the original blocking V3 setting.
    if s['lookahead_roll_deg']>c['max_roll_step_deg']:
        parser.error('Streaming roll lookahead exceeds existing rotation limit')
    camera=WristCamera()
    try:
        if c['target_u'] is not None and not 2<=c['target_u']<camera.size[0]-2:parser.error('target_u outside image')
        print(f"V3.4 滚动目标：gain={s['gain']:g}，最多 {s['update_hz']:g} Hz 更新；中间不等到位，探测和终点仍等待。",flush=True)
        print(f"V3.4 前伸目标距离上限 {s['forward_lead_mm']:g} mm；偏差接近恢复阈值时收短，恢复后 {s['forward_ramp_seconds']:g}s 渐增。",flush=True)
        print('视觉阶段使用 CPU Jacobian 关节伺服；复位、粗定位和抓取仍使用 CuRobo。',flush=True)
        if args.check_config:
            if remaining:parser.error('Unknown check arguments: '+' '.join(remaining))
            print('配置和标定通过；未连接设备。运行时复用 V3.1 直接伺服适配器（10176）；原规划器不变。');return
        planner_scripts=Path(__file__).resolve().parent.parent/'dual_v2_2_control/cuarm_upper_planner/scripts'
        sys.path.insert(0,str(planner_scripts))
        from v3_1_readiness import require_ready
        require_ready()
        from cart_book_grasp_v3_1_ports import dedicated_planner
        import dual_v2_2_agent_pcl as agent
        from cart_book_grasp_v1_3 import main as v13_main
        from cart_book_grasp_v3_motion import installed
        root=Path(__file__).parent;saved=sys.argv;sys.argv=[saved[0],*remaining]
        saved_camera_factory=agent.start_calibrated_orbbec
        from head_rgbd.client import CalibratedCamera
        agent.start_calibrated_orbbec=lambda calibration,warmup_frames=5: CalibratedCamera(
            calibration,warmup_frames=warmup_frames,prefer_native=True)
        try:
            with dedicated_planner(agent), installed(None,column_path=args.column_config,stream_config=s,wrist_boundary_selector=wrist_fallback):
                v13_main(config_path=root/'cart_book_grasp_v3_config.json',wrist_config_path=root/'cart_book_grasp_v3_wrist_config.json')
        finally:
            sys.argv=saved
            agent.start_calibrated_orbbec=saved_camera_factory
    finally:camera.close()


if __name__=='__main__':
    try:main()
    except (ValueError,OSError) as error:
        print(f'V3.4: {error}',file=sys.stderr);raise SystemExit(2)
