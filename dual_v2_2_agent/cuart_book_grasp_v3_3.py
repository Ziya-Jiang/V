"""V3.3: direct visual servo; ordinary robot actions retain CuRobo planning."""
import argparse
import sys
from pathlib import Path
from cart_book_grasp_v3_column import load_config
CONFIG_PATH=Path(__file__).with_name('cart_book_grasp_v3_2_column.json')
from cart_book_grasp_v3_2_stream import CONFIG,load_stream_config
from cart_book_grasp_v3_camera import WristCamera


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--column-config',type=Path,default=CONFIG_PATH)
    parser.add_argument('--stream-config',type=Path,default=CONFIG)
    parser.add_argument('--check-config',action='store_true')
    args,remaining=parser.parse_known_args(argv)
    c=load_config(args.column_config);s=load_stream_config(args.stream_config)
    print('V3.3 头部分割：使用共享源最高原始分辨率 RGB-D，不使用缩略图。',flush=True)
    from cart_book_grasp_v3_2_head_fallback import load_config as load_head_fallback
    load_head_fallback()
    if c['mode']!='centerline_distance':parser.error('V3.3 requires centerline_distance')
    # V3.3 translation/gain are independently bounded by load_stream_config.
    # The shared column max_step_mm remains the original blocking V3 setting.
    if s['lookahead_roll_deg']>c['max_roll_step_deg']:
        parser.error('Streaming roll lookahead exceeds existing rotation limit')
    camera=WristCamera()
    try:
        if c['target_u'] is not None and not 2<=c['target_u']<camera.size[0]-2:parser.error('target_u outside image')
        distance_label=('无固定上限（受剩余路径预算约束）' if s['lookahead_mm'] is None
                        else f"{s['lookahead_mm']:g} mm")
        print(f"V3.3 滚动目标：gain={s['gain']:g}，最多 {s['update_hz']:g} Hz 更新，实测位置前瞻 {distance_label}；中间不等到位，探测和终点仍等待。",flush=True)
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
            with dedicated_planner(agent), installed(None,column_path=args.column_config,stream_config=s):
                v13_main(config_path=root/'cart_book_grasp_v3_config.json',wrist_config_path=root/'cart_book_grasp_v3_wrist_config.json')
        finally:
            sys.argv=saved
            agent.start_calibrated_orbbec=saved_camera_factory
    finally:camera.close()


if __name__=='__main__':
    try:main()
    except (ValueError,OSError) as error:
        print(f'V3.3: {error}',file=sys.stderr);raise SystemExit(2)
