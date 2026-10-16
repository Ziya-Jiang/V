"""V3 fast centerline alignment; vertical image position ignored."""
import argparse
import sys
from pathlib import Path
from cart_book_grasp_v3_column import CONFIG_PATH,load_config
from cart_book_grasp_v3_camera import WristCamera


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--column-config',type=Path,default=CONFIG_PATH)
    parser.add_argument('--check-config',action='store_true')
    parser.add_argument('--check-setup',action='store_true')
    args,remaining=parser.parse_known_args(argv)
    if any(x in remaining for x in ('--cotracker-config','--mask')):
        parser.error('Use --legacy-cotracker for CoTracker options; default V3 uses --column-config')
    c=load_config(args.column_config);camera=WristCamera()
    if c['target_u'] is not None and not 2<=c['target_u']<camera.size[0]-2:
        parser.error('target_u is outside calibrated image')
    print('V3 模式：特征点跟踪中线（少于 6 点转固定姿态直接前伸）→ Y 与腕部轴向旋转同时对准；Z 固定；沿 V1_3 深度前伸。无需 CoTracker 模型。' if c['mode']=='centerline_distance' else 'V3 模式：中心列 Y 对准；Z 与姿态固定。',flush=True)
    if args.check_config or args.check_setup:
        if remaining:parser.error('Unknown utility arguments: '+' '.join(remaining))
        print('视觉伺服配置和腕部标定通过；未打开相机或连接机器人。');return
    from cart_book_grasp_v1_3 import main as v13_main
    from cart_book_grasp_v3_motion import installed
    root=Path(__file__).parent;saved=sys.argv;sys.argv=[saved[0],*remaining]
    try:
        with installed(None,column_path=args.column_config):
            v13_main(config_path=root/'cart_book_grasp_v3_config.json',wrist_config_path=root/'cart_book_grasp_v3_wrist_config.json')
    finally:sys.argv=saved;camera.close()
