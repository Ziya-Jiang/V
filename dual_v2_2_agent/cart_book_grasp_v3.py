"""V2 dual-view book selection + wrist-image base-Y/Z visual servo.

Use --help for read-only utilities and taught-reference registration. Importing
this file does not initialize cameras, GPT, or the robot.
"""
import argparse
import json
from pathlib import Path
import sys

from cart_book_grasp_v3_servo import CONFIG_PATH, ServoError, load_config


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--servo-config', type=Path, default=CONFIG_PATH)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--check-config', action='store_true', help='Validate configuration without opening robot/cameras')
    modes.add_argument('--check-gpt', action='store_true')
    modes.add_argument('--capture-wrist', type=Path, metavar='NEW_PNG', help='Capture only; no robot commands')
    modes.add_argument('--set-reference', nargs=2, type=float, metavar=('U', 'V'), help='Record measured aligned target pixel, not image center')
    parser.add_argument('--surface-gap-mm', type=float, help='Measured book base-X minus TCP base-X at reference pose')
    parser.add_argument('--tool-euler-deg', nargs=3, type=float, metavar=('RX', 'RY', 'RZ'))
    args, remaining = parser.parse_known_args(argv)
    utility = args.check_config or args.check_gpt or args.capture_wrist or args.set_reference
    if utility and remaining:
        parser.error('Unknown utility arguments: '+' '.join(remaining))
    if args.set_reference is None and (args.surface_gap_mm is not None or args.tool_euler_deg is not None):
        parser.error('Reference pose arguments require --set-reference')
    if args.check_gpt:
        from cart_book_grasp_v2 import check_gpt_connection
        return check_gpt_connection()
    if args.capture_wrist:
        import cv2
        import time
        from cart_book_grasp_v2_dual_view import capture_wrist, load_config as camera_config
        if args.capture_wrist.exists():
            raise ServoError('Output already exists; choose a new image path')
        frame = capture_wrist(camera_config(), time.monotonic())
        if not cv2.imwrite(str(args.capture_wrist), cv2.cvtColor(frame['rgb'], cv2.COLOR_RGB2BGR)):
            raise ServoError('Cannot save wrist frame')
        print(f'手腕图像已保存：{args.capture_wrist.resolve()}；未连接机器人。')
        return
    if args.set_reference is not None:
        if args.surface_gap_mm is None or args.tool_euler_deg is None:
            parser.error('--set-reference requires --surface-gap-mm and --tool-euler-deg')
        from cart_book_grasp_v2_dual_view import load_config as camera_config
        config = load_config(args.servo_config, require_reference=False)
        camera = camera_config()
        config['reference'] = dict(uv=args.set_reference, surface_gap_mm=args.surface_gap_mm,
            tool_euler_deg=args.tool_euler_deg, **{k: camera[k] for k in ('device', 'width', 'height')})
        # Validate the complete new document before replacing the existing one.
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', dir=args.servo_config.parent,
                                         encoding='utf-8', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(config, stream, indent=2)
        try:
            load_config(temporary)
            temporary.replace(args.servo_config)
        finally:
            temporary.unlink(missing_ok=True)
        print(f'V3 实测参考已登记：{args.servo_config}；未连接机器人。')
        return
    load_config(args.servo_config)  # Missing reference fails before robot init.
    from cart_book_grasp_v2_dual_view import load_config as camera_config
    if not camera_config()['enabled']:
        raise ServoError('V3 requires dual-view selection enabled')
    if args.check_config:
        print('V3 配置通过；未连接机器人或相机。实机参考准确性仍需验证。')
        return
    from cart_book_grasp_v2 import main as v2_main
    from cart_book_grasp_v3_motion import installed
    saved_argv = sys.argv
    sys.argv = [saved_argv[0], *remaining]
    try:
        with installed(args.servo_config):
            v2_main(config_path=Path(__file__).with_name('cart_book_grasp_v3_config.json'),
                    wrist_config_path=Path(__file__).with_name('cart_book_grasp_v3_wrist_config.json'))
    finally:
        sys.argv = saved_argv


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError) as error:
        print(f'V3: {error}', file=sys.stderr)
        raise SystemExit(2)
