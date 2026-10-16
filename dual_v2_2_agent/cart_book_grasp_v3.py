"""V1_3 grasp with a taught wrist point and bounded BASE Y/Z image servo.

python V3.py --help
Imports and utilities never connect to the robot. --teach captures a stationary
wrist image; it does not move the arm into a teaching pose.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import uuid

from cart_book_grasp_v3_servo import CONFIG_PATH, ServoError, load_config

ROOT = Path(__file__).resolve().parent


def save_reference(path, uv, gap, euler, metadata):
    config = load_config(path, require_reference=False)
    config['reference'] = dict(uv=list(uv), surface_gap_mm=gap, tool_euler_deg=list(euler),
        **{k: metadata[k] for k in ('device', 'width', 'height', 'coordinate_system', 'calibration_sha256')})
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', dir=path.parent,
                                     encoding='utf-8', delete=False) as stream:
        temporary = Path(stream.name)
        json.dump(config, stream, indent=2)
        stream.write('\n')
    try:
        load_config(temporary)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def capture_reference(camera, output):
    import cv2
    if output.suffix.lower() != '.png':
        raise ServoError('Use a .png path for the rectified teaching image')
    metadata_path = output.with_suffix('.json')
    if output.exists() or metadata_path.exists():
        raise ServoError('Teaching output already exists; choose a new path')
    frame = camera.capture()
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), cv2.cvtColor(frame['rgb'], cv2.COLOR_RGB2BGR)):
        raise ServoError('Cannot save teaching image')
    metadata = dict(**{k: camera.config[k] for k in ('device', 'width', 'height')},
                    coordinate_system='undistorted_K', calibration_sha256=camera.calibration_hash)
    metadata_path.write_text(json.dumps(metadata, indent=2)+'\n')
    raw = output.with_name(output.stem+'_raw.png')
    if not raw.exists():
        cv2.imwrite(str(raw), cv2.cvtColor(frame['raw_rgb'], cv2.COLOR_RGB2BGR))
    return frame['rgb'], metadata


def pick_point(rgb):
    """Explicit point teaching on a frozen rectified frame, Enter to accept."""
    import cv2
    selected = []
    name = 'V3 teach / select V1_3 spine alignment point'
    def click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            selected[:] = [float(x), float(y)]
    cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(name, click)
    try:
        while True:
            image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.putText(image, 'Click alignment point; Enter saves; Esc cancels (no robot motion)',
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2)
            if selected:
                cv2.drawMarker(image, tuple(map(int, selected)), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
            cv2.imshow(name, image)
            key = cv2.waitKey(30) & 255
            if key in (10, 13) and selected:
                return selected
            if key in (27, ord('q')) or cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1:
                raise ServoError('Teaching cancelled; reference unchanged')
    finally:
        cv2.destroyWindow(name)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--servo-config', type=Path, default=CONFIG_PATH)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--check-setup', action='store_true', help='Check calibration/dependencies without a reference or hardware')
    modes.add_argument('--check-config', action='store_true', help='Validate taught reference without opening hardware')
    modes.add_argument('--capture-wrist', type=Path, metavar='NEW_PNG', help='Save rectified image + calibration metadata; no robot')
    modes.add_argument('--teach', action='store_true', help='Capture and click taught point in a frozen image; no robot')
    modes.add_argument('--set-reference', nargs=2, type=float, metavar=('U', 'V'), help='Register a point measured on --reference-image')
    parser.add_argument('--reference-image', type=Path, help='Rectified PNG previously created by --capture-wrist')
    parser.add_argument('--surface-gap-mm', type=float, help='Measured book surface BASE-X minus right TCP BASE-X when teaching')
    parser.add_argument('--tool-euler-deg', nargs=3, type=float, metavar=('RX', 'RY', 'RZ'), help='Measured right tool Euler XYZ angles when teaching')
    args, remaining = parser.parse_known_args(argv)
    utility = args.check_setup or args.check_config or args.capture_wrist or args.teach or args.set_reference is not None
    if utility and remaining:
        parser.error('Unknown utility arguments: '+' '.join(remaining))
    teaching = args.teach or args.set_reference is not None
    if teaching and (args.surface_gap_mm is None or args.tool_euler_deg is None):
        parser.error('Teaching requires --surface-gap-mm and --tool-euler-deg from the stationary teaching pose')
    if not teaching and (args.surface_gap_mm is not None or args.tool_euler_deg is not None or args.reference_image is not None):
        parser.error('Teaching pose/image arguments require --teach or --set-reference')
    if args.teach and args.reference_image is not None:
        parser.error('--teach captures a fresh image; --reference-image is for --set-reference')
    # Missing target must fail before loading the robot or starting first_reset.
    config = load_config(args.servo_config, require_reference=not (teaching or args.check_setup or args.capture_wrist))
    from cart_book_grasp_v3_camera import WristCamera
    camera = WristCamera()
    if args.check_setup:
        print('V3 环境通过：V1_3 + 标定内参 + 去畸变；reference '+('已登记' if config['reference'] else '尚未示教')+'。未打开相机或连接机器人。')
        return
    if args.capture_wrist:
        capture_reference(camera, args.capture_wrist)
        print(f'去畸变图像及标定信息已保存：{args.capture_wrist.resolve()}；未连接机器人。')
        return
    if teaching:
        if args.teach:
            output = ROOT/'RobotRAM'/'output'/('v3_teaching_'+uuid.uuid4().hex)/'reference.png'
            rgb, metadata = capture_reference(camera, output)
            uv = pick_point(rgb)
        else:
            if args.reference_image is None:
                parser.error('--set-reference requires --reference-image from --capture-wrist')
            import cv2
            output = args.reference_image
            image = cv2.imread(str(output))
            if image is None or image.shape[:2] != (camera.size[1], camera.size[0]):
                raise ServoError('Missing or wrong-size reference image')
            metadata = json.loads(output.with_suffix('.json').read_text())
            camera.check_reference(metadata)
            uv = args.set_reference
        save_reference(args.servo_config, uv, args.surface_gap_mm, args.tool_euler_deg, metadata)
        output.with_name(output.stem+'_taught.json').write_text(json.dumps(
            load_config(args.servo_config)['reference'], indent=2)+'\n')
        print(f'示教点 {uv} 已保存：{args.servo_config}；图像：{output}。未连接机器人。')
        return
    camera.check_reference(config['reference'])
    if args.check_config:
        print('V3 示教、相机、分辨率和标定一致；未打开相机或连接机器人。')
        return
    from cart_book_grasp_v1_3 import main as v13_main
    from cart_book_grasp_v3_motion import installed
    saved_argv = sys.argv
    sys.argv = [saved_argv[0], *remaining]
    try:
        with installed(args.servo_config):
            v13_main(config_path=ROOT/'cart_book_grasp_v3_config.json',
                     wrist_config_path=ROOT/'cart_book_grasp_v3_wrist_config.json')
    finally:
        sys.argv = saved_argv


def cli():
    try:
        main()
    except (ValueError, OSError) as error:
        print(f'V3: {error}', file=sys.stderr)
        raise SystemExit(2)


if __name__ == '__main__':
    cli()
