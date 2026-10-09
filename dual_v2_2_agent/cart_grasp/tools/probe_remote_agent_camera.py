#!/usr/bin/env python3
"""Use the original Agent camera interface with erazer RGB-D; no RobotArm."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, default=5)
    args = parser.parse_args()
    if not 1 <= args.count <= 60:
        parser.error('count must be 1..60')
    v = Path('/mnt/ssd/user/V')
    root = v/'dual_v2_2_agent/cart_grasp'
    sys.path.insert(0, str(v/'dual_v2_2_agent/RobotRAM'))
    from controllor.extern_camera import OrbbecCamera
    from seldon_cart_grasp.remote_rgbd import remote_camera_class, pull
    from seldon_cart_grasp.frame_graph import FrameGraph
    from seldon_cart_grasp.rgbd_bridge import blue_points
    from seldon_cart_grasp.handle_pair import detect_handle_pair
    half = FrameGraph(v/'dual_v2_2_configuration/share/left_hand_right_gripper_half/dual_arm_half.urdf')
    destination = root/'runtime/erazer_rgbd'
    camera = remote_camera_class(OrbbecCamera)(destination, half.relative('dummy_world', 'SJ4Z'))
    evidence = []
    def consume(record):
        rgb, depth_mm = camera.capture_image()
        result = {'sequence': record['sequence'], 'rgb_shape': list(rgb.shape),
                  'depth_mm_positive_fraction': float(np.mean(depth_mm > 0)),
                  'camera_T_half_mm': camera.T.tolist(), 'motion_commands_sent': False}
        with np.load(destination/'latest.npz', allow_pickle=False) as f:
            data = {key: f[key] for key in f.files}
        try:
            obs = detect_handle_pair(blue_points(data), float(data['stamp_s']))
            result['handle_proposal'] = obs
        except Exception as error:
            result['handle_rejection'] = str(error)
        evidence.append(result)
        (root/'runtime/original_agent_remote_camera.json').write_text(json.dumps({
            'camera_parent': OrbbecCamera.__module__, 'local_usb_opened': False,
            'robotarm_constructed': False, 'observations': evidence}, indent=2))
        print(json.dumps(result), flush=True)
    command = ('cd /home/erazer/Seldon/cart_grasp && OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 '
               '/home/erazer/Seldon/keyboard_cockpit/.venv/bin/python -m seldon_cart_grasp.remote_rgbd '
               'serve --path runtime/rgbd/latest.npz')
    try:
        pull(['ssh', '-S', '/tmp/cart-erazer-control', '-o', 'BatchMode=yes',
              '-o', 'ConnectTimeout=5', 'erazer@10.30.3.155'], command, destination,
             count=args.count, on_frame=consume)
    finally:
        camera.close()


if __name__ == '__main__':
    main()
