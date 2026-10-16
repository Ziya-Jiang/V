#!/usr/bin/env python3
"""Publish unrectified wrist-camera frames for the upstream ROS calibrator."""
import argparse
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=15)
    parser.add_argument('--topic', default='/wrist_camera/image_raw')
    parser.add_argument('--snapshot', type=Path, help='Read one frame, save it and exit (no ROS node).')
    args = parser.parse_args()
    if min(args.width, args.height, args.fps) <= 0:
        parser.error('width, height and fps must be positive')
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    node = None
    try:
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open camera: {args.device}')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(5):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('Camera did not deliver a frame')
        actual = (frame.shape[1], frame.shape[0])
        if actual != (args.width, args.height):
            raise RuntimeError(f'Requested {args.width}x{args.height}, received {actual}')
        if args.snapshot:
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.snapshot), frame):
                raise RuntimeError('Failed to save snapshot')
            print(f'Saved {actual[0]}x{actual[1]} frame to {args.snapshot}')
            return
        rclpy.init()
        node = Node('wrist_calibration_image_publisher')
        publisher = node.create_publisher(Image, args.topic, 1)
        bridge = CvBridge()
        failures = 0

        def publish():
            nonlocal failures
            ok, frame = cap.read()
            if not ok:
                failures += 1
                if failures >= 10:
                    raise RuntimeError('Camera stopped delivering frames')
                return
            failures = 0
            if (frame.shape[1], frame.shape[0]) != actual:
                raise RuntimeError('Camera resolution changed during calibration')
            msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'wrist_camera_optical_frame'
            publisher.publish(msg)

        node.create_timer(1.0 / args.fps, publish)
        node.get_logger().info(f'{args.device}: {actual} -> {args.topic}')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
