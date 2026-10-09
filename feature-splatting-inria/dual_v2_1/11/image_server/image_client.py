import cv2
import zmq
import numpy as np
import time
import struct
from collections import deque
from multiprocessing import shared_memory
import socket

from shared_memory_manager import SharedMemoryManager
from zmq_stream import parse_zmq_message

DEFAULT_DEPTH_SCALE = 0.00025
DEFAULT_MAX_DEPTH_M = 5.0


def decode_depth_jpeg(depth_jpg, depth_scale=DEFAULT_DEPTH_SCALE, max_depth_m=DEFAULT_MAX_DEPTH_M):
    """
    解码 image_server 深度 payload 为米。
    - uint16 PNG（推荐）: depth_m = raw * depth_scale
    - JET 伪彩色 JPEG（旧版）: 仅近似，不可信
    返回 (depth_m, is_raw_uint16)
    """
    if not depth_jpg:
        return None, False

    depth_raw = cv2.imdecode(np.frombuffer(depth_jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if depth_raw is None:
        return None, False

    if depth_raw.ndim == 2 and depth_raw.dtype == np.uint16:
        depth_m = depth_raw.astype(np.float32) * float(depth_scale)
        is_raw = True
    elif depth_raw.ndim == 2:
        depth_m = depth_raw.astype(np.float32) / 255.0 * float(max_depth_m)
        is_raw = False
    elif depth_raw.ndim == 3:
        gray = cv2.cvtColor(depth_raw, cv2.COLOR_BGR2GRAY)
        depth_m = gray.astype(np.float32) / 255.0 * float(max_depth_m)
        is_raw = False
    else:
        return None, False

    invalid = ~np.isfinite(depth_m) | (depth_m <= 0)
    if invalid.any():
        depth_m = depth_m.copy()
        depth_m[invalid] = 0.0
    return depth_m, is_raw


def depth_m_to_vis(depth_m, max_depth_m=DEFAULT_MAX_DEPTH_M):
    """将深度米制图转伪彩色，用于 --show-depth 显示。"""
    if depth_m is None:
        return None
    vis = np.zeros_like(depth_m, dtype=np.float32)
    valid = np.isfinite(depth_m) & (depth_m > 0)
    vis[valid] = np.clip(depth_m[valid], 0.0, float(max_depth_m))
    vis_u8 = cv2.normalize(vis, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    return cv2.applyColorMap(vis_u8, cv2.COLORMAP_JET)


def get_center_depth_m(depth_m):
    """返回图像中心像素深度（米）及像素坐标。"""
    if depth_m is None:
        return None, None, None
    h, w = depth_m.shape[:2]
    cx, cy = w // 2, h // 2
    z = float(depth_m[cy, cx])
    return cx, cy, z


def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception as e:
        print(f"get local ip failed: {e}")
        return "127.0.0.1"


class ImageClient:
    def __init__(self, tv_img_shape=None, tv_image_shm: SharedMemoryManager=None, 
                       wrist_img_shape=None, wrist_image_shm: SharedMemoryManager=None,
                       image_show = False, depth_show = False, server_address = "127.0.0.1", port = 5555, Unit_Test = False,
                       print_center_depth = True, depth_scale = DEFAULT_DEPTH_SCALE, max_depth_m = DEFAULT_MAX_DEPTH_M):
        """
        tv_image_shm: Shared memory is used to easily transfer images across processes to the Vuer.

        wrist_image_shm: Shared memory is used to easily transfer images.
        
        image_show: Whether to display received images in real time.

        server_address: The ip address to execute the image server script.

        port: The port number to bind to. It should be the same as the image server.

        Unit_Test: When both server and client are True, it can be used to test the image transfer latency, \
                   network jitter, frame loss rate and other information.
        """
        self.running = True
        self._image_show = image_show
        self._depth_show = depth_show
        self._print_center_depth = print_center_depth
        self._depth_scale = float(depth_scale)
        self._max_depth_m = float(max_depth_m)
        self._depth_colormap_warned = False
        self._server_address = server_address
        self._port = port
        
        self.tv_img_shape = tv_img_shape
        self.wrist_img_shape = wrist_img_shape

        self.tv_enable_shm = False
        if self.tv_img_shape is not None:
            if tv_image_shm is not None:
                self.tv_image_shm = tv_image_shm
                input_size = np.prod(self.tv_img_shape) * np.uint8().itemsize
                assert input_size == self.tv_image_shm.get_size(), "input tv img shape must be equal to that of given tv image shm !!!"
            else:
                self.tv_image_shm = shared_memory.SharedMemory(shape=self.tv_img_shape)
            self.tv_enable_shm = True
            
        self.wrist_enable_shm = False
        if self.wrist_img_shape is not None:
            if wrist_image_shm is not None:
                self.wrist_image_shm = wrist_image_shm
                input_size = np.prod(self.wrist_img_shape) * np.uint8().itemsize
                assert input_size == self.wrist_image_shm.get_size(), "input wrist img shape must be equal to that of given wrist image shm !!!"
            else:
                self.wrist_image_shm = shared_memory.SharedMemory(shape=self.tv_img_shape)
            self.wrist_enable_shm = True

        # Performance evaluation parameters
        self._enable_performance_eval = Unit_Test
        if self._enable_performance_eval:
            self._init_performance_metrics()

    def _init_performance_metrics(self):
        self._frame_count = 0  # Total frames received
        self._last_frame_id = -1  # Last received frame ID

        # Real-time FPS calculation using a time window
        self._time_window = 1.0  # Time window size (in seconds)
        self._frame_times = deque()  # Timestamps of frames received within the time window

        # Data transmission quality metrics
        self._latencies = deque()  # Latencies of frames within the time window
        self._lost_frames = 0  # Total lost frames
        self._total_frames = 0  # Expected total frames based on frame IDs

    def _update_performance_metrics(self, timestamp, frame_id, receive_time):
        # Update latency
        latency = receive_time - timestamp
        self._latencies.append(latency)

        # Remove latencies outside the time window
        while self._latencies and self._frame_times and self._latencies[0] < receive_time - self._time_window:
            self._latencies.popleft()

        # Update frame times
        self._frame_times.append(receive_time)
        # Remove timestamps outside the time window
        while self._frame_times and self._frame_times[0] < receive_time - self._time_window:
            self._frame_times.popleft()

        # Update frame counts for lost frame calculation
        expected_frame_id = self._last_frame_id + 1 if self._last_frame_id != -1 else frame_id
        if frame_id != expected_frame_id:
            lost = frame_id - expected_frame_id
            if lost < 0:
                print(f"[Image Client] Received out-of-order frame ID: {frame_id}")
            else:
                self._lost_frames += lost
                print(f"[Image Client] Detected lost frames: {lost}, Expected frame ID: {expected_frame_id}, Received frame ID: {frame_id}")
        self._last_frame_id = frame_id
        self._total_frames = frame_id + 1

        self._frame_count += 1

    def _print_performance_metrics(self, receive_time):
        if self._frame_count % 30 == 0:
            # Calculate real-time FPS
            real_time_fps = len(self._frame_times) / self._time_window if self._time_window > 0 else 0

            # Calculate latency metrics
            if self._latencies:
                avg_latency = sum(self._latencies) / len(self._latencies)
                max_latency = max(self._latencies)
                min_latency = min(self._latencies)
                jitter = max_latency - min_latency
            else:
                avg_latency = max_latency = min_latency = jitter = 0

            # Calculate lost frame rate
            lost_frame_rate = (self._lost_frames / self._total_frames) * 100 if self._total_frames > 0 else 0

            print(f"[Image Client] Real-time FPS: {real_time_fps:.2f}, Avg Latency: {avg_latency*1000:.2f} ms, Max Latency: {max_latency*1000:.2f} ms, \
                  Min Latency: {min_latency*1000:.2f} ms, Jitter: {jitter*1000:.2f} ms, Lost Frame Rate: {lost_frame_rate:.2f}%")
    
    def _close(self):
        self._socket.close()
        self._context.term()
        if self._image_show or self._depth_show:
            cv2.destroyAllWindows()
        print("Image client has been closed.")
    
    def receive_process(self):
        # Set up ZeroMQ context and socket
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.connect(f"tcp://{self._server_address}:{self._port}")
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")

        print(f"\nImage client has started, connecting to tcp://{self._server_address}:{self._port} ...")
        try:
            while self.running:
                # Receive message
                message = self._socket.recv()
                receive_time = time.time()

                if self._enable_performance_eval:
                    header_size = struct.calcsize('dI')
                    try:
                        header = message[:header_size]
                        payload = message[header_size:]
                        timestamp, frame_id = struct.unpack('dI', header)
                    except struct.error as e:
                        print(f"[Image Client] Error unpacking header: {e}, discarding message.")
                        continue
                else:
                    payload = message
                    timestamp = frame_id = None

                color_jpg, depth_jpg = parse_zmq_message(payload if self._enable_performance_eval else message)
                if color_jpg is None:
                    print("[Image Client] Failed to parse frame payload.")
                    continue
                jpg_bytes = color_jpg
                # Decode image
                np_img = np.frombuffer(jpg_bytes, dtype=np.uint8)
                current_image = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
                if current_image is None:
                    print("[Image Client] Failed to decode image.")
                    continue

                depth_image = None
                depth_m = None
                depth_is_raw = False
                if depth_jpg:
                    depth_m, depth_is_raw = decode_depth_jpeg(
                        depth_jpg,
                        depth_scale=self._depth_scale,
                        max_depth_m=self._max_depth_m,
                    )
                    if depth_m is not None:
                        depth_image = (
                            depth_m_to_vis(depth_m, max_depth_m=self._max_depth_m)
                            if depth_is_raw
                            else cv2.imdecode(
                                np.frombuffer(depth_jpg, dtype=np.uint8), cv2.IMREAD_COLOR
                            )
                        )
                    if (
                        depth_m is not None
                        and not depth_is_raw
                        and not self._depth_colormap_warned
                    ):
                        print(
                            "[Image Client] 警告: 收到 JET 伪彩色深度，中心深度仅为近似值；"
                            "请升级 image_server 发送 uint16 PNG 原始深度。",
                            flush=True,
                        )
                        self._depth_colormap_warned = True

                if self._print_center_depth and depth_m is not None:
                    cx, cy, z = get_center_depth_m(depth_m)
                    src = "uint16" if depth_is_raw else "colormap~"
                    if z is None or z <= 0:
                        print(
                            f"[Image Client] center ({cx},{cy}) depth: invalid [{src}]",
                            flush=True,
                        )
                    else:
                        print(
                            f"[Image Client] center ({cx},{cy}) depth: {z:.3f} m [{src}]",
                            flush=True,
                        )
                
                if self.tv_enable_shm:
                    self.tv_image_shm.write_numpy(np.array(current_image[:, :self.tv_img_shape[1]]))
                
                if self.wrist_enable_shm:
                    self.wrist_image_shm.write_numpy(np.array(current_image[:, -self.wrist_img_shape[1]:]))
                
                if self._image_show or self._depth_show:
                    if self._image_show:
                        height, width = current_image.shape[:2]
                        display = cv2.resize(current_image, (width // 2, height // 2))
                        cv2.imshow('Image Client RGB', display)
                    if self._depth_show and depth_image is not None:
                        dh, dw = depth_image.shape[:2]
                        display = cv2.resize(depth_image, (dw // 2, dh // 2))
                        cv2.imshow('Image Client Depth', display)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.running = False

                if self._enable_performance_eval:
                    self._update_performance_metrics(timestamp, frame_id, receive_time)
                    self._print_performance_metrics(receive_time)

        except KeyboardInterrupt:
            print("Image client interrupted by user.")
        except Exception as e:
            print(f"[Image Client] An error occurred while receiving data: {e}")
        finally:
            self._close()

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Image client for RealSense image_server")
    parser.add_argument("--host", required=True, help="image_server 所在机器的 IP 地址")
    parser.add_argument("--port", type=int, default=5555, help="Server port (default: 5555)")
    parser.add_argument("--show", action="store_true", help="Display RGB stream (needs local display)")
    parser.add_argument("--show-depth", action="store_true", help="Display depth stream (needs local display)")
    parser.add_argument(
        "--no-print-center-depth",
        action="store_true",
        help="关闭每帧中心点深度打印",
    )
    parser.add_argument(
        "--depth-scale",
        type=float,
        default=DEFAULT_DEPTH_SCALE,
        help="uint16 depth -> meters (default: 0.00025)",
    )
    parser.add_argument(
        "--max-depth-m",
        type=float,
        default=DEFAULT_MAX_DEPTH_M,
        help="JET colormap depth denormalize max (meters, default: 5.0)",
    )
    parser.add_argument("--unit-test", action="store_true", help="Enable performance metrics")
    args = parser.parse_args()

    client = ImageClient(
        image_show=args.show,
        depth_show=args.show_depth,
        server_address=args.host,
        port=args.port,
        Unit_Test=args.unit_test,
        print_center_depth=not args.no_print_center_depth,
        depth_scale=args.depth_scale,
        max_depth_m=args.max_depth_m,
    )
    client.receive_process()