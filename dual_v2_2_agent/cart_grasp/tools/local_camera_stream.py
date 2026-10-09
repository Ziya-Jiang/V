#!/usr/bin/env python3
"""Low-load local head-camera MJPEG preview; no depth, robot or RT imports."""
import argparse
import cv2
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np


class CameraState:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.updated = 0.0
        self.frames = 0
        self.error = None
        self.stop = threading.Event()


def capture_loop(state, serial, fps, quality):
    import pyorbbecsdk as ob
    cv2.setNumThreads(1)
    pipeline = None
    try:
        ctx = ob.Context()
        devices = ctx.query_devices()
        device = devices.get_device_by_serial_number(serial)
        profiles = device.get_sensor(ob.OBSensorType.COLOR_SENSOR).get_stream_profile_list()
        profile = profiles.get_video_stream_profile(0)  # calibrated 848x530 YUYV head stream
        pipeline = ob.Pipeline(device)
        config = ob.Config()
        config.enable_stream(profile)
        pipeline.start(config)
        interval = 1.0 / fps
        next_frame = time.monotonic()
        while not state.stop.is_set():
            frames = pipeline.wait_for_frames(500)
            if frames is None:
                continue
            frame = frames.get_color_frame()
            if frame is None:
                continue
            raw = np.frombuffer(frame.get_data(), np.uint8)
            fmt = frame.get_format()
            h, w = frame.get_height(), frame.get_width()
            if fmt == ob.OBFormat.YUYV:
                image = cv2.cvtColor(raw.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUY2)
            elif fmt == ob.OBFormat.MJPG:
                image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            elif fmt == ob.OBFormat.BGR:
                image = raw.reshape(h, w, 3)
            else:
                raise RuntimeError("Unsupported color format: " + str(fmt))
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                continue
            with state.lock:
                state.jpeg = encoded.tobytes()
                state.updated = time.time()
                state.frames += 1
                state.error = None
            next_frame += interval
            delay = next_frame - time.monotonic()
            if delay > 0:
                state.stop.wait(delay)
            else:
                next_frame = time.monotonic()
    except Exception as exc:
        with state.lock:
            state.error = str(exc)
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass


class Handler(BaseHTTPRequestHandler):
    state = None

    def log_message(self, *_):
        return

    def do_GET(self):
        if self.path == '/':
            body = b'''<!doctype html><meta charset="utf-8"><title>vlmdemo head camera</title>
<style>body{margin:0;background:#111;color:#ddd;font:16px sans-serif}img{max-width:100vw;max-height:94vh;display:block;margin:auto}p{margin:8px;text-align:center}</style>
<img src="/stream.mjpg"><p>Read-only vlmdemo head camera preview</p>'''
            self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body); return
        if self.path == '/health':
            with self.state.lock:
                data = {'frames': self.state.frames, 'updated_unix_s': self.state.updated, 'error': self.state.error, 'motion_commands_sent': False}
            body = json.dumps(data).encode(); self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body); return
        if self.path == '/snapshot.jpg':
            with self.state.lock: image = self.state.jpeg
            if image is None: self.send_error(503, 'camera not ready'); return
            self.send_response(200); self.send_header('Content-Type', 'image/jpeg'); self.send_header('Cache-Control', 'no-store'); self.send_header('Content-Length', str(len(image))); self.end_headers(); self.wfile.write(image); return
        if self.path != '/stream.mjpg':
            self.send_error(404); return
        self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame'); self.send_header('Cache-Control', 'no-store'); self.end_headers()
        last = -1
        try:
            while not self.state.stop.is_set():
                with self.state.lock: image, stamp = self.state.jpeg, self.state.updated
                if image is not None and stamp != last:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(image)).encode() + b'\r\n\r\n' + image + b'\r\n'); self.wfile.flush(); last = stamp
                time.sleep(.02)
        except (BrokenPipeError, ConnectionResetError):
            return


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=8800)
    p.add_argument('--serial', default='CV2TC5100060')
    p.add_argument('--fps', type=float, default=5.)
    p.add_argument('--quality', type=int, default=75)
    args = p.parse_args()
    if not 1 <= args.fps <= 10 or not 10 <= args.quality <= 95 or not 1024 <= args.port <= 65535:
        raise SystemExit('invalid stream limits')
    state = CameraState(); Handler.state = state
    thread = threading.Thread(target=capture_loop, args=(state, args.serial, args.fps, args.quality), daemon=True); thread.start()
    server = ThreadingHTTPServer((args.host, args.port), Handler); server.timeout = .5
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, lambda *_: (state.stop.set(), server.shutdown()))
    try: server.serve_forever()
    finally: state.stop.set(); server.server_close(); thread.join(timeout=3)


if __name__ == '__main__': main()
