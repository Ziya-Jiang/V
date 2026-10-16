"""Local read-only live viewer. No robot sockets or motion commands."""
import base64
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import cv2
import numpy as np
from .core import Renderer, RUNTIME, BOOT, read_state
from .stream import CameraStream
from head_rgbd.client import SharedCamera

def shared_camera_stream():
    return CameraStream(factory=SharedCamera, warmup=1)

def jpeg(image):
    ok,data=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,87])
    if not ok:raise ValueError('JPEG encoding failed')
    return base64.b64encode(data).decode('ascii')

class Live:
    def __init__(self,state_path):
        self.state_path=state_path;self.lock=threading.Lock();self.stop=threading.Event()
        self.data={'mode':'starting','message':'正在连接头部 RGB-D','image':None}
        self.raw=None;self.frame_stamp=0.;self.state_stamp=0.;self.has_ar=False;self.camera_stream=None
        if state_path==RUNTIME/'state.json':
            try:
                held=json.loads((RUNTIME/'held_frame.json').read_text())
                if held.get('boot_id')==BOOT and held.get('image') and held.get('height_mm') is not None:
                    self.data=held;self.frame_stamp=held['frame_unix_s'];self.has_ar=True
                    self.data.update(mode='paused',message='保留上次画面 · 等待新的面板实测状态')
            except (OSError,ValueError,KeyError):pass

    def snapshot(self,with_image=True):
        with self.lock:
            d=self.data.copy();now=time.time()
            d['frame_age_s']=round(now-self.frame_stamp,3) if self.frame_stamp else None
            d['frame_unix_s']=self.frame_stamp or None
            if self.camera_stream:d.update(self.camera_stream.status())
            if d['mode']=='live' and self.frame_stamp and now-self.frame_stamp>.6:
                d.update(mode='paused',message='画面已暂停 · 保留上一帧，等待新数据')
            elif d['mode']=='live' and now-self.state_stamp>.5:
                d.update(mode='paused',message='画面已暂停 · 手部状态过期，保留上一帧')
            if not with_image:d.pop('image',None)
            return d

    def run(self,renderer_factory=Renderer,stream_factory=shared_camera_stream):
        camera=None
        try:
            renderer=renderer_factory();camera=stream_factory();self.camera_stream=camera
            last=time.monotonic();fps=0.
            while not self.stop.is_set():
                try:
                    bgr,depth,stamp,metadata=camera.read()
                    if time.time()-stamp>.5:raise ValueError('等待相机最新帧 · 自动恢复中')
                    renderer.configure_camera(metadata)
                    raw=jpeg(bgr);state=None
                    try:
                        source=json.loads(Path(self.state_path).read_text())
                        if source.get('source_kind')!='panel_measured':raise ValueError('尚未收到面板实测状态，请检查上位机 Remote 连接')
                        state=read_state(self.state_path)
                        if state.get('urdf_sha256')!=renderer.urdf_hash:raise ValueError('手部状态 URDF 不匹配')
                        if abs(stamp-state['captured_unix_s'])>.2:raise ValueError('图像与手部状态时间差超过 200ms')
                        height=float(state['position_mm'][2])
                        rendered=renderer.render(bgr,depth,height)
                        mode='live';message='实时跟随右手末端高度 · 面上原色 · 面下压暗 · 遮挡裁切关闭';image=jpeg(rendered)
                    except (OSError,ValueError,KeyError,TypeError) as e:
                        mode='waiting_state';message='持续监听手部状态，Connect 后自动恢复' if isinstance(e,FileNotFoundError) else str(e)+' · 持续监听，恢复后自动更新'
                        height=None;image=raw
                    now=time.monotonic();dt=now-last;last=now;fps=.8*fps+.2/max(dt,.001)
                    with self.lock:
                        # Keep RGB, depth-derived overlay and height as one coherent
                        # frame; never paste old geometry over a newer camera image.
                        if mode!='live' and self.has_ar:
                            self.data.update(mode='paused',message='等待状态恢复 · '+message,
                                state_source=state.get('source','未接通') if state else '未接通')
                            continue
                        self.has_ar=mode=='live'
                        self.raw=raw;self.frame_stamp=stamp;self.state_stamp=state['captured_unix_s'] if state else 0.
                        self.data=dict(mode=mode,message=message,image=image,height_mm=height,fps=round(fps,1),
                            reference='arm_end_effector_r',frame='dummy_world',approach='EE -Z → base +X',
                            state_source=state.get('source','未接通') if state else '未接通',
                            state_age_s=round(time.time()-state['captured_unix_s'],3) if state else None,
                            depth_valid_fraction=round(float((depth>0).mean()),3),camera_serial=metadata['serial'])
                except Exception as e:
                    with self.lock:self.data.update(mode='paused' if self.data.get('image') else 'camera_error',message='自动恢复中 · '+str(e))
                    self.stop.wait(.1)
        except Exception as e:
            with self.lock:self.data.update(mode='paused' if self.data.get('image') else 'camera_error',message=str(e))
        finally:
            if camera:camera.close()

def serve(port=8791,state_path=RUNTIME/'state.json',duration=None):
    live=Live(state_path)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            if self.path.split('?')[0]=='/':
                data=Path(__file__).with_name('viewer.html').read_bytes();mime='text/html; charset=utf-8'
            elif self.path.split('?')[0]=='/api/depth':
                frame=live.camera_stream.peek() if live.camera_stream else None
                if frame is None or time.time()-frame[2]>.6:
                    self.send_error(503,'Fresh RGB-D frame unavailable');return
                bgr,depth,stamp,metadata=frame
                mm=np.rint(np.where(np.isfinite(depth),depth,0)).clip(0,65535).astype(np.uint16)
                ok,png=cv2.imencode('.png',mm)
                if not ok:self.send_error(500);return
                data=json.dumps(dict(captured_unix_s=stamp,depth_units='mm',
                    depth_png_base64=base64.b64encode(png).decode(),rgb_jpeg_base64=jpeg(bgr),metadata=metadata)).encode()
                mime='application/json; charset=utf-8'
            elif self.path.split('?')[0] in ('/api/frame','/api/status'):
                data=json.dumps(live.snapshot(self.path.split('?')[0]=='/api/frame'),ensure_ascii=False).encode();mime='application/json; charset=utf-8'
            else:self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler);server.timeout=.25
    def stop(*_):live.stop.set()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    worker=threading.Thread(target=live.run,name='readonly-rgbd',daemon=True);worker.start()
    print(f'Hand workspace AR: http://127.0.0.1:{port} (read-only)',flush=True)
    started=time.monotonic()
    try:
        while not live.stop.is_set() and (duration is None or time.monotonic()-started<duration):server.handle_request()
    finally:
        live.stop.set();worker.join(timeout=6);server.server_close()
