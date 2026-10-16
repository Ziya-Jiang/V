"""Continuously drain RGB-D into a single latest-frame slot, with recovery."""
import threading
import time
from .camera import Camera


class CameraStream:
    def __init__(self,factory=Camera,warmup=15,retry_delay=1.,failure_limit=3):
        self.factory=factory;self.warmup=warmup;self.retry_delay=retry_delay;self.failure_limit=failure_limit
        self.stop=threading.Event();self.condition=threading.Condition()
        self.latest=None;self.sequence=0;self.consumed=0;self.reconnects=0
        self.message='正在连接相机';self.camera_stamp=0.
        self.worker=threading.Thread(target=self._run,name='continuous-rgbd',daemon=True)
        self.worker.start()

    def _run(self):
        while not self.stop.is_set():
            camera=None
            try:
                camera=self.factory();valid=0;failures=0
                while not self.stop.is_set():
                    try:
                        bgr,depth,stamp=camera.read()
                    except Exception as error:
                        failures+=1
                        with self.condition:
                            self.message=f'相机恢复中：{error}';self.condition.notify_all()
                        if failures>=self.failure_limit:raise
                        # No sleep here: keep draining queued SDK frames promptly.
                        continue
                    failures=0;valid+=1
                    if valid<self.warmup:continue
                    with self.condition:
                        metadata=dict(camera.metadata)
                        if hasattr(camera,'native_bgr'):
                            metadata['_native_bgr']=camera.native_bgr
                            metadata['_native_depth']=camera.native_depth
                        self.latest=(bgr,depth,stamp,metadata)
                        self.camera_stamp=stamp;self.sequence+=1;self.message='相机持续采集中'
                        self.condition.notify_all()
            except Exception as error:
                with self.condition:
                    self.reconnects+=1;self.message=f'相机自动重连中：{error}';self.condition.notify_all()
            finally:
                if camera:
                    try:camera.close()
                    except Exception:pass
            self.stop.wait(self.retry_delay)

    def read(self,timeout=.8):
        with self.condition:
            ready=self.condition.wait_for(lambda:self.sequence>self.consumed or self.stop.is_set(),timeout)
            if not ready or self.latest is None or self.sequence<=self.consumed:
                raise TimeoutError(self.message)
            self.consumed=self.sequence
            return self.latest

    def status(self):
        with self.condition:
            return dict(camera_message=self.message,camera_reconnects=self.reconnects,
                        camera_age_s=round(time.time()-self.camera_stamp,3) if self.camera_stamp else None)

    def peek(self):
        """Read latest immutable RGB-D pair without consuming the render slot."""
        with self.condition:
            return self.latest

    def close(self):
        self.stop.set()
        with self.condition:self.condition.notify_all()
        self.worker.join(timeout=5)
