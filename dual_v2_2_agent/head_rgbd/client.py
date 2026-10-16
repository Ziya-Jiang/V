"""Independent subscribers; close never stops the shared camera owner."""
import time
import urllib.error
from urllib.parse import quote
import cv2
import numpy as np
from .lifecycle import URL, OPENER, ensure_service
from .protocol import decode


class Subscriber:
    def __init__(self, url=URL, autostart=True):
        self.url = url
        self.autostart = autostart
        self.token = ''
        self.closed = False
        self.next_recovery = 0.
        if autostart:
            ensure_service()

    def read(self, timeout=.9):
        if self.closed:
            raise RuntimeError('RGB-D subscriber is closed')
        try:
            with OPENER.open(self.url + '/frame?after=' + quote(self.token), timeout=timeout) as response:
                payload = response.read()
        except urllib.error.HTTPError as error:
            # Device reconnect: service remains alive, no restart or USB fallback.
            if error.code == 503:
                raise TimeoutError('共享相机等待新帧，自动恢复中') from error
            raise
        except OSError:
            if self.autostart and time.monotonic() >= self.next_recovery:
                self.next_recovery = time.monotonic() + 2
                ensure_service()
            raise
        bgr, depth, header = decode(payload)
        if header['token'] == self.token:
            raise ValueError('Repeated shared RGB-D frame')
        self.token = header['token']
        return bgr, depth, header

    def close(self):
        self.closed = True


class SharedCamera:
    """Adapter for the AR latest-frame thread (BGR + float32 millimetres)."""
    def __init__(self):
        self.subscriber = Subscriber()
        self.metadata = None

    def read(self):
        bgr, depth, header = self.subscriber.read()
        self.metadata = header['metadata']
        return bgr, depth, header['captured_unix_s']

    def close(self):
        self.subscriber.close()


class CalibratedCamera:
    """RobotRAM capture_image contract; all geometry belongs to this subscriber.

    Perception may adjust its own R/T without changing the service or AR camera.
    Each capture validates the published calibration against the startup baseline.
    """
    def __init__(self, calibration, warmup_frames=5, subscriber=None, *, prefer_native=False):
        self.subscriber = subscriber if subscriber is not None else Subscriber()
        self.baseline = {key: np.array(calibration[key], copy=True)
                         for key in ('K', 'D', 'R', 'T')}
        for key, value in self.baseline.items():
            setattr(self, key, value.copy())
        self.w, self.h = int(calibration['w']), int(calibration['h'])
        self.baseline_size=(self.w,self.h)
        self.prefer_native=prefer_native
        self.camparas_file = calibration['path']
        self.use_dual_color = False
        self._orbbec_depth_enabled = True
        self._shared_source = True
        self.max_depth = 5000
        self.type = 'external'
        self.undistort = False
        self.rgb_image = np.zeros((self.h, self.w, 3), np.uint8)
        self.rgb_right_image = self.rgb_image.copy()
        self.depth_map = np.zeros((self.h, self.w), np.float32)
        self.metadata = None
        self._orbbec_align_mode = 'shared D2C'
        try:
            deadline = time.monotonic() + 20
            valid = 0
            last_error = None
            while valid < max(1, warmup_frames) and time.monotonic() < deadline:
                try:
                    _, depth = self.capture_image()
                    if np.any(depth > 0):
                        valid += 1
                except OSError as error:
                    last_error = error
                    time.sleep(.05)
            if valid < max(1, warmup_frames):
                raise RuntimeError(f'共享头部 RGB-D 尚未就绪；请查看 head_rgbd status：{last_error}')
        except Exception:
            self.close()
            raise

    def _validate(self, metadata):
        if metadata.get('serial') != 'CV2TC5100060':
            raise ValueError('Shared head camera serial does not match this robot')
        if (metadata['width'], metadata['height']) != self.baseline_size:
            raise ValueError('Shared head camera resolution differs from grasp calibration')
        transform = np.asarray(metadata['T_dummy_world_camera_mm'], float)
        values = dict(K=metadata['K'], D=metadata['D'], R=transform[:3, :3], T=transform[:3, 3])
        for key, expected in self.baseline.items():
            actual = np.asarray(values[key]).reshape(-1)
            if actual.shape != expected.reshape(-1).shape or not np.allclose(actual, expected.reshape(-1), atol=1e-8, rtol=0):
                raise ValueError(f'Shared head camera {key} differs from grasp calibration')

    def capture_image(self):
        bgr, depth, header = self.subscriber.read()
        self._validate(header['metadata'])
        self.metadata = header['metadata']
        self._orbbec_align_mode = self.metadata['depth_alignment']
        self.captured_unix_s = header['captured_unix_s']
        self.frame_token = header['token']
        if self.prefer_native:
            if 'native_bgr' not in header or not self.metadata.get('native_is_maximum'):
                raise ValueError('共享头部服务未提供最高原始分辨率；请更新并重启 head_rgbd 服务')
            bgr,depth=header['native_bgr'],header['native_depth']
            from .native import scaled_intrinsics
            self.w,self.h=bgr.shape[1],bgr.shape[0]
            self.K=scaled_intrinsics(self.baseline['K'],self.baseline_size,(self.w,self.h))
        self.rgb_image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self.rgb_right_image = self.rgb_image.copy()
        self.depth_map = depth
        return self.rgb_image, self.depth_map

    def close(self):
        self.subscriber.close()

    @property
    def c2w(self):
        value = np.eye(4)
        value[:3, :3] = self.R
        value[:3, 3] = self.T.reshape(3)
        return value

    @property
    def w2c(self):
        return np.linalg.inv(self.c2w)

    width = property(lambda self: self.w)
    height = property(lambda self: self.h)
    fx = property(lambda self: self.K[0, 0])
    fy = property(lambda self: self.K[1, 1])
    cx = property(lambda self: self.K[0, 2])
    cy = property(lambda self: self.K[1, 2])
    left_image = property(lambda self: self.rgb_image, lambda self, value: setattr(self, 'rgb_image', value))
    right_image = property(lambda self: self.rgb_right_image, lambda self, value: setattr(self, 'rgb_right_image', value))
