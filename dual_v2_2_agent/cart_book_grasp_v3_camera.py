"""Calibrated wrist RGB capture. Imports do not open a camera or robot."""
import hashlib
from pathlib import Path
import tarfile

import cv2
import numpy as np
import yaml

from cart_book_grasp_v2_dual_view import stationary_guard
from cart_book_grasp_v2_dual_view import load_config as _load_camera
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_camera_stream import PersistentWristCapture

ROOT = Path(__file__).resolve().parent
CAMERA_PATH = ROOT / 'cart_book_grasp_v3_camera.json'
CALIBRATION_PATH = ROOT / 'calibrationdata.tar.gz'


def camera_config():
    config = _load_camera(CAMERA_PATH)
    if not config['enabled']:
        raise ServoError('V3 wrist camera is disabled')
    return config


class WristCamera:
    def __init__(self, config=None, calibration_path=CALIBRATION_PATH):
        self._stream = None
        self.config = camera_config() if config is None else dict(config)
        with tarfile.open(calibration_path, 'r:gz') as archive:
            member = archive.getmember('ost.yaml')
            if not member.isfile() or member.size > 65536:
                raise ServoError('Invalid calibration YAML in archive')
            raw = archive.extractfile(member).read()
        self.calibration_hash = hashlib.sha256(raw).hexdigest()
        doc = yaml.safe_load(raw)
        self.size = (self.config['width'], self.config['height'])
        if (doc['image_width'], doc['image_height']) != self.size:
            raise ServoError('Wrist resolution differs from calibrated resolution')
        if doc['distortion_model'] != 'plumb_bob':
            raise ServoError('V3 expects a plumb_bob calibration')
        self.k = np.asarray(doc['camera_matrix']['data'], float).reshape(3, 3)
        self.d = np.asarray(doc['distortion_coefficients']['data'], float)
        if (not np.isfinite(self.k).all() or not np.isfinite(self.d).all()
                or self.d.shape != (5,) or min(self.k[0, 0], self.k[1, 1]) <= 0
                or not np.allclose(self.k[2], [0, 0, 1])
                or abs(self.k[0, 1])+abs(self.k[1, 0]) > 1e-9):
            raise ServoError('Invalid wrist intrinsics or distortion')
        # Keep K and the original dimensions: teaching and runtime share this
        # exact rectified coordinate system; never mix it with archive P pixels.
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.k, self.d, np.eye(3), self.k, self.size, cv2.CV_32FC1)

    def rectify(self, rgb):
        if rgb.shape != (self.size[1], self.size[0], 3) or rgb.dtype != np.uint8:
            raise ServoError('Unexpected wrist frame format/resolution')
        return cv2.remap(rgb, self.mapx, self.mapy, cv2.INTER_LINEAR)

    def capture(self, checkpoint=lambda: None):
        if self._stream is None:self._stream = PersistentWristCapture(self.config)
        result = self._stream.capture(checkpoint)
        result['raw_rgb'] = result['rgb']
        result['rgb'] = self.rectify(result['rgb'])
        return result

    def close(self):
        if self._stream is not None and self._stream.close():self._stream=None

    def __del__(self):
        try:self.close()
        except Exception:pass

    def check_reference(self, reference):
        if reference['calibration_sha256'] != self.calibration_hash:
            raise ServoError('Calibration changed since teaching; teach the target again')
        if reference['coordinate_system'] != 'undistorted_K':
            raise ServoError('Reference must use the V3 rectified image coordinates')
        if any(reference[k] != self.config[k] for k in ('device', 'width', 'height')):
            raise ServoError('Wrist camera/resolution changed since teaching')
