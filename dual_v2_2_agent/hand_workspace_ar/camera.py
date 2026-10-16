"""Camera-only RGB-D owner; no imports of motion/control pipelines."""
import ast
import hashlib
import json
import logging
import sys
import time
from pathlib import Path
import cv2
import numpy as np
from .core import ROOT

CALIBRATION=Path('/mnt/ssd/user/wx/vlm_demo/calibration/session_seed/base_to_camera/results/machine-readable results.json')
SERIAL='CV2TC5100060'

class Camera:
    def __init__(self):
        sys.path.insert(0,str(ROOT/'RobotRAM'))
        from controllor.extern_camera import OrbbecCamera
        source=ROOT/'dual_v2_2_agent_pcl.py'
        cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='OrbbecRgbdCamera')
        namespace={'OrbbecCamera':OrbbecCamera,'logging':logging}
        exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),namespace)
        c=json.loads(CALIBRATION.read_text())['left']
        self.camera=namespace['OrbbecRgbdCamera'](is_warmup=False,preset_name='Default')
        self.last_stamp=None;self.last_depth_stamp=None;self.offset=None
        cam=self.camera
        for name in ['K','D','R','T']:setattr(cam,name,np.array(c[name],float))
        cam.w,cam.h=int(c['w']),int(c['h'])
        from head_rgbd.native import highest_color_profile
        def native_color(sensor_type,OBSensorType,OBFormat):
            profiles=cam.pipeline.get_stream_profile_list(sensor_type)
            profile=highest_color_profile(profiles,(OBFormat.MJPG,OBFormat.RGB,OBFormat.BGR))
            self.native_size=(profile.get_width(),profile.get_height())
            logging.warning("Head native profile: %sx%s %sfps %s",*self.native_size,profile.get_fps(),profile.get_format())
            return profile
        cam._orbbec_get_color_profile=native_color
        original_profile=cam._orbbec_get_video_profile
        def depth_profile(*args,**kwargs):
            profile=original_profile(*args,**kwargs)
            logging.warning('Head depth profile: %sx%s %sfps %s',profile.get_width(),profile.get_height(),profile.get_fps(),profile.get_format())
            return profile
        cam._orbbec_get_video_profile=depth_profile
        try:
            cam._initialize_camera()
            actual=cam._ob_device.get_device_info().get_serial_number()
            if actual!=SERIAL:raise ValueError(f'Wrong head camera: {actual}')
            self.metadata=dict(K=cam.K.tolist(),D=cam.D.tolist(),T_dummy_world_camera_mm=cam.c2w.tolist(),
                width=cam.w,height=cam.h,serial=actual,calibration_path=str(CALIBRATION),
                calibration_sha256=hashlib.sha256(CALIBRATION.read_bytes()).hexdigest(),depth_alignment=str(cam._orbbec_align_mode),native_width=self.native_size[0],native_height=self.native_size[1],native_is_maximum=True)
        except Exception:
            cam.close();raise

    def read(self):
        cam=self.camera;deadline=time.monotonic()+3
        while True:
            frames=cam.pipeline.wait_for_frames(500)
            color=frames.get_color_frame() if frames is not None else None
            depth=frames.get_depth_frame() if frames is not None else None
            if color is not None and depth is not None:break
            if time.monotonic()>=deadline:raise ValueError('RGB-D timeout: no complete aligned pair within 3 seconds')
        stamp=color.get_timestamp_us()/1e6;dstamp=depth.get_timestamp_us()/1e6
        now=time.time()
        if abs(stamp-dstamp)>.05:raise ValueError('RGB/depth timestamp skew > 50ms')
        if self.last_stamp is not None and (stamp<=self.last_stamp or dstamp<=self.last_depth_stamp):
            raise ValueError('Repeated or regressing camera frame')
        self.last_stamp=stamp;self.last_depth_stamp=dstamp
        # Minimum observed host-device offset detects accumulating frame backlog.
        self.offset=min(self.offset if self.offset is not None else now-stamp,now-stamp)
        captured=stamp+self.offset
        if now-captured>.3:raise ValueError('Camera frame backlog > 300ms')
        rgb=cam._frame_to_rgb(color)
        d=np.frombuffer(depth.get_data(),np.uint16).reshape(depth.get_height(),depth.get_width()).astype(np.float32)*depth.get_depth_scale()
        d[~np.isfinite(d)|(d<=0)|(d>5000)]=0
        if (rgb.shape[1],rgb.shape[0])!=self.native_size or d.shape!=rgb.shape[:2]:
            raise ValueError('Maximum-resolution RGB and D2C depth dimensions do not match')
        self.native_bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
        self.native_depth=d.copy()
        bgr=cv2.cvtColor(cv2.resize(rgb,(cam.w,cam.h)),cv2.COLOR_RGB2BGR)
        d=cv2.resize(d,(cam.w,cam.h),interpolation=cv2.INTER_NEAREST)
        return bgr,d,captured

    def close(self):self.camera.close()
