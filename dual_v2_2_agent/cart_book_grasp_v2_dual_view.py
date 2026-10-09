"""Uncalibrated wrist RGB evidence for GPT; never creates poses or depth.

Capture runs in a bounded child process so a blocked V4L2 read can be cancelled
with p/s. Every call opens a fresh stream; no cached frame or silent fallback.
"""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import cv2
import numpy as np

CONFIG_PATH=Path(__file__).with_suffix('.json')


def load_config(path=CONFIG_PATH):
    c=json.loads(Path(path).read_text())
    fields={'enabled','device','width','height','warmup_frames','capture_timeout_seconds',
            'max_pair_gap_seconds','max_joint_drift_deg','min_match_confidence','wrist_min_score'}
    if set(c)!=fields or type(c['enabled']) is not bool:raise ValueError('Invalid dual-view configuration')
    if not isinstance(c['device'],str) or not c['device'].startswith('/dev/'):
        raise ValueError('Wrist camera device must be an explicit /dev/ path')
    for key,lo,hi in [('width',160,4096),('height',120,2160),('warmup_frames',1,30)]:
        if type(c[key]) is not int or not lo<=c[key]<=hi:raise ValueError(f'Invalid {key}')
    for key,lo,hi in [('capture_timeout_seconds',.5,15),('max_pair_gap_seconds',.1,5),
                      ('max_joint_drift_deg',.01,2),('min_match_confidence',.5,1),('wrist_min_score',.1,1)]:
        if type(c[key]) not in (int,float) or not np.isfinite(c[key]) or not lo<=c[key]<=hi:
            raise ValueError(f'Invalid {key}')
    return c


def stationary_guard(robot,tolerance):
    def read():
        state=robot.get_current_state()
        joints=np.asarray(state['joint_state'] if state else None,float)
        if joints.size!=14 or not np.isfinite(joints).all():raise ValueError('双视角采集缺少有效关节状态')
        return joints.reshape(14).copy()
    baseline=read()
    def check():
        error=float(np.rad2deg(np.max(np.abs(read()-baseline))))
        if error>tolerance:raise ValueError(f'双视角观测期间机械臂姿态变化 {error:.2f}°，请重新采集')
    return check


def validate_frame(rgb,captured_at,head_captured_at,config):
    if not isinstance(rgb,np.ndarray) or rgb.dtype!=np.uint8 or rgb.shape!=(config['height'],config['width'],3):
        raise ValueError('手部相机图像尺寸或格式与配置不符')
    now=time.monotonic()
    if not all(np.isfinite(v) for v in (captured_at,head_captured_at)) or not head_captured_at<=captured_at<=now:
        raise ValueError('双视角图像时间戳无效')
    gap=captured_at-head_captured_at
    if gap>config['max_pair_gap_seconds'] or now-captured_at>config['capture_timeout_seconds']:
        raise ValueError(f'双视角采集间隔过长（{gap:.2f} 秒），请回车重新采集')
    return gap


def capture_wrist(config,head_captured_at,checkpoint=lambda:None):
    checkpoint()
    with tempfile.TemporaryDirectory(prefix='book-wrist-') as directory:
        output=Path(directory)/'frame.npz'
        args=[sys.executable,str(Path(__file__).resolve()),'--capture-worker',config['device'],
              str(config['width']),str(config['height']),str(config['warmup_frames']),str(output)]
        process=subprocess.Popen(args,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        deadline=time.monotonic()+config['capture_timeout_seconds']
        try:
            while process.poll() is None:
                checkpoint()
                if time.monotonic()>deadline:raise ValueError('手部相机读取超时；本轮未执行选书')
                time.sleep(.025)
            checkpoint()
            if process.returncode or not output.exists():
                raise ValueError('手部相机读取失败，请关闭占用相机的预览（如 guvcview），并检查连接和权限')
            with np.load(output,allow_pickle=False) as data:
                rgb=data['rgb'].copy();stamp=float(data['captured_at'])
            gap=validate_frame(rgb,stamp,head_captured_at,config)
            return dict(rgb=rgb,captured_at=stamp,pair_gap_seconds=gap)
        finally:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=.5)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=.5)


def wrist_candidates(book_geom,rgb,config,checkpoint=lambda:None):
    from cart_book_grasp_v2_instances import clean_masks,load_config as instance_config
    c=instance_config();c['min_score']=config['wrist_min_score']
    checkpoint()
    processor,to_numpy,image_cls=book_geom._load_sam3_detector()
    raw=book_geom._sam3_raw_books(processor,to_numpy,image_cls.fromarray(rgb),prompt='book')
    checkpoint()
    kept=clean_masks(raw,rgb.shape[:2],c)
    kept.sort(key=lambda item:item[2][0])
    result=[]
    for index,(score,mask,box) in enumerate(kept,1):
        yy,xx=np.nonzero(mask)
        result.append(dict(id=f'wrist_{index}',book_mask=mask,mask=mask,box=box,score=score,
                           uv=[float(np.median(xx)),float(np.median(yy))],recognition='wrist_book_proposal'))
    if not result:raise ValueError('手部视角没有书本候选，请调整场景后重试')
    return result


def _capture_worker(device,width,height,warmup,output):
    cap=cv2.VideoCapture(device,cv2.CAP_V4L2)
    try:
        if not cap.isOpened():raise ValueError('Cannot open wrist camera')
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,int(width));cap.set(cv2.CAP_PROP_FRAME_HEIGHT,int(height))
        for _ in range(int(warmup)):
            ok,frame=cap.read()
            if not ok or frame is None:raise ValueError('Cannot read wrist camera')
        stamp=time.monotonic()
        np.savez(output,rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB),captured_at=stamp)
    finally:cap.release()


if __name__=='__main__':
    if len(sys.argv)!=7 or sys.argv[1]!='--capture-worker':raise SystemExit('Internal capture worker only')
    _capture_worker(*sys.argv[2:])
