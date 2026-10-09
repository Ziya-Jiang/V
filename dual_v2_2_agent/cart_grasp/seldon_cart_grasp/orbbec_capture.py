"""One head-camera owner feeds the existing cockpit and an atomic RGB-D spool.

Called as a camera child by shared_cockpit. No base/arm control imports here.
"""
import json,os,time
from pathlib import Path
import cv2
import numpy as np
import yaml
from .camera_model import image_intrinsics,confirmed_upper_camera
from .rgbd_alignment import align_depth,intrinsic_matrix,distortion_vector
ROOT=Path(__file__).resolve().parents[1]

def run_camera(source,config,frames,shutdown):
    if source!='orbbec':
        from seldon_cockpit.camera import run_camera as legacy
        return legacy(source,config,frames,shutdown)
    from seldon_cockpit.backend import offer
    import pyorbbecsdk as ob
    cv2.setNumThreads(1);runtime=ROOT/'runtime/rgbd';runtime.mkdir(parents=True,exist_ok=True);pipeline=None
    def status(state,**kw):
        temp=runtime/'status.tmp';temp.write_text(json.dumps({'state':state,'updated_unix_s':time.time(),'hardware_motion_commands_sent':False,**kw},indent=2));temp.replace(runtime/'status.json')
    try:
        c=yaml.safe_load((ROOT/'config/camera_user_20260921.yaml').read_text());w,h=c['image_width'],c['image_height'];k,d=image_intrinsics(c,w,h);upper=confirmed_upper_camera(c);serial=config.get('orbbec_serial')
        if serial!='CV2TC5100060':raise ValueError('Shared calibrated head-camera serial mismatch')
        ctx=ob.Context();device=ctx.query_devices().get_device_by_serial_number(serial);color_profile=device.get_sensor(ob.OBSensorType.COLOR_SENSOR).get_stream_profile_list().get_video_stream_profile(w,h,ob.OBFormat.MJPG,5);depth_profile=device.get_sensor(ob.OBSensorType.DEPTH_SENSOR).get_stream_profile_list().get_video_stream_profile(w,h,ob.OBFormat.Y16,5)
        e=depth_profile.get_extrinsic_to(color_profile);extrinsic=np.eye(4);extrinsic[:3,:3]=np.asarray(e.rot).reshape(3,3);extrinsic[:3,3]=np.asarray(e.transform).reshape(3)*.001;dk=intrinsic_matrix(depth_profile);dd=distortion_vector(depth_profile)
        pipeline=ob.Pipeline(device);cfg=ob.Config();cfg.enable_stream(color_profile);cfg.enable_stream(depth_profile);pipeline.enable_frame_sync();pipeline.start(cfg);last_color=-1;last_depth=-1;last_write=0.;last_frame=time.monotonic();sequence=0;rejects=0
        while not shutdown.is_set():
            packet=pipeline.wait_for_frames(200)
            if packet is None:
                if time.monotonic()-last_frame>3:status('NO_FRAMES');offer(frames,{'error':'共享 RGB-D 相机连续3秒无帧'})
                continue
            color=packet.get_color_frame();depth=packet.get_depth_frame()
            if color is None or depth is None:continue
            ct,dt=color.get_timestamp(),depth.get_timestamp()
            if ct<=last_color or dt<=last_depth or abs(ct-dt)>40:rejects+=1;continue
            last_color,last_depth=ct,dt;last_frame=time.monotonic();host_stamp=time.time();bgr=cv2.imdecode(np.frombuffer(color.get_data(),np.uint8),cv2.IMREAD_COLOR)
            if bgr is None or bgr.shape!=(h,w,3):raise ValueError('Color profile does not match calibrated dimensions')
            ok,jpeg=cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,80])
            if ok:offer(frames,{'time':last_frame,'jpeg':jpeg.tobytes()})
            if last_frame-last_write<.5:continue
            last_write=last_frame;raw=np.frombuffer(depth.get_data(),np.uint16).reshape(depth.get_height(),depth.get_width()).copy();scale=float(depth.get_depth_scale())*.001;aligned=align_depth(raw,scale,dk,dd,k,d,extrinsic,w,h);sequence+=1
            tmp=runtime/'latest.tmp'
            with tmp.open('wb') as stream:np.savez_compressed(stream,bgr=bgr,depth=aligned,k=k,d=d,depth_scale=1.,stamp_s=host_stamp,monotonic_s=last_frame,sequence=sequence,color_hardware_ms=ct,depth_hardware_ms=dt,upper_T_camera=upper,aligned_depth=np.array(True),serial=np.array(serial),boot_id=np.array(Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
            tmp.replace(runtime/'latest.npz');status('STREAMING',serial=serial,sequence=sequence,resolution=[w,h],device_fps=5,spool_max_hz=2,paired_skew_ms=abs(ct-dt),depth_scale_native_m=scale,valid_depth_fraction=float(np.mean(aligned>0)),rejected_pairs=rejects,registration='CPU SDK depth geometry to user-calibrated raw color K/D; no hole fill',alignment_physical_validation=False)
    except Exception as error:
        status('FAILED',error=str(error));offer(frames,{'error':'共享 RGB-D 相机不可用: '+str(error)})
    finally:
        if pipeline is not None:
            try:pipeline.stop()
            except Exception:pass
