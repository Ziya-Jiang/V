"""Bounded, camera-only maximum-resolution head/wrist capture.

Run only after stopping this project's AR subscriber and head_rgbd owner.
Never imports RobotArm or sends robot commands. Restoring the shared owner is
handled by the caller's finally block. Host receipt timing is not HW sync.
"""
import json,time,threading
from pathlib import Path
import cv2,numpy as np
import pyorbbecsdk as ob
from cart_book_grasp_v3_camera import WristCamera
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'RobotRAM/output/v3_head_max_resolution_20261010'

def state():
    try:return json.loads((ROOT/'RobotRAM/output/hand_workspace_ar_live/state.json').read_text())
    except (OSError,ValueError):return None

def main():
    OUT.mkdir(exist_ok=True)
    context=ob.Context();devices=context.query_devices();device=None
    for i in range(len(devices)):
        candidate=devices.get_device_by_index(i)
        if candidate.get_device_info().get_serial_number()=='CV2TC5100060':device=candidate;break
    if device is None:raise RuntimeError('Expected head camera not found')
    info=device.get_device_info();pipeline=ob.Pipeline(device)
    profiles=pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
    rows=[];usable=[]
    for i in range(len(profiles)):
        p=profiles[i].as_video_stream_profile()
        row=dict(width=p.get_width(),height=p.get_height(),fps=p.get_fps(),format=str(p.get_format()))
        rows.append(row)
        if p.get_format() in (ob.OBFormat.MJPG,ob.OBFormat.RGB,ob.OBFormat.BGR):usable.append((p,row))
    listing=dict(name=info.get_name(),serial=info.get_serial_number(),profiles=rows)
    (OUT/'profiles.json').write_text(json.dumps(listing,indent=2))
    maximum=max(r['width']*r['height'] for r in rows)
    top=[(p,r) for p,r in usable if r['width']*r['height']==maximum]
    if not top:raise RuntimeError('Maximum advertised resolution has no supported RGB/MJPG decoder')
    top.sort(key=lambda pr:(pr[1]['fps']<=30,pr[1]['fps'] if pr[1]['fps']<=30 else -pr[1]['fps'],pr[0].get_format()==ob.OBFormat.MJPG),reverse=True)
    chosen,row=top[0]
    intr=chosen.get_intrinsic();dist=chosen.get_distortion()
    metadata=dict(**listing,selected_profile=row,native_K=[[intr.fx,0,intr.cx],[0,intr.fy,intr.cy],[0,0,1]],
        native_distortion={k:float(getattr(dist,k)) for k in ('k1','k2','p1','p2','k3')},
        camera_opened=True,robot_connected=False,motion_sent=False,
        note='RGB-only native maximum profile for offline matching; factory native K, no robot-coordinate use. Pair gap is host receipt estimate, not hardware synchronization.',state_before=state(),pairs=[])
    (OUT/'capture.json').write_text(json.dumps(metadata,indent=2))
    print('HEAD MAX PROFILE',row,flush=True)
    config=ob.Config();config.enable_stream(chosen)
    stop=threading.Event();lock=threading.Lock();latest=[None];errors=[]
    wrist_model=WristCamera()
    def wrist_reader():
        cap=cv2.VideoCapture(wrist_model.config['device'],cv2.CAP_V4L2)
        try:
            if not cap.isOpened():raise RuntimeError('Wrist camera busy/unavailable')
            cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720);cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
            count=0
            while not stop.is_set():
                ok,bgr=cap.read();stamp=time.monotonic();count+=1
                if not ok:raise RuntimeError('Wrist frame failed')
                if count<15:continue
                if bgr.shape!=(720,1280,3):raise RuntimeError('Wrist resolution changed')
                with lock:latest[0]=(bgr.copy(),stamp)
        except Exception as error:errors.append(repr(error))
        finally:cap.release()
    worker=threading.Thread(target=wrist_reader,daemon=True)
    started=False
    try:
        pipeline.start(config);started=True;worker.start()
        deadline=time.monotonic()+25;valid=0;last_device=-1;last_save=-1
        while time.monotonic()<deadline and len(metadata['pairs'])<3:
            if errors:raise RuntimeError(errors[0])
            frames=pipeline.wait_for_frames(1000)
            if frames is None:continue
            f=frames.get_color_frame()
            if f is None:continue
            head_stamp=time.monotonic();device_stamp=f.get_timestamp_us()
            if device_stamp<=last_device:continue
            last_device=device_stamp;valid+=1
            if valid<20 or head_stamp-last_save<.6:continue
            with lock:wrist=latest[0]
            if wrist is None or abs(head_stamp-wrist[1])>.1:continue
            data=np.frombuffer(f.get_data(),np.uint8)
            if f.get_format()==ob.OBFormat.RGB:bgr=cv2.cvtColor(data.reshape(f.get_height(),f.get_width(),3),cv2.COLOR_RGB2BGR)
            elif f.get_format()==ob.OBFormat.BGR:bgr=data.reshape(f.get_height(),f.get_width(),3).copy()
            else:bgr=cv2.imdecode(data,cv2.IMREAD_COLOR)
            if bgr is None or bgr.shape[:2]!=(row['height'],row['width']):raise RuntimeError('Native head frame size mismatch')
            n=len(metadata['pairs']);name=f'pair_{n}'
            wrgb=cv2.cvtColor(wrist[0],cv2.COLOR_BGR2RGB)
            for suffix,img in [('head_native',bgr),('head_848',cv2.resize(bgr,(848,530),interpolation=cv2.INTER_AREA)),('wrist_raw',wrist[0]),('wrist_undistorted',cv2.cvtColor(wrist_model.rectify(wrgb),cv2.COLOR_RGB2BGR))]:
                if not cv2.imwrite(str(OUT/f'{name}_{suffix}.png'),img):raise RuntimeError('Image write failed')
            metadata['pairs'].append(dict(name=name,head_host_monotonic=head_stamp,wrist_host_monotonic=wrist[1],host_gap_s=abs(head_stamp-wrist[1]),head_device_timestamp_us=device_stamp,captured_unix_s=time.time(),wrist_coordinate_system='undistorted_K',wrist_calibration_sha256=wrist_model.calibration_hash))
            last_save=head_stamp
            print('CAPTURED',name,'host_gap',abs(head_stamp-wrist[1]),flush=True)
        if len(metadata['pairs'])<3:raise RuntimeError('Insufficient fresh pairs within 25 seconds')
    finally:
        stop.set()
        if started:pipeline.stop()
        if worker.ident:worker.join(3)
        metadata['state_after']=state()
        (OUT/'capture.json').write_text(json.dumps(metadata,indent=2))
    print(OUT,flush=True)

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUT)
    OUT=parser.parse_args().output.resolve()
    if (OUT/'capture.json').exists():raise SystemExit('Capture exists; use a new --output directory')
    main()
