#!/usr/bin/env python3
"""Cart handle ArUco tools and right-handle coarse + wrist visual alignment.

align previews by default; --execute enables Y/tool-roll motion only.
Optional --approach uses head RGB-D for X endpoint. No gripper closure.
"""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np

DICT_NAME = 'DICT_4X4_50'


def dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def check_id(marker_id):
    if isinstance(marker_id, bool) or not isinstance(marker_id, int) or not 0 <= marker_id < 50:
        raise ValueError('Marker ID must be an integer in 0..49')


def make_marker(out, marker_id=0, marker_mm=20., white_margin_mm=4.):
    check_id(marker_id)
    if not np.isfinite(marker_mm) or not 10 <= marker_mm <= 150:
        raise ValueError('Black outer square size must be 10..150 mm')
    if not np.isfinite(white_margin_mm) or not marker_mm/6 <= white_margin_mm <= marker_mm:
        raise ValueError('Keep at least one marker cell of white margin')
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    bits=cv2.aruco.generateImageMarker(dictionary(),marker_id,6)
    code=cv2.aruco.generateImageMarker(dictionary(),marker_id,600)
    border=int(round(600*white_margin_mm/marker_mm))
    raster=cv2.copyMakeBorder(code,border,border,border,border,cv2.BORDER_CONSTANT,value=255)
    # SVG has physical dimensions; the PNG is just a preview.
    margin=white_margin_mm;side=marker_mm+2*margin;cell=marker_mm/6
    rectangles=''.join(f'<rect x="{margin+x*cell:.9f}" y="{margin+y*cell:.9f}" width="{cell:.9f}" height="{cell:.9f}"/>'
                       for y in range(6) for x in range(6) if bits[y,x]==0)
    svg=(f'<svg xmlns="http://www.w3.org/2000/svg" width="{side:g}mm" height="{side:g}mm" viewBox="0 0 {side:g} {side:g}">'
         f'<rect width="{side:g}" height="{side:g}" fill="white"/><g fill="black" shape-rendering="crispEdges">{rectangles}</g></svg>')
    (out/'marker.svg').write_text(svg)
    cv2.imwrite(str(out/'marker_preview.png'),raster)
    meta=dict(dictionary=DICT_NAME,marker_id=marker_id,black_square_mm=marker_mm,
              white_margin_mm=margin,print_page_mm=side,
              print_instruction='Print SVG at 100%, measure BLACK outer square. Keep white margin. Mount flat and rigid; do not wrap around a round handle.')
    (out/'marker.json').write_text(json.dumps(meta,indent=2))
    return meta


class HandleMarker:
    def __init__(self, marker_id=0, marker_mm=20.):
        check_id(marker_id)
        if not np.isfinite(marker_mm) or marker_mm <= 0: raise ValueError('Invalid physical marker size')
        self.marker_id=marker_id;self.marker_mm=float(marker_mm)
        params=cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector=cv2.aruco.ArucoDetector(dictionary(),params)

    def detect(self,bgr,stamp=None,k=None):
        began=time.perf_counter()
        corners,ids,rejected=self.detector.detectMarkers(bgr)
        found=[] if ids is None else [i for i,value in enumerate(ids.ravel()) if value==self.marker_id]
        result=dict(marker_id=self.marker_id,dictionary=DICT_NAME,timestamp=stamp,
                    detected_ids=[] if ids is None else [int(value) for value in ids.ravel()],
                    rejected_candidate_count=len(rejected),
                    tracking_valid=False,control_ready=False,corners_px=None,center_px=None,
                    reason='target_marker_missing' if not found else 'duplicate_target_id')
        if len(found)==1:
            q=np.asarray(corners[found[0]],np.float32).reshape(4,2)
            h,w=bgr.shape[:2];edges=np.linalg.norm(q-np.roll(q,-1,axis=0),axis=1)
            border=min(q[:,0].min(),q[:,1].min(),w-1-q[:,0].max(),h-1-q[:,1].max())
            if border<3:result['reason']='marker_touches_image_border'
            elif min(edges)<20 or not cv2.isContourConvex(q):result['reason']='marker_too_small_or_degenerate'
            else:
                # Projective center, not the arithmetic corner mean under tilt.
                H=cv2.getPerspectiveTransform(np.float32([[0,0],[1,0],[1,1],[0,1]]),q)
                center=cv2.perspectiveTransform(np.float32([[[.5,.5]]]),H)[0,0]
                result.update(tracking_valid=True,reason='decoded_id_and_four_corners',
                              corners_px=q.tolist(),center_px=center.tolist(),
                              area_px2=float(abs(cv2.contourArea(q))),min_edge_px=float(min(edges)),
                              s_px=q.reshape(-1).tolist(),image_size=[w,h])
                if k is not None:
                    s=self.marker_mm/2
                    object_points=np.float64([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]])
                    ok,rvec,tvec=cv2.solvePnP(object_points,q.astype(float),np.asarray(k,float),np.zeros(5),flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok:
                        projected,_=cv2.projectPoints(object_points,rvec,tvec,k,np.zeros(5))
                        rms=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-q)**2,axis=1))))
                        result.update(marker_camera_translation_mm=tvec.ravel().tolist(),
                                      marker_camera_rotation_vector=rvec.ravel().tolist(),reprojection_rms_px=rms,
                                      pose_note='Diagnostic planar PnP estimate only; not a robot/base pose or verified grasp endpoint.')
        result['processing_ms']=(time.perf_counter()-began)*1000
        return result


def annotate(bgr, result):
    out=bgr.copy()
    if result['tracking_valid']:
        q=np.float32(result['corners_px']).reshape(1,4,2)
        cv2.aruco.drawDetectedMarkers(out,[q],np.int32([[result['marker_id']]]))
        cv2.circle(out,tuple(np.int32(result['center_px'])),5,(0,0,255),2)
    cv2.putText(out,result['reason'],(15,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,180,255),2)
    return out


def select_reference_sample(samples):
    """Select an actual synchronized frame, not a fabricated average target."""
    from scipy.spatial.transform import Rotation
    if len(samples)<5:raise ValueError('Need at least five stationary observations')
    pixels=np.asarray([s['marker']['s_px'] for s in samples],float)
    poses=np.asarray([s['pose']['q'] for s in samples],float)
    angles=np.asarray([s['pose']['angles'] for s in samples],float)
    if (pixels.shape!=(len(samples),8) or poses.shape!=(len(samples),3) or
            angles.shape!=(len(samples),3) or not np.isfinite(pixels).all() or
            not np.isfinite(poses).all() or not np.isfinite(angles).all()):
        raise ValueError('Invalid reference samples')
    ids={s['marker']['marker_id'] for s in samples}
    if len(ids)!=1 or not all(s['marker']['tracking_valid'] for s in samples):
        raise ValueError('Reference identity not stable')
    if np.max(np.std(pixels,axis=0))>1.5:raise ValueError('Marker image not stable for teaching')
    if np.max(np.linalg.norm(poses-poses[0],axis=1))>.5:raise ValueError('TCP moved during teaching')
    rotations=Rotation.from_euler('xyz',angles,degrees=True)
    if np.max(np.degrees((rotations[0].inv()*rotations).magnitude()))>.25:
        raise ValueError('TCP rotated during teaching')
    return int(np.argmin(np.linalg.norm(pixels-np.median(pixels,axis=0),axis=1)))


def teach(a):
    """Read images/feedback only: never issue an arm or gripper command."""
    from cart_book_grasp_v3_camera import WristCamera
    from cart_book_grasp_v3_2_runtime import StreamTransport,capture_paired_frame
    from servor import robot_connection
    camera=WristCamera();samples=[]
    c=dict(max_orientation_error_deg=1.)
    settings=dict(feedback_max_age_seconds=.25,frame_max_age_seconds=.25,
        max_pose_pair_gap_seconds=.1,max_pose_pair_uncertainty_mm=3.,
        max_pose_pair_uncertainty_deg=1.,pair_recovery_timeout_seconds=2.)
    detector=HandleMarker(a.marker_id,a.marker_mm)
    try:
        with robot_connection() as (robot,base):
            transport=StreamTransport(robot,base,c,settings)
            def stationary():
                transport.read()
                velocity=np.asarray(transport.last_state['joint_velocity'],float)
                if not np.isfinite(velocity).all() or max(abs(velocity))>.01:
                    raise ValueError('Robot must stay still while teaching')
                mode=transport.last_state['right_gripper']
                if getattr(mode,'value',mode)!=0:
                    raise ValueError('Right gripper must be commanded open for a pre-closure reference')
            deadline=time.monotonic()+15.;last_stamp=-np.inf
            while len(samples)<8:
                frame,paired,_,_,_=capture_paired_frame(transport,camera,stationary,lambda event:None,last_stamp,
                    clock=time.monotonic,sleep=time.sleep,total_deadline=deadline)
                last_stamp=frame['captured_at'];bgr=cv2.cvtColor(frame['rgb'],cv2.COLOR_RGB2BGR)
                marker=detector.detect(bgr,last_stamp,camera.k)
                if not marker['tracking_valid']:raise ValueError('Teaching requires a fully visible unique physical marker')
                samples.append(dict(marker=marker,pose=paired,image=bgr))
            sample=samples[select_reference_sample(samples)]
            reference=dict(schema_version=1,target_source='real_preclosure_demonstration',
                robot_frame='existing_planner_base',marker_dictionary=DICT_NAME,
                marker_id=a.marker_id,black_square_mm=a.marker_mm,
                camera=dict(device=camera.config['device'],image_size=list(camera.size),
                    calibration_sha256=camera.calibration_hash,coordinate_system='undistorted_K',K=camera.k.tolist()),
                target_marker=sample['marker'],tcp_xyz_mm=sample['pose']['q'].tolist(),
                tcp_euler_xyz_deg=sample['pose']['angles'].tolist(),
                pose_pair_gap_seconds=sample['pose']['pair_gap_seconds'],
                marker_to_grasp_binding='User placed open gripper at intended closing pose; keep marker mounting rigid.',
                hardware_execution_validated=False,robot_commands_sent=0)
            a.out.mkdir(parents=True,exist_ok=False)
            (a.out/'reference.json').write_text(json.dumps(reference,ensure_ascii=False,indent=2,allow_nan=False))
            cv2.imwrite(str(a.out/'reference.png'),sample['image'])
            cv2.imwrite(str(a.out/'reference_annotated.png'),annotate(sample['image'],sample['marker']))
            print(json.dumps(dict(event='reference_saved',out=str(a.out),robot_commands_sent=0)))
    finally:camera.close()


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    commands=p.add_subparsers(dest='command',required=True)
    make=commands.add_parser('marker',help='生成可打印的真实 ArUco 标记')
    make.add_argument('--out',type=Path,required=True)
    make.add_argument('--white-margin-mm',type=float,default=4.)
    observe=commands.add_parser('observe',help='只读检测腕部图像，不控制机器人')
    observe.add_argument('--image',type=Path)
    observe.add_argument('--seconds',type=float,default=30.)
    observe.add_argument('--out',type=Path,required=True)
    record=commands.add_parser('teach',help='只读记录真实闭合前终点；手动摆好右手并保持夹爪张开')
    record.add_argument('--out',type=Path,required=True)
    for cmd in (make,observe,record):
        cmd.add_argument('--marker-id',type=int,default=0)
        cmd.add_argument('--marker-mm',type=float,default=20.,help='标记黑色外框的实测边长，不含白边')
    align=commands.add_parser('align',help='右车把 ID 0：头部粗对准 + 腕部伺服；--approach 启用头部深度前伸，不夹紧')
    align.add_argument('--execute',action='store_true',help='执行实机对准；默认只生成粗对准方案')
    align.add_argument('--check-config',action='store_true',help='只检查配置，不连接设备')
    align.add_argument('--config',type=Path,default=Path(__file__).with_name('cart_grasp_alignment.json'))
    align.add_argument('--out',type=Path)
    mode=align.add_mutually_exclusive_group()
    mode.add_argument('--approach',action='store_true',help='用头部标记深度确定 X 终点，边前伸边视觉纠偏；不夹紧')
    mode.add_argument('--wrist-only',action='store_true',help='仅腕部 ID 0 视觉对准，跳过头部和粗移动；X/Z 固定')
    align.add_argument('--approach-config',type=Path,default=Path(__file__).with_name('cart_grasp_approach.json'))
    a=p.parse_args(argv)
    if a.command=='align':
        from cart_grasp_alignment import load_config,run
        if a.check_config:
            config=load_config(a.config)
            if a.approach:
                from cart_grasp_depth import load_approach
                config=dict(alignment=config,approach=load_approach(a.approach_config))
            print(json.dumps(config,indent=2));return 0
        return run(a)
    if a.command=='marker':print(json.dumps(make_marker(a.out,a.marker_id,a.marker_mm,a.white_margin_mm),ensure_ascii=False));return 0
    if a.command=='teach':teach(a);return 0
    if not np.isfinite(a.seconds) or not 0<a.seconds<=600:p.error('--seconds must be 0..600')
    a.out.mkdir(parents=True,exist_ok=False)
    detector=HandleMarker(a.marker_id,a.marker_mm);camera=None
    try:
        if not a.image:
            from cart_book_grasp_v3_camera import WristCamera
            camera=WristCamera()
        end=time.monotonic()+a.seconds;index=0
        with (a.out/'observations.jsonl').open('w') as log:
            while time.monotonic()<end:
                if camera:
                    frame=camera.capture();bgr=cv2.cvtColor(frame['rgb'],cv2.COLOR_RGB2BGR)
                    stamp=frame['captured_at'];k=camera.k
                else:
                    bgr=cv2.imread(str(a.image));stamp=None;k=None
                    if bgr is None:raise ValueError('Cannot read image')
                result=detector.detect(bgr,stamp,k)
                log.write(json.dumps(result,allow_nan=False)+'\n');log.flush()
                if index%200==0:cv2.imwrite(str(a.out/f'observation_{index:05d}.png'),annotate(bgr,result))
                index+=1
                if not camera:print(json.dumps(result,ensure_ascii=False));break
        print(json.dumps(dict(event='observation_complete',frames=index,robot_commands_sent=0,out=str(a.out))))
    finally:
        if camera:camera.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
