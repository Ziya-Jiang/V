"""Offline actual-frame replay, known synthetic motion, and blank-image negative."""
from pathlib import Path
import hashlib,json,subprocess,uuid
import cv2
import numpy as np
from cart_book_grasp_v3_cotracker_points import CoTrackerModel,PointLock,draw_points
from cart_book_grasp_v3_distance_servo import load_config
from cart_book_grasp_v3_servo import ServoError


def main():
    root=Path(__file__).resolve().parent
    output=root/'RobotRAM/output'/('v3_cotracker_validation_'+uuid.uuid4().hex);output.mkdir()
    c=load_config();model=CoTrackerModel(c)
    def rgb(path):return cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB)
    capture=root/'RobotRAM/output/v3_head_max_observation_20261010'
    frames=[rgb(capture/f'pair_{i}_wrist_undistorted.png') for i in range(3)]
    polygon=json.loads((capture/'diagnostic_rois.json').read_text())['wrist']['cream']
    mask=np.zeros(frames[0].shape[:2],np.uint8);cv2.fillConvexPoly(mask,np.int32(polygon),1)
    report=dict(motion_sent=False,camera_opened=False,device=model.device,
        source='https://github.com/facebookresearch/co-tracker',
        commit=subprocess.check_output(['git','-C',str(root/'third_party/co-tracker'),'rev-parse','HEAD'],text=True).strip(),
        checkpoint_sha256=hashlib.sha256((root/c['checkpoint']).read_bytes()).hexdigest(),
        real_frames=[],synthetic_frames=[],blank_rejected=False)
    lock=PointLock(model,frames[0],mask,c)
    for i,frame in enumerate(frames[1:],1):
        o=lock.observe(frame)
        report['real_frames'].append(dict(frame=i,visible=int(o['visible'].sum()),inliers=o['inliers'],
            anchor_shift_px=float(np.linalg.norm(o['uv']-lock.anchor)),seconds=o['inference_seconds']))
        cv2.imwrite(str(output/f'real_{i}.png'),cv2.cvtColor(draw_points(frame,o,[639.5,359.5]),cv2.COLOR_RGB2BGR))
    lock=PointLock(model,frames[0],mask,c);anchor=lock.anchor.copy()
    for i in range(1,12):
        s=1+.012*i
        matrix=np.array([[s,0,(1-s)*anchor[0]+i*2],[0,s,(1-s)*anchor[1]+i]],float)
        frame=cv2.warpAffine(frames[0],matrix,(1280,720))
        o=lock.observe(frame);truth=matrix@np.r_[anchor,1.]
        report['synthetic_frames'].append(dict(frame=i,anchor_error_px=float(np.linalg.norm(o['uv']-truth)),
                                              seconds=o['inference_seconds'],inliers=o['inliers']))
        if i==11:cv2.imwrite(str(output/'synthetic_11.png'),cv2.cvtColor(draw_points(frame,o,truth),cv2.COLOR_RGB2BGR))
    lock=PointLock(model,frames[0],mask,c)
    try:lock.observe(np.zeros_like(frames[0]))
    except ServoError as error:report.update(blank_rejected=True,blank_reason=str(error))
    report['synthetic_max_error_px']=max(x['anchor_error_px'] for x in report['synthetic_frames'])
    report['max_inference_seconds']=max(x['seconds'] for x in report['real_frames']+report['synthetic_frames'])
    report['passed']=bool(report['blank_rejected'] and report['synthetic_max_error_px']<=3 and report['max_inference_seconds']<c['max_frame_age_seconds'])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));print('OUTPUT',output,flush=True)
    if not report['passed']:raise SystemExit(2)


if __name__=='__main__':main()
