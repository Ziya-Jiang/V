"""Head RGB-D snapshot only. Never imports or creates a robot controller."""
import argparse
import ast
from datetime import datetime
import hashlib
import json
import logging
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parent
DEFAULT_CALIBRATION=Path('/mnt/ssd/user/wx/vlm_demo/calibration/session_seed/base_to_camera/results/machine-readable results.json')

def capture(output,calibration_path=DEFAULT_CALIBRATION):
    sys.path.insert(0,str(ROOT/'RobotRAM'))
    from controllor.extern_camera import OrbbecCamera
    # Load only the existing camera subclass, without importing the motion pipeline.
    source=ROOT/'dual_v2_2_agent_pcl.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='OrbbecRgbdCamera')
    namespace={'OrbbecCamera':OrbbecCamera,'logging':logging}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),namespace)
    document=json.loads(Path(calibration_path).read_text());c=document['left']
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    camera=namespace['OrbbecRgbdCamera'](is_warmup=False,preset_name='Default')
    for name in ['K','D','R','T']:setattr(camera,name,np.array(c[name],float))
    camera.w,camera.h=int(c['w']),int(c['h'])
    try:
        camera._initialize_camera();valid=0
        for _ in range(45):
            rgb,depth=camera.capture_image()
            if rgb is not None and depth is not None and np.mean(depth>0)>.05:
                valid+=1
                if valid>=15:break
        if valid<15:raise RuntimeError('No stable valid RGB-D snapshot received')
        stamp=datetime.now().astimezone().isoformat()
        Image.fromarray(rgb).save(out/'head_rgb.png');np.save(out/'depth_mm.npy',depth)
        info=camera._ob_device.get_device_info()
        metadata=dict(captured_at=stamp,captured_unix_s=time.time(),K=camera.K.tolist(),D=camera.D.tolist(),
            T_dummy_world_camera_mm=camera.c2w.tolist(),calibration_path=str(calibration_path),
            calibration_sha256=hashlib.sha256(Path(calibration_path).read_bytes()).hexdigest(),
            frame_binding='half-model dummy_world as used by V3/Curobo',
            width=camera.w,height=camera.h,serial=info.get_serial_number(),
            color_encoding='RGB',image_geometry='raw_distorted_resized_as_existing_V3_driver',
            depth_alignment=str(camera._orbbec_align_mode),depth_units='mm',valid_depth_fraction=float(np.mean(depth>0)),
            robot_connected=False,motion_commands_sent=False,
            registration_note='Existing stored extrinsic; no new physical registration validation performed')
        (out/'camera.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False))
        print(json.dumps(metadata,indent=2,ensure_ascii=False))
    finally:camera.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION)
    args=parser.parse_args();capture(args.output,args.calibration)
