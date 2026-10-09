"""Read-only commissioning and reproducible offline tools."""
import argparse,hashlib,importlib.util,json,os,subprocess,sys,time
from pathlib import Path
import numpy as np
import yaml
from .geometry import Rejected,pose,apply,calibrated_frames,register_points
from .perception import CartDetector,StableTracker,model_points,cloud_from_rgbd,CartObservation
from .planning import ArmModel,DualPlanner,BoxScreen
from .collision import UpperBodyEnvelopes
from .integration import read_arm_state,measured_arm_inputs,actuator_contract

def config(path):return yaml.safe_load(Path(path).read_text())
def save(path,value):
    p=Path(path);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

def doctor(args):
    cfg=config(args.config);root=Path(args.seldon);model=Path(cfg['robot']['model_path']);result={'hardware_commands_sent':False,'model_exists':model.is_file(),'model_sha256':hashlib.sha256(model.read_bytes()).hexdigest() if model.is_file() else None,'dependencies':{m:importlib.util.find_spec(m) is not None for m in ['numpy','scipy','cv2','yaml','rclpy','cv_bridge','tf2_ros','message_filters']},'actuator_contract':actuator_contract(),'waist':'fixed, measured base_T_upper required; no waist actuation','blockers':[]}
    camera_path=Path(args.config).parent/'camera_user_20260921.yaml'
    if camera_path.exists():
        from .camera_model import validate_camera_document
        result['provided_camera']=validate_camera_document(config(camera_path))
        if config(camera_path).get('target_frame')=='half_model::dummy_world':
            from .camera_model import confirmed_upper_camera
            result['provided_camera']['upper_T_camera']=confirmed_upper_camera(config(camera_path)).tolist()
        if not result['provided_camera']['ready_for_frame_binding']:result['blockers'].append('Supplied camera extrinsic target frame (arm base versus chassis base) awaiting confirmation')
    if args.probe_cockpit:
        try:
            s=read_arm_state(root/'dual_arm_cockpit/runtime/web.json');result['arm']={k:s.get(k) for k in ['fresh','active','simulation','rt_running','gripper_feedback_sources','capabilities','coordinate_system']}
            if not s.get('fresh'):result['blockers'].append('No fresh RT arm joint feedback; RT not started by this package')
            if 'command_echo' in s.get('gripper_feedback_sources',[]):result['blockers'].append('Right gripper feedback is command echo, not contact sensing')
        except Exception as exc:result['blockers'].append('Cockpit read-only probe failed: '+type(exc).__name__)
    if args.probe_ros:
        r=subprocess.run(['ros2','topic','list','-t'],capture_output=True,text=True,timeout=15);result['ros_topics']=r.stdout.splitlines()
        if r.returncode:result['blockers'].append('ROS topic discovery failed')
        services=subprocess.run(['ros2','service','list','-t'],capture_output=True,text=True,timeout=15);result['ros_services']=services.stdout.splitlines()
        required=['/chassis/set_mode_0','/chassis/enable','/chassis/disable']
        if not all(any(line.startswith(name+' ') for line in result['ros_services']) for name in required):result['blockers'].append('Current ROS domain does not provide the documented Seldon chassis services; do not assume /mobile_base is compatible')
        for key in ['rgb_topic','depth_topic','info_topic']:
            if not any(line.startswith(cfg['ros'][key]+' ') for line in result['ros_topics']):result['blockers'].append('Missing configured '+key+': '+cfg['ros'][key])
    if not cfg['ros']['rectified_aligned_depth'] or not cfg['ros']['camera_tf_calibrated']:result['blockers'].append('RGB-D alignment and camera/floor calibration not commissioned')
    result['hardware_ready']=False;save(args.output,result);print(json.dumps(result,indent=2))

def demo(args):
    cfg=config(args.config);detector=CartDetector(cfg);rng=np.random.default_rng(7);t=pose([.9,0,cfg['cart']['base_floor_z_m']],[0,0,np.pi/2]);points=apply(t,model_points(cfg['cart'],.008));points+=rng.normal(0,.001,points.shape)
    # Synthetic evidence is explicitly labeled and never hardware feedback.
    observed=detector.estimate(points,10.);tracker=StableTracker(cfg['perception']);stable=False
    for i in range(4):observed.stamp=10.+i*.2;stable=tracker.update(observed)
    height_changes=[]
    for height in [.70,.90,1.10]:
        upper=pose([0,0,height]);height_changes.append({'upper_origin_height_m':height,'handle_z_in_upper_m':apply(np.linalg.inv(upper),observed.handles)[:,2].tolist()})
    result={'synthetic':True,'hardware_commands_sent':False,'perception':observed.to_dict(),'stable':stable,'fixed_waist_adaptation':height_changes,'note':'Synthetic point-cloud test, not a hardware success or contact claim.'};save(args.output,result);print(json.dumps(result,indent=2))

def observe(args):
    cfg=config(args.config);c=config(args.calibration);calibrated_frames(c,cfg['robot']['model_path'],args.session);cfg['cart']['base_floor_z_m']=c['base_floor_z_m']
    with np.load(args.input,allow_pickle=False) as z:
        stamp=float(z['stamp_s']);scale=float(z['depth_scale']);points,_=cloud_from_rgbd(z['bgr'],z['depth'],z['k'],c['base_T_camera'],cfg['perception'],scale,c['camera_alignment_verified'],distortion=z['d'] if 'd' in z.files else None);observation=CartDetector(cfg).estimate(points,stamp)
    save(args.output,{'replay_only':True,'calibration_session':args.session,**observation.to_dict()});print('Recorded RGB-D observation saved; no commands sent.')

def plan(args):
    cfg=config(args.config);cal=config(args.calibration);frames=calibrated_frames(cal,cfg['robot']['model_path'],args.session);record=json.loads(Path(args.observation).read_text());state=json.loads(Path(args.state).read_text());world=json.loads(Path(args.obstacles).read_text())
    if record.get('calibration_session')!=args.session or world.get('calibration_session')!=args.session or world.get('frame_id')!=cal['base_frame']:raise Rejected('Observation/obstacle calibration session mismatch')
    if not all(world.get(k) is True for k in ['includes_cart','includes_lower_body','includes_environment']):raise Rejected('Need cart, measured lower body and environment collision coverage')
    q,bounds=measured_arm_inputs(state);model=ArmModel(cfg['robot']['model_path'],cfg['robot']);obs=CartObservation(**{k:np.array(record[k]) if k in ('base_T_cart','handles','bar_axis') else record[k] for k in CartObservation.__dataclass_fields__});geometry=UpperBodyEnvelopes(model,frames['base_T_upper'],state.get('hand_joint_positions_rad',{}));screen=BoxScreen(geometry,world['boxes'],geometry.exclusions)
    result=DualPlanner(model).plan(obs,frames['base_T_upper'],[frames['left_tip_T_contact'],frames['right_tip_T_contact']],q,bounds,screen);result.update(calibration_session=args.session,replay_only=True,robot_model_sha256=cal['robot_model_sha256']);save(args.output,result);print('Offline plan saved; not an execution request.')

def main():
    default=Path(__file__).resolve().parents[1]/'config/cart_grasp.yaml'
    if not default.exists():
        from ament_index_python.packages import get_package_share_directory
        default=Path(get_package_share_directory('seldon_cart_grasp'))/'config/cart_grasp.yaml'
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',default=str(default));sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('doctor');p.add_argument('--seldon',default='/home/erazer/Seldon');p.add_argument('--probe-cockpit',action='store_true');p.add_argument('--probe-ros',action='store_true');p.add_argument('--output',default='runtime/doctor.json');p.set_defaults(func=doctor)
    p=sub.add_parser('demo');p.add_argument('--output',default='runtime/synthetic_demo.json');p.set_defaults(func=demo)
    p=sub.add_parser('frame-audit');p.add_argument('--full-urdf',required=True);p.add_argument('--half-urdf',required=True);p.add_argument('--camera',required=True);p.add_argument('--output',default='runtime/frame_audit.json')
    def frame_audit(a):
        from .frame_graph import audit_frames
        report=audit_frames(a.full_urdf,a.half_urdf,config(a.camera));save(a.output,report);print(json.dumps(report,indent=2))
    p.set_defaults(func=frame_audit)
    p=sub.add_parser('camera-check');p.add_argument('--camera',required=True);p.add_argument('--output',default='runtime/camera_check.json')
    def camera_check(a):
        from .camera_model import validate_camera_document
        report=validate_camera_document(config(a.camera))
        if config(a.camera).get('target_frame')=='half_model::dummy_world':
            from .camera_model import confirmed_upper_camera
            report['upper_T_camera']=confirmed_upper_camera(config(a.camera)).tolist()
        save(a.output,report);print(json.dumps(report,indent=2))
    p.set_defaults(func=camera_check)
    p=sub.add_parser('compose-waist-camera');p.add_argument('--camera',required=True);p.add_argument('--waist-calibration',required=True);p.add_argument('--output',required=True)
    def compose_camera(a):
        from .camera_model import compose_fixed_waist_camera
        result=compose_fixed_waist_camera(config(a.camera),config(a.waist_calibration));Path(a.output).write_text(yaml.safe_dump(result,sort_keys=False));print('Camera composed with measured fixed waist. TCP/floor/depth checks remain active.')
    p.set_defaults(func=compose_camera)
    p=sub.add_parser('calibrate-points');p.add_argument('--pairs',required=True);p.add_argument('--output',required=True)
    def calibrate(a):
        pairs=json.loads(Path(a.pairs).read_text());t,quality=register_points(pairs['source_xyz_m'],pairs['target_xyz_m']);save(a.output,{'target_T_source':t.tolist(),'quality':quality,'accepted':quality['maximum_m']<=.008,'requires_measured_frame_labels_and_session':True})
    p.set_defaults(func=calibrate)
    p=sub.add_parser('observe-recording');p.add_argument('--input',required=True);p.add_argument('--calibration',required=True);p.add_argument('--session',required=True);p.add_argument('--output',required=True);p.set_defaults(func=observe)
    p=sub.add_parser('plan-recording');p.add_argument('--observation',required=True);p.add_argument('--state',required=True);p.add_argument('--obstacles',required=True);p.add_argument('--calibration',required=True);p.add_argument('--session',required=True);p.add_argument('--output',required=True);p.set_defaults(func=plan)
    args=parser.parse_args()
    try:args.func(args)
    except (Rejected,ValueError,KeyError,FileNotFoundError) as error:parser.exit(2,'REJECTED: '+str(error)+'\n')
if __name__=='__main__':main()
