"""Create measured target geometry and audit nominal tool offsets without motion."""
import hashlib,json,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from .geometry import finite,transform,apply,Rejected
from .frame_graph import FrameGraph
from .planning import ArmModel
from .collision import stl_bounds

def bounds_in_tip(graph,doc,tip,link,positions):
    element=next(l for l in doc.findall('link') if l.get('name')==link);points=[]
    for visual in element.findall('visual'):
        mesh=visual.find('geometry/mesh')
        if mesh is None:continue
        path=graph.path.parent/mesh.get('filename');lo,hi=stl_bounds(path);scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ');corners=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])*scale
        t=graph.relative(tip,link,positions)@ArmModel.origin(visual);points.extend(apply(t,corners))
    if not points:raise Rejected('No mesh data for TCP audit')
    p=np.array(points);return {'lower':p.min(0).tolist(),'upper':p.max(0).tolist()}

def tool_audit(seldon):
    root=Path(seldon);model=root/'dual_arm_cockpit/models/full_v2/robot.urdf';graph=FrameGraph(model);doc=ET.parse(model).getroot();components=root/'dual_arm_cockpit/config_sources/V2/dual_v2_2_configuration/share/config/components';result={'model_sha256':graph.sha256,'tools':{},'contact_tcp_calibrated':False}
    for side,filename,links in [('left','hand_left.yaml',['hand_base_link_l']),('right','gripper_right.yaml',['gripper_base_link_r','gripper_L1_r','gripper_L2_r'])]:
        p=components/filename;c=yaml.safe_load(p.read_text());suffix='l' if side=='left' else 'r';tip='arm_L7_'+suffix;ee='arm_end_effector_'+suffix;t=graph.relative(tip,ee);offset=finite(c['offset_from_ee'],(3,));point=apply(t,offset);boxes={link:bounds_in_tip(graph,doc,tip,link,{'gripper_J1_r':0.,'gripper_J1_r_1':0.}) for link in links};entry={'source':str(p),'source_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'tip_link':tip,'ee_link':ee,'nominal_tip_point_m':point.tolist(),'contact_frame_rotation':None,'geometry_at_explicit_reference_gripper_q_zero':boxes,'usable_as_contact_tcp':False}
        if side=='right':
            lower=np.min([b['lower'] for b in boxes.values()],axis=0);upper=np.max([b['upper'] for b in boxes.values()],axis=0);distance=np.maximum(np.maximum(lower-point,point-upper),0);entry['distance_outside_all_finger_body_bounds_m']=float(np.linalg.norm(distance));entry['issue']='Configured -0.20 m tool reference lies beyond current URDF finger reach; do not reuse it as grip center.'
        else:entry['issue']='Configured hand tool point is a nominal reference; palm/finger contact frame and wrap pose remain to be measured.'
        result['tools'][side]=entry
    return result

def contact_geometry(proposal,now,maximum_age=1.5):
    if proposal.get('frame')!='SJ4Z' or proposal.get('kind')!='parallel_blue_bar_proposals':raise Rejected('Wrong observation frame/type')
    stamp=float(finite(proposal.get('stamp'),()));age=float(now)-stamp
    if not 0<=age<=maximum_age or proposal.get('temporal_stable') is not True:raise Rejected('Need fresh stable handle observations')
    points=finite(proposal['handles'],(2,3));axis=finite(proposal['bar_axis'],(3,));normal=finite(proposal['approach_normal'],(3,))
    if np.linalg.norm(axis)<.9 or np.linalg.norm(normal)<.9:raise Rejected('Invalid measured handle basis')
    normal=normal/np.linalg.norm(normal);axis=axis-(axis@normal)*normal
    if np.linalg.norm(axis)<.8:raise Rejected('Handle direction/normal inconsistent')
    axis/=np.linalg.norm(axis);x=np.cross(axis,normal);separation=np.linalg.norm(points[0]-points[1])
    if not .415<separation<.515 or points[0,1]<=points[1,1]:raise Rejected('Handle assignment/spacing invalid')
    stages=[]
    for stage,distance in [('pregrasp',.08),('approach',.04),('contact',0.)]:
        targets=[]
        for point in points:
            t=np.eye(4);t[:3,:3]=np.column_stack([x,axis,normal]);t[:3,3]=point+distance*normal;targets.append(transform(t).tolist())
        stages.append({'stage':stage,'upper_T_contact_targets':targets})
    return {'frame':'SJ4Z','observation_stamp':stamp,'age_s':age,'left_assignment':'left hand','right_assignment':'right claw','stages':stages,'target_type':'geometric_contact_frames_not_flange_or_joint_commands','hardware_executable':False}

def create_review(seldon,proposal,snapshot,now):
    geometry=contact_geometry(proposal,now);audit=tool_audit(seldon);blockers=[]
    if not snapshot.get('fresh'):blockers.append('No current measured arm joints; no start configuration or collision path has been generated.')
    blockers.append('Nominal tool offsets are not calibrated contact TCPs; orientation and finger wrap have not been validated.')
    blockers.append('Measured contact feedback and whole-body path checks remain required.')
    return {'created_unix_s':float(now),'contact_geometry':geometry,'tool_audit':audit,'arm_feedback_fresh':bool(snapshot.get('fresh')),'joint_trajectory':None,'collision_checked_path':False,'hardware_executable':False,'blockers':blockers}
