"""URDF frame audit and explicit source-model-to-upper-body rebasing.

Names are scoped to a model: half::dummy_world is NOT full::dummy_world.
A movable joint requires an explicitly supplied position, including at zero.
"""
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import Rejected,finite,pose,transform

class FrameGraph:
    def __init__(self,path):
        self.path=Path(path);self.sha256=hashlib.sha256(self.path.read_bytes()).hexdigest();doc=ET.parse(path).getroot();self.links={l.get('name') for l in doc.findall('link')};self.joints={j.get('name'):j for j in doc.findall('joint')};self.parent={}
        for joint in self.joints.values():
            child=joint.find('child').get('link');parent=joint.find('parent').get('link')
            if child in self.parent or child not in self.links or parent not in self.links:raise Rejected('Invalid URDF frame graph')
            self.parent[child]=joint
        roots=self.links-set(self.parent)
        if len(roots)!=1:raise Rejected('URDF must have one connected root')
        self.root=roots.pop()
    def chain(self,ancestor,descendant):
        if ancestor not in self.links or descendant not in self.links:raise Rejected('Unknown frame')
        result=[];current=descendant;seen=set()
        while current!=ancestor:
            if current in seen or current not in self.parent:raise Rejected('Frames do not form an ancestor chain')
            seen.add(current);j=self.parent[current];result.append(j);current=j.find('parent').get('link')
        return result[::-1]
    def relative(self,ancestor,descendant,positions=None):
        values=positions or {};t=np.eye(4)
        for j in self.chain(ancestor,descendant):
            o=j.find('origin');xyz=[float(x) for x in o.get('xyz','0 0 0').split()] if o is not None else [0,0,0];rpy=[float(x) for x in o.get('rpy','0 0 0').split()] if o is not None else [0,0,0];t=t@pose(xyz,rpy);kind=j.get('type')
            if kind!='fixed':
                name=j.get('name')
                if name not in values:raise Rejected('Actual fixed-waist joint position required: '+name)
                q=float(finite(values[name],()));axis=finite([float(x) for x in j.find('axis').get('xyz').split()],(3,));n=np.linalg.norm(axis)
                if n<1e-8:raise Rejected('Zero joint axis')
                axis/=n;m=np.eye(4)
                if kind in ('revolute','continuous'):m[:3,:3]=Rotation.from_rotvec(axis*q).as_matrix()
                elif kind=='prismatic':m[:3,3]=axis*q
                else:raise Rejected('Unsupported joint type '+kind)
                t=t@m
        return transform(t)
    def describe(self,ancestor,descendant):
        return [{'joint':j.get('name'),'type':j.get('type'),'parent':j.find('parent').get('link'),'child':j.find('child').get('link'),'origin':dict(j.find('origin').attrib) if j.find('origin') is not None else {}} for j in self.chain(ancestor,descendant)]

def camera_to_upper(source_graph,source_frame,source_T_camera,upper_frame='SJ4Z'):
    # Only a fully fixed calibration-source chain is accepted without joint data.
    source_T_upper=source_graph.relative(source_frame,upper_frame)
    return transform(np.linalg.inv(source_T_upper)@transform(source_T_camera))

def verify_arm_frame_compatibility(a,b,upper='SJ4Z'):
    """Confirm identical V2 shoulder/arm frame definitions before rebasing."""
    for side in ('l','r'):
        ca=a.chain(upper,'arm_L7_'+side);cb=b.chain(upper,'arm_L7_'+side)
        if len(ca)!=len(cb):raise Rejected('Different arm frame chains')
        for ja,jb in zip(ca,cb):
            if (ja.get('name'),ja.get('type'))!=(jb.get('name'),jb.get('type')):raise Rejected('Arm joint identity mismatch')
            for tag,attrs,defaults in [('origin',('xyz','rpy'),('0 0 0','0 0 0')),('axis',('xyz',),('1 0 0',))]:
                for attr,default in zip(attrs,defaults):
                    na,nb=ja.find(tag),jb.find(tag);va=np.fromstring(na.get(attr,default) if na is not None else default,sep=' ');vb=np.fromstring(nb.get(attr,default) if nb is not None else default,sep=' ')
                    if not np.allclose(va,vb,atol=1e-7,rtol=0):raise Rejected('Arm frame geometry mismatch')
    return True

def audit_frames(full_path,half_path,camera_document):
    full,half=FrameGraph(full_path),FrameGraph(half_path);verify_arm_frame_compatibility(full,half);raw=transform(camera_document['target_T_camera']);half_T_upper=half.relative(half.root,'SJ4Z');upper_T_camera=camera_to_upper(half,half.root,raw)
    chain=full.describe(full.root,'SJ4Z');nominal={j['joint']:0. for j in chain if j['type']!='fixed'};full_T_upper=full.relative(full.root,'SJ4Z',nominal)
    confirmed=(camera_document.get('target_frame_confirmed') is True and camera_document.get('target_frame')=='half_model::dummy_world' and camera_document.get('source_urdf_sha256')==half.sha256)
    return {'calibration_source_confirmed':confirmed,'full_model':str(full.path),'full_model_sha256':full.sha256,'half_model':str(half.path),'half_model_sha256':half.sha256,'arm_frame_geometry_matches':True,'full_chain':chain,'half_root_T_upper':half_T_upper.tolist(),'zero_waist_illustration_only':{'joint_positions_rad':nominal,'full_root_T_upper':full_T_upper.tolist(),'current_hardware_pose_known':False},'candidate_if_camera_calibrated_in_half_root':{'source_frame':half.root,'model_scope':'half','upper_frame':'SJ4Z','upper_T_camera':upper_T_camera.tolist(),'full_root_T_camera_at_reference_waist':(full_T_upper@upper_T_camera).tolist(),'status':'user_confirmed_half_model_root' if confirmed else 'inferred_candidate_not_confirmed','reason':'User confirmed half-body calibration; source-model world offset is removed exactly once' if confirmed else 'Half-model world offset is retained by root-relative FK; exact screenshot calibration provenance still unavailable'},'camera_links_in_full':[n for n in sorted(full.links) if 'camera' in n.lower() or 'optical' in n.lower()],'same_link_name_across_models_is_not_same_frame':True,'hardware_commands_sent':False}
