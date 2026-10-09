"""Actual triangle-mesh self-clearance diagnostic; no actuator calls.

python-fcl is loaded only for this optional diagnostic. Only rigidly connected
or directly articulated body pairs are omitted as kinematic neighbours.
"""
from pathlib import Path
import struct,itertools,hashlib
import yaml
import numpy as np
from .geometry import Rejected,apply
from .planning import ArmModel,BoxScreen
from .collision import UpperBodyEnvelopes

def stl_vertices(path):
    raw=Path(path).read_bytes()
    if len(raw)>=84 and 84+50*struct.unpack_from('<I',raw,80)[0]==len(raw):
        dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]);p=np.frombuffer(raw,dtype=dtype,offset=84)['v'].reshape(-1,3).astype(float)
    else:p=np.array([[float(v) for v in line.split()[1:]] for line in raw.decode().splitlines() if line.strip().startswith('vertex ')])
    if len(p)%3 or not len(p) or not np.isfinite(p).all():raise Rejected('Invalid STL triangles')
    return p,np.arange(len(p),dtype=np.int32).reshape(-1,3)

class MeshScreen:
    def __init__(self,model,other_joints,mount_config=None):
        import fcl
        self.fcl=fcl;self.envelopes=UpperBodyEnvelopes(model,np.eye(4),other_joints);self.objects={};self.geometries=[];self.model=model
        links={l.get('name'):l for l in model.doc.findall('link')}
        for name in self.envelopes.local:
            vertices=[];triangles=[];offset=0
            for entry in links[name].findall('collision') or links[name].findall('visual'):
                mesh=entry.find('geometry/mesh')
                if mesh is None:raise Rejected('Mesh diagnostic expects mesh URDF')
                p,faces=stl_vertices(model.path.parent/mesh.get('filename'));scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ');p=apply(ArmModel.origin(entry),p*scale);vertices.append(p);triangles.append(faces+offset);offset+=len(p)
            p=np.concatenate(vertices);faces=np.concatenate(triangles);bvh=fcl.BVHModel();bvh.beginModel(len(p),len(faces));bvh.addSubModel(p,faces);bvh.endModel();self.geometries.append(bvh);self.objects[name]=fcl.CollisionObject(bvh)
        parent={n:n for n in links}
        def root(n):
            while parent[n]!=n:n=parent[n]
            return n
        for j in model.doc.findall('joint'):
            if j.get('type')=='fixed':parent[root(j.find('child').get('link'))]=root(j.find('parent').get('link'))
        self.mount_pairs=set();self.mount_policy=None
        if mount_config is not None:
            config_path=Path(mount_config);config=yaml.safe_load(config_path.read_text());kinematics=config.get('robot_cfg',config).get('kinematics',{})
            if not kinematics:raise Rejected('Missing source robot collision policy')
            ignore=kinematics.get('self_collision_ignore',{})
            for side in ('l','r'):
                link='arm_L2_'+side;first=model.joints['arm_J2_'+side].find('origin');xyz=np.fromstring(first.get('xyz','0 0 0'),sep=' ')
                if np.linalg.norm(xyz)>1e-7:raise Rejected('Shoulder mount is not colocated in this URDF')
                if link not in ignore.get('SJ4Z',[]):raise Rejected('Shoulder mount pair is not declared by source robot policy')
                self.mount_pairs.add(frozenset(('SJ4Z',link)))
            self.mount_policy={'path':str(config_path),'sha256':hashlib.sha256(config_path.read_bytes()).hexdigest(),'scope':'source-declared colocated shoulder mount pairs only; still reject mesh penetration'}
        self.group={n:root(n) for n in links};self.neighbours={frozenset((self.group[j.find('parent').get('link')],self.group[j.find('child').get('link')])) for j in model.doc.findall('joint')}
    def inspect(self,q,margin=.01):
        boxes=self.envelopes(q);poses=self.envelopes.last_poses
        for n,obj in self.objects.items():t=poses[n];obj.setTransform(self.fcl.Transform(t[:3,:3],t[:3,3]))
        violations=[];checked=0
        for a,b in itertools.combinations(self.objects,2):
            if self.group[a]==self.group[b] or frozenset((self.group[a],self.group[b])) in self.neighbours:continue
            if BoxScreen.separation(boxes[a],boxes[b])>=margin:continue
            result=self.fcl.DistanceResult();distance=float(self.fcl.distance(self.objects[a],self.objects[b],self.fcl.DistanceRequest(),result));checked+=1
            pair_margin=0. if frozenset((a,b)) in self.mount_pairs else margin
            if distance<pair_margin:violations.append({'first':a,'second':b,'mesh_distance_m':distance})
        return {'margin_m':margin,'mesh_pairs_checked':checked,'violations':violations,'self_clear':not violations,'scope':'upper-body triangle mesh only; not an environment or lower-body collision certificate','mount_policy':self.mount_policy}
