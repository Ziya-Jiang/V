"""Read native URDF STL envelopes without trimesh/GPU dependencies."""
from pathlib import Path
import struct
import numpy as np
from .geometry import Rejected,finite,apply,pose,transform
from .planning import ArmModel,BoxScreen
from scipy.spatial.transform import Rotation

def stl_bounds(path):
    raw=Path(path).read_bytes()
    if len(raw)>=84 and 84+50*struct.unpack_from('<I',raw,80)[0]==len(raw):
        count=struct.unpack_from('<I',raw,80)[0]
        dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]);points=np.frombuffer(raw,dtype=dtype,count=count,offset=84)['vertices'].reshape(-1,3).astype(float)
    else:
        points=np.array([[float(v) for v in line.split()[1:]] for line in raw.decode().splitlines() if line.strip().startswith('vertex ')])
    if not len(points):raise Rejected('Empty STL collision geometry')
    points=finite(points);return points.min(0),points.max(0)

class UpperBodyEnvelopes:
    def __init__(self,model,base_T_upper,other_joint_values):
        self.model=model;self.base_T_upper=transform(base_T_upper);self.other=dict(other_joint_values);self.local={};self.exclusions=[];self.geometry_sources={}
        links={l.get('name'):l for l in model.doc.findall('link')};root=model.cfg['root_link'];desc={root};self.ordered=[]
        while True:
            added=False
            for j in model.doc.findall('joint'):
                parent=j.find('parent').get('link');child=j.find('child').get('link')
                if parent in desc and child not in desc:desc.add(child);self.ordered.append(j);added=True
            if not added:break
        for name in desc:
            clouds=[]
            entries=links[name].findall('collision') or links[name].findall('visual')
            self.geometry_sources[name]='collision' if links[name].findall('collision') else 'visual_envelope'
            for collision in entries:
                geometry=collision.find('geometry');mesh=geometry.find('mesh');box=geometry.find('box')
                if mesh is not None:
                    filename=mesh.get('filename');path=model.path.parent/filename
                    if filename.startswith(('package:','file:')):raise Rejected('Resolve package URI collision meshes before use')
                    low,high=stl_bounds(path);scale=finite([float(x) for x in mesh.get('scale','1 1 1').split()],(3,));low*=scale;high*=scale
                elif box is not None:
                    half=np.array([float(x) for x in box.get('size').split()])/2;low,high=-half,half
                else:raise Rejected('Unsupported collision geometry; no silent omission')
                corners=np.array([[x,y,z] for x in [low[0],high[0]] for y in [low[1],high[1]] for z in [low[2],high[2]]]);clouds.append(apply(ArmModel.origin(collision),corners))
            if clouds:self.local[name]=np.concatenate(clouds)
        # Exempt only mechanically adjacent links; conservative false positives
        # remain for intricate hand mechanisms and must be reviewed, not ignored.
        for j in self.ordered:self.exclusions.append((j.find('parent').get('link'),j.find('child').get('link')))
        required={model.joints[n].find('child').get('link') for n in sum(model.names,[])}
        if not required.issubset(self.local):raise Rejected('Missing collision/visual geometry for an arm link')
        self.root=root
    def __call__(self,q):
        q=finite(q,(2,7));values={**self.other,**dict(zip(sum(self.model.names,[]),q.ravel()))};poses={self.root:self.base_T_upper};boxes={}
        for j in self.ordered:
            name=j.get('name');kind=j.get('type');m=np.eye(4)
            if kind!='fixed':
                mimic=j.find('mimic')
                if mimic is not None:
                    source=mimic.get('joint')
                    if source not in values:raise Rejected('Missing measured mimic source '+source)
                    value=values[source]*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'));values[name]=value
                elif name in values:value=values[name]
                else:raise Rejected('Missing measured/reference hand joint '+name)
                if not np.isfinite(value):raise Rejected('Nonfinite hand joint')
                axis=np.array([float(x) for x in j.find('axis').get('xyz').split()]);axis/=np.linalg.norm(axis)
                if kind=='prismatic':m[:3,3]=axis*value
                else:m[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix()
            poses[j.find('child').get('link')]=poses[j.find('parent').get('link')]@ArmModel.origin(j)@m
        for name,corners in self.local.items():
            p=apply(poses[name],corners);low=p.min(0);high=p.max(0);boxes[name]=((low+high)/2,np.maximum((high-low)/2,1e-5))
        self.last_poses=poses
        return boxes
