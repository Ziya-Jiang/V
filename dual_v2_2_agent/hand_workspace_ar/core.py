import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import time
import cv2
import numpy as np
from scipy.ndimage import map_coordinates

ROOT=Path(__file__).resolve().parents[1]
CACHE=ROOT/'RobotRAM/output/head_workspace_ar_20261010'
WORKSPACE=ROOT/'RobotRAM/output/right_arm_workspace_20261010'
RUNTIME=ROOT/'RobotRAM/output/hand_workspace_ar_live'
BOOT=Path('/proc/sys/kernel/random/boot_id').read_text().strip()

def atomic_json(path,document):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(document,ensure_ascii=False,allow_nan=False));temp.replace(path)

def read_state(path,max_age=.5,now=None):
    now=time.time() if now is None else now
    s=json.loads(Path(path).read_text())
    if s.get('boot_id')!=BOOT:raise ValueError('状态来自其他启动会话')
    age=now-float(s['captured_unix_s'])
    if not 0<=age<=max_age:raise ValueError(f'手部状态过期 {age:.2f}s')
    if s.get('frame')!='dummy_world' or s.get('reference')!='arm_end_effector_r':raise ValueError('手部参考坐标系不匹配')
    p=np.array(s['position_mm'],float)
    if p.shape!=(3,) or not np.isfinite(p).all() or np.max(np.abs(p))>3000:raise ValueError('手部位置格式/单位错误')
    s['age_s']=age;return s

class Renderer:
    def __init__(self):
        self.doc=json.loads((CACHE/'surface_payload.json').read_text())
        with np.load(CACHE/'surface_fields.npz') as f:self.field=f['field'].astype(float)
        self.workspace=json.loads((WORKSPACE/'forward_workspace_data.json').read_text())
        self.urdf_hash=self.workspace['stats']['sha256']
        if hashlib.sha256(Path(self.workspace['stats']['urdf']).read_bytes()).hexdigest()!=self.urdf_hash:raise ValueError('URDF 已变化，需要重建工作空间')
        RUNTIME.mkdir(parents=True,exist_ok=True)
        source=Path(__file__).with_name('surface.cpp');digest=hashlib.sha256(source.read_bytes()).hexdigest()[:12]
        lib=RUNTIME/f'surface_{digest}.so'
        if not lib.exists():subprocess.run(['g++','-O3','-shared','-fPIC','-o',str(lib),str(source)],check=True)
        self.lib=ctypes.CDLL(str(lib));self.fn=self.lib.render_surface
        ptr=lambda kind:np.ctypeslib.ndpointer(dtype=kind,flags='C_CONTIGUOUS')
        self.fn.argtypes=[ptr(np.float32),ctypes.c_int,ctypes.c_int,ptr(np.float64),ptr(np.float64),ptr(np.float32),ptr(np.uint8),ptr(np.uint8),ctypes.c_int]+[ctypes.c_double]*4+[ctypes.c_bool,ctypes.c_double]
        self.fn.restype=None;self.geometry_key=None

    def configure_camera(self,c):
        key=json.dumps({k:c[k] for k in ['K','D','T_dummy_world_camera_mm','width','height']},sort_keys=True)
        if key==self.geometry_key:return
        h,w=c['height'],c['width'];yy,xx=np.indices((h,w));uv=np.stack([xx,yy],-1).astype(float).reshape(-1,1,2)
        xy=cv2.undistortPoints(uv,np.array(c['K']),np.array(c['D'])).reshape(h,w,2)
        T=np.array(c['T_dummy_world_camera_mm'],float)
        if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('相机外参无效')
        np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)
        self.rays=np.ascontiguousarray(np.dstack([xy,np.ones((h,w))])@T[:3,:3].T)
        self.camera=np.ascontiguousarray(T[:3,3]);self.geometry_key=key;self.shape=(h,w)

    def water_regions(self,depth,plane,z):
        """Classify measured world height; never infer geometry from screen rows."""
        g=self.doc['grid'];valid=np.isfinite(depth)&(depth>=50)&(depth<=5000)
        xyz=self.camera+self.rays*np.where(valid,depth,0)[:,:,None]
        coords=np.moveaxis((xyz[:,:,:2]-np.array(g['origin_mm'][:2]))/g['step_mm'],-1,0)
        footprint=map_coordinates(plane,coords,order=1,mode='constant',cval=-120)>=0
        # Fade into the submerged treatment over 40 mm to avoid a noisy hard seam.
        submerged=np.clip((float(z)-xyz[:,:,2])/40.,0,1)*valid*footprint
        return submerged,valid&(xyz[:,:,2]>=float(z)),valid

    def render(self,bgr,depth,z,occlusion=False,strength=1.,depth_effects=True):
        if bgr.shape[:2]!=self.shape or depth.shape!=self.shape:raise ValueError('RGB-D 与标定尺寸不匹配')
        strength=float(np.clip(strength,0,1))
        g=self.doc['grid'];origin=g['origin_mm'];step=g['step_mm'];fz=(float(z)-origin[2])/step;k=int(np.floor(fz));t=fz-k
        if not 0<=k<self.field.shape[2]-1:return bgr.copy()
        plane=np.ascontiguousarray(self.field[:,:,k]*(1-t)+self.field[:,:,k+1]*t,dtype=np.float32)
        if not depth_effects:
            # Stable geometric overlay: measured depth never masks or shades RGB.
            out=np.empty_like(bgr)
            guard=np.zeros(self.shape,dtype=np.float32)
            self.fn(plane,*plane.shape,self.rays,self.camera,guard,np.ascontiguousarray(bgr),out,depth.size,*origin[:2],step,float(z),False,strength)
            return out
        submerged,above,valid=self.water_regions(depth,plane,z)
        guard=cv2.erode(np.where(valid,depth,65000).astype(np.float32),np.ones((3,3),np.uint8));guard[~valid]=0
        out=np.empty_like(bgr)
        self.fn(plane,*plane.shape,self.rays,self.camera,np.ascontiguousarray(guard),np.ascontiguousarray(bgr),out,depth.size,*origin[:2],step,float(z),occlusion,strength)
        # Warm gold, darker underwater: full effect reaches ~half brightness.
        amount=(submerged*strength)[:,:,None]
        shaded=np.rint(out*(1-amount*np.array([.59,.52,.45]))).clip(0,255).astype(np.uint8)
        under=submerged>0
        out[under]=np.minimum(shaded[under],bgr[under])
        # Objects above the water and unknown depth stay exactly as captured.
        out[above|~valid]=bgr[above|~valid]
        return out

def ee_from_joints(q,chain):
    q=np.asarray(q,float)
    if q.shape!=(7,) or not np.isfinite(q).all() or np.max(np.abs(q))>2*np.pi+.02:raise ValueError('右臂关节角无效，要求弧度')
    R=np.eye(3);p=np.array([0.,0.,342.]);k=0
    for j in chain:
        p=p+R@np.array(j['p']);R=R@np.array(j['R'])
        if j['type']=='revolute':
            c,s=np.cos(q[k]),np.sin(q[k]);R=R@np.array([[c,-s,0],[s,c,0],[0,0,1]]);k+=1
    return p,R
