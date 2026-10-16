"""Project sampled forward-facing right-arm workspace onto calibrated head RGB-D.

Offline processing only. All distances are mm. Pixel geometry is distorted RGB.
Outputs are geometric illustrations, not a collision-aware graspability map.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
DEFAULT_WORKSPACE=ROOT/'RobotRAM/output/right_arm_workspace_20261010'

def project_base(points_mm,metadata):
    T=np.asarray(metadata['T_dummy_world_camera_mm'],float)
    K=np.asarray(metadata['K'],float);D=np.asarray(metadata['D'],float)
    pc=(np.asarray(points_mm)-T[:3,3])@T[:3,:3]
    uv=np.full((len(pc),2),np.nan)
    valid=np.isfinite(pc).all(1)&(pc[:,2]>50)
    uv[valid]=cv2.projectPoints(pc[valid],np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
    return uv,pc[:,2],valid

def depth_flags(uv,z,valid,depth,margin_mm=15):
    h,w=depth.shape
    inside=valid&np.isfinite(uv).all(1)&(uv[:,0]>=0)&(uv[:,0]<w-1)&(uv[:,1]>=0)&(uv[:,1]<h-1)
    flag=np.full(len(z),-2,int)
    ids=np.flatnonzero(inside);pix=np.rint(uv[ids]).astype(int);observed=depth[pix[:,1],pix[:,0]]
    known=np.isfinite(observed)&(observed>0)
    # 0 visible, 1 occluded, 2 depth unknown. Unknown depth is not assumed free.
    flag[ids]=np.where(~known,2,np.where(z[ids]<=observed+margin_mm,0,1))
    return flag

def build(snapshot,workspace=DEFAULT_WORKSPACE):
    folder=Path(snapshot);workspace=Path(workspace)
    c=json.loads((folder/'camera.json').read_text());stats=json.loads((workspace/'forward_stats.json').read_text())
    rgb=cv2.imread(str(folder/'head_rgb.png'));depth=np.load(folder/'depth_mm.npy',allow_pickle=False)
    assert rgb.shape[:2]==depth.shape==(c['height'],c['width'])
    T=np.asarray(c['T_dummy_world_camera_mm']);np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)
    urdf=Path(stats['urdf']);assert hashlib.sha256(urdf.read_bytes()).hexdigest()==stats['sha256']
    root=ET.parse(urdf).getroot();joint=root.find("joint[@name='w0']")
    assert joint.find('parent').get('link')=='dummy_world' and joint.find('child').get('link')=='SJ4Z'
    origin=joint.find('origin');assert np.allclose(np.fromstring(origin.get('rpy'),sep=' '),0)
    offset=np.fromstring(origin.get('xyz'),sep=' ')*1000
    with np.load(workspace/'forward_workspace_samples.npz',allow_pickle=False) as arr:
        points=arr['positions_mm']+offset
    uv,z,valid=project_base(points,c);flags=depth_flags(uv,z,valid,depth)
    ids=np.flatnonzero(flags>=0)
    # Retain depth-separated samples along each ray; no convex-hull filling.
    bins=np.c_[np.floor(uv[ids]/4),np.floor(points[ids,2]/30),flags[ids]].astype(int)
    _,keep=np.unique(bins,axis=0,return_index=True);ids=ids[keep]
    rng=np.random.default_rng(20261010)
    if len(ids)>11000:ids=rng.choice(ids,11000,replace=False)
    ids=ids[np.argsort(z[ids])[::-1]]
    packed=np.c_[np.round(uv[ids],1),np.round(points[ids]),np.round(z[ids]),flags[ids]].tolist()
    # Visible surfaces may be compared to nearby sampled EE positions, not declared graspable.
    # Keep measured per-pixel base coordinates for accurate click inspection.
    yy,xx=np.indices(depth.shape);pixels=np.stack([xx,yy],-1).astype(np.float64)
    normalized=cv2.undistortPoints(pixels.reshape(-1,1,2),np.array(c['K']),np.array(c['D'])).reshape(*depth.shape,2)
    camera_pts=np.dstack([normalized,np.ones(depth.shape)])*depth[:,:,None]
    base_pts=camera_pts@T[:3,:3].T+T[:3,3]
    valid_depth=np.isfinite(depth)&(depth>0)
    np.save(folder/'observed_base_points_mm.npy',np.where(valid_depth[:,:,None],base_pts,np.nan).astype(np.float32))
    ok,jpg=cv2.imencode('.jpg',rgb,[cv2.IMWRITE_JPEG_QUALITY,87]);assert ok
    info=dict(captured_at=c['captured_at'],workspace_frame='SJ4Z',camera_extrinsic_frame='dummy_world',
        T_dummy_world_SJ4Z_translation_mm=offset.tolist(),point_count=len(points),
        in_image_count=int((flags>=0).sum()),visible_count=int((flags==0).sum()),
        occluded_count=int((flags==1).sum()),unknown_depth_count=int((flags==2).sum()),
        display_count=len(ids),occlusion_tolerance_mm=15,collision_checked=False,
        capture_is_live=False,registration='Existing stored head extrinsic; physical registration not revalidated',
        calibration_sha256=c['calibration_sha256'],urdf_sha256=stats['sha256'])
    np.savez_compressed(folder/'projected_workspace.npz',uv=uv[ids],positions_base_mm=points[ids],camera_z_mm=z[ids],visibility=flags[ids])
    (folder/'projection_report.json').write_text(json.dumps(info,indent=2,ensure_ascii=False))
    # Static scientific overlays for inspection/export. Colors are display encodings.
    for name,sliced,xray in [('overlay_volume.png',False,False),('overlay_z0.png',True,False),('overlay_xray.png',False,True)]:
        layer=rgb.copy();sel=ids[(flags[ids]==0) if not xray else np.ones(len(ids),bool)]
        if sliced:sel=sel[np.abs(points[sel,2])<=30]
        for i in sel:
            color=(220,200,30) if flags[i]==0 else (190,130,70)
            cv2.circle(layer,tuple(np.rint(uv[i]).astype(int)),1,color,-1,cv2.LINE_AA)
        out=cv2.addWeighted(layer,.52,rgb,.48,0)
        label='Forward +X workspace | '+('Z_base = 0 +/- 30 mm' if sliced else 'X-ray' if xray else 'Depth occlusion')
        cv2.rectangle(out,(8,8),(min(c['width']-8,790),66),(28,28,28),-1)
        cv2.putText(out,label,(18,31),cv2.FONT_HERSHEY_SIMPLEX,.53,(245,245,245),1,cv2.LINE_AA)
        cv2.putText(out,'Snapshot '+c['captured_at'][:19]+' | geometric samples, not collision checked',(18,54),cv2.FONT_HERSHEY_SIMPLEX,.39,(235,235,235),1,cv2.LINE_AA)
        cv2.imwrite(str(folder/name),out)
    payload=dict(metadata=c,report=info,points=packed,
        jpeg='data:image/jpeg;base64,'+base64.b64encode(jpg).decode())
    (folder/'ar_payload.json').write_text(json.dumps(payload,separators=(',',':')))
    print(json.dumps(info,indent=2));return payload

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',type=Path,required=True)
    parser.add_argument('--workspace',type=Path,default=DEFAULT_WORKSPACE)
    args=parser.parse_args();build(args.snapshot,args.workspace)
