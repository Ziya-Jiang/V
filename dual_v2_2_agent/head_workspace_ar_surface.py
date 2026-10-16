"""Depth-tested AR workspace slab from the existing paired RGB-D snapshot.

The smooth support field is a visualization envelope within 25 mm of sampled
forward-facing EE positions, not a new IK or collision-feasibility result.
"""
import argparse
import base64
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import map_coordinates,gaussian_filter

ROOT=Path(__file__).resolve().parent

def data_png(array):
    ok,b=cv2.imencode('.png',array,[cv2.IMWRITE_PNG_COMPRESSION,9]);assert ok
    return 'data:image/png;base64,'+base64.b64encode(b).decode()

def prepare(snapshot):
    folder=Path(snapshot);payload=json.loads((folder/'ar_payload.json').read_text())
    c=payload['metadata'];report=payload['report']
    with np.load(ROOT/'RobotRAM/output/right_arm_workspace_20261010/forward_workspace_samples.npz') as f:
        points=f['positions_mm']+np.array(report['T_dummy_world_SJ4Z_translation_mm'])
    step=20.;support=25.
    origin=np.floor((points.min(0)-60)/step)*step
    shape=np.ceil((points.max(0)+60-origin)/step).astype(int)+1
    indices=np.indices(tuple(shape)).reshape(3,-1).T
    centers=origin+indices*step
    distance=cKDTree(points).query(centers,workers=2)[0]
    continuous=(support-distance).reshape(tuple(shape))
    # Mild smoothing of a sampled support field; no hull or circular replacement.
    continuous=gaussian_filter(continuous,sigma=.85,mode='constant',cval=-120)
    field=np.clip(np.rint(continuous),-120,120).astype(np.int8)
    # One byte per grid node, encoded losslessly as a grayscale PNG.
    field_png=(field.astype(np.int16)+128).astype(np.uint8).reshape(shape[0],shape[1]*shape[2])
    depth=np.load(folder/'depth_mm.npy');valid=np.isfinite(depth)&(depth>0)
    guarded=cv2.erode(np.where(valid,depth,65000).astype(np.float32),np.ones((3,3),np.uint8))
    guarded=np.where(valid,guarded,0).round().clip(0,65535).astype(np.uint16)
    # OpenCV BGR -> browser RGB; encode depth low byte in R and high byte in G.
    depth_png=np.zeros((*depth.shape,3),np.uint8)
    depth_png[:,:,2]=guarded&255;depth_png[:,:,1]=guarded>>8
    c['surface_depth_note']='Invalid pixels remain hidden; 3x3 nearest-depth guard; +3 mm z-test tolerance'
    doc=dict(metadata=c,report=report,jpeg=payload['jpeg'],depth_png=data_png(depth_png),field_png=data_png(field_png),
        grid=dict(origin_mm=origin.tolist(),shape=shape.tolist(),step_mm=step,support_radius_mm=support,smoothing_sigma_mm=17),
        style=dict(thickness_mm=24,depth_tolerance_mm=3,grid_spacing_mm=100),
        coordinate_note='Virtual slab center Z and click positions use dummy_world, not SJ4Z')
    (folder/'surface_payload.json').write_text(json.dumps(doc,separators=(',',':')))
    np.savez_compressed(folder/'surface_fields.npz',field=field,depth_guard_mm=guarded,origin=origin,step=step)
    print('payload bytes:',(folder/'surface_payload.json').stat().st_size,'grid shape:',shape)
    return doc,field,guarded

def rays(metadata):
    h,w=metadata['height'],metadata['width'];yy,xx=np.indices((h,w))
    uv=np.stack([xx,yy],-1).astype(float).reshape(-1,1,2)
    xy=cv2.undistortPoints(uv,np.array(metadata['K']),np.array(metadata['D'])).reshape(h,w,2)
    T=np.array(metadata['T_dummy_world_camera_mm']);v=np.dstack([xy,np.ones((h,w))])@T[:3,:3].T
    return T[:3,3],v

def render_geometry(doc,field,depth,height=0,occlusion=True):
    center,ray=rays(doc['metadata']);shape=depth.shape;rz=ray[:,:,2]
    half=doc['style']['thickness_mm']/2
    with np.errstate(divide='ignore',invalid='ignore'):
        ta=(height-half-center[2])/rz;tb=(height+half-center[2])/rz
    near=np.maximum(50,np.minimum(ta,tb));far=np.maximum(ta,tb)
    valid=np.isfinite(near)&np.isfinite(far)&(far>=near)&(far>50)&(far<8000)
    hit=np.full(shape,np.nan);side=np.zeros(shape,bool)
    origin=np.array(doc['grid']['origin_mm']);step=doc['grid']['step_mm']
    def sample(t):
        xyz=center+ray*t[:,:,None];xyz[:,:,2]=height
        coords=np.moveaxis((xyz-origin)/step,-1,0)
        return map_coordinates(field.astype(float),coords,order=1,mode='constant',cval=-120)
    # Sample through the thin slab and refine the first boundary crossing.
    prev=near.copy()
    for i in range(7):
        t=near+(far-near)*(i/6)
        inside=valid&np.isnan(hit)&(sample(t)>=0)
        if i==0:hit[inside]=t[inside]
        else:
            low=prev.copy();high=t.copy()
            for _ in range(7):
                mid=(low+high)/2;positive=sample(mid)>=0
                high=np.where(positive,mid,high);low=np.where(positive,low,mid)
            hit[inside]=high[inside];side[inside]=True
        prev=t
    exists=np.isfinite(hit)
    visible=exists.copy()
    if occlusion:visible&=(depth>0)&(hit<=depth+doc['style']['depth_tolerance_mm'])
    xyz=center+ray*np.nan_to_num(hit)[:,:,None]
    xyz_center=xyz.copy();xyz_center[:,:,2]=height
    coords=np.moveaxis((xyz_center-origin)/step,-1,0)
    distance=map_coordinates(field.astype(float),coords,order=1,mode='constant',cval=-120)
    border=(distance<6)|side
    grid=(np.minimum(np.mod(xyz[:,:,0],100),100-np.mod(xyz[:,:,0],100))<1.8)|(np.minimum(np.mod(xyz[:,:,1],100),100-np.mod(xyz[:,:,1],100))<1.8)
    alpha=np.where(side,.48,np.where(border,.8,np.where(grid,.48,.2)))*visible
    # Front-facing +X chevrons, spaced on a metric grid.
    a=np.mod(xyz[:,:,0],200)-100;b=np.mod(xyz[:,:,1],200)-100
    chevron=(np.abs(a+np.abs(b)*.8)<2)&(np.abs(b)<13)
    alpha=np.where(visible&chevron,.74,alpha)
    return alpha,hit,visible,exists,xyz

def previews(folder,doc,field,depth):
    rgb=cv2.imread(str(folder/'head_rgb.png')).astype(float)
    counts={}
    for height in [0,-150]:
        for occlusion in [True,False]:
            alpha,t,visible,exists,xyz=render_geometry(doc,field,depth,height,occlusion)
            color=np.array([220,205,35.])
            out=np.rint(rgb*(1-alpha[:,:,None])+color*alpha[:,:,None]).astype(np.uint8)
            name=f'surface_z{height}_{"depth" if occlusion else "xray"}.png';cv2.imwrite(str(folder/name),out)
            counts[name]=dict(surface_pixels=int(exists.sum()),visible_pixels=int(visible.sum()),hidden_pixels=int((exists&~visible).sum()))
            if height==0 and occlusion:
                np.savez_compressed(folder/'surface_reference_z0.npz',alpha=alpha,depth=t,visible=visible)
    (folder/'surface_report.json').write_text(json.dumps(counts,indent=2));print(counts)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--snapshot',type=Path,required=True);args=parser.parse_args()
    doc,field,depth=prepare(args.snapshot);previews(args.snapshot,doc,field,depth)
