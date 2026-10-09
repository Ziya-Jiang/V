"""Reject top-face axes before accepting an upright cart book's spine.

Depth here is robot-base X, measured along the spine's base-Z extent.
Camera optical depth is allowed to change with camera pitch. Y/Z leaning
is allowed; no fixed image angle, center shift or robot motion is introduced.
"""
from pathlib import Path
import json
import numpy as np

CONFIG_PATH=Path(__file__).with_suffix('.json')


def load_config(path=CONFIG_PATH):
    config=json.loads(Path(path).read_text())
    limits={'max_base_x_change_mm':(1.,100.),'max_base_x_per_z':(.01,1.),'min_base_z_span_mm':(5.,100.)}
    if set(config)!=set(limits):raise ValueError('Invalid base-Z spine validation configuration')
    for key,(lo,hi) in limits.items():
        value=config[key]
        if type(value) not in (float,int) or not np.isfinite(value) or not lo<=value<=hi:
            raise ValueError(f'Invalid {key}')
    return config


def validate_spine_base_depth(geometry,c2w,config):
    endpoints=np.asarray(geometry.get('spine_endpoints_camera_m'),float)
    transform=np.asarray(c2w,float)
    if endpoints.shape!=(2,3) or not np.isfinite(endpoints).all():
        raise ValueError('缺少可靠的 3D 书脊端点，不能验证基座 Z 方向深度')
    if transform.shape!=(4,4) or not np.isfinite(transform).all():
        raise ValueError('Invalid camera-to-base transform')
    # Differences need rotation only, independent of translation units/origin.
    delta=transform[:3,:3]@(endpoints[1]-endpoints[0])*1000.
    dx,dy,dz=np.abs(delta)
    slope=float(dx/max(dz,1e-9))
    stats=dict(spine_base_x_change_mm=float(dx),spine_base_y_span_mm=float(dy),
               spine_base_z_span_mm=float(dz),spine_base_x_per_z=slope)
    if dz<config['min_base_z_span_mm'] or dx>config['max_base_x_change_mm'] or slope>config['max_base_x_per_z']:
        raise ValueError(f'疑似书顶面/前后边缘：基座 Z 跨度 {dz:.1f} mm，X 深度变化 {dx:.1f} mm，'
                         f'|dX/dZ|={slope:.2f}；不符合竖直书脊的深度一致性')
    return dict(stats,spine_base_depth_status='valid')
