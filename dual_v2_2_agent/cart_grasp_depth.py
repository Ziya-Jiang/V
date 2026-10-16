"""Head RGB-D base-X endpoint; never uses wrist PnP depth or book insertion."""
import json
from pathlib import Path
import numpy as np
from cart_book_grasp_v1_3_depth import tool_rotation

CONFIG=Path(__file__).with_name('cart_grasp_approach.json')


def load_approach(path=CONFIG):
    c=json.loads(Path(path).read_text())
    bounds=dict(max_path_mm=(50,500),fingertip_offset_mm=(0,150),standoff_mm=(0,200),max_forward_mm=(1,300),
                max_x_step_mm=(1,10),x_tolerance_mm=(.5,3),forward_gain=(.05,.5),ramp_seconds=(.2,1.))
    if set(c)!=set(bounds):raise ValueError('Unexpected/missing cart approach settings')
    for key,(lo,hi) in bounds.items():
        v=c[key]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not np.isfinite(v) or not lo<=v<=hi:
            raise ValueError('Invalid cart approach setting: '+key)
    if c['max_path_mm']<c['max_forward_mm']+10:raise ValueError('Cart path budget must include forward travel and probe margin')
    return c


def approach_goal(head,current,angles,c):
    """All positions in base mm; physical fingers point along tool -Z."""
    point=np.asarray(head['point_base_mm'],float);current=np.asarray(current,float)
    if head['marker_id']!=0 or point.shape!=(3,) or current.shape!=(3,) or not np.isfinite([*point,*current]).all():
        raise ValueError('Invalid right handle head geometry')
    projection=-tool_rotation(angles)[0,2]
    if projection<.5:raise ValueError('Fingers must face base +X before approach')
    tip_x=float(c['fingertip_offset_mm']*projection)
    goal=float(point[0]-tip_x-c['standoff_mm']);distance=goal-current[0]
    if distance < -c['x_tolerance_mm']:
        raise ValueError('Head depth endpoint is behind current TCP; no automatic retreat')
    if distance>c['max_forward_mm']:
        raise ValueError('Head depth exceeds cart forward travel bound')
    return dict(source='head_rgbd_calibrated_base_x',marker_id=0,
                marker_base_x_mm=float(point[0]),fingertip_x_offset_mm=tip_x,
                standoff_mm=c['standoff_mm'],goal_tcp_x_mm=goal,remaining_x_mm=float(max(0,distance)))


class ForwardStep:
    def __init__(self,c):self.c=c;self.value=0.;self.time=None
    def update(self,now,remaining,aligned):
        dt=0 if self.time is None else max(0,now-self.time);self.time=now
        wanted=min(self.c['max_x_step_mm'],self.c['forward_gain']*max(0,remaining)) if aligned else 0.
        self.value=min(wanted,self.value+self.c['max_x_step_mm']*dt/self.c['ramp_seconds'])
        return self.value
