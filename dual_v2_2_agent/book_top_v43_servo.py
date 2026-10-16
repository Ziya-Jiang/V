"""V4.3 lift-axis adapter for the existing V3.4 image Jacobian controller.

Planner-paced closed loop, not the fixed-Z streaming backend. No device IO.
"""
import json
import time
from pathlib import Path
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from book_top_v43_control import run_lift_centerline

CONFIG=Path(__file__).with_name('cart_book_grasp_v4_3_raise.json')


def load_config(path=CONFIG):
    data=json.loads(Path(path).read_text())
    outer=dict(max_lateral_mm=(1,100),feedback_max_age_seconds=(.05,.25),
               frame_wait_seconds=(.5,10),selection_hold_mm=(.1,3),selection_hold_deg=(.1,1),
               correction_position_tolerance_mm=(.3,1),correction_orientation_tolerance_deg=(.1,.5))
    if set(data)!=set(outer)|{'controller'}:raise ServoError('Invalid V4.3 lift settings')
    c=data['controller']
    bounds=dict(probe_mm=(4,6),max_step_mm=(4,15),base_step_mm=(4,15),near_step_mm=(4,10),near_goal_mm=(10,40),progress_pause_error_ratio=(1.2,2.),max_forward_mm=(1,400),max_path_mm=(20,500),
        gain=(.05,.7),tolerance_px=(.5,8),x_tolerance_mm=(.1,3),max_seconds=(10,900),
        max_frame_age_seconds=(.1,2),max_pose_error_mm=(.1,3),max_orientation_error_deg=(.1,1),
        coarse_pose_error_mm=(1,8),coarse_settle_seconds=(.1,1),coarse_feedback_timeout_seconds=(.5,5),
        settle_seconds=(0,2),probe_roll_deg=(1,3),max_roll_step_deg=(.5,3),max_roll_deg=(3,20),angle_tolerance_deg=(.5,3))
    ints=dict(fast_aligned_steps=(2,5),stable_frames=(2,5),max_iterations=(1,400),snapshot_every=(1,1000))
    if not isinstance(c,dict) or set(c)!=set(bounds)|set(ints)|{'mode','target_u'} or c['mode']!='centerline_distance':
        raise ServoError('Invalid V4.3 controller schema')
    for values,limits in ((data,outer),(c,bounds),(c,ints)):
        for key,(lo,hi) in limits.items():
            value=values[key]
            types=(int,) if limits is ints else (int,float)
            if type(value) not in types or not np.isfinite(value) or not lo<=value<=hi:
                raise ServoError('Invalid V4.3 setting: '+key)
    if not c['max_pose_error_mm'] < c['near_step_mm'] <= c['base_step_mm'] <= c['max_step_mm']:
        raise ServoError('Invalid V4.3 adaptive step order')
    if c['probe_mm']<=c['max_pose_error_mm'] or c['max_step_mm']<=c['max_pose_error_mm']:
        raise ServoError('V4.3 step/probe must exceed arrival tolerance')
    if c['target_u'] is not None and (type(c['target_u']) not in (int,float) or not np.isfinite(c['target_u'])):
        raise ServoError('Invalid V4.3 target column')
    return data


def run_raise(observe,move,target_u,goal_z,settings,*,path_available,
              checkpoint=lambda:None,log=lambda event:None,clock=time.monotonic,recover=None):
    """Map [base X,Y,Z] to the legacy controller's [progress,Y,fixed] axes.

    Z progress uses hysteresis and slows with image error; final tolerance is strict. X stays fixed, Y and local
    tool-Z roll correct the image. Explicit point loss may use measured history.
    """
    c=dict(settings['controller']);c['history_max_lateral_mm']=settings['max_lateral_mm'];c['max_path_mm']=min(c['max_path_mm'],path_available)
    if not np.isfinite(c['max_path_mm']) or c['max_path_mm']<=0:raise ServoError('V4.3 no lift path budget')
    origin=None
    def read():
        nonlocal origin
        o=dict(observe());q=np.asarray(o['xyz_mm'],float)
        if q.shape!=(3,) or not np.isfinite(q).all():raise ServoError('V4.3 invalid measured TCP')
        if origin is None:origin=q.copy()
        if abs(q[1]-origin[1])>settings['max_lateral_mm']+c['max_pose_error_mm']:
            raise ServoError('V4.3 measured Y correction bound exceeded')
        o['xyz_mm']=q[[2,1,0]]
        return o
    def go(mapped,roll):
        q=np.asarray(mapped,float)[[2,1,0]]
        if abs(q[1]-origin[1])>settings['max_lateral_mm']:
            raise ServoError('V4.3 Y correction command bound exceeded')
        return move(q,roll)
    def event(e):
        e=dict(e)
        if 'xyz_mm' in e:e['xyz_mm']=np.asarray(e['xyz_mm'])[[2,1,0]].tolist()
        for key in ('target_mm','actual_mm','delta_mm'):
            if key in e:e[key]=np.asarray(e[key])[[2,1,0]].tolist()
        if 'actual_x_mm' in e:e['actual_z_mm']=e.pop('actual_x_mm')
        e['advance_axis']='base_Z';e['fixed_axis']='base_X'
        log(e)
    try:
        result=run_lift_centerline(read,go,target_u,goal_z,c,checkpoint=checkpoint,log=event,clock=clock,recover=recover)
    except ServoError as error:
        # Legacy messages refer to virtual axes; never mislabel physical Z as X.
        message=str(error).replace('Centerline X','V4.3 Z').replace('measured Z drift','measured X drift').replace('after Z drift','after X drift').replace('forward budget','lift budget')
        raise ServoError(message) from error
    result=dict(result);result['actual_z_mm']=result.pop('actual_x_mm')
    return result
