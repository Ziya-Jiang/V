"""V4.1 task extension of the V3.4/V1_3 loop, perception and execution.

Reuses the keyboard loop, camera and action protocol; selects the V4.1 planner.
Import is inert.
"""
from contextlib import contextmanager
from pathlib import Path
import json
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parent
STAGES=('align_y','raise_z','forward_x','lower_z','withdraw_x')
PREFIX='book_v41_top_'


def load_config(path=None):
    from book_top_model_coarse import load_config as base_config,CONFIG
    config=base_config(CONFIG if path is None else path,hover_limit_mm=200.,descent_limit_mm=150.)
    return validate_distances(config)


def validate_distances(config):
    from book_top_model_coarse import validate_config
    config=validate_config(config,hover_limit_mm=200.,descent_limit_mm=150.)
    config.setdefault('forward_insertion_mm',5.)
    insertion=config['forward_insertion_mm']
    if type(insertion) not in (int,float) or not np.isfinite(insertion) or not 0<=insertion<=config['max_forward_mm']:
        raise ValueError('Invalid forward_insertion_mm')
    withdraw=config.get('withdraw_mm',50.)
    if type(withdraw) not in (int,float) or not np.isfinite(withdraw) or not 0<withdraw<=config['max_forward_mm']:
        raise ValueError('Invalid withdraw_mm')
    config['withdraw_mm']=withdraw
    value=config.get('lower_offset_mm')
    if config.get('final_z_offset_from_top_mm') is None and (type(value) not in (int,float) or not np.isfinite(value) or not -config['max_descent_mm']<=value<0):
        raise ValueError('lower_offset_mm must be negative and within max_descent_mm')
    return config


def reload_distances(config,stage=None):
    """Atomic per-stage snapshot: edits affect the next stage, not its waypoints."""
    from book_top_v41_distances import read_distances,DISTANCE_KEYS
    path=config.get('_distance_config_path')
    snapshot=dict(config)
    if path:
        snapshot.update(read_distances(path))
    snapshot=validate_distances(snapshot)
    if stage is not None:
        print('V4.1 距离参数已热加载：'+json.dumps(dict(stage=stage,file=path,
              parameters={key:snapshot[key] for key in sorted(DISTANCE_KEYS)}),ensure_ascii=False),flush=True)
    return snapshot


def descent_description(config):
    final=config.get('final_z_offset_from_top_mm')
    return (f"Z从实际位置固定下降{-config['lower_offset_mm']:g}mm" if final is None
            else f'Z下降至上沿偏移{final:+g}mm')


def stage_offset(stage,current,rotation,geometry,c):
    """Relative displacement of the current configured finger reference."""
    q=np.asarray(current,float);r=np.asarray(rotation,float)
    if q.shape!=(3,) or r.shape!=(3,3) or not np.isfinite(q).all() or not np.isfinite(r).all():
        raise ValueError('Invalid current pose for V4.1 offset')
    finger=q+r@np.asarray(c['tcp_to_finger_tool_mm'],float)
    if stage=='align_y':axis=1;delta=geometry['spine_center_base_mm'][1]-finger[1];limit=c['max_y_travel_mm']
    elif stage=='raise_z':axis=2;delta=geometry['top_base_mm'][2]-finger[2]+c['hover_above_top_mm'];limit=c['max_raise_mm']
    elif stage=='forward_x':
        axis=0
        delta=geometry['book_median_x_mm']+c['forward_insertion_mm']-finger[0]
        limit=c['max_forward_mm']
    elif stage=='lower_z':
        axis=2
        final=c.get('final_z_offset_from_top_mm')
        delta=c['lower_offset_mm'] if final is None else geometry['top_base_mm'][2]+final-finger[2]
        limit=c['max_descent_mm']
    elif stage=='withdraw_x':axis=0;delta=-c['withdraw_mm'];limit=c['max_forward_mm']
    else:raise ValueError('Unknown V4.1 offset stage')
    if not np.isfinite(delta) or abs(delta)>limit+1e-6:raise ValueError(stage+' offset exceeds configured range')
    if stage in ('raise_z','forward_x') and delta<-.01:raise ValueError(stage+' would move in the opposite direction')
    if stage=='lower_z' and delta>0:raise ValueError('lower_z would require upward motion')
    offset=np.zeros(3);offset[axis]=delta
    return offset


def top_geometry(rgb,depth,intrinsic,c2w,selected,book_mask):
    """Add the upper end of the already selected spine; never select a new book."""
    from book_top_local_depth import local_landmark
    mask=np.asarray(selected['mask'],bool)
    yy,xx=np.nonzero(mask)
    if not len(xx):raise ValueError('V4.1 selected spine mask is empty')
    points=np.c_[xx,yy].astype(float)
    along=np.asarray(selected['spine_axis_uv'],float)
    if along.shape!=(2,) or not np.isfinite(along).all() or np.linalg.norm(along)<1e-8:
        raise ValueError('V4.1 invalid selected spine axis')
    along=along/np.linalg.norm(along)
    if along[1]<0:along=-along
    across=np.array([-along[1],along[0]])
    t=points@along;s=points@across
    top_t=float(t.min())
    band=s[t<=top_t+3.]
    top_uv=along*top_t+across*float(np.median(band))
    measured=np.asarray(depth,float).copy()
    valid=measured[np.isfinite(measured)&(measured>0)]
    if valid.size and np.median(valid)<20:measured*=1000.
    transform=np.asarray(c2w,float).copy()
    if np.linalg.norm(transform[:3,3])<10:transform[:3,3]*=1000.
    contour=cv2.convexHull(points.astype(np.float32)).reshape(-1,2)
    top,quality=local_landmark(top_uv,top_uv,contour,measured,intrinsic,np.zeros(5),transform,
                               quality_warning_only=True)
    center=transform[:3,:3]@(np.asarray(selected['point_camera_m'])*1000.)+transform[:3,3]
    depth_center=center.copy();depth_center[0]=float(selected['book_surface_x_mm'])
    # Median measured extent along the actual advance axis, not camera Z.
    full_mask=np.asarray(book_mask,bool)
    if full_mask.shape!=measured.shape:raise ValueError('V4.1 full-book depth mask mismatch')
    vy,vx=np.nonzero(full_mask & np.isfinite(measured) & (measured>0))
    if not len(vx):raise ValueError('V4.1 no measured full-book median depth')
    rays=np.linalg.solve(np.asarray(intrinsic,float),np.array([vx,vy,np.ones(len(vx))]))
    camera=(rays/rays[2]*measured[vy,vx]).T
    base=camera@transform[:3,:3].T+transform[:3,3]
    if not np.isfinite(base).all():raise ValueError('V4.1 invalid full-book depth coordinates')
    median_x=float(np.median(base[:,0]))
    nearest=int(np.argmin(np.abs(base[:,0]-median_x)))
    # The median need not coincide with a measured point (even sample count).
    # 3D shows an exact X reference; 2D marks the nearest measured sample.
    median_reference=base[nearest].copy();median_reference[0]=median_x
    if not np.isfinite([top,center,depth_center]).all():raise ValueError('V4.1 non-finite measured book coordinates')
    return dict(top_base_mm=top,spine_center_base_mm=center,depth_center_base_mm=depth_center,
                book_median_x_mm=median_x,book_median_reference_base_mm=median_reference,
                book_median_sample_uv=[int(vx[nearest]),int(vy[nearest])],
                forward_depth_method='median_measured_base_x_in_full_book_mask',
                top_center_uv=top_uv,
                top_edge_uv=np.array([along*top_t+across*float(v) for v in (band.min(),band.max())]),
                local_depth=quality,
                source=selected.get('selection_source','sam3'),contact_confirmed=False)


def log_upper_edge(logger,geometry,config):
    """Annotate the exact control landmark; do not recompute or move it."""
    import rerun as rr
    top=np.asarray(geometry['top_base_mm'],float)
    uv=np.asarray(geometry['top_center_uv'],float)
    label=f'Upper edge | base Z={top[2]:.1f} mm'
    gold=[255,200,30]
    for root in ('book/images/book','book/images/spine','book/depth'):
        rr.log(root+'/upper_edge',rr.Points2D([uv],colors=gold,radii=7,
                                             labels=[label],show_labels=True))
        rr.log(root+'/upper_edge/span',rr.LineStrips2D([geometry['top_edge_uv']],colors=gold,radii=2))
    rr.log('world/book/upper_edge',rr.Points3D([top/1000.],colors=gold,radii=.006,
                                            labels=[label],show_labels=True))
    median_label=f"Book median X={geometry['book_median_x_mm']:.1f} mm"
    rr.log('book/depth/median_x',rr.Points2D([geometry['book_median_sample_uv']],colors=[255,100,180],
                                            radii=6,labels=[median_label+" (nearest sample)"],show_labels=True))
    rr.log('world/book/median_x',rr.Points3D([np.asarray(geometry['book_median_reference_base_mm'])/1000.],
                colors=[255,100,180],radii=.005,labels=[median_label+" reference"],show_labels=True))
    offsets=[float(config['hover_above_top_mm'])]
    final=config.get('final_z_offset_from_top_mm')
    offsets.append(offsets[0]+float(config['lower_offset_mm']) if final is None else float(final))
    points=np.array([top+np.array([0.,0.,z]) for z in offsets])
    labels=[f'Nominal finger reference {z:+g} mm | base Z={point[2]:.1f} mm' for z,point in zip(offsets,points)]
    rr.log('world/book/upper_edge/reference',rr.Points3D(points/1000.,colors=[80,220,255],
                                                      radii=.004,labels=labels,show_labels=True))
    rr.log('world/book/upper_edge/height',rr.LineStrips3D([[top/1000.,points[0]/1000.]],
                                                       colors=gold,radii=.0015))
    logger.flush_book()


def task_list(tasks,through_step):
    if not tasks:return tasks
    seed=next((t for t in tasks if t.get('book_grasp_mode')=='cart+' and t.get('arm')=='right'),None)
    if seed is None:raise ValueError('V4.1 requires the inherited right-book cart+ task')
    return [dict(seed,action=PREFIX+stage,target_position=None,target_rotation=None,
                 description='V4.1 upper-edge '+stage) for stage in STAGES[:through_step-1]]


def execute_stage(robot,task,index,objects,execute,step5,state,config):
    from book_top_coarse_alignment import segments
    stage=task['action'][len(PREFIX):]
    config=reload_distances(config,stage)
    geometry=state.get('geometry')
    if geometry is None:raise ValueError('V4.1 has no selected book from this cycle')
    pose=state['pose_reader'](robot,step5)
    if pose is None:return task.get('object'),False
    start,angles=pose
    if stage=='align_y':state['angles']=np.asarray(angles).copy();state['path']=0.
    state['distance_snapshot']=dict(config)
    display=state.get('display')
    if display is not None:
        from book_top_v41_distances import DISTANCE_KEYS
        display.safe('log_json_document','book/v41_distances',dict(stage=stage,
                     parameters={key:config[key] for key in sorted(DISTANCE_KEYS)}))
        if getattr(display,'logger',None) is not None:
            try:log_upper_edge(display.logger,geometry,config)
            except Exception as error:print(f'V4.1 距离显示警告：{error}',flush=True)
    angles=state['angles'];rotation=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
    offset=stage_offset(stage,start,rotation,geometry,config)
    # The inherited planner takes absolute TCP poses; convert only this stage.
    target=np.asarray(start,float)+offset
    if stage=='forward_x':
        print('V4.1 前伸计算：'+json.dumps(dict(book_median_x_mm=geometry['book_median_x_mm'],
              insertion_mm=config['forward_insertion_mm'],
              fingertip_x_offset_mm=float((rotation@np.asarray(config['tcp_to_finger_tool_mm']))[0]),
              goal_tcp_x_mm=float(target[0]),forward_mm=float(offset[0])),ensure_ascii=False),flush=True)
    print('V4.1 相对位移：'+json.dumps(dict(stage=stage,start_tcp_mm=np.asarray(start).tolist(),
          offset_mm=offset.tolist(),planner_tcp_mm=target.tolist()),ensure_ascii=False),flush=True)
    distance=float(np.linalg.norm(target-start))
    if state['path']+distance>config['max_path_mm']+1e-6:raise ValueError('V4.1 coarse path budget exceeded')
    limit=config['descent_step_mm'] if stage=='lower_z' else config['coarse_step_mm']
    old_wait=robot._wait_for_robot_stop
    def wait(*args,**kwargs):
        kwargs.update(position_tolerance_m=config['position_tolerance_mm']/1000.,
                      orientation_tolerance_rad=np.deg2rad(config['orientation_tolerance_deg']),
                      require_fresh_stable=True,max_wait_time=60.,check_interval=.02)
        return old_wait(*args,**kwargs)
    robot._wait_for_robot_stop=wait
    try:
        for point in segments(start,target,limit):
            getattr(robot,'operator_checkpoint',lambda:None)()
            # Custom ordinary action avoids V1_3's automatic pre-grasp opening
            # and its +100 mm grasp insertion, while using the same executor.
            waypoint=dict(task,target_position=point.tolist(),target_rotation=angles.tolist())
            display=state.get('display')
            if display is not None:display.safe('log_book_target',waypoint)
            result=execute(robot,waypoint,index,objects)
            if result[1] is not True:return task.get('object'),False
        state['path']+=distance
    finally:robot._wait_for_robot_stop=old_wait
    return task.get('object'),True


@contextmanager
def installed(config,through_step=6,*,raise_servo=None):
    """Install inside V3's motion hooks, before V1_3.main installs perception."""
    import cart_book_grasp_v1_3_motion as motion
    import cart_book_grasp_v1_3_visualization as vis
    saved={k:getattr(motion,k) for k in ('_install_spine_center','_install_right_arm_reset','_keep_cart_plus_grasp_only')}
    old_preview=vis.preview_targets
    entry_execute=getattr(sys.modules.get('dual_v2_2_agent_pcl'),'execute_main',None)
    state=dict(geometry=None,display=None,path=0.,pose_reader=motion._right_tool_mm_euler)
    restored=[]

    class Capture:
        def __init__(self,display):self.display=display
        def __getattr__(self,key):return getattr(self.display,key)
        def safe(self,*a,**kw):
            if self.display is not None:return self.display.safe(*a,**kw)
        def update_geometry(self,*a):
            if self.display is not None:return self.display.update_geometry(*a)
        def capture(self,rgb,depth,k,c2w,selected,book_mask):
            g=top_geometry(rgb,depth,k,c2w,selected,book_mask)
            state['geometry']=g
            print('V4.1 复用感知上沿：'+json.dumps(g,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else v,
                                                    ensure_ascii=False),flush=True)
            if self.display is not None:
                self.display.capture(rgb,depth,k,c2w,selected,book_mask)
                self.display.safe('log_json_document','book/v41_upper_edge',g)

    def spine(geometry,visualization=None,**kwargs):
        state['display']=visualization
        if raise_servo is not None:
            from cart_book_grasp_v3_wrist_center import detector_from_geometry
            state['wrist_detector']=detector_from_geometry(geometry)
        def selection_review(selected,book_mask,rgb,depth,k,c2w,*,box_detector=None,observation_robot=None):
            from book_top_v41_selection import review_distance
            import step5_execute as step5
            return review_distance(selected,book_mask,rgb,depth,k,c2w,
                pose_reader=lambda:state['pose_reader'](observation_robot,step5),
                config=reload_distances(config),box_detector=box_detector,
                checkpoint=getattr(observation_robot,'operator_checkpoint',lambda:None))
        kwargs['selection_review']=selection_review
        saved['_install_spine_center'](geometry,Capture(visualization),**kwargs)
        segment=geometry.segment_cart_book_with_sam3
        def current(*args,**kw):
            state['geometry']=None;state['path']=0.
            if raise_servo is not None:
                state['wrist_detector'].box_detector=kw.get('box_detector')
            return segment(*args,**kw)
        for key in ('gpt_spine_fallback','gpt_polygon_fallback'):
            if hasattr(segment,key):setattr(current,key,getattr(segment,key))
        geometry.segment_cart_book_with_sam3=current

    def runtime():
        import dual_v2_2_agent_pcl as agent
        import step5_execute as step5
        for obj,keys in ((agent,('grounding_main','execute_main','RerunPipelineLogger')),
                         (step5,('execute_single_task','execute_main','recover_to_reset_after_failure'))):
            for key in keys:
                if hasattr(obj,key):
                    value=entry_execute if obj is agent and key=='execute_main' and entry_execute is not None else getattr(obj,key)
                    restored.append((obj,key,value))
        ordinary=step5.execute_single_task
        grounding=agent.grounding_main
        def close_then_ground(*args,**kwargs):
            robot=kwargs.get('robot',args[4] if len(args)>4 else None)
            state['geometry']=None
            if robot is not None and config['close_fingers_after_reset']:
                result=ordinary(robot,dict(arm='right',object='object_0',action='close_gripper',
                    book_grasp_mode='cart+',description='V4.1 close fingers after first_reset'),0,{'object_0':'book'})
                if result[1] is not True:raise ValueError('V4.1 inherited close_gripper failed')
            return grounding(*args,**kwargs)
        # Original V1_3 grounding wrapper resets first, then calls this hook.
        agent.grounding_main=close_then_ground
        saved['_install_right_arm_reset']()
        logger_class=getattr(agent,'RerunPipelineLogger',None)
        if logger_class is not None and hasattr(logger_class,'log_book_perception'):
            original_perception=logger_class.log_book_perception
            def log_perception(logger):
                original_perception(logger)
                display=state.get('display')
                if state['geometry'] is not None and display is not None and display.frame is not None:
                    log_upper_edge(logger,state['geometry'],reload_distances(config))
            logger_class.log_book_perception=log_perception
        if logger_class is not None and hasattr(logger_class,'log_book_preview'):
            original_preview=logger_class.log_book_preview
            def log_preview(logger):
                original_preview(logger)
                logger.log_text_document('book/motion',
                    'V4.1: closed fingers; relative Y align, Z upper-edge gap + clearance, X book median plus insertion minus finger X offset and current TCP, Z descent to configured upper-edge reference. '
                    'Yellow: nominal TCP preview. Orange: issued targets. Cyan: measured TCP. '
                    'No automatic release/withdraw. Position arrival does not confirm contact.')
            logger_class.log_book_preview=log_preview
        original=step5.execute_single_task
        def execute(robot,task,index,objects,**kwargs):
            if task.get('action')==PREFIX+'raise_z' and raise_servo is not None:
                return raise_servo(robot,task,index,objects,ordinary,step5,state,config)
            if str(task.get('action','')).startswith(PREFIX):
                return execute_stage(robot,task,index,objects,ordinary,step5,state,config)
            return original(robot,task,index,objects,**kwargs)
        step5.execute_single_task=execute

    def preview(start,angles,*args,**kwargs):
        if state['geometry'] is None:return old_preview(start,angles,*args,**kwargs)
        preview_config=reload_distances(config)
        q=np.asarray(start,float).copy();points=[q.copy()]
        stages=STAGES[:through_step-1]
        rotation=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
        for stage in stages:
            q=q+stage_offset(stage,q,rotation,state['geometry'],preview_config);points.append(q.copy())
        points=np.asarray(points)
        return dict(points_mm=points,labels=['nominal start',*stages],
                    grasp_mm=points[-1],parameters=dict(preview_config),rotation_deg=angles)

    motion._install_spine_center=spine
    motion._install_right_arm_reset=runtime
    def upper_edge_tasks(tasks):
        result=task_list(tasks,through_step)
        current_config=reload_distances(config)
        print(f"V4.1 实际任务：Y对齐→抬Z（上沿高度差+{current_config['hover_above_top_mm']:g}mm）→X书深度中位数+伸入量−指尖X偏移−当前TCP"
              +('→'+descent_description(current_config) if through_step>=5 else '')
              +(f"→沿-X抽回{current_config['withdraw_mm']:g}mm" if through_step>=6 else '')
              +'；完成后保持闭指和当前位置，等待回车开始下一轮。',flush=True)
        return result
    motion._keep_cart_plus_grasp_only=upper_edge_tasks
    vis.preview_targets=preview
    try:yield state
    finally:
        for obj,key,value in reversed(restored):setattr(obj,key,value)
        for key,value in saved.items():setattr(motion,key,value)
        vis.preview_targets=old_preview


def main(argv=None,*,raise_servo=None):
    import argparse
    from book_top_model_coarse import CONFIG
    parser=argparse.ArgumentParser(description='V4.1: inherit V3.4 SAM3/loop/p/s/planner; upper-edge actions only')
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--execute',action='store_true');mode.add_argument('--check-config',action='store_true')
    parser.add_argument('--config',type=Path,default=CONFIG)
    parser.add_argument('--through-step',type=int,choices=(4,5,6),default=6)
    parser.add_argument('--no-viewer',action='store_true')
    args,remaining=parser.parse_known_args(argv)
    c=load_config(args.config)
    if '--target' in remaining:parser.error('本入口与V3.4一样自动选一本书，不使用--target')
    if args.check_config:
        print('V4.1：复用V3.4/V1_3主循环、SAM3→GPT备用、Rerun、p复位、s停止及10195/10196 V4.1规划器。')
        reload_distances(c,'check-config')
        print(f"动作：first_reset→闭指→Y对齐→Z上沿差+{c['hover_above_top_mm']:g}mm→X书深度中位数+伸入量−指尖X偏移−当前TCP→{descent_description(c)}→沿-X抽回{c['withdraw_mm']:g}mm→保持等回车；未连接设备。")
        return 0
    if args.no_viewer:remaining.append('--no-rerun-viewer')
    # Earlier V4.1 used the saved head calibration; preserve that launch mode.
    if not any(x in remaining for x in ('--assume-head-camera-calibrated','--head-camera-correction','--head-camera-correction-only')):
        remaining.append('--assume-head-camera-calibrated')
    sys.path.insert(0,str(ROOT.parent/'dual_v2_2_control/cuarm_upper_planner/scripts'))
    from v4_1_planner_readiness import require_ready
    require_ready()
    import dual_v2_2_agent_pcl as agent
    from cart_book_grasp_v1_3 import main as v13_main
    from cart_book_grasp_v3_motion import installed as v3_installed
    from cart_book_grasp_v4_1_ports import dedicated_planner
    from cart_book_grasp_v3_4_policy import load_stream_config
    from head_rgbd.client import CalibratedCamera
    saved_argv=sys.argv;camera_factory=agent.start_calibrated_orbbec
    sys.argv=[saved_argv[0],*remaining]
    agent.start_calibrated_orbbec=lambda calibration,warmup_frames=5:CalibratedCamera(calibration,warmup_frames=warmup_frames,prefer_native=True)
    try:
        print('V4.1 继承V3.4：SAM3优先，GPT备用；p保持/松爪/复位，s本轮结束后停止。',flush=True)
        with dedicated_planner(agent),v3_installed(None,stream_config=load_stream_config()),installed(c,args.through_step,raise_servo=raise_servo):
            v13_main(config_path=ROOT/'cart_book_grasp_v3_config.json',wrist_config_path=ROOT/'cart_book_grasp_v3_wrist_config.json',wait_for_enter_after_success=True)
    finally:
        sys.argv=saved_argv;agent.start_calibrated_orbbec=camera_factory
    return 0
