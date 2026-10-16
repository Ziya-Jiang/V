"""Offline CuRobo FK buffer isolation check. No planner, camera or UDP is started.

Run explicitly with the curobo Python environment; requires a CUDA GPU.
"""
import ast
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import numpy as np
import torch


def main():
    root=Path(__file__).resolve().parents[1]
    scripts=root/'dual_v2_2_control/cuarm_upper_planner/scripts'
    sys.path.insert(0,str(scripts))
    from curobo_config import CuroboConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import RobotConfig
    from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
    from curobo.types.state import JointState
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig,MotionGenResult
    config=CuroboConfig.load_from_yaml_path(str(root/'dual_v2_2_configuration/config.yaml'))
    tensor_args=TensorDeviceType()
    model_config=RobotConfig.from_dict(config.robot_cfg,tensor_args).kinematics
    # Exercise the initializer used by the actual planner, without constructing
    # MotionGen or any of its communication threads.
    tree=ast.parse((scripts/'curobo_planner_v3_1.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CuroboPlanner')
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('_initialize_fk_models','_make_stream_plan_config')]
    ns={'CudaRobotModel':CudaRobotModel}
    exec(compile(ast.Module(body=methods,type_ignores=[]),str(scripts/'curobo_planner_v3_1.py'),'exec'),ns)
    p=SimpleNamespace();ns['_initialize_fk_models'](p,model_config)
    feedback,path=p._feedback_kinematics,p._path_kinematics
    assert feedback._link_pos_seq.data_ptr()!=path._link_pos_seq.data_ptr()
    barrier=threading.Barrier(2)
    def exercise(model,batches,value):
        dof=model.get_dof()
        q=tensor_args.to_device(np.full((1,dof),value,dtype=np.float32))
        expected=model.get_link_poses(q,link_names=config.target_link_names).position.cpu().numpy().copy()
        barrier.wait(timeout=10.)
        for i in range(200):
            batch=batches[i%len(batches)]
            out=model.get_link_poses(q.repeat(batch,1),link_names=config.target_link_names)
            actual=out.position.cpu().numpy().copy()
            assert actual.shape==(batch,len(config.target_link_names),3)
            np.testing.assert_allclose(actual,np.repeat(expected,batch,axis=0),atol=1e-6,rtol=1e-6)
        return 200
    with ThreadPoolExecutor(max_workers=2) as pool:
        f=pool.submit(exercise,feedback,[1],0.)
        g=pool.submit(exercise,path,[4,128,3,1],.01)
        count=f.result(timeout=60.)+g.result(timeout=60.)
    torch.cuda.synchronize()
    print(f'PASS: {count} concurrent CuRobo FK calls; feedback batch=1; path batches=4/128/3/1; shapes and values match independent references. No robot connection.')
    p._plan_config=MotionGenPlanConfig(time_dilation_factor=.4)
    fast=ns['_make_stream_plan_config'](p)
    assert p._plan_config.time_dilation_factor==.4 and fast.time_dilation_factor==.9
    dts=[]
    for cfg in (p._plan_config,fast):
        zeros=tensor_args.to_device(np.zeros((6,feedback.get_dof()),np.float32))
        plan=JointState(position=zeros.clone(),velocity=zeros.clone(),acceleration=zeros.clone(),jerk=zeros.clone())
        result=MotionGenResult(optimized_plan=plan,optimized_dt=tensor_args.to_device([.2]),path_buffer_last_tstep=[6])
        result.retime_trajectory(cfg.time_dilation_factor,interpolate_trajectory=False)
        dts.append(float(result.optimized_dt.item()))
    np.testing.assert_allclose(dts,[.5,.2/.9],atol=1e-6)
    print(f'PASS: real CuRobo retiming dt={dts}; six-waypoint interval durations=2.50s / 1.11s. This does not measure robot execution speed.')


if __name__=='__main__':main()
