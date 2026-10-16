"""Explicit OFFLINE CUDA/CPU/interpolator verification. Never imports a robot client.

No socket, camera, running planner or hardware command is constructed.
Run with the installed curobo Python environment on a CUDA-capable host.
"""
from pathlib import Path
import sys
import time
import threading
from types import SimpleNamespace
import numpy as np
import torch


def main():
    import argparse
    args=argparse.ArgumentParser(description=__doc__)
    args.add_argument('--lag',action='store_true',help='Simulate 60 ms command delay and unequal joint tracking lag')
    args.add_argument('--raw',action='store_true',help='Also run the real RT RAW filter before the lagged plant')
    args.add_argument('--recover-z',action='store_true',help='Inject a persistent joint tracking bias during approach')
    options=args.parse_args();lag=options.lag or options.raw
    if options.recover_z and not options.raw:args.error('--recover-z requires --raw')
    root=Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(root/'dual_v2_2_control/cuarm_upper_planner/scripts'))
    from curobo_config import CuroboConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import RobotConfig
    from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
    from curobo.wrap.reacher.motion_gen import MotionGen,MotionGenConfig,MotionGenPlanConfig
    from curobo.types.state import JointState
    from curobo.types.math import Pose
    from curobo.geom.sdf.world import CollisionCheckerType
    from v3_1_cpu_fk import CpuKinematics
    from v3_1_panel_path import PanelPath
    from v3_1_motion_bridge import MotionBridge
    c=CuroboConfig.load_from_yaml_path(str(root/'dual_v2_2_configuration/config.yaml'))
    ta=TensorDeviceType();rc=RobotConfig.from_dict(c.robot_cfg,ta)
    cpu=CpuKinematics(rc.kinematics.kinematics_config);gpu=CudaRobotModel(rc.kinematics)
    names=rc.kinematics.kinematics_config.joint_names
    q=np.random.default_rng(12).uniform(-.4,.4,(300,len(names)))
    cp,cq=cpu.poses(q,c.target_link_names)
    gp=gpu.get_link_poses(ta.to_device(q),link_names=c.target_link_names)
    max_error=np.max(np.abs(cp-gp.position.cpu().numpy()))
    dots=np.abs((cq*gp.quaternion.cpu().numpy()).sum(-1))
    assert max_error<1e-6 and np.min(dots)>1.-1e-6,(max_error,np.min(dots))
    print(f'PASS: 300 CPU/GPU FK comparisons, max position component error={max_error*1000:.6f}mm')
    recorded=np.array([-.007885,.00326,.002613,-.002805,-.006927,.00067,.00012,
        1.775319,1.236196,-1.199982,1.850888,1.384346,-.35061,-.379846,
        .055,.02,.08,.08,.08,.08,.00198])
    order=c.total_arm_joint_names+c.total_gripper_joint_names
    q=np.array([recorded[order.index(n)] for n in names])
    t=time.perf_counter()
    for _ in range(1000):cpu.poses(q,c.target_link_names)
    print(f'PASS: 1000 CPU feedback FK calls in {time.perf_counter()-t:.3f}s; no GPU lock')
    cfg=MotionGenConfig.load_from_robot_config(rc,c.world_cfg,ta,trajopt_tsteps=10,
        trim_steps=[2,-2],minimum_trajectory_dt=.2,interpolation_dt=.02,use_cuda_graph=True,
        collision_checker_type=CollisionCheckerType.MESH,collision_activation_distance=0.,
        position_threshold=.001,rotation_threshold=.1,self_collision_check=True,self_collision_opt=True)
    motion=MotionGen(cfg)
    config=MotionGenPlanConfig(check_start_validity=False,time_dilation_factor=.9,max_attempts=2)
    def plan(dx):
        poses=gpu.get_link_poses(ta.to_device(q[None]),link_names=c.target_link_names)
        positions=poses.position.clone();positions[0,1,0]+=dx
        goals={name:Pose(position=positions[:,i],quaternion=poses.quaternion[:,i]) for i,name in enumerate(c.target_link_names)}
        js=JointState(position=ta.to_device(q[None]),velocity=ta.to_device(np.zeros_like(q[None])),
            acceleration=ta.to_device(np.zeros_like(q[None])),jerk=ta.to_device(np.zeros_like(q[None])),joint_names=names)
        result=motion.plan_single(js,goals[c.ee_link_name],config,link_poses=goals)
        assert result.success.item(),result.status
        return result
    result=plan(.02)
    points=result.optimized_plan.get_ordered_joint_state(c.total_arm_joint_names).position.cpu().numpy()
    # The unrequested left arm is held exactly as in the real command path.
    points[:,:7]=recorded[:7]
    p=SimpleNamespace(config=c.config,target_link_names=c.target_link_names,end_effector_offset=c.end_effector_offset,
        target_gripper_joint=recorded[14:].tolist(),_gpu_lock=threading.Lock(),_motion_gen=motion,tensor_args=ta,
        reorder_input=lambda arm,grip=None,as_curobo=True:np.r_[arm,grip] if grip is not None and len(grip) else arm)
    bridge=MotionBridge(p,cpu,PanelPath())
    position,orientation=bridge.tcp(recorded[:14]);p.target_arm_tool_position=position[0].copy()
    p._panel_feedback_source=lambda:(SimpleNamespace(arm_joint_position=recorded[:14].reshape(2,7),
        arm_joint_velocity=np.zeros((2,7))),time.monotonic())
    accepted=bridge.validate(points,float(result.optimized_dt.item()))
    # A rejection is an acceptable validator outcome, but independently run
    # actual rollout batches to verify mutable buffers across planning calls.
    for n in (1,128,3,64,1):
        metrics=motion.rollout_fn.rollout_constraint(ta.to_device(np.tile(q,(n,1)))[:,None],use_batch_env=False)
        assert metrics.feasible.shape[0]==n
        assert metrics.feasible.all().item(),'recorded pose failed collision/limit check'
    plan(.01)
    print(f'PASS: real MotionGen plan -> real panel interpolation -> CPU height/retreat + GPU collision gates (accepted={accepted}, reason={bridge.failure}); replan after batches 1/128/3/64/1 succeeds')
    # Exercise the actual planner adapter as well, with thread startup disabled
    # and synthetic PanelState injected. No communication object is constructed.
    sys.path[:0]=[str(root/'dual_v2_2_configuration/robot_platform_utils/python/scripts'),
                  str(root/'dual_v2_2_configuration/curi_udp/python')]
    from unittest.mock import patch
    from curobo_planner_v3_1 import CuroboPlanner
    from cuarm_state import (PlannerConnectionMethod,RobotState,TargetCommand,
                             TargetCommandGripperCommandType,CuroboStatus)
    c=CuroboConfig.load_from_yaml_path(str(root/'dual_v2_2_configuration/config.yaml'))
    with patch('threading.Thread.start',lambda self:None):
        adapter=CuroboPlanner(c,PlannerConnectionMethod.REMOTE_TARGET_AND_LOCAL_ROBOT,
            catch_SIGINT=False,skip_warmup=True,trajopt_tsteps=10,trim_steps=[2,-2],
            time_dilation_factor=.4,solver_position_threshold=.001,solver_rotation_threshold=.1,soft_limit_k=.99)
    adapter._curobo_state_sending_thread=None
    try:
        adapter._panel_feedback_source=p._panel_feedback_source
        # Exercise real protocol ingestion/conversion, not ndarray assignment:
        # live gripper feedback becomes list before entering stream planning.
        def convert_grip(prefix):
            return adapter.gripper_convertion(recorded[14:].tolist(),
                adapter.curobo_used_gripper_type,adapter.curobo_used_gripper_urdf_order,
                adapter.curobo_used_gripper_normalized,adapter.curobo_used_gripper_zero_open,
                TargetCommandGripperCommandType.JOINT,
                getattr(adapter,prefix+'_urdf_order'),getattr(adapter,prefix+'_normalized'),
                getattr(adapter,prefix+'_zero_open'))
        grip=convert_grip('robot_sent_gripper')
        robot_state=RobotState(arm_size=2,arm_joint_size=[7,7],
            arm_joint_position=recorded[:14].reshape(2,7).tolist(),arm_joint_velocity=np.zeros((2,7)).tolist(),
            gripper_size=2,gripper_joint_size=[6,1],gripper_joint_position=[grip[:6],grip[6:]])
        adapter._retrieve_robot_state(robot_state)
        assert isinstance(adapter.gripper_joint_position,list)
        np.testing.assert_allclose(adapter.gripper_joint_position,recorded[14:],atol=1e-8)
        position,orientation=adapter._v31_bridge.tcp(recorded[:14])
        adapter.arm_tool_position=position[0].copy();adapter.arm_tool_orientation=orientation[0].copy()
        target_position=position[0].copy();target_position[1,0]+=.02
        grip=convert_grip('gripper_input')
        target=TargetCommand(arm_size=2,arm_joint_size=[7,7],
            arm_tool_command=np.c_[target_position,orientation[0]].tolist(),
            gripper_size=2,gripper_joint_size=[6,1],gripper_command_type=TargetCommandGripperCommandType.JOINT,
            gripper_joint_command=[grip[:6],grip[6:]],description='book_v3_1_stream')
        adapter._retrieve_target_command(target);adapter.last_received_target_command=target
        from v3_1_execution import ExecutionGuard
        from cuarm_state import RemotePanelCommand,PanelTargetMode
        from cuarm_utils import convert_curobo_command_to_remote_panel_command
        from cuarm_udp import CuarmUdp
        rng=np.random.default_rng(19)
        samples=np.tile(q,(64,1));samples[:,:14]+=rng.uniform(-.15,.15,(64,14))
        gpu_valid=adapter._motion_gen.rollout_fn.rollout_constraint(ta.to_device(samples)[:,None],use_batch_env=False).feasible.cpu().numpy().reshape(-1)
        cpu_valid=np.array([adapter._v31_direct.collision.check(x) for x in samples])
        np.testing.assert_array_equal(cpu_valid,gpu_valid)
        print('PASS: 64 CPU sphere/self/world/limit checks match GPU constraint checks')
        collision=adapter._v31_direct.collision
        saved_boxes=collision.boxes
        collision.boxes=[(np.zeros(3),np.eye(3),np.ones(3)*10)]
        assert not collision.check(q),'enclosing obstacle must reject pose'
        collision.boxes=saved_boxes
        outside=q.copy();outside[0]=collision.bounds[1,0]+1
        assert not collision.check(outside),'joint bounds must reject pose'
        # Simulated joint plant, real controller/codec; never create a socket.
        t=[100.];plant=recorded[:14].copy();velocity=np.zeros(14);sent=[]
        adapter._panel_feedback_source=lambda:(SimpleNamespace(arm_joint_position=plant.reshape(2,7),
            arm_joint_velocity=velocity.reshape(2,7),gripper_joint_position=robot_state.gripper_joint_position),t[0])
        adapter._target_udp=SimpleNamespace(receive_with_timestamp=lambda:(target,t[0]),terminate=lambda:None)
        adapter._v31_bridge.guard=ExecutionGuard(sent.append,clock=lambda:t[0],log=lambda _:None)
        panel=RemotePanelCommand(arm_size=2,arm_joint_size=[7,7],gripper_size=2,gripper_joint_size=[6,1],
                                 arm_target_mode=PanelTargetMode.RAW_SINGLE_POINT)
        class ForbiddenMotionGen:
            def __getattr__(self,name):raise AssertionError('Visual phase called CuRobo: '+name)
        saved_motion=adapter._motion_gen;adapter._motion_gen=ForbiddenMotionGen()
        z=[];elapsed=[];queue=[plant.copy() for _ in range(3)]
        from v3_1_panel_path import RawPanel
        raw=(RawPanel(plant,adapter._v31_bridge.limits.reshape(-1)*adapter._v31_direct.c['raw_velocity_ratio'])
             if options.raw else None)
        raw_z=[];joint_bias=np.zeros(14);recovery_ticks=0
        try:
            with patch('time.monotonic',side_effect=lambda:t[0]):
                for tick in range(750):
                    if options.recover_z and tick==550:
                        # Persistent tracking/load bias, selected through the
                        # actual local Jacobian to reproduce a +2.04 mm height
                        # deviation (including the existing tracking error).
                        eps=1e-4;perturbed=np.tile(plant,(7,1))
                        perturbed[np.arange(7),7+np.arange(7)]+=eps
                        p0,_=adapter._v31_bridge.tcp(plant)
                        pp,_=adapter._v31_bridge.tcp(perturbed)
                        jac=(pp[:,1]-p0[0,1]).T/eps
                        dz=target_position[1,2]+.00204-p0[0,1,2]
                        joint_bias[7:]=np.linalg.pinv(jac)@np.array([0.,0.,dz])
                        plant+=joint_bias
                    if tick in (250,500):
                        # Change Y/roll while active, then approach another 80 mm.
                        if tick==250:
                            from scipy.spatial.transform import Rotation
                            target_position[1,1]-=.02
                            rotation=Rotation.from_rotvec([np.deg2rad(2),0,0])*Rotation.from_quat(orientation[0,1,[1,2,3,0]])
                            orientation[0,1]=rotation.as_quat()[[3,0,1,2]]
                        else:target_position[1,0]+=.08
                        target.arm_tool_command=np.c_[target_position,orientation[0]].tolist()
                        adapter._retrieve_target_command(target)
                    began=time.perf_counter();adapter.compute();elapsed.append(time.perf_counter()-began)
                    recovery_ticks+=int(adapter._v31_direct.recovering_z)
                    assert adapter.status==CuroboStatus.PLANNING_SUCCEEDED,adapter._v31_bridge.guard.fault
                    assert adapter.curobo_cmd.waypoints_size==0
                    convert_curobo_command_to_remote_panel_command(panel,adapter.curobo_cmd,target,PanelTargetMode.RAW_SINGLE_POINT)
                    adapter._v31_direct.configure_panel(panel)
                    assert panel.interpolation_acceleration_time==0.
                    b=adapter._v31_bridge
                    accepted=b.guard.publish(panel,b.checked_start,b.serial,requested=b.requested_target)
                    adapter._v31_direct.dispatched(accepted);assert accepted
                    CuarmUdp.pack_remote_panel_command(object.__new__(CuarmUdp),panel)
                    next_q=np.array(panel.arm_joint_command).reshape(-1)
                    if lag:
                        queue.append(next_q);delayed=queue.pop(0)
                        if raw is not None:
                            raw_path=raw.advance(delayed,20)
                            raw_z.extend(adapter._v31_bridge.tcp(raw_path)[0][:,1,2])
                            delayed=raw_path[-1]
                        # Distinct first-order joint responses, approx 60-125 ms
                        # time constants, plus the 60 ms transport queue.
                        alpha=np.tile([.25,.2,.16,.22,.18,.26,.15],2)
                        next_q=plant+alpha*(delayed+joint_bias-plant)
                    velocity=(next_q-plant)/.02;plant=next_q
                    adapter._refresh_feedback_pose();adapter._update_curobo_state()
                    assert adapter._v31_bridge.guard.fault is None,adapter._v31_bridge.guard.fault
                    np.testing.assert_array_equal(adapter.curobo_state.processed_arm_tool_command,target.arm_tool_command)
                    z.append(adapter.arm_tool_position[1,2]);t[0]+=.02
            position_error=np.linalg.norm(adapter.arm_tool_position[1]-target_position[1])*1000
            max_z=np.max(np.abs(np.array(z)-target_position[1,2]))*1000
            assert position_error<2. and max_z<(3. if options.recover_z else 2.),(position_error,max_z)
            if options.recover_z:
                assert recovery_ticks>0 and not adapter._v31_direct.recovering_z,recovery_ticks
                print(f'PASS: persistent Z tracking bias recovered; recovery_ticks={recovery_ticks}')
            if raw_z:
                raw_deviation=np.max(np.abs(np.asarray(raw_z)-target_position[1,2]))*1000
                # Recovery intentionally starts outside the 2 mm planning
                # envelope. Both filtered and measured paths must stay below
                # the unchanged 3 mm independent stop boundary in this test.
                assert raw_deviation<(3. if options.recover_z else 2.),raw_deviation
                print(f'PASS: actual RAW filtered path max Z deviation={raw_deviation:.3f}mm')
            # C++ biquad arithmetic on a constant input has roundoff (~1e-14).
            np.testing.assert_allclose(plant[:7],recorded[:7],rtol=0.,atol=1e-12)
            print(f'PASS: 750 visual servo ticks, raw={options.raw}, lag={lag}, Y/roll -> X target switching, NO CuRobo runtime calls; '
                  f'final_error={position_error:.3f}mm, max_Z={max_z:.3f}mm, '
                  f'mean_tick_ms={np.mean(elapsed)*1000:.2f}, p99_ms={np.percentile(elapsed,99)*1000:.2f}')
        finally:
            if raw is not None:raw.close()
            adapter._motion_gen=saved_motion
    finally:
        adapter.terminate()
    print('No UDP, camera or robot motion was started.')


if __name__=='__main__':main()
