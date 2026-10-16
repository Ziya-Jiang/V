"""Offline integration with the real RobotArm command builder; no sockets."""
import copy
import json
from contextlib import nullcontext
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

import numpy as np

ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'RobotRAM'),str(ROOT.parent/'dual_v2_2_control/cuarm_upper_planner/scripts')]
from controllor.hkstp_arm import RobotArm
from cuarm_state import TargetCommand,TargetCommandArmCommandType as ArmType,GripperMode
import book_top_v34_runtime as runtime
from cart_book_grasp_v4_1 import load_config


class Runtime(unittest.TestCase):
    def robot(self):
        robot=RobotArm.__new__(RobotArm)
        robot.udp=Mock();robot.target_command=TargetCommand();robot.ik_lib='curobo';robot.arm_size=2
        robot.target_command.gripper_size=2
        robot.target_command.arm_command_type=ArmType.JOINT
        robot.target_command.arm_joint_command=[[.01]*7,[.02]*7]
        robot.target_command.arm_tool_command=[[0.,0.,0.,1.,0.,0.,0.],[9.,9.,9.,1.,0.,0.,0.]]
        robot.target_command.description='first_reset:joint_reset:123'
        robot.target_command.gripper_mode_command=[GripperMode.OPEN,GripperMode.OPEN]
        robot.get_cat_gripper=lambda status,category:GripperMode.CLOSE if status=='close' else GripperMode.OPEN
        robot._wait_for_robot_stop=Mock(return_value=True)
        self.addCleanup(lambda:setattr(robot,'udp',None))
        return robot

    def state(self,stamp):
        return dict(received_monotonic=stamp,pose_sample_monotonic=stamp,
            curobo_status=9,robot_status=0,joint_velocity=[0.]*14,
            right_gripper=GripperMode.CLOSE,left_gripper=GripperMode.OPEN,
            left_arm=dict(position=[.04,1.17,.38],orientation=[1.,0.,0.,0.]),
            right_arm=dict(position=[.41,-.05,0.],orientation=[1.,0.,0.,0.]))

    def test_real_gripper_builder_preserves_joint_identity_and_left_hand(self):
        robot=self.robot();before=copy.deepcopy(robot.target_command);clock=[10.]
        robot.get_current_state=lambda:self.state(clock[0])
        fake_time=NS(monotonic=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+dt))
        with patch.object(runtime,'time',fake_time),patch('book_top_landing_runtime.actual_aperture',return_value=1.):
            runtime.close_fingers(robot,load_config(),Mock())
        sent=robot.udp.send.call_args.args[0]
        self.assertEqual(sent.arm_command_type,ArmType.JOINT)
        self.assertEqual(sent.description,before.description)
        np.testing.assert_allclose(sent.arm_tool_command,before.arm_tool_command)
        np.testing.assert_allclose(sent.arm_joint_command,before.arm_joint_command)
        self.assertEqual(sent.gripper_mode_command,[GripperMode.OPEN,GripperMode.CLOSE])
        robot._wait_for_robot_stop.assert_not_called();robot.udp.send.assert_called_once()

    def test_gripper_requires_measured_closure(self):
        robot=self.robot();clock=[10.];robot.get_current_state=lambda:self.state(clock[0])
        fake_time=NS(monotonic=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+dt))
        with patch.object(runtime,'time',fake_time),patch('book_top_landing_runtime.actual_aperture',return_value=0.):
            with self.assertRaisesRegex(ValueError,'closure timeout'):runtime.close_fingers(robot,load_config(),Mock())

    def test_real_cartesian_builder_uses_ordinary_actions_and_correct_units(self):
        for stage,action in runtime.STAGE_ACTIONS.items():
            robot=self.robot();s=self.state(10.);robot.get_current_state=lambda:s
            old_read=robot.get_current_state;old_wait=robot._wait_for_robot_stop
            io=NS(robot=robot,c=load_config(),rotation=np.eye(3),guard=lambda:s,log=Mock())
            io.c['landing']['coarse_position_tolerance_mm']=3.;io.c['landing']['coarse_timeout_seconds']=60.
            runtime.coarse_move(io,[450.,-50.,20.],stage)
            sent=robot.udp.send.call_args.args[0]
            self.assertEqual(sent.arm_command_type,ArmType.CARTESIAN)
            self.assertEqual(sent.description,action)
            self.assertNotIn(action,['book_v4_1_close','book_v4_1_coarse','book_v3_1_stream','book_v3_xyz_servo'])
            np.testing.assert_allclose(sent.arm_tool_command[1][:3],[.45,-.05,.02])
            np.testing.assert_allclose(sent.arm_tool_command[0][:3],s['left_arm']['position'])
            self.assertEqual(old_wait.call_args.kwargs['position_tolerance_m'],.003)
            self.assertTrue(old_wait.call_args.kwargs['require_fresh_stable'])
            self.assertEqual(robot._cartesian_commanded_arms,(1,))
            self.assertIs(robot.get_current_state,old_read);self.assertIs(robot._wait_for_robot_stop,old_wait)
            self.assertFalse(hasattr(robot,'_servo_initial_state'))

    def test_failed_motion_restores_wrappers_and_stops(self):
        robot=self.robot();s=self.state(10.);robot.get_current_state=lambda:s
        old_read=robot.get_current_state;old_wait=robot._wait_for_robot_stop
        old_wait.return_value=False
        io=NS(robot=robot,c=load_config(),rotation=np.eye(3),guard=lambda:s,log=Mock())
        with self.assertRaisesRegex(ValueError,'not confirmed'):
            runtime.coarse_move(io,[450.,-50.,0.],'align_y')
        self.assertIs(robot.get_current_state,old_read);self.assertIs(robot._wait_for_robot_stop,old_wait)

    def test_connection_reuses_v34_receipt_and_ports_only(self):
        robot=self.robot();robot.cleanup=Mock();s=self.state(10.)
        robot.get_current_state=lambda:s
        with patch('v3_1_readiness.require_ready') as ready,\
                patch('controllor.hkstp_arm.RobotArm',return_value=robot) as factory,\
                patch.object(runtime,'time',NS(monotonic=lambda:10.,sleep=Mock())),\
                patch('v4_1_readiness.require_ready',side_effect=AssertionError('wrong planner')):
            with runtime.connection() as connected:self.assertIs(connected,robot)
        ready.assert_called_once_with(10176)
        factory.assert_called_once_with(target_local_port=10175,target_remote_port=10176)
        robot.cleanup.assert_called_once()

    def test_full_five_step_entry_uses_v34_runtime_and_no_v41_backend(self):
        from book_top_model_coarse import run,load_config as task_config
        from scipy.spatial.transform import Rotation
        robot=self.robot();position=np.array([410.,-50.,0.]);events=[];targets=[]
        def state():
            s=self.state(10.);s['right_arm']['position']=(position/1000).tolist();return s
        robot.get_current_state=state
        io=NS(guard=state,rotation=Rotation.from_euler('y',-90,degrees=True).as_matrix(),
              cancel=Mock(),log=Mock(),observation_anchor=None,
              begin_head_observation=Mock(),end_head_observation=Mock())
        g=dict(spine_center_base_mm=[600.,-100.,20.],top_base_mm=[600.,-100.,100.],
               depth_center_base_mm=[600.,-100.,20.])
        reference=object()
        def detect(*args,**kw):
            events.append('detect')
            if events.count('detect')==2:self.assertIs(args[4],reference)
            return g,reference
        def move(io,target,stage):
            events.append(stage);targets.append((stage,np.array(target)));position[:]=target
        with tempfile.TemporaryDirectory() as td,\
                patch.object(runtime,'connection',return_value=nullcontext(robot)),\
                patch.object(runtime,'reset',side_effect=lambda *a:events.append('reset')),\
                patch.object(runtime,'close_fingers',side_effect=lambda *a:events.append('close')),\
                patch.object(runtime,'coarse_move',side_effect=move),\
                patch('book_top_model_coarse.RobotIO',return_value=io),\
                patch('book_top_model_coarse.wait_observation_ready',side_effect=lambda *a:state()),\
                patch('book_top_landing_runtime.actual_aperture',return_value=1.),\
                patch('cart_grasp_alignment.open_head_camera',return_value=Mock()),\
                patch('book_top_model_coarse.perceive',side_effect=detect),\
                patch('book_top_model_ui.CoarseVisualization'),\
                patch('book_top_tilt_servo.servo_connection',side_effect=AssertionError('wrong backend')):
            out=Path(td)/'run'
            run(NS(execute=True,through_step=5,output=out,no_viewer=True),task_config())
            result=json.loads((out/'result.json').read_text())
        self.assertEqual(events[:3],['reset','close','detect'])
        stages=[k for k in events if k not in ('reset','close','detect')]
        self.assertEqual(list(dict.fromkeys(stages)),['align_y','raise_z','forward_x','lower_z'])
        np.testing.assert_allclose(position,[550.,-100.,80.])
        self.assertEqual(result['completed_step'],5);self.assertFalse(result['contact_confirmed'])
        io.cancel.assert_called_once()

    def test_reset_reuses_v34_j6_hot_reload_and_restores_config_scope(self):
        import cart_book_grasp_v1_3_motion as motion
        robot=NS(right_safe_joint_deg=np.zeros(7));old=motion._CART_PLUS_CONFIG_PATH
        with patch('book_top_model_coarse.confirmed_reset') as reset:
            runtime.reset(robot,Mock())
        self.assertAlmostEqual(robot.right_safe_joint_deg[5],-20.3164)
        self.assertEqual(motion._CART_PLUS_CONFIG_PATH,old)
        reader=reset.call_args.kwargs['read_state']
        with patch.object(runtime,'read_state',return_value=dict(curobo_status=3)) as read:
            reader(robot);reader(robot)
        self.assertEqual([x.kwargs['check_backend'] for x in read.call_args_list],[False,True])


if __name__=='__main__':unittest.main()
