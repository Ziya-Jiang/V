"""Actual local RT RAW filter, protocol parameter isolation; no sockets."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'dual_v2_2_control/cuarm_upper_planner/scripts'),
             str(ROOT/'dual_v2_2_configuration/robot_platform_utils/python/scripts')]
from v3_1_panel_path import RawPanel
from v3_1_direct_servo import DirectVisualServo,load_settings
from cuarm_state import RemotePanelCommand,TargetCommand,CuroboCommand,PanelTargetMode
from cuarm_utils import convert_curobo_command_to_remote_panel_command


class RawTests(unittest.TestCase):
    def test_streaming_time_ramp_vs_zero_using_real_cpp(self):
        distances=[]
        # Isolate ONLY the time parameter, with the same speed limit and lead.
        for target_time in (5.,0.):
            with RawPanel([0.],[.1745],target_time=target_time) as raw:
                q=0.
                for _ in range(150):
                    q=float(raw.advance([min(.2,q+.025)],20)[-1,0])
                distances.append(q)
        self.assertLess(distances[0],.02)
        self.assertGreater(distances[1],5*distances[0])
        print(f'RAW C++ 3-second streaming comparison: T=5s {distances[0]:.6f}rad; T=0 {distances[1]:.6f}rad')

    def test_zero_time_keeps_filter_and_velocity_cap(self):
        vmax=.1745
        with RawPanel([0.],[vmax],target_time=0.) as raw:
            path=raw.advance([1.],2000)[:,0]
        self.assertLess(path[0],.001)  # no direct position jump
        self.assertLess(np.max(np.abs(np.diff(path)))/.001,vmax)
        self.assertLess(path[-1],.2)

    def test_visual_settings_and_return_to_ordinary_waypoint(self):
        panel=RemotePanelCommand();target=TargetCommand()
        cmd=CuroboCommand(arm_joint_command=[[0.]*7]*2,gripper_joint_command=[[.9],[.2]],
                         waypoints_size=2,waypoints_dt=.5,waypoints=[[[0.]*7]*2]*2)
        convert_curobo_command_to_remote_panel_command(panel,cmd,target,PanelTargetMode.RAW_SINGLE_POINT)
        DirectVisualServo.configure_panel(NS(c=load_settings()),panel)
        self.assertEqual(panel.interpolation_acceleration_time,0.)
        self.assertEqual(panel.no_interpolation_max_velocity_ratio,.3)
        # Reusing the actual panel object must restore ordinary action values.
        convert_curobo_command_to_remote_panel_command(panel,cmd,target,PanelTargetMode.WAYPOINT)
        self.assertEqual(panel.interpolation_acceleration_time,5.)
        self.assertEqual(panel.no_interpolation_max_velocity_ratio,.1)
        self.assertEqual(panel.gripper_joint_command,[[.9],[.2]])


if __name__=='__main__':unittest.main()
