import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from contextlib import contextmanager
from unittest.mock import Mock,patch
import cv2
import numpy as np
from cart_grasp import main,dictionary
from cart_grasp_alignment import run,CONFIG


class WristOnlyTests(unittest.TestCase):
    def frame(self):
        image=np.full((720,1280,3),255,np.uint8)
        marker=cv2.aruco.generateImageMarker(dictionary(),0,240)
        image[240:480,520:760]=cv2.cvtColor(marker,cv2.COLOR_GRAY2RGB)
        return dict(rgb=image,captured_at=1.)
    def args(self,out,execute):
        return SimpleNamespace(config=CONFIG,out=out,wrist_only=True,approach=False,execute=execute)
    def test_modes_mutually_exclusive_before_hardware(self):
        with self.assertRaises(SystemExit) as error:
            main(['align','--wrist-only','--approach','--execute'])
        self.assertEqual(error.exception.code,2)
    def test_preview_never_opens_robot_or_head(self):
        camera=Mock();camera.capture.return_value=self.frame()
        with tempfile.TemporaryDirectory() as d, \
            patch('servor.robot_connection',side_effect=AssertionError('robot opened')), \
            patch('cart_grasp_alignment.read_head',side_effect=AssertionError('head opened')), \
            patch('cart_book_grasp_v3_camera.WristCamera',return_value=camera):
            out=Path(d)/'preview'
            self.assertEqual(run(self.args(out,False)),0)
            events=[json.loads(line) for line in (out/'events.jsonl').read_text().splitlines()]
            self.assertEqual(events[-1]['robot_commands_sent'],0)
            self.assertFalse(events[-1]['head_camera_used'])
            camera.close.assert_called_once()
    def test_execute_skips_coarse_and_keeps_original_xz(self):
        camera=Mock();camera.capture.return_value=self.frame()
        robot=Mock();transport=Mock()
        transport.read.return_value=dict(q=np.array([500.,-50.,10.]),roll=0.)
        @contextmanager
        def connection():yield robot,np.array([-90.,0.,90.])
        def loop(read,observe,send,ready,hold,origin,c,log,**kwargs):
            np.testing.assert_array_equal(origin,[500.,-50.,10.])
            self.assertIsNone(kwargs['goal_x']);self.assertIsNone(kwargs['approach'])
            send(origin,0.,False)
            log(dict(event='cart_aligned'))
        with tempfile.TemporaryDirectory() as d, \
            patch('servor.robot_connection',connection), \
            patch('cart_book_grasp_v3_camera.WristCamera',return_value=camera), \
            patch('cart_book_grasp_v3_2_runtime.StreamTransport',return_value=transport), \
            patch('cart_grasp_alignment.read_head',side_effect=AssertionError('head opened')), \
            patch('cart_grasp_alignment.coarse_move',side_effect=AssertionError('coarse move')), \
            patch('cart_grasp_alignment.align_loop',side_effect=loop):
            robot.master_control.return_value=True
            robot.target_command.arm_tool_command=np.zeros((2,7))
            out=Path(d)/'execute'
            self.assertEqual(run(self.args(out,True)),0)
            plan=json.loads((out/'plan.json').read_text())
            self.assertFalse(plan['head_camera_used']);self.assertIsNone(plan['head'])
            call=robot.master_control.call_args.kwargs
            self.assertEqual(call['action'],'book_v3_1_stream')
            self.assertNotIn('right_gripper',call);self.assertNotIn('left_arm',call)
            np.testing.assert_array_equal(np.array(call['right_arm'])[[0,2]],[500.,10.])
            camera.close.assert_called_once()

if __name__=='__main__':unittest.main()
