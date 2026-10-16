import io
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import threading
import time
import unittest
import cv2
import numpy as np
from .core import BOOT,CACHE,WORKSPACE,Renderer,atomic_json,read_state,ee_from_joints
from .state_relay import codec,decode,decode_panel,packets
from .live import Live
from .stream import CameraStream

class RuntimeTests(unittest.TestCase):
    def test_camera_capture_drains_latest_frame_and_reopens_after_failure(self):
        created=[]
        class FakeCamera:
            metadata={'serial':'test'}
            def __init__(self):self.count=0;self.closed=False;created.append(self)
            def read(self):
                time.sleep(.002);self.count+=1
                if self is created[0] and self.count>4:raise ValueError('Camera frame backlog > 300ms')
                return self.count,None,time.time()
            def close(self):self.closed=True
        stream=CameraStream(factory=FakeCamera,warmup=1,retry_delay=.005)
        try:
            with stream.condition:
                self.assertTrue(stream.condition.wait_for(lambda:len(created)>=2 and created[-1].count>=25,2))
            bgr,_,stamp,_=stream.read()
            self.assertGreaterEqual(bgr,25) # No queue of the previous 24 frames.
            self.assertTrue(created[0].closed)
            self.assertEqual(stream.status()['camera_reconnects'],1)
            self.assertLess(time.time()-stamp,.1)
        finally:stream.close()
        self.assertTrue(created[-1].closed)

    def test_live_restores_after_repeated_state_disconnects_without_restart(self):
        class FakeRenderer:
            urdf_hash='test'
            def configure_camera(self,metadata):pass
            def render(self,bgr,depth,height):return np.full_like(bgr,int(height))
        class FakeStream:
            def __init__(self):self.closed=False
            def read(self):
                time.sleep(.01)
                return np.zeros((4,4,3),np.uint8),np.ones((4,4)),time.time(),{'serial':'test'}
            def status(self):return {'camera_message':'相机持续采集中'}
            def close(self):self.closed=True
        def wait_for(predicate):
            limit=time.monotonic()+3
            while time.monotonic()<limit:
                if predicate():return
                time.sleep(.01)
            self.fail('Live display did not automatically recover')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'state.json';live=Live(path)
            def state(height,old=False):
                atomic_json(path,dict(boot_id=BOOT,captured_unix_s=time.time()-(10 if old else 0),
                    frame='dummy_world',reference='arm_end_effector_r',position_mm=[400,0,height],
                    source_kind='panel_measured',source='test',urdf_sha256='test'))
            worker=threading.Thread(target=live.run,args=(FakeRenderer,FakeStream));worker.start()
            try:
                state(10);wait_for(lambda:live.snapshot()['mode']=='live')
                for height in [40,80,120]:
                    before=live.snapshot();state(height,old=True)
                    wait_for(lambda:live.snapshot()['mode']=='paused')
                    self.assertEqual(live.snapshot()['image'],before['image'])
                    state(height)
                    wait_for(lambda:live.snapshot()['mode']=='live' and live.snapshot()['height_mm']==height)
                    self.assertNotEqual(live.snapshot()['image'],before['image'])
                    self.assertTrue(worker.is_alive())
            finally:live.stop.set();worker.join(2)
            self.assertFalse(worker.is_alive())

    def test_state_freshness_and_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'state.json'
            s=dict(boot_id=BOOT,captured_unix_s=100.,frame='dummy_world',reference='arm_end_effector_r',position_mm=[500,0,10])
            atomic_json(p,s);self.assertEqual(read_state(p,now=100.1)['position_mm'][2],10)
            for change,now in [({},101.),({},99.),({'boot_id':'old'},100.1),({'reference':'tool'},100.1),({'position_mm':[1,2]},100.1)]:
                atomic_json(p,{**s,**change})
                with self.assertRaises(ValueError):read_state(p,now=now)

    def test_packet_and_measured_joint_decode(self):
        c=codec()
        from cuarm_state import CuroboState,GripperMode
        ws=json.loads((WORKSPACE/'forward_workspace_data.json').read_text())
        q=ws['example_rad']
        s=CuroboState(arm_size=2,arm_joint_size=[7,7],arm_joint_position=[[0]*7,q],arm_joint_velocity=[[0]*7]*2,
            arm_tool_pose=[[0]*7]*2,processed_arm_joint_command=[[1]*7]*2,processed_arm_tool_command=[[0]*7]*2,
            gripper_size=2,gripper_joint_size=[1,1],gripper_joint_position=[[0],[0]],
            processed_gripper_mode_command=[GripperMode.OPEN]*2,processed_gripper_joint_command=[[0],[0]])
        raw=c.pack_curobo_state(s).encode();result=decode(raw,100.,c,ws)
        self.assertEqual(result['joints_rad'],q)
        p,R=ee_from_joints(q,ws['chain']);np.testing.assert_allclose(result['position_mm'],p)
        np.testing.assert_allclose(-R[:,2],[1,0,0],atol=1e-6)
        # Arbitrary ephemeral source port; never assume sender port 10166.
        udp=struct.pack('!HHHH',49152,10165,len(raw)+8,0)+raw
        ip=struct.pack('!BBHHHBBH4s4s',0x45,0,len(udp)+20,0,0,64,17,0,b'\x7f\0\0\1',b'\x7f\0\0\1')+udp
        eth=bytes(12)+b'\x08\x00'+ip
        pcap=struct.pack('<IHHIIII',0xa1b2c3d4,2,4,0,0,65535,1)+struct.pack('<IIII',100,250000,len(eth),len(eth))+eth
        got=list(packets(io.BytesIO(pcap)));self.assertEqual(got,[(100.25,raw)])
        wrong=pcap.replace(struct.pack('!H',10165),struct.pack('!H',10164));self.assertEqual(list(packets(io.BytesIO(wrong))),[])

    def test_water_brightness_uses_world_height_and_preserves_above(self):
        r=Renderer();r.field=np.full((4,4,4),25.)
        r.doc['grid']=dict(origin_mm=[-100,-100,-100],step_mm=100)
        r.shape=(1,5);r.camera=np.array([0.,0.,100.])
        r.rays=np.tile([0.,0.,-1.],(1,5,1));r.rays[0,4]=[10,0,-1]
        rgb=np.full((1,5,3),180,np.uint8)
        # Above, below, on the plane, missing depth, outside workspace.
        depth=np.array([[50.,150.,100.,0.,150.]],np.float32)
        out=r.render(rgb,depth,0)
        np.testing.assert_array_equal(out[:,[0,2,3,4]],rgb[:,[0,2,3,4]])
        self.assertLess(out[0,1].mean(),rgb[0,1].mean()*.7)
        # Raising the water plane submerges the formerly on-plane pixel.
        raised=r.render(rgb,depth,40)
        self.assertLess(raised[0,2].mean(),rgb[0,2].mean()*.7)
        np.testing.assert_array_equal(r.render(rgb,depth,0,strength=0),rgb)

    def test_optional_flat_plane_ignores_depth_noise_and_holes(self):
        r=Renderer();r.configure_camera(json.loads((CACHE/'camera.json').read_text()))
        rgb=np.full((*r.shape,3),100,np.uint8)
        clear=r.render(rgb,np.zeros(r.shape,np.float32),0,depth_effects=False)
        self.assertTrue(np.any(clear!=rgb))
        noisy=np.random.default_rng(42).uniform(0,5000,r.shape).astype(np.float32)
        noisy[::2,::3]=np.nan
        np.testing.assert_array_equal(clear,r.render(rgb,noisy,0,depth_effects=False))
        np.testing.assert_array_equal(clear,r.render(rgb,np.full(r.shape,10,np.float32),0,depth_effects=False))
        self.assertFalse(np.array_equal(clear,r.render(rgb,noisy,150,depth_effects=False)))

    def test_water_snapshot_above_depth_invalid_and_outside_height(self):
        r=Renderer();r.configure_camera(json.loads((CACHE/'camera.json').read_text()))
        rgb=cv2.imread(str(CACHE/'head_rgb.png'));depth=np.load(CACHE/'depth_mm.npy')
        out=r.render(rgb,depth,0)
        valid=np.isfinite(depth)&(depth>=50)&(depth<=5000)
        world_z=r.camera[2]+r.rays[:,:,2]*np.where(valid,depth,0)
        unchanged=(valid&(world_z>=0))|~valid
        np.testing.assert_array_equal(out[unchanged],rgb[unchanged])
        self.assertGreater(np.any(out!=rgb,axis=2).sum(),1000)
        np.testing.assert_array_equal(r.render(rgb,np.zeros_like(depth),0,depth_effects=True),rgb)
        np.testing.assert_array_equal(r.render(rgb,depth,10000),rgb)

    def test_panel_measured_state_tracks_changed_joint_height(self):
        c=codec();ws=json.loads((WORKSPACE/'forward_workspace_data.json').read_text())
        q=list(ws['example_rad'])
        def message(q):
            # PanelState wire layout: status, arms, actual pos/vel/torque,
            # EE/tool poses, interpolated targets, gripper dimensions/positions.
            return c._to_string([0,2,[7,7],[[0]*7,q],[[0]*7]*2,[[0]*7]*2,
                                 [[0]*7]*2,[[0]*7]*2,[[1]*7]*2,2,[1,1],[[0],[0]]]).encode()
        before=decode_panel(message(q),100.,c,ws);q[1]+=.3
        after=decode_panel(message(q),101.,c,ws)
        self.assertEqual(after['source_kind'],'panel_measured')
        self.assertGreater(abs(before['position_mm'][2]-after['position_mm'][2]),10)

    def test_timeout_preserves_whole_frame_and_marks_paused(self):
        live=Live(Path('/unused'));now=time.time();live.frame_stamp=now;live.state_stamp=now-1
        live.raw='RAW';live.data=dict(mode='live',image='OVERLAY',height_mm=0)
        self.assertEqual(live.snapshot()['image'],'OVERLAY')
        self.assertEqual(live.snapshot()['mode'],'paused')
        self.assertEqual(live.snapshot()['height_mm'],0)
        live.frame_stamp=now-2
        self.assertEqual(live.snapshot()['image'],'OVERLAY')
        self.assertEqual(live.snapshot()['mode'],'paused')
        self.assertEqual(live.snapshot()['frame_unix_s'],now-2)

    def test_production_cpp_panel_packet_including_c_string_terminator(self):
        cpp=WORKSPACE.parents[3]/'dual_v2_2_configuration/robot_platform_utils/cpp'
        ws=json.loads((WORKSPACE/'forward_workspace_data.json').read_text());c=codec()
        with tempfile.TemporaryDirectory() as folder:
            binary=Path(folder)/'panel_packet'
            subprocess.run(['g++','-std=c++17','-ffunction-sections','-fdata-sections','-Wl,--gc-sections',
                '-I'+str(cpp/'include'),str(Path(__file__).with_name('test_panel_packet.cpp')),
                str(cpp/'src/cuarm_message_handler.cpp'),'-o',str(binary)],check=True,capture_output=True)
            payload=subprocess.check_output([str(binary)])
        self.assertTrue(payload.endswith(b'#\x00'))
        for p in [payload,payload+b'\x00'*32,payload[:-1]]:
            result=decode_panel(p,100.,c,ws)
            self.assertEqual(result['source_kind'],'panel_measured')
            self.assertAlmostEqual(result['joints_rad'][1],.3)
            expected,_=ee_from_joints([0,.3,0,0,0,0,0],ws['chain'])
            np.testing.assert_allclose(result['position_mm'],expected)
        for invalid in [payload+b'x',payload[:8]+b'\x00'+payload[8:]]:
            with self.assertRaises(ValueError):decode_panel(invalid,100.,c,ws)

if __name__=='__main__':unittest.main()
