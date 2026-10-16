"""No hardware/API: settling, warmup ordering and immutable acquisition anchor."""
import copy
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
import cart_grap_v1_1 as cycle
import cart_grap_v1_perception as perception

class AcquisitionTests(unittest.TestCase):
    def setUp(self):
        self.clock=SimpleNamespace(now=100.)
        self.events=[]
        self.state=dict(received_monotonic=100.,pose_sample_monotonic=100.,robot_status=0,curobo_status=9,
            right_arm=dict(position=[.411,-.058,-.011],orientation=[1.,0.,0.,0.]),
            processed_arm_joint_command=np.zeros((2,7)),joint_state=np.zeros((2,7)),joint_velocity=np.zeros((2,7)))
        self.robot=SimpleNamespace(get_current_state=self.read)
        self.addCleanup(patch.stopall)
        patch.object(cycle.time,'monotonic',lambda:self.clock.now).start()
        patch.object(cycle.time,'sleep',lambda dt:setattr(self.clock,'now',self.clock.now+dt)).start()
    def read(self):
        s=copy.deepcopy(self.state)
        s['received_monotonic']=s['pose_sample_monotonic']=self.clock.now
        return s
    def io(self,execute=True):
        return cycle.CycleIO(self.robot,[0,0,0],cycle.load_config(),{}, {}, None,self.events.append,
            reset_target=np.zeros(7) if execute else None,defer_acquisition=True)
    def test_sub_half_mm_per_second_drift_must_not_pass(self):
        def read():
            s=self.read();s['right_arm']['position'][0]+=(self.clock.now-100.)*.0003
            return s
        with self.assertRaisesRegex(ValueError,'TCP settling timed out'):
            cycle.wait_tcp_stable(read,self.events.append,timeout=3.)
        self.assertFalse(any(e['event']=='tcp_settle_confirmed' for e in self.events))
    def test_pending_controller_motion_cannot_settle(self):
        self.state['robot_status']=1
        with self.assertRaisesRegex(ValueError,'TCP settling timed out'):
            cycle.wait_tcp_stable(self.read,self.events.append,timeout=1.5)
    def test_duplicate_samples_cannot_settle(self):
        with self.assertRaisesRegex(ValueError,'Stale pose'):
            cycle.wait_tcp_stable(lambda:copy.deepcopy(self.state),self.events.append)
    def test_warmup_displacement_is_not_acquisition_drift(self):
        io=self.io()
        with self.assertRaisesRegex(ValueError,'anchor not ready'):io.stationary()
        def warm():
            self.clock.now+=1.;self.state['right_arm']['position'][0]+=.0006
            self.state['right_arm']['orientation']=[np.cos(.01),0,0,np.sin(.01)]
        camera=SimpleNamespace(capture_image=Mock(side_effect=warm))
        io.prepare_acquisition(camera)
        io.stationary()
        self.assertAlmostEqual(io.initial_q[0],411.6)
        np.testing.assert_allclose(io.transport.base,io.base)
        np.testing.assert_allclose(io.transport.last_angles,io.base)
        self.assertAlmostEqual(io.reference_z,io.initial_q[2]+io.c['alignment_raise_mm'])
        self.assertEqual([e['event'] for e in self.events],['acquisition_cameras_warmed',
            'tcp_settle_start','tcp_settle_confirmed','acquisition_anchor_locked'])
        with self.assertRaisesRegex(ValueError,'already locked'):io.prepare_acquisition(camera)
        camera.capture_image.assert_called_once()
        self.state['right_arm']['position'][0]+=.0006
        with self.assertRaisesRegex(ValueError,'position_delta=0.600mm'):io.stationary()
        self.assertAlmostEqual(io.initial_q[0],411.6)
    def test_joint_drift_rejected_even_if_tcp_is_unchanged(self):
        io=self.io();self.state['joint_state'][1,3]=np.deg2rad(.31)
        with self.assertRaisesRegex(ValueError,'left first_reset'):io.pre_acquisition()
        self.assertFalse(io.acquisition_ready)
        self.assertEqual(self.events[-1]['event'],'acquisition_reset_drift')
    def test_new_planning_or_fault_cannot_authorize_acquisition(self):
        io=self.io()
        for status in (0,1,2,3,5,6,7,8,99):
            self.state['curobo_status']=status
            with self.assertRaises(ValueError):io.pre_acquisition()
    def test_post_reset_constructor_accepts_success_status_with_same_target(self):
        self.state['curobo_status']=4
        io=self.io()
        self.assertFalse(io.acquisition_ready)
        io.prepare_acquisition(SimpleNamespace(capture_image=Mock()))
        io.stationary()
        self.assertTrue(io.acquisition_ready)
    def test_9_to_4_transition_with_small_joint_error_does_not_erase_anchor(self):
        io=self.io();io.prepare_acquisition(SimpleNamespace(capture_image=Mock()))
        initial=copy.deepcopy(io.initial)
        self.state['joint_state'][1,0]=np.deg2rad(.1)
        for status in (4,9,4):
            self.state['curobo_status']=status;io.stationary()
        np.testing.assert_array_equal(io.initial['joint_state'],initial['joint_state'])
        self.assertEqual(sum(e['event']=='acquisition_reset_status' for e in self.events),1)
        self.state['right_arm']['position'][0]+=.0006
        with self.assertRaisesRegex(ValueError,'position_delta=0.600mm'):io.stationary()
    def test_success_status_never_overrides_measured_joint_error(self):
        io=self.io();self.state['curobo_status']=4
        self.state['joint_state'][1,0]=np.deg2rad(.31)
        with self.assertRaisesRegex(ValueError,'left first_reset'):io.pre_acquisition()
    def test_changed_or_missing_reset_target_cannot_use_success_status(self):
        io=self.io();self.state['curobo_status']=4
        self.state['processed_arm_joint_command'][1,0]=.01
        with self.assertRaisesRegex(ValueError,'target changed'):io.pre_acquisition()
        del self.state['processed_arm_joint_command']
        with self.assertRaisesRegex(ValueError,'Invalid first_reset target'):io.pre_acquisition()

    def test_preview_has_no_home_requirement_or_motion(self):
        io=self.io(False);self.state['joint_state'][1,0]=.9;self.state['curobo_status']=4
        io.prepare_acquisition(SimpleNamespace(capture_image=Mock()))
        io.stationary()
        self.assertTrue(io.acquisition_ready)
        self.assertFalse(io.commanded)
    def test_head_prepare_precedes_source_and_gpt(self):
        order=[];io=self.io();camera=SimpleNamespace(captured_unix_s=0.,close=Mock())
        def capture():
            order.append('capture');camera.captured_unix_s=100.+len(order)*.01
            return np.zeros((20,20,3),np.uint8),np.ones((20,20))
        camera.capture_image=capture
        def prepare(cam):
            order.append('prepare');io.prepare_acquisition(cam);order.append('locked')
        def select(*args):
            order.append('gpt');io.stationary();raise RuntimeError('test stops at API boundary')
        client=SimpleNamespace(select=select)
        with tempfile.TemporaryDirectory() as folder,patch.object(perception,'open_head_camera',return_value=camera),patch.object(perception.time,'time',return_value=100.1):
            with self.assertRaisesRegex(RuntimeError,'API boundary'):
                perception.locate(Path(folder),cycle.load_config(),checkpoint=io.stationary,
                    client=client,prepare_camera=prepare)
        self.assertEqual(order,['prepare','capture','locked','capture','gpt'])
        camera.close.assert_called_once()
    def test_unstable_prepare_does_not_call_gpt_and_closes_camera(self):
        io=self.io();self.state['joint_velocity'][:]=.006
        camera=SimpleNamespace(capture_image=Mock(),close=Mock());client=Mock()
        with tempfile.TemporaryDirectory() as folder,patch.object(perception,'open_head_camera',return_value=camera):
            with self.assertRaisesRegex(ValueError,'TCP settling timed out'):
                perception.locate(Path(folder),cycle.load_config(),client=client,prepare_camera=io.prepare_acquisition)
        client.select.assert_not_called();camera.close.assert_called_once()
        self.assertFalse(io.acquisition_ready)

if __name__=='__main__':unittest.main()
