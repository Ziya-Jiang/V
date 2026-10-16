"""Offline geometry, measured grip gating and post-grasp sequencing regression."""
import unittest
import tempfile
import json
from pathlib import Path
from unittest.mock import Mock, patch
from types import SimpleNamespace
import numpy as np
import cart_grap_v1 as cycle
import cart_grap_v1_perception as perception
from cart_grasp_depth import load_approach


class CycleTests(unittest.TestCase):
    def test_stable_center_surface_removes_edge_depth_only(self):
        q=np.float32([[20,20],[100,20],[100,100],[20,100]])
        k=np.array([[100.,0,60],[0,100.,60],[0,0,1]])
        depth=np.full((120,120),680.);depth[40:48,:]=780.
        r=perception.roi_depth(q,depth,k,np.zeros(5),np.eye(4))
        self.assertEqual(r['depth_selection'],'center_supported_connected_depth')
        self.assertEqual(r['depth_median_mm'],680)
        self.assertGreaterEqual(r['depth_fallback']['selected_roi_fraction'],.75)
        self.assertEqual(r['depth_fallback']['core_support_fraction'],1)
        self.assertEqual(r['depth_p10_p90_mm'],[680,680])

    def test_stable_but_minority_center_does_not_authorize_depth(self):
        q=np.float32([[20,20],[100,20],[100,100],[20,100]])
        k=np.array([[100.,0,60],[0,100.,60],[0,0,1]])
        depth=np.full((120,120),780.);depth[50:71,50:71]=680.
        with self.assertRaisesRegex(ValueError,'no dominant connected surface'):
            perception.roi_depth(q,depth,k,np.zeros(5),np.eye(4))
        depth[:]=680.;depth[40:48,:]=780.;depth[50:71,50:71]=0
        with self.assertRaisesRegex(ValueError,'insufficient core depth'):
            perception.roi_depth(q,depth,k,np.zeros(5),np.eye(4))

    def test_rejected_depth_saves_actual_pair_and_diagnostic(self):
        rgb=np.random.default_rng(3).integers(0,255,(200,200,3),dtype=np.uint8)
        depth=np.full((200,200),600.);depth[:,40:]=900.
        camera=SimpleNamespace(K=np.array([[100.,0,40],[0,100.,40],[0,0,1]]),D=np.zeros(5),
            c2w=np.eye(4),captured_unix_s=99.8,close=Mock())
        def capture():
            camera.captured_unix_s+=.05
            return rgb.copy(),depth.copy()
        camera.capture_image=capture
        client=Mock()
        def select(payload,*args):
            scene=json.loads(payload['input'][0]['content'][0]['text'])['scene_id']
            return dict(scene_id=scene,status='selected',confidence=.9,
                        marker_quad=[[30,30],[50,30],[50,50],[30,50]])
        client.select.side_effect=select
        with tempfile.TemporaryDirectory() as folder,patch.object(perception,'open_head_camera',return_value=camera),patch.object(perception.time,'time',return_value=100.):
            out=Path(folder)
            with self.assertRaisesRegex(ValueError,'spread=300.000mm'):
                perception.locate(out,dict(cycle.load_config(),gpt_depth_review_enabled=False),client=client)
            with np.load(out/'head_rejected_geometry.npz') as data:
                np.testing.assert_array_equal(data['depth_mm'],depth)
                self.assertAlmostEqual(float(data['captured_unix_s']),99.9)
            self.assertEqual(json.loads((out/'head_rejection.json').read_text())['sample_index'],0)
            self.assertFalse((out/'head_geometry.json').exists())
        camera.close.assert_called_once()

    def test_raise_plan_and_original_return_reference(self):
        c=dict(cycle.load_config(),alignment_raise_mm=50.);initial=np.array([411.,-58.8,-11.])
        plan=cycle.grasp_plan({'point_base_mm':[430,-355,-42]},initial,[-90,0,90],c,load_approach())
        np.testing.assert_array_equal(plan['raise_target_mm'],[411,-58.8,39])
        self.assertEqual(plan['fixed_z_mm'],39)
        np.testing.assert_array_equal(initial,[411,-58.8,-11])
        state=dict(right_arm=dict(position=initial/1000,orientation=[1,0,0,0]),joint_state=np.zeros(14))
        with patch.object(cycle.CycleIO,'state',return_value=state):
            io=cycle.CycleIO(Mock(),[-90,0,90],c,{},load_approach(),None,Mock())
        self.assertEqual(io.reference_z,39)
        np.testing.assert_array_equal(io.initial_q,initial)
        np.testing.assert_array_equal(io.initial['right_arm']['position'],initial/1000)

    def test_lift_precedes_y_and_forward_and_failure_stops_sequence(self):
        c=dict(cycle.load_config(),alignment_raise_mm=50.)
        plan=cycle.grasp_plan({'point_base_mm':[430,-355,-42]},[411,-58.8,-11],[-90,0,90],c,load_approach())
        plan['coarse_y_target_mm']=[411,-355,39]
        for fail in (None,'raise_alignment_height','coarse_y'):
            events=[]
            io=SimpleNamespace(reference_z=39,pose6=lambda:np.array([411.,-355.2,38.8,-90,0,90]))
            def move(q,angles,label):
                events.append((label,np.asarray(q).copy()))
                if label==fail:raise ValueError('arrival failed')
            io.move=move
            if fail is None:cycle.coarse_approach(io,plan,[-90,0,90])
            else:
                with self.assertRaises(ValueError):cycle.coarse_approach(io,plan,[-90,0,90])
            labels=['raise_alignment_height','coarse_y','coarse_forward']
            count=3 if fail is None else labels.index(fail)+1
            self.assertEqual([e[0] for e in events],labels[:count])
            for _,q in events:self.assertEqual(q[2],39)
            np.testing.assert_array_equal(events[0][1][:2],[411,-58.8])

    def test_cycle_coarse_y_unbounded_but_fixed_xz_and_finite(self):
        c=cycle.load_alignment(cycle.ROOT/cycle.load_config()['alignment_config'])
        self.assertIsNone(c['max_coarse_y_mm'])
        np.testing.assert_array_equal(cycle.coarse_target([411,-58.8,-10],[430,-353.75,-42],c),
                                      [411,-353.75,-10])
        with self.assertRaisesRegex(ValueError,'geometry'):
            cycle.coarse_target([411,-58.8,-10],[430,np.nan,-42],c)

    def test_depth_endpoint_and_independent_withdrawal(self):
        c=cycle.load_config()
        plan=cycle.grasp_plan({'point_base_mm':[600,-100,80]},[411,-60,-10],[-90,0,90],c,load_approach())
        self.assertAlmostEqual(plan['goal_tcp_x_mm'],650)
        self.assertAlmostEqual(plan['forward_mm'],239)
        self.assertEqual(plan['withdraw_mm'],100)
        self.assertEqual(plan['hold_seconds'],3)
        with self.assertRaisesRegex(ValueError,'budget'):
            cycle.grasp_plan({'point_base_mm':[900,-100,80]},[411,-60,-10],[-90,0,90],c,load_approach())

    def test_finish_order_and_stop_on_each_failure(self):
        c=cycle.load_config()
        expected=[('gripper',True),('hold_seconds',3),('gripper',False),('withdraw',100),('return_initial',None)]
        for fail in [None,0,1,2,3]:
            events=[]
            def record(name):
                def call(value=None):
                    events.append((name,value))
                    if len(events)-1==fail:raise ValueError('failed')
                return call
            io=SimpleNamespace(**{k:record(k) for k in ['gripper','hold_seconds','withdraw','return_initial']})
            if fail is None:cycle.finish_cycle(io,c)
            else:
                with self.assertRaises(ValueError):cycle.finish_cycle(io,c)
            self.assertEqual(events,expected if fail is None else expected[:fail+1])

    def test_withdraw_base_negative_x_preserves_y_z_orientation(self):
        io=object.__new__(cycle.CycleIO)
        io.require_open=Mock();io.pose6=Mock(return_value=np.array([650.,-150.,-10.,-88.,-20.,94.]))
        io.move=Mock()
        io.withdraw(100)
        io.require_open.assert_called_once()
        np.testing.assert_array_equal(io.move.call_args.args[0],[550,-150,-10])
        np.testing.assert_array_equal(io.move.call_args.args[1],[-88,-20,94])
        io.move.reset_mock();io.require_open.side_effect=ValueError('not open')
        with self.assertRaises(ValueError):io.withdraw(100)
        io.move.assert_not_called()

    def gripper_io(self,actual):
        io=object.__new__(cycle.CycleIO);io.c=cycle.load_config();io.anchor=np.zeros(6)
        io.keep_anchor=Mock();io.log=Mock()
        io.robot=SimpleNamespace(master_control=Mock(return_value=True),get_cat_gripper=lambda status,category:1 if status=='close' else 0)
        clock=SimpleNamespace(now=100.)
        io.gripper_sample=lambda:(actual(clock.now),0,clock.now)
        return io,clock

    def test_open_requires_measured_feedback_not_just_command_ack(self):
        for opens in [False,True]:
            io,clock=self.gripper_io(lambda t:0. if opens and t>=100.2 else .4)
            with patch.object(cycle.time,'monotonic',lambda:clock.now),patch.object(cycle.time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt)):
                if opens:
                    io.gripper(False)
                    self.assertEqual(io.log.call_args.args[0]['event'],'open_measured')
                else:
                    with self.assertRaisesRegex(ValueError,'opening.*timeout'):io.gripper(False)
            self.assertGreaterEqual(clock.now,100.8)

    def test_stale_measured_open_does_not_complete(self):
        io,clock=self.gripper_io(lambda t:0.)
        io.gripper_sample=lambda:(0.,0,99.)
        with patch.object(cycle.time,'monotonic',lambda:clock.now),patch.object(cycle.time,'sleep',lambda dt:setattr(clock,'now',clock.now+dt)):
            with self.assertRaisesRegex(ValueError,'timeout'):io.gripper(False)
        io.log.assert_not_called()

    def test_soft_hold_has_no_cancel_and_refuses_height_drift(self):
        io=object.__new__(cycle.CycleIO);io.commanded=True;io.reference_z=-10
        io.alignment={'fixed_xz_mm':3};io.send_visual=Mock()
        io.transport=SimpleNamespace(read=lambda:dict(q=np.array([550.,-100.,-10.8]),roll=2.),pending='old')
        io.soft_hold();self.assertIsNone(io.transport.pending)
        np.testing.assert_array_equal(io.send_visual.call_args.args[0],[550,-100,-10])
        self.assertEqual([call.args[2] for call in io.send_visual.call_args_list],[True,False])
        np.testing.assert_array_equal(io.send_visual.call_args_list[0].args[0],io.send_visual.call_args_list[1].args[0])
        io.send_visual.reset_mock()
        io.transport.read=lambda:dict(q=np.array([550.,-100.,-14.]),roll=2.)
        with self.assertRaises(ValueError):io.soft_hold()
        io.send_visual.assert_not_called()

    def test_gpt_region_is_not_identity_or_depth(self):
        c=cycle.load_config();q=[[30,30],[50,30],[50,50],[30,50]]
        result=dict(scene_id='test',status='selected',confidence=.9,marker_quad=q)
        np.testing.assert_array_equal(perception.validate_region(result,(200,200,3),'test',c),q)
        for change in [dict(scene_id='old'),dict(status='ambiguous'),dict(confidence=.4),dict(marker_quad=[[0,0]]*4)]:
            with self.assertRaises(ValueError):perception.validate_region(dict(result,**change),(200,200,3),'test',c)

    def test_measured_depth_transform_and_holes(self):
        q=np.array([[30,30],[50,30],[50,50],[30,50]],np.float32)
        k=np.array([[100.,0,40],[0,100.,40],[0,0,1]])
        transform=np.eye(4);transform[:3,3]=[100,200,300]
        depth=np.full((80,80),600.)
        r=perception.roi_depth(q,depth,k,np.zeros(5),transform)
        np.testing.assert_allclose(r['point_base_mm'],[100,200,900])
        depth[:]=0
        with self.assertRaisesRegex(ValueError,'insufficient'):perception.roi_depth(q,depth,k,np.zeros(5),transform)
        depth[:]=600;depth[:,40:]=900
        with self.assertRaisesRegex(ValueError,'multiple'):perception.roi_depth(q,depth,k,np.zeros(5),transform)

    def test_head_patch_change_rejected(self):
        image=np.random.default_rng(5).integers(0,255,(80,80,3),dtype=np.uint8)
        q=np.array([[30,30],[50,30],[50,50],[30,50]])
        perception.same_patch(image,image.copy(),q)
        with self.assertRaisesRegex(ValueError,'moved/changed'):perception.same_patch(image,np.roll(image,5,axis=1),q)


if __name__=='__main__':unittest.main()
