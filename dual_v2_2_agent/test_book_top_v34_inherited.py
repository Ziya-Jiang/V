"""Offline V4.1 extension tests: no devices, models, network or arm commands."""
import sys
import unittest
import json
import tempfile
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from contextlib import nullcontext
from pathlib import Path
import numpy as np
import book_top_v34_inherited as v
from book_top_v34_inherited import load_config
import cart_book_grasp_v1_3_motion as motion
import cart_book_grasp_v1_3_visualization as vis


class Inherited(unittest.TestCase):
    def setUp(self):
        self.c=load_config();self.c['tcp_to_finger_tool_mm']=[0,0,0]
        self.c.pop('_distance_config_path',None)  # Isolate synthetic fixtures from live edits.
        self.c['hover_above_top_mm']=110.
        self.c.update(forward_insertion_mm=5.,final_z_offset_from_top_mm=0.,lower_offset_mm=None)
        self.g=dict(spine_center_base_mm=[550,-100,30],top_base_mm=[550,-100,100],
                    depth_center_base_mm=[560,-100,40],book_median_x_mm=590.)
        self.seed=dict(arm='right',object='book0',book_grasp_mode='cart+',action='move_above_grasp')

    def test_only_four_upper_edge_actions(self):
        tasks=v.task_list([self.seed,dict(self.seed,action='open_gripper')],5)
        self.assertEqual([t['action'] for t in tasks],[v.PREFIX+s for s in v.STAGES[:4]])
        self.assertEqual(len(v.task_list([self.seed],4)),3)
        self.assertEqual(self.seed['action'],'move_above_grasp')

    def test_sequence_axis_only_and_descent_to_upper_edge(self):
        q=np.array([410.,-50.,0.]);commands=[];wait=Mock(return_value=True)
        robot=NS(_wait_for_robot_stop=wait,operator_checkpoint=Mock())
        state=dict(geometry=self.g,path=0.,pose_reader=lambda *_:(q.copy(),np.zeros(3)))
        def execute(r,t,*args):
            goal=np.array(t['target_position']);delta=goal-q
            self.assertLessEqual(np.count_nonzero(abs(delta)>1e-8),1)
            bound=10 if t['action'].endswith('lower_z') else 50
            self.assertLessEqual(np.linalg.norm(delta),bound+1e-8)
            r._wait_for_robot_stop();q[:]=goal;commands.append(t)
            return 'book0',True
        endpoints=[]
        for task in v.task_list([self.seed],5):
            self.assertTrue(v.execute_stage(robot,task,0,{},execute,None,state,self.c)[1]);endpoints.append(q.copy())
        np.testing.assert_allclose(endpoints,[[410,-100,0],[410,-100,210],[595,-100,210],[595,-100,100]])
        self.assertIs(robot._wait_for_robot_stop,wait)
        self.assertEqual(wait.call_args.kwargs['position_tolerance_m'],.003)
        self.assertTrue(wait.call_args.kwargs['require_fresh_stable'])

    def test_descent_then_withdraw_from_fresh_pose_keeps_other_axes(self):
        c=dict(self.c,final_z_offset_from_top_mm=None,lower_offset_mm=-30.,withdraw_mm=50.)
        q=np.array([600.,-100.,200.]);commands=[]
        robot=NS(_wait_for_robot_stop=Mock(return_value=True))
        state=dict(geometry=self.g,path=0.,angles=np.array([0.,0.,20.]),
                   pose_reader=lambda *_:(q.copy(),np.zeros(3)))
        tasks=v.task_list([self.seed],6)
        self.assertEqual(tasks[-1]['action'],v.PREFIX+'withdraw_x')
        def execute(r,t,*args):
            commands.append(t);q[:]=t['target_position'];return 'book0',True
        self.assertTrue(v.execute_stage(robot,tasks[-2],0,{},execute,None,state,c)[1])
        np.testing.assert_allclose(q,[600,-100,170])
        q[:]=[600.4,-99.8,170.3]  # measured arrival, not nominal cached endpoint
        self.assertTrue(v.execute_stage(robot,tasks[-1],0,{},execute,None,state,c)[1])
        np.testing.assert_allclose(q,[550.4,-99.8,170.3])
        self.assertEqual(commands[-1]['target_rotation'],[0.,0.,20.])
        self.assertAlmostEqual(state['path'],80.)
        for invalid in (-50.,float('inf'),401.):
            with self.assertRaises(ValueError):v.validate_distances(dict(c,withdraw_mm=invalid))

    def test_descent_reaches_upper_edge_from_actual_pose(self):
        q=np.array([560.,-100.,203.]);wait=Mock(return_value=True)
        robot=NS(_wait_for_robot_stop=wait)
        state=dict(geometry=self.g,path=0.,angles=np.zeros(3),pose_reader=lambda *_:(q.copy(),np.zeros(3)))
        def execute(r,t,*args):q[:]=t['target_position'];return 'book0',True
        task=v.task_list([self.seed],5)[-1]
        self.assertTrue(v.execute_stage(robot,task,0,{},execute,None,state,self.c)[1])
        np.testing.assert_allclose(q,[560,-100,100])

    def test_offsets_respect_tool_reference_and_fixed_axes(self):
        c=dict(self.c,tcp_to_finger_tool_mm=[50,0,-50])
        np.testing.assert_allclose(v.stage_offset('raise_z',[410,-100,50],np.eye(3),self.g,c),[0,0,210])
        np.testing.assert_allclose(v.stage_offset('forward_x',[410,-100,350],np.eye(3),self.g,c),[135,0,0])

    def test_forward_uses_rotated_finger_x_projection(self):
        c=dict(self.c,tcp_to_finger_tool_mm=[0,0,-50])
        r=v.Rotation.from_euler('y',-60,degrees=True).as_matrix()
        q=np.array([410.,-100.,200.])
        offset=v.stage_offset('forward_x',q,r,self.g,c)
        self.assertAlmostEqual(offset[0],590+5-50*np.sin(np.pi/3)-410)
        self.assertAlmostEqual((q+offset+r@np.array(c['tcp_to_finger_tool_mm']))[0],595.)
        np.testing.assert_allclose(offset[1:],[0,0])

    def test_deployed_config_matches_hot_snapshot(self):
        c=load_config()
        from book_top_v41_distances import read_distances
        for key,value in read_distances(c['_distance_config_path']).items():
            self.assertEqual(c[key],value)

    def test_fixed_50mm_descent_ignores_top_and_compensates_forward_100mm(self):
        c=dict(self.c,forward_insertion_mm=100.,final_z_offset_from_top_mm=None,
               lower_offset_mm=-50.,tcp_to_finger_tool_mm=[50,0,0])
        np.testing.assert_allclose(v.stage_offset('forward_x',[410,0,210],np.eye(3),self.g,c),[230,0,0])
        for z in (210.,218.):
            np.testing.assert_allclose(v.stage_offset('lower_z',[640,0,z],np.eye(3),self.g,c),[0,0,-50])
        self.assertIn('固定下降50mm',v.descent_description(c))

    def test_hot_reload_between_stages_and_freeze_during_waypoints(self):
        from book_top_v41_distances import DISTANCE_KEYS
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'distances.json'
            values={k:self.c[k] for k in DISTANCE_KEYS}
            values['forward_insertion_mm']=7.
            path.write_text(json.dumps(values))
            c=dict(self.c,_distance_config_path=str(path))
            q=np.array([410.,-100.,210.]);commands=[]
            robot=NS(_wait_for_robot_stop=Mock(return_value=True))
            state=dict(geometry=self.g,path=0.,angles=np.zeros(3),pose_reader=lambda *_:(q.copy(),np.zeros(3)))
            def execute(r,t,*args):
                q[:]=t['target_position'];commands.append(q.copy())
                values['forward_insertion_mm']=15.
                values['final_z_offset_from_top_mm']=5.
                path.write_text(json.dumps(values))
                return 'book0',True
            task=dict(self.seed,action=v.PREFIX+'forward_x')
            self.assertTrue(v.execute_stage(robot,task,0,{},execute,None,state,c)[1])
            self.assertAlmostEqual(q[0],597.)  # Snapshot 7mm, not the mid-motion edit to 15mm.
            self.assertGreater(len(commands),1)
            task=dict(self.seed,action=v.PREFIX+'lower_z')
            self.assertTrue(v.execute_stage(robot,task,0,{},execute,None,state,c)[1])
            self.assertAlmostEqual(q[2],105.)  # Next stage sees updated +5mm upper-edge offset.
            self.assertEqual(c['forward_insertion_mm'],5.)

    def test_hot_reload_rejects_invalid_file_before_motion(self):
        from book_top_v41_distances import DISTANCE_KEYS
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'distances.json'
            values={k:self.c[k] for k in DISTANCE_KEYS}
            c=dict(self.c,_distance_config_path=str(path))
            invalid=['{',json.dumps(dict(values,forward_insertion_mm=-1)),
                     json.dumps(dict(values,descent_step_mm=1)),
                     json.dumps(dict(values,max_descent_mm=999))]
            execute=Mock();robot=NS(_wait_for_robot_stop=Mock())
            for content in invalid:
                path.write_text(content)
                with self.assertRaises(ValueError):
                    v.execute_stage(robot,dict(self.seed,action=v.PREFIX+'forward_x'),0,{},execute,None,{},c)
            path.unlink()
            with self.assertRaises(ValueError):v.reload_distances(c)
            execute.assert_not_called()

    def test_distance_path_resolves_relative_to_main_config(self):
        from book_top_v41_distances import DISTANCE_KEYS
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);values={k:self.c[k] for k in DISTANCE_KEYS}
            config={k:value for k,value in self.c.items() if k not in DISTANCE_KEYS}
            config['distance_config']='custom.json'
            values['hover_above_top_mm']=120.
            (root/'custom.json').write_text(json.dumps(values))
            (root/'main.json').write_text(json.dumps(config))
            loaded=load_config(root/'main.json')
            self.assertEqual(loaded['hover_above_top_mm'],120.)
            self.assertEqual(loaded['_distance_config_path'],str(root/'custom.json'))

    def test_lower_reaches_finger_height_and_rejects_upward_or_excess_travel(self):
        c=dict(self.c,tcp_to_finger_tool_mm=[0,0,-50])
        r=v.Rotation.from_euler('y',-60,degrees=True).as_matrix()
        q=np.array([555.,-100.,235.])
        delta=v.stage_offset('lower_z',q,r,self.g,c)
        np.testing.assert_allclose(delta,[0,0,-110],atol=1e-9)
        self.assertAlmostEqual((q+delta+r@np.array(c['tcp_to_finger_tool_mm']))[2],100.)
        for z in (120.,300.):
            with self.assertRaises(ValueError):
                v.stage_offset('lower_z',[555,-100,z],r,self.g,c)

    def test_failure_and_operator_interrupt_restore_wait(self):
        for failure in (False,KeyboardInterrupt()):
            wait=Mock();robot=NS(_wait_for_robot_stop=wait)
            state=dict(geometry=self.g,path=0.,pose_reader=lambda *_:(np.array([410.,0.,0.]),np.zeros(3)))
            call=Mock(return_value=('book0',False),side_effect=failure if isinstance(failure,BaseException) else None)
            task=v.task_list([self.seed],5)[0]
            if isinstance(failure,BaseException):
                with self.assertRaises(KeyboardInterrupt):v.execute_stage(robot,task,0,{},call,None,state,self.c)
            else:self.assertFalse(v.execute_stage(robot,task,0,{},call,None,state,self.c)[1])
            self.assertIs(robot._wait_for_robot_stop,wait);self.assertEqual(call.call_count,1)

    def test_geometry_from_same_mask_and_measured_depth_units(self):
        mask=np.zeros((120,160),bool);mask[20:100,70:81]=True
        k=np.array([[200.,0,80],[0,200.,60],[0,0,1]])
        selected=dict(mask=mask,spine_axis_uv=[0,1],point_camera_m=[-.02,0,.8],book_surface_x_mm=550)
        rgb=np.zeros((120,160,3),np.uint8);c2w=np.eye(4);c2w[:3,3]=[100,200,300]
        a=v.top_geometry(rgb,np.full(mask.shape,800.),k,c2w,selected,mask)
        c2w[:3,3]/=1000
        b=v.top_geometry(rgb,np.full(mask.shape,.8),k,c2w,selected,mask)
        np.testing.assert_allclose(a['top_center_uv'],[75,20])
        np.testing.assert_allclose(a['top_base_mm'],[80,40,1100])
        for field in ('top_base_mm','spine_center_base_mm','depth_center_base_mm'):
            np.testing.assert_allclose(a[field],b[field])
        self.assertEqual(a['depth_center_base_mm'][0],550)
        self.assertEqual(a['book_median_x_mm'],80.)
        self.assertEqual(a['book_median_x_mm'],b['book_median_x_mm'])
        self.assertFalse(a['contact_confirmed'])

    def test_median_is_book_base_x_not_camera_z_or_background(self):
        mask=np.zeros((120,160),bool);mask[20:100,70:81]=True
        k=np.array([[200.,0,80],[0,200.,60],[0,0,1]])
        selected=dict(mask=mask,spine_axis_uv=[0,1],point_camera_m=[-.02,0,.8],book_surface_x_mm=50)
        depth=np.full(mask.shape,800.);depth[40,70]=900.;depth[40,150]=2000.
        transform=np.eye(4);transform[:3,3]=[100,200,300]
        g=v.top_geometry(None,depth,k,transform,selected,mask)
        self.assertEqual(g['book_median_x_mm'],80.)
        self.assertEqual(g['book_median_sample_uv'][0],75)
        self.assertEqual(g['book_median_reference_base_mm'][2],1100.)

    def test_median_resists_one_depth_outlier_and_interpolates_even_count(self):
        mask=np.zeros((120,160),bool);mask[20:100,70:81]=True
        k=np.array([[200.,0,80],[0,200.,60],[0,0,1]])
        selected=dict(mask=mask,spine_axis_uv=[0,1],point_camera_m=[-.02,0,.8],book_surface_x_mm=50)
        depth=np.full(mask.shape,800.);transform=np.eye(4);transform[:3,3]=[100,200,300]
        depth[40,70]=8000.  # A depth outlier cannot set the forward endpoint.
        g=v.top_geometry(None,depth,k,transform,selected,mask)
        self.assertEqual(g['book_median_x_mm'],80.)
        self.assertEqual(g['forward_depth_method'],'median_measured_base_x_in_full_book_mask')
        full=np.zeros_like(mask);full[50,74]=True;full[50,76]=True
        g=v.top_geometry(None,depth,k,transform,selected,full)
        self.assertEqual(g['book_median_x_mm'],80.)  # median of 76 and 84
        self.assertEqual(g['book_median_reference_base_mm'][0],80.)
        self.assertIn(g['book_median_sample_uv'][0],(74,76))

    def test_additional_clearance_does_not_expand_travel_limits(self):
        with self.assertRaisesRegex(ValueError,'range'):
            v.stage_offset('raise_z',[410,0,-301],np.eye(3),self.g,self.c)

    def test_real_v13_runtime_reset_close_perceive_and_restores(self):
        events=[]
        def ordinary(r,t,*a,**kw):events.append(t['action']);return t['object'],True
        step5=NS(execute_single_task=ordinary,execute_main=Mock(),recover_to_reset_after_failure=Mock())
        agent=NS(grounding_main=lambda *a,**kw:events.append('perceive'),execute_main=Mock())
        geom=NS(BOOK_GRASP_MODE='cart+')
        display=NS(logger=None,active=False,safe=Mock())
        robot=NS(control_joint_position=lambda **kw:events.append(kw['action']) or True)
        original_spine=motion._install_spine_center
        with patch.dict(sys.modules,{'dual_v2_2_agent_pcl':agent,'step5_execute':step5,'book_grasp':geom}), \
             patch.object(vis,'install_book_visualization',return_value=display), \
             patch.object(motion,'_reload_initial_right_j6'), \
             patch.object(motion,'_install_spine_center',side_effect=lambda g,*a,**kw:setattr(g,'segment_cart_book_with_sam3',Mock())):
            before=agent.grounding_main
            with v.installed(self.c) as state:
                import cart_book_grasp_v1_3_perception as perception
                def bootstrap(**kw):
                    motion._install_right_arm_reset()
                    agent.grounding_main(robot=robot)
                with patch.object(motion,'main',side_effect=bootstrap):
                    perception.main(task_types=['cart+'])
                self.assertEqual(events,['first_reset','close_gripper','perceive'])
                state['geometry']=self.g
                geom.segment_cart_book_with_sam3()
                self.assertIsNone(state['geometry'])
            self.assertIs(step5.execute_single_task,ordinary)
            self.assertIs(agent.grounding_main,before)
        self.assertIs(motion._install_spine_center,original_spine)

    def test_real_v3_hook_keeps_sam3_gpt_fallback(self):
        from cart_book_grasp_v3_motion import installed as v3_installed
        from cart_book_grasp_v3_4_policy import load_stream_config
        got={};geom=NS()
        def base(g,display,**kw):
            got.update(kw);g.segment_cart_book_with_sam3=Mock()
        with patch.object(motion,'_install_spine_center',side_effect=base), \
             patch('cart_book_grasp_v3_wrist_center.detector_from_geometry',return_value=NS()):
            with v3_installed(None,stream_config=load_stream_config()),v.installed(self.c):
                motion._install_spine_center(geom)
                self.assertTrue(geom.segment_cart_book_with_sam3.gpt_spine_fallback)
                self.assertEqual(got['spine_fallback'].__module__,'cart_book_grasp_v3_2_head_fallback')
                self.assertIn('spine_review',got)

    def test_check_config_has_no_runtime(self):
        with patch.dict(sys.modules,{'dual_v2_2_agent_pcl':None}):
            self.assertEqual(v.main(['--check-config']),0)

    def test_execute_entry_calls_original_loop_with_shared_camera_and_ports(self):
        factory=Mock();agent=NS(start_calibrated_orbbec=factory)
        argv=sys.argv
        def loop(**kw):
            self.assertTrue(str(kw['config_path']).endswith('cart_book_grasp_v3_config.json'))
            self.assertTrue(kw['wait_for_enter_after_success'])
            self.assertIn('--assume-head-camera-calibrated',sys.argv)
            self.assertIn('--no-rerun-viewer',sys.argv)
            self.assertIsNot(agent.start_calibrated_orbbec,factory)
        with patch.dict(sys.modules,{'dual_v2_2_agent_pcl':agent,'v4_1_planner_readiness':NS(require_ready=Mock())}), \
             patch('cart_book_grasp_v1_3.main',side_effect=loop) as original_loop, \
             patch('cart_book_grasp_v4_1_ports.dedicated_planner',return_value=nullcontext()) as ports:
            self.assertEqual(v.main(['--execute','--no-viewer']),0)
            original_loop.assert_called_once();ports.assert_called_once_with(agent)
        self.assertIs(sys.argv,argv);self.assertIs(agent.start_calibrated_orbbec,factory)

    def test_original_executor_keeps_fingers_closed_for_custom_motion(self):
        # Load only this real function and its pure side validator, avoiding
        # step5's perception imports. All hardware calls are recording fakes.
        import ast
        tree=ast.parse((Path(__file__).parent/'RobotRAM/scripts/step5_execute.py').read_text())
        nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in
               ('execute_single_task','validate_arm_target_side')]
        env=dict(RIGHT_CART_BOOK_VERTICAL_ACTIONS=set(),GRIPPER_CLOSE_SETTLE_TIME_S=0.,time=NS(sleep=Mock()))
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'<real step5 functions>','exec'),env)
        robot=NS(master_control=Mock(return_value=True),operator_sleep=Mock())
        task=dict(self.seed,action=v.PREFIX+'align_y',target_position=[410,-100,0],target_rotation=[-90,0,90])
        self.assertTrue(env['execute_single_task'](robot,task,0,{'book0':'book'})[1])
        self.assertEqual(robot.master_control.call_args.args, (None,[410,-100,0,-90,0,90],None,None))


if __name__=='__main__':unittest.main()
