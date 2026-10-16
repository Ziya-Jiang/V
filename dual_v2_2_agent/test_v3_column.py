import unittest
from unittest.mock import patch
import cv2
import numpy as np
from cart_book_grasp_v3_column import ColumnLock,visible_center_column,run_column_servo,load_config
from cart_book_grasp_v3_servo import ServoError


class ColumnTests(unittest.TestCase):
    def test_visible_center_ignores_vertical_clipping(self):
        for top,bottom in ((0,240),(0,100),(180,240)):
            mask=np.zeros((240,320),bool);mask[top:bottom,175:205]=True
            self.assertEqual(visible_center_column(mask),189.5)

    def test_selection_ranks_horizontal_distance_without_vertical_penalty(self):
        from cart_book_grasp_v3_wrist_center import select_center_book
        rgb=np.zeros((240,320,3),np.uint8);a=np.zeros((240,320),bool);b=a.copy()
        a[:30,150:170]=True;b[80:180,180:210]=True
        book,_=select_center_book(rgb,[(.9,a,[]),(.9,b,[])],horizontal_only=True)
        np.testing.assert_array_equal(book['mask'],a)

    def test_lk_column_translation_with_vertical_clipping(self):
        rng=np.random.default_rng(71);rgb=rng.integers(0,256,(240,320,3),dtype=np.uint8)
        mask=np.zeros((240,320),bool);mask[:,140:200]=True
        lock=ColumnLock(rgb,mask)
        for dx,dy in ((3,5),(6,15),(10,25)):
            frame=cv2.warpAffine(rgb,np.float32([[1,0,dx],[0,1,dy]]),(320,240))
            o=lock.observe(frame)
            self.assertAlmostEqual(o['uv'][0],169.5+dx,delta=1.)
            self.assertEqual(o['uv'][1],119.5)
        with self.assertRaises(ServoError):lock.observe(np.zeros_like(rgb))

    def test_controller_ignores_vertical_error_and_never_commands_z_changes(self):
        c=load_config();q=np.array([500.,0.,17.]);moves=[];t=[10.]
        def observe():
            t[0]+=.01
            return dict(uv=np.array([160.+90.+2*q[1],-4000.]),scale=1.,xyz_mm=q.copy(),captured_at=t[0])
        def move(goal):moves.append(goal.copy());q[:]=goal;return True
        result=run_column_servo(observe,move,160.,520.,c,clock=lambda:t[0])
        self.assertTrue(result['converged']);self.assertLessEqual(abs(q[0]-520),1.)
        self.assertGreater(abs(q[1]),20.)
        self.assertTrue(all(x[2]==17. for x in moves))
        self.assertEqual(moves[0][0],500.);self.assertEqual(moves[1][0],500.)
        self.assertTrue(all(np.linalg.norm(b-a)<=5.000001 for a,b in zip([np.array([500.,0.,17.]),*moves],moves)))

    def test_centered_and_at_depth_ignores_y_pixel_without_any_probe(self):
        c=load_config();t=[0.];moves=[]
        def observe():
            t[0]+=.01
            return dict(uv=[160,10000],scale=1.,xyz_mm=[500,0,17],captured_at=t[0])
        result=run_column_servo(observe,lambda q:moves.append(q),160,500,c,clock=lambda:t[0])
        self.assertTrue(result['converged']);self.assertEqual(moves,[])

    def test_tracking_loss_stops_before_next_motion(self):
        c=load_config();t=[0.];moves=[]
        def observe():
            t[0]+=.01
            if moves:raise ServoError('lost points')
            return dict(uv=[180,100],scale=1.,xyz_mm=[500,0,17],captured_at=t[0])
        def move(q):moves.append(q);return True
        with self.assertRaisesRegex(ServoError,'lost points'):
            run_column_servo(observe,move,160,520,c,clock=lambda:t[0])
        self.assertEqual(len(moves),1)

    def test_failed_motion_z_drift_stale_frame_and_path_budget_stop(self):
        for failure in ('motion','z','stale','path'):
            c=load_config();q=np.array([500.,0.,17.]);t=[0.];moves=[]
            if failure=='path':c['max_path_mm']=1.
            def observe():
                t[0]+=.01
                stamp=-10. if failure=='stale' else t[0]
                return dict(uv=[180.+2*q[1],0],scale=1.,xyz_mm=q.copy(),captured_at=stamp)
            def move(goal):
                moves.append(goal);q[:]=goal
                if failure=='z':q[2]+=c['max_pose_error_mm']+.1
                return failure!='motion'
            with self.assertRaises(ServoError):
                run_column_servo(observe,move,160,520,c,clock=lambda:t[0])
            self.assertLessEqual(len(moves),1)

    def test_column_hook_dispatches_without_loading_cotracker(self):
        import cart_book_grasp_v1_3_motion as motion
        from cart_book_grasp_v3_motion import installed
        from cart_book_grasp_v3_column import CONFIG_PATH
        with patch('cart_book_grasp_v3_cotracker_runtime.execute_cotracker_approach',return_value=('book',True)) as run:
            with installed(None,column_path=CONFIG_PATH):
                motion._execute_cart_plus_forward(None,{},0,{},None,None)
            self.assertTrue(run.call_args.kwargs['column_mode'])
            self.assertIsNone(run.call_args.kwargs['model'])

    def test_runtime_fixed_height_and_no_cotracker_model(self):
        from test_cart_book_grasp_v3_distance import RuntimeTests
        fixture=RuntimeTests()
        result,commands,context=fixture.run_runtime(column=True)
        self.assertEqual(result,('book',True));self.assertTrue(context['book']['forward_complete'])
        self.assertTrue(all(t['target_position'][2]==0 for t in commands))
        self.assertLessEqual(abs(context['book']['visual_final_xyz_mm'][0]-650),1)

    def test_default_and_legacy_entry_routes(self):
        import V3
        with patch('cart_book_grasp_v3_column_main.main') as current:
            V3.main(['--check-config']);current.assert_called_once_with(['--check-config'])
        with patch('cart_book_grasp_v3_cotracker_main.main') as legacy:
            V3.main(['--legacy-cotracker','--check-config']);legacy.assert_called_once_with(['--check-config'])


if __name__=='__main__':unittest.main()
