"""No hardware: path rejection, actual RT interpolation, scoped solver cost."""
import contextlib
import io
from types import SimpleNamespace as NS
import unittest
import numpy as np
import torch
from test_v4_2_planner_precision import Planner, planner, original
from v4_2_x_path import ACTION, right_x_objective, validate_path, x_report
from v3_1_panel_path import PanelPath
import ast


def cost(weight=None):
    return NS(run_vec_weight=torch.zeros(6) if weight is None else weight,
              project_distance_tensor=torch.ones(1, dtype=torch.uint8))


class Tests(unittest.TestCase):
    def test_right_cost_only_restores_shared_storage_on_exception(self):
        a=cost();b=cost(a.run_vec_weight);left=cost()
        mg=NS(get_all_pose_rollout_instances=lambda:[
            NS(_link_pose_costs={'arm_end_effector_r':a},goal_cost=left),
            NS(_link_pose_costs={'arm_end_effector_r':b})])
        with self.assertRaisesRegex(RuntimeError,'solver error'):
            with right_x_objective(mg,ACTION):
                self.assertEqual(a.run_vec_weight.tolist(),[0,0,0,1,0,0])
                self.assertEqual(a.project_distance_tensor.item(),0)
                self.assertEqual(left.run_vec_weight.tolist(),[0]*6)
                self.assertEqual(left.project_distance_tensor.item(),1)
                raise RuntimeError('solver error')
        self.assertEqual(a.run_vec_weight.tolist(),[0]*6)
        self.assertEqual(b.run_vec_weight.tolist(),[0]*6)
        self.assertEqual(a.project_distance_tensor.item(),1)
        self.assertEqual(b.project_distance_tensor.item(),1)

    def test_missing_or_aliased_right_cost_fails_closed(self):
        with self.assertRaisesRegex(ValueError,'unavailable'):
            with right_x_objective(NS(get_all_pose_rollout_instances=lambda:[]),ACTION):pass
        a=cost();left=cost(a.run_vec_weight)
        with self.assertRaisesRegex(ValueError,'aliases'):
            with right_x_objective(NS(get_all_pose_rollout_instances=lambda:[
                    NS(_link_pose_costs={'arm_end_effector_r':a},goal_cost=left)]),ACTION):pass
        self.assertEqual(a.run_vec_weight.tolist(),[0]*6)

    def test_non_v42_does_not_access_solver_internals(self):
        with right_x_objective(object(),'move_down_to_grasp'):pass

    def bridge(self, *, bow=0., drift=0.):
        sample=PanelPath()  # Real RT C++ interpolation, no UDP.
        start=np.zeros(14);points=np.zeros((3,14));points[:,7]=[0,.5,1.]
        def tcp(q):
            q=np.atleast_2d(q);p=np.zeros((len(q),2,3));p[:,1,0]=.411+bow*np.sin(np.pi*q[:,7])
            p[:,1,2]=q[:,7]*.01
            return p,None
        calls=[0]
        def snapshot():
            q=start.copy();q[7]=drift if calls[0] else 0.;calls[0]+=1
            return dict(joints=q,velocity=np.zeros(14),position=tcp(q)[0][0])
        b=NS(snapshot=snapshot,tcp=tcp,panel_path=sample,limits=np.ones((2,7))*3)
        return b,points

    def test_real_panel_interpolation_accepted_with_reserve(self):
        b,p=self.bridge(bow=.001);reports=[]
        self.assertTrue(validate_path(b,p,p,.2,1,.411,reports.append))
        self.assertEqual([r['source'] for r in reports],['curobo','panel','panel','panel','validation_start'])
        self.assertGreater(reports[1]['samples'],len(p))
        self.assertAlmostEqual(reports[1]['tracking_reserve_mm'],1.5)

    def test_valid_endpoints_cannot_hide_dense_path_excursion(self):
        b,p=self.bridge(bow=.004);reports=[]
        self.assertFalse(validate_path(b,p,p,.2,1,.411,reports.append))
        self.assertAlmostEqual(reports[0]['max_deviation_mm'],4.)

    def test_panel_curve_excursion_between_valid_knots_rejected(self):
        b,p=self.bridge(bow=.004);p=p[[0,2]];reports=[]
        self.assertFalse(validate_path(b,p,p,.2,1,.411,reports.append))
        self.assertTrue(reports[0]['accepted'])
        self.assertEqual(reports[1]['source'],'panel');self.assertFalse(reports[1]['accepted'])

    def test_validation_start_movement_rejected(self):
        b,p=self.bridge(drift=.003);reports=[]
        self.assertFalse(validate_path(b,p,p,.2,1,.411,reports.append))
        self.assertFalse(reports[-1]['accepted'])

    def test_nonfinite_and_wrong_path_shapes_rejected(self):
        for p in (np.zeros((0,2,3)),np.zeros((2,3)),np.full((1,2,3),np.nan)):
            with self.assertRaises(ValueError):x_report(p,1,.411,source='test')

    def test_actual_planner_path_method_and_publication_order(self):
        p=planner();b,points=self.bridge(bow=.004)
        p._v31_bridge=b;p.target_link_names=['left','arm_end_effector_r']
        p.target_arm_tool_position[1,0]=.411;p.limited_waypoints=points;p.waypoints_dt=.2
        p._cmd_plan=NS(position=NS(cpu=lambda:NS(numpy=lambda:points)))
        with contextlib.redirect_stdout(io.StringIO()):self.assertFalse(p._check_v42_x_path())
        p.target_description='move_down_to_grasp';self.assertTrue(p._check_v42_x_path())
        method=next(n for n in original.body if isinstance(n,ast.FunctionDef) and n.name=='_retrieve_motion_gen_result')
        source=ast.unparse(method)
        self.assertLess(source.index('not self._check_v42_x_path()'),source.index('self.curobo_cmd.waypoints ='))
        self.assertIn('self._hold_current_arm_command()',source[source.index('not self._check_v42_x_path()'):])

if __name__=='__main__':unittest.main()
