"""Pure endpoint diagnostics tests; no sockets or hardware."""
import ast
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np

SCRIPTS=Path(__file__).resolve().parents[1]/'dual_v2_2_control/cuarm_upper_planner/scripts'
sys.path.insert(0,str(SCRIPTS))
from v4_1_endpoint_diagnostics import diagnose_endpoint,report_failure,report_optimizer_failure
from v4_1_planner_direct_servo import CpuCollision


def model():
    c=object.__new__(CpuCollision)
    c.bounds=np.array([[-1.,-1.],[1.,1.]])
    c.links=np.array([0,1]);c.inverse=np.array([0,1])
    c.pairs=np.array([[0,1]]);c.offset=np.zeros(2)
    c.spheres=np.array([[0.,0.,0.,.1],[1.,0.,0.,.1]])
    c.boxes=[]
    c.cpu=NS(store={'left':0,'right':1},transforms=lambda q,indices:np.tile(np.eye(4),(len(q),len(indices),1,1)))
    return c


class EndpointDiagnosticsTests(unittest.TestCase):
    def test_valid_endpoint_agrees_with_existing_checker(self):
        c=model();before=copy.deepcopy(c.spheres)
        r=diagnose_endpoint(c,['a','b'],[0.,0.])
        self.assertEqual(r['cpu_geometry_feasible'],c.check([[0.,0.]]))
        self.assertTrue(r['cpu_geometry_feasible']);np.testing.assert_array_equal(c.spheres,before)

    def test_joint_limit_is_not_reported_as_collision(self):
        c=model();r=diagnose_endpoint(c,['a','b'],[1.2,0.])
        self.assertEqual(r['joint_limit_violations'][0]['joint'],'a')
        self.assertEqual(r['self_collision_count'],0)
        self.assertFalse(c.check([[1.2,0.]]));self.assertFalse(r['cpu_geometry_feasible'])

    def test_self_contact_reports_links_and_penetration(self):
        c=model();c.spheres[1,0]=.15
        r=diagnose_endpoint(c,['a','b'],[0.,0.])
        self.assertEqual(r['self_collision_count'],1)
        hit=r['self_collision_pairs'][0]
        self.assertEqual(hit['link_a']['name'],'left');self.assertEqual(hit['link_b']['name'],'right')
        self.assertAlmostEqual(hit['penetration_mm'],50.)
        self.assertEqual(r['cpu_geometry_feasible'],c.check([[0.,0.]]))

    def test_world_contact_reports_box_without_self_contact(self):
        c=model();c.boxes=[(np.zeros(3),np.eye(3),np.ones(3)*.2)]
        r=diagnose_endpoint(c,['a','b'],[0.,0.])
        self.assertEqual(r['self_collision_count'],0)
        self.assertEqual(r['world_collision_count'],1)
        self.assertEqual(r['world_collisions'][0]['box_index'],0)
        self.assertEqual(r['cpu_geometry_feasible'],c.check([[0.,0.]]))

    def test_report_keeps_start_goal_distinct_and_preserves_status(self):
        p=NS(_v31_direct=NS(collision=model()),target_description='first_reset',status=3)
        out=io.StringIO()
        with contextlib.redirect_stdout(out):report_failure(p,['a','b'],[1.2,0.],[0.,0.])
        r=json.loads(out.getvalue().split(': ',1)[1])
        self.assertFalse(r['start']['cpu_geometry_feasible']);self.assertTrue(r['goal']['cpu_geometry_feasible'])
        self.assertEqual(p.status,3)
        with contextlib.redirect_stdout(out):report_failure(p,['a','b'],[float('nan'),0.],[0.,0.])
        self.assertIn('diagnostics unavailable',out.getvalue());self.assertEqual(p.status,3)

    def test_reported_j4_decimal_roundoff_is_not_a_model_violation(self):
        c=model();c.bounds=c.bounds.astype(np.float32)
        c.bounds[1,0]=np.float32(2.4434609413146973)
        q=[2.443461,0.];before=np.asarray(q).copy()
        r=diagnose_endpoint(c,['J4','b'],q)
        self.assertEqual(r['joint_limit_violations'],[])
        self.assertEqual(r['roundoff_only'][0]['joint'],'J4')
        self.assertAlmostEqual(r['roundoff_only'][0]['raw_excess_si'],5.868530283947848e-8)
        self.assertTrue(r['cpu_geometry_feasible'])
        np.testing.assert_array_equal(q,before)
        r=diagnose_endpoint(c,['J4','b'],[2.443466,0.])
        self.assertEqual(r['joint_limit_violations'][0]['joint'],'J4')
        self.assertFalse(r['cpu_geometry_feasible'])

    def test_optimizer_report_preserves_independent_failure_evidence(self):
        traj=NS(success=np.array([False,False]),metrics=NS(feasible=np.array([[True,False],[True,True]]),
            cspace_error=np.array([[.2,.03],[.4,.02]])),optimized_dt=np.array([.5,.7]),debug_info={})
        result=NS(debug_info={'trajopt_result':traj},status='TRAJOPT_FAIL')
        out=io.StringIO()
        with contextlib.redirect_stdout(out):report_optimizer_failure(result)
        r=json.loads(out.getvalue().split(': ',1)[1])
        self.assertEqual(r['feasible']['failed'],1)
        self.assertEqual(r['terminal_cspace_error']['minimum'],.02)
        self.assertEqual(r['terminal_cspace_error']['maximum'],.03)
        self.assertEqual(r['optimized_dt_s']['maximum'],.7)
        self.assertIsNone(r['smooth_label'])
        self.assertFalse(traj.success.any())

    def test_actual_joint_planner_calls_diagnostics_only_after_failure(self):
        tree=ast.parse((SCRIPTS/'curobo_planner_v4_1.py').read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_motion_gen_plan_single_js')
        guard=next(n for n in ast.walk(method) if isinstance(n,ast.If) and any(isinstance(x,ast.ImportFrom) and x.module=='v4_1_endpoint_diagnostics' for x in n.body))
        self.assertEqual(ast.unparse(guard.test),'not result.success.item()')
        old=(SCRIPTS/'curobo_planner_v3_1.py').read_text()
        self.assertNotIn('v4_1_endpoint_diagnostics',old)

if __name__=='__main__':unittest.main()
