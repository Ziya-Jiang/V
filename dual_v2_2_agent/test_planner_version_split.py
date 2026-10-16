"""Offline version routing and historical restoration checks; no hardware."""
import ast
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
AGENT=ROOT/'dual_v2_2_agent'
PLANNER=ROOT/'dual_v2_2_control/cuarm_upper_planner'
SCRIPTS=PLANNER/'scripts'
sys.path.insert(0,str(SCRIPTS))
import v3_1_readiness as old_ready
import v4_1_planner_readiness as new_ready
import cart_book_grasp_v3_1_ports as old_ports
import cart_book_grasp_v4_1_ports as new_ports

class VersionSplitTests(unittest.TestCase):
    def test_restored_files_match_historical_inverse_patch_manifest(self):
        data=json.loads((AGENT/'test_fixtures/planner_version_split_20261014.json').read_text())
        for name,info in data['files'].items():
            with self.subTest(name=name):
                self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),info['restored_sha256'])

    def test_entries_route_to_separate_changed_dependencies(self):
        for v,port in [('3_1',10176),('4_1',10196)]:
            text=(PLANNER/f'dual_v2_2_upper_planner_v{v}.py').read_text()
            self.assertIn(f'from curobo_planner_v{v} import CuroboPlanner',text)
            self.assertIn(f'from v{v}_panel_transport import PanelTransport',text)
            self.assertIn(str(port),text)
        old=(SCRIPTS/'curobo_planner_v3_1.py').read_text()
        new=(SCRIPTS/'curobo_planner_v4_1.py').read_text()
        self.assertNotIn('constrain_robot_config',old)
        self.assertNotIn('trajectory_timing',old)
        self.assertIn('from v4_1_joint_limits import',new)
        self.assertIn('from v4_1_planning_time import',new)
        self.assertIn('from v4_1_planner_direct_servo import',new)
        self.assertIn('from curobo_ik_solver_v4_1 import',new)
        self.assertNotEqual(old_ready.fingerprint(),new_ready.fingerprint())
        self.assertNotEqual(old_ready.receipt_path(10196),new_ready.receipt_path(10196))

    def test_readiness_rejects_old_fingerprint_even_at_new_port(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'receipt.json'
            receipt=dict(pid=123,start='same',fingerprint=old_ready.fingerprint(),port=10196)
            p.write_text(json.dumps(receipt))
            with patch.object(new_ready,'receipt_path',return_value=p),patch.object(new_ready,'process_start',return_value='same'):
                with self.assertRaisesRegex(ValueError,'planner_v4_1'):new_ready.require_ready()
                receipt['fingerprint']=new_ready.fingerprint();p.write_text(json.dumps(receipt))
                new_ready.require_ready()
                receipt['port']=10176;p.write_text(json.dumps(receipt))
                with self.assertRaises(ValueError):new_ready.require_ready()

    def test_port_context_restores_original_decoder_and_agent_on_error(self):
        class FakeUDP:unpack_curobo_state=object()
        decoder=FakeUDP.unpack_curobo_state
        for module,expected in [(old_ports,(10175,10176)),(new_ports,(10195,10196))]:
            agent=NS(TARGET_LOCAL_PORT=1,TARGET_REMOTE_PORT=2)
            with patch.dict(sys.modules,{'cuarm_udp':NS(CuarmUdp=FakeUDP),'v3_1_feedback':NS(unpack_feedback=object())}):
                with self.assertRaisesRegex(RuntimeError,'test abort'):
                    with module.dedicated_planner(agent):
                        self.assertEqual((agent.TARGET_LOCAL_PORT,agent.TARGET_REMOTE_PORT),expected)
                        self.assertIsNot(FakeUDP.unpack_curobo_state,decoder)
                        raise RuntimeError('test abort')
            self.assertEqual((agent.TARGET_LOCAL_PORT,agent.TARGET_REMOTE_PORT),(1,2))
            self.assertIs(FakeUDP.unpack_curobo_state,decoder)

    def test_v41_client_new_route_v34_route_unchanged(self):
        text=(AGENT/'book_top_v34_inherited.py').read_text()
        self.assertIn('from v4_1_planner_readiness import require_ready',text)
        self.assertIn('from cart_book_grasp_v4_1_ports import dedicated_planner',text)
        text=(AGENT/'cuart_book_grasp_v3_4.py').read_text()
        self.assertNotIn('v4_1_planner_readiness',text)
        self.assertNotIn('cart_book_grasp_v4_1_ports',text)

if __name__=='__main__':unittest.main()
