"""Scope the dedicated agent UDP endpoints to the V4.1 entry point only."""
from contextlib import contextmanager

AGENT_PORT=10195
PLANNER_PORT=10196

@contextmanager
def dedicated_planner(agent):
    # This context exists only in the V4.1 entry point. Existing V3 / planner
    # processes keep the unextended protocol. The UDP thread captures this
    # decoder when constructed, and the class method is restored on exit.
    import sys
    from pathlib import Path
    scripts=Path(__file__).resolve().parents[1]/'dual_v2_2_control/cuarm_upper_planner/scripts'
    if str(scripts) not in sys.path:sys.path.insert(0,str(scripts))
    from v3_1_feedback import unpack_feedback
    from cuarm_udp import CuarmUdp
    previous_decoder=CuarmUdp.unpack_curobo_state
    previous=agent.TARGET_LOCAL_PORT,agent.TARGET_REMOTE_PORT
    agent.TARGET_LOCAL_PORT,agent.TARGET_REMOTE_PORT=AGENT_PORT,PLANNER_PORT
    CuarmUdp.unpack_curobo_state=unpack_feedback
    try:yield
    finally:
        agent.TARGET_LOCAL_PORT,agent.TARGET_REMOTE_PORT=previous
        CuarmUdp.unpack_curobo_state=previous_decoder
