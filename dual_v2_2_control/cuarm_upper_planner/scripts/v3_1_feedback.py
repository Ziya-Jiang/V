"""V3.1-only pose provenance on the existing state channel (same host clock).

Times refer to panel UDP receipt and FK completion, not encoder hardware time.
No device or socket is opened by this module.
"""
import math
from cuarm_udp import CuarmUdp

_pack_state = CuarmUdp.pack_curobo_state
_unpack_state = CuarmUdp.unpack_curobo_state
_MARKER = 'V31POSE1#'


def pack_feedback(udp, state):
    source = float(state.pose_sample_monotonic)
    completed = float(state.pose_fk_completed_monotonic)
    if not (math.isfinite(source) and math.isfinite(completed) and 0 < source <= completed):
        raise ValueError('V3.1 invalid pose provenance')
    return _pack_state(udp, state) + _MARKER + f'{source!r}#{completed!r}#'


def unpack_feedback(udp, raw):
    if not raw:
        return None
    if raw.count(_MARKER) != 1:
        raise ValueError('V3.1 feedback missing pose provenance; restart both V3.1 programs')
    body, metadata = raw.rsplit(_MARKER, 1)
    fields = metadata.split('#')
    if len(fields) != 3 or fields[-1] != '':
        raise ValueError('V3.1 malformed pose provenance')
    source, completed = map(float, fields[:2])
    if not (math.isfinite(source) and math.isfinite(completed) and 0 < source <= completed):
        raise ValueError('V3.1 invalid pose provenance')
    state = _unpack_state(udp, body)
    state.pose_sample_monotonic = source
    state.pose_fk_completed_monotonic = completed
    return state
