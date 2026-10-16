"""V3.4 soft forward lead, with unchanged independent motion hard limits."""
import json
import math
from pathlib import Path

EXTRA_DEFAULTS=dict(forward_lead_mm=20.,forward_ramp_seconds=.4,recovery_stable_seconds=.2)


def load_stream_config(path=Path(__file__).with_name('cart_book_grasp_v3_4_stream.json')):
    from cart_book_grasp_v3_2_stream import validate_stream_config
    data=json.loads(Path(path).read_text())
    missing=set(EXTRA_DEFAULTS)-set(data)
    if missing:raise ValueError('Missing V3.4 stream fields: '+', '.join(sorted(missing)))
    base={k:v for k,v in data.items() if k not in EXTRA_DEFAULTS}
    validate_stream_config(base)
    for key,lo,hi in [('forward_lead_mm',5.,30.),('forward_ramp_seconds',.2,1.),('recovery_stable_seconds',.2,.5)]:
        value=data[key]
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:
            raise ValueError('Invalid V3.4 setting: '+key)
    return data


class ForwardLead:
    """Reduce lead before a recovery gate; ramp increases after settling.

    ratio is the greatest normalized error relative to a soft recovery gate.
    This limits a position target's lead, not the backend's velocity limits.
    """
    def __init__(self,settings):
        self.maximum=settings['forward_lead_mm']
        self.ramp=settings['forward_ramp_seconds']
        self.value=0.;self.last_time=None

    def update(self,now,ratio,advancing):
        if not math.isfinite(ratio) or ratio<0:raise ValueError('Invalid forward recovery ratio')
        dt=0. if self.last_time is None else max(0.,now-self.last_time)
        self.last_time=now
        requested=self.maximum*max(0.,min(1.,2.*(1.-ratio))) if advancing else 0.
        # Reductions are immediate. Starting/resuming progress cannot restore
        # a far endpoint in a single frame after recovery.
        self.value=min(requested,self.value+self.maximum*dt/self.ramp)
        return self.value
