"""V4.1-only process receipt; cannot connect its XYZ client to fixed-Z V3."""
import hashlib
import json
import os
from pathlib import Path
from v3_1_readiness import fingerprint as base_fingerprint, process_start

ROOT = Path(__file__).resolve().parent.parent
PORT = 10186


def fingerprint():
    return hashlib.sha256(base_fingerprint().encode()+b''.join(p.read_bytes() for p in (
        ROOT/'dual_v2_2_upper_servo_v4_1.py', Path(__file__),
        ROOT/'scripts/v4_1_direct_servo.py'))).hexdigest()


def receipt_path():
    return Path(f'/tmp/cuarm-v41-{os.getuid()}-{PORT}.json')


def publish_ready(port):
    if port != PORT:
        raise ValueError('V4.1 must use its dedicated port 10186')
    path = receipt_path()
    tmp = path.with_suffix(f'.{os.getpid()}.tmp')
    tmp.write_text(json.dumps(dict(pid=os.getpid(), start=process_start(os.getpid()),
                                  fingerprint=fingerprint(), port=PORT)))
    tmp.replace(path)


def require_ready():
    try:
        value = json.loads(receipt_path().read_text())
        if (value['port'] != PORT or value['start'] != process_start(value['pid']) or
                value['fingerprint'] != fingerprint()):
            raise ValueError('Old V4.1 process')
    except (OSError, KeyError, ValueError, TypeError) as exc:
        raise ValueError('Start dual_v2_2_upper_servo_v4_1.py on port 10186; V4.1 cannot use the V3 fixed-Z process') from exc
