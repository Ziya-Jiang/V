"""Manage only the dedicated camera service, never robot/controller processes."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request
from .protocol import PROTOCOL

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'RobotRAM/output/head_rgbd_service'
URL = 'http://127.0.0.1:8792'
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def status():
    with OPENER.open(URL + '/status', timeout=.5) as response:
        value = json.load(response)
    if value.get('protocol') != PROTOCOL:
        raise ValueError('Port 8792 is occupied by another service')
    return value


def running(record):
    try:
        proc = Path(f'/proc/{int(record["pid"])}')
        return ((proc / 'stat').read_text().split()[21] == record['start_ticks'] and
                b'head_rgbd\x00serve' in (proc / 'cmdline').read_bytes())
    except (OSError, ValueError, KeyError):
        return False


def record():
    try:
        return json.loads((RUNTIME / 'service.json').read_text())
    except (OSError, ValueError):
        return {}


def ensure_service():
    try:
        return status()
    except OSError:
        pass
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / 'startup.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            return status()
        except OSError:
            pass
        if not running(record()):
            with (RUNTIME / 'service.log').open('ab') as log:
                subprocess.Popen([sys.executable, '-m', 'head_rgbd', 'serve'], cwd=ROOT,
                    stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            try:
                return status()
            except OSError:
                time.sleep(.1)
        raise RuntimeError(f'Head RGB-D service unavailable; see {RUNTIME}/service.log')


def stop_service():
    value = record()
    if running(value):
        os.kill(value['pid'], signal.SIGTERM)
        for _ in range(80):
            if not running(value):
                return
            time.sleep(.1)
        raise RuntimeError('Camera service still shutting down; no forced kill issued')
