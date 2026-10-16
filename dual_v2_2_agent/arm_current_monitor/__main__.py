"""Passive RT feedback -> loopback web charts. Python standard library only."""
import argparse
from collections import deque
import csv
import io
import json
import math
import os
from pathlib import Path
import pwd
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, parse_qs
from .pcap import packets

ROOT = Path(__file__).resolve().parents[1]

def decode_rt(payload, captured):
    """C++ pack_planner_state layout; MotorTor carries signed Realman current A.

    MotorCur is NOT populated in this driver. Never substitute it for MotorTor.
    Source unit verified through WhjMotor.cur_ (mA*0.001), RobotArms.get_current,
    DualV2_2Arm.getState and rt_control RobotState.MotorTor.
    """
    raw = payload.rstrip(b'\0').decode('ascii')
    if '\0' in raw or not raw.endswith('#'):
        raise ValueError('Invalid RT string framing')
    a = raw[:-1].split('#')
    if len(a) < 260 or int(a[4]) != 2:
        raise ValueError('Not a dual-arm RT PlannerState')
    rt_us = int(a[1])
    if rt_us <= 0 or not math.isfinite(captured):
        raise ValueError('Invalid feedback timestamp')
    arms = []
    i = 5
    for arm in range(2):
        n = int(a[i]); i += 1
        if n != 7:
            raise ValueError('Expected seven joints per arm')
        fields = {}
        for name in ('enabled', 'q', 'dq', 'joint_tor', 'motor_q', 'motor_dq', 'unused_motor_cur', 'current'):
            v = [float(x) for x in a[i:i+n]]; i += n
            if len(v) != n or not all(math.isfinite(x) for x in v):
                raise ValueError('Invalid '+name)
            fields[name] = v
        if any(x not in (0, 1) for x in fields['enabled']):
            raise ValueError('Invalid enable flags')
        i += 7*n  # JointPose
        ee = [float(x) for x in a[i:i+7]]; i += 14  # EE and ToolPose
        i += n  # interpolated_target
        if len(ee) != 7 or not all(math.isfinite(x) for x in ee):
            raise ValueError('Invalid end-effector pose')
        arms.append(dict(current=fields['current'], q=fields['q'], dq=fields['dq'],
                         enabled=fields['enabled'], ee=ee))
    return dict(t=captured, rt_us=rt_us, arms=arms)

class Store:
    def __init__(self, capacity=60000):
        self.lock = threading.Lock()
        self.rows = deque(maxlen=capacity)
        self.seq = 0
        self.boot = str(time.time_ns())
        self.rejected = 0
        self.last_rt = None
        self.last_capture = None
        self.last_mono = None
        self.error = ''
        self.capture_alive = False
        self.stopped = threading.Event()

    def add(self, row, now=None):
        now = time.time() if now is None else now
        if not 0 <= now-row['t'] <= .5 or abs(row['rt_us']/1e6-row['t']) > .5:
            self.rejected += 1
            return False
        with self.lock:
            if self.last_rt == row['rt_us'] or (self.last_capture is not None and row['t'] <= self.last_capture):
                self.rejected += 1
                return False
            self.seq += 1
            row['id'] = self.seq
            self.rows.append(row)
            self.last_rt = row['rt_us']
            self.last_capture = row['t']
            self.last_mono = time.monotonic()
            self.error = ''
        return True

    def snapshot(self, after=0):
        with self.lock:
            rows = list(self.rows)
            age = None if self.last_mono is None else time.monotonic()-self.last_mono
            recent = [x for x in rows if rows[-1]['t']-x['t'] <= 3] if rows else []
            hz = (len(recent)-1)/(recent[-1]['t']-recent[0]['t']) if len(recent)>1 else 0
            return dict(boot=self.boot, rows=[x for x in rows if x['id']>after or after>self.seq][-4000:], age_s=age,
                        live=age is not None and age<.5 and self.capture_alive,
                        hz=hz if age is not None and age<.5 else 0, rejected=self.rejected,
                        error=self.error, source='RT PlannerState.MotorTor / Realman current', unit='A',
                        retained_samples=len(rows))

def descendants(pid):
    """Only identify descendants of the capture process we started."""
    result = []
    try:
        children = Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
    except OSError:
        return result
    for p in children:
        result.append(int(p)); result.extend(descendants(int(p)))
    return result

def capture(store, port):
    # Capture a COPY via AF_PACKET; never bind a UDP feedback/control socket.
    user = pwd.getpwuid(os.getuid()).pw_name
    command = ['sudo','-n','/usr/bin/setsid','/usr/bin/tcpdump','--immediate-mode',
               '-i','lo','-n','-s','0','-U','-Z',user,'-w','-',
               f'udp and src host 127.0.0.1 and dst host 127.0.0.1 and dst port {port}']
    while not store.stopped.is_set():
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        owned = []
        def stderr_reader():
            for line in iter(proc.stderr.readline, b''):
                msg = line.decode(errors='replace').strip()
                print('capture:', msg, flush=True)
                if 'listening on' not in msg:
                    store.error = msg
        threading.Thread(target=stderr_reader, daemon=True).start()
        def shutdown_capture():
            while not store.stopped.wait(.2):
                if proc.poll() is not None:
                    return
                for p in [proc.pid]+descendants(proc.pid):
                    try:
                        if (Path(f'/proc/{p}').stat().st_uid == os.getuid()
                                and Path(f'/proc/{p}/comm').read_text().strip() == 'tcpdump'):
                            if p not in owned: owned.append(p)
                    except OSError:
                        pass
            for p in owned:
                try: os.kill(p, signal.SIGTERM)
                except ProcessLookupError: pass
        watcher = threading.Thread(target=shutdown_capture, daemon=True); watcher.start()
        store.capture_alive = True
        try:
            for stamp, data in packets(proc.stdout, port):
                if store.stopped.is_set(): break
                try: store.add(decode_rt(data, stamp))
                except (ValueError, IndexError, UnicodeError) as e:
                    store.rejected += 1; store.error = str(e)
        except (ValueError, KeyError, OSError) as e:
            store.error = str(e)
        finally:
            store.capture_alive = False
            proc.stdout.close()
            try: proc.wait(timeout=2)
            except subprocess.TimeoutExpired: pass
        if not store.stopped.is_set():
            store.error = '采集进程退出；2秒后重新连接。'+store.error
            store.stopped.wait(2)

def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def do_GET(self):
            u = urlsplit(self.path)
            if u.path == '/':
                body = Path(__file__).with_name('index.html').read_bytes(); kind='text/html; charset=utf-8'
            elif u.path == '/api/state':
                try: after=max(0,int(parse_qs(u.query).get('after',['0'])[0]))
                except ValueError: self.send_error(400); return
                body=json.dumps(store.snapshot(after),allow_nan=False).encode(); kind='application/json'
            elif u.path == '/api/export.csv':
                with store.lock: rows=list(store.rows)
                out=io.StringIO(); w=csv.writer(out)
                w.writerow(['unix_s','rt_unix_us','sample_id']+[f'{side}_J{j}_{unit}' for side in ('left','right') for unit in ('A','rad','rad_s') for j in range(1,8)])
                for row in rows:
                    w.writerow([row['t'],row['rt_us'],row['id']]+[v for a in row['arms'] for key in ('current','q','dq') for v in a[key]])
                body=out.getvalue().encode(); kind='text/csv; charset=utf-8'
            else: self.send_error(404); return
            self.send_response(200); self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store'); self.send_header('Content-Length',str(len(body)))
            if u.path.endswith('.csv'): self.send_header('Content-Disposition','attachment; filename="arm_current.csv"')
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError): pass
    return Handler

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',type=int,default=8793)
    ap.add_argument('--feedback-port',type=int,default=21101)
    args=ap.parse_args()
    store=Store()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(store))
    worker=threading.Thread(target=capture,args=(store,args.feedback_port),daemon=True)
    worker.start()
    print(f'Read-only arm current: http://127.0.0.1:{args.port}/ — no robot commands',flush=True)
    def stop(*unused): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try: server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt: pass
    finally:
        store.stopped.set(); server.server_close(); worker.join(timeout=4)

if __name__=='__main__': main()
