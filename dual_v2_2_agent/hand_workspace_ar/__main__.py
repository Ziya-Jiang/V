import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request
from .core import ROOT,RUNTIME,atomic_json

def running(record):
    try:
        pid=int(record['pid']);proc=Path(f'/proc/{pid}')
        # PID reuse must never stop an unrelated process.
        return (proc/'stat').read_text().split()[21]==record['start_ticks'] and b'hand_workspace_ar\x00serve' in (proc/'cmdline').read_bytes()
    except (OSError,ValueError,KeyError):return False

def main():
    parser=argparse.ArgumentParser(description='Read-only head RGB-D hand-height workspace AR')
    parser.add_argument('action',choices=['start','serve','status','stop'])
    parser.add_argument('--port',type=int,default=8791)
    parser.add_argument('--state',type=Path,default=RUNTIME/'state.json')
    parser.add_argument('--duration',type=float)
    args=parser.parse_args();RUNTIME.mkdir(parents=True,exist_ok=True);pidfile=RUNTIME/'viewer.json'
    if args.action=='serve':
        with open(RUNTIME/'viewer.lock','w') as lock:
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:raise SystemExit('An AR viewer already owns this runtime')
            atomic_json(pidfile,dict(pid=os.getpid(),start_ticks=Path('/proc/self/stat').read_text().split()[21],port=args.port))
            from .live import serve
            try:serve(args.port,args.state,args.duration)
            finally:pidfile.unlink(missing_ok=True)
        return
    try:record=json.loads(pidfile.read_text())
    except (OSError,ValueError):record={}
    active=running(record)
    if args.action=='stop':
        if active:
            os.kill(record['pid'],signal.SIGTERM)
            for _ in range(60):
                if not running(record):break
                time.sleep(.1)
            if running(record):raise SystemExit('Viewer still shutting down; no forced kill issued')
        print('AR viewer stopped; robot processes unchanged.');return
    if args.action=='start' and not active:
        with open(RUNTIME/'viewer.log','a') as log:
            cmd=[sys.executable,'-m','hand_workspace_ar','serve','--port',str(args.port),'--state',str(args.state)]
            if args.duration is not None:cmd+=['--duration',str(args.duration)]
            child=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=log,start_new_session=True)
        for _ in range(50):
            if child.poll() is not None:raise SystemExit(f'Viewer failed; inspect {RUNTIME}/viewer.log')
            try:
                record=json.loads(pidfile.read_text())
                with urllib.request.urlopen(f'http://127.0.0.1:{record["port"]}/api/status',timeout=.3):break
            except (OSError,ValueError):time.sleep(.1)
        active=running(record)
    if active:
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{record["port"]}/api/status',timeout=2) as response:status=json.load(response)
        except OSError as e:status={'mode':'unavailable','message':str(e)}
        print(json.dumps(dict(running=True,url=f'http://127.0.0.1:{record["port"]}',**status),ensure_ascii=False,indent=2))
    else:print(json.dumps({'running':False,'log':str(RUNTIME/'viewer.log')}))

if __name__=='__main__':main()
