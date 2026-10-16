"""Persistent V4L2 worker with bounded, post-request fresh-frame capture."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import time
import warnings

import cv2
import numpy as np


def stop_capture_process(process):
    """Bound cleanup without replacing a capture/interrupt exception."""
    if process.poll() is not None:return True
    for signal in (process.terminate,process.kill):
        try:
            signal();process.wait(timeout=.5);return True
        except ProcessLookupError:
            return True
        except subprocess.TimeoutExpired:
            pass
        except OSError as error:
            warnings.warn(f'Camera worker cleanup: {error}')
    warnings.warn(f'Camera worker PID {process.pid} has not exited after terminate/kill; do not reopen device yet')
    return False


class PersistentWristCapture:
    def __init__(self,config):
        self.config=dict(config);self.process=None;self.directory=None
        self.error_file=None;self.request_id=0;self.buffer=b''
        self.cleanup_pending=False

    def _start(self):
        if self.process is not None:
            raise ValueError('Previous wrist worker has not exited; cannot reopen camera')
        self.directory=tempfile.TemporaryDirectory(prefix='book-wrist-stream-')
        self.output=Path(self.directory.name)/'frame.npz'
        self.error_file=open(Path(self.directory.name)/'worker.log','w+b')
        c=self.config
        args=[sys.executable,str(Path(__file__).resolve()),'--worker',c['device'],
              str(c['width']),str(c['height']),str(c['warmup_frames']),str(self.output)]
        self.process=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                                      stderr=self.error_file,bufsize=0)
        self.buffer=b''

    def _details(self):
        if self.error_file is None:return ''
        self.error_file.seek(0,os.SEEK_END);size=self.error_file.tell()
        self.error_file.seek(max(0,size-1500))
        return self.error_file.read().decode(errors='replace').strip()

    def capture(self,checkpoint=lambda:None):
        checkpoint();requested=time.monotonic()
        try:
            if self.cleanup_pending:
                raise ValueError('Previous wrist worker has not exited; cannot reopen camera')
            if self.process is None:self._start()
            if self.process.poll() is not None:
                raise ValueError('Wrist camera worker exited: '+self._details())
            self.request_id+=1;request_id=self.request_id
            self.process.stdin.write((json.dumps(dict(id=request_id,after=requested))+'\n').encode())
            deadline=requested+self.config['capture_timeout_seconds']
            while True:
                checkpoint()
                if time.monotonic()>=deadline:
                    raise ValueError(f'Wrist camera fresh-frame timeout after {self.config["capture_timeout_seconds"]:.1f}s; '+self._details())
                ready,_,_=select.select([self.process.stdout],[],[],.025)
                if ready:
                    chunk=os.read(self.process.stdout.fileno(),4096)
                    if not chunk:raise ValueError('Wrist camera worker disconnected: '+self._details())
                    self.buffer+=chunk
                    while b'\n' in self.buffer:
                        line,self.buffer=self.buffer.split(b'\n',1);reply=json.loads(line)
                        if 'error' in reply:raise ValueError('Wrist camera read failed: '+reply['error'])
                        if reply.get('id')!=request_id:continue
                        with np.load(self.output,allow_pickle=False) as data:
                            rgb=data['rgb'].copy();stamp=float(data['captured_at'])
                            started=float(data['read_started_at']);saved_id=int(data['request_id'])
                        now=time.monotonic()
                        if (saved_id!=request_id or not np.isfinite([started,stamp]).all()
                                or not requested<=started<=stamp<=now or now-stamp>1.):
                            raise ValueError('Stale or mismatched wrist frame; capture cancelled')
                        if rgb.dtype!=np.uint8 or rgb.shape!=(self.config['height'],self.config['width'],3):
                            raise ValueError('Wrist frame resolution/format differs from calibration')
                        checkpoint()
                        return dict(rgb=rgb,captured_at=stamp,read_started_at=started,
                                    pair_gap_seconds=stamp-requested)
        except BaseException:
            try:self.close()
            except Exception as error:warnings.warn(f'Camera cleanup failed: {error}')
            raise

    def close(self):
        if self.process is not None:
            if not stop_capture_process(self.process):
                self.cleanup_pending=True;return False
            for pipe in (self.process.stdin,self.process.stdout):
                if pipe is not None:pipe.close()
            self.process=None
            self.cleanup_pending=False
        if self.error_file is not None:self.error_file.close();self.error_file=None
        if self.directory is not None:self.directory.cleanup();self.directory=None
        return True

    def __del__(self):
        try:self.close()
        except Exception:pass


def worker(device,width,height,warmup,output):
    cap=None
    try:
        cap=cv2.VideoCapture(device,cv2.CAP_V4L2)
        if not cap.isOpened():raise ValueError('Cannot open wrist camera; check connection/device ownership')
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,int(width));cap.set(cv2.CAP_PROP_FRAME_HEIGHT,int(height))
        cap.set(cv2.CAP_PROP_BUFFERSIZE,1)  # Best effort; drain continuously between requests.
        pending=None;warmup_remaining=max(0,int(warmup));buffer=b''
        while True:
            ready,_,_=select.select([sys.stdin.buffer],[],[],0)
            if ready:
                chunk=os.read(sys.stdin.fileno(),4096)
                if not chunk:return
                buffer+=chunk
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1);pending=json.loads(line)
            started=time.monotonic();ok,frame=cap.read();stamp=time.monotonic()
            if not ok or frame is None:raise ValueError('V4L2 read failed/disconnected')
            if warmup_remaining:
                warmup_remaining-=1;continue  # Startup/reopen only, never per request.
            if pending is None:continue  # Drain continuously while the arm is moving.
            if started<pending['after']:continue  # Still require a post-request read.
            temporary=str(output)+'.tmp.npz'
            np.savez(temporary,rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB),captured_at=stamp,
                     read_started_at=started,request_id=pending['id'])
            os.replace(temporary,output)
            print(json.dumps(dict(id=pending['id'])),flush=True);pending=None
    except Exception as error:
        print(json.dumps(dict(error=f'{type(error).__name__}: {error}')),flush=True)
    finally:
        if cap is not None:cap.release()


if __name__=='__main__':
    if len(sys.argv)!=7 or sys.argv[1]!='--worker':raise SystemExit('Internal persistent camera worker only')
    worker(*sys.argv[2:])
