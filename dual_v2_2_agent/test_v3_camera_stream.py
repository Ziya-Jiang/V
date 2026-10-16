"""Real child-process IPC using a synthetic camera; never opens a device."""
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path
import unittest
from unittest.mock import Mock,patch
import numpy as np
from cart_book_grasp_v3_camera_stream import PersistentWristCapture,stop_capture_process


FAKE_WORKER='''
import sys,time,json
import numpy as np
from pathlib import Path
sys.path.insert(0,sys.argv[1])
import cart_book_grasp_v3_camera_stream as s
mode,output=sys.argv[2:4]
if mode=='stale':
    req=json.loads(sys.stdin.readline())
    np.savez(output,rgb=np.zeros((12,16,3),np.uint8),captured_at=req['after']-1,
             read_started_at=req['after']-2,request_id=req['id'])
    print(json.dumps(dict(id=req['id'])),flush=True)
    time.sleep(30)
else:
    class Camera:
        def __init__(self,*args):
            self.n=0
            with open(output+'.opens','a') as f:f.write('open\\n')
        def isOpened(self):return True
        def set(self,*args):return True
        def release(self):pass
        def read(self):
            if mode=='blocked':time.sleep(30)
            time.sleep(.01);self.n+=1
            if mode=='failed':return False,None
            return True,np.full((12,16,3),self.n%250,np.uint8)
    s.cv2.VideoCapture=Camera
    s.worker('/fake',16,12,4,output)
'''


class FakeStream(PersistentWristCapture):
    def __init__(self,mode='normal',timeout=3.):
        super().__init__(dict(device='/fake',width=16,height=12,warmup_frames=4,capture_timeout_seconds=timeout))
        self.mode=mode
    def _start(self):
        self.directory=tempfile.TemporaryDirectory(prefix='test-wrist-stream-')
        self.output=Path(self.directory.name)/'frame.npz'
        self.error_file=open(Path(self.directory.name)/'log','w+b')
        self.process=subprocess.Popen([sys.executable,'-c',FAKE_WORKER,str(Path(__file__).resolve().parent),self.mode,str(self.output)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.error_file,bufsize=0)
        self.buffer=b''


class StreamTests(unittest.TestCase):
    def test_reuses_camera_and_each_frame_is_after_new_request(self):
        stream=FakeStream();self.addCleanup(stream.close)
        a=stream.capture();pid=stream.process.pid
        self.assertGreaterEqual(int(a['rgb'][0,0,0]),5)
        time.sleep(.08);requested=time.monotonic();b=stream.capture()
        self.assertEqual(stream.process.pid,pid)
        self.assertEqual(Path(str(stream.output)+'.opens').read_text().splitlines(),['open'])
        self.assertGreaterEqual(b['read_started_at'],requested)
        self.assertGreater(b['captured_at'],a['captured_at'])
        self.assertFalse(np.array_equal(a['rgb'],b['rgb']))
        process=stream.process;stream.close();self.assertIsNotNone(process.poll())

    def test_blocked_read_timeout_preserves_capture_error_and_reaps_child(self):
        stream=FakeStream('blocked',.8);self.addCleanup(stream.close)
        started=time.monotonic()
        with self.assertRaisesRegex(ValueError,'fresh-frame timeout after 0.8s'):stream.capture()
        self.assertLess(time.monotonic()-started,2.5)
        self.assertIsNone(stream.process);self.assertIsNone(stream.directory)

    def test_disconnect_reports_original_read_error(self):
        stream=FakeStream('failed');self.addCleanup(stream.close)
        with self.assertRaisesRegex(ValueError,'V4L2 read failed/disconnected'):stream.capture()
        self.assertIsNone(stream.process)

    def test_old_frame_is_rejected_even_with_matching_request_id(self):
        stream=FakeStream('stale');self.addCleanup(stream.close)
        with self.assertRaisesRegex(ValueError,'Stale or mismatched'):stream.capture()
        self.assertIsNone(stream.process)

    def test_operator_interrupt_closes_worker_without_replacing_exception(self):
        stream=FakeStream('blocked');self.addCleanup(stream.close);calls=[0]
        def checkpoint():
            calls[0]+=1
            if calls[0]>8:raise KeyboardInterrupt('operator stop')
        with self.assertRaisesRegex(KeyboardInterrupt,'operator stop'):stream.capture(checkpoint)
        self.assertIsNone(stream.process)

    def test_cleanup_timeout_is_bounded_and_does_not_escape(self):
        process=Mock();process.poll.return_value=None
        process.wait.side_effect=subprocess.TimeoutExpired('fake',.5)
        with self.assertWarnsRegex(UserWarning,'has not exited'):
            self.assertFalse(stop_capture_process(process))
        process.terminate.assert_called_once();process.kill.assert_called_once()
        self.assertEqual(process.wait.call_count,2)

    def test_legacy_cleanup_does_not_mask_initial_timeout(self):
        from cart_book_grasp_v2_dual_view import capture_wrist
        process=Mock();process.poll.return_value=None
        process.wait.side_effect=subprocess.TimeoutExpired('fake',.5)
        c=dict(device='/fake',width=16,height=12,warmup_frames=4,capture_timeout_seconds=.1)
        with patch('cart_book_grasp_v2_dual_view.subprocess.Popen',return_value=process), \
             patch('cart_book_grasp_v2_dual_view.time.monotonic',side_effect=[0.,1.]), \
             self.assertWarns(UserWarning), self.assertRaisesRegex(ValueError,'相机读取超时'):
            capture_wrist(c,0.)

    def test_unreaped_worker_blocks_reopening(self):
        stream=PersistentWristCapture(dict(capture_timeout_seconds=1.))
        process=Mock();stream.process=process;stream.cleanup_pending=True
        with patch('cart_book_grasp_v3_camera_stream.stop_capture_process',return_value=False), \
             patch.object(stream,'_start') as start, \
             self.assertRaisesRegex(ValueError,'cannot reopen'):
            stream.capture()
        start.assert_not_called();process.stdin.write.assert_not_called()
        stream.process=None  # The fake process has no resources.


    def test_worker_warms_only_once_then_returns_first_postrequest_frame(self):
        import cart_book_grasp_v3_camera_stream as module
        frames=[];n=[0];requests=[dict(id=1,after=0.)];clock=[1.]
        class Camera:
            def __init__(self,*args):pass
            def isOpened(self):return True
            def set(self,*args):pass
            def release(self):pass
            def read(self):
                n[0]+=1;return True,np.full((12,16,3),n[0],np.uint8)
        def receive(fd,count):
            if len(frames)==2:return b''
            return (json.dumps(requests.pop(0))+'\n').encode()
        def output(message,**kw):
            reply=json.loads(message)
            if 'error' in reply:raise AssertionError(reply['error'])
            if len(frames)==1:requests.append(dict(id=2,after=0.))
        def now():clock[0]+=.001;return clock[0]
        with patch.object(module.cv2,'VideoCapture',Camera), \
             patch.object(module.select,'select',side_effect=lambda *a:([0],[],[]) if requests or len(frames)==2 else ([],[],[])), \
             patch.object(module.os,'read',side_effect=receive),patch.object(module.os,'replace'), \
             patch.object(module.time,'monotonic',side_effect=now), \
             patch.object(module.np,'savez',side_effect=lambda *a,**kw:frames.append(int(kw['rgb'][0,0,0]))), \
             patch('builtins.print',side_effect=output):
            module.worker('/fake',16,12,4,'unused')
        self.assertEqual(frames,[5,6])


if __name__=='__main__':unittest.main()
