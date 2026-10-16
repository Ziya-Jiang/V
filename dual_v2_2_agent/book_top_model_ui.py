"""Read-only V1_3-style diagnostics and end-of-cycle keyboard handling."""
import json
import socket
import threading
import time

import numpy as np


def viewer_ready(port=9876,timeout=2.):
    """Bounded local readiness check before installing a network sink."""
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            with socket.create_connection(('127.0.0.1',port),timeout=min(.1,max(.001,deadline-time.monotonic()))):
                return True
        except OSError:
            time.sleep(min(.05,max(0,deadline-time.monotonic())))
    return False


class CycleKeys:
    """Adapter for the existing V1_3 cbreak listener; no motion callbacks."""
    def __init__(self):
        self.stop=threading.Event()

    def request_stop(self):
        if not self.stop.is_set():
            self.stop.set()
            print('\n收到 s：当前轮次结束后退出，不再启动下一轮。',flush=True)

    def request_pause(self):
        print('\n本粗定位入口不执行 p 松爪复位；中断当前轮请按 Ctrl+C。',flush=True)

    def request_retry(self):
        # Do not queue a new descent, clear backend faults or open the gripper.
        pass

    def wait_after_failure(self):
        print('本轮失败，已尝试保持，不自动松爪或复位。按 s 退出；'
              '重新执行前请检查现场及后端状态。',flush=True)
        while not self.stop.wait(.1):
            pass


class CoarseVisualization:
    """Independent recording. Observe existing state reads, never receive UDP."""
    def __init__(self,out,spawn_viewer=True):
        self.recording=None;self.warning=False;self.sequence=0
        self.last_sample=-float('inf');self.points=[];self.closed=False
        try:
            import rerun as rr
            import rerun.blueprint as rrb
            self.rr=rr
            self.recording=rr.RecordingStream('book_grasp_v41_head_coarse')
            path=out/'head_coarse.rrd'
            # Set both sinks explicitly: attaching a viewer must not replace
            # the durable file recording.
            sinks=[rr.FileSink(path)]
            if spawn_viewer:
                try:
                    # There is intentionally no global recording. Without
                    # recording=..., SDK spawn silently skips viewer startup.
                    rr.spawn(connect=False,recording=self.recording)
                    if viewer_ready():
                        sinks.append(rr.GrpcSink())
                    else:
                        print('Rerun查看器未就绪：仅保存head_coarse.rrd，'
                              '不建立gRPC连接；可稍后用Rerun打开记录。',flush=True)
                except Exception as exc:
                    print(f'Rerun窗口未打开，保留文件记录：{exc}',flush=True)
            self.recording.set_sinks(*sinks)
            self.recording.send_blueprint(rrb.Blueprint(rrb.Horizontal(
                rrb.Vertical(
                    rrb.Spatial2DView(origin='/head/rgb',name='Head RGB / spine / upper edge'),
                    rrb.Spatial2DView(origin='/head/depth',name='Aligned depth')),
                rrb.Vertical(
                    rrb.Spatial3DView(origin='/world',name='Book / stage target / measured TCP'),
                    rrb.TextDocumentView(origin='/status',name='Current action / failure'),
                    rrb.TextDocumentView(origin='/geometry',name='Measured book geometry'))),
                auto_layout=False,auto_views=False))
            self.recording.log('world',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
            self.recording.log('world/base',rr.Arrows3D(
                origins=np.zeros((3,3)),vectors=np.eye(3)*.1,
                colors=[[255,60,60],[60,255,60],[60,100,255]],labels=['X','Y','Z']),static=True)
            self.event(dict(event='waiting_for_first_reset' if spawn_viewer else 'recording_started'))
            print('Rerun记录：',path,flush=True)
        except Exception as exc:
            self._warn(exc)

    def _warn(self,exc):
        if not self.warning:
            self.warning=True
            print(f'Rerun显示/记录异常（不改变运动检查）：{exc}',flush=True)

    def safe(self,method,*args):
        if self.recording is None:return
        try:getattr(self,method)(*args)
        except Exception as exc:self._warn(exc)

    def tick(self):
        self.sequence+=1
        self.recording.set_time('event',sequence=self.sequence)

    def event(self,r):
        self.tick()
        self.recording.log('status',self.rr.TextDocument(json.dumps(
            r,indent=2,ensure_ascii=True,default=lambda v:np.asarray(v).tolist())))
        if r.get('event')=='stage':
            start=np.asarray(r['start_tcp_mm'])/1000;goal=np.asarray(r['target_tcp_mm'])/1000
            self.recording.log('world/stage/path',self.rr.LineStrips3D([[start,goal]],colors=[255,190,30]))
            self.recording.log('world/stage/target',self.rr.Points3D([goal],radii=.006,
                colors=[255,190,30],labels=[r['name']+' TCP target']))

    def state(self,s):
        if not s:return
        stamp=s.get('pose_sample_monotonic')
        if stamp is None or not np.isfinite(stamp) or not 0<=time.monotonic()-stamp<=.25:return
        if stamp-self.last_sample<.1:return
        p=np.asarray(s['right_arm']['position'],float)
        if p.shape!=(3,) or not np.isfinite(p).all():return
        self.last_sample=stamp;self.points.append(p.copy());self.tick()
        self.recording.log('world/tcp/current',self.rr.Points3D([p],colors=[50,255,130],radii=.004))
        if len(self.points)>1:
            self.recording.log('world/tcp/measured',self.rr.LineStrips3D([self.points],colors=[50,255,130]))

    def frame(self,rgb,depth,k,d,c2w,selection=None):
        from book_top_tilt_perception import rays
        self.tick();rr=self.rr;r=self.recording
        r.log('head/rgb/selection',rr.Clear(recursive=True))
        r.log('head/rgb',rr.Image(rgb))
        r.log('head/depth',rr.DepthImage(np.asarray(depth,dtype=np.float32),meter=1000.))
        y,x=np.indices(depth.shape);y=y[::6,::6].ravel();x=x[::6,::6].ravel()
        z=np.asarray(depth[y,x],float);valid=np.isfinite(z)&(z>100)&(z<2500)
        x=x[valid];y=y[valid];z=z[valid]
        points=rays(np.c_[x,y],k,d)*z[:,None]
        points=(points@np.asarray(c2w)[:3,:3].T+np.asarray(c2w)[:3,3])/1000
        r.log('world/scene',rr.Points3D(points,colors=rgb[y,x],radii=.0015))
        if selection is not None:
            for key,col in [('book_quad',[60,160,255]),('spine_quad',[255,190,30])]:
                q=np.asarray(selection[key]);r.log('head/rgb/selection/'+key,
                    rr.LineStrips2D([np.vstack([q,q[0]])],colors=col))
            q=np.asarray(selection['spine_quad'])
            top=np.asarray(selection.get('top_center_uv',q[:2].mean(0)))
            center=np.asarray(selection.get('spine_center_uv',q.mean(0)))
            r.log('head/rgb/selection/top',rr.Points2D([top],colors=[255,60,60],radii=5))
            r.log('head/rgb/selection/centers',rr.Points2D([center,top],
                colors=[[255,190,30],[255,60,60]],radii=5,labels=['spine center','upper edge center']))

    def geometry(self,g):
        self.tick()
        self.recording.log('geometry',self.rr.TextDocument(json.dumps(g,indent=2,
            default=lambda v:np.asarray(v).tolist())))
        for key,col in [('spine_center_base_mm',[255,190,30]),('top_base_mm',[255,60,60]),
                        ('depth_center_base_mm',[60,160,255])]:
            self.recording.log('world/book/'+key,self.rr.Points3D(
                [np.asarray(g[key])/1000],colors=col,radii=.005,labels=[key]))

    def close(self):
        if not self.closed and self.recording is not None:
            self.closed=True
            try:self.recording.flush(timeout_sec=2.)
            finally:self.recording.disconnect()
