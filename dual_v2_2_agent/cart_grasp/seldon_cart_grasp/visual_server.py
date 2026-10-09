"""Authenticated read-only hand feedback UI. Selection changes tracking, not motors."""
import argparse,asyncio,contextlib,fcntl,json,os,secrets,sys,time,threading
from collections import OrderedDict
from pathlib import Path
from urllib.parse import urlparse
import cv2,numpy as np
from aiohttp import web
from .visual_feedback import VisualFeedback,annotate
from .rgbd_bridge import packet_valid
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=8792);p.add_argument('--mode',choices=['markers','manual'],default='markers');p.add_argument('--ros',action='store_true');a=p.parse_args();cv2.setNumThreads(1);out=ROOT/'runtime/hand_feedback';out.mkdir(parents=True,exist_ok=True);out.chmod(0o700);lock=(out/'server.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);token=secrets.token_urlsafe(24);sessions=set();engine=VisualFeedback(a.mode);cache=OrderedDict();boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip();report={'state':'waiting','mode':a.mode,'hands':{},'hardware_output':False};last=None;stop=threading.Event();worker=None
    def origin_ok(request):
        origin=request.headers.get('Origin')
        return bool(origin and urlparse(origin).netloc==request.host and urlparse(origin).scheme in ('http','https'))
    @web.middleware
    async def auth(request,handler):
        if request.method=='POST' and not origin_ok(request):raise web.HTTPForbidden(text='Origin mismatch')
        if request.path not in ('/','/session') and request.cookies.get('visual_session') not in sessions:raise web.HTTPUnauthorized(text='Authentication required')
        response=await handler(request);response.headers['Cache-Control']='no-store';return response
    async def index(request):return web.FileResponse(ROOT/'web/hand_feedback.html')
    async def session(request):
        data=await request.json()
        if not isinstance(data.get('token'),str) or not secrets.compare_digest(data['token'],token):raise web.HTTPUnauthorized(text='Invalid token')
        if len(sessions)>=32:raise web.HTTPTooManyRequests()
        cookie=secrets.token_urlsafe(24);sessions.add(cookie);response=web.json_response({'ok':True});response.set_cookie('visual_session',cookie,httponly=True,samesite='Strict');return response
    async def state(request):return web.json_response(report)
    async def frame(request):
        try:seq=int(request.query.get('seq',last))
        except (TypeError,ValueError):raise web.HTTPServiceUnavailable(text='No frame')
        if seq not in cache:raise web.HTTPGone(text='Frame expired')
        if time.monotonic()-float(cache[seq]['monotonic_s'])>1.5:raise web.HTTPServiceUnavailable(text='Camera stale')
        return web.Response(body=cache[seq]['jpeg'],content_type='image/jpeg')
    async def mode(request):
        nonlocal engine,last,report
        value=(await request.json()).get('mode')
        if value not in ('markers','manual'):raise web.HTTPBadRequest(text='Unknown mode')
        engine=VisualFeedback(value);last=None;report={'state':'waiting','mode':value,'hands':{},'hardware_output':False};return web.json_response({'ok':True})
    async def clear(request):
        nonlocal report,last
        engine.clear();last=None;report={'state':'waiting','mode':engine.mode,'hands':{},'hardware_output':False};return web.json_response({'ok':True})
    async def select(request):
        data=await request.json();seq=data.get('sequence')
        if type(seq) is not int or seq not in cache:raise web.HTTPConflict(text='Displayed frame expired; refresh and select again')
        if time.monotonic()-float(cache[seq]['monotonic_s'])>1.5:raise web.HTTPConflict(text='Selection too old; select again on a current frame')
        try:engine.initialize_manual(data.get('role'),cache[seq]['bgr'],data.get('box'))
        except (ValueError,TypeError,cv2.error) as error:raise web.HTTPBadRequest(text=str(error))
        return web.json_response({'ok':True,'hardware_output':False})
    async def markers(request):return web.FileResponse(out/'left7_right19_30mm_300dpi.png')
    async def process(app):
        nonlocal report,last
        while True:
            try:
                with np.load(ROOT/'runtime/rgbd/latest.npz',allow_pickle=False) as z:data={k:z[k] for k in z.files}
                packet_valid(data,time.monotonic(),boot);seq=int(data['sequence'])
                if seq!=last:
                    handles=None
                    with contextlib.suppress(OSError,ValueError):handles=json.loads((ROOT/'runtime/rgbd/live_proposals.json').read_text())
                    report=engine.analyze(data,time.monotonic(),handles);ok,jpeg=cv2.imencode('.jpg',annotate(data['bgr'],report),[cv2.IMWRITE_JPEG_QUALITY,85])
                    if ok:cache[seq]={**data,'jpeg':jpeg.tobytes()}
                    while len(cache)>8:cache.popitem(last=False)
                    last=seq
                else:report={**report,'sample_age_s':time.monotonic()-float(data['monotonic_s'])}
            except Exception as error:
                engine.clear();last=None;cache.clear();report={'state':'stale_camera','mode':engine.mode,'hands':{},'reason':str(error),'hardware_output':False}
            tmp=out/'latest.tmp';tmp.write_text(json.dumps(report,indent=2,allow_nan=False));tmp.replace(out/'latest.json');await asyncio.sleep(.25)
    def ros_worker():
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        rclpy.init();node=Node('seldon_visual_hand_feedback');pub=node.create_publisher(String,'/cart_grasp/visual_hand_feedback',2)
        try:
            while not stop.wait(.25):
                msg=String();msg.data=json.dumps(report,allow_nan=False);pub.publish(msg)
        finally:node.destroy_node();rclpy.shutdown()
    async def startup(app):
        nonlocal worker
        app['processor']=asyncio.create_task(process(app))
        if a.ros:worker=threading.Thread(target=ros_worker,daemon=True);worker.start()
    async def cleanup(app):
        stop.set();app['processor'].cancel()
        with contextlib.suppress(asyncio.CancelledError):await app['processor']
        if worker:await asyncio.to_thread(worker.join,3)
    app=web.Application(middlewares=[auth],client_max_size=4096);app.router.add_get('/',index);app.router.add_post('/session',session);app.router.add_get('/api/state',state);app.router.add_get('/frame.jpg',frame);app.router.add_post('/api/mode',mode);app.router.add_post('/api/select',select);app.router.add_post('/api/clear',clear);app.router.add_get('/markers.png',markers);app.on_startup.append(startup);app.on_cleanup.append(cleanup)
    fd=os.open(out/'web.json',os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
    with os.fdopen(fd,'w') as stream:json.dump({'pid':os.getpid(),'port':a.port,'url':f'http://10.30.3.155:{a.port}/#token={token}','local_url':f'http://127.0.0.1:{a.port}/#token={token}','read_only':True},stream)
    web.run_app(app,host=a.host,port=a.port,access_log=None)
if __name__=='__main__':main()
