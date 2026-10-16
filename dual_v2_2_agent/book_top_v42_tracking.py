"""One camera owner continuously tracks between planner-paced lift waypoints."""
import threading
import time
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_feature_centerline import FeaturePointsExhausted, CenterlineGeometryError


class TrackingStream:
    def __init__(self,camera,lock,*,max_age_seconds=2.,allow_point_loss=False,allow_geometry_recovery=False,clock=time.monotonic):
        self.camera,self.lock,self.clock=camera,lock,clock
        self.allow_point_loss=allow_point_loss;self.point_loss=None
        self.allow_geometry_recovery=allow_geometry_recovery
        self.max_age=max_age_seconds;self.stop=threading.Event();self.condition=threading.Condition()
        self.latest=None;self.error=None;self.count=0;self.last_stamp=float('-inf')
        self.thread=threading.Thread(target=self._run,name='v42-wrist-tracking',daemon=True)
    def start(self):self.thread.start();return self
    def _checkpoint(self):
        if self.stop.is_set():raise InterruptedError('V4.2 tracking stopped')
    def _run(self):
        try:
            while not self.stop.is_set():
                frame=self.camera.capture(self._checkpoint)
                stamp=float(frame['captured_at']);age=self.clock()-stamp
                if stamp<=self.last_stamp or not 0<=age<=self.max_age:
                    raise ServoError('V4.2 stale/non-monotonic wrist frame')
                if self.point_loss is None:
                    try:observation=self.lock.observe(frame['rgb'])
                    except FeaturePointsExhausted as error:
                        if not self.allow_point_loss:raise
                        self.point_loss=dict(tracking_valid=False,point_loss=True,
                            remaining_points=error.count,minimum_points=error.minimum,reason=str(error))
                        observation=dict(self.point_loss)
                    except CenterlineGeometryError as error:
                        if not self.allow_geometry_recovery:raise
                        self.point_loss=dict(tracking_valid=False,geometry_failure=True,reason=str(error))
                        observation=dict(self.point_loss)
                else:
                    # Keep checking real camera freshness; never label old pixels
                    # as a new visual measurement during historical continuation.
                    observation=dict(self.point_loss)
                self.last_stamp=stamp
                with self.condition:
                    self.count+=1
                    self.latest=(frame,observation,self.count)
                    self.condition.notify_all()
        except BaseException as error:
            if not self.stop.is_set():
                with self.condition:self.error=error;self.condition.notify_all()
    def check(self):
        with self.condition:
            if self.error is not None:raise ServoError(f'V4.2 tracking unavailable: {self.error}') from self.error
            if self.latest is not None and self.clock()-self.latest[0]['captured_at']>self.max_age:
                raise ServoError('V4.2 wrist tracking stalled')
    def after(self,stamp,timeout,checkpoint=lambda:None):
        deadline=self.clock()+timeout
        while True:
            checkpoint();self.check()
            with self.condition:
                if self.latest is not None and self.latest[0]['captured_at']>stamp:return self.latest
                if self.clock()>=deadline:raise ServoError('V4.2 waiting for fresh tracked frame timed out')
                self.condition.wait(.02)
    def close(self):
        self.stop.set()
        with self.condition:self.condition.notify_all()
        if self.thread.ident is not None:self.thread.join(3.)
        if self.thread.is_alive():raise ServoError('V4.2 camera tracking worker did not exit')
