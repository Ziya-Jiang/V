"""No sockets, cameras or robot commands: offline contract/control tests."""
import contextlib
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import cv2
import numpy as np
import servor


class SemanticTests(unittest.TestCase):
    def task(self):
        return dict(status='selected', confidence=.9, reason='box label',
            patch_polygon=[[100,80],[500,80],[500,400],[100,400]],
            grasp_center=[300,240], alignment_axis=[[300,180],[300,300]])

    def test_generic_object_payload(self):
        args=SimpleNamespace(task=None,object='工具盒',description='对准正面的标签',model='gpt-6-astra',key_file=None)
        with patch('cart_book_grasp_v2_gpt.GPTBookClient.select',return_value=self.task()) as api:
            result=servor.select_task(np.zeros((480,640,3),np.uint8),args)
        self.assertEqual(result['object'],'工具盒')
        self.assertIn('工具盒',api.call_args.args[0]['input'][0]['content'][0]['text'])

    def test_reject_ambiguous_and_outside(self):
        t=self.task();t['status']='ambiguous'
        with self.assertRaises(ValueError):servor.validate_task(t,(480,640,3))
        t=self.task();t['grasp_center']=[520,200]
        with self.assertRaises(ValueError):servor.validate_task(t,(480,640,3))

    def test_semantic_center_not_texture_mean(self):
        H=np.array([[1.,0,12],[0,1,0],[0,0,1]])
        landmarks=np.array([[300,240],[300,200],[300,280]],np.float32)
        valid=np.array([[270,180],[350,180],[350,310],[270,310]])
        r=dict(tracking_valid=True,features=[dict(valid=True,xy=p.tolist()) for p in valid],
               image_size=[640,480],homography_reference_to_frame=H)
        error,pts=servor.measurement(r,landmarks)
        self.assertAlmostEqual(error[0],-7.5)
        self.assertAlmostEqual(error[1],0)
        r['tracking_valid']=False
        with self.assertRaises(ValueError):servor.measurement(r,landmarks)

    def test_measured_probe_and_bounded_correction(self):
        j=np.array([[4,.4],[-.2,-1.2]])
        poses=np.array([[1.4,.02],[1.5,1.15]])
        start=np.array([35.,10.])
        measured=[start+j@q for q in poses]
        np.testing.assert_allclose(servor.fit_jacobian(start,measured,poses),j,atol=1e-10)
        e=start.copy()
        for _ in range(30):
            d=servor.correction(e,j)
            self.assertLessEqual(abs(d[0]),4.00001);self.assertLessEqual(abs(d[1]),1.00001)
            e+=j@d
        self.assertLess(np.linalg.norm(e),.1)
        with self.assertRaises(ValueError):servor.fit_jacobian(start,measured,[[0,0],[0,0]])

    def test_anchor_selection_surrounds_center_and_rejects_one_sided_texture(self):
        polygon=np.float32([[50,30],[150,30],[150,470],[50,470]])
        points=np.float32([[x,y] for x in (65,85,115,135) for y in range(60,450,30)])
        # Strong/early points on one side cannot monopolize the fixed IDs.
        anchors=servor.supported_anchors(points,polygon,[100,250],[200,500])
        self.assertEqual(len(anchors),8)
        self.assertGreaterEqual(cv2.pointPolygonTest(cv2.convexHull(anchors),(100.,250.),True),0)
        self.assertTrue(all(any(np.array_equal(p,q) for q in points) for p in anchors))
        np.testing.assert_array_equal(anchors,servor.supported_anchors(points,polygon,[100,250],[200,500]))
        with self.assertRaisesRegex(ValueError,'cannot support'):
            servor.supported_anchors(points[points[:,0]<100],polygon,[120,250],[200,500])

    def test_failed_geometry_requests_same_template_redetection(self):
        tracker=SimpleNamespace(redetect_every=10)
        with patch.object(servor,'measurement',side_effect=ValueError('Tracking lost: occluded')):
            with self.assertRaises(ValueError):servor.control_measurement(tracker,{},None)
        self.assertEqual(tracker.redetect_every,1)
        with patch.object(servor,'measurement',return_value=(np.zeros(2),None)):
            servor.control_measurement(tracker,{},None)
        self.assertEqual(tracker.redetect_every,10)


class LoopTests(unittest.TestCase):
    def run_sim(self,lose=False,drift=False):
        import cart_book_grasp_v3_2_runtime as rt
        clock=SimpleNamespace(now=100.)
        def now():clock.now+=.001;return clock.now
        q=np.array([500.,0.,-10.]);base=q.copy();roll=[0.];commands=[];events=[];seen=[0]
        J=np.array([[4,.2],[-.1,-1.2]])
        class Robot:
            target_command=SimpleNamespace(arm_tool_command=np.zeros((2,7)))
            def master_control(self,**kw):
                target=np.asarray(kw['right_arm']);commands.append(kw)
                if kw['wait_for_completion']:
                    q[:]+=.6*(target[:3]-q);roll[0]+=.6*(target[5]-roll[0])
                else:q[:]=target[:3];roll[0]=target[5]
                if drift and len(commands)>=3:q[2]-=4
                return True
        robot=Robot()
        class Transport:
            def __init__(self,robot,base,c,s,**kw):self.s=s;self.pending=None
            def read(self):return dict(q=q.copy(),roll=roll[0],angles=np.array([0,0,roll[0]]))
            def ready(self):self.pending=None;return True
            def abort(self):events.append(dict(event='hold'));self.pending=None
        @contextlib.contextmanager
        def connect():yield robot,np.zeros(3)
        def capture(t,*a,**kw):
            clock.now+=.12
            return dict(rgb=np.zeros((20,20,3),np.uint8),captured_at=now()),t.read(),None,None,0
        def measure(*a):
            seen[0]+=1
            if lose and seen[0] in (5,6):raise ValueError('Tracking lost: occluded')
            return np.array([40.,8.])+J@np.array([q[1],roll[0]]),None
        tracker=SimpleNamespace(meta={'image_size':[20,20]}, process=lambda *a,**kw:{})
        with patch.object(servor,'robot_connection',connect),patch.object(rt,'StreamTransport',Transport),\
             patch.object(rt,'capture_paired_frame',capture),patch.object(servor,'measurement',measure),\
             patch.object(servor.time,'monotonic',now):
            if drift:
                with self.assertRaisesRegex(ValueError,'Fixed X/Z drift'):
                    servor.live_control(None,tracker,None,SimpleNamespace(seconds=120),events.append)
            else:servor.live_control(None,tracker,None,SimpleNamespace(seconds=120),events.append)
        for cmd in commands:
            np.testing.assert_allclose(np.asarray(cmd['right_arm'])[[0,2]],base[[0,2]])
            self.assertNotIn('right_gripper',cmd)
            self.assertIn(cmd['action'],('book_v3_xyz_servo','book_v3_1_stream'))
        return events

    def test_converges_with_actual_probe_shortfall(self):
        self.assertIn('aligned',[e['event'] for e in self.run_sim()])

    def test_loss_holds_then_same_target_recovers(self):
        names=[e['event'] for e in self.run_sim(lose=True)]
        self.assertIn('tracking_hold',names);self.assertIn('same_target_reacquired',names)
        self.assertIn('aligned',names)

    def test_fixed_axis_drift_aborts(self):
        events=self.run_sim(drift=True)
        self.assertEqual(events[-1]['event'],'hold')


if __name__=='__main__':unittest.main()
