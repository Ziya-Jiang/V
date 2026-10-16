"""Hardware-free transport tests, using real loopback HTTP."""
import ast
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import queue
import threading
import time
import unittest
import numpy as np
from .client import Subscriber, CalibratedCamera
from .protocol import encode, decode
from .service import Hub, make_server

META = dict(serial='CV2TC5100060', width=6, height=4, K=np.eye(3).tolist(),
            D=[0., 0., 0., 0., 0.], T_dummy_world_camera_mm=np.eye(4).tolist(), depth_alignment='test D2C')
CAL = dict(path='test', w=6, h=4, K=np.eye(3), D=np.zeros(5), R=np.eye(3), T=np.zeros((3, 1)))

class FakeStream:
    def __init__(self):
        self.frames = queue.Queue()
        self.closed = False
    def push(self, value=17):
        bgr = np.full((4, 6, 3), value, np.uint8)
        bgr[:, :, 0] = value + 1
        self.frames.put((bgr, np.full((4, 6), value + .125, np.float32), time.time(), META))
    def read(self):
        try:
            return self.frames.get(timeout=.05)
        except queue.Empty:
            raise TimeoutError('Test camera disconnected')
    def status(self):
        return dict(camera_reconnects=0)
    def close(self):
        self.closed = True

class SharedTests(unittest.TestCase):
    def setUp(self):
        self.stream = FakeStream()
        self.hub = Hub(self.stream)
        self.server = make_server(self.hub, 0)
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        self.worker = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': .01})
        self.worker.start()
    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.worker.join(2)
        self.hub.close()

    def test_independent_subscribers_receive_identical_lossless_pair(self):
        a, b = [Subscriber(self.url, autostart=False) for _ in range(2)]
        self.stream.push()
        with ThreadPoolExecutor(2) as pool:
            fa, fb = pool.submit(a.read), pool.submit(b.read)
            rgb_a, depth_a, ha = fa.result()
            rgb_b, depth_b, hb = fb.result()
        self.assertEqual(ha['token'], hb['token'])
        np.testing.assert_array_equal(rgb_a, rgb_b)
        np.testing.assert_array_equal(depth_a, depth_b)
        self.assertEqual(depth_a[0, 0], 17.125)
        rgb_a[:] = 0
        self.assertTrue(rgb_b.any())
        a.close()
        self.stream.push(42)
        self.assertEqual(b.read()[1][0, 0], 42.125)
        self.assertFalse(self.stream.closed)

    def test_latest_frame_skips_backlog_and_camera_recovers(self):
        fast, slow = [Subscriber(self.url, autostart=False) for _ in range(2)]
        for value in range(10, 15):
            self.stream.push(value)
            self.assertEqual(fast.read()[1][0, 0], value + .125)
        self.assertEqual(slow.read()[1][0, 0], 14.125)
        time.sleep(.55)
        with self.assertRaises(TimeoutError):
            slow.read()
        self.stream.push(80)
        self.assertEqual(slow.read()[1][0, 0], 80.125)
        self.assertEqual(fast.read()[1][0, 0], 80.125)

    def test_subscriber_survives_publisher_restart_sequence_reset(self):
        subscriber = Subscriber(self.url, autostart=False)
        self.stream.push(10)
        token = subscriber.read()[2]['token']
        port = self.server.server_port
        self.server.shutdown()
        self.server.server_close()
        self.worker.join(2)
        self.hub.close()
        self.stream = FakeStream()
        self.hub = Hub(self.stream)
        self.server = make_server(self.hub, port)
        self.worker = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': .01})
        self.worker.start()
        self.stream.push(20)
        self.assertNotEqual(subscriber.read()[2]['token'], token)

    def test_grasp_contract_color_units_geometry_and_local_correction(self):
        self.stream.push(30)
        camera = CalibratedCamera(CAL, warmup_frames=1, subscriber=Subscriber(self.url, False))
        self.assertEqual(camera.rgb_image[0, 0].tolist(), [30, 30, 31])
        self.assertEqual(camera.depth_map[0, 0], 30.125)
        camera.T[2, 0] = 123
        self.stream.push(40)
        camera.capture_image()
        self.assertEqual(camera.c2w[2, 3], 123)
        self.assertEqual(META['T_dummy_world_camera_mm'][2][3], 0)
        with self.assertRaisesRegex(ValueError, 'resolution'):
            camera._validate({**META, 'width': 99})
        with self.assertRaisesRegex(ValueError, 'calibration'):
            camera._validate({**META, 'K': (np.eye(3)*2).tolist()})
        camera.close()
        self.assertFalse(self.stream.closed)

    def test_reject_stale_future_or_wrong_shape_packet(self):
        bgr, depth = np.zeros((4, 6, 3), np.uint8), np.ones((4, 6), np.float32)
        payload = encode(bgr, depth, 100, META, 'session:1')
        decode(payload, now=100.1)
        for now in (102, 99):
            with self.assertRaisesRegex(ValueError, 'stale'):
                decode(payload, now=now)
        with self.assertRaisesRegex(ValueError, 'dimensions'):
            decode(encode(bgr, depth[:2], 100, META, 'session:2'), now=100.1)

    def test_common_grasp_factory_uses_subscriber_without_hardware_import(self):
        from unittest.mock import patch
        path = Path(__file__).resolve().parents[1] / 'dual_v2_2_agent_pcl.py'
        tree = ast.parse(path.read_text())
        factory = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'start_calibrated_orbbec')
        scope = {}
        exec(compile(ast.Module(body=[factory], type_ignores=[]), str(path), 'exec'), scope)
        with patch('head_rgbd.client.CalibratedCamera', return_value='subscriber') as constructor:
            self.assertEqual(scope['start_calibrated_orbbec'](CAL, 3), 'subscriber')
            constructor.assert_called_once_with(CAL, warmup_frames=3)

if __name__ == '__main__':
    unittest.main()
