"""Local-only broadcaster. Its sole hardware owner drains the SDK continuously."""
import json
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit
from .protocol import PROTOCOL, MAX_AGE, encode


class Hub:
    def __init__(self, stream):
        self.stream = stream
        self.session = uuid.uuid4().hex
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.latest = None
        self.sequence = 0
        self.error = 'Waiting for camera'
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def run(self):
        while not self.stop.is_set():
            try:
                bgr, depth, stamp, metadata = self.stream.read()
                if not -.1 <= time.time() - stamp <= MAX_AGE:
                    raise ValueError('Camera frame is stale')
                self.sequence += 1
                token = f'{self.session}:{self.sequence}'
                payload = encode(bgr, depth, stamp, metadata, token)
                with self.condition:
                    self.latest = (token, stamp, payload)
                    self.error = ''
                    self.condition.notify_all()
            except Exception as error:
                with self.condition:
                    self.error = str(error)
                self.stop.wait(.02)

    def get(self, after='', wait=.5):
        with self.condition:
            self.condition.wait_for(lambda: self.stop.is_set() or
                (self.latest is not None and self.latest[0] != after and
                 -.1 <= time.time() - self.latest[1] <= MAX_AGE), timeout=wait)
            frame = self.latest
            if frame is None or frame[0] == after or not -.1 <= time.time() - frame[1] <= MAX_AGE:
                raise TimeoutError(self.error or 'Waiting for fresh RGB-D')
            return frame[2]

    def status(self):
        with self.condition:
            frame = self.latest
            age = time.time() - frame[1] if frame else None
            return dict(protocol=PROTOCOL, session=self.session,
                mode='live' if age is not None and -.1 <= age <= MAX_AGE else 'waiting_camera',
                sequence=self.sequence, frame_age_s=age, error=self.error, **self.stream.status())

    def close(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
        self.stream.close()
        self.worker.join(3)


def make_server(hub, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            self.connection.settimeout(2)
            path = urlsplit(self.path)
            try:
                if path.path == '/status':
                    data = json.dumps(hub.status()).encode()
                    mime = 'application/json'
                elif path.path == '/frame':
                    after = parse_qs(path.query).get('after', [''])[0]
                    data = hub.get(after)
                    mime = 'application/octet-stream'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)
            except TimeoutError:
                try:
                    self.send_error(503, 'Fresh RGB-D unavailable; retry')
                except OSError:
                    pass
            except (BrokenPipeError, ConnectionResetError):
                pass
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    return server
