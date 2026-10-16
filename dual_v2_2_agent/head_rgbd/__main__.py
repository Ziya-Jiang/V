import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import threading
from .lifecycle import RUNTIME, ensure_service, status, stop_service


def serve():
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / 'owner.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('A head RGB-D owner is already running')
        from hand_workspace_ar.stream import CameraStream
        from .service import Hub, make_server
        hub = Hub(CameraStream())
        try:
            server = make_server(hub, 8792)
        except Exception:
            hub.close()
            raise
        server.timeout = .2
        pidfile = RUNTIME / 'service.json'
        pidfile.write_text(json.dumps(dict(pid=os.getpid(),
            start_ticks=Path('/proc/self/stat').read_text().split()[21])))
        stopping = threading.Event()
        signal.signal(signal.SIGTERM, lambda *_: stopping.set())
        signal.signal(signal.SIGINT, lambda *_: stopping.set())
        try:
            while not stopping.is_set():
                server.handle_request()
        finally:
            server.server_close()
            hub.close()
            pidfile.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description='Shared head RGB-D source (no robot commands)')
    parser.add_argument('action', choices=['start', 'serve', 'status', 'stop'])
    args = parser.parse_args()
    if args.action == 'serve':
        serve()
    elif args.action == 'stop':
        stop_service()
        print('Camera service stopped. Stop subscribers first to prevent their automatic recovery.')
    else:
        try:
            result = ensure_service() if args.action == 'start' else status()
        except OSError as error:
            result = dict(mode='stopped', error=str(error))
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
