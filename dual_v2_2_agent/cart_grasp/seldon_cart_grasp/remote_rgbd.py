"""Read-only latest-frame transfer over SSH; no cross-host clock assumptions.

The sender measures age against its own monotonic clock. The receiver adds the
full round-trip duration as a conservative bound. A received packet is never
assigned a new capture timestamp. No control sockets or motor API imports.
"""
import argparse
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import re
import select
import signal
import shlex
import struct
import subprocess
import sys
import time
import uuid
import numpy as np

MAX_BYTES = 8 * 1024 * 1024
SERIAL = "CV2TC5100060"
MAX_AGE_S = 1.5


class FrameNotReady(ValueError):
    """Discard this sample; a later fresh sample can still be accepted."""


def snapshot(path, nonce, boot_id, now=None):
    if not re.fullmatch(r"[0-9a-f]{32}", nonce):
        raise ValueError("Invalid request nonce")
    payload = Path(path).read_bytes()
    if not 0 < len(payload) <= MAX_BYTES:
        raise ValueError("RGB-D payload exceeds limit")
    with np.load(io.BytesIO(payload), allow_pickle=False) as frame:
        if str(frame["boot_id"]) != boot_id:
            raise ValueError("RGB-D belongs to a previous source boot")
        if str(frame["serial"]) != SERIAL or not bool(frame["aligned_depth"]):
            raise ValueError("Wrong camera or unaligned depth")
        age = (time.monotonic() if now is None else now) - float(frame["monotonic_s"])
        if not np.isfinite(age) or age < 0:
            raise ValueError("Invalid source frame timestamp")
        if age > 1.:
            raise FrameNotReady("Source RGB-D is stale")
        header = {"ok": True, "nonce": nonce, "bytes": len(payload),
                  "sha256": hashlib.sha256(payload).hexdigest(), "boot_id": boot_id,
                  "sequence": int(frame["sequence"]), "source_age_s": age,
                  "capture_unix_s": float(frame["stamp_s"]), "serial": SERIAL}
    return header, payload


def serve(path, inp=None, out=None):
    inp = sys.stdin.buffer if inp is None else inp
    out = sys.stdout.buffer if out is None else out
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    out.write(b"RGBD1\n")
    out.flush()
    while True:
        request = inp.readline(80)
        if not request:
            return
        nonce = request.decode("ascii").strip()
        try:
            header, payload = snapshot(path, nonce, boot_id)
        except Exception as error:
            header, payload = {"ok": False, "nonce": nonce, "bytes": 0, "error": str(error),
                               "retryable": isinstance(error, (FrameNotReady, FileNotFoundError))}, b""
        raw = json.dumps(header).encode()
        out.write(struct.pack("!I", len(raw)) + raw + payload)
        out.flush()


def read_exact(fd, count, deadline):
    parts = []
    while count:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
            raise TimeoutError("RGB-D SSH response timed out")
        part = os.read(fd, min(count, 65536))
        if not part:
            raise EOFError("RGB-D SSH connection closed")
        parts.append(part)
        count -= len(part)
    return b"".join(parts)


def validate_received(header, payload, nonce, rtt, previous=None):
    if header.get("nonce") != nonce:
        raise ValueError("Response does not match current request")
    if header.get("ok") is not True:
        if header.get("retryable") is True or header.get("error") == "Source RGB-D is stale":
            raise FrameNotReady(header.get("error", "Source unavailable"))
        raise ValueError(header.get("error", "Source unavailable"))
    if len(payload) != header["bytes"] or hashlib.sha256(payload).hexdigest() != header["sha256"]:
        raise ValueError("Corrupt RGB-D response")
    age = float(header["source_age_s"])
    if not np.isfinite(age) or not np.isfinite(rtt) or age < 0 or rtt < 0:
        raise ValueError("Invalid RGB-D age")
    if age + rtt > MAX_AGE_S:
        raise FrameNotReady("RGB-D too old after transport")
    if previous:
        if header["boot_id"] != previous[0]:
            raise ValueError("Source rebooted; establish a new observation session")
        if header["sequence"] < previous[1]:
            raise ValueError("Repeated/out-of-order RGB-D frame")
        if header["sequence"] == previous[1]:
            raise FrameNotReady("Repeated RGB-D frame; waiting for next sequence")
    with np.load(io.BytesIO(payload), allow_pickle=False) as f:
        if f["bgr"].shape != (530, 848, 3) or f["depth"].shape != (530, 848):
            raise ValueError("Wrong calibrated RGB-D resolution")
        if str(f["serial"]) != SERIAL or not bool(f["aligned_depth"]):
            raise ValueError("Invalid RGB-D source geometry")
        if str(f["boot_id"]) != header["boot_id"] or int(f["sequence"]) != header["sequence"]:
            raise ValueError("Inconsistent frame metadata")
        if not np.isclose(float(f["depth_scale"]), 1.):
            raise ValueError("Expected aligned float depth in metres")
        if f["bgr"].dtype != np.uint8 or f["depth"].dtype.kind != "f":
            raise ValueError("Expected uint8 BGR and floating-point depth")
        for key, shape in (("k", (3, 3)), ("d", (5,)), ("upper_T_camera", (4, 4))):
            if f[key].shape != shape or not np.isfinite(f[key]).all():
                raise ValueError("Invalid camera geometry: " + key)
    return age + rtt


def publish_cache(destination, record, payload):
    """Commit metadata and pixels together. Legacy files are diagnostic only."""
    raw = json.dumps(record).encode()
    for name, data in (("latest.npz", payload),
                       ("transport.json", raw),
                       ("latest.rgbd", struct.pack("!I", len(raw)) + raw + payload)):
        tmp = destination / (name + ".tmp")
        tmp.write_bytes(data)
        tmp.replace(destination / name)


def read_cache(directory):
    bundle = directory / "latest.rgbd"
    if bundle.exists():
        data = bundle.read_bytes()
        if not 4 < len(data) <= MAX_BYTES + 4100:
            raise ValueError("Invalid RGB-D cache bundle size")
        size = struct.unpack("!I", data[:4])[0]
        if not 0 < size <= 4096 or 4 + size >= len(data):
            raise ValueError("Invalid RGB-D cache bundle header")
        return json.loads(data[4:4+size]), data[4+size:]
    # Compatibility with an older puller: never accept mismatched generations.
    raw = (directory / "transport.json").read_bytes()
    payload = (directory / "latest.npz").read_bytes()
    record = json.loads(raw)
    if raw != (directory / "transport.json").read_bytes() or \
            hashlib.sha256(payload).hexdigest() != record["sha256"]:
        raise FrameNotReady("RGB-D cache update in progress")
    return record, payload


def cache_age(record):
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if record.get("receiver_boot_id") != boot:
        raise ValueError("RGB-D cache predates receiver boot")
    elapsed = time.monotonic() - float(record["receiver_monotonic_s"])
    # Recompute from original receive evidence; never reset capture time.
    age_at_receive = float(record["source_age_s"]) + float(record["rtt_s"])
    if not np.isfinite(elapsed) or elapsed < 0 or not np.isfinite(age_at_receive) or age_at_receive < 0:
        raise ValueError("Invalid RGB-D cache timestamps")
    if elapsed + age_at_receive > MAX_AGE_S:
        raise FrameNotReady("Remote RGB-D cache is stale")
    return elapsed + age_at_receive


def remote_camera_class(original_camera):
    """Original Agent camera interface, using remote data and no local USB.

    Inherits its projection helpers. RGB and millimetre depth match the original
    OrbbecCamera API; internal transfer remains BGR and metre depth.
    """
    class RemoteHeadCamera(original_camera):
        def __init__(self, directory, half_T_upper, capture_timeout_s=5.):
            from .geometry import transform
            self.directory = Path(directory)
            self.half_T_upper = transform(half_T_upper)
            self.previous = None
            if not np.isfinite(capture_timeout_s) or capture_timeout_s < 0:
                raise ValueError("Invalid capture timeout")
            self.capture_timeout_s = capture_timeout_s
            self.use_dual_color = False
            self._orbbec_depth_enabled = True
            self.pipeline = None
            self.w, self.h = 848, 530
            self.camparas_file = str(self.directory / "transport.json")
            self.left_image = self.right_image = None

        def _initialize_camera(self):
            return None

        def capture_image(self):
            deadline = time.monotonic() + self.capture_timeout_s
            while True:
                try:
                    return self._capture_once()
                except (FrameNotReady, FileNotFoundError) as error:
                    if time.monotonic() >= deadline:
                        raise FrameNotReady(
                            f"No fresh remote RGB-D within {self.capture_timeout_s:g}s: {error}; "
                            f"check {self.directory / 'agent_remote_rgbd_pull.log'}"
                        ) from error
                    time.sleep(min(.05, max(0., deadline-time.monotonic())))

        def _capture_once(self):
            from .geometry import transform
            record, payload = read_cache(self.directory)
            cache_age(record)
            validate_received(record, payload, record["nonce"], float(record["rtt_s"]), self.previous)
            with np.load(io.BytesIO(payload), allow_pickle=False) as f:
                half_T_camera = self.half_T_upper @ transform(f["upper_T_camera"])
                k, d = f["k"].copy(), f["d"].copy()
                rgb = f["bgr"][..., ::-1].copy()
                depth = f["depth"].copy() * 1000.
            cache_age(record)  # Include decoding time before returning pixels.
            self.K, self.D = k, d
            self.R, self.T = half_T_camera[:3, :3].copy(), half_T_camera[:3, 3].copy() * 1000.
            self.rgb_image = rgb
                # RobotRAM's grounding stage always saves both left_image and
                # right_image, even in the single-RGB Orbbec path.  Remote
                # RGB-D has one calibrated color stream, so expose the same
                # frame through both legacy aliases instead of leaving the
                # right image as None.
            self.rgb_right_image = rgb.copy()
            self.left_image, self.right_image = self.rgb_image, self.rgb_right_image
            self.depth_map = depth
            self.previous = (record["boot_id"], record["sequence"])
            return self.rgb_image, self.depth_map

        def close(self):
            # Shared source belongs to erazer; an Agent camera never stops it.
            return None
    return RemoteHeadCamera


def close_puller(process):
    if process is None:
        return
    try:
        process.stdin.close()
    except OSError:
        pass
    try:
        process.wait(timeout=1)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    process.stdout.close()


def pull(ssh_command, remote_command, destination, count=10, period=.5,
         on_frame=None, outage_timeout_s=15.):
    """Count accepted frames, retry transient failures, never publish old data.

    count=None runs until the caller stops it, without accumulating a list.
    Identity/geometry/hash errors are fatal; lost SSH and stale/duplicate frames
    retry only within the bounded no-fresh-frame window. Reconnect keeps the
    previous source boot/sequence so it cannot silently accept a source reset.
    """
    if count is not None and count < 1:
        raise ValueError("count must be positive or None")
    if not np.isfinite(period) or period <= 0 or not np.isfinite(outage_timeout_s) or outage_timeout_s <= 0:
        raise ValueError("period and outage_timeout_s must be positive")
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    previous = None
    records = []
    accepted = 0
    process = None
    last_good = time.monotonic()
    last_log = -float('inf')
    last_error = 'waiting for first frame'
    # Two Agents cannot overwrite each other's image/metadata cache.
    with (destination / '.pull.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Another RGB-D puller already owns '+str(destination)) from exc
        try:
            while count is None or accepted < count:
                started = time.monotonic()
                remaining = outage_timeout_s - (started-last_good)
                if remaining <= 0:
                    raise FrameNotReady(f'RGB-D pull stopped: no fresh frame for {outage_timeout_s:g}s; {last_error}')
                try:
                    if process is None:
                        process = subprocess.Popen([*ssh_command, remote_command], stdin=subprocess.PIPE,
                                                   stdout=subprocess.PIPE, bufsize=0)
                        if read_exact(process.stdout.fileno(), 6, started+min(10., remaining)) != b'RGBD1\n':
                            raise ValueError('Unexpected RGB-D server protocol')
                    nonce = uuid.uuid4().hex
                    process.stdin.write((nonce + '\n').encode())
                    deadline = min(time.monotonic()+5., last_good+outage_timeout_s)
                    size = struct.unpack('!I', read_exact(process.stdout.fileno(), 4, deadline))[0]
                    if not 0 < size <= 4096:
                        raise ValueError('Invalid RGB-D header size')
                    header = json.loads(read_exact(process.stdout.fileno(), size, deadline))
                    size = header.get('bytes', -1)
                    if type(size) is not int or not 0 <= size <= MAX_BYTES:
                        raise ValueError('Invalid RGB-D payload size')
                    payload = read_exact(process.stdout.fileno(), size, deadline)
                    received = time.monotonic()
                    age = validate_received(header, payload, nonce, received-started, previous)
                    record = {**header, 'rtt_s': received-started,
                              'age_upper_bound_at_receive_s': age,
                              'receiver_monotonic_s': received,
                              'receiver_boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                              'motion_commands_sent': False}
                    cache_age(record)
                except (FrameNotReady, OSError, EOFError) as error:
                    last_error = str(error)
                    if not isinstance(error, FrameNotReady):
                        close_puller(process)
                        process = None
                    if time.monotonic()-last_log >= 2.:
                        print(json.dumps({'event': 'RGBD_RETRY', 'reason': last_error,
                                          'freshness_limit_s': MAX_AGE_S}), flush=True)
                        last_log = time.monotonic()
                    time.sleep(max(.05, period-(time.monotonic()-started)))
                    continue
                # Publication errors are fatal (e.g. disk full), not network retries.
                publish_cache(destination, record, payload)
                previous = (header['boot_id'], header['sequence'])
                last_good = time.monotonic()
                accepted += 1
                if count is not None:
                    records.append(record)
                print(json.dumps({k: record[k] for k in ('sequence', 'rtt_s', 'age_upper_bound_at_receive_s')}), flush=True)
                if on_frame is not None:
                    on_frame(record)
                if count is None or accepted < count:
                    time.sleep(max(0., period-(time.monotonic()-started)))
        finally:
            close_puller(process)
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    server = sub.add_parser("serve")
    server.add_argument("--path", type=Path, required=True)
    client = sub.add_parser("pull")
    client.add_argument("--ssh-control", required=True)
    client.add_argument("--host", default="erazer@10.30.3.155")
    client.add_argument("--output", type=Path, required=True)
    client.add_argument("--count", type=int, default=10)
    client.add_argument("--continuous", action="store_true", help="Pull until stopped; no hour limit")
    client.add_argument("--period", type=float, default=.5)
    client.add_argument("--outage-timeout", type=float, default=15.)
    args = parser.parse_args()
    if args.mode == "serve":
        serve(args.path)
    else:
        if not 1 <= args.count <= 3600:
            parser.error("count must be 1..3600")
        root = "/home/erazer/Seldon/cart_grasp"
        command = ("cd " + shlex.quote(root) + " && OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "
                   "/home/erazer/Seldon/keyboard_cockpit/.venv/bin/python -m seldon_cart_grasp.remote_rgbd "
                   "serve --path runtime/rgbd/latest.npz")
        pull(["ssh", "-S", args.ssh_control, "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", args.host],
             command, args.output, None if args.continuous else args.count,
             period=args.period, outage_timeout_s=args.outage_timeout)


if __name__ == "__main__":
    # Agent cleanup terminates this child; close its SSH session too.
    def stop_signal(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_signal)
    try:
        main()
    except KeyboardInterrupt:
        pass
