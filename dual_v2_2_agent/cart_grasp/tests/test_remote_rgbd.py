import hashlib
import json
from pathlib import Path
import time
import numpy as np
import pytest
from seldon_cart_grasp.remote_rgbd import snapshot, validate_received, remote_camera_class


@pytest.fixture
def packet(tmp_path):
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    frame = tmp_path / 'latest.npz'
    bgr = np.zeros((530, 848, 3), np.uint8)
    bgr[..., 0] = 255
    np.savez_compressed(frame, bgr=bgr, depth=np.full((530, 848), .5, np.float32),
                        k=np.eye(3), d=np.zeros(5), upper_T_camera=np.eye(4),
                        depth_scale=1., boot_id=boot, serial='CV2TC5100060',
                        aligned_depth=True, monotonic_s=time.monotonic(),
                        stamp_s=100., sequence=1)
    header, payload = snapshot(frame, 'a'*32, boot)
    return frame, header, payload, boot


@pytest.mark.parametrize('bad', ['nonce', 'hash', 'delay', 'repeat', 'reboot'])
def test_transfer_rejects_bad_or_stale_frame(packet, bad):
    _, h, data, boot = packet
    previous = None
    rtt = .1
    if bad == 'nonce': h['nonce'] = 'b'*32
    if bad == 'hash': data += b'bad'
    if bad == 'delay': rtt = 2.
    if bad == 'repeat': previous = (boot, 1)
    if bad == 'reboot': previous = ('previous-boot', 0)
    with pytest.raises(ValueError): validate_received(h, data, 'a'*32, rtt, previous)


def test_source_rejects_previous_boot_and_stale_frame(packet):
    path, _, _, boot = packet
    with pytest.raises(ValueError): snapshot(path, 'a'*32, 'old-boot')
    with pytest.raises(ValueError): snapshot(path, 'a'*32, boot, now=time.monotonic()+3.)


def test_original_camera_contract_rgb_mm_and_half_root(packet):
    path, h, _, boot = packet
    h.update(rtt_s=.01, age_upper_bound_at_receive_s=.02,
             receiver_monotonic_s=time.monotonic(), receiver_boot_id=boot)
    (path.parent/'transport.json').write_text(json.dumps(h))
    class Original:
        def __init__(self): raise AssertionError('Must not open local camera')
    offset = np.eye(4)
    offset[2, 3] = .342
    camera = remote_camera_class(Original)(path.parent, offset)
    rgb, depth = camera.capture_image()
    assert rgb[0, 0].tolist() == [0, 0, 255]
    assert depth[0, 0] == 500
    assert np.allclose(camera.T, [0, 0, 342])
    with pytest.raises(ValueError): camera.capture_image()
    h['receiver_monotonic_s'] -= 10
    (path.parent/'transport.json').write_text(json.dumps(h))
    with pytest.raises(ValueError): camera.capture_image()
