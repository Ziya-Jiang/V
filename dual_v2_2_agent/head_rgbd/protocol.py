"""Lossless paired frames; never pickle or transmit a rendered AR image."""
import io
import json
import time
import numpy as np

PROTOCOL = 'head-rgbd-v1'
MAX_AGE = .5


def encode(bgr, depth, stamp, metadata, token):
    metadata=dict(metadata);extra={}
    if '_native_bgr' in metadata:
        extra['native_bgr']=metadata.pop('_native_bgr')
        extra['native_depth']=metadata.pop('_native_depth')
    header = dict(protocol=PROTOCOL, token=token, captured_unix_s=stamp,
                  color_encoding='bgr8', depth_encoding='32FC1', depth_units='mm', metadata=metadata)
    out = io.BytesIO()
    np.savez(out, bgr=np.asarray(bgr, dtype=np.uint8), depth=np.asarray(depth, dtype=np.float32),
             header=np.array(json.dumps(header, allow_nan=False)),**extra)
    return out.getvalue()


def decode(payload, now=None):
    with np.load(io.BytesIO(payload), allow_pickle=False) as data:
        header = json.loads(str(data['header']))
        bgr, depth = data['bgr'].copy(), data['depth'].copy()
        if 'native_bgr' in data:
            header['native_bgr']=data['native_bgr'].copy()
            header['native_depth']=data['native_depth'].copy()
    if (header['protocol'] != PROTOCOL or header['color_encoding'] != 'bgr8'
            or header['depth_encoding'] != '32FC1' or header['depth_units'] != 'mm'):
        raise ValueError('Unsupported shared RGB-D format')
    m = header['metadata']
    shape = (int(m['height']), int(m['width']))
    if bgr.shape != (*shape, 3) or bgr.dtype != np.uint8 or depth.shape != shape or depth.dtype != np.float32:
        raise ValueError('Shared RGB-D dimensions or dtype mismatch')
    if 'native_bgr' in header:
        native_shape=(int(m['native_height']),int(m['native_width']))
        if (header['native_bgr'].shape!=(*native_shape,3) or header['native_bgr'].dtype!=np.uint8
                or header['native_depth'].shape!=native_shape or header['native_depth'].dtype!=np.float32):
            raise ValueError('Native shared RGB-D dimensions or dtype mismatch')
    age = (time.time() if now is None else now) - float(header['captured_unix_s'])
    if not -.1 <= age <= MAX_AGE:
        raise ValueError(f'Shared RGB-D frame is stale ({age:.3f}s)')
    return bgr, depth, header
