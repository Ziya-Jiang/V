"""Sample the local RT controller's real interpolation implementation.

The library is built before opening UDP. Source changes invalidate the cache.
This models position interpolation, not physical tracking or network latency.
"""
import ctypes
import hashlib
from pathlib import Path
import subprocess
import os
import numpy as np

SCRIPTS = Path(__file__).resolve().parent
CONTROL = SCRIPTS.parents[1]
CONFIG = Path(os.environ.get('CUARM_CONFIG_ROOT', str(CONTROL.parent/'dual_v2_2_configuration')))
SOURCE = CONTROL/'cuarm_rt_control/cuarm_robotics/src'


def sources():
    return [SCRIPTS/'v3_1_panel_path_bridge.cpp', SOURCE/'interpolation.cpp', SOURCE/'interpolation.h',
            CONFIG/'robot_platform_utils/cpp/include/eigen_vector.h',
            CONFIG/'robot_platform_utils/cpp/include/cuarm_state.h']


class PanelPath:
    def __init__(self):
        digest = hashlib.sha256(b''.join(p.read_bytes() for p in sources())).hexdigest()[:20]
        cache = SCRIPTS.parent/'output/v31_panel_path'
        cache.mkdir(parents=True, exist_ok=True)
        library = cache/f'path_{digest}.so'
        if not library.exists():
            temporary = library.with_suffix(f'.{os.getpid()}.tmp')
            subprocess.run(['g++', '-std=c++17', '-O2', '-shared', '-fPIC',
                '-I/usr/include/eigen3', '-I'+str(CONFIG), '-I'+str(SOURCE),
                str(SCRIPTS/'v3_1_panel_path_bridge.cpp'), str(SOURCE/'interpolation.cpp'),
                '-o', str(temporary)], check=True, capture_output=True, text=True, timeout=60)
            temporary.replace(library)
        self.library = ctypes.CDLL(str(library))
        ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        self.library.v31_sample_path.argtypes = [ctypes.c_int, ctypes.c_int, ptr, ptr, ctypes.c_double,
                                               ptr, ctypes.c_double, ctypes.c_int, ptr, ptr]
        self.library.v31_sample_path.restype = ctypes.c_int
        self.library.v31_path_error.restype = ctypes.c_char_p
        self.library.v31_raw_create.argtypes=[ctypes.c_int,ptr,ptr,ctypes.c_double,ctypes.c_double]
        self.library.v31_raw_create.restype=ctypes.c_void_p
        self.library.v31_raw_destroy.argtypes=[ctypes.c_void_p]
        self.library.v31_raw_step.argtypes=[ctypes.c_void_p,ptr,ctypes.c_int,ptr]
        self.library.v31_raw_step.restype=ctypes.c_int

    def sample(self, start, points, interval, limits, sample_dt=.005):
        start, points, limits = [np.ascontiguousarray(v, dtype=np.float64) for v in (start, points, limits)]
        if (start.ndim != 1 or points.ndim != 2 or points.shape[1] != len(start)
                or limits.shape != start.shape or len(points) < 1
                or not all(np.isfinite(v).all() for v in (start, points, limits))
                or not np.isfinite([interval,sample_dt]).all() or interval <= 0 or sample_dt <= 0
                or (limits <= 0).any()):
            raise ValueError('Invalid panel path inputs')
        output = np.empty((20001, len(start)), dtype=np.float64)
        duration = np.zeros(1)
        n = self.library.v31_sample_path(len(start), len(points), start, points, float(interval),
                                        limits, sample_dt, len(output), output, duration)
        if n <= 0 or not np.isfinite(output[:n]).all():
            raise ValueError(f'Panel path sampling failed ({n}): {self.library.v31_path_error().decode()}')
        return output[:n].copy(), float(duration[0])


class RawPanel:
    """Offline stateful RAW filter; each advance returns all RT substeps."""
    def __init__(self,start,limits,target_time=0.,sample_dt=.001):
        self.library=PanelPath().library
        start,limits=[np.ascontiguousarray(x,dtype=np.float64) for x in (start,limits)]
        if (start.ndim!=1 or limits.shape!=start.shape or not len(start) or
                not np.isfinite(np.r_[start,limits,target_time,sample_dt]).all() or
                (limits<=0).any() or target_time<0 or sample_dt<=0):
            raise ValueError('Invalid RAW filter inputs')
        self.joints=len(start)
        self.handle=self.library.v31_raw_create(self.joints,start,limits,sample_dt,target_time)
        if not self.handle:raise ValueError(self.library.v31_path_error().decode())

    def advance(self,target,ticks):
        target=np.ascontiguousarray(target,dtype=np.float64)
        if (not self.handle or target.shape!=(self.joints,) or not np.isfinite(target).all()
                or not isinstance(ticks,int) or not 1<=ticks<=10000):
            raise ValueError('Invalid RAW filter step')
        output=np.empty((ticks,self.joints))
        result=self.library.v31_raw_step(self.handle,target,ticks,output)
        if result!=ticks or not np.isfinite(output).all():
            raise ValueError(self.library.v31_path_error().decode())
        return output

    def close(self):
        if self.handle:self.library.v31_raw_destroy(self.handle);self.handle=None

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
