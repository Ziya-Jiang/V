"""Track only the displacement introduced by a successful depth fallback."""
from pathlib import Path
import numpy as np
import book_spine_recovery_v1_1 as recovery


def recover_stable_spine(result, *args):
    original = np.asarray(result['point_camera_m'], dtype=float).copy()
    saved = recovery._GEOMETRY_CONFIG_PATH
    try:
        recovery._GEOMETRY_CONFIG_PATH = Path(__file__).with_name('cart_book_grasp_v1_3_geometry_config.json')
        recovery.recover_stable_spine(result, *args)
    finally:
        recovery._GEOMETRY_CONFIG_PATH = saved
    # Save the actual old 3D estimate, not the whole-book centroid or image Y.
    result['spine_original_point_camera_m'] = original.tolist()


def base_z_compensation(selected, c2w):
    if selected.get('spine_center_recovery') != 'stable_region':
        return {'depth_recovery_delta_z_mm': 0.}
    original = np.asarray(selected['spine_original_point_camera_m'], dtype=float)
    current = np.asarray(selected['point_camera_m'], dtype=float)
    transform = np.asarray(c2w, dtype=float)
    if original.shape != (3,) or current.shape != (3,) or transform.shape != (4, 4):
        raise ValueError('Invalid center displacement or camera-to-base transform')
    if not all(np.isfinite(value).all() for value in (original, current, transform)):
        raise ValueError('Non-finite center displacement or transform')
    delta = transform[:3, :3] @ (current-original)*1000.
    return {'depth_recovery_delta_z_mm': float(delta[2]),
            'depth_recovery_center_delta_base_mm': delta.tolist(),
            'spine_original_point_camera_m': original.tolist()}
