"""Read one complete distance snapshot; never sends robot commands."""
import json
from pathlib import Path

DISTANCE_KEYS = frozenset((
    'hover_above_top_mm', 'forward_insertion_mm',
    'final_z_offset_from_top_mm', 'lower_offset_mm',
    'coarse_step_mm', 'descent_step_mm', 'withdraw_mm',
))


def read_distances(path):
    path = Path(path)
    try:
        values = json.loads(path.read_text())
        if not isinstance(values, dict) or set(values) != DISTANCE_KEYS:
            raise ValueError('Expected exactly: ' + ', '.join(sorted(DISTANCE_KEYS)))
        return values
    except (OSError, ValueError) as error:
        raise ValueError(f'V4.1 distance config {path}: {error}') from error
