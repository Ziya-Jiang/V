"""Experimental semantic comparison of spine, cover and page-edge masks.

    python cart_book_grasp_v1_tiral.py --assume-head-camera-calibrated

The spelling 'tiral' is intentional. Existing v1 behavior is unchanged.
The trial preserves the auto cycle, stop key, depth gates and motion checks.
"""
import json
from pathlib import Path

import cv2
import numpy as np

import book_grasp_1_2_local as book
import cart_book_grasp_v1 as runner

CONFIG = Path(__file__).with_name('cart_book_grasp_v1_tiral_semantic_config.json')


def load_config():
    values = json.loads(CONFIG.read_text())
    names = {'spine_min_score', 'book_min_score', 'competitor_min_score',
             'book_coverage_min', 'competitor_overlap_max'}
    if set(values) != names:
        raise ValueError('Invalid semantic configuration fields')
    for key, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not np.isfinite(value) or not 0 < value <= 1:
            raise ValueError(f'Invalid semantic threshold: {key}')
    return values


def normalize_detections(detections, kind, shape):
    records = []
    for index, (score, mask, _) in enumerate(detections):
        mask = np.asarray(mask, dtype=np.uint8)
        if not np.isfinite(score) or mask.ndim != 2:
            continue
        if mask.shape != shape:
            mask = cv2.resize(mask, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
        mask = mask.astype(bool)
        if mask.any():
            records.append(dict(kind=kind, index=index, score=float(score), mask=mask))
    return records


def screen_spines(groups, config):
    """Reject semantic conflicts. Scores across prompts are not probabilities."""
    accepted, rows = [], []
    books = [r for r in groups['book'] if r['score'] >= config['book_min_score']]
    competitors = [r for kind in ('cover', 'pages') for r in groups[kind]
                   if r['score'] >= config['competitor_min_score']]
    for candidate in groups['spine']:
        mask = candidate['mask']
        row = dict(kind='spine', index=candidate['index'], score=candidate['score'], reasons=[])
        if candidate['score'] < config['spine_min_score']:
            row['reasons'].append('low spine score')
        try:
            mask = book._spine_frame(mask)[0]
        except (ValueError, np.linalg.LinAlgError) as error:
            row['reasons'].append(str(error))
        area = np.count_nonzero(mask)
        coverage = [np.count_nonzero(mask & r['mask'])/area for r in books]
        best = int(np.argmax(coverage)) if coverage else None
        row['book_coverage'] = float(coverage[best]) if best is not None else 0.
        row['book_index'] = books[best]['index'] if best is not None else None
        if row['book_coverage'] < config['book_coverage_min']:
            row['reasons'].append('not associated with a reliable whole-book mask')
        row['competitors'] = []
        for competitor in competitors:
            overlap = np.count_nonzero(mask & competitor['mask'])/area
            row['competitors'].append(dict(kind=competitor['kind'], index=competitor['index'],
                                           score=competitor['score'], spine_overlap=float(overlap)))
            if overlap >= config['competitor_overlap_max']:
                row['reasons'].append(f"ambiguous with {competitor['kind']} #{competitor['index']}: {overlap:.0%} overlap")
        row['decision'] = 'rejected' if row['reasons'] else 'semantic pass; geometry/depth pending'
        rows.append(row)
        if not row['reasons']:
            yy, xx = np.nonzero(mask)
            accepted.append((candidate['score'], mask,
                             np.array([xx.min(), yy.min(), xx.max()+1, yy.max()+1], dtype=np.float32)))
    return accepted, rows


class TrialDetector(book._TiltDetector):
    def __init__(self, geometry, visualization=None):
        super().__init__(geometry)
        object.__setattr__(self, '_visualization', visualization)
        object.__setattr__(self, '_book_cache', None)

    def _sam3_raw_books(self, processor, to_numpy, image, prompt='book'):
        if prompt != 'book spine':
            if prompt == 'book' and self._book_cache is not None:
                return self._book_cache
            return self._geometry._sam3_raw_books(processor, to_numpy, image, prompt=prompt)
        object.__setattr__(self, '_book_cache', None)
        rgb = np.asarray(image)
        groups = {kind: [] for kind in ('spine', 'book', 'cover', 'pages')}
        report = dict(version='tiral', accepted_count=0, candidates=[], errors=[],
                      note='Semantic scores are not calibrated probabilities. Final selection still requires geometry and depth checks.')
        try:
            config = load_config()
            report['thresholds'] = config
            spines = super()._sam3_raw_books(processor, to_numpy, image, prompt='book spine')
            groups['spine'] = normalize_detections(spines, 'spine', rgb.shape[:2])
            for kind, text in (('book', 'book'), ('cover', 'book cover'), ('pages', 'book page edges')):
                detections = self._geometry._sam3_raw_books(processor, to_numpy, image, prompt=text)
                groups[kind] = normalize_detections(detections, kind, rgb.shape[:2])
                if kind == 'book':
                    object.__setattr__(self, '_book_cache', detections)
            accepted, report['candidates'] = screen_spines(groups, config)
            report['accepted_count'] = len(accepted)
        except Exception as error:
            report['errors'].append(str(error))
            accepted = []
        report['detection_counts'] = {kind: len(records) for kind, records in groups.items()}
        print('tiral semantic report: ' + json.dumps(report, ensure_ascii=True), flush=True)
        if self._visualization is not None:
            self._visualization.capture_candidates(rgb, groups, report)
        return accepted


def main():
    def trial_book(**kwargs):
        return book.main(detector_factory=TrialDetector, **kwargs)
    print('tiral：同时检查书脊、封面与书页侧面；语义有歧义时停止本轮。', flush=True)
    runner.main(book_entry=trial_book,
        config_path=Path(__file__).with_name('cart_book_grasp_v1_tiral_config.json'),
        wrist_config_path=Path(__file__).with_name('cart_book_grasp_v1_tiral_wrist_config.json'))


if __name__ == '__main__':
    main()
