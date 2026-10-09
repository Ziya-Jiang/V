"""Offline competing-mask tests; no SAM3 weights or hardware are loaded."""
import types
import unittest
from unittest.mock import Mock, patch
import numpy as np

import cart_book_grasp_v1_tiral as trial


class SemanticTrialTests(unittest.TestCase):
    def setUp(self):
        self.mask = np.zeros((160, 180), bool)
        self.mask[30:130, 80:91] = True
        self.whole = np.zeros_like(self.mask)
        self.whole[25:135, 75:135] = True
        self.cover = np.zeros_like(self.mask)
        self.cover[30:130, 92:130] = True
        self.config = trial.load_config()

    def groups(self, spine_score=.7, cover=None, pages=None, whole=None):
        detections = dict(spine=[(spine_score, self.mask, None)],
                          book=[(.8, self.whole if whole is None else whole, None)],
                          cover=[(.8, self.cover if cover is None else cover, None)],
                          pages=[] if pages is None else [(.8, pages, None)])
        return {kind: trial.normalize_detections(data, kind, self.mask.shape) for kind, data in detections.items()}

    def test_distinct_spine_passes_without_cover_overlap(self):
        accepted, rows = trial.screen_spines(self.groups(), self.config)
        self.assertEqual(len(accepted), 1)
        self.assertEqual(rows[0]['reasons'], [])
        self.assertEqual(rows[0]['book_coverage'], 1.)

    def test_page_edges_conflict_is_rejected_even_with_high_spine_score(self):
        accepted, rows = trial.screen_spines(self.groups(spine_score=.99, pages=self.mask), self.config)
        self.assertFalse(accepted)
        self.assertTrue(any('ambiguous with pages' in reason for reason in rows[0]['reasons']))

    def test_cover_conflict_is_rejected(self):
        accepted, rows = trial.screen_spines(self.groups(cover=self.mask), self.config)
        self.assertFalse(accepted)
        self.assertTrue(any('ambiguous with cover' in reason for reason in rows[0]['reasons']))

    def test_low_score_and_unrelated_book_are_rejected(self):
        for groups in (self.groups(spine_score=.18), self.groups(whole=np.roll(self.whole, 70, axis=1))):
            accepted, rows = trial.screen_spines(groups, self.config)
            self.assertFalse(accepted)
            self.assertTrue(rows[0]['reasons'])

    def test_empty_competitors_are_reported_as_empty_not_fabricated(self):
        groups = self.groups()
        groups['cover'] = []
        accepted, rows = trial.screen_spines(groups, self.config)
        self.assertEqual(len(accepted), 1)
        self.assertEqual(rows[0]['competitors'], [])

    def detector(self, fail_prompt=None):
        groups = self.groups()
        prompts = {'book spine': 'spine', 'book': 'book', 'book cover': 'cover', 'book page edges': 'pages'}
        def raw(*args, prompt):
            if prompt == fail_prompt:
                raise RuntimeError('detector failure')
            return [(r['score'], r['mask'], None) for r in groups[prompts[prompt]]]
        geom = types.SimpleNamespace(_sam3_raw_books=Mock(side_effect=raw))
        visualization = types.SimpleNamespace(capture_candidates=Mock())
        return trial.TrialDetector(geom, visualization), geom, visualization

    def test_all_prompts_run_and_book_result_is_reused(self):
        detector, geom, visualization = self.detector()
        rgb = np.zeros((160, 180, 3), np.uint8)
        result = detector._sam3_raw_books(None, None, rgb, prompt='book spine')
        self.assertEqual(len(result), 1)
        self.assertEqual([call.kwargs['prompt'] for call in geom._sam3_raw_books.call_args_list],
                         ['book spine', 'book', 'book cover', 'book page edges'])
        detector._sam3_raw_books(None, None, rgb, prompt='book')
        self.assertEqual(geom._sam3_raw_books.call_count, 4)
        self.assertEqual(visualization.capture_candidates.call_args.args[2]['accepted_count'], 1)

    def test_competitor_failure_rejects_instead_of_reusing_last_detection(self):
        detector, geom, visualization = self.detector(fail_prompt='book page edges')
        result = detector._sam3_raw_books(None, None, np.zeros((160, 180, 3), np.uint8), prompt='book spine')
        self.assertFalse(result)
        self.assertTrue(visualization.capture_candidates.call_args.args[2]['errors'])

    def test_entrypoint_isolated_from_existing_configs(self):
        with patch.object(trial.runner, 'main') as run:
            trial.main()
        kwargs = run.call_args.kwargs
        self.assertEqual(kwargs['config_path'].name, 'cart_book_grasp_v1_tiral_config.json')
        self.assertEqual(kwargs['wrist_config_path'].name, 'cart_book_grasp_v1_tiral_wrist_config.json')
        with patch.object(trial.book, 'main') as main:
            kwargs['book_entry'](task_types=['cart+'])
            main.assert_called_once_with(detector_factory=trial.TrialDetector, task_types=['cart+'])


if __name__ == '__main__':
    unittest.main()
