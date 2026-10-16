"""Single-frame candidate acceptance, independent of servo geometry checks.

Candidate labels are identifiers only: never use equality with the source label
to select the winner. This offline rule does not authorize robot motion.
"""
import copy
import math
from collections import defaultdict


def candidate_checks(candidate, policy):
    reasons = []
    for field, key, minimum in [
        ('inliers', 'min_inliers', True),
        ('inlier_ratio', 'min_inlier_ratio', True),
        ('rms_px', 'max_rms_px', False),
        ('source_hull_mask_fraction', 'min_source_hull_mask_fraction', True),
    ]:
        value = candidate.get(field)
        if (not isinstance(value, (int, float)) or not math.isfinite(value)
                or (value < policy[key] if minimum else value > policy[key])):
            reasons.append(field + '_outside_limit')
    return reasons


def annotate_report(original, policy):
    report = copy.deepcopy(original)
    report['acceptance_policy'] = dict(policy)
    groups = defaultdict(list)
    key = lambda r: (r['pair'], r['book'], r['resolution'], r['method'])
    for candidate in report['candidates']:
        groups[key(candidate)].append(candidate)
    for row in report['rows']:
        candidates = groups[key(row)]
        for candidate in candidates:
            reasons = candidate_checks(candidate, policy)
            others = [c['inliers'] for c in candidates if c is not candidate]
            margin = candidate['inliers'] - max(others, default=0)
            if not others:
                reasons.append('missing_competing_candidates')
            elif margin < policy['min_inlier_margin']:
                reasons.append('ambiguous_competing_candidate')
            candidate.update(match_success=not reasons, match_reasons=reasons,
                             inlier_margin=margin)
        winners = [c for c in candidates if c['match_success']]
        row['match_success'] = len(winners) == 1
        row['matched_candidate'] = winners[0]['wrist_candidate'] if len(winners) == 1 else None
        # Preserve original screen_pass and identity_texture_screen verbatim.
        row['match_scope'] = 'single_frame_candidate_identity_only'
    return report
