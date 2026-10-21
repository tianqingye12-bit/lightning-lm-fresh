"""Conservative semantic gates; baseline is camera translation, not object motion."""
import numpy as np


def evidence_options(cfg):
    opts = dict(enabled=False, min_classified_ratio=0.6, min_positive_baseline_m=0.05,
                write_uncertain=False)
    supplied = cfg.get('semantic_evidence', {})
    if not isinstance(supplied, dict) or set(supplied) - set(opts):
        raise ValueError('Unknown semantic_evidence option')
    opts.update(supplied)
    for name in ('enabled', 'write_uncertain'):
        if not isinstance(opts[name], bool):
            raise ValueError(f'semantic_evidence.{name} must be boolean')
    for name in ('min_classified_ratio', 'min_positive_baseline_m'):
        value = opts[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
            raise ValueError(f'semantic_evidence.{name} must be finite numeric')
    if not 0 < opts['min_classified_ratio'] <= 1 or opts['min_positive_baseline_m'] < 0:
        raise ValueError('Invalid semantic evidence coverage/baseline')
    return opts


class PositiveBaseline:
    def __init__(self, count, cfg):
        opts = evidence_options(cfg)
        self.enabled = opts['enabled'] and opts['min_positive_baseline_m'] > 0
        # O(points), 28 extra bytes/point when enabled; no image-mask history.
        self.first = np.full((count, 3), np.nan, np.float64) if self.enabled else None
        self.maximum = np.zeros(count, np.float32) if self.enabled else None

    def update(self, indices, camera_center):
        if not self.enabled or len(indices) == 0:
            return
        center = np.asarray(camera_center, dtype=np.float64)
        if center.shape != (3,) or not np.isfinite(center).all():
            raise ValueError('Camera center must contain three finite coordinates')
        first = self.first[indices]
        new = ~np.isfinite(first[:, 0])
        first[new] = center
        self.first[indices] = first
        distance = np.linalg.norm(first.astype(float) - center, axis=1)
        self.maximum[indices] = np.maximum(self.maximum[indices], distance)


def decide(visible, positive, background, cfg, baseline, protected=None, positive_score=None):
    from clean_map_instances import association_options, removal_votes
    from segmentation_quality import quality_options
    options = evidence_options(cfg)
    quality = quality_options(cfg)
    observed, candidate = removal_votes(visible, positive, background, cfg)
    coverage = (positive + background) / np.maximum(visible, 1)
    coverage_ok = coverage >= options['min_classified_ratio']
    baseline_ok = np.ones(len(visible), bool)
    if options['enabled'] and options['min_positive_baseline_m'] > 0:
        if baseline.maximum is None:
            raise ValueError('Positive baseline tracking was not enabled')
        baseline_ok = baseline.maximum >= options['min_positive_baseline_m']
    remove = candidate.copy()
    if options['enabled']:
        remove &= coverage_ok & baseline_ok
    confidence_ok = np.ones(len(visible), bool)
    if quality['enabled']:
        score = np.asarray(positive_score, dtype=float)
        if score.shape != positive.shape or not np.isfinite(score).all() or np.any(score < 0) or np.any(score > positive + 1e-4*np.maximum(positive, 1)):
            raise ValueError('Confidence sums must be finite, nonnegative, and <= positive view counts')
        mean_confidence = score / np.maximum(positive, 1)
        weighted_ratio = score / np.maximum(score + background, 1e-12)
        confidence_ok = ((mean_confidence >= quality['min_mean_confidence']) &
                         (weighted_ratio >= cfg['vehicle_ratio']))
        remove &= confidence_ok  # Minimum view count and the raw background veto still apply.
    protected = np.zeros(len(visible), bool) if protected is None else np.asarray(protected, bool)
    remove &= ~protected
    assoc = association_options(cfg)
    confirmed_background = ((background >= assoc['min_background_views']) &
                            (background / np.maximum(positive + background, 1) >= assoc['background_ratio']))
    unknown = ~(remove | protected | confirmed_background)
    counts = dict(total=int(len(visible)), removed=int(remove.sum()),
                  protected=int(protected.sum()),
                  background=int((confirmed_background & ~protected & ~remove).sum()),
                  uncertain=int(unknown.sum()), unobserved=int((visible == 0).sum()),
                  blocked_coverage=int((candidate & ~coverage_ok).sum()) if options['enabled'] else 0,
                  blocked_baseline=int((candidate & ~baseline_ok).sum()) if options['enabled'] else 0,
                  blocked_confidence=int((candidate & ~confidence_ok).sum()) if quality['enabled'] else 0)
    return observed, remove, unknown, counts


def timing_stats(traj, timestamp, cfg):
    aligned = float(timestamp + cfg['time_offset'])
    i = int(np.searchsorted(traj.t, aligned))
    nearest = min((abs(float(traj.t[j]) - aligned) for j in (i-1, i) if 0 <= j < len(traj.t)),
                  default=None)
    exact = i < len(traj.t) and traj.t[i] == aligned
    gap = 0.0 if exact else (float(traj.t[i] - traj.t[i-1]) if 0 < i < len(traj.t) else None)
    result = dict(aligned_timestamp=aligned, nearest_keyframe_dt_s=nearest, pose_bracket_gap_s=gap)
    if hasattr(traj, 'association_info'):
        result['keyframe_bracket_gap_s'] = gap
        result.update(traj.association_info(aligned))
    return result


def report_semantics():
    return dict(uncertain_retained=True, classes_are_not_motion_labels=True,
                baseline_definition='max camera translation from first positive observation',
                confidence_is_not_calibrated_probability=True,
                point_counts_are_not_accuracy_metrics=True)
