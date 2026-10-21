"""Count disjoint rejection stages for voxels with at least one positive vote."""
import numpy as np
from clean_map_instances import association_options


def rejection_stages(visible, positive, background, floor, removed, cfg, legacy):
    remaining = positive > 0
    report = dict(positive_voxels=int(remaining.sum()))
    def reject(name, gate):
        report[name] = int((remaining & ~gate).sum())
        remaining[:] &= gate
    reject('below_min_views', visible >= cfg['min_views'])
    opts = association_options(cfg)
    reject('below_positive_views', np.ones(len(visible), bool) if legacy else positive >= opts['min_vehicle_views'])
    denominator = visible if legacy else positive + background
    reject('below_vote_ratio', positive / np.maximum(denominator, 1) >= cfg['vehicle_ratio'])
    veto = np.zeros(len(visible), bool) if legacy else ((background >= opts['min_background_views']) &
            (background / np.maximum(positive + background, 1) >= opts['background_ratio']))
    reject('background_veto', ~veto)
    reject('floor_protection', ~floor)
    reject('additional_gates', removed)
    report['seed_voxels'] = int(remaining.sum())
    return report
