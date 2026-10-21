"""Bounded geometry completion from semantic seeds, never a new semantic detector."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


DEFAULTS = dict(enabled=False, link_m=0.25, seed_link_m=0.35,
                padding_m=0.60, max_seed_distance_m=0.60,
                min_seeds=6, max_candidates=20000, max_groups=2000,
                max_extent_m=[7.0, 7.0, 3.0], background_ratio=0.20)


def options(cfg):
    supplied = cfg.get('seed_local_cleanup', {})
    if not isinstance(supplied, dict):
        raise ValueError('seed_local_cleanup must be a mapping')
    if set(supplied) - set(DEFAULTS):
        raise ValueError('Unknown seed_local_cleanup option')
    result = dict(DEFAULTS, **supplied)
    if not isinstance(result['enabled'], bool):
        raise ValueError('seed_local_cleanup.enabled must be boolean')
    for key in ('link_m', 'seed_link_m', 'padding_m', 'max_seed_distance_m'):
        value = result[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not np.isfinite(value) or not 0 < value <= 1:
            raise ValueError(f'seed_local_cleanup.{key} must be in (0,1] metres')
    for key, upper in (('min_seeds', 1000), ('max_candidates', 100000), ('max_groups', 10000)):
        if isinstance(result[key], bool) or not isinstance(result[key], int) or not 1 <= result[key] <= upper:
            raise ValueError(f'Invalid seed_local_cleanup.{key}')
    extent = np.asarray(result['max_extent_m'], dtype=float)
    if extent.shape != (3,) or not np.isfinite(extent).all() or (extent <= 0).any() or (extent > 10).any():
        raise ValueError('max_extent_m must contain three positive dimensions <=10m')
    ratio = result['background_ratio']
    if isinstance(ratio, bool) or not isinstance(ratio, (float, int)) or not np.isfinite(ratio) or not 0 <= ratio < 1:
        raise ValueError('background_ratio must be in [0,1)')
    return result


def labels(xyz, distance):
    """Bounded 16-neighbor graph: avoids dense all-pairs edges on large surfaces."""
    if not len(xyz):
        return np.empty(0, np.int32)
    distances, neighbors = cKDTree(xyz).query(xyz, k=min(16, len(xyz)), distance_upper_bound=distance)
    distances = np.asarray(distances).reshape(len(xyz), -1)
    neighbors = np.asarray(neighbors).reshape(len(xyz), -1)
    rows = np.broadcast_to(np.arange(len(xyz))[:, None], neighbors.shape)
    valid = np.isfinite(distances) & (neighbors < len(xyz))
    graph = coo_matrix((np.ones(int(valid.sum()), np.int8), (rows[valid], neighbors[valid])),
                       shape=(len(xyz), len(xyz)))
    return connected_components(graph, directed=False)[1]


def complete(xyz, seeds, visible, positive, background, floor, cfg, spatial_index=None):
    opts = options(cfg)
    output = np.asarray(seeds, bool).copy()
    report = dict(enabled=opts['enabled'], seed_voxels=int(output.sum()), added_voxels=0,
                  groups=0, expanded_groups=0, skipped_small=0, skipped_extent=0,
                  skipped_candidates=0, skipped_budget=0, background_veto_voxels=0)
    if not opts['enabled'] or not output.any():
        return output, report
    # In legacy mode background[] is absent evidence (all zero). Repeated visible
    # non-vehicle observations provide a conservative veto as well.
    veto = (np.asarray(background) > 0) | ((visible >= cfg['min_views']) &
            (positive / np.maximum(visible, 1) <= opts['background_ratio']))
    report['background_veto_voxels'] = int(veto.sum())
    seed_ids = np.flatnonzero(output & ~floor & ~veto)
    if not len(seed_ids):
        return output, report
    seed_labels = labels(xyz[seed_ids], opts['seed_link_m'])
    order = np.argsort(seed_labels, kind='stable')
    groups = np.split(seed_ids[order], np.flatnonzero(np.diff(seed_labels[order])) + 1)
    groups.sort(key=len, reverse=True)
    report['groups'] = len(groups)
    tree = spatial_index if spatial_index is not None else cKDTree(xyz)
    extent_limit = np.asarray(opts['max_extent_m'])
    for group_index, ids in enumerate(groups):
        if group_index % 50 == 0:
            print(f'Local completion groups: {group_index}/{len(groups)}; expanded={report["expanded_groups"]}', flush=True)
        if group_index >= opts['max_groups']:
            report['skipped_budget'] = len(groups) - group_index
            break
        if len(ids) < opts['min_seeds']:
            report['skipped_small'] += 1
            continue
        seed_xyz = xyz[ids]
        low, high = seed_xyz.min(axis=0), seed_xyz.max(axis=0)
        if ((high - low) > extent_limit).any():
            report['skipped_extent'] += 1
            continue
        padding = np.minimum(opts['padding_m'], (extent_limit - (high - low)) / 2)
        low -= padding
        high += padding
        center = (low + high) / 2
        radius = np.linalg.norm(high - low) / 2 + 1e-9
        if tree.query_ball_point(center, radius, return_length=True) > opts['max_candidates']:
            report['skipped_candidates'] += 1
            continue
        candidate = np.asarray(tree.query_ball_point(center, radius), dtype=np.intp)
        points = xyz[candidate]
        keep = (~floor[candidate] & ~veto[candidate] &
                (points >= low).all(axis=1) & (points <= high).all(axis=1))
        candidate = candidate[keep]
        distance = cKDTree(seed_xyz).query(xyz[candidate])[0]
        candidate = candidate[distance <= opts['max_seed_distance_m'] + 1e-9]
        if not len(candidate):
            continue
        component = labels(xyz[candidate], opts['link_m'])
        # Every expanded local component must have its own seed support.
        is_seed = np.isin(candidate, ids)
        support = np.bincount(component, weights=is_seed)
        chosen = candidate[support[component] >= opts['min_seeds']]
        added = int((~output[chosen]).sum())
        output[chosen] = True
        report['expanded_groups'] += int(added > 0)
    report['added_voxels'] = int(output.sum() - np.count_nonzero(seeds))
    return output, report
