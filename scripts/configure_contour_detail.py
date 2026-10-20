#!/usr/bin/env python3
"""Generate an opt-in contour-preserving profile from validated sensor YAML."""
import argparse
import copy
import math
from pathlib import Path
import yaml


def configure(config, registration_resolution=.25, keyframe_max_points=60000, storage_profile='compact'):
    if not isinstance(config, dict):
        raise ValueError('input YAML must be a mapping')
    cfg = copy.deepcopy(config)
    for key in ('fasterlio', 'system'):
        if not isinstance(cfg.get(key), dict):
            raise ValueError(f'input must contain a {key} mapping')
    if not math.isfinite(registration_resolution) or not .1 <= registration_resolution <= .5:
        raise ValueError('registration resolution must be in [0.1, 0.5] metres')
    if isinstance(keyframe_max_points, bool) or not isinstance(keyframe_max_points, int) or not 1000 <= keyframe_max_points <= 1000000:
        raise ValueError('keyframe point cap must be an integer in [1000, 1000000]')
    if storage_profile not in ('compact', 'full'):
        raise ValueError('storage profile must be compact or full')
    lio = cfg['fasterlio']
    existing = float(lio['filter_size_scan'])
    if not math.isfinite(existing) or existing <= 0:
        raise ValueError('base filter_size_scan must be positive and finite')
    # Refine a coarse baseline; never coarsen a previously finer registration scan.
    lio['filter_size_scan'] = min(existing, registration_resolution)
    detail = lio.setdefault('map_detail', {})
    feature = lio.setdefault('feature_quality', {})
    export = cfg['system'].setdefault('map_export', {})
    if not all(isinstance(x, dict) for x in (detail, feature, export)):
        raise ValueError('map_detail, feature_quality and map_export must be mappings')
    detail.update(preserve_observed_points=True, input_point_filter_num=1, registration_min_ratio=0.,
                  keyframe_resolution=.05, keyframe_max_points=keyframe_max_points, export_resolution=.05)
    # Keep already validated local-fill and keyframe trigger thresholds.
    feature['enabled'] = True
    feature['robust_weight'] = True
    feature.setdefault('huber_delta', .1)
    feature['surface_support'] = dict(enabled=True, max_neighbor_distance=.6,
                                      max_plane_residual=.15, max_tangent_gap=.10)
    cfg['system']['export_keyframe_clouds'] = True
    export.update(compress_keyframes=True, localization_resolution=.05,
                  save_global_pcd=storage_profile == 'full', save_localization_tiles=storage_profile == 'full')
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--registration-resolution', type=float, default=.25)
    parser.add_argument('--keyframe-max-points', type=int, default=60000)
    parser.add_argument('--storage-profile', choices=['compact', 'full'], default='compact')
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        parser.error('choose a separate output file to preserve the baseline')
    try:
        cfg = configure(yaml.safe_load(args.input.read_text()), args.registration_resolution,
                        args.keyframe_max_points, args.storage_profile)
        with args.output.open('x') as stream:
            yaml.safe_dump(cfg, stream, allow_unicode=True, sort_keys=False)
    except (ValueError, TypeError, KeyError, OSError, yaml.YAMLError) as error:
        parser.exit(1, f'{error}\n')
    print(f'Created {args.output}: observed points, registration={cfg["fasterlio"]["filter_size_scan"]}m, '
          f'keyframe/export=0.05m, cap={args.keyframe_max_points}, storage={args.storage_profile}')
    print('Sensor calibration, IMU prediction limits, map index, keyframe triggers and grid settings preserved.')


if __name__ == '__main__':
    main()
