#!/usr/bin/env python3
"""Create an independent navigation-detail configuration from validated sensor YAML."""
import argparse
import copy
from pathlib import Path

import yaml


def configure(config, export_resolution=.05, keyframe_max_points=60000):
    cfg = copy.deepcopy(config)
    for key in ('fasterlio', 'g2p5', 'system'):
        if not isinstance(cfg.get(key), dict):
            raise ValueError(f'input must contain a {key} mapping')
    if export_resolution not in (.03, .05):
        raise ValueError('export resolution must be .03 or .05 metres')
    if not isinstance(keyframe_max_points, int) or not 1000 <= keyframe_max_points <= 1000000:
        raise ValueError('keyframe point cap must be an integer in [1000, 1000000]')
    if isinstance(keyframe_max_points, bool):
        raise ValueError('keyframe point cap must be an integer')
    fasterlio = cfg['fasterlio']
    fasterlio.setdefault('filter_size_scan', .3)  # Preserve the already validated registration setting.
    detail = fasterlio.setdefault('map_detail', {})
    if not isinstance(detail, dict):
        raise ValueError('fasterlio.map_detail must be a mapping')
    detail.update(input_point_filter_num=1, keyframe_resolution=.05,
                  keyframe_max_points=keyframe_max_points, export_resolution=export_resolution)
    detail.setdefault('registration_min_ratio', 0.)
    detail.setdefault('local_fill', dict(enabled=True, min_interval=.3, translation=.25,
                                        rotation_deg=3., min_effective=50))
    cfg['g2p5']['grid_map_resolution'] = .05
    cfg['g2p5'].setdefault('max_grid_cells', 8000000)
    cfg['system']['export_keyframe_clouds'] = True
    map_export = cfg['system'].setdefault('map_export', {})
    if not isinstance(map_export, dict):
        raise ValueError('system.map_export must be a mapping')
    map_export['localization_resolution'] = .05
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--export-resolution', type=float, choices=[.03, .05], default=.05)
    parser.add_argument('--keyframe-max-points', type=int, default=60000)
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        parser.error('use a different output to preserve your validated sensor configuration')
    try:
        cfg = configure(yaml.safe_load(args.input.read_text()), args.export_resolution, args.keyframe_max_points)
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    with args.output.open('x') as stream:
        yaml.safe_dump(cfg, stream, allow_unicode=True, sort_keys=False)
    print(f'Created {args.output}: keyframes=0.05m/{args.keyframe_max_points} points, '
          f'PCD={args.export_resolution}m, localization tiles=0.05m, navigation grid=0.05m')
    print('Sensor calibration, prediction limits, registration/map filters and keyframe triggers preserved.')


if __name__ == '__main__':
    main()
