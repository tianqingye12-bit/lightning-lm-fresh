#!/usr/bin/env python3
"""Create a separate localization config, preserving sensor calibration and IMU prediction limits."""
import argparse
from pathlib import Path
import yaml


def configure(base: Path, map_path: Path, output: Path, with_ui: bool = False):
    config = yaml.safe_load(base.read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise ValueError('base must be a YAML mapping')
    for section in ('common', 'fasterlio', 'roi', 'lidar_loc', 'maps', 'system'):
        if not isinstance(config.get(section), dict):
            raise ValueError(f'base config lacks {section}; use a complete project configuration')
    map_path = map_path.resolve(strict=True)
    if not (map_path / 'index.txt').is_file():
        raise ValueError('localization requires tiled-map index.txt; a keyframe-only/compact mapping archive is insufficient')
    system = config['system']
    system['map_path'] = str(map_path)
    system['with_ui'] = with_ui
    defaults = dict(imu_timeout_s=1.0, lidar_timeout_s=2.0, result_timeout_s=2.0,
                    max_result_lag_s=1.0, max_match_lag_s=3.0, recovery_matches=3,
                    match_interval_s=1.0, min_match_score=1.0)
    health = system.setdefault('localization_health', {})
    if not isinstance(health, dict):
        raise ValueError('system.localization_health must be a YAML mapping')
    for key, value in defaults.items():
        health.setdefault(key, value)
    # A separate file, with exclusive creation: never overwrite the validated SLAM config.
    text = yaml.safe_dump(config, allow_unicode=True, sort_keys=False)
    with output.open('x', encoding='utf-8') as stream:
        stream.write(text)
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--map', dest='map_path', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('config/m2dgr_loc_health.yaml'))
    parser.add_argument('--with-ui', action='store_true')
    args = parser.parse_args()
    try:
        configure(args.base, args.map_path, args.output, args.with_ui)
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.exit(1, f'{error}\n')
    print(f'Created {args.output}; calibration and prediction horizon preserved.')


if __name__ == '__main__':
    main()
