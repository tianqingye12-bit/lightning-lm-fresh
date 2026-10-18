#!/usr/bin/env python3
"""Generate a conservative map-detail test YAML from an existing sensor configuration."""
import argparse
from pathlib import Path
import yaml


def configure(config):
    fasterlio = config.get('fasterlio')
    if not isinstance(fasterlio, dict):
        raise ValueError('input YAML must contain a fasterlio mapping')
    # Keep sensor calibration, topics, IMU settings, keyframe thresholds and map index.
    fasterlio['filter_size_scan'] = 0.3
    fasterlio['map_detail'] = {
        'input_point_filter_num': 1,
        'registration_min_ratio': 0.0,
        'keyframe_resolution': 0.1,
        'keyframe_max_points': 60000,
        'export_resolution': 0.05,
        'local_fill': {
            'enabled': True,
            'min_interval': 0.3,
            'translation': 0.25,
            'rotation_deg': 3.0,
            'min_effective': 50,
        },
    }
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        parser.error('choose a separate output path to preserve the baseline configuration')
    config = yaml.safe_load(args.input.read_text())
    if not isinstance(config, dict):
        parser.error('input YAML must be a mapping')
    try:
        configure(config)
    except ValueError as error:
        parser.error(str(error))
    # Exclusive creation also protects a previous test configuration.
    with args.output.open('x') as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    print(f'Created {args.output}; scan=0.3m, keyframe=0.1m, export=0.05m; local fill enabled')
    print('Input stride=1 increases deskew work; keyframes are capped at 60000 points. Export cannot recover missing data.')


if __name__ == '__main__':
    main()
