#!/usr/bin/env python3
"""Create independent mapping/semantic quality profiles from validated YAML."""
import argparse
import copy
from pathlib import Path

import yaml


def configure(cfg, mode, max_constraints=3000, diagnostics_only=False):
    if not isinstance(cfg, dict):
        raise ValueError('Input YAML must be a mapping')
    cfg = copy.deepcopy(cfg)
    if mode == 'mapping':
        if not isinstance(cfg.get('fasterlio'), dict):
            raise ValueError('Mapping input must contain fasterlio')
        if isinstance(max_constraints, bool) or not isinstance(max_constraints, int) or not 20 <= max_constraints <= 1000000:
            raise ValueError('max_constraints must be an integer in [20, 1000000]')
        cfg['fasterlio']['registration_quality'] = dict(balance=not diagnostics_only,
            max_constraints=max_constraints, diagnostics=True, rotation_scale_m=5.0, relative_eigenvalue=.001)
        feature = cfg['fasterlio'].setdefault('feature_quality', {})
        if not isinstance(feature, dict):
            raise ValueError('feature_quality must be a mapping')
        if feature.get('stats_interval', 0) == 0:
            feature['stats_interval'] = 20
    elif mode == 'semantic':
        from segmentation_quality import quality_options
        from semantic_evidence import evidence_options
        # Neither sensor calibration nor model size, inference resolution or storage is changed.
        evidence = evidence_options(cfg)
        evidence['enabled'] = True
        cfg['semantic_evidence'] = evidence
        quality = quality_options(cfg)
        quality.update(enabled=True, min_instance_confidence=max(quality['min_instance_confidence'], float(cfg['yolo_conf'])))
        cfg['segmentation_quality'] = quality
        quality_options(cfg)
    else:
        raise ValueError('Mode must be mapping or semantic')
    return cfg


def write_profile(source, destination, mode, **kwargs):
    cfg = configure(yaml.safe_load(Path(source).read_text()), mode, **kwargs)
    with Path(destination).open('x') as stream:
        yaml.safe_dump(cfg, stream, allow_unicode=True, sort_keys=False)
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['mapping', 'semantic'])
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-constraints', type=int, default=3000, help='Per-term correspondence cap (mapping only)')
    parser.add_argument('--diagnostics-only', action='store_true', help='Mapping diagnostics without balancing')
    args = parser.parse_args()
    try:
        write_profile(args.input, args.output, args.mode,
                      max_constraints=args.max_constraints, diagnostics_only=args.diagnostics_only)
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
        parser.exit(1, f'{error}\n')
    print(f'Created {args.output}; original profile and calibration preserved')


if __name__ == '__main__':
    main()
