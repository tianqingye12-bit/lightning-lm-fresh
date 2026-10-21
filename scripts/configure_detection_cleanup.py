#!/usr/bin/env python3
"""Generate a separate box-only map cleanup profile, preserving sensor calibration."""
import argparse
from pathlib import Path
import yaml
from restore_map_cleanup import recovery_profile
from seed_local_cleanup import DEFAULTS
from vehicle_detection import detection_options


def profile(cfg, model):
    result = recovery_profile(cfg)
    result.update(vehicle_perception='detect', legacy_map_cleanup=False, yolo_model=model)
    # Missing boxes are unknown, not background. Only 3D-associated positives vote.
    result['instance_filter'] = dict(result.get('instance_filter', {}), min_vehicle_views=3)
    result['seed_local_cleanup'] = dict(DEFAULTS, enabled=True, visible_negative_veto=False)
    result['detection_filter'] = detection_options({})
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--model', required=True, help='Detection weights, e.g. yolo26n.pt (not -seg)')
    args = parser.parse_args()
    result = profile(yaml.safe_load(Path(args.input).read_text()), args.model)
    with Path(args.output).open('x') as stream:
        yaml.safe_dump(result, stream, sort_keys=False, allow_unicode=True)
    print(f'Created {args.output}; detection boxes + depth/3D association, no segmentation')
