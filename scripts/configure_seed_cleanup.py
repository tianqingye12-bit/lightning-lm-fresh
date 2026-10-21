#!/usr/bin/env python3
"""Generate an opt-in map cleanup comparison profile without changing calibration."""
import argparse
from pathlib import Path
import yaml
from restore_map_cleanup import recovery_profile
from seed_local_cleanup import DEFAULTS, options


def profile(cfg):
    result = recovery_profile(cfg)
    result['seed_local_cleanup'] = dict(DEFAULTS, enabled=True)
    options(result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    cfg = profile(yaml.safe_load(Path(args.input).read_text()))
    with Path(args.output).open('x') as stream:
        yaml.safe_dump(cfg, stream, sort_keys=False, allow_unicode=True)
    print(f'Created {args.output}; bounded seed completion replaces whole-component growth')
