#!/usr/bin/env python3
"""Enable region association only; preserve segmentation and deletion settings."""
import argparse
import copy
from pathlib import Path
import yaml
from vehicle_instance_tracks import options


def profile(cfg):
    result=copy.deepcopy(cfg)
    opts=options(cfg)
    if not opts['enabled'] or result.get('vehicle_perception','segment')!='segment' or result.get('clean_mode','map')!='map':
        raise ValueError('Input must be an enabled map/segment instance-tracking profile')
    result['vehicle_instance_tracks']=dict(result['vehicle_instance_tracks'],region_matching=True)
    options(result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    result=profile(yaml.safe_load(Path(args.input).read_text()))
    with Path(args.output).open('x') as stream:yaml.safe_dump(result,stream,sort_keys=False,allow_unicode=True)
    print(f'Created {args.output}; region matching and mutual one-to-one assignment enabled')
