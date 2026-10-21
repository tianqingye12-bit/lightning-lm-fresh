#!/usr/bin/env python3
"""Create a separate instance-segmentation and static multi-image association profile."""
import argparse
from pathlib import Path
import yaml
from restore_map_cleanup import recovery_profile
from vehicle_instance_tracks import DEFAULTS


def profile(cfg):
    result=recovery_profile(cfg)
    result.update(vehicle_perception='segment',legacy_map_cleanup=False)
    result['vehicle_instance_tracks']=dict(DEFAULTS,enabled=True)
    result['seed_local_cleanup']=dict(enabled=False)
    result['segmentation_quality']=dict(enabled=True,min_instance_confidence=max(.35,float(cfg['yolo_conf'])),
                                        min_mean_confidence=max(.45,float(cfg['yolo_conf'])),core_erode_px=1)
    result['instance_filter']=dict(result.get('instance_filter',{}),min_depth_pixels=6)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    with Path(args.output).open('x') as stream:
        yaml.safe_dump(profile(yaml.safe_load(Path(args.input).read_text())),stream,sort_keys=False,allow_unicode=True)
    print(f'Created {args.output}; separate scored masks and static multi-image instance tracks')
