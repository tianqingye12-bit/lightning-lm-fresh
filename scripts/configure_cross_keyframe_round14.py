#!/usr/bin/env python3
"""Enable shared world-voxel semantics while preserving calibration and trajectory selection."""
import argparse
import copy
from pathlib import Path
import yaml
from cross_keyframe_semantics import cross_frame_options


def configure(cfg,voxel_size=.1):
    if not isinstance(cfg,dict):raise ValueError('Input must be a YAML mapping')
    result=copy.deepcopy(cfg)
    result['clean_mode']='keyframes'
    result['cross_keyframe_semantics']=dict(enabled=True,voxel_size=voxel_size)
    cross_frame_options(result)
    return result


def write_profile(source,output,voxel_size=.1):
    result=configure(yaml.safe_load(Path(source).read_text()),voxel_size)
    with Path(output).open('x') as stream:yaml.safe_dump(result,stream,sort_keys=False,allow_unicode=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--voxel-size',type=float,default=.1)
    args=parser.parse_args()
    try:write_profile(args.input,args.output,args.voxel_size)
    except (ValueError,TypeError,OSError,yaml.YAMLError) as error:parser.exit(1,f'{error}\n')
    print(f'Created {args.output}; calibration, thresholds, time offset and trajectory source preserved')
