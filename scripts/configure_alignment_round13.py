#!/usr/bin/env python3
"""Enable scan recording/dense association without changing sensor calibration."""
import argparse
import copy
from pathlib import Path
import yaml


def configure(cfg, mode, max_records=200000, max_gap_s=.2):
    if not isinstance(cfg, dict): raise ValueError('Input must be a YAML mapping')
    cfg = copy.deepcopy(cfg)
    if mode == 'mapping':
        if not isinstance(cfg.get('fasterlio'), dict): raise ValueError('Missing fasterlio mapping')
        if isinstance(max_records, bool) or not isinstance(max_records, int) or not 100 <= max_records <= 2000000:
            raise ValueError('max_records must be an integer in [100, 2000000]')
        cfg['fasterlio']['trajectory_recording'] = dict(enabled=True,max_records=max_records)
    elif mode == 'semantic':
        from pose_trajectory import trajectory_options
        cfg['pose_trajectory'] = dict(source='dense',max_gap_s=max_gap_s)
        trajectory_options(cfg)
    else: raise ValueError('Mode must be mapping or semantic')
    return cfg


def write_profile(source, destination, mode, **kwargs):
    cfg = configure(yaml.safe_load(Path(source).read_text()),mode,**kwargs)
    with Path(destination).open('x') as stream:
        yaml.safe_dump(cfg,stream,allow_unicode=True,sort_keys=False)
    return cfg


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['mapping','semantic'])
    parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--max-records',type=int,default=200000)
    parser.add_argument('--max-gap-s',type=float,default=.2)
    args=parser.parse_args()
    try: write_profile(args.input,args.output,args.mode,max_records=args.max_records,max_gap_s=args.max_gap_s)
    except (ValueError,TypeError,KeyError,OSError,yaml.YAMLError) as error: parser.exit(1,f'{error}\n')
    print(f'Created {args.output}')


if __name__=='__main__':main()
