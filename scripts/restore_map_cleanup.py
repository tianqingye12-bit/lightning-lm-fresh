#!/usr/bin/env python3
"""Restore the old configuration's map voting/cluster path in an independent profile and directory."""
import argparse
import copy
import json
from pathlib import Path
import shutil
import numpy as np
import yaml
from clean_map_output import create_output_directory


def recovery_profile(cfg):
    if not isinstance(cfg,dict):raise ValueError('Expected a YAML mapping')
    result=copy.deepcopy(cfg)
    result['clean_mode']='map'
    result['legacy_map_cleanup']=True
    for name in ('cross_keyframe_semantics','semantic_evidence','segmentation_quality'):
        result[name]=dict(result.get(name,{}) or {},enabled=False)
    result['pose_trajectory']=dict(result.get('pose_trajectory',{}) or {},source='keyframes')
    return result


def write_profile(source,output):
    result=recovery_profile(yaml.safe_load(Path(source).read_text()))
    with Path(output).open('x') as stream:yaml.safe_dump(result,stream,allow_unicode=True,sort_keys=False)
    print(f'Recovery profile: {Path(output).resolve()}; map voting and legacy bounded cluster growth',flush=True)
    return result


def prepare(source,output,resolution=.1):
    from clean_map_runtime import load_frame_contract
    from clean_map_keyframes import load_manifest
    from clean_map_vehicles import Trajectory,read_pcd
    from voxel_pcd_stream import VoxelPcdStream
    source=Path(source).expanduser().resolve()
    if (source/'INCOMPLETE').exists():raise ValueError('Source map is incomplete')
    if not np.isfinite(resolution) or not .02<=resolution<=.2:raise ValueError('Resolution must be in [0.02,0.2] metres')
    _,contract=load_frame_contract(source)
    trajectory=Trajectory(source/'keyframes.txt',2.)
    original=source/'global.pcd'
    entries=None
    if not original.is_file():
        if contract!='imu_v2':raise ValueError('Missing global.pcd; frame rebuilding requires IMU v2 coordinates')
        entries=load_manifest(source,trajectory)
    output=create_output_directory(source,output)
    mode='copied_original_global' if entries is None else 'rebuilt_original_frames'
    if entries is None:
        # copy2 streams the payload; this helper never loads the whole original map.
        shutil.copy2(original,output/'global.pcd')
    else:
        writer=None
        try:
            for i,(_,timestamp,path) in enumerate(entries):
                header,cloud=read_pcd(path)
                if writer is None:writer=VoxelPcdStream(output/'global.pcd',header,resolution)
                xyz=np.column_stack([cloud[k] for k in ('x','y','z')]).astype(float)
                finite=np.isfinite(xyz).all(axis=1)
                pose=trajectory.pose(timestamp)
                world=xyz[finite]@pose[:3,:3].T+pose[:3,3]
                block=cloud[finite].copy()
                for k,values in zip(('x','y','z'),world.T):block[k]=values
                writer.append(block)
                if i==0 or (i+1)%20==0:print(f'Preparing original global map: {i+1}/{len(entries)} frames',flush=True)
            if writer is None:raise ValueError('No source frames')
            writer.finish()
        finally:
            if writer is not None:writer.close()
    for name in ('keyframes.txt','map_frames.yaml'):
        if (source/name).exists():shutil.copy2(source/name,output/name)
    report=dict(source_map=str(source),output_map=str(output),preparation=mode,
        reconstruction_resolution_m=resolution if entries is not None else None,
        source_unchanged=True,semantic_cleaning_performed=False)
    (output/'PREPARED_MAP.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'INCOMPLETE').unlink()
    print(f'Map-mode input ready: {output}',flush=True)
    print('This is the ORIGINAL geometry; vehicle cleaning is the next command.',flush=True)
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    profile=sub.add_parser('config');profile.add_argument('--input',required=True);profile.add_argument('--output',required=True)
    preparation=sub.add_parser('prepare');preparation.add_argument('--map',required=True);preparation.add_argument('--output',required=True)
    preparation.add_argument('--voxel-size',type=float,default=.1)
    args=parser.parse_args()
    try:
        if args.command=='config':write_profile(args.input,args.output)
        else:prepare(args.map,args.output,args.voxel_size)
    except (ValueError,TypeError,OSError,yaml.YAMLError) as error:parser.exit(1,f'{error}\n')
