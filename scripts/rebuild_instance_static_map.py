#!/usr/bin/env python3
"""Rebuild a compact static map from original keyframes using confirmed instance labels."""
import argparse
import json
from pathlib import Path
import shutil
import numpy as np
from clean_map_output import create_output_directory
from clean_map_runtime import load_frame_contract
from instance_map_labels import LabelLookup, digest


def rebuild(source,labels,output,resolution=.1):
    from clean_map_vehicles import Trajectory,read_pcd
    from clean_map_keyframes import load_manifest
    from voxel_pcd_stream import VoxelPcdStream
    source=Path(source).resolve()
    if (source/'INCOMPLETE').exists():raise ValueError('Original map is incomplete')
    if not np.isfinite(resolution) or not .02<=resolution<=.2:raise ValueError('Export resolution must be in [0.02,0.2]')
    _,contract=load_frame_contract(source)
    if contract!='imu_v2':raise ValueError('Instance rebuilding requires imu_v2 coordinates')
    trajectory=Trajectory(source/'keyframes.txt',2.)
    entries=load_manifest(source,trajectory)
    lookup=LabelLookup(labels,source);writer=None
    try:
        out=create_output_directory(source,output)
        stats=dict(frames=len(entries),input_points=0,removed_points=0,invalid_points=0,
                   result_layout='global_only',label_resolution_m=lookup.resolution,
                   export_resolution_m=resolution,labels_sha256=digest(labels),source_map=str(source))
        for index,(_,timestamp,path) in enumerate(entries):
            header,cloud=read_pcd(path)
            if writer is None:writer=VoxelPcdStream(out/'global_clean.pcd',header,resolution)
            xyz=np.column_stack([cloud[k] for k in ('x','y','z')]).astype(float)
            finite=np.isfinite(xyz).all(axis=1);block=cloud[finite].copy()
            pose=trajectory.pose(timestamp);world=xyz[finite]@pose[:3,:3].T+pose[:3,3]
            remove=lookup.lookup(world)
            for key,values in zip(('x','y','z'),world.T):block[key]=values
            writer.append(block[~remove])
            stats['input_points']+=len(cloud);stats['removed_points']+=int(remove.sum())
            stats['invalid_points']+=int((~finite).sum())
            if index==0 or (index+1)%20==0:print(f'Static rebuild: {index+1}/{len(entries)} frames; removed={stats["removed_points"]}',flush=True)
        if writer is None:raise ValueError('No original keyframes')
        writer.finish();stats['exported_points']=writer.count
        for name in ('keyframes.txt','map_frames.yaml'):shutil.copy2(source/name,out/name)
        (out/'summary.json').write_text(json.dumps(stats,indent=2)+'\n')
        (out/'INCOMPLETE').unlink()
        print(f'Full static map: {out/"global_clean.pcd"}',flush=True)
        return out
    finally:
        if writer is not None:writer.close()
        lookup.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True,help='ORIGINAL saved keyframe map, not the prepared map-only directory')
    parser.add_argument('--labels',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--voxel-size',type=float,default=.1)
    args=parser.parse_args()
    rebuild(args.map,args.labels,args.output,args.voxel_size)
