#!/usr/bin/env python3
"""Deduplicate an existing cleaned binary PCD into a new, small result; never rerun semantics."""
import argparse
import json
from pathlib import Path
import shutil
from clean_map_output import create_output_directory
from voxel_pcd_stream import VoxelPcdStream


def compact(source, output, resolution=.1):
    from build_static_navigation import binary_blocks
    source=Path(source).resolve()
    if (source/'INCOMPLETE').exists():raise ValueError('Input cleaning is incomplete')
    path=source/'global_clean.pcd'
    header=[]
    with path.open('rb') as stream:
        for _ in range(256):
            line=stream.readline().decode('ascii').strip()
            if not line:raise ValueError('Missing PCD DATA header')
            header.append(line)
            if line.startswith('DATA '):break
        else:raise ValueError('PCD header too long')
    if header[-1]!='DATA binary':raise ValueError('Expected the binary global_clean.pcd from keyframe cleaning')
    target=create_output_directory(source,output)
    writer=VoxelPcdStream(target/'global_clean.pcd',header,resolution)
    try:
        last_million=0
        for block in binary_blocks(path):
            writer.append(block)
            million=writer.input_count//1000000
            if million>last_million:
                print(f'Compacting: {writer.input_count} input points',flush=True)
                last_million=million
        writer.finish()
        report=json.loads((source/'summary.json').read_text()) if (source/'summary.json').exists() else {}
        report.update(result_layout='global_only',source_result=str(source),output_directory=str(target),
            semantic_processing_rerun=False,global_export=dict(voxel_size_m=resolution,
            input_points=writer.input_count,points=writer.count,policy='first_observed_point_per_voxel'),
            compact_bytes=(target/'global_clean.pcd').stat().st_size)
        for name in ('map_frames.yaml','keyframes.txt'):
            if (source/name).is_file():shutil.copy2(source/name,target/name)
        (target/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        (target/'INCOMPLETE').unlink()
        print(f'Compact map: {target / "global_clean.pcd"}; {writer.input_count} -> {writer.count} points',flush=True)
        print('Geometry compaction only; vehicle classification is unchanged.',flush=True)
        return target
    finally:writer.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--voxel-size',type=float,default=.1)
    args=parser.parse_args()
    compact(args.result,args.output,args.voxel_size)
