#!/usr/bin/env python3
"""Inspect a saved map or create a NEW purpose-specific compact copy; never delete source files."""
import argparse
import json
from pathlib import Path
import shutil
import time

import numpy as np
import yaml

from map_storage import compress_pcd


def inventory(source):
    source = Path(source)
    if not source.is_dir():
        raise FileNotFoundError(f'Map directory does not exist: {source}')
    groups = {}
    for path in Path(source).rglob('*'):
        if not path.is_file() or path.is_symlink():
            continue
        relative = path.relative_to(source)
        if relative.parts[0] == 'keyframe_clouds':
            key = 'keyframes'
        elif relative.parts[0] == 'clean_diagnostics':
            key = 'clean_runs'
        elif relative.parts[0].startswith('.vehicle_mask_cache'):
            key = 'segmentation_cache'
        elif path.name.startswith('global') and path.suffix == '.pcd':
            key = 'global_point_clouds'
        elif path.suffix == '.pcd':
            key = 'tiles_or_other_clouds'
        else:
            key = 'metadata_images_other'
        groups[key] = groups.get(key,0) + path.stat().st_size
    return groups


def plan(source, profile):
    """Build an explicit allowlist. Neither cache nor diagnostic history is copied."""
    if (source/'INCOMPLETE').exists():
        raise ValueError('Source map is incomplete')
    if profile == 'mapping':
        from clean_map_runtime import load_frame_contract
        from clean_map_keyframes import load_manifest
        from clean_map_vehicles import Trajectory
        if load_frame_contract(source)[1] != 'imu_v2':
            raise ValueError('Mapping copy needs IMU v2 keyframe clouds; an old global-only map cannot lose its global PCD')
        trajectory = Trajectory(source/'keyframes.txt', np.finfo(float).max)
        entries = load_manifest(source, trajectory)
        paths = [Path('keyframes.txt'), Path('map_frames.yaml'), Path('keyframe_clouds/index.csv')]
        paths += [path.relative_to(source) for _,_,path in entries]
        if (source/'summary.json').is_file():
            paths.append(Path('summary.json'))
    elif profile == 'navigation':
        if not (source/'index.txt').is_file() or not (source/'map.yaml').is_file():
            raise ValueError('Navigation copy requires index.txt, tile PCDs and map.yaml')
        paths = [Path('index.txt'),Path('map.yaml')]
        rows = (source/'index.txt').read_text().splitlines()
        if not rows or len(rows[0].split()) != 3 or not np.isfinite([float(x) for x in rows[0].split()]).all():
            raise ValueError('Invalid tile origin')
        ids = set()
        for row in rows[1:]:
            if row.strip() == '# functional points':
                break
            if not row.strip():
                continue
            values=row.split()
            if len(values) < 4:
                raise ValueError('Invalid tile index row')
            identifier=int(values[0]); int(values[1]); int(values[2])
            if identifier < 0 or identifier in ids:
                raise ValueError('Duplicate/negative tile ID')
            ids.add(identifier)
            paths.append(Path(f'{identifier}.pcd'))
            if (source/f'{identifier}_dyn.pcd').is_file():
                paths.append(Path(f'{identifier}_dyn.pcd'))
        if not ids:
            raise ValueError('Tile index contains no static tiles')
        grid = yaml.safe_load((source/'map.yaml').read_text())
        image = Path(grid['image'])
        if image.is_absolute() or '..' in image.parts or not np.isfinite(grid['resolution']) or grid['resolution'] <= 0:
            raise ValueError('Map image must be inside the map directory, with positive resolution')
        paths.append(image)
        paths += [Path(name) for name in ('localization.yaml','dynamic_polygon.txt','map_frames.yaml',
                  'keyframes.txt','BUILD_COMPLETE.json') if (source/name).is_file()]
    else:
        raise ValueError('profile must be mapping or navigation')
    for relative in paths:
        file=source/relative
        if not file.is_file() or file.is_symlink() or source not in file.resolve().parents:
            raise ValueError(f'Missing or unsafe map file: {relative}')
    return sorted(set(paths))


def compact(source, output, profile, compress=True):
    source, output=Path(source).resolve(),Path(output).resolve()
    if source == output or source in output.parents or output in source.parents:
        raise ValueError('Choose a separate sibling output directory')
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    paths=plan(source,profile)
    before=inventory(source)
    output.mkdir(parents=True,exist_ok=False)
    incomplete=output/'INCOMPLETE'; incomplete.write_text('Compact copy not finished.\n')
    started=time.monotonic()
    compressed, skipped = 0, 0
    for relative in paths:
        target=output/relative; target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source/relative,target)
        if compress and target.suffix == '.pcd':
            result=compress_pcd(target)
            compressed += result['status'] == 'compressed'
            skipped += result['status'] == 'over_budget'
    if profile == 'mapping' and (output/'summary.json').exists():
        summary=json.loads((output/'summary.json').read_text())
        summary.update(global_clean_written=False,removed_cloud_written=False,
                       compact_copy_of=str(source))
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if (output/'localization.yaml').exists():
        config=yaml.safe_load((output/'localization.yaml').read_text())
        config.setdefault('system',{})['map_path']=str(output)+'/'
        (output/'localization.yaml').write_text(yaml.safe_dump(config,sort_keys=False,allow_unicode=True))
    stats=dict(profile=profile,source=str(source),source_bytes=sum(before.values()),
               copied_payload_bytes=sum((output/path).stat().st_size for path in paths),
               source_categories=before, copied_files=len(paths),compressed_files=compressed,
               compression_over_budget_files=skipped,elapsed_seconds=time.monotonic()-started,
               source_unchanged=True)
    (output/'STORAGE_COMPLETE.json').write_text(json.dumps(stats,indent=2)+'\n')
    incomplete.unlink()
    print(json.dumps(stats,indent=2),flush=True)
    print(f'Compact copy: {output}. Original directory is unchanged.',flush=True)
    return stats


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--inspect',action='store_true',help='Read-only category sizes; no output required')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--profile',choices=['mapping','navigation'],default='navigation')
    parser.add_argument('--no-compress',action='store_true',help='Only omit redundant copies; no PCL tool needed')
    args=parser.parse_args()
    if args.inspect:
        sizes=inventory(args.source)
        print(json.dumps(dict(total_bytes=sum(sizes.values()),categories=sizes),indent=2))
        return
    if args.output is None:
        parser.error('--output is required unless using --inspect')
    compact(args.source,args.output,args.profile,compress=not args.no_compress)


if __name__ == '__main__':
    main()
