#!/usr/bin/env python3
"""Build localization tiles AND a navigation grid from a round-6 cleaned-frame run.

Use a NEW output directory. The source is never modified. Disk-backed voxel aggregation
bounds point-buffer memory; the compiled grid renderer processes one cleaned frame at a time.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import sqlite3
import subprocess
import tempfile
import time

import numpy as np
import yaml

from clean_map_runtime import load_frame_contract
from clean_map_keyframes import load_manifest
from clean_map_vehicles import PCD_TYPES, Trajectory

TILE_SIZE = 100.0  # Must match TiledMap::Options and Pos2Grid in the existing loader.
OUTPUT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('time', '<f8')])


def pcd_header(count):
    return ('VERSION .7\nFIELDS x y z intensity time\nSIZE 4 4 4 4 8\nTYPE F F F F F\n'
            f'COUNT 1 1 1 1 1\nWIDTH {count}\nHEIGHT 1\nPOINTS {count}\nDATA binary\n').encode('ascii')


def binary_blocks(path, block_points=100000):
    """Read the uncompressed binary PCD produced by round 6 without loading its whole body."""
    with Path(path).open('rb') as stream:
        meta = {}
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError('PCD has no DATA header')
            parts = raw.decode('ascii').split()
            if parts and not parts[0].startswith('#'):
                meta[parts[0]] = parts[1:]
            if parts and parts[0] == 'DATA':
                break
        if meta['DATA'] != ['binary']:
            raise ValueError('Use global_clean.pcd from keyframe mode (uncompressed binary)')
        names = meta['FIELDS']
        sizes, kinds = meta['SIZE'], meta['TYPE']
        counts = meta.get('COUNT', ['1'] * len(names))
        if not len(names) == len(sizes) == len(kinds) == len(counts):
            raise ValueError('Inconsistent PCD field metadata')
        dtype = np.dtype([(name if name != '_' else f'_pad{i}', PCD_TYPES[(kind, int(size))],
                           (int(count),) if int(count) > 1 else ())
                          for i, (name, kind, size, count) in enumerate(zip(names, kinds, sizes, counts))])
        for axis in ('x', 'y', 'z'):
            if axis not in dtype.names or dtype[axis].shape:
                raise ValueError('PCD must have scalar XYZ fields')
        total = int(meta['POINTS'][0])
        offset = stream.tell()
        if total < 0 or Path(path).stat().st_size != offset + total * dtype.itemsize:
            raise ValueError('PCD body length does not match POINTS/fields')
        if total == 0:
            return
    points = np.memmap(path, mode='r', dtype=dtype, offset=offset, shape=(total,))
    for begin in range(0, total, block_points):
        yield points[begin:begin + block_points]


def aggregate_tiles(source, database, resolution):
    if not math.isfinite(resolution) or not .02 <= resolution <= .2:
        raise ValueError('localization resolution must be in [0.02, 0.2] metres')
    database.execute('PRAGMA cache_size=-16384')  # 16 MiB SQLite page cache; temp tables use disk.
    database.execute('PRAGMA temp_store=FILE')
    database.execute('''CREATE TABLE voxels(
        tx INTEGER, ty INTEGER, vx INTEGER, vy INTEGER, vz INTEGER,
        n INTEGER, sx REAL, sy REAL, sz REAL, si REAL, st REAL,
        PRIMARY KEY(tx,ty,vx,vy,vz)) WITHOUT ROWID''')
    query = '''INSERT INTO voxels VALUES(?,?,?,?,?,?,?,?,?,?,?)
        ON CONFLICT(tx,ty,vx,vy,vz) DO UPDATE SET
        n=n+excluded.n, sx=sx+excluded.sx, sy=sy+excluded.sy,
        sz=sz+excluded.sz, si=si+excluded.si, st=st+excluded.st'''
    total, invalid = 0, 0
    for block in binary_blocks(source):
        xyz = np.column_stack([block[axis] for axis in ('x', 'y', 'z')]).astype(float)
        finite = np.isfinite(xyz).all(axis=1)
        invalid += int((~finite).sum())
        block, xyz = block[finite], xyz[finite]
        if not len(xyz):
            continue
        grid = np.floor(xyz[:, :2]/TILE_SIZE + .5)
        voxel = np.floor(xyz/resolution)
        if np.any(np.abs(voxel) >= 2.0**63) or np.any(np.abs(grid) >= 2.0**31):
            raise ValueError('Map coordinates exceed loader/index range')
        keys, inverse = np.unique(np.column_stack([grid, voxel]).astype(np.int64), axis=0, return_inverse=True)
        counts = np.bincount(inverse)
        fields = [xyz[:, i] for i in range(3)]
        for name in ('intensity', 'time'):
            values = block[name].astype(float) if name in block.dtype.names else np.zeros(len(block))
            fields.append(np.where(np.isfinite(values), values, 0.))
        sums = [np.bincount(inverse, weights=value) for value in fields]
        def rows():
            for i, key in enumerate(keys):
                yield (*map(int, key), int(counts[i]), *(float(value[i]) for value in sums))
        database.executemany(query, rows())
        database.commit()
        total += len(xyz)
        if total % 1000000 < len(xyz):
            print(f'localization aggregation: {total} valid input points', flush=True)
    if total == 0:
        raise ValueError('Cleaned global map contains no finite points')
    return dict(input_points=total, invalid_points=invalid)


def export_tiles(database, output, start):
    count = database.execute('SELECT count(*) FROM voxels').fetchone()[0]
    tiles = list(database.execute('SELECT tx,ty,count(*) FROM voxels GROUP BY tx,ty ORDER BY tx,ty'))
    with (output / 'index.txt').open('x') as index, (output / 'global.pcd').open('xb') as full:
        index.write('0 0 0\n')
        full.write(pcd_header(count))
        for tile_id, (tx, ty, size) in enumerate(tiles):
            filename = f'{tile_id}.pcd'
            index.write(f'{tile_id} {tx} {ty} {filename}\n')
            query = database.execute('SELECT sx/n,sy/n,sz/n,si/n,st/n FROM voxels WHERE tx=? AND ty=?', (tx,ty))
            with (output / filename).open('xb') as tile:
                tile.write(pcd_header(size))
                while True:
                    rows = query.fetchmany(25000)
                    if not rows:
                        break
                    points = np.array([tuple(row) for row in rows], dtype=OUTPUT_DTYPE)
                    body = points.tobytes()
                    tile.write(body)
                    full.write(body)
        index.write('# functional points\nstart ' + ' '.join(f'{float(x):.17g}' for x in start) + '\n')
    return dict(output_points=count, tiles=len(tiles), tile_size=TILE_SIZE)


def validate_navigation_config(config):
    g2p5 = config.get('g2p5', {})
    keys = ('grid_map_resolution', 'min_th_floor', 'max_th_floor', 'floor_height', 'lidar_height')
    if not isinstance(g2p5, dict) or any(key not in g2p5 for key in keys):
        raise ValueError('Configuration must contain calibrated G2P5 resolution and height limits')
    if not np.isfinite([g2p5[key] for key in keys]).all():
        raise ValueError('G2P5 geometry must be finite')
    if g2p5['grid_map_resolution'] <= 0 or g2p5['min_th_floor'] >= g2p5['max_th_floor']:
        raise ValueError('Invalid G2P5 resolution/height interval')
    if not isinstance(config.get('system'), dict):
        raise ValueError('Configuration must contain system mapping')
    if not isinstance(config['system'].get('map_export', {}), dict):
        raise ValueError('system.map_export must be a mapping')
    if not isinstance(config.get('maps', {}), dict):
        raise ValueError('maps must be a mapping')
    limit = g2p5.get('max_grid_cells', 8000000)
    if isinstance(limit, bool) or not isinstance(limit, int) or limit < 0:
        raise ValueError('g2p5.max_grid_cells must be a nonnegative integer')


def build(source, config_path, output, grid_tool):
    source, output = Path(source).resolve(), Path(output).resolve()
    config_path, grid_tool = Path(config_path).resolve(), Path(grid_tool).resolve()
    if output == source or source in output.parents or output in source.parents:
        raise ValueError('Choose a separate sibling output directory')
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite existing output: {output}')
    summary = json.loads((source / 'summary.json').read_text())
    if summary.get('source') != 'retained_keyframes':
        raise ValueError('Expected a completed round-6 keyframe-cleaning run, not an old map-mode result')
    if not grid_tool.is_file():
        raise FileNotFoundError('Missing build_navigation_grid; compile the new executable first')
    config = yaml.safe_load(config_path.read_text())
    validate_navigation_config(config)
    _, contract = load_frame_contract(source)
    if contract != 'imu_v2':
        raise ValueError('Static navigation rebuilding requires IMU v2 frames')
    traj = Trajectory(source / 'keyframes.txt', np.finfo(float).max)
    entries = load_manifest(source, traj)
    resolution = float(config['system'].get('map_export', {}).get('localization_resolution', .05))
    if not math.isfinite(resolution) or not .02 <= resolution <= .2:
        raise ValueError('localization resolution must be in [0.02, 0.2] metres')
    output.mkdir(parents=True, exist_ok=False)
    incomplete = output / 'INCOMPLETE'
    incomplete.write_text('Do not load this directory until BUILD_COMPLETE.json exists.\n')
    started = time.monotonic()
    # All intermediate data is below the new output; source PCD/frames remain untouched.
    with tempfile.TemporaryDirectory(prefix='work_', dir=output) as work:
        work = Path(work)
        frame_list = work / 'frames.txt'
        with frame_list.open('w') as stream:
            for i, (identifier, timestamp, path) in enumerate(entries):
                pose = [*traj.p[i], *traj.q[i]]
                stream.write(f'{identifier} {timestamp:.17g} ' + ' '.join(f'{float(x):.17g}' for x in pose) +
                             ' ' + json.dumps(str(path), ensure_ascii=False) + '\n')
        # Run grid rendering first, so invalid geometry/budget fails before expensive tile aggregation.
        subprocess.run([str(grid_tool), '--config='+str(config_path), '--input_frames='+str(frame_list),
                        '--frame_contract='+str(source / 'map_frames.yaml'), '--output='+str(output)], check=True)
        grid = yaml.safe_load((output / 'map.yaml').read_text())
        if not np.isclose(grid['resolution'], config['g2p5']['grid_map_resolution'], rtol=1e-6, atol=1e-9):
            raise ValueError('Navigation YAML resolution differs from configured grid')
        if len(grid['origin']) != 3 or not np.isfinite(grid['origin']).all():
            raise ValueError('Navigation origin must be finite [x,y,yaw]')
        if grid['image'] != 'map.pgm':
            raise ValueError('Expected map.pgm from navigation renderer')
        with (output / 'map.pgm').open('rb') as image:
            if image.readline().strip() != b'P5':
                raise ValueError('Expected a binary PGM navigation image')
            dimensions = [int(value) for value in image.readline().split()]
            if len(dimensions) != 2 or min(dimensions) <= 0 or image.readline().strip() != b'255':
                raise ValueError('Invalid navigation image dimensions')
            if (output / 'map.pgm').stat().st_size - image.tell() != dimensions[0]*dimensions[1]:
                raise ValueError('Navigation image body size mismatch')
            cell_counts = dict(occupied_cells=0, free_cells=0, unknown_cells=0)
            while True:
                block = image.read(1024*1024)
                if not block:
                    break
                pixels = np.frombuffer(block, np.uint8)
                cell_counts['occupied_cells'] += int((pixels == 0).sum())
                cell_counts['free_cells'] += int((pixels == 255).sum())
                cell_counts['unknown_cells'] += int(((pixels != 0) & (pixels != 255)).sum())
        with sqlite3.connect(work / 'voxels.sqlite3') as database:
            stats = aggregate_tiles(source / 'global_clean.pcd', database, resolution)
            stats.update(export_tiles(database, output, [*traj.p[0], *traj.q[0]]))
    for name in ('keyframes.txt', 'map_frames.yaml'):
        shutil.copy2(source / name, output / name)
    local_config = yaml.safe_load(config_path.read_text())
    local_config['system']['map_path'] = str(output) + '/'
    maps = local_config.setdefault('maps', {})
    maps.update(load_dyn_cloud=False, save_dyn_when_quit=False, save_dyn_when_unload=False)
    with (output / 'localization.yaml').open('x') as stream:
        yaml.safe_dump(local_config, stream, allow_unicode=True, sort_keys=False)
    stats.update(cell_counts)
    stats.update(source=str(source), localization_resolution=resolution,
                 navigation_resolution=grid['resolution'], navigation_width=dimensions[0],
                 navigation_height=dimensions[1], config=str(config_path),
                 elapsed_seconds=time.monotonic()-started,
                 navigation_method='G2P5 rays from cleaned IMU-frame keyframes')
    (output / 'BUILD_COMPLETE.json').write_text(json.dumps(stats, indent=2) + '\n')
    incomplete.unlink()
    print(f'Completed navigation map: {output}\n'
          f'Localization: {output / "localization.yaml"}\nNav2 map: {output / "map.yaml"}', flush=True)
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path, help='Round-6 clean_diagnostics/keyframes_* directory')
    parser.add_argument('--config', required=True, type=Path, help='Your validated sensor/G2P5 configuration')
    parser.add_argument('--output', required=True, type=Path, help='A NEW separate navigation-map directory')
    parser.add_argument('--grid-tool', type=Path, default=Path(__file__).resolve().parents[1] / 'bin/build_navigation_grid')
    args = parser.parse_args()
    build(args.source, args.config, args.output, args.grid_tool)


if __name__ == '__main__':
    main()
