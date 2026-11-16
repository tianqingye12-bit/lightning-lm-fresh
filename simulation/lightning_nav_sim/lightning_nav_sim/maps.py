"""Demo fixtures and bounded-memory PCD height-slice projection to Nav2 maps."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .grid import Grid, MAX_CELLS, save_grid


def demo(output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    cells = np.zeros((140, 200), dtype=np.int8)
    cells[:3,:] = cells[-3:,:] = cells[:,:3] = cells[:,-3:] = 100
    cells[55:85, 60:140] = 100  # Long building: routes on either side.
    cells[105:125, 25:40] = 100
    world = Grid(cells, .1, (0.,0.,0.))
    save_grid(world, output, 'world')
    save_grid(world, output, 'clean')
    raw = cells.copy()
    # Stale parked-car evidence partially blocks the lower passage.
    raw[13:40, 95:113] = 100
    save_grid(Grid(raw,.1,(0.,0.,0.)), output, 'raw')
    scenario = dict(world='world.yaml', start=[2.,2.5,0.],
                    goals=[[17.,2.5,0.],[17.,11.,math.pi/2],[5.,11.,math.pi]],
                    seed=42, obstacles=[dict(name='moving_vehicle', center=[15.,7.],
                        size=[1.8,.8], yaw=math.pi/2,
                        motion=dict(axis='y',amplitude=2.,period=24.,phase=0.))])
    (output/'scenario.yaml').write_text(yaml.safe_dump(scenario,sort_keys=False))
    (output/'README.txt').write_text(
        'Synthetic integration fixture. raw/clean maps are manually constructed, not algorithm outputs.\n'
        'Keep world.yaml and scenario.yaml identical for both trials.\n')
    return output/'scenario.yaml'


def pcd_blocks(path, block_size=100000):
    """Scalar/vector PCD fields, ASCII or uncompressed binary; no global point array."""
    types = {('F',4):'<f4',('F',8):'<f8',('I',1):'i1',('I',2):'<i2',('I',4):'<i4',
             ('U',1):'u1',('U',2):'<u2',('U',4):'<u4'}
    with Path(path).open('rb') as stream:
        meta = {}
        for _ in range(100):
            raw = stream.readline(65537)
            if not raw or len(raw) > 65536:
                raise ValueError('Invalid PCD header')
            parts = raw.decode('ascii').split()
            if parts and not parts[0].startswith('#'):
                meta[parts[0].upper()] = parts[1:]
            if parts and parts[0].upper() == 'DATA':
                break
        else:
            raise ValueError('PCD header too long')
        names, sizes, kinds = meta['FIELDS'], meta['SIZE'], meta['TYPE']
        counts = [int(v) for v in meta.get('COUNT', ['1']*len(names))]
        if not len(names) == len(sizes) == len(kinds) == len(counts) or min(counts) < 1:
            raise ValueError('Inconsistent PCD fields')
        fields = [(name if name != '_' else f'_pad{i}', types[(kind,int(size))],
                   (count,) if count > 1 else ())
                  for i,(name,size,kind,count) in enumerate(zip(names,sizes,kinds,counts))]
        dtype = np.dtype(fields)
        for axis in ('x','y','z'):
            if axis not in dtype.names or dtype[axis].shape:
                raise ValueError('PCD must have scalar x/y/z fields')
        total = int(meta['POINTS'][0])
        if total < 0:
            raise ValueError('Negative point count')
        encoding = meta['DATA'][0]
        if encoding == 'binary':
            if Path(path).stat().st_size-stream.tell() != total*dtype.itemsize:
                raise ValueError('PCD body length mismatch')
            for begin in range(0,total,block_size):
                block = np.frombuffer(stream.read(min(block_size,total-begin)*dtype.itemsize),dtype=dtype)
                yield np.column_stack([block[k] for k in ('x','y','z')])
        elif encoding == 'ascii':
            columns = np.cumsum([0,*counts[:-1]])
            xyz_columns = [int(columns[names.index(axis)]) for axis in ('x','y','z')]
            rows = []
            count = 0
            for line in stream:
                if not line.strip():
                    continue
                row = line.split()
                if len(row) != sum(counts):
                    raise ValueError('PCD ASCII column count mismatch')
                rows.append([float(row[k]) for k in xyz_columns])
                count += 1
                if len(rows) == block_size:
                    yield np.asarray(rows); rows=[]
            if count != total:
                raise ValueError('PCD ASCII point count mismatch')
            if rows:
                yield np.asarray(rows)
        else:
            raise ValueError('binary_compressed PCD is not supported here; export an uncompressed binary/ASCII copy first')


def project(source, output, bounds, ground_z, resolution=.1, obstacle_min=.2,
            obstacle_max=1.8, ground_tolerance=.12):
    bounds = np.asarray(bounds,dtype=float)
    params = [*bounds,ground_z,resolution,obstacle_min,obstacle_max,ground_tolerance]
    if (len(bounds) != 4 or not np.isfinite(params).all() or resolution < .02
            or not 0 <= ground_tolerance < obstacle_min < obstacle_max
            or bounds[2] <= bounds[0] or bounds[3] <= bounds[1]):
        raise ValueError('Invalid ROI/height thresholds; require ground tolerance < obstacle minimum < maximum')
    dimensions=(bounds[2:]-bounds[:2])/resolution
    if not np.isfinite(dimensions).all() or np.any(dimensions > MAX_CELLS):
        raise ValueError('ROI exceeds 8 million cells; crop the region or increase resolution')
    width,height = [math.ceil(float(v)) for v in dimensions]
    if width*height > MAX_CELLS:
        raise ValueError('ROI exceeds 8 million cells; crop the region or increase resolution')
    output = Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    (output/'INCOMPLETE').touch()
    cells = np.full((height,width),-1,dtype=np.int8)
    total=valid=0
    for xyz in pcd_blocks(source):
        total += len(xyz)
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        inside = ((xyz[:,0]>=bounds[0]) & (xyz[:,0]<bounds[2]) &
                  (xyz[:,1]>=bounds[1]) & (xyz[:,1]<bounds[3]))
        xyz = xyz[inside]
        valid += len(xyz)
        indices = np.floor((xyz[:,:2]-bounds[:2])/resolution).astype(np.int64)
        z = xyz[:,2]-ground_z
        floor = np.abs(z)<=ground_tolerance
        obstacle = (z>=obstacle_min) & (z<=obstacle_max)
        # A ground sample establishes free space only in its own cell. Obstacles win.
        x,y = indices[floor].T
        existing = cells[y,x]
        cells[y[existing != 100],x[existing != 100]] = 0
        x,y = indices[obstacle].T
        cells[y,x] = 100
    if not valid or not np.any(cells == 100) or not np.any(cells == 0):
        raise ValueError('ROI must contain both observed ground and obstacles; check ground_z and the height slice')
    grid = Grid(cells,resolution,(*bounds[:2],0.))
    save_grid(grid,output)
    report = dict(source=str(Path(source).resolve()), input_points=total,roi_points=valid,
                  bounds=bounds.tolist(),ground_z=ground_z,resolution=resolution,
                  obstacle_min=obstacle_min,obstacle_max=obstacle_max,ground_tolerance=ground_tolerance,
                  occupied_cells=int((cells==100).sum()),free_cells=int((cells==0).sum()),
                  unknown_cells=int((cells<0).sum()),map_hash=grid.fingerprint(),
                  assumption='Single flat ground plane; unobserved cells remain unknown, not free.')
    (output/'projection.json').write_text(json.dumps(report,indent=2))
    (output/'INCOMPLETE').unlink()
    return output/'map.yaml'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command',required=True)
    d = sub.add_parser('demo'); d.add_argument('--output',required=True)
    p = sub.add_parser('project')
    p.add_argument('--pcd',required=True); p.add_argument('--output',required=True)
    p.add_argument('--bounds',type=float,nargs=4,required=True,metavar=('XMIN','YMIN','XMAX','YMAX'))
    p.add_argument('--ground-z',type=float,required=True)
    p.add_argument('--resolution',type=float,default=.1)
    p.add_argument('--obstacle-min',type=float,default=.2)
    p.add_argument('--obstacle-max',type=float,default=1.8)
    p.add_argument('--ground-tolerance',type=float,default=.12)
    args=parser.parse_args()
    try:
        if args.command == 'demo':
            result=demo(args.output)
        else:
            result=project(args.pcd,args.output,args.bounds,args.ground_z,args.resolution,
                           args.obstacle_min,args.obstacle_max,args.ground_tolerance)
        print(result)
    except (OSError,ValueError,KeyError) as exc:
        parser.exit(1,str(exc)+'\n')


if __name__ == '__main__':
    main()
