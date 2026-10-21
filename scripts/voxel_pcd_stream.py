"""Disk-backed PCD export: keep one observed point per world voxel, including all fields."""
from pathlib import Path
import sqlite3
import tempfile
import numpy as np


class VoxelPcdStream:
    def __init__(self, path, header, resolution):
        if not np.isfinite(resolution) or not .02 <= resolution <= .3:
            raise ValueError('Global voxel size must be in [0.02, 0.3] metres')
        self.path, self.header, self.resolution = Path(path), header, float(resolution)
        self.dtype, self.count, self.input_count = None, 0, 0
        self.work = tempfile.TemporaryDirectory(prefix='.voxel_export_', dir=self.path.parent)
        self.db = sqlite3.connect(str(Path(self.work.name)/'voxels.sqlite'))
        self.db.execute('PRAGMA cache_size=-16384')
        self.db.execute('PRAGMA temp_store=FILE')
        self.db.execute('PRAGMA journal_mode=OFF')
        self.db.execute('PRAGMA synchronous=OFF')
        self.db.execute('CREATE TABLE points(x INTEGER,y INTEGER,z INTEGER,p BLOB,PRIMARY KEY(x,y,z)) WITHOUT ROWID')

    def append(self, cloud):
        if self.dtype is None:
            self.dtype = cloud.dtype
        elif self.dtype != cloud.dtype:
            raise ValueError('Inconsistent keyframe PCD fields')
        for start in range(0,len(cloud),50000):
            block = cloud[start:start+50000]
            xyz = np.column_stack([block[k] for k in ('x','y','z')]).astype(float)
            finite = np.isfinite(xyz).all(axis=1)
            block, xyz = block[finite], xyz[finite]
            self.input_count += len(block)
            keys = np.floor(xyz/self.resolution)
            if np.any(np.abs(keys) >= 2.0**63):
                raise ValueError('Global voxel coordinates exceed int64')
            _, first = np.unique(keys.astype(np.int64),axis=0,return_index=True)
            keys, block = keys[first].astype(np.int64), block[first]
            self.db.executemany('INSERT OR IGNORE INTO points VALUES(?,?,?,?)',
                ((int(k[0]),int(k[1]),int(k[2]),p.tobytes()) for k,p in zip(keys,block)))
        self.db.commit()

    def finish(self):
        self.count = int(self.db.execute('SELECT count(*) FROM points').fetchone()[0])
        lines=[]
        for line in self.header:
            key=line.split()[0] if line else ''
            lines.append({'WIDTH':f'WIDTH {self.count}','HEIGHT':'HEIGHT 1',
                          'POINTS':f'POINTS {self.count}','DATA':'DATA binary'}.get(key,line))
        temporary=self.path.with_suffix('.pcd.tmp')
        with temporary.open('xb') as stream:
            stream.write(('\n'.join(lines)+'\n').encode('ascii'))
            cursor=self.db.execute('SELECT p FROM points ORDER BY x,y,z')
            while True:
                rows=cursor.fetchmany(25000)
                if not rows:break
                stream.write(b''.join(row[0] for row in rows))
        temporary.replace(self.path)
        self.close()

    def close(self):
        if self.db is not None:
            self.db.close()
            self.db=None
        self.work.cleanup()
