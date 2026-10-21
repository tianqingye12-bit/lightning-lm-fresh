"""Transfer accepted world-voxel labels back to all copies in original keyframes."""
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import os
import numpy as np
from cross_keyframe_semantics import voxel_keys


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def label_keys(xyz,resolution):
    # Global PCD float32 rounding and double-precision keyframe transforms can
    # lie on opposite sides of the same voxel boundary. Snap only a 0.1% cell
    # neighborhood, consistently at creation and lookup (0.1mm at 0.1m).
    xyz=np.asarray(xyz,float)
    voxel_keys(xyz,resolution)  # Validate finite coordinates and int64 range.
    scaled=xyz/resolution;nearest=np.rint(scaled)
    return np.floor(np.where(np.abs(scaled-nearest)<=.001,nearest,scaled)).astype(np.int64)


def write_labels(path,directory,xyz,remove,floor_z,resolution):
    path=Path(path);directory=Path(directory)
    if not np.isfinite(resolution) or not .02<=resolution<=.2:raise ValueError('Label resolution must be in [0.02,0.2]')
    metadata=dict(version=1,coordinate_contract='imu_v2',resolution=float(resolution),boundary_snap_fraction=.001,complete=True,
                  keyframes_sha256=digest(directory/'keyframes.txt'),frames_sha256=digest(directory/'map_frames.yaml'))
    with tempfile.NamedTemporaryFile(dir=path.parent,suffix='.sqlite',delete=False) as stream:temp=Path(stream.name)
    db=None
    try:
        db=sqlite3.connect(str(temp))
        db.execute('CREATE TABLE metadata(value TEXT NOT NULL)')
        db.execute('INSERT INTO metadata VALUES(?)',(json.dumps(metadata),))
        db.execute('CREATE TABLE labels(x INTEGER,y INTEGER,z INTEGER,floor_z REAL,PRIMARY KEY(x,y,z)) WITHOUT ROWID')
        selected=np.flatnonzero(remove)
        for start in range(0,len(selected),10000):
            ids=selected[start:start+10000];keys=label_keys(xyz[ids],resolution)
            db.executemany('INSERT INTO labels VALUES(?,?,?,?) ON CONFLICT(x,y,z) DO UPDATE SET floor_z=MAX(floor_z,excluded.floor_z)',
                ((int(k[0]),int(k[1]),int(k[2]),float(floor_z[i])) for k,i in zip(keys,ids)))
        db.commit();db.close();db=None;os.replace(temp,path)
    finally:
        if db is not None:db.close()
        temp.unlink(missing_ok=True)


class LabelLookup:
    def __init__(self,path,directory):
        self.db=sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro',uri=True)
        try:
            self.metadata=json.loads(self.db.execute('SELECT value FROM metadata').fetchone()[0])
            m=self.metadata
            if m.get('version')!=1 or m.get('coordinate_contract')!='imu_v2' or m.get('complete') is not True:
                raise ValueError('Unsupported/incomplete instance labels')
            for name,key in (('keyframes.txt','keyframes_sha256'),('map_frames.yaml','frames_sha256')):
                if digest(Path(directory)/name)!=m[key]:raise ValueError(f'Label/map pose contract mismatch: {name}')
            self.resolution=float(m['resolution'])
            if m.get('boundary_snap_fraction')!=.001:raise ValueError('Unsupported label quantization')
            if not np.isfinite(self.resolution) or not .02<=self.resolution<=.2:raise ValueError('Invalid label resolution')
            self.db.execute('CREATE TEMP TABLE query(i INTEGER PRIMARY KEY,x INTEGER,y INTEGER,z INTEGER)')
        except Exception:
            self.db.close();raise

    def lookup(self,xyz):
        keys=label_keys(xyz,self.resolution);remove=np.zeros(len(keys),bool)
        for start in range(0,len(keys),10000):
            chunk=keys[start:start+10000]
            self.db.execute('DELETE FROM query')
            self.db.executemany('INSERT INTO query VALUES(?,?,?,?)',((i,int(k[0]),int(k[1]),int(k[2])) for i,k in enumerate(chunk)))
            for i,floor_z in self.db.execute('SELECT q.i,l.floor_z FROM query q JOIN labels l ON q.x=l.x AND q.y=l.y AND q.z=l.z'):
                remove[start+i]=xyz[start+i,2]>=floor_z
        return remove

    def close(self):self.db.close()
