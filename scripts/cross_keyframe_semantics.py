"""Unique-image world-voxel evidence; disk storage, one image of pending arrays."""
from pathlib import Path
import sqlite3
import tempfile
from types import SimpleNamespace
import numpy as np


def cross_frame_options(cfg):
    opts=dict(enabled=False,voxel_size=.1)
    supplied=cfg.get('cross_keyframe_semantics',{})
    if not isinstance(supplied,dict) or set(supplied)-set(opts):
        raise ValueError('Unknown cross_keyframe_semantics option')
    opts.update(supplied)
    if not isinstance(opts['enabled'],bool):raise ValueError('cross_keyframe_semantics.enabled must be boolean')
    size=opts['voxel_size']
    if isinstance(size,bool) or not isinstance(size,(int,float)) or not np.isfinite(size) or not .02<=size<=.2:
        raise ValueError('cross_keyframe_semantics.voxel_size must be in [0.02, 0.2] metres')
    return opts


def voxel_keys(xyz, resolution):
    xyz=np.asarray(xyz,dtype=float)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():
        raise ValueError('Semantic voxel coordinates must be finite Nx3')
    keys=np.floor(xyz/resolution)
    if np.any(np.abs(keys)>=2.**63):raise ValueError('Semantic voxel coordinate exceeds int64')
    return keys.astype(np.int64)


def merge_events(keys, positive, background, scores):
    unique,inverse=np.unique(keys,axis=0,return_inverse=True)
    pos=np.zeros(len(unique),np.uint8); bg=pos.copy(); score=np.zeros(len(unique),float)
    np.maximum.at(pos,inverse,positive)
    np.maximum.at(bg,inverse,background)
    np.maximum.at(score,inverse,np.where(positive,scores,0.))
    return unique,pos,bg,score


class SemanticVoxelEvidence:
    def __init__(self,directory,resolution):
        self.resolution=cross_frame_options({'cross_keyframe_semantics':{'voxel_size':resolution}})['voxel_size']
        self.work=tempfile.TemporaryDirectory(prefix='.semantic_votes_',dir=directory)
        self.db=sqlite3.connect(str(Path(self.work.name)/'votes.sqlite'))
        self.db.execute('PRAGMA cache_size=-16384')
        self.db.execute('PRAGMA temp_store=FILE')
        self.db.execute('PRAGMA journal_mode=OFF')
        self.db.execute('PRAGMA synchronous=OFF')
        self.db.execute('CREATE TABLE votes(x INTEGER,y INTEGER,z INTEGER,v INTEGER,p INTEGER,b INTEGER,s REAL,'
                        'cx REAL,cy REAL,cz REAL,d2 REAL,PRIMARY KEY(x,y,z)) WITHOUT ROWID')
        self.db.execute('CREATE TEMP TABLE lookup(i INTEGER PRIMARY KEY,x INTEGER,y INTEGER,z INTEGER)')
        self.pending=[];self.last_timestamp=-np.inf;self.collect=False;self.in_image=False
        self.metrics=dict(unique_images=0,duplicate_images=0,voxel_image_pairs=0,conflicting_pairs=0)

    def begin_image(self,timestamp,camera_center):
        if self.in_image:raise ValueError('Finish the previous semantic image first')
        center=np.asarray(camera_center,dtype=float)
        if not np.isfinite(timestamp) or center.shape!=(3,) or not np.isfinite(center).all():
            raise ValueError('Invalid image timestamp/camera center')
        if timestamp<self.last_timestamp:raise ValueError('Semantic images must be time ordered')
        self.collect=timestamp!=self.last_timestamp
        self.metrics['duplicate_images']+=int(not self.collect)
        self.last_timestamp=float(timestamp);self.center=center.copy();self.pending=[];self.in_image=True

    def add(self,world,association):
        if not self.in_image:raise ValueError('begin_image must precede evidence')
        if not self.collect or association is None:return
        xyz=np.asarray(world)[association['indices']]
        keys=voxel_keys(xyz,self.resolution)
        positive=np.asarray(association['positive'],bool)
        background=np.asarray(association['background'],bool)
        scores=np.asarray(association['scores'],float)
        if (positive.shape!=(len(keys),) or background.shape!=positive.shape or scores.shape!=positive.shape or
                not np.isfinite(scores).all() or np.any((scores<0)|(scores>1))):
            raise ValueError('Invalid per-point semantic evidence')
        # Reduce each active frame before combining the frames of this image.
        if len(keys):self.pending.append(merge_events(keys,positive,background,scores))

    def end_image(self):
        if not self.in_image:raise ValueError('No semantic image is open')
        if self.collect:
            self.metrics['unique_images']+=1
            if self.pending:
                values=[np.concatenate([part[j] for part in self.pending]) for j in range(4)]
                keys,pos,bg,scores=merge_events(*values)
                conflict=(pos>0)&(bg>0)
                self.metrics['voxel_image_pairs']+=len(keys)
                self.metrics['conflicting_pairs']+=int(conflict.sum())
                pos,bg=(pos>0)&~conflict,(bg>0)&~conflict
                cx,cy,cz=map(float,self.center)
                query='''INSERT INTO votes VALUES(?,?,?,1,?,?,?,?,?,?,0)
                    ON CONFLICT(x,y,z) DO UPDATE SET v=v+1,p=p+excluded.p,b=b+excluded.b,s=s+excluded.s,
                    d2=MAX(d2,CASE WHEN excluded.p>0 AND cx IS NOT NULL THEN
                    (cx-excluded.cx)*(cx-excluded.cx)+(cy-excluded.cy)*(cy-excluded.cy)+(cz-excluded.cz)*(cz-excluded.cz)
                    ELSE 0 END),cx=COALESCE(cx,excluded.cx),cy=COALESCE(cy,excluded.cy),cz=COALESCE(cz,excluded.cz)'''
                self.db.executemany(query,((int(k[0]),int(k[1]),int(k[2]),int(p),int(b),float(s) if p else 0.,
                    cx if p else None,cy if p else None,cz if p else None) for k,p,b,s in zip(keys,pos,bg,scores)))
                self.db.commit()
        self.pending=[];self.in_image=False

    def lookup(self,world,views=0):
        if self.in_image:raise ValueError('Cannot export unfinished semantic image')
        keys=voxel_keys(world,self.resolution)
        unique,inverse=np.unique(keys,axis=0,return_inverse=True)
        values=np.zeros((len(unique),5),float)
        for start in range(0,len(unique),10000):
            chunk=unique[start:start+10000]
            self.db.execute('DELETE FROM lookup')
            self.db.executemany('INSERT INTO lookup VALUES(?,?,?,?)',
                ((i,int(k[0]),int(k[1]),int(k[2])) for i,k in enumerate(chunk)))
            rows=self.db.execute('SELECT COALESCE(v,0),COALESCE(p,0),COALESCE(b,0),COALESCE(s,0),COALESCE(d2,0) '
                'FROM lookup l LEFT JOIN votes t ON l.x=t.x AND l.y=t.y AND l.z=t.z ORDER BY l.i').fetchall()
            values[start:start+len(chunk)]=np.asarray(rows,float)
        values=values[inverse]
        return SimpleNamespace(visible=values[:,0].astype(np.int64),vehicle=values[:,1].astype(np.int64),
            background=values[:,2].astype(np.int64),score_sum=values[:,3],
            baseline=SimpleNamespace(maximum=np.sqrt(values[:,4])),views=views)

    def summary(self):
        count,positive,background=self.db.execute('SELECT count(*),COALESCE(sum(p>0),0),COALESCE(sum(b>0),0) FROM votes').fetchone()
        return dict(self.metrics,enabled=True,voxel_size_m=self.resolution,stored_voxels=count,positive_voxels=positive,
            background_voxels=background,evidence_unit='one image per world voxel',
            within_image_conflict='unknown',neighbor_expansion=False)

    def close(self):
        if self.db is not None:self.db.close();self.db=None
        self.pending=[];self.work.cleanup()
