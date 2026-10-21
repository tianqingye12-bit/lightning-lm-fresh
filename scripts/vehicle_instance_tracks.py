"""Conservative static-vehicle instance association in the exported map frame."""
import cv2
import numpy as np
from scipy.spatial import cKDTree
from clean_map_instances import associate_instances
from segmentation_quality import accepted_instances
from seed_local_cleanup import labels


DEFAULTS = dict(enabled=False, min_pixels=6, min_points=12, link_m=.35,
                min_images=3, min_point_images=2, min_baseline_m=.35,
                min_mean_confidence=.45, max_gap_s=3., match_distance_m=2.,
                min_overlap=.15, max_center_drift_m=1.5, padding_m=.4,
                max_seed_distance_m=1., max_extent_m=[7., 3.5, 3.],
                max_candidates=30000, max_tracks=1000, max_memberships=500000)


def options(cfg):
    supplied=cfg.get('vehicle_instance_tracks',{})
    if not isinstance(supplied,dict) or set(supplied)-set(DEFAULTS):
        raise ValueError('Unknown vehicle_instance_tracks option')
    opts=dict(DEFAULTS,**supplied)
    if not isinstance(opts['enabled'],bool):raise ValueError('Instance enabled must be boolean')
    for name in ('min_pixels','min_points','min_images','min_point_images','max_candidates','max_tracks','max_memberships'):
        value=opts[name]
        if isinstance(value,bool) or not isinstance(value,int) or not 1<=value<=1000000:
            raise ValueError(f'Invalid vehicle_instance_tracks.{name}')
    for name in ('link_m','min_baseline_m','max_gap_s','match_distance_m','max_center_drift_m','padding_m','max_seed_distance_m'):
        value=opts[name]
        if isinstance(value,bool) or not isinstance(value,(float,int)) or not np.isfinite(value) or not 0<value<=10:
            raise ValueError(f'Invalid vehicle_instance_tracks.{name}')
    for name in ('min_overlap','min_mean_confidence'):
        value=opts[name]
        if isinstance(value,bool) or not isinstance(value,(float,int)) or not np.isfinite(value) or not 0<value<=1:
            raise ValueError(f'Invalid vehicle_instance_tracks.{name}')
    extent=np.asarray(opts['max_extent_m'],float)
    if extent.shape!=(3,) or not np.isfinite(extent).all() or (extent<=0).any() or (extent>10).any():
        raise ValueError('Instance max_extent_m must contain three dimensions in (0,10]')
    return opts


def extract(prediction,scene,cam,u,v,z,visible,floor,cfg):
    """Retain separate mask components. Unclassified pixels always abstain."""
    opts=options(cfg)
    eligible=accepted_instances(prediction,cfg)
    union=prediction.masks.any(axis=0) if len(prediction.masks) else np.zeros(scene.shape,bool)
    observations=[]
    radius=cfg['segmentation_quality'].get('core_erode_px',1)
    kernel=np.ones((2*radius+1,)*2,np.uint8)
    for i in np.flatnonzero(eligible):
        mask=prediction.masks[i]
        core=cv2.erode(mask.astype(np.uint8),kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool)
        seeds,_,_=associate_instances(core[None],scene,u,v,z,visible&~floor,dict(cfg,mask_dilate_px=0))
        seeds &= visible&~floor
        pixel=np.unique(v[seeds]*scene.shape[1]+u[seeds])
        if len(pixel)<opts['min_pixels']:continue
        low,high=np.quantile(z[seeds],[.05,.95])
        candidate=np.flatnonzero(visible&~floor&mask[v,u]&(z>=low-.25)&(z<=high+.5))
        if len(candidate)>opts['max_candidates']:continue
        component=labels(cam[candidate],opts['link_m'])
        seed_counts=np.bincount(component,weights=seeds[candidate])
        sizes=np.bincount(component)
        for label in np.flatnonzero((seed_counts>=opts['min_pixels'])&(sizes>=opts['min_points'])):
            ids=candidate[component==label]
            if len(np.unique(v[ids]*scene.shape[1]+u[ids]))<opts['min_pixels']:continue
            observations.append(dict(indices=ids, class_id=int(prediction.classes[i]),score=float(prediction.scores[i])))
    # Shared pixels/points are ambiguous between instances, not a reason to merge them.
    ownership=np.zeros(len(z),np.int32)
    for obs in observations:ownership[obs['indices']]+=1
    retained=[];scores=np.zeros(len(z),np.float32)
    for obs in observations:
        ids=obs['indices'][ownership[obs['indices']]==1]
        if len(ids)<opts['min_points'] or len(np.unique(v[ids]*scene.shape[1]+u[ids]))<opts['min_pixels']:continue
        obs=dict(obs,indices=ids);retained.append(obs)
        scores[ids]=obs['score']
    return scores>0,np.zeros(len(z),bool),union,scores,retained


def oriented_frame(points):
    center=np.mean(points,axis=0)
    xy=points[:,:2]-center[:2]
    _,vectors=np.linalg.eigh(xy.T@xy)
    rotation=np.eye(3);rotation[:2,:2]=vectors[:,::-1]
    local=(points-center)@rotation
    return center,rotation,local.min(axis=0),local.max(axis=0)


class InstanceTracks:
    def __init__(self,xyz,cfg):
        self.xyz=xyz;self.opts=options(cfg);self.tracks=[];self.last=-np.inf
        self.memberships=0;self.used=set();self.active=False
        self.metrics=dict(duplicate_images=0,ambiguous_matches=0,resource_rejections=0,observations=0)

    def begin_image(self,timestamp,camera):
        camera=np.asarray(camera,float)
        if not np.isfinite(timestamp) or camera.shape!=(3,) or not np.isfinite(camera).all() or timestamp<self.last:
            raise ValueError('Instance images must be finite and time ordered')
        self.active=timestamp!=self.last
        self.metrics['duplicate_images']+=int(not self.active)
        self.last=float(timestamp);self.camera=camera.copy();self.used=set()

    def add(self,ids,class_id,score):
        if not self.active:return
        ids=np.unique(np.asarray(ids,dtype=np.intp))
        if len(ids)<self.opts['min_points']:return
        if ids[0]<0 or ids[-1]>=len(self.xyz):raise ValueError('Invalid instance point indices')
        center=self.xyz[ids].mean(axis=0);matches=[]
        for i,t in enumerate(self.tracks):
            if i in self.used or t['class_id']!=class_id or self.last-t['last']>self.opts['max_gap_s']:continue
            if np.linalg.norm(center-t['center'])>self.opts['match_distance_m']:continue
            shared=sum(int(j) in t['counts'] for j in ids)
            overlap=shared/max(1,min(len(ids),len(t['counts'])))
            if shared>=self.opts['min_pixels'] and overlap>=self.opts['min_overlap']:matches.append((overlap,i))
        matches.sort(reverse=True)
        if len(matches)>1 and matches[0][0]-matches[1][0]<.1:
            self.metrics['ambiguous_matches']+=1;return
        if matches:
            index=matches[0][1];t=self.tracks[index]
            extra=sum(int(j) not in t['counts'] for j in ids)
        else:
            if len(self.tracks)>=self.opts['max_tracks']:
                self.metrics['resource_rejections']+=1;return
            index=len(self.tracks);t=None;extra=len(ids)
        if self.memberships+extra>self.opts['max_memberships']:
            self.metrics['resource_rejections']+=1;return
        if t is None:
            t=dict(class_id=int(class_id),counts={},images=0,first_center=center.copy(),center=center.copy(),
                   first_camera=self.camera.copy(),baseline=0.,drift=0.,score_sum=0.,last=self.last)
            self.tracks.append(t)
        for j in ids:t['counts'][int(j)]=t['counts'].get(int(j),0)+1
        self.memberships+=extra;t['images']+=1;t['score_sum']+=float(score)
        t['drift']=max(t['drift'],float(np.linalg.norm(center-t['first_center'])))
        t['baseline']=max(t['baseline'],float(np.linalg.norm(self.camera-t['first_camera'])))
        t['center']=center;t['last']=self.last;self.used.add(index);self.metrics['observations']+=1
        return index

    def finish(self,floor,spatial_index):
        remove=np.zeros(len(self.xyz),bool);seed_mask=remove.copy();reports=[]
        limits=np.asarray(self.opts['max_extent_m'])
        for index,t in enumerate(self.tracks):
            if index%20==0:print(f'Instance completion: {index}/{len(self.tracks)} tracks',flush=True)
            ids=np.array([j for j,count in t['counts'].items() if count>=self.opts['min_point_images']],np.intp)
            report=dict(id=index,class_id=t['class_id'],images=t['images'],baseline_m=t['baseline'],
                        center_drift_m=t['drift'],mean_confidence=t['score_sum']/t['images'],
                        seed_points=len(ids),added_points=0,status='accepted')
            reason=None
            if t['images']<self.opts['min_images']:reason='insufficient_images'
            elif t['baseline']<self.opts['min_baseline_m']:reason='insufficient_baseline'
            elif t['drift']>self.opts['max_center_drift_m']:reason='unstable_geometry'
            elif report['mean_confidence']<self.opts['min_mean_confidence']:reason='low_confidence'
            elif len(ids)<self.opts['min_points']:reason='insufficient_repeated_points'
            if reason:report['status']=reason;reports.append(report);continue
            center,rotation,low,high=oriented_frame(self.xyz[ids])
            if ((high-low)>limits).any():
                report['status']='oversized';reports.append(report);continue
            ids=ids[~floor[ids]];seed_mask[ids]=True;remove[ids]=True
            pad=np.minimum(self.opts['padding_m'],(limits-(high-low))/2)
            low-=pad;high+=pad
            box_center=center+((low+high)/2)@rotation.T
            radius=float(np.linalg.norm(high-low)/2+1e-9)
            report.update(center=box_center.tolist(),axes=rotation.tolist(),extent_m=(high-low).tolist())
            if spatial_index.query_ball_point(box_center,radius,return_length=True)>self.opts['max_candidates']:
                report['completion']='candidate_limit';reports.append(report);continue
            candidates=np.asarray(spatial_index.query_ball_point(box_center,radius),np.intp)
            local=(self.xyz[candidates]-center)@rotation
            candidates=candidates[(local>=low-1e-9).all(axis=1)&(local<=high+1e-9).all(axis=1)&~floor[candidates]]
            if not len(ids):report['completion']='no_seeds';reports.append(report);continue
            distance=cKDTree(self.xyz[ids]).query(self.xyz[candidates])[0]
            candidates=candidates[distance<=self.opts['max_seed_distance_m']]
            component=labels(self.xyz[candidates],self.opts['link_m'])
            support=np.bincount(component,weights=np.isin(candidates,ids))
            chosen=candidates[support[component]>=self.opts['min_pixels']]
            report['added_points']=int((~remove[chosen]).sum());remove[chosen]=True
            report['completion']='bounded_connected_points';reports.append(report)
        return remove,dict(enabled=True,**self.metrics,tracks=len(reports),
                           accepted_tracks=sum(r['status']=='accepted' for r in reports),
                           seed_voxels=int(seed_mask.sum()),final_voxels=int(remove.sum()),
                           instances=reports,scope='static_geometry_only')
