"""Opt-in map association/voting matching historical commit 4173ff3."""
import cv2
import numpy as np
from clean_map_instances import normalize_instances


def legacy_enabled(cfg):
    enabled=cfg.get('legacy_map_cleanup',False)
    if not isinstance(enabled,bool):raise ValueError('legacy_map_cleanup must be boolean')
    if enabled:
        radius=cfg.get('mask_dilate_px')
        margin=cfg.get('depth_margin')
        if isinstance(radius,bool) or not isinstance(radius,int) or not 0<=radius<=50:
            raise ValueError('Legacy mask_dilate_px must be an integer in [0,50]')
        if isinstance(margin,bool) or not isinstance(margin,(int,float)) or not np.isfinite(margin) or margin<0:
            raise ValueError('Legacy depth_margin must be finite and nonnegative')
    return enabled


def associate_legacy(masks,scene,u,v,z,cfg):
    instances=normalize_instances(masks)
    mask=instances.any(axis=0) if len(instances) else np.zeros(scene.shape,bool)
    radius=cfg['mask_dilate_px']
    front=np.where(mask,scene,1e6).astype(np.float32)
    front=cv2.erode(front,np.ones((2*radius+1,)*2,np.uint8))
    hit=(front[v,u]<1e5)&(z<=front[v,u]+cfg['depth_margin'])
    return hit,mask


def legacy_votes(visible,vehicle,floor,cfg):
    observed=visible>=cfg['min_views']
    remove=observed&(vehicle/np.maximum(visible,1)>=cfg['vehicle_ratio'])&~floor
    unknown=~(remove|floor)
    counts=dict(total=len(visible),removed=int(remove.sum()),protected=int(floor.sum()),background=0,
        uncertain=int(unknown.sum()),unobserved=int((visible==0).sum()),blocked_coverage=0,
        blocked_baseline=0,blocked_confidence=0)
    return observed,remove,unknown,counts
