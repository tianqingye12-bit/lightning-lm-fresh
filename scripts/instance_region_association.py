"""Bounded region matching for partial observations of the same static object."""
import numpy as np


def describe(points):
    points=np.asarray(points,float)
    center=points.mean(axis=0);xy=points[:,:2]-center[:2]
    values,vectors=np.linalg.eigh(xy.T@xy)
    axis=vectors[:,-1]
    return dict(center=center,low=points.min(axis=0),high=points.max(axis=0),
                yaw=float(np.arctan2(axis[1],axis[0])),
                elongation=float(values[-1]/max(values[0],1e-12)))


def region_score(observation,previous,first_center,all_low,all_high,opts):
    """Return score/rejection. No region is ever used as a deletion mask."""
    distance=float(np.linalg.norm(observation['center']-previous['center']))
    if distance>opts['region_center_m']:
        return None,'region_center'
    if np.linalg.norm(observation['center']-first_center)>opts['region_center_m']:
        return None,'region_anchor'
    pad=opts['region_padding_m']
    a0,a1=observation['low']-pad,observation['high']+pad
    b0,b1=previous['low']-pad,previous['high']+pad
    sa,sb=a1-a0,b1-b0
    if (np.maximum(sa/sb,sb/sa)>opts['region_size_ratio']).any():
        return None,'region_size'
    overlap=float(np.prod(np.maximum(0,np.minimum(a1,b1)-np.maximum(a0,b0))))
    va,vb=float(np.prod(sa)),float(np.prod(sb))
    iou=overlap/max(va+vb-overlap,1e-12)
    coverage=overlap/max(min(va,vb),1e-12)
    if iou<opts['region_iou'] or coverage<opts['region_cover']:
        return None,'region_overlap'
    if (observation['elongation']>=1.5 and previous['elongation']>=1.5 and
        max(sa[:2])>=.5 and max(sb[:2])>=.5):
        angle=abs((observation['yaw']-previous['yaw']+np.pi/2)%np.pi-np.pi/2)
        if np.degrees(angle)>opts['region_yaw_deg']:
            return None,'region_yaw'
    span=np.maximum(all_high,observation['high'])-np.minimum(all_low,observation['low'])
    limits=opts['max_extent_m']
    if np.linalg.norm(span[:2])>np.hypot(limits[0],limits[1]) or span[2]>limits[2]:
        return None,'region_extent'
    # Shared-point candidates have scores >=1 and remain preferable.
    score=.4+.3*iou+.2*coverage+.1*(1-distance/opts['region_center_m'])
    return float(score),None


def mutual_matches(candidates,margin):
    """Mutual best with row AND column ambiguity checks, no greedy ordering."""
    result={};rejected={}
    columns={}
    for row,values in enumerate(candidates):
        for score,index,kind in values:columns.setdefault(index,[]).append((score,row))
    for row,values in enumerate(candidates):
        ordered=sorted(values,reverse=True)
        if not ordered:continue
        if len(ordered)>1 and ordered[0][0]-ordered[1][0]<margin-1e-12:
            rejected[row]='ambiguous_row';continue
        score,index,kind=ordered[0]
        competing=sorted(columns[index],reverse=True)
        if competing[0][1]!=row:
            rejected[row]='one_to_one_conflict';continue
        if len(competing)>1 and competing[0][0]-competing[1][0]<margin-1e-12:
            rejected[row]='ambiguous_column';continue
        result[row]=(index,kind)
    return result,rejected
