#!/usr/bin/env python3
"""Explain disk usage and semantic rejection counters; no model or ROS required."""
import argparse
import csv
import json
from pathlib import Path


def vote_diagnostics(visible, positive, background, cfg, protected, pre_floor_remove):
    import numpy as np
    from clean_map_instances import association_options
    opts=association_options(cfg)
    hit=positive>0
    classified=positive+background
    veto=(background>=opts['min_background_views']) & (background/np.maximum(classified,1)>=opts['background_ratio'])
    return dict(positive_points=int(hit.sum()),
        blocked_min_views=int((hit & (visible<cfg['min_views'])).sum()),
        blocked_positive_views=int((hit & (positive<opts['min_vehicle_views'])).sum()),
        blocked_background=int((hit & veto).sum()),
        blocked_floor=int((pre_floor_remove & protected).sum()))


def audit(directory):
    directory=Path(directory).resolve()
    report=json.loads((directory/'summary.json').read_text())
    usage={}
    for entry in directory.iterdir():
        if entry.is_symlink():continue
        usage[entry.name]=entry.stat().st_size if entry.is_file() else sum(
            p.stat().st_size for p in entry.rglob('*') if p.is_file() and not p.is_symlink())
    views=dict(rows=0,with_objects=0,with_projection=0,with_vehicle_votes=0)
    path=directory/'views.csv'
    if path.exists():
        with path.open(newline='') as stream:
            for row in csv.DictReader(stream):
                views['rows']+=1
                views['with_objects']+=int(float(row.get('objects') or 0)>0)
                views['with_projection']+=int(float(row.get('projected') or 0)>0)
                views['with_vehicle_votes']+=int(float(row.get('vehicle_votes') or 0)>0)
    signals=[]
    if views['rows'] and not views['with_projection']:
        signals.append('没有投影到图像的点：先检查轨迹、坐标系、外参和时间。')
    elif views['with_projection'] and not views['with_objects']:
        signals.append('有投影但没有分割对象：检查模型、类别、置信度及原始图像。')
    elif views['with_objects'] and not views['with_vehicle_votes']:
        signals.append('有分割对象但没有车辆点票：检查投影位置和实例深度关联。')
    if report.get('floor_unknown_frames',0):
        signals.append('部分关键帧无法估计地面，删除候选被保留；检查 floor_unknown_frames。')
    return dict(directory=str(directory),incomplete=(directory/'INCOMPLETE').exists(),
        total_bytes=sum(usage.values()),signals=signals,
        total_mib=round(sum(usage.values())/1024**2,2),
        files_mib={k:round(v/1024**2,2) for k,v in sorted(usage.items(),key=lambda item:-item[1])},
        cleaning={k:report.get(k) for k in ('frames','frames_with_votes','frames_without_views',
            'floor_unknown_frames','selected_images','retained_points','removed_points',
            'evidence','vote_gates','global_export','selection','pose_queries','trajectory','cross_keyframe_semantics',
            'removed_points_without_local_views')},
        views=views,view_count_units='image-keyframe pairs; not independent images',
        overlays=[str(p) for p in sorted(directory.glob('overlay_*.jpg'))[:3]])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result',required=True)
    args=parser.parse_args()
    print(json.dumps(audit(args.result),indent=2,ensure_ascii=False))
