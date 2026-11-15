#!/usr/bin/env python3
"""Validate the NEW scan timing contract and its anchors without loading point clouds."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import yaml
from pose_trajectory import PoseTrajectory


def audit(directory,require_normalized=True):
    directory=Path(directory)
    metadata=yaml.safe_load((directory/'trajectory.yaml').read_text())
    with (directory/'scan_trajectory.csv').open(newline='') as stream:
        rows=list(csv.DictReader(stream))
    report=dict(metadata=metadata,rows=len(rows),
        statuses=dict(Counter(r['status'] for r in rows)),
        usable_rows=sum(r['usable']=='1' for r in rows),
        scans_with_negative_offsets=sum(int(r['point_negative'])>0 for r in rows),
        scans_with_nonfinite_offsets=sum(int(r['point_nonfinite'])>0 for r in rows),
        point_end_mismatch_scans=sum(not np.isfinite(v:=float(r['point_end_difference_s'])) or abs(v)>.002 for r in rows))
    duration=np.array([float(r['scan_duration_s']) for r in rows])
    report['scan_duration_median_s']=float(np.median(duration)) if len(duration) else None
    values=np.loadtxt(directory/'keyframes.txt',ndmin=2)
    values=values[np.argsort(values[:,1])]
    keyframes=SimpleNamespace(t=values[:,1],p=values[:,2:5],q=values[:,5:9],
                              ids=values[:,0].astype(int),max_gap=2.)
    try:
        traj=PoseTrajectory(directory,dict(pose_trajectory=dict(source='dense',max_gap_s=.2,
                            require_normalized_timing=require_normalized)),keyframes)
        report.update(valid=True,trajectory=traj.description)
    except (ValueError,KeyError,IndexError) as exc:
        report.update(valid=False,error=str(exc))
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map',required=True)
    p.add_argument('--allow-legacy',action='store_true',help='Inspect older exports; never changes their usable flags')
    args=p.parse_args()
    try:report=audit(args.map,not args.allow_legacy)
    except (OSError,ValueError,KeyError) as exc:p.exit(1,str(exc)+'\n')
    print(json.dumps(report,indent=2,allow_nan=False))
    raise SystemExit(0 if report['valid'] else 1)


if __name__=='__main__':main()
