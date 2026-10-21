#!/usr/bin/env python3
"""Compare image pose sources/time offsets using manually matched STATIC landmarks.

Read-only for map and calibration. All candidate metrics use the SAME common
correspondences; pose failures and lost coverage remain visible in the report.
"""
import argparse
import copy
import csv
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from clean_map_runtime import file_digest, load_frame_contract, rigid_transform


def project(traj, timestamp, xyz, cfg, delta, T_imu_lidar):
    aligned = timestamp+cfg['time_offset']+delta
    info = traj.association_info(aligned)
    pose = traj.pose(aligned)
    count = len(xyz)
    if pose is None:
        return np.zeros((count,2)),np.zeros(count,bool),dict(info,aligned_timestamp=aligned,outside_image=count)
    T_cam_world = rigid_transform(cfg['T_cam_lidar'],'T_cam_lidar') @ np.linalg.inv(pose@T_imu_lidar)
    cam = xyz@T_cam_world[:3,:3].T + T_cam_world[:3,3]
    good = np.isfinite(cam).all(axis=1) & (cam[:,2]>.1)
    uv = np.zeros((count,2))
    if good.any():
        result,_=cv2.projectPoints(cam[good],np.zeros(3),np.zeros(3),
            np.asarray(cfg['K'],float).reshape(3,3),np.asarray(cfg['distortion'],float))
        uv[good]=result.reshape(-1,2)
    good &= np.isfinite(uv).all(axis=1)
    inside = good & (uv[:,0]>=0) & (uv[:,0]<cfg['width']) & (uv[:,1]>=0) & (uv[:,1]<cfg['height'])
    return uv,good,dict(info,aligned_timestamp=float(aligned),outside_image=int((~inside).sum()))


def evaluate(samples, trajectories, cfg, offsets, T_imu_lidar, output=None, max_overlays=20):
    if not samples or not offsets or not np.isfinite(offsets).all() or len(set(offsets))!=len(offsets):
        raise ValueError('Provide nonempty samples and distinct finite offset deltas')
    if not np.isfinite(cfg['time_offset']): raise ValueError('time_offset must be finite')
    variants=[(source,float(delta)) for source in trajectories for delta in offsets]
    errors={variant:[] for variant in variants}
    excluded={variant:0 for variant in variants}
    rows=[]; total=common_count=overlays=0
    for index,sample in enumerate(samples):
        timestamp=float(sample['timestamp'])
        xyz,pixels=np.asarray(sample['points_world'],float),np.asarray(sample['pixels'],float)
        if (not np.isfinite(timestamp) or xyz.ndim!=2 or xyz.shape[1]!=3 or len(xyz)==0 or
                pixels.shape!=(len(xyz),2) or not np.isfinite(xyz).all() or not np.isfinite(pixels).all() or
                (pixels<0).any() or (pixels[:,0]>=cfg['width']).any() or (pixels[:,1]>=cfg['height']).any()):
            raise ValueError('Each sample needs a finite timestamp, static Nx3 map points and matched Nx2 image pixels')
        predictions={variant:project(trajectories[variant[0]],timestamp,xyz,cfg,variant[1],T_imu_lidar) for variant in variants}
        common=np.logical_and.reduce([value[1] for value in predictions.values()])
        total+=len(xyz);common_count+=int(common.sum())
        image=None
        if output is not None and sample.get('image') and overlays<max_overlays:
            image=cv2.imread(str(sample['image']))
            if image is None or image.shape[:2]!=(cfg['height'],cfg['width']):
                raise ValueError('Diagnostic image missing or different from calibrated dimensions')
        for variant_index,(variant,(uv,good,info)) in enumerate(predictions.items()):
            residual=np.linalg.norm(uv[common]-pixels[common],axis=1)
            errors[variant].extend(residual.tolist());excluded[variant]+=int((~good).sum())
            rows.append(dict(info,sample=index,timestamp=timestamp,pose_source=variant[0],delta_s=variant[1],
                total_points=len(xyz),valid_points=int(good.sum()),common_points=int(common.sum()),
                rmse_px=float(np.sqrt(np.mean(residual**2))) if len(residual) else None))
            if image is not None and overlays<max_overlays:
                overlay=image.copy()
                for measured,predicted,valid in zip(pixels,uv,good):
                    measured=tuple(np.round(measured).astype(int));cv2.circle(overlay,measured,4,(0,255,0),1)
                    if valid and 0<=predicted[0]<cfg['width'] and 0<=predicted[1]<cfg['height']:
                        predicted=tuple(np.round(predicted).astype(int))
                        cv2.circle(overlay,predicted,3,(0,0,255),1);cv2.line(overlay,measured,predicted,(0,255,255),1)
                cv2.putText(overlay,f'{variant[0]} delta={variant[1]:+.3f}s common={int(common.sum())}/{len(xyz)}',
                    (8,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                if not cv2.imwrite(str(Path(output)/f'projection_{index:04d}_{variant_index:02d}.jpg'),overlay):
                    raise OSError('Cannot write projection overlay')
                overlays+=1
    metrics=[]
    for source,delta in variants:
        values=np.asarray(errors[(source,delta)])
        metrics.append(dict(pose_source=source,delta_s=delta,time_offset_s=float(cfg['time_offset']+delta),
            correspondences=len(values),invalid_correspondences=excluded[(source,delta)],
            rmse_px=float(np.sqrt(np.mean(values**2))) if len(values) else None,
            median_px=float(np.median(values)) if len(values) else None,
            p95_px=float(np.quantile(values,.95)) if len(values) else None))
    return dict(total_correspondences=total,common_correspondences=common_count,excluded_from_common=total-common_count,
                coverage_complete=common_count==total,metrics=metrics,views=rows,overlays_written=overlays,
                trajectories={source:traj.description for source,traj in trajectories.items()},
                limitations='Manual STATIC map-to-image correspondences. Metrics use a common set across ALL '
                'sources and offsets; inspect coverage/failures too. This does not estimate calibration, prove '
                'sensor synchronization, or measure vehicle recognition accuracy. No settings were changed.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True)
    parser.add_argument('--config',required=True)
    parser.add_argument('--samples',required=True,help='JSON list: timestamp, points_world, pixels, optional image')
    parser.add_argument('--offset-deltas',nargs='+',type=float,default=[-.05,0.,.05])
    parser.add_argument('--compare-keyframes',action='store_true')
    parser.add_argument('--max-overlays',type=int,default=20)
    parser.add_argument('--output',required=True,help='NEW report directory')
    args=parser.parse_args()
    if args.max_overlays<0:parser.error('max-overlays must be nonnegative')
    from pose_trajectory import load_trajectory
    from clean_map_vehicles import Trajectory
    cfg=yaml.safe_load(Path(args.config).read_text())
    T,contract=load_frame_contract(Path(args.map))
    if contract!='imu_v2':parser.error('Projection comparison requires IMU v2 map coordinates')
    selected=load_trajectory(args.map,cfg,Trajectory)
    trajectories={selected.source:selected}
    if args.compare_keyframes and selected.source!='keyframes':
        sparse=copy.deepcopy(cfg);sparse['pose_trajectory']=dict(source='keyframes')
        trajectories['keyframes']=load_trajectory(args.map,sparse,Trajectory)
    path=Path(args.samples);samples=json.loads(path.read_text())
    if not isinstance(samples,list):parser.error('samples must be a JSON list')
    for sample in samples:
        if sample.get('image'): sample['image']=str(path.parent/sample['image'])
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    report=evaluate(samples,trajectories,cfg,args.offset_deltas,T,output,args.max_overlays)
    report.update(config=cfg,samples_sha256=file_digest(path))
    with (output/'views.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(report['views'][0]));writer.writeheader();writer.writerows(report['views'])
    (output/'summary.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(f'Projection report: {output}; common={report["common_correspondences"]}/{report["total_correspondences"]}')


if __name__=='__main__':main()
