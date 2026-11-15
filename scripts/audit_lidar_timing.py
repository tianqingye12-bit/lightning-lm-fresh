#!/usr/bin/env python3
"""Read raw Velodyne time fields without SLAM/YOLO; test an explicit relative-header hypothesis."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
import yaml


def time_values(msg):
    fields=[f for f in msg.fields if f.name=='time']
    if len(fields)!=1 or fields[0].datatype!=7 or fields[0].count!=1:
        raise ValueError('Expected one scalar FLOAT32 PointCloud2 time field')
    field=fields[0]
    width,height,step,row=map(int,(msg.width,msg.height,msg.point_step,msg.row_step))
    if (min(width,height,step)<=0 or width*height>2000000 or
            row<width*step or field.offset<0 or field.offset+4>step):
        raise ValueError('Invalid PointCloud2 dimensions, stride or time offset')
    payload=memoryview(msg.data)
    if len(payload)!=height*row:
        raise ValueError('PointCloud2 payload does not match row_step*height')
    return np.ndarray((height,width),dtype='>f4' if msg.is_bigendian else '<f4',
                      buffer=payload,offset=field.offset,strides=(row,step)).astype(float).ravel()


def interval(values,header,time_scale,limits):
    values=np.asarray(values,dtype=float)*time_scale/1000.
    if len(values)<2 or not np.isfinite(values).all() or not np.isfinite(header):
        raise ValueError('Empty/nonfinite raw scan timing')
    low,high=float(values.min()),float(values.max())
    duration=high-low
    if not limits['min_duration_s']<=duration<=limits['max_duration_s']:
        raise ValueError('scan_duration_outside_limits')
    if max(abs(low),abs(high))>limits['max_header_offset_s']:
        raise ValueError('offset_outside_header_window')
    return dict(header=header,begin=header+low,end=header+high,minimum_s=low,
                maximum_s=high,duration_s=duration,negative_points=int((values<0).sum()))


def audit(bag,cfg,count):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores,get_typestore
    lio=cfg['fasterlio'];limits=lio['scan_timing']
    if limits.get('reference')!='relative_header' or lio.get('lidar_type')!=2:
        raise ValueError('Use the newly generated M2DGR relative_header mapping profile')
    results=[];errors=Counter();scans=0;previous_end=None;overlaps=[];samples=[]
    with AnyReader([Path(bag)],default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        connections=[c for c in reader.connections if c.topic==cfg['common']['lidar_topic']]
        if not connections:raise ValueError('Configured LiDAR topic is absent from bag')
        for connection,_,raw in reader.messages(connections=connections):
            msg=reader.deserialize(raw,connection.msgtype)
            header=float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
            scans+=1
            try:
                row=interval(time_values(msg),header,float(lio['time_scale']),limits)
                if previous_end is not None:
                    if row['end']<=previous_end:raise ValueError('non_increasing_scan_end')
                    overlaps.append(previous_end-row['begin'])
                previous_end=row['end'];results.append(row)
            except ValueError as exc:
                errors[str(exc)]+=1
                if len(samples)<20:samples.append(dict(scan=scans,header=header,error=str(exc)))
            if scans>=count:break
    def quantiles(values):
        return dict(zip(('min','median','p99','max'),map(float,np.quantile(values,[0,.5,.99,1])))) if len(values) else None
    return dict(scans=scans,valid_scans=len(results),errors=dict(errors),error_samples=samples,
        duration_s=quantiles([r['duration_s'] for r in results]),
        minimum_offset_s=quantiles([r['minimum_s'] for r in results]),
        maximum_offset_s=quantiles([r['maximum_s'] for r in results]),
        prefix_overlap_s=quantiles(overlaps),
        scans_with_negative_offsets=sum(r['negative_points']>0 for r in results),
        assumed_reference='raw seconds relative to ROS header, converted by time_scale',
        limitation='Checks field layout and internal timing consistency; does not establish physical synchronization or calibrate a time offset.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',required=True);p.add_argument('--config',required=True)
    p.add_argument('--scans',type=int,default=300);p.add_argument('--output',required=True)
    args=p.parse_args()
    if not 2<=args.scans<=20000:p.error('--scans must be 2..20000')
    try:
        result=audit(args.bag,yaml.safe_load(Path(args.config).read_text()),args.scans)
        with Path(args.output).open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    except (OSError,ValueError,KeyError,ImportError) as exc:p.exit(1,str(exc)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['scans'] and not result['errors'] else 1)


if __name__=='__main__':main()
