#!/usr/bin/env python3
"""Create NEW M2DGR timing profiles; original YAML and old maps are never changed."""
import argparse
import copy
from pathlib import Path
import yaml

# Official calibration_results.txt gives T_lidar_handsfree; this is its inverse,
# T_imu_lidar, as consumed by LaserMapping/GetTImuLidar.
HANDSFREE_T_IMU_LIDAR = [.27255, -.00053, .17954]
IDENTITY_ROTATION = [1.,0.,0.,0.,1.,0.,0.,0.,1.]


def configure(cfg, mode, handsfree_extrinsics=False):
    if not isinstance(cfg,dict):
        raise ValueError('Input must be a YAML mapping')
    out=copy.deepcopy(cfg)
    if mode=='mapping':
        common,lio=out.get('common',{}),out.get('fasterlio',{})
        if (common.get('dataset')!='m2dgr' or lio.get('lidar_type')!=2 or
                lio.get('time_scale')!=1000.):
            raise ValueError('This profile requires M2DGR Velodyne, lidar_type=2 and seconds->ms time_scale=1000')
        lio['scan_timing']=dict(reference='relative_header',min_duration_s=.05,
                                max_duration_s=.15,max_header_offset_s=.2)
        lio['trajectory_recording']={**lio.get('trajectory_recording',{}), 'enabled':True}
        lio['trajectory_recording'].setdefault('max_records',200000)
        if handsfree_extrinsics:
            if common.get('imu_topic')!='/handsfree/imu':
                raise ValueError('--handsfree-extrinsics requires /handsfree/imu')
            lio['extrinsic_T']=HANDSFREE_T_IMU_LIDAR.copy()
            lio['extrinsic_R']=IDENTITY_ROTATION.copy()
            lio['extrinsic_est_en']=False
        out['fasterlio']=lio
        if not isinstance(out.get('system'),dict):
            raise ValueError('Missing system mapping')
        out['system']['export_keyframe_clouds']=True
    elif mode=='semantic':
        if handsfree_extrinsics:
            raise ValueError('IMU calibration is applied when mapping, not to an existing semantic map')
        out['pose_trajectory']=dict(source='dense',max_gap_s=.2,require_normalized_timing=True)
    else:
        raise ValueError('Mode must be mapping or semantic')
    return out


def write_profile(source,output,mode,handsfree_extrinsics=False):
    result=configure(yaml.safe_load(Path(source).read_text()),mode,handsfree_extrinsics)
    with Path(output).open('x') as stream:
        yaml.safe_dump(result,stream,allow_unicode=True,sort_keys=False)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['mapping','semantic'])
    parser.add_argument('--input',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--handsfree-extrinsics',action='store_true')
    args=parser.parse_args()
    try: write_profile(args.input,args.output,args.mode,args.handsfree_extrinsics)
    except (OSError,ValueError,TypeError,yaml.YAMLError) as exc:parser.exit(1,str(exc)+'\n')
    print(f'Created {args.output}; use only with a NEW map')


if __name__=='__main__':main()
