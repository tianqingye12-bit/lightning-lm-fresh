"""Regression coverage for the supplied M2DGR timing failures and new-map contract."""
import copy
import csv
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from configure_scan_timing import configure,write_profile,HANDSFREE_T_IMU_LIDAR
from audit_lidar_timing import time_values,interval
from audit_scan_trajectory import audit
from pose_trajectory import PoseTrajectory
from restore_map_cleanup import prepare


def fixture(directory):
    d=Path(directory);d.mkdir(parents=True,exist_ok=True)
    (d/'map_frames.yaml').write_text('format_version: 2\nkeyframe_pose_frame: imu\nkeyframe_cloud_frame: imu\n'
        'T_imu_lidar: [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]\n')
    np.savetxt(d/'keyframes.txt',[[0,10,0,0,0,0,0,0,1],[1,10.2,.2,0,0,0,0,0,1]],fmt='%.18g')
    fields=('tx','ty','tz','qx','qy','qz','qw')
    rows=[]
    for i,t in enumerate((10.,10.1,10.2)):
        rows.append(dict(scan_id=i,begin_time=t-.1,timestamp=t,state_timestamp=t,status='accepted',usable=1,
            tx=t-10,ty=0,tz=0,qx=0,qy=0,qz=0,qw=1,imu_gap=0,point_count=100,
            point_nonfinite=0,point_negative=0,point_min_s=0,point_max_s=.1,scan_duration_s=.1,
            clock_error_s=0,point_end_difference_s=0,correction_mode='anchor' if i!=1 else 'interpolated_correction'))
    with (d/'scan_trajectory.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with (d/'trajectory_anchors.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['id','timestamp',*fields,*('raw_'+v for v in fields)])
        writer.writeheader()
        for i,t in enumerate((10.,10.2)):
            values=[t-10,0,0,0,0,0,1]
            writer.writerow(dict(id=i,timestamp=t,**dict(zip(fields,values)),
                **dict(zip(('raw_'+v for v in fields),values))))
    meta=dict(format_version=2,pose_frame='imu',coordinate_frame='exported_map',timestamp_reference='lidar_end',
        correction_method='interpolated_keyframe_correction',complete=True,rows=3,usable_rows=3,
        recording_faults=0,keyframes=2,point_time_reference='scan_begin',scan_timing_policy='relative_header',
        point_end_tolerance_s=.002)
    (d/'trajectory.yaml').write_text(yaml.safe_dump(meta))
    return d


class ScanTimingTests(unittest.TestCase):
    def test_profiles_preserve_map_detail_and_model_and_require_new_dense_map(self):
        original=dict(common=dict(dataset='m2dgr',imu_topic='/handsfree/imu'),fasterlio=dict(
            lidar_type=2,time_scale=1000.,extrinsic_T=[0.,0.,0.],extrinsic_R=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
            map_detail={'keyframe_resolution':.05}),system={'map_export':{'save_global_pcd':False}})
        before=copy.deepcopy(original)
        result=configure(original,'mapping')
        self.assertEqual(original,before)
        self.assertEqual(result['fasterlio']['map_detail'],original['fasterlio']['map_detail'])
        self.assertEqual(result['fasterlio']['extrinsic_T'],[0.,0.,0.])
        self.assertTrue(result['system']['export_keyframe_clouds'])
        calibrated=configure(original,'mapping',True)
        self.assertEqual(calibrated['fasterlio']['extrinsic_T'],HANDSFREE_T_IMU_LIDAR)
        old=dict(yolo_model='model-seg.pt',time_offset=0.,vehicle_instance_tracks={'enabled':True})
        semantic=configure(old,'semantic')
        for k in old:self.assertEqual(old[k],semantic[k])
        self.assertTrue(semantic['pose_trajectory']['require_normalized_timing'])
        wrong=copy.deepcopy(original);wrong['common']['imu_topic']='/camera/imu'
        with self.assertRaises(ValueError):configure(wrong,'mapping',True)
        wrong=copy.deepcopy(original);wrong['fasterlio']['time_scale']=1
        with self.assertRaises(ValueError):configure(wrong,'mapping')

    def test_profile_generation_never_overwrites_input_or_existing_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'source.yaml';dest=Path(tmp)/'new.yaml'
            source.write_text('yolo_model: model-seg.pt\n')
            before=source.read_bytes()
            write_profile(source,dest,'semantic')
            with self.assertRaises(FileExistsError):write_profile(source,dest,'semantic')
            self.assertEqual(source.read_bytes(),before)

    def test_pointcloud_time_reader_respects_rows_padding_and_big_endian(self):
        for endian in ('<','>'):
            payload=bytearray(48)
            for offset,value in ((4,-.0995),(20,.0006428),(28,-.050),(44,0.)):
                payload[offset:offset+4]=np.array([value],dtype=endian+'f4').tobytes()
            # 2 rows; each point has x plus time, then 8 bytes of row padding.
            msg=SimpleNamespace(fields=[SimpleNamespace(name='time',datatype=7,count=1,offset=4)],
                width=2,height=2,point_step=8,row_step=24,data=payload,is_bigendian=endian=='>')
            # Actual time positions are 4,12,28,36.
            for offset,value in ((12,.0006428),(36,0.)):
                payload[offset:offset+4]=np.array([value],dtype=endian+'f4').tobytes()
            np.testing.assert_allclose(time_values(msg),[-.0995,.0006428,-.05,0.],atol=1e-8)
            msg.fields[0].datatype=8
            with self.assertRaises(ValueError):time_values(msg)

    def test_signed_raw_offsets_preserve_absolute_sample_times(self):
        raw=np.array([-.09949707794,.000642816,-.020])
        limits=dict(min_duration_s=.05,max_duration_s=.15,max_header_offset_s=.2)
        result=interval(raw,1628081061.,1000.,limits)
        self.assertAlmostEqual(result['duration_s'],.10013989394)
        normalized=raw-raw.min()
        np.testing.assert_allclose(1628081061.+raw,result['begin']+normalized,atol=3e-7,rtol=0)
        for values in ([0,.0006428],[0,float('nan')],[1.,1.1]):
            with self.assertRaises(ValueError):interval(values,10.,1000.,limits)

    def test_valid_new_trajectory_and_prepare_copy_full_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=fixture(Path(tmp)/'source')
            # Existing global copy path does not read the PCD body.
            (source/'global.pcd').write_bytes(b'original map payload')
            report=audit(source)
            self.assertTrue(report['valid'])
            original={p.name:p.read_bytes() for p in source.iterdir() if p.is_file()}
            out=prepare(source,Path(tmp)/'prepared',require_dense=True)
            for name in ('scan_trajectory.csv','trajectory.yaml','trajectory_anchors.csv'):
                self.assertEqual((out/name).read_bytes(),original[name])
            self.assertTrue(audit(out)['valid'])
            self.assertEqual(original,{p.name:p.read_bytes() for p in source.iterdir() if p.is_file()})
            self.assertTrue(json.loads((out/'PREPARED_MAP.json').read_text())['normalized_dense_verified'])

    def test_old_map_cannot_be_relabelled_as_new_or_forced_usable(self):
        for problem in ('legacy','duration','negative','reference','missing','prefix'):
            with self.subTest(problem=problem),tempfile.TemporaryDirectory() as tmp:
                source=fixture(Path(tmp)/'source')
                meta=yaml.safe_load((source/'trajectory.yaml').read_text())
                with (source/'scan_trajectory.csv').open(newline='') as stream:
                    rows=list(csv.DictReader(stream))
                if problem=='legacy':meta['format_version']=1
                elif problem=='duration':
                    rows[1]['scan_duration_s']=.0006428;rows[1]['begin_time']=10.1-.0006428
                    rows[1]['point_max_s']=.098565;rows[1]['point_end_difference_s']=.0979222
                elif problem=='negative':rows[1]['point_negative']=1
                elif problem=='reference':meta['point_time_reference']='header'
                elif problem=='missing':(source/'trajectory_anchors.csv').unlink()
                else:rows[1]['point_min_s']=.01
                (source/'trajectory.yaml').write_text(yaml.safe_dump(meta))
                with (source/'scan_trajectory.csv').open('w',newline='') as stream:
                    writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
                self.assertFalse(audit(source)['valid'])
                with self.assertRaises(ValueError):prepare(source,Path(tmp)/'refused',require_dense=True)
                self.assertFalse((Path(tmp)/'refused').exists())

    def test_malformed_valid_flag_is_checked_even_for_legacy_version(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=fixture(tmp);meta=yaml.safe_load((source/'trajectory.yaml').read_text());meta['format_version']=1
            (source/'trajectory.yaml').write_text(yaml.safe_dump(meta))
            with (source/'scan_trajectory.csv').open(newline='') as stream:rows=list(csv.DictReader(stream))
            rows[1]['point_end_difference_s']=.0979
            with (source/'scan_trajectory.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
            result=audit(source,require_normalized=False)
            self.assertFalse(result['valid']);self.assertIn('point timing',result['error'])


if __name__=='__main__':unittest.main()
