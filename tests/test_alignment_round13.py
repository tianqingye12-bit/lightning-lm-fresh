"""Dense pose selection, invalid-scan barriers and actual cleaning export integration."""
import csv
import json
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import clean_map_vehicles as vr
from clean_map_keyframes import run_keyframe_clean
from pose_trajectory import load_trajectory, POSE_FIELDS, DENSE_FILES, trajectory_options
from clean_map_runtime import ViewSelector
from configure_alignment_round13 import configure, write_profile
from semantic_evidence import timing_stats
from diagnose_projection import evaluate
from compact_saved_map import compact
from segmentation_quality import ScoredInstances


def fixture(directory, keyframes=None, scans=None, rejected=()):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    if keyframes is None:keyframes=[[0,1,0,0,0,0,0,0,1],[1,2,2,0,0,0,0,0,1]]
    if scans is None:scans=[[t,2*(t-1),2*(t-1)*(2-t),0,0,0,0,1] for t in np.linspace(1,2,11)]
    np.savetxt(directory/'keyframes.txt',keyframes)
    (directory/'map_frames.yaml').write_text('format_version: 2\nkeyframe_pose_frame: imu\nkeyframe_cloud_frame: imu\n'
        'T_imu_lidar: [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]\n')
    with (directory/'trajectory_anchors.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['id','timestamp',*POSE_FIELDS,*('raw_'+k for k in POSE_FIELDS)])
        writer.writeheader()
        for frame in keyframes:
            writer.writerow(dict(id=int(frame[0]),timestamp=frame[1],**dict(zip(POSE_FIELDS,frame[2:])),
                                 **dict(zip(('raw_'+k for k in POSE_FIELDS),frame[2:]))))
    rows=[]
    for i,scan in enumerate(scans):
        t=scan[0];usable=int(i not in rejected)
        rows.append(dict(scan_id=i,timestamp=t,status='accepted' if usable else 'registration_rejected',usable=usable,
            **dict(zip(POSE_FIELDS,scan[1:])),imu_gap=0,point_nonfinite=0,point_negative=0,clock_error_s=0,
            point_end_difference_s=0,correction_mode='anchor' if t in np.asarray(keyframes)[:,1] else 'interpolated_correction'))
    with (directory/'scan_trajectory.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (directory/'trajectory.yaml').write_text(yaml.safe_dump(dict(format_version=1,pose_frame='imu',
        coordinate_frame='exported_map',timestamp_reference='lidar_end',correction_method='interpolated_keyframe_correction',
        complete=True,rows=len(rows),usable_rows=len(rows)-len(rejected),recording_faults=0,keyframes=len(keyframes))))


def config():
    cfg=yaml.safe_load((ROOT/'config/m2dgr_clean_instances.yaml').read_text())
    cfg.update(width=32,height=32,K=[10.,0.,16.,0.,10.,16.,0.,0.,1.],distortion=[0.]*5,
        T_cam_lidar=np.eye(4).ravel().tolist(),_T_imu_lidar=np.eye(4).tolist(),
        image_period=.1,view_selection=dict(translation_m=0,rotation_deg=0,max_period_s=0),mask_dilate_px=0,
        semantic_evidence=dict(enabled=True,write_uncertain=True),segmentation_quality=dict(enabled=True),
        pose_trajectory=dict(source='dense',max_gap_s=.25),cache=dict(enabled=False),diagnostics=dict(max_overlays=0),
        storage=dict(compress_keyframes=False,write_global_clean=True,write_removed_points=True))
    cfg['instance_filter']['min_depth_pixels']=1
    return cfg


class AlignmentTests(unittest.TestCase):
    def test_dense_curve_and_correct_time_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture(tmp);traj=load_trajectory(tmp,config(),vr.Trajectory)
            np.testing.assert_allclose(traj.pose(1.5)[:3,3],[1,.5,0])
            self.assertEqual(traj.keyframes.pose(1.5)[1,3],0)
            np.testing.assert_allclose(traj.pose(1.55)[:3,3],[1.1,.49,0])
            info=timing_stats(traj,1.55,{'time_offset':0})
            self.assertEqual(info['pose_source'],'dense');self.assertAlmostEqual(info['pose_bracket_gap_s'],.1)
            self.assertEqual(info['keyframe_bracket_gap_s'],1)
            self.assertEqual(len(traj.t),2);self.assertEqual(traj.description['rows'],11)

    def test_rejected_scan_prevents_both_neighbor_interpolations(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture(tmp,rejected=(5,));traj=load_trajectory(tmp,config(),vr.Trajectory)
            for t in (1.45,1.5,1.55):self.assertIsNone(traj.pose(t))
            self.assertIsNotNone(traj.pose(1.35));self.assertIsNone(traj.pose(.9));self.assertIsNone(traj.pose(2.1))
            self.assertEqual(traj.query_stats['invalid_scan'],3)

    def test_gap_limit_exact_anchor_and_explicit_legacy_selection(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture(tmp,scans=[[1,0,0,0,0,0,0,1],[2,2,0,0,0,0,0,1]])
            traj=load_trajectory(tmp,config(),vr.Trajectory)
            self.assertIsNone(traj.pose(1.5));self.assertIsNotNone(traj.pose(1))
            cfg=config();cfg['pose_trajectory']['source']='keyframes'
            self.assertIsNotNone(load_trajectory(tmp,cfg,vr.Trajectory).pose(1.5))

    def test_auto_old_map_fallback_and_require_dense_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture(tmp)
            for name in DENSE_FILES:(Path(tmp)/name).unlink()
            cfg=config()
            with self.assertRaises(ValueError):load_trajectory(tmp,cfg,vr.Trajectory)
            cfg['pose_trajectory']['source']='auto'
            traj=load_trajectory(tmp,cfg,vr.Trajectory);self.assertEqual(traj.source,'keyframes')
            np.testing.assert_allclose(traj.pose(1.5),traj.keyframes.pose(1.5))

    def test_partial_incomplete_stale_and_truncated_exports_reject(self):
        for problem in ('partial','incomplete','stale','truncated','false_usable'):
            with self.subTest(problem=problem),tempfile.TemporaryDirectory() as tmp:
                fixture(tmp);d=Path(tmp)
                if problem=='partial':(d/'trajectory_anchors.csv').unlink()
                elif problem=='incomplete':
                    meta=yaml.safe_load((d/'trajectory.yaml').read_text());meta['complete']=False
                    (d/'trajectory.yaml').write_text(yaml.safe_dump(meta))
                elif problem=='stale':
                    data=np.loadtxt(d/'keyframes.txt');data[1,2]+=.1;np.savetxt(d/'keyframes.txt',data)
                elif problem=='truncated':
                    lines=(d/'scan_trajectory.csv').read_text().splitlines();(d/'scan_trajectory.csv').write_text('\n'.join(lines[:-1])+'\n')
                else:
                    text=(d/'scan_trajectory.csv').read_text().replace('accepted','registration_rejected',1)
                    (d/'scan_trajectory.csv').write_text(text)
                cfg=config();cfg['pose_trajectory']['source']='auto'
                with self.assertRaises(ValueError):load_trajectory(tmp,cfg,vr.Trajectory)

    def test_actual_bag_iteration_uses_dense_pose_and_skips_invalid_scan(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture(tmp,rejected=(5,));cfg=config();cfg['image_period']=.01
            extrinsic=np.eye(4);extrinsic[:3,3]=[.2,-.1,.05];cfg['_T_imu_lidar']=extrinsic.tolist()
            traj=load_trajectory(tmp,cfg,vr.Trajectory);selector=ViewSelector(cfg)
            connection=SimpleNamespace(topic=cfg['color_topic'],msgtype='sensor_msgs/msg/CompressedImage')
            _,encoded=cv2.imencode('.png',np.zeros((32,32,3),np.uint8))
            messages=[SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=n)),
                format='png',data=encoded.tobytes()) for n in (350000000,450000000,500000000,550000000,650000000)]
            class Reader:
                def __init__(self,*a,**kw):self.connections=[connection]
                def __enter__(self):return self
                def __exit__(self,*a):pass
                def messages(self,**kw):return iter((connection,0,m) for m in messages)
                def deserialize(self,raw,kind):return raw
            modules={'rosbags.highlevel':SimpleNamespace(AnyReader=Reader),
                     'rosbags.typesys':SimpleNamespace(Stores=SimpleNamespace(ROS2_HUMBLE=0),get_typestore=lambda _:None)}
            with patch.dict(sys.modules,modules):views=list(vr.iterate_views('unused',cfg,traj,selector))
            self.assertEqual(len(views),2);self.assertEqual(selector.stats['invalid_pose'],3)
            np.testing.assert_allclose(views[0][1],traj.pose(1.35)@extrinsic);self.assertEqual(views[0][2].shape,(32,32,3))

    def test_failure_samples_are_bounded_and_json_safe(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture(tmp);traj=load_trajectory(tmp,config(),vr.Trajectory)
            for _ in range(1000):self.assertIsNone(traj.pose(0))
            self.assertIsNone(traj.pose(float('nan')))
            self.assertEqual(traj.query_stats['outside_range'],1000)
            self.assertEqual(len(traj.failure_samples),41)
            self.assertIsNone(traj.failure_samples[-1]['timestamp'])
            json.dumps(traj.failure_samples,allow_nan=False)

    def test_configuration_preserves_previous_quality_and_calibration(self):
        source={'fasterlio':{'trajectory_recording':{'enabled':False},'registration_quality':{'balance':True},
            'imu_prediction':{'max_horizon_s':.5}},'system':{'map_export':{'save_global_pcd':False}}}
        result=configure(source,'mapping');self.assertEqual(result['system'],source['system'])
        self.assertEqual(result['fasterlio']['imu_prediction'],source['fasterlio']['imu_prediction'])
        cfg=config();result=configure(cfg,'semantic')
        for key in cfg:
            if key!='pose_trajectory':self.assertEqual(cfg[key],result[key])
        with tempfile.TemporaryDirectory() as tmp:
            a,b=Path(tmp)/'old.yaml',Path(tmp)/'new.yaml';a.write_text(yaml.safe_dump(cfg));before=a.read_bytes()
            write_profile(a,b,'semantic')
            with self.assertRaises(FileExistsError):write_profile(a,b,'semantic')
            self.assertEqual(a.read_bytes(),before)
        for value in (0,True,float('nan')):
            with self.assertRaises(ValueError):trajectory_options({'pose_trajectory':{'max_gap_s':value}})

    def exercise_clean(self,mode):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);frames=d/'keyframe_clouds';frames.mkdir()
            times=[.8,1.,1.2];poses=[[t,x,0,0,0,0,0,1] for t,x in zip(times,[0,.06,.12])]
            fixture(d,[[0,1,.06,0,0,0,0,0,1]],poses)
            header=['VERSION .7','FIELDS x y z intensity','SIZE 4 4 4 4','TYPE F F F F',
                'COUNT 1 1 1 1','WIDTH 3','HEIGHT 1','POINTS 3','DATA binary']
            cloud=np.array([(0,0,2,7),(1,0,2,9),(float('nan'),0,2,11)],
                dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4')])
            vr.write_pcd(frames/'0.pcd',header,cloud)
            world=cloud.copy();world['x']+=.06;vr.write_pcd(d/'global.pcd',header,world)
            (frames/'index.csv').write_text('id,timestamp,file\n0,1,0.pcd\n')
            originals={name:(d/name).read_bytes() for name in (*DENSE_FILES,'keyframes.txt','global.pcd','keyframe_clouds/0.pcd')}
            cfg=config();cfg['clean_mode']=mode;path=d/'cfg.yaml';path.write_text(yaml.safe_dump(cfg))
            tr=load_trajectory(d,cfg,vr.Trajectory)
            views=[(t,tr.pose(t),np.zeros((32,32,3),np.uint8),None) for t in times]
            masks=np.zeros((1,32,32),bool);masks[0,14:19,14:19]=True
            pred=ScoredInstances(masks,[2],[.9])
            with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=lambda _:SimpleNamespace(task='segment'))}), \
                 patch.object(vr,'iterate_views',return_value=views), \
                 patch.object(vr.VehicleVoter,'infer_mask',return_value=(pred,1)), \
                 patch.object(vr,'estimate_floor',return_value=np.full(2,-1.)):
                if mode=='keyframes':run_keyframe_clean(SimpleNamespace(map=tmp,bag='unused',device='cpu',no_cache=True,rviz=False),cfg,'imu_v2')
                else:
                    with patch.object(sys,'argv',['clean','--map',tmp,'--bag','unused','--config',str(path)]):vr.main()
            run=next((d/'clean_diagnostics').iterdir());output=run if mode=='keyframes' else d
            removed=vr.read_pcd(output/'global_vehicles.pcd')[1]
            self.assertEqual(len(removed),1);self.assertEqual(float(removed['intensity'][0]),7)
            self.assertAlmostEqual(float(removed['x'][0]),.06,places=6)
            report=json.loads((run/'summary.json').read_text());self.assertEqual(report['trajectory']['source'],'dense')
            rows=list(csv.DictReader((run/'views.csv').read_text().splitlines()))
            self.assertEqual({r['pose_source'] for r in rows},{'dense'})
            self.assertEqual({name:(d/name).read_bytes() for name in originals},originals)
            if mode=='keyframes':
                reloaded=load_trajectory(run,cfg,vr.Trajectory);self.assertEqual(reloaded.source,'dense')
                copied=d.parent/(d.name+'_copy')
                try:
                    compact(run,copied,'mapping',compress=False)
                    self.assertEqual(load_trajectory(copied,cfg,vr.Trajectory).source,'dense')
                finally:shutil.rmtree(copied,ignore_errors=True)

    def test_map_and_keyframe_clean_keep_dense_contract_and_originals(self):
        for mode in ('map','keyframes'):
            with self.subTest(mode=mode):self.exercise_clean(mode)

    def test_static_projection_known_offset_and_fair_common_coverage(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture(tmp);cfg=config();tr=load_trajectory(tmp,cfg,vr.Trajectory)
            xyz=np.array([[1,1,4],[2,1,4]])
            correct_time=1.45;T=tr.pose(correct_time)
            cam=(xyz-T[:3,3])@T[:3,:3]
            pixels=np.column_stack((10*cam[:,0]/cam[:,2]+16,10*cam[:,1]/cam[:,2]+16))
            sample=dict(timestamp=1.5,points_world=xyz.tolist(),pixels=pixels.tolist())
            report=evaluate([sample],{'dense':tr},cfg,[-.05,0,.05],np.eye(4))
            self.assertTrue(report['coverage_complete']);self.assertLess(report['metrics'][0]['rmse_px'],1e-10)
            self.assertGreater(report['metrics'][1]['rmse_px'],.1)
            report=evaluate([dict(sample,timestamp=1)],{'dense':tr},cfg,[-.05,0],np.eye(4))
            self.assertFalse(report['coverage_complete']);self.assertEqual(report['common_correspondences'],0)
            self.assertTrue(all(row['rmse_px'] is None for row in report['metrics']))


if __name__=='__main__':unittest.main()
