"""Evidence gates and real cleaning/export integration, no ROS/model downloads."""
import csv
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from semantic_evidence import PositiveBaseline, decide, evidence_options, timing_stats
from configure_semantic_evidence import configure
import clean_map_vehicles as vr
from clean_map_keyframes import run_keyframe_clean


class EvidenceTests(unittest.TestCase):
    def cfg(self):
        cfg = yaml.safe_load((ROOT/'config/m2dgr_clean_instances.yaml').read_text())
        cfg.update(width=32, height=32, K=[10.,0.,16.,0.,10.,16.,0.,0.,1.],
                   distortion=[0.]*5, T_cam_lidar=np.eye(4).ravel().tolist(),
                   mask_dilate_px=0, cache={'enabled':False}, diagnostics={'max_overlays':0},
                   semantic_evidence={'enabled':True, 'write_uncertain':True},
                   storage={'compress_keyframes':False,'write_global_clean':True,'write_removed_points':True})
        cfg['instance_filter']['min_depth_pixels'] = 1
        return cfg

    def test_coverage_baseline_veto_floor_and_partition(self):
        cfg = self.cfg(); baseline = PositiveBaseline(6,cfg)
        baseline.update(np.arange(6), [0,0,0]); baseline.update(np.arange(6), [.1,0,0])
        baseline.maximum[1] = 0
        # 0: insufficient classified coverage, 1: static camera, 2: remove,
        # 3: explicit background, 4: floor protected, 5: never seen.
        _,remove,unknown,c = decide(np.array([20,3,3,6,3,0]),np.array([3,3,3,3,3,0]),
            np.array([0,0,0,3,0,0]),cfg,baseline,np.array([0,0,0,0,1,0],bool))
        np.testing.assert_array_equal(remove,[0,0,1,0,0,0])
        np.testing.assert_array_equal(unknown,[1,1,0,0,0,1])
        self.assertEqual(c['blocked_coverage'],1); self.assertEqual(c['blocked_baseline'],1)
        self.assertEqual(sum(c[k] for k in ('removed','protected','background','uncertain')),c['total'])

    def test_baseline_counts_only_positive_visible_observations(self):
        cfg=self.cfg(); b=PositiveBaseline(2,cfg)
        b.update(np.array([0]),[10000000.,0,0]); b.update(np.array([1]),[0,0,0])
        b.update(np.array([0]),[10000000.06,0,0])
        self.assertGreater(b.maximum[0],.05); self.assertEqual(b.maximum[1],0)
        # Repeated same center cannot manufacture a baseline.
        for _ in range(10): b.update(np.array([1]),[0,0,0])
        self.assertEqual(b.maximum[1],0)

    def test_real_projection_baseline_ignores_occlusion_and_background(self):
        cfg=self.cfg()
        xyz=np.array([[0.,0.,2.],[.5,0.,2.],[0.,0.,5.]])
        voter=vr.VehicleVoter(cfg,xyz,None,'cpu')
        masks=np.zeros((1,32,32),bool);masks[0,16,16]=True
        voter.vehicle_mask=lambda _: (masks,1)
        image=np.zeros((32,32,3),np.uint8)
        voter.add_view(np.eye(4),image,None)
        pose=np.eye(4);pose[0,3]=.06
        voter.add_view(pose,image,None)
        self.assertGreater(voter.baseline.maximum[0],.05)
        self.assertEqual(voter.baseline.maximum[1],0)
        self.assertEqual(voter.baseline.maximum[2],0)
        self.assertTrue(np.isnan(voter.baseline.first[1:]).all())
        stats=voter.last_stats
        self.assertEqual(stats['vehicle_votes']+stats['background_votes']+stats['unknown_votes'],stats['visible'])

    def test_disabled_is_legacy_and_allocates_no_history(self):
        cfg=self.cfg();cfg.pop('semantic_evidence');b=PositiveBaseline(2,cfg)
        self.assertIsNone(b.first);self.assertIsNone(b.maximum)
        _,remove,_,_=decide(np.array([20,3]),np.array([3,3]),np.array([0,0]),cfg,b)
        self.assertTrue(remove.all())

    def test_invalid_configuration_and_calibration_preservation(self):
        for bad in ({'enabled':1},{'min_classified_ratio':0},{'min_positive_baseline_m':float('nan')},
                    {'min_positive_baseline_m':True},{'bad_key':1},{'write_uncertain':'false'}):
            with self.subTest(bad=bad), self.assertRaises(ValueError): evidence_options({'semantic_evidence':bad})
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'base.yaml';dst=Path(tmp)/'new.yaml';cfg=self.cfg()
            cfg.pop('semantic_evidence');src.write_text(yaml.safe_dump(cfg));old=src.read_bytes()
            got=configure(src,dst)
            self.assertEqual({k:v for k,v in got.items() if k!='semantic_evidence'},cfg)
            self.assertFalse(got['semantic_evidence']['write_uncertain'])
            with self.assertRaises(FileExistsError):configure(src,dst)
            self.assertEqual(src.read_bytes(),old)

    def test_timing_reports_interpolation_gap_not_false_sync_accuracy(self):
        t=SimpleNamespace(t=np.array([1.,3.]))
        got=timing_stats(t,1.5,{'time_offset':.5})
        self.assertEqual(got,dict(aligned_timestamp=2.,nearest_keyframe_dt_s=1.,pose_bracket_gap_s=2.))
        self.assertEqual(timing_stats(t,1.,{'time_offset':0})['pose_bracket_gap_s'],0)

    def exercise_keyframes(self, moving, export=True, floor_unknown=False):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);frames=d/'keyframe_clouds';frames.mkdir()
            header=['VERSION .7','FIELDS x y z intensity','SIZE 4 4 4 4','TYPE F F F F',
                    'COUNT 1 1 1 1','WIDTH 3','HEIGHT 1','POINTS 3','DATA binary']
            cloud=np.array([(0,0,2,7),(.5,0,2,9),(float('nan'),0,2,11)],
                dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4')])
            vr.write_pcd(frames/'0.pcd',header,cloud);original=(frames/'0.pcd').read_bytes()
            np.savetxt(d/'keyframes.txt',[[0,1,0,0,0,0,0,0,1]])
            (frames/'index.csv').write_text('id,timestamp,file\n0,1,0.pcd\n')
            (d/'map_frames.yaml').write_text('format_version: 2\n')
            cfg=self.cfg();cfg['semantic_evidence']['write_uncertain']=export
            pose=np.eye(4);other=np.eye(4);other[0,3]=.06 if moving else 0
            views=[(t,p,np.zeros((32,32,3),np.uint8),None) for t,p in zip([.8,1.,1.2],[pose,other,other])]
            masks=np.zeros((1,32,32),bool);masks[0,16,16]=True
            model=SimpleNamespace(task='segment')
            args=SimpleNamespace(map=tmp,bag='unused',device='cpu',no_cache=True,rviz=False)
            floor_kw={'side_effect':RuntimeError('floor')} if floor_unknown else {'return_value':np.array([-1.,-1.])}
            with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=lambda _:model)}), \
                 patch.object(vr,'iterate_views',return_value=views), \
                 patch.object(vr.VehicleVoter,'infer_mask',return_value=(masks,1)), \
                 patch.object(vr,'estimate_floor',**floor_kw):
                run_keyframe_clean(args,cfg,'imu_v2')
            run=next((d/'clean_diagnostics').glob('keyframes_*'))
            report=json.loads((run/'summary.json').read_text())
            removed=vr.read_pcd(run/'global_vehicles.pcd')[1]
            self.assertEqual(len(removed),int(moving and not floor_unknown))
            clean=vr.read_pcd(run/'keyframe_clouds/0.pcd')[1]
            self.assertEqual(len(clean)+len(removed),len(cloud)) # invalid points retained in frames
            self.assertEqual((frames/'0.pcd').read_bytes(),original)
            self.assertEqual(report['invalid_points'],1)
            self.assertEqual(report['evidence']['total'],2)
            self.assertEqual(report['uncertain_cloud_written'],export)
            self.assertEqual((run/'global_uncertain.pcd').exists(),export)
            if export and not moving:
                unknown=vr.read_pcd(run/'global_uncertain.pcd')[1]
                self.assertEqual(len(unknown),1)
                self.assertEqual(unknown['intensity'][0],7)
            rows=list(csv.DictReader((run/'views.csv').read_text().splitlines()))
            self.assertEqual(len(rows),3);self.assertIn('nearest_keyframe_dt_s',rows[0])
            self.assertEqual(len(list(csv.DictReader((run/'frames.csv').read_text().splitlines()))),1)
            return report

    def test_stationary_camera_keeps_target_as_uncertain(self):
        report=self.exercise_keyframes(False)
        self.assertEqual(report['evidence']['blocked_baseline'],1)

    def test_moving_camera_removes_only_supported_target(self):
        report=self.exercise_keyframes(True)
        self.assertEqual(report['evidence']['removed'],1)

    def test_compact_output_does_not_force_uncertain_export(self):
        self.exercise_keyframes(False,export=False)

    def test_map_mode_disables_unseen_cluster_growth_and_exports_uncertainty(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp)
            header=['VERSION .7','FIELDS x y z intensity','SIZE 4 4 4 4','TYPE F F F F',
                    'COUNT 1 1 1 1','WIDTH 4','HEIGHT 1','POINTS 4','DATA binary']
            cloud=np.array([(0,0,2,7),(.5,0,2,9),(0,0,5,10),(float('nan'),0,2,11)],
                dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4')])
            vr.write_pcd(d/'global.pcd',header,cloud)
            original=(d/'global.pcd').read_bytes()
            np.savetxt(d/'keyframes.txt',[[0,1,0,0,0,0,0,0,1]])
            cfg=self.cfg();cfg['clean_mode']='map';cfg['diagnostics']={'max_overlays':1}
            path=d/'config.yaml';path.write_text(yaml.safe_dump(cfg))
            pose=np.eye(4);other=np.eye(4);other[0,3]=.06
            views=[(t,p,np.zeros((32,32,3),np.uint8),None) for t,p in zip([.8,1.,1.2],[pose,other,other])]
            masks=np.zeros((1,32,32),bool);masks[0,16,16]=True
            argv=['clean','--map',tmp,'--bag','unused','--config',str(path)]
            with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=lambda _:SimpleNamespace(task='segment'))}), \
                 patch.object(sys,'argv',argv), patch.object(vr,'iterate_views',return_value=views), \
                 patch.object(vr.VehicleVoter,'infer_mask',return_value=(masks,1)), \
                 patch.object(vr,'estimate_floor',return_value=np.full(3,-1.)), \
                 patch.object(vr,'components',side_effect=AssertionError('must not grow')):
                vr.main()
            run=next((d/'clean_diagnostics').glob('run_*'))
            report=json.loads((run/'summary.json').read_text())
            self.assertFalse(report['cluster_growth_enabled'])
            self.assertEqual(len(vr.read_pcd(d/'global_vehicles.pcd')[1]),1)
            self.assertEqual(len(vr.read_pcd(run/'global_uncertain.pcd')[1]),1)
            self.assertEqual(len(vr.read_pcd(d/'global_clean.pcd')[1]),3)
            self.assertEqual((d/'global.pcd').read_bytes(),original)
            self.assertEqual(report['evidence']['unobserved'],1)
            self.assertTrue((run/'overlay_000.jpg').is_file())

    def test_floor_unknown_preserves_candidates(self):
        report=self.exercise_keyframes(True,floor_unknown=True)
        self.assertEqual(report['floor_unknown_frames'],1)
        self.assertEqual(report['evidence']['removed'],0)


if __name__=='__main__':unittest.main()
