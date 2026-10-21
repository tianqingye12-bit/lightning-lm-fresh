"""Shared image votes, conservative conflicts, and actual two-pass PCD rebuilding."""
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

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from cross_keyframe_semantics import SemanticVoxelEvidence,cross_frame_options
from configure_cross_keyframe_round14 import configure,write_profile
from semantic_evidence import decide
from test_alignment_round13 import config,fixture
import clean_map_vehicles as vr
from clean_map_keyframes import run_keyframe_clean
from segmentation_quality import ScoredInstances


def add(store,t,positive=True,background=False,camera=(0,0,0),score=.9,point=(.025,.025,2.025),copies=1):
    store.begin_image(t,camera)
    for _ in range(copies):
        store.add(np.array([point]),dict(indices=np.array([0]),positive=np.array([positive]),
            background=np.array([background]),scores=np.array([score])))
    store.end_image()


class CrossFrameTests(unittest.TestCase):
    def test_frame_and_pixel_duplicates_are_one_vote_per_image_and_timestamp(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=SemanticVoxelEvidence(tmp,.1)
            try:
                for t,x in [(1,0),(2,.06),(3,.12)]:add(store,t,camera=(x,0,0),copies=20)
                add(store,3,camera=(.12,0,0),copies=20)
                voter=store.lookup([[.03,.03,2.03]])
                self.assertEqual(voter.visible.tolist(),[3]);self.assertEqual(voter.vehicle.tolist(),[3])
                np.testing.assert_allclose(voter.score_sum,[2.7]);np.testing.assert_allclose(voter.baseline.maximum,[.12])
                removed=decide(voter.visible,voter.vehicle,voter.background,config(),voter.baseline,
                    positive_score=voter.score_sum)[1]
                self.assertTrue(removed[0])
                report=store.summary();self.assertEqual(report['voxel_image_pairs'],3)
                self.assertEqual(report['duplicate_images'],1)
            finally:store.close()
            self.assertFalse(any(Path(tmp).glob('.semantic_votes_*')))

    def test_same_image_vehicle_background_conflict_abstains(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=SemanticVoxelEvidence(tmp,.1)
            try:
                store.begin_image(1,[0,0,0])
                for p,b in [(True,False),(False,True)]:
                    store.add([[.025,.025,2.025]],dict(indices=[0],positive=[p],background=[b],scores=[.9]))
                store.end_image();voter=store.lookup([[.03,.03,2.03]])
                self.assertEqual(voter.visible.tolist(),[1]);self.assertEqual(voter.vehicle.tolist(),[0])
                self.assertEqual(voter.background.tolist(),[0]);self.assertEqual(voter.score_sum.tolist(),[0])
                self.assertEqual(store.summary()['conflicting_pairs'],1)
            finally:store.close()

    def test_many_points_in_voxel_use_max_confidence_and_one_view(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=SemanticVoxelEvidence(tmp,.1)
            try:
                store.begin_image(1,[0,0,0])
                store.add([[.01,.01,2.01],[.02,.02,2.02]],dict(indices=[0,1],
                    positive=[True,True],background=[False,False],scores=[.5,.9]))
                store.end_image();voter=store.lookup([[.03,.03,2.03]])
                self.assertEqual(voter.vehicle.tolist(),[1])
                np.testing.assert_allclose(voter.score_sum,[.9])
            finally:store.close()

    def test_background_veto_baseline_confidence_and_floor_are_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=SemanticVoxelEvidence(tmp,.1)
            try:
                for t in (1,2,3):add(store,t,camera=(t*.06,0,0))
                voter=store.lookup([[.025,.025,2.025]])
                def removal(cfg,protected=None):
                    return decide(voter.visible,voter.vehicle,voter.background,cfg,voter.baseline,
                        protected=protected,positive_score=voter.score_sum)[1][0]
                self.assertTrue(removal(config()));self.assertFalse(removal(config(),np.array([True])))
                cfg=config();cfg['segmentation_quality']['min_mean_confidence']=.95
                self.assertFalse(removal(cfg))
                cfg=config();cfg['semantic_evidence']['min_positive_baseline_m']=1.
                self.assertFalse(removal(cfg))
                for t in (4,5,6):add(store,t,positive=False,background=True)
                voter=store.lookup([[.025,.025,2.025]])
                self.assertFalse(removal(config()))
            finally:store.close()

    def test_unseen_neighbor_voxels_and_negative_coordinates_do_not_inherit_votes(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=SemanticVoxelEvidence(tmp,.1)
            try:
                for t in (1,2,3):add(store,t,camera=(t*.06,0,0),point=(-.025,.025,2.025))
                voter=store.lookup([[-.01,.03,2.03],[.01,.03,2.03],[100,100,100]])
                self.assertEqual(voter.vehicle.tolist(),[3,0,0])
                self.assertEqual(voter.visible.tolist(),[3,0,0])
                mask=decide(voter.visible,voter.vehicle,voter.background,config(),voter.baseline,
                    positive_score=voter.score_sum)[1]
                self.assertEqual(mask.tolist(),[True,False,False])
            finally:store.close()

    def test_configuration_validation_and_no_overwrite(self):
        cfg=config();result=configure(cfg)
        for name in cfg:
            if name not in ('clean_mode','cross_keyframe_semantics'):self.assertEqual(cfg[name],result[name])
        self.assertNotIn('cross_keyframe_semantics',cfg)
        for value in (0,True,float('nan'),.3):
            with self.assertRaises(ValueError):cross_frame_options({'cross_keyframe_semantics':{'voxel_size':value}})
        with self.assertRaises(ValueError):cross_frame_options({'cross_keyframe_semantics':{'other':1}})
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'a.yaml';output=Path(tmp)/'b.yaml';source.write_text(yaml.safe_dump(cfg))
            write_profile(source,output)
            with self.assertRaises(FileExistsError):write_profile(source,output)

    def test_image_order_and_nonfinite_evidence_fail_without_silent_reuse(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=SemanticVoxelEvidence(tmp,.1)
            try:
                add(store,2)
                with self.assertRaises(ValueError):store.begin_image(1,[0,0,0])
                store.begin_image(3,[0,0,0])
                with self.assertRaises(ValueError):store.add([[np.nan,0,0]],
                    dict(indices=[0],positive=[True],background=[False],scores=[.9]))
                with self.assertRaises(ValueError):store.lookup([[0,0,0]])
            finally:store.close()

    def test_batched_lookup_preserves_point_order_and_duplicate_mapping(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=SemanticVoxelEvidence(tmp,.1)
            try:
                count=10003
                world=np.column_stack((np.arange(count)*.1+.025,np.full(count,.025),np.full(count,2.025)))
                scores=np.linspace(.5,.9,count)
                store.begin_image(1,[0,0,0])
                store.add(world,dict(indices=np.arange(count),positive=np.ones(count,bool),
                    background=np.zeros(count,bool),scores=scores))
                store.end_image()
                query=np.concatenate([world[::-1],world[[0,-1]]])
                result=store.lookup(query)
                np.testing.assert_allclose(result.score_sum,np.concatenate([scores[::-1],scores[[0,-1]]]))
                self.assertTrue(np.all(result.visible==1))
            finally:store.close()

    def exercise_pipeline(self,enabled,compact=False,overlap=False,fail_active=False):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'input';source.mkdir();frames=source/'keyframe_clouds';frames.mkdir()
            keyframes=[[i,1+i*.1 if overlap else float(i+1),i*.06,0,0,0,0,0,1] for i in range(4)]
            scans=[[row[1],row[2],0,0,0,0,0,1] for row in keyframes]
            fixture(source,keyframes,scans)
            header=['VERSION .7','FIELDS x y z intensity','SIZE 4 4 4 4','TYPE F F F F',
                'COUNT 1 1 1 1','WIDTH 3','HEIGHT 1','POINTS 3','DATA binary']
            dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4')])
            for i in range(4):
                cloud=np.array([(.025-i*.06,0,2,7),(1.025-i*.06,0,2,9),(-10.025-i*.06,0,2,11)],dtype=dtype)
                vr.write_pcd(frames/f'{i}.pcd',header,cloud)
            (frames/'index.csv').write_text('id,timestamp,file\n'+''.join(f'{i},{row[1]},{i}.pcd\n' for i,row in enumerate(keyframes)))
            original={p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()}
            cfg=config();cfg['cross_keyframe_semantics']=dict(enabled=enabled,voxel_size=.1)
            if overlap:cfg['semantic_evidence']['min_positive_baseline_m']=0.
            if fail_active:cfg['keyframe_clean']['max_active_frames']=1
            tr=vr.Trajectory(source/'keyframes.txt',2.)
            times=(1.15,) if overlap else (1,2,3)
            views=[(float(t),tr.pose(float(t)),np.zeros((32,32,3),np.uint8),None) for t in times]
            masks=np.zeros((1,32,32),bool);masks[0,14:19,14:19]=True
            prediction=ScoredInstances(masks,[2],[.9])
            output=Path(tmp)/'result'
            args=SimpleNamespace(map=str(source),bag='unused',device='cpu',no_cache=True,rviz=False,
                output=output,compact_result=compact,global_voxel_size=.1)
            with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=lambda _:SimpleNamespace(task='segment'))}), \
                patch.object(vr,'iterate_views',return_value=views), \
                patch.object(vr.VehicleVoter,'infer_mask',return_value=(prediction,1)) as infer, \
                patch.object(vr,'estimate_floor',side_effect=lambda xyz,traj:np.full(len(xyz),-1.)):
                if fail_active:
                    with self.assertRaisesRegex(RuntimeError,'simultaneous keyframes'):
                        run_keyframe_clean(args,cfg,'imu_v2')
                    self.assertEqual(infer.call_count,0)
                    self.assertTrue((output/'INCOMPLETE').is_file())
                    self.assertFalse((output/'summary.json').exists())
                    self.assertFalse(any(output.glob('.semantic_votes_*')))
                    self.assertEqual({p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()},original)
                    return
                run_keyframe_clean(args,cfg,'imu_v2')
                self.assertEqual(infer.call_count,len(times))
            report=json.loads((output/'summary.json').read_text())
            deletes=enabled and not overlap
            self.assertEqual(report['removed_points'],4 if deletes else 0)
            clean=vr.read_pcd(output/'global_clean.pcd')[1]
            self.assertEqual(set(clean['intensity']),{9.,11.} if deletes else {7.,9.,11.})
            if enabled:
                self.assertEqual(report['cross_keyframe_semantics']['unique_images'],len(times))
                self.assertEqual(report['cross_keyframe_semantics']['positive_voxels'],1)
                self.assertEqual(report['frames_without_views'],0 if overlap else 1)
                self.assertEqual(report['removed_points_without_local_views'],1 if deletes else 0)
                if overlap:self.assertEqual(report['cross_keyframe_semantics']['voxel_image_pairs'],2)
            if not compact:
                # The fourth keyframe never had a matching image. Its car copy follows the shared decision.
                clean_frame=vr.read_pcd(output/'keyframe_clouds/3.pcd')[1]
                self.assertEqual(set(clean_frame['intensity']),{9.,11.} if deletes else {7.,9.,11.})
            else:self.assertFalse((output/'keyframe_clouds').exists())
            self.assertEqual({p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()},original)
            self.assertFalse((output/'INCOMPLETE').exists())
            self.assertFalse(any(output.glob('.semantic_votes_*')))
            with (output/'frames.csv').open() as stream:rows=list(csv.DictReader(stream))
            self.assertEqual(sum(int(r['removed']) for r in rows),report['removed_points'])

    def test_two_pass_removes_every_evidenced_copy_and_retains_background_and_unknown(self):
        self.exercise_pipeline(False)
        self.exercise_pipeline(True)

    def test_compact_result_uses_the_same_cross_frame_decisions(self):
        self.exercise_pipeline(True,compact=True)

    def test_overlapping_frames_share_one_inference_and_cannot_inflate_a_single_image(self):
        self.exercise_pipeline(True,overlap=True)

    def test_failed_collection_keeps_incomplete_marker_and_cleans_database(self):
        self.exercise_pipeline(True,overlap=True,fail_active=True)


if __name__=='__main__':unittest.main()
