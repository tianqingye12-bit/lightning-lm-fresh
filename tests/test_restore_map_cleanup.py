import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import yaml
# Load sparse types before patch.dict restores sys.modules between pipeline runs.
import scipy.sparse
import scipy.sparse.csgraph
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from restore_map_cleanup import recovery_profile,prepare,write_profile
from clean_map_vehicles import read_pcd,write_pcd
from test_alignment_round13 import fixture
from test_alignment_round13 import config
import clean_map_vehicles as vr
from legacy_map_cleanup import associate_legacy,legacy_votes
from clean_map_instances import associate_instances

HEADER=['VERSION .7','FIELDS x y z intensity','SIZE 4 4 4 4','TYPE F F F F',
    'COUNT 1 1 1 1','WIDTH 1','HEIGHT 1','POINTS 1','DATA binary']
DTYPE=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4')])


class RestoreTests(unittest.TestCase):
    def test_profile_preserves_old_calibration_model_and_cluster_settings(self):
        cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/m2dgr_clean.yaml').read_text())
        result=recovery_profile(cfg)
        for name in cfg:
            if name not in ('clean_mode','pose_trajectory','cross_keyframe_semantics','semantic_evidence','segmentation_quality'):
                self.assertEqual(result[name],cfg[name])
        self.assertEqual(result['mask_dilate_px'],cfg['mask_dilate_px'])
        self.assertEqual(result['clean_mode'],'map')
        self.assertTrue(result['legacy_map_cleanup'])
        self.assertEqual(result['pose_trajectory']['source'],'keyframes')
        for name in ('cross_keyframe_semantics','semantic_evidence','segmentation_quality'):self.assertFalse(result[name]['enabled'])
        self.assertNotIn('clean_mode',cfg)
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'a.yaml';out=Path(tmp)/'b.yaml';source.write_text(yaml.safe_dump(cfg))
            write_profile(source,out)
            with self.assertRaises(FileExistsError):write_profile(source,out)

    def test_existing_global_is_copied_without_changing_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'source';fixture(source)
            cloud=np.array([(5,0,2,7)],dtype=DTYPE);write_pcd(source/'global.pcd',HEADER,cloud)
            before={p.name:p.read_bytes() for p in source.iterdir() if p.is_file()}
            output=prepare(source,Path(tmp)/'map_mode')
            self.assertEqual((output/'global.pcd').read_bytes(),before['global.pcd'])
            self.assertEqual({p.name:p.read_bytes() for p in source.iterdir() if p.is_file()},before)
            self.assertFalse((output/'INCOMPLETE').exists())
            self.assertFalse((output/'trajectory.yaml').exists())
            self.assertFalse(json.loads((output/'PREPARED_MAP.json').read_text())['semantic_cleaning_performed'])

    def test_missing_global_rebuild_uses_saved_poses_and_original_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'source'
            fixture(source,keyframes=[[0,1,10,0,0,0,0,0,1],[1,2,20,0,0,0,0,0,1]])
            frames=source/'keyframe_clouds';frames.mkdir()
            cloud=np.array([(.025,0,2,7)],dtype=DTYPE)
            for i in range(2):write_pcd(frames/f'{i}.pcd',HEADER,cloud)
            (frames/'index.csv').write_text('id,timestamp,file\n0,1,0.pcd\n1,2,1.pcd\n')
            original={p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()}
            output=prepare(source,Path(tmp)/'map_mode')
            result=read_pcd(output/'global.pcd')[1]
            np.testing.assert_allclose(result['x'],[10.025,20.025])
            self.assertEqual(result['intensity'].tolist(),[7,7])
            self.assertEqual({p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()},original)
            self.assertFalse((output/'INCOMPLETE').exists())
            with self.assertRaises(FileExistsError):prepare(source,output)

    def test_historical_depth_margin_and_sparse_mask_are_used(self):
        cfg=dict(mask_dilate_px=1,depth_margin=2.,min_views=3,instance_filter={})
        scene=np.full((8,8),np.inf);scene[3,3]=2.
        masks=np.zeros((1,8,8),bool);masks[0,3,3]=True
        u=np.array([3,4,4]);v=np.array([3,3,3]);z=np.array([2.,3.9,4.1])
        hit,_=associate_legacy(masks,scene,u,v,z,cfg)
        self.assertEqual(hit.tolist(),[True,True,False])
        modern=associate_instances(masks,scene,u,v,z,np.ones(3,bool),cfg)[0]
        self.assertFalse(modern.any())  # Fewer than three supported depth pixels.

    def test_historical_ratio_is_positive_over_visible_without_new_positive_count_gate(self):
        cfg=dict(min_views=3,vehicle_ratio=.5)
        observed,removed,_,_=legacy_votes(np.array([3,2,3,3]),np.array([2,2,1,3]),
            np.array([False,False,False,True]),cfg)
        self.assertEqual(observed.tolist(),[True,False,True,True])
        self.assertEqual(removed.tolist(),[True,False,False,False])

    def test_real_map_pipeline_restores_seed_votes_and_historical_cluster_expansion(self):
        for grow in (False,True,'local','local_occluded'):
            with self.subTest(grow=grow),tempfile.TemporaryDirectory() as tmp:
                source=Path(tmp)/'map';fixture(source,keyframes=[[0,1,0,0,0,0,0,0,1]])
                cloud=np.array([(0,0,2,7),(.5,0,2,9)],dtype=DTYPE)
                write_pcd(source/'global.pcd',HEADER,cloud)
                original=(source/'global.pcd').read_bytes()
                cfg=config();cfg.update(depth_margin=2.,mask_dilate_px=0,cluster_voxel=1.,
                    cluster_min_observed=2 if grow else 999,cluster_vehicle_ratio=.5)
                cfg=recovery_profile(cfg)
                if isinstance(grow,str):
                    cfg['seed_local_cleanup'] = dict(enabled=True, min_seeds=1, link_m=.6)
                path=Path(tmp)/'config.yaml';path.write_text(yaml.safe_dump(cfg))
                image=np.zeros((32,32,3),np.uint8)
                depth=None
                if grow == 'local_occluded':
                    depth=np.zeros((32,32),np.float32)
                    depth[16,18]=1./cfg['depth_scale']  # second map point has no visible observations
                views=[(t,np.eye(4),image,depth) for t in (.8,1.,1.2)]
                mask=np.zeros((1,32,32),bool);mask[0,16,16]=True
                predictions=[(mask,1),(mask,1),(np.empty((0,32,32),bool),0)]
                with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=lambda _:SimpleNamespace(task='segment'))}), \
                    patch.object(vr,'iterate_views',return_value=views), \
                    patch.object(vr.VehicleVoter,'infer_mask',side_effect=predictions), \
                    patch.object(vr,'estimate_floor',side_effect=lambda xyz,traj:np.full(len(xyz),-1.)), \
                    patch.object(sys,'argv',['clean','--map',str(source),'--bag','unused','--config',str(path)]):
                    vr.main()
                removed=read_pcd(source/'global_vehicles.pcd')[1]
                expanded = grow is True or grow == 'local_occluded'
                self.assertEqual(len(removed),2 if expanded else 1)
                self.assertEqual(set(removed['intensity']),{7.,9.} if expanded else {7.})
                self.assertEqual((source/'global.pcd').read_bytes(),original)
                summary=json.loads(next((source/'clean_diagnostics').glob('*/summary.json')).read_text())
                self.assertTrue(summary['legacy_map_cleanup'])
                self.assertEqual(summary['removal_stages']['vote_voxels'], 1)
                if isinstance(grow,str):
                    self.assertTrue(summary['seed_local_cleanup']['enabled'])
                    self.assertFalse(summary['cluster_growth_enabled'])
                    self.assertEqual(summary['seed_local_cleanup']['added_voxels'],int(grow == 'local_occluded'))


if __name__=='__main__':unittest.main()
