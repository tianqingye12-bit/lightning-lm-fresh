from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial import cKDTree
import scipy.sparse.csgraph
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from vehicle_instance_tracks import InstanceTracks, extract, options
from configure_instance_tracks import profile
from instance_map_labels import write_labels, LabelLookup, label_keys
from rebuild_instance_static_map import rebuild
from segmentation_quality import ScoredInstances
import clean_map_vehicles as vr
from test_alignment_round13 import fixture,config

HEADER=['FIELDS x y z intensity','SIZE 4 4 4 4','TYPE F F F F','COUNT 1 1 1 1','WIDTH 1','HEIGHT 1','POINTS 1','DATA binary']
DTYPE=np.dtype([(k,'<f4') for k in ('x','y','z','intensity')])


class TracksTests(unittest.TestCase):
    def cfg(self,**opts):
        cfg=profile(config());cfg['vehicle_instance_tracks'].update(min_points=3,min_pixels=3,**opts)
        return cfg

    def xyz(self):return np.array([[i*.1,0,1.] for i in range(12)])

    def observe(self,tracker,ids=(0,1,2),cls=2,score=.9,cameras=(0,.2,.4)):
        for i,x in enumerate(cameras):
            tracker.begin_image(float(i),[x,0,0]);tracker.add(ids,cls,score)

    def test_multiframe_confirmation_and_bounded_oriented_completion(self):
        xyz=self.xyz();t=InstanceTracks(xyz,self.cfg());self.observe(t)
        removed,report=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
        self.assertEqual(report['accepted_tracks'],1);self.assertEqual(report['seed_voxels'],3)
        self.assertTrue(removed[:7].all());self.assertFalse(removed[7:].any())

    def test_same_image_cannot_confirm_track_or_merge_separate_proposals(self):
        xyz=self.xyz();t=InstanceTracks(xyz,self.cfg())
        for _ in range(3):t.begin_image(1.,[0,0,0]);t.add([0,1,2],2,.9)
        removed,report=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
        self.assertFalse(removed.any());self.assertEqual(report['duplicate_images'],2)
        self.assertEqual(report['instances'][0]['images'],1)

    def test_stationary_camera_is_not_independent_viewpoint_evidence(self):
        xyz=self.xyz();t=InstanceTracks(xyz,self.cfg());self.observe(t,cameras=(0,0,0))
        mask,r=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
        self.assertFalse(mask.any());self.assertEqual(r['instances'][0]['status'],'insufficient_baseline')

    def test_separate_cars_keep_distinct_tracks_and_space_between(self):
        xyz=np.vstack([self.xyz(),self.xyz()+[4,0,0]])
        t=InstanceTracks(xyz,self.cfg())
        for i in range(3):
            t.begin_image(float(i),[i*.2,0,0]);t.add([0,1,2],2,.9);t.add([12,13,14],2,.9)
        mask,r=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
        self.assertEqual(r['accepted_tracks'],2);self.assertFalse(mask[7:12].any())

    def test_floor_and_disconnected_background_not_filled(self):
        xyz=self.xyz();xyz[3:6,1]=2
        t=InstanceTracks(xyz,self.cfg());self.observe(t)
        floor=np.zeros(len(xyz),bool);floor[6:]=True
        mask,_=t.finish(floor,cKDTree(xyz))
        self.assertTrue(mask[:3].all());self.assertFalse(mask[3:].any())

    def test_ambiguous_merge_abstains(self):
        xyz=self.xyz();t=InstanceTracks(xyz,self.cfg(min_overlap=.1))
        t.begin_image(0,[0,0,0]);t.add([0,1,2],2,.9);t.add([3,4,5],2,.9)
        t.begin_image(1,[.2,0,0]);t.add([0,1,2,3,4,5],2,.9)
        self.assertEqual(t.metrics['ambiguous_matches'],1)
        self.assertEqual([r['images'] for r in t.tracks],[1,1])

    def test_resource_cap_does_not_partially_add_observations(self):
        xyz=self.xyz();t=InstanceTracks(xyz,self.cfg(max_memberships=3))
        t.begin_image(0,[0,0,0]);t.add([0,1,2],2,.9)
        t.begin_image(1,[.2,0,0]);t.add([0,1,2,3],2,.9)
        self.assertEqual(t.memberships,3);self.assertEqual(t.tracks[0]['images'],1)
        self.assertEqual(t.metrics['resource_rejections'],1)

    def test_low_confidence_track_is_not_confirmed(self):
        xyz=self.xyz();t=InstanceTracks(xyz,self.cfg());self.observe(t,score=.4)
        mask,r=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
        self.assertFalse(mask.any());self.assertEqual(r['instances'][0]['status'],'low_confidence')

    def test_single_observation_point_does_not_become_distant_seed(self):
        xyz=self.xyz();t=InstanceTracks(xyz,self.cfg())
        for i,ids in enumerate(([0,1,2,11],[0,1,2],[0,1,2])):
            t.begin_image(float(i),[i*.2,0,0]);t.add(ids,2,.9)
        mask,r=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
        self.assertEqual(r['seed_voxels'],3);self.assertFalse(mask[11])

    def test_completion_candidate_limit_keeps_confirmed_seeds(self):
        xyz=self.xyz();t=InstanceTracks(xyz,self.cfg(max_candidates=2));self.observe(t)
        mask,r=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
        self.assertEqual(mask.sum(),3);self.assertEqual(r['instances'][0]['completion'],'candidate_limit')

    def test_long_gap_and_class_conflict_do_not_merge(self):
        xyz=self.xyz()
        for times,classes in (((0,10,20),(2,2,2)),((0,1,2),(2,3,2))):
            t=InstanceTracks(xyz,self.cfg())
            for i,(timestamp,cls) in enumerate(zip(times,classes)):
                t.begin_image(float(timestamp),[i*.2,0,0]);t.add([0,1,2],cls,.9)
            mask,r=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
            self.assertFalse(mask.any());self.assertEqual(r['accepted_tracks'],0)

    def test_voxel_boundary_snap_handles_float32_export_without_neighbor_dilation(self):
        a=np.array([[.2,-.8,1.]])
        np.testing.assert_array_equal(label_keys(a,.1),label_keys(a.astype(np.float32).astype(float),.1))
        self.assertNotEqual(label_keys(np.array([[.199,0,1]]),.1)[0,0],label_keys(np.array([[.2,0,1]]),.1)[0,0])

    def test_drifting_geometry_is_not_claimed_as_static_vehicle(self):
        xyz=self.xyz();t=InstanceTracks(xyz,self.cfg(max_center_drift_m=.01))
        t.begin_image(0,[0,0,0]);t.add([0,1,2],2,.9)
        t.begin_image(1,[.2,0,0]);t.add([0,1,2,3],2,.9)
        t.begin_image(2,[.4,0,0]);t.add([0,1,2,3],2,.9)
        mask,r=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
        self.assertFalse(mask.any());self.assertEqual(r['instances'][0]['status'],'unstable_geometry')

    def test_individual_masks_do_not_merge_shared_points(self):
        cfg=self.cfg();cfg['segmentation_quality']['core_erode_px']=0;cfg['instance_filter']['min_depth_pixels']=3
        masks=np.ones((2,10,10),bool);pred=ScoredInstances(masks,[2,2],[.9,.9])
        scene=np.full((10,10),2.);u=np.array([3,4,5]);v=np.array([3,3,3]);z=np.full(3,2.)
        cam=np.array([[0,0,2],[.1,0,2],[.2,0,2]])
        hit,bg,_,_,observations=extract(pred,scene,cam,u,v,z,np.ones(3,bool),np.zeros(3,bool),cfg)
        self.assertFalse(hit.any());self.assertFalse(bg.any());self.assertEqual(observations,[])

    def test_validation_and_timestamp_order(self):
        with self.assertRaises(ValueError):options({'vehicle_instance_tracks':{'min_images':True}})
        t=InstanceTracks(self.xyz(),self.cfg());t.begin_image(1,[0,0,0])
        with self.assertRaises(ValueError):t.begin_image(0,[0,0,0])

    def test_label_transfer_removes_all_frame_copies_preserves_fields_and_floor(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'original'
            fixture(source,keyframes=[[0,1,0,0,0,0,0,0,1],[1,2,0,0,0,0,0,0,1]])
            frames=source/'keyframe_clouds';frames.mkdir()
            cloud=np.array([(.15,0,1.15,7),(.15,0,1.11,8),(3,0,1.15,9)],dtype=DTYPE)
            for i in range(2):vr.write_pcd(frames/f'{i}.pcd',HEADER,cloud)
            (frames/'index.csv').write_text('id,timestamp,file\n0,1,0.pcd\n1,2,1.pcd\n')
            labels=Path(tmp)/'labels.sqlite'
            write_labels(labels,source,np.array([[.15,0,1.15]]),np.array([True]),np.array([1.13]),.1)
            original={p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()}
            output=rebuild(source,labels,Path(tmp)/'static',.1)
            result=vr.read_pcd(output/'global_clean.pcd')[1]
            self.assertEqual(set(result['intensity']),{8,9})
            self.assertEqual(json.loads((output/'summary.json').read_text())['removed_points'],2)
            self.assertEqual({p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()},original)
            self.assertFalse((output/'INCOMPLETE').exists())
            self.assertFalse((output/'keyframe_clouds').exists())
            with self.assertRaises(FileExistsError):rebuild(source,labels,output)
            (source/'keyframes.txt').write_text('changed')
            with self.assertRaisesRegex(ValueError,'mismatch'):LabelLookup(labels,source)

    def test_real_map_pipeline_tracks_masks_and_exports_transferable_labels(self):
        self._map_pipeline(moving=True)

    def test_rejected_track_preserves_already_confirmed_point_votes_and_labels(self):
        self._map_pipeline(moving=False)

    def test_duplicate_images_cannot_inflate_fallback_point_votes(self):
        self._map_pipeline(moving=False,duplicate=True)

    def _map_pipeline(self,moving,duplicate=False):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'map';fixture(source,keyframes=[[0,1,0,0,0,0,0,0,1]])
            points=[((u-16)*.2,(v-16)*.2,2.,7.) for v in (12,13,14) for u in (12,13,14,15)]
            points += [(0,0,8,9.)]
            vr.write_pcd(source/'global.pcd',HEADER,np.array(points,dtype=DTYPE))
            original=(source/'global.pcd').read_bytes();cfg=profile(config())
            path=Path(tmp)/'config.yaml';path.write_text(yaml.safe_dump(cfg))
            masks=np.zeros((1,32,32),bool);masks[0,8:22,8:22]=True
            prediction=ScoredInstances(masks,[2],[.9])
            views=[]
            for i in range(3):
                pose=np.eye(4);pose[0,3]=i*.2 if moving else 0.
                views.append((.8 if duplicate else .8+i*.2,pose,np.zeros((32,32,3),np.uint8),None))
            with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=lambda _:SimpleNamespace(task='segment'))}), \
                 patch.object(vr,'iterate_views',return_value=views), \
                 patch.object(vr.VehicleVoter,'infer_mask',return_value=(prediction,1)), \
                 patch.object(vr,'estimate_floor',side_effect=lambda xyz,traj:np.full(len(xyz),-1.)), \
                 patch.object(sys,'argv',['clean','--map',str(source),'--bag','unused','--config',str(path)]):
                vr.main()
            result=vr.read_pcd(source/'global_clean.pcd')[1]
            self.assertEqual(result['intensity'].tolist(),[7.]*12+[9.] if duplicate else [9.])
            self.assertEqual((source/'global.pcd').read_bytes(),original)
            self.assertTrue((source/'vehicle_labels.sqlite').is_file())
            report=json.loads(next((source/'clean_diagnostics').glob('*/summary.json')).read_text())
            self.assertEqual(report['vehicle_instance_tracks']['accepted_tracks'],int(moving and not duplicate))
            self.assertEqual(report['vehicle_instance_tracks']['point_vote_voxels'],0 if duplicate else 12)
            self.assertEqual(report['vehicle_instance_tracks']['combined_final_voxels'],0 if duplicate else 12)
            self.assertGreaterEqual(report['removal_stages']['added_voxels'],0)
            if not moving and not duplicate:
                self.assertEqual(report['vehicle_instance_tracks']['instance_only_voxels'],0)
            lookup=LabelLookup(source/'vehicle_labels.sqlite',source)
            try:self.assertEqual(lookup.lookup(np.array(points)[:,:3]).tolist(),[False]*13 if duplicate else [True]*12+[False])
            finally:lookup.close()


if __name__=='__main__':unittest.main()
