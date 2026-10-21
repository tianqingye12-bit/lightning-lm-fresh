from pathlib import Path
import copy
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
from instance_region_association import describe,region_score,mutual_matches
from vehicle_instance_tracks import InstanceTracks,options
from configure_instance_association import profile
from configure_instance_tracks import profile as base_profile
from segmentation_quality import ScoredInstances
from test_alignment_round13 import fixture,config
from test_vehicle_instance_tracks import HEADER,DTYPE
import clean_map_vehicles as vr


class RegionTests(unittest.TestCase):
    def cloud(self):
        xyz=np.array([[x*.1,y*.1,1.] for y in range(4) for x in range(6)])
        parity=np.array([(x+y)%2 for y in range(4) for x in range(6)])
        return xyz,np.flatnonzero(parity==0),np.flatnonzero(parity==1)

    def cfg(self,enabled=True):
        cfg=base_profile(config());cfg['vehicle_instance_tracks']['region_matching']=enabled
        return cfg

    def obs(self,ids,cls=2):return dict(indices=ids,class_id=cls,score=.9)

    def test_alternating_partial_views_recover_one_track_without_relaxing_confirmation(self):
        xyz,a,b=self.cloud()
        for enabled in (False,True):
            with self.subTest(enabled=enabled):
                t=InstanceTracks(xyz,self.cfg(enabled))
                for i,ids in enumerate((a,b,a,b)):
                    t.begin_image(float(i),[i*.2,0,0]);t.add_observations([self.obs(ids)])
                removed,r=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
                self.assertEqual(r['tracks'],1 if enabled else 2)
                self.assertEqual(r['accepted_tracks'],int(enabled))
                self.assertEqual(removed.sum(),24 if enabled else 0)
                if enabled:
                    self.assertEqual(r['region_matches'],1)
                    self.assertEqual(r['shared_point_matches'],2)
                    self.assertEqual(r['instances'][0]['images'],4)

    def test_region_matches_do_not_bypass_repeated_point_requirement(self):
        xyz,a,b=self.cloud();xyz=np.vstack([xyz,xyz+[.001,.001,0]])
        t=InstanceTracks(xyz,self.cfg())
        for i,ids in enumerate((a,b,a+24)):
            t.begin_image(float(i),[i*.2,0,0]);t.add_observations([self.obs(ids)])
        removed,r=t.finish(np.zeros(len(xyz),bool),cKDTree(xyz))
        self.assertEqual(r['tracks'],1);self.assertFalse(removed.any())
        self.assertEqual(r['instances'][0]['status'],'insufficient_repeated_points')
        for key in ('first_timestamp','last_timestamp','center_world','observed_low','observed_high','region_matches'):
            self.assertIn(key,r['instances'][0])

    def test_region_resource_limit_does_not_partially_update_track(self):
        xyz,a,b=self.cloud();cfg=self.cfg();cfg['vehicle_instance_tracks']['max_memberships']=20
        t=InstanceTracks(xyz,cfg);t.begin_image(0,[0,0,0]);t.add_observations([self.obs(a)])
        t.begin_image(1,[.2,0,0]);self.assertEqual(t.add_observations([self.obs(b)]),[None])
        self.assertEqual(t.tracks[0]['images'],1);self.assertEqual(t.memberships,12)
        self.assertEqual(t.metrics['resource_rejections'],1)

    def test_nearby_nonoverlapping_objects_are_not_joined(self):
        xyz,a,_=self.cloud();xyz=xyz*.3;xyz[:,2]=1
        xyz=np.vstack([xyz,xyz+[.6,0,0]])
        t=InstanceTracks(xyz,self.cfg())
        for i,ids in enumerate((a,a+24)):
            t.begin_image(float(i),[i*.2,0,0]);t.add_observations([self.obs(ids)])
        self.assertEqual(len(t.tracks),2)
        self.assertGreater(t.association_rejections.get('region_overlap',0),0)

    def test_two_vehicles_match_independently_of_proposal_order(self):
        cloud,a,b=self.cloud();xyz=np.vstack([cloud,cloud+[4,0,0]])
        for reverse in (False,True):
            t=InstanceTracks(xyz,self.cfg());t.begin_image(0,[0,0,0])
            self.assertEqual(t.add_observations([self.obs(a),self.obs(a+24)]),[0,1])
            t.begin_image(1,[.2,0,0])
            proposals=[self.obs(b),self.obs(b+24)]
            if reverse:proposals.reverse()
            self.assertEqual(t.add_observations(proposals),[1,0] if reverse else [0,1])

    def test_mutual_assignment_rejects_row_and_column_ambiguity(self):
        matches,rejected=mutual_matches([[(.9,0,'region'),(.9,1,'region')]],.1)
        self.assertFalse(matches);self.assertEqual(rejected,{0:'ambiguous_row'})
        matches,rejected=mutual_matches([[(.9,0,'region')],[(.9,0,'region')]],.1)
        self.assertFalse(matches);self.assertEqual(set(rejected.values()),{'ambiguous_column','one_to_one_conflict'})

    def test_stronger_shared_match_wins_column_without_greedy_creation(self):
        matches,rejected=mutual_matches([[(.9,0,'region')],[(1.5,0,'shared')]],.1)
        self.assertEqual(matches,{1:(0,'shared')});self.assertEqual(rejected,{0:'one_to_one_conflict'})

    def test_duplicate_image_and_long_gap_do_not_gain_region_votes(self):
        xyz,a,b=self.cloud();t=InstanceTracks(xyz,self.cfg())
        t.begin_image(0,[0,0,0]);t.add_observations([self.obs(a)])
        t.begin_image(0,[0,0,0]);self.assertEqual(t.add_observations([self.obs(b)]),[None])
        t.begin_image(10,[.2,0,0]);t.add_observations([self.obs(b)])
        self.assertEqual(len(t.tracks),2);self.assertEqual([r['images'] for r in t.tracks],[1,1])

    def test_region_static_anchor_prevents_progressive_drift(self):
        xyz,a,_=self.cloud();xyz=np.vstack([xyz+[shift,0,0] for shift in (0,.3,.6,.9)])
        t=InstanceTracks(xyz,self.cfg())
        for i in range(4):
            t.begin_image(float(i),[i*.2,0,0]);t.add_observations([self.obs(a+24*i)])
        self.assertEqual(len(t.tracks),2)
        self.assertGreater(t.association_rejections.get('region_anchor',0),0)

    def test_size_orientation_and_cumulative_extent_gates(self):
        cfg=options(self.cfg());xyz,a,_=self.cloud();desc=describe(xyz[a])
        large=describe((xyz[a]-desc['center'])*5+desc['center'])
        self.assertEqual(region_score(large,desc,desc['center'],desc['low'],desc['high'],cfg)[1],'region_size')
        long=np.array([[-.4,-.1,1],[-.4,.1,1],[.4,-.1,1],[.4,.1,1]])
        rotated=long.copy();rotated[:,:2]=long[:,:2]@np.array([[0,1],[-1,0]])
        prior=describe(long);observation=describe(rotated)
        self.assertEqual(region_score(observation,prior,prior['center'],prior['low'],prior['high'],dict(cfg,region_cover=.4))[1],'region_yaw')
        self.assertEqual(region_score(desc,desc,desc['center'],desc['low'],desc['high']+[20,0,0],cfg)[1],'region_extent')

    def test_configuration_preserves_all_previous_settings(self):
        cfg=self.cfg(False);before=copy.deepcopy(cfg);result=profile(cfg)
        self.assertEqual(cfg,before)
        expected=copy.deepcopy(cfg);expected['vehicle_instance_tracks']['region_matching']=True
        self.assertEqual(result,expected)
        for bad in ({'region_matching':'true'},{'region_center_m':2},{'region_yaw_deg':float('nan')}):
            with self.assertRaises(ValueError):options({'vehicle_instance_tracks':bad})

    def test_real_map_pipeline_recovers_fragmented_observations_and_preserves_point_fallback(self):
        for enabled in (False,True):
            with self.subTest(enabled=enabled),tempfile.TemporaryDirectory() as tmp:
                source=Path(tmp)/'map';fixture(source,keyframes=[[0,1,0,0,0,0,0,0,1]])
                points=[((u-16)*.2,(v-16)*.2,2.,7.) for v in range(12,16) for u in range(12,18)]
                points += [(0,0,8,9.)]
                vr.write_pcd(source/'global.pcd',HEADER,np.array(points,dtype=DTYPE))
                original=(source/'global.pcd').read_bytes();cfg=self.cfg(enabled)
                cfg['segmentation_quality']['core_erode_px']=0
                path=Path(tmp)/'config.yaml';path.write_text(yaml.safe_dump(cfg))
                predictions=[];views=[]
                for i in range(4):
                    mask=np.zeros((1,32,32),bool)
                    for v in range(12,16):
                        for u in range(12,18):
                            if (u+v)%2==i%2:mask[0,v,u-i]=True
                    predictions.append((ScoredInstances(mask,[2],[.9]),1))
                    pose=np.eye(4);pose[0,3]=i*.2
                    views.append((.8+i*.2,pose,np.zeros((32,32,3),np.uint8),None))
                with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=lambda _:SimpleNamespace(task='segment'))}), \
                     patch.object(vr,'iterate_views',return_value=views), \
                     patch.object(vr.VehicleVoter,'infer_mask',side_effect=predictions), \
                     patch.object(vr,'estimate_floor',side_effect=lambda xyz,traj:np.full(len(xyz),-1.)), \
                     patch.object(sys,'argv',['clean','--map',str(source),'--bag','unused','--config',str(path)]):
                    vr.main()
                result=vr.read_pcd(source/'global_clean.pcd')[1]
                self.assertEqual(len(result),1 if enabled else 25)
                self.assertEqual((source/'global.pcd').read_bytes(),original)
                report=json.loads(next((source/'clean_diagnostics').glob('*/summary.json')).read_text())
                self.assertEqual(report['removal_stages']['vote_voxels'],0)
                self.assertEqual(report['vehicle_instance_tracks']['accepted_tracks'],int(enabled))
                if enabled:self.assertGreater(report['vehicle_instance_tracks']['region_matches'],0)


if __name__=='__main__':unittest.main()
