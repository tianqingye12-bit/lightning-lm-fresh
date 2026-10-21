import copy
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from seed_local_cleanup import complete, options, labels
from configure_seed_cleanup import profile


class SeedCompletionTests(unittest.TestCase):
    def run_cloud(self, xyz, seed_ids=(0,1,2), **kwargs):
        xyz = np.asarray(xyz, float)
        n = len(xyz)
        seeds = np.zeros(n, bool); seeds[list(seed_ids)] = True
        visible = kwargs.pop('visible', np.where(seeds, 3, 0))
        positive = kwargs.pop('positive', np.where(seeds, 3, 0))
        background = kwargs.pop('background', np.zeros(n, int))
        floor = kwargs.pop('floor', np.zeros(n, bool))
        cfg = dict(min_views=3, seed_local_cleanup=dict(enabled=True, min_seeds=3, **kwargs))
        result = complete(xyz, seeds, np.asarray(visible), np.asarray(positive), np.asarray(background), np.asarray(floor), cfg)
        np.testing.assert_array_equal(seeds, np.arange(n) < 3)  # caller's seeds remain unchanged
        return result

    def test_continuous_residual_grows_but_cannot_chain_far_from_seed(self):
        xyz = [[i*.1,0,1] for i in range(20)]
        mask, r = self.run_cloud(xyz)
        self.assertTrue(mask[:9].all())  # last seed .2 + .6m
        self.assertFalse(mask[9:].any())
        self.assertEqual(r['added_voxels'], 6)

    def test_disconnected_patch_inside_seed_envelope_stays(self):
        mask, _ = self.run_cloud([[0,0,1],[.1,0,1],[.2,0,1],[.6,0,1]])
        self.assertFalse(mask[3])

    def test_floor_and_explicit_background_are_not_removed(self):
        xyz = [[0,0,1],[.1,0,1],[.2,0,1],[.3,0,1],[.4,0,1]]
        mask, _ = self.run_cloud(xyz, floor=[0,0,0,1,0], background=[0,0,0,0,1])
        self.assertEqual(mask.tolist(), [True,True,True,False,False])

    def test_repeated_negative_observations_veto_even_without_background_array(self):
        mask, r = self.run_cloud([[i*.1,0,1] for i in range(4)],
                                 visible=[3,3,3,5], positive=[3,3,3,0])
        self.assertFalse(mask[3])
        self.assertEqual(r['background_veto_voxels'], 1)

    def test_candidate_cap_skips_whole_group_without_partial_deletion(self):
        mask, r = self.run_cloud([[i*.1,0,1] for i in range(5)], max_candidates=3)
        self.assertEqual(mask.sum(), 3)
        self.assertEqual(r['skipped_candidates'], 1)

    def test_large_seed_surface_rejected_instead_of_deleting_neighbor_structure(self):
        mask, r = self.run_cloud([[i*.1,0,1] for i in range(5)], max_extent_m=[.15,1,1])
        self.assertEqual(mask.sum(), 3)
        self.assertEqual(r['skipped_extent'], 1)

    def test_expansion_envelope_is_clipped_to_extent_limit(self):
        mask,_=self.run_cloud([[i*.1,0,1] for i in range(8)], max_extent_m=[.4,1,1])
        self.assertTrue(mask[:4].all())
        self.assertFalse(mask[4:].any())

    def test_observed_background_barrier_cannot_be_used_as_a_bridge(self):
        xyz=[[i*.1,0,1] for i in range(8)]
        mask,_=self.run_cloud(xyz, background=[0,0,0,1,1,1,0,0])
        self.assertFalse(mask[3:].any())

    def test_group_budget_keeps_unprocessed_groups_at_seed_decisions(self):
        xyz=[[0,0,1],[.1,0,1],[.2,0,1],[.3,0,1],
             [3,0,1],[3.1,0,1],[3.2,0,1],[3.3,0,1]]
        seeds=np.array([True,True,True,False,True,True,True,False])
        cfg=dict(min_views=3,seed_local_cleanup=dict(enabled=True,min_seeds=3,max_groups=1))
        mask,r=complete(np.array(xyz),seeds,np.where(seeds,3,0),np.where(seeds,3,0),
                        np.zeros(8,int),np.zeros(8,bool),cfg)
        self.assertTrue(mask[3]);self.assertFalse(mask[7])
        self.assertEqual(r['skipped_budget'],1)

    def test_small_independent_seed_groups_do_not_share_support(self):
        mask, r = self.run_cloud([[0,0,1],[.1,0,1],[2,0,1],[.2,0,1],[2.1,0,1]])
        self.assertEqual(mask.sum(), 3)
        self.assertEqual(r['skipped_small'], 2)

    def test_empty_seeds_and_default_disabled_do_nothing(self):
        xyz=np.array([[0,0,1],[.1,0,1]],float)
        for cfg, seeds in (({},[True,False]), ({'seed_local_cleanup':{'enabled':True}},[False,False])):
            result,_=complete(xyz,np.array(seeds),np.zeros(2),np.zeros(2),np.zeros(2),np.zeros(2,bool),cfg)
            self.assertEqual(result.tolist(),seeds)
        self.assertEqual(labels(np.empty((0,3)),.2).tolist(), [])
        self.assertEqual(labels(xyz[:1],.2).tolist(), [0])

    def test_profile_preserves_calibration_and_detection_thresholds(self):
        cfg={'K':list(range(9)), 'time_offset':.123, 'max_range':15., 'min_views':3,
             'vehicle_ratio':.5, 'depth_margin':2., 'mask_dilate_px':7}
        before=copy.deepcopy(cfg)
        result=profile(cfg)
        self.assertEqual(cfg,before)
        for key,value in cfg.items():self.assertEqual(result[key],value)
        self.assertTrue(result['legacy_map_cleanup'])
        self.assertTrue(result['seed_local_cleanup']['enabled'])

    def test_validation_rejects_invalid_or_misspelled_options(self):
        for supplied in ({'enabled':'true'}, {'link_m':float('nan')}, {'min_seeds':0},
                         {'max_extent_m':[1,2]}, {'background_ratio':1}, {'max_canidates':10}):
            with self.subTest(supplied=supplied),self.assertRaises(ValueError):
                options({'seed_local_cleanup':supplied})


if __name__ == '__main__':unittest.main()
