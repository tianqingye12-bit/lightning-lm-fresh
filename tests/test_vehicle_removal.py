"""Run with: python3 -m unittest discover -s tests -p 'test_vehicle_removal.py'."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
import scipy.sparse
import scipy.spatial
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
spec = importlib.util.spec_from_file_location('vehicle_removal', ROOT / 'scripts/clean_map_vehicles.py')
vr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vr)


class VehicleRemovalTests(unittest.TestCase):
    def trajectory(self, rows):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = Path(tmp.name) / 'keyframes.txt'
        np.savetxt(path, rows)
        return vr.Trajectory(path, 2.0)

    def test_exact_endpoints_and_normalized_quaternions(self):
        traj = self.trajectory([[0, 10, 0, 0, 0, 0, 0, 0, 2],
                                [1, 12, 2, 0, 0, 0, 0, 0, 3]])
        for timestamp, x in [(10, 0), (11, 1), (12, 2)]:
            pose = traj.pose(timestamp)
            np.testing.assert_allclose(pose[:3, :3], np.eye(3))
            self.assertEqual(pose[0, 3], x)
        self.assertIsNone(traj.pose(9))
        self.assertIsNone(traj.pose(float('nan')))

    def test_single_keyframe(self):
        traj = self.trajectory([[0, 10, 3, 0, 0, 0, 0, 0, 1]])
        self.assertEqual(traj.pose(10)[0, 3], 3)
        self.assertIsNone(traj.pose(10.1))

    def test_reject_invalid_trajectory(self):
        for rows in ([[0, 10, 0, 0, 0, 0, 0, 0, 0]],
                     [[0, 10, 0, 0, 0, 0, 0, 0, 1], [1, 10, 1, 0, 0, 0, 0, 0, 1]]):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                self.trajectory(rows)

    def test_voxel_keys_do_not_collide(self):
        # The old 21-bit packed coordinates alias across the y/z boundary.
        xyz = np.array([[0, 1, 0], [0, 0, 2 ** 21], [-3, -2, -1]], dtype=float)
        self.assertEqual(len(np.unique(vr.voxel_keys(xyz, 1), axis=0)), 3)
        with self.assertRaises(ValueError):
            vr.voxel_keys(xyz, 0)

    def test_components_match_geometric_neighbors(self):
        xyz = np.array([[-2, 0, 0], [-1, 1, 1], [0, 2, 2], [10, 0, 0],
                        [0, 0, 2 ** 21]], dtype=float)
        labels = vr.components(xyz, 1)
        self.assertEqual(labels[0], labels[1])
        self.assertEqual(labels[1], labels[2])
        self.assertEqual(len(np.unique(labels)), 3)
        self.assertEqual(len(vr.components(np.empty((0, 3)), 1)), 0)

    def test_truncated_pcd_fails_instead_of_looping(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'bad.pcd'
            path.write_text('VERSION .7\nFIELDS x y z\n')
            with self.assertRaisesRegex(ValueError, 'DATA'):
                vr.read_pcd(path)

    def test_spatial_selection_matches_full_scan(self):
        rng = np.random.default_rng(42)
        xyz = rng.uniform(-30, 30, (10000, 3))
        cfg = yaml.safe_load((ROOT / 'config/m2dgr_clean.yaml').read_text())
        voter = vr.VehicleVoter(cfg, xyz, None, 'cpu')
        for origin in (np.zeros(3), np.array([1, -3, 7])):
            expected = np.flatnonzero((np.abs(xyz - origin) < cfg['max_range']).all(axis=1))
            actual = voter.spatial_index.query_ball_point(origin, cfg['max_range'], p=np.inf)
            np.testing.assert_array_equal(np.sort(actual), expected)

    def test_projection_votes_only_for_visible_front_surface(self):
        cfg = yaml.safe_load((ROOT / 'config/m2dgr_clean.yaml').read_text())
        cfg.update(width=32, height=32, K=[10., 0., 16., 0., 10., 16., 0., 0., 1.],
                   distortion=[0.] * 5, T_cam_lidar=np.eye(4).ravel().tolist(), mask_dilate_px=0,
                   instance_filter={"min_depth_pixels": 1})
        xyz = np.array([[0., 0., 2.], [0., 0., 5.], [0., 0., -1.]])
        voter = vr.VehicleVoter(cfg, xyz, None, 'cpu')
        voter.vehicle_mask = lambda _: (np.ones((32, 32), dtype=bool), 1)
        image = np.zeros((32, 32, 3), dtype=np.uint8)
        voter.add_view(np.eye(4), image, None)
        np.testing.assert_array_equal(voter.visible, [1, 0, 0])
        np.testing.assert_array_equal(voter.vehicle, [1, 0, 0])
        depth = np.full((32, 32), 2000, dtype=np.uint16)
        voter.add_view(np.eye(4), image, depth)
        np.testing.assert_array_equal(voter.vehicle, [2, 0, 0])
        with self.assertRaisesRegex(ValueError, 'aligned'):
            voter.add_view(np.eye(4), image, depth[:16])

    def test_pipeline_keeps_invalid_points_out_of_centroids_and_handles_all_floor(self):
        # Exercise the real main/output path without a model download or ROS bag.
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            cloud = np.array([(1., 2., 3.), (float('nan'), 0., 0.)],
                             dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
            header = ['VERSION .7', 'FIELDS x y z', 'SIZE 4 4 4', 'TYPE F F F',
                      'COUNT 1 1 1', 'WIDTH 2', 'HEIGHT 1', 'POINTS 2', 'DATA binary']
            vr.write_pcd(directory / 'global.pcd', header, cloud)
            np.savetxt(directory / 'keyframes.txt', [[0, 10, 0, 0, 0, 0, 0, 0, 1]])
            cfg = yaml.safe_load((ROOT / 'config/m2dgr_clean.yaml').read_text())
            cfg['cache'] = {'enabled': False}
            cfg['diagnostics'] = {'max_overlays': 0}
            config_path = directory / 'config.yaml'
            config_path.write_text(yaml.safe_dump(cfg))
            class FakeVoter:
                def __init__(self, config, centroids, model, device):
                    np.testing.assert_array_equal(centroids, [[1., 2., 3.]])
                    self.visible = np.array([3])
                    self.vehicle = np.array([3])
                    self.background = np.array([0])
                    self.views = 0
                    self.last_stats = dict(projected=1, visible=1, occluded=0, vehicle_votes=1, objects=1)
                def add_view(self, *args):
                    self.views += 1
                    return 1
            fake_yolo = types.SimpleNamespace(YOLO=lambda _: types.SimpleNamespace(task='segment'))
            argv = ['clean_map_vehicles.py', '--map', tmp, '--bag', 'unused', '--config', str(config_path)]
            with patch.dict(sys.modules, {'ultralytics': fake_yolo}), patch.object(sys, 'argv', argv), \
                    patch.object(vr, 'VehicleVoter', FakeVoter), \
                    patch.object(vr, 'iterate_views', return_value=[(10, np.eye(4), None, None)]), \
                    patch.object(vr, 'estimate_floor', return_value=np.array([3.])):
                vr.main()
            _, clean = vr.read_pcd(directory / 'global_clean.pcd')
            _, removed = vr.read_pcd(directory / 'global_vehicles.pcd')
            self.assertEqual(len(clean), 2)
            self.assertTrue(np.isnan(clean['x'][1]))
            self.assertEqual(len(removed), 0)

    def test_cluster_growth_keeps_observed_background(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            cloud = np.array([(0., 0., 1.), (.11, 0., 1.), (.22, 0., 1.), (.33, 0., 1.)],
                             dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
            header = ['VERSION .7', 'FIELDS x y z', 'SIZE 4 4 4', 'TYPE F F F',
                      'COUNT 1 1 1', 'WIDTH 4', 'HEIGHT 1', 'POINTS 4', 'DATA binary']
            vr.write_pcd(directory / 'global.pcd', header, cloud)
            np.savetxt(directory / 'keyframes.txt', [[0, 10, 0, 0, 0, 0, 0, 0, 1]])
            cfg = yaml.safe_load((ROOT / 'config/m2dgr_clean.yaml').read_text())
            cfg.update(cache={'enabled': False}, diagnostics={'max_overlays': 0},
                       cluster_min_observed=3)
            config = directory / 'config.yaml'
            config.write_text(yaml.safe_dump(cfg))
            class Voter:
                def __init__(self, *args):
                    self.visible = np.array([3, 3, 3, 0])
                    self.vehicle = np.array([3, 3, 0, 0])
                    self.background = np.array([0, 0, 3, 0])
                    self.views = 0
                    self.last_stats = dict(projected=4, visible=3, occluded=1, vehicle_votes=2, objects=1)
                def add_view(self, *args):
                    self.views += 1
                    return 1
            yolo = types.SimpleNamespace(YOLO=lambda _: types.SimpleNamespace(task='segment'))
            argv = ['clean', '--map', tmp, '--bag', 'unused', '--config', str(config)]
            with patch.dict(sys.modules, {'ultralytics': yolo}), patch.object(sys, 'argv', argv), \
                    patch.object(vr, 'VehicleVoter', Voter), \
                    patch.object(vr, 'iterate_views', return_value=[(10, np.eye(4), None, None)]), \
                    patch.object(vr, 'estimate_floor', return_value=np.full(4, -10.)):
                vr.main()
            _, clean = vr.read_pcd(directory / 'global_clean.pcd')
            _, removed = vr.read_pcd(directory / 'global_vehicles.pcd')
            # Two voted vehicle voxels plus one unseen connected voxel are removed;
            # the well-observed background voxel in the SAME connected component survives.
            self.assertEqual(len(removed), 3)
            np.testing.assert_allclose(clean['x'], [.22])


if __name__ == '__main__':
    unittest.main()
