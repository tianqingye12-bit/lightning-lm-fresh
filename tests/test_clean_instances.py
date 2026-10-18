"""Round 6 instance/depth/vote regressions without ROS or model downloads."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import clean_map_vehicles as vr
from clean_map_instances import InstanceMaskCache, associate_instances, removal_votes
from clean_map_keyframes import load_manifest, run_keyframe_clean


class InstanceTests(unittest.TestCase):
    def config(self):
        cfg = yaml.safe_load((ROOT / 'config/m2dgr_clean_instances.yaml').read_text())
        cfg.update(width=32, height=32, K=[10., 0., 16., 0., 10., 16., 0., 0., 1.],
                   distortion=[0.] * 5, T_cam_lidar=np.eye(4).ravel().tolist(),
                   mask_dilate_px=1, diagnostics={'max_overlays': 0}, cache={'enabled': False})
        return cfg

    def test_instances_keep_separate_depth_bands_and_background(self):
        cfg = self.config()
        masks = np.zeros((2, 32, 32), bool)
        masks[0, 8:12, 4:10] = True
        masks[1, 8:12, 10:16] = True
        scene = np.full((32, 32), np.inf)
        scene[8:12, 4:10] = 2
        scene[8:12, 10:16] = 10
        # Full supported strips + window/background points deep inside each mask,
        # plus a dilation-edge point at the WRONG depth of the neighboring car.
        u = np.array([5, 6, 7, 11, 12, 13, 8, 14, 9, 20])
        v = np.full(len(u), 9)
        z = np.array([2, 2, 2, 10, 10, 10, 8, 16, 10, 2])
        scene[9, 8] = 8
        scene[9, 14] = 16
        visible = np.ones(len(u), bool)
        vehicle, background, union = associate_instances(masks, scene, u, v, z, visible, cfg)
        np.testing.assert_array_equal(vehicle, [1, 1, 1, 1, 1, 1, 0, 0, 0, 0])
        np.testing.assert_array_equal(background, [0, 0, 0, 0, 0, 0, 0, 0, 0, 1])
        self.assertEqual(int(union.sum()), 48)

    def test_missing_depth_and_boundary_abstain(self):
        cfg = self.config()
        masks = np.zeros((1, 32, 32), bool)
        masks[0, 10:12, 10:12] = True
        scene = np.full((32, 32), np.inf)
        scene[10, 10] = 2  # Only one supported pixel, even if many duplicate points exist.
        vehicle, background, _ = associate_instances(masks, scene,
            np.array([10, 10, 9, 20]), np.array([10, 10, 10, 20]),
            np.full(4, 2), np.ones(4, bool), cfg)
        self.assertFalse(vehicle.any())
        np.testing.assert_array_equal(background, [0, 0, 0, 1])

    def test_multiview_minimum_and_background_veto(self):
        cfg = self.config()
        observed, remove = removal_votes(np.array([10, 3, 6, 3]), np.array([1, 3, 3, 0]),
                                         np.array([0, 0, 3, 3]), cfg)
        self.assertTrue(observed.all())
        np.testing.assert_array_equal(remove, [0, 1, 0, 0])

    def test_cache_preserves_overlapping_instances_and_empty(self):
        with tempfile.TemporaryDirectory() as tmp:
            cache = InstanceMaskCache(tmp, {'schema': 2})
            image = np.zeros((32, 32, 3), np.uint8)
            masks = np.zeros((2, 32, 32), bool)
            masks[0, 4:10, 4:10] = True
            masks[1, 6:12, 6:12] = True
            key = cache.key(image)
            cache.put(key, masks, 2)
            restored, count = cache.get(key, (32, 32))
            self.assertEqual(count, 2)
            np.testing.assert_array_equal(restored, masks)
            cache.put(key, np.empty((0, 32, 32), bool), 0)
            self.assertEqual(cache.get(key, (32, 32))[0].shape, (0, 32, 32))
            (Path(tmp) / key[:2] / (key + '.npz')).write_bytes(b'bad')
            self.assertIsNone(cache.get(key, (32, 32)))

    def test_actual_inference_path_uses_masks_and_rejects_boxes_only(self):
        cfg = self.config()
        masks = np.zeros((2, 32, 32), np.float32)
        masks[0, 4:8, 4:8] = 1
        masks[1, 6:10, 6:10] = 1
        result = SimpleNamespace(masks=SimpleNamespace(data=SimpleNamespace(
            cpu=lambda: SimpleNamespace(numpy=lambda: masks))), boxes=[1, 2])
        calls = []
        def model(*args, **kwargs):
            calls.append(kwargs)
            return [result]
        voter = vr.VehicleVoter(cfg, np.empty((0, 3)), model, 'cpu')
        actual, count = voter.infer_mask(np.zeros((32, 32, 3), np.uint8))
        self.assertEqual(count, 2)
        np.testing.assert_array_equal(actual, masks.astype(bool))
        self.assertTrue(calls[0]['retina_masks'])
        result.masks = None
        with self.assertRaisesRegex(RuntimeError, 'no segmentation'):
            voter.infer_mask(np.zeros((32, 32, 3), np.uint8))
        result.boxes = []
        self.assertEqual(voter.infer_mask(np.zeros((32, 32, 3), np.uint8))[0].shape, (0, 32, 32))

    def test_keyframe_rebuild_preserves_unknown_points_fields_and_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            frames = directory / 'keyframe_clouds'
            frames.mkdir()
            header = ['VERSION .7', 'FIELDS x y z intensity', 'SIZE 4 4 4 4',
                      'TYPE F F F F', 'COUNT 1 1 1 1', 'WIDTH 2', 'HEIGHT 1',
                      'POINTS 2', 'DATA binary']
            cloud = np.array([(0, 0, 2, 7), (.5, 0, 2, 9)],
                             dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
            for i in range(2):
                vr.write_pcd(frames / f'{i}.pcd', header, cloud)
            originals = [(frames / f'{i}.pcd').read_bytes() for i in range(2)]
            np.savetxt(directory / 'keyframes.txt', [[0, 1, 10, 0, 0, 0, 0, 0, 1],
                                                    [1, 3, 20, 0, 0, 0, 0, 0, 1]])
            (frames / 'index.csv').write_text('id,timestamp,file\n0,1,0.pcd\n1,3,1.pcd\n')
            (directory / 'map_frames.yaml').write_text('format_version: 2\n')
            cfg = self.config()
            cfg['_T_imu_lidar'] = np.eye(4).tolist()
            # Three views apply only to frame 0; frame 1 has no image evidence and must survive.
            pose = np.eye(4); pose[0, 3] = 10
            views = [(t, pose, np.zeros((32, 32, 3), np.uint8), None) for t in [.8, 1, 1.2]]
            model = SimpleNamespace(task='segment')
            modules = {'ultralytics': SimpleNamespace(YOLO=lambda _: model)}
            args = SimpleNamespace(map=tmp, bag='unused', device='cpu', no_cache=True, rviz=False)
            # Mock only inference and floor; run real projection, association, votes and PCD rebuilding.
            cfg['instance_filter']['min_depth_pixels'] = 1
            masks = np.zeros((1, 32, 32), bool); masks[0, 16, 16] = True
            with patch.dict(sys.modules, modules), patch.object(vr, 'iterate_views', return_value=views), \
                    patch.object(vr.VehicleVoter, 'infer_mask', return_value=(masks, 1)) as infer, \
                    patch.object(vr, 'estimate_floor', return_value=np.array([-1., -1.])):
                run_keyframe_clean(args, cfg, 'imu_v2')
                self.assertEqual(infer.call_count, 3)
            run = next((directory / 'clean_diagnostics').glob('keyframes_*'))
            _, clean = vr.read_pcd(run / 'global_clean.pcd')
            _, removed = vr.read_pcd(run / 'global_vehicles.pcd')
            self.assertEqual(len(clean), 3)
            self.assertEqual(len(removed), 1)
            np.testing.assert_allclose(removed['x'], [10])
            np.testing.assert_allclose(removed['intensity'], [7])
            np.testing.assert_allclose(np.sort(clean['x']), [10.5, 20, 20.5])
            for i in range(2):
                self.assertEqual((frames / f'{i}.pcd').read_bytes(), originals[i])
            _, retained = vr.read_pcd(run / 'keyframe_clouds/0.pcd')
            np.testing.assert_allclose(retained['x'], [.5])  # still in IMU coordinates
            self.assertTrue((run / 'summary.json').is_file())

    def test_overlapping_keyframes_share_inference_and_bound_admission(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            frames = directory / 'keyframe_clouds'; frames.mkdir()
            header = ['VERSION .7', 'FIELDS x y z', 'SIZE 4 4 4', 'TYPE F F F',
                      'COUNT 1 1 1', 'WIDTH 1', 'HEIGHT 1', 'POINTS 1', 'DATA binary']
            cloud = np.array([(0, 0, 2)], dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
            for i in range(2):
                vr.write_pcd(frames / f'{i}.pcd', header, cloud)
            np.savetxt(directory / 'keyframes.txt', [[0, 1, 0, 0, 0, 0, 0, 0, 1],
                                                    [1, 1.1, 0, 0, 0, 0, 0, 0, 1]])
            (frames / 'index.csv').write_text('id,timestamp,file\n0,1,0.pcd\n1,1.1,1.pcd\n')
            (directory / 'map_frames.yaml').write_text('format_version: 2\n')
            args = SimpleNamespace(map=tmp, bag='unused', device='cpu', no_cache=True, rviz=False)
            cfg = self.config()
            image = np.zeros((32, 32, 3), np.uint8)
            views = [(1.05, np.eye(4), image, None)]
            model = SimpleNamespace(task='segment')
            with patch.dict(sys.modules, {'ultralytics': SimpleNamespace(YOLO=lambda _: model)}), \
                    patch.object(vr, 'iterate_views', side_effect=lambda *a: iter(views)), \
                    patch.object(vr.VehicleVoter, 'infer_mask', return_value=(np.zeros((0, 32, 32), bool), 0)) as infer:
                cfg['keyframe_clean']['max_active_frames'] = 1
                with self.assertRaisesRegex(RuntimeError, 'simultaneous keyframes'):
                    run_keyframe_clean(args, cfg, 'imu_v2')
                infer.assert_not_called()
                cfg['keyframe_clean']['max_active_frames'] = 2
                run_keyframe_clean(args, cfg, 'imu_v2')
                self.assertEqual(infer.call_count, 1)
            successful = [p for p in (directory / 'clean_diagnostics').glob('keyframes_*')
                          if (p / 'summary.json').is_file()]
            self.assertEqual(len(successful), 1)
            self.assertEqual(len(vr.read_pcd(successful[0] / 'global_clean.pcd')[1]), 2)
            self.assertEqual(len(vr.read_pcd(successful[0] / 'global_vehicles.pcd')[1]), 0)

    def test_manifest_missing_mismatch_and_traversal(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            traj = SimpleNamespace(t=np.array([1.]), ids=np.array([0]))
            with self.assertRaisesRegex(RuntimeError, 'Missing keyframe'):
                load_manifest(directory, traj)
            frames = directory / 'keyframe_clouds'; frames.mkdir()
            (directory / 'outside.pcd').write_bytes(b'bad')
            (frames / 'index.csv').write_text('id,timestamp,file\n0,1,../outside.pcd\n')
            with self.assertRaisesRegex(ValueError, 'unsafe'):
                load_manifest(directory, traj)


if __name__ == '__main__':
    unittest.main()
