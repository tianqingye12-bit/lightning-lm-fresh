"""Synthetic coordinate/selection/cache/occlusion tests; no ROS or model download."""
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
from clean_map_runtime import MaskCache, ViewSelector, load_frame_contract, rigid_transform


class Round4Tests(unittest.TestCase):
    def config(self):
        cfg = yaml.safe_load((ROOT / 'config/m2dgr_clean.yaml').read_text())
        cfg.update(width=32, height=32, K=[10., 0., 16., 0., 10., 16., 0., 0., 1.],
                   distortion=[0.] * 5, T_cam_lidar=np.eye(4).ravel().tolist(), mask_dilate_px=0)
        return cfg

    def test_metadata_legacy_and_invalid(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertWarnsRegex(UserWarning, 'Legacy'):
                T, version = load_frame_contract(tmp)
            self.assertEqual(version, 'legacy')
            np.testing.assert_array_equal(T, np.eye(4))
            path = Path(tmp) / 'map_frames.yaml'
            record = dict(format_version=2, keyframe_pose_frame='imu', keyframe_cloud_frame='imu',
                          T_imu_lidar=np.eye(4).ravel().tolist())
            path.write_text(yaml.safe_dump(record))
            self.assertEqual(load_frame_contract(tmp)[1], 'imu_v2')
            record['format_version'] = 9
            path.write_text(yaml.safe_dump(record))
            with self.assertRaises(ValueError):
                load_frame_contract(tmp)
        bad = np.eye(4); bad[0, 0] = -1
        with self.assertRaises(ValueError):
            rigid_transform(bad, 'reflected')

    def test_selection_translation_rotation_heartbeat(self):
        selector = ViewSelector(dict(image_period=.2, view_selection=dict(
            translation_m=.1, rotation_deg=2, max_period_s=1)))
        T = np.eye(4)
        self.assertTrue(selector.accept(0, T))
        T[0, 3] = .2
        self.assertFalse(selector.accept(.1, T))  # time gate takes priority
        self.assertTrue(selector.accept(.3, T))
        self.assertFalse(selector.accept(.6, T))
        self.assertTrue(selector.accept(1.4, T))  # stationary heartbeat
        T[:3, :3] = vr.quat_to_mat([0, 0, np.sin(.05), np.cos(.05)])
        self.assertTrue(selector.accept(1.7, T))
        self.assertFalse(selector.accept(2, None))
        self.assertEqual(selector.stats, dict(selected=4, skipped_period=1, skipped_motion=1, invalid_pose=1))
        legacy = ViewSelector(dict(image_period=.2))
        self.assertTrue(legacy.accept(0, T))
        self.assertTrue(legacy.accept(.3, T))

    def test_cache_roundtrip_invalidation_and_corruption(self):
        with tempfile.TemporaryDirectory() as tmp:
            cache = MaskCache(tmp, dict(weights='a', conf=.25))
            image = np.zeros((3, 5, 3), dtype=np.uint8)
            mask = np.arange(15).reshape(3, 5) % 2 == 0
            key = cache.key(image)
            self.assertIsNone(cache.get(key, mask.shape))
            cache.put(key, mask, 2)
            restored, count = cache.get(key, mask.shape)
            np.testing.assert_array_equal(mask, restored)
            self.assertEqual(count, 2)
            image[0, 0, 0] = 1
            self.assertNotEqual(key, cache.key(image))
            other = MaskCache(tmp, dict(weights='b', conf=.25))
            image[:] = 0
            self.assertNotEqual(key, other.key(image))
            other = MaskCache(tmp, dict(weights='a', conf=.5))
            self.assertNotEqual(key, other.key(image))
            (Path(tmp) / key[:2] / (key + '.npz')).write_bytes(b'broken')
            self.assertIsNone(cache.get(key, mask.shape))
            self.assertEqual(cache.corrupt, 1)

    def test_cached_votes_equal_uncached_votes(self):
        cfg = self.config()
        points = np.array([[0., 0., 2.], [0., 0., 5.], [1., 0., 2.]])
        image = np.zeros((32, 32, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as tmp:
            cache = MaskCache(tmp, {'test_model': 1})
            voters = [vr.VehicleVoter(cfg, points, None, 'cpu') for _ in range(2)]
            for voter in voters:
                voter.cache = cache
            with patch.object(voters[0], 'infer_mask', return_value=(np.ones((32, 32), bool), 1)) as infer:
                voters[0].add_view(np.eye(4), image, None)
                infer.assert_called_once()
            with patch.object(voters[1], 'infer_mask', side_effect=AssertionError('cache missed')):
                voters[1].add_view(np.eye(4), image, None)
            np.testing.assert_array_equal(voters[0].vehicle, voters[1].vehicle)
            np.testing.assert_array_equal(voters[0].visible, voters[1].visible)
            self.assertEqual(cache.hits, 1)

    def test_nonidentity_extrinsics_and_camera_offset(self):
        cfg = self.config()
        # A camera far outside the LiDAR-centered search cube must still see its nearby point.
        T_cam_lidar = np.eye(4); T_cam_lidar[0, 3] = -30
        cfg['T_cam_lidar'] = T_cam_lidar.ravel().tolist()
        T_world_imu = np.eye(4)
        T_world_imu[:3, :3] = vr.quat_to_mat([0, 0, np.sin(.3), np.cos(.3)])
        T_world_imu[:3, 3] = [10, 20, 1]
        T_imu_lidar = np.eye(4); T_imu_lidar[:3, 3] = [.5, -.2, .7]
        T_imu_lidar[:3, :3] = vr.quat_to_mat([0, np.sin(.1), 0, np.cos(.1)])
        T_world_lidar = T_world_imu @ T_imu_lidar
        points_cam = np.array([[0, 0, 2, 1], [0, 0, 5, 1]])
        world = (T_world_lidar @ np.linalg.inv(T_cam_lidar) @ points_cam.T).T[:, :3]
        voter = vr.VehicleVoter(cfg, world, None, 'cpu')
        voter.vehicle_mask = lambda _: (np.ones((32, 32), bool), 1)
        voter.add_view(T_world_lidar, np.zeros((32, 32, 3), np.uint8), None)
        np.testing.assert_array_equal(voter.vehicle, [1, 0])
        self.assertEqual(voter.last_stats['occluded'], 1)

    def test_no_projected_points_does_not_infer(self):
        voter = vr.VehicleVoter(self.config(), np.array([[0., 0., -1.]]), None, 'cpu')
        with patch.object(voter, 'vehicle_mask', side_effect=AssertionError('should not infer')):
            self.assertIsNone(voter.add_view(np.eye(4), np.zeros((32, 32, 3), np.uint8), None))
        self.assertEqual(voter.views, 0)

    def test_chunked_pcd_preserves_fields_and_empty_output(self):
        cloud = np.array([(1, 2, 3, 7), (4, 5, 6, 8), (7, 8, 9, 9)],
                         dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
        header = ['VERSION .7', 'FIELDS x y z intensity', 'SIZE 4 4 4 4', 'TYPE F F F F',
                  'COUNT 1 1 1 1', 'WIDTH 3', 'HEIGHT 1', 'POINTS 3', 'DATA binary']
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp) / 'selected.pcd'
            vr.write_pcd(file, header, cloud, np.array([True, False, True]), chunk_points=1)
            _, result = vr.read_pcd(file)
            np.testing.assert_array_equal(result, cloud[[0, 2]])
            vr.write_pcd(file, header, cloud, np.zeros(3, bool), chunk_points=1)
            self.assertEqual(len(vr.read_pcd(file)[1]), 0)

    def test_reader_composes_extrinsic_after_interpolating_imu_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'keyframes.txt'
            np.savetxt(path, [[0, 0, 0, 0, 0, 0, 0, 0, 1],
                              [1, 2, 0, 0, 0, 0, 0, 1, 0]])
            traj = vr.Trajectory(path, 3)
            cfg = self.config()
            extrinsic = np.eye(4); extrinsic[0, 3] = 1
            cfg['_T_imu_lidar'] = extrinsic.tolist()
            connection = SimpleNamespace(topic=cfg['color_topic'], msgtype='image')
            message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
            class Reader:
                def __init__(self, *args, **kwargs): self.connections = [connection]
                def __enter__(self): return self
                def __exit__(self, *args): pass
                def messages(self, **kwargs): yield connection, 0, b'image'
                def deserialize(self, *args): return message
            modules = {'rosbags.highlevel': SimpleNamespace(AnyReader=Reader),
                       'rosbags.typesys': SimpleNamespace(Stores=SimpleNamespace(ROS2_HUMBLE=1),
                                                         get_typestore=lambda _: None)}
            with patch.dict(sys.modules, modules), patch.object(vr, 'decode_image', return_value='decoded'):
                views = list(vr.iterate_views('unused', cfg, traj))
            self.assertEqual(len(views), 1)
            np.testing.assert_allclose(views[0][1][:3, 3], [0, 1, 0], atol=1e-12)
            # Interpolating endpoint LiDAR positions would incorrectly give [0,0,0].


if __name__ == '__main__':
    unittest.main()
