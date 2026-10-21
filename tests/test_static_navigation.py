"""Static map rebuilding tests; compiled ROS/PCL grid renderer is mocked explicitly."""
import copy
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import build_static_navigation as nav
import configure_navigation_detail as fine
import clean_map_vehicles as vr


class StaticNavigationTests(unittest.TestCase):
    def config(self):
        return {'common': {'imu_topic': '/imu'},
                'fasterlio': {'filter_size_scan': .3, 'filter_size_map': .3,
                             'extrinsic_T': [.1,.2,.3], 'kf_dis_th': .5,
                             'imu_prediction': {'max_horizon_s': .5}},
                'system': {'with_g2p5': False, 'map_path': './original'},
                'g2p5': {'grid_map_resolution': .2, 'esti_floor': False, 'floor_height': -1.2,
                         'min_th_floor': .3, 'max_th_floor': 1., 'lidar_height': 0.}}

    def test_fine_config_preserves_validated_settings_and_input(self):
        original = self.config(); snapshot = copy.deepcopy(original)
        config = fine.configure(original)
        self.assertEqual(original, snapshot)
        self.assertEqual(config['common'], original['common'])
        for name in ('extrinsic_T','kf_dis_th','filter_size_scan','filter_size_map','imu_prediction'):
            self.assertEqual(config['fasterlio'][name], original['fasterlio'][name])
        self.assertEqual(config['fasterlio']['map_detail']['keyframe_resolution'], .05)
        self.assertEqual(config['fasterlio']['map_detail']['keyframe_max_points'], 60000)
        self.assertEqual(config['system']['map_export']['localization_resolution'], .05)
        self.assertFalse(config['system']['with_g2p5'])
        self.assertTrue(config['system']['export_keyframe_clouds'])
        for key in ('floor_height', 'min_th_floor', 'max_th_floor', 'lidar_height'):
            self.assertEqual(config['g2p5'][key], original['g2p5'][key])
        self.assertEqual(fine.configure(original,.03)['fasterlio']['map_detail']['export_resolution'], .03)
        for cfg in ({}, {'fasterlio': []}):
            with self.assertRaises(ValueError): fine.configure(cfg)
        with self.assertRaises(ValueError): fine.configure(original,.01)

    def write_cloud(self, path, rows):
        cloud = np.array(rows, dtype=nav.OUTPUT_DTYPE)
        path.write_bytes(nav.pcd_header(len(cloud))+cloud.tobytes())
        return cloud

    def test_disk_voxels_cross_block_means_negative_tile_boundaries_and_first_points(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            source = directory / 'source.pcd'
            self.write_cloud(source, [(0.001,0,1,2,10),(.009,0,1,4,20),
                (-50.001,0,1,5,0),(-50,0,1,6,0),(49.999,0,1,7,0),(50,0,1,8,0),
                (150,0,1,9,0),(float('nan'),0,1,0,0)])
            output = directory / 'out'; output.mkdir()
            reader = nav.binary_blocks
            with sqlite3.connect(directory / 'db.sqlite') as db, \
                    patch.object(nav, 'binary_blocks', side_effect=lambda p: reader(p,block_points=1)):
                stats = nav.aggregate_tiles(source, db, .05)
                stats.update(nav.export_tiles(db, output, [1,2,3,0,0,0,1]))
            self.assertEqual(stats['input_points'],7)
            self.assertEqual(stats['invalid_points'],1)
            self.assertEqual(stats['output_points'],6)
            _, cloud = vr.read_pcd(output / 'global.pcd')
            origin = cloud[np.abs(cloud['x']) < .01]
            np.testing.assert_allclose(origin['x'], [.005])
            np.testing.assert_allclose(origin['intensity'], [3])
            np.testing.assert_allclose(origin['time'], [15])
            lines = (output / 'index.txt').read_text().splitlines()
            self.assertEqual(lines[0], '0 0 0')
            self.assertEqual(lines[-1], 'start 1 2 3 0 0 0 1')
            tile_rows = [line.split() for line in lines[1:] if line[0].isdigit()]
            self.assertEqual([int(row[1]) for row in tile_rows], [-1,0,1,2])
            for tile_id, tx, ty, filename in tile_rows:
                self.assertEqual(filename, tile_id+'.pcd')
                _, points = vr.read_pcd(output / filename)
                self.assertGreater(len(points),0)  # Single-point tiles must not be lost.
                np.testing.assert_array_equal(np.floor(points['x']/100+.5), int(tx))

    def make_source(self, root):
        source = root / 'clean source'; source.mkdir()
        self.write_cloud(source / 'global_clean.pcd', [(0,0,1,3,0),(1,0,1,5,0)])
        frames = source / 'keyframe_clouds'; frames.mkdir()
        self.write_cloud(frames / '0.pcd', [(0,0,1,3,0)])
        (frames / 'index.csv').write_text('id,timestamp,file\n0,1,0.pcd\n')
        np.savetxt(source / 'keyframes.txt', [[0,1,0,0,0,0,0,0,1]])
        (source / 'map_frames.yaml').write_text(yaml.safe_dump(dict(format_version=2,
            keyframe_pose_frame='imu',keyframe_cloud_frame='imu',T_imu_lidar=np.eye(4).ravel().tolist())))
        (source / 'summary.json').write_text(json.dumps({'source':'retained_keyframes'}))
        return source

    def test_full_packaging_config_and_source_preservation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source=self.make_source(root)
            original={str(p.relative_to(source)):p.read_bytes() for p in source.rglob('*') if p.is_file()}
            config=root/'sensor.yaml'; config.write_text(yaml.safe_dump(fine.configure(self.config())))
            output=root/'nav map'
            tool=root/'grid_tool'; tool.touch()
            def renderer(command, **kwargs):
                self.assertTrue(kwargs['check'])
                frame_arg=next(x.split('=',1)[1] for x in command if x.startswith('--input_frames='))
                self.assertIn('"'+str(source),Path(frame_arg).read_text())
                (output/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[0,0,0])))
                (output/'map.pgm').write_bytes(b'P5\n1 1\n255\n\x80')
            with patch.object(nav.subprocess, 'run', side_effect=renderer):
                stats=nav.build(source, config, output, tool)
            self.assertEqual(stats['output_points'],2)
            self.assertTrue((output/'BUILD_COMPLETE.json').is_file())
            self.assertFalse((output/'INCOMPLETE').exists())
            generated=yaml.safe_load((output/'localization.yaml').read_text())
            self.assertEqual(generated['system']['map_path'],str(output.resolve())+'/')
            self.assertFalse(generated['maps']['load_dyn_cloud'])
            self.assertEqual(generated['fasterlio']['imu_prediction']['max_horizon_s'],.5)
            for name,data in original.items(): self.assertEqual((source/name).read_bytes(),data)
            with self.assertRaises(FileExistsError): nav.build(source,config,output,tool)

    def test_failed_renderer_cannot_publish_complete_map_and_old_mode_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source=self.make_source(root)
            config=root/'sensor.yaml'; config.write_text(yaml.safe_dump(fine.configure(self.config())))
            tool=root/'tool'; tool.touch(); output=root/'failed'
            with patch.object(nav.subprocess,'run',side_effect=RuntimeError('renderer failed')):
                with self.assertRaisesRegex(RuntimeError,'renderer failed'): nav.build(source,config,output,tool)
            self.assertTrue((output/'INCOMPLETE').exists())
            self.assertFalse((output/'BUILD_COMPLETE.json').exists())
            (source/'summary.json').write_text(json.dumps({'source':'map'}))
            with self.assertRaisesRegex(ValueError,'round-6'): nav.build(source,config,root/'new',tool)

    def test_truncated_and_compressed_pcd_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'bad.pcd'; path.write_bytes(nav.pcd_header(2))
            with self.assertRaisesRegex(ValueError,'body length'): list(nav.binary_blocks(path))
            path.write_bytes(nav.pcd_header(0).replace(b'DATA binary', b'DATA binary_compressed'))
            with self.assertRaisesRegex(ValueError,'uncompressed'): list(nav.binary_blocks(path))


if __name__ == '__main__': unittest.main()
