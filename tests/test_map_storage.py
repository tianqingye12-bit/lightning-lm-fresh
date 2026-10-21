"""Compact pipeline regressions. Real PCL codec test runs only when pcl-tools is installed."""
import json
from pathlib import Path
import shutil
import sqlite3
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import map_storage as storage
import compact_saved_map as compact
import build_static_navigation as nav
import clean_map_vehicles as vr
import clean_map_keyframes as frames
import configure_navigation_detail as configure


class StorageTests(unittest.TestCase):
    def cloud(self,path,points=None):
        cloud=np.array(points or [(0,0,2,7,.125),(.1,0,2,9,.25)],dtype=nav.OUTPUT_DTYPE)
        path.write_bytes(nav.pcd_header(len(cloud))+cloud.tobytes())
        return cloud

    def source(self,root):
        source=root/'source'; source.mkdir()
        (source/'keyframe_clouds').mkdir()
        cloud=self.cloud(source/'keyframe_clouds/0.pcd')
        (source/'keyframe_clouds/index.csv').write_text('id,timestamp,file\n0,1,0.pcd\n')
        np.savetxt(source/'keyframes.txt',[[0,1,0,0,0,0,0,0,1]])
        (source/'map_frames.yaml').write_text(yaml.safe_dump(dict(format_version=2,
            keyframe_pose_frame='imu',keyframe_cloud_frame='imu',T_imu_lidar=np.eye(4).ravel().tolist())))
        (source/'summary.json').write_text(json.dumps(dict(source='retained_keyframes',global_clean_written=False)))
        return source,cloud

    def config(self):
        cfg=yaml.safe_load((ROOT/'config/m2dgr_clean_compact.yaml').read_text())
        cfg.update(width=32,height=32,K=[10,0,16,0,10,16,0,0,1],distortion=[0.]*5,
                   T_cam_lidar=np.eye(4).ravel().tolist(),_T_imu_lidar=np.eye(4).tolist(),
                   cache={'enabled':False},diagnostics={'max_overlays':0},rviz_max_points=1)
        cfg['storage']['compress_keyframes']=False  # Real codec covered by the optional test below.
        return cfg

    def test_compact_clean_no_world_spool_and_bounded_preview(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source,original=self.source(root)
            cfg=self.config(); args=SimpleNamespace(map=str(source),bag='unused',device='cpu',no_cache=True,rviz=True)
            image=np.zeros((32,32,3),np.uint8)
            views=[(t,np.eye(4),image,None) for t in [.8,1,1.2]]
            clouds={}
            overlays=[]
            class Publisher:
                def __init__(self,**kwargs): pass
                def publish_trajectory(self,*args): pass
                def publish_cloud(self,name,xyz): clouds[name]=xyz
                def publish_view(self,*args): overlays.append(args[0])
                def spin(self): pass
            modules={'ultralytics':SimpleNamespace(YOLO=lambda _:SimpleNamespace(task='segment')),
                     'clean_map_rviz':SimpleNamespace(RvizPublisher=Publisher)}
            with patch.dict(sys.modules,modules), patch.object(vr,'iterate_views',return_value=views), \
                    patch.object(vr.VehicleVoter,'infer_mask',return_value=(np.empty((0,32,32),bool),0)), \
                    patch.object(frames,'PcdStream',side_effect=AssertionError('compact must not spool world PCD')):
                frames.run_keyframe_clean(args,cfg,'imu_v2')
            run=next((source/'clean_diagnostics').glob('keyframes_*'))
            self.assertFalse((run/'global_clean.pcd').exists())
            self.assertFalse((run/'global_vehicles.pcd').exists())
            np.testing.assert_array_equal(vr.read_pcd(run/'keyframe_clouds/0.pcd')[1],original)
            self.assertLessEqual(len(clouds['clean']),1)
            self.assertLessEqual(len(clouds['raw']),1)
            self.assertGreater(len(overlays),0)
            self.assertEqual(len(clouds['vehicles']),0)
            report=json.loads((run/'summary.json').read_text())
            self.assertFalse(report['global_clean_written'])
            self.assertEqual(report['retained_points'],len(original))

    def test_navigation_build_from_frames_without_global_cloud(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source,cloud=self.source(root)
            cfg=self.config()
            cfg.update(system={'map_export':{'localization_resolution':.05}},g2p5=dict(
                grid_map_resolution=.05,min_th_floor=.3,max_th_floor=1.,floor_height=-1.,lidar_height=0.))
            config=root/'sensor.yaml';config.write_text(yaml.safe_dump(cfg))
            tool=root/'tool';tool.touch(); output=root/'nav'
            def render(*args,**kwargs):
                (output/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[0,0,0])))
                (output/'map.pgm').write_bytes(b'P5\n1 1\n255\n\x80')
            with patch.object(nav.subprocess,'run',side_effect=render):
                report=nav.build(source,config,output,tool,keep_global=False,compress_tiles=False)
            self.assertEqual(report['aggregation_source'],'cleaned_keyframes')
            self.assertFalse((output/'global.pcd').exists())
            rebuilt=vr.read_pcd(output/'0.pcd')[1]
            np.testing.assert_array_equal(rebuilt,cloud)
            self.assertTrue((output/'BUILD_COMPLETE.json').exists())

    def test_frame_aggregation_matches_world_export_with_rotated_imu_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source,cloud=self.source(root)
            q=np.array([0.,0.,np.sin(.3),np.cos(.3)])
            np.savetxt(source/'keyframes.txt',[[0,1,10,2,1,*q]])
            traj=vr.Trajectory(source/'keyframes.txt',2.)
            entries=frames.load_manifest(source,traj)
            expected=cloud.copy()
            xyz=np.column_stack([cloud[n] for n in ('x','y','z')]).astype(float)
            world=xyz@vr.quat_to_mat(q).T+np.array([10,2,1])
            for name,values in zip(('x','y','z'),world.T):expected[name]=values
            path=root/'world.pcd';path.write_bytes(nav.pcd_header(len(expected))+expected.tobytes())
            with sqlite3.connect(root/'a.db') as a,sqlite3.connect(root/'b.db') as b:
                nav.aggregate_tiles(path,a,.05)
                nav.aggregate_tiles(None,b,.05,blocks=nav.world_keyframe_blocks(entries,traj))
                self.assertEqual(a.execute('SELECT * FROM voxels').fetchall(),
                                 b.execute('SELECT * FROM voxels').fetchall())

    def test_tiles_identical_without_duplicated_global(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source=root/'input.pcd';self.cloud(source)
            full=root/'full';full.mkdir(); small=root/'small';small.mkdir()
            with sqlite3.connect(root/'db') as db:
                nav.aggregate_tiles(source,db,.05)
                nav.export_tiles(db,full,[0,0,0,0,0,0,1])
                nav.export_tiles(db,small,[0,0,0,0,0,0,1],keep_global=False)
            self.assertEqual((full/'0.pcd').read_bytes(),(small/'0.pcd').read_bytes())
            self.assertEqual((full/'index.txt').read_bytes(),(small/'index.txt').read_bytes())
            self.assertFalse((small/'global.pcd').exists())

    def test_mapping_compact_copy_retains_points_and_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source,cloud=self.source(root)
            (source/'global_clean.pcd').write_bytes(b'x'*100000)
            (source/'.vehicle_mask_cache').mkdir(); (source/'.vehicle_mask_cache/a').write_bytes(b'x'*5000)
            originals={str(p.relative_to(source)):p.read_bytes() for p in source.rglob('*') if p.is_file()}
            output=root/'copy'
            report=compact.compact(source,output,'mapping',compress=False)
            self.assertLess(report['copied_payload_bytes'],report['source_bytes'])
            self.assertFalse((output/'global_clean.pcd').exists())
            self.assertFalse((output/'.vehicle_mask_cache').exists())
            np.testing.assert_array_equal(vr.read_pcd(output/'keyframe_clouds/0.pcd')[1],cloud)
            for name,data in originals.items():self.assertEqual((source/name).read_bytes(),data)
            self.assertFalse(json.loads((output/'summary.json').read_text())['global_clean_written'])
            with self.assertRaises(FileExistsError):compact.compact(source,output,'mapping',False)

    def test_navigation_compact_preserves_tiles_dynamic_layer_and_rewrites_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'nav';source.mkdir()
            cloud=self.cloud(source/'0.pcd'); self.cloud(source/'0_dyn.pcd')
            (source/'index.txt').write_text('0 0 0\n0 0 0 old_absolute_path\n# functional points\nstart 0 0 0 0 0 0 1\n')
            (source/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05)))
            (source/'map.pgm').write_bytes(b'P5\n1 1\n255\n\x80')
            (source/'global.pcd').write_bytes(b'x'*10000)
            (source/'localization.yaml').write_text('system:\n  map_path: old\n')
            output=root/'copy'
            compact.compact(source,output,'navigation',compress=False)
            self.assertFalse((output/'global.pcd').exists())
            for name in ('0.pcd','0_dyn.pcd','index.txt','map.yaml','map.pgm'):
                self.assertEqual((source/name).read_bytes(),(output/name).read_bytes())
            config=yaml.safe_load((output/'localization.yaml').read_text())
            self.assertEqual(config['system']['map_path'],str(output.resolve())+'/')

    def test_compression_limits_and_failure_do_not_modify_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'cloud.pcd';self.cloud(path);before=path.read_bytes()
            with patch.object(storage,'require_compressor',side_effect=AssertionError('must not load large cloud')):
                self.assertEqual(storage.compress_pcd(path,max_bytes=1)['status'],'over_budget')
            def bad(command,**kwargs):
                out=Path(command[2]);out.write_bytes(before.replace(b'POINTS 2',b'POINTS 1').replace(b'DATA binary',b'DATA binary_compressed'))
            with patch.object(storage,'require_compressor',return_value='fake-pcl'), \
                    patch.object(storage.subprocess,'run',side_effect=bad):
                with self.assertRaisesRegex(ValueError,'changed fields/count'):storage.compress_pcd(path)
            self.assertEqual(path.read_bytes(),before)
            self.assertFalse(list(path.parent.glob('.pcd_compress_*')))

    def test_profiles_and_invalid_storage_options(self):
        self.assertTrue(storage.storage_options({})['write_global_clean'])
        for values in ({'compress_keyframes':'yes'},{'typo':True},[]):
            with self.assertRaises(ValueError):storage.storage_options({'storage':values})
        base={'fasterlio':{'imu_prediction':{'max_horizon_s':.5}},'system':{},'g2p5':{}}
        small=configure.configure(base)
        flags=small['system']['map_export']
        self.assertFalse(flags['save_global_pcd']);self.assertFalse(flags['save_localization_tiles'])
        self.assertTrue(flags['compress_keyframes'])
        self.assertEqual(small['fasterlio']['imu_prediction']['max_horizon_s'],.5)
        self.assertTrue(configure.configure(base,storage_profile='full')['system']['map_export']['save_global_pcd'])

    @unittest.skipUnless(shutil.which('pcl_convert_pcd_ascii_binary'),'PCL codec integration needs pcl-tools')
    def test_real_pcl_codec_preserves_every_field_and_value(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'cloud.pcd'
            cloud=np.zeros(10000,dtype=nav.OUTPUT_DTYPE)
            cloud['x']=np.arange(len(cloud),dtype=np.float32)*.01
            cloud['intensity']=7;cloud['time']=1.234567890123
            path.write_bytes(nav.pcd_header(len(cloud))+cloud.tobytes())
            result=storage.compress_pcd(path)
            self.assertEqual(result['status'],'compressed')
            np.testing.assert_array_equal(vr.read_pcd(path)[1],cloud)


if __name__ == '__main__':unittest.main()
