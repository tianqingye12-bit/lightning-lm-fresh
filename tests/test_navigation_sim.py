"""Offline geometry/export checks; ROS2/Nav2 integration needs a Humble host."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'simulation/lightning_nav_sim'))
from lightning_nav_sim.grid import Grid,read_pgm,load_grid,save_grid
from lightning_nav_sim.core import World,load_scenario,integrate
from lightning_nav_sim.maps import demo,pcd_blocks,project
from lightning_nav_sim.compare import compare
from lightning_nav_sim.configuration import nav2_parameters


class NavigationSimulationTests(unittest.TestCase):
    def test_grid_roundtrip_unknown_row_order_and_rotated_origin(self):
        cells=np.array([[-1,0,100],[100,-1,0]],dtype=np.int8)
        with tempfile.TemporaryDirectory() as tmp:
            grid=Grid(cells,.1,(-3.,2.,math.pi/3))
            path=save_grid(grid,tmp)
            loaded=load_grid(path)
            np.testing.assert_array_equal(loaded.cells,cells)
            self.assertEqual(loaded.fingerprint(),grid.fingerprint())
            np.testing.assert_array_equal(read_pgm(Path(tmp)/'map.pgm'),[[0,205,254],[205,254,0]])
            xy=np.array([[.25,.05],[.15,.05]])
            np.testing.assert_allclose(grid.local(grid.world(xy)),xy,atol=1e-14)
            np.testing.assert_array_equal(grid.blocked(grid.world(xy)),[True,False])

    def test_pgm_binary_header_does_not_consume_whitespace_or_hash_pixels(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'binary.pgm'
            path.write_bytes(b'P5\n# comment\n2 2\n255\n'+bytes([10,32,35,255]))
            np.testing.assert_array_equal(read_pgm(path),[[10,32],[35,255]])
            path.write_bytes(b'P2\n# hi\n2 2\n255\n0 254\n# row\n205 0\n')
            np.testing.assert_array_equal(read_pgm(path),[[0,254],[205,0]])
            path.write_bytes(b'P5\n2 2\n255\n\x00')
            with self.assertRaisesRegex(ValueError,'body length'): read_pgm(path)

    def test_grid_limits_and_circle_corner_collision(self):
        cells=np.zeros((10,10),dtype=np.int8); cells[5,5]=100
        grid=Grid(cells,1.,(0,0,0))
        self.assertTrue(grid.collides([4.9,4.9,0],.15))
        self.assertFalse(grid.collides([4.8,4.8,0],.15))
        self.assertTrue(grid.collides([.1,2,0],.2))
        cells[1,1]=-1
        self.assertTrue(grid.collides([1.5,1.5,0],.1))
        for resolution in (0,-.1,float('nan')):
            with self.assertRaises(ValueError): Grid(cells,resolution,(0,0,0))

    def test_integrator_straight_and_quarter_circle(self):
        np.testing.assert_allclose(integrate([1.,2.,0.],2.,0.,.5),[2,2,0])
        np.testing.assert_allclose(integrate([0.,0.,0.],1.,1.,math.pi/2),[1,1,math.pi/2],atol=1e-12)

    def world(self,tmp):
        path=demo(Path(tmp)/'demo')
        return World(load_scenario(path)),path

    def test_truth_world_independent_of_candidate_car_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            world,path=self.world(tmp)
            raw=load_grid(path.parent/'raw.yaml'); clean=load_grid(path.parent/'clean.yaml')
            self.assertTrue(raw.blocked([10.,2.5])); self.assertFalse(clean.blocked([10.,2.5]))
            self.assertFalse(world.grid.blocked([10.,2.5]))
            first=world.world_hash
            (path.parent/'raw.pgm').write_bytes(b'invalid candidate')
            other=World(load_scenario(path))
            self.assertEqual(other.world_hash,first)
            np.testing.assert_equal(world.scan()[1],other.scan()[1])

    def test_repeated_control_and_sensor_seed_are_reproducible(self):
        with tempfile.TemporaryDirectory() as tmp:
            a,path=self.world(tmp); b=World(load_scenario(path))
            for _ in range(50):
                a.step(.3,.15); b.step(.3,.15)
            np.testing.assert_allclose(a.pose,b.pose,atol=0)
            np.testing.assert_equal(a.scan()[1],b.scan()[1])
            self.assertNotEqual(a.odom[0],a.pose[0]-a.cfg['start'][0])

    def test_swept_collision_blocks_wall_and_counts_contact_episodes(self):
        with tempfile.TemporaryDirectory() as tmp:
            w,path=self.world(tmp)
            w.pose=np.array([5.7,6.,0.])
            for _ in range(100): w.step(1000.,0.)
            self.assertLess(w.pose[0]+w.cfg['robot']['radius'],6.)
            self.assertEqual(w.collisions,1)
            w.step(-.3,0.)
            self.assertFalse(w.contact)
            for _ in range(10): w.step(.3,0.)
            self.assertEqual(w.collisions,2)

    def test_analytical_vehicle_laser_hit_and_max_range_infinity(self):
        with tempfile.TemporaryDirectory() as tmp:
            w,path=self.world(tmp)
            w.cfg['obstacles']=[dict(center=[4.,2.5],size=[1.,1.],yaw=0.,
                motion=dict(axis='y',amplitude=0.,period=20.,phase=0.))]
            w.cfg['laser']['noise_std']=0.
            angles,ranges=w.scan()
            self.assertAlmostEqual(ranges[np.argmin(abs(angles))],1.5,places=8)
            self.assertTrue(np.isinf(ranges).any())
            self.assertTrue(w.collides([3.3,2.5,0.],0.))

    def test_invalid_scenario_and_start_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            w,path=self.world(tmp); original=yaml.safe_load(path.read_text())
            for change in ({'start':[0.,0.,0.]},{'goals':[]},{'laser':{'range':float('nan')}},
                           {'robot':{'dt':0.}},{'seed':-1}):
                cfg={**original,**change}; path.write_text(yaml.safe_dump(cfg))
                with self.assertRaises((ValueError,KeyError)): World(load_scenario(path))
            path.write_text(yaml.safe_dump(original))
            cfg=load_scenario(path); cfg['goals'][0][0]+=.1
            self.assertNotEqual(World(cfg).protocol_hash,w.protocol_hash)

    def binary_pcd(self,path,rows):
        dtype=np.dtype([('descriptor','<f4',(2,)),('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4')])
        points=np.zeros(len(rows),dtype=dtype)
        for k,axis in enumerate(('x','y','z')): points[axis]=np.asarray(rows)[:,k]
        header=('FIELDS descriptor x y z intensity\nSIZE 4 4 4 4 4\nTYPE F F F F F\n'
                f'COUNT 2 1 1 1 1\nPOINTS {len(rows)}\nDATA binary\n').encode()
        path.write_bytes(header+points.tobytes())

    def test_streaming_pcd_vector_fields_and_height_projection_unknown_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); pcd=root/'cloud.pcd'
            rows=[[.05,.05,0.],[.05,.05,.5],[.15,.05,0.],[.25,.05,2.5],
                  [20.,20.,0.],[float('nan'),0.,0.]]
            self.binary_pcd(pcd,rows)
            blocks=list(pcd_blocks(pcd,block_size=2))
            self.assertEqual([len(v) for v in blocks],[2,2,2])
            out=project(pcd,root/'projection',[0,0,1,1],0.)
            cells=load_grid(out).cells
            np.testing.assert_equal(cells[0,:3],[100,0,-1])
            report=json.loads((out.parent/'projection.json').read_text())
            self.assertEqual(report['input_points'],6); self.assertEqual(report['roi_points'],4)
            self.assertFalse((out.parent/'INCOMPLETE').exists())
            with self.assertRaises(FileExistsError): project(pcd,out.parent,[0,0,1,1],0.)

    def test_pcd_ascii_truncation_and_compression_rejection(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'ascii.pcd'
            header=b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 2\nDATA ascii\n'
            path.write_bytes(header+b'1 2 3\n4 5 6\n')
            np.testing.assert_equal(np.concatenate(list(pcd_blocks(path,1))),[[1,2,3],[4,5,6]])
            path.write_bytes(header+b'1 2 3\n')
            with self.assertRaisesRegex(ValueError,'count mismatch'): list(pcd_blocks(path))
            path.write_bytes(header.replace(b'ascii',b'binary_compressed'))
            with self.assertRaisesRegex(ValueError,'binary_compressed'): list(pcd_blocks(path))

    def test_failed_projection_has_incomplete_marker_and_keeps_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); path=root/'input.pcd'
            self.binary_pcd(path,[[.05,.05,0.]])
            before=path.read_bytes()
            with self.assertRaisesRegex(ValueError,'ground and obstacles'):
                project(path,root/'failed',[0,0,1,1],0.)
            self.assertTrue((root/'failed/INCOMPLETE').exists())
            self.assertEqual(before,path.read_bytes())
            with self.assertRaisesRegex(ValueError,'8 million'):
                project(path,root/'oversized',[0,0,1000,1000],0.,.02)
            self.assertFalse((root/'oversized').exists())

    def report(self):
        return dict(complete=True,scope='2d_navigation_map_evaluation_amcl',world_hash='world',
                    protocol_hash='protocol',navigation_config_hash='nav2',goals_planned=3,
                    goal_timeout_s=180.,wall_timeout_s=900.,truth_tolerance_m=.35,
                    truth_yaw_tolerance_rad=.4,navigation_map_hash='raw',success=False)

    def test_ab_comparison_checks_world_protocol_configuration_and_completion(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=Path(tmp)/'a'; a.mkdir(); b=Path(tmp)/'b'; b.mkdir()
            first=self.report(); second={**first,'navigation_map_hash':'clean'}
            (a/'summary.json').write_text(json.dumps(first))
            (b/'summary.json').write_text(json.dumps(second))
            self.assertFalse(compare(a,b)['same_navigation_map'])
            for key in ('world_hash','protocol_hash','navigation_config_hash','goal_timeout_s','complete'):
                altered={**second,key:False}
                (b/'summary.json').write_text(json.dumps(altered))
                with self.assertRaises(ValueError): compare(a,b)

    def test_nav2_overrides_preserve_installed_plugin_inventory(self):
        with tempfile.TemporaryDirectory() as tmp:
            w,path=self.world(tmp)
            defaults=dict(amcl={'ros__parameters':{}},map_server={'ros__parameters':{}},
                bt_navigator={'ros__parameters':{'plugin_lib_names':['installed_humble_plugin']}},
                controller_server={'ros__parameters':dict(progress_checker={},general_goal_checker={},FollowPath={})},
                planner_server={'ros__parameters':{'GridBased':{}}})
            for key in ('local_costmap','global_costmap'):
                defaults[key]={key:{'ros__parameters':{'inflation_layer':{},'footprint':'remove this'}}}
            before=copy.deepcopy(defaults)
            p=nav2_parameters(defaults,w.cfg,'/candidate/map.yaml')
            self.assertEqual(defaults,before)
            self.assertEqual(p['bt_navigator']['ros__parameters']['plugin_lib_names'],['installed_humble_plugin'])
            self.assertEqual(p['amcl']['ros__parameters']['base_frame_id'],'base_link')
            self.assertEqual(p['amcl']['ros__parameters']['initial_pose']['x'],2.)
            self.assertFalse(p['planner_server']['ros__parameters']['GridBased']['allow_unknown'])
            self.assertTrue(p['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']['inf_is_valid'])


if __name__=='__main__': unittest.main()
