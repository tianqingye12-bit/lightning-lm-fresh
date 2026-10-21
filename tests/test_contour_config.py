import copy
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'scripts/configure_contour_detail.py'
spec = importlib.util.spec_from_file_location('contour_config', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ContourConfigTests(unittest.TestCase):
    def baseline(self):
        return dict(common={'imu_topic':'/handsfree/imu'},
                    fasterlio={'filter_size_scan':.3, 'filter_size_map':.3, 'ivox_grid_resolution':.5,
                               'extrinsic_T':[.1,.2,.3], 'kf_dis_th':1.0,
                               'imu_prediction':{'max_horizon_s':.5},
                               'map_detail':{'local_fill':{'enabled':True,'translation':.25}},
                               'feature_quality':{'min_spread':.03}},
                    g2p5={'grid_map_resolution':.05}, system={'with_ui':True, 'with_loop_closing':True})

    def test_preserves_sensor_timing_and_map_limits(self):
        base = self.baseline()
        original = copy.deepcopy(base)
        cfg = module.configure(base)
        self.assertEqual(base, original)
        self.assertEqual(cfg['common'], base['common'])
        self.assertEqual(cfg['g2p5'], base['g2p5'])
        for key in ('extrinsic_T','filter_size_map','ivox_grid_resolution','kf_dis_th','imu_prediction'):
            self.assertEqual(cfg['fasterlio'][key], base['fasterlio'][key])
        self.assertEqual(cfg['fasterlio']['filter_size_scan'], .25)
        self.assertEqual(cfg['fasterlio']['map_detail']['local_fill'],base['fasterlio']['map_detail']['local_fill'])
        self.assertEqual(cfg['fasterlio']['map_detail']['keyframe_max_points'],60000)
        self.assertTrue(cfg['fasterlio']['map_detail']['preserve_observed_points'])
        self.assertFalse(cfg['system']['map_export']['save_global_pcd'])
        base['fasterlio']['filter_size_scan'] = .15
        self.assertEqual(module.configure(base)['fasterlio']['filter_size_scan'],.15)
        self.assertTrue(module.configure(base,storage_profile='full')['system']['map_export']['save_localization_tiles'])

    def test_invalid_and_output_protection(self):
        for value in (0, float('nan'), .01, 2):
            with self.assertRaises(ValueError):
                module.configure(self.baseline(),registration_resolution=value)
        with tempfile.TemporaryDirectory() as tmp:
            source, output = Path(tmp)/'base.yaml', Path(tmp)/'contour.yaml'
            source.write_text(yaml.safe_dump(self.baseline()))
            original = source.read_bytes()
            args = [sys.executable,str(SCRIPT),'--input',str(source),'--output',str(output)]
            subprocess.run(args,check=True,capture_output=True)
            before = output.read_bytes()
            self.assertNotEqual(subprocess.run(args,capture_output=True).returncode,0)
            self.assertEqual(output.read_bytes(),before)
            self.assertEqual(source.read_bytes(),original)
            args[-1] = str(source)
            self.assertNotEqual(subprocess.run(args,capture_output=True).returncode,0)
            self.assertEqual(source.read_bytes(),original)


if __name__ == '__main__':
    unittest.main()
