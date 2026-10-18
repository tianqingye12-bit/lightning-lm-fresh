import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/configure_map_detail.py'
spec = importlib.util.spec_from_file_location('configure_map_detail', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MapDetailConfigTest(unittest.TestCase):
    def test_sensor_settings_and_baseline_survive(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'baseline.yaml'
            output = Path(tmp) / 'detail.yaml'
            config = {'common': {'lidar_topic': '/velodyne_points', 'imu_topic': '/handsfree/imu'},
                      'fasterlio': {'point_filter_num': 4, 'filter_size_scan': 0.5,
                                    'filter_size_map': 0.3, 'ivox_grid_resolution': 0.5,
                                    'extrinsic_T': [0.1, 0.2, 0.3], 'kf_dis_th': 0.5,
                                    'feature_quality': {'enabled': True}},
                      'system': {'map_path': './data/test'}}
            original = yaml.safe_dump(config)
            source.write_text(original)
            subprocess.run([sys.executable, str(SCRIPT), '--input', str(source), '--output', str(output)],
                           check=True, capture_output=True)
            generated = yaml.safe_load(output.read_text())
            self.assertEqual(source.read_text(), original)
            self.assertEqual(generated['common'], config['common'])
            self.assertEqual(generated['system'], config['system'])
            for key in ('point_filter_num', 'filter_size_map', 'ivox_grid_resolution',
                        'extrinsic_T', 'kf_dis_th', 'feature_quality'):
                self.assertEqual(generated['fasterlio'][key], config['fasterlio'][key])
            detail = generated['fasterlio']['map_detail']
            self.assertEqual(generated['fasterlio']['filter_size_scan'], 0.3)
            self.assertGreater(detail['keyframe_resolution'], detail['export_resolution'])
            self.assertTrue(detail['local_fill']['enabled'])
            self.assertEqual(detail['keyframe_max_points'], 60000)
            previous = output.read_text()
            result = subprocess.run([sys.executable, str(SCRIPT), '--input', str(source), '--output', str(output)],
                                    capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(output.read_text(), previous)
            result = subprocess.run([sys.executable, str(SCRIPT), '--input', str(source), '--output', str(source)],
                                    capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(source.read_text(), original)

    def test_reject_non_sensor_config(self):
        for config in ({}, {'fasterlio': []}, {'fasterlio': 1}):
            with self.assertRaises(ValueError):
                module.configure(config)


if __name__ == '__main__':
    unittest.main()
