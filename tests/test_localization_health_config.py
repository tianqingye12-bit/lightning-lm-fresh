import importlib.util
from pathlib import Path
import tempfile
import unittest
import yaml

spec = importlib.util.spec_from_file_location('health_config', Path(__file__).resolve().parents[1] /
                                            'scripts/configure_localization_health.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ConfigTests(unittest.TestCase):
    def test_preserves_calibration_prediction_and_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = dict(common={'imu_topic': '/handsfree/imu'}, fasterlio={'extrinsic_T': [1, 2, 3],
                          'imu_prediction': {'max_horizon_s': .5}}, roi={}, lidar_loc={}, maps={}, system={})
            base = root/'base.yaml'
            base.write_text(yaml.safe_dump(config))
            original = base.read_bytes()
            map_path = root/'map'
            map_path.mkdir()
            output = root/'loc.yaml'
            with self.assertRaises(ValueError):
                module.configure(base, map_path, output)
            self.assertFalse(output.exists())
            (map_path/'index.txt').write_text('0 0 0\n')
            result = module.configure(base, map_path, output)
            self.assertEqual(result['fasterlio'], config['fasterlio'])
            self.assertEqual(result['common'], config['common'])
            self.assertEqual(result['system']['localization_health']['recovery_matches'], 3)
            self.assertEqual(base.read_bytes(), original)
            before = output.read_bytes()
            with self.assertRaises(FileExistsError):
                module.configure(base, map_path, output)
            self.assertEqual(output.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
