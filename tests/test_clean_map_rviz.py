"""Exercise RViz serialization without requiring a ROS installation."""
import ast
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('clean_map_vehicles', ROOT / 'scripts/clean_map_vehicles.py')
vehicles = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vehicles)


def load_function(name, namespace):
    """Load the actual function body while leaving ROS imports to test doubles."""
    source = ast.parse((ROOT / 'scripts/clean_map_rviz.py').read_text())
    candidates = list(source.body)
    for node in source.body:
        if isinstance(node, ast.ClassDef):
            candidates.extend(node.body)
    function = next(n for n in candidates if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<rviz function>', 'exec'), namespace)
    return namespace[name]


class RvizRegressionTests(unittest.TestCase):
    def test_downsampling_preserves_point_rows_and_matching_intensity(self):
        namespace = dict(np=np, PointField=Mock(FLOAT32=7), PointCloud2=SimpleNamespace,
                         rclpy=Mock())
        publish = load_function('publish_cloud', namespace)
        publisher = SimpleNamespace(header=lambda: None, node=Mock(), clouds={'raw': Mock()})
        # One duplicate voxel; coordinate values are deliberately all distinct.
        xyz = np.array([[1.001, 2.001, 3.001], [1.002, 2.002, 3.002],
                        [4.001, 5.001, 6.001], [7.001, 8.001, 9.001]])
        intensity = np.array([10., 20., 30., 40.])
        with patch.dict(sys.modules, {'clean_map_vehicles': vehicles}):
            publish(publisher, 'raw', xyz, intensity, max_points=3)
        msg = publisher.clouds['raw'].publish.call_args.args[0]
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        expected = np.column_stack([xyz[[0, 2, 3]], intensity[[0, 2, 3]]])
        np.testing.assert_allclose(points, expected, rtol=1e-6)
        self.assertEqual(msg.width, 3)
        self.assertEqual(msg.row_step, len(msg.data))

    def test_standalone_viewer_accepts_exported_keyframes(self):
        import argparse
        viewer = Mock()
        main = load_function('main', dict(argparse=argparse, Path=Path, np=np,
                                         RvizPublisher=lambda: viewer))
        with tempfile.TemporaryDirectory() as tmp:
            np.savetxt(Path(tmp) / 'keyframes.txt', [[0, 10, 0, 0, 0, 0, 0, 0, 1]])
            with patch.dict(sys.modules, {'clean_map_vehicles': vehicles}), \
                    patch.object(sys, 'argv', ['viewer', '--map', tmp]):
                main()
        trajectory = viewer.publish_trajectory.call_args.args[0]
        np.testing.assert_allclose(trajectory.pose(10), np.eye(4))
        viewer.spin.assert_called_once()


if __name__ == '__main__':
    unittest.main()
