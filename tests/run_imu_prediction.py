#!/usr/bin/env python3
"""Build std-only time-boundary regression tests, without ROS/PCL."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--sanitize', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
flags = ['-fsanitize=undefined', '-fno-sanitize-recover=all'] if args.sanitize else []
with tempfile.TemporaryDirectory(prefix='imu-prediction-') as tmp:
    binary = str(Path(tmp) / 'imu_prediction')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O1', '-Wall', '-Wextra',
                    '-pedantic', *flags, '-I' + str(root / 'src'),
                    str(root / 'tests/imu_prediction.cc'), '-o', binary], check=True)
    subprocess.run([binary], check=True)
