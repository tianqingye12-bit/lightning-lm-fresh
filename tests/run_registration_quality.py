#!/usr/bin/env python3
"""Standalone registration tests; no ROS/PCL dependency."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--eigen-include', default='/usr/include/eigen3')
parser.add_argument('--sanitize', action='store_true')
args = parser.parse_args()
if not (Path(args.eigen_include)/'Eigen/Core').is_file():
    parser.error('Supply --eigen-include or install libeigen3-dev')
root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='registration-quality-') as temp:
    binary = str(Path(temp)/'registration_quality')
    flags = ['-fsanitize=undefined', '-fno-sanitize-recover=all'] if args.sanitize else []
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O1', '-Wall', '-Wextra', '-pedantic',
        *flags, '-I'+args.eigen_include, '-I'+str(root/'src'), str(root/'tests/registration_quality.cc'),
        '-o', binary], check=True)
    subprocess.run([binary], check=True)
