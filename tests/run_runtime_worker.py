#!/usr/bin/env python3
"""Compile thread/queue/display policy tests without ROS/PCL/OpenGL."""
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
with tempfile.TemporaryDirectory(prefix='runtime-worker-') as tmp:
    binary = str(Path(tmp) / 'runtime_worker')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O1', '-g', '-Wall', '-Wextra',
                    '-pedantic', '-pthread', *flags, '-I' + str(root / 'src'),
                    str(root / 'tests/runtime_worker.cc'), '-o', binary], check=True)
    subprocess.run([binary], check=True, timeout=15)
