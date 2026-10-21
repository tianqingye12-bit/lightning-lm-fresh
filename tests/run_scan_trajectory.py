#!/usr/bin/env python3
"""Build geometry-only recorder tests and verify actual CSV output with the Python reader."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--eigen-include',default='/usr/include/eigen3')
parser.add_argument('--sanitize',action='store_true')
args=parser.parse_args()
if not (Path(args.eigen_include)/'Eigen/Core').is_file():parser.error('Supply --eigen-include')
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='scan-trajectory-') as temp:
    binary=str(Path(temp)/'scan_trajectory')
    flags=['-fsanitize=undefined','-fno-sanitize-recover=all'] if args.sanitize else []
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-O1','-Wall','-Wextra','-pedantic',*flags,
        '-I'+args.eigen_include,'-I'+str(root/'src'),str(root/'tests/scan_trajectory.cc'),'-o',binary],check=True)
    directory=Path(temp)/'export'
    subprocess.run([binary,str(directory)],check=True)
    subprocess.run([sys.executable,str(root/'tests/verify_scan_export.py'),str(directory)],check=True)
