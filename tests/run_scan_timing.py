#!/usr/bin/env python3
"""Compile the point-time normalization regression without ROS2/PCL."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--sanitize',action='store_true')
args=p.parse_args();root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='scan-timing-') as tmp:
    binary=str(Path(tmp)/'test')
    flags=['-fsanitize=undefined','-fno-sanitize-recover=all'] if args.sanitize else []
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-O1','-Wall','-Wextra','-pedantic',
                    *flags,'-I'+str(root/'src'),str(root/'tests/scan_timing.cc'),'-o',binary],check=True)
    subprocess.run([binary],check=True)
