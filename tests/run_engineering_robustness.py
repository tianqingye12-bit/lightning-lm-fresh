#!/usr/bin/env python3
"""Compile Linux filesystem/health fault tests without ROS or PCL."""
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
with tempfile.TemporaryDirectory(prefix='engineering-round9-') as tmp:
    for source in ['engineering_robustness', 'async_message_shutdown']:
        binary = str(Path(tmp) / source)
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O1', '-g', '-Wall', '-Wextra',
                        '-Werror', '-pedantic', '-pthread', *flags, '-I'+str(root/'src'),
                        str(root/'tests'/f'{source}.cc'), '-o', binary], check=True)
        subprocess.run([binary, tmp], check=True, timeout=30)
