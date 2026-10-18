#!/usr/bin/env python3
"""Run after sourcing ROS2/install environments and building lightning.libs."""
from pathlib import Path
import shlex
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
flags = (root / 'build/lightning/src/CMakeFiles/lightning.libs.dir/flags.make').read_text().splitlines()
opts = []
for line in flags:
    if line.startswith(('CXX_INCLUDES =', 'CXX_DEFINES =')):
        opts += shlex.split(line.split('=', 1)[1])
with tempfile.TemporaryDirectory(prefix='lio-regression-') as tmp:
    for name in ('nav_state_prediction', 'imu_deskew', 'eskf_update_rejection', 'feature_quality'):
        binary = str(Path(tmp) / name)
        subprocess.run(['c++', '-std=c++17', '-O1', *opts, str(root / 'tests' / (name + '.cc')),
                        '-L' + str(root / 'install/lightning/lib'),
                        '-Wl,-rpath,' + str(root / 'install/lightning/lib'),
                        '-llightning.libs', '-lglog', '-lfmt', '-o', binary], check=True)
        subprocess.run([binary], check=True)

