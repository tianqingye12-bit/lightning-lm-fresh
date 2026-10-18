#!/usr/bin/env python3
"""Run std-only export/budget tests and check actual PGM/YAML outputs."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import yaml

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--sanitize', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
flags = ['-fsanitize=undefined', '-fno-sanitize-recover=all'] if args.sanitize else []
with tempfile.TemporaryDirectory(prefix='navigation-grid-') as tmp:
    binary = str(Path(tmp) / 'test')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O1', '-g', '-Wall', '-Wextra',
                    '-Werror', '-pedantic', *flags, '-I'+str(root / 'src'),
                    str(root / 'tests/navigation_grid.cc'), '-o', binary], check=True)
    subprocess.run([binary, tmp], check=True, timeout=15)
    for name, resolution in [('fine', .05), ('coarse', .2)]:
        path = Path(tmp) / name
        metadata = yaml.safe_load((path / 'map.yaml').read_text())
        assert metadata['resolution'] == resolution
        assert metadata['origin'] == [-1.25, 2.5, .3]
        assert (path / metadata['image']).read_bytes() == b'P5\n3 2\n255\n' + bytes([0,255,128,255,0,128])
    assert not (Path(tmp) / 'map.yaml').exists()
print('actual 5cm/20cm YAML scales, origin/yaw and PGM row orientation verified')
