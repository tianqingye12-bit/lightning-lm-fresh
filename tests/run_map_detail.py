#!/usr/bin/env python3
"""Run map detail policy tests; --with-pcl also exercises the production cloud filter."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--with-pcl', action='store_true')
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    sanitizer = ['-fsanitize=undefined', '-fno-sanitize-recover=all'] if args.sanitize else []
    with tempfile.TemporaryDirectory(prefix='map-detail-') as tmp:
        names = ['map_detail'] + (['map_detail_cloud'] if args.with_pcl else [])
        for name in names:
            flags = []
            if name == 'map_detail_cloud':
                flags = shlex.split(subprocess.check_output(
                    ['pkg-config', '--cflags', '--libs', 'pcl_filters'], text=True))
            binary = str(Path(tmp) / name)
            subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O1', '-Wall', '-Wextra',
                            *sanitizer, '-I' + str(root / 'src'),
                            str(root / 'tests' / (name + '.cc')), '-o', binary, *flags], check=True)
            subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
