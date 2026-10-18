#!/usr/bin/env python3
"""Run loop validation/refresh scheduling tests without ROS, PCL or Eigen."""
import argparse
import os
from pathlib import Path
import subprocess
import shlex
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--backend', action='store_true', help='Also link/run real graph tests after a ROS build')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='loop-quality-') as tmp:
        binary = str(Path(tmp) / 'loop_quality')
        flags = ['-fsanitize=undefined', '-fno-sanitize-recover=all'] if args.sanitize else []
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O1', '-Wall', '-Wextra', '-pthread',
                        *flags, '-I' + str(root / 'src'), str(root / 'tests/loop_quality.cc'), '-o', binary],
                       check=True)
        subprocess.run([binary], check=True)
        if args.backend:
            build_flags = root / 'build/lightning/src/CMakeFiles/lightning.libs.dir/flags.make'
            opts = []
            for line in build_flags.read_text().splitlines():
                if line.startswith(('CXX_INCLUDES =', 'CXX_DEFINES =')):
                    opts += shlex.split(line.split('=', 1)[1])
            backend = str(Path(tmp) / 'loop_backend')
            subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O1', '-pthread', *opts,
                            str(root / 'tests/loop_backend.cc'),
                            '-L' + str(root / 'install/lightning/lib'),
                            '-Wl,-rpath,' + str(root / 'install/lightning/lib'),
                            '-llightning.libs', '-lglog', '-lfmt', '-o', backend], check=True)
            subprocess.run([backend], check=True)


if __name__ == '__main__':
    main()
