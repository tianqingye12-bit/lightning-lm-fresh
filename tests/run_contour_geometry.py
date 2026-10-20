#!/usr/bin/env python3
"""Run contour policy tests; --with-pcl also checks production cloud adapters."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--sanitize', action='store_true')
parser.add_argument('--with-pcl', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
sanitize = ['-fsanitize=undefined', '-fno-sanitize-recover=all'] if args.sanitize else []
with tempfile.TemporaryDirectory(prefix='contour-round10-') as tmp:
    for name in ['contour_geometry'] + (['contour_cloud'] if args.with_pcl else []):
        flags = shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','pcl_filters'], text=True)) if name == 'contour_cloud' else []
        binary = str(Path(tmp)/name)
        subprocess.run([os.environ.get('CXX','c++'), '-std=c++17','-O1','-g','-Wall','-Wextra','-Werror',
                        '-pedantic',*sanitize,'-I'+str(root/'src'),str(root/'tests'/f'{name}.cc'),
                        '-o',binary,*flags],check=True)
        subprocess.run([binary],check=True,timeout=30)
