"""PCD storage helpers. Compression uses PCL's native lossless codec, never a voxel filter."""
from pathlib import Path
import shutil
import subprocess
import tempfile


def header(path):
    meta = {}
    with Path(path).open('rb') as stream:
        for _ in range(256):
            line = stream.readline()
            if not line:
                raise ValueError(f'{path}: missing PCD DATA')
            parts = line.decode('ascii').split()
            if parts and not parts[0].startswith('#'):
                meta[parts[0]] = parts[1:]
            if parts and parts[0] == 'DATA':
                return meta
    raise ValueError('PCD header too long')


def descriptor(meta):
    fields, sizes, kinds = meta['FIELDS'], meta['SIZE'], meta['TYPE']
    counts = meta.get('COUNT', ['1']*len(fields))
    if not len(fields) == len(sizes) == len(kinds) == len(counts):
        raise ValueError('Inconsistent PCD field descriptor')
    return [(n, int(s), t, int(c)) for n,s,t,c in zip(fields,sizes,kinds,counts) if n != '_']


def require_compressor():
    tool = shutil.which('pcl_convert_pcd_ascii_binary')
    if tool is None:
        raise RuntimeError('Missing pcl_convert_pcd_ascii_binary (pcl-tools); '
                           'install it or disable compress_keyframes/use --no-compress')
    return tool


def compress_pcd(path, max_bytes=128*1024*1024):
    """Compress ONE file atomically; keep original if compression is bigger or size exceeds budget.

    PCL loads a complete file. The byte cap therefore applies before invoking PCL;
    it is an input-size cap, not a promise about the process's peak memory.
    """
    path = Path(path)
    meta = header(path)
    before = path.stat().st_size
    result = dict(before=before, after=before, status='unchanged')
    if meta['DATA'] == ['binary_compressed']:
        return dict(result, status='already_compressed')
    if int(meta['POINTS'][0]) == 0:
        return dict(result, status='empty')
    if before > max_bytes:
        return dict(result, status='over_budget')
    tool = require_compressor()
    with tempfile.TemporaryDirectory(prefix='.pcd_compress_', dir=path.parent) as tmp:
        output = Path(tmp) / 'compressed.pcd'
        subprocess.run([tool, str(path), str(output), '2'], check=True, stdout=subprocess.DEVNULL)
        new_meta = header(output)
        if (new_meta['DATA'] != ['binary_compressed'] or descriptor(meta) != descriptor(new_meta) or
                int(meta['POINTS'][0]) != int(new_meta['POINTS'][0])):
            raise ValueError('PCD compression changed fields/count or did not produce binary_compressed')
        # PCL retains VIEWPOINT. Reject a changed sensor pose instead of silently dropping it.
        old_pose = [float(x) for x in meta.get('VIEWPOINT', ['0','0','0','1','0','0','0'])]
        new_pose = [float(x) for x in new_meta.get('VIEWPOINT', ['0','0','0','1','0','0','0'])]
        if len(old_pose) != len(new_pose) or any(abs(a-b) > 1e-6*(1+abs(a)) for a,b in zip(old_pose,new_pose)):
            raise ValueError('PCD compression changed VIEWPOINT')
        after = output.stat().st_size
        if after < before:
            output.replace(path)
            return dict(before=before, after=after, status='compressed')
    return result


def storage_options(cfg):
    options = dict(write_global_clean=True, write_removed_points=True, compress_keyframes=False)
    values = cfg.get('storage', {})
    if not isinstance(values, dict):
        raise ValueError('storage must be a mapping')
    unknown = set(values)-set(options)
    if unknown:
        raise ValueError(f'Unknown storage options: {sorted(unknown)}')
    options.update(values)
    if any(not isinstance(value, bool) for value in options.values()):
        raise ValueError('storage options must be booleans')
    return options
