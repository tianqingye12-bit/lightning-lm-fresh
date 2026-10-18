"""Coordinate contracts, view selection and content-addressed segmentation cache."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import warnings
import zipfile

import numpy as np
import yaml


def rigid_transform(values, name):
    T = np.asarray(values, dtype=float).reshape(4, 4)
    R = T[:3, :3]
    if (not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(R), 1, atol=1e-4)):
        raise ValueError(f'{name} must be a finite rigid transform (row-major 4x4)')
    return T


def load_frame_contract(map_dir):
    path = Path(map_dir) / 'map_frames.yaml'
    if not path.exists():
        warnings.warn('Legacy map without map_frames.yaml: retaining the old projection convention. '
                      'Non-identity LiDAR/IMU extrinsics require remapping with round 4; '
                      'do not add metadata to an old map to pretend it is corrected.')
        return np.eye(4), 'legacy'
    data = yaml.safe_load(path.read_text())
    if (data.get('format_version') != 2 or data.get('keyframe_pose_frame') != 'imu'
            or data.get('keyframe_cloud_frame') != 'imu'):
        raise ValueError('Unsupported map_frames.yaml coordinate contract')
    return rigid_transform(data['T_imu_lidar'], 'T_imu_lidar'), 'imu_v2'


class ViewSelector:
    """Minimum time gate, then translation OR rotation OR maximum-time heartbeat."""
    def __init__(self, cfg):
        self.period = float(cfg['image_period'])
        options = cfg.get('view_selection', {})
        self.translation = float(options.get('translation_m', 0))
        self.rotation = float(options.get('rotation_deg', 0))
        self.max_period = float(options.get('max_period_s', 0))
        values = [self.period, self.translation, self.rotation, self.max_period]
        if not np.isfinite(values).all() or min(values) < 0 or self.period <= 0:
            raise ValueError('View selection thresholds must be finite/nonnegative; image_period > 0')
        if 0 < self.max_period < self.period:
            raise ValueError('max_period_s must be zero (disabled) or >= image_period')
        self.last_t, self.last_pose = None, None
        self.stats = dict(selected=0, skipped_period=0, skipped_motion=0, invalid_pose=0)

    def accept(self, t, pose):
        if pose is None or not np.isfinite(t) or not np.isfinite(pose).all():
            self.stats['invalid_pose'] += 1
            return False
        if self.last_t is not None:
            dt = t - self.last_t
            if dt < self.period:
                self.stats['skipped_period'] += 1
                return False
            distance = np.linalg.norm(pose[:3, 3] - self.last_pose[:3, 3])
            R = self.last_pose[:3, :3].T @ pose[:3, :3]
            angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
            enabled = self.translation > 0 or self.rotation > 0
            moved = ((self.translation > 0 and distance >= self.translation)
                     or (self.rotation > 0 and angle >= self.rotation))
            heartbeat = self.max_period > 0 and dt >= self.max_period
            if enabled and not (moved or heartbeat):
                self.stats['skipped_motion'] += 1
                return False
        self.last_t, self.last_pose = t, pose.copy()
        self.stats['selected'] += 1
        return True


def file_digest(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


class MaskCache:
    """Only caches raw segmentation, never map-dependent votes or dilated masks."""
    def __init__(self, directory, identity):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.identity = json.dumps(identity, sort_keys=True, separators=(',', ':')).encode()
        self.hits = self.misses = self.corrupt = 0

    def key(self, image):
        image = np.ascontiguousarray(image)
        digest = hashlib.sha256(self.identity)
        digest.update(str((image.shape, image.dtype.str)).encode())
        digest.update(memoryview(image).cast('B'))
        return digest.hexdigest()

    def get(self, key, shape):
        path = self.directory / key[:2] / (key + '.npz')
        try:
            with np.load(path, allow_pickle=False) as data:
                if tuple(data['shape']) != tuple(shape):
                    raise ValueError('Cached shape mismatch')
                bits, count = data['bits'], int(data['objects'].item())
                if bits.dtype != np.uint8 or bits.size != (int(np.prod(shape)) + 7) // 8 or count < 0:
                    raise ValueError('Invalid mask cache')
                mask = np.unpackbits(bits, count=int(np.prod(shape))).reshape(shape).astype(bool)
            self.hits += 1
            return mask, count
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, EOFError, zipfile.BadZipFile):
            self.corrupt += 1
        self.misses += 1
        return None

    def put(self, key, mask, objects):
        directory = self.directory / key[:2]
        directory.mkdir(exist_ok=True)
        # Atomic replacement means interrupted inference never leaves a valid-looking partial cache.
        temp = None
        try:
            with tempfile.NamedTemporaryFile(dir=directory, suffix='.npz', delete=False) as stream:
                temp = Path(stream.name)
                np.savez_compressed(stream, bits=np.packbits(mask), shape=np.array(mask.shape),
                                    objects=np.array(objects))
            os.replace(temp, directory / (key + '.npz'))
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)
