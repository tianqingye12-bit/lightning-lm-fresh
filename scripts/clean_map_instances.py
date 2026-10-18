"""Instance masks and conservative depth association; no ROS or YOLO dependency."""
import os
from pathlib import Path
import tempfile
import zipfile

import cv2
import numpy as np
from clean_map_runtime import MaskCache


class InstanceMaskCache(MaskCache):
    """Keep overlapping instances separate. Never read the old union-mask cache."""
    def get(self, key, shape):
        path = self.directory / key[:2] / (key + '.npz')
        try:
            with np.load(path, allow_pickle=False) as data:
                dims = tuple(int(x) for x in data['shape'])
                if len(dims) != 3 or dims[1:] != tuple(shape) or not 0 <= dims[0] <= 300:
                    raise ValueError('Invalid instance dimensions')
                bits = data['bits']
                count = int(np.prod(dims))
                if bits.dtype != np.uint8 or bits.size != (count + 7) // 8:
                    raise ValueError('Invalid packed instances')
                masks = np.unpackbits(bits, count=count).reshape(dims).astype(bool)
            self.hits += 1
            return masks, dims[0]
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, EOFError, zipfile.BadZipFile):
            self.corrupt += 1
        self.misses += 1
        return None

    def put(self, key, masks, objects):
        masks = normalize_instances(masks)
        if len(masks) != objects:
            raise ValueError('Instance count mismatch')
        directory = self.directory / key[:2]
        directory.mkdir(exist_ok=True)
        temp = None
        try:
            with tempfile.NamedTemporaryFile(dir=directory, suffix='.npz', delete=False) as stream:
                temp = Path(stream.name)
                np.savez_compressed(stream, bits=np.packbits(masks), shape=np.array(masks.shape))
            os.replace(temp, directory / (key + '.npz'))
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)


def normalize_instances(masks):
    masks = np.asarray(masks, dtype=bool)
    if masks.ndim == 2:  # Compatibility with older API callers/tests.
        masks = masks[None] if masks.any() else np.empty((0, *masks.shape), bool)
    if masks.ndim != 3:
        raise ValueError('Masks must be NxHxW')
    return masks


def association_options(cfg):
    options = dict(min_depth_pixels=3, depth_quantile=0.10,
                   max_depth_span_m=3.0, depth_band_margin_m=0.25,
                   min_vehicle_views=cfg['min_views'], min_background_views=3,
                   background_ratio=0.5)
    options.update(cfg.get('instance_filter', {}))
    for key in ('min_depth_pixels', 'min_vehicle_views', 'min_background_views'):
        value = options[key]
        if not isinstance(value, int) or isinstance(value, bool) or value < 1:
            raise ValueError(f'instance_filter.{key} must be a positive integer')
    for key in ('max_depth_span_m', 'depth_band_margin_m'):
        if not np.isfinite(options[key]) or options[key] <= 0:
            raise ValueError(f'instance_filter.{key} must be finite and positive')
    if not 0 <= options['depth_quantile'] < .5 or not 0 < options['background_ratio'] <= 1:
        raise ValueError('Invalid instance depth quantile/background ratio')
    return options


def associate_instances(masks, scene, u, v, z, visible, cfg):
    """Return vehicle, known background, union. Ambiguous mask/halo points abstain.

    Each instance gets its own depth band from distinct visible image pixels.
    Missing evidence cannot be interpreted as background or as a positive vote.
    This is a conservative heuristic, not a proof that a masked surface is a car.
    """
    masks = normalize_instances(masks)
    options = association_options(cfg)
    shape = scene.shape
    union = masks.any(axis=0) if len(masks) else np.zeros(shape, bool)
    halo = np.zeros(shape, bool)
    hit = np.zeros(len(z), bool)
    kernel = np.ones((2 * cfg['mask_dilate_px'] + 1,) * 2, np.uint8)
    for mask in masks:
        if mask.shape != shape:
            raise ValueError('Instance mask resolution mismatch')
        expanded = cv2.dilate(mask.astype(np.uint8), kernel).astype(bool)
        # A dilation halo must not invade the raw mask of another instance.
        expanded &= ~union | mask
        halo |= expanded
        # Use only supported image pixels, not the number of duplicated map points.
        pixel = np.unique(v[visible & mask[v, u]] * shape[1] + u[visible & mask[v, u]])
        samples = scene.ravel()[pixel]
        samples = samples[np.isfinite(samples) & (samples > .1)]
        if len(samples) < options['min_depth_pixels']:
            continue
        anchor = np.quantile(samples, options['depth_quantile'])
        samples = samples[(samples >= anchor - options['depth_band_margin_m']) &
                          (samples <= anchor + options['max_depth_span_m'])]
        if len(samples) < options['min_depth_pixels']:
            continue
        low, high = np.quantile(samples, [.05, .95])
        margin = options['depth_band_margin_m']
        hit |= expanded[v, u] & (z >= low - margin) & (z <= high + margin)
    return hit, ~halo[v, u], union


def removal_votes(visible, vehicle, background, cfg):
    options = association_options(cfg)
    classified = vehicle + background
    observed = visible >= cfg['min_views']
    remove = (observed & (vehicle >= options['min_vehicle_views']) &
              (vehicle / np.maximum(classified, 1) >= cfg['vehicle_ratio']))
    veto = ((background >= options['min_background_views']) &
            (background / np.maximum(classified, 1) >= options['background_ratio']))
    return observed, remove & ~veto
