"""Scored instance masks and conservative core-only 3D association.

Scores are detector confidences, not calibrated probabilities. Rejected proposals
and mask boundaries abstain; they must never turn into background votes.
"""
from dataclasses import dataclass
import os
from pathlib import Path
import tempfile
import zipfile

import cv2
import numpy as np
from clean_map_runtime import MaskCache
from clean_map_instances import InstanceMaskCache, associate_instances


def quality_options(cfg):
    options = dict(enabled=False, min_instance_confidence=.40, class_confidence={},
                   core_erode_px=1, min_mean_confidence=.55)
    supplied = cfg.get('segmentation_quality', {})
    if not isinstance(supplied, dict) or set(supplied) - set(options):
        raise ValueError('Unknown segmentation_quality option')
    options.update(supplied)
    if not isinstance(options['enabled'], bool):
        raise ValueError('segmentation_quality.enabled must be boolean')
    radius = options['core_erode_px']
    if isinstance(radius, bool) or not isinstance(radius, int) or not 0 <= radius <= 20:
        raise ValueError('core_erode_px must be an integer in [0, 20]')
    def confidence(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or not 0 < value <= 1:
            raise ValueError('Confidence thresholds must be numeric in (0, 1]')
        return float(value)
    for name in ('min_instance_confidence', 'min_mean_confidence'):
        options[name] = confidence(options[name])
    if not isinstance(options['class_confidence'], dict):
        raise ValueError('class_confidence must map class IDs to confidence thresholds')
    thresholds = {}
    for key, value in options['class_confidence'].items():
        if isinstance(key, bool) or not str(key).isdigit():
            raise ValueError('class_confidence keys must be nonnegative integer class IDs')
        thresholds[int(key)] = confidence(value)
    options['class_confidence'] = thresholds
    if options['enabled']:
        # Post-inference gates cannot recover proposals already filtered by the model.
        floor = confidence(cfg['yolo_conf'])
        if min([options['min_instance_confidence'], *thresholds.values()]) < floor:
            raise ValueError('Instance confidence thresholds must be >= yolo_conf')
        if isinstance(cfg['mask_dilate_px'], bool) or not isinstance(cfg['mask_dilate_px'], int) or not 0 <= cfg['mask_dilate_px'] <= 50:
            raise ValueError('mask_dilate_px must be an integer in [0, 50]')
    return options


@dataclass
class ScoredInstances:
    masks: np.ndarray
    classes: np.ndarray
    scores: np.ndarray

    def __post_init__(self):
        self.masks = np.asarray(self.masks, dtype=bool)
        classes, scores = np.asarray(self.classes), np.asarray(self.scores)
        count = len(self.masks)
        if self.masks.ndim != 3 or not 0 <= count <= 300 or classes.shape != (count,) or scores.shape != (count,):
            raise ValueError('Masks, classes and scores must have the same instance count')
        if not np.isfinite(classes).all() or np.any(classes < 0) or np.any(classes > 2**31-1) or np.any(classes != np.floor(classes)):
            raise ValueError('Instance classes must be nonnegative int32 IDs')
        if not np.isfinite(scores).all() or np.any(scores < 0) or np.any(scores > 1):
            raise ValueError('Instance scores must be in [0, 1]')
        self.classes, self.scores = classes.astype(np.int32), scores.astype(np.float32)


class ScoredInstanceCache(MaskCache):
    """Schema 3 never substitutes unit confidence for legacy mask-only entries."""
    def get(self, key, shape):
        try:
            with np.load(self.directory / key[:2] / (key + '.npz'), allow_pickle=False) as data:
                dims = tuple(int(x) for x in data['shape'])
                if len(dims) != 3 or dims[1:] != tuple(shape) or not 0 <= dims[0] <= 300:
                    raise ValueError('Invalid scored instance dimensions')
                count = int(np.prod(dims))
                bits = data['bits']
                if bits.dtype != np.uint8 or bits.ndim != 1 or bits.size != (count + 7)//8:
                    raise ValueError('Invalid packed scored instances')
                prediction = ScoredInstances(np.unpackbits(bits, count=count).reshape(dims),
                                             data['classes'], data['scores'])
            self.hits += 1
            return prediction, dims[0]
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, KeyError, EOFError, zipfile.BadZipFile):
            self.corrupt += 1
        self.misses += 1
        return None

    def put(self, key, prediction, objects):
        if not isinstance(prediction, ScoredInstances) or len(prediction.masks) != objects:
            raise ValueError('Scored cache requires classes and confidences for every instance')
        directory = self.directory / key[:2]
        directory.mkdir(exist_ok=True)
        temp = None
        try:
            with tempfile.NamedTemporaryFile(dir=directory, suffix='.npz', delete=False) as stream:
                temp = Path(stream.name)
                np.savez_compressed(stream, bits=np.packbits(prediction.masks),
                                    shape=np.array(prediction.masks.shape),
                                    classes=prediction.classes, scores=prediction.scores)
            os.replace(temp, directory / (key + '.npz'))
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)


def instance_cache(directory, identity, cfg):
    if quality_options(cfg)['enabled']:
        return ScoredInstanceCache(directory, dict(identity, schema=3, scored_instances=True))
    return InstanceMaskCache(directory, identity)


def accepted_instances(prediction, cfg):
    options = quality_options(cfg)
    thresholds = np.array([options['class_confidence'].get(int(c), options['min_instance_confidence'])
                           for c in prediction.classes])
    return np.isin(prediction.classes, cfg['vehicle_classes']) & (prediction.scores >= thresholds)


def associate_scored_instances(prediction, scene, u, v, z, visible, cfg):
    if not isinstance(prediction, ScoredInstances) or prediction.masks.shape[1:] != scene.shape:
        raise ValueError('Scored instances at calibrated image resolution are required')
    options = quality_options(cfg)
    accepted = accepted_instances(prediction, cfg)
    union = prediction.masks.any(axis=0) if len(prediction.masks) else np.zeros(scene.shape, bool)
    # All proposals, including low-confidence ones, block a background vote.
    radius = cfg['mask_dilate_px']
    halo = cv2.dilate(union.astype(np.uint8), np.ones((2*radius+1,)*2, np.uint8)).astype(bool)
    scores = np.zeros(len(z), np.float32)
    core_radius = options['core_erode_px']
    kernel = np.ones((2*core_radius+1,)*2, np.uint8)
    core_cfg = dict(cfg, mask_dilate_px=0)
    for index in np.flatnonzero(accepted):
        mask, score = prediction.masks[index], prediction.scores[index]
        # Explicit border value: an image-edge mask also loses its uncertain boundary.
        core = cv2.erode(mask.astype(np.uint8), kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
        if not core.any():
            continue
        hit, _, _ = associate_instances(core[None], scene, u, v, z, visible, core_cfg)
        # Overlaps yield ONE vote with the strongest eligible score, never a sum.
        scores[hit] = np.maximum(scores[hit], score)
    return scores > 0, ~halo[v, u], union, scores
