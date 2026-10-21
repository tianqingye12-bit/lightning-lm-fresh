"""Box-only inference and bounded 3D association. Never converts boxes to masks."""
from dataclasses import dataclass
import os
from pathlib import Path
import tempfile
import zipfile
import numpy as np
from clean_map_runtime import MaskCache
from seed_local_cleanup import labels


def perception_mode(cfg):
    mode = cfg.get('vehicle_perception', 'segment')
    if mode not in ('segment', 'detect'):
        raise ValueError('vehicle_perception must be segment or detect')
    return mode


def detection_options(cfg):
    opts = dict(core_inset=0.15, core_bottom=0.70, min_core_pixels=6,
                depth_bin_m=0.5, front_margin_m=0.25, depth_span_m=3.0,
                link_m=0.35, min_component_points=10, max_candidates=10000,
                max_extent_cam_m=[7., 3., 7.])
    supplied = cfg.get('detection_filter', {})
    if not isinstance(supplied, dict) or set(supplied) - set(opts):
        raise ValueError('Unknown detection_filter option')
    opts.update(supplied)
    for key in ('min_core_pixels', 'min_component_points', 'max_candidates'):
        if isinstance(opts[key], bool) or not isinstance(opts[key], int) or not 1 <= opts[key] <= 100000:
            raise ValueError(f'Invalid detection_filter.{key}')
    for key in ('core_inset', 'core_bottom', 'depth_bin_m', 'front_margin_m', 'depth_span_m', 'link_m'):
        value = opts[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid detection_filter.{key}')
    if not opts['core_inset'] < .5 or not opts['core_inset'] < opts['core_bottom'] <= 1-opts['core_inset']:
        raise ValueError('Invalid detection core fractions')
    if opts['link_m'] > 1 or opts['depth_span_m'] > 7 or opts['depth_bin_m'] > 2 or opts['front_margin_m'] > 1:
        raise ValueError('Detection geometry limits are too large')
    extent = np.asarray(opts['max_extent_cam_m'], float)
    if extent.shape != (3,) or not np.isfinite(extent).all() or (extent <= 0).any() or (extent > 10).any():
        raise ValueError('max_extent_cam_m must contain three dimensions in (0,10]')
    return opts


@dataclass
class Detections:
    boxes: np.ndarray
    confidence: np.ndarray
    classes: np.ndarray

    def __post_init__(self):
        self.boxes = np.asarray(self.boxes, float)
        self.confidence = np.asarray(self.confidence, float)
        classes = np.asarray(self.classes, float)
        n = len(self.boxes)
        if self.boxes.shape != (n, 4) or self.confidence.shape != (n,) or classes.shape != (n,) or n > 100:
            raise ValueError('Detection arrays must be aligned Nx4, N, N (max 100)')
        if not all(np.isfinite(a).all() for a in (self.boxes, self.confidence, classes)):
            raise ValueError('Nonfinite detection output')
        if ((self.confidence < 0) | (self.confidence > 1)).any() or (classes < 0).any() or (classes != np.floor(classes)).any():
            raise ValueError('Invalid detection confidence/class')
        if (self.boxes[:, 2:] <= self.boxes[:, :2]).any():
            raise ValueError('Detection box has nonpositive extent')
        self.classes = classes.astype(np.int64)


def infer(model, image, cfg, device):
    result = model(image, conf=cfg['yolo_conf'], classes=cfg['vehicle_classes'],
                   imgsz=int(cfg.get('yolo_imgsz', 640)), max_det=100,
                   verbose=False, device=device)[0]
    boxes = result.boxes
    if boxes is None:
        return Detections(np.empty((0, 4)), np.empty(0), np.empty(0))
    raw = Detections(boxes.xyxy.cpu().numpy(), boxes.conf.cpu().numpy(), boxes.cls.cpu().numpy())
    keep = np.isin(raw.classes, cfg['vehicle_classes']) & (raw.confidence >= cfg['yolo_conf'])
    return Detections(raw.boxes[keep], raw.confidence[keep], raw.classes[keep])


class DetectionCache(MaskCache):
    """Cache raw boxes only, with a separate task/schema identity and file format."""
    def __init__(self, directory, identity):
        super().__init__(directory, dict(identity, representation='raw_xyxy_detections_v1'))
        self.height, self.width = int(identity['height']), int(identity['width'])

    def get(self, key, shape):
        path = self.directory / key[:2] / (key + '.npz')
        try:
            if path.stat().st_size > 100000:
                raise ValueError('Oversized detection cache')
            with np.load(path, allow_pickle=False) as data:
                if tuple(data['shape']) != tuple(shape):
                    raise ValueError('Detection image shape mismatch')
                detection = Detections(data['boxes'], data['confidence'], data['classes'])
            self.hits += 1
            return detection, len(detection.boxes)
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, KeyError, EOFError, zipfile.BadZipFile):
            self.corrupt += 1
        self.misses += 1
        return None

    def put(self, key, detection, objects):
        if objects != len(detection.boxes):
            raise ValueError('Detection count mismatch')
        directory = self.directory / key[:2]
        directory.mkdir(exist_ok=True)
        temp = None
        try:
            with tempfile.NamedTemporaryFile(dir=directory, suffix='.npz', delete=False) as stream:
                temp = Path(stream.name)
                np.savez_compressed(stream, boxes=detection.boxes, confidence=detection.confidence,
                                    classes=detection.classes, shape=np.asarray((self.height, self.width)))
            os.replace(temp, directory / (key + '.npz'))
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)


def associate(detection, cam, u, v, visible, floor, shape, cfg):
    """Per-box depth anchor + 3D components. Unmatched points always abstain."""
    opts = detection_options(cfg)
    hit = np.zeros(len(cam), bool)
    region = np.zeros(shape, bool)  # rectangle overlay only; never used as positive evidence
    stats = dict(boxes=len(detection.boxes), accepted_components=0, skipped_core=0,
                 skipped_candidates=0, skipped_extent=0)
    for box in detection.boxes:
        x1, y1, x2, y2 = box.copy()
        x1, x2 = np.clip([x1, x2], 0, shape[1])
        y1, y2 = np.clip([y1, y2], 0, shape[0])
        if x2 <= x1 or y2 <= y1:
            stats['skipped_core'] += 1
            continue
        region[int(np.floor(y1)):int(np.ceil(y2)), int(np.floor(x1)):int(np.ceil(x2))] = True
        inside = visible & ~floor & (u >= x1) & (u < x2) & (v >= y1) & (v < y2)
        fraction = opts['core_inset']
        core = inside & (u >= x1 + fraction*(x2-x1)) & (u < x2 - fraction*(x2-x1))
        core &= (v >= y1 + fraction*(y2-y1)) & (v < y1 + opts['core_bottom']*(y2-y1))
        core_ids = np.flatnonzero(core)
        # Pixel support cannot be inflated by overlapping saved scans.
        _, unique = np.unique(v[core_ids]*shape[1]+u[core_ids], return_index=True)
        core_ids = core_ids[unique]
        if len(core_ids) < opts['min_core_pixels']:
            stats['skipped_core'] += 1
            continue
        bins = np.floor(cam[core_ids, 2] / opts['depth_bin_m']).astype(np.int64)
        values, counts = np.unique(bins, return_counts=True)
        supported = values[counts >= opts['min_core_pixels']]
        if not len(supported):
            stats['skipped_core'] += 1
            continue
        # Nearest supported depth mode, not all depths behind a 2D box.
        anchor_ids = core_ids[bins == supported[0]]
        anchor = np.median(cam[anchor_ids, 2])
        candidate = np.flatnonzero(inside & (cam[:, 2] >= anchor-opts['front_margin_m']) &
                                   (cam[:, 2] <= anchor+opts['depth_span_m']))
        if len(candidate) > opts['max_candidates']:
            stats['skipped_candidates'] += 1
            continue
        component = labels(cam[candidate], opts['link_m'])
        support = np.bincount(component, weights=np.isin(candidate, anchor_ids))
        counts = np.bincount(component)
        accepted = np.flatnonzero((support >= opts['min_core_pixels']) &
                                  (counts >= opts['min_component_points']))
        for label in accepted:
            members = candidate[component == label]
            if (np.ptp(cam[members], axis=0) > opts['max_extent_cam_m']).any():
                stats['skipped_extent'] += 1
                continue
            hit[members] = True
            stats['accepted_components'] += 1
    return hit, np.zeros(len(cam), bool), region, stats
