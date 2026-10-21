#!/usr/bin/env python3
"""Fixed-sample plane error and class-aware instance-mask evaluation (not mAP)."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def ratio(numerator, denominator):
    return float(numerator / denominator) if denominator else None


def instance_counts(pred_masks, pred_classes, scores, truth_masks, truth_classes, iou_threshold=.5):
    """Score-ordered, one-to-one, same-class mask matching at one IoU threshold."""
    pred_masks, truth_masks = np.asarray(pred_masks, bool), np.asarray(truth_masks, bool)
    pred_classes, truth_classes, scores = map(np.asarray, (pred_classes, truth_classes, scores))
    if pred_masks.ndim != 3 or truth_masks.ndim != 3 or pred_masks.shape[1:] != truth_masks.shape[1:]:
        raise ValueError('Prediction and annotation masks must be NxHxW at identical image resolution')
    if pred_classes.shape != (len(pred_masks),) or scores.shape != pred_classes.shape or truth_classes.shape != (len(truth_masks),):
        raise ValueError('Mask/class/score count mismatch')
    if not 0 < iou_threshold <= 1 or not np.isfinite(scores).all() or np.any(scores < 0) or np.any(scores > 1):
        raise ValueError('Invalid scores or IoU threshold')
    if any(not masks.reshape(len(masks), -1).any(axis=1).all() for masks in (pred_masks, truth_masks) if len(masks)):
        raise ValueError('Instance masks must not be empty')
    matches = np.zeros(len(truth_masks), bool)
    counts = {int(c): dict(tp=0, fp=0, fn=0) for c in np.union1d(pred_classes, truth_classes)}
    for i in np.argsort(-scores, kind='stable'):
        candidates = np.flatnonzero((truth_classes == pred_classes[i]) & ~matches)
        best, best_iou = -1, -1.
        for j in candidates:
            intersection = np.count_nonzero(pred_masks[i] & truth_masks[j])
            union = np.count_nonzero(pred_masks[i] | truth_masks[j])
            iou = intersection / union
            if iou > best_iou:
                best, best_iou = j, iou
        c = counts[int(pred_classes[i])]
        if best >= 0 and best_iou >= iou_threshold:
            matches[best] = True
            c['tp'] += 1
        else:
            c['fp'] += 1
    for j in np.flatnonzero(~matches):
        counts[int(truth_classes[j])]['fn'] += 1
    pred_union, truth_union = pred_masks.any(axis=0), truth_masks.any(axis=0)
    pixels = dict(tp=int(np.count_nonzero(pred_union & truth_union)),
                  fp=int(np.count_nonzero(pred_union & ~truth_union)),
                  fn=int(np.count_nonzero(~pred_union & truth_union)))
    return counts, pixels


def metrics(counts):
    tp, fp, fn = (counts[k] for k in ('tp', 'fp', 'fn'))
    return dict(counts, precision=ratio(tp, tp+fp), recall=ratio(tp, tp+fn),
                f1=ratio(2*tp, 2*tp+fp+fn), iou=ratio(tp, tp+fp+fn))


def plane_metrics(points, specs):
    """Distances to FIXED reference planes, never fitted separately to each run."""
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3 or not np.issubdtype(points.dtype, np.number):
        raise ValueError('Points must be an Nx3 numeric array in the reference coordinate frame')
    reports = []
    for spec in specs:
        normal = np.asarray(spec['normal'], float)
        low, high = np.asarray(spec['min'], float), np.asarray(spec['max'], float)
        offset, cell = float(spec['offset']), float(spec.get('coverage_cell_m', .1))
        if any(v.shape != (3,) or not np.isfinite(v).all() for v in (normal, low, high)) or np.any(low >= high):
            raise ValueError('Plane normal and ROI bounds must be finite xyz; min < max')
        length = np.linalg.norm(normal)
        if length <= 0 or not np.isfinite(offset) or not np.isfinite(cell) or cell <= 0:
            raise ValueError('Invalid plane equation or coverage cell')
        normal, offset = normal/length, offset/length
        selected = points[np.isfinite(points).all(axis=1) & (points >= low).all(axis=1) & (points <= high).all(axis=1)]
        distance = selected @ normal + offset
        # Coverage is measured on one common plane grid, so deleting geometry cannot
        # be mistaken for an improvement in plane thickness alone.
        seed = np.eye(3)[np.argmin(np.abs(normal))]
        tangent = np.cross(normal, seed); tangent /= np.linalg.norm(tangent)
        bitangent = np.cross(normal, tangent)
        projected = (selected-low) @ np.column_stack((tangent, bitangent))
        occupied = len(np.unique(np.floor(projected/cell).astype(np.int64), axis=0))
        reports.append(dict(name=spec['name'], points=len(selected), occupied_plane_cells=occupied,
                            coverage_cell_m=cell, mean_signed_m=float(distance.mean()) if len(distance) else None,
                            rmse_m=float(np.sqrt(np.mean(distance**2))) if len(distance) else None,
                            p95_abs_m=float(np.quantile(np.abs(distance), .95)) if len(distance) else None,
                            thickness_p95_p05_m=float(np.quantile(distance, .95)-np.quantile(distance, .05)) if len(distance) else None))
    return reports


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def evaluate_segmentation(args):
    import cv2
    from segmentation_quality import ScoredInstances, accepted_instances, quality_options
    cfg = yaml.safe_load(Path(args.config).read_text())
    quality = quality_options(cfg)
    manifest = Path(args.manifest)
    rows = json.loads(manifest.read_text())
    if not isinstance(rows, list) or not rows:
        raise ValueError('Manifest must be a nonempty list of fixed samples')
    keys = [row['image'] for row in rows]
    if len(set(keys)) != len(keys):
        raise ValueError('Manifest must not repeat an image')
    voter, weight_hash = None, None
    totals, pixel_totals, frames = {}, dict(tp=0, fp=0, fn=0), []
    for row in rows:
        image_path, truth_path = (manifest.parent/row[k] for k in ('image', 'truth'))
        image = cv2.imread(str(image_path))
        if image is None or image.shape[:2] != (cfg['height'], cfg['width']):
            raise ValueError(f'Image missing or different from calibrated resolution: {image_path}')
        if 'prediction' in row:
            pred_path = manifest.parent/row['prediction']
            with np.load(pred_path, allow_pickle=False) as data:
                pred = ScoredInstances(data['masks'], data['classes'], data['scores'])
            source = dict(prediction_sha256=digest(pred_path))
        else:
            if voter is None:
                from ultralytics import YOLO
                from clean_map_vehicles import VehicleVoter
                model = YOLO(cfg['yolo_model'])
                if model.task != 'segment':
                    raise ValueError('Evaluation requires a segmentation model')
                # Retain metadata even when evaluating the legacy profile. Thresholding
                # below still follows the original profile, not this inference-only flag.
                scored_cfg = dict(cfg, segmentation_quality=dict(enabled=True,
                    min_instance_confidence=cfg['yolo_conf']))
                voter = VehicleVoter(scored_cfg, np.empty((0, 3)), model, args.device)
                weight_hash = digest(Path(getattr(model, 'ckpt_path', None) or cfg['yolo_model']))
            pred, _ = voter.infer_mask(image)
            source = dict(weights_sha256=weight_hash)
        with np.load(truth_path, allow_pickle=False) as data:
            truth = ScoredInstances(data['masks'], data['classes'], np.ones(len(data['classes'])))
        if pred.masks.shape[1:] != image.shape[:2] or truth.masks.shape[1:] != image.shape[:2]:
            raise ValueError('Masks must use original image pixel coordinates')
        keep = np.isin(pred.classes, cfg['vehicle_classes']) & (pred.scores >= cfg['yolo_conf'])
        if quality['enabled']:
            keep &= accepted_instances(pred, cfg)
        target = np.isin(truth.classes, cfg['vehicle_classes'])
        counts, pixels = instance_counts(pred.masks[keep], pred.classes[keep], pred.scores[keep],
                                         truth.masks[target], truth.classes[target], args.iou)
        for c, values in counts.items():
            acc = totals.setdefault(c, dict(tp=0, fp=0, fn=0))
            for k, value in values.items(): acc[k] += value
        for k, value in pixels.items(): pixel_totals[k] += value
        frames.append(dict(image=row['image'], image_sha256=digest(image_path), truth_sha256=digest(truth_path),
                           **source, instances=counts, pixels=pixels))
    overall = {k: sum(v[k] for v in totals.values()) for k in ('tp', 'fp', 'fn')}
    # IoU on counts is meaningful only for pixels, not the object matching counts.
    instance_metrics = {str(c): {k:v for k,v in metrics(n).items() if k!='iou'} for c,n in totals.items()}
    return dict(task='class-aware instance segmentation', iou_threshold=args.iou,
                overall={k:v for k,v in metrics(overall).items() if k!='iou'}, per_class=instance_metrics,
                pixels=metrics(pixel_totals), frames=frames, config=cfg, manifest_sha256=digest(manifest),
                limitations='Fixed-threshold precision/recall/F1, not mAP; 2D masks, not 3D removal accuracy. '
                             'Annotate every target instance including small/occluded objects and include negative images.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='task', required=True)
    geometry = sub.add_parser('geometry', help='Evaluate a cropped Nx3 .npy cloud against fixed reference planes')
    geometry.add_argument('--points', required=True)
    geometry.add_argument('--planes', required=True, help='JSON list: name, normal, offset, min, max, coverage_cell_m')
    semantic = sub.add_parser('segmentation', help='Evaluate manually annotated fixed images')
    semantic.add_argument('--manifest', required=True, help='JSON list: image, truth (.npz masks/classes), optional prediction')
    semantic.add_argument('--config', required=True)
    semantic.add_argument('--device', default='cpu')
    semantic.add_argument('--iou', type=float, default=.5)
    for p in (geometry, semantic): p.add_argument('--output', required=True)
    args = parser.parse_args()
    if Path(args.output).exists(): parser.error('Output already exists; choose a new report path')
    if args.task == 'geometry':
        specs = json.loads(Path(args.planes).read_text())
        if not isinstance(specs, list) or not specs or len({s['name'] for s in specs}) != len(specs):
            parser.error('Planes must be a nonempty list with unique names')
        points = np.load(args.points, mmap_mode='r', allow_pickle=False)
        report = dict(task='fixed reference plane distances', planes=plane_metrics(points, specs),
                      points_sha256=digest(args.points), reference_sha256=digest(args.planes), reference=specs,
                      limitations='Use the same surveyed/reference planes, ROIs and one common coordinate frame. '
                                   'Plane errors are not whole-map/trajectory accuracy; compare coverage as well.')
    else:
        report = evaluate_segmentation(args)
    with Path(args.output).open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(f'Wrote {args.output}')


if __name__ == '__main__':
    main()
