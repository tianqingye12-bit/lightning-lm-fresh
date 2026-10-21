"""Stream retained IMU-frame keyframes, clean temporally local views, rebuild PCD.

No changes to the original map or keyframe clouds. Unknown points are retained.
Rebuilt files concatenate retained frame points (overlap retained, no global voxel filter).
"""
import csv
import importlib.metadata
import json
from pathlib import Path
import shutil
import tempfile
import time

import numpy as np
from clean_map_instances import InstanceMaskCache
from semantic_evidence import decide, evidence_options, timing_stats, report_semantics
from clean_map_runtime import ViewSelector, file_digest
from map_storage import storage_options, compress_pcd, require_compressor


def load_manifest(directory, traj):
    path = directory / 'keyframe_clouds/index.csv'
    if not path.is_file():
        raise RuntimeError('Missing keyframe_clouds/index.csv. Rebuild, enable '
                           'system.export_keyframe_clouds: true, then save a NEW map; '
                           'or use --mode map for an existing map.')
    root = path.parent.resolve()
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != len(traj.t):
        raise ValueError('Keyframe cloud manifest/trajectory count mismatch')
    entries = []
    for row in rows:
        file = (root / row['file']).resolve()
        if file.parent != root or not file.is_file():
            raise ValueError(f'Missing or unsafe keyframe file: {row["file"]}')
        entries.append((int(row['id']), float(row['timestamp']), file))
    entries.sort(key=lambda entry: entry[1])
    if len({e[0] for e in entries}) != len(entries):
        raise ValueError('Duplicate keyframe IDs')
    if not np.allclose([e[1] for e in entries], traj.t, rtol=0, atol=1e-6):
        raise ValueError('Keyframe cloud times differ from trajectory')
    if not np.array_equal([e[0] for e in entries], traj.ids):
        raise ValueError('Keyframe cloud IDs differ from trajectory')
    return entries


class PcdStream:
    """Spool binary bodies to disk; finalize headers with the exact count."""
    def __init__(self, path, header):
        self.path, self.header, self.dtype, self.count = Path(path), header, None, 0
        self.body = tempfile.TemporaryFile(dir=self.path.parent)

    def append(self, cloud):
        if self.dtype is None:
            self.dtype = cloud.dtype
        elif self.dtype != cloud.dtype:
            raise ValueError('Inconsistent keyframe PCD fields')
        self.body.write(cloud.tobytes())
        self.count += len(cloud)

    def finish(self):
        lines = []
        for line in self.header:
            key = line.split()[0] if line else ''
            lines.append({'WIDTH': f'WIDTH {self.count}', 'HEIGHT': 'HEIGHT 1',
                          'POINTS': f'POINTS {self.count}', 'DATA': 'DATA binary'}.get(key, line))
        temporary = self.path.with_suffix('.pcd.tmp')
        self.body.seek(0)
        with temporary.open('wb') as stream:
            stream.write(('\n'.join(lines) + '\n').encode('ascii'))
            shutil.copyfileobj(self.body, stream, length=1024 * 1024)
        temporary.replace(self.path)
        self.body.close()

    def close(self):
        self.body.close()


def run_keyframe_clean(args, cfg, frame_contract):
    # Delayed import avoids circular import and keeps the geometry tests lightweight.
    import clean_map_vehicles as vr
    if frame_contract != 'imu_v2':
        raise ValueError('Keyframe rebuilding requires a version-2 IMU-frame map')
    evidence = evidence_options(cfg)
    storage = storage_options(cfg)
    if storage['compress_keyframes']:
        require_compressor()
    directory = Path(args.map)
    traj = vr.Trajectory(directory / 'keyframes.txt', cfg['max_kf_gap'])
    entries = load_manifest(directory, traj)  # Fail before loading/downloading any model.
    options = cfg.get('keyframe_clean', {})
    window = float(options.get('max_image_dt_s', .35))
    max_active = options.get('max_active_frames', 8)
    if not np.isfinite(window) or window <= 0 or window > 1:
        raise ValueError('keyframe_clean.max_image_dt_s must be in (0, 1]')
    if isinstance(max_active, bool) or not isinstance(max_active, int) or not 1 <= max_active <= 64:
        raise ValueError('keyframe_clean.max_active_frames must be an integer in [1, 64]')
    cfg = dict(cfg, _keyframe_times=traj.t.tolist(), _keyframe_window=window)
    selector = ViewSelector(cfg)
    from ultralytics import YOLO
    model = YOLO(cfg['yolo_model'])
    if model.task != 'segment':
        raise RuntimeError('Keyframe cleaning requires an instance segmentation (-seg) model')
    # One inference owner shares masks among frames of the SAME image; persistent cache optional.
    inference = vr.VehicleVoter(cfg, np.empty((0, 3)), model, args.device)
    if cfg.get('cache', {}).get('enabled', False) and not args.no_cache:
        weights = Path(getattr(model, 'ckpt_path', None) or cfg['yolo_model'])
        if not weights.is_file():
            raise RuntimeError('Cannot fingerprint model weights; use --no-cache')
        identity = dict(schema=2, instance_masks=True, weights=file_digest(weights),
                        ultralytics=importlib.metadata.version('ultralytics'),
                        torch=importlib.metadata.version('torch'), device=args.device,
                        conf=cfg['yolo_conf'], classes=cfg['vehicle_classes'],
                        imgsz=int(cfg.get('yolo_imgsz', 640)), max_det=100,
                        width=cfg['width'], height=cfg['height'], retina_masks=True)
        inference.cache = InstanceMaskCache(cfg.get('cache', {}).get('directory') or
                                            directory / '.vehicle_mask_cache', identity)
    diagnostic_root = directory / 'clean_diagnostics'
    diagnostic_root.mkdir(exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix='keyframes_', dir=diagnostic_root))
    clean_dir = run_dir / 'keyframe_clouds'
    clean_dir.mkdir()
    print(f'Keyframe mode: {len(entries)} frames; image window=±{window:.2f}s; '
          f'active limit={max_active}; output={run_dir}', flush=True)
    active, finished, output = {}, set(), {}
    previews = {'clean': [], 'vehicles': []}
    preview_budget = max(1, int(cfg.get('rviz_max_points', 300000)))
    frame_stride = max(1, int(np.ceil(len(entries)/preview_budget)))
    frame_budget = max(1, preview_budget//max(1, int(np.ceil(len(entries)/frame_stride))))
    stats = dict(frames=len(entries), frames_with_votes=0, frames_without_views=0,
                 floor_unknown_frames=0, retained_points=0, removed_points=0,
                 invalid_points=0, selected_images=0, point_view_votes=0)
    started = time.monotonic()
    overlays = 0
    decision_counts = {}
    frame_file = (run_dir / 'frames.csv').open('w', newline='')
    frame_columns = ['keyframe_id', 'timestamp', 'views', 'invalid_points', 'floor_unknown',
                     'total', 'removed', 'protected', 'background', 'uncertain', 'unobserved',
                     'blocked_coverage', 'blocked_baseline']
    frame_writer = csv.DictWriter(frame_file, fieldnames=frame_columns)
    frame_writer.writeheader()
    view_file = (run_dir / 'views.csv').open('w', newline='')
    view_columns = ['timestamp', 'keyframe_id', 'keyframe_dt_s', 'aligned_timestamp',
                    'nearest_keyframe_dt_s', 'pose_bracket_gap_s', 'status', 'objects',
                    'projected', 'visible', 'occluded', 'vehicle_votes', 'background_votes',
                    'unknown_votes', 'depth', 'seconds']
    view_writer = csv.DictWriter(view_file, fieldnames=view_columns)
    view_writer.writeheader()
    diagnostics = cfg.get('diagnostics', {})
    max_overlays = max(0, int(diagnostics.get('max_overlays', 20)))
    overlay_period = max(1, int(diagnostics.get('every_views', 20)))

    def load(index):
        header, cloud = vr.read_pcd(entries[index][2])
        xyz = np.column_stack([cloud['x'], cloud['y'], cloud['z']]).astype(float)
        finite = np.isfinite(xyz).all(axis=1)
        pose = traj.pose(entries[index][1])
        world = xyz[finite] @ pose[:3, :3].T + pose[:3, 3]
        voter = vr.VehicleVoter(cfg, world, model, args.device)
        return header, cloud, finite, world, voter

    def finish(index, state):
        nonlocal output
        header, cloud, finite, world, voter = state
        _, remove, _, _ = decide(voter.visible, voter.vehicle, voter.background, cfg, voter.baseline)
        protected = np.zeros(len(world), bool)
        floor_unknown = False
        stats['frames_with_votes'] += int(voter.views > 0)
        stats['frames_without_views'] += int(voter.views == 0)
        stats['point_view_votes'] += int(voter.vehicle.sum())
        if remove.any():
            # Only this frame's sensor height is relevant, including on sloping/multi-level roads.
            local_traj = type('FrameTrajectory', (), {'t': traj.t[index:index+1],
                                                     'p': traj.p[index:index+1]})()
            try:
                floor = vr.estimate_floor(world, local_traj)
                protected = world[:, 2] < floor + cfg['floor_keep']
            except RuntimeError:
                protected[:] = True  # Insufficient floor evidence: preserve all, report abstention.
                floor_unknown = True
                stats['floor_unknown_frames'] += 1
        _, remove, uncertain, counts = decide(voter.visible, voter.vehicle, voter.background,
                                               cfg, voter.baseline, protected)
        for key, value in counts.items():
            decision_counts[key] = decision_counts.get(key, 0) + value
        frame_writer.writerow(dict(keyframe_id=entries[index][0], timestamp=entries[index][1],
                                   views=voter.views, invalid_points=int((~finite).sum()),
                                   floor_unknown=int(floor_unknown), **counts))
        frame_file.flush()
        drop = np.zeros(len(cloud), bool)
        drop[finite] = remove
        frame_path = clean_dir / f'{entries[index][0]}.pcd'
        vr.write_pcd(frame_path, header, cloud, ~drop)
        if storage['compress_keyframes']:
            compress_pcd(frame_path)
        # Frame files remain in IMU coordinates; only rebuilt visualization clouds are world-frame.
        world_cloud = None
        if storage['write_global_clean'] or storage['write_removed_points'] or evidence['write_uncertain']:
            world_cloud = cloud[finite].copy()
            for axis, value in zip(('x', 'y', 'z'), world.T):
                world_cloud[axis] = value
        for name, enabled, selection in (
                ('clean', storage['write_global_clean'], ~remove),
                ('vehicles', storage['write_removed_points'], remove),
                ('uncertain', evidence['write_uncertain'], uncertain)):
            if enabled:
                if name not in output:
                    filename = {'clean': 'global_clean.pcd', 'vehicles': 'global_vehicles.pcd',
                                'uncertain': 'global_uncertain.pcd'}[name]
                    output[name] = PcdStream(run_dir / filename, header)
                output[name].append(world_cloud[selection])
            elif name in previews and args.rviz and index % frame_stride == 0:
                selected = world[selection]
                if len(selected):
                    sample = np.linspace(0, len(selected)-1, min(len(selected), frame_budget), dtype=np.intp)
                    previews[name].append(selected[sample])
        stats['retained_points'] += int((~remove).sum())
        stats['removed_points'] += int(remove.sum())
        stats['invalid_points'] += int((~finite).sum())
        finished.add(index)

    try:
        previous = -np.inf
        for t, pose, image, depth in vr.iterate_views(args.bag, cfg, traj, selector):
            aligned_t = t + cfg['time_offset']
            if aligned_t < previous:
                raise ValueError('Image header timestamps must be nondecreasing in keyframe mode')
            previous = aligned_t
            for i in list(active):
                if entries[i][1] < aligned_t - window:
                    finish(i, active.pop(i))
            indices = range(np.searchsorted(traj.t, aligned_t - window, side='left'),
                            np.searchsorted(traj.t, aligned_t + window, side='right'))
            # Memory bounded by explicit number of clouds; fail rather than silently skip frames.
            if len(set(active).union(indices)) > max_active:
                raise RuntimeError('Too many simultaneous keyframes. Reduce max_image_dt_s or '
                                   'increase max_active_frames only if memory permits.')
            masks = None
            def shared_masks(image):
                nonlocal masks
                if masks is None:
                    masks = inference.vehicle_mask(image)
                return masks
            for i in indices:
                if i in finished:
                    raise ValueError('Image ordering revisited an already finalized keyframe')
                if i not in active:
                    active[i] = load(i)
                voter = active[i][-1]
                voter.vehicle_mask = shared_masks
                before = time.monotonic()
                count = voter.add_view(pose, image, depth)
                view_writer.writerow(dict(timestamp=t, keyframe_id=entries[i][0],
                    keyframe_dt_s=aligned_t-entries[i][1], **timing_stats(traj, t, cfg),
                    status='no_projection' if count is None else 'voted',
                    **voter.last_stats, depth=int(depth is not None), seconds=time.monotonic()-before))
                view_file.flush()
                voter.vehicle_mask = None  # Do not retain masks from old images in inactive frames.
                if (count is not None and overlays < max_overlays and
                        stats['selected_images'] % overlay_period == 0):
                    import cv2
                    cv2.imwrite(str(run_dir / f'overlay_{overlays:03d}.jpg'),
                                vr.make_overlay(voter, image, t, traj))
                    overlays += 1
            stats['selected_images'] += 1
            if stats['selected_images'] == 1 or stats['selected_images'] % 20 == 0:
                print(f'keyframe clean: images={stats["selected_images"]}, '
                      f'finished={len(finished)}/{len(entries)}, active={len(active)}, '
                      f'removed={stats["removed_points"]}', flush=True)
        for i in range(len(entries)):
            if i not in finished:
                finish(i, active.pop(i) if i in active else load(i))
        if stats['selected_images'] == 0:
            raise RuntimeError('No usable temporally matched images; no final map published')
        for stream in output.values():
            stream.finish()
        # Copy the coordinate/pose contract for retained IMU-frame clouds.
        shutil.copy2(directory / 'keyframe_clouds/index.csv', clean_dir / 'index.csv')
        shutil.copy2(directory / 'keyframes.txt', run_dir / 'keyframes.txt')
        shutil.copy2(directory / 'map_frames.yaml', run_dir / 'map_frames.yaml')
        cache = inference.cache
        stats.update(evidence=decision_counts, evidence_units='finite keyframe points (overlap retained)',
                     semantics=report_semantics(), uncertain_cloud_written=evidence['write_uncertain'],
                     cache_hits=cache.hits if cache else 0, cache_misses=cache.misses if cache else 0,
                     cache_corrupt=cache.corrupt if cache else 0,
                     storage=storage, global_clean_written=storage['write_global_clean'],
                     removed_cloud_written=storage['write_removed_points'], overlays_written=overlays,
                     elapsed_seconds=time.monotonic()-started, config=cfg,
                     selection=selector.stats, coordinate_contract=frame_contract,
                     source='retained_keyframes', overlapping_frame_points_retained=True)
        (run_dir / 'summary.json').write_text(json.dumps(stats, indent=2) + '\n')
        print(f'Finished: cleaned frames={clean_dir}; world_cloud={storage["write_global_clean"]}; '
              f'removed_debug={storage["write_removed_points"]}; '
              f'{stats["removed_points"]}/'
              f'{stats["retained_points"] + stats["removed_points"]} points removed', flush=True)
        if args.rviz:
            # Display each final cloud sequentially; publisher bounds message size.
            from clean_map_rviz import RvizPublisher
            viz = RvizPublisher(max_points=int(cfg.get('rviz_max_points', 300000)))
            viz.publish_trajectory(traj)
            for topic, name in [('clean', 'global_clean.pcd'), ('vehicles', 'global_vehicles.pcd')]:
                # Our rebuilt files are uncompressed binary. Sample through mmap; do not
                # load a potentially huge concatenated map just for an RViz preview.
                path = run_dir / name
                if topic not in output:
                    sample = np.concatenate(previews[topic]) if previews[topic] else np.empty((0,3))
                    viz.publish_cloud(topic, sample[:preview_budget])
                    continue
                with path.open('rb') as stream:
                    while True:
                        line = stream.readline()
                        if not line:
                            raise ValueError('Missing rebuilt PCD header')
                        if line.startswith(b'DATA '):
                            offset = stream.tell()
                            break
                count = output[topic].count
                if count:
                    points = np.memmap(path, mode='r', dtype=output[topic].dtype,
                                       offset=offset, shape=(count,))
                    stride = max(1, int(np.ceil(count / int(cfg.get('rviz_max_points', 300000)))))
                    sample = points[::stride]
                    xyz = np.column_stack([sample['x'], sample['y'], sample['z']])
                    viz.publish_cloud(topic, xyz)
                    del sample, points
                else:
                    viz.publish_cloud(topic, np.empty((0, 3)))
            print('RViz final clouds ready; Ctrl+C to exit', flush=True)
            viz.spin()
    finally:
        frame_file.close()
        view_file.close()
        for stream in output.values():
            stream.close()
