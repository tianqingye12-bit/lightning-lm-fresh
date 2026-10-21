#!/usr/bin/env python3
"""离线实例分割去车：累计地图投票或逐关键帧清理重建。

--mode keyframes 使用 keyframe_clouds/index.csv 和版本2坐标约定；
结果默认保存在输入地图旁的 <map>_clean 目录；--output 可指定目录，
--export-global 可额外导出完整 global_clean.pcd。见 doc/CLEAN_MAP_OUTPUT.md。

SLAM 用完整点云建图，结束后用 SaveMap 导出的 global.pcd + keyframes.txt，
回放 bag 中的彩色图 (和对齐深度图)，YOLO 分割车辆，把地图体素投影到每个视角投票：
  - 可见次数: 体素在视野内且未被遮挡 (深度图一致性，深度无效时用地图 z-buffer)
  - 车辆次数: 可见且落在车辆掩码内
车辆占比超过阈值的体素删除；整簇大部分是车辆时整簇删除，覆盖相机没看到的车身。
贴地一层不删，避免地面出现空洞。

用法:
  python3 scripts/clean_map_vehicles.py --map data/garage --bag <bag目录或.bag> \
      --config config/garage_clean.yaml [--rviz]
map 模式输出: <map>/global_clean.pcd, <map>/global_vehicles.pcd。
keyframes 模式输出: <map>_clean/keyframe_clouds/ 和配套位姿，完整 PCD 按配置/参数导出。
"""
import argparse
import csv
import json
import importlib.metadata
import subprocess
import tempfile
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml

from clean_map_runtime import ViewSelector, file_digest, load_frame_contract, rigid_transform
from clean_map_instances import associate_instances, association_options
from semantic_evidence import PositiveBaseline, decide, evidence_options, timing_stats, report_semantics
from segmentation_quality import (ScoredInstances, quality_options, instance_cache,
                                  associate_scored_instances, accepted_instances)
from pose_trajectory import load_trajectory, trajectory_options
from cross_keyframe_semantics import cross_frame_options
from legacy_map_cleanup import legacy_enabled,associate_legacy,legacy_votes

PCD_TYPES = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4',
             ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}


def read_pcd(path):
    """返回 (header 行列表, 结构化数组)。binary_compressed 先用 pcl 工具解压。"""
    with open(path, 'rb') as f:
        header = []
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError(f'{path}: missing PCD DATA header')
            line = raw.decode('ascii').strip()
            header.append(line)
            if line.startswith('DATA'):
                break
        body = f.read()
    meta = {l.split()[0]: l.split()[1:] for l in header if l and not l.startswith('#')}
    data_type = meta['DATA'][0]
    if data_type == 'binary_compressed':
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'binary.pcd'
            subprocess.run(['pcl_convert_pcd_ascii_binary', str(path), str(out), '1'],
                           check=True, stdout=subprocess.DEVNULL)
            return read_pcd(out)
    names, counts = [], [int(c) for c in meta.get('COUNT', ['1'] * len(meta['FIELDS']))]
    for i, n in enumerate(meta['FIELDS']):
        names.append(n if n != '_' else f'_pad{i}')
    dtype = np.dtype([(n, PCD_TYPES[(t, int(s))], (c,) if c > 1 else ())
                      for n, t, s, c in zip(names, meta['TYPE'], meta['SIZE'], counts)])
    n_points = int(meta['POINTS'][0])
    if data_type == 'binary':
        cloud = np.frombuffer(body, dtype=dtype, count=n_points).copy()
    elif data_type == 'ascii':
        cloud = np.loadtxt(body.decode().splitlines(), dtype=dtype, ndmin=1)
    else:
        raise RuntimeError(f'Unsupported PCD DATA {data_type}')
    return header, cloud


def write_pcd(path, header, cloud, selection=None, chunk_points=250000):
    if selection is not None:
        selection = np.asarray(selection, dtype=bool)
        if selection.shape != (len(cloud),):
            raise ValueError('PCD selection must have one boolean per point')
    count = len(cloud) if selection is None else int(selection.sum())
    lines = []
    for line in header:
        key = line.split()[0] if line else ''
        if key == 'WIDTH':
            line = f'WIDTH {count}'
        elif key == 'HEIGHT':
            line = 'HEIGHT 1'
        elif key == 'POINTS':
            line = f'POINTS {count}'
        elif key == 'DATA':
            line = 'DATA binary'
        lines.append(line)
    with open(path, 'wb') as f:
        f.write(('\n'.join(lines) + '\n').encode('ascii'))
        for begin in range(0, len(cloud), chunk_points):
            block = cloud[begin:begin + chunk_points]
            if selection is not None:
                block = block[selection[begin:begin + chunk_points]]
            f.write(block.tobytes())


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    """旋转矩阵 -> (x, y, z, w)"""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def slerp(q0, q1, a):
    dot = np.dot(q0, q1)
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = np.arccos(dot)
    return (np.sin((1 - a) * theta) * q0 + np.sin(a * theta) * q1) / np.sin(theta)


class Trajectory:
    """keyframes.txt: id timestamp tx ty tz qx qy qz qw；位姿即 global.pcd 中关键帧点云的变换。"""

    def __init__(self, path, max_gap):
        data = np.loadtxt(path, ndmin=2)
        if data.shape[0] == 0 or data.shape[1] != 9 or not np.isfinite(data).all():
            raise ValueError('keyframes.txt must contain finite id timestamp tx ty tz qx qy qz qw rows')
        if not np.isfinite(max_gap) or max_gap <= 0:
            raise ValueError('max_kf_gap must be positive and finite')
        order = np.argsort(data[:, 1])
        if np.any(data[:, 0] < 0) or np.any(data[:, 0] != np.floor(data[:, 0])) or np.any(data[:, 0] >= 2.0 ** 63):
            raise ValueError('Keyframe IDs must be nonnegative int64 integers')
        self.ids = data[order, 0].astype(np.int64)
        if len(np.unique(self.ids)) != len(self.ids):
            raise ValueError('Keyframe IDs must be unique')
        self.t, self.p, self.q = data[order, 1], data[order, 2:5], data[order, 5:9]
        if np.any(np.diff(self.t) <= 0):
            raise ValueError('Keyframe timestamps must be unique')
        norms = np.linalg.norm(self.q, axis=1)
        if np.any(norms < 1e-12):
            raise ValueError('Keyframe quaternions must have nonzero norm')
        self.q = self.q / norms[:, None]
        self.max_gap = max_gap

    def pose(self, t):
        if not np.isfinite(t):
            return None
        i = np.searchsorted(self.t, t)
        if i < len(self.t) and self.t[i] == t:
            T = np.eye(4)
            T[:3, :3] = quat_to_mat(self.q[i])
            T[:3, 3] = self.p[i]
            return T
        if i == 0 or i == len(self.t):
            return None
        t0, t1 = self.t[i - 1], self.t[i]
        if t1 - t0 > self.max_gap:
            return None
        a = (t - t0) / (t1 - t0)
        T = np.eye(4)
        T[:3, :3] = quat_to_mat(slerp(self.q[i - 1], self.q[i], a))
        T[:3, 3] = (1 - a) * self.p[i - 1] + a * self.p[i]
        return T


def voxel_keys(xyz, size):
    """Collision-free integer triplets, including negative and large coordinates."""
    if not np.isfinite(size) or size <= 0:
        raise ValueError('Voxel size must be positive and finite')
    scaled = np.floor(np.asarray(xyz, dtype=float) / size)
    if not np.isfinite(scaled).all() or np.any(np.abs(scaled) >= 2.0 ** 63):
        raise ValueError('Voxel coordinates must be finite and fit int64')
    return scaled.astype(np.int64)


def estimate_floor(centroids, traj):
    """每个关键帧下方点数最多的水平层作为地面高度；体素取 3D 最近关键帧的地面。"""
    from scipy.spatial import cKDTree
    floors = np.full(len(traj.t), np.nan)
    neighbors = cKDTree(centroids[:, :2]).query_ball_point(traj.p[:, :2], r=3.0, p=np.inf)
    for i, (p, near) in enumerate(zip(traj.p, neighbors)):
        z = centroids[near, 2]
        z = z[(z > p[2] - 4) & (z < p[2] - 0.1)]
        if len(z) > 30:
            hist, edges = np.histogram(z, bins=np.arange(p[2] - 4, p[2] - 0.1, 0.05))
            floors[i] = edges[np.argmax(hist)] + 0.025
    valid = np.isfinite(floors)
    if not valid.any():
        raise RuntimeError('Cannot estimate floor height from map')
    _, nearest = cKDTree(traj.p[valid]).query(centroids)
    return floors[valid][nearest]


def components(centroids, size):
    """体素 26 邻域连通分量。"""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    if len(centroids) == 0:
        return np.empty(0, dtype=np.int32)
    keys = voxel_keys(centroids, size)
    uniq, cell = np.unique(keys, axis=0, return_inverse=True)
    # Structured lexicographic keys avoid packed-coordinate overflow and carry.
    key_dtype = np.dtype([('x', '<i8'), ('y', '<i8'), ('z', '<i8')])
    def records(cells):
        return np.ascontiguousarray(cells, dtype='<i8').view(key_dtype).ravel()
    ordered = records(uniq)
    rows, cols = [], []
    offsets = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
               if (dx, dy, dz) > (0, 0, 0)]
    for dx, dy, dz in offsets:
        nb = records(uniq + np.array([dx, dy, dz], dtype=np.int64))
        j = np.searchsorted(ordered, nb)
        j = np.minimum(j, len(uniq) - 1)
        hit = ordered[j] == nb
        rows.append(np.nonzero(hit)[0])
        cols.append(j[hit])
    rows, cols = np.concatenate(rows), np.concatenate(cols)
    graph = coo_matrix((np.ones(len(rows), dtype=np.int8), (rows, cols)), shape=(len(uniq),) * 2)
    _, label = connected_components(graph, directed=False)
    return label[cell]


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def decode_image(msg, depth=False):
    if hasattr(msg, 'format'):  # CompressedImage
        if 'compressedDepth' in msg.format:
            raise RuntimeError('compressedDepth is not supported; record raw depth')
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                           cv2.IMREAD_UNCHANGED if depth else cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError('Unable to decode compressed image')
        return img
    enc = msg.encoding.lower()
    dtype = {'16uc1': '<u2', 'mono16': '<u2', '32fc1': '<f4'}.get(enc, np.uint8)
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(enc, 1)
    row = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = row[:, :msg.width * channels * dtype.itemsize].view(dtype).reshape(msg.height, msg.width, channels)
    img = img[:, :, 0] if channels == 1 else img
    if enc == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif enc in ('rgba8', 'bgra8'):
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR)
    return np.ascontiguousarray(img)


class VehicleVoter:
    def __init__(self, cfg, centroids, model, device):
        from scipy.spatial import cKDTree
        self.cfg, self.centroids, self.model, self.device = cfg, centroids, model, device
        self.spatial_index = cKDTree(centroids)
        self.K = np.array(cfg['K'], dtype=float).reshape(3, 3)
        self.dist = np.array(cfg['distortion'], dtype=float)
        self.T_cam_lidar = rigid_transform(cfg['T_cam_lidar'], 'T_cam_lidar')
        self.w, self.h = int(cfg['width']), int(cfg['height'])
        # 归一化坐标范围，防止畸变模型把视野外的点折回图像内
        self.x_lim = 1.2 * max(self.K[0, 2], self.w - self.K[0, 2]) / self.K[0, 0]
        self.y_lim = 1.2 * max(self.K[1, 2], self.h - self.K[1, 2]) / self.K[1, 1]
        self.visible = np.zeros(len(centroids), np.int32)
        self.vehicle = np.zeros(len(centroids), np.int32)
        self.background = np.zeros(len(centroids), np.int32)
        self.quality = quality_options(cfg)
        self.score_sum = np.zeros(len(centroids), np.float32) if self.quality['enabled'] else None
        association_options(cfg)
        self.baseline = PositiveBaseline(len(centroids), cfg)
        self.views = 0
        self.cache = None
        self.last_stats = {}
        self.collect_world_evidence = cross_frame_options(cfg)['enabled']
        self.legacy = legacy_enabled(cfg)

    def vehicle_mask(self, image):
        key = self.cache.key(image) if self.cache is not None else None
        if key is not None:
            cached = self.cache.get(key, (self.h, self.w))
            if cached is not None:
                return cached
        result = self.infer_mask(image)
        if key is not None:
            self.cache.put(key, *result)
        return result

    def infer_mask(self, image):
        r = self.model(image, conf=self.cfg['yolo_conf'], classes=self.cfg['vehicle_classes'],
                       imgsz=int(self.cfg.get('yolo_imgsz', 640)), max_det=100,
                       retina_masks=True, verbose=False, device=self.device)[0]
        if r.masks is None:
            if r.boxes is not None and len(r.boxes):
                raise RuntimeError('Detections have no segmentation masks; use a compatible -seg model')
            empty = np.empty((0, self.h, self.w), bool)
            return (ScoredInstances(empty, [], []) if self.quality['enabled'] else empty), 0
        masks = r.masks.data.cpu().numpy()
        if masks.ndim != 3:
            raise RuntimeError('YOLO masks must be NxHxW')
        if len(masks) and masks.shape[1:] != (self.h, self.w):
            masks = np.stack([cv2.resize(mask, (self.w, self.h),
                                        interpolation=cv2.INTER_NEAREST) for mask in masks])
        if not len(masks):
            masks = np.empty((0, self.h, self.w), bool)
        if self.quality['enabled']:
            if r.boxes is None or not hasattr(r.boxes, 'cls') or not hasattr(r.boxes, 'conf'):
                raise RuntimeError('Scored segmentation requires aligned boxes.cls and boxes.conf')
            return ScoredInstances(masks > .5, r.boxes.cls.cpu().numpy(), r.boxes.conf.cpu().numpy()), len(masks)
        return masks > 0.5, len(masks)

    def add_view(self, T_world_lidar, image, depth):
        if image.shape[:2] != (self.h, self.w):
            raise RuntimeError(f'Image {image.shape[1]}x{image.shape[0]} != calibration {self.w}x{self.h}')
        self.last_stats = dict(projected=0, visible=0, occluded=0, vehicle_votes=0,
                               background_votes=0, unknown_votes=0, objects=0,
                               accepted_objects=0, positive_confidence_sum=0.)
        self.last_association = None
        T = self.T_cam_lidar @ np.linalg.inv(T_world_lidar)
        rng = self.cfg['max_range']
        near = np.asarray(self.spatial_index.query_ball_point(
            (T_world_lidar @ np.linalg.inv(self.T_cam_lidar))[:3, 3],
            rng * np.sqrt(1 + self.x_lim ** 2 + self.y_lim ** 2)), dtype=np.intp)
        cam = self.centroids[near] @ T[:3, :3].T + T[:3, 3]
        z = cam[:, 2]
        keep = (z > 0.3) & (z < rng)
        keep &= (np.abs(cam[:, 0] / np.maximum(z, 1e-6)) < self.x_lim)
        keep &= (np.abs(cam[:, 1] / np.maximum(z, 1e-6)) < self.y_lim)
        near, cam, z = near[keep], cam[keep], z[keep]
        if not len(near):
            return
        uv, _ = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), self.K, self.dist)
        uv = np.round(uv.reshape(-1, 2)).astype(int)
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < self.w) & (uv[:, 1] >= 0) & (uv[:, 1] < self.h)
        near, z, u, v = near[inside], z[inside], uv[inside, 0], uv[inside, 1]

        if not len(near):
            return None
        tol = self.cfg['occlusion_tol'] + 0.02 * z
        # 地图 z-buffer (4x4 像素块)，用于深度图无效处
        b = 4
        zbuf = np.full((self.h // b + 1, self.w // b + 1), np.inf)
        np.minimum.at(zbuf, (v // b, u // b), z)
        visible = z <= zbuf[v // b, u // b] + tol
        scene = np.repeat(np.repeat(zbuf, b, axis=0), b, axis=1)[:self.h, :self.w]
        if depth is not None:
            if depth.shape != (self.h, self.w):
                raise ValueError('Depth image must be aligned to color image and have the same resolution')
            dimg = depth.astype(np.float32) * self.cfg['depth_scale']
            dimg[~(np.isfinite(dimg) & (dimg > 0.1))] = np.nan
            d = dimg[v, u].astype(float)
            valid = np.isfinite(d)
            visible[valid] = np.abs(z[valid] - d[valid]) < tol[valid]
            scene = np.where(np.isfinite(dimg), dimg, scene)

        masks, n_obj = self.vehicle_mask(image)
        if self.legacy:
            is_vehicle,mask=associate_legacy(masks,scene,u,v,z,self.cfg)
            background=np.zeros(len(z),bool)
            scores=np.zeros(len(z),float)
            accepted_objects,confidence_sum=n_obj,0.
        elif self.quality['enabled']:
            is_vehicle, background, mask, scores = associate_scored_instances(masks, scene, u, v, z, visible, self.cfg)
            self.score_sum[near[visible & is_vehicle]] += scores[visible & is_vehicle]
            accepted_objects = int(accepted_instances(masks, self.cfg).sum())
            confidence_sum = float(scores[visible & is_vehicle].sum())
        else:
            is_vehicle, background, mask = associate_instances(masks, scene, u, v, z, visible, self.cfg)
            accepted_objects, confidence_sum = n_obj, 0.
            scores = np.ones(len(near),float)
        self.visible[near[visible]] += 1
        self.vehicle[near[visible & is_vehicle]] += 1
        self.baseline.update(near[visible & is_vehicle], np.linalg.inv(T)[:3, 3])
        self.background[near[visible & background]] += 1
        self.views += 1
        self.last_view = (u, v, visible, is_vehicle, mask)
        self.last_background = background
        if self.collect_world_evidence:
            self.last_association = dict(indices=near[visible],positive=is_vehicle[visible],
                                         background=background[visible],scores=scores[visible])
        self.last_stats = dict(projected=len(near), visible=int(visible.sum()),
                               occluded=int((~visible).sum()),
                               vehicle_votes=int((visible & is_vehicle).sum()),
                               background_votes=int((visible & background).sum()),
                               unknown_votes=int((visible & ~is_vehicle & ~background).sum()), objects=n_obj,
                               accepted_objects=accepted_objects, positive_confidence_sum=confidence_sum)
        return n_obj


def make_overlay(voter, image, timestamp, traj=None):
    """Blue mask; gray occluded; yellow unknown; green background; red vehicle votes."""
    u, v, visible, vehicle, mask = voter.last_view
    output = image.copy()
    output[mask] = (0.5 * output[mask] + [127, 60, 0]).astype(np.uint8)
    output[v[~visible], u[~visible]] = (128, 128, 128)
    background = visible & voter.last_background
    unknown = visible & ~vehicle & ~voter.last_background
    output[v[unknown], u[unknown]] = (0, 255, 255)
    output[v[background], u[background]] = (0, 255, 0)
    hit = visible & vehicle
    output[v[hit], u[hit]] = (0, 0, 255)
    cv2.putText(output, f'view {voter.views} t={timestamp:.1f} visible={int(visible.sum())} '
                f'vehicle={int(hit.sum())}', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    if traj is not None:
        timing = timing_stats(traj, timestamp, voter.cfg)
        cv2.putText(output, f"offset={voter.cfg['time_offset']:+.3f}s "
                    f"nearest_kf_dt={timing['nearest_keyframe_dt_s']} "
                    f"pose_gap={timing['pose_bracket_gap_s']}", (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(output, f"pose={timing.get('pose_source','keyframes')} "
                    f"dt={timing.get('nearest_pose_dt_s')} gap={timing['pose_bracket_gap_s']} "
                    f"{timing.get('pose_correction','')}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
    return output


def iterate_views(bag, cfg, traj, selector=None):
    """Yield (timestamp, T_world_lidar, color, depth), selecting before image decoding."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    color_topic, depth_topic = cfg['color_topic'], cfg.get('depth_topic') or None
    selector = selector or ViewSelector(cfg)
    T_imu_lidar = np.asarray(cfg.get('_T_imu_lidar', np.eye(4)))
    keyframe_times = cfg.get('_keyframe_times')
    keyframe_times = None if keyframe_times is None else np.asarray(keyframe_times)
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conns = {c.topic: c for c in reader.connections}
        if color_topic not in conns:
            raise RuntimeError(f'{color_topic} not in bag; topics: {sorted(conns)}')
        if depth_topic and depth_topic not in conns:
            raise RuntimeError(f'{depth_topic} not in bag; set depth_topic empty to use z-buffer only')
        wanted = [conns[color_topic]] + ([conns[depth_topic]] if depth_topic else [])
        depths, pending = deque(maxlen=60), deque()

        def flush(now):
            while pending and (depth_topic is None or now - pending[0][0] > 0.1):
                t, msg = pending.popleft()
                pose = traj.pose(t + cfg['time_offset'])
                if pose is not None:
                    pose = pose @ T_imu_lidar
                if keyframe_times is not None:
                    times = keyframe_times
                    i = np.searchsorted(times, t + cfg['time_offset'])
                    if min((abs(times[j] - t - cfg['time_offset'])
                            for j in (i-1, i) if 0 <= j < len(times)), default=np.inf) > cfg['_keyframe_window']:
                        continue
                if not selector.accept(t, pose):
                    continue
                depth = None
                if depths:
                    td, dmsg = min(depths, key=lambda item: abs(item[0] - t))
                    if abs(td - t) < 0.04:
                        depth = decode_image(dmsg, depth=True)
                yield t, pose, decode_image(msg), depth

        last_progress = time.monotonic()
        for conn, _, raw in reader.messages(connections=wanted):
            msg = reader.deserialize(raw, conn.msgtype)
            t = stamp(msg)
            if conn.topic == depth_topic:
                depths.append((t, msg))
            else:
                pending.append((t, msg))
            yield from flush(t)
            if time.monotonic() - last_progress >= 10:
                print(f'bag image stamp={t:.3f}; selection={selector.stats}', flush=True)
                last_progress = time.monotonic()
        yield from flush(np.inf)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map', required=True, help='SaveMap 输出目录 (含 global.pcd, keyframes.txt)')
    parser.add_argument('--bag', required=True, help='建图时录制的 bag (ROS1 .bag 或 ROS2 目录)')
    parser.add_argument('--config', default='config/garage_clean.yaml')
    parser.add_argument('--device', default=None, help='YOLO 设备, 如 cpu / 0')
    parser.add_argument('--rviz', action='store_true',
                        help='Live RViz preview with bounded sampled clouds')
    parser.add_argument('--mode', choices=['map', 'keyframes'], default=None,
                        help='map: existing map voting; keyframes: clean saved frames and rebuild')
    parser.add_argument('--no-cache', action='store_true', help='Disable segmentation cache')
    parser.add_argument('--output', type=Path,
                        help='Keyframe result directory; default: next to input as <map>_clean (numbered if present)')
    parser.add_argument('--export-global', action='store_true',
                        help='Keyframe mode: also write a complete global_clean.pcd, overriding compact storage')
    parser.add_argument('--compact-result',action='store_true',
                        help='Keyframe mode: save only final global map and diagnostics, omit frame copies/debug clouds')
    parser.add_argument('--global-voxel-size',type=float,default=0.,
                        help='World voxel size for global export; compact-result defaults to 0.10m; 0 keeps all points')
    args = parser.parse_args()
    with open(args.config) as config_file:
        cfg = yaml.safe_load(config_file)
    association_options(cfg)
    evidence = evidence_options(cfg)
    quality = quality_options(cfg)
    trajectory_options(cfg)
    cross_options = cross_frame_options(cfg)
    mode = args.mode or cfg.get('clean_mode', 'map')
    if mode not in ('map', 'keyframes'):
        raise ValueError('clean_mode must be map or keyframes')
    legacy=legacy_enabled(cfg)
    if legacy and (mode!='map' or evidence['enabled'] or quality['enabled'] or cross_options['enabled']):
        parser.error('legacy_map_cleanup requires map mode and disabled extra semantic/quality/cross-frame gates')
    if mode != 'keyframes' and cross_options['enabled']:
        parser.error('cross_keyframe_semantics requires keyframes mode')
    if mode != 'keyframes' and (args.output is not None or args.export_global or args.compact_result or args.global_voxel_size):
        parser.error('Output layout/global export options require keyframes mode')
    map_dir = Path(args.map)
    T_imu_lidar, frame_contract = load_frame_contract(map_dir)
    cfg['_T_imu_lidar'] = T_imu_lidar.tolist()
    selector = ViewSelector(cfg)
    print(f'coordinate contract: {frame_contract}', flush=True)

    if mode == 'keyframes':
        from clean_map_keyframes import run_keyframe_clean
        run_keyframe_clean(args, cfg, frame_contract)
        return

    traj = load_trajectory(map_dir, cfg, Trajectory)
    print(f'Image pose trajectory: {traj.source}; max gap={traj.max_gap}s', flush=True)

    from ultralytics import YOLO
    model = YOLO(cfg['yolo_model'])
    if model.task != 'segment':
        raise RuntimeError(f'{cfg["yolo_model"]} is a {model.task} model; use a -seg model')

    header, cloud = read_pcd(map_dir / 'global.pcd')
    xyz = np.column_stack([cloud['x'], cloud['y'], cloud['z']]).astype(float)
    finite = np.isfinite(xyz).all(axis=1)
    if not finite.any():
        raise ValueError('Map contains no finite XYZ points')
    # Invalid points must not bias the centroid of the voxel around the origin.
    _, point_voxel = np.unique(voxel_keys(xyz[finite], cfg['voxel_size']), axis=0, return_inverse=True)
    n_vox = point_voxel.max() + 1
    centroids = np.zeros((n_vox, 3))
    np.add.at(centroids, point_voxel, xyz[finite])
    centroids /= np.bincount(point_voxel, minlength=n_vox)[:, None]
    print(f'map: {len(cloud)} points, {n_vox} voxels, {len(traj.t)} keyframes', flush=True)

    voter = VehicleVoter(cfg, centroids, model, args.device)
    cache_options = cfg.get('cache', {})
    if cache_options.get('enabled', False) and not args.no_cache:
        weights = Path(getattr(model, 'ckpt_path', None) or cfg['yolo_model'])
        if not weights.is_file():
            raise RuntimeError('Cannot fingerprint model weights for cache; use --no-cache')
        identity = dict(schema=2, instance_masks=True, imgsz=int(cfg.get("yolo_imgsz", 640)),
                        max_det=100, weights=file_digest(weights),
                        ultralytics=importlib.metadata.version('ultralytics'),
                        torch=importlib.metadata.version('torch'), device=args.device,
                        conf=cfg['yolo_conf'], classes=cfg['vehicle_classes'],
                        width=cfg['width'], height=cfg['height'], retina_masks=True)
        voter.cache = instance_cache(cache_options.get('directory') or map_dir / '.vehicle_mask_cache', identity, cfg)
    viz = None
    if args.rviz:
        from clean_map_rviz import RvizPublisher
        viz = RvizPublisher(max_points=int(cfg.get('rviz_max_points', 300000)))
        viz.publish_trajectory(traj)
        viz.publish_cloud('raw', xyz[finite])
    start, with_depth, no_projection = time.monotonic(), 0, 0
    diagnostics = cfg.get('diagnostics', {})
    every = max(1, int(diagnostics.get('every_views', 20)))
    max_overlays = max(0, int(diagnostics.get('max_overlays', 20)))
    diagnostic_root = map_dir / 'clean_diagnostics'
    diagnostic_root.mkdir(exist_ok=True)
    diagnostic_dir = Path(tempfile.mkdtemp(prefix='run_', dir=diagnostic_root))
    print(f'Diagnostics: {diagnostic_dir}', flush=True)
    overlays = 0
    last_log = start
    print('Reading bag and selecting views; first CPU inference may take a while...', flush=True)
    with open(diagnostic_dir / 'views.csv', 'w', newline='') as stream:
        columns = ['timestamp', 'status', 'objects', 'projected', 'visible', 'occluded',
                   'vehicle_votes', 'background_votes', 'unknown_votes', 'depth', 'seconds',
                   'aligned_timestamp', 'nearest_keyframe_dt_s', 'pose_bracket_gap_s',
                   'accepted_objects', 'positive_confidence_sum', 'pose_source', 'pose_status',
                   'nearest_pose_dt_s', 'keyframe_bracket_gap_s', 'pose_correction']
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for t, pose, image, depth in iterate_views(args.bag, cfg, traj, selector):
            before = time.monotonic()
            n_obj = voter.add_view(pose, image, depth)
            with_depth += depth is not None
            no_projection += n_obj is None
            stats = voter.last_stats
            writer.writerow(dict(timestamp=f'{t:.9f}', status='no_projection' if n_obj is None else 'voted',
                                 **stats, depth=int(depth is not None), seconds=time.monotonic()-before,
                                 **timing_stats(traj, t, cfg)))
            stream.flush()
            if n_obj is not None:
                if viz is not None:
                    viz.publish_view(t, pose, voter, image)
                if overlays < max_overlays and (voter.views == 1 or voter.views % every == 0):
                    cv2.imwrite(str(diagnostic_dir / f'overlay_{overlays:03d}.jpg'), make_overlay(voter, image, t, traj))
                    overlays += 1
            now = time.monotonic()
            if voter.views == 1 or now - last_log >= 10:
                cache = getattr(voter, 'cache', None)
                progress = np.clip((t + cfg['time_offset'] - traj.t[0]) /
                                   max(traj.t[-1] - traj.t[0], 1e-9), 0, 1)
                print(f'progress={100*progress:.1f}% of trajectory time, views={voter.views} '
                      f'no_projection={no_projection} objects={n_obj} '
                      f'cache_hits={cache.hits if cache else 0} '
                      f'{(now-start)/max(voter.views, 1):.2f}s/view; {selector.stats}', flush=True)
                last_log = now
    (diagnostic_dir/'pose_failures.json').write_text(json.dumps(traj.failure_samples,indent=2,allow_nan=False)+'\n')
    if voter.views == 0:
        raise RuntimeError(f'No usable projected view: selection={selector.stats}, '
                           f'no_projection={no_projection}; check time_offset, calibration and map frames')

    print('Voting finished; estimating floor and growing vehicle components...', flush=True)
    floor = centroids[:, 2] < estimate_floor(centroids, traj) + cfg['floor_keep']
    if legacy:
        observed,remove,uncertain,evidence_counts=legacy_votes(voter.visible,voter.vehicle,floor,cfg)
    else:
        observed, remove, uncertain, evidence_counts = decide(
            voter.visible, voter.vehicle, voter.background, cfg, voter.baseline, floor,
            positive_score=getattr(voter, 'score_sum', None))
    by_voxel = int(remove.sum())

    car_comp = np.zeros(0, bool)
    if not evidence['enabled'] and not quality['enabled']:
        above = np.nonzero(~floor)[0]
        label = components(centroids[above], cfg['cluster_voxel'])
        n_comp = int(label.max()) + 1 if len(label) else 0
        obs_cnt = np.bincount(label, weights=observed[above], minlength=n_comp)
        veh_cnt = np.bincount(label, weights=remove[above], minlength=n_comp)
        lo = np.full((n_comp, 3), np.inf)
        hi = np.full((n_comp, 3), -np.inf)
        np.minimum.at(lo, label, centroids[above])
        np.maximum.at(hi, label, centroids[above])
        ext = hi - lo
        ext = np.column_stack([np.sort(ext[:, :2], axis=1)[:, ::-1], ext[:, 2]])
        lim = np.array(cfg['cluster_max_extent'], dtype=float)
        lim = np.concatenate([np.sort(lim[:2])[::-1], lim[2:]])
        car_comp = (obs_cnt >= cfg['cluster_min_observed']) & \
                   (veh_cnt >= cfg['cluster_vehicle_ratio'] * obs_cnt) & (ext <= lim).all(axis=1)
        # Historical mode grows the whole bounded component; modern mode vetoes background.
        growth = car_comp[label] if legacy else car_comp[label] & (~observed[above] | remove[above]) & (voter.background[above] == 0)
        remove[above[growth]] = True
    uncertain &= ~remove
    evidence_counts['removed'] = int(remove.sum())
    evidence_counts['uncertain'] = int(uncertain.sum())

    drop = np.zeros(len(cloud), dtype=bool)
    drop[finite] = remove[point_voxel]
    write_pcd(map_dir / 'global_clean.pcd', header, cloud, ~drop)
    write_pcd(map_dir / 'global_vehicles.pcd', header, cloud, drop)
    if evidence['write_uncertain']:
        uncertain_points = np.zeros(len(cloud), bool)
        uncertain_points[finite] = uncertain[point_voxel]
        write_pcd(diagnostic_dir / 'global_uncertain.pcd', header, cloud, uncertain_points)
    print(f'views={voter.views}, observed voxels={observed.sum()}/{n_vox}, '
          f'vehicle voxels: vote={by_voxel}, after clusters={remove.sum()} ({car_comp.sum()} vehicle clusters)')
    print(f'removed {drop.sum()}/{len(cloud)} points -> {map_dir / "global_clean.pcd"}')
    cache = getattr(voter, 'cache', None)
    summary = dict(evidence=evidence_counts, evidence_units="voxels",
                   semantics=report_semantics(), cluster_growth_enabled=not evidence["enabled"] and not quality['enabled'],
                   uncertain_cloud_written=evidence["write_uncertain"],
                   coordinate_contract=frame_contract, views=voter.views,
                   selection=selector.stats, no_projection=no_projection,
                   cache_hits=cache.hits if cache else 0, cache_misses=cache.misses if cache else 0,
                   cache_corrupt=cache.corrupt if cache else 0,
                   removed_points=int(drop.sum()), total_points=len(cloud),
                   observed_voxels=int(observed.sum()), total_voxels=int(n_vox),
                   elapsed_seconds=time.monotonic()-start, overlays_written=overlays,
                   config=cfg, with_depth=with_depth, invalid_points=int((~finite).sum()))
    summary.update(trajectory=traj.description, pose_queries=traj.query_stats)
    summary['legacy_map_cleanup']=legacy
    (diagnostic_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    if viz is not None:
        viz.publish_cloud('clean', xyz[finite & ~drop])
        viz.publish_cloud('vehicles', xyz[drop])
        print('RViz topics are latched; Ctrl+C to exit', flush=True)
        viz.spin()


if __name__ == '__main__':
    main()

