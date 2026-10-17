#!/usr/bin/env python3
"""离线去除 SLAM 地图中的车辆点云 (地下车库)。

SLAM 用完整点云建图，结束后用 SaveMap 导出的 global.pcd + keyframes.txt，
回放 bag 中的彩色图 (和对齐深度图)，YOLO 分割车辆，把地图体素投影到每个视角投票：
  - 可见次数: 体素在视野内且未被遮挡 (深度图一致性，深度无效时用地图 z-buffer)
  - 车辆次数: 可见且落在车辆掩码内
车辆占比超过阈值的体素删除；整簇大部分是车辆时整簇删除，覆盖相机没看到的车身。
贴地一层不删，避免地面出现空洞。

用法:
  python3 scripts/clean_map_vehicles.py --map data/garage --bag <bag目录或.bag> \
      --config config/garage_clean.yaml [--rviz]
输出: <map>/global_clean.pcd, <map>/global_vehicles.pcd (被删除的点, 用于检查)
"""
import argparse
import subprocess
import tempfile
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml

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


def write_pcd(path, header, cloud):
    lines = []
    for line in header:
        key = line.split()[0] if line else ''
        if key == 'WIDTH':
            line = f'WIDTH {len(cloud)}'
        elif key == 'HEIGHT':
            line = 'HEIGHT 1'
        elif key == 'POINTS':
            line = f'POINTS {len(cloud)}'
        elif key == 'DATA':
            line = 'DATA binary'
        lines.append(line)
    with open(path, 'wb') as f:
        f.write(('\n'.join(lines) + '\n').encode('ascii'))
        f.write(cloud.tobytes())


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
        self.T_cam_lidar = np.array(cfg['T_cam_lidar'], dtype=float).reshape(4, 4)
        self.w, self.h = int(cfg['width']), int(cfg['height'])
        # 归一化坐标范围，防止畸变模型把视野外的点折回图像内
        self.x_lim = 1.2 * max(self.K[0, 2], self.w - self.K[0, 2]) / self.K[0, 0]
        self.y_lim = 1.2 * max(self.K[1, 2], self.h - self.K[1, 2]) / self.K[1, 1]
        self.visible = np.zeros(len(centroids), np.int32)
        self.vehicle = np.zeros(len(centroids), np.int32)
        self.kernel = np.ones((2 * cfg['mask_dilate_px'] + 1,) * 2, np.uint8)
        self.views = 0

    def vehicle_mask(self, image):
        r = self.model(image, conf=self.cfg['yolo_conf'], classes=self.cfg['vehicle_classes'],
                       retina_masks=True, verbose=False, device=self.device)[0]
        if r.masks is None:
            return np.zeros((self.h, self.w), bool), 0
        mask = r.masks.data.cpu().numpy().any(axis=0).astype(np.uint8)
        if mask.shape != (self.h, self.w):
            mask = cv2.resize(mask, (self.w, self.h), interpolation=cv2.INTER_NEAREST)
        return mask > 0, len(r.masks)

    def add_view(self, T_world_lidar, image, depth):
        if image.shape[:2] != (self.h, self.w):
            raise RuntimeError(f'Image {image.shape[1]}x{image.shape[0]} != calibration {self.w}x{self.h}')
        T = self.T_cam_lidar @ np.linalg.inv(T_world_lidar)
        rng = self.cfg['max_range']
        near = np.asarray(self.spatial_index.query_ball_point(
            T_world_lidar[:3, 3], rng, p=np.inf), dtype=np.intp)
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

        # 掩码膨胀用最小值滤波实现：膨胀区域内只有不深于车辆前表面 + depth_margin 的体素才算车辆，
        # 避免轮廓边缘和车窗后面的墙柱被误删
        mask, n_obj = self.vehicle_mask(image)
        front = np.where(mask, scene, 1e6).astype(np.float32)
        front = cv2.erode(front, self.kernel)
        is_vehicle = (front[v, u] < 1e5) & (z <= front[v, u] + self.cfg['depth_margin'])
        self.visible[near[visible]] += 1
        self.vehicle[near[visible & is_vehicle]] += 1
        self.views += 1
        self.last_view = (u, v, visible, is_vehicle, mask)
        return n_obj


def iterate_views(bag, cfg, traj):
    """按时间顺序产出 (pose, color, depth)，每 image_period 秒一张。"""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    color_topic, depth_topic = cfg['color_topic'], cfg.get('depth_topic') or None
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conns = {c.topic: c for c in reader.connections}
        if color_topic not in conns:
            raise RuntimeError(f'{color_topic} not in bag; topics: {sorted(conns)}')
        if depth_topic and depth_topic not in conns:
            raise RuntimeError(f'{depth_topic} not in bag; set depth_topic empty to use z-buffer only')
        wanted = [conns[color_topic]] + ([conns[depth_topic]] if depth_topic else [])
        depths, pending, last_t = deque(maxlen=60), deque(), -np.inf

        def flush(now):
            while pending and (depth_topic is None or now - pending[0][0] > 0.1):
                t, msg = pending.popleft()
                pose = traj.pose(t + cfg['time_offset'])
                if pose is None:
                    continue
                depth = None
                if depths:
                    td, dmsg = min(depths, key=lambda item: abs(item[0] - t))
                    if abs(td - t) < 0.04:
                        depth = decode_image(dmsg, depth=True)
                yield t, pose, decode_image(msg), depth

        for conn, _, raw in reader.messages(connections=wanted):
            msg = reader.deserialize(raw, conn.msgtype)
            t = stamp(msg)
            if conn.topic == depth_topic:
                depths.append((t, msg))
            elif t - last_t >= cfg['image_period']:
                pending.append((t, msg))
                last_t = t
            yield from flush(t)
        yield from flush(np.inf)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map', required=True, help='SaveMap 输出目录 (含 global.pcd, keyframes.txt)')
    parser.add_argument('--bag', required=True, help='建图时录制的 bag (ROS1 .bag 或 ROS2 目录)')
    parser.add_argument('--config', default='config/garage_clean.yaml')
    parser.add_argument('--device', default=None, help='YOLO 设备, 如 cpu / 0')
    parser.add_argument('--rviz', action='store_true', help='在 RViz 中实时显示 (见 config/clean_map.rviz)')
    args = parser.parse_args()
    with open(args.config) as config_file:
        cfg = yaml.safe_load(config_file)
    map_dir = Path(args.map)

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
    traj = Trajectory(map_dir / 'keyframes.txt', cfg['max_kf_gap'])
    print(f'map: {len(cloud)} points, {n_vox} voxels, {len(traj.t)} keyframes', flush=True)

    voter = VehicleVoter(cfg, centroids, model, args.device)
    viz = None
    if args.rviz:
        from clean_map_rviz import RvizPublisher
        viz = RvizPublisher()
        viz.publish_trajectory(traj)
        viz.publish_cloud('raw', xyz[finite])
    start, with_depth = time.time(), 0
    for t, pose, image, depth in iterate_views(args.bag, cfg, traj):
        n_obj = voter.add_view(pose, image, depth)
        with_depth += depth is not None
        if viz is not None and n_obj is not None:
            viz.publish_view(t, pose, voter, image)
        if voter.views % 50 == 0:
            print(f'views={voter.views} (depth {with_depth}) t={t:.2f} vehicles={n_obj} '
                  f'{(time.time() - start) / voter.views:.2f}s/view', flush=True)
    if voter.views == 0:
        raise RuntimeError('No image matched the keyframe time range; check topics and time_offset')

    observed = voter.visible >= cfg['min_views']
    ratio = voter.vehicle / np.maximum(voter.visible, 1)
    remove = observed & (ratio >= cfg['vehicle_ratio'])
    floor = centroids[:, 2] < estimate_floor(centroids, traj) + cfg['floor_keep']
    remove &= ~floor
    by_voxel = int(remove.sum())

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
    remove[above[car_comp[label]]] = True

    drop = np.zeros(len(cloud), dtype=bool)
    drop[finite] = remove[point_voxel]
    write_pcd(map_dir / 'global_clean.pcd', header, cloud[~drop])
    write_pcd(map_dir / 'global_vehicles.pcd', header, cloud[drop])
    print(f'views={voter.views}, observed voxels={observed.sum()}/{n_vox}, '
          f'vehicle voxels: vote={by_voxel}, after clusters={remove.sum()} ({car_comp.sum()} vehicle clusters)')
    print(f'removed {drop.sum()}/{len(cloud)} points -> {map_dir / "global_clean.pcd"}')
    if viz is not None:
        viz.publish_cloud('clean', xyz[finite & ~drop])
        viz.publish_cloud('vehicles', xyz[drop])
        print('RViz topics are latched; Ctrl+C to exit', flush=True)
        viz.spin()


if __name__ == '__main__':
    main()

