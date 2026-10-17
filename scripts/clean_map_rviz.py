#!/usr/bin/env python3
"""离线去车结果的 RViz 可视化 (配合 config/clean_map.rviz)。

话题 (frame: map, 点云和轨迹为 transient local，RViz 后启动也能收到):
  /clean_map/raw        原始地图
  /clean_map/clean      去车后的地图
  /clean_map/vehicles   被删除的点
  /clean_map/votes      处理过程中得到车辆票的体素，intensity = 车辆票占比
  /clean_map/trajectory 关键帧轨迹
  /clean_map/camera     当前处理的相机位置
  /clean_map/overlay    当前图像: 蓝色=分割掩码, 绿点=可见体素, 红点=投车辆票的体素

单独查看已有结果:
  python3 scripts/clean_map_rviz.py --map data/<地图目录>
  rviz2 -d config/clean_map.rviz
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header


def mat_to_pose(T, header):
    from clean_map_vehicles import mat_to_quat
    msg = PoseStamped(header=header)
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, T[:3, 3])
    q = mat_to_quat(T[:3, :3])
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, q)
    return msg


class RvizPublisher:
    def __init__(self, frame='map'):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('clean_map_viz')
        self.frame = frame
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clouds = {name: self.node.create_publisher(PointCloud2, f'/clean_map/{name}', latched)
                       for name in ('raw', 'clean', 'vehicles', 'votes')}
        self.path_pub = self.node.create_publisher(PathMsg, '/clean_map/trajectory', latched)
        self.cam_pub = self.node.create_publisher(PoseStamped, '/clean_map/camera', 10)
        self.overlay_pub = self.node.create_publisher(Image, '/clean_map/overlay', 10)
        self.count = 0

    def header(self):
        return Header(frame_id=self.frame, stamp=self.node.get_clock().now().to_msg())

    def publish_cloud(self, name, xyz, intensity=None, max_points=3000000):
        xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
        if len(xyz) > max_points:  # 大地图按 5cm 体素降采样，RViz 才显示得动
            from clean_map_vehicles import voxel_keys
            _, keep = np.unique(voxel_keys(xyz.astype(float), 0.05), return_index=True)
            xyz = xyz[keep]
            intensity = None if intensity is None else np.asarray(intensity)[keep]
        fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        data = np.zeros((len(xyz), 4), np.float32)
        data[:, :3] = xyz
        if intensity is not None:
            data[:, 3] = intensity
        msg = PointCloud2(header=self.header(), height=1, width=len(xyz), fields=fields,
                          is_bigendian=False, point_step=16, row_step=16 * len(xyz),
                          data=data.tobytes(), is_dense=True)
        self.clouds[name].publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0)

    def publish_trajectory(self, traj):
        from clean_map_vehicles import quat_to_mat
        msg = PathMsg(header=self.header())
        for p, q in zip(traj.p, traj.q):
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = quat_to_mat(q), p
            msg.poses.append(mat_to_pose(T, msg.header))
        self.path_pub.publish(msg)

    def publish_view(self, t, T_world_lidar, voter, image):
        u, v, visible, is_vehicle, mask = voter.last_view
        vis = image.copy()
        vis[mask] = (0.5 * vis[mask] + [127, 60, 0]).astype(np.uint8)
        vis[v[visible & ~is_vehicle], u[visible & ~is_vehicle]] = (0, 255, 0)
        hit = visible & is_vehicle
        for x, y in zip(u[hit], v[hit]):
            cv2.circle(vis, (int(x), int(y)), 2, (0, 0, 255), -1)
        cv2.putText(vis, f'view {voter.views}  t={t:.1f}  vehicle votes={int(hit.sum())}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        h = self.header()
        msg = Image(header=h, height=vis.shape[0], width=vis.shape[1], encoding='bgr8',
                    is_bigendian=0, step=vis.shape[1] * 3, data=vis.tobytes())
        self.overlay_pub.publish(msg)
        self.cam_pub.publish(mat_to_pose(T_world_lidar @ np.linalg.inv(voter.T_cam_lidar), h))
        self.count += 1
        if self.count % 10 == 0:
            voted = voter.vehicle > 0
            ratio = voter.vehicle[voted] / np.maximum(voter.visible[voted], 1)
            self.publish_cloud('votes', voter.centroids[voted], ratio)
        rclpy.spin_once(self.node, timeout_sec=0)

    def spin(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map', required=True)
    args = parser.parse_args()
    from clean_map_vehicles import Trajectory, read_pcd
    map_dir = Path(args.map)
    viz = RvizPublisher()
    for name, file in (('raw', 'global.pcd'), ('clean', 'global_clean.pcd'), ('vehicles', 'global_vehicles.pcd')):
        if (map_dir / file).exists():
            _, cloud = read_pcd(map_dir / file)
            xyz = np.column_stack([cloud['x'], cloud['y'], cloud['z']])
            viz.publish_cloud(name, xyz[np.isfinite(xyz).all(axis=1)])
            print(f'/clean_map/{name}: {len(cloud)} points from {file}', flush=True)
    if (map_dir / 'keyframes.txt').exists():
        viz.publish_trajectory(Trajectory(map_dir / 'keyframes.txt', np.inf))
    print('Published (latched). Ctrl+C to exit.', flush=True)
    viz.spin()


if __name__ == '__main__':
    main()
