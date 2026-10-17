# Lightning-LM SLAM 快速使用指南

## 1. 编译

```bash
cd ~/lightning-lm-fresh
./build_and_run.sh
```

或者手动编译：

```bash
cd ~/  # 回到工作空间上层
source /opt/ros/humble/setup.bash
colcon build --packages-select lightning --symlink-install
source install/setup.bash
```

## 2. 运行 SLAM

### 方式 1: 使用 launch 文件

```bash
# 使用默认配置
ros2 launch lightning mapping.launch.py

# 使用自定义配置
ros2 launch lightning mapping.launch.py config:=config/default_robosense.yaml
```

### 方式 2: 直接运行节点

```bash
ros2 run lightning run_slam_online --ros-args --params-file config/default.yaml
```

## 3. 保存地图

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

输出文件：
- `data/<map_id>/global.pcd` - 全局地图
- `data/<map_id>/keyframes.txt` - 关键帧位姿
- `data/<map_id>/*.pcd` - 各个关键帧点云

## 4. 话题订阅

SLAM 需要订阅：
- `/rslidar_points` 或配置中的 `lidar_topic` - 点云
- `/imu/data` 或配置中的 `imu_topic` - IMU 数据

## 5. 配置文件

### 可用配置
- `config/default.yaml` - 默认 LiDAR 配置
- `config/default_robosense.yaml` - 速腾雷达配置
- `config/garage_clean.yaml` - 地下车库（车辆去除用）

### 主要参数

```yaml
common:
  lidar_topic: /rslidar_points      # 点云话题
  imu_topic: /imu/data              # IMU 话题
  
fasterlio:
  lidar_type: 4                     # 0=Avia 1=Velodyne 2=Ouster 3=Robosense 4=其他
  filter_size_map: 0.5              # 地图体素大小
  ivox_grid_resolution: 0.5         # iVox 网格分辨率
  
system:
  map_path: data/default_map        # 地图保存路径
  with_loop_closing: true           # 是否启用回环检测
  with_ui: true                     # 是否启用可视化
```

## 6. 常见问题

### Q: Package 'lightning' not found

需要先编译并 source 工作空间：
```bash
cd ~/  # 工作空间上层
colcon build --packages-select lightning
source install/setup.bash
```

### Q: 没有收到点云数据

检查话题名称：
```bash
ros2 topic list
ros2 topic echo /rslidar_points --no-arr
```

修改配置文件中的 `lidar_topic` 和 `imu_topic`。

### Q: 如何使用车辆去除系统

SLAM 建图完成后：

```bash
# 1. 保存地图
ros2 service call /save_map std_srvs/srv/Trigger

# 2. 运行车辆去除
./run_vehicle_removal.sh \
  --map data/my_map \
  --bag /path/to/bag \
  --config config/garage_clean.yaml \
  --rviz
```

详见 `doc/VEHICLE_REMOVAL_GUIDE.md`

## 7. 可执行文件

- `run_slam_online` - 在线 SLAM（从 ROS 话题）
- `run_slam_offline` - 离线 SLAM（从 bag 文件）
- `run_loc_online` - 在线定位
- `run_loc_offline` - 离线定位

## 8. 相关文档

- **车辆去除系统**: `doc/VEHICLE_REMOVAL_GUIDE.md`
- **快速开始**: `doc/QUICKSTART.md`
- **项目概述**: `README_VEHICLE_REMOVAL.md`
