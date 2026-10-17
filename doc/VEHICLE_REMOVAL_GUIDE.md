# 地下车库地图车辆点云去除指南

## 概述

本系统提供离线多视角投票算法，用于从 SLAM 建图结果中去除车辆点云，生成干净的地下车库静态地图。

**核心思想：**
- SLAM 使用完整点云进行精确定位和建图
- 建图完成后，离线回放图像，YOLO 分割车辆，投影地图体素到多个视角进行投票
- 多视角投票可以处理相机视野限制（FOV 窄）和单帧误检问题
- 深度感知过滤确保不会误删车窗后面的墙壁和柱子

## 系统要求

### 硬件配置
- **LiDAR**: 速腾雷达（RoboSense）或其他 3D 激光雷达
- **相机**: 奥比中光深度相机（Orbbec）或其他 RGB-D 相机
  - 需要彩色图像（用于 YOLO 检测）
  - 需要对齐的深度图像（推荐，用于精确遮挡判断）
- **处理器**: 建议配备 GPU（YOLO 推理加速），CPU 也可运行但较慢

### 软件依赖
```bash
# Python 依赖
pip install ultralytics opencv-python numpy scipy pyyaml rosbags

# ROS2 (用于可视化，可选)
sudo apt install ros-humble-desktop

# PCL 工具 (用于读取 binary_compressed 格式)
sudo apt install pcl-tools
```

## 完整操作流程

### 步骤 1: 数据采集

录制包含以下话题的 ROS bag：

```bash
# ROS2 录制示例
ros2 bag record \
  /rslidar_points \          # 速腾雷达点云 (需包含 timestamp 字段)
  /camera/color/image_raw \   # 彩色图像 (或 CompressedImage)
  /camera/depth/image_raw \   # 对齐的深度图像 (可选但推荐)
  /imu/data                   # IMU 数据
```

**重要配置：**
- 奥比相机驱动需开启 `depth_registration: true`（深度图对齐到彩色相机）
- 录制原始深度图（`sensor_msgs/Image`），不要用 `compressedDepth`
- 确保时间戳同步（LiDAR、相机、IMU）

### 步骤 2: SLAM 建图

使用完整点云（不做车辆过滤）进行 SLAM 建图：

```bash
# 编辑配置文件，确保不过滤车辆
vim config/garage_mapping.yaml
# 设置: lidar_type: 4  (RoboSense)

# 运行 SLAM
FUSION_FILTER=false RVIZ=1 MAP_ID=my_garage bash scripts/start_slam.sh

# 或直接运行
ros2 launch faster_lio mapping.launch.py config:=config/garage_mapping.yaml
```

建图完成后，使用 `/save_map` 服务保存地图：

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

**输出文件：**
- `data/my_garage/global.pcd` - 完整地图点云
- `data/my_garage/keyframes.txt` - 关键帧位姿（时间戳 + 6DOF 位姿）
- `data/my_garage/*.pcd` - 各个关键帧点云（可选）

### 步骤 3: 相机标定

车辆去除需要精确的相机内参和雷达-相机外参。

#### 3.1 相机内参标定

使用 ROS camera_calibration 或 OpenCV 标定板：

```bash
# ROS2 相机标定
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/camera/color/image_raw
```

记录标定结果：
- `K`: 3x3 内参矩阵
- `distortion`: 畸变系数 [k1, k2, p1, p2, k3]
- `width`, `height`: 图像分辨率

#### 3.2 雷达-相机外参标定

使用标定板或 ACSC (Automatic Camera-LiDAR Calibration) 等工具。

外参格式为 4x4 变换矩阵 `T_cam_lidar`，表示雷达坐标系到相机光学坐标系的变换：
- 相机坐标系：Z 轴朝前，X 轴朝右，Y 轴朝下

### 步骤 4: 配置车辆去除参数

复制模板配置并填入真实标定参数：

```bash
cp config/garage_clean.yaml config/my_garage_clean.yaml
vim config/my_garage_clean.yaml
```

**必须修改的参数：**

```yaml
# 1. 话题名称（根据你的 bag 实际话题）
color_topic: /camera/color/image_raw
depth_topic: /camera/depth/image_raw  # 无深度图则留空
depth_scale: 0.001  # 深度图单位转米 (16UC1 毫米为 0.001)

# 2. 图像分辨率
width: 1280
height: 720

# 3. 相机内参（从标定结果复制）
K: [fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0]
distortion: [k1, k2, p1, p2, k3]

# 4. 雷达-相机外参（从标定结果复制，行优先 4x4）
T_cam_lidar: [r11, r12, r13, tx,
              r21, r22, r23, ty,
              r31, r32, r33, tz,
              0.0, 0.0, 0.0, 1.0]

# 5. 时间戳偏差（如果相机和雷达时钟不同步）
time_offset: 0.0  # t_lidar = t_image + time_offset
```

**可选调优参数：**

```yaml
# YOLO 检测
yolo_model: yolo26n-seg.pt  # 分割模型（必须是 -seg）
yolo_conf: 0.25  # 置信度阈值（多视角投票会过滤误检）
vehicle_classes: [1, 2, 3, 5, 7]  # COCO 类别: 自行车、汽车、摩托车、公交、卡车

# 投票阈值
min_views: 3  # 至少被看到 3 次才参与判定
vehicle_ratio: 0.5  # 50% 视角判为车辆才删除
  # 如果车辆漏删：降低 vehicle_ratio (如 0.4)
  # 如果静态物体误删：提高 min_views (如 5)

# 深度容差
depth_margin: 2.0  # 只删除不深于车辆前表面 + 2m 的体素
mask_dilate_px: 7  # 掩码膨胀像素数，补偿标定误差

# 处理速度
image_period: 0.2  # 每 0.2 秒处理一张图像（越小越密集，处理越慢）
```

### 步骤 5: 运行车辆去除

```bash
python3 scripts/clean_map_vehicles.py \
  --map data/my_garage \
  --bag /path/to/garage.bag \
  --config config/my_garage_clean.yaml \
  --device 0  # GPU 设备号，或 cpu
```

**可选参数：**
- `--rviz`: 启动实时可视化（需要 ROS2 和 RViz2）
- `--device cpu`: 在 CPU 上运行 YOLO（无 GPU 时）
- `--device 0`: 在 GPU 0 上运行（推荐，快 5-10 倍）

**处理时间估计：**
- GPU: ~0.05-0.1 秒/帧
- CPU: ~0.3-0.5 秒/帧
- 示例：500 帧图像，GPU 约 25-50 秒，CPU 约 2.5-4 分钟

**输出文件：**
- `data/my_garage/global_clean.pcd` - 去除车辆后的干净地图
- `data/my_garage/global_vehicles.pcd` - 被删除的车辆点云（用于验证）

**控制台输出示例：**
```
map: 15234567 points, 6123456 voxels, 423 keyframes
views=50 (depth 48) t=1628059267.72 vehicles=2 0.08s/view
views=100 (depth 97) t=1628059279.99 vehicles=3 0.09s/view
...
views=523, observed voxels=1456789/6123456, vehicle voxels: vote=12345, after clusters=15678 (23 vehicle clusters)
removed 234567/15234567 points -> data/my_garage/global_clean.pcd
```

解读输出：
- `(depth 48)`: 48 张图有深度图匹配，其余用地图 z-buffer
- `vehicles=2`: 当前帧检测到 2 辆车
- `vote=12345`: 按投票比例删除 12345 个体素
- `after clusters=15678`: 簇扩展后删除 15678 个体素（包含相机未看到的车身侧面）
- `23 vehicle clusters`: 检测到 23 个车辆连通簇

### 步骤 6: 结果可视化

#### 6.1 RViz 可视化（推荐）

```bash
# 终端 1: 启动点云发布器
python3 scripts/clean_map_rviz.py --map data/my_garage

# 终端 2: 启动 RViz
rviz2 -d config/clean_map.rviz
```

**RViz 显示内容：**
- **Clean map (去车后)**: 按高度彩色编码的干净地图
- **Removed vehicles (被删除的点)**: 红色显示的车辆点云
- **Vehicle votes (车辆票占比)**: 黄色到紫色渐变，显示投票过程
- **Keyframe trajectory**: 绿色轨迹线
- **Raw map (原始地图)**: 灰色原始地图（默认隐藏，可切换）

**检查要点：**
1. 红色点是否确实是车辆？
2. 是否有车辆漏删（假阴性）？
3. 是否有墙壁、柱子误删（假阳性）？
4. 地面是否完整（贴地层应该保留）？

#### 6.2 CloudCompare 可视化

```bash
# 安装 CloudCompare
sudo snap install cloudcompare

# 打开地图
cloudcompare.CloudCompare \
  data/my_garage/global_clean.pcd \
  data/my_garage/global_vehicles.pcd
```

将两个点云设置不同颜色查看对比。

### 步骤 7: 结果评估与调优

#### 7.1 如果车辆漏删（假阴性）

**可能原因：**
1. 投票阈值过高
2. 可见视角太少（相机 FOV 窄、车辆位置偏僻）
3. YOLO 检测失败

**解决方案：**
```yaml
# 降低投票阈值
vehicle_ratio: 0.4  # 从 0.5 降到 0.4
min_views: 2  # 从 3 降到 2

# 增加图像采样密度
image_period: 0.1  # 从 0.2 降到 0.1

# 降低 YOLO 阈值（可能增加误检）
yolo_conf: 0.15  # 从 0.25 降到 0.15
```

#### 7.2 如果静态物体误删（假阳性）

**可能原因：**
1. 标定不准（投影偏移）
2. 时间戳不同步
3. YOLO 误检（如垃圾桶、广告牌）

**解决方案：**
```yaml
# 提高投票阈值
vehicle_ratio: 0.6  # 从 0.5 升到 0.6
min_views: 5  # 从 3 升到 5

# 增加掩码膨胀补偿标定误差
mask_dilate_px: 10  # 从 7 升到 10

# 调整深度容差
depth_margin: 1.5  # 从 2.0 降到 1.5（更保守）
```

**检查标定：**
- 在 RViz 实时模式（`--rviz`）查看 overlay 图像
- 绿色点应该准确落在可见物体上
- 蓝色掩码应该覆盖车辆轮廓
- 如果投影明显偏移，重新标定外参

#### 7.3 如果地面出现空洞

地面保护层可能设置不当：

```yaml
floor_keep: 0.15  # 从 0.12 升到 0.15（保留更厚的地面层）
```

#### 7.4 如果车辆只删了一半

簇扩展参数可能过严：

```yaml
cluster_vehicle_ratio: 0.4  # 从 0.5 降到 0.4
cluster_max_extent: [8.0, 8.0, 3.5]  # 放宽尺寸限制
```

## 实时可视化模式

使用 `--rviz` 参数可以在处理过程中实时查看投影结果：

```bash
# 终端 1: 启动 RViz
rviz2 -d config/clean_map.rviz

# 终端 2: 运行车辆去除（带实时可视化）
python3 scripts/clean_map_vehicles.py \
  --map data/my_garage \
  --bag /path/to/garage.bag \
  --config config/my_garage_clean.yaml \
  --device 0 \
  --rviz
```

**实时显示内容：**
- **Overlay 图像**（右下角）：
  - 蓝色半透明：YOLO 分割掩码
  - 绿色点：可见但非车辆的体素
  - 红色点：投车辆票的体素
- **相机位姿**：彩色坐标轴
- **投票进度**：点云逐渐累积车辆票（黄→紫渐变）

用于调试：
1. 检查投影是否准确（绿色点应该在物体表面）
2. 检查掩码是否正确（蓝色覆盖车辆）
3. 检查深度过滤（车窗后的墙不应该变红）

## 常见问题

### Q1: 输出 `(depth 0)` 表示什么？

没有深度图匹配上（时间戳差异 > 40ms 或 depth_topic 为空）。算法会使用地图 z-buffer 判断遮挡，精度略低但仍然有效。

**解决方案：**
- 检查 `depth_topic` 配置是否正确
- 确保深度图和彩色图时间戳对齐（相机驱动配置）
- 调整 `time_offset` 补偿时钟偏差

### Q2: YOLO 检测不到车辆怎么办？

1. 检查车辆类别是否正确：
   ```yaml
   vehicle_classes: [1, 2, 3, 5, 7]  # COCO 80 类中的车辆
   ```

2. 降低检测阈值：
   ```yaml
   yolo_conf: 0.15  # 默认 0.25
   ```

3. 尝试更大的 YOLO 模型：
   ```yaml
   yolo_model: yolo26m-seg.pt  # 从 yolo26n-seg 升级到 m
   ```

### Q3: 处理速度太慢怎么办？

1. 使用 GPU：
   ```bash
   --device 0  # 而不是 cpu
   ```

2. 降低采样密度：
   ```yaml
   image_period: 0.5  # 从 0.2 升到 0.5（帧数减半）
   ```

3. 使用更小的 YOLO 模型：
   ```yaml
   yolo_model: yolo26n-seg.pt  # nano 模型最快
   ```

### Q4: 内存不足怎么办？

大地图（> 5000 万点）可能导致内存不足。

**解决方案：**
1. 分区处理：手动分割地图，分别处理
2. 增加体素大小：
   ```yaml
   voxel_size: 0.15  # 从 0.1 增加到 0.15
   ```
3. 限制处理范围：
   ```yaml
   max_range: 10.0  # 从 15.0 降到 10.0（只处理近处体素）
   ```

### Q5: 如何处理透明玻璃？

车窗玻璃可能导致激光穿透，误删背后的墙壁。

**解决方案：**
```yaml
depth_margin: 3.0  # 增加深度容差（从 2.0 到 3.0）
vehicle_ratio: 0.6  # 提高投票阈值（减少误删）
```

注意：`depth_margin` 设置过大会保留车内物体（如座椅、方向盘）。

### Q6: 标定精度要求有多高？

- **内参**：建议重投影误差 < 0.5 像素
- **外参**：平移误差 < 5cm，旋转误差 < 2°
- 不够精确的标定可通过增大 `mask_dilate_px` 补偿（如 10-15 像素）

### Q7: 没有深度相机，只有单目相机可以吗？

可以，但精度会降低：

```yaml
depth_topic: ""  # 留空，使用地图 z-buffer
occlusion_tol: 0.5  # 增大容差（从 0.25）
```

地图 z-buffer 依赖地图完整性，如果地图稀疏（如动态物体遮挡导致的空洞），可能误删背景物体。

## 与在线融合的对比

| 特性 | 在线融合 (`lightning_vision_fusion_custommsg.py`) | 离线多视角投票 (`clean_map_vehicles.py`) |
|------|--------------------------------------------------|------------------------------------------|
| **处理时机** | 实时，在 LIO 之前过滤 | 离线，建图后处理 |
| **视野覆盖** | 单帧 ~15%（相机 FOV 窄） | 多视角累积 ~100% |
| **误删风险** | 高（单帧误检即污染地图） | 低（多视角投票过滤误检） |
| **定位影响** | 有（删除 LIO 输入点） | 无（LIO 用完整点云） |
| **车窗处理** | 差（2D 框删除背景） | 好（深度感知过滤） |
| **适用场景** | 快速原型、实时需求 | 精确建图、离线处理 |

**推荐：**
- 地下车库静态建图：使用离线多视角投票（本方案）
- 实时导航避障：使用在线融合（需改进误检过滤）

## 输出格式

### global_clean.pcd
与 `global.pcd` 格式相同，包含字段：
- `x`, `y`, `z`: 坐标（米）
- `intensity`: 激光反射强度
- `time`: 时间戳（相对扫描起始时间）

### global_vehicles.pcd
仅包含被删除的点，格式同上。用于：
1. 验证删除结果（用 RViz 或 CloudCompare 查看）
2. 统计车辆点云数量和分布
3. 调试参数（如果删除错误，检查这个文件）

### keyframes.txt
每行一个关键帧：
```
id timestamp tx ty tz qx qy qz qw
0 1628059267.123456 1.23 -0.45 0.12 0.001 0.002 0.003 0.999
1 1628059267.623456 1.56 -0.43 0.11 0.002 0.003 0.004 0.998
...
```
- `id`: 关键帧序号
- `timestamp`: 时间戳（秒）
- `tx ty tz`: 位置（米）
- `qx qy qz qw`: 姿态四元数

## 下一步

### 使用干净地图进行定位

将 `global_clean.pcd` 作为先验地图用于定位：

```yaml
# config/localization.yaml
system:
  map_path: data/my_garage/global_clean.pcd
  localization_mode: true
```

### 2D 栅格地图生成

从干净的 3D 地图生成 2D 占据栅格：

```bash
# 使用 g2p5 模块生成 2D 地图
# （已集成在 SLAM 中，保存地图时自动生成）
```

### 多层车库处理

如果车库有多层：

1. 方案 A：分层建图
   ```bash
   # 每层单独录制 bag 和建图
   python3 scripts/clean_map_vehicles.py --map data/garage_floor1 ...
   python3 scripts/clean_map_vehicles.py --map data/garage_floor2 ...
   ```

2. 方案 B：整体建图 + 高度分割
   ```bash
   # 建图时记录高度信息，后处理时按 z 坐标分层
   ```

## 参考

- **YOLO 模型**: [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- **Lightning-LM SLAM**: 基于 Faster-LIO 的激光-IMU 里程计
- **M2DGR 数据集**: [M2DGR Dataset](https://github.com/SJTU-ViSYS/M2DGR)

## 许可

本项目遵循 Lightning-LM 的开源许可协议。
