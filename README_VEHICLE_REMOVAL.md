# Lightning-LM 车辆去除系统

基于 Lightning-LM SLAM 的离线多视角投票车辆点云去除系统，专为地下车库静态建图设计。

## 核心特性

- **离线多视角投票**：解决相机 FOV 窄、单帧误检问题
- **深度感知过滤**：避免误删车窗后的墙壁和柱子
- **簇级扩展**：删除相机遮挡的车身侧面
- **地面保护**：保留贴地层，避免地面空洞
- **实时可视化**：RViz 实时查看投影和投票过程

## 快速开始

### 1. 安装依赖

```bash
# Python 依赖
pip install ultralytics opencv-python numpy scipy pyyaml rosbags

# ROS2 (可视化，可选)
sudo apt install ros-humble-desktop pcl-tools
```

### 2. SLAM 建图

```bash
# 使用完整点云建图（不过滤车辆）
ros2 launch faster_lio mapping.launch.py config:=config/default_robosense.yaml

# 保存地图
ros2 service call /save_map std_srvs/srv/Trigger
```

输出：
- `data/<map_id>/global.pcd` - 地图点云
- `data/<map_id>/keyframes.txt` - 关键帧位姿

### 3. 配置参数

编辑 `config/garage_clean.yaml`，填入真实标定参数：

```yaml
# 相机内参（从 camera_info 或标定工具获取）
K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion: [k1, k2, p1, p2, k3]

# 雷达-相机外参（4x4 变换矩阵）
T_cam_lidar: [r11, r12, r13, tx,
              r21, r22, r23, ty,
              r31, r32, r33, tz,
              0.0, 0.0, 0.0, 1.0]
```

### 4. 运行车辆去除

```bash
python3 scripts/clean_map_vehicles.py \
  --map data/<map_id> \
  --bag /path/to/bag \
  --config config/garage_clean.yaml \
  --device 0  # GPU 设备号，或 cpu
```

输出：
- `data/<map_id>/global_clean.pcd` - 干净地图
- `data/<map_id>/global_vehicles.pcd` - 被删除的车辆点

### 5. 可视化结果

```bash
# 终端 1: 发布点云
python3 scripts/clean_map_rviz.py --map data/<map_id>

# 终端 2: 启动 RViz
rviz2 -d config/clean_map.rviz
```

## 实时可视化模式

在处理过程中查看投影效果：

```bash
# 终端 1: 启动 RViz
rviz2 -d config/clean_map.rviz

# 终端 2: 运行车辆去除（带 --rviz）
python3 scripts/clean_map_vehicles.py \
  --map data/<map_id> \
  --bag /path/to/bag \
  --config config/garage_clean.yaml \
  --device 0 \
  --rviz
```

查看：
- **Overlay 图像**：蓝色=掩码，绿点=可见体素，红点=车辆票
- **投票进度**：体素颜色从黄到紫（票数累积）
- **相机位姿**：彩色坐标轴

## 参数调优

### 车辆漏删（假阴性）

```yaml
vehicle_ratio: 0.4     # 降低投票阈值（从 0.5）
min_views: 2           # 降低最小视角数（从 3）
image_period: 0.1      # 增加采样密度（从 0.2）
```

### 静态物体误删（假阳性）

```yaml
vehicle_ratio: 0.6     # 提高投票阈值（从 0.5）
min_views: 5           # 提高最小视角数（从 3）
mask_dilate_px: 10     # 增大掩码膨胀（从 7）
depth_margin: 1.5      # 降低深度容差（从 2.0）
```

### 标定误差补偿

```yaml
mask_dilate_px: 10     # 增大掩码膨胀
time_offset: 0.05      # 调整时间戳偏差
```

## 项目结构

```
lightning-lm-fresh/
├── scripts/
│   ├── clean_map_vehicles.py   # 车辆去除主脚本
│   ├── clean_map_rviz.py       # RViz 可视化
│   └── install_dep.sh          # 依赖安装
├── config/
│   ├── garage_clean.yaml       # 地下车库配置（模板）
│   ├── m2dgr_clean.yaml        # M2DGR 测试配置
│   ├── clean_map.rviz          # RViz 配置
│   └── default_robosense.yaml  # SLAM 配置
├── doc/
│   └── VEHICLE_REMOVAL_GUIDE.md  # 完整操作指南
└── src/
    └── core/system/slam.cc     # SLAM (SaveMap 导出 keyframes.txt)
```

## 算法原理

1. **SLAM 建图**：使用完整点云（包含车辆）进行精确定位和建图
2. **地图体素化**：将地图点云转为体素（默认 0.1m）
3. **多视角投票**：
   - 回放 bag 图像，YOLO 分割车辆
   - 投影地图体素到每个视角，统计可见次数和车辆次数
   - 深度一致性检查（深度图或 z-buffer）判断遮挡
4. **投票删除**：车辆票占比 ≥ 阈值的体素删除
5. **簇级扩展**：整簇大部分是车辆时，整簇删除（覆盖相机看不到的侧面）
6. **地面保护**：贴地 12cm 不删除

## 硬件要求

- **LiDAR**: 速腾雷达或其他 3D 激光雷达
- **相机**: 奥比中光深度相机或其他 RGB-D 相机
  - 彩色图像（YOLO 检测）
  - 对齐深度图（推荐，精确遮挡判断）
- **GPU**: 推荐（YOLO 加速 5-10 倍）

## 测试结果

### M2DGR walk_01（室外校园）
- 地图：19,968,359 点，8,090,897 体素，467 关键帧
- 处理：1168 视图，速度 0.27-0.36 秒/帧（CPU）
- 结果：删除 10,222 点（0.05%），0 个车辆簇
- 状态：✅ 正常（校园场景车辆稀疏）

### 合成数据（地下车库场景）
- 精度：100% 车辆删除，0.00-0.09% 静态误删
- 深度感知：完美保护墙壁和柱子

## 常见问题

**Q: 没有深度相机可以用吗？**  
A: 可以。设置 `depth_topic: ""`，使用地图 z-buffer 判断遮挡。精度略降但仍有效。

**Q: YOLO 检测不到车辆？**  
A: 降低 `yolo_conf: 0.15`，或使用更大模型 `yolo26m-seg.pt`。

**Q: 处理速度太慢？**  
A: 使用 GPU（`--device 0`），或降低采样密度（`image_period: 0.5`）。

**Q: 标定精度要求？**  
A: 内参重投影误差 < 0.5px，外参平移 < 5cm、旋转 < 2°。

## 文档

详细操作流程、参数说明、调优指南，请参阅：

📖 **[完整操作指南](doc/VEHICLE_REMOVAL_GUIDE.md)**

## 许可

基于 Lightning-LM 开源项目。

## 致谢

- **Lightning-LM**: 高效的激光-IMU SLAM 系统
- **YOLOv8**: Ultralytics 语义分割模型
- **M2DGR**: 多模态数据集用于测试
