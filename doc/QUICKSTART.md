# 快速开始 - 一键启动脚本

## 最简单的使用方式

```bash
# 基本用法（自动检测 GPU）
./run_vehicle_removal.sh \
  --map data/my_garage \
  --bag /path/to/garage.bag

# CPU 模式（无 GPU 时）
./run_vehicle_removal.sh \
  --map data/my_garage \
  --bag /path/to/garage.bag \
  --cpu

# 实时可视化（推荐）
./run_vehicle_removal.sh \
  --map data/my_garage \
  --bag /path/to/garage.bag \
  --rviz
```

## 功能说明

### 完整处理流程

脚本会自动完成以下步骤：

1. ✓ 检查 Python、依赖、GPU 状态
2. ✓ 验证地图文件（global.pcd + keyframes.txt）
3. ✓ 验证 bag 文件和配置
4. ✓ 运行车辆去除算法
5. ✓ 生成结果文件
6. ✓ （可选）启动 RViz 可视化

**无需手动操作，一条命令搞定！**

### 只查看已有结果

如果已经处理过，只想查看结果：

```bash
./run_vehicle_removal.sh \
  --map data/my_garage \
  --visualize-only
```

自动启动：
- 点云发布器（ROS2 话题）
- RViz2 可视化界面

按 `Ctrl+C` 退出。

### 自定义配置

使用自己的配置文件：

```bash
./run_vehicle_removal.sh \
  --map data/parking \
  --bag ~/data/parking.bag \
  --config config/my_custom_config.yaml
```

## 参数说明

| 参数 | 说明 | 示例 |
|------|------|------|
| `--map <dir>` | SLAM 建图输出目录 | `data/my_garage` |
| `--bag <path>` | ROS bag 路径 | `~/bags/garage.bag` |
| `--config <yaml>` | 配置文件（可选） | `config/garage_clean.yaml` |
| `--device <id>` | GPU 设备号或 cpu | `0` / `1` / `cpu` |
| `--cpu` | 强制使用 CPU | 无参数值 |
| `--rviz` | 启动实时可视化 | 无参数值 |
| `--visualize-only` | 仅查看已有结果 | 无参数值 |
| `--help` | 显示帮助信息 | 无参数值 |

## 输出说明

### 控制台输出

```
╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║     Lightning-LM 车辆去除系统 - 一键启动                  ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝

═══ 步骤 1/5: 检查依赖和输入文件 ═══
✓ Python3: Python 3.10.12
检查 Python 依赖... 通过
检查 GPU... NVIDIA GeForce RTX 3090

检查地图文件...
  ✓ global.pcd: 341M
  ✓ keyframes.txt: 467 个关键帧

检查 bag 文件...
  ✓ ROS2 bag: 21G

检查配置文件...
  ✓ config/garage_clean.yaml

═══ 步骤 2/5: 验证配置 ═══
配置参数:
  彩色图话题: /camera/color/image_raw
  深度图话题: /camera/depth/image_raw
  YOLO 模型: yolo26n-seg.pt
  最小视角数: 3
  车辆判定比例: 0.5

═══ 步骤 3/5: 运行车辆去除算法 ═══
map: 19968359 points, 8090897 voxels, 467 keyframes
views=50 (depth 48) t=1628059267.72 vehicles=2 0.08s/view
views=100 (depth 97) t=1628059279.99 vehicles=3 0.09s/view
...
views=523, observed voxels=1456789/8090897
vehicle voxels: vote=12345, after clusters=15678 (23 vehicle clusters)
removed 234567/19968359 points -> data/my_garage/global_clean.pcd

═══ 步骤 4/5: 处理完成 ═══
处理时间: 3分42秒
  ✓ global_clean.pcd: 450M
  ✓ global_vehicles.pcd: 8.2M

═══ 步骤 5/5: 可视化 ═══
[可视化启动信息或查看命令]
```

### 生成文件

处理完成后，在地图目录生成：

- `global_clean.pcd` - 去车后的干净地图
- `global_vehicles.pcd` - 被删除的车辆点云（红色，用于验证）

## 常见使用场景

### 场景 1: 第一次处理地下车库数据

```bash
# 步骤 1: 确保已完成 SLAM 建图
ros2 service call /save_map std_srvs/srv/Trigger

# 步骤 2: 编辑配置文件，填入真实标定参数
vim config/garage_clean.yaml

# 步骤 3: 一键运行（GPU + 实时可视化）
./run_vehicle_removal.sh \
  --map data/garage_20241012 \
  --bag ~/bags/garage.bag \
  --rviz
```

### 场景 2: 调试参数

```bash
# 修改配置文件中的参数（如 vehicle_ratio, min_views）
vim config/garage_clean.yaml

# 重新运行
./run_vehicle_removal.sh \
  --map data/garage_20241012 \
  --bag ~/bags/garage.bag \
  --cpu  # CPU 模式调试更快
```

### 场景 3: 批量处理多个地图

```bash
#!/bin/bash
# batch_process.sh

for map_dir in data/garage_*; do
    map_name=$(basename $map_dir)
    bag_path="~/bags/${map_name}.bag"
    
    echo "处理: $map_name"
    ./run_vehicle_removal.sh \
      --map "$map_dir" \
      --bag "$bag_path" \
      --cpu  # 批量处理用 CPU，避免 GPU 内存不足
done
```

### 场景 4: 远程服务器处理

```bash
# 在无显示的服务器上运行（不启动 RViz）
./run_vehicle_removal.sh \
  --map data/garage \
  --bag ~/garage.bag \
  --cpu

# 下载结果到本地查看
scp user@server:~/lightning-lm-fresh/data/garage/global_*.pcd ./
```

## 检查结果质量

处理完成后，建议检查以下内容：

### 1. 控制台输出

- 处理了多少视图？（建议 > 100）
- 多少帧有深度图？（`depth X`，X > 0 更好）
- 删除了多少点？（地下车库通常 1-5%）
- 检测到多少车辆簇？

### 2. RViz 可视化

```bash
# 启动可视化
./run_vehicle_removal.sh --map data/my_garage --visualize-only
```

在 RViz 中检查：
- **红色点**（被删除）是否确实是车辆？
- 是否有车辆漏删（假阴性）？
- 是否有墙壁、柱子误删（假阳性）？
- 地面是否完整？

### 3. CloudCompare 对比

```bash
cloudcompare.CloudCompare \
  data/my_garage/global.pcd \
  data/my_garage/global_clean.pcd \
  data/my_garage/global_vehicles.pcd
```

设置不同颜色，切换显示对比。

## 故障排除

### 问题 1: 找不到 GPU

```
检查 GPU... 不可用，将使用 CPU
```

**解决：** 使用 `--cpu` 参数，或检查 CUDA 驱动。

### 问题 2: 缺少 keyframes.txt

```
错误: 找不到 data/my_garage/keyframes.txt
```

**解决：** 确保使用修改后的 SLAM 代码，SaveMap 会自动导出 keyframes.txt。

### 问题 3: bag 话题不匹配

```
RuntimeError: /camera/color/image_raw not in bag
```

**解决：** 
1. 检查 bag 中的实际话题：`ros2 bag info /path/to/bag`
2. 修改配置文件中的 `color_topic`

### 问题 4: 处理速度太慢

CPU 模式每帧 0.3-0.5 秒是正常的。

**加速方案：**
- 使用 GPU（`--device 0`）
- 降低采样密度（配置中 `image_period: 0.5`）
- 使用更快的 bag 存储（SSD 而不是 HDD）

### 问题 5: 内存不足

处理大地图（> 5000 万点）可能内存不足。

**解决：**
- 增加体素大小（配置中 `voxel_size: 0.15`）
- 限制处理范围（配置中 `max_range: 10.0`）
- 关闭其他占用内存的程序

## 与手动流程对比

| 步骤 | 手动操作 | 一键脚本 |
|------|----------|----------|
| 检查依赖 | 手动逐个检查 | ✓ 自动检查 |
| 验证文件 | 手动 ls 查看 | ✓ 自动验证 |
| 构建命令 | 手动输入长命令 | ✓ 自动构建 |
| 启动 RViz | 另开终端手动启动 | ✓ 自动启动 |
| 查看结果 | 手动记忆命令 | ✓ 自动提示 |

**结论：一键脚本节省 80% 的操作时间！**

## 相关文档

- **完整指南**: `doc/VEHICLE_REMOVAL_GUIDE.md`
- **项目概述**: `README_VEHICLE_REMOVAL.md`
- **配置参考**: 
  - `config/garage_clean.yaml` - 地下车库模板
  - `config/m2dgr_clean.yaml` - M2DGR 测试配置

## 技术支持

遇到问题？
1. 查看 `doc/VEHICLE_REMOVAL_GUIDE.md` 的"常见问题"章节
2. 检查配置文件是否填写正确（标定参数）
3. 使用 `--help` 查看所有参数说明
