# 第二轮：地图细节保留

本轮基于 `codex/frontend-feature-quality-round1`，保留 PCA、Huber 和配准失败不插图的保护。
目标是分别控制配准计算量、关键帧保留量和地图导出采样；不声称仅靠更密的点云提高定位精度。

## 数据流与兼容性

输入点云 → ROI/盲区过滤与输入抽样 → IMU 去畸变 → 两条路径：

- 配准：`fasterlio.filter_size_scan` 体素 → ESKF → 配准用局部 iVox 地图。
- 保存：去畸变点云 → `map_detail.keyframe_resolution` → 可选点数上限 → 关键帧 → 全局地图导出。

`filter_size_map` 仍控制局部插点筛选，`ivox_grid_resolution` 仍控制搜索网格；不把它们等同于导出体素。
关键帧不从配准降采样点云生成。`export_resolution` 同时作用于系统保存服务和前端 `SaveMap()` 的默认导出路径；
代码显式传入正数分辨率时优先使用该数值，`use_voxel=false` 保持不过滤的语义。
导出保留原有的关键帧体素过滤与最终全局体素过滤两步，未引入额外的高密度全扫描缓存。

旧数据集 YAML 不添加 `map_detail` 仍沿用旧行为：输入抽样不变、关键帧不额外过滤、不补点、导出 0.1m。
`config/default.yaml` 显式列出同样的兼容默认值。

## 生成 M2DGR 测试配置

不要用其他雷达的默认配置替换已验证的 M2DGR 内外参。由现有配置生成独立文件：

```bash
cd ~/lightning-lm-fresh
python3 scripts/configure_map_detail.py \
  --input config/m2dgr_slam.yaml \
  --output config/m2dgr_slam_detail.yaml
```

生成器保留话题、标定、IMU、关键帧门槛、iVox 参数、第一轮质量参数等设置，仅设置配准体素为 0.3m 和本轮参数：

```yaml
fasterlio:
  filter_size_scan: 0.3
  map_detail:
    input_point_filter_num: 1
    registration_min_ratio: 0.0
    keyframe_resolution: 0.1
    keyframe_max_points: 60000
    export_resolution: 0.05
    local_fill:
      enabled: true
      min_interval: 0.3
      translation: 0.25
      rotation_deg: 3.0
      min_effective: 50
```

输入抽样 1 表示保留所有通过现有几何过滤的点；增加去畸变和临时点云开销，不表示存储每一帧。
`registration_min_ratio: 0` 关闭“配准点数不足原输入 10% 时自动改用 0.1m”的比例规则，
避免增密输入间接导致配准计算暴涨；少于 `fasterlio.min_pts` 时仍保留原有 0.1m 回退。
启用独立输入抽样时，初始化地图也先按配准体素过滤，避免第一帧插入全部高密度点。

每个关键帧最多 60000 点；超限后在全点序均匀取样，不只保留扫描前半段。此上限有细节损失，
如经常触发可按内存情况提高或设 0。累计关键帧、导出拼图及回环仍消耗内存，上限不等于总内存限制。
0.05m 导出不能补回输入抽样、关键帧过滤或点数预算已丢失的信息，也不代表达到 5cm 精度。

## 局部补点条件

配准成功、没有生成关键帧，并且距上一次地图更新 ≥0.3s；同时满足位移 ≥0.25m 或转角 ≥3°，
且点面或点点有效约束至少有一类达到 50。门槛相对于最近一次实际调用地图更新的位姿和时间。
关键帧插图也会重置门槛。失败扫描、时间倒退、原地不动均不能触发补点。
补点只复用配准降采样点和既有插点筛选，不增加后端关键帧、回环节点或全局保存扫描。
因此它用于补充配准局部地图；不能替代更密的全局地图关键帧覆盖。

## 编译和验证

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
python3 tests/run_map_detail.py --sanitize --with-pcl
python3 -m unittest discover -s tests -p 'test_map_detail_config.py'
```

无 ROS/PCL 的环境可以运行 `python3 tests/run_map_detail.py --sanitize`，仅验证门槛和点数预算策略；
`--with-pcl` 额外编译运行生产点云保留函数的体素、点数预算、输入不被修改和空云测试。
完整 LIO 回归仍使用 `python3 tests/run_lio_regressions.py`（需要完整构建环境）。

在两个终端中运行（先启动算法再回放）：

```bash
# 终端一
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash
./bin/run_slam_online --config ./config/m2dgr_slam_detail.yaml
```

```bash
# 终端二
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
ros2 bag play \
  "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/walk_01_ros2" \
  --rate 0.5 \
  --qos-profile-overrides-path ./config/m2dgr_playback_qos.yaml
```

启动应看到 `[map_detail] input_stride=1 registration_m=0.3 ... export_m=0.05 local_fill=1`。
生成关键帧时打印保留点数；补点时打印 `[map_detail] local_fill ...`；保存时打印导出体素。
若没有补点日志，应先查看是否一直生成关键帧、运动/时间门槛未达到或配准被拒绝。

与第一轮配置分别运行相同 bag、回放速度和时段，使用不同保存目录。比较：

1. 原始/保留关键帧点数、补点频率与内存峰值、配准耗时和是否持续积压。
2. 用相同点大小显示导出 PCD，检查路沿、细杆、墙面厚度及重影。
3. 精度比较另做相同最终体素尺寸的导出；有真值时比较 ATE/RPE，避免把密度当精度。

电脑吃紧时先将 `input_point_filter_num` 改为 2，再将 `keyframe_resolution` 改为 0.15，
必要时关闭 `local_fill.enabled`。复现第一轮直接使用原 `m2dgr_slam.yaml`。
当前提交的自动验证不能替代你的 ROS2/PCL 完整编译和真实数据集效果验证。
