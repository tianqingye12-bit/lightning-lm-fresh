# 第十轮：建图轮廓与真实观测点保留

本轮基于 `codex/engineering-robustness-round9`。改进雷达几何匹配和地图采样；现有标定、去畸变、回环优化、坐标约定与 IMU 预测上限保持不变。新策略默认关闭，通过独立配置启用，便于同一数据包对比。

## 修改原因与实现

1. **体素内选择真实点，保留原始字段。** 原来的 PCL VoxelGrid 求平均，在一个体素同时包含墙角两面、前后表面时会生成不在任一真实表面上的点。新策略选距离体素中心最近的实际输入点，完整复制强度、时间等字段；配准、关键帧保留、全局导出和定位分块使用同一策略。平局按 XYZ 排序保证几何选择稳定，不修改输入点云。每体素仍最多输出一个点，不能恢复低于体素尺度的结构。
2. **全局导出在世界坐标中一次聚合。** 关键帧按优化后 IMU 位姿变换到世界坐标，逐帧写入体素选择器；跳过原来的“每帧再平均一次、全部拼接、再次平均”过程。内存随最终占用体素数增长，不再另存所有变换点的完整拼接副本。5 cm 大地图仍可能很大。
3. **局部地图补点只与同一体素内的点比较。** 原逻辑可能拿相邻体素中更靠近当前体素中心的点压掉新点，造成轮廓缺点。新策略过滤不同体素的邻居，对负坐标和边界使用 floor，不因相邻体素已有点而忽略空体素。
4. **限制平面匹配的局部支撑。** PCA 平面通过后，再检查最近邻距离、点面距离及面内范围。防止边缘附近的点匹配到缺少邻居支撑的平面延长部分。面内判断是邻域半径上界，不是精确凸包、遮挡或动态目标判定。被拒约束不进入点面/对应点 ICP，日志新增 `support_rejected`。仍由原有约束数检查决定是否接受整帧。
5. **细采样回退不变粗。** 原先少点回退固定 0.1 m；现在取原配准分辨率与 0.1 m 中较小者，避免对更细配置反而降精度。

真实点选择会保留部分原始测距噪声；平滑的单一墙面上，平均点可能更平滑。局部支撑门槛太紧会减少有效约束，远距离稀疏点尤其明显。不能据合成测试承诺真实场景定位精度提升，需要按下面步骤回放同一 bag 评估。时间同步、雷达点内时间单位、外参或轨迹偏差引起的重影，仍需从对应源头修复。

## 配置

新增选项，旧配置不启用时沿用原采样/匹配路径：

```yaml
fasterlio:
  map_detail:
    preserve_observed_points: true
  feature_quality:
    surface_support:
      enabled: true
      max_neighbor_distance: 0.6
      max_plane_residual: 0.15
      max_tangent_gap: 0.10
```

三个距离门槛单位均为米，属于待数据验证的初始值。独立配置脚本还设置：配准不粗于 0.25 m、输入步长 1、关键帧和导出 0.05 m、关键帧最多 60000 点，关闭按保留比例强制回退到 0.1 m。已有更细配准配置不会被改粗。局部地图滤波、IVox 搜索索引、关键帧位移/转角阈值与 local_fill 配置保持原值，不通过大量增加关键帧来制造“更密”的效果。

默认紧凑保存：压缩关键帧，不重复保存 global.pcd/定位分块。需要传统 global.pcd 和定位分块时，生成配置加 `--storage-profile full`。两种方式的关键帧采样一致；5 cm 导出无法恢复关键帧预算或采样已丢掉的信息。

## 本机更新与运行

```bash
cd ~/lightning-lm-fresh
git fetch origin
git switch --track origin/codex/contour-preservation-round10

source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash

python3 scripts/configure_contour_detail.py \
  --input config/m2dgr_slam_prediction.yaml \
  --output config/m2dgr_slam_contour.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 ./bin/run_slam_online --config config/m2dgr_slam_contour.yaml
```

若基准配置文件名不同，使用本机已经跑通的配置作 `--input`。输出文件独占创建，已存在则拒绝覆盖；不要修改原标定或将已验证的 `max_horizon_s: 0.5` 改成其他数值。首次分支切换如 Git 提示本地修改冲突，先保留修改，不使用强制重置。原来的基准启动命令仍可用于 A/B 对比。

另一个终端加载 ROS2 环境，从数据包开头运行：

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
ros2 bag play \
  "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --rate 0.5 --topics /velodyne_points /handsfree/imu \
  --qos-profile-overrides-path config/m2dgr_playback_qos.yaml
```

启动日志应有 `observed_points=1` 和 `[surface_support] enabled=1`。先比较同样路段、相同回放速度、相同视角和点大小下的墙角、杆体、路沿、相邻平面交界，同时检查是否出现双层墙、局部缺点、有效约束数降低或处理排队。两轮结果保存为不同 map_id，确保比较的是新建的地图；不能只重新导出旧地图就评估这一轮前端效果。

如果配准耗时/排队明显增加，先另生成一份 `--registration-resolution 0.3` 的配置，保持轮廓采样和支撑检查但沿用较粗配准；不要通过高速回放做质量验收。如果 `support_rejected` 很多且有效约束不足，可在单独副本中关闭 `surface_support.enabled` 分离判断原因，不能把所有阈值盲目放大。若出现重影，记录发生路段、配准有效点数和位姿跳变，继续查同步/轨迹，而非仅把显示点调小。

紧凑模式保存后的去车继续使用 `--mode keyframes --config config/m2dgr_clean_compact.yaml`；原来的全局地图模式需要 global.pcd。Python 离线静态导航聚合器的平均策略不在本轮修改范围内。

## 验证

```bash
python3 tests/run_contour_geometry.py --sanitize
# 本机安装了 PCL 后，再检查实际点云适配器：
python3 tests/run_contour_geometry.py --sanitize --with-pcl
python3 tests/run_map_detail.py --sanitize --with-pcl
python3 tests/run_feature_quality.py --sanitize
python3 -m unittest discover -s tests -p 'test_*.py'
```

新增独立测试覆盖两面墙同体素的平均偏差、真实点/元数据保留、负坐标及体素边界、非法输入、流式分块与输入乱序的一致几何结果、相邻体素不误抑制补点、局部平面外延拒绝以及旋转平移不变性。配置测试验证源文件不被覆盖、标定/0.5 秒预测不变和紧凑/完整保存选择。

Python 回归共 47 项，46 通过、1 项因缺少真实 PCL 压缩工具跳过。使用运行环境内 Eigen 头文件的 PCA 特征质量回归也已通过。当前环境缺少 ROS2/PCL，完成了独立几何、地图策略、保存健康保护、导航导出与队列测试；完整项目编译、PCL 适配器回归和 street_06 实际效果需本机验证。本轮以独立草稿 PR 发布，尚未并入 master。
