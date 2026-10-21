# 第十四轮：跨关键帧统一语义证据

基于 `codex/trajectory-alignment-round13` 的最新目录、紧凑导出及诊断修改。本轮不改变 YOLO 模型、标定、图像时间偏移、LIO、置信度、地面保护或原来的多视角门限。默认关闭，在独立配置中启用：

```yaml
cross_keyframe_semantics:
  enabled: true
  voxel_size: 0.10
```

## 为什么需要两遍处理

原流程各关键帧独立累积投票、独立删除，再拼接。一个世界位置有多份重叠观测，每份可能只有一两张匹配图像，达不到最少观察数。某帧没有相机覆盖时，即使相同位置在另一帧已得到可靠车辆证据，该份副本也会保留。

新流程第一遍仍按原关键帧时间窗口、可见性、实例深度关联和分割核心提取证据，但将实际可见点映射到统一的世界坐标体素。所有激活关键帧处理完同一张图像后统一合并：

- 同一图像、同一体素最多一条观察。重复点、重叠关键帧不能增加票数。
- 只有车辆证据时计一个正票，置信度取该图像该体素正证据的最大值，不累加点数量。
- 只有背景证据时计一个背景票。未知点不产生背景票。
- 同图同体素出现车辆和背景冲突时，该次观察记为未知，正票/背景票/置信度均不增加。它仍增加观察数，因此影响已分类覆盖率。
- 重复图像时间戳不再次计票，时间倒退或非有限时刻报错。位姿缺口仍由原轨迹查询规则跳过。
- 正票基线记录第一次正票相机位置到后续正票相机位置的最大距离，定义与原流程一致。

第一遍结束后，第二遍逐帧读取**原始保存的关键帧**，查询对应体素的统一票数，使用原 `decide` 的最少观察数、最少正票、比例、背景否决、覆盖率、基线及置信度门限判定，并按每帧原地面估计再执行地面保护。最终关键帧和 global_clean.pcd 使用同一删除结果。原始地图及关键帧文件不变。

未曾匹配图像的关键帧副本可以使用同体素的已有可靠证据；完全没有证据的体素、相邻体素仍保留。不进行近邻扩张、包围盒整簇删除或未知点批量删除。地面估计不足时继续保留候选点并计数。

## 内存、耗时和显示

SQLite 临时数据库只保存体素累计统计，不保存全图点云或全部图像掩码。页缓存设置为 16 MiB；每张图像的待合并数组受已有 `max_active_frames` 和关键帧点数限制，第二遍每次查询至多 10000 个不同体素。Python、NumPy 和操作系统缓存仍有额外内存，不能视为总内存硬上限。数据库会使用临时磁盘，处理结束或失败关闭时移除。输出目录的 INCOMPLETE 标记仍用于区分未完成结果。

第二遍增加一次关键帧读取/解压；不会第二次运行 YOLO。第一遍继续显示 raw、轨迹、当前相机、投票和分割叠加图；第一遍的 votes 是当前帧局部投票预览。因为未来图像的证据尚未收集完，clean/vehicles 在第二遍统一导出时逐步更新。终端明确打印 `phase 1/2`、`phase 2/2` 和导出进度，避免将第一阶段 removed=0 当作处理失败。

`summary.json` 新增 cross_keyframe_semantics，报告独立时间戳图像数、重复图像数、体素-图像对、同图冲突数量、存储体素数和有正/背景证据的体素数；这些不等于识别精度。frames.csv 的 views 仍表示该帧局部投影成功的图像数，其他票数及删除门限统计来自共享证据。`removed_points_without_local_views` 单独统计没有局部成功视图的帧中依靠共享证据删除的点；不等于所有跨帧转移点数。

## 本机运行

```bash
cd ~/lightning-lm-fresh
git fetch origin
git switch --track origin/codex/cross-keyframe-semantics-round14
# 若本地已存在该分支，使用 git switch codex/cross-keyframe-semantics-round14
# 然后 git pull --ff-only origin codex/cross-keyframe-semantics-round14

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/configure_cross_keyframe_round14.py \
  --input config/m2dgr_clean_alignment13_keyframes.yaml \
  --output config/m2dgr_clean_consistency14.yaml --voxel-size 0.10

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_vehicle_test4 \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_consistency14.yaml \
  --device cpu --rviz --compact-result --global-voxel-size 0.10
```

配置生成器独占创建输出文件，已存在时拒绝覆盖。输入使用本机已经跑通的标定及 keyframes 备用轨迹配置，也支持有效的 dense 配置，生成器不会偷偷切换来源。只需 Python 更新，无需 colcon 编译。需重新执行语义处理；已经丢弃的每图证据不能从旧 global_clean.pcd 中恢复。已有原始分割缓存身份未变时可复用，不承诺所有历史缓存都能命中。

完整轻量结果仍保存为 `data/street_vehicle_test4_clean[_编号]/global_clean.pcd`。需要后续现有逐关键帧导航构建时，不使用 --compact-result，改用 --export-global --global-voxel-size 0.10，保留清理关键帧归档。语义体素大小和导出体素大小是两个独立参数，均为 0.10 只是本示例设置。

```bash
python3 scripts/audit_clean_map.py --result data/实际结果目录
```

## 验证及限制

本环境完整 Python 回归共 102 项，101 项通过、1 项因缺少 PCL 工具跳过；新增 12 项测试。Python 语法检查通过。未运行完整 ROS2 工程构建。

回归包括重复点/多关键帧同图去重、最大置信度、同图正背景冲突、重复时间戳、负坐标与邻格隔离、基线/置信度/背景否决/地面保护。实际投影、关联、投票和 PCD 读写的合成流程中：四份相同世界位置的车辆副本各自票数不足，旧流程保留；新流程汇总三张独立图像后删除四份，包括一份没有匹配图像的关键帧副本。静态背景和无证据点保留，字段和原始文件保持不变。另检验同一张图像关联四个重叠帧时只执行一次模型推理且不能虚增到四票。精简结果使用相同语义判定。

没有真实 bag、模型及 ROS2 环境，尚不能据合成测试宣称真实车辆去除率、误删率、运行速度或峰值内存改善。传感器时间参考、外参、位姿质量或掩码本身错误时，共享证据也会传播错误；小体素不能解决这些问题，大体素还可能混合相邻表面。位于不同体素的偏移副本、相机未覆盖的车辆背面不会自动删除。

此前日志中的负逐点时间及去畸变问题未在本轮修复；keyframes 备用轨迹也不能证明投影准确。需要结合真实车辆 overlay、静态投影误差和逐点时间约定继续核查。建议先同图对照配置开启/关闭，仅在独立路段检查效果后再合并主分支。
