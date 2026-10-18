# 第五轮：接收与计算分离、性能统计、显示预算和预测状态

基线为第四轮已验证正常的 `0cbbaea`。本轮分支 `codex/runtime-efficiency-round5`，依赖第四轮，草稿 PR 以第四轮分支为目标。用户本地已验证的 `config/m2dgr_slam_prediction.yaml`（预测上限 0.5 s）继续使用；本轮不改写该文件，也不把全局默认预测上限从 0.3 改为 0.5。

## 线程与输入边界

在线默认启用独立接收/串行计算：

- ROS 传感器回调处于同一互斥回调组，按回调实际提交顺序将 IMU/PointCloud2/Livox 消息交给 FIFO 工作线程。顺序指入队顺序，不代表不同话题经过全局时间排序；原有时间检查与测量同步保留。
- 工作线程串行执行所有实时 LIO 操作、关键帧转交以及保存地图。没有并发调用 ESKF，也没有让接收线程直接改 `kf_`/`kf_imu_`。
- SaveMap 服务在独立回调组中等待串行任务；ROS executor 固定两个线程，服务等待期间另一个线程可继续接收。public SaveMap/StartSLAM 在在线模式也串行调度；离线调用保持同步。
- 默认待处理 IMU 任务上限 4096，LiDAR 上限 2，控制任务上限 8。满时丢弃新到的 LiDAR 并记录计数；IMU 溢出或计算任务异常则停止核心数据接入，标记故障，允许保存已有地图并要求重启。不会静默丢掉 IMU 后继续宣称正常建图。
- LIO 内部缓存也有在线上限：IMU 与外部容量相同，LiDAR 为 `max(4, lidar_capacity + 2)`。避免没有相机/雷达数据时，任务虽然已被消费，内部 IMU 缓冲却无限增长。内部 LiDAR 满时同样报告丢弃；内部 IMU 溢出锁定接入故障。
- 退出时关闭入队并排空已接纳任务，先 join 核心工作线程，再退出回环和 UI。异常任务不会让排队的保存 future 永久等待。

增大队列和分离接收不提高算法本身的吞吐量；当计算长期慢于输入时仍应降低播放速度。丢 LiDAR 会改变参与建图的帧，必须把拒绝计数纳入效果对比。建议暂停 bag 后调用 SaveMap，保存期间仍持续播放可能填满有界队列。

## 性能日志

每处理 20 个 LiDAR 任务打印 `[runtime]`：

- 外部 `queue imu/lidar`、内部 `core queue`、任务拒绝数、内部 LiDAR 丢弃数。
- `preprocess`：点云预处理；`deskew`：IMU 传播与去畸变。
- `downsample`：配准扫描体素过滤；`registration`：整次 ESKF 配准更新。
- `matching`：每次观测迭代的世界点变换、近邻查找和几何/残差筛选合计，不能当作纯近邻查询时间。
- `map_update`：局部地图插入；`ui_prepare`：显示抽样、转换和交接。
- `frame_total`：一个核心 LiDAR 任务的总耗时；`imu_update`：一个 IMU 处理任务。
- `imu_wait/lidar_wait`：任务入队到开始执行的墙钟等待时间。
- `save_map`：保存地图耗时；`render_frame`：渲染一帧的墙钟时间，可能包含图形驱动等待，不是 CPU 使用率。

各项为累计平均、最近 256 个样本的 p95、累计最大和样本数，固定内存。配准包含 matching、帧总耗时包含多个子阶段，不应把这些相互重叠的数字相加；不同指标的样本单位也不同。所有新统计加锁；旧 Timer 的共享记录也增加同步，避免后台模块并发记录时访问 map 发生竞争。

## 显示与预测状态

默认每秒最多渲染 20 帧、交接 5 份扫描。每份扫描最多 20000 点，当前加历史扫描最多 200000 点、最多 20 份，前端/激光显示轨迹各最多 50000 点。原有回环地图显示点数预算仍由 `loop_display` 控制。

显示交接先限频，再只复制抽样后的点，并在抽样副本上应用 LiDAR→IMU 外参；原扫描不修改，前端配准、保留关键帧、导出 PCD 的分辨率不变。渲染线程只接收独立副本。定位模块的外部全局/动态图层不在本轮的扫描预算范围内。

界面显示：

- `IMU prediction: VALID/PAUSED`、暂停原因、预测相对最后成功激光更新的时间差。
- 排队数量和拒绝数量。
- 暂停时坐标轴灰色保留在上次显示位置，表示历史值，不代表最新有效定位。
- 暂停/恢复事件在两次渲染之间发生也会锁存“断线”，轨迹恢复时开启新段，不跨越无效区间连线。
- 超过 1 秒墙钟未收到状态交接也显示 `no_recent_imu`；手动暂停 bag 时这可能是预期现象。该显示超时与消息时间上的预测上限是两个不同概念。

本轮没有在显示层平滑或修改估计位置。缩小坐标轴的停顿问题仍需依赖预测有效性和计算跟得上输入。

## 配置

旧配置无需增加字段即可运行。若已有 `system`，将字段并入它，不要新增重复根键：

```yaml
system:
  runtime:
    async: true
    imu_capacity: 4096
    lidar_capacity: 2
    stats_interval: 20
  runtime_display:
    render_hz: 20
    scan_hz: 5
    scan_max_points: 20000
    history_max_points: 200000
    history_scans: 20
    trajectory_max_points: 50000
    stale_wall_s: 1.0
```

`async: false` 可切回同步输入计算作对照，保存服务也恢复单线程 executor；在线内部缓冲保护仍保留。显示预算可独立调整。各容量必须为正且在合理范围内；`scan_hz <= render_hz`、`history_max_points >= scan_max_points`。

## 本地更新与验证

先检查本地修改，保留 M2DGR 标定、话题和已验证的 0.5 秒预测配置。切换冲突时先备份/暂存，不要 reset --hard。

```bash
cd ~/lightning-lm-fresh
git status --short
git fetch origin
git switch --track origin/codex/runtime-efficiency-round5
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
./bin/run_slam_online --config ./config/m2dgr_slam_prediction.yaml \
  2>&1 | tee /tmp/m2dgr_round5.log
```

另一终端 source ROS 后，从头以 0.5 倍速回放：

```bash
ros2 bag play \
  "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --rate 0.5 \
  --qos-profile-overrides-path ./config/m2dgr_playback_qos.yaml
```

比对相同包、相同配置、相同播放速度的第四轮结果：队列是否持续增长、拒绝是否为 0、配准和帧耗时、暂停次数、地图轮廓与轨迹是否一致。内存和 CPU 需用系统监视器或外部工具测量；新日志并未测量真实峰值 RSS 或 CPU 百分比。暂停/结束播放后保存为新名称，例如 `street_06_round5`，不要覆盖基线地图。

```bash
grep -E '\[runtime\]|\[imu_prediction\]|\[imu_clock\]' /tmp/m2dgr_round5.log | tail -n 100
python3 tests/run_runtime_worker.py --sanitize
python3 tests/run_imu_prediction.py --sanitize
python3 tests/run_map_detail.py --sanitize
```

开发环境已运行 std-only 队列/并发/性能统计/轨迹分段以及前轮策略测试，UBSan 通过。队列测试覆盖有界准入、跨传感器 FIFO、串行执行、保存屏障、异常返回、重入调用及退出排空。真实 ROS executor、DDS、PCL、Pangolin/OpenGL 的工程编译与集成回放未在开发环境执行；不能据此宣称实际处理速度或地图精度提高。草稿 PR 等待本机编译和回放结果。
