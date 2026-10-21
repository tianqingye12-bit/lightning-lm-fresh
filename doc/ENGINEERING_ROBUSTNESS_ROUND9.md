# 第九轮：保存可靠性与定位健康状态

本轮基于 `codex/map-storage-round8`，保留前几轮地图分辨率、压缩和显示配置。

## 保存地图

`run_slam_online` 的 `/lightning/save_map` 接口不变：`response=0` 表示成功，`response=3` 表示失败，具体原因看日志。

1. 检查 map_id，只允许一个非隐藏目录名，禁止路径穿越和控制字符。
2. 对目标加文件锁，检查至少 64 MiB 可用空间，在同级隐藏目录写入完整新地图。
3. 检查 PCD 返回值、索引/位姿/元数据流的写入与关闭；文件和目录 fsync 成功后才发布。
4. 首次保存使用 Linux `renameat2(RENAME_NOREPLACE)`；覆盖同名地图使用 `RENAME_EXCHANGE`。不支持这些操作的文件系统会拒绝发布，保留原图，不退回先删后写。
5. 保存成功日志打印正式路径与上一版路径。上一版保存在同级 `.地图名.version-*`；失败写入会清理本次临时目录。进程被强杀时可能留下未发布的版本目录，程序不会自动加载或删除它们。不要靠目录名判断其是否可用。

旧地图与新地图必须同时容得下。64 MiB 是启动保存的最低余量，不能保证整幅地图一定写得下。实际磁盘耗尽、PCD 写入失败、fsync 失败会中止发布。成功替换后如果父目录 fsync 失败，会明确打印“已发布但持久性未确认”，不会谎报“旧地图未变化”。文件系统/硬件仍决定断电持久性。

上一版不会自动删除，避免误删唯一可恢复版本。确认新图可读取并另有备份后再人工清理；连续同名保存会增加占用。地图目录切换对单次打开原子，但跨多个文件的已有读者应固定版本目录；不要在正在使用的定位地图上同时保存/修改动态图层。

保存测试：建图运行时连续调用两次同名保存，确认两次 `response=0`，正式目录存在 `save_complete.yaml`，第二次日志有上一版路径。不要为验证磁盘满而填满实际工作盘。

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap \
  "{map_id: street_01_vehicle_test}"
```

## 定位健康状态

这部分接入 **`run_loc_online` 已建图定位节点**。`run_slam_online` 的建图位姿与 IMU 显示预测维持原有行为，已验证的 0.5 秒预测上限不变。

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/lightning/localization_health` | `std_msgs/msg/String` | 每 200 ms 发布 JSON：状态、原因、输入年龄、结果/匹配延迟和恢复计数 |
| `/lightning/localization_valid` | `std_msgs/msg/Bool` | 仅 GOOD 为 true；供上层导航接入 |
| 定位 TF | 原有 `map -> base_link` | 无效时停止发送新 TF；不改变坐标链结构 |

状态为 INITIALIZING、GOOD、DEGRADED、LOST。传感器断流、结果陈旧会进入 LOST；匹配被拒绝、融合结果无效会降级。恢复需连续 3 次新有效地图匹配，不能靠重复的旧匹配或 IMU 外推恢复。

- 断流按单调墙钟计算，处理延迟按消息时间计算，可以回放历史 bag。
- IMU/点云时间倒退后锁定 LOST 并停止接收后续数据，需要重启节点和重新回放，避免把新的时间段接入旧状态。重复时间戳丢弃。
- 定位结果必须有效、GOOD 且有限；失败/重复扫描不会把上次结果重新交给 PGO。
- NDT 必须收敛、变换/分数有限，初始化必须达到已有 `min_init_confidence`。健康检查另外使用 `min_match_score`；NDT 分数不是统计概率，默认 1.0 只是起始配置，需要实际数据验证。
- 即使只在关键帧定位，也每 1 秒消息时间允许一次匹配，避免静止时健康结果永久陈旧。停车复用位姿不刷新绝对匹配健康度，并最多间隔 1 秒尝试重新匹配。
- 限制点云订阅和异步队列，避免积压；IMU 和 LIO 点云处理通过互斥访问同一 ESKF。关停先停止上游工作线程，再退出地图/UI，支持初始化失败退出。
- 删除每次 NDT 匹配无条件写入 `data/tgt.pcd` 的磁盘操作。
- 缺失/空地图块、损坏索引或地图初始化失败会直接拒绝启动，不继续使用单位位姿进入定位。

健康超时配置为可选项，旧配置可继续运行。默认：

```yaml
system:
  localization_health:
    imu_timeout_s: 1.0
    lidar_timeout_s: 2.0
    result_timeout_s: 2.0
    max_result_lag_s: 1.0
    max_match_lag_s: 3.0
    recovery_matches: 3
    match_interval_s: 1.0
    min_match_score: 1.0
```

墙钟阈值需要结合采样频率、回放速度和 CPU 性能调整，低速回放暂停也会触发断流。

停止 TF 不会删除 TF 缓存，也不会自动让底盘停止。上层导航必须订阅有效性、监控健康消息自身是否超时，并执行对应停车/暂停策略；本轮没有接入 Nav2 控制器，也没有实现自动全局重定位。未将旧 `map -> base_link` 改成 `map -> odom -> base_link`，接 Nav2 前还需统一 TF 发布方。

## 本地更新与验证

首次切换本轮分支；如果 Git 提示本地修改冲突，先保留自己的修改，不使用 `reset --hard` 或 `clean`。本轮没有改写本地 M2DGR 配置文件。

```bash
cd ~/lightning-lm-fresh
git fetch origin
git switch --track origin/codex/engineering-robustness-round9
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
```

保存保护编译后直接作用于原来的建图启动命令。定位健康测试另外启动定位程序，先关闭原来的建图程序，避免相同 TF 冲突。使用包含 `index.txt` 与分块 PCD 的地图；只有关键帧的 compact 建图归档不能直接定位。

```bash
# 从本机已验证的配置派生新文件，保留标定与 IMU 预测上限。
# --map 换成你实际保存且包含 index.txt 的地图；示例沿用前面保存的名称。
python3 scripts/configure_localization_health.py \
  --base config/m2dgr_slam_prediction.yaml \
  --map data/street_01_vehicle_test \
  --output config/m2dgr_loc_health.yaml --with-ui

./bin/run_loc_online --config config/m2dgr_loc_health.yaml
```

另一个终端 source ROS 与本项目环境后：

```bash
ros2 bag play \
  "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --rate 0.5 --topics /velodyne_points /handsfree/imu \
  --qos-profile-overrides-path config/m2dgr_playback_qos.yaml
```

观察终端：

```bash
ros2 topic echo /lightning/localization_health
# 或
ros2 topic echo /lightning/localization_valid
```

验证：正常回放与有效匹配后出现 GOOD/true；在播放终端按空格暂停，约 1 秒加 200 ms 定时器周期后应出现 LOST/false；恢复播放，需要连续 3 次新有效匹配后恢复 true。如果一直为 false，先查看 reason/lag/age，不能只把阈值无限增大。入口默认单位初值适用于本次建图的原点附近起播，其他起点仍需正确初值/重定位流程。

独立回归：

```bash
python3 tests/run_engineering_robustness.py --sanitize
python3 -m unittest discover -s tests -p 'test_*.py'
python3 tests/run_navigation_grid.py --sanitize
python3 tests/run_runtime_worker.py --sanitize
```

本轮独立测试覆盖真实 Linux 原子目录交换、流写入失败、模拟 ENOSPC、空间余量不足、并发保存、旧版保留、传感器断流/恢复、时间倒退、陈旧匹配/结果及异步队列关停。测试环境没有 ROS2/PCL，未完成整仓编译和 street_06 回放；上面的本机编译/回放是合并前必要验证。
