# 红色 IMU 轨迹大幅折返：时间保护与诊断

截图中红色轨迹对应最新 IMU 预测，绿色对应激光扫描位姿。截图显示红色折返，不能只凭图片认定激光建图发散或断言只是显示问题。

原有实时预测与激光更新后的缓冲重放都没有单步/总预测时长上限；激光未及时提供校正、IMU 时间异常或运行积压时，高频轨迹可能持续外推后再被激光拉回。跳过激光扫描的分支还会把较旧的 LIO 状态直接送到红色轨迹，时间次序可能倒退。本补丁修复这两个代码层面的缺口，同时记录用于验证实际原因的日志。它没有证实当前截图必然由这些缺口导致。

## 行为

- 高速预测的实时处理与缓冲重放共用时间检查。默认单步最多 0.10 s，相对上次成功激光更新最多外推 0.30 s（边界容差 1 微秒）。初始化建图首帧建立初始时间锚点。
- 被拒绝或被主动跳过的激光帧不会刷新“成功校正时间”。超过边界则暂停高频预测及发布，下次激光重置后重新评估；没有假装预测有效，也不对红线做平滑。暂停不会使已有红线历史消失。
- 未初始化时不预测；丢弃非有限 IMU 和非递增的输入时间戳。切换/重放新时间段须重启 SLAM，不能在同一个运行实例中让 bag 时间倒退。
- 跳过的激光扫描仍可显示，但不向红色轨迹写入旧状态。发布高频状态时保证时间戳递增。
- 不改变激光配准 ESKF 的传播、校正、关键帧及地图插入规则。此补丁仅为独立 `kf_imu_` 预测链路增加有效性边界（非法 IMU 输入也不再入主缓冲）。有效范围内的预测若仍抖动，需要继续核对标定、加速度、时间单位和激光校正差。
- `GetIMUState()` 在预测暂停时将 `pose_is_ok_` 标记为 false；使用者需要尊重该有效性标记。

可选配置，不添加也使用上述默认值：

```yaml
fasterlio:
  imu_prediction:
    max_step_s: 0.10
    max_horizon_s: 0.30
    stats_interval: 200
```

实际正常激光校正间隔若因 `skip_lidar_num` 等设置超过 0.3 s，需要根据传感器频率和应用延迟预算调整，而不是看到暂停就任意放大阈值。`max_step_s` 必须大于 0，`max_horizon_s >= max_step_s`。这些是时间边界，不是位置平滑参数。

## 复现与检查

更新 `codex/vision-cleanup-round4` 后低并发编译，重启 SLAM 并从 bag 开头重放。运行旧进程不会应用新代码，旧折线也不会自动清除。

```bash
cd ~/lightning-lm-fresh
git fetch origin
git switch codex/vision-cleanup-round4
git pull --ff-only origin codex/vision-cleanup-round4
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
./bin/run_slam_online --config ./config/m2dgr_slam_detail.yaml \
  2>&1 | tee /tmp/m2dgr_prediction.log
```

另一个加载 ROS 的终端低速回放（已有配置文件保留不动）：

```bash
ros2 bag play \
  "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --rate 0.5 \
  --qos-profile-overrides-path ./config/m2dgr_playback_qos.yaml
```

查看相关日志：

```bash
grep -E '\[imu_prediction\]|registration rejected|Reject ESKF|abnormal dt|断流' \
  /tmp/m2dgr_prediction.log | tail -n 100
```

- `reason=stale_lio`：距最后成功激光更新超过预测预算。
- `reason=step_gap`：IMU 积分单步过大。
- `latest_imu_lead_s`：最新收到 IMU 时间相对已处理激光帧尾的差，不是墙钟处理耗时；可由积压、话题延迟或时间偏差导致，单凭此值不能区分。
- `accepted_lio_age_s`：当前预测/目标相对最后成功校正的时间差。
- `same_time_jump_m`：激光重置前后在相同时间的预测位置差；`-1` 表示无法在相同时间比较，不代表负误差。
- `lio_clock_error_s`：主滤波器时间与已处理激光帧尾的差。
- `offset_m`、`speed_mps`：预测相对 LIO 的位置差和预测速度。

若红线停止更新而绿色继续，检查暂停原因；不能以红线不再长出折返作为地图精度已经提高的证据。若没有暂停且仍有大幅折返，重点观察 `same_time_jump_m` 并提供相同时间段的日志和配置。

## 验证范围

`python3 tests/run_imu_prediction.py --sanitize` 测试时间预算、连续小步累计超过预算、IMU 大间隔、激光重置恢复、重复/乱序时间、拒绝的激光不续期、非有限时间及发布顺序。已在无 ROS/PCL 的开发环境运行通过。完整工程编译、真实数据和真实 ESKF 积分未在开发环境运行，需要在本机验证。

## 主滤波器时间落后与 IMU 接收历史修正

实测日志出现 `accepted=1 suspended=1`，同时 `lio_clock_error_s` 为 -0.12～-0.31 s、`reason=step_gap`。这证实预测被暂停，并暴露出主状态和激光帧尾时间不一致；它不能由“红线被覆盖”解释。原 `UndistortPcl` 的大间隔分支只把状态时间改到 head 样本然后 `break`，最后仍用完整 IMU 组的末尾计算补偿时间，可能漏掉中间传播；后续激光处理却可继续给这一帧打上帧尾时间。修正为：

- 按当前滤波器时间逐段积分到每个绝对 IMU 目标时间，再到激光帧尾。取消取绝对值的尾部时间差；没有用简单 SetTime 替代漏掉的物理积分。
- 最多 1 s 的恢复区间按不超过 0.02 s 的步长传播。输入仍采用相邻观测平均/末值保持假设；细分不会创造丢失的测量。过程噪声按步数补偿 dt² 缩放，并保留每次预测的协方差膨胀。
- 任何大于 0.10 s 的 IMU 观测/传播缺口影响到的激光帧不参与配准、关键帧或地图插入。状态可在短间隔假设下推进，后续覆盖完整的帧恢复；不能把缺口内的运动当作已验证。
- 大于 1 s 的帧间传播跨度、非法时间/样本或非有限传播状态会回滚该次传播并锁定故障，要求重启/降低速率。主滤波器停住时不能靠增大独立预测阈值恢复。
- 每次 Process 先清空输出，防止初始化、无 IMU 或故障时重复使用上一帧点云。进入 LIO 配准前再次检查主状态与激光帧尾时间差，超过 10 微秒则拒绝这一帧。
- IMU 滤波在副本上执行，不再改写与缓冲重放共享的原始测量。
- 单线程在线订阅的 IMU DDS 接收历史由 10 增为 2000 条，LiDAR 历史保持 10 条。200 Hz 下 10 条 IMU 仅覆盖 50 ms，耗时激光回调可能让其溢出。`common.imu_queue_depth` 可设置 10～100000；保留原来的 QoS 可靠性约定。增大队列只缓解短时阻塞，不提高计算吞吐量，仍可能发生积压或丢包。

推荐从 `--rate 0.5` 重新回放；若仍经常缺测/超时，降到 `--rate 0.2`。不要在旧进程中继续尝试，旧状态已经可能发生时钟偏差。日志改为同时关注：

```bash
grep -E '\[imu_clock\]|\[imu_prediction\]|registration rejected' \
  /tmp/m2dgr_prediction.log | tail -n 100
```

新的正常目标是 `lio_clock_error_s` 接近 0。`latest_imu_lead_s > 0.3` 时红线仍可能被有效性保护暂停，必须根据回放吞吐、话题延迟和校正频率继续排查。

新增 std-only 的传播计划边界测试已运行 UBSan 通过；`tests/imu_deskew.cc` 增加真实 ImuProcess 的 0.25 s 缺测物理传播、跳过污染帧、下一帧恢复以及大缺口回滚/故障锁定测试。后者需要 ROS/PCL 和已编译的 lightning.libs，开发环境未执行：

```bash
python3 tests/run_lio_regressions.py
```
