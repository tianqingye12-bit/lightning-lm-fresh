# 第一轮前端特征质量优化

本改动面向激光惯性前端的点面约束，不增加视觉里程计或在线车辆剔除。

## 行为

- 用中心化 PCA 拟合局部平面，支持过原点的平面；拒绝非有限点、重合点、近似共线点、
  空间跨度过小和非平面邻域。继续使用 `esti_plane_threshold` 检查邻点到平面的距离，
  并保留原先的当前扫描点残差门限及至少 20 个有效平面约束的要求。
- 对通过筛选的点面残差使用 Huber IRLS 权重。小残差权重为 1，大残差权重为
  `huber_delta / abs(residual)`，同一权重同时作用于 HTH 和 HTr。
  ESKF 原有的残差诊断仍使用未加权残差；可选点到点 ICP 分支保持原有行为。
- 每隔若干个已处理配准扫描输出特征统计。计数来自该扫描最后一次执行的观测模型，
  不是多次 ESKF 迭代累计值，也不表示该次位姿更新一定被接受。
  工作线程仅写各自的状态，归并和日志在并行循环结束后执行。

这会改变旧 YAML 的默认前端行为，但所有新字段都是可选的，不需要修改旧配置结构。
未对所有地图定位模块的通用 `math::esti_plane` 做替换，影响范围限于前端点面匹配。

## 配置与对照

在实际使用的 SLAM YAML 的 `fasterlio` 内添加：

```yaml
feature_quality:
  enabled: true
  min_spread: 0.03
  min_tangent_ratio: 0.01
  max_normal_ratio: 0.10
  robust_weight: true
  huber_delta: 0.10
  stats_interval: 20
```

- `min_spread`：邻域次大协方差特征值的平方根下限，单位 m。
- `min_tangent_ratio`：次大/最大特征值下限，防止细长、近似共线邻域被认为可靠平面。
- `max_normal_ratio`：最小/次大特征值上限；越小，平面性要求越严格。
- `huber_delta`：点面残差的 Huber 拐点，单位 m。
- `stats_interval`：每多少个完成配准尝试的扫描输出一次；0 关闭日志输出。

建议使用同一输入与其他参数做三组对照：

| 组别 | enabled | robust_weight |
|---|---|---|
| 原拟合/等权重对照 | false | false |
| 仅平面质量检查 | true | false |
| 平面质量与鲁棒权重 | true | true |

对照关闭后仍保留非有限点和法向量有效性检查，因此是带安全检查的原拟合对照。
不要用 `m2dgr_clean.yaml` 作为 SLAM 配置；它属于建图后的视觉去车。

## 编译与测试

本次修改包含 C++，拉取分支后必须重新编译。按低负载方式在项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --base-paths "$PWD" \
  --packages-select lightning --parallel-workers 1 --symlink-install
source install/setup.bash
```

只编译运行新增的几何回归测试，不需要 ROS/PCL 或完整项目编译：

```bash
python3 tests/run_feature_quality.py
# 可选：启用未定义行为检查
python3 tests/run_feature_quality.py --sanitize
```

需要 C++17 编译器和 Eigen3；自定义安装路径可使用 `--eigen-include /path/to/eigen3`。
完整项目构建完成后，原来的回归入口也包含了新增测试：

```bash
python3 tests/run_lio_regressions.py
```

新增测试覆盖过原点/旋转平移后的平面、共线与重合点、细长与过小邻域、非平面点集、
距离门限、NaN/Inf、无效参数、Huber 拐点和对称性、异常匹配影响、统计分箱及空统计。

## 回放验证

每组使用同一 bag、同一播放速率（建议先 0.5 倍）和同一初始条件，分别保存日志和地图。
每次完整重启建图程序后从 bag 开头播放，不在同一个状态估计器里循环播放。

```bash
./bin/run_slam_online --config ./config/m2dgr_slam.yaml 2>&1 | tee /tmp/m2dgr_features.log
# 另一终端播放现有 ROS2 bag，保留已验证的 topic/QoS 配置。
```

查看日志中的 `[features]`：

- `candidates/accepted`：待匹配点数和有效点面匹配数。
- `no_neighbors/invalid/degenerate/nonplanar/residual_rejected`：拒绝原因。
- `huber_downweighted/mean_weight`：降权数量和接受点的平均权重。
- `azimuth`：雷达坐标系方位角，按 [-pi, pi) 分为 8 个 45 度区间；+pi 归入最后一格。
- `normal_xyz`：世界坐标系法向绝对值的主方向，忽略正负号。
- `range`：雷达坐标系测距 [0,5)、[5,15)、[15,+inf) m。
- `update_accepted`：ESKF 本帧是否接受更新。

这些直方图只观察覆盖情况，不是完整的可观测性判据，不会自动删除特征。
如果 `accepted` 经常不足 20，应检查标定与时间戳，再在对照试验中调整几何阈值。

## 验证边界

开发环境验证了几何/权重/统计辅助代码的 C++ 编译与 UBSan 回归。
开发环境没有 ROS2/PCL，尚未完成整包编译、真实 bag 回放或定位误差评估。
PCA 每次近邻拟合增加一次 3x3 特征分解，不承诺提速；应记录匹配耗时、有效特征数、
更新拒绝次数、轨迹误差和地图重影，确认结果后再合并或部署。
