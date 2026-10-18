# 第四轮：坐标一致性、视觉选帧、分割缓存与去车诊断

本轮以 `c09b691`（前三轮合并后的 master）为基线。不修改 ESKF 估计或用平滑掩盖轨迹波动。

## 坐标约定及旧地图

- 去畸变扫描、配准输入仍为 LiDAR 坐标；配准继续使用 `T_world_imu * T_imu_lidar`。
- 关键帧保留点云在独立副本上转换到 IMU 坐标；图节点仍为 IMU 位姿。回环源/目标、全局地图和关键帧显示由此使用同一约定，不能再次乘外参。G2P5 的射线原点和地面高度配置仍以雷达为基准，因此该模块通过关键帧的只读外参恢复雷达点云，并配合 `T_world_imu * T_imu_lidar` 使用，保留原来的高度参数语义。
- 当前扫描显示也转换到 IMU 坐标，配 IMU 位姿；`GetProjCloud()` 将历史 IMU 点转换回当前 LiDAR 坐标，且不再原地修改去畸变扫描。
- SaveMap 同时导出 `map_frames.yaml`，版本为 2，包含 `T_imu_lidar`、`keyframe_pose_frame: imu` 和 `keyframe_cloud_frame: imu`。`keyframes.txt` 保持九列及同一次优化位姿快照。
- 去车先插值 IMU 位姿，再计算 `T_world_lidar = T_world_imu * T_imu_lidar`，相机投影为 `T_cam_lidar * inverse(T_world_lidar)`。不能先插值雷达位姿，否则旋转时会把外参平移产生的圆弧误插成直线。
- 缺少元数据的旧地图明确提示 `legacy`，沿用旧投影约定，不擅自补外参。旧地图生成时遗漏的非单位外参无法通过给成品 PCD 增加一个固定变换完整修复；需要重新建图。不要给旧地图手工补版本 2 元数据。
- 本轮没有重新标定 M2DGR。现有配置中的相机内外参仍需用叠加图验证；这次修正也不代表已有定位模块与旧地图的兼容性完成实测。

## 选帧与缓存

`config/m2dgr_clean.yaml` 增加：

```yaml
view_selection:
  translation_m: 0.10
  rotation_deg: 2.0
  max_period_s: 1.0
cache:
  enabled: true
  directory: ''
rviz_max_points: 300000
diagnostics:
  every_views: 20
  max_overlays: 20
```

先满足原有 `image_period: 0.2`，然后满足位移、旋转、最长间隔中的任意一个条件。选帧在图像解码和推理前完成。最长间隔允许静止时继续积累视角；不能据此认为静止图像是独立观测。设置两个运动阈值为 0 恢复仅按时间选帧；没有新字段的配置保持旧选帧行为。

缓存默认在 `<map>/.vehicle_mask_cache`，只存未膨胀分割掩膜。键包含图像内容、尺寸/类型、模型权重 SHA256、Ultralytics/Torch 版本、设备、置信度和类别。模型/图像/推理设置改变即失效；地图、外参、投票或扩张阈值改变会重新投影投票，但可复用分割结果。文件原子替换，损坏文件重新推理。`--no-cache` 完全禁用读写；第二次执行仍会加载模型并扫描 bag，缓存不是完整断点续跑。

每个地图独立缓存；希望跨地图复用时设置明确的 `cache.directory`。缓存会随处理图像数量增长，可在停止去车程序后删除该缓存目录。相同 map_id 的 SaveMap 原有逻辑会覆盖整个目录，因此请用新地图名做对比。

## 去车与资源使用

- 空视野/零投影跳过推理，避免旧版零视角统计除零。
- 邻域搜索以相机位置为中心，球半径覆盖归一化视锥，避免外参平移或相机旋转时漏选投影点。
- 保留遮挡深度检查、投票和地面保护；簇扩张不得删除已有足够观测且未达到车辆比例的背景体素。该规则偏保守，可能保留部分漏检车身，需要对比漏删与误删。
- RViz 每个点云有硬点数上限，按索引均匀抽样，只影响显示，不影响结果分辨率。图像覆盖层和落盘抽样图使用同一渲染函数。
- PCD 结果按块筛选写出，避免同时建立整份筛选点云及大字节缓冲。体素化、空间索引和连通分量仍在内存中，尚非全流程流式建图/清理。

每次运行创建独立 `<map>/clean_diagnostics/run_*`，输出：

- `views.csv`：时间、是否投票、目标数、投影/可见/遮挡/车辆票点数、是否有深度、单视角耗时。
- `overlay_*.jpg`：蓝色掩膜、灰色遮挡点、绿色可见背景、红色车辆票，默认最多 20 张。
- `summary.json`：坐标版本、选帧/跳过/缓存统计、删除比例、观测覆盖、总耗时及实际配置。仅成功结束后生成。

控制台显示轨迹时间覆盖进度（并非剩余耗时预测），选帧和无投影原因。最终 `global_clean.pcd` 和 `global_vehicles.pcd` 仍会在重跑时覆盖；`global.pcd` 不改。

## 用户机器更新与运行

先检查本地修改；如切换被 Git 阻止，备份/暂存相关修改，不要使用 `reset --hard`。

```bash
cd ~/lightning-lm-fresh
git status --short
git fetch origin
git switch --track origin/codex/vision-cleanup-round4
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
```

已有同名本地分支则用 `git switch codex/vision-cleanup-round4` 后 `git pull --ff-only`。

若要验证坐标修复，从头回放 `street_06_ros2` 重新建图并保存为新名称，例如 `street_06_round4`，避免覆盖旧结果。建图用原来的 detail 配置与低速回放；保存后确认新目录有 `map_frames.yaml`：

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: street_06_round4}"
cat data/street_06_round4/map_frames.yaml
```

停止建图和回放后执行：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_06_round4 \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean.yaml --device cpu --rviz
```

另一终端加载 ROS 后运行 `rviz2 -d config/clean_map.rviz`。去车脚本直接读取 bag，不需要 ros2 bag play。

现有 `data/street_01_vehicle_test` 可直接测试选帧、缓存、诊断，但会提示 legacy，不能用于验证新的坐标修复。

## 验证范围

自动化测试覆盖非单位旋转/平移、相机偏移、IMU 插值后的外参组合、选帧门限与静止兜底、缓存命中/失效/损坏、缓存前后投票相同、遮挡、空投影、PCD 分块与字段保留、RViz 点数/强度对应，以及原有去车回归。

```bash
python3 -m unittest discover -s tests -p 'test_clean*.py'
python3 -m unittest discover -s tests -p 'test_vehicle_removal.py'
python3 tests/run_map_detail.py --sanitize
# ROS/PCL 开发环境：实际点云变换和保留测试
python3 tests/run_map_detail.py --sanitize --with-pcl
# 完成 colcon 编译后：关键帧外参往返与图优化集成测试
python3 tests/run_loop_quality.py --sanitize --backend
```

开发环境已运行 Python 回归和不依赖 PCL 的 C++ 策略测试。开发环境无 ROS/PCL，未执行完整 colcon 编译、PCL 变换测试、真实 YOLO 或 M2DGR 回放。不能据合成测试宣称实测精度/耗时提高。

实测应在相同 bag、相同电脑上保留两份输出：先核对掩膜与点云轮廓，再比较车辆漏删、背景误删；相同配置第二次执行检查 cache_hits；比较每次 diagnostics 的耗时与覆盖率。运动选帧会改变票数，必要时恢复时间选帧作对照，不宜只比较删除点数。
