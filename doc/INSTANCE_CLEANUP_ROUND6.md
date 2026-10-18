# 第六轮：实例级深度保护与关键帧静态地图重建

基于第五轮 `bf75f10`，保留前三轮地图细节、第四轮坐标约定和第五轮线程/显示策略。本轮不修改位姿估计、预测时限或配准点云；不是在线视觉紧耦合 SLAM。

## 修正点

已有代码实际使用 `result.masks.data`，并非仅使用检测框；问题在于把所有实例合并成一个二值图、依赖累计地图的深度缓冲、仅导出累计地图而没有逐帧点云。本轮改为：

- 读取每个实例独立的 NxHxW 掩膜。检测到 boxes 却没有 masks 时明确报错，避免模型/推理接口不兼容后无声无效。
- 每个实例依据不同有效像素的可见深度建立独立深度区间，抑制掩膜内远处背景；膨胀边缘不能侵入另一车辆的原始掩膜。深度区间、至少支持像素数均可配置。
- 掩膜内部/边缘证据不足的点标记未知，不直接算车辆，也不算背景。分类投票要求足够车辆票，并设置显式背景否决。视角次数是选取图像数，不保证它们是独立视角。
- 缓存原始实例掩膜（含重叠），使用新缓存身份版本；旧的 union 掩膜缓存不会复用。模型权重、库版本、输入尺寸、置信度、类别等参与缓存键。
- 新增 opt-in 关键帧导出和 `--mode keyframes`：用时间接近的图像清理各关键帧，再按导出位姿拼接世界点云；不再用全地图代替当前扫描来建立每帧遮挡关系。
- 未观察到/证据不足/无法可靠估计地面的关键帧点保留。关键帧模式不进行整簇无证据扩张。不补造被车辆挡住的背景。

`--mode map` 仍支持已有 `global.pcd + keyframes.txt`，并使用新的实例过滤；其中深度仍来自累计地图/对齐深度图，无法获得关键帧模式的时间局部遮挡优势。缺少关键帧文件时不会悄悄回退，命令会明确说明如何重新导出。

## 性能与边界

- `system.export_keyframe_clouds` 默认 false，避免旧配置突然大量写盘。开启后 SaveMap 新增 `keyframe_clouds/<id>.pcd` 和最后发布的 `index.csv`。点云是当前保留的 IMU 系去畸变关键帧，受第二轮关键帧分辨率/点数预算约束，不是原始雷达包。
- 关键帧点云与 `keyframes.txt` 共享同一份回环/前端位姿快照，仍使用版本 2 坐标约定。读取时验证 ID、时间、路径和帧数。旧地图不能靠手写 manifest 修复。
- 仅解码关键帧 ±0.35 s 附近选中的图像，默认 0.2 s 最小间隔。同一图像即使对应多关键帧也只分割一次，持久缓存可跨次复用。每帧至少 3 张图像的车辆证据；端点/稀疏图像可能不够，结果是保留而非强删。
- 同时活动关键帧默认最多 8 个；超过限制报错，而非静默丢弃。内存仍随单关键帧大小及模型大小变化，不承诺固定 RSS。
- PCD 逐关键帧读写，地图二进制体写临时文件后补头，不在内存拼整图。最终 RViz 通过 mmap 抽样。`--rviz` 在关键帧模式只显示最终结果，过程用日志和 overlay 检查。
- **拼接保留重叠关键帧点，没有最终全局体素去重**，因此点数/文件大小可能比原 `global.pcd` 大；不能直接用总点数差衡量去车质量。清理不降低 retained frame 分辨率，也无法恢复此前关键帧已经丢弃的细节。
- 无效 XYZ 留在原始/清理后单帧 PCD，重建的世界点云排除无效点并统计数量。
- 深度分位数过滤仍是启发式：掩膜完全覆盖背景、标定误差、稀疏雷达、运动目标时间差都可能误判。M2DGR 没有对齐深度时使用单帧 z-buffer；未实现完整多帧射线自由空间推理/目标追踪/在线分割。
- 输出是清理点云及保留关键帧，不会同步更新原导航瓦片、G2P5 或位姿图。需要部署为定位地图时应另做地图转换，不能只替换原 `global.pcd` 便认为瓦片已更新。

## 本地更新与编译

保留本地标定、QoS 和已验证的 0.5 s 预测配置；不要 reset --hard。若有切换冲突先备份本地修改。

```bash
cd ~/lightning-lm-fresh
git status --short
git fetch origin
git switch --track origin/codex/instance-cleanup-round6
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
```

第一次创建独立建图配置，原 `m2dgr_slam_prediction.yaml` 保持不变：

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
src = Path('config/m2dgr_slam_prediction.yaml')
dst = Path('config/m2dgr_slam_instances.yaml')
if dst.exists():
    raise SystemExit(f'{dst} 已存在；请检查其中 system.export_keyframe_clouds 为 true')
cfg = yaml.safe_load(src.read_text())
cfg.setdefault('system', {})['export_keyframe_clouds'] = True
dst.write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
print(dst)
PY
./bin/run_slam_online --config config/m2dgr_slam_instances.yaml
```

另一终端以 0.5 倍速从头播放。仅建激光+IMU地图时：

```bash
source /opt/ros/humble/setup.bash
ros2 bag play "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --rate 0.5 --read-ahead-queue-size 3000 \
  --qos-profile-overrides-path ./config/m2dgr_playback_qos.yaml \
  --topics /velodyne_points /handsfree/imu
```

原数据包中的相机数据仍保留，去车脚本直接读取，无需播放相机。暂停/结束回放后，建图程序仍运行时保存到**新名称**（原 SaveMap 会重建同名目录）：

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap \
  "{map_id: street_06_instances}"
ls data/street_06_instances/keyframe_clouds/index.csv
```

关键帧导出开关在程序启动时读取。已经结束且只剩旧 global.pcd 的地图无法补回独立关键帧，需要用新配置重放建图。

## 去车

停止建图和 bag，释放内存。校核新配置的相机标定/话题与本地已验证配置一致；新配置不会自动继承本地修改。

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_06_instances \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_instances.yaml --device cpu --rviz
```

CPU 推理可能很慢，不必提高线程。已有本地可用的 `-seg` 权重可继续填写到 `yolo_model`；首次下载/模型版本兼容性不属于本次合成测试覆盖范围。

输出在打印的 `data/street_06_instances/clean_diagnostics/keyframes_<随机后缀>/`：

- `global_clean.pcd`：去车后的世界地图。
- `global_vehicles.pcd`：被删除点，用于检查误删。
- `keyframe_clouds/`：保持 IMU 坐标和字段的清理后关键帧，含 index。
- `keyframes.txt`、`map_frames.yaml`：对应位姿和坐标约定。
- `summary.json`：覆盖/无视图帧、地面证据不足帧、车辆票、移除/保留/无效点等统计。
- `overlay_*.jpg`：蓝色掩膜、红色车辆票、绿色背景、黄色未知、灰色遮挡。

原始地图、单帧点云不覆盖。若保留较多车辆，先看 overlay 和证据覆盖，不要直接降低阈值强删。

已有地图不重建的兼容用法（仍会写该目录下 `global_clean.pcd/global_vehicles.pcd`）：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_01_vehicle_test \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_instances.yaml --mode map --device cpu --rviz
```

## 验证

```bash
python3 -m unittest discover -s tests -p 'test_clean*.py'
python3 -m unittest discover -s tests -p 'test_vehicle_removal.py'
```

开发环境执行不下载模型的合成回归：实例重叠缓存、逐实例深度、背景/未知保护、实际 masks 接口、至少车辆票数、背景否决、关键帧清理/世界拼接/额外字段/源文件不变，以及第四轮坐标/旧地图/PCD 测试。完整 ROS2/PCL 编译、真实 YOLO 权重推理和 street_06 数据集误删/残留率尚需本机验证，不能宣称已提高实测精度。
