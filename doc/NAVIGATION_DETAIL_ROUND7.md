# 第七轮：细节保留、尺度一致性和静态导航地图生成

基线：master `fd2a721`，已包含前六轮。分支 `codex/navigation-detail-round7`。本轮不改变已验证的 IMU 预测限时、配准算法或相机标定；没有承诺厘米级实际定位精度。

## 改动

1. 原 SaveMap 的 `map.yaml` 分辨率固定写成 0.05，现改为写入实际 OccupancyGrid 分辨率和原点/yaw，PGM 按 ROS→图像的上下方向转换。共享保存函数同时用于在线导出和新的离线转换。若旧地图实际为 0.20 m/格却被写成 0.05，则旧 YAML 的尺度有误；不能通过缩放点云修复。应从原地图重新导出，并确认当时实际生成分辨率。
2. 新精细配置生成器：关键帧 0.05 m、默认最多 60000 点，三维导出默认 0.05 m（可选 0.03），定位瓦片 0.05 m，导航栅格 0.05 m。配准降采样、局部地图过滤、关键帧触发条件、地面/障碍物高度、传感器标定与 0.5 s 预测设置均继承本地输入配置。不会修改原配置，也不会覆盖已有输出。
3. SaveMap 增加 `system.map_export.localization_resolution`，旧配置默认仍为 0.1 m；新配置使用 0.05。否则更细 global.pcd 在定位瓦片中仍会被降到 10 cm。
4. 修复原 TiledMap 转换中新瓦片漏加第一个点的问题，避免单点瓦片直接丢失。
5. 新增 `build_static_navigation.py` 和 `bin/build_navigation_grid`：从第六轮的**完成且含清理后关键帧**的运行目录，分别生成定位瓦片与导航栅格。旧 map 模式结果不具备清理后逐帧数据，拒绝混用，防止把未清理关键帧重新用于导航栅格。

## 转换的几何与资源边界

- 定位层读取世界系 `global_clean.pcd`，与现有 TiledMap 一样采用 100 m 瓦片，索引为 `floor(x/100 + 0.5)`，包含负坐标。体素按世界 XYZ 去重并取均值，保留 x/y/z/intensity/time；其他字段不用于现有定位接口。
- 使用 SQLite 磁盘聚合和分块 PCD 读取，不在内存中持有整张世界点云。默认块为 10 万点，SQLite page cache 16 MiB。这不代表整个进程只占16 MiB；NumPy、模型外部环境、页缓存和导航网格还会占用内存。SQLite 可产生 GB 级临时文件，CPU 转换可能较慢。
- 栅格使用现有 G2P5 射线逻辑，从清理后 IMU 系关键帧和同一组优化位姿重新计算，并正确应用 IMU→LiDAR 外参。逐帧同步渲染、不保留完整关键帧列表、不启动回环重绘线程。
- 没有把“删除车辆后的空洞”直接涂成可通行区，仍依赖剩余真实观测；未知区域可能保留。沿用 G2P5 原有地面与射线启发式，不是新自由空间重建算法；角度分箱仍为1度，5厘米格子并不代表所有距离上都有5厘米观测精度。
- `g2p5.max_grid_cells` 在分配前限制网格尺寸，新配置默认800万格，旧配置缺省0维持原预算行为，但统一限制整数索引溢出。在线超限时停止发布/导出该网格并记录错误，LIO继续；离线超限明确失败。必要时选择更粗分辨率或分区地图，不能仅为消除提示无限增大。
- 输出必须是新的独立目录。失败保留 `INCOMPLETE`，只有 `BUILD_COMPLETE.json` 出现且 `INCOMPLETE` 消失才表示转换完成。原始地图/去车输出不修改，不复制旧动态图层；生成的定位配置禁用旧动态图层加载/保存。
- 细节提升受原雷达采样、标定、去畸变、位姿一致性和关键帧点数上限限制。导出0.03不能恢复已经被过滤的点；画面更密不能作为定位精度提高的证据。

## 更新与编译

```bash
cd ~/lightning-lm-fresh
git status --short
git fetch origin
git switch --track origin/codex/navigation-detail-round7
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
ls -lh bin/build_navigation_grid
```

保留本地标定/QoS/预测配置；切换冲突时备份修改，不要强制覆盖。

## 1. 生成独立精细建图配置

```bash
python3 scripts/configure_navigation_detail.py \
  --input config/m2dgr_slam_prediction.yaml \
  --output config/m2dgr_slam_navigation_fine.yaml
```

默认关键帧和导出均5厘米。可另选 `--export-resolution 0.03`，但先用默认设置验证负载。关键帧仍最多60000点；实际单帧频繁触顶时会均匀抽样，不能假定降体素后细节必然提高。若输入配准是0.3m，生成后仍为0.3m。`system.with_g2p5` 开关继承原值，允许仅在离线转换时生成5厘米栅格。

## 2. 从头建图并保存新地图

```bash
./bin/run_slam_online --config config/m2dgr_slam_navigation_fine.yaml
```

另终端：

```bash
source /opt/ros/humble/setup.bash
ros2 bag play "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --rate 0.5 --read-ahead-queue-size 3000 \
  --qos-profile-overrides-path ./config/m2dgr_playback_qos.yaml \
  --topics /velodyne_points /handsfree/imu
```

结束/暂停回放，待队列处理完成后，在建图仍运行时保存到新名称：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /lightning/save_map lightning/srv/SaveMap \
  "{map_id: street_06_navigation_fine}"
ls data/street_06_navigation_fine/keyframe_clouds/index.csv
```

## 3. 按关键帧去车

停止建图/回放释放资源，然后执行。确认新去车配置内的相机话题和标定与本地验证版本一致。

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_06_navigation_fine \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_instances.yaml --mode keyframes --device cpu
```

若已有第六轮成功的 keyframe 模式输出，可以直接转换，无需重新建图；但其中此前丢失的细节不能恢复。旧的 map 模式输出不能直接用于本命令。

## 4. 从去车结果生成完整静态导航地图

把 `--source` 的后缀替换为上一步打印的**实际运行目录**，不要直接传原始地图目录：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/build_static_navigation.py \
  --source data/street_06_navigation_fine/clean_diagnostics/keyframes_实际后缀 \
  --config config/m2dgr_slam_navigation_fine.yaml \
  --output data/street_06_static_navigation
```

输出：

- `global.pcd`、编号 `.pcd`、`index.txt`：同一套去车后、5厘米去重的定位地图，含起点功能点。
- `map.pgm`、`map.yaml`：从对应清理关键帧重算的二维导航地图，真实分辨率与原点。
- `localization.yaml`：继承传感器/定位参数，`system.map_path` 指向新目录。
- `keyframes.txt`、`map_frames.yaml`、`BUILD_COMPLETE.json`：坐标与转换统计。

已有输出目录不覆盖；需要重跑时换一个新的输出名称。生成配置使用本机绝对路径，地图移动后需要相应更新 `system.map_path`。

## 5. 查看与验证

```bash
cat data/street_06_static_navigation/BUILD_COMPLETE.json
cat data/street_06_static_navigation/map.yaml
```

现有雷达定位程序可加载新定位配置：

```bash
./bin/run_loc_online --config data/street_06_static_navigation/localization.yaml
```

仍沿用项目原有初始化位姿和坐标系约定；如果测试起点变化，需要按现有接口设置正确初始位姿。Nav2地图服务加载 `data/street_06_static_navigation/map.yaml`，并核对机器人 footprint、TF、局部障碍物层和代价地图配置。G2P5的障碍高度参数必须对应实际机器人，数据集配置不能自动代表真实机器人可通行空间。本轮没有配置整套Nav2或进行实机导航验证。

同包同速度比较：墙面是否单层、路沿是否连续、回到原位置时有无重影、定位误差/重复性和掉帧统计。使用独立回放/实测路线做定位对比；不能只在建图轨迹上看“能对齐”。

## 验证范围

已执行：

```bash
python3 tests/run_navigation_grid.py --sanitize
python3 -m unittest discover -s tests -p 'test_static_navigation.py'
python3 -m unittest discover -s tests -p 'test_clean*.py'
python3 -m unittest discover -s tests -p 'test_vehicle_removal.py'
python3 -m unittest discover -s tests -p 'test_map_detail_config.py'
```

覆盖实际PGM/YAML保存的5cm/20cm比例与方向、预算边界、跨批次去重、负坐标瓦片边界、点数和强度均值、完整包装、失败标识、路径带空格、原始文件保护及前轮去车回归。Python打包测试明确模拟C++栅格工具，没有声称跑过真实G2P5。

当前环境无ROS/PCL，未完成全工程编译、真实G2P5/定位瓦片加载或street_06回放。为本机新增可选集成测试（在完整colcon构建后）：

```bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1 \
  --packages-select lightning --cmake-target test_navigation_export
./bin/test_navigation_export /tmp/lightning_navigation_round7_test
```

测试目录必须不存在。该测试检查PCL瓦片重载不丢首点、非单位外参下流式/保留关键帧G2P5输出一致及预算拒绝。本轮保持草稿PR，待本机完整构建和地图检查。
