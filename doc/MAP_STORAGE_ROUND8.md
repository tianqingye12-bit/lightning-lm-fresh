# 第八轮：地图存储精简（保留5厘米设置）

基于第七轮 `e2f6f33`，分支 `codex/map-storage-round8`；第七轮 PR #9 仍未合并时，本轮 PR 以它为基线。这里优化的是保存方式，不通过增大体素、丢弃关键帧或截断高度来减小文件。

## 已确认的代码层面体积来源

原始保存同时包含全局点云、定位瓦片与逐关键帧；第六轮清理结果又包含保留所有重叠点的世界点云和清理后关键帧；第七轮导航输出还同时写全局点云与同一批瓦片。分割缓存、每次独立诊断运行也会占空间。用户本机文件未上传，因此没有测量本机实际比例，不承诺压缩百分比。

## 保存流程

| 用途 | 精简模式保留 | 精简模式省略 |
|---|---|---|
| 原始建图，供视觉去车 | 原分辨率压缩关键帧、位姿、坐标约定；按原开关保留二维图 | 重复 global.pcd 和原始定位瓦片 |
| 关键帧去车结果 | 清理后关键帧、位姿、坐标约定、统计和有限叠加图 | 巨大的重叠世界点云、被删车辆调试点云 |
| 最终定位/导航 | 定位瓦片、index.txt、二维地图、定位配置及完成记录 | 与瓦片重复的 global.pcd、历史关键帧与推理缓存 |

- C++ SaveMap 的关键帧默认改用 PCL `binary_compressed`，不修改点坐标或字段；旧配置仍默认导出 global.pcd 和瓦片。
- 新配置生成器默认 `--storage-profile compact`。`--storage-profile full` 恢复完整导出；原有本地配置不会自动被改写。
- 精简原始地图主要用于后续 `--mode keyframes` 去车，不能直接当成最终定位瓦片目录。
- 新清理配置 `config/m2dgr_clean_compact.yaml` 与实例分割配置使用相同标定/阈值，增加存储选项。旧 `m2dgr_clean_instances.yaml` 继续写完整世界点云。
- `build_static_navigation.py` 可直接逐帧读取清理结果，在同一IMU位姿约定下变换到世界系、按原5厘米体素聚合，不再要求中间 `global_clean.pcd`。默认不写重复 global.pcd，需要完整展示文件时加 `--keep-global`。
- Python压缩调用现有 `pcl_convert_pcd_ascii_binary`，确认字段、点数和VIEWPOINT一致；压缩变大时保留二进制原文件。一次最多对128 MiB输入文件调用转换器，较大瓦片跳过并计数，避免对大图进行一次性内存解压/压缩。128 MiB是输入大小限制，不是进程内存上限。`--no-compress` 可只省重复副本。
- 不自动删除任何已有地图、原始关键帧、缓存或旧诊断。创建精简副本后，旧目录仍占空间，需要用户确认用途和结果后自行管理。脚本统计的是新副本体积，并非系统磁盘立即释放量。

## 更新

```bash
cd ~/lightning-lm-fresh
git status --short
git fetch origin
git switch --track origin/codex/map-storage-round8
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
```

保留本地改动、相机标定/QoS和0.5秒预测配置。新分支包含第七轮功能；切换冲突先备份，不要强制覆盖。

## 已经保存的地图：先查看，再生成独立精简副本

下面的目录名需要换成本机实际路径。查看不会修改文件，也不需要重新建图：

```bash
python3 scripts/compact_saved_map.py \
  --source data/street_06_navigation_fine --inspect
```

输出按关键帧、全局点云、瓦片、历史清理结果、分割缓存等分类的字节数。

如果目录含 `index.txt`、编号PCD及 `map.yaml`，可以生成定位/导航部署副本：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/compact_saved_map.py \
  --source data/street_06_static_navigation \
  --output data/street_06_static_navigation_compact \
  --profile navigation
```

保留静态瓦片、索引/功能点、二维图及原目录实际存在的动态图层/动态多边形；不复制重复全局点云、关键帧、历史诊断和分割缓存。如果有 `localization.yaml`，更新其地图路径到新副本。它仅精简存储，不会把未经去车的地图变成去车地图，也不修复旧地图本身的标定或尺度错误。

如果目录有版本2元数据和 `keyframe_clouds/index.csv`，希望保留未来视觉去车能力：

```bash
nice -n 19 python3 scripts/compact_saved_map.py \
  --source data/street_06_navigation_fine \
  --output data/street_06_mapping_compact \
  --profile mapping
```

这不是可直接定位的部署目录，而是保留全部逐帧点数据的处理中间件。完成的关键帧清理结果也支持这个副本模式，并更新“世界点云未写出”元数据，便于直接生成导航地图。没有独立关键帧的旧地图会拒绝该模式，不能省去其唯一完整点云。

缺少PCL工具时可先检查：

```bash
command -v pcl_convert_pcd_ascii_binary
```

工具未安装时按环境安装 `pcl-tools`，或在精简/导航转换命令加 `--no-compress`；这仍会省去重复文件，但不压缩PCD。关键帧去车压缩开关为 `storage.compress_keyframes: false`。脚本不修改已存在的输出目录；失败带 `INCOMPLETE`，成功精简带 `STORAGE_COMPLETE.json`。

## 后续建图：直接采用精简保存

生成独立配置，保留原0.5秒预测和传感器设置：

```bash
python3 scripts/configure_navigation_detail.py \
  --input config/m2dgr_slam_prediction.yaml \
  --output config/m2dgr_slam_navigation_compact.yaml \
  --storage-profile compact
./bin/run_slam_online --config config/m2dgr_slam_navigation_compact.yaml
```

按第七轮以0.5倍速从头回放，暂停并等待队列处理完，再保存新名称，例如 `street_06_compact_raw`。保存结果会有关键帧文件，没有 global.pcd 和原始瓦片是该模式的预期行为。

手工配置时合并到已有system，不要重复根键：

```yaml
system:
  export_keyframe_clouds: true
  map_export:
    localization_resolution: 0.05
    compress_keyframes: true
    save_global_pcd: false
    save_localization_tiles: false
```

三类点云不能全部关闭，否则初始化报错。原SaveMap仍会重建同名目录，应始终用新的地图名称做对比。

## 精简视觉去车

停止建图与回放，确认新配置的相机标定与本机验证版本一致：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_06_compact_raw \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_compact.yaml --mode keyframes --device cpu
```

`--rviz` 仍可选，未写世界PCD时显示受点数预算限制的逐帧抽样，不需要临时拼整图。所有保留关键帧点仍在文件中，抽样只影响预览。

最终将实际 `keyframes_...` 路径代入：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/build_static_navigation.py \
  --source data/street_06_compact_raw/clean_diagnostics/keyframes_实际后缀 \
  --config config/m2dgr_slam_navigation_compact.yaml \
  --output data/street_06_compact_navigation
```

确认 `BUILD_COMPLETE.json` 存在、没有 `INCOMPLETE`，再加载其中的 `localization.yaml` 或 `map.yaml`。定位通过 `index.txt` 加载瓦片，不要求重复的全局PCD。

## 验证范围

```bash
python3 -m unittest discover -s tests -p 'test_map_storage.py'
python3 -m unittest discover -s tests -p 'test_static_navigation.py'
python3 -m unittest discover -s tests -p 'test_clean*.py'
python3 -m unittest discover -s tests -p 'test_vehicle_removal.py'
python3 -m unittest discover -s tests -p 'test_map_detail_config.py'
python3 tests/run_navigation_grid.py --sanitize
```

已验证省略世界点云不创建整图spool、预览点数有界、关键帧字段/点值保留、非单位IMU位姿下直接逐帧聚合与世界点云聚合一致、不写global时瓦片字节相同、独立副本与原文件保护、导航静/动态图层保留、路径重写和压缩失败不替换原文件。压缩适配单元测试模拟PCL进程；当前环境没有pcl-tools，真实PCL逐字段/数值往返测试自动跳过。完整ROS/PCL工程构建、真实压缩比、真实M2DGR回放/定位未在开发环境验证，不宣称实际节省百分比或定位精度提高。
