# 用原有效配置恢复全图清理对照

用户反馈原 `m2dgr_clean.yaml` 能删除车辆，后续 keyframes 配置效果下降。代码差异包括：旧配置缺省走 map 模式，对整张地图的体素查询可见性，不限制每个点只能使用关键帧时间附近的图像；旧配置 mask_dilate_px=7，后续紧凑配置为 2；后续质量配置启用实例最低置信度、核心掩码、均值置信度及证据覆盖/基线门限；map 模式的受限簇扩展仅在这些额外证据/质量门限关闭时启用，keyframes 模式没有这种扩展。

这些改变可能降低召回。不能根据更多轮次或合成测试宣称实际去车效果更好。另一个实际差异是旧 depth_margin 键在后来实例关联中已不再使用。恢复模式核对历史提交 4173ff3 后，显式接回该版本的掩码并集、最小深度滤波膨胀及 depth_margin、正票/可见次数比例和整簇扩展逻辑，不回滚现有坐标约定、日志或整个历史工程。

`restore_map_cleanup.py config` 深拷贝旧文件，保留原标定、模型、掩码膨胀、采样与簇参数，明确选择 map、keyframes 位姿来源，关闭新 cross/semantic_evidence/segmentation_quality 开关并启用 legacy_map_cleanup。原配置文件不改。cluster 最大尺寸、最少观测数和地面保护仍执行；旧扩展会整簇删除，可能删除同簇的非车辆结构，不能宣称保留了后来新增的背景否决。应检查叠加图和删除点云。默认配置不启用此恢复选项，keyframes 模式拒绝启用。

`prepare` 为独立的对照目录准备输入：若原地图已有 global.pcd，流式复制该原始地图；若没有，则逐帧读原始关键帧，按保存位姿变换到世界系，以指定体素分辨率保留真实观测点构成 global.pcd，不运行 YOLO。只复制位姿/坐标元数据，不复制整套关键帧归档，不复用此前删过的地图。原始文件不变；输出已存在则拒绝覆盖。准备完成不代表已经清理。

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/restore_map_cleanup.py config \
  --input config/m2dgr_clean.yaml --output config/m2dgr_clean_recovery.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/restore_map_cleanup.py prepare \
  --map data/street_vehicle_test4 --output data/street_vehicle_test4_map_mode \
  --voxel-size 0.10

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_vehicle_test4_map_mode \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_recovery.yaml --mode map --device cpu --rviz
```

此命令不加 --compact-result 或 --global-voxel-size，它们属于 keyframes 导出选项。对照输出直接在 `data/street_vehicle_test4_map_mode/global_clean.pcd`，删除点在 global_vehicles.pcd，原始全图在 global.pcd。整个地图参与投票时内存开销比关键帧模式大，不与其他清理任务同时运行；脚本保留现有全图模式的处理方式，未承诺严格内存上限。CPU 线程数继续限制为 2。

这份对照目录会保留原图和清理图便于比较。验证有效后可用已有 compact_clean_map.py 导出单份紧凑交付地图；此处不自动删除任何历史结果。RViz 中分别开关 Raw map、Clean map、Removed vehicles，避免原图叠加使车辆看起来仍存在。

先看真实 overlay 是否正确覆盖车辆、投影是否落在车身，再比较 vote 与 after clusters 删除量、地面和背景误删情况。该恢复没有修复此前负点时间/去畸变问题，也没有真实数据或 ROS2 验证。新增测试验证配置保留、模式/开关恢复、原文件保护、原始点云坐标/字段准备、历史深度关联与投票，以及合成点云经过实际 map 流程的种子删除和整簇扩展。
