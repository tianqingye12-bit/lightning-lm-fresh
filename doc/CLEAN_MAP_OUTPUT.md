# 容易找到的语义地图输出

关键帧清理默认将整份结果直接保存到输入地图旁边。例如输入 `data/street_vehicle_test4`，输出为 `data/street_vehicle_test4_clean/`。存在同名结果时使用 `street_vehicle_test4_clean_2/`、`_3/`，保留之前的结果。原始地图不变。

结果目录包含 `keyframe_clouds/`、`keyframes.txt`、`map_frames.yaml`、处理统计、视图日志与诊断图；稠密轨迹有效并被使用时也保留配套轨迹。紧凑配置仍然只保存完整的清理关键帧集合。点云帧位于 IMU 系，必须与结果目录内的位姿文件一起使用。`--export-global` 可在结果目录根部额外生成 `global_clean.pcd`，坐标为保存地图的世界系，包含各保留关键帧的全部有限点、原有字段及帧间重叠，不使用 RViz 采样点代替完整地图，也不额外降采样。完整 PCD 可能较大，按逐帧流式方式写入，避免一次加载整图；磁盘临时空间和最终文件仍有开销。

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_vehicle_test4 \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_alignment13_keyframes.yaml \
  --device cpu --rviz --export-global
```

完成时打印绝对路径 `Clean map saved to: ...` 和 `Full cleaned map: .../global_clean.pcd`。不需要额外播放 bag。更新为 Python 改动，不需要 colcon 编译；已启动的进程不会自动加载修改。

`--output data/street06_static` 可指定结果目录，目录已存在则拒绝覆盖，不自动改写显式指定的名称。`--output` 与 `--export-global` 仅适用于 keyframes 模式；map 模式仍按原方式输出。过程预览继续使用有点数上限的采样地图，不影响保存的点云内容。

任务开始时写入 `INCOMPLETE`，保存全部结果和汇总后才移除。中断或错误产生的目录可能有部分 PCD，但不会移除这个标记，不能将其作为已完成的地图。每次新运行都重新处理，不自动断点续跑。

旧版本已经生成的结果仍留在原 `clean_diagnostics/keyframes_*` 下，更新不会自动移动它们。若仅需整理旧结果，先让对应的清理进程退出，再把**整个已完成结果目录**移动到 `data/<地图名>_clean/`；不要只移动 IMU 系 PCD 而遗漏位姿。旧紧凑结果没有单个完整 PCD 时，仅移动目录不会自动生成 global_clean.pcd。

验证覆盖默认同级目录、重复运行不覆盖、自定义路径拒绝覆盖与输入目录保护，以及真实投影/投票/PCD 输出流程中的完整地图参数、字段/坐标、原文件保护和过程预览。当前环境没有 ROS2，真实 RViz 显示与大规模数据包的磁盘占用仍需本机验证。
