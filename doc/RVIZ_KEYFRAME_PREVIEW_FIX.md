# 关键帧语义清理的 RViz 过程预览

旧版本在 `run_keyframe_clean` 全部处理完成后才创建 ROS 发布节点，只发布最终清理点云和删除点云，没有调用叠加图、当前相机和车辆票的过程发布。即使日志中的 `images`、`finished` 持续增长，处理期间 RViz 仍然空白。此问题与全图模式的实时发布逻辑不同。

修复后 `--rviz` 在开始遍历图像前创建发布节点并发布关键帧轨迹。原始预览随实际加载的关键帧累积，清理及删除预览随完成的关键帧累积。点云刷新间隔至少 2 秒，首次可投影视角及后续至少间隔 1 秒的视角发布叠加图和相机位姿；复用已计算的分割与投票，不额外执行模型推理。车辆票沿用发布器每十次视角更新一次的规则，仅表示所展示关键帧的累积投票。

预览按关键帧分配采样预算，每个点云话题的累计点数不超过 `rviz_max_points`；仅持有采样结果，不读取完整 global.pcd 或保留全部关键帧点云。点云预览使用世界系 `map`，单帧文件仍保存 IMU 系。只有清理完成的帧进入 clean/vehicles；处理中的帧可能先出现在 raw。预览是采样后的处理进度，不是完整地图或全部删除点。

所有保存、置信度、语义证据和判定参数保持原流程；预览不修改原地图。不使用 `--rviz` 时不导入 ROS 可视化模块。处理结束仍发布最终采样预览并保持节点存活，直到 Ctrl+C。

Python 修改不需要 colcon 编译。正在运行的进程不会自动加载新代码；可以等待当前任务结束后更新，再运行同一个清理命令。每次清理生成独立的 `clean_diagnostics/keyframes_*` 目录，不覆盖原地图或之前的输出。

启动 RViz：

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d config/clean_map.rviz
```

Fixed Frame 使用 `map`。点云和轨迹使用 transient-local/reliable 发布；点云显示的 Durability Policy 设为 Transient Local。两终端使用相同 ROS_DOMAIN_ID 和 RMW 环境。无需额外执行 ros2 bag play，脚本直接读取 bag。

验证使用真实几何投影/投票与 PCD 读写，替换模型推理和 ROS 发布器：检查原始点云、叠加图在完成前已经发布，最终清理/删除点云仍发布，采样点数不超预算，原始地图文件不变，输出删除结果不变。本环境无 ROS2，真实 DDS 发现、RViz 显示和本机负载仍需用户运行验证。

此修改不处理点云负时间偏移或稠密轨迹 `usable=0`。已有地图通过显式 `pose_trajectory.source: keyframes` 可以继续排查语义流程，但不能据此证明逐点去畸变或投影精度正确。
