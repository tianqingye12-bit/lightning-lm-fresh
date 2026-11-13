# M2DGR 逐点时间与逐帧轨迹修复

本轮针对上传的 street_vehicle_test4 诊断：4928 条逐帧轨迹只有 7 条 usable，
大多数帧的点时间横跨约 0.1 秒，而旧同步区间约为 0.000643 秒。
那 7 条 usable 也存在约 0.098 秒的点时间/帧末差异，不能通过修改 CSV 标记修复。

本轮完成时间适配、轨迹一致性检查、原始消息审计、工作地图轨迹复制和新配置生成。
车辆分割、投票、实例关联及删除阈值保持原配置；本轮不增加同步扫描帧上的语义关联，
也不宣称已经提升实际去车率。应先验证这条时间基础，再调整车辆关联。

## 时间约定

新配置显式使用 fasterlio.scan_timing.reference=relative_header：

- Velodyne 的原始 time 是相对 ROS header 的秒偏移，time_scale=1000 转成内部毫秒。
- 根据全帧最早和最晚点确定起止时间，不使用最后一个点代替最大值。
- 将内部点时间平移到扫描起点，保持 header+原始偏移 == 新起点+新偏移。
- time 字段存在时，不因为最后一个值为负/零而切换到角度估算。
- 允许 50～150ms 的扫描跨度，点时间须位于 header 前后 200ms 内。
  无效时间、错误字段或非递增扫描末时刻会停止输入并使轨迹导出标为不完整。
- IMU 去畸变覆盖扫描起点。相邻帧最多 2ms 的重叠用已有状态作短时回推；
  超过该范围时仅推进 IMU 状态，跳过该扫描的配准/入图，并保留不可用状态。

该模式只支持本次 Velodyne PointCloud2 输入，不自动猜测其他驱动的时间含义。
未配置新模式时继续使用 legacy 同步策略。原始审计检验数值与字段布局，
不能单独证明相机/IMU/雷达的物理同步。

## 1. 更新并低负载编译

本轮以补丁交付，尚未发布远端分支。补丁以你上传记录中的
codex/instance-region-association、d792cfd 为基线。
先下载 scan-time-normalization.patch；以下假设下载到 ~/Downloads：

~~~bash
cd ~/lightning-lm-fresh
git branch --show-current
git log -1 --oneline
git apply --check "$HOME/Downloads/scan-time-normalization.patch"
git apply "$HOME/Downloads/scan-time-normalization.patch"

source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
~~~

若检查报错，先保留输出，不要强制覆盖本地修改。
补丁只修改本轮代码、测试和说明，保留本地生成的 M2DGR YAML 和旧地图。
本轮有 C++ 修改，必须重新编译后再运行。

## 2. 生成配套的新配置

~~~bash
cd ~/lightning-lm-fresh

python3 scripts/configure_scan_timing.py mapping \
  --input config/m2dgr_slam_alignment13.yaml \
  --output config/m2dgr_slam_timing.yaml \
  --handsfree-extrinsics

python3 scripts/configure_scan_timing.py semantic \
  --input config/m2dgr_clean_instance_regions.yaml \
  --output config/m2dgr_clean_timing.yaml
~~~

输入使用你上传的现有配置。生成器保留建图细节参数、相机标定、YOLO、实例与投票设置。
输出文件必须不存在；重复实验时换一个新文件名，不覆盖原配置。

--handsfree-extrinsics 仅用于 /handsfree/imu：
官方文件提供 T_lidar_handsfree，取逆后得到本项目需要的 T_imu_lidar，
平移为 [0.27255, -0.00053, 0.17954] 米，旋转为单位阵。
这个选项明确关闭在线外参估计，应用于随后新建的地图。
若仅对时间处理做单因素对照，省略该选项，保留原外参。

不要给旧地图改写 map_frames.yaml，也不要将新建图轨迹替换进旧地图。
新坐标约定会由实际建图程序写入新地图。

标定来源：[M2DGR calibration_results.txt](https://github.com/SJTU-ViSYS/M2DGR/blob/main/calibration_results.txt)。
相机的 K、畸变和 T_cam_lidar 不由本轮修改。

## 3. 先审计原始点时间

无需启动 SLAM 或 YOLO：

~~~bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/audit_lidar_timing.py \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_slam_timing.yaml \
  --scans 300 \
  --output data/street06_raw_timing_audit.json
~~~

检查 errors 是否为空，扫描跨度是否在约 0.1 秒附近。
原始 negative_offsets 可以存在；归一化后的内部偏移应从 0 开始。
若报错，保留报告检查驱动字段、单位和时间含义，不放宽时间范围绕过错误。
报告输出也不覆盖已有文件。

## 4. 先做 30～60 秒建图验证

终端一：

~~~bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 ./bin/run_slam_online --config config/m2dgr_slam_timing.yaml
~~~

终端二只回放建图需要的传感器话题：

~~~bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash

ros2 bag play \
  "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --rate 0.5 \
  --topics /velodyne_points /handsfree/imu \
  --qos-profile-overrides-path config/m2dgr_playback_qos.yaml
~~~

播放约 30～60 秒数据后按空格暂停，等建图追上。
终端三保存测试地图，建图进程此时仍需运行：

~~~bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /lightning/save_map lightning/srv/SaveMap \
  "{map_id: street06_timing_smoke}"

python3 scripts/audit_scan_trajectory.py --map data/street06_timing_smoke
~~~

验收项：

- 程序日志中 scan_timing duration_s 接近 0.1 秒，不再是约 0.000643 秒。
- 审计 valid=true，format_version=2，point_end_mismatch_scans=0，
  scans_with_negative_offsets=0。
- 每个保存的关键帧都对应一条可信的逐帧位姿。
- 初始化、配准拒绝、IMU 间断可造成 unusable，不要求全程所有行都是 1。
- 继续用叠加图/静态标志物检验投影；通过时间审计不代表建图精度已经被真值验证。

测试通过后，退出旧进程并从头回放整包，保存到新的 map_id，例如 street06_timing_01。
再运行同一审计命令检查完整地图。不要用未暂停的旧进程重放另一个时间周期。

## 5. 准备工作地图并清理

完整地图审计通过后，关闭建图进程以释放内存：

~~~bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/restore_map_cleanup.py prepare \
  --map data/street06_timing_01 \
  --output data/street06_timing_01_instances \
  --require-dense

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street06_timing_01_instances \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_timing.yaml \
  --mode map \
  --device cpu \
  --rviz
~~~

--require-dense 在生成大地图前验证新时间契约，并复制 trajectory.yaml、
scan_trajectory.csv、trajectory_anchors.csv。默认旧 prepare 命令仍保留原来的关键帧模式。
新语义配置显式要求配套的新逐帧轨迹，防止自动退回稀疏插值。

可另开终端：

~~~bash
rviz2 -d config/clean_map.rviz
~~~

完整清理结果在 data/street06_timing_01_instances/global_clean.pcd，
车辆标签在同目录 vehicle_labels.sqlite。保留原始关键帧作为几何来源，
需要重新导出静态地图时：

~~~bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/rebuild_instance_static_map.py \
  --map data/street06_timing_01 \
  --labels data/street06_timing_01_instances/vehicle_labels.sqlite \
  --output data/street06_timing_01_static \
  --voxel-size 0.10
~~~

结果直接位于 data/street06_timing_01_static/global_clean.pcd。

## 验证范围

离线 C++ 测试覆盖符号偏移、点顺序、绝对时刻不变、错误时间拒绝、轨迹导出/读取契约。
Python 测试覆盖原始 PointCloud2 字段读取、配置保留、旧错误 usable 检出、轨迹复制及源文件保护。
本开发环境没有 ROS2/PCL 或完整 bag，无法执行实际建图/完整 ROS2 编译。
在 Humble 编译后，可运行以下实际库集成回归（包含预处理和去畸变前缀覆盖测试）：

~~~bash
python3 tests/run_lio_regressions.py
~~~

下一轮再对同步扫描帧上的语义关联、静态车辆可见面变化与夜景识别做对照。
实际效果需以新地图的投影、结构保留和车辆残留检查为准。
