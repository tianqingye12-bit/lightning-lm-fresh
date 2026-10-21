# 多帧车辆实例关联与原始关键帧静态地图重建

这是一条独立的可选流程，默认配置保持不变。仍用原分割权重与原标定，要求 map/segment、imu_v2 坐标约定及带类别置信度的分割输出。
本轮重点是几何稳定的停放车辆：不把车体部分残点当作独立目标，不用检测框直接删除，不叠加旧整簇扩展或上一轮种子补全。
未实现行驶车辆的按时间实例跟踪/逐扫描剔除；几何不稳定实例会被拒绝，保留原点云。
世界坐标中的静态一致性无法可靠区分慢速车辆与视角变化，不应宣称动态目标处理已完成。

## 实例提取和多帧确认

每张图保留独立分割实例及类别置信度。先保护估计地面，再从掩膜内缩核心提取深度一致的种子，沿完整掩膜内的三维连通分量提取车身。
默认至少 6 个不同投影像素、12 个点，连接距离 0.35m；超出单实例 30000 候选点则跳过。
重叠实例的共有点视为歧义，从两者的证据中排除，不借此合并两辆车。掩膜外或未分割到的点不自动记背景。

关联在统一地图坐标系执行，要求类别相同、距离不超过 2m、共享点至少 6 个且覆盖小实例的 15%以上；两种匹配接近时放弃关联。
同一张图不能为同一轨迹计两次，同一时间戳不重复计票；间隔超过 3s 不直接续接旧轨迹，不进行跨长时间重识别。
由于要求共享三维点，不相交的互补视角可能形成不同轨迹，这是一项保守限制，不代表已经解决所有实例关联。
最多 1000 条轨迹、共 500000 个轨迹-点成员记录；超过上限跳过整个新观测并记录，不静默截断实例。

确认条件默认：至少 3 张图、相机相对首个观测位置移动至少 0.35m、平均检测置信度至少 0.45，几何中心漂移不超过 1.5m。
仅保留至少在 2 张图中重复关联到的点作为三维种子。中心漂移是拒绝几何不稳定实例的启发式条件，不是运动真值。
以确认种子的水平 PCA 方向构建车体有向包围区域，长宽高上限 7×3.5×3m，外扩最多 0.4m。
只选择区域内、距离种子不超过 1m、非地面且与足够种子连通的点；不会删除整个包围盒，邻近独立结构仍保留。
候选过多时保留已确认种子的删除决策，跳过补全。
车体点不完整时，PCA 范围也可能不完整；连接到车体的背景结构仍可能误删，不能保证整车全部删除。

## 标签回传及重建

第一阶段生成 global_clean.pcd、global_vehicles.pcd 与 vehicle_labels.sqlite。
标签为已确认删除体素及其地面保护高度，包含原 keyframes.txt、map_frames.yaml 摘要；重建前必须与原始地图完全匹配。
标签写入原子替换。世界体素边界附近 0.1% 单元范围一致吸附到边界，减少全局 PCD float32 和关键帧 double 变换舍入不一致造成的漏查，0.1m 体素对应 0.1mm。
这不是向相邻整个体素膨胀；同体素点仍执行地面高度保护。

第二阶段逐个读取原始 keyframe_clouds，按保存位姿变换到世界坐标，查询共同标签，清除所有命中副本，保留字段，并以指定分辨率重新融合。
没有命中标签的点保留，标签表覆盖不到的车辆残点仍会进入最终地图；标签回传不是新的语义推断。
最终输出 global_only，只保留完整 PCD 和少量元数据，不额外保存整套关键帧副本，不修改原图、不删除历史结果。
此 global_only 结果不能代替需要完整关键帧归档的其他工具输入。

## 操作

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/configure_instance_tracks.py \
  --input config/m2dgr_clean.yaml \
  --output config/m2dgr_clean_instance_tracks.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/restore_map_cleanup.py prepare \
  --map data/street_vehicle_test4_map_mode \
  --output data/street_vehicle_test4_instances

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_vehicle_test4_instances \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_instance_tracks.yaml \
  --mode map --device cpu --rviz
```

在第一阶段结束保存结果后，使用另一个终端从原始关键帧重建；该步骤不再运行 YOLO，不用重播 bag：

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/rebuild_instance_static_map.py \
  --map data/street_vehicle_test4 \
  --labels data/street_vehicle_test4_instances/vehicle_labels.sqlite \
  --output data/street_vehicle_test4_static \
  --voxel-size 0.10
```

最终交付地图：data/street_vehicle_test4_static/global_clean.pcd。
第二阶段的 --map 必须指向含原始 keyframe_clouds/index.csv 的保存目录，不能指向只有 global.pcd 的准备目录。
需要先核对实际地图目录，不能用不同轨迹的关键帧拼接。配置与输出目录已存在则拒绝覆盖，使用新名字。

## 对齐与诊断

实例模式的 overlay_*.jpg 按轨迹时间分段保存，而不是只保留最开始的一段，便于检查前中后段投影。
图像红点应落在真实车身上。该可视检查不会自动验证外参、逐点去畸变或时钟同步。
已有 scripts/diagnose_projection.py 支持人工静态三维点-二维像素对应、不同时间偏差和轨迹来源的误差对照；需要真实人工对应，不能编造对应数据。
本轮没有修改相机外参、time_offset 或之前存在问题的逐点时间实现；未取得人工对应时不能宣布对齐已验证。

clean_diagnostics/run_*/vehicle_instances.json 保存每个实例的观测数、基线、置信度、中心漂移、确认状态及有向范围。
accepted_tracks 是确认的轨迹段数量，长时间重访或未关联的互补视角可能重复表示同一辆车，不能当作去重车辆总数。
summary.json 的 vehicle_instance_tracks 提供汇总。vote_rejections 仍解释原始逐体素投票；实例确认是另一条判定流程，不能把两者误当成相同门限。
removal_stages 中 instance_seed_voxels 与 instance_completion_voxels 分别表示实例种子和几何补全；raw_vote_difference_voxels 可以为负，added_voxels 在此模式为 null。
实例确认更严格时可能删除更少，不能用删点总数代替车辆召回率和背景误删率。

测试覆盖合成多视角、相邻车辆分离、重复图像、低置信度、相机基线、疑似运动几何、地面保护、资源上限、浮点边界，以及实际 map 主流程和多关键帧标签回传重建。
尚未使用用户原始 bag、真实 YOLO 权重或 ROS2 完成实测，也未测量实际识别准确率或运行速度。
