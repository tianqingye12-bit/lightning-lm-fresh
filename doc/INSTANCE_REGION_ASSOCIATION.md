# 部分车身观测的区域关联对照

用户实测旧实例关联：162 个轨迹段中 88 段观测不足、50 段确认；存在 44 次歧义匹配。
该统计提示关联断段值得排查，但不证明真实原因一定是断段，也不能替代实际掩膜/投影检查。
本轮保留 PR #17 的点级删除兜底，只修改实例关联，不改变 YOLO 权重、分割/投票门限、时间偏移、标定、地面保护或删点补全范围。

## 方法

vehicle_instance_tracks.region_matching 默认为 false，旧配置仍使用共有点关联。
新配置仅将此开关设为 true，并启用整张图的批量双向一对一匹配：

- 共有三维点及其比例满足原条件时优先匹配。
- 共有点不足时，允许类别一致的局部三维区域匹配。按世界坐标轴向包围区域比较，双方各加 0.15m 的关联容差。
- 新旧观测中心相距不超过 0.75m，新中心也须位于轨迹首个观测中心 0.75m 内，防止逐次小偏移无限串接。
- 区域 IoU 至少 0.20、交集占较小区域至少 0.50；各维尺寸比不超过 2.5。
- 形状具有明确水平主方向时，主方向差不超过 25°；累计空间跨度仍受车辆尺寸的水平外接对角线和高度限制，最终确认另执行原有向尺寸检查。
- 时间间隔仍不超过原来的 3s（或输入配置原值），不做长间隔重识别，不跨类别合并。
- 所有提议对同一张图开始时的轨迹状态评分。只有提议和轨迹互为最优、且双方与次优差值至少 0.1 时才关联。
  歧义和竞争失败的观测不强行建立重复轨迹；同一轨迹在同一张图最多更新一次。

关联容差只用于判断两个部分观测是否可能属于同一目标，不是车辆删除膨胀，不直接生成删除点。
区域关联不会把观测一次的点自动升为多次观测：仍要求实例至少 3 张图、相机基线至少 0.35m、点至少重复观测 2 次以及原置信度、地面和尺寸门限。
完全没有分割到的车辆、投影错误、不相交且形状差异过大的视角、间隔过长或重复点不足的实例，仍可能保留。
紧邻或部分重叠的目标仍有误关联风险；双向检查是保守约束，不是关联正确性的保证，也不是行驶车辆跟踪器。

## 新统计

vehicle_instance_tracks 中增加 shared_point_matches、region_matches、new_tracks、one_to_one_conflicts，以及 association_rejections。
time_gap、center、region_overlap/size/yaw/anchor/extent 等拒绝计数是候选“观测-轨迹对”的数量，不是车辆数量，不能与轨迹状态直接相加。
每个 vehicle_instances.json 轨迹段包括首次/末次时间、观测中心和范围、两类成功关联次数，包括被拒绝的轨迹段，方便查找同位置短段。
accepted_tracks 是轨迹段数：成功合并后它也可能减少，不能单靠它评价车辆召回率。

## 使用

本轮仅 Python，无须重新编译。输入必须是之前已启用实例处理的配置，生成器深拷贝并只改变 region_matching：

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/configure_instance_association.py \
  --input config/m2dgr_clean_instance_tracks.yaml \
  --output config/m2dgr_clean_instance_regions.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_vehicle_test4_instances_v2 \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_instance_regions.yaml \
  --mode map --device cpu --rviz
```

重复使用原准备目录，不重新复制整图；原始 global.pcd 保留，global_clean.pcd 和车辆标签更新。
推理设置、权重和图像一致时可复用已有 scored cache；诊断另建 run_* 目录。
等出现 RViz topics are latched 后 Ctrl+C，再使用更新标签重建新的对照地图：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/rebuild_instance_static_map.py \
  --map data/street_vehicle_test4 \
  --labels data/street_vehicle_test4_instances_v2/vehicle_labels.sqlite \
  --output data/street_vehicle_test4_static_regions \
  --voxel-size 0.10
```

最终地图：data/street_vehicle_test4_static_regions/global_clean.pcd；原始关键帧不改、不复制整套归档。
新配置/输出已存在时换新名字，工具拒绝覆盖。若需要保存第一阶段旧清理图作精确对照，应在重跑前自行另存；之前生成的 static 目录不受影响。

## 验证边界

合成对照使用同一地图、同一帧序列和原门限：交替观测车身不同点四次，旧方案产生两条各两帧的短段，新方案合成一条四帧轨迹。
实际 map 主流程中，旧方案保留全部 24 个车辆点，新方案删除这 24 点，后方背景点保留。
同时验证观测仅一次的点不能借区域关联通过重复点门限、相邻目标分离、提议顺序不变性、双向歧义、共有点优先、尺寸/方向/漂移/时间限制及默认关闭。
这些不是用户真实数据上的识别精度、删除召回率或性能测量。未取得真实 overlay 对照，不能保证这一轮改善实际残留。
