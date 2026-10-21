# 车辆种子局部补全

用户的恢复模式结果有 7971489 点，删除 39722 点，未观测体素 6193115。
这些统计说明直接视觉覆盖有限，但不能把全部未观测点当成车辆；尚无残留车辆的真实投影诊断。
此改动只针对已有车辆种子周围的连续残点，不修改标定、位姿插值、YOLO 阈值、15m 有效距离或投票门限。

## 方法与边界

新 `seed_local_cleanup.enabled` 默认关闭。显式开启仅支持 map 模式的 legacy_map_cleanup 恢复流程，并替代原整簇扩展，二者不叠加。
种子必须是原投票已判定删除的非地面体素。将种子按 0.35m 邻接分组；默认每组至少 6 个种子。
每组在种子包围盒外最多扩 0.6m，包围盒总尺寸受 7×7×3m 上限约束；候选点距离原始种子不得超过 0.6m，不能用新补点不断向远处延伸。
以 0.25m 邻接构建局部连通图，每点最多查询 16 个邻居，只保留含至少 6 个本组种子的连通分量。
地面、明确背景票和至少 3 次可见但车辆票比例不高于 0.2 的体素禁止作为补点或连通桥。
已确定的种子删除决策不受局部扩展的跳过影响。

复用现有地图 KD-tree，先检查局部球内候选数，超过 20000 则整组跳过。最多处理 2000 组，优先较大种子组。
图边数受每点 16 个邻居限制，不构建整张地图的邻接图。原全图读取及体素投票仍可能占用较多内存。
所有跳过原因和新增体素数写入 summary.json；这是工程资源上限，不是车辆识别准确率保证。

该方法仍可能扩展到紧贴车身的非车辆结构，必须对照删除点云和图像叠加图验证。
完全没有车辆种子的目标、离种子较远的背面、被背景证据保护的残点，仍会保留；不会凭空推断整辆车。
它也没有修复原始逐点时间或相机/雷达时空标定问题。

## 运行

从原有效配置生成新配置，原文件不变，输出文件存在时拒绝覆盖：

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/configure_seed_cleanup.py \
  --input config/m2dgr_clean.yaml \
  --output config/m2dgr_clean_seed_local.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/restore_map_cleanup.py prepare \
  --map data/street_vehicle_test4_map_mode \
  --output data/street_vehicle_test4_seed_local

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_vehicle_test4_seed_local \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_seed_local.yaml \
  --mode map --device cpu --rviz
```

prepare 复制 map_mode/global.pcd（原图），不复制 global_clean.pcd，不修改前次结果。
最终完整地图在 `data/street_vehicle_test4_seed_local/global_clean.pcd`，删除点云为同目录 `global_vehicles.pcd`。
summary.json 在 `clean_diagnostics/run_*/` 下，新增 `removal_stages`（投票、最终、新增体素数）、
`seed_local_cleanup`（小簇、尺寸、候选数、组数上限等跳过计数）、`coverage`（未观测、不足观测、正票覆盖）。
比较的是同一原图的不同扩展策略，并不保证删除总数高于旧整簇扩展；不能仅用删除点数评估好坏。

仅通过合成几何和实际 map 主流程的回归测试验证；未在用户原始 bag、真实 YOLO 或 ROS2 上验证。
