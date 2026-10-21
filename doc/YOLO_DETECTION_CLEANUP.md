# YOLO 检测框驱动的车辆点云清理

`vehicle_perception: detect` 使用检测模型的 xyxy、类别及置信度，不读取分割掩膜。
默认仍为 segment，不自动把 -seg 模型改成检测模型。实际权重任务与配置不符时直接报错。
首版检测流程只支持 map 模式，独立配置关闭 legacy_map_cleanup 和额外 segmentation_quality、semantic_evidence、cross_keyframe_semantics 开关。
不影响已有分割/关键帧清理路径。

## 检测框不是删除掩码

每张图中各个检测框独立处理：

1. 先按保存轨迹估计地面，保护贴地层；点云仍经过原有投影和遮挡检查。
2. 取框中央的内缩区域（默认排除左右/顶部 15%，下边界在框高 70%），寻找非地面可见深度支持。
3. 同一像素最多贡献一次核心支持，默认需要 6 个不同像素落入同一个 0.5m 深度桶。
4. 以最近的受支持深度桶为锚点，只查询前方 0.25m 至后方 3m 的框内点云。
5. 构建 0.35m 邻接图，每点至多 16 个邻居。保留具有核心支持、至少 10 个点、相机系尺寸不超过 7×3×7m 的连通分量。
6. 一张图的重叠框对同一地图体素最多记一张正票；未匹配的点记未知，不记背景。

默认至少需要 3 张有效正票才形成删除种子；分母采用正票加显式背景票，不把未检测到当成车辆反票。
置信度仅用原 yolo_conf 过滤检测框，没有声称它是可靠概率。
之后可启用上一轮的 0.6m 局部几何补全；检测配置关闭“可见但未命中即背景”的补全否决，仍执行地面、距离、尺寸与资源约束。
整个大簇扩展在检测模式下始终关闭。

框内不连续的远处背景和受保护地面不会仅因落在框内就被删除。但相邻车辆、紧贴车体的结构、核心被遮挡或框内只有墙面等情况仍可能误关联/漏关联。
这是二维语义和三维几何的启发式关联，不是经过训练的三维车辆检测器，也未实现跨图车辆实例跟踪。
独立检测框在单张图内保留，不意味着已经完成跨帧实例关联。
完全没有检测到、没有足够深度支持的车辆，以及超过局部补全距离的遮挡背面，仍可能保留。
不承诺比实例分割更准确或删除全部车辆；需要相同地图上的对照和图像投影验证。

## 缓存与统计

原始检测框、类别和置信度缓存在单独的 .vehicle_box_cache 中，不复用分割缓存，不缓存地图相关删除决策。
缓存键包含权重摘要、推理设置、版本、图像内容和独立检测表示标识；写入原子替换，损坏缓存重新推理。
单框候选超过 10000 个点时跳过；局部图边数至多约 16×候选数，避免全对全搜索。
全局地图读取、KD-tree 和投票本身仍可能占用较多内存，不承诺全流程固定内存上限。

summary.json 新增 vehicle_perception、detection_association（核心支持不足、候选过多、尺寸超限、接受分量数），
以及 vote_rejections。后者对获得过正票的体素按顺序互斥统计：观测不足、正票次数不足、比例不足、背景否决、地面保护、额外门限、通过种子。
因此可以定位之前“有正票但没有成为删除种子”的原因。这些数是体素/检测框几何统计，不是车辆召回率。
分割模式也会输出 vote_rejections，便于同图对照。地面在检测关联前已被排除，其保护点不产生正票，因此不计入检测模式正票拒绝统计。
overlay 蓝色区域是检测矩形，红色投影点才是有效车辆正票，矩形内黄色点表示未知。

## 操作

本次只改 Python，无须 colcon 重新编译。保留原结果，从已有 map_mode/global.pcd 复制原图到新目录：

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/configure_detection_cleanup.py \
  --input config/m2dgr_clean.yaml \
  --output config/m2dgr_clean_detect.yaml \
  --model yolo26n.pt

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/restore_map_cleanup.py prepare \
  --map data/street_vehicle_test4_map_mode \
  --output data/street_vehicle_test4_detect

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_vehicle_test4_detect \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_detect.yaml \
  --mode map --device cpu --rviz
```

可将 --model 替换为已有的兼容 Ultralytics 检测权重路径；必须是 detect 任务。首次使用未下载的模型需要联网。
配置和输出目录已存在则生成/准备工具拒绝覆盖，应选择新名字。
最终完整地图直接在 data/street_vehicle_test4_detect/global_clean.pcd，删除点为同目录 global_vehicles.pcd。
图像叠加与统计在 clean_diagnostics/run_*/ 下。保留原图做对照；不自动删除任何旧结果。

测试使用合成几何、模拟模型输出和真实 map 主处理流程；没有在用户 bag、真实检测权重或 ROS2 上验证识别精度和速度。

接口依据：https://docs.ultralytics.com/modes/predict/ （Results.boxes.xyxy/conf/cls）。
