# 第十二轮：配准约束均衡、分割置信度与固定样本评测

本轮以 `codex/semantic-evidence-round11` 为基线，分支为 `codex/mapping-semantic-quality-round12`。包含前十一轮的代码；新增策略通过单独配置启用，旧配置默认关闭。本轮仍是激光惯性建图加离线视觉语义处理，相机没有进入位姿估计。

## 建图端

通过 PCA、局部支撑及残差检查的约束，按 **LiDAR 坐标系的 8 个方位 × 3 个主法向** 分桶。每个非空桶尽量均分数量预算，稀少桶保留后将剩余预算交给其他桶；桶内均匀取样，不只取前几个点。法向正负不影响分桶，法向先从世界坐标变换到 LiDAR 坐标。该方法旨在减少大面积地面或单面墙占用过多配准约束，不能生成场景中不存在的约束。

点面和可选点到点 ICP 分别最多 3000 个对应点，开启两项时合计最多 6000 个。点 ICP 的平面对应使用相同分桶，非平面对应另按 8 个方位分桶，保留边缘点的参与资格。点 ICP 仍需通过原来的距离检查。未超过预算时全部保留；近邻搜索仍处理整帧，不能把预算减少等同于整帧计算量按比例下降。关键帧保留、导出分辨率、局部补点策略和 IMU 预测上限沿用输入配置。

约束没有被重复或放大权重。削减数量会改变激光观测相对 IMU 的总信息量，过低预算可能降低精度；3000 是需要实测的初值。新设置不直接修复标定、点时间单位、同步或去畸变错误，也不保证所有场景都优于原配置。

```yaml
fasterlio:
  registration_quality:
    balance: true
    max_constraints: 3000
    diagnostics: true
    rotation_scale_m: 5.0
    relative_eigenvalue: 0.001
```

诊断使用 Huber 加权的 **点面观测 6×6 信息矩阵**，除以权重和，并用 `[平移, L×转角]` 尺度归一化。它不包含 IMU 先验和点 ICP，也不修改状态、协方差或更新接受条件。日志每 `feature_quality.stats_interval` 帧报告最后一次观测评估：

- `plane=使用数/候选数`、`point_icp=通过距离检查数/候选数`。
- `plane_info_valid`、`plane_rank` 和最小/最大特征值比 `min_max`。有效且 rank 小于 6 表示在当前门限下存在点面弱约束方向，不等于完整融合系统失效。
- `weak_t_world_Lrot_body`：最弱方向，前三项是世界系平移，后三项是机体系转角乘 L；方向符号任意，可能同时包含平移和旋转。它不是定位误差或标准差。

## 语义端

YOLO 推理后保留每个实例的类别和置信度。可按类别配置门限；默认实例门限 0.40。图像原始掩膜向内收缩 1 像素作为核心，只有核心中通过独立深度带检查且可见的点可以贡献目标票。原掩膜边缘及膨胀区域只阻止背景票，不再因膨胀直接产生删除证据。小目标核心被收缩掉、低置信度实例和缺乏深度支撑的区域暂不作确定判断。

```yaml
segmentation_quality:
  enabled: true
  min_instance_confidence: 0.40
  class_confidence: {}  # 可按当前模型类别 ID 设置，如 {2: 0.45, 7: 0.50}
  core_erode_px: 1
  min_mean_confidence: 0.55
```

低于实例门限的已有检测仍遮住对应背景投票区域，避免误把不确定检测当成已知背景。低于 `yolo_conf` 的实例没有进入结果，无法被本策略保护，因此实例门限不得低于 `yolo_conf`。不自动调整模型类别 ID、权重或推理分辨率。

同一视图中多个重叠实例只计一票，取合格实例的最大置信度。删除同时要求原有最少目标视图数、分类覆盖率、正向相机移动基线、背景否决、地面保护，以及：

- 目标票的平均实例置信度至少 0.55。
- `目标置信度之和 / (目标置信度之和 + 背景票数)` 达到原来的 `vehicle_ratio`。

这些分数不是经过标定的概率；没有新增模型训练、目标运动估计或跨帧实例跟踪。保守门限可能减少误删，也可能保留更多远处、细小或夜间目标。需要同时比较 precision 和 recall，不能只看删除数量。

新增置信度累计数组常驻约 4 字节/点或体素，未启用不分配；判定时还有临时数组。关键帧模式继续限制活动帧数、共享同图推理，逐实例处理核心掩膜，避免额外复制全部合格掩膜。缓存使用 schema 3，保存类别和置信度；旧缓存不会被错误当成置信度为 1，首次需重新推理。两项新策略均不要求增加 YOLO 模型大小。

`views.csv` 新增 `accepted_objects`（类别/置信度门限合格实例数）和 `positive_confidence_sum`；前者不保证存在足够核心深度点。`frames.csv/summary.json` 新增可重叠解释项 `blocked_confidence`，不与 removed/background/protected/uncertain 相加。保留点、不确定点及世界/IMU 输出坐标约定沿用第十一轮。启用新语义策略时，全局模式禁止向没有通过门限的簇扩展。

## 更新与运行

先查看本地修改。下面的分支切换不丢弃修改，若 Git 报冲突，先备份或提交相关修改，不强制重置。

```bash
cd ~/lightning-lm-fresh
git status --short
git fetch origin
git switch codex/mapping-semantic-quality-round12
git pull --ff-only origin codex/mapping-semantic-quality-round12

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ws_livox/install/setup.bash" ]; then source "$HOME/ws_livox/install/setup.bash"; fi
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
```

Git 默认会从同名远程分支创建本地跟踪分支。本轮修改了 C++ 建图部分，建图前需要成功编译并加载安装环境；仅运行 Python 语义脚本不需要因本轮重新编译。

用本机当前已经验证的建图配置作为输入。示例沿用之前的名称，若目前使用的是 `m2dgr_slam_contour.yaml` 或其他文件，请替换输入路径；生成器保留其采样、外参、时间和保存设置。新文件存在会拒绝覆盖。

```bash
python3 scripts/configure_quality_round12.py mapping \
  --input config/m2dgr_slam_detail.yaml \
  --output config/m2dgr_slam_quality12.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 ./bin/run_slam_online --config config/m2dgr_slam_quality12.yaml
```

另开终端，从同一 bag 开头播放，建图只需要这两个话题：

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
ros2 bag play \
  "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --rate 0.5 --topics /velodyne_points /handsfree/imu \
  --qos-profile-overrides-path config/m2dgr_playback_qos.yaml
```

建完后在加载 `install/setup.bash` 的终端保存为新名字，避免覆盖基准地图：

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap \
  '{map_id: street06_quality12}'
```

语义配置也从当前已验证配置生成。下面输入 `m2dgr_clean.yaml` 沿用全局地图模式，要求保存有 `global.pcd`。如果建图使用紧凑保存，改用已验证的关键帧配置（如 `m2dgr_clean_compact.yaml`）作输入，要求地图有 `keyframe_clouds/index.csv` 和版本 2 坐标约定；不必为了语义处理重新导出巨大总地图。

```bash
python3 scripts/configure_quality_round12.py semantic \
  --input config/m2dgr_clean.yaml \
  --output config/m2dgr_clean_quality12.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street06_quality12 \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_quality12.yaml --device cpu --rviz
```

离线处理直接读 bag，无需同时播放 bag。原始地图和关键帧不改写。map 模式仍写地图目录的 `global_clean.pcd/global_vehicles.pcd`，反复运行会替换这两个派生文件，A/B 前分别留存；关键帧模式每次创建新的 `clean_diagnostics/keyframes_*` 结果目录，紧凑配置仍可只保存压缩后的关键帧。

## 固定样本验证

先只比较建图配置，使用同一包、相同回放速度和各自新启动的 SLAM 进程，保存不同 map_id。不要同时改变体素尺寸、回环门限和显示比例。原配置作为 A，第十二轮配置作为 B；也可给 mapping 配置生成器加 `--diagnostics-only` 得到只记录诊断、不做均衡的 A。若约束本来低于预算，均衡不会改变该帧；日志可以确认。

### 参考平面误差

从两张地图裁出同一组固定墙面/地面区域并导出 **Nx3 数值数组 `.npy`，单位米**，不能包含强度列。用一次确定的刚体对齐变换进入共同参考系，不能为每个 ROI 单独拟合或缩放。`planes.json` 是列表，例如：

```json
[
  {"name":"wall", "normal":[1,0,0], "offset":-10,
   "min":[9.5,-2,0.2], "max":[10.5,2,2.5], "coverage_cell_m":0.1}
]
```

示例表示参考墙为 `x=10`；**必须替换成实际测量/可信参考的平面和 ROI，不是 street_06 的已知坐标**。两轮共用同一文件，不能分别对输出地图拟合平面后声称绝对误差改善。

```bash
python3 scripts/evaluate_quality.py geometry \
  --points evaluation/map_A_roi.npy --planes evaluation/planes.json --output evaluation/geometry_A.json
python3 scripts/evaluate_quality.py geometry \
  --points evaluation/map_B_roi.npy --planes evaluation/planes.json --output evaluation/geometry_B.json
```

报告固定平面 RMSE、绝对距离 P95、有符号均值偏差、P95-P05 厚度、点数与固定平面网格占用数。厚度小但均值偏差大仍是位置错误；点数/覆盖急剧减少时不能仅凭厚度变小判定改善。这些是局部几何指标，不代替轨迹 ATE/RPE 或导航定位误差；没有可信参考时只能报告相对厚度和覆盖变化。

### 实例识别精度

固定一组白天/夜间、近处/远处、遮挡及无目标图像；用原图像尺寸人工标注所有待评估类别的实例，保持模型的类别 ID。每幅图真值 `.npz` 含 `masks`（N×H×W 布尔数组）和 `classes`（N 个整数）；无目标图也要提供形状 `(0,H,W)` 的空数组。不能只标容易检测的车辆。`samples.json` 列出相对该 JSON 所在目录的路径：

```json
[
  {"image":"images/frame_001.png", "truth":"labels/frame_001.npz"},
  {"image":"images/frame_002.png", "truth":"labels/frame_002.npz"}
]
```

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/evaluate_quality.py segmentation \
  --manifest evaluation/samples.json --config config/m2dgr_clean_quality12.yaml \
  --device cpu --iou 0.5 --output evaluation/recognition_B.json
```

用基准配置和另一个输出路径运行 A。报告按类别、按置信度顺序做一对一 mask IoU 匹配的 TP/FP/FN、precision/recall/F1，以及目标像素并集 IoU。错类别不会算作正确实例，重复检测计 FP，无目标图上的检测同样计 FP；分母为零报告 null。不是 mAP，不评估 3D 删除质量，也不对用于投票的收缩核心单独打分。可与原配置在相同模型输出下比较阈值筛选效果；后续更换模型/训练时也沿用同一份留出的评测集。

每行还可指定 `prediction` 路径，文件含原始 `masks/classes/scores`，程序直接读固定预测，不加载模型。预测必须来自同一图像，保留不低于两轮最小 `yolo_conf` 的提议和相同 max_det/尺寸设置，不能先删掉低分实例。报告记录图像、真值、固定预测或模型权重的 SHA256 和配置，避免无意换样本。3D 结果另外检查固定路沿/墙面区域的误删和目标残留，删除计数不是精度。

## 回归与发布边界

```bash
python3 tests/run_registration_quality.py --sanitize
python3 tests/run_feature_quality.py --sanitize
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 -m unittest discover -s tests -p 'test_*.py'
```

独立 C++ 测试覆盖预算、稀少方向保留、桶内取样、法向符号/世界旋转一致性、单平面秩 3、双平面秩 5、三正交平面秩 6、重复观测归一化和非法输入。Python 新增 11 项，含两种模式真实投影/遮挡/投票/PCD 导出的集成测试、置信度缓存与配置保护、固定参考误差及实例精度计数。

本轮验证：69 项 Python 回归中 68 项通过，1 项真实 PCL 压缩测试因缺少工具跳过；新增配准质量和原有 PCA 特征质量两组独立 C++ 测试均通过 UBSan 检查。当前环境没有 ROS2/PCL 或用户真实 bag/模型。完整项目编译、实际运行时延/峰值内存及建图和识别精度提升需本机 A/B 验证，不能据测试通过给出提升百分比。本轮以草稿 PR 发布，尚未合入 master。
