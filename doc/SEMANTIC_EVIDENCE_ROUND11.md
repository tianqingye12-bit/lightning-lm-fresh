# 第十一轮：语义证据约束与可检查的地图净化

本轮基于 `codex/contour-preservation-round10`，继续现有激光惯性建图与离线语义处理路线。没有接入 LVI-SAM，也没有将相机测量加入位姿估计。

## 行为

已有实例分割、遮挡/深度关联、投票和背景否决继续使用。新增策略通过独立配置启用；原配置的判定门限与簇扩展保持兼容。

- 有效分类比例 `(目标票 + 背景票) / 可见次数` 至少 0.6，避免只积累少量目标票、其余大量不确定观测时仍将目标比例视为 100%。可见但处于掩膜边界/缺少深度支撑的观测保留为不确定证据。
- 同一点/体素的正向目标观测，相机中心相对首次正向观测至少移动 0.05 m。只记录可见且通过实例深度检查的正向观测，遮挡或背景观测不能贡献基线。纯旋转和原地重复图像不能满足此条件。
- 基线是“相对首次正向相机中心的最大平移距离”，不是全部视点的最大两两距离，不等于统计独立观测或目标运动检测。停车目标和运动目标仍由配置的语义类别筛选，未新增跨帧实例追踪。阈值是待真实数据验证的初值，慢速短窗口可能使更多目标保留；降低门限需检查投影和残留，不能仅追求删除率。
- 新策略启用时，全局地图模式关闭整簇向无观测区域扩展，所有剔除均需通过证据门限与地面保护；同时省去这一配置下的连通图计算。
- 不确定点保留在清理后的地图中。额外不确定点云是保留点的子集，不能与干净地图相加；默认不生成，避免大地图重复占用磁盘。
- 开启基线记录额外使用约 28 字节/点或体素，关键帧模式仍受活动关键帧数量限制；未启用时不分配基线历史数组。没有增加 YOLO 推理次数，多个活动关键帧仍共享同一张图像的推理结果。

```yaml
semantic_evidence:
  enabled: true
  min_classified_ratio: 0.6
  min_positive_baseline_m: 0.05
  write_uncertain: false
```

## 更新与运行

先保存本地修改；Git 提示冲突时不要强制重置。首次切换：

```bash
cd ~/lightning-lm-fresh
git fetch origin
git switch --track origin/codex/semantic-evidence-round11
source /opt/ros/humble/setup.bash
# 有安装环境则加载
if [ -f install/setup.bash ]; then source install/setup.bash; fi
```

已存在该本地分支时，用以下命令代替 `--track`：

```bash
git switch codex/semantic-evidence-round11
git pull --ff-only origin codex/semantic-evidence-round11
```

本轮只修改 Python 离线处理。运行这些脚本无需重新编译 C++；若准备运行继承自第十轮的新建图程序，且尚未编译对应代码，再用受限并发构建：

```bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
```

从本机已验证的语义配置生成新文件。输入可换成 `config/m2dgr_clean_compact.yaml`，以继承紧凑存储。脚本保留原标定、时间偏差、模式、图像窗口和存储选择；目标文件已存在会拒绝覆盖。

```bash
python3 scripts/configure_semantic_evidence.py \
  --input config/m2dgr_clean.yaml \
  --output config/m2dgr_clean_evidence.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street01_vehicle_test4 \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_evidence.yaml \
  --device cpu --rviz
```

地图名应使用这包数据实际建图保存的目录。离线处理直接读取 bag，无需额外 `ros2 bag play`。`map` 模式要求 `global.pcd`，会沿用原来的 `global_clean.pcd/global_vehicles.pcd` 输出位置；关键帧模式要求版本2 IMU 坐标约定与完整 manifest，原始地图、关键帧不修改，新结果保存在打印的运行目录。

需要额外检查不确定点时，在生成配置命令追加 `--export-uncertain`，并使用另一个输出配置文件名。它可能生成很大的额外 PCD；紧凑配置默认不启用。不要扩大关键帧窗口来强行凑票，扩大窗口会增加运动目标位置变化与内存风险。

## 检查结果

程序打印 `clean_diagnostics/run_*`（map）或 `clean_diagnostics/keyframes_*`（关键帧）目录。

- `overlay_*.jpg`：蓝色掩膜、红色目标票、绿色背景票、黄色不确定、灰色遮挡。增加配置时间偏差、最近关键帧时间差及插值区间长度。图像叠加用于人工检查外参和投影。
- `views.csv`：逐视图投影/可见/遮挡/目标/背景/不确定票、深度图是否可用、耗时，以及位姿关联时间信息。关键帧模式每行是一对图像与活动关键帧，还包含关键帧ID和有符号时间差；同一图像可有多行，不能当作多张独立图像。
- `frames.csv`：关键帧模式每帧的保留判定统计与无效点、地面估计失败标志。
- `summary.json`：汇总 `evidence` 中的 removed、background、protected、uncertain；四项互斥且合计 total。unobserved、blocked_coverage、blocked_baseline 是可重叠的解释统计，不能与前四项相加。protected 包含地面保护，也包含地面估计失败导致的整帧安全保留，不能直接解释为地面点数。
- `global_uncertain.pcd`：仅显式开启时生成，世界坐标、仅有限XYZ。map模式位于运行诊断目录，关键帧模式位于运行结果目录。保留原强度等字段。

map 模式的 evidence 单位是体素；关键帧模式是有限关键帧点，重叠点按各帧重复计数。无效点另记，关键帧原坐标输出中继续保留；拼接世界点云只含有限XYZ。报告记录缓存命中及耗时。

最近关键帧时间差与插值区间不等于图像/雷达真实同步误差。删除数量和比例也不等于分割精度或定位改善。仍需同一地图、相同输入、人工标注检查区域，对比目标残留、路面/墙面误删及定位表现。

## 验证与限制

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
python3 -m unittest discover -s tests -p 'test_*.py'
```

新增回归覆盖有效分类比例、静止相机重复票、实际投影的遮挡/背景不贡献基线、背景否决与地面保护、无观测点、不确定导出、原始字段与文件保留、紧凑存储选项、时间诊断以及全局模式不执行无证据簇扩展。

当前环境完成 Python 合成与文件输出集成回归；缺少真实 PCL 压缩工具的测试跳过。没有访问用户本机 bag/模型，未验证 street_06 效果、实际 CPU/峰值内存或定位精度。本轮独立草稿 PR，尚未并入 master。
