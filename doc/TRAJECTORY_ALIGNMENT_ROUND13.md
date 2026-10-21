# 第十三轮：逐扫描轨迹、地图坐标一致性与投影诊断

分支 `codex/trajectory-alignment-round13` 基于第十二轮 `codex/mapping-semantic-quality-round12`。本轮新增可选扫描轨迹记录、两种语义处理模式的稠密位姿关联和静态点投影检查工具。用于减少稀疏关键帧插值带来的位姿近似，并为标定、同步、去畸变问题提供检查依据。实际效果需同包对照；这些改动没有将视觉测量加入 LIO 状态估计。

## 保存了什么

开启以下配置后，每次前端成功取出一个同步扫描包时记录一行，包括初始化、IMU 缺口、点数不足、跳过配准、配准拒绝、首次地图建立和正常配准。缓冲区中尚未处理或已经丢弃的扫描不属于记录行；现有输入丢弃统计仍需检查。

```yaml
fasterlio:
  trajectory_recording:
    enabled: true
    max_records: 200000
```

记录结构只存位姿和诊断，不存点云。每条结构约几百字节，记录数组随扫描数增长，有明确数量上限；CSV 磁盘体积随记录数增长，也不是零开销。记录每帧时间检查会额外遍历一次预处理点云，需本机检查耗时。默认关闭，不改变已有前端、预测上限、采样和保存分辨率。

SaveMap 在原有事务目录中生成三个文件：

- `scan_trajectory.csv`：扫描起止、状态时刻、原始 LIO 位姿、地图坐标下的位姿、可用标志、状态原因、校正方式，以及时间诊断。
- `trajectory_anchors.csv`：每个关键帧 ID、时间、原始 LIO 位姿及保存地图使用的位姿。
- `trajectory.yaml`：版本、坐标约定、完整性、记录数量和可用数量。

地图、关键帧和轨迹使用同一份优化位姿快照；再次保存时重新计算校正，不复用上次保存时的坐标。后端在快照之后继续优化，不会改变这次已固定的导出结果。

## 回环后的轨迹如何对齐

对每个关键帧计算左乘校正 `C = T_map_imu × inverse(T_lio_imu)`。扫描位于两个关键帧之间时，对相邻校正的平移做线性插值、旋转做四元数插值，再左乘该扫描的原始 LIO 位姿。因此保留原始逐扫描运动，而不是直接把原始运动替换为两个关键帧之间的直线运动。

关键帧处严格恢复保存地图使用的关键帧位姿。第一、最后一个关键帧之外，在已记录扫描范围内保持最近校正，分别标记 `head_hold/tail_hold`；不在记录时间范围外外推位姿。无回环修正时校正为单位变换。

这是以关键帧为锚点的近似轨迹校正，不是增加了逐帧图优化或重新优化了历史 IMU 状态；相邻校正变化大时需要检查其影响。`head_hold/tail_hold` 也不能证明相应区间没有漂移。

## 视觉处理的关联规则

```yaml
pose_trajectory:
  source: dense
  max_gap_s: 0.2
```

- `dense`：要求完整、匹配当前地图的三份轨迹文件；缺失即报错。第十三轮生成器使用此设置，避免误以为旧地图已经具有逐扫描轨迹。
- `auto`：默认设置。三份文件都不存在时沿用关键帧轨迹；存在时读取并校验稠密轨迹。部分文件、记录不完整、关键帧不匹配或 CSV 截断会报错，不悄悄回退。
- `keyframes`：明确选择旧的关键帧插值，用于同图对照。该模式继续使用原来的 `max_kf_gap`；`max_gap_s` 仅限制稠密扫描间插值。

稠密轨迹只允许使用首次地图建立/配准接受、位姿有限、时钟对齐、无 IMU 缺口、无负值或非有限内部点时间的扫描。其他扫描保留为不可用行；图像时间恰好落在不可用行，或插值两端有一行不可用时跳过该图像。超过 `max_gap_s` 的区间及记录范围外同样跳过。

如果基准配置开启了 `skip_lidar_num`，跳过配准的扫描也会构成不可用行。可用图像过少时先检查此项、失效记录和时间范围；不要仅为了凑图像而放宽缺口门限。需要关闭跳帧做对照时，在独立配置中修改，并降低回放速度以控制负载。

图像时间使用现有 `t_image + time_offset`，先查询 IMU 位姿，再且只应用一次 `T_imu_lidar`。关键帧点云仍按其精确优化位姿变换到世界系；地面估计、manifest、关键帧窗口继续使用关键帧时间，不用稠密扫描 ID 替代关键帧 ID。

两种清理模式均保留第十二轮分割、置信度和证据规则。关键帧清理结果与 mapping 紧凑副本会保留三份轨迹文件，方便复核或再次处理。导航专用导出不要求这些图像关联文件。

## 时间和失败诊断

`scan_trajectory.csv` 中的点时间是 **预处理后内部 `.time` 字段按毫秒转秒的结果**，不是直接声称传感器原始字段单位正确。记录最小/最大/末点偏移、负值及非有限值数量、实际同步器采用的扫描时长、最大点偏移与该时长之差。末点并不一定是最大时间，差值可用于发现同步器回退或点排列相关问题，但诊断不会自动改变同步器规则。

还记录当前包 IMU 数量、首末 IMU 时刻、当时最新 IMU 时刻、传播开始时刻、已有 IMU 缺口标志，以及 `state_timestamp - scan_end`。当前包首样本晚于扫描开始不一定意味着缺数据：传播还会使用上一包末样本和此前状态，需结合这些列检查。此诊断不自动推断原始时间单位、不自动改外参或时间偏移。

语义输出新增：

- `views.csv`：`pose_source`、`pose_status`、`nearest_pose_dt_s`、实际 `pose_bracket_gap_s`、`keyframe_bracket_gap_s` 和校正方式。最近扫描/关键帧时间差均不等于真实传感器同步误差。
- `summary.json`：轨迹来源、文件 SHA256、扫描状态/时间异常汇总、位姿查询成功/失败计数。
- `pose_failures.json`：被拒位姿查询的时间和原因，每种失败原因最多 40 条、总计最多 200 条；完整失败数量看汇总。不是失败图像全集。即使没有可用图像，也尽量留下失败记录供检查。
- `overlay_*.jpg`：原有语义叠加图补充轨迹来源、实际插值间隔与校正方式。

## 本机更新与运行

```bash
cd ~/lightning-lm-fresh
git status --short
git fetch origin
git switch codex/trajectory-alignment-round13
git pull --ff-only origin codex/trajectory-alignment-round13

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ws_livox/install/setup.bash" ]; then source "$HOME/ws_livox/install/setup.bash"; fi
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
```

输入使用本机已经跑通的第十二轮配置；实际文件名不同时替换 `--input`。输出文件独占创建，存在则拒绝覆盖。配置生成器继承已有标定、时间偏移、语义门限、预测上限与紧凑保存设置，不自动调整它们。

```bash
python3 scripts/configure_alignment_round13.py mapping \
  --input config/m2dgr_slam_quality12.yaml \
  --output config/m2dgr_slam_alignment13.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 ./bin/run_slam_online --config config/m2dgr_slam_alignment13.yaml
```

启动应有 `[scan_trajectory] enabled=1`。另开终端播放同一包：

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
ros2 bag play \
  "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --rate 0.5 --topics /velodyne_points /handsfree/imu \
  --qos-profile-overrides-path config/m2dgr_playback_qos.yaml
```

处理完后，保持建图程序运行，在加载安装环境的终端保存：

```bash
cd ~/lightning-lm-fresh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /lightning/save_map lightning/srv/SaveMap \
  '{map_id: street06_alignment13}'
ls -lh data/street06_alignment13/trajectory.yaml \
  data/street06_alignment13/scan_trajectory.csv \
  data/street06_alignment13/trajectory_anchors.csv
```

返回 `response=0` 后可退出建图程序，释放 CPU。检查 `trajectory.yaml` 中 `complete: true`、`recording_faults: 0`。达到记录上限或扫描时间不递增会记录 fault，地图仍可保存，但新语义配置拒绝不完整轨迹；不要手工修改完整标志。

```bash
python3 scripts/configure_alignment_round13.py semantic \
  --input config/m2dgr_clean_quality12.yaml \
  --output config/m2dgr_clean_alignment13.yaml

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street06_alignment13 \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_alignment13.yaml --device cpu --rviz
```

启动应打印 `Image pose trajectory: dense`。处理模式由输入配置继承：map 模式需要 `global.pcd`；紧凑关键帧模式需要 `keyframe_clouds/index.csv`。旧地图只有关键帧时无法恢复过去未保存的真实扫描运动；要验证第十三轮需重新回放建图并保存新地图。

## 静态点投影和时间偏移对照

人工选择固定墙角/杆体等静态结构，在导出地图中读取 XYZ，在对应相机图像上标出相同物理点的像素坐标。不要使用运动中的车辆。准备 JSON 列表，每行一幅图像，示例中的数字须替换为真实对应：

```json
[
  {"timestamp": 1000.0,
   "points_world": [[2.0,1.0,3.0],[2.5,1.0,3.0]],
   "pixels": [[300.0,200.0],[340.0,200.0]],
   "image": "images/frame.png"}
]
```

时间使用图像 header stamp，单位秒；地图坐标单位米，像素坐标使用原图尺寸。图像路径相对 JSON 所在目录，可省略以只输出数值。

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/diagnose_projection.py \
  --map data/street06_alignment13 --config config/m2dgr_clean_alignment13.yaml \
  --samples evaluation/static_matches.json --offset-deltas -0.05 0 0.05 \
  --compare-keyframes --max-overlays 20 --output evaluation/projection13
```

输出目录必须不存在。偏移候选是当前 `time_offset` 的增量，不会写回配置。报告每种来源/偏移的共同对应点 RMSE、中位误差、P95、位姿失效和落在图像外的点数；叠加图绿色为人工像素、红色为投影、黄色为连线。所有候选使用同一组共同有效点，若某候选丢失覆盖，其他候选也不会借此得到不同测试集。必须同时看 `common_correspondences/total_correspondences`，共同点为零时指标为 null。

静止或缺少激励的场景可能无法区分时间偏移；手工对应、外参和地图误差也会影响指标。该工具不自动标定、不证明相机/雷达同步，也不测量车辆识别精度。对照应覆盖直行、转弯、加减速，并保留独立路段验证。

## 验证边界

```bash
python3 tests/run_scan_trajectory.py --sanitize
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 -m unittest discover -s tests -p 'test_*.py'
```

C++ 独立测试检查非直线运动、变化校正、关键帧锚点、世界变换一致性、头尾校正、点时间异常及记录上限；随后直接用 Python 读取其真实导出文件，验证跨语言约定、不可用扫描隔断和非单位外参。Python 新增回归检查实际 bag 消息迭代、稠密/关键帧关联差异、损坏与过期导出、两种语义处理输出、紧凑副本、失败采样上限、配置保护及已知时间偏移的重投影检查。

本轮验证结果：79 项 Python 回归中 78 项通过、1 项真实 PCL 压缩测试因工具缺失跳过，其中新增 10 项。C++ 轨迹测试通过 UBSan，其真实 CSV 导出到 Python 读取的检查也通过；修改的 Python 文件语法检查通过。本环境没有 ROS2/PCL、用户 bag 和实际模型，完整项目编译和 street_06 的精度、速度、内存效果需本机验证。没有据合成测试给出实际精度提升百分比。本轮以独立草稿 PR 发布，尚未合入 master。
