# 第三轮：回环验证、图优化与显示一致性

基于第二轮 `codex/map-detail-round2`，包含前两轮改动。本轮不重置前端 ESKF、也不重建前端 iVox 地图。
更密的地图不等于更高精度；本轮阈值需要在实际 M2DGR 数据中校准。

## 改动

1. 多分辨率 NDT 的最终层必须收敛；结果必须有限且旋转有效，原有 NDT probability 门槛继续使用。
   额外验证源关键帧到历史子图的单向重叠率、重叠内点 RMSE、相对预测初值的平移/旋转修正量。
   验证点最多 2000 个，均匀覆盖源点序；每次最多匹配 3 个最近候选。
   构建历史目标子图时也排除源关键帧及其时间邻域，防止源点云混入目标后自匹配。
2. 固定图中首个关键帧，消除 SE(3) 全局自由度。只在出现通过验证的候选时求解。
   第一遍最多 20 次迭代；计算回环边误差并停用异常边，再从优化前的估计出发求解最多 10 次。
   本回环模块关闭 miao 的增量活动集缓存，按 level=0 重建活动边，确保已停用边不继续参与求解。
   其他定位图优化模块不变。已接受回环继续保留 Cauchy 核，未来回环时可再次检查它们。
3. 求解失败、非有限结果或二次求解后仍存在异常回环边时，恢复优化前的估计、停用本次新边，不发布结果。
   异常边仍保存在图容器中，level=1；不会自动重新启用，计数应看 active_loops 而非总边数。
4. 图位姿整批提交时与快照读取互斥，锁内只复制位姿，不拼点云。
   `global.pcd`、分块地图起始位姿和 `keyframes.txt` 复用同一份导出位姿快照。
   这避免保存过程中恰遇回环时，点云和车辆清理使用的轨迹来自不同优化版本。
5. 每次图优化提交后，将关键帧云引用和冻结位姿交给一个后台显示任务。
   最多保留一个执行中任务和一个最新待处理任务；默认重建完成后至少间隔 2s 再处理下一次。
   全局显示云最多 200000 点，按所有关键帧的累计点序均匀抽样。该预算不改变导出文件。
   Pangolin 渲染线程同时切换显示地图、优化轨迹和 map_from_odom 修正。
   近期扫描、前端轨迹和跟随相机通过同一修正显示；前端状态和连续里程计坐标不变。

后台线程只做 CPU 点云准备，不操作 OpenGL。系统退出时先等待回环线程退出，再停止显示后台线程。
没有开启二维栅格地图时，三维回环显示也可工作；保留原二维地图回环重绘回调。

## 配置

旧 M2DGR YAML 可继续使用，缺省启用本轮回环检查和回环后显示。确认：

```yaml
system:
  with_loop_closing: true
  with_ui: true
  loop_display:
    enabled: true
    min_interval: 2.0
    max_points: 200000

loop_closing:
  # 保留已有 loop_kf_gap、closest_id_th、ndt_score_th 等设置
  validation:
    overlap_distance: 0.5
    min_overlap: 0.3
    max_inlier_rmse: 0.3
    max_translation: 5.0
    max_rotation_deg: 30.0
    max_samples: 2000
    min_inliers: 50
    max_candidates: 3
    max_loop_chi2: 25.0
    refine_iterations: 10
```

以上嵌套内容应合并到已有 section 中，不要在同一 YAML 文件末尾再追加重复的 system/loop_closing 键。
`min_interval` 单位秒，必须大于 0 且不超过 3600。
`max_loop_chi2` 是该实现的信息加权残差门槛，不宣称为经过统计标定的某一置信水平。
NDT probability 和重叠率不是同一个指标，前者沿用现有分数定义。

修正量门槛可拒绝错误匹配，也可能拒绝长距离累计漂移产生的真回环。先检查日志、标定和匹配质量，
再按实际环境调整门槛；不要只为触发回环无条件放宽所有阈值。

## 更新、编译与测试

```bash
cd ~/lightning-lm-fresh
# 指定完整 ref，兼容此前只拉取部分分支的本地配置
git fetch origin \
  refs/heads/codex/loop-robustness-round3:refs/remotes/origin/codex/loop-robustness-round3
git switch --track origin/codex/loop-robustness-round3
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 nice -n 19 colcon build --parallel-workers 1
source install/setup.bash
python3 tests/run_loop_quality.py --sanitize --backend
python3 tests/run_map_detail.py --sanitize --with-pcl
```

`--backend` 依赖 `build/lightning/src/CMakeFiles/lightning.libs.dir/flags.make` 和
`install/lightning/lib/liblightning.libs.so`；独立编译并链接真实的 miao/LoopClosing 集成测试。
如果构建目录不同，调整测试脚本中的路径，不要把缺少依赖误判为算法失败。

本轮集成测试包含：固定首帧、接受一致约束、拒绝强矛盾约束、剔除后的求解结果、
重复求解不重复活动边、旧异常边保持停用、强制求解失败后的回滚和无回调、并发位姿快照一致性。
无 ROS/PCL 的环境只能运行 `python3 tests/run_loop_quality.py --sanitize`，
验证匹配门槛、异常边门槛、显示配置、后台任务合并/限频等待/退出。

## 回放观察

使用第二轮已经生成的配置，不必重新生成：

```bash
./bin/run_slam_online --config ./config/m2dgr_slam_detail.yaml
```

另一个终端按原命令以 0.5 倍速回放 walk_01_ros2。须包含重访历史区域的时段。

日志：

- `[loop_match] ... accepted=0 reason=...`：完成验证后的拒绝原因；空云、点数不足等可能提前退出。
- `[loop_match] ... accepted=1`：通过匹配验证，尚不等于最终图中被接受。
- `[loop_graph] rejected=... active_loops=... second_pass=...`：图优化后的有效约束情况。
- `optimize finished, active loops: ...`：位姿提交完成；active_loops=0 时不能视为有效闭环。
- `[loop_display] prepared ...`、`[loop_display] applied ...`：后台准备和界面实际切换完成。

检查 Pangolin 的紫色优化轨迹、历史全局点云和当前扫描是否衔接；分别保存到新的目录，
比较墙面厚度、回访重影和地图闭合程度。有真值再比较 ATE/RPE。
本轮没有新增 ROS 全局点云话题；上述刷新针对 Pangolin，RViz 不会因此自动增加显示层。

## 验证边界

当前执行环境缺少 ROS2/PCL/Pangolin，完整工程构建、真实后端集成测试和 M2DGR 回放尚未执行。
独立策略测试及第二轮策略/配置回归已通过，不代替真实系统验证，PR 保持草稿。
全局显示采用关键帧采样，近期扫描采用最近一次 map_from_odom 刚体修正，
不表示所有历史里程计扫描均已按各自优化位姿重建。地图仍可能存在未触发回环的漂移或错误回环。
