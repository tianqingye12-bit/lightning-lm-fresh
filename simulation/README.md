# 导航仿真平台（ROS2 Humble）

第一版检验 lightning-lm 输出的地图是否适合 **二维定位、规划和避障**。
采用固定场景、二维激光、带比例误差的轮式里程计、AMCL、Nav2 和 RViz，不需要 Gazebo。
机器人和车辆用 RViz 几何体显示，车辆可以按固定轨迹运动。

**当前版本不运行 lightning 的三维 LIO 定位，不生成相机图像，也不测试 YOLO 检出率。**
演示中的 raw/clean 地图是手工构造的集成测试样例，不能当作去车算法的实验结果。
当前提交仅通过离线几何/文件测试；ROS2/Nav2 的实际运行仍需在 Humble 主机验证。

## 1. 安装与构建

适用 Ubuntu 22.04 / ROS2 Humble。仿真包单独构建，避免重复编译 lightning 的 C++、PCL。
仿真源码在主项目包里面，colcon 默认递归发现可能跳过子包，因此必须使用下面的 --base-paths。

建议保留现有工作目录，在独立工作树检出本分支：

~~~bash
cd ~/lightning-lm-fresh
git fetch origin
git worktree add ~/lightning-nav-platform origin/codex/nav2-simulation-platform

source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-rviz2 python3-colcon-common-extensions python3-numpy python3-yaml

mkdir -p ~/lightning_nav_sim_ws
cd ~/lightning_nav_sim_ws
MAKEFLAGS=-j2 nice -n 19 colcon build \
  --base-paths ~/lightning-nav-platform/simulation \
  --packages-select lightning_nav_sim \
  --parallel-workers 1 --symlink-install
source install/setup.bash
~~~

如果源码在其他工作树，替换 --base-paths 中的路径。
本包是 Python 包，不需要重新构建主项目。
不要同时运行数据包回放、lightning 建图、其他 /clock 或 /cmd_vel 发布节点。
可在每个仿真终端使用相同的 export ROS_DOMAIN_ID=42 隔离已有 ROS 节点。

## 2. 生成演示场景

~~~bash
source /opt/ros/humble/setup.bash
source ~/lightning_nav_sim_ws/install/setup.bash

ros2 run lightning_nav_sim nav_sim_maps demo \
  --output "$HOME/lightning_nav_sim_ws/scenes/street_demo"
~~~

生成固定世界 world.yaml、原始候选地图 raw.yaml、清理候选地图 clean.yaml 和任务 scenario.yaml。
原始候选地图中有模拟的停车残影，固定世界没有该残影；运行时另有一辆往复运动的车辆。
这些文件用于确认整套流程能运行，不代表真实数据集。
输出目录必须是新目录，防止覆盖已有数据。

## 3. 启动清理地图导航

终端一：

~~~bash
source /opt/ros/humble/setup.bash
source ~/lightning_nav_sim_ws/install/setup.bash

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 ros2 launch lightning_nav_sim navigation.launch.py \
  scenario:="$HOME/lightning_nav_sim_ws/scenes/street_demo/scenario.yaml" \
  map:="$HOME/lightning_nav_sim_ws/scenes/street_demo/clean.yaml" \
  rate:=0.5
~~~

RViz 中，绿色机器人与橙色车辆表示仿真真值，TF 表示定位估计，红点是实时激光，
蓝线是 Nav2 全局路径。可以勾选 Truth world 对照候选地图。
启动后机器人保持暂停，传感器和时钟正常工作，等待 Nav2 初始化。
AMCL 的初始位姿从场景的 start 读取，无需重复点击 2D Pose Estimate。

终端二，运行固定目标序列并保存指标：

~~~bash
source /opt/ros/humble/setup.bash
source ~/lightning_nav_sim_ws/install/setup.bash

ros2 run lightning_nav_sim nav_sim_trial \
  --scenario "$HOME/lightning_nav_sim_ws/scenes/street_demo/scenario.yaml" \
  --output "$HOME/lightning_nav_sim_ws/results/clean_001"
~~~

程序等待 Nav2 激活后，启动仿真并依次发送目标。目标超时默认为 180 秒仿真时间，
另有 900 秒的单目标墙钟上限，避免时钟停止后一直等待。
未到达、真实终点偏差过大、碰撞接触或超时会判为失败；失败时停止后续目标。
结果直接保存在：

~~~text
~/lightning_nav_sim_ws/results/clean_001/summary.json
~/lightning_nav_sim_ws/results/clean_001/samples.csv
~~~

报告包含任务成功率、实际行驶距离、耗时、碰撞接触次数、定位平移/角度 RMSE
及成功配对的定位样本比例。RMSE 使用相同时间戳的 TF 估计与仿真真值。
低定位覆盖率的 RMSE 不能代表全程质量。
这是一次任务的结果，不能用一次运行声称统计上的成功率提升。
中途异常会保留 INCOMPLETE 标记，不能拿这种结果做对照。

想手动点目标时，不运行 trial，而是在终端二执行：

~~~bash
ros2 service call /simulation/start std_srvs/srv/Trigger "{}"
~~~

随后在 RViz 选择 Nav2 Goal。停止运动：

~~~bash
ros2 service call /simulation/stop std_srvs/srv/Trigger "{}"
~~~

每次新任务都应关闭并重新启动终端一。停止后不能原地重启同一次场景。
手动控制不生成固定任务评估报告。

## 4. 对照原始地图

先 Ctrl+C 关闭终端一；重新执行第 3 步启动命令，把 clean.yaml 改成 raw.yaml。
其余场景、起点、目标、速率与参数保持一致。
再运行：

~~~bash
ros2 run lightning_nav_sim nav_sim_trial \
  --scenario "$HOME/lightning_nav_sim_ws/scenes/street_demo/scenario.yaml" \
  --output "$HOME/lightning_nav_sim_ws/results/raw_001"

ros2 run lightning_nav_sim nav_sim_compare \
  --first "$HOME/lightning_nav_sim_ws/results/raw_001" \
  --second "$HOME/lightning_nav_sim_ws/results/clean_001"
~~~

比较器检查固定世界、任务、导航参数和成功标准是否一致。
**激光和碰撞始终读取 scenario.yaml 指定的 world，不会随 raw/clean 候选地图变化。**
移动车辆从任务启动时刻计时；激光随机数种子在任务启动时重置。
AMCL 与异步规划仍可能存在随机性和调度差异，需要多次测试；失败得早导致的短路径没有优势。

## 5. 接入实际保存的 PCD

已有正确的 Nav2 map.yaml + PGM 时，可以直接用作候选地图。
scripts/build_static_navigation.py 支持从保留关键帧的清理结果生成地图；
只有 global_clean.pcd 的结果，可以用本包的投影工具。

下面是**参数示例**，必须根据真实点云确认地面高度与局部范围后修改，
不能直接假设你的地图地面高度为 0：

~~~bash
ros2 run lightning_nav_sim nav_sim_maps project \
  --pcd "$HOME/lightning-lm-fresh/data/street_vehicle_test4/global.pcd" \
  --output "$HOME/lightning_nav_sim_ws/maps/street_raw" \
  --bounds -10 -10 40 40 \
  --ground-z 0.0 \
  --resolution 0.10 \
  --obstacle-min 0.20 --obstacle-max 1.80
~~~

清理后的 global_clean.pcd 用完全相同的范围、地面高度、分辨率、障碍高度和地面容差，
导出到另一个新目录。输出是 map.yaml、map.pgm 和 projection.json。
原始 PCD 必须是完整原图，而不是上一轮已经清理过的结果。
工具只读源文件，流式读取 ASCII / 未压缩 binary PCD；
压缩 PCD 会明确报错，需要先另外导出为未压缩副本。
栅格上限为 800 万格；建议先测试 30～50 米的局部区域。

投影只把观测到的地面格设为空闲，障碍点优先，未观测格保留未知。
移除车辆后没有留下地面观测的区域仍可能不可通行，应在地图生成阶段解决。
固定高度切片只适用于地面接近平面的局部区域，坡道和多层空间应使用保留关键帧的 G2P5 导出。

实际地图还需要一份**坐标对齐的固定参考环境**，不能直接套用演示街区。
在自己的 scenario.yaml 中设置 world 指向固定参考地图，并填写真实的 start 和 goals：

~~~yaml
world: /absolute/path/to/reference_world/map.yaml
start: [2.0, 3.0, 0.0]
goals:
  - [8.0, 3.0, 0.0]
seed: 42
~~~

上述数字仅演示字段格式。原始/清理两组都使用同一份 scenario。
参考环境应来自人工核验、独立测量或可信的场景建模。
如果仅把本算法导出的地图当作参考世界，结果只能说明该重建模型中的运行情况，
不能证明实际环境中的地图精度或车辆识别准确率。

## 6. 资源与排错

- 默认物理更新 20 Hz，二维激光 5 Hz / 180 线，控制器 10 Hz，速率 0.5。
  可加 rviz:=false 减少显示开销，不要同时运行多个 Nav2 实例。
- Start footprint 报错：起点周围存在障碍或未知，核对地图原点、朝向和机器人半径。
- RViz 有地图但机器人不动：确认 trial 是否已启动，或是否已调用 /simulation/start。
- Nav2/AMCL not ready：检查终端一日志及以下命令。

~~~bash
ros2 topic hz /clock
ros2 topic hz /scan
ros2 lifecycle get /bt_navigator
~~~

- TF 必须只有 AMCL 的 map->odom 和模拟里程计的 odom->base_link；
  base_link->base_laser 是静态变换。不要同时启动当前直接发布 map->base_link 的 lightning 定位。
- 找不到包：每个终端都先 source Humble，再 source 独立仿真工作区。
- 输出目录已存在：换一个 clean_002 等新名字，保留旧结果作对照。

## 7. 验证与后续范围

开发环境运行：

~~~bash
cd ~/lightning-lm-fresh
python3 -m unittest discover -s tests -p test_navigation_sim.py -v
~~~

离线测试覆盖地图行方向、旋转原点、未知格、PCD 流式投影、碰撞扫掠、车辆激光交点、
可重复的场景/传感器模型和对照条件检查。它们不代替 ROS2 主机上的 launch/action/TF 联调。

后续真正验证建图和视觉语义，需要有三维场景真值的仿真器、带逐点时间的 PointCloud2、
IMU、相机与标定/同步、lightning 定位的标准 map->odom->base_link 适配，
并设计车辆识别和静态结构保留的独立标注指标。当前平台仅完成导航地图评估这一层。

参考接口：
- [Nav2 Humble bringup](https://github.com/ros-navigation/navigation2/tree/humble/nav2_bringup)
- [Nav2 TF 约定](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/)
