#!/bin/bash
# M2DGR walk_01 数据集测试脚本

set -e

# 默认参数
FUSION_FILTER=${FUSION_FILTER:-false}
RVIZ=${RVIZ:-0}
RATE=${RATE:-0.5}
MAP_ID=${MAP_ID:-m2dgr_walk01}

# M2DGR 数据集路径
BAG_PATH="/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/walk_01_ros2"

# 配置文件
CONFIG="config/m2dgr_clean.yaml"

echo "=========================================="
echo "M2DGR walk_01 测试"
echo "=========================================="
echo "地图ID: $MAP_ID"
echo "播放速率: ${RATE}x"
echo "RViz: $RVIZ"
echo "配置: $CONFIG"
echo ""

# 检查数据集是否存在
if [ ! -d "$BAG_PATH" ]; then
    echo "错误: 找不到 M2DGR 数据集"
    echo "路径: $BAG_PATH"
    exit 1
fi

# 检查是否已编译
PROJECT_DIR=$(pwd)
INSTALL_DIR="$PROJECT_DIR/install"

if [ ! -f "$INSTALL_DIR/setup.bash" ]; then
    echo "正在编译 Lightning..."
    source /opt/ros/humble/setup.bash
    colcon build --packages-select lightning --symlink-install \
      --build-base "$PROJECT_DIR/build" \
      --install-base "$INSTALL_DIR" \
      --base-paths "$PROJECT_DIR"
fi

# Source 工作空间
source "$INSTALL_DIR/setup.bash"

# 创建输出目录
mkdir -p data/$MAP_ID

# 启动 SLAM 和 bag 播放
echo "=========================================="
echo "启动测试"
echo "=========================================="
echo ""

# 检查是否启动 RViz
if [ "$RVIZ" = "1" ]; then
    echo "启动 RViz..."
    rviz2 &
    RVIZ_PID=$!
    sleep 2
fi

echo "启动 SLAM 节点..."
ros2 launch lightning mapping.launch.py config:=$CONFIG &
SLAM_PID=$!

echo "等待 SLAM 初始化..."
sleep 5

echo ""
echo "开始播放 bag (${RATE}x 速度)..."
echo "bag: $BAG_PATH"
echo ""

ros2 bag play "$BAG_PATH" --rate $RATE

echo ""
echo "=========================================="
echo "Bag 播放完成"
echo "=========================================="
echo ""
echo "保存地图..."
sleep 2
ros2 service call /save_map std_srvs/srv/Trigger

echo ""
echo "地图已保存到: data/$MAP_ID/"
echo ""

# 清理
if [ ! -z "$RVIZ_PID" ]; then
    kill $RVIZ_PID 2>/dev/null || true
fi
kill $SLAM_PID 2>/dev/null || true

echo "=========================================="
echo "测试完成！"
echo "=========================================="
echo ""
echo "输出文件:"
ls -lh data/$MAP_ID/

echo ""
echo "运行车辆去除:"
echo "  ./run_vehicle_removal.sh \\"
echo "    --map data/$MAP_ID \\"
echo "    --bag \"$BAG_PATH\" \\"
echo "    --config config/m2dgr_clean.yaml \\"
echo "    --cpu"
