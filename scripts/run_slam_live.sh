#!/bin/bash
# Lightning-LM SLAM 实时建图脚本（带 RViz 可视化）

set -e

# 默认参数
CONFIG="${CONFIG:-config/default_robosense.yaml}"
MAP_ID="${MAP_ID:-default_map}"
RVIZ="${RVIZ:-1}"
BAG_PATH="${BAG_PATH:-}"
BAG_RATE="${BAG_RATE:-1.0}"

echo "=========================================="
echo "Lightning-LM SLAM 实时建图"
echo "=========================================="
echo "配置文件: $CONFIG"
echo "地图ID: $MAP_ID"
echo "RViz可视化: $RVIZ"
if [ -n "$BAG_PATH" ]; then
    echo "Bag路径: $BAG_PATH"
    echo "播放速率: ${BAG_RATE}x"
fi
echo ""

# 检查配置文件
if [ ! -f "$CONFIG" ]; then
    echo "错误: 配置文件不存在: $CONFIG"
    exit 1
fi

# 检查是否已编译
PROJECT_DIR=$(pwd)
INSTALL_DIR="$PROJECT_DIR/install"

if [ ! -f "$INSTALL_DIR/setup.bash" ]; then
    echo "正在编译 Lightning 包..."
    source /opt/ros/humble/setup.bash
    colcon build --packages-select lightning --symlink-install \
      --build-base "$PROJECT_DIR/build" \
      --install-base "$INSTALL_DIR" \
      --base-paths "$PROJECT_DIR"

    if [ $? -ne 0 ]; then
        echo "编译失败！"
        exit 1
    fi
    echo "✓ 编译完成"
    echo ""
fi

# Source 工作空间
source "$INSTALL_DIR/setup.bash"

# 创建地图输出目录
mkdir -p "data/$MAP_ID"

echo "=========================================="
echo "启动 SLAM"
echo "=========================================="
echo ""

# 启动 RViz（如果需要）
if [ "$RVIZ" = "1" ]; then
    echo "启动 RViz 可视化..."
    rviz2 &
    RVIZ_PID=$!
    echo "✓ RViz PID: $RVIZ_PID"
    sleep 2
fi

# 修改配置文件中的 map_id（临时）
TEMP_CONFIG="/tmp/lightning_config_$$.yaml"
sed "s|map_path:.*|map_path: data/$MAP_ID|g" "$CONFIG" > "$TEMP_CONFIG"

echo "启动 SLAM 节点..."
ros2 launch lightning mapping.launch.py config:="$TEMP_CONFIG" &
SLAM_PID=$!
echo "✓ SLAM PID: $SLAM_PID"

# 等待 SLAM 初始化
echo "等待 SLAM 初始化..."
sleep 5

# 如果指定了 bag，自动播放
if [ -n "$BAG_PATH" ]; then
    if [ ! -e "$BAG_PATH" ]; then
        echo "错误: Bag 文件不存在: $BAG_PATH"
        kill $SLAM_PID 2>/dev/null || true
        [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null || true
        rm -f "$TEMP_CONFIG"
        exit 1
    fi

    echo ""
    echo "=========================================="
    echo "播放 Bag (${BAG_RATE}x 速度)"
    echo "=========================================="
    echo "Bag: $BAG_PATH"
    echo ""

    ros2 bag play "$BAG_PATH" --rate $BAG_RATE

    echo ""
    echo "=========================================="
    echo "Bag 播放完成"
    echo "=========================================="
    echo ""

    # 自动保存地图
    echo "保存地图..."
    sleep 2
    ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: \"$MAP_ID\"}"

    echo ""
    echo "✓ 地图已保存到: data/$MAP_ID/"
    ls -lh "data/$MAP_ID/" 2>/dev/null || true

    # 清理
    echo ""
    echo "关闭 SLAM..."
    kill $SLAM_PID 2>/dev/null || true
    [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null || true
    rm -f "$TEMP_CONFIG"

    echo ""
    echo "=========================================="
    echo "建图完成！"
    echo "=========================================="
else
    # 实时模式（等待用户手动停止）
    echo ""
    echo "=========================================="
    echo "SLAM 运行中（实时模式）"
    echo "=========================================="
    echo ""
    echo "RViz 中可以看到："
    echo "  - /cloud_registered - 配准后的点云"
    echo "  - /Odometry - 里程计轨迹"
    echo "  - /map_cloud - 全局地图"
    echo ""
    echo "保存地图："
    echo "  ros2 service call /lightning/save_map lightning/srv/SaveMap \"{map_id: \\\"$MAP_ID\\\"}\""
    echo ""
    echo "按 Ctrl+C 停止 SLAM"
    echo ""

    # 设置清理函数
    cleanup() {
        echo ""
        echo "正在停止..."
        kill $SLAM_PID 2>/dev/null || true
        [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null || true
        rm -f "$TEMP_CONFIG"
        echo "已停止"
        exit 0
    }

    trap cleanup INT TERM

    # 等待 SLAM 进程
    wait $SLAM_PID

    cleanup
fi
