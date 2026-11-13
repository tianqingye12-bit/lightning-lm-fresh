#!/bin/bash
# Lightning-LM SLAM 编译和运行脚本

set -e

echo "=========================================="
echo "Lightning-LM SLAM 编译和启动"
echo "=========================================="
echo ""

# 检查是否在 ROS2 包目录中
if [ ! -f "package.xml" ]; then
    echo "错误: 不在 ROS2 包目录中（需要 package.xml）"
    exit 1
fi

PROJECT_DIR=$(pwd)
INSTALL_DIR="$PROJECT_DIR/install"
CONFIG_FILE="${1:-config/default_robosense.yaml}"

echo "项目目录: $PROJECT_DIR"
echo ""

# 步骤 1: 编译（在本目录内，不要用父目录当 workspace）
echo "=========================================="
echo "步骤 1: 编译 Lightning 包"
echo "=========================================="

if [ -z "$ROS_DISTRO" ]; then
    echo "正在 source ROS2 环境..."
    source /opt/ros/humble/setup.bash
fi

echo "开始编译..."
colcon build --packages-select lightning --symlink-install \
  --build-base "$PROJECT_DIR/build" \
  --install-base "$INSTALL_DIR" \
  --base-paths "$PROJECT_DIR"

echo ""
echo "✓ 编译完成"
echo ""

# 步骤 2: Source 本项目 install
echo "=========================================="
echo "步骤 2: Source 工作空间"
echo "=========================================="
# 重新 source ROS，避免被其它 workspace 覆盖
source /opt/ros/humble/setup.bash
source "$INSTALL_DIR/setup.bash"
echo "✓ 已加载: $INSTALL_DIR"
echo ""

# 步骤 3: 检查配置文件
echo "=========================================="
echo "步骤 3: 检查配置"
echo "=========================================="

if [ ! -f "$PROJECT_DIR/$CONFIG_FILE" ]; then
    echo "错误: 配置文件不存在: $CONFIG_FILE"
    echo ""
    echo "可用的配置文件:"
    ls -1 "$PROJECT_DIR/config/"*.yaml 2>/dev/null || echo "  (无)"
    exit 1
fi

echo "使用配置: $CONFIG_FILE"
echo ""

# 步骤 4: 启动 SLAM
echo "=========================================="
echo "步骤 4: 启动 SLAM"
echo "=========================================="
echo ""
echo "启动命令:"
echo "  ros2 launch lightning mapping.launch.py config:=$CONFIG_FILE"
echo ""
echo "保存地图:"
echo "  ros2 service call /lightning/save_map lightning/srv/SaveMap \"{map_id: new_map}\""
echo ""
echo "按 Ctrl+C 停止"
echo ""

cd "$PROJECT_DIR"
ros2 launch lightning mapping.launch.py config:="$CONFIG_FILE"
