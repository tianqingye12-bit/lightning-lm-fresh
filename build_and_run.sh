#!/bin/bash
# Lightning-LM SLAM 编译和运行脚本

set -e

echo "=========================================="
echo "Lightning-LM SLAM 编译和启动"
echo "=========================================="
echo ""

# 检查是否在 ROS2 工作空间中
if [ ! -f "package.xml" ]; then
    echo "错误: 不在 ROS2 包目录中"
    exit 1
fi

# 获取项目根目录
PROJECT_DIR=$(pwd)
WORKSPACE_DIR=$(dirname "$PROJECT_DIR")

echo "项目目录: $PROJECT_DIR"
echo "工作空间: $WORKSPACE_DIR"
echo ""

# 步骤 1: 编译
echo "=========================================="
echo "步骤 1: 编译 Lightning 包"
echo "=========================================="
cd "$WORKSPACE_DIR"

# 检查是否已 source ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "正在 source ROS2 环境..."
    source /opt/ros/humble/setup.bash
fi

echo "开始编译..."
colcon build --packages-select lightning --symlink-install

if [ $? -ne 0 ]; then
    echo ""
    echo "编译失败！请检查错误信息。"
    exit 1
fi

echo ""
echo "✓ 编译完成"
echo ""

# 步骤 2: Source 工作空间
echo "=========================================="
echo "步骤 2: Source 工作空间"
echo "=========================================="
source "$WORKSPACE_DIR/install/setup.bash"
echo "✓ 工作空间已加载"
echo ""

# 步骤 3: 检查配置文件
echo "=========================================="
echo "步骤 3: 检查配置"
echo "=========================================="

CONFIG_FILE="${1:-config/default_robosense.yaml}"

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
echo "  ros2 service call /save_map std_srvs/srv/Trigger"
echo ""
echo "按 Ctrl+C 停止"
echo ""

cd "$PROJECT_DIR"
ros2 launch lightning mapping.launch.py config:="$CONFIG_FILE"
