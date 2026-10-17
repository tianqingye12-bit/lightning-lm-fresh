#!/bin/bash
# 一键启动车辆去除算法
# 用法: ./run_vehicle_removal.sh [选项]

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
MAP_DIR=""
BAG_PATH=""
CONFIG="config/garage_clean.yaml"
DEVICE="0"
RVIZ=false
VISUALIZE_ONLY=false
SKIP_PROCESSING=false

# 帮助信息
show_help() {
    cat << EOF
${GREEN}Lightning-LM 车辆去除系统 - 一键启动脚本${NC}

${YELLOW}用法:${NC}
  $0 --map <地图目录> --bag <bag路径> [选项]

${YELLOW}必需参数:${NC}
  --map <dir>         SLAM 建图输出目录 (包含 global.pcd 和 keyframes.txt)
  --bag <path>        建图时录制的 ROS bag (ROS1 .bag 或 ROS2 目录)

${YELLOW}可选参数:${NC}
  --config <yaml>     配置文件 (默认: config/garage_clean.yaml)
  --device <id>       YOLO 设备: 0/1/2... (GPU) 或 cpu (默认: 0)
  --rviz              启动实时可视化 (需要 ROS2 和 RViz2)
  --cpu               使用 CPU 运行 (等同于 --device cpu)
  --visualize-only    只可视化已有结果，不重新处理
  --help              显示此帮助信息

${YELLOW}示例:${NC}
  # 基本用法（GPU）
  $0 --map data/my_garage --bag ~/bags/garage.bag

  # CPU 模式
  $0 --map data/my_garage --bag ~/bags/garage.bag --cpu

  # 实时可视化模式
  $0 --map data/my_garage --bag ~/bags/garage.bag --rviz

  # 只查看已有结果
  $0 --map data/my_garage --visualize-only

  # 使用自定义配置
  $0 --map data/parking --bag ~/data.bag --config config/my_config.yaml

${YELLOW}工作流程:${NC}
  1. 检查依赖和输入文件
  2. 验证配置文件
  3. 运行车辆去除算法
  4. 生成结果文件
  5. (可选) 启动 RViz 可视化

${YELLOW}输出文件:${NC}
  <map_dir>/global_clean.pcd     - 去车后的干净地图
  <map_dir>/global_vehicles.pcd  - 被删除的车辆点云

${YELLOW}相关文档:${NC}
  doc/VEHICLE_REMOVAL_GUIDE.md   - 完整操作指南
  README_VEHICLE_REMOVAL.md      - 快速开始

EOF
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --map)
            MAP_DIR="$2"
            shift 2
            ;;
        --bag)
            BAG_PATH="$2"
            shift 2
            ;;
        --config)
            CONFIG="$2"
            shift 2
            ;;
        --device)
            DEVICE="$2"
            shift 2
            ;;
        --cpu)
            DEVICE="cpu"
            shift
            ;;
        --rviz)
            RVIZ=true
            shift
            ;;
        --visualize-only)
            VISUALIZE_ONLY=true
            SKIP_PROCESSING=true
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}错误: 未知参数 '$1'${NC}"
            echo "使用 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# 打印横幅
echo -e "${BLUE}"
cat << "EOF"
╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║     Lightning-LM 车辆去除系统 - 一键启动                  ║
║                                                           ║
║     地下车库地图车辆点云离线去除                           ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝
EOF
echo -e "${NC}"

# 只可视化模式
if [ "$VISUALIZE_ONLY" = true ]; then
    if [ -z "$MAP_DIR" ]; then
        echo -e "${RED}错误: 可视化模式需要指定 --map 参数${NC}"
        exit 1
    fi

    if [ ! -f "$MAP_DIR/global.pcd" ]; then
        echo -e "${RED}错误: 找不到 $MAP_DIR/global.pcd${NC}"
        exit 1
    fi

    echo -e "${GREEN}═══ 启动可视化 ═══${NC}"
    echo -e "地图目录: ${YELLOW}$MAP_DIR${NC}"
    echo ""

    # 启动 RViz publisher
    echo -e "${BLUE}启动点云发布器...${NC}"
    python3 scripts/clean_map_rviz.py --map "$MAP_DIR" &
    PUB_PID=$!

    sleep 3

    # 启动 RViz
    if command -v rviz2 &> /dev/null; then
        echo -e "${BLUE}启动 RViz2...${NC}"
        rviz2 -d config/clean_map.rviz &
        RVIZ_PID=$!

        echo ""
        echo -e "${GREEN}可视化已启动！${NC}"
        echo -e "  - 点云发布器 PID: ${YELLOW}$PUB_PID${NC}"
        echo -e "  - RViz PID: ${YELLOW}$RVIZ_PID${NC}"
        echo ""
        echo -e "按 ${YELLOW}Ctrl+C${NC} 退出"

        # 等待用户中断
        trap "kill $PUB_PID $RVIZ_PID 2>/dev/null; exit" INT TERM
        wait
    else
        echo -e "${YELLOW}警告: 未找到 rviz2，只运行点云发布器${NC}"
        echo -e "  - 点云发布器 PID: ${YELLOW}$PUB_PID${NC}"
        echo -e "  - 你可以在另一个终端运行: rviz2 -d config/clean_map.rviz"
        echo ""
        echo -e "按 ${YELLOW}Ctrl+C${NC} 退出"

        trap "kill $PUB_PID 2>/dev/null; exit" INT TERM
        wait $PUB_PID
    fi
    exit 0
fi

# 检查必需参数
if [ -z "$MAP_DIR" ] || [ -z "$BAG_PATH" ]; then
    echo -e "${RED}错误: 缺少必需参数${NC}"
    echo "使用 --help 查看帮助信息"
    exit 1
fi

# 进入项目根目录
cd "$(dirname "$0")"

echo -e "${GREEN}═══ 步骤 1/5: 检查依赖和输入文件 ═══${NC}"
echo ""

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}错误: 未找到 python3${NC}"
    exit 1
fi
echo -e "✓ Python3: $(python3 --version)"

# 检查关键 Python 库
echo -n "检查 Python 依赖... "
MISSING_DEPS=()
for module in ultralytics cv2 numpy scipy yaml rosbags; do
    if ! python3 -c "import $module" 2>/dev/null; then
        MISSING_DEPS+=("$module")
    fi
done

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo -e "${RED}失败${NC}"
    echo -e "${RED}缺少依赖: ${MISSING_DEPS[*]}${NC}"
    echo -e "${YELLOW}运行以下命令安装:${NC}"
    echo "  pip install ultralytics opencv-python numpy scipy pyyaml rosbags"
    exit 1
fi
echo -e "${GREEN}通过${NC}"

# 检查 GPU（如果指定）
if [ "$DEVICE" != "cpu" ]; then
    echo -n "检查 GPU... "
    GPU_AVAILABLE=$(python3 -c "import torch; print(torch.cuda.is_available())" 2>/dev/null || echo "False")
    if [ "$GPU_AVAILABLE" = "True" ]; then
        GPU_NAME=$(python3 -c "import torch; print(torch.cuda.get_device_name(0))" 2>/dev/null)
        echo -e "${GREEN}$GPU_NAME${NC}"
    else
        echo -e "${YELLOW}不可用，将使用 CPU${NC}"
        DEVICE="cpu"
    fi
fi

# 检查地图文件
echo ""
echo "检查地图文件..."
if [ ! -d "$MAP_DIR" ]; then
    echo -e "${RED}错误: 地图目录不存在: $MAP_DIR${NC}"
    exit 1
fi

if [ ! -f "$MAP_DIR/global.pcd" ]; then
    echo -e "${RED}错误: 找不到 $MAP_DIR/global.pcd${NC}"
    exit 1
fi
echo -e "  ✓ global.pcd: $(du -h "$MAP_DIR/global.pcd" | cut -f1)"

if [ ! -f "$MAP_DIR/keyframes.txt" ]; then
    echo -e "${RED}错误: 找不到 $MAP_DIR/keyframes.txt${NC}"
    echo -e "${YELLOW}提示: 确保 SLAM SaveMap 时导出了关键帧位姿${NC}"
    exit 1
fi
KF_COUNT=$(wc -l < "$MAP_DIR/keyframes.txt")
echo -e "  ✓ keyframes.txt: $KF_COUNT 个关键帧"

# 检查 bag 文件
echo ""
echo "检查 bag 文件..."
if [ ! -e "$BAG_PATH" ]; then
    echo -e "${RED}错误: bag 文件不存在: $BAG_PATH${NC}"
    exit 1
fi

if [ -d "$BAG_PATH" ]; then
    BAG_SIZE=$(du -sh "$BAG_PATH" | cut -f1)
    echo -e "  ✓ ROS2 bag: $BAG_SIZE"
elif [ -f "$BAG_PATH" ]; then
    BAG_SIZE=$(du -h "$BAG_PATH" | cut -f1)
    echo -e "  ✓ ROS1 bag: $BAG_SIZE"
fi

# 检查配置文件
echo ""
echo "检查配置文件..."
if [ ! -f "$CONFIG" ]; then
    echo -e "${RED}错误: 配置文件不存在: $CONFIG${NC}"
    exit 1
fi
echo -e "  ✓ $CONFIG"

echo ""
echo -e "${GREEN}═══ 步骤 2/5: 验证配置 ═══${NC}"
echo ""

# 读取配置关键参数
COLOR_TOPIC=$(python3 -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['color_topic'])" 2>/dev/null || echo "未知")
DEPTH_TOPIC=$(python3 -c "import yaml; print(yaml.safe_load(open('$CONFIG')).get('depth_topic', '无'))" 2>/dev/null || echo "未知")
YOLO_MODEL=$(python3 -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['yolo_model'])" 2>/dev/null || echo "未知")
MIN_VIEWS=$(python3 -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['min_views'])" 2>/dev/null || echo "未知")
VEHICLE_RATIO=$(python3 -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['vehicle_ratio'])" 2>/dev/null || echo"未知")

echo "配置参数:"
echo -e "  彩色图话题: ${YELLOW}$COLOR_TOPIC${NC}"
echo -e "  深度图话题: ${YELLOW}${DEPTH_TOPIC:-无}${NC}"
echo -e "  YOLO 模型: ${YELLOW}$YOLO_MODEL${NC}"
echo -e "  最小视角数: ${YELLOW}$MIN_VIEWS${NC}"
echo -e "  车辆判定比例: ${YELLOW}$VEHICLE_RATIO${NC}"

echo ""
echo -e "${GREEN}═══ 步骤 3/5: 运行车辆去除算法 ═══${NC}"
echo ""

# 构建命令
CMD="python3 scripts/clean_map_vehicles.py"
CMD="$CMD --map \"$MAP_DIR\""
CMD="$CMD --bag \"$BAG_PATH\""
CMD="$CMD --config \"$CONFIG\""
CMD="$CMD --device $DEVICE"

if [ "$RVIZ" = true ]; then
    # RViz 模式：先启动 RViz，再运行算法
    echo -e "${BLUE}启动 RViz (实时可视化模式)...${NC}"

    if ! command -v rviz2 &> /dev/null; then
        echo -e "${YELLOW}警告: 未找到 rviz2，跳过可视化${NC}"
        RVIZ=false
    else
        rviz2 -d config/clean_map.rviz &> /dev/null &
        RVIZ_PID=$!
        sleep 2
        echo -e "  ✓ RViz 已启动 (PID: $RVIZ_PID)"
        echo ""
        CMD="$CMD --rviz"
    fi
fi

echo "执行命令:"
echo -e "${YELLOW}$CMD${NC}"
echo ""

START_TIME=$(date +%s)

# 运行算法
eval $CMD

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))
MINUTES=$((DURATION / 60))
SECONDS=$((DURATION % 60))

echo ""
echo -e "${GREEN}═══ 步骤 4/5: 处理完成 ═══${NC}"
echo ""
echo -e "处理时间: ${YELLOW}${MINUTES}分${SECONDS}秒${NC}"

# 检查输出文件
if [ -f "$MAP_DIR/global_clean.pcd" ]; then
    CLEAN_SIZE=$(du -h "$MAP_DIR/global_clean.pcd" | cut -f1)
    echo -e "  ✓ global_clean.pcd: $CLEAN_SIZE"
else
    echo -e "${RED}  ✗ 未生成 global_clean.pcd${NC}"
fi

if [ -f "$MAP_DIR/global_vehicles.pcd" ]; then
    VEHICLE_SIZE=$(du -h "$MAP_DIR/global_vehicles.pcd" | cut -f1)
    echo -e "  ✓ global_vehicles.pcd: $VEHICLE_SIZE"
else
    echo -e "${YELLOW}  ! 未生成 global_vehicles.pcd (可能没有检测到车辆)${NC}"
fi

echo ""
echo -e "${GREEN}═══ 步骤 5/5: 可视化 ═══${NC}"
echo ""

if [ "$RVIZ" = true ]; then
    echo -e "${GREEN}RViz 实时可视化已在运行${NC}"
    echo -e "  - PID: ${YELLOW}$RVIZ_PID${NC}"
    echo ""
    echo -e "按 ${YELLOW}Ctrl+C${NC} 退出"
    trap "kill $RVIZ_PID 2>/dev/null; exit" INT TERM
    wait $RVIZ_PID
else
    echo "可以使用以下命令查看结果:"
    echo ""
    echo -e "  ${YELLOW}# 方式 1: RViz 可视化 (推荐)${NC}"
    echo "  # 终端 1:"
    echo "  python3 scripts/clean_map_rviz.py --map $MAP_DIR"
    echo "  # 终端 2:"
    echo "  rviz2 -d config/clean_map.rviz"
    echo ""
    echo -e "  ${YELLOW}# 方式 2: 使用此脚本${NC}"
    echo "  $0 --map $MAP_DIR --visualize-only"
    echo ""
    echo -e "  ${YELLOW}# 方式 3: CloudCompare${NC}"
    echo "  cloudcompare.CloudCompare $MAP_DIR/global_clean.pcd $MAP_DIR/global_vehicles.pcd"
fi

echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}           车辆去除完成！           ${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════${NC}"
echo ""
