#!/bin/bash
# 清理项目，只保留车辆去除系统相关文件

set -e

echo "开始清理项目，保留车辆去除系统核心文件..."

# 进入项目根目录
cd "$(dirname "$0")/.."

# 备份当前状态
BACKUP_DIR="backup_$(date +%Y%m%d_%H%M%S)"
echo "创建备份目录: $BACKUP_DIR"
mkdir -p "../$BACKUP_DIR"

# 要保留的核心文件列表
KEEP_FILES=(
    # 核心 C++ 源码和头文件（SLAM 系统）
    "src/core/system/slam.cc"
    "src/core/system/slam.h"
    "src/core/lio/eskf.cc"
    "src/core/lio/eskf.hpp"
    "src/core/lio/laser_mapping.cc"
    "src/core/lio/imu_processing.hpp"
    "src/common/nav_state.h"
    "src/ui/ui_cloud.cc"
    "src/ui/ui_cloud.h"

    # CMake 构建文件
    "CMakeLists.txt"
    "thirdparty/livox_ros_driver/CMakeLists.txt"
    "thirdparty/livox_ros_driver/msg/CustomPoint.msg"

    # 车辆去除脚本
    "scripts/clean_map_vehicles.py"
    "scripts/clean_map_rviz.py"
    "scripts/install_dep.sh"

    # 配置文件（车辆去除 + 示例建图配置）
    "config/garage_clean.yaml"
    "config/m2dgr_clean.yaml"
    "config/clean_map.rviz"
    "config/default.yaml"
    "config/default_robosense.yaml"

    # 文档
    "doc/VEHICLE_REMOVAL_GUIDE.md"
    "README.md"
    "README_CN.md"
)

# 要删除的文件和目录
echo ""
echo "删除不相关的配置文件..."
rm -f config/centennial_fusion.yaml config/centennial_fusion.rviz
rm -f config/fusion_visualization*.rviz
rm -f config/hku_*.yaml config/hku_*.rviz
rm -f config/m2dgr_fusion.rviz config/m2dgr_walk_fusion.yaml config/m2dgr_walk_stream.yaml
rm -f config/nuscenes_*.yaml
rm -f config/outdoor_road.yaml
rm -f config/r3live_*.yaml
rm -f config/test_simple.rviz
rm -f config/visual_fusion.yaml config/visual_fusion.yaml.backup
rm -f config/default_livox.yaml config/default_nclt.yaml config/default_utbm.yaml config/default_vbr.yaml

echo "删除不相关的脚本..."
rm -f scripts/auto_save_fusion_map.py
rm -f scripts/centennial_fusion.py
rm -f scripts/check_fusion_system.sh
rm -f scripts/check_m2dgr_topics.py
rm -f scripts/check_r3live_environment.sh
rm -f scripts/check_slam_status.sh
rm -f scripts/convert_m2dgr_to_ros2.sh
rm -f scripts/custommsg_to_pc2.py
rm -f scripts/download_kitti_dataset.sh
rm -f scripts/download_m2dgr_walk.sh
rm -f scripts/fake_imu_publisher.py
rm -f scripts/fix_livox_package_name.sh
rm -f scripts/kitti_*.py
rm -f scripts/lightning_vision_fusion_*.py
rm -f scripts/m2dgr_fusion.py
rm -f scripts/merge_bags.py
rm -f scripts/monitor_*.sh
rm -f scripts/nuscenes_*.py
rm -f scripts/r3live_*.py
rm -f scripts/r3live_*.sh
rm -f scripts/ros1_image_republisher.py
rm -f scripts/ros2_image_publisher.py
rm -f scripts/run_m2dgr_walk.sh
rm -f scripts/save_default_map.sh
rm -f scripts/save_fusion_map.sh
rm -f scripts/simple_image_viewer.py
rm -f scripts/start_*.sh
rm -f scripts/stop_all.sh
rm -f scripts/test_*.sh
rm -f scripts/test_*.py
rm -f scripts/USAGE.sh

echo "删除不相关的文档..."
rm -f doc/CURRENT_STATUS.md
rm -f doc/DIAGNOSIS_REPORT.md
rm -f doc/FUSION_SLAM_INTEGRATION.md
rm -f doc/Fusion_System_Principle_Introduction.md
rm -f doc/Fusion_System_Principle_Introduction.docx
rm -f doc/HKU_AUTO_SAVE.md
rm -f doc/HKU_PARK_FUSION_GUIDE.md
rm -f doc/KITTI_RAW_TEST.md
rm -f doc/Lightning-LM_YOLO视觉融合系统_原理分析.md
rm -f doc/Lightning-LM_YOLO视觉融合系统_原理分析.docx
rm -f doc/M2DGR_STREAM.md
rm -f doc/M2DGR_WALK_ADAPTATION_GUIDE.md
rm -f doc/M2DGR_WALK_USAGE_GUIDE.md
rm -f doc/NUSCENES_SLAM_STATUS.md
rm -f doc/R3LIVE_FUSION_GUIDE.md
rm -f doc/R3LIVE_INTEGRATION_SUMMARY.md
rm -f doc/USER_GUIDE.md
rm -f doc/VISION_FUSION_ANALYSIS.md
rm -f doc/VISION_FUSION_ARCHITECTURE.md
rm -f doc/VISION_FUSION_DIAGNOSIS.md
rm -f doc/VISION_FUSION_QUICKSTART.md
rm -f doc/VISUAL_FUSION_COMPLETE_GUIDE.md
rm -f doc/融合完成总结.md
rm -f doc/.~lock.*

echo "删除测试数据和临时文件..."
rm -rf data/*/overlay_*.png 2>/dev/null || true
rm -f synth/*.pcd synth/*.png 2>/dev/null || true

echo ""
echo "清理完成！保留的核心文件："
echo "  - 车辆去除脚本: scripts/clean_map_vehicles.py, scripts/clean_map_rviz.py"
echo "  - 配置文件: config/garage_clean.yaml, config/m2dgr_clean.yaml, config/clean_map.rviz"
echo "  - 文档: doc/VEHICLE_REMOVAL_GUIDE.md"
echo "  - SLAM 核心代码: src/core/system/slam.cc (SaveMap 导出 keyframes.txt)"
echo "  - 依赖安装: scripts/install_dep.sh"
echo ""
echo "项目现在只包含车辆去除系统的必要文件。"
echo "备份已创建在: ../$BACKUP_DIR (如果需要恢复旧文件)"
