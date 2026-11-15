#!/usr/bin/env bash
# 用 s100_fusion ROS2 bag 跑 Lightning-LM 离线建图
#
# 用法:
#   ./scripts/run_s100_offline.sh
#   ./scripts/run_s100_offline.sh "/path/to/s100_fusion_20261114_164937"
#   BAG_PATH=/path/to/bag HEADLESS=1 ./scripts/run_s100_offline.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BAG_NAME="s100_fusion_20261114_164937"
CONFIG="${CONFIG:-$ROOT/config/s100_fusion_offline.yaml}"
MAP_DIR="${MAP_DIR:-$ROOT/data/${BAG_NAME}}"
HEADLESS="${HEADLESS:-0}"

resolve_bag_path() {
  # 1) 命令行参数  2) 环境变量  3) 常见挂载点自动探测
  if [[ $# -ge 1 && -n "${1:-}" ]]; then
    printf '%s\n' "$1"
    return 0
  fi
  if [[ -n "${BAG_PATH:-}" ]]; then
    printf '%s\n' "$BAG_PATH"
    return 0
  fi

  local candidates=(
    "/media/lizhi/Seagate Backup Plus Drive/研究生/${BAG_NAME}"
    "/media/${USER}/Seagate Backup Plus Drive/研究生/${BAG_NAME}"
    "/run/media/${USER}/Seagate Backup Plus Drive/研究生/${BAG_NAME}"
    "/mnt/seagate/研究生/${BAG_NAME}"
    "$HOME/data/${BAG_NAME}"
    "$ROOT/data/bags/${BAG_NAME}"
  )
  local c
  for c in "${candidates[@]}"; do
    if [[ -f "${c}/metadata.yaml" ]]; then
      printf '%s\n' "$c"
      return 0
    fi
  done

  # 在已挂载的 /media 与 /run/media 下按目录名搜索一层
  local root found=""
  for root in "/media/${USER}" "/media/lizhi" "/run/media/${USER}" /mnt; do
    [[ -d "$root" ]] || continue
    found="$(find "$root" -maxdepth 4 -type d -name "$BAG_NAME" 2>/dev/null | head -n 1 || true)"
    if [[ -n "$found" && -f "${found}/metadata.yaml" ]]; then
      printf '%s\n' "$found"
      return 0
    fi
  done
  return 1
}

if ! BAG_PATH="$(resolve_bag_path "${1:-}")"; then
  echo "错误: 找不到 bag 目录 ${BAG_NAME}"
  echo ""
  echo "当前未检测到 Seagate 盘挂载 (/media/lizhi 为空)。请先插入/挂载移动硬盘，或显式指定路径:"
  echo "  ./scripts/run_s100_offline.sh \"/你的挂载点/研究生/${BAG_NAME}\""
  echo "  BAG_PATH=\"/你的挂载点/研究生/${BAG_NAME}\" ./scripts/run_s100_offline.sh"
  echo ""
  echo "查看挂载: ls /media/\$USER ; lsblk -o NAME,LABEL,MOUNTPOINT"
  exit 1
fi

echo "=========================================="
echo "S100 Fusion 离线建图"
echo "=========================================="
echo "Bag:    $BAG_PATH"
echo "配置:  $CONFIG"
echo "地图:  $MAP_DIR"
echo "无界面: $HEADLESS"
echo ""

if [[ ! -e "$BAG_PATH" ]]; then
  echo "错误: bag 不存在: $BAG_PATH"
  exit 1
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "错误: 配置不存在: $CONFIG"
  exit 1
fi
if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
  echo "错误: 不是 ROS2 bag 目录 (缺少 metadata.yaml): $BAG_PATH"
  exit 1
fi

# 话题自检 (依赖 rosbags；没有则跳过)
if python3 - <<'PY' "$BAG_PATH"
import sys
from pathlib import Path
try:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
except Exception as e:
    print(f'(跳过话题检查: {e})')
    sys.exit(0)
bag = Path(sys.argv[1])
need = {'/lidar/lidar_02/points', '/lidar/lidar_02/imu'}
with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
    topics = {c.topic for c in reader.connections}
missing = sorted(need - topics)
if missing:
    print('错误: bag 缺少话题:', ', '.join(missing))
    print('现有话题:', ', '.join(sorted(topics)))
    sys.exit(1)
print('话题检查通过:', ', '.join(sorted(need)))
PY
then
  :
else
  exit 1
fi

# colcon/ROS setup.bash 会读未定义变量；临时关闭 nounset
set +u
if [[ -z "${ROS_DISTRO:-}" ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

INSTALL_SETUP="$ROOT/install/setup.bash"
if [[ ! -f "$INSTALL_SETUP" ]]; then
  # 兼容把 lightning 当作 workspace 子包编译的情况
  WS_SETUP="$(dirname "$ROOT")/install/setup.bash"
  if [[ -f "$WS_SETUP" ]]; then
    INSTALL_SETUP="$WS_SETUP"
  else
    set -u
    echo "错误: 未找到 install/setup.bash，请先编译 lightning"
    echo "  source /opt/ros/humble/setup.bash && colcon build --packages-select lightning --symlink-install"
    exit 1
  fi
fi
# shellcheck source=/dev/null
source "$INSTALL_SETUP"
set -u

mkdir -p "$MAP_DIR"

RUN_CONFIG="$CONFIG"
CLEANUP_CONFIG=0
if [[ "$HEADLESS" == "1" ]]; then
  RUN_CONFIG="$(mktemp /tmp/s100_offline_XXXXXX.yaml)"
  CLEANUP_CONFIG=1
  # 关闭 Pangolin / 2D UI，便于无显示器或 SSH 冒烟
  sed -e 's/with_ui:.*/with_ui: false/' \
      -e 's/with_2dui:.*/with_2dui: false/' \
      -e "s|map_path:.*|map_path: ${MAP_DIR}/|" \
      "$CONFIG" > "$RUN_CONFIG"
else
  # 仅覆盖地图输出路径，避免改仓库内配置
  RUN_CONFIG="$(mktemp /tmp/s100_offline_XXXXXX.yaml)"
  CLEANUP_CONFIG=1
  sed -e "s|map_path:.*|map_path: ${MAP_DIR}/|" "$CONFIG" > "$RUN_CONFIG"
fi

cleanup() {
  if [[ "$CLEANUP_CONFIG" == "1" ]]; then
    rm -f "$RUN_CONFIG"
  fi
}
trap cleanup EXIT

echo "启动: ros2 run lightning run_slam_offline"
echo "  --config $RUN_CONFIG"
echo "  --input_bag $BAG_PATH"
echo ""

ros2 run lightning run_slam_offline \
  --config "$RUN_CONFIG" \
  --input_bag "$BAG_PATH"

echo ""
echo "完成。地图目录: $MAP_DIR"
ls -lh "$MAP_DIR" 2>/dev/null || true
echo ""
echo "说明: 若日志出现大量 'No point or unreliable IMU coverage' / gap longer than 1 second，"
echo "      多半是 bag 内点云/IMU 录制空洞，不是话题配置错误；前半连续段仍可建图。"
echo ""
echo "可选去车 (视觉融合后处理):"
echo "  ./run_vehicle_removal.sh --map \"$MAP_DIR\" --bag \"$BAG_PATH\" --config config/garage_clean.yaml"
