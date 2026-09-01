#!/usr/bin/env bash
# 下载并安装核心 deb 包（供 init_repo.sh 与单独使用）
# 用法:
#   install_core_debs.sh [--channel conf|latest|pre-release]
#                       [--arms-variant full|standard]
#                       [--only <ocs2,common,arms>]
#                       [--ros-distro <distro>] [--config <file>]

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

. "${SCRIPT_DIR}/lib_common.sh"
. "${SCRIPT_DIR}/lib_github.sh"
. "${SCRIPT_DIR}/lib_deb.sh"

ONLY_LIST=""

usage() {
  cat <<EOF
用法: install_core_debs.sh [--channel <conf|latest|pre-release>]
                          [--arms-variant <full|standard>]
                          [--only <list>]
                          [--ros-distro <distro>] [--config <file>]

从 deb_versions.conf 读取仓库信息，按依赖顺序安装核心 deb 包：
  1. ros-jazzy-ocs2
  2. ros-jazzy-robot-descriptions-common
  3. ros-jazzy-arms-ros2-control-full（含 fairino_ros2_control 等硬件驱动）

  --channel <name>      发布通道（默认 conf，按 deb_versions.conf 固定 tag）：
                         conf       使用 deb_versions.conf 中的固定 tag
                         latest     GitHub Latest 稳定版（仅 API 查询）
                         pre-release 各仓库 pre-release 浮动标签（仅 API 查询）
  --arms-variant <full|standard>
                        arms deb 变体（默认 full；standard 装 ros-jazzy-arms-ros2-control）
  --only <list>         仅安装指定包，逗号分隔。可用短名：ocs2, common, arms
安装支持降级/升级（apt install --allow-downgrades）。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --config) DEB_CONF="$2"; shift 2 ;;
    --channel)
      set_deb_channel "$2" || exit 1
      shift 2
      ;;
    --arms-variant)
      ARMS_VARIANT="$(trim "$2")"
      case "$ARMS_VARIANT" in
        full|standard) ;;
        *) print_error "未知 --arms-variant: $2（可用: full, standard）"; exit 1 ;;
      esac
      shift 2
      ;;
    --only)
      ONLY_LIST="$(trim "$2")"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) print_error "未知参数: $1"; usage; exit 1 ;;
  esac
done

if ! command -v apt-get >/dev/null 2>&1; then
  print_error "未找到 apt-get，请确认在 Debian/Ubuntu 环境中运行。"
  exit 1
fi

if [ ! -f "${DEB_CONF}" ]; then
  print_error "未找到配置文件: ${DEB_CONF}"
  exit 1
fi

print_info "安装核心 deb（ROS ${ROS_DISTRO}，架构 ${DEB_ARCH}，通道 ${DEB_CHANNEL}，arms 变体 ${ARMS_VARIANT}）"
print_info "安装顺序: ocs2 → robot-descriptions-common → arms-ros2-control(-full)"

install_core_debs "${ONLY_LIST}" || exit 1

print_info ""
print_info "使用前请 source ROS 环境:"
print_info "  source /opt/ros/${ROS_DISTRO}/setup.bash"
