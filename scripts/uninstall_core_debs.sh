#!/usr/bin/env bash
# 按与安装相反的顺序卸载核心 deb 包：arms-full → common → ocs2
# 用法:
#   uninstall_core_debs.sh [--only <ocs2,common,arms>] [--purge]

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

. "${SCRIPT_DIR}/lib_common.sh"
. "${SCRIPT_DIR}/lib_deb.sh"

ONLY_LIST=""

usage() {
  cat <<EOF
用法: uninstall_core_debs.sh [--only <list>] [--purge]

卸载核心 deb 包（逆序）：
  ros-jazzy-arms-ros2-control-full → ros-jazzy-robot-descriptions-common → ros-jazzy-ocs2
  （若仍装有旧版标准 arms 包也会一并尝试卸载）

  --only <list>  仅卸载指定包，逗号分隔。可用短名：ocs2, common, arms
                 未指定时卸载全部核心包。
  --purge        同时清除包配置文件（apt-get purge）
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --only)
      ONLY_LIST="$(trim "$2")"
      shift 2
      ;;
    --purge) PURGE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) print_error "未知参数: $1"; usage; exit 1 ;;
  esac
done

if ! command -v apt-get >/dev/null 2>&1; then
  print_error "未找到 apt-get，请确认在 Debian/Ubuntu 环境中运行。"
  exit 1
fi

uninstall_core_debs "${ONLY_LIST}" || exit 1
