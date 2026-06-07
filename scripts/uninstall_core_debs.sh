#!/usr/bin/env bash
# 按与安装相反的顺序卸载核心 deb 包：arms → common → ocs2

set -uo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info()  { echo -e "${GREEN}[INFO]${NC} $1" >&2; }
print_warn()  { echo -e "${YELLOW}[WARN]${NC} $1" >&2; }
print_error() { echo -e "${RED}[ERROR]${NC} $1" >&2; }

usage() {
  cat <<EOF
用法: uninstall_core_debs.sh [--purge]

卸载核心 deb 包（逆序）：
  ros-jazzy-arms-ros2-control → ros-jazzy-robot-descriptions-common → ros-jazzy-ocs2

  --purge  同时清除包配置文件（apt-get purge）
EOF
}

PURGE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --purge) PURGE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) print_error "未知参数: $1"; usage; exit 1 ;;
  esac
done

if ! command -v apt-get >/dev/null 2>&1; then
  print_error "未找到 apt-get，请确认在 Debian/Ubuntu 环境中运行。"
  exit 1
fi

apt_run() {
  if [[ "$(id -u)" -eq 0 ]]; then
    apt-get "$@"
  else
    sudo apt-get "$@"
  fi
}

is_pkg_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

# 与安装顺序相反
CORE_DEB_PACKAGES=(
  "ros-jazzy-arms-ros2-control"
  "ros-jazzy-robot-descriptions-common"
  "ros-jazzy-ocs2"
)

remove_cmd=(remove -y)
if [[ "$PURGE" -eq 1 ]]; then
  remove_cmd=(purge -y)
fi

print_info "卸载核心 deb 包..."
removed=()
skipped=()

for pkg in "${CORE_DEB_PACKAGES[@]}"; do
  if is_pkg_installed "$pkg"; then
    print_info "卸载 ${pkg} ..."
    if apt_run "${remove_cmd[@]}" "$pkg"; then
      removed+=("$pkg")
    else
      print_error "卸载 ${pkg} 失败"
      exit 1
    fi
  else
    print_warn "未安装，跳过: ${pkg}"
    skipped+=("$pkg")
  fi
done

echo ""
print_info "=========================================="
if [[ ${#removed[@]} -gt 0 ]]; then
  print_info "已卸载："
  for pkg in "${removed[@]}"; do
    print_info "  ✓ ${pkg}"
  done
fi
if [[ ${#skipped[@]} -gt 0 ]]; then
  print_info "已跳过（未安装）："
  for pkg in "${skipped[@]}"; do
    print_info "  - ${pkg}"
  done
fi
print_info "=========================================="
