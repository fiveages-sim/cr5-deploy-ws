#!/usr/bin/env bash
# 按与安装相反的顺序卸载核心 deb 包：arms-full → common → ocs2

set -uo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info()  { echo -e "${GREEN}[INFO]${NC} $1" >&2; }
print_warn()  { echo -e "${YELLOW}[WARN]${NC} $1" >&2; }
print_error() { echo -e "${RED}[ERROR]${NC} $1" >&2; }

trim() { local v="$1"; v="${v#"${v%%[![:space:]]*}"}"; echo "${v%"${v##*[![:space:]]}"}"; }

resolve_only_token() {
  case "$1" in
    ocs2|ocs2_ros2|ros-jazzy-ocs2) echo "ros-jazzy-ocs2" ;;
    common|robot-descriptions-common|ros-jazzy-robot-descriptions-common) echo "ros-jazzy-robot-descriptions-common" ;;
    arms|arms_ros2_control|arms_full|arms-full|ros-jazzy-arms-ros2-control|ros-jazzy-arms-ros2-control-full) \
      echo "ros-jazzy-arms-ros2-control-full" ;;
    ht|ht_ros2_control|ht-ros2-control) echo "ros-jazzy-arms-ros2-control-full" ;;
    *) return 1 ;;
  esac
}

usage() {
  cat <<EOF
用法: uninstall_core_debs.sh [--purge] [--only <list>]

卸载核心 deb 包（逆序）：
  ros-jazzy-arms-ros2-control-full → ros-jazzy-robot-descriptions-common → ros-jazzy-ocs2
  （若仍装有旧版标准 arms 包也会一并尝试卸载）

  --purge        同时清除包配置文件（apt-get purge）
  --only <list>  仅卸载指定包，逗号分隔。可用短名：ocs2, common, arms, ht
                 未指定时卸载全部核心包。
EOF
}

PURGE=0
ONLY_FILTER=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --purge) PURGE=1; shift ;;
    --only)
      IFS=',' read -ra _only_tokens <<< "$2"
      for _tok in "${_only_tokens[@]}"; do
        _tok="$(trim "$_tok")"
        [[ -z "$_tok" ]] && continue
        if ! _resolved="$(resolve_only_token "$_tok")"; then
          print_error "未知 --only 项: $_tok（可用: ocs2, common, arms, ht）"
          exit 1
        fi
        ONLY_FILTER+=("$_resolved")
      done
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

# 与安装顺序相反；顺带清理可能残留的标准版 arms
CORE_DEB_PACKAGES=(
  "ros-jazzy-arms-ros2-control-full"
  "ros-jazzy-arms-ros2-control"
  "ros-jazzy-robot-descriptions-common"
  "ros-jazzy-ocs2"
)

should_uninstall_pkg() {
  local pkg="$1" f
  if [[ ${#ONLY_FILTER[@]} -eq 0 ]]; then
    return 0
  fi
  for f in "${ONLY_FILTER[@]}"; do
    [[ "$f" == "$pkg" ]] && return 0
    # --only arms 时同时卸掉旧标准包
    if [[ "$f" == "ros-jazzy-arms-ros2-control-full" && "$pkg" == "ros-jazzy-arms-ros2-control" ]]; then
      return 0
    fi
  done
  return 1
}

remove_cmd=(remove -y)
if [[ "$PURGE" -eq 1 ]]; then
  remove_cmd=(purge -y)
fi

print_info "卸载核心 deb 包..."
if [[ ${#ONLY_FILTER[@]} -gt 0 ]]; then
  print_info "仅卸载: ${ONLY_FILTER[*]}"
fi
removed=()
skipped=()

for pkg in "${CORE_DEB_PACKAGES[@]}"; do
  if ! should_uninstall_pkg "$pkg"; then
    continue
  fi
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
