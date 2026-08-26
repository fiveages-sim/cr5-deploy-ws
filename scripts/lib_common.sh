#!/usr/bin/env bash
# 公共基础库：颜色 / 常用工具 / 架构检测 / apt 辅助 / 通用菜单
# 供 init_repo.sh / quick_start.sh / release.sh / scripts/*.sh source 引入

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info()  { echo -e "${GREEN}[INFO]${NC} $1" >&2; }
print_warn()  { echo -e "${YELLOW}[WARN]${NC} $1" >&2; }
print_error() { echo -e "${RED}[ERROR]${NC} $1" >&2; }

trim() {
  local v="$1"
  v="${v#"${v%%[![:space:]]*}"}"
  printf '%s' "${v%"${v##*[![:space:]]}"}"
}

need_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    print_error "缺少命令: $cmd"
    return 1
  fi
  return 0
}

detect_machine_arch() {
  local arch
  arch="$(dpkg --print-architecture 2>/dev/null || true)"
  case "${arch}" in
    amd64|arm64) echo "${arch}" ;;
    x86_64) echo "amd64" ;;
    aarch64) echo "arm64" ;;
    *)
      print_warn "无法识别架构 (${arch:-unknown})，默认使用 amd64"
      echo "amd64"
      ;;
  esac
}

resolve_deb_arch() {
  local input="${1:-}"
  case "${input}" in
    amd64|x64|x86_64) echo "amd64" ;;
    arm64|aarch64|arm) echo "arm64" ;;
    *)
      print_error "未知架构: ${input}（可选: amd64 / arm64）"
      return 1
      ;;
  esac
}

# conf 中 tag（v1.5.0）→ deb 文件名版本段（1.5.0）；latest / pre-release 返回空
tag_to_deb_version() {
  local tag="$1"
  case "${tag}" in
    latest|"*"|pre-release|prerelease|pre) echo "" ;;
    *)
      tag="${tag#v}"
      tag="${tag#V}"
      echo "${tag}"
      ;;
  esac
}

is_pkg_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

pkg_version() {
  dpkg-query -W -f='${Version}' "$1" 2>/dev/null || true
}

apt_run() {
  if [[ "$(id -u)" -eq 0 ]]; then
    apt-get "$@"
  else
    sudo apt-get "$@"
  fi
}

# 通用菜单选择：选项输出到 stderr（默认项标号前加 *），输入返回所选数字
# 用法: choice="$(menu_select "请选择" 1 "选项A" "选项B")"
menu_select() {
  local prompt="$1" default="$2"; shift 2
  local -a opts=("$@")
  local n="${#opts[@]}" i choice
  echo "" >&2
  for ((i = 0; i < n; i++)); do
    if [ $((i + 1)) -eq "${default}" ]; then
      echo " *$((i + 1))) ${opts[i]}" >&2
    else
      echo "  $((i + 1))) ${opts[i]}" >&2
    fi
  done
  read -rp "${prompt} [1-${n}]（回车=默认 ${default}）: " choice
  if [[ ! "${choice}" =~ ^[0-9]+$ ]] || [ "${choice}" -lt 1 ] || [ "${choice}" -gt "${n}" ]; then
    choice="${default}"
  fi
  printf '%s' "${choice}"
}

# 确认提示（默认否）
confirm_yn() {
  local prompt="$1" ans
  read -rp "${prompt} [y/N]: " ans
  case "${ans}" in
    y|Y|yes|YES) return 0 ;;
    *) return 1 ;;
  esac
}
