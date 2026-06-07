#!/usr/bin/env bash
# 快速模式：从 GitHub Release 下载并安装 ocs2 / common / arms_ros2_control deb 包

set -uo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info()  { echo -e "${GREEN}[INFO]${NC} $1" >&2; }
print_warn()  { echo -e "${YELLOW}[WARN]${NC} $1" >&2; }
print_error() { echo -e "${RED}[ERROR]${NC} $1" >&2; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DEB_CONF="${REPO_DIR}/deb_versions.conf"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DEB_ARCH="$(dpkg --print-architecture 2>/dev/null || true)"
DOWNLOAD_DIR="${REPO_DIR}/.deb_cache"

trim() { local v="$1"; v="${v#"${v%%[![:space:]]*}"}"; echo "${v%"${v##*[![:space:]]}"}"; }

usage() {
  cat <<EOF
用法: install_core_debs.sh [--ros-distro <distro>] [--config <file>]

从 deb_versions.conf 读取仓库信息，按依赖顺序安装核心 deb 包：
  1. ros-jazzy-ocs2
  2. ros-jazzy-robot-descriptions-common
  3. ros-jazzy-arms-ros2-control（标准版，不含硬件驱动）

配置中 tag 留空或填 latest 时，自动从 GitHub 获取最新 release。
安装使用 apt-get install，自动处理依赖。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --config) DEB_CONF="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) print_error "未知参数: $1"; usage; exit 1 ;;
  esac
done

if [[ -z "$DEB_ARCH" ]]; then
  print_error "无法检测 dpkg 架构，请确认在 Debian/Ubuntu 环境中运行。"
  exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
  print_error "未找到 apt-get，请确认在 Debian/Ubuntu 环境中运行。"
  exit 1
fi

if [[ ! -f "$DEB_CONF" ]]; then
  print_error "未找到配置文件: $DEB_CONF"
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

# 与 ros-jazzy-robot-descriptions-common 文件路径冲突的旧版独立 deb
COMMON_CONFLICT_PACKAGES=(
  "ros-${ROS_DISTRO}-robotiq-description"
)

prepare_common_deb_install() {
  local pkg conflict_choice
  local -a installed_conflicts=()

  for pkg in "${COMMON_CONFLICT_PACKAGES[@]}"; do
    is_pkg_installed "$pkg" && installed_conflicts+=("$pkg")
  done

  [[ ${#installed_conflicts[@]} -eq 0 ]] && return 0

  print_warn "安装 ros-jazzy-robot-descriptions-common 前需先卸载以下冲突包（安装路径重叠）："
  for pkg in "${installed_conflicts[@]}"; do
    print_warn "  - ${pkg}"
  done
  print_warn "common deb 已包含 robotiq 等公共描述，保留旧包会导致 apt 安装失败。"
  read -rp "是否自动卸载上述包？[Y/n]: " conflict_choice
  case "$conflict_choice" in
    n|N|no|NO)
      print_error "已取消安装。请手动执行: sudo apt-get remove ${installed_conflicts[*]}"
      exit 1
      ;;
  esac

  for pkg in "${installed_conflicts[@]}"; do
    print_info "卸载冲突包: ${pkg} ..."
    apt_run remove -y "$pkg" || {
      print_error "卸载 ${pkg} 失败，请手动删除后重试"
      exit 1
    }
  done
  echo "" >&2
}

# 按前缀匹配 release 中的 deb 资产（兼容新旧命名）
asset_patterns_for_prefix() {
  local prefix="$1"
  case "$prefix" in
    ros-jazzy-ocs2)
      printf '%s\n' \
        "${prefix}_*_${DEB_ARCH}.deb" \
        "*${prefix}*_${DEB_ARCH}.deb" \
        "ros-jazzy-ocs2-ros2-mobile-manipulator_*_${DEB_ARCH}.deb" \
        "*ocs2*jazzy*mobile-manipulator*_${DEB_ARCH}.deb" \
        "ocs2-ros2-jazzy-mobile-manipulator_*_${DEB_ARCH}.deb"
      ;;
    ros-jazzy-robot-descriptions-common)
      printf '%s\n' \
        "${prefix}_*_${DEB_ARCH}.deb" \
        "*${prefix}*_${DEB_ARCH}.deb" \
        "*robot-descriptions*jazzy*common*_${DEB_ARCH}.deb" \
        "robot-descriptions-jazzy-common_*_${DEB_ARCH}.deb"
      ;;
    ros-jazzy-arms-ros2-control)
      # 标准 WBC 包，不含 marvin/modbus 等硬件驱动（排除 *-full*）
      printf '%s\n' \
        "${prefix}_*_${DEB_ARCH}.deb"
      ;;
    *)
      printf '%s\n' \
        "${prefix}_*_${DEB_ARCH}.deb" \
        "*${prefix}*_${DEB_ARCH}.deb"
      ;;
  esac
}

should_exclude_deb_asset() {
  local prefix="$1" asset="$2"
  case "$prefix" in
    ros-jazzy-arms-ros2-control)
      [[ "$asset" == *"-full_"* || "$asset" == ros-jazzy-arms-ros2-control-full* ]] && return 0
      ;;
  esac
  return 1
}

glob_match() {
  local name="$1" pattern="$2"
  case "$name" in
    $pattern) return 0 ;;
    *) return 1 ;;
  esac
}

deb_file_size() {
  stat -c%s "$1" 2>/dev/null || stat -f%z "$1" 2>/dev/null || wc -c < "$1"
}

fetch_release_assets_with_size() {
  local repo="$1" tag="$2"
  curl -fsSL "https://api.github.com/repos/${repo}/releases/tags/${tag}" \
    | python3 -c "
import json, sys
for a in json.load(sys.stdin).get('assets', []):
    print(str(a['name']) + '|' + str(a['size']))
" 2>/dev/null || true
}

lookup_asset_size() {
  local asset_name="$1"
  shift
  local line name size
  for line in "$@"; do
    name="${line%%|*}"
    size="${line#*|}"
    if [[ "$name" == "$asset_name" ]]; then
      printf '%s\n' "$size"
      return 0
    fi
  done
  return 1
}

is_valid_deb_file() {
  local deb_file="$1" expected_size="${2:-}"
  local actual_size=""

  [[ -f "$deb_file" ]] || return 1
  actual_size="$(deb_file_size "$deb_file")"
  [[ -n "$actual_size" && "$actual_size" -gt 0 ]] || return 1

  if [[ -n "$expected_size" && "$actual_size" -ne "$expected_size" ]]; then
    return 1
  fi

  ar t "$deb_file" >/dev/null 2>&1 || return 1
  if command -v dpkg-deb >/dev/null 2>&1; then
    dpkg-deb -I "$deb_file" >/dev/null 2>&1 || return 1
  fi
  return 0
}

fetch_release_assets() {
  local repo="$1" tag="$2"
  fetch_release_assets_with_size "$repo" "$tag" | cut -d'|' -f1
}

release_exists() {
  local repo="$1" tag="$2"
  curl -fsSL -o /dev/null -w '%{http_code}' "https://api.github.com/repos/${repo}/releases/tags/${tag}" 2>/dev/null | grep -q '^200$'
}

fetch_latest_release_tag() {
  local repo="$1"
  curl -fsSL "https://api.github.com/repos/${repo}/releases?per_page=20" \
    | python3 -c '
import json, sys
releases = json.load(sys.stdin)
for r in releases:
    if r.get("draft"):
        continue
    tag = r.get("tag_name", "")
    if tag:
        print(tag)
        break
' 2>/dev/null \
    || curl -fsSL "https://api.github.com/repos/${repo}/releases/latest" \
    | python3 -c 'import json,sys; print(json.load(sys.stdin).get("tag_name",""))' 2>/dev/null \
    || true
}

resolve_release_tag() {
  local requested_tag="$1" primary_repo="$2" label="$3"
  local tag=""

  requested_tag="$(trim "$requested_tag")"
  primary_repo="$(trim "$primary_repo")"
  if [[ -z "$primary_repo" ]]; then
    print_error "${label}: 未配置 GitHub 仓库"
    return 1
  fi

  if [[ -n "$requested_tag" && "$requested_tag" != "latest" && "$requested_tag" != "*" ]]; then
    if [[ "$requested_tag" != [vV]* ]]; then
      requested_tag="v${requested_tag}"
    fi
    printf '%s\n' "$requested_tag"
    return 0
  fi

  tag="$(fetch_latest_release_tag "$primary_repo")"
  if [[ -n "$tag" ]]; then
    print_info "  ${label}: 使用 ${primary_repo} 最新 release ${tag}"
    printf '%s\n' "$tag"
    return 0
  fi

  print_error "无法解析 ${label} 的最新 release tag（仓库: ${primary_repo}）"
  return 1
}

download_file_with_progress() {
  local url="$1" dest="$2" filename="$3"
  print_info "  下载 ${filename} ..."
  print_info "  来源: ${url}"

  # --progress-bar 输出到 stderr，不干扰命令替换；大文件显示下载进度
  if curl -fL --progress-bar --connect-timeout 30 --retry 3 --retry-delay 2 \
      -o "$dest" "$url"; then
    local size
    size="$(du -h "$dest" | awk '{print $1}')"
    print_info "  下载完成 (${size}): ${dest}"
    return 0
  fi
  return 1
}

purge_stale_deb_cache() {
  local prefix="$1" keep_file="$2"
  local f bn

  [[ -d "$DOWNLOAD_DIR" ]] || return 0

  shopt -s nullglob
  for f in "${DOWNLOAD_DIR}/${prefix}"_*.deb "${DOWNLOAD_DIR}/${prefix}"_*.deb.part; do
    bn="$(basename "$f")"
    [[ "$bn" == "$keep_file" || "$bn" == "${keep_file}.part" ]] && continue
    print_info "  清除旧版本缓存: ${bn}"
    rm -f "$f"
  done
  shopt -u nullglob
}

download_deb_for_package() {
  local prefix="$1" tag="$2" primary_repo="$3"
  local repo asset patterns=() matched="" url dest tmp_dest resolved_tag expected_size=""
  local -a asset_lines=()

  primary_repo="$(trim "$primary_repo")"
  resolved_tag="$(resolve_release_tag "$tag" "$primary_repo" "$prefix")" || return 1
  tag="$resolved_tag"

  if ! release_exists "$primary_repo" "$tag"; then
    print_error "  ${primary_repo}@${tag} 不存在"
    return 1
  fi

  mapfile -t asset_lines < <(fetch_release_assets_with_size "$primary_repo" "$tag")
  if [[ ${#asset_lines[@]} -eq 0 ]]; then
    print_error "  ${primary_repo}@${tag} 无 release 资产"
    return 1
  fi

  matched=""
  mapfile -t patterns < <(asset_patterns_for_prefix "$prefix")
  for pattern in "${patterns[@]}"; do
    for asset in "${asset_lines[@]}"; do
      asset="${asset%%|*}"
      should_exclude_deb_asset "$prefix" "$asset" && continue
      if glob_match "$asset" "$pattern"; then
        matched="$asset"
        break 2
      fi
    done
  done

  if [[ -z "$matched" ]]; then
    print_error "  ${primary_repo}@${tag} 未找到匹配 ${prefix} 的 deb（架构 ${DEB_ARCH}）"
    print_warn "  可用资产: $(printf '%s\n' "${asset_lines[@]}" | cut -d'|' -f1 | tr '\n' ' ')"
    return 1
  fi

  expected_size="$(lookup_asset_size "$matched" "${asset_lines[@]}")" || expected_size=""

  mkdir -p "$DOWNLOAD_DIR"
  dest="${DOWNLOAD_DIR}/${matched}"
  tmp_dest="${dest}.part"

  purge_stale_deb_cache "$prefix" "$matched"

  if [[ -f "$dest" ]]; then
    if is_valid_deb_file "$dest" "$expected_size"; then
      print_info "  已缓存: $dest"
      printf '%s\n' "$dest"
      return 0
    fi
    print_warn "  缓存文件不完整或已损坏（期望 ${expected_size:-未知} 字节），将重新下载..."
    rm -f "$dest" "${dest}.part"
  fi

  url="https://github.com/${primary_repo}/releases/download/${tag}/${matched}"
  rm -f "$tmp_dest"
  if download_file_with_progress "$url" "$tmp_dest" "$matched"; then
    if is_valid_deb_file "$tmp_dest" "$expected_size"; then
      mv -f "$tmp_dest" "$dest"
      printf '%s\n' "$dest"
      return 0
    fi
    print_error "  下载完成但文件校验失败（可能网络中断），请重试"
  fi
  rm -f "$tmp_dest"
  print_error "  下载失败: $url"
  return 1
}

install_deb() {
  local deb_file="$1"
  print_info "安装 ${deb_file} ..."
  if apt_run install -y "$deb_file"; then
    return 0
  fi
  print_warn "安装失败，删除可能损坏的缓存以便下次重新下载: ${deb_file}"
  rm -f "$deb_file" "${deb_file}.part"
  return 1
}

# 读取配置（文件顺序即安装顺序：ocs2 → common → arms）
DEB_SPECS=()
while IFS= read -r line || [[ -n "$line" ]]; do
  line="${line%%#*}"
  line="$(trim "$line")"
  [[ -z "$line" ]] && continue
  DEB_SPECS+=("$line")
done < "$DEB_CONF"

if [[ ${#DEB_SPECS[@]} -eq 0 ]]; then
  print_error "deb_versions.conf 中没有有效条目"
  exit 1
fi

print_info "安装核心 deb 包（ROS ${ROS_DISTRO}，架构 ${DEB_ARCH}）"
print_info "安装顺序: ocs2 → common → arms_ros2_control"
print_info "配置文件: $DEB_CONF"
echo "" >&2

installed=()
step=1
for spec in "${DEB_SPECS[@]}"; do
  IFS='|' read -r prefix tag primary_repo <<< "$spec"
  prefix="$(trim "$prefix")"
  tag="$(trim "$tag")"
  primary_repo="$(trim "$primary_repo")"

  print_info "[${step}/${#DEB_SPECS[@]}] 处理 ${prefix} ..."
  deb_path="$(download_deb_for_package "$prefix" "$tag" "$primary_repo")" || exit 1
  if [[ "$prefix" == "ros-jazzy-robot-descriptions-common" ]]; then
    prepare_common_deb_install
  fi
  install_deb "$deb_path" || {
    print_error "安装 ${prefix} 失败"
    exit 1
  }
  installed+=("$prefix")
  step=$((step + 1))
  echo "" >&2
done

print_info "=========================================="
print_info "核心 deb 包安装完成："
for pkg in "${installed[@]}"; do
  print_info "  ✓ ${pkg}"
done
print_info "=========================================="
print_info ""
print_info "使用前请 source ROS 环境："
print_info "  source /opt/ros/${ROS_DISTRO}/setup.bash"
