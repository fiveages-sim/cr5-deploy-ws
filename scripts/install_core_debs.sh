#!/usr/bin/env bash
# 快速模式：从 GitHub Release 下载并安装 ocs2 / common / arms_ros2_control(-full) deb 包
# 本工作空间默认 arms standard；可用 --arms-variant full 切换（仍不含 arx_ros2_control）

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DEB_CONF="${REPO_DIR}/deb_versions.conf"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DEB_ARCH="$(dpkg --print-architecture 2>/dev/null || true)"
DOWNLOAD_DIR="${REPO_DIR}/.deb_cache"

# 共用函数库（颜色输出 / GitHub API / 下载 / deb 校验等）
LIB_DEB_COMMON="${SCRIPT_DIR}/lib_deb_common.sh"
if [ ! -f "${LIB_DEB_COMMON}" ]; then
  echo "[ERROR] 未找到共用函数库: ${LIB_DEB_COMMON}" >&2
  exit 1
fi
# shellcheck source=lib_deb_common.sh
# shellcheck disable=SC1090
. "${LIB_DEB_COMMON}"

# arms 变体：full（ros-jazzy-arms-ros2-control-full）| standard（ros-jazzy-arms-ros2-control）
# 空 = 未指定，沿用 deb_versions.conf 中的前缀
ARMS_VARIANT=""

# 解析 arms 变体对应的有效前缀（--arms-variant 覆盖 conf 中的前缀，不修改 deb_versions.conf）
resolve_effective_prefix() {
  local prefix="$1"
  case "$prefix" in
    ros-jazzy-arms-ros2-control|ros-jazzy-arms-ros2-control-full)
      case "$ARMS_VARIANT" in
        standard) echo "ros-jazzy-arms-ros2-control" ;;
        full) echo "ros-jazzy-arms-ros2-control-full" ;;
        *) echo "$prefix" ;;
      esac
      ;;
    *) echo "$prefix" ;;
  esac
}

# 短名 → 短名（用于 --only 过滤；arms 统一为 arms，安装时再按变体解析实际前缀）
resolve_only_token() {
  case "$1" in
    ocs2|ocs2_ros2|ros-jazzy-ocs2) echo "ocs2" ;;
    common|robot-descriptions-common|ros-jazzy-robot-descriptions-common) echo "common" ;;
    arms|arms_ros2_control|arms_full|arms-full|ros-jazzy-arms-ros2-control|ros-jazzy-arms-ros2-control-full) \
      echo "arms" ;;
    arx|arx_ros2_control|arx-ros2-control) echo "arms" ;;  # 兼容短名；实际 HI 仍为源码
    *) return 1 ;;
  esac
}

# conf 中的包前缀 → 短名（用于与 --only 过滤比较）
prefix_to_short() {
  case "$1" in
    ros-jazzy-ocs2) echo "ocs2" ;;
    ros-jazzy-robot-descriptions-common) echo "common" ;;
    ros-jazzy-arms-ros2-control|ros-jazzy-arms-ros2-control-full) echo "arms" ;;
    *) echo "$1" ;;
  esac
}

usage() {
  cat <<EOF
用法: install_core_debs.sh [--ros-distro <distro>] [--config <file>]
                          [--only <list>] [--channel <latest|pre-release>]
                          [--arms-variant <full|standard>]

从 deb_versions.conf 读取仓库信息，按依赖顺序安装核心 deb 包：
  1. ros-jazzy-ocs2
  2. ros-jazzy-robot-descriptions-common
  3. ros-jazzy-arms-ros2-control（控制器栈；真机 HI 用 src/arx-ros2-control）

  --only <list>     仅安装指定包，逗号分隔。可用短名：ocs2, common, arms
  --channel <name>  发布通道（覆盖 conf 中的固定 tag）：
                    latest       GitHub Latest（稳定版，排除 pre-release）
                    pre-release  各仓库的 pre-release 浮动标签
                    未指定时按 conf 中的 tag 安装。
  --arms-variant <full|standard>
                    arms deb 变体（覆盖 deb_versions.conf 中的前缀，不修改配置）：
                    standard   ros-jazzy-arms-ros2-control（默认）
                    full       ros-jazzy-arms-ros2-control-full（含 ht 等，仍不含 arx）

也可通过环境变量 LIFT2S_DEB_CHANNEL 或 HT_DEB_CHANNEL=latest|pre-release 指定。
安装使用 apt-get install，自动处理依赖。
EOF
}

ONLY_FILTER=()
DEB_CHANNEL=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --config) DEB_CONF="$2"; shift 2 ;;
    --channel)
      DEB_CHANNEL="$(trim "$2")"
      case "$DEB_CHANNEL" in
        latest|stable) DEB_CHANNEL="latest" ;;
        pre-release|prerelease|pre) DEB_CHANNEL="pre-release" ;;
        *)
          print_error "未知 --channel: $2（可用: latest, pre-release）"
          exit 1
          ;;
      esac
      shift 2
      ;;
    --arms-variant)
      ARMS_VARIANT="$(trim "$2")"
      case "$ARMS_VARIANT" in
        full|standard) ;;
        *)
          print_error "未知 --arms-variant: $2（可用: full, standard）"
          exit 1
          ;;
      esac
      shift 2
      ;;
    --only)
      IFS=',' read -ra _only_tokens <<< "$2"
      for _tok in "${_only_tokens[@]}"; do
        _tok="$(trim "$_tok")"
        [[ -z "$_tok" ]] && continue
        if ! _resolved="$(resolve_only_token "$_tok")"; then
          print_error "未知 --only 项: $_tok（可用: ocs2, common, arms）"
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

# init_repo.sh 可通过环境变量传入通道
if [[ -z "$DEB_CHANNEL" && -n "${LIFT2S_DEB_CHANNEL:-${HT_DEB_CHANNEL:-}}" ]]; then
  _ch="$(trim "${LIFT2S_DEB_CHANNEL:-${HT_DEB_CHANNEL}}")"
  case "${_ch}" in
    latest|stable) DEB_CHANNEL="latest" ;;
    pre-release|prerelease|pre) DEB_CHANNEL="pre-release" ;;
    *)
      print_error "未知 LIFT2S_DEB_CHANNEL/HT_DEB_CHANNEL: ${_ch}（可用: latest, pre-release）"
      exit 1
      ;;
  esac
fi

should_install_prefix() {
  local prefix="$1" short f
  if [[ ${#ONLY_FILTER[@]} -eq 0 ]]; then
    return 0
  fi
  short="$(prefix_to_short "$prefix")"
  for f in "${ONLY_FILTER[@]}"; do
    [[ "$f" == "$short" ]] && return 0
  done
  return 1
}

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

# arms 标准版与 full 文件路径重叠，安装前卸掉对立包
prepare_arms_variant_install() {
  local target="$1"
  local other conflict_choice
  case "$target" in
    ros-jazzy-arms-ros2-control) other="ros-jazzy-arms-ros2-control-full" ;;
    ros-jazzy-arms-ros2-control-full) other="ros-jazzy-arms-ros2-control" ;;
    *) return 0 ;;
  esac
  is_pkg_installed "$other" || return 0

  print_warn "安装 ${target} 前需先卸载对立包（路径重叠）：${other}"
  read -rp "是否自动卸载 ${other}？[Y/n]: " conflict_choice
  case "$conflict_choice" in
    n|N|no|NO)
      print_error "已取消安装。请手动执行: sudo apt-get remove ${other}"
      exit 1
      ;;
  esac
  print_info "卸载对立包: ${other} ..."
  apt_run remove -y "$other" || {
    print_error "卸载 ${other} 失败，请手动删除后重试"
    exit 1
  }
  echo "" >&2
}

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
    ros-jazzy-arms-ros2-control-full)
      # Lift2S 默认 standard；full 仍可选
      printf '%s\n' \
        "${prefix}_*_${DEB_ARCH}.deb"
      ;;
    ros-jazzy-arms-ros2-control)
      # 兼容旧配置：标准包需排除 *-full*
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
    ros-jazzy-arms-ros2-control-full)
      [[ "$asset" == *"-full_"* || "$asset" == ros-jazzy-arms-ros2-control-full* ]] && return 1
      return 0
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

fetch_release_assets_from_html() {
  local repo="$1" tag="$2"
  curl -fsSL "https://github.com/${repo}/releases/expanded_assets/${tag}" 2>/dev/null \
    | grep -oE '[A-Za-z0-9._~-]+'"_${DEB_ARCH}"'\.deb' \
    | sort -u \
    | while IFS= read -r name; do
        [[ -n "$name" ]] && printf '%s|\n' "$name"
      done
}

fetch_release_assets_with_size() {
  local repo="$1" tag="$2"
  local api_out=""
  api_out="$(github_curl "https://api.github.com/repos/${repo}/releases/tags/${tag}" 2>/dev/null \
    | python3 -c "
import json, sys
data = json.load(sys.stdin)
for a in data.get('assets', []):
    print(str(a['name']) + '|' + str(a['size']))
" 2>/dev/null || true)"
  if [[ -n "$api_out" ]]; then
    printf '%s\n' "$api_out"
    return 0
  fi
  print_warn "  GitHub API 不可用，改从 Release 页面解析 deb 列表（${repo}@${tag}）"
  fetch_release_assets_from_html "$repo" "$tag"
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

fetch_release_assets() {
  local repo="$1" tag="$2"
  fetch_release_assets_with_size "$repo" "$tag" | cut -d'|' -f1
}

release_exists() {
  local repo="$1" tag="$2"
  local code=""
  code="$(github_curl -o /dev/null -w '%{http_code}' "https://api.github.com/repos/${repo}/releases/tags/${tag}" 2>/dev/null || true)"
  [[ "$code" == "200" ]] && return 0
  code="$(curl -fsSL -o /dev/null -w '%{http_code}' "https://github.com/${repo}/releases/tag/${tag}" 2>/dev/null || true)"
  [[ "$code" == "200" ]]
}

# 稳定版 Latest（排除 draft / prerelease）
fetch_latest_release_tag() {
  local repo="$1" tag=""
  tag="$(github_curl "https://api.github.com/repos/${repo}/releases/latest" 2>/dev/null \
    | python3 -c 'import json,sys; print(json.load(sys.stdin).get("tag_name",""))' 2>/dev/null || true)"
  if [[ -n "$tag" ]]; then
    printf '%s\n' "$tag"
    return 0
  fi
  tag="$(github_curl "https://api.github.com/repos/${repo}/releases?per_page=30" 2>/dev/null \
    | python3 -c '
import json, sys
releases = json.load(sys.stdin)
for r in releases:
    if r.get("draft") or r.get("prerelease"):
        continue
    tag = r.get("tag_name", "")
    if tag:
        print(tag)
        break
' 2>/dev/null || true)"
  if [[ -n "$tag" ]]; then
    printf '%s\n' "$tag"
    return 0
  fi
  # 无 token 时 API 常 403；用 /releases/latest 重定向解析 tag（GitHub 稳定版）
  tag="$(curl -fsSL -o /dev/null -w '%{url_effective}' "https://github.com/${repo}/releases/latest" 2>/dev/null \
    | sed -n 's|.*/tag/||p')"
  if [[ -n "$tag" ]]; then
    print_warn "  GitHub API 限流，已通过 releases/latest 解析 tag: ${tag}" >&2
    printf '%s\n' "$tag"
    return 0
  fi
  return 1
}

# 预发布：优先固定浮动标签 pre-release，否则取最新 prerelease
fetch_prerelease_tag() {
  local repo="$1" tag=""
  if release_exists "$repo" "pre-release"; then
    printf '%s\n' "pre-release"
    return 0
  fi
  tag="$(github_curl "https://api.github.com/repos/${repo}/releases?per_page=30" 2>/dev/null \
    | python3 -c '
import json, sys
releases = json.load(sys.stdin)
for r in releases:
    if r.get("draft") or not r.get("prerelease"):
        continue
    tag = r.get("tag_name", "")
    if tag:
        print(tag)
        break
' 2>/dev/null || true)"
  if [[ -n "$tag" ]]; then
    printf '%s\n' "$tag"
    return 0
  fi
  return 1
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

  # --channel 覆盖 conf 中的固定 tag
  case "${DEB_CHANNEL:-}" in
    latest) requested_tag="latest" ;;
    pre-release) requested_tag="pre-release" ;;
  esac

  if [[ "$requested_tag" == "pre-release" || "$requested_tag" == "prerelease" || "$requested_tag" == "pre" ]]; then
    tag="$(fetch_prerelease_tag "$primary_repo")"
    if [[ -n "$tag" ]]; then
      print_info "  ${label}: 通道 pre-release → ${primary_repo}@${tag}"
      printf '%s\n' "$tag"
      return 0
    fi
    print_error "无法解析 ${label} 的 pre-release（仓库: ${primary_repo}）"
    return 1
  fi

  if [[ -n "$requested_tag" && "$requested_tag" != "latest" && "$requested_tag" != "*" ]]; then
    if [[ "$requested_tag" != [vV]* && "$requested_tag" != pre-release ]]; then
      requested_tag="v${requested_tag}"
    fi
    printf '%s\n' "$requested_tag"
    return 0
  fi

  tag="$(fetch_latest_release_tag "$primary_repo")"
  if [[ -n "$tag" ]]; then
    print_info "  ${label}: 通道 latest → ${primary_repo}@${tag}"
    printf '%s\n' "$tag"
    return 0
  fi

  print_error "无法解析 ${label} 的最新 release tag（仓库: ${primary_repo}）"
  return 1
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

  prune_deb_cache "$DOWNLOAD_DIR" "$prefix" "$matched"

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
print_info "安装顺序: ocs2 → common → arms_ros2_control(-full)"
if [[ -n "${DEB_CHANNEL:-}" ]]; then
  print_info "发布通道: ${DEB_CHANNEL}（覆盖 conf 固定 tag）"
else
  print_info "发布通道: conf（使用 deb_versions.conf 中的固定 tag）"
fi
if [[ -n "${ARMS_VARIANT:-}" ]]; then
  print_info "arms 变体: ${ARMS_VARIANT}"
fi
print_info "配置文件: $DEB_CONF"
if [[ ${#ONLY_FILTER[@]} -gt 0 ]]; then
  print_info "仅安装: ${ONLY_FILTER[*]}"
fi
echo "" >&2

installed=()
step=0
total_to_install=0
for spec in "${DEB_SPECS[@]}"; do
  IFS='|' read -r prefix _tag _repo <<< "$spec"
  prefix="$(trim "$prefix")"
  should_install_prefix "$prefix" && total_to_install=$((total_to_install + 1))
done
if [[ "$total_to_install" -eq 0 ]]; then
  print_error "没有匹配 --only 的包可安装"
  exit 1
fi

for spec in "${DEB_SPECS[@]}"; do
  IFS='|' read -r prefix tag primary_repo <<< "$spec"
  prefix="$(trim "$prefix")"
  tag="$(trim "$tag")"
  primary_repo="$(trim "$primary_repo")"

  if ! should_install_prefix "$prefix"; then
    continue
  fi

  # arms 变体覆盖 conf 中的前缀（--arms-variant，不改 deb_versions.conf）
  eff_prefix="$(resolve_effective_prefix "$prefix")"

  step=$((step + 1))
  print_info "[${step}/${total_to_install}] 处理 ${eff_prefix} ..."
  deb_path="$(download_deb_for_package "$eff_prefix" "$tag" "$primary_repo")" || exit 1
  if [[ "$eff_prefix" == "ros-jazzy-robot-descriptions-common" ]]; then
    prepare_common_deb_install
  fi
  if [[ "$eff_prefix" == "ros-jazzy-arms-ros2-control" || "$eff_prefix" == "ros-jazzy-arms-ros2-control-full" ]]; then
    prepare_arms_variant_install "$eff_prefix"
  fi
  install_deb "$deb_path" || {
    print_error "安装 ${eff_prefix} 失败"
    exit 1
  }
  installed+=("$eff_prefix")
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
