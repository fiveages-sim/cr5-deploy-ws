#!/usr/bin/env bash
# deb 缓存公共逻辑（release.sh / quick_start.sh 共用）
# 用法：先设置 WS_DIR，再 source 本文件

: "${WS_DIR:?WS_DIR must be set before sourcing deb_cache_lib.sh}"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DEB_CACHE_DIR="${DEB_CACHE_DIR:-${WS_DIR}/.deb_cache}"

# owner/repo|显示名|deb 文件名前缀（匹配 ${prefix}_*_${arch}.deb）
# Lift2S：arms 用 standard（不含厂商 HI）；真机 HI 见发布包内 src/arx-ros2-control 源码
DEB_RELEASE_SOURCES=(
  "legubiao/ocs2_ros2|ocs2_ros2|ros-${ROS_DISTRO}-ocs2"
  "fiveages-sim/robot-descriptions-common|robot-descriptions-common|ros-${ROS_DISTRO}-robot-descriptions-common"
  "fiveages-sim/arms_ros2_control|arms-ros2-control|ros-${ROS_DISTRO}-arms-ros2-control"
)
ARMS_ROS2_CONTROL_DEB_PREFIX="ros-${ROS_DISTRO}-arms-ros2-control"

_deb_cache_warn() {
  if declare -F print_warn >/dev/null 2>&1; then
    print_warn "$1"
  else
    echo "[WARN] $1" >&2
  fi
}

detect_machine_arch() {
  local arch
  arch="$(dpkg --print-architecture 2>/dev/null || true)"
  case "${arch}" in
    amd64|arm64) echo "${arch}" ;;
    x86_64) echo "amd64" ;;
    aarch64) echo "arm64" ;;
    *)
      _deb_cache_warn "无法识别架构 (${arch:-unknown})，默认使用 amd64"
      echo "amd64"
      ;;
  esac
}

# 同一前缀+架构只保留指定 deb，删除 cache_dir 中的旧版本
prune_deb_cache_prefix() {
  local prefix="$1"
  local arch="$2"
  local keep_path="$3"
  local cache_dir="${4:-${DEB_CACHE_DIR}}"
  local keep_bn f

  keep_bn="$(basename "${keep_path}")"
  shopt -s nullglob
  for f in "${cache_dir}/${prefix}"_*_"${arch}".deb; do
    if [[ "$(basename "${f}")" != "${keep_bn}" ]]; then
      if declare -F print_info >/dev/null 2>&1; then
        print_info "移除旧 deb: $(basename "${f}")"
      fi
      rm -f "${f}"
    fi
  done
  shopt -u nullglob
}

deb_cache_channel_marker() {
  printf '%s/.release_channel' "${1}"
}

_deb_cache_has_debs() {
  local cache_dir="$1"
  local -a matches=()

  [ -d "${cache_dir}" ] || return 1
  shopt -s nullglob
  matches=("${cache_dir}"/*.deb)
  shopt -u nullglob
  [ "${#matches[@]}" -gt 0 ]
}

clear_deb_cache_debs() {
  local cache_dir="$1"
  local f

  [ -d "${cache_dir}" ] || return 0
  shopt -s nullglob
  for f in "${cache_dir}"/*.deb; do
    rm -f "${f}"
  done
  shopt -u nullglob
}

read_deb_cache_channel() {
  local cache_dir="$1"
  local marker

  marker="$(deb_cache_channel_marker "${cache_dir}")"
  if [ -f "${marker}" ]; then
    tr -d '[:space:]' < "${marker}"
  fi
}

write_deb_cache_channel() {
  local cache_dir="$1"
  local channel="$2"

  mkdir -p "${cache_dir}"
  printf '%s\n' "${channel}" > "$(deb_cache_channel_marker "${cache_dir}")"
}

# 来源通道变更时清空旧 deb（latest / prerelease 不混存）
ensure_deb_cache_channel() {
  local cache_dir="$1"
  local channel="$2"
  local old_channel

  [ -n "${cache_dir}" ] || return 0
  old_channel="$(read_deb_cache_channel "${cache_dir}")"
  if [ -n "${old_channel}" ] && [ "${old_channel}" != "${channel}" ] && _deb_cache_has_debs "${cache_dir}"; then
    if declare -F print_info >/dev/null 2>&1; then
      print_info "deb 来源通道变更 (${old_channel} → ${channel})，清空 ${cache_dir} 旧 deb"
    fi
    clear_deb_cache_debs "${cache_dir}"
  fi
}

# 将新 deb 同步回持久化缓存目录（发布打包时回写工作区 .deb_cache）
sync_deb_to_cache_dir() {
  local cache_dir="$1"
  local prefix="$2"
  local arch="$3"
  local src_path="$4"
  local dest_path bn

  [ -n "${cache_dir}" ] || return 0
  [ -f "${src_path}" ] || return 0

  bn="$(basename "${src_path}")"
  dest_path="${cache_dir}/${bn}"
  mkdir -p "${cache_dir}"
  if [ ! -f "${dest_path}" ] || ! cmp -s "${src_path}" "${dest_path}"; then
    cp "${src_path}" "${dest_path}"
    if declare -F print_info >/dev/null 2>&1; then
      print_info "已更新本地缓存: ${dest_path}"
    fi
  fi
  prune_deb_cache_prefix "${prefix}" "${arch}" "${dest_path}" "${cache_dir}"
}

# 查找 .deb_cache/ 中匹配前缀+架构的最新 deb 路径（stdout）
_resolve_cached_deb_path() {
  local prefix="$1"
  local arch="$2"
  local -a matches=()

  if [ ! -d "${DEB_CACHE_DIR}" ]; then
    return 1
  fi

  shopt -s nullglob
  matches=("${DEB_CACHE_DIR}/${prefix}"_*_"${arch}".deb)
  if [ "${#matches[@]}" -eq 0 ] && [ "${arch}" != "amd64" ]; then
    matches=("${DEB_CACHE_DIR}/${prefix}"_*_amd64.deb)
  fi
  shopt -u nullglob

  if [ "${#matches[@]}" -eq 0 ]; then
    return 1
  fi

  if [ "${#matches[@]}" -gt 1 ]; then
    mapfile -t matches < <(printf '%s\n' "${matches[@]}" | sort -V)
  fi
  printf '%s' "${matches[-1]}"
}

# 从 .deb_cache/ 中 deb 文件名解析版本（${prefix}_<version>_<arch>.deb）
resolve_cached_deb_version() {
  local prefix="$1"
  local arch="$2"
  local deb_path bn version file_arch

  deb_path="$(_resolve_cached_deb_path "${prefix}" "${arch}")" || return 1
  bn="$(basename "${deb_path}" .deb)"
  if [[ "${bn}" == *_amd64 ]]; then
    file_arch="amd64"
  elif [[ "${bn}" == *_arm64 ]]; then
    file_arch="arm64"
  else
    return 1
  fi
  bn="${bn%_${file_arch}}"
  version="${bn#${prefix}_}"
  if [ -z "${version}" ] || [ "${version}" = "${bn}" ]; then
    return 1
  fi
  printf '%s' "${version}"
}

resolve_cached_deb_basename() {
  local prefix="$1"
  local arch="$2"
  local deb_path

  deb_path="$(_resolve_cached_deb_path "${prefix}" "${arch}")" || return 1
  basename "${deb_path}"
}

deb_cache_has_any() {
  local arch entry prefix
  local version

  [ -d "${DEB_CACHE_DIR}" ] || return 1
  arch="$(detect_machine_arch)"
  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r _owner_repo _display prefix <<< "${entry}"
    version="$(resolve_cached_deb_version "${prefix}" "${arch}" 2>/dev/null || true)"
    if [ -n "${version}" ]; then
      return 0
    fi
  done
  return 1
}

# quick_start 版本查询：打印 .deb_cache/ 中各组件 deb 版本
print_deb_cache_versions_table() {
  local blue="${1:-}"
  local nc="${2:-}"
  local yellow="${3:-}"
  local arch entry display prefix version deb_name
  local found=0

  deb_cache_has_any || return 1

  arch="$(detect_machine_arch)"
  echo ""
  echo -e "${blue}================ deb 缓存版本 (.deb_cache/) ================${nc}"
  printf "%-32s | %-16s | %s\n" "Component" "Version" "Deb File"
  printf "%-32s-+-%-16s-+-%s\n" "--------------------------------" "----------------" "----------------------------------------"

  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r _owner_repo display prefix <<< "${entry}"
    version="$(resolve_cached_deb_version "${prefix}" "${arch}" 2>/dev/null || true)"
    deb_name="$(resolve_cached_deb_basename "${prefix}" "${arch}" 2>/dev/null || true)"
    if [ -n "${version}" ]; then
      found=1
      printf "%-32s | %-16s | %s\n" "${display}" "${version}" "${deb_name}"
    else
      printf "%-32s | %-16s | %s\n" "${display}" "-" "${yellow}未缓存${nc}"
    fi
  done

  if [ "${found}" = "0" ]; then
    return 1
  fi

  local cache_channel
  cache_channel="$(read_deb_cache_channel "${DEB_CACHE_DIR}")"
  if [ -n "${cache_channel}" ]; then
    echo -e "${blue}  来源通道: ${cache_channel}${nc}"
  fi
  echo -e "${blue}  架构: ${arch}    目录: ${DEB_CACHE_DIR}${nc}"
  echo -e "${blue}===========================================================${nc}"
  return 0
}
