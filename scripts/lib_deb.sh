#!/usr/bin/env bash
# 核心 deb 配置 / 发布通道 / 下载 / 安装 / 卸载
# 依赖：lib_common.sh、lib_github.sh（需先 source）
# 全局配置（可被调用方覆盖后 source）：
#   DEB_CONF / ROS_DISTRO / DEB_CACHE_DIR
#   DEB_CHANNEL（conf|latest|pre-release，默认 conf）
#   ARMS_VARIANT（full|standard，默认 full；仅覆盖安装前缀，不改 deb_versions.conf）
#   PURGE（卸载时是否 purge，默认 0）
#   ONLY_FILTER（数组，短名过滤，由 set_only_filter 填充）

DEB_CONF="${DEB_CONF:-${REPO_DIR:-.}/deb_versions.conf}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DEB_ARCH="$(detect_machine_arch)"
DEB_CACHE_DIR="${DEB_CACHE_DIR:-${REPO_DIR:-.}/.deb_cache}"
DEB_CHANNEL="${DEB_CHANNEL:-conf}"
ARMS_VARIANT="${ARMS_VARIANT:-full}"
PURGE="${PURGE:-0}"
DEB_RELEASE_SOURCES=()
ONLY_FILTER=()

# 从 deb_versions.conf 加载: repo|display|prefix|tag（文件顺序即安装顺序）
load_deb_conf() {
  local line prefix tag repo display
  DEB_RELEASE_SOURCES=()

  if [ ! -f "${DEB_CONF}" ]; then
    print_error "未找到 ${DEB_CONF}"
    return 1
  fi

  while IFS= read -r line || [ -n "${line}" ]; do
    line="${line%%#*}"
    line="$(trim "${line}")"
    [ -z "${line}" ] && continue
    IFS='|' read -r prefix tag repo <<< "${line}"
    prefix="$(trim "${prefix}")"
    tag="$(trim "${tag}")"
    repo="$(trim "${repo}")"
    [ -z "${prefix}" ] || [ -z "${repo}" ] && continue
    [ -z "${tag}" ] && tag="latest"
    display="${prefix#ros-${ROS_DISTRO}-}"
    DEB_RELEASE_SOURCES+=("${repo}|${display}|${prefix}|${tag}")
  done < "${DEB_CONF}"

  if [ "${#DEB_RELEASE_SOURCES[@]}" -eq 0 ]; then
    print_error "deb_versions.conf 中没有有效条目"
    return 1
  fi
  return 0
}

# conf 中 arms 行的变体（full/standard）
conf_arms_prefix() {
  local line
  line="$(grep -E '^(ros-jazzy-arms-ros2-control|ros-jazzy-arms-ros2-control-full)\|' "${DEB_CONF}" 2>/dev/null | head -n1 || true)"
  case "${line}" in
    ros-jazzy-arms-ros2-control-full*) echo "full" ;;
    *) echo "standard" ;;
  esac
}

# 按 ARMS_VARIANT 返回实际安装前缀（仅覆盖本次安装，不改 deb_versions.conf）
resolve_arms_prefix() {
  local prefix="$1"
  case "${prefix}" in
    ros-jazzy-arms-ros2-control|ros-jazzy-arms-ros2-control-full)
      case "${ARMS_VARIANT:-full}" in
        standard) echo "ros-jazzy-arms-ros2-control" ;;
        *) echo "ros-jazzy-arms-ros2-control-full" ;;
      esac
      ;;
    *) echo "${prefix}" ;;
  esac
}

prefix_to_short() {
  case "$1" in
    ros-jazzy-ocs2) echo "ocs2" ;;
    ros-jazzy-robot-descriptions-common) echo "common" ;;
    ros-jazzy-arms-ros2-control|ros-jazzy-arms-ros2-control-full) echo "arms" ;;
    *) echo "$1" ;;
  esac
}

resolve_only_token() {
  case "$1" in
    ocs2|ocs2_ros2|ros-jazzy-ocs2) echo "ocs2" ;;
    common|robot-descriptions-common|ros-jazzy-robot-descriptions-common) echo "common" ;;
    arms|arms_ros2_control|arms_full|arms-full|ros-jazzy-arms-ros2-control|ros-jazzy-arms-ros2-control-full) echo "arms" ;;
    fairino|fairino_ros2_control|fairino-ros2-control) echo "arms" ;;
    *) return 1 ;;
  esac
}

# 设置 ONLY_FILTER；$1 = 逗号分隔短名列表（空 = 全部）
set_only_filter() {
  local list="$1" tok resolved
  ONLY_FILTER=()
  [ -z "${list}" ] && return 0
  IFS=',' read -ra _tokens <<< "${list}"
  for tok in "${_tokens[@]}"; do
    tok="$(trim "${tok}")"
    [ -z "${tok}" ] && continue
    if ! resolved="$(resolve_only_token "${tok}")"; then
      print_error "未知包名: ${tok}（可用: ocs2, common, arms）"
      return 1
    fi
    ONLY_FILTER+=("${resolved}")
  done
  return 0
}

should_install_prefix() {
  local prefix="$1" short f
  if [ "${#ONLY_FILTER[@]}" -eq 0 ]; then
    return 0
  fi
  short="$(prefix_to_short "${prefix}")"
  for f in "${ONLY_FILTER[@]}"; do
    [[ "${f}" == "${short}" ]] && return 0
  done
  return 1
}

# 设置 DEB_CHANNEL 并校验合法值
set_deb_channel() {
  local v="$1"
  case "$(trim "${v}")" in
    conf|config) DEB_CHANNEL="conf" ;;
    latest|stable) DEB_CHANNEL="latest" ;;
    pre-release|prerelease|pre) DEB_CHANNEL="pre-release" ;;
    *)
      print_error "未知通道: ${v}（可用: conf, latest, pre-release）"
      return 1
      ;;
  esac
  return 0
}

# 交互：选择 deb 发布通道（默认 conf，标 *）
prompt_channel() {
  local c
  c="$(menu_select "请选择deb发布通道" 1 "conf文件" "latest" "pre-release")"
  case "${c}" in
    2) DEB_CHANNEL="latest" ;;
    3) DEB_CHANNEL="pre-release" ;;
    *) DEB_CHANNEL="conf" ;;
  esac
  print_info "deb 通道: ${DEB_CHANNEL}"
}

# 交互：选择 arms deb 变体（默认 full）
# conf 通道：读取 conf 中 arms 变体，提示是否本次切换（默认不切换，仅本次生效）
prompt_arms_variant() {
  local c conf_var other
  if [ "${DEB_CHANNEL}" = "conf" ]; then
    conf_var="$(conf_arms_prefix)"
    if [ "${conf_var}" = "full" ]; then
      other="standard"
    else
      other="full"
    fi
    c="$(menu_select "conf 当前 arms 变体: ${conf_var}" 1 \
      "不切换（${conf_var}）" "切换为 ${other}（仅本次生效）")"
    if [ "${c}" = "2" ]; then
      ARMS_VARIANT="${other}"
    else
      ARMS_VARIANT="${conf_var}"
    fi
  else
    c="$(menu_select "请选择 arms 变体" 1 "full" "standard")"
    if [ "${c}" = "2" ]; then
      ARMS_VARIANT="standard"
    else
      ARMS_VARIANT="full"
    fi
  fi
  print_info "arms deb 变体: ${ARMS_VARIANT}"
}

# conf 固定版本 → 期望缓存文件名（prefix_ver_arch.deb）；非固定版本返回空
conf_expected_deb_name() {
  local prefix="$1" conf_tag="$2" arch="${3:-${DEB_ARCH}}"
  local ver
  ver="$(tag_to_deb_version "${conf_tag}")"
  if [ -n "${ver}" ]; then
    printf '%s_%s_%s.deb' "${prefix}" "${ver}" "${arch}"
  fi
}

# 按前缀 + 架构从资产名列表中挑选 deb（兼容旧命名；无 arch 包时回退 amd64）
# $1 = 架构  $2 = 前缀  其余 = 资产名（可带 |size，仅取 | 前部分）
select_asset_name() {
  local arch="$1" prefix="$2"; shift 2
  local name
  local -a candidates=()

  for name in "$@"; do
    name="${name%%|*}"
    case "${name}" in
      "${prefix}_"*"_${arch}.deb") candidates+=("${name}") ;;
    esac
  done
  if [ "${#candidates[@]}" -eq 0 ] && [ "${arch}" != "amd64" ]; then
    for name in "$@"; do
      name="${name%%|*}"
      case "${name}" in
        "${prefix}_"*"_amd64.deb") candidates+=("${name}") ;;
      esac
    done
  fi
  # 兼容旧命名：*prefix*_arch.deb
  if [ "${#candidates[@]}" -eq 0 ]; then
    for name in "$@"; do
      name="${name%%|*}"
      case "${name}" in
        *"${prefix}"*"_${arch}.deb"|*"${prefix}"*"_amd64.deb") candidates+=("${name}") ;;
      esac
    done
  fi
  if [ "${#candidates[@]}" -eq 0 ]; then
    return 1
  fi
  printf '%s\n' "${candidates[@]}" | sort -V | tail -n1
}

# 固定 tag：直接构造 releases/download URL 下载（不查 GitHub API，避免限流）
# $1 = owner/repo  $2 = 显示名  $3 = 包前缀  $4 = release tag  $5 = 版本段  $6 = 架构
download_deb_by_constructed_url() {
  local owner_repo="$1" display="$2" prefix="$3" conf_tag="$4" conf_ver="$5" arch="$6"
  local name url dest cand arch_label
  local -a candidates=("${prefix}_${conf_ver}_${arch}.deb|${arch}")

  if [ "${arch}" != "amd64" ]; then
    candidates+=("${prefix}_${conf_ver}_amd64.deb|amd64")
  fi

  for cand in "${candidates[@]}"; do
    name="${cand%%|*}"
    arch_label="${cand##*|}"
    url="https://github.com/${owner_repo}/releases/download/${conf_tag}/${name}"
    dest="${DEB_CACHE_DIR}/${name}"

    if [ -f "${dest}" ]; then
      print_info "${display}: 复用缓存 ${name} (${conf_tag})"
      prune_deb_cache "${DEB_CACHE_DIR}" "${prefix}" "${name}"
      printf '%s\n' "${dest}"
      return 0
    fi

    print_info "${display}: 直接下载 ${name} (${conf_tag}) ..."
    if download_file "${url}" "${dest}.part"; then
      if is_valid_deb_file "${dest}.part"; then
        mv "${dest}.part" "${dest}"
        print_info "已保存: ${dest}"
        prune_deb_cache "${DEB_CACHE_DIR}" "${prefix}" "${name}"
        printf '%s\n' "${dest}"
        return 0
      fi
      print_warn "${display}: 下载内容不是有效 deb，回退 API 查询: ${name}"
      rm -f "${dest}.part"
      return 1
    fi
    rm -f "${dest}.part"
    if [ "${arch_label}" != "${arch}" ]; then
      print_warn "${display}: 无 ${arch} 包，回退 amd64: ${name}"
    else
      print_warn "${display}: 下载失败: ${url}"
    fi
  done

  return 1
}

# 解析通道对应的实际 release tag
# $1 = 通道解析后的 tag（conf_tag / latest / pre-release）  $2 = owner/repo  $3 = 显示名
resolve_release_tag() {
  local requested_tag="$1" repo="$2" label="$3"
  local tag=""
  case "${requested_tag}" in
    pre-release|prerelease|pre)
      tag="$(fetch_prerelease_tag "${repo}")" || {
        print_error "无法解析 ${label} 的 pre-release（仓库: ${repo}）"
        return 1
      }
      ;;
    latest|"*")
      tag="$(fetch_latest_release_tag "${repo}")" || {
        print_error "无法解析 ${label} 的最新 release tag（仓库: ${repo}）"
        return 1
      }
      ;;
    *)
      tag="${requested_tag}"
      [[ "${tag}" == [vV]* ]] || tag="v${tag}"
      ;;
  esac
  printf '%s\n' "${tag}"
  return 0
}

# 下载一个包的 deb（按 DEB_CHANNEL 与 ARMS_VARIANT），成功输出 deb 路径
# $1 = entry（repo|display|prefix|conf_tag）  $2 = 目标架构（可选，默认本机）
download_deb_for_source() {
  local entry="$1"
  local arch="${2:-${DEB_ARCH}}"
  local owner_repo display prefix conf_tag
  local eff_prefix conf_ver expected deb_path url tag resolved_tag
  local -a assets=()
  local matched expected_size=""

  IFS='|' read -r owner_repo display prefix conf_tag <<< "${entry}"
  eff_prefix="$(resolve_arms_prefix "${prefix}")"
  mkdir -p "${DEB_CACHE_DIR}"

  # 通道 → tag
  case "${DEB_CHANNEL}" in
    latest) tag="latest" ;;
    pre-release) tag="pre-release" ;;
    *) tag="${conf_tag}" ;;
  esac

  # conf 固定版本：按构造文件名检查缓存；未命中则直接拼 URL 下载，失败再回退 API
  conf_ver="$(tag_to_deb_version "${tag}")"
  if [ -n "${conf_ver}" ]; then
    expected="${DEB_CACHE_DIR}/${eff_prefix}_${conf_ver}_${arch}.deb"
    if [ -f "${expected}" ]; then
      print_info "${display}: 复用缓存 $(basename "${expected}") (${tag})"
      prune_deb_cache "${DEB_CACHE_DIR}" "${eff_prefix}" "$(basename "${expected}")"
      printf '%s\n' "${expected}"
      return 0
    fi
    if actual="$(download_deb_by_constructed_url "${owner_repo}" "${display}" "${eff_prefix}" "${tag}" "${conf_ver}" "${arch}")"; then
      printf '%s\n' "${actual}"
      return 0
    fi
    print_warn "${display}: 直接 URL 下载失败，回退 GitHub API 查询 ..."
  fi

  # latest / pre-release（或 API 回退）：仅通过 GitHub API 查询下载地址
  print_info "查询 ${display} (${owner_repo}@${tag}) ..."
  resolved_tag="$(resolve_release_tag "${tag}" "${owner_repo}" "${display}")" || return 1
  tag="${resolved_tag}"
  if ! release_exists "${owner_repo}" "${tag}"; then
    print_error "${display}: ${owner_repo}@${tag} 不存在"
    return 1
  fi

  mapfile -t assets < <(fetch_release_assets_with_size "${owner_repo}" "${tag}")
  if [ "${#assets[@]}" -eq 0 ]; then
    print_error "${display}: ${owner_repo}@${tag} 无 release 资产"
    return 1
  fi

  matched="$(select_asset_name "${arch}" "${eff_prefix}" "${assets[@]}")" || {
    print_error "${display}: ${owner_repo}@${tag} 未找到匹配 deb（前缀: ${eff_prefix}, 架构: ${arch}）"
    return 1
  }
  expected_size="$(lookup_asset_size "${matched}" "${assets[@]}")" || expected_size=""

  deb_path="${DEB_CACHE_DIR}/${matched}"
  if [ -f "${deb_path}" ]; then
    if is_valid_deb_file "${deb_path}" "${expected_size}"; then
      print_info "${display}: 复用缓存 ${matched} (${tag})"
      prune_deb_cache "${DEB_CACHE_DIR}" "${eff_prefix}" "${matched}"
      printf '%s\n' "${deb_path}"
      return 0
    fi
    print_warn "${display}: 缓存文件不完整或已损坏，将重新下载..."
    rm -f "${deb_path}" "${deb_path}.part"
  fi

  url="https://github.com/${owner_repo}/releases/download/${tag}/${matched}"
  print_info "下载 ${matched} (${tag}) ..."
  if ! download_file_with_progress "${url}" "${deb_path}.part" "${matched}"; then
    print_error "下载失败: ${url}"
    rm -f "${deb_path}.part"
    return 1
  fi
  if ! is_valid_deb_file "${deb_path}.part" "${expected_size}"; then
    print_error "下载完成但文件校验失败（可能网络中断），请重试"
    rm -f "${deb_path}.part"
    return 1
  fi
  mv -f "${deb_path}.part" "${deb_path}"
  print_info "已保存: ${deb_path}"
  prune_deb_cache "${DEB_CACHE_DIR}" "${eff_prefix}" "${matched}"
  printf '%s\n' "${deb_path}"
  return 0
}

# 打包通道非 conf 时：把 deb_versions.conf 的 tag 改为与打包通道一致（保持注释不变）
# $1 = conf 文件路径  $2 = 通道（latest / pre-release）
rewrite_conf_tag_to_channel() {
  local conf_file="$1" channel="$2"
  python3 - "${conf_file}" "${channel}" <<'PY'
import sys
path, ch = sys.argv[1], sys.argv[2]
lines = open(path, encoding="utf-8").read().splitlines()
out = []
for line in lines:
    s = line.strip()
    if not s or s.startswith("#"):
        out.append(line)
        continue
    parts = line.split("|")
    if len(parts) >= 2:
        parts[1] = ch
        out.append("|".join(parts))
    else:
        out.append(line)
open(path, "w", encoding="utf-8").write("\n".join(out) + "\n")
PY
  print_info "zip 内 deb_versions.conf 通道已改为: ${channel}"
}

# 与 ros-jazzy-robot-descriptions-common 文件路径冲突的旧版独立 deb
COMMON_CONFLICT_PACKAGES=(
  "ros-${ROS_DISTRO}-robotiq-description"
)

# arms 标准版与 full 文件路径重叠，安装前卸掉对立包
prepare_arms_variant_install() {
  local target="$1"
  local other conflict_choice
  case "${target}" in
    ros-jazzy-arms-ros2-control) other="ros-jazzy-arms-ros2-control-full" ;;
    ros-jazzy-arms-ros2-control-full) other="ros-jazzy-arms-ros2-control" ;;
    *) return 0 ;;
  esac
  is_pkg_installed "${other}" || return 0

  print_warn "安装 ${target} 前需先卸载对立包（路径重叠）：${other}"
  read -rp "是否自动卸载 ${other}？[Y/n]: " conflict_choice
  case "${conflict_choice}" in
    n|N|no|NO)
      print_error "已取消安装。请手动执行: sudo apt-get remove ${other}"
      exit 1
      ;;
  esac
  print_info "卸载对立包: ${other} ..."
  apt_run remove -y "${other}" || {
    print_error "卸载 ${other} 失败，请手动删除后重试"
    exit 1
  }
  echo "" >&2
}

prepare_common_deb_install() {
  local pkg conflict_choice
  local -a installed_conflicts=()

  for pkg in "${COMMON_CONFLICT_PACKAGES[@]}"; do
    is_pkg_installed "${pkg}" && installed_conflicts+=("${pkg}")
  done

  [[ ${#installed_conflicts[@]} -eq 0 ]] && return 0

  print_warn "安装 ros-jazzy-robot-descriptions-common 前需先卸载以下冲突包（安装路径重叠）："
  for pkg in "${installed_conflicts[@]}"; do
    print_warn "  - ${pkg}"
  done
  print_warn "common deb 已包含 robotiq 等公共描述，保留旧包会导致 apt 安装失败。"
  read -rp "是否自动卸载上述包？[Y/n]: " conflict_choice
  case "${conflict_choice}" in
    n|N|no|NO)
      print_error "已取消安装。请手动执行: sudo apt-get remove ${installed_conflicts[*]}"
      exit 1
      ;;
  esac

  for pkg in "${installed_conflicts[@]}"; do
    print_info "卸载冲突包: ${pkg} ..."
    apt_run remove -y "${pkg}" || {
      print_error "卸载 ${pkg} 失败，请手动删除后重试"
      exit 1
    }
  done
  echo "" >&2
}

install_deb() {
  local deb_file="$1"
  print_info "安装 $(basename "${deb_file}") ..."
  # --allow-downgrades：支持在已装更高版本时降级到通道/conf 指定版本
  if apt_run install --allow-downgrades -y "${deb_file}"; then
    return 0
  fi
  print_warn "安装失败，删除可能损坏的缓存以便下次重新下载: ${deb_file}"
  rm -f "${deb_file}" "${deb_file}.part"
  return 1
}

# 按通道下载并安装核心 deb（安装顺序: ocs2 → common → arms）
# $1 = only 列表（可选，逗号分隔；空 = 全部）
install_core_debs() {
  local only_list="${1:-}"
  local entry owner_repo display prefix conf_tag eff_prefix deb_path
  local -a installed=()
  local step=0 total=0

  load_deb_conf || return 1
  if ! set_only_filter "${only_list}"; then
    return 1
  fi

  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r _owner_repo _display prefix _tag <<< "${entry}"
    should_install_prefix "$(trim "${prefix}")" && total=$((total + 1))
  done
  if [ "${total}" -eq 0 ]; then
    print_error "没有匹配的包可安装"
    return 1
  fi

  print_info "安装核心 deb（通道: ${DEB_CHANNEL}，架构: ${DEB_ARCH}，ROS ${ROS_DISTRO}）"
  if [ "${#ONLY_FILTER[@]}" -gt 0 ]; then
    print_info "仅安装: ${ONLY_FILTER[*]}"
  fi
  echo "" >&2

  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r owner_repo display prefix conf_tag <<< "${entry}"
    prefix="$(trim "${prefix}")"
    conf_tag="$(trim "${conf_tag}")"
    should_install_prefix "${prefix}" || continue

    eff_prefix="$(resolve_arms_prefix "${prefix}")"
    step=$((step + 1))
    print_info "[${step}/${total}] 处理 ${eff_prefix} ..."
    deb_path="$(download_deb_for_source "${owner_repo}|${display}|${prefix}|${conf_tag}")" || return 1

    if [[ "${eff_prefix}" == "ros-jazzy-robot-descriptions-common" ]]; then
      prepare_common_deb_install
    fi
    if [[ "${eff_prefix}" == "ros-jazzy-arms-ros2-control" || "${eff_prefix}" == "ros-jazzy-arms-ros2-control-full" ]]; then
      prepare_arms_variant_install "${eff_prefix}"
    fi
    install_deb "${deb_path}" || {
      print_error "安装 ${eff_prefix} 失败"
      return 1
    }
    installed+=("${eff_prefix}")
    echo "" >&2
  done

  print_info "核心 deb 安装完成：${installed[*]}"
  return 0
}

# 按逆序卸载核心 deb（arms-full → arms-standard → common → ocs2）
# $1 = only 列表（可选；空 = 全部）
uninstall_core_debs() {
  local only_list="${1:-}"
  local pkg short i=1 total
  local -a removed=() skipped=()
  local remove_order=(
    "ros-${ROS_DISTRO}-arms-ros2-control-full"
    "ros-${ROS_DISTRO}-arms-ros2-control"
    "ros-${ROS_DISTRO}-robot-descriptions-common"
    "ros-${ROS_DISTRO}-ocs2"
  )

  if ! set_only_filter "${only_list}"; then
    return 1
  fi

  total="${#remove_order[@]}"
  print_info "按逆序卸载核心 deb..."
  if [ "${#ONLY_FILTER[@]}" -gt 0 ]; then
    print_info "仅卸载: ${ONLY_FILTER[*]}"
  fi

  local -a rm_cmd=(remove -y)
  if [ "${PURGE}" -eq 1 ]; then
    rm_cmd=(purge -y)
  fi

  for pkg in "${remove_order[@]}"; do
    short="$(prefix_to_short "${pkg}")"
    # --only 过滤（arms 两个变体共享 short=arms，会一并处理）
    if [ "${#ONLY_FILTER[@]}" -gt 0 ]; then
      local f ok=0
      for f in "${ONLY_FILTER[@]}"; do
        [ "${f}" = "${short}" ] && ok=1
      done
      [ "${ok}" -eq 1 ] || continue
    fi
    print_info "[${i}/${total}] 卸载 ${pkg} ..."
    if is_pkg_installed "${pkg}"; then
      if apt_run "${rm_cmd[@]}" "${pkg}"; then
        removed+=("${pkg}")
      else
        print_error "卸载 ${pkg} 失败"
        return 1
      fi
    else
      print_warn "未安装，跳过: ${pkg}"
      skipped+=("${pkg}")
    fi
    i=$((i + 1))
  done

  if [ "${#removed[@]}" -gt 0 ]; then
    print_info "已卸载: ${removed[*]}"
  fi
  if [ "${#skipped[@]}" -gt 0 ]; then
    print_info "已跳过（未安装）: ${skipped[*]}"
  fi
  return 0
}

# 显示当前已安装的核心 deb 包及其版本
show_installed_core_debs() {
  local -a pkgs=(
    "ros-${ROS_DISTRO}-ocs2"
    "ros-${ROS_DISTRO}-robot-descriptions-common"
    "ros-${ROS_DISTRO}-arms-ros2-control-full"
    "ros-${ROS_DISTRO}-arms-ros2-control"
  )
  local pkg found=0
  for pkg in "${pkgs[@]}"; do
    if is_pkg_installed "${pkg}"; then
      printf '  - %s (%s)\n' "${pkg}" "$(pkg_version "${pkg}")"
      found=1
    fi
  done
  [ "${found}" -eq 1 ] || echo "  （未安装任何核心 deb 包）"
}
