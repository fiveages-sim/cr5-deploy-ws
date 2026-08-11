#!/usr/bin/env bash

# ARX Lift2S / ACone 工作空间 — deb 安装与发布打包脚本
# 使用方：解压发布 zip → ./release.sh --install → ./quick_start.sh
#         --package 含 .git（可 git pull）；--package-no-git 为纯快照包（更小）
# deb 版本与仓库以 deb_versions.conf 为准（非 GitHub Latest）
# deb 安装顺序: ocs2 → common → arms-ros2-control-full（含 arx_ros2_control）
#
# 用法:
#   ./release.sh              # 交互式菜单
#   ./release.sh --download   # 按 deb_versions.conf 下载/校验 deb 到 .deb_cache/
#   ./release.sh --install    # 从 .deb_cache/ 安装 deb（需 sudo）
#   ./release.sh --uninstall  # 按逆序卸载系统中的 deb 包（需 sudo）
#   ./release.sh --package           # 维护者：打包 zip（含 .git）
#   ./release.sh --package-no-git    # 维护者：打包 zip（不含 .git，体积更小）
#   ./release.sh --list       # 维护者：查看 conf 对应 Release deb 信息

set -u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEB_CACHE_DIR="${WS_DIR}/.deb_cache"
DIST_DIR="${WS_DIR}/dist"
DEB_CONF="${WS_DIR}/deb_versions.conf"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DEB_INSTALL_PATHS=()
RELEASE_CHANNEL="${RELEASE_CHANNEL:-conf}"

QS_CONFIG="${WS_DIR}/config/quick_start.conf"
_load_workspace_config() {
  if [[ ! -f "${QS_CONFIG}" ]]; then
    return 0
  fi
  # shellcheck source=/dev/null
  source "${QS_CONFIG}"
}
_load_workspace_config
RELEASE_WS_NAME="${RELEASE_WS_NAME:-lift2s-ws}"
RELEASE_GIT_BRANCH="${RELEASE_GIT_BRANCH:-main}"
RELEASE_ROBOT_NAME="${RELEASE_ROBOT_NAME:-arx_lift2s}"
if [[ ${#RELEASE_SUBMODULE_PATHS[@]} -eq 0 ]]; then
  echo "[ERROR] config/quick_start.conf 未定义 RELEASE_SUBMODULE_PATHS" >&2
  exit 1
fi
PACK_SOURCE_SUBMODULES=("${RELEASE_SUBMODULE_PATHS[@]}")

# 由 load_deb_versions_conf 从 deb_versions.conf 填充
# 每项: owner/repo|显示名|deb前缀|release tag
DEB_RELEASE_SOURCES=()

# 共用函数库（颜色输出 / GitHub API / 下载 / deb 校验等）
LIB_DEB_COMMON="${WS_DIR}/scripts/lib_deb_common.sh"
if [ ! -f "${LIB_DEB_COMMON}" ]; then
  echo "[ERROR] 未找到共用函数库: ${LIB_DEB_COMMON}" >&2
  exit 1
fi
# shellcheck source=scripts/lib_deb_common.sh
# shellcheck disable=SC1090
. "${LIB_DEB_COMMON}"

# 从 deb_versions.conf 加载：prefix|tag|repo → DEB_RELEASE_SOURCES
load_deb_versions_conf() {
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
    display="${prefix}"
    display="${display#ros-${ROS_DISTRO}-}"
    DEB_RELEASE_SOURCES+=("${repo}|${display}|${prefix}|${tag}")
  done < "${DEB_CONF}"

  if [ "${#DEB_RELEASE_SOURCES[@]}" -eq 0 ]; then
    print_error "deb_versions.conf 中没有有效条目"
    return 1
  fi
  print_info "已加载 deb_versions.conf（${#DEB_RELEASE_SOURCES[@]} 个包）"
  return 0
}

prompt_package_arch() {
  local choice
  echo "" >&2
  echo "请选择发布包 deb / zip 目标架构:" >&2
  echo "  1) amd64 (x64)" >&2
  echo "  2) arm64 (ARM)" >&2
  echo "  0) 取消" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  case "${choice}" in
    0) echo "" ;;
    1) echo "amd64" ;;
    2) echo "arm64" ;;
    *)
      print_warn "无效选项，已取消" >&2
      echo ""
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

prompt_release_channel() {
  local choice
  echo "" >&2
  echo "请选择 deb 来源:" >&2
  echo "  1) conf         — deb_versions.conf 固定 tag（默认）" >&2
  echo "  2) latest       — GitHub Latest 正式版" >&2
  echo "  3) pre-release  — 滚动 pre-release" >&2
  echo "  0) 取消" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3，默认 1]: " choice
  case "${choice}" in
    0) echo "" ;;
    2) echo "latest" ;;
    3) echo "pre-release" ;;
    1|"") echo "conf" ;;
    *)
      print_warn "无效选项，已取消" >&2
      echo ""
      ;;
  esac
}

# 从源缓存目录复制目标架构、匹配前缀的 deb 到当前 DEB_CACHE_DIR（仅作候选，稍后按远端版本校验）
seed_deb_cache_from() {
  local src_cache="$1"
  local arch="$2"
  local entry prefix display
  local -a matches=()
  local src_deb dest_deb copied=0

  mkdir -p "${DEB_CACHE_DIR}"
  if [ -z "${src_cache}" ] || [ ! -d "${src_cache}" ]; then
    print_info "无可用的源 .deb_cache，将按需下载"
    return 0
  fi

  print_info "从工作区缓存导入候选 deb: ${src_cache} (arch=${arch})"
  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r _owner_repo display prefix _tag <<< "${entry}"
    matches=()
    shopt -s nullglob
    matches=("${src_cache}/${prefix}"_*_"${arch}".deb)
    shopt -u nullglob
    if [ "${#matches[@]}" -eq 0 ]; then
      print_info "  ${display}: 缓存中无 ${arch} 包"
      continue
    fi
    for src_deb in "${matches[@]}"; do
      dest_deb="${DEB_CACHE_DIR}/$(basename "${src_deb}")"
      if [ -f "${dest_deb}" ]; then
        continue
      fi
      cp -f "${src_deb}" "${dest_deb}"
      print_info "  ${display}: 导入候选 $(basename "${src_deb}")"
      copied=$((copied + 1))
    done
  done
  print_info "候选导入完成（新复制 ${copied} 个；将按 deb_versions.conf 校验版本）"
}


# 从 GitHub API 获取指定 tag（或 latest）release 的 .deb 资产
# 输出格式（每行）: tag\tname\turl
# $1 = owner/repo  $2 = release tag（latest / pre-release / vX.Y.Z）
fetch_release_deb_assets() {
  local owner_repo="$1"
  local req_tag="${2:-latest}"
  local owner="${owner_repo%%/*}"
  local repo="${owner_repo#*/}"
  local api_url gh_path
  local gh_token
  gh_token="$(resolve_github_api_token)"

  case "${req_tag}" in
    latest|"*")
      api_url="https://api.github.com/repos/${owner}/${repo}/releases/latest"
      gh_path="repos/${owner}/${repo}/releases/latest"
      ;;
    *)
      api_url="https://api.github.com/repos/${owner}/${repo}/releases/tags/${req_tag}"
      gh_path="repos/${owner}/${repo}/releases/tags/${req_tag}"
      ;;
  esac

  # 已安装且已登录 gh 时优先走 gh api（自动带凭据）
  if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    if gh api "${gh_path}" \
        --jq '.tag_name as $t | .assets[] | select(.name | endswith(".deb")) | "\($t)\t\(.name)\t\(.browser_download_url)"' \
        2>"${TMPDIR:-/tmp}/gh_api.err.$$"; then
      rm -f "${TMPDIR:-/tmp}/gh_api.err.$$"
      return 0
    fi
    if grep -qi 'rate limit' "${TMPDIR:-/tmp}/gh_api.err.$$" 2>/dev/null; then
      cat "${TMPDIR:-/tmp}/gh_api.err.$$" >&2
      rm -f "${TMPDIR:-/tmp}/gh_api.err.$$"
      print_github_rate_limit_help >&2
      return 3
    fi
    rm -f "${TMPDIR:-/tmp}/gh_api.err.$$"
  fi

  GITHUB_API_TOKEN="${gh_token}" python3 - "${api_url}" <<'PY'
import json
import os
import sys
import urllib.error
import urllib.request

api_url = sys.argv[1]
token = os.environ.get("GITHUB_API_TOKEN", "")

headers = {
    "Accept": "application/vnd.github+json",
    "User-Agent": "lift2s-ws-release.sh",
    "X-GitHub-Api-Version": "2022-11-28",
}
if token:
    headers["Authorization"] = f"Bearer {token}"

try:
    req = urllib.request.Request(api_url, headers=headers)
    with urllib.request.urlopen(req, timeout=30) as resp:
        data = json.load(resp)
except urllib.error.HTTPError as e:
    body = ""
    try:
        body = e.read().decode("utf-8", errors="replace")
    except Exception:
        pass
    if e.code == 404:
        print(f"ERROR\tNOT_FOUND\t{api_url}", file=sys.stderr)
        sys.exit(2)
    if e.code == 403 and "rate limit" in body.lower():
        print(f"ERROR\tRATE_LIMIT\t{body}", file=sys.stderr)
        sys.exit(3)
    print(f"ERROR\tHTTP_{e.code}\t{body or api_url}", file=sys.stderr)
    sys.exit(1)
except Exception as e:
    print(f"ERROR\t{e}\t{api_url}", file=sys.stderr)
    sys.exit(1)

tag = data.get("tag_name", "")
for asset in data.get("assets", []):
    name = asset.get("name", "")
    url = asset.get("browser_download_url", "")
    if name.endswith(".deb") and url:
        print(f"{tag}\t{name}\t{url}")
PY
}

# 在资产列表中选取匹配的 deb（按前缀与架构）
select_deb_asset() {
  local arch="$1"
  local prefix="$2"
  local assets_file="$3"

  python3 - "${arch}" "${prefix}" "${assets_file}" <<'PY'
import sys

arch = sys.argv[1]
prefix = sys.argv[2]
path = sys.argv[3]

assets = []
with open(path, encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith("ERROR"):
            continue
        parts = line.split("\t")
        if len(parts) < 3:
            continue
        tag, name, url = parts[0], parts[1], parts[2]
        if not name.endswith(".deb"):
            continue
        if prefix and not name.startswith(prefix):
            continue
        assets.append((tag, name, url))

if not assets:
    sys.exit(1)

arch_suffix = f"_{arch}.deb"
matched = [a for a in assets if a[1].endswith(arch_suffix)]
if not matched:
    amd64_suffix = "_amd64.deb"
    matched = [a for a in assets if a[1].endswith(amd64_suffix)]
    if matched and arch != "amd64":
        print(f"WARN\t无 {arch} 包，回退 amd64\t{matched[0][1]}", file=sys.stderr)

if len(matched) == 1:
    tag, name, url = matched[0]
    print(f"{tag}\t{name}\t{url}")
    sys.exit(0)

# 多个匹配时取版本号排序后最后一个（通常版本最高）
matched.sort(key=lambda x: x[1])
tag, name, url = matched[-1]
print(f"{tag}\t{name}\t{url}")
PY
}

# 固定 tag：直接构造 releases/download URL 下载 deb（不查 GitHub API，避免速率限制）
# 资产名按 deb_versions.conf 期望格式 ${prefix}_${conf_ver}_${arch}.deb 构造；
# 无 <arch> 包时回退 amd64（与 select_deb_asset 行为一致）。
# 成功返回 0；下载失败或非有效 deb 返回 1（调用方回退 API 查询）。
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
      print_info "${display}: 复用缓存 $(basename "${dest}") (${conf_tag})"
      prune_deb_cache "${DEB_CACHE_DIR}" "${prefix}" "$(basename "${dest}")"
      return 0
    fi

    print_info "${display}: 直接下载 ${name} (${conf_tag}) ..."
    if download_file "${url}" "${dest}.part"; then
      if is_valid_deb_file "${dest}.part"; then
        mv "${dest}.part" "${dest}"
        print_info "已保存: ${dest}"
        prune_deb_cache "${DEB_CACHE_DIR}" "${prefix}" "$(basename "${dest}")"
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

download_deb_for_source() {
  local entry="$1"
  local arch="$2"
  local owner_repo display prefix conf_tag
  local assets_file tag name url dest sel_line
  local conf_ver expected
  local -a stale=()
  local f

  IFS='|' read -r owner_repo display prefix conf_tag <<< "${entry}"

  mkdir -p "${DEB_CACHE_DIR}"

  conf_ver="$(tag_to_deb_version "${conf_tag}")"
  # 固定 tag：先按 deb_versions.conf 期望文件名校验本地缓存
  if [ -n "${conf_ver}" ]; then
    expected="${DEB_CACHE_DIR}/${prefix}_${conf_ver}_${arch}.deb"
    if [ -f "${expected}" ]; then
      print_info "${display}: 与 deb_versions.conf 一致，复用 $(basename "${expected}") (tag=${conf_tag})"
      prune_deb_cache "${DEB_CACHE_DIR}" "${prefix}" "$(basename "${expected}")"
      return 0
    fi
    shopt -s nullglob
    stale=("${DEB_CACHE_DIR}/${prefix}"_*_"${arch}".deb)
    shopt -u nullglob
    if [ "${#stale[@]}" -gt 0 ]; then
      print_warn "${display}: 缓存不满足 deb_versions.conf（需要 ${prefix}_${conf_ver}_${arch}.deb / ${conf_tag}）"
      for f in "${stale[@]}"; do
        print_warn "  旧缓存: $(basename "${f}")"
      done
    fi

    # 固定 tag：直接构造 releases/download URL 下载（不查 GitHub API，避免限流），失败再回退 API
    if download_deb_by_constructed_url "${owner_repo}" "${display}" "${prefix}" "${conf_tag}" "${conf_ver}" "${arch}"; then
      return 0
    fi
    print_warn "${display}: 直接 URL 下载失败，回退 GitHub API 查询 ..."
  fi

  print_info "查询 ${display} (${owner_repo}@${conf_tag}) ..."

  assets_file="$(mktemp)"
  if ! fetch_release_deb_assets "${owner_repo}" "${conf_tag}" > "${assets_file}" 2>"${assets_file}.err"; then
    local fetch_rc=$?
    if grep -q $'\tNOT_FOUND\t' "${assets_file}.err" 2>/dev/null; then
      print_warn "跳过 ${display}: 无 Release tag ${conf_tag}"
    elif [ "${fetch_rc}" -eq 3 ] || grep -q $'\tRATE_LIMIT\t' "${assets_file}.err" 2>/dev/null; then
      print_error "获取 Release 失败: ${owner_repo}@${conf_tag}（GitHub API 速率限制）"
      cat "${assets_file}.err" >&2
      print_github_rate_limit_help
    else
      print_error "获取 Release 失败: ${owner_repo}@${conf_tag}"
      cat "${assets_file}.err" >&2
    fi
    rm -f "${assets_file}" "${assets_file}.err"
    return 1
  fi
  rm -f "${assets_file}.err"

  sel_line="$(select_deb_asset "${arch}" "${prefix}" "${assets_file}" 2>"${assets_file}.warn")" || sel_line=""
  if [ -z "${sel_line}" ]; then
    print_warn "跳过 ${display}: 未找到匹配 deb (前缀: ${prefix}, 架构: ${arch}, tag: ${conf_tag})"
    rm -f "${assets_file}" "${assets_file}.warn"
    return 1
  fi
  if [ -s "${assets_file}.warn" ]; then
    while IFS=$'\t' read -r _ msg deb_name; do
      print_warn "${display}: ${msg} -> ${deb_name}"
    done < "${assets_file}.warn"
  fi
  rm -f "${assets_file}" "${assets_file}.warn"

  tag="$(echo "${sel_line}" | cut -f1)"
  name="$(echo "${sel_line}" | cut -f2)"
  url="$(echo "${sel_line}" | cut -f3)"

  dest="${DEB_CACHE_DIR}/${name}"

  if [ -f "${dest}" ]; then
    print_info "${display}: 版本匹配，复用缓存 ${name} (${tag})"
    prune_deb_cache "${DEB_CACHE_DIR}" "${prefix}" "$(basename "${dest}")"
    return 0
  fi

  print_info "下载 ${name} (${tag} / conf=${conf_tag}) ..."
  if ! download_file "${url}" "${dest}.part"; then
    print_error "下载失败: ${url}"
    rm -f "${dest}.part"
    return 1
  fi
  mv "${dest}.part" "${dest}"
  print_info "已保存: ${dest}"

  prune_deb_cache "${DEB_CACHE_DIR}" "${prefix}" "$(basename "${dest}")"
  return 0
}

# 检查 .deb_cache 是否已含 deb_versions.conf 期望的全部 deb（按固定 tag）
# $1 = 架构；全部命中返回 0，否则返回 1
deb_cache_is_complete() {
  local arch="$1"
  local entry _owner_repo display prefix conf_tag conf_ver
  local missing=0

  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r _owner_repo display prefix conf_tag <<< "${entry}"
    conf_ver="$(tag_to_deb_version "${conf_tag}")"
    if [ -n "${conf_ver}" ] && [ -f "${DEB_CACHE_DIR}/${prefix}_${conf_ver}_${arch}.deb" ]; then
      continue
    fi
    # latest / * 等无固定版本标签无法离线判断，按缺失处理
    print_info "  ${display}: 缓存缺失 ${prefix}_${conf_ver}_${arch}.deb"
    missing=1
  done

  [ "${missing}" -eq 0 ]
}


resolve_release_channel() {
  local input="${1:-}"
  case "${input}" in
    conf|"") echo "conf" ;;
    latest|stable|formal) echo "latest" ;;
    prerelease|pre-release|pre|preview) echo "pre-release" ;;
    *)
      print_error "未知 Release 来源: ${input}（可选: conf / latest / pre-release）"
      return 1
      ;;
  esac
}

run_download_debs() {
  local arch="${1:-}" channel="${2:-${RELEASE_CHANNEL}}"
  channel="$(resolve_release_channel "${channel}")" || return 1
  [ -z "${arch}" ] && arch="$(detect_machine_arch)"
  case "${channel}" in
    conf) download_debs "${arch}" ;;
    latest|pre-release)
      print_info "install_core_debs.sh → ${DEB_CACHE_DIR} (channel=${channel}) ..."
      ARMS_VARIANT=full CACHE_ONLY=1 bash "${WS_DIR}/scripts/install_core_debs.sh"         --channel "${channel}" --arms-variant full --cache-only || return 1
      print_info "下载完成。安装: ./release.sh --install"
      ;;
  esac
}

run_download_debs_cli() {
  shift
  local deb_arch="" release_channel="${RELEASE_CHANNEL}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --arch) deb_arch="$2"; shift 2 ;;
      --release-channel) release_channel="$2"; shift 2 ;;
      *) print_error "未知参数: $1"; return 1 ;;
    esac
  done
  run_download_debs "${deb_arch}" "${release_channel}"
}

# 从 deb_versions.conf / install_core_debs 拉取 deb 到 .deb_cache/
# $1 = 目标架构（可选，默认本机架构）
download_debs() {
  local arch="${1:-}"
  local entry
  local fail=0

  need_cmd python3 || return 1
  if [ -z "${arch}" ]; then
    arch="$(detect_machine_arch)"
  fi
  print_info "按 deb_versions.conf 同步 deb 到 ${DEB_CACHE_DIR} (arch=${arch}) ..."

  mkdir -p "${DEB_CACHE_DIR}"

  # 已有完整缓存则直接复用，避免 GitHub API 查询
  if deb_cache_is_complete "${arch}"; then
    echo ""
    print_info "已有完整缓存，跳过下载（直接使用 .deb_cache/ 中的 deb）"
    print_info "下载完成。安装请执行: ./release.sh --install"
    return 0
  fi

  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    if ! download_deb_for_source "${entry}" "${arch}"; then
      fail=1
    fi
  done

  echo ""
  if [ "${fail}" -eq 0 ]; then
    print_info "下载完成。安装请执行: ./release.sh --install"
  else
    print_warn "部分 deb 下载失败"
  fi

  return "${fail}"
}

# 从 .deb_cache/ 按安装顺序收集 deb（发布包解压后使用）
collect_cached_debs() {
  local arch entry prefix display conf_tag conf_ver
  local -a matches=()
  local deb_path

  DEB_INSTALL_PATHS=()
  arch="$(detect_machine_arch)"

  if [ ! -d "${DEB_CACHE_DIR}" ]; then
    print_error "未找到 ${DEB_CACHE_DIR}"
    print_info "请先解压发布 zip，或执行: ./release.sh --download"
    return 1
  fi

  print_info "从 ${DEB_CACHE_DIR} 收集 deb (arch=${arch}) ..."

  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r _owner_repo display prefix conf_tag <<< "${entry}"
    matches=()
    shopt -s nullglob
    conf_ver="$(tag_to_deb_version "${conf_tag}")"
    if [ -n "${conf_ver}" ] && [ -f "${DEB_CACHE_DIR}/${prefix}_${conf_ver}_${arch}.deb" ]; then
      matches=("${DEB_CACHE_DIR}/${prefix}_${conf_ver}_${arch}.deb")
    else
      matches=("${DEB_CACHE_DIR}/${prefix}"_*_"${arch}".deb)
      if [ "${#matches[@]}" -eq 0 ]; then
        matches=("${DEB_CACHE_DIR}/${prefix}"_*_amd64.deb)
        if [ "${#matches[@]}" -gt 0 ] && [ "${arch}" != "amd64" ]; then
          print_warn "${display}: 无 ${arch} 包，使用 amd64: $(basename "${matches[0]}")"
        fi
      fi
    fi
    shopt -u nullglob

    if [ "${#matches[@]}" -eq 0 ]; then
      print_error "缺少 deb: ${prefix}_*_${arch}.deb （组件: ${display}，conf=${conf_tag}）"
      return 1
    fi

    if [ "${#matches[@]}" -gt 1 ]; then
      mapfile -t matches < <(printf '%s\n' "${matches[@]}" | sort -V)
    fi
    deb_path="${matches[-1]}"
    DEB_INSTALL_PATHS+=("${deb_path}")
    print_info "  ${display}: $(basename "${deb_path}")"
  done

  return 0
}

do_install_debs() {
  collect_cached_debs || return 1
  echo ""
  install_debs "${DEB_INSTALL_PATHS[@]}"
}

apt_install_local_debs() {
  # 使用 apt 安装本地 .deb，自动解析并安装 Depends 中的系统依赖
  if command -v apt >/dev/null 2>&1; then
    sudo apt install -y "$@"
  else
    need_cmd apt-get || return 1
    sudo apt-get install -y "$@"
  fi
}

apt_remove_packages() {
  # 卸载系统中的包（保留配置文件，避免误删系统级配置）
  if command -v apt >/dev/null 2>&1; then
    sudo apt remove -y "$@"
  else
    need_cmd apt-get || return 1
    sudo apt-get remove -y "$@"
  fi
}

install_debs() {
  local -a deb_files=("$@")
  local deb bn deb_abs
  local i=1 total="${#deb_files[@]}"

  if [ "${total}" -eq 0 ]; then
    print_error "没有可安装的 deb 文件"
    return 1
  fi

  need_cmd sudo || return 1
  print_info "按顺序 apt 安装 ${total} 个 deb（自动拉取依赖）..."
  print_info "顺序: ocs2_ros2 → robot-descriptions-common → arms-ros2-control-full"

  for deb in "${deb_files[@]}"; do
    deb_abs="$(cd "$(dirname "${deb}")" && pwd)/$(basename "${deb}")"
    bn="$(basename "${deb_abs}")"
    print_info "[${i}/${total}] apt install ${bn} ..."
    if ! apt_install_local_debs "${deb_abs}"; then
      print_error "安装失败: ${bn}"
      print_info "可尝试: sudo apt-get install -f -y  后重新运行 ./release.sh --install"
      return 1
    fi
    i=$((i + 1))
  done

  print_info "全部 deb 安装完成"
  print_info "下一步: cd \"${WS_DIR}\" && ./quick_start.sh"
}

uninstall_debs() {
  local -a uninstall_entries=()
  local entry display prefix
  local pkg i total

  need_cmd sudo || return 1

  # 按安装顺序反向卸载：arms-ros2-control-full -> robot-descriptions-common -> ocs2_ros2
  for ((i=${#DEB_RELEASE_SOURCES[@]}-1; i>=0; i--)); do
    uninstall_entries+=("${DEB_RELEASE_SOURCES[i]}")
  done

  total="${#uninstall_entries[@]}"
  print_info "按逆序卸载 ${total} 个 deb 对应系统包..."
  print_info "顺序: arms-ros2-control-full → robot-descriptions-common → ocs2_ros2"

  i=1
  for entry in "${uninstall_entries[@]}"; do
    IFS='|' read -r _owner_repo display prefix _conf_tag <<< "${entry}"
    pkg="${prefix}"
    print_info "[${i}/${total}] 卸载 ${display} (${pkg}) ..."

    if dpkg-query -W -f='${Status}' "${pkg}" 2>/dev/null | grep -q "install ok installed"; then
      if ! apt_remove_packages "${pkg}"; then
        print_error "卸载失败: ${pkg}"
        return 1
      fi
    else
      print_warn "未安装，跳过: ${pkg}"
    fi
    i=$((i + 1))
  done

  print_info "逆序卸载完成"
  print_info "如需清理孤立依赖，可执行: sudo apt autoremove -y"
}

get_submodule_default_branch() {
  local path="$1"
  git config -f "${WS_DIR}/.gitmodules" --get "submodule.${path}.branch" 2>/dev/null || echo "main"
}

# 打包用子模块：同步配置并检出当前已记录/已检出的版本（默认不 fetch）
# $1 = 1 时额外拉取各子模块默认分支最新提交（--update-submodules）
prepare_pack_submodules() {
  local pull_latest="${1:-0}"
  local path branch

  need_cmd git || return 1
  cd "${WS_DIR}" || return 1

  if [ ! -f .gitmodules ]; then
    print_error "未找到 .gitmodules"
    return 1
  fi

  print_info "同步子模块配置（仅打包保留的 ${#PACK_SOURCE_SUBMODULES[@]} 个）..."
  for path in "${PACK_SOURCE_SUBMODULES[@]}"; do
    git submodule sync -- "${path}" 2>/dev/null || true
  done

  if [[ "${pull_latest}" == "1" ]]; then
    print_info "将从远程拉取子模块最新提交 ..."
  else
    print_info "使用当前已检出的子模块版本（不拉取远程）"
    print_info "若需最新，打包请使用: ./release.sh --package --update-submodules"
  fi

  for path in "${PACK_SOURCE_SUBMODULES[@]}"; do
    branch="$(get_submodule_default_branch "${path}")"
    if [[ "${pull_latest}" == "1" ]]; then
      print_info "更新 ${path} (分支: ${branch}) ..."
    else
      print_info "检出 ${path} ..."
    fi

    if ! git submodule update --init -- "${path}"; then
      print_error "子模块初始化失败: ${path}"
      return 1
    fi

    if ! (
      cd "${path}" || exit 1
      if [ -n "$(git status --porcelain 2>/dev/null)" ]; then
        print_warn "  ${path} 工作区有未提交变更，临时目录内将丢弃 ..."
        git reset --hard HEAD
        git clean -fd
      fi
      if [[ "${pull_latest}" == "1" ]]; then
        git fetch origin
        if ! git ls-remote --exit-code --heads origin "${branch}" >/dev/null 2>&1; then
          print_error "  远程分支不存在: origin/${branch}"
          exit 1
        fi
        if git show-ref --verify --quiet "refs/heads/${branch}"; then
          git checkout "${branch}"
        else
          git checkout -B "${branch}" "origin/${branch}"
        fi
        git reset --hard "origin/${branch}"
      fi
    ); then
      print_error "子模块处理失败: ${path}"
      return 1
    fi

    print_info "  ✓ $(cd "${path}" && git rev-parse --short HEAD) ($(cd "${path}" && git log -1 --format='%s'))"
  done

  return 0
}

_write_deb_submodule_placeholder() {
  local path="$1"
  mkdir -p "${path}"
  cat > "${path}/.gitkeep" <<'EOF'
# 此目录在发布包中已清空；对应功能由 deb 包提供（arms-full 含 arx_ros2_control）。
# 若需源码开发，请在工作空间中执行: ./init_repo.sh
EOF
}

# 遍历 .gitmodules 中除 PACK_SOURCE_SUBMODULES 外的路径
_each_deb_provided_submodule_path() {
  local path all_paths=() keep skip

  if [ ! -f "${WS_DIR}/.gitmodules" ]; then
    return 0
  fi

  mapfile -t all_paths < <(git config --file "${WS_DIR}/.gitmodules" --get-regexp path 2>/dev/null | awk '{print $2}')
  for path in "${all_paths[@]}"; do
    skip=false
    for keep in "${PACK_SOURCE_SUBMODULES[@]}"; do
      if [ "${path}" = "${keep}" ]; then
        skip=true
        break
      fi
    done
    if [ "${skip}" = true ]; then
      continue
    fi
    printf '%s\n' "${path}"
  done
}


_each_orphan_src_path() {
  local path all_gitmodules_paths=() keep skip
  if [ ! -d "${WS_DIR}/src" ]; then return 0; fi
  if [ -f "${WS_DIR}/.gitmodules" ]; then
    mapfile -t all_gitmodules_paths < <(git config --file "${WS_DIR}/.gitmodules" --get-regexp path 2>/dev/null | awk '{print $2}')
  fi
  for path in "${WS_DIR}"/src/*/; do
    [ -d "${path}" ] || continue
    path="${path%/}"; path="${path#${WS_DIR}/}"
    skip=false
    for keep in "${PACK_SOURCE_SUBMODULES[@]}"; do
      if [ "${path}" = "${keep}" ]; then skip=true; break; fi
    done
    [ "${skip}" = true ] && continue
    for keep in "${all_gitmodules_paths[@]}"; do
      if [ "${path}" = "${keep}" ]; then skip=true; break; fi
    done
    [ "${skip}" = true ] && continue
    printf '%s\n' "${path}"
  done
}
remove_orphan_src_paths() {
  local path
  print_info "排除不在发布范围内的 src 残留目录..."
  while IFS= read -r path; do
    [ -n "${path}" ] || continue
    print_info "  排除: ${path}"
    rm -rf "${WS_DIR:?}/${path}"
  done < <(_each_orphan_src_path)
  return 0
}

# 不含 .git 打包：rsync 已跳过 deb 子模块，此处仅创建占位目录
create_deb_submodule_placeholders() {
  local path

  cd "${WS_DIR}" || return 1
  print_info "创建 deb 子模块占位目录（未复制源码）..."
  while IFS= read -r path; do
    [ -n "${path}" ] || continue
    _write_deb_submodule_placeholder "${path}"
    print_info "  占位: ${path}"
  done < <(_each_deb_provided_submodule_path)

  return 0
}

# 含 .git 打包：deinit 并清空 deb 子模块目录
clear_unused_submodules() {
  local path

  cd "${WS_DIR}" || return 1

  print_info "清空 deb 子模块目录..."
  while IFS= read -r path; do
    [ -n "${path}" ] || continue
    print_info "  清空: ${path} ..."
    git submodule deinit -f -- "${path}" 2>/dev/null || true
    rm -rf ".git/modules/${path}"
    rm -rf "${path}"
    _write_deb_submodule_placeholder "${path}"
  done < <(_each_deb_provided_submodule_path)

  return 0
}

# $1 = 是否包含 .git（1=是，0=否；默认 1）
# $2 = zip/deb 目标架构（amd64 | arm64）
create_release_zip() {
  local include_git="${1:-1}"
  local arch="${2:-$(detect_machine_arch)}"
  local zip_name zip_path ts git_tag="" deb
  local -a zip_excludes=(
    "build/*"
    "install/*"
    "log/*"
    "logs/*"
    "dist/*"
    "*.zip"
    ".vscode/*"
    ".idea/*"
    "*__pycache__/*"
    "*.pyc"
  )
  local -a deb_files=()

  need_cmd zip || return 1
  ts="$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${DIST_DIR}"
  if [[ "${include_git}" == "0" ]]; then
    git_tag="_nogit"
  fi
  zip_name="${RELEASE_WS_NAME}_${ts}_${arch}${git_tag}.zip"
  zip_path="${DIST_DIR}/${zip_name}"

  # 打包前确认 .deb_cache 已就绪（现场 --install 依赖此目录）
  shopt -s nullglob
  deb_files=("${WS_DIR}/.deb_cache"/*.deb)
  shopt -u nullglob
  if [ "${#deb_files[@]}" -eq 0 ]; then
    print_error "打包目录中没有 deb: ${WS_DIR}/.deb_cache/"
    print_error "请检查下载步骤是否写入了正确的缓存目录"
    return 1
  fi
  print_info "将打入 zip 的 .deb_cache (${#deb_files[@]} 个 deb):"
  for deb in "${deb_files[@]}"; do
    print_info "  - $(basename "${deb}")"
  done

  print_info "打包 zip: ${zip_path} ..."
  if [[ "${include_git}" == "1" ]]; then
    print_info "模式: 含 .git（部署机可 git pull 更新脚本）"
  else
    print_info "模式: 不含 .git（纯快照，体积更小；脚本更新需重新发 zip 或 git clone）"
    zip_excludes+=(".git/*" "*/.git/*")
  fi

  (
    cd "${WS_DIR}" || exit 1
    # 显式带上 .deb_cache，避免隐藏目录在部分环境下被漏打
    zip -r "${zip_path}" . .deb_cache -x "${zip_excludes[@]}"
  ) || return 1

  if ! unzip -l "${zip_path}" 2>/dev/null | grep -q '\.deb_cache/.*\.deb'; then
    print_error "校验失败: zip 内未找到 .deb_cache/*.deb"
    return 1
  fi

  print_info "打包完成: ${zip_path} ($(du -h "${zip_path}" | cut -f1))"
  print_info "zip 内已包含 .deb_cache（可用 unzip -l 查看）"
  return 0
}

# 复制工作区到临时目录，避免 --package 修改维护机上的子模块状态
# $1 = 是否包含 .git（0=不含：排除 .git 与 deb 子模块源码，仅 rsync 保留 PACK_SOURCE_SUBMODULES）
prepare_package_staging() {
  local include_git="${1:-1}"
  local staging path
  local -a rsync_args=(
    -a
    --exclude='build/'
    --exclude='install/'
    --exclude='log/'
    --exclude='logs/'
    --exclude='dist/'
    --exclude='.deb_cache/'
  )

  need_cmd rsync || return 1
  staging="$(mktemp -d "${TMPDIR:-/tmp}/lift2s_ws_release.XXXXXX")" || return 1

  if [[ "${include_git}" == "0" ]]; then
    rsync_args+=(--exclude='.git/')
    while IFS= read -r path; do
      [ -n "${path}" ] || continue
      rsync_args+=(--exclude="${path}/")
    done < <(_each_deb_provided_submodule_path)
    print_info "复制到临时目录（不含 .git，不复制 deb 子模块源码）: ${staging}"
  else
    print_info "复制工作区到临时目录: ${staging}"
  fi

  rsync "${rsync_args[@]}" "${WS_DIR}/" "${staging}/" || {
    rm -rf "${staging}"
    return 1
  }

  if [ ! -d "${staging}" ]; then
    print_error "临时目录无效: ${staging}"
    return 1
  fi

  # 仅将路径打到 stdout，供 $(...) 捕获；日志已全部走 stderr
  printf '%s' "${staging}"
  return 0
}

cleanup_package_staging() {
  local staging="${1:-}"
  if [ -n "${staging}" ] && [ -d "${staging}" ]; then
    print_info "清理临时目录: ${staging}"
    rm -rf "${staging}"
  fi
}

# $1 = 是否包含 .git（1=是，0=否；默认 1）
# $2 = deb/zip 目标架构（amd64 | arm64；默认本机架构）
# $3 = 是否拉取子模块最新（1=是，0=否；默认 0）
# $4 = deb 通道（conf | latest | pre-release；默认 conf）
do_package_release() {
  local include_git="${1:-1}"
  local package_arch="${2:-$(detect_machine_arch)}"
  local update_submodules="${3:-0}"
  local release_channel="${4:-conf}"
  local staging=""
  local orig_dist
  local host_deb_cache=""
  local -a saved_ws=()

  package_release_cleanup() {
    if [ -n "${staging}" ]; then
      cleanup_package_staging "${staging}"
      staging=""
    fi
    if [ "${#saved_ws[@]}" -gt 0 ]; then
      WS_DIR="${saved_ws[0]}"
      DEB_CACHE_DIR="${saved_ws[1]}"
      DIST_DIR="${saved_ws[2]}"
    fi
  }

  print_info "======== 开始发布打包 ========"
  print_info "打包保留: ${PACK_SOURCE_SUBMODULES[*]}"
  if [[ "${include_git}" == "1" ]]; then
    print_info "deb 子模块: 复制后清空为占位（由 deb 提供）"
    print_info "zip 将包含 .git"
  else
    print_info "deb 子模块: 不复制源码，仅保留占位目录"
    print_info "zip 将排除 .git"
  fi
  print_info "在临时目录打包（rsync 已排除 build/install/log），不修改当前工作区"
  print_info "目标架构: ${package_arch}（deb 与 zip 文件名）"
  echo ""

  if [ ! -d "${WS_DIR}/.git" ]; then
    print_error "当前目录不是 git 仓库，无法处理子模块"
    return 1
  fi

  staging="$(prepare_package_staging "${include_git}")" || return 1
  if [ -z "${staging}" ] || [ ! -d "${staging}" ]; then
    print_error "无法创建打包临时目录（staging='${staging}'）"
    return 1
  fi
  orig_dist="${WS_DIR}/dist"
  host_deb_cache="${DEB_CACHE_DIR}"
  saved_ws=("${WS_DIR}" "${DEB_CACHE_DIR}" "${DIST_DIR}")
  trap package_release_cleanup EXIT

  WS_DIR="${staging}"
  DEB_CACHE_DIR="${WS_DIR}/.deb_cache"

  remove_orphan_src_paths || return 1
  DIST_DIR="${orig_dist}"
  print_info "打包工作目录: ${WS_DIR}"
  print_info "deb 缓存目录: ${DEB_CACHE_DIR}"

  if [[ "${include_git}" == "1" ]]; then
    prepare_pack_submodules "${update_submodules}" || return 1
    clear_unused_submodules || return 1
  else
    create_deb_submodule_placeholders || return 1
    print_info "源码子模块为 rsync 复制的当前文件（无 git 元数据）"
    for path in "${PACK_SOURCE_SUBMODULES[@]}"; do
      print_info "  ✓ ${path}"
    done
  fi

  print_info "准备 .deb_cache（channel=${release_channel}；版本匹配则复用，否则下载）..."
  mkdir -p "${DEB_CACHE_DIR}"
  seed_deb_cache_from "${host_deb_cache}" "${package_arch}"
  RELEASE_CHANNEL="${release_channel}"
  if ! run_download_debs "${package_arch}" "${release_channel}"; then
    print_warn "部分 deb 下载失败，将检查缓存后继续"
  fi
  if ! compgen -G "${DEB_CACHE_DIR}/*.deb" >/dev/null; then
    print_error "下载后 ${DEB_CACHE_DIR} 仍无 .deb，无法打包可用的发布包"
    return 1
  fi

  create_release_zip "${include_git}" "${package_arch}" || return 1

  trap - EXIT
  package_release_cleanup

  print_info "======== 发布打包完成 ========"
  return 0
}

list_releases() {
  local entry owner_repo display prefix conf_tag arch
  local assets_file tag

  need_cmd python3 || return 1
  arch="$(detect_machine_arch)"

  echo ""
  echo -e "${BLUE}======== deb_versions.conf 对应 Release (arch=${arch}) ========${NC}"
  echo -e "${BLUE}配置文件: ${DEB_CONF}${NC}"
  printf "%-36s | %-12s | %s\n" "组件" "conf tag" "deb 文件"
  printf "%-36s-+-%-12s-+-%s\n" "------------------------------------" "------------" "----------------------------------------"

  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r owner_repo display prefix conf_tag <<< "${entry}"
    conf_ver="$(tag_to_deb_version "${conf_tag}")"
    if [ -n "${conf_ver}" ]; then
      # 固定 tag：直接 Range 探测 releases/download URL（不查 GitHub API，避免限流）
      _cand="${prefix}_${conf_ver}_${arch}.deb"
      if curl -fsSL -o /dev/null --max-time 15 -r 0-0 "https://github.com/${owner_repo}/releases/download/${conf_tag}/${_cand}" 2>/dev/null; then
        printf "%-36s | %-12s | %s\n" "${display}" "${conf_tag}" "${_cand}"
      else
        printf "%-36s | %-12s | %s\n" "${display}" "${conf_tag}" "(无 ${_cand}，见 Release 页面)"
      fi
      continue
    fi
    assets_file="$(mktemp)"
    if ! fetch_release_deb_assets "${owner_repo}" "${conf_tag}" > "${assets_file}" 2>/dev/null; then
      printf "%-36s | %-12s | %s\n" "${display}" "${conf_tag}" "无 Release"
      rm -f "${assets_file}"
      continue
    fi
    tag="$(head -n1 "${assets_file}" | cut -f1)"
    if _sel_line="$(select_deb_asset "${arch}" "${prefix}" "${assets_file}" 2>/dev/null)" && [ -n "${_sel_line}" ]; then
      _deb_name="$(echo "${_sel_line}" | cut -f2)"
      printf "%-36s | %-12s | %s\n" "${display}" "${conf_tag}" "${_deb_name}"
    else
      printf "%-36s | %-12s | %s\n" "${display}" "${conf_tag}" "(无 ${arch} 匹配包, release=${tag})"
    fi
    rm -f "${assets_file}"
  done
  echo -e "${BLUE}=======================================================${NC}"
  echo ""
  echo "Release 页面:"
  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r owner_repo _display _prefix conf_tag <<< "${entry}"
    echo "  - https://github.com/${owner_repo}/releases/tag/${conf_tag}"
  done
}

show_interactive_menu() {
  local choice

  while true; do
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    ARX Lift2S 部署 / 发布${NC}"
    echo -e "${BLUE}  Workspace: ${WS_DIR}${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo "请选择操作:"
    echo "  1) 下载 deb 依赖包（从 deb_versions.conf 中获取，安装到 .deb_cache）"
    echo "  2) 安装 deb 依赖包（从 .deb_cache 安装，需 sudo）"
    echo "  3) 一键卸载 deb（按安装逆序，需 sudo）"
    echo "  4) 发布打包 zip（含 .git）"
    echo "  5) 发布打包 zip（不含 .git，体积更小）"
    echo "  0) 退出"
    echo ""
    read -r -p "请输入选项 [0-5]: " choice

    case "${choice}" in
      1)
        local _dl_channel _dl_arch
        _dl_channel="$(prompt_release_channel)"
        if [ -z "${_dl_channel}" ]; then
          print_info "已取消"
          continue
        fi
        _dl_arch="$(detect_machine_arch)"
        run_download_debs "${_dl_arch}" "${_dl_channel}"
        ;;
      2)
        echo ""
        echo "安装顺序:"
        echo "  1) ocs2_ros2"
        echo "  2) robot-descriptions-common"
        echo "  3) arms-ros2-control-full"
        echo ""
        do_install_debs
        exit $?
        ;;
      3)
        echo ""
        echo "卸载顺序（安装顺序倒序）:"
        echo "  1) arms-ros2-control-full"
        echo "  2) robot-descriptions-common"
        echo "  3) ocs2_ros2"
        echo ""
        uninstall_debs
        ;;
      4)
        echo ""
        local _pkg_channel
        _pkg_channel="$(prompt_release_channel)"
        if [ -z "${_pkg_channel}" ]; then
          print_info "已取消"
          continue
        fi
        print_info "在临时目录打包（含 .git，deb 通道 ${_pkg_channel}），不会修改当前工作区子模块"
        read -r -p "确认继续？[y/N]: " confirm
        case "${confirm}" in
          y|Y|yes|YES)
            if do_package_release 1 "$(detect_machine_arch)" 0 "${_pkg_channel}"; then
              exit 0
            fi
            ;;
          *)
            print_info "已取消"
            ;;
        esac
        ;;
      5)
        echo ""
        local _pkg_arch _pkg_channel
        _pkg_arch="$(prompt_package_arch)"
        if [ -n "${_pkg_arch}" ]; then
          _pkg_channel="$(prompt_release_channel)"
          if [ -z "${_pkg_channel}" ]; then
            print_info "已取消"
            continue
          fi
          print_info "在临时目录打包（不含 .git，架构 ${_pkg_arch}，通道 ${_pkg_channel}），不会修改当前工作区子模块"
          read -r -p "确认继续？[y/N]: " confirm
          case "${confirm}" in
            y|Y|yes|YES)
              if do_package_release 0 "${_pkg_arch}" 0 "${_pkg_channel}"; then
                exit 0
              fi
              ;;
            *)
              print_info "已取消"
              ;;
          esac
        else
          print_info "已取消"
        fi
        ;;
      0)
        echo "退出"
        exit 0
        ;;
      *)
        print_warn "无效选项"
        ;;
    esac
  done
}

usage() {
  cat <<EOF
用法: $0 [选项]

选项:
  --download            按 deb_versions.conf 下载 deb 到 ${DEB_CACHE_DIR}/（已有匹配缓存则直接复用）
                        固定 tag 时直接使用 releases/download URL（不查 GitHub API，避免限流）
  --install             从 .deb_cache/ 按顺序 apt 安装 deb（需 sudo）
  --uninstall           按安装逆序卸载系统中的 deb 包（需 sudo）
  --package             维护者：打包 zip（含 .git；deb 架构=本机；默认 channel=conf）
  --package-no-git      维护者：打包 zip（不含 .git；可配合 --arch）
  --arch <amd64|arm64>  与 --package-no-git 联用，指定 deb/zip 目标架构
  --release-channel <conf|latest|pre-release>
                        与 --download / --package* 联用，指定 deb 来源（默认 conf）
  --update-submodules   打包前从远程拉取保留子模块最新（默认用当前已检出版本）
  --list                维护者：列出 deb_versions.conf 对应 Release deb
  -h, --help            显示帮助

典型部署流程:
  1. 解压发布 zip（已含 .deb_cache/ 时可跳过下载）
  2. ./release.sh --install
  3. ./quick_start.sh

若 zip 含 .git，可更新工作区脚本（需 git remote 访问权限）:
  git pull --ff-only
  git submodule update --init -- src/robot-descriptions-arx
  （不含 .git 的 *_nogit.zip 需重新发 zip 或自行 clone 主仓）

若缺少 deb 或需更新版本:
  ./release.sh --download [--release-channel conf|latest|pre-release]
  ./release.sh --install

deb 安装顺序（读取 ${DEB_CACHE_DIR}/ 下已打包的 deb）:
  1. ros-${ROS_DISTRO}-ocs2_*_<arch>.deb
  2. ros-${ROS_DISTRO}-robot-descriptions-common_*_<arch>.deb
  3. ros-${ROS_DISTRO}-arms-ros2-control-full_*_<arch>.deb

deb 卸载顺序（安装顺序倒序）:
  1. ros-${ROS_DISTRO}-arms-ros2-control-full
  2. ros-${ROS_DISTRO}-robot-descriptions-common
  3. ros-${ROS_DISTRO}-ocs2

发布打包 (--package / --package-no-git) 流程（在临时目录执行，不修改当前工作区）:
  1. --package: 检出保留子模块 + 清空 deb 子模块为占位
     --package-no-git: 仅 rsync 保留子模块文件，deb 子模块不复制、只建占位
  2. 导入工作区 .deb_cache 候选；按 deb 通道下载/校验 deb
  3. 生成 ${DIST_DIR}/${RELEASE_WS_NAME}_<时间>_<架构>[_nogit].zip 并删除临时目录

打包前若需最新子模块，请加 --update-submodules（或运行 ./init_repo.sh --init-release 后重新打包）

示例:
  ./release.sh --package --release-channel conf
  ./release.sh --package-no-git --arch arm64 --release-channel conf
  ./release.sh --package --update-submodules
EOF
}

run_package_release() {
  local include_git="$1"
  shift
  local deb_arch=""
  local update_submodules=0
  local release_channel="${RELEASE_CHANNEL:-conf}"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --arch)
        if [[ -z "${2:-}" ]]; then
          print_error "--arch 需要参数（amd64 或 arm64）"
          return 1
        fi
        deb_arch="$2"
        shift 2
        ;;
      --release-channel)
        if [[ -z "${2:-}" ]]; then
          print_error "--release-channel 需要参数（conf / latest / pre-release）"
          return 1
        fi
        release_channel="$2"
        shift 2
        ;;
      --update-submodules)
        update_submodules=1
        shift
        ;;
      *)
        print_error "未知参数: $1"
        return 1
        ;;
    esac
  done

  release_channel="$(resolve_release_channel "${release_channel}")" || return 1

  if [[ -z "${deb_arch}" ]]; then
    if [[ "${include_git}" == "0" ]]; then
      print_error "请指定目标架构: ./release.sh --package-no-git --arch amd64|arm64"
      print_info "交互式菜单选 5) 也可选择架构"
      return 1
    fi
    deb_arch="$(detect_machine_arch)"
  else
    deb_arch="$(resolve_deb_arch "${deb_arch}")" || return 1
  fi

  do_package_release "${include_git}" "${deb_arch}" "${update_submodules}" "${release_channel:-conf}"
}

main() {
  case "${1:-}" in
    -h|--help)
      usage
      exit 0
      ;;
  esac

  load_deb_versions_conf || exit 1

  case "${1:-}" in
    --list)
      list_releases
      exit 0
      ;;
    --download)
      run_download_debs_cli "$@"
      exit $?
      ;;
    --install)
      do_install_debs
      exit $?
      ;;
    --uninstall)
      uninstall_debs
      exit $?
      ;;
    --package)
      shift
      run_package_release 1 "$@"
      exit $?
      ;;
    --package-no-git)
      shift
      run_package_release 0 "$@"
      exit $?
      ;;
    "")
      show_interactive_menu
      ;;
    *)
      print_error "未知参数: $1"
      usage
      exit 1
      ;;
  esac
}

main "$@"
