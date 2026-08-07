#!/usr/bin/env bash

# ARX Lift2S / ACone 工作空间 — deb 安装与发布打包脚本
# 使用方：解压发布 zip → ./release.sh --install → ./quick_start.sh
#         --package 含 .git（可 git pull）；--package-no-git 为纯快照包（更小）
#
# 发布 zip 保留的源码子模块见 config/quick_start.conf → RELEASE_SUBMODULE_PATHS；
# 其余 .gitmodules 子模块在 zip 内清空为占位，由 deb 提供（ocs2 / common / arms）。
# 真机 HI（arx_ros2_control）不在 arms deb 内，须保留在 RELEASE_SUBMODULE_PATHS。
# deb 安装顺序: ocs2_ros2 → robot-descriptions-common → arms-ros2-control（standard）
#
# 用法:
#   ./release.sh              # 交互式菜单
#   ./release.sh --download   # 从 GitHub Release 下载 deb 到 .deb_cache/
#   ./release.sh --install    # 从 .deb_cache/ 安装 deb（需 sudo）
#   ./release.sh --uninstall  # 按逆序卸载系统中的 deb 包（需 sudo）
#   ./release.sh --package           # 维护者：打包 zip（含 .git）
#   ./release.sh --package-no-git    # 维护者：打包 zip（不含 .git，体积更小）
#   ./release.sh --list       # 维护者：查看 GitHub Release deb 信息
#
# deb 来源（--release-channel）:
#   latest      — 最新正式 Release（默认）
#   prerelease  — 滚动 pre-release（tag: pre-release，各组件 CI 合并 main 后发布）
#                 若某仓库尚无 pre-release，自动回退 latest 正式版（如 ocs2_ros2）

set -u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEB_CACHE_DIR="${WS_DIR}/.deb_cache"
DIST_DIR="${WS_DIR}/dist"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PRE_RELEASE_TAG="${PRE_RELEASE_TAG:-pre-release}"
RELEASE_CHANNEL="${RELEASE_CHANNEL:-latest}"

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
DEB_INSTALL_PATHS=()

# shellcheck source=scripts/deb_cache_lib.sh
source "${WS_DIR}/scripts/deb_cache_lib.sh"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

if [[ ${#RELEASE_SUBMODULE_PATHS[@]} -eq 0 ]]; then
  print_error "config/quick_start.conf 未定义 RELEASE_SUBMODULE_PATHS"
  exit 1
fi
PACK_SOURCE_SUBMODULES=("${RELEASE_SUBMODULE_PATHS[@]}")

need_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    print_error "缺少命令: $cmd"
    return 1
  fi
  return 0
}

# 归一化用户输入的 deb/zip 目标架构（amd64 | arm64）
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

# 归一化 deb 来源：latest | prerelease
resolve_release_channel() {
  local input="${1:-}"
  case "${input}" in
    latest|stable|formal|"") echo "latest" ;;
    prerelease|pre-release|pre|preview) echo "prerelease" ;;
    *)
      print_error "未知 Release 来源: ${input}（可选: latest / prerelease）"
      return 1
      ;;
  esac
}

prompt_release_channel() {
  local choice
  echo "" >&2
  echo "请选择 deb 来源（GitHub Release）:" >&2
  echo "  1) latest     — 最新正式 Release（默认）" >&2
  echo "  2) prerelease — 滚动 pre-release（tag: ${PRE_RELEASE_TAG}）" >&2
  echo "  0) 取消" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2，默认 1]: " choice
  case "${choice}" in
    0) echo "" ;;
    1|"") echo "latest" ;;
    2) echo "prerelease" ;;
    *)
      print_warn "无效选项，已取消" >&2
      echo ""
      ;;
  esac
}

sanitize_zip_token() {
  printf '%s' "$1" | sed 's/[[:space:]/]/_/g'
}

# 打包 zip 版本号以 arms_ros2_control deb 为准
resolve_package_zip_version() {
  local arch="$1"
  local version=""

  version="$(resolve_cached_deb_version "${ARMS_ROS2_CONTROL_DEB_PREFIX}" "${arch}" 2>/dev/null || true)"
  if [ -n "${version}" ]; then
    sanitize_zip_token "${version}"
    return 0
  fi

  print_warn "未在 ${DEB_CACHE_DIR} 找到 ${ARMS_ROS2_CONTROL_DEB_PREFIX}_*_${arch}.deb，zip 版本记为 unknown"
  echo "unknown"
  return 1
}

# GitHub API 凭据（可选）：认证请求限额 5000/小时，未认证仅 60/小时（按出口 IP 共享）
# 支持 GH_TOKEN / GITHUB_TOKEN，或已登录的 gh CLI（gh auth login）
resolve_github_api_token() {
  if [ -n "${GITHUB_TOKEN:-}" ]; then
    printf '%s' "${GITHUB_TOKEN}"
    return 0
  fi
  if [ -n "${GH_TOKEN:-}" ]; then
    printf '%s' "${GH_TOKEN}"
    return 0
  fi
  if command -v gh >/dev/null 2>&1; then
    gh auth token 2>/dev/null || true
  fi
}

print_github_rate_limit_help() {
  print_error "GitHub API 速率限制（未认证请求约 60 次/小时，同一公网 IP 共享）"
  print_info "请任选其一后重试:"
  print_info "  export GH_TOKEN=ghp_xxxxxxxx   # Personal Access Token（public_repo 读权限即可）"
  print_info "  gh auth login                  # 使用 GitHub CLI 登录"
  print_info "文档: https://docs.github.com/rest/overview/resources-in-the-rest-api#rate-limiting"
}

# 从 GitHub API 获取 Release 的 tag 与 .deb 资产
# $2 = 来源通道：latest（默认）| prerelease
# 输出格式（每行一个字段，用 TAB 分隔）: tag\tname\turl
fetch_release_deb_assets() {
  local owner_repo="$1"
  local channel="${2:-latest}"
  local owner="${owner_repo%%/*}"
  local repo="${owner_repo#*/}"
  local api_url gh_release_ref
  local gh_token
  gh_token="$(resolve_github_api_token)"

  case "${channel}" in
    prerelease)
      api_url="https://api.github.com/repos/${owner}/${repo}/releases/tags/${PRE_RELEASE_TAG}"
      gh_release_ref="tags/${PRE_RELEASE_TAG}"
      ;;
    latest|*)
      api_url="https://api.github.com/repos/${owner}/${repo}/releases/latest"
      gh_release_ref="latest"
      ;;
  esac

  # 已安装且已登录 gh 时优先走 gh api（自动带凭据）
  if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    if gh api "repos/${owner}/${repo}/releases/${gh_release_ref}" \
        --jq '.tag as $tag | .assets[] | select(.name | endswith(".deb")) | "\($tag)\t\(.name)\t\(.browser_download_url)"' \
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

  GITHUB_API_TOKEN="${gh_token}" CHANNEL="${channel}" PRE_RELEASE_TAG="${PRE_RELEASE_TAG}" python3 - "${api_url}" <<'PY'
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
        channel = os.environ.get("CHANNEL", "latest")
        pre_tag = os.environ.get("PRE_RELEASE_TAG", "pre-release")
        if channel == "prerelease":
            print(f"ERROR\tNOT_FOUND\tno pre-release tag {pre_tag} on {api_url}", file=sys.stderr)
        else:
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

tag = data.get("tag_name") or ""
if not tag:
    channel = os.environ.get("CHANNEL", "latest")
    if channel == "prerelease":
        tag = os.environ.get("PRE_RELEASE_TAG", "pre-release")
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
        # standard 前缀勿匹配 *-full_*（如 arms-ros2-control vs arms-ros2-control-full）
        if prefix and not prefix.endswith("-full") and (
            name.startswith(prefix + "-full_") or name.startswith(prefix + "-full.")
        ):
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

download_file() {
  local url="$1"
  local dest="$2"

  if command -v curl >/dev/null 2>&1; then
    curl -fL --retry 3 --connect-timeout 15 -o "${dest}" "${url}"
  elif command -v wget >/dev/null 2>&1; then
    wget -q -O "${dest}" "${url}"
  else
    print_error "需要 curl 或 wget"
    return 1
  fi
}

download_deb_for_source() {
  local entry="$1"
  local arch="$2"
  local channel="${3:-latest}"
  local reuse_cache_dir="${4:-}"
  local owner_repo display prefix
  local assets_file tag name url dest sel_line
  local fetch_channel="${channel}"
  local fetch_rc=0
  local used_fallback=0

  IFS='|' read -r owner_repo display prefix <<< "${entry}"

  print_info "查询 ${display} (${owner_repo}, channel=${channel}) ..."

  assets_file="$(mktemp)"
  while true; do
    rm -f "${assets_file}.err"
    if fetch_release_deb_assets "${owner_repo}" "${fetch_channel}" > "${assets_file}" 2>"${assets_file}.err"; then
      break
    fi
    fetch_rc=$?
    if [[ "${channel}" == "prerelease" && "${fetch_channel}" == "prerelease" ]] \
      && grep -q $'\tNOT_FOUND\t' "${assets_file}.err" 2>/dev/null; then
      print_warn "${display}: 无 pre-release（tag: ${PRE_RELEASE_TAG}），回退 latest 正式版"
      fetch_channel="latest"
      used_fallback=1
      continue
    fi
    if grep -q $'\tNOT_FOUND\t' "${assets_file}.err" 2>/dev/null; then
      if [[ "${channel}" == "prerelease" && "${used_fallback}" == "1" ]]; then
        print_warn "跳过 ${display}: 无 Release"
      elif [[ "${channel}" == "prerelease" ]]; then
        print_warn "跳过 ${display}: 无 pre-release（tag: ${PRE_RELEASE_TAG}）"
      else
        print_warn "跳过 ${display}: 无 Release"
      fi
    elif [ "${fetch_rc}" -eq 3 ] || grep -q $'\tRATE_LIMIT\t' "${assets_file}.err" 2>/dev/null; then
      print_error "获取 Release 失败: ${owner_repo}（GitHub API 速率限制）"
      cat "${assets_file}.err" >&2
      print_github_rate_limit_help
    else
      print_error "获取 Release 失败: ${owner_repo}"
      cat "${assets_file}.err" >&2
    fi
    rm -f "${assets_file}" "${assets_file}.err"
    return 1
  done
  rm -f "${assets_file}.err"

  sel_line="$(select_deb_asset "${arch}" "${prefix}" "${assets_file}" 2>"${assets_file}.warn")" || sel_line=""
  if [ -z "${sel_line}" ]; then
    print_warn "跳过 ${display}: 未找到匹配 deb (前缀: ${prefix}, 架构: ${arch})"
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

  if [[ -z "${tag}" || "${tag}" == "null" || "${tag}" == "None" ]]; then
    if [[ "${fetch_channel}" == "prerelease" ]]; then
      tag="${PRE_RELEASE_TAG}"
    elif [[ "${used_fallback}" == "1" ]]; then
      tag="latest"
    else
      tag="${channel}"
    fi
  fi
  if [[ "${used_fallback}" == "1" ]]; then
    print_info "${display}: 使用 latest 正式版（${tag}）"
  fi

  mkdir -p "${DEB_CACHE_DIR}"
  dest="${DEB_CACHE_DIR}/${name}"

  if [ -f "${dest}" ]; then
    print_info "已存在，跳过下载: ${name}"
  elif [ -n "${reuse_cache_dir}" ] && [ -f "${reuse_cache_dir}/${name}" ]; then
    print_info "远程版本与本地缓存一致，跳过下载: ${name}"
    cp "${reuse_cache_dir}/${name}" "${dest}"
  else
    print_info "下载 ${name} (${tag}) ..."
    if ! download_file "${url}" "${dest}.part"; then
      print_error "下载失败: ${url}"
      rm -f "${dest}.part"
      return 1
    fi
    mv "${dest}.part" "${dest}"
    print_info "已保存: ${dest}"
  fi

  prune_deb_cache_prefix "${prefix}" "${arch}" "${dest}"

  if [ -n "${reuse_cache_dir}" ] && [ "${reuse_cache_dir}" != "${DEB_CACHE_DIR}" ]; then
    sync_deb_to_cache_dir "${reuse_cache_dir}" "${prefix}" "${arch}" "${dest}"
  fi
  return 0
}

# 从 GitHub Release 拉取 deb 到 .deb_cache/
# $1 = 目标架构（可选，默认本机架构）
# $2 = 来源通道（可选，默认 RELEASE_CHANNEL 或 latest）
# $3 = 复用缓存目录（可选；远程 deb 文件名与其中一致时跳过下载，用于发布打包）
download_debs() {
  local arch="${1:-}"
  local channel="${2:-${RELEASE_CHANNEL}}"
  local reuse_cache_dir="${3:-}"
  local persist_cache_dir=""
  local entry
  local fail=0

  need_cmd python3 || return 1
  channel="$(resolve_release_channel "${channel}")" || return 1
  if [ -z "${arch}" ]; then
    arch="$(detect_machine_arch)"
  fi

  if [ -n "${reuse_cache_dir}" ]; then
    persist_cache_dir="${reuse_cache_dir}"
  else
    persist_cache_dir="${DEB_CACHE_DIR}"
  fi
  ensure_deb_cache_channel "${persist_cache_dir}" "${channel}"

  if [ -n "${reuse_cache_dir}" ]; then
    print_info "从 GitHub Release 同步 deb 到 ${DEB_CACHE_DIR} (arch=${arch}, channel=${channel}，可复用 ${reuse_cache_dir}) ..."
  else
    print_info "从 GitHub Release 下载 deb 到 ${DEB_CACHE_DIR} (arch=${arch}, channel=${channel}) ..."
  fi

  mkdir -p "${DEB_CACHE_DIR}"
  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    if ! download_deb_for_source "${entry}" "${arch}" "${channel}" "${reuse_cache_dir}"; then
      fail=1
    fi
  done

  if [ "${fail}" -eq 0 ]; then
    write_deb_cache_channel "${persist_cache_dir}" "${channel}"
  fi

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
  local arch entry prefix display
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
    IFS='|' read -r _owner_repo display prefix <<< "${entry}"
    matches=()
    shopt -s nullglob
    matches=("${DEB_CACHE_DIR}/${prefix}"_*_"${arch}".deb)
    if [ "${#matches[@]}" -eq 0 ]; then
      matches=("${DEB_CACHE_DIR}/${prefix}"_*_amd64.deb)
      if [ "${#matches[@]}" -gt 0 ] && [ "${arch}" != "amd64" ]; then
        print_warn "${display}: 无 ${arch} 包，使用 amd64: $(basename "${matches[0]}")"
      fi
    fi
    shopt -u nullglob

    if [ "${#matches[@]}" -eq 0 ]; then
      print_error "缺少 deb: ${prefix}_*_${arch}.deb （组件: ${display}）"
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
  print_info "顺序: ocs2_ros2 → robot-descriptions-common → arms-ros2-control"

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

  # 按安装顺序反向卸载：arms-ros2-control -> robot-descriptions-common -> ocs2_ros2
  for ((i=${#DEB_RELEASE_SOURCES[@]}-1; i>=0; i--)); do
    uninstall_entries+=("${DEB_RELEASE_SOURCES[i]}")
  done

  total="${#uninstall_entries[@]}"
  print_info "按逆序卸载 ${total} 个 deb 对应系统包..."
  print_info "顺序: arms-ros2-control → robot-descriptions-common → ocs2_ros2"

  i=1
  for entry in "${uninstall_entries[@]}"; do
    IFS='|' read -r _owner_repo display prefix <<< "${entry}"
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
# 此目录在发布包中已清空；对应功能由 deb 包提供（ocs2 / common / arms）。
# 真机 HI 与描述见 RELEASE_SUBMODULE_PATHS（robot-descriptions-arx / arx-ros2-control）。
# 若需完整源码开发，请在工作空间中执行: ./init_repo.sh
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

# src/ 下既非打包保留、也未在 .gitmodules 注册的目录（切换分支后的残留）
_each_orphan_src_path() {
  local path all_gitmodules_paths=() keep skip

  if [ ! -d "${WS_DIR}/src" ]; then
    return 0
  fi

  if [ -f "${WS_DIR}/.gitmodules" ]; then
    mapfile -t all_gitmodules_paths < <(git config --file "${WS_DIR}/.gitmodules" --get-regexp path 2>/dev/null | awk '{print $2}')
  fi

  for path in "${WS_DIR}"/src/*/; do
    [ -d "${path}" ] || continue
    path="${path%/}"
    path="${path#${WS_DIR}/}"

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

    for keep in "${all_gitmodules_paths[@]}"; do
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
# $3 = arms_ros2_control deb 版本（zip 文件名）
# $4 = deb 来源通道（latest | prerelease）
create_release_zip() {
  local include_git="${1:-1}"
  local arch="${2:-$(detect_machine_arch)}"
  local version="${3:-unknown}"
  local channel="${4:-latest}"
  local zip_name zip_path git_tag=""
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

  need_cmd zip || return 1
  version="$(sanitize_zip_token "${version}")"
  channel="$(sanitize_zip_token "${channel}")"
  mkdir -p "${DIST_DIR}"
  if [[ "${include_git}" == "0" ]]; then
    git_tag="_nogit"
  fi
  zip_name="${RELEASE_WS_NAME}_${version}_${channel}_${arch}${git_tag}.zip"
  zip_path="${DIST_DIR}/${zip_name}"

  print_info "打包 zip: ${zip_path} ..."
  if [[ "${include_git}" == "1" ]]; then
    print_info "模式: 含 .git（部署机可 git pull 更新脚本）"
  else
    print_info "模式: 不含 .git（纯快照，体积更小；脚本更新需重新发 zip 或 git clone）"
    zip_excludes+=(".git/*" "*/.git/*")
  fi

  (
    cd "${WS_DIR}" || exit 1
    zip -r "${zip_path}" . -x "${zip_excludes[@]}"
  ) || return 1

  print_info "打包完成: ${zip_path} ($(du -h "${zip_path}" | cut -f1))"
  echo "${zip_path}"
  return 0
}

# 复制工作区到临时目录，避免 --package 修改维护机上的子模块状态
# $1 = 是否包含 .git（0=不含：排除 .git 与 deb 子模块源码，仅 rsync 保留的 3 个）
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
  staging="$(mktemp -d "${TMPDIR:-/tmp}/${RELEASE_WS_NAME}_release.XXXXXX")" || return 1

  while IFS= read -r path; do
    [ -n "${path}" ] || continue
    rsync_args+=(--exclude="${path}/")
  done < <(_each_orphan_src_path)

  if [[ "${include_git}" == "0" ]]; then
    rsync_args+=(--exclude='.git/')
    while IFS= read -r path; do
      [ -n "${path}" ] || continue
      rsync_args+=(--exclude="${path}/")
    done < <(_each_deb_provided_submodule_path)
    print_info "复制到临时目录（不含 .git，不复制 deb 子模块源码）: ${staging}" >&2
  else
    print_info "复制工作区到临时目录: ${staging}" >&2
  fi

  rsync "${rsync_args[@]}" "${WS_DIR}/" "${staging}/" || {
    rm -rf "${staging}"
    return 1
  }

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
# $4 = deb 来源通道（latest | prerelease；默认 latest）
do_package_release() {
  local include_git="${1:-1}"
  local package_arch="${2:-$(detect_machine_arch)}"
  local update_submodules="${3:-0}"
  local release_channel="${4:-latest}"
  local staging=""
  local orig_dist
  local orig_deb_cache=""
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

  print_info "======== 开始 Lift2S 发布打包 ========"
  print_info "目标机器人: ${RELEASE_ROBOT_NAME}"
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
  print_info "deb 来源: ${release_channel}"
  echo ""

  if [ ! -d "${WS_DIR}/.git" ]; then
    print_error "当前目录不是 git 仓库，无法处理子模块"
    return 1
  fi

  staging="$(prepare_package_staging "${include_git}")" || return 1
  orig_dist="${WS_DIR}/dist"
  orig_deb_cache="${DEB_CACHE_DIR}"
  saved_ws=("${WS_DIR}" "${DEB_CACHE_DIR}" "${DIST_DIR}")
  trap package_release_cleanup EXIT

  WS_DIR="${staging}"
  DEB_CACHE_DIR="${WS_DIR}/.deb_cache"
  DIST_DIR="${orig_dist}"

  remove_orphan_src_paths || return 1

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

  print_info "准备发布包 .deb_cache（仅 ${package_arch} 最新 deb，channel=${release_channel}）..."
  rm -rf "${DEB_CACHE_DIR}"
  mkdir -p "${DEB_CACHE_DIR}"
  download_debs "${package_arch}" "${release_channel}" "${orig_deb_cache}" || print_warn "部分 deb 下载失败，将继续打包"

  local package_version
  package_version="$(resolve_package_zip_version "${package_arch}")"
  print_info "zip 版本（arms_ros2_control deb）: ${package_version}"
  print_info "zip 发布通道: ${release_channel}"

  create_release_zip "${include_git}" "${package_arch}" "${package_version}" "${release_channel}" || return 1

  trap - EXIT
  package_release_cleanup

  print_info "======== 发布打包完成 ========"
  return 0
}

list_releases() {
  local entry owner_repo display prefix arch channel
  local assets_file tag

  need_cmd python3 || return 1
  arch="$(detect_machine_arch)"
  channel="$(resolve_release_channel "${1:-${RELEASE_CHANNEL}}")" || return 1

  echo ""
  if [[ "${channel}" == "prerelease" ]]; then
    echo -e "${BLUE}======== GitHub pre-release deb 一览 (tag=${PRE_RELEASE_TAG}, arch=${arch}) ========${NC}"
  else
    echo -e "${BLUE}======== GitHub Release deb 一览 (latest, arch=${arch}) ========${NC}"
  fi
  echo -e "${BLUE}安装顺序: 1. ocs2_ros2  2. robot-descriptions-common  3. arms-ros2-control${NC}"
  printf "%-28s | %-12s | %s\n" "组件" "Tag" "deb 文件"
  printf "%-28s-+-%-12s-+-%s\n" "----------------------------" "------------" "----------------------------------------"

  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r owner_repo display prefix <<< "${entry}"
    assets_file="$(mktemp)"
    if ! fetch_release_deb_assets "${owner_repo}" "${channel}" > "${assets_file}" 2>/dev/null; then
      if [[ "${channel}" == "prerelease" ]]; then
        printf "%-28s | %-12s | %s\n" "${display}" "-" "无 pre-release"
      else
        printf "%-28s | %-12s | %s\n" "${display}" "-" "无 Release"
      fi
      rm -f "${assets_file}"
      continue
    fi
    tag="$(head -n1 "${assets_file}" | cut -f1)"
    if _sel_line="$(select_deb_asset "${arch}" "${prefix}" "${assets_file}" 2>/dev/null)" && [ -n "${_sel_line}" ]; then
      _deb_name="$(echo "${_sel_line}" | cut -f2)"
      printf "%-28s | %-12s | %s\n" "${display}" "${tag}" "${_deb_name}"
    else
      printf "%-28s | %-12s | %s\n" "${display}" "${tag}" "(无 ${arch} 匹配包)"
    fi
    rm -f "${assets_file}"
  done
  echo -e "${BLUE}=======================================================${NC}"
  echo ""
  echo "Release 页面:"
  for entry in "${DEB_RELEASE_SOURCES[@]}"; do
    IFS='|' read -r owner_repo _display _prefix <<< "${entry}"
    echo "  - https://github.com/${owner_repo}/releases"
  done
}

show_interactive_menu() {
  local choice

  while true; do
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    Lift2S 部署 / 发布${NC}"
    echo -e "${BLUE}  Workspace: ${WS_DIR}${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo "请选择操作:"
    echo "  1) 下载 deb（GitHub Release → .deb_cache/）"
    echo "  2) 安装 deb（从 .deb_cache/，apt 自动拉依赖，需 sudo）"
    echo "  3) 一键卸载 deb（按安装逆序，需 sudo）"
    echo "  4) 发布打包 zip（含 .git）"
    echo "  5) 发布打包 zip（不含 .git，体积更小）"
    echo "  0) 退出"
    echo ""
    read -r -p "请输入选项 [0-5]: " choice

    case "${choice}" in
      1)
        local _dl_channel
        _dl_channel="$(prompt_release_channel)"
        if [ -n "${_dl_channel}" ]; then
          download_debs "" "${_dl_channel}"
        else
          print_info "已取消"
        fi
        ;;
      2)
        echo ""
        echo "安装顺序:"
        echo "  1) ocs2_ros2"
        echo "  2) robot-descriptions-common"
        echo "  3) arms-ros2-control"
        echo ""
        do_install_debs
        ;;
      3)
        echo ""
        echo "卸载顺序（安装顺序倒序）:"
        echo "  1) arms-ros2-control"
        echo "  2) robot-descriptions-common"
        echo "  3) ocs2_ros2"
        echo ""
        uninstall_debs
        ;;
      4)
        echo ""
        local _pkg_channel
        _pkg_channel="$(prompt_release_channel)"
        if [ -n "${_pkg_channel}" ]; then
          print_info "在临时目录打包（含 .git，deb=${_pkg_channel}），不会修改当前工作区子模块"
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
        else
          print_info "已取消"
        fi
        ;;
      5)
        echo ""
        local _pkg_arch _pkg_channel
        _pkg_arch="$(prompt_package_arch)"
        _pkg_channel="$(prompt_release_channel)"
        if [ -n "${_pkg_arch}" ] && [ -n "${_pkg_channel}" ]; then
          print_info "在临时目录打包（不含 .git，架构 ${_pkg_arch}，deb=${_pkg_channel}），不会修改当前工作区子模块"
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

整机部署（robot:=${RELEASE_ROBOT_NAME}）；
发布 zip 保留源码子模块：${PACK_SOURCE_SUBMODULES[*]}

选项:
  --download            从 GitHub Release 下载 deb 到 ${DEB_CACHE_DIR}/
  --install             从 .deb_cache/ 按顺序 apt 安装 deb（需 sudo）
  --uninstall           按安装逆序卸载系统中的 deb 包（需 sudo）
  --package             维护者：打包 zip（含 .git；deb 架构=本机）
  --package-no-git      维护者：打包 zip（不含 .git；可配合 --arch）
  --arch <amd64|arm64>  与 --package-no-git 联用，指定 deb/zip 目标架构
  --release-channel <latest|prerelease>
                        deb 来源：latest=最新正式 Release（默认）；prerelease=tag ${PRE_RELEASE_TAG}
  --update-submodules   打包前从远程拉取保留子模块最新（默认用当前已检出版本）
  --list                维护者：列出 GitHub Release deb 信息（可配合 --release-channel）
  -h, --help            显示帮助

典型部署流程:
  1. 解压发布 zip（已含 .deb_cache/ 时可跳过下载）
     或: git clone -b ${RELEASE_GIT_BRANCH} git@github.com:fiveages-sim/open-deploy-ws.git ${RELEASE_WS_NAME}
  2. ./release.sh --install
  3. ./quick_start.sh   # 编译描述 + arx_ros2_control，再启动

若 zip 含 .git，可更新工作区脚本（需 git remote 访问权限）:
  git pull --ff-only origin ${RELEASE_GIT_BRANCH}
  git submodule update --init -- ${PACK_SOURCE_SUBMODULES[*]}
  （不含 .git 的 *_nogit.zip 需重新发 zip 或自行 clone 主仓 ${RELEASE_GIT_BRANCH} 分支）

若缺少 deb 或需更新版本:
  ./release.sh --download && ./release.sh --install

deb 安装顺序（读取 ${DEB_CACHE_DIR}/ 下已打包的 deb）:
  1. ros-${ROS_DISTRO}-ocs2_*_<arch>.deb
  2. ros-${ROS_DISTRO}-robot-descriptions-common_*_<arch>.deb
  3. ros-${ROS_DISTRO}-arms-ros2-control_*_<arch>.deb

deb 卸载顺序（安装顺序倒序）:
  1. ros-${ROS_DISTRO}-arms-ros2-control
  2. ros-${ROS_DISTRO}-robot-descriptions-common
  3. ros-${ROS_DISTRO}-ocs2

发布打包 (--package / --package-no-git) 流程（在临时目录执行，不修改当前工作区）:
  1. 排除 src/ 下非保留、非 .gitmodules 的残留目录
  2. --package: 检出保留子模块 + 清空 deb 子模块为占位
     --package-no-git: 仅 rsync 保留子模块文件，deb 子模块不复制、只建占位
  3. 清空 .deb_cache/ 后仅拉取目标架构 deb（不打包旧版本；可选 latest / prerelease）
  4. 生成 ${DIST_DIR}/${RELEASE_WS_NAME}_<arms版本>_<latest|prerelease>_<架构>[_nogit].zip 并删除临时目录
     （版本取自 .deb_cache 中 ros-${ROS_DISTRO}-arms-ros2-control deb 文件名）

打包前若需最新子模块，请加 --update-submodules（或先 ./init_repo.sh 更新后再打包）

示例:
  ./release.sh --package-no-git --arch arm64
  ./release.sh --package-no-git --arch arm64 --release-channel prerelease
  ./release.sh --download --release-channel prerelease
  ./release.sh --list --release-channel prerelease
  ./release.sh --package --update-submodules
EOF
}

run_package_release() {
  local include_git="$1"
  shift
  local deb_arch=""
  local update_submodules=0
  local release_channel="${RELEASE_CHANNEL}"

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
          print_error "--release-channel 需要参数（latest 或 prerelease）"
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

  do_package_release "${include_git}" "${deb_arch}" "${update_submodules}" "${release_channel}"
}

run_list_releases() {
  shift
  local release_channel="${RELEASE_CHANNEL}"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --release-channel)
        if [[ -z "${2:-}" ]]; then
          print_error "--release-channel 需要参数（latest 或 prerelease）"
          return 1
        fi
        release_channel="$2"
        shift 2
        ;;
      *)
        print_error "未知参数: $1"
        return 1
        ;;
    esac
  done

  list_releases "${release_channel}"
}

run_download_debs() {
  shift
  local deb_arch=""
  local release_channel="${RELEASE_CHANNEL}"

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
          print_error "--release-channel 需要参数（latest 或 prerelease）"
          return 1
        fi
        release_channel="$2"
        shift 2
        ;;
      *)
        print_error "未知参数: $1"
        return 1
        ;;
    esac
  done

  release_channel="$(resolve_release_channel "${release_channel}")" || return 1
  if [[ -n "${deb_arch}" ]]; then
    deb_arch="$(resolve_deb_arch "${deb_arch}")" || return 1
  fi
  download_debs "${deb_arch}" "${release_channel}"
}

main() {
  case "${1:-}" in
    -h|--help)
      usage
      exit 0
      ;;
    --list)
      run_list_releases "$@"
      exit $?
      ;;
    --download)
      run_download_debs "$@"
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
