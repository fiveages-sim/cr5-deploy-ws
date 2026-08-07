#!/usr/bin/env bash
# 共用函数库：由 release.sh 与 scripts/install_core_debs.sh source 引入
# 集中管理颜色输出、GitHub API 访问、下载与 deb 校验等通用逻辑，避免跨脚本重复。

# 颜色
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
  echo "${v%"${v##*[![:space:]]}"}"
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

# conf 中的 tag（如 v1.5.0）→ deb 文件名中的版本段（1.5.0）
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

# 可选 GH_TOKEN / GITHUB_TOKEN 提高 API 限额；API 限流时回退 HTML 解析（无需 token）
github_curl() {
  local token="${GH_TOKEN:-${GITHUB_TOKEN:-}}"
  if [ -n "$token" ]; then
    curl -fsSL -H "Authorization: Bearer ${token}" -H "Accept: application/vnd.github+json" "$@"
  else
    curl -fsSL "$@"
  fi
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

deb_file_size() {
  stat -c%s "$1" 2>/dev/null || stat -f%z "$1" 2>/dev/null || wc -c < "$1"
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

# 清理缓存中某前缀的旧 deb（保留指定文件，同时清理 .part 半成品）
# $1 = 缓存目录  $2 = 包前缀  $3 = 保留的文件名（basename）
prune_deb_cache() {
  local cache_dir="$1" prefix="$2" keep_file="$3"
  local f bn

  [[ -d "$cache_dir" ]] || return 0

  shopt -s nullglob
  for f in "${cache_dir}/${prefix}"_*.deb "${cache_dir}/${prefix}"_*.deb.part; do
    bn="$(basename "$f")"
    [[ "$bn" == "$keep_file" || "$bn" == "${keep_file}.part" ]] && continue
    print_info "移除旧 deb: ${bn}"
    rm -f "$f"
  done
  shopt -u nullglob
}
