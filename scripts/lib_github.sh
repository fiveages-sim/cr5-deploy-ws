#!/usr/bin/env bash
# GitHub API / 下载 / deb 校验（依赖 lib_common.sh）

resolve_github_api_token() {
  if [ -n "${GITHUB_TOKEN:-}" ]; then printf '%s' "${GITHUB_TOKEN}"; return 0; fi
  if [ -n "${GH_TOKEN:-}" ]; then printf '%s' "${GH_TOKEN}"; return 0; fi
  if command -v gh >/dev/null 2>&1; then gh auth token 2>/dev/null || true; fi
}

print_github_rate_limit_help() {
  print_error "GitHub API 速率限制（未认证请求约 60 次/小时，同一公网 IP 共享）"
  print_info "请任选其一后重试:"
  print_info "  export GH_TOKEN=ghp_xxxxxxxx   # Personal Access Token（public_repo 读权限即可）"
  print_info "  gh auth login                  # 使用 GitHub CLI 登录"
}

github_curl() {
  local token="${GH_TOKEN:-${GITHUB_TOKEN:-}}"
  if [ -n "$token" ]; then
    curl -fsSL -H "Authorization: Bearer ${token}" -H "Accept: application/vnd.github+json" "$@"
  else
    curl -fsSL "$@"
  fi
}

download_file() {
  local url="$1" dest="$2"
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

# 查询 release 资产，输出 name|size（GitHub API 失败时回退 Release 页面 HTML 解析）
fetch_release_assets_with_size() {
  local repo="$1" tag="$2"
  local api_out
  api_out="$(github_curl "https://api.github.com/repos/${repo}/releases/tags/${tag}" 2>/dev/null \
    | python3 -c "
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(1)
for a in data.get('assets', []):
    print(str(a['name']) + '|' + str(a['size']))
" 2>/dev/null || true)"
  if [[ -n "$api_out" ]]; then
    printf '%s\n' "$api_out"
    return 0
  fi
  print_warn "  GitHub API 不可用，改从 Release 页面解析 deb 列表（${repo}@${tag}）" >&2
  curl -fsSL "https://github.com/${repo}/releases/expanded_assets/${tag}" 2>/dev/null \
    | grep -oE '[A-Za-z0-9._~-]+\.deb' \
    | sort -u
}

lookup_asset_size() {
  local asset_name="$1"; shift
  local line name size
  for line in "$@"; do
    name="${line%%|*}"
    size="${line#*|}"
    if [[ "$name" == "$asset_name" ]]; then
      printf '%s' "$size"
      return 0
    fi
  done
  return 1
}

release_exists() {
  local repo="$1" tag="$2"
  local code
  code="$(github_curl -o /dev/null -w '%{http_code}' "https://api.github.com/repos/${repo}/releases/tags/${tag}" 2>/dev/null || true)"
  [[ "$code" == "200" ]] && return 0
  code="$(curl -fsSL -o /dev/null -w '%{http_code}' "https://github.com/${repo}/releases/tag/${tag}" 2>/dev/null || true)"
  [[ "$code" == "200" ]]
}

# 获取 latest 稳定版 tag（排除 draft / prerelease）
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
try:
    releases = json.load(sys.stdin)
except Exception:
    sys.exit(1)
for r in releases:
    if r.get("draft") or r.get("prerelease"):
        continue
    t = r.get("tag_name", "")
    if t:
        print(t)
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

# 获取 pre-release tag：优先固定浮动标签 pre-release，否则取最新 prerelease
fetch_prerelease_tag() {
  local repo="$1" tag=""
  if release_exists "$repo" "pre-release"; then
    printf '%s\n' "pre-release"
    return 0
  fi
  tag="$(github_curl "https://api.github.com/repos/${repo}/releases?per_page=30" 2>/dev/null \
    | python3 -c '
import json, sys
try:
    releases = json.load(sys.stdin)
except Exception:
    sys.exit(1)
for r in releases:
    if r.get("draft") or not r.get("prerelease"):
        continue
    t = r.get("tag_name", "")
    if t:
        print(t)
        break
' 2>/dev/null || true)"
  if [[ -n "$tag" ]]; then
    printf '%s\n' "$tag"
    return 0
  fi
  return 1
}
