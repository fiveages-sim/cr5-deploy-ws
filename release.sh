#!/usr/bin/env bash

# HighTorque Panthera HT 工作空间 — deb 安装与发布打包脚本
# 使用方：解压发布 zip → ./release.sh --install → ./quick_start.sh
#         --package 含 .git（可 git pull）；--package-no-git 为纯快照包（更小）
#
# deb 发布通道（下载/安装/打包时选择，默认 conf）：
#   conf        — 按 deb_versions.conf 固定 tag（固定版本直接拼接下载路径）
#   latest      — GitHub Latest 稳定版（仅 GitHub API 查询）
#   pre-release — 各仓库 pre-release 浮动标签（仅 GitHub API 查询）
# arms 打包固定为 full 变体（含 ht_ros2_control）
# deb 安装顺序: ocs2 → common → arms-ros2-control-full
# 打包 zip 中仅保留 src/robot-descriptions-ht 源码，其余子模块只保留目录
# 打包通道非 conf 时，zip 内 deb_versions.conf 的 tag 改为与打包通道一致
#
# 用法:
#   ./release.sh              # 交互式菜单
#   ./release.sh --download [--channel <conf|latest|pre-release>]
#                            # 按通道下载/校验 deb 到 .deb_cache/
#   ./release.sh --install    # 从 .deb_cache/ 安装 deb（需 sudo）
#   ./release.sh --uninstall  # 按逆序卸载系统中的 deb 包（需 sudo）
#   ./release.sh --package [--channel <conf|latest|pre-release>] [--update-submodules]
#                            # 维护者：打包 zip（含 .git）
#   ./release.sh --package-no-git --arch <amd64|arm64> [--channel ...]
#                            # 维护者：打包 zip（不含 .git）
#   ./release.sh --list       # 维护者：查看 conf 对应 Release deb 信息

set -u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${WS_DIR}"
DEB_CACHE_DIR="${WS_DIR}/.deb_cache"
DIST_DIR="${WS_DIR}/dist"
DEB_CONF="${WS_DIR}/deb_versions.conf"
DEB_INSTALL_PATHS=()

. "${WS_DIR}/scripts/lib_common.sh"
. "${WS_DIR}/scripts/lib_github.sh"
. "${WS_DIR}/scripts/lib_deb.sh"

# 打包时保留的源码子模块（其余由 deb 提供，含 ht_ros2_control via arms-full）
PACK_SOURCE_SUBMODULES=(
  "src/robot-descriptions-ht"
  "src/drag_teleop_controller"
)

# 打包时 arms 总是 full 变体
ARMS_VARIANT="full"

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
  print_info "候选导入完成（新复制 ${copied} 个；将按 ${DEB_CHANNEL} 通道校验版本）"
}

# 从 GitHub Release 拉取 deb 到 .deb_cache/（按 DEB_CHANNEL 通道）
# $1 = 目标架构（可选，默认本机架构）
download_debs() {
  local arch="${1:-}"
  local entry fail=0

  need_cmd python3 || return 1
  if [ -z "${arch}" ]; then
    arch="$(detect_machine_arch)"
  fi
  print_info "按通道 ${DEB_CHANNEL} 同步 deb 到 ${DEB_CACHE_DIR} (arch=${arch}) ..."

  mkdir -p "${DEB_CACHE_DIR}"

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

# 逆序卸载（复用 lib_deb.sh 的 uninstall_core_debs，卸载全部）
uninstall_debs() {
  uninstall_core_debs ""
}

get_submodule_default_branch() {
  local path="$1"
  git config -f "${WS_DIR}/.gitmodules" --get "submodule.${path}.branch" 2>/dev/null || echo "main"
}

# 打包用子模块：同步配置并检出当前已记录/已检出的版本（默认不 fetch）
# $1 = 1 时额外拉取各子模块默认分支最新提交
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
# 此目录在发布包中已清空；对应功能由 deb 包提供（arms-full 含 ht_ros2_control）。
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

# 含 .git 打包：deinit 并清空 deb 子模块目录（只保留目录占位）
clear_unused_submodules() {
  local path

  cd "${WS_DIR}" || return 1

  print_info "清空 deb 子模块目录（只保留目录）..."
  while IFS= read -r path; do
    [ -n "${path}" ] || continue
    print_info "  清空: ${path} ..."
    git submodule deinit -f -- "${path}" 2>/dev/null || true
    rm -rf ".git/modules/${path}"
    find "${path}" -mindepth 1 -maxdepth 1 -exec rm -rf {} + 2>/dev/null || true
    _write_deb_submodule_placeholder "${path}"
  done < <(_each_deb_provided_submodule_path)

  return 0
}

# 打包通道非 conf 时：把 zip 内 deb_versions.conf 的 tag 改为与打包通道一致
# 实现见 scripts/lib_deb.sh 的 rewrite_conf_tag_to_channel

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
  zip_name="ht_deploy_ws_${ts}_${arch}${git_tag}.zip"
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
  staging="$(mktemp -d "${TMPDIR:-/tmp}/ht_deploy_ws_release.XXXXXX")" || return 1

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
do_package_release() {
  local include_git="${1:-1}"
  local package_arch="${2:-$(detect_machine_arch)}"
  local update_submodules="${3:-0}"
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
      DEB_CONF="${saved_ws[3]}"
    fi
  }

  print_info "======== 开始发布打包 ========"
  print_info "deb 来源通道: ${DEB_CHANNEL}（arms 固定 full 变体）"
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
  saved_ws=("${WS_DIR}" "${DEB_CACHE_DIR}" "${DIST_DIR}" "${DEB_CONF}")
  trap package_release_cleanup EXIT

  WS_DIR="${staging}"
  DEB_CACHE_DIR="${WS_DIR}/.deb_cache"
  DIST_DIR="${orig_dist}"
  DEB_CONF="${WS_DIR}/deb_versions.conf"
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

  # 打包通道非 conf 时：修改 zip 内 deb_versions.conf 的 tag 与打包通道一致
  if [ "${DEB_CHANNEL}" != "conf" ]; then
    rewrite_conf_tag_to_channel "${DEB_CONF}" "${DEB_CHANNEL}"
  fi

  print_info "按通道 ${DEB_CHANNEL} 准备 .deb_cache（缓存匹配则复用，否则下载）..."
  mkdir -p "${DEB_CACHE_DIR}"
  seed_deb_cache_from "${host_deb_cache}" "${package_arch}"
  if ! download_debs "${package_arch}"; then
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
    if ! fetch_release_assets_with_size "${owner_repo}" "${conf_tag}" > "${assets_file}" 2>/dev/null; then
      printf "%-36s | %-12s | %s\n" "${display}" "${conf_tag}" "无 Release"
      rm -f "${assets_file}"
      continue
    fi
    tag="$(head -n1 "${assets_file}" | cut -f1)"
    mapfile -t _asset_names < <(cut -d'|' -f1 "${assets_file}")
    if _sel_line="$(select_asset_name "${arch}" "${prefix}" "${_asset_names[@]}")" && [ -n "${_sel_line}" ]; then
      printf "%-36s | %-12s | %s\n" "${display}" "${conf_tag}" "${_sel_line}"
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

show_interactive_menu() {
  local choice

  while true; do
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    Panthera HT 部署 / 发布${NC}"
    echo -e "${BLUE}  Workspace: ${WS_DIR}${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo "请选择操作:"
    echo "  1) 下载 deb 依赖包（按发布通道到 .deb_cache）"
    echo "  2) 安装 deb 依赖包（从 .deb_cache 安装，需 sudo）"
    echo "  3) 一键卸载 deb（按安装逆序，需 sudo）"
    echo "  4) 发布打包 zip（含 .git）"
    echo "  5) 发布打包 zip（不含 .git，体积更小）"
    echo "  0) 退出"
    echo ""
    read -r -p "请输入选项 [0-5]: " choice

    case "${choice}" in
      1)
        prompt_channel
        download_debs
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
        print_info "在临时目录打包（含 .git），不会修改当前工作区子模块"
        print_info "请选择本次打包的 deb 来源通道："
        prompt_channel
        read -r -p "确认继续？[y/N]: " confirm
        case "${confirm}" in
          y|Y|yes|YES)
            if do_package_release 1; then
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
        local _pkg_arch
        _pkg_arch="$(prompt_package_arch)"
        if [ -n "${_pkg_arch}" ]; then
          print_info "在临时目录打包（不含 .git，架构 ${_pkg_arch}），不会修改当前工作区子模块"
          print_info "请选择本次打包的 deb 来源通道："
          prompt_channel
          read -r -p "确认继续？[y/N]: " confirm
          case "${confirm}" in
            y|Y|yes|YES)
              if do_package_release 0 "${_pkg_arch}"; then
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
  --download [--channel <conf|latest|pre-release>]
                        按发布通道下载 deb 到 ${DEB_CACHE_DIR}/（缓存匹配则复用）
                        conf 固定版本直接使用 releases/download URL（不查 GitHub API）；
                        latest / pre-release 仅通过 GitHub API 查询下载地址
  --install             从 .deb_cache/ 按顺序 apt 安装 deb（需 sudo）
  --uninstall           按安装逆序卸载系统中的 deb 包（需 sudo）
  --package [--channel <conf|latest|pre-release>] [--update-submodules]
                        维护者：打包 zip（含 .git；deb 架构=本机）
  --package-no-git --arch <amd64|arm64> [--channel ...]
                        维护者：打包 zip（不含 .git）
  --arch <amd64|arm64>  与 --package-no-git 联用，指定 deb/zip 目标架构
  --channel <conf|latest|pre-release>
                        发布通道（默认 conf；打包/下载时可用，--package-no-git 时优先）
  --update-submodules   打包前从远程拉取保留子模块最新（默认用当前已检出版本）
  --list                维护者：列出 deb_versions.conf 对应 Release deb
  -h, --help            显示帮助

典型部署流程:
  1. 解压发布 zip（已含 .deb_cache/ 时可跳过下载）
  2. ./release.sh --install
  3. ./quick_start.sh

若 zip 含 .git，可更新工作区脚本（需 git remote 访问权限）:
  git pull --ff-only
  git submodule update --init -- src/robot-descriptions-ht
  （不含 .git 的 *_nogit.zip 需重新发 zip 或自行 clone 主仓）

若缺少 deb 或需更新版本:
  ./release.sh --download && ./release.sh --install

deb 安装顺序（读取 ${DEB_CACHE_DIR}/ 下已打包的 deb）:
  1. ros-${ROS_DISTRO}-ocs2_*_<arch>.deb
  2. ros-${ROS_DISTRO}-robot-descriptions-common_*_<arch>.deb
  3. ros-${ROS_DISTRO}-arms-ros2-control-full_*_<arch>.deb

deb 卸载顺序（安装顺序倒序）:
  1. ros-${ROS_DISTRO}-arms-ros2-control-full
  2. ros-${ROS_DISTRO}-robot-descriptions-common
  3. ros-${ROS_DISTRO}-ocs2

发布打包 (--package / --package-no-git) 流程（在临时目录执行，不修改当前工作区）:
  1. 选择 deb 来源通道（conf / latest / pre-release）；arms 固定 full
  2. --package: 检出保留子模块 + 清空 deb 子模块为占位
     --package-no-git: 仅 rsync 保留子模块文件，deb 子模块不复制、只建占位
  3. 通道非 conf 时，zip 内 deb_versions.conf 的 tag 改为与打包通道一致
  4. 导入工作区 .deb_cache 候选；按通道校验版本，匹配则复用，否则下载
  5. 生成 ${DIST_DIR}/ht_deploy_ws_<时间>_<架构>[_nogit].zip 并删除临时目录

打包前若需最新子模块，请加 --update-submodules（或运行 ./init_repo.sh 后重新打包）

示例:
  ./release.sh --package-no-git --arch arm64 --channel pre-release
  ./release.sh --package --channel latest --update-submodules
EOF
}

run_package_release() {
  local include_git="$1"
  shift
  local deb_arch=""
  local update_submodules=0

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
      --channel)
        set_deb_channel "${2:-}" || return 1
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

  do_package_release "${include_git}" "${deb_arch}" "${update_submodules}"
}

main() {
  case "${1:-}" in
    -h|--help)
      usage
      exit 0
      ;;
  esac

  load_deb_conf || exit 1

  case "${1:-}" in
    --list)
      list_releases
      exit 0
      ;;
    --download)
      shift
      while [[ $# -gt 0 ]]; do
        case "$1" in
          --channel)
            set_deb_channel "${2:-}" || exit 1
            shift 2
            ;;
          *)
            print_error "未知参数: $1"
            usage
            exit 1
            ;;
        esac
      done
      download_debs
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
