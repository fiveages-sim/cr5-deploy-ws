#!/usr/bin/env bash
# 子模块操作（依赖 lib_common.sh）
# 全局：REPO_DIR（必填）、USE_DEB_OCS2 / USE_DEB_ARMS / USE_DEB_COMMON（0/1）、ARMS_VARIANT（full/standard）
# 安全约束：
#   1) 子模块更新只做快进（--ff-only），绝不切换分支；
#   2) 本地修改先 git stash 暂存、更新成功后恢复，绝不 reset --hard；
#   3) 顶层保护：若进入的 git 仓库就是工作空间自身，直接跳过。

# 列出 .gitmodules 中所有子模块路径
all_submodule_paths() {
  git config --file "${REPO_DIR}/.gitmodules" --get-regexp path 2>/dev/null | awk '{print $2}'
}

path_has_content() {
  local path="$1"
  [ -d "${REPO_DIR}/${path}" ] || return 1
  [ -n "$(ls -A "${REPO_DIR}/${path}" 2>/dev/null)" ]
}

path_is_git_checkout() {
  local path="$1"
  [ -e "${REPO_DIR}/${path}/.git" ] || return 1
  (cd "${REPO_DIR}/${path}" && git rev-parse --git-dir >/dev/null 2>&1)
}

# 顶层子模块：已选 deb 的模块跳过；arms-full 时跳过 src/ht-ros2-control
should_skip_top_submodule() {
  local path="$1"
  case "${path}" in
    src/ocs2_ros2) [ "${USE_DEB_OCS2:-0}" -eq 1 ] && return 0 ;;
    src/arms_ros2_control) [ "${USE_DEB_ARMS:-0}" -eq 1 ] && return 0 ;;
    src/robot-descriptions-common) [ "${USE_DEB_COMMON:-0}" -eq 1 ] && return 0 ;;
    src/ht-ros2-control) [ "${USE_DEB_ARMS:-0}" -eq 1 ] && [ "${ARMS_VARIANT:-full}" = "full" ] && return 0 ;;
  esac
  return 1
}

# 嵌套子模块：arms 源码模式时不初始化 hardwares/*
should_skip_nested_submodule() {
  local parent="$1" nested="$2"
  if [ "${parent}" = "src/arms_ros2_control" ] && [[ "${nested}" == hardwares/* ]]; then
    return 0
  fi
  return 1
}

# 将指定子模块更新到当前分支的最新提交（仅快进，不切换分支；本地修改 stash 恢复）
# $1 = 子模块路径（相对 REPO_DIR）
update_submodule_to_latest() {
  local path="$1"
  local stashed=0

  if [ ! -d "${REPO_DIR}/${path}" ]; then
    print_warn "子模块路径不存在: ${path}"
    return 0
  fi

  print_info "处理子模块: ${path}"
  (
    cd "${REPO_DIR}/${path}" || { print_warn "无法进入目录 ${path}，跳过"; exit 0; }

    if ! git rev-parse --git-dir >/dev/null 2>&1; then
      print_warn "子模块 ${path} 不是有效的 git 仓库，跳过"
      exit 0
    fi

    # 顶层保护：绝不操作工作空间自身所在仓库
    if [ "$(git rev-parse --show-toplevel 2>/dev/null || true)" = "${REPO_DIR}" ]; then
      print_warn "跳过 ${path}：它指向顶层工作空间仓库（避免修改工作空间分支）"
      exit 0
    fi

    # 暂存本地修改（更新后恢复），绝不删除
    if ! git diff-index --quiet HEAD -- 2>/dev/null; then
      print_warn "  检测到本地修改，先暂存（更新后恢复）..."
      if ! git stash push -m "Auto-stash before submodule update" 2>/dev/null; then
        print_warn "  暂存失败，跳过更新以保留本地修改"
        exit 0
      fi
      stashed=1
    fi

    print_info "  获取远程更新..."
    git fetch origin 2>/dev/null || print_warn "  获取远程更新失败，继续..."

    current_branch="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")"
    if [ "${current_branch}" = "HEAD" ]; then
      # 游离 HEAD：仅尝试快进到 FETCH_HEAD，不切换分支
      git merge --ff-only FETCH_HEAD 2>/dev/null \
        && print_info "  已快进到远端提交" \
        || print_warn "  游离 HEAD 且无法快进，跳过（不改动分支）"
    elif git rev-parse --verify --quiet "refs/remotes/origin/${current_branch}" >/dev/null 2>&1; then
      print_info "  更新当前分支 ${current_branch} 到最新..."
      git merge --ff-only "origin/${current_branch}" 2>/dev/null \
        || print_warn "  无法快进更新（本地有未推送提交或冲突），保留当前状态"
    else
      print_warn "  远端无分支 origin/${current_branch}，跳过更新"
    fi

    # 恢复暂存的本地修改
    if [ "${stashed}" -eq 1 ]; then
      print_info "  恢复本地暂存的修改..."
      git stash pop 2>/dev/null \
        || print_warn "  暂存恢复失败（可手动执行: git -C \"${REPO_DIR}/${path}\" stash pop）"
    fi
  )
  print_info "✓ ${path} 已更新到当前分支最新（不改动分支）"
}

# 递归初始化并更新子模块（含嵌套内容，跳过 arms hardwares/*）
# $1 = 父路径（顶层为 ""）；$2 = 子路径（相对父路径）
init_submodule_recursive() {
  local parent="$1" nested="$2"
  local path rel_branch sub

  if [ -n "${parent}" ]; then
    path="${parent}/${nested}"
  else
    path="${nested}"
  fi

  # 初始化当前子模块的嵌套子模块
  if [ -e "${REPO_DIR}/${path}/.git" ] && [ -f "${REPO_DIR}/${path}/.gitmodules" ]; then
    while IFS= read -r sub; do
      if should_skip_nested_submodule "${path}" "${sub}"; then
        print_info "跳过嵌套子模块（arms 源码模式）: ${path}/${sub}"
        continue
      fi
      git -C "${REPO_DIR}/${path}" submodule update --init -- "${sub}" 2>/dev/null \
        || print_warn "初始化嵌套子模块失败: ${path}/${sub}"
    done < <(git -C "${REPO_DIR}/${path}" config --file "${REPO_DIR}/${path}/.gitmodules" --get-regexp path 2>/dev/null | awk '{print $2}')
  fi

  # 更新当前子模块到当前分支最新
  update_submodule_to_latest "${path}"

  # 递归处理嵌套子模块
  if [ -f "${REPO_DIR}/${path}/.gitmodules" ]; then
    while IFS= read -r sub; do
      should_skip_nested_submodule "${path}" "${sub}" && continue
      init_submodule_recursive "${path}" "${sub}"
    done < <(git -C "${REPO_DIR}/${path}" config --file "${REPO_DIR}/${path}/.gitmodules" --get-regexp path 2>/dev/null | awk '{print $2}')
  fi
}

# 清空子模块目录内容但保留目录本身（用于源码 → deb 切换）
# $1 = 子模块路径（相对 REPO_DIR）
clear_submodule_content() {
  local path="$1"
  [ -d "${REPO_DIR}/${path}" ] || return 0
  print_info "清空子模块内容（保留目录）: ${path}"
  git -C "${REPO_DIR}" submodule deinit -f -- "${path}" >/dev/null 2>&1 || true
  # 删除目录内所有内容（含隐藏文件），目录本身保留
  find "${REPO_DIR}/${path}" -mindepth 1 -maxdepth 1 -exec rm -rf {} + 2>/dev/null || true
  rm -rf "${REPO_DIR}/.git/modules/${path}" 2>/dev/null || true
  print_info "✓ 已清空 ${path}"
}

# 模块安装状态：mixed（deb+源码都在）| deb | source | none
# $1 = 子模块路径  $2 = deb 包名
detect_module_state() {
  local path="$1" deb_pkg="$2"
  local has_deb=0 has_src=0
  is_pkg_installed "${deb_pkg}" && has_deb=1
  # arms-full 与旧标准包都算 arms deb
  if [ "${deb_pkg}" = "ros-jazzy-arms-ros2-control-full" ]; then
    is_pkg_installed "ros-jazzy-arms-ros2-control" && has_deb=1
  fi
  path_is_git_checkout "${path}" && has_src=1
  if [ "${has_deb}" -eq 1 ] && [ "${has_src}" -eq 1 ]; then
    echo "mixed"
  elif [ "${has_deb}" -eq 1 ]; then
    echo "deb"
  elif [ "${has_src}" -eq 1 ]; then
    echo "source"
  else
    echo "none"
  fi
}

# 初始化并迭代更新所有非 deb 子模块（跳过 hardwares/*；arms-full 跳过 ht-ros2-control）
init_workspace_submodules() {
  local path
  local -a init_paths=()

  print_info "同步子模块配置..."
  git -C "${REPO_DIR}" submodule sync

  print_info "初始化顶层子模块（跳过已选 deb 的核心仓库）..."
  while IFS= read -r path; do
    should_skip_top_submodule "${path}" && continue
    init_paths+=("${path}")
  done < <(all_submodule_paths)

  if [ "${#init_paths[@]}" -gt 0 ]; then
    git -C "${REPO_DIR}" submodule update --init "${init_paths[@]}"
  else
    print_info "无需要初始化的源码子模块"
  fi

  print_info "将源码子模块更新到当前分支的最新提交（含嵌套子模块，不切换分支）..."
  for path in "${init_paths[@]}"; do
    init_submodule_recursive "" "${path}"
  done

  print_info ""
  print_info "当前子模块状态："
  git -C "${REPO_DIR}" submodule status
}

# 仅对源码子模块路径运行 rosdep
run_rosdep_install() {
  local path
  local -a rosdep_paths=()

  print_info "运行 rosdep install（仅源码子模块路径）..."
  if ! command -v rosdep >/dev/null 2>&1; then
    print_error "未找到 rosdep，请先安装 ROS 环境后重试"
    print_info "  sudo apt install python3-rosdep"
    print_info "  sudo rosdep init && rosdep update"
    return 1
  fi

  while IFS= read -r path; do
    should_skip_top_submodule "${path}" && continue
    rosdep_paths+=("${path}")
  done < <(all_submodule_paths)

  if [ "${#rosdep_paths[@]}" -eq 0 ]; then
    print_info "无源码路径需要 rosdep"
    return 0
  fi

  rosdep install --from-paths "${rosdep_paths[@]}" --ignore-src -r -y \
    || print_warn "rosdep 安装部分依赖失败，可稍后重试或检查 package.xml"
}
