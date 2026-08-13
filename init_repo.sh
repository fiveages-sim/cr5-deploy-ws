#!/usr/bin/env bash

# HighTorque Panthera HT ROS2 部署工作空间初始化脚本
# 功能：逐模块 source/deb 初始化子模块，下载并安装核心 deb，支持切换/卸载/rosdep
# 说明：
#   - 模块方式与通道每次运行选择，不保存配置
#   - 通道 conf 使用 deb_versions.conf 固定 tag（直接拼接下载路径）；
#     latest / pre-release 仅通过 GitHub API 查询下载地址
#   - arms=deb 时可选变体 full（含 ht_ros2_control，跳过 src/ht-ros2-control）或 standard
#   - 子模块更新只做快进（不切换分支），本地修改暂存并在更新后恢复

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${SCRIPT_DIR}"

. "${SCRIPT_DIR}/scripts/lib_common.sh"
. "${SCRIPT_DIR}/scripts/lib_github.sh"
. "${SCRIPT_DIR}/scripts/lib_deb.sh"
. "${SCRIPT_DIR}/scripts/lib_submodule.sh"

# 本次运行有效的模块方式（不保存）
USE_DEB_OCS2=1
USE_DEB_ARMS=1
USE_DEB_COMMON=1

module_short_to_path() {
  case "$1" in
    ocs2) echo "src/ocs2_ros2" ;;
    arms) echo "src/arms_ros2_control" ;;
    common) echo "src/robot-descriptions-common" ;;
  esac
}

module_short_to_deb() {
  case "$1" in
    ocs2) echo "ros-${ROS_DISTRO}-ocs2" ;;
    arms)
      if [ "${ARMS_VARIANT}" = "standard" ]; then
        echo "ros-${ROS_DISTRO}-arms-ros2-control"
      else
        echo "ros-${ROS_DISTRO}-arms-ros2-control-full"
      fi
      ;;
    common) echo "ros-${ROS_DISTRO}-robot-descriptions-common" ;;
  esac
}

get_use_deb_for_module() {
  case "$1" in
    ocs2) echo "$USE_DEB_OCS2" ;;
    arms) echo "$USE_DEB_ARMS" ;;
    common) echo "$USE_DEB_COMMON" ;;
    *) echo "0" ;;
  esac
}

set_use_deb_for_module() {
  case "$1" in
    ocs2) USE_DEB_OCS2="$2" ;;
    arms) USE_DEB_ARMS="$2" ;;
    common) USE_DEB_COMMON="$2" ;;
  esac
}

# 逐模块选择 source/deb（回车=deb）
prompt_sd() {
  local name="$1" var="$2" ans
  read -rp "  ${name} [D/s]（回车=deb）: " ans
  case "${ans}" in
    s|S|source) eval "${var}=0" ;;
    *) eval "${var}=1" ;;
  esac
}

mode_label() {
  if [ "$1" -eq 1 ]; then
    echo "deb"
  else
    echo "source"
  fi
}

# 清空本次选 deb 且目录有源码内容的子模块（只清内容，保留目录）
cleanup_deb_module_sources() {
  local -a paths=() p
  [ "$USE_DEB_OCS2" -eq 1 ] && path_has_content "src/ocs2_ros2" && paths+=("src/ocs2_ros2")
  [ "$USE_DEB_ARMS" -eq 1 ] && path_has_content "src/arms_ros2_control" && paths+=("src/arms_ros2_control")
  [ "$USE_DEB_COMMON" -eq 1 ] && path_has_content "src/robot-descriptions-common" && paths+=("src/robot-descriptions-common")
  if [ "$USE_DEB_ARMS" -eq 1 ] && [ "${ARMS_VARIANT}" = "full" ]; then
    path_has_content "src/ht-ros2-control" && paths+=("src/ht-ros2-control")
  fi

  [ "${#paths[@]}" -eq 0 ] && return 0

  echo "以下目录将改由 deb 提供，将清空其中内容（目录本身保留）："
  for p in "${paths[@]}"; do
    echo "  - ${p}"
    if [ "${p}" = "src/ht-ros2-control" ]; then
      echo "    （arms-full deb 已包含 ht_ros2_control）"
    fi
  done
  if ! confirm_yn "确认清空上述目录内容？"; then
    print_warn "已跳过清理（可能仍占用磁盘，且可能与 deb 冲突）"
    return 0
  fi

  for p in "${paths[@]}"; do
    clear_submodule_content "${p}"
  done
  echo ""
}

# 安装本次选择的 deb 包
install_selected_debs() {
  local -a parts=() list
  [ "$USE_DEB_OCS2" -eq 1 ] && parts+=("ocs2")
  [ "$USE_DEB_COMMON" -eq 1 ] && parts+=("common")
  [ "$USE_DEB_ARMS" -eq 1 ] && parts+=("arms")
  [ "${#parts[@]}" -eq 0 ] && return 0
  local IFS=,
  list="${parts[*]}"
  install_core_debs "${list}" || print_error "deb 安装失败，请检查 deb_versions.conf 或网络连接"
}

# ===================== 各操作流程 =====================

flow_init() {
  echo ""
  echo "核心模块安装方式（d=deb, s=source，回车=deb）："
  echo "  robot-descriptions-ht 始终源码"
  prompt_sd "ocs2_ros2" USE_DEB_OCS2
  prompt_sd "arms_ros2_control" USE_DEB_ARMS
  prompt_sd "robot-descriptions-common" USE_DEB_COMMON

  if [ "$USE_DEB_ARMS" -eq 1 ] && [ "$USE_DEB_OCS2" -eq 0 ]; then
    print_warn "arms 选 deb 而 ocs2 选 source：arms deb 通常依赖 ocs2 包，安装可能失败。"
  fi
  if [ "$USE_DEB_ARMS" -eq 1 ] && [ "$USE_DEB_COMMON" -eq 0 ]; then
    print_warn "arms 选 deb 而 common 选 source：arms deb 通常依赖 common 包，安装可能失败。"
  fi

  prompt_channel
  if [ "$USE_DEB_ARMS" -eq 1 ]; then
    prompt_arms_variant
  fi

  echo ""
  print_info "模块方式: ocs2=$(mode_label "$USE_DEB_OCS2"), arms=$(mode_label "$USE_DEB_ARMS"), common=$(mode_label "$USE_DEB_COMMON"), variant=${ARMS_VARIANT}, channel=${DEB_CHANNEL}"
  echo ""

  cleanup_deb_module_sources
  init_workspace_submodules
  run_rosdep_install
  install_selected_debs
}

flow_switch() {
  local m p pkg st tgt
  echo ""
  echo "当前模块状态："
  for m in ocs2 arms common; do
    p="$(module_short_to_path "$m")"
    pkg="$(module_short_to_deb "$m")"
    st="$(detect_module_state "${p}" "${pkg}")"
    print_info "  ${m}: ${st}"
  done

  echo ""
  echo "为每个模块选择目标（s=source, d=deb, k=keep，回车=keep）："
  for m in ocs2 arms common; do
    p="$(module_short_to_path "$m")"
    pkg="$(module_short_to_deb "$m")"
    st="$(detect_module_state "${p}" "${pkg}")"
    read -rp "  ${m} 当前=${st} [s/d/K]: " tgt
    tgt="${tgt:-k}"
    case "${tgt}" in
      s|S|source) set_use_deb_for_module "${m}" 0 ;;
      d|D|deb) set_use_deb_for_module "${m}" 1 ;;
      *)
        case "${st}" in
          deb) set_use_deb_for_module "${m}" 1 ;;
          *) set_use_deb_for_module "${m}" 0 ;;
        esac
        ;;
    esac
  done

  if [ "$USE_DEB_OCS2" -eq 1 ] || [ "$USE_DEB_ARMS" -eq 1 ] || [ "$USE_DEB_COMMON" -eq 1 ]; then
    prompt_channel
  fi
  if [ "$USE_DEB_ARMS" -eq 1 ]; then
    prompt_arms_variant
  fi

  # deb → source：卸载对应已装 deb（避免与源码冲突）
  local -a to_uninstall=()
  for m in ocs2 arms common; do
    if [ "$(get_use_deb_for_module "${m}")" -eq 0 ]; then
      pkg="$(module_short_to_deb "${m}")"
      if is_pkg_installed "${pkg}"; then
        to_uninstall+=("${m}")
      elif [ "${m}" = "arms" ] && is_pkg_installed "ros-${ROS_DISTRO}-arms-ros2-control"; then
        to_uninstall+=("${m}")
      fi
    fi
  done
  if [ "${#to_uninstall[@]}" -gt 0 ]; then
    local IFS=,
    print_info "以下模块改为源码，将先卸载对应 deb: ${to_uninstall[*]}"
    uninstall_core_debs "${to_uninstall[*]}" || print_warn "部分 deb 卸载失败，继续..."
  fi

  cleanup_deb_module_sources
  init_workspace_submodules
  install_selected_debs
}

flow_deb_only() {
  local only
  echo ""
  echo "当前已安装的核心 deb："
  show_installed_core_debs
  echo ""
  read -rp "选择要安装/更新的包（逗号分隔，如 ocs2,common；回车=全部）: " only
  only="$(echo "${only}" | tr -d '[:space:]')"

  prompt_channel
  if [ -z "${only}" ] || [[ ",${only}," == *,arms,* ]]; then
    prompt_arms_variant
  fi

  install_core_debs "${only}" || exit 1
}

flow_deb_uninstall() {
  local only
  echo ""
  echo "当前已安装的核心 deb："
  show_installed_core_debs
  echo ""
  read -rp "选择要卸载的包（逗号分隔，如 ocs2,common；回车=全部）: " only
  only="$(echo "${only}" | tr -d '[:space:]')"

  uninstall_core_debs "${only}" || exit 1
}

flow_submodule_only() {
  local m pkg installed_any=0
  echo ""
  echo "当前已安装的核心 deb："
  show_installed_core_debs
  echo ""
  echo "将不会拉取已安装 deb 对应模块的源码子模块："
  for m in ocs2 arms common; do
    pkg="$(module_short_to_deb "${m}")"
    if is_pkg_installed "${pkg}"; then
      echo "  - $(module_short_to_path "${m}")（${pkg} 已安装）"
      installed_any=1
      set_use_deb_for_module "${m}" 1
    elif [ "${m}" = "arms" ] && is_pkg_installed "ros-${ROS_DISTRO}-arms-ros2-control"; then
      # 兼容旧版标准 arms deb
      echo "  - $(module_short_to_path "${m}")（ros-${ROS_DISTRO}-arms-ros2-control 已安装）"
      installed_any=1
      set_use_deb_for_module "${m}" 1
    else
      set_use_deb_for_module "${m}" 0
    fi
  done
  # arms-full 已装时跳过 ht-ros2-control
  if is_pkg_installed "ros-${ROS_DISTRO}-arms-ros2-control-full"; then
    echo "  - src/ht-ros2-control（arms-full 已含 ht_ros2_control）"
    echo "  如需手动拉取该源码子模块，可执行："
    echo "    git submodule update --init src/ht-ros2-control"
  fi
  [ "${installed_any}" -eq 1 ] || echo "  （无已安装的核心 deb）"

  if ! confirm_yn "确认开始拉取子模块？"; then
    echo "已取消"
    return 0
  fi

  init_workspace_submodules
}

flow_rosdep() {
  run_rosdep_install || exit 1
}

# ===================== 主入口 =====================

cd "${REPO_DIR}" || exit 1

if [ ! -d ".git" ]; then
  print_error "当前目录不是 git 仓库！"
  print_info "请先克隆 panthera-ht 分支："
  print_info "  git clone -b panthera-ht git@github.com:fiveages-sim/open-deploy-ws.git ht-deploy-ws"
  exit 1
fi

print_info "工作空间目录: ${REPO_DIR}"
echo ""

c="$(menu_select "请选择操作" 1 \
  "初始化工作空间（逐模块 source/deb）" \
  "切换安装方式（逐模块 source/deb/keep）" \
  "仅安装/更新核心 deb" \
  "卸载核心 deb" \
  "仅拉取子模块" \
  "仅运行 rosdep")"

case "${c}" in
  2) flow_switch ;;
  3) flow_deb_only ;;
  4) flow_deb_uninstall ;;
  5) flow_submodule_only ;;
  6) flow_rosdep ;;
  *) flow_init ;;
esac

print_info ""
print_info "=========================================="
print_info "完成！"
print_info "=========================================="
print_info ""
print_info "后续步骤："
print_info "  1. source /opt/ros/${ROS_DISTRO}/setup.bash"
print_info "  2. ./quick_start.sh → 编译仿真/真机所需包"
if [ "$USE_DEB_ARMS" -eq 1 ] && [ "${ARMS_VARIANT}" = "full" ]; then
  print_info "     （arms=deb(full) 时 ht_ros2_control 已在 arms-full 中，通常只需编译 panthera_ht_description）"
else
  print_info "     （arms=源码 或 arms=deb(standard) 时 ht_ros2_control 由源码编译）"
fi
print_info ""
print_info "如需在源码与 deb 间切换，重新运行 ./init_repo.sh 并选择选项 2"
