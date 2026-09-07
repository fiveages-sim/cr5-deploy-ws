#!/bin/bash

# ARX Lift2S / ACone ROS2 部署工作空间初始化脚本
# 功能：逐模块 source/deb 初始化子模块；arms 默认 full（含 arx_ros2_control）
# arms=deb(full) 时跳过 src/arx-ros2-control；--init-release 仅 init 描述子模块
# 子模块更新策略（对齐 fa_w2）：检出 .gitmodules 分支（或保留当前分支）并 merge 到 tip

set -u

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$SCRIPT_DIR"
MODE_STATE_FILE="$REPO_DIR/.core_module_mode"

# 嵌套子模块：仅更新白名单到各自分支 tip（跳过 hardwares/*，真机 HI 用顶层 arx-ros2-control）
declare -A NESTED_SUBMODULES=(
  ["src/arms_ros2_control"]="libraries/lina_planning libraries/ocs2_humanoid controller/ocs2_wbc_controller"
)

QS_CONFIG="${REPO_DIR}/config/quick_start.conf"
if [[ -f "${QS_CONFIG}" ]]; then
  # shellcheck source=/dev/null
  source "${QS_CONFIG}"
fi
if [[ -z "${RELEASE_SUBMODULE_PATHS+set}" ]] || [[ ${#RELEASE_SUBMODULE_PATHS[@]} -eq 0 ]]; then
  echo "[ERROR] config/quick_start.conf 未定义 RELEASE_SUBMODULE_PATHS" >&2
  exit 1
fi

UPDATE_MODE=false
SKIP_MENU=false
INIT_SUBSET=""
FLOW="init"
# true：有 named branch 时保留并更新该分支；false：强制切到 .gitmodules 默认分支
KEEP_BRANCH=true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --init|-i)
      UPDATE_MODE=false
      SKIP_MENU=true
      shift
      ;;
    --init-release)
      UPDATE_MODE=false
      SKIP_MENU=true
      INIT_SUBSET="release"
      shift
      ;;
    --update-release)
      UPDATE_MODE=true
      SKIP_MENU=true
      INIT_SUBSET="release"
      shift
      ;;
    --default-branch)
      KEEP_BRANCH=false
      shift
      ;;
    --keep-branch)
      KEEP_BRANCH=true
      shift
      ;;
    --rosdep)
      SKIP_MENU=true
      FLOW="rosdep"
      shift
      ;;
    --help|-h)
      echo "用法: $0 [--init-release] [--update-release] [--default-branch|--keep-branch] [--rosdep]"
      echo "  无参数：交互菜单（source/deb、deb 通道、arms 变体）"
      echo "  --init-release    仅初始化发布所需子模块（robot-descriptions-arx）"
      echo "  --update-release  仅更新发布所需子模块到分支 tip"
      echo "  --keep-branch     保留子模块当前分支并更新到 tip（默认）"
      echo "  --default-branch  强制切换到 .gitmodules 配置的默认分支再更新"
      exit 0
      ;;
    *)
      print_error "未知参数: $1"
      exit 1
      ;;
  esac
done

is_submodule_placeholder() {
  local submodule_path="$1"
  [ -d "${REPO_DIR}/${submodule_path}" ] || return 1
  [ -e "${REPO_DIR}/${submodule_path}/.git" ] && return 1
  return 0
}

clear_submodule_worktree() {
  local submodule_path="$1"
  git -C "${REPO_DIR}" submodule deinit -f -- "${submodule_path}" 2>/dev/null || true
  rm -rf "${REPO_DIR}/.git/modules/${submodule_path}"
  rm -rf "${REPO_DIR}/${submodule_path}"
}

run_init_release_subset() {
  local path
  print_info "仅初始化发布所需子模块（RELEASE_SUBMODULE_PATHS）..."
  cd "${REPO_DIR}" || exit 1
  git submodule sync
  for path in "${RELEASE_SUBMODULE_PATHS[@]}"; do
    if is_submodule_placeholder "${path}"; then
      print_info "  清理占位目录: ${path}"
      clear_submodule_worktree "${path}"
    fi
    git submodule update --init -- "${path}" || exit 1
    if [[ "${UPDATE_MODE}" == "true" ]]; then
      update_submodule_to_latest "${path}" || true
    fi
  done
  print_info "发布子模块初始化完成"
  git submodule status "${RELEASE_SUBMODULE_PATHS[@]}"
}

check_arx_hi_external() {
  local pkg="${REPO_DIR}/src/arx-ros2-control"
  local sdk="${pkg}/external/arx5-sdk"
  local lift="${pkg}/external/arx_lift_src"
  local arch="x86_64"
  case "$(uname -m)" in
    aarch64|arm64|armv7l|armv6l) arch="aarch64" ;;
  esac
  print_info "检查 arx-ros2-control 真机依赖..."
  if [ ! -d "$pkg" ]; then
    print_error "未找到 ${pkg}（arms=standard/source 时须 init 该子模块）"
    return 1
  fi
  if [ ! -f "${sdk}/include/app/joint_controller.h" ]; then
    print_warn "  缺少 Stanford SDK：${sdk}/include/app/joint_controller.h"
  fi
  if [ ! -f "${sdk}/lib/${arch}/libhardware.so" ] || [ ! -f "${sdk}/lib/${arch}/libsolver.so" ]; then
    print_warn "  缺少 arx5-sdk/lib/${arch}/libhardware.so 或 libsolver.so"
  fi
  if [ ! -f "${lift}/lib/${arch}/libarx_lift_src.so" ]; then
    print_warn "  缺少 ${lift}/lib/${arch}/libarx_lift_src.so（Lift2S 升降真机需要）"
  fi
}

MODE_STATE_FILE="$REPO_DIR/.core_module_mode"

# 模块安装方式：1=deb，0=source
USE_DEB_OCS2=1
USE_DEB_ARMS=1
USE_DEB_COMMON=1
# deb 发布通道：latest | pre-release | conf（按 deb_versions.conf 固定 tag）
DEB_CHANNEL="conf"
# arms deb 变体：full（ros-jazzy-arms-ros2-control-full，含 arx_ros2_control）| standard（ros-jazzy-arms-ros2-control）
ARMS_VARIANT="full"
FLOW="init" # init | deb_only | deb_uninstall | switch | rosdep

run_rosdep_install() {
 print_info "运行 rosdep install（仅源码子模块路径）..."
 if ! command -v rosdep >/dev/null 2>&1; then
 print_error "未找到 rosdep，请先安装 ROS 环境后重试"
 print_info " sudo apt install python3-rosdep"
 print_info " sudo rosdep init && rosdep update"
 return 1
 fi
 local rosdep_paths=()
 while IFS= read -r submodule_path; do
 should_skip_top_submodule "$submodule_path" && continue
 rosdep_paths+=("$submodule_path")
 done < <(git config --file .gitmodules --get-regexp path | awk '{print $2}')
 if [ ${#rosdep_paths[@]} -eq 0 ]; then
 print_info "无源码路径需要 rosdep"
 return 0
 fi
 rosdep install --from-paths "${rosdep_paths[@]}" --ignore-src -r -y \
 || print_warn "rosdep 安装部分依赖失败，可稍后重试或检查 package.xml"
}

run_install_core_debs() {
 local only_list="${1:-}"
 print_info "安装核心 deb 包（顺序: ocs2 → common → arms_ros2_control(-full)，通道: ${DEB_CHANNEL}）..."
 local install_script="$REPO_DIR/scripts/install_core_debs.sh"
 if [ ! -x "$install_script" ]; then
 chmod +x "$install_script" 2>/dev/null || true
 fi
 if [ ! -f "$install_script" ]; then
 print_error "未找到安装脚本: $install_script"
 return 1
 fi
 local -a args=()
 case "$DEB_CHANNEL" in
 latest|pre-release) args+=(--channel "$DEB_CHANNEL") ;;
 esac
 # 安装列表包含 arms 时传递变体（空列表 = 全部安装，含 arms）
 if [ -z "$only_list" ] || [[ ",$only_list," == *,arms,* ]]; then
 args+=(--arms-variant "$ARMS_VARIANT")
 fi
 if [ -n "$only_list" ]; then
 args+=(--only "$only_list")
 fi
 bash "$install_script" "${args[@]}"
}

run_uninstall_core_debs() {
 local only_list="${1:-}"
 print_info "卸载核心 deb 包（顺序: arms_ros2_control-full → common → ocs2）..."
 local uninstall_script="$REPO_DIR/scripts/uninstall_core_debs.sh"
 if [ ! -x "$uninstall_script" ]; then
 chmod +x "$uninstall_script" 2>/dev/null || true
 fi
 if [ ! -f "$uninstall_script" ]; then
 print_error "未找到卸载脚本: $uninstall_script"
 return 1
 fi
 if [ -n "$only_list" ]; then
 bash "$uninstall_script" --only "$only_list"
 else
 bash "$uninstall_script"
 fi
}

is_pkg_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

save_module_mode_state() {
  cat > "$MODE_STATE_FILE" <<EOF
USE_DEB_OCS2=${USE_DEB_OCS2}
USE_DEB_ARMS=${USE_DEB_ARMS}
USE_DEB_COMMON=${USE_DEB_COMMON}
DEB_CHANNEL=${DEB_CHANNEL}
ARMS_VARIANT=${ARMS_VARIANT}
EOF
}

load_state_file() {
  if [ ! -f "$MODE_STATE_FILE" ]; then
    return 0
  fi
  # shellcheck source=/dev/null
  source "$MODE_STATE_FILE"
}

prompt_deb_channel() {
  local choice
  echo ""
  echo "选择 deb 发布通道（回车=当前 ${DEB_CHANNEL}）："
  echo "  1) conf         — deb_versions.conf 固定 tag（发布打包默认）"
  echo "  2) latest       — GitHub Latest 正式版"
  echo "  3) pre-release  — 滚动 pre-release"
  read -rp "请输入 [1/2/3]: " choice
  case "$choice" in
    1) DEB_CHANNEL="conf" ;;
    2) DEB_CHANNEL="latest" ;;
    3) DEB_CHANNEL="pre-release" ;;
    "") DEB_CHANNEL="${DEB_CHANNEL:-conf}" ;;
    *) print_warn "无效选项，保持 ${DEB_CHANNEL}" ;;
  esac
}

prompt_arms_variant() {
  local choice
  echo ""
  echo "选择 arms deb 变体（回车=当前 ${ARMS_VARIANT}）："
  echo "  1) full     — ros-jazzy-arms-ros2-control-full（含 arx_ros2_control，推荐）"
  echo "  2) standard — ros-jazzy-arms-ros2-control（须源码 init arx-ros2-control）"
  read -rp "请输入 [1/2]: " choice
  case "$choice" in
    2) ARMS_VARIANT="standard" ;;
    1|"") ARMS_VARIANT="${ARMS_VARIANT:-full}" ;;
    *) print_warn "无效选项，保持 ${ARMS_VARIANT}" ;;
  esac
}

path_has_submodule_content() {
 local path="$1"
 [ -d "$REPO_DIR/$path" ] || return 1
 [ -e "$REPO_DIR/$path/.git" ] && return 0
 [ -n "$(ls -A "$REPO_DIR/$path" 2>/dev/null)" ]
}

path_is_git_checkout() {
 local path="$1"
 [ -e "$REPO_DIR/$path/.git" ] || return 1
 (cd "$REPO_DIR/$path" && git rev-parse --git-dir >/dev/null 2>&1)
}

detect_module_state() {
 local path="$1"
 local deb_pkg="$2"
 local has_deb=0 has_src=0
 is_pkg_installed "$deb_pkg" && has_deb=1
 # arms-full 与旧标准包都算 arms deb
 if [ "$deb_pkg" = "ros-jazzy-arms-ros2-control-full" ]; then
 is_pkg_installed "ros-jazzy-arms-ros2-control" && has_deb=1
 fi
 path_is_git_checkout "$path" && has_src=1
 if [ "$has_deb" -eq 1 ] && [ "$has_src" -eq 1 ]; then
 echo "mixed"
 elif [ "$has_deb" -eq 1 ]; then
 echo "deb"
 elif [ "$has_src" -eq 1 ]; then
 echo "source"
 else
 echo "none"
 fi
}

module_short_to_path() {
 case "$1" in
 ocs2) echo "src/ocs2_ros2" ;;
 arms) echo "src/arms_ros2_control" ;;
 common) echo "src/robot-descriptions-common" ;;
 *) return 1 ;;
 esac
}

module_short_to_deb() {
 case "$1" in
 ocs2) echo "ros-jazzy-ocs2" ;;
 arms)
 if [ "$ARMS_VARIANT" = "standard" ]; then
 echo "ros-jazzy-arms-ros2-control"
 else
 echo "ros-jazzy-arms-ros2-control-full"
 fi
 ;;
 common) echo "ros-jazzy-robot-descriptions-common" ;;
 *) return 1 ;;
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

selected_deb_only_list() {
 local parts=()
 [ "$USE_DEB_OCS2" -eq 1 ] && parts+=("ocs2")
 [ "$USE_DEB_COMMON" -eq 1 ] && parts+=("common")
 [ "$USE_DEB_ARMS" -eq 1 ] && parts+=("arms")
 if [ ${#parts[@]} -eq 0 ]; then
 echo ""
 else
 local IFS=,
 echo "${parts[*]}"
 fi
}

prompt_sd() {
 local prompt="$1"
 local default="$2"
 local __result_var="$3"
 local choice default_hint
 if [ "$default" = "d" ]; then
 default_hint="D/s"
 else
 default_hint="d/S"
 fi
 read -rp "$prompt [$default_hint]: " choice
 choice="${choice:-$default}"
 case "$choice" in
 d|D|deb|DEB) printf -v "$__result_var" '%s' "1" ;;
 s|S|source|SOURCE) printf -v "$__result_var" '%s' "0" ;;
 *)
 if [ "$default" = "d" ]; then
 printf -v "$__result_var" '%s' "1"
 else
 printf -v "$__result_var" '%s' "0"
 fi
 ;;
 esac
}

mode_label() {
 if [ "$1" -eq 1 ]; then
 echo "deb"
 else
 echo "source"
 fi
}

should_skip_top_submodule() {
 local path="$1"
 case "$path" in
 src/ocs2_ros2) [ "$USE_DEB_OCS2" -eq 1 ] && return 0 ;;
 src/arms_ros2_control) [ "$USE_DEB_ARMS" -eq 1 ] && return 0 ;;
 src/robot-descriptions-common) [ "$USE_DEB_COMMON" -eq 1 ] && return 0 ;;
 # arms=deb 且变体=full 时 arms-full 已含 arx_ros2_control，不再拉源码
 src/arx-ros2-control) [ "$USE_DEB_ARMS" -eq 1 ] && [ "$ARMS_VARIANT" = "full" ] && return 0 ;;
 esac
 return 1
}

remove_submodule_path() {
 local path="$1"
 print_info "清理源码目录: $path"
 # 复制的工作区可能缺少完整 submodule 注册信息，deinit 失败时直接删目录即可
 git -C "$REPO_DIR" submodule deinit -f -- "$path" >/dev/null 2>&1 || true
 rm -rf "$REPO_DIR/$path"
 rm -rf "$REPO_DIR/.git/modules/$path" 2>/dev/null || true
 print_info "✓ 已清理 $path"
}

cleanup_deb_module_sources() {
 local paths_to_clean=() p clean_choice

 [ "$USE_DEB_OCS2" -eq 1 ] && path_has_submodule_content "src/ocs2_ros2" && \
 paths_to_clean+=("src/ocs2_ros2")
 [ "$USE_DEB_ARMS" -eq 1 ] && path_has_submodule_content "src/arms_ros2_control" && \
 paths_to_clean+=("src/arms_ros2_control")
 [ "$USE_DEB_COMMON" -eq 1 ] && path_has_submodule_content "src/robot-descriptions-common" && \
 paths_to_clean+=("src/robot-descriptions-common")
 # arms=deb 且变体=full 时 arms-full 已含 arx_ros2_control，自动清理其源码
 if [ "$USE_DEB_ARMS" -eq 1 ] && [ "$ARMS_VARIANT" = "full" ]; then
 path_has_submodule_content "src/arx-ros2-control" && \
 paths_to_clean+=("src/arx-ros2-control")
 fi

 if [ ${#paths_to_clean[@]} -eq 0 ]; then
 return 0
 fi

 print_warn "以下目录将改由 deb 提供，检测到已有源码内容："
 for p in "${paths_to_clean[@]}"; do
 print_warn " - $p"
 if [ "$p" = "src/arx-ros2-control" ]; then
 print_warn " （arms-full deb 已包含 arx_ros2_control）"
 fi
 done
 read -rp "是否清理上述目录？[Y/n]: " clean_choice
 case "$clean_choice" in
 n|N|no|NO)
 print_warn "已跳过清理（可能仍占用磁盘，且可能与 deb 冲突）"
 return 0
 ;;
 esac

 for p in "${paths_to_clean[@]}"; do
 remove_submodule_path "$p"
 done
 echo ""
}

# 是否应跳过嵌套子模块（arms=源码 时不初始化 hardwares/*，硬件驱动由 deb/单独提供）
should_skip_nested_submodule() {
 local parent="$1" nested="$2"
 if [ "$parent" = "src/arms_ros2_control" ] && [[ "$nested" == hardwares/* ]]; then
 return 0
 fi
 return 1
}

# 将指定子模块更新到目标分支 tip（对齐 fa_w2）。
# 安全约束：
# 1) KEEP_BRANCH=true 且已在 named branch → 保留该分支；否则（含 detached）切到 default_branch；
# 2) 本地修改先 stash，更新后 stash pop；绝不 reset --hard；
# 3) 若工作区仅有「已跟踪文件缺失」，先 restore，避免 stash pop 再删空；
# 4) 顶层保护：若进入的 git 仓库就是工作空间自身，直接跳过。
# $1 = 子模块路径（相对 REPO_DIR）
# $2 = 默认分支（来自 .gitmodules；可空，此时读顶层 .gitmodules 或回退 main）
update_submodule_to_latest() {
 local path="$1"
 local default_branch="${2:-}"
 local stashed=0
 local rc=0

 if [ ! -d "$path" ]; then
 print_warn "子模块路径不存在: $path"
 return 0
 fi

 if [ -z "$default_branch" ]; then
 default_branch="$(git -C "$REPO_DIR" config --file "$REPO_DIR/.gitmodules" --get "submodule.$path.branch" 2>/dev/null || echo main)"
 fi

 print_info "处理子模块: $path"
 (
 cd "$REPO_DIR/$path" || { print_warn "无法进入目录 $path，跳过"; exit 0; }

 if ! git rev-parse --git-dir > /dev/null 2>&1; then
 print_warn "子模块 $path 不是有效的 git 仓库，跳过"
 exit 0
 fi

 # 确保在子模块自身仓库根（占位目录会冒泡到父仓）
 abs_path="$(pwd)"
 abs_toplevel="$(git rev-parse --show-toplevel 2>/dev/null || true)"
 if [ -z "$abs_toplevel" ] || [ "$abs_path" != "$abs_toplevel" ]; then
 print_warn "子模块 $path 未正确检出，跳过"
 exit 0
 fi

 if [ "$(git rev-parse --show-toplevel 2>/dev/null || true)" = "$REPO_DIR" ]; then
 print_warn "跳过 $path：它指向顶层工作空间仓库（避免修改工作空间分支）"
 exit 0
 fi

 # 半残工作区：仅有删除、无内容改动 → 先恢复检出
 if ! git diff-index --quiet HEAD -- 2>/dev/null; then
 n_deleted="$(git ls-files -d 2>/dev/null | wc -l)"
 n_dirty="$(git diff --name-only --diff-filter=ACMRTUXB HEAD -- 2>/dev/null | wc -l)"
 if [ "$n_deleted" -gt 0 ] && [ "$n_dirty" -eq 0 ]; then
 print_warn " 检测到工作区文件缺失（${n_deleted} 个），先恢复检出..."
 git restore . 2>/dev/null || git checkout -- . 2>/dev/null \
 || { print_warn " 恢复检出失败，跳过更新"; exit 0; }
 fi
 fi

 if ! git diff-index --quiet HEAD -- 2>/dev/null; then
 print_warn " 检测到本地修改，先暂存（更新后恢复）..."
 if ! git stash push -m "Auto-stash before submodule update" 2>/dev/null; then
 print_warn " 暂存失败，跳过更新以保留本地修改"
 exit 0
 fi
 stashed=1
 fi

 print_info " 获取远程更新..."
 git fetch origin 2>/dev/null || print_warn " 获取远程更新失败，继续..."

 current_branch="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")"
 branch_name="$default_branch"
 if [ "$KEEP_BRANCH" = true ] && [ "$current_branch" != "HEAD" ] && [ -n "$current_branch" ]; then
 branch_name="$current_branch"
 if [ "$current_branch" != "$default_branch" ]; then
 print_info " 保留当前分支: $branch_name（默认: $default_branch）"
 fi
 elif [ "$current_branch" = "HEAD" ]; then
 print_info " 当前 detached HEAD，切换到默认分支: $branch_name"
 else
 print_info " 目标分支: $branch_name"
 fi

 if ! git rev-parse --verify --quiet "refs/remotes/origin/$branch_name" >/dev/null 2>&1; then
 # 再试 ls-remote（本地可能尚无 remote-tracking ref）
 if ! git ls-remote --exit-code --heads origin "$branch_name" >/dev/null 2>&1; then
 print_warn " 远端无分支 origin/$branch_name，跳过 $path"
 [ "$stashed" -eq 1 ] && git stash pop >/dev/null 2>&1 || true
 exit 1
 fi
 git fetch origin "$branch_name" 2>/dev/null || true
 fi

 current_branch="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")"
 if [ "$current_branch" != "$branch_name" ]; then
 print_info " 从 $current_branch 切换到 $branch_name..."
 if git show-ref --verify --quiet "refs/heads/$branch_name"; then
 git checkout "$branch_name" 2>/dev/null || git checkout -f "$branch_name" \
 || { print_error " 无法切换到 $branch_name"; exit 1; }
 else
 git checkout -b "$branch_name" "origin/$branch_name" 2>/dev/null \
 || git checkout "$branch_name" 2>/dev/null \
 || { print_error " 无法创建/切换到 $branch_name"; exit 1; }
 fi
 final_branch="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")"
 if [ "$final_branch" != "$branch_name" ]; then
 print_error " 切换失败：仍在 $final_branch，目标 $branch_name"
 exit 1
 fi
 else
 print_info " 已在 $branch_name 分支"
 fi

 git fetch origin "$branch_name" 2>/dev/null || true
 local_commit="$(git rev-parse HEAD 2>/dev/null)"
 remote_commit="$(git rev-parse "origin/$branch_name" 2>/dev/null || true)"
 if [ -z "$remote_commit" ]; then
 print_warn " 无法解析 origin/$branch_name，跳过合并"
 elif [ "$local_commit" = "$remote_commit" ]; then
 print_info " 已是最新提交"
 else
 print_info " 合并 origin/$branch_name 到 tip..."
 if git merge "origin/$branch_name" --no-edit; then
 print_info " ✓ 已更新到最新提交"
 else
 conflicted="$(git ls-files -u 2>/dev/null | awk '{print $4}' | sort -u)"
 if [ -n "$conflicted" ]; then
 print_error " 合并冲突，请手动解决: $path"
 echo "$conflicted" | while read -r f; do print_warn "   - $f"; done
 [ "$stashed" -eq 1 ] && print_warn " 本地修改仍在 stash 中"
 exit 1
 fi
 print_warn " 合并失败且未检测到冲突，继续"
 fi
 fi

 if [ "$stashed" -eq 1 ]; then
 print_info " 恢复本地暂存的修改..."
 git stash pop 2>/dev/null \
 || print_warn " 暂存恢复失败（可手动: git -C \"$path\" stash pop）"
 fi
 exit 0
 )
 rc=$?
 if [ "$rc" -eq 0 ]; then
 print_info "✓ $path 已更新到分支 tip"
 else
 print_warn "✗ $path 更新未完全成功（rc=$rc）"
 fi
 return "$rc"
}

# 初始化/更新顶层子模块，并按白名单处理嵌套子模块到分支 tip
# $1 = 父路径（顶层为 ""）；$2 = 子路径（相对父路径）
init_submodule_recursive() {
 local parent="$1" nested="$2"
 local path default_branch nested_config nested_path nested_branch

 if [ -n "$parent" ]; then
 path="$parent/$nested"
 else
 path="$nested"
 fi

 if [ -n "$parent" ] && [ -f "$parent/.gitmodules" ]; then
 default_branch="$(git -C "$parent" config --file .gitmodules --get "submodule.$nested.branch" 2>/dev/null || echo main)"
 else
 default_branch="$(git config --file .gitmodules --get "submodule.$path.branch" 2>/dev/null || echo main)"
 fi

 update_submodule_to_latest "$path" "$default_branch" || true

 nested_config="${NESTED_SUBMODULES[$path]:-}"
 if [ -z "$nested_config" ]; then
 return 0
 fi
 if [ ! -e "$path/.git" ] || [ ! -f "$path/.gitmodules" ]; then
 print_warn "跳过嵌套更新：缺少 $path/.git 或 .gitmodules"
 return 0
 fi

 print_info "更新 $path 的嵌套子模块（白名单）..."
 if [ "$nested_config" = "*" ]; then
 while IFS= read -r nested_path; do
 [ -z "$nested_path" ] && continue
 if should_skip_nested_submodule "$path" "$nested_path"; then
 print_info " 跳过嵌套子模块（arms 源码模式）: $path/$nested_path"
 continue
 fi
 if ! git -C "$path" submodule update --init -- "$nested_path"; then
 print_warn "初始化嵌套子模块失败: $path/$nested_path"
 continue
 fi
 nested_branch="$(git -C "$path" config --file .gitmodules --get "submodule.$nested_path.branch" 2>/dev/null || echo main)"
 update_submodule_to_latest "$path/$nested_path" "$nested_branch" || true
 done < <(git -C "$path" config --file .gitmodules --get-regexp path 2>/dev/null | awk '{print $2}')
 return 0
 fi

 # shellcheck disable=SC2206
 local nested_paths=($nested_config)
 for nested_path in "${nested_paths[@]}"; do
 if should_skip_nested_submodule "$path" "$nested_path"; then
 print_info " 跳过嵌套子模块（arms 源码模式）: $path/$nested_path"
 continue
 fi
 if [ "$(git -C "$path" config --file .gitmodules --get "submodule.$nested_path.path" 2>/dev/null)" != "$nested_path" ]; then
 print_warn " 嵌套子模块未在 .gitmodules 中配置: $nested_path"
 continue
 fi
 print_info " 处理嵌套子模块: $path/$nested_path"
 if [ ! -e "$path/$nested_path/.git" ]; then
 if ! git -C "$path" submodule update --init -- "$nested_path"; then
 print_warn " 初始化失败: $path/$nested_path"
 continue
 fi
 fi
 if [ ! -e "$path/$nested_path/.git" ]; then
 print_warn " 嵌套路径仍未检出: $path/$nested_path"
 continue
 fi
 nested_branch="$(git -C "$path" config --file .gitmodules --get "submodule.$nested_path.branch" 2>/dev/null || echo main)"
 update_submodule_to_latest "$path/$nested_path" "$nested_branch" || true
 done
}

switch_uninstall_debs_for_source_targets() {
 local to_uninstall=()
 local m pkg
 for m in ocs2 arms common; do
 if [ "$(get_use_deb_for_module "$m")" -eq 0 ]; then
 pkg="$(module_short_to_deb "$m")"
 if is_pkg_installed "$pkg"; then
 to_uninstall+=("$m")
 elif [ "$m" = "arms" ] && is_pkg_installed "ros-jazzy-arms-ros2-control"; then
 # 兼容旧标准版 arms deb
 to_uninstall+=("$m")
 fi
 fi
 done
 if [ ${#to_uninstall[@]} -eq 0 ]; then
 return 0
 fi
 local IFS=,
 local list="${to_uninstall[*]}"
 print_info "以下模块改为源码，将先卸载对应 deb: $list"
 run_uninstall_core_debs "$list" || print_warn "部分 deb 卸载失败，继续尝试源码初始化..."
}

if [[ "${INIT_SUBSET:-}" == "release" ]]; then
  run_init_release_subset
  exit 0
fi

if [[ "${SKIP_MENU:-}" == "true" && "${FLOW:-}" == "rosdep" ]]; then
  run_rosdep_install || exit 1
  exit 0
fi

print_info "工作空间目录: $REPO_DIR"
cd "$REPO_DIR"

if [ ! -d ".git" ]; then
 print_error "当前目录不是 git 仓库！"
 print_info "请先克隆 arx-lift2s 分支："
 print_info " git clone -b arx-lift2s git@github.com:fiveages-sim/open-deploy-ws.git lift2s-ws"
 exit 1
fi

if [[ "${SKIP_MENU:-}" == "true" ]]; then
  FLOW="init"
  load_state_file
  USE_DEB_OCS2="${USE_DEB_OCS2:-1}"
  USE_DEB_ARMS="${USE_DEB_ARMS:-1}"
  USE_DEB_COMMON="${USE_DEB_COMMON:-1}"
  DEB_CHANNEL="${DEB_CHANNEL:-conf}"
  ARMS_VARIANT="${ARMS_VARIANT:-full}"
else
echo ""
echo "请选择操作："

echo ""
echo " 1) 初始化工作空间（逐模块 source/deb）"
echo " 默认推荐: ocs2=deb，arms=deb(full，含 arx_ros2_control)，common=deb"
echo " arms=deb 时可选择 full / standard 变体；ARX 描述包仍源码"
echo " deb 可选手动选择通道: latest / pre-release / conf"
echo " 2) 切换模块安装方式（源码 ↔ deb）"
echo " 3) 仅安装/更新核心 deb 包（跳过 Git 子模块拉取）"
echo " 4) 卸载核心 deb 包"
echo " 5) 仅运行 rosdep 安装依赖"
echo ""
read -rp "请输入选项 [1/2/3/4/5]（默认: 1）: " flow_choice
case "$flow_choice" in
 2) FLOW="switch" ;;
 3) FLOW="deb_only" ;;
 4) FLOW="deb_uninstall" ;;
 5) FLOW="rosdep" ;;
 *) FLOW="init" ;;
esac
fi

if [ "$FLOW" = "rosdep" ]; then
 print_info "模式: 仅运行 rosdep"
 echo ""
 run_rosdep_install || exit 1
 exit 0
fi

if [ "$FLOW" = "deb_only" ]; then
 echo ""
 echo "选择要安装/更新的包（逗号分隔短名，回车=全部）："
 echo " ocs2, common, arms（arms 变体随后选择）"
 read -rp "包列表: " only_choice
 only_choice="$(echo "$only_choice" | tr -d '[:space:]')"
 load_state_file
 prompt_deb_channel
 # 安装列表包含 arms 时选择变体
 if [ -z "$only_choice" ] || [[ ",$only_choice," == *,arms,* ]]; then
 prompt_arms_variant
 fi
 print_info "模式: 仅安装核心 deb 包（不拉取 Git 子模块）"
 echo ""
 run_install_core_debs "$only_choice" || exit 1
 save_module_mode_state
 print_info ""
 print_info "完成后请执行: source /opt/ros/jazzy/setup.bash"
 exit 0
fi

if [ "$FLOW" = "deb_uninstall" ]; then
 echo ""
 print_info "检测当前已安装的核心 deb 包："
 _detected=()
 if is_pkg_installed "ros-jazzy-ocs2"; then
 print_info " - ros-jazzy-ocs2 (ocs2)"
 _detected+=("ocs2")
 fi
 if is_pkg_installed "ros-jazzy-robot-descriptions-common"; then
 print_info " - ros-jazzy-robot-descriptions-common (common)"
 _detected+=("common")
 fi
 if is_pkg_installed "ros-jazzy-arms-ros2-control-full"; then
 print_info " - ros-jazzy-arms-ros2-control-full (arms, full)"
 _detected+=("arms")
 elif is_pkg_installed "ros-jazzy-arms-ros2-control"; then
 print_info " - ros-jazzy-arms-ros2-control (arms, standard)"
 _detected+=("arms")
 fi
 if [ ${#_detected[@]} -eq 0 ]; then
 print_info " 未检测到已安装的核心 deb 包"
 fi
 echo ""
 echo "选择要卸载的包（逗号分隔短名，回车=全部）："
 echo " ocs2, common, arms"
 read -rp "包列表: " only_choice
 only_choice="$(echo "$only_choice" | tr -d '[:space:]')"
 print_info "模式: 卸载核心 deb 包"
 echo ""
 run_uninstall_core_debs "$only_choice" || exit 1
 exit 0
fi

if [ "$FLOW" = "init" ]; then
 load_state_file
 print_info "当前默认: ocs2=$(mode_label "$USE_DEB_OCS2"), arms=$(mode_label "$USE_DEB_ARMS"), common=$(mode_label "$USE_DEB_COMMON"), arms_variant=${ARMS_VARIANT}, channel=${DEB_CHANNEL}"

 echo ""
 echo "核心模块安装方式（d=deb, s=source，回车用括号内默认）："
 echo " robot-descriptions-arx 始终源码；arms=deb 时 arx-ros2-control 视变体而定"
 prompt_sd " ocs2_ros2 默认 $(mode_label "$USE_DEB_OCS2")" \
 "$( [ "$USE_DEB_OCS2" -eq 1 ] && echo d || echo s )" USE_DEB_OCS2
 prompt_sd " arms_ros2_control 默认 $(mode_label "$USE_DEB_ARMS")" \
 "$( [ "$USE_DEB_ARMS" -eq 1 ] && echo d || echo s )" USE_DEB_ARMS
 prompt_sd " robot-descriptions-common 默认 $(mode_label "$USE_DEB_COMMON")" \
 "$( [ "$USE_DEB_COMMON" -eq 1 ] && echo d || echo s )" USE_DEB_COMMON
fi

if [ "$FLOW" = "switch" ]; then
 load_state_file
 echo ""
 echo "当前模块状态："
 for m in ocs2 arms common; do
 p="$(module_short_to_path "$m")"
 pkg="$(module_short_to_deb "$m")"
 st="$(detect_module_state "$p" "$pkg")"
 print_info " $m ($p): $st"
 done
 echo ""
 echo "为每个模块选择目标（s=source, d=deb, k=保持），回车=保持："
 for m in ocs2 arms common; do
 p="$(module_short_to_path "$m")"
 pkg="$(module_short_to_deb "$m")"
 st="$(detect_module_state "$p" "$pkg")"
 read -rp " $m 当前=$st [s/d/K]: " tgt
 tgt="${tgt:-k}"
 case "$tgt" in
 s|S|source) set_use_deb_for_module "$m" 0 ;;
 d|D|deb) set_use_deb_for_module "$m" 1 ;;
 *)
 case "$st" in
 deb) set_use_deb_for_module "$m" 1 ;;
 *) set_use_deb_for_module "$m" 0 ;;
 esac
 ;;
 esac
 done
fi

if [ "$USE_DEB_ARMS" -eq 1 ] && [ "$USE_DEB_OCS2" -eq 0 ]; then
 print_warn "arms 选择 deb 而 ocs2 选择 source：arms deb 通常依赖 ocs2 包，安装可能失败。"
fi
if [ "$USE_DEB_ARMS" -eq 1 ] && [ "$USE_DEB_COMMON" -eq 0 ]; then
 print_warn "arms 选择 deb 而 common 选择 source：arms deb 通常依赖 common 包，安装可能失败。"
fi

# 只要有模块选 deb，就询问发布通道
if [ "$USE_DEB_OCS2" -eq 1 ] || [ "$USE_DEB_ARMS" -eq 1 ] || [ "$USE_DEB_COMMON" -eq 1 ]; then
 prompt_deb_channel
fi

# arms=deb 时选择 deb 变体（full/standard）
if [ "$USE_DEB_ARMS" -eq 1 ]; then
 prompt_arms_variant
 if [ "$ARMS_VARIANT" = "full" ]; then
 print_info "arms=deb(full) → 安装 arms-full（含 arx_ros2_control），将跳过/清理 src/arx-ros2-control"
 else
 print_info "arms=deb(standard) → 安装标准 arms 包，将源码初始化 src/arx-ros2-control"
 fi
fi

print_info "模块方式: ocs2=$(mode_label "$USE_DEB_OCS2"), arms=$(mode_label "$USE_DEB_ARMS"), common=$(mode_label "$USE_DEB_COMMON"), arms_variant=${ARMS_VARIANT}, channel=${DEB_CHANNEL}"
echo ""

if [ "$FLOW" = "switch" ]; then
 switch_uninstall_debs_for_source_targets
fi

cleanup_deb_module_sources

print_info "开始初始化子模块..."

print_info "同步子模块配置..."
git submodule sync

print_info "初始化顶层子模块（跳过已选 deb 的核心仓库）..."
init_paths=()
while IFS= read -r submodule_path; do
 should_skip_top_submodule "$submodule_path" && continue
 init_paths+=("$submodule_path")
done < <(git config --file .gitmodules --get-regexp path | awk '{print $2}')
if [ ${#init_paths[@]} -gt 0 ]; then
 git submodule update --init "${init_paths[@]}"
else
 print_info "无需要初始化的顶层源码子模块"
fi

print_info "将源码子模块更新到分支 tip（KEEP_BRANCH=${KEEP_BRANCH}；含 arms 嵌套白名单）..."

for submodule_path in "${init_paths[@]}"; do
 init_submodule_recursive "" "$submodule_path"
done

print_info ""
print_info "=========================================="
print_info "子模块初始化完成！"
print_info "=========================================="
print_info ""
print_info "当前子模块状态："
git submodule status

print_info ""
if [ "$USE_DEB_ARMS" -eq 0 ] || [ "$ARMS_VARIANT" = "standard" ]; then
  check_arx_hi_external || print_warn "arx-ros2-control external 不完整，真机 HI 源码编译可能失败"
fi

print_info "安装 rosdep 依赖..."
run_rosdep_install

deb_only_list="$(selected_deb_only_list)"
if [ -n "$deb_only_list" ]; then
 print_info ""
 run_install_core_debs "$deb_only_list" || print_error "deb 安装失败，请检查 deb_versions.conf 或网络连接"
fi

save_module_mode_state

print_info ""
print_info "后续步骤："
print_info " 1. source /opt/ros/jazzy/setup.bash"
print_info " 2. ./quick_start.sh → 编译仿真/真机所需包"
if [ "$USE_DEB_ARMS" -eq 1 ] && [ "$ARMS_VARIANT" = "full" ]; then
 print_info " （arms=deb(full) 时 arx_ros2_control 已在 arms-full 中，通常只需编译 ARX 描述包）"
if [ -d "${REPO_DIR}/install/arx_ros2_control" ]; then
  print_warn "  检测到 install/arx_ros2_control，可能遮住 deb HI；请删除后重新 source"
fi
else
 print_info " （arms=源码 或 arms=deb(standard) 时 arx_ros2_control 由源码编译）"
fi
print_info ""
print_info "如需在源码与 deb 间切换，重新运行 ./init_repo.sh 并选择选项 2"
print_info "子模块默认保留当前分支更新；强制默认分支: ./init_repo.sh --default-branch"
if [ "$USE_DEB_ARMS" -eq 0 ] || [ "$ARMS_VARIANT" = "standard" ]; then
 print_info "如需单独更新 ARX 源码子模块，可再跑一次 init，或："
 print_info " git -C src/arx-ros2-control fetch && git -C src/arx-ros2-control merge --ff-only origin/main"
else
 print_info "如需更新 ARX 描述子模块，可再跑一次 init，或："
 print_info " git -C src/robot-descriptions-arx fetch && git -C src/robot-descriptions-arx merge --ff-only origin/main"
fi