#!/bin/bash

# HighTorque Panthera HT ROS2 部署工作空间初始化脚本
# 功能：逐模块 source/deb 初始化子模块，支持 ocs2 / arms / common 核心包 deb 安装
# 说明：arms 可选安装 ros-jazzy-arms-ros2-control / -full 两种变体；
#       arms=deb 且变体=full 时跳过并清理 src/ht-ros2-control 源码；
#       arms=源码 时不初始化 src/arms_ros2_control/hardwares/* 嵌套子模块
# 注意：子模块更新只做快进（不切换分支），本地修改会暂存并在更新后恢复，
#       绝不会修改当前工作空间自身所在的分支。

set -u

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$SCRIPT_DIR"
MODE_STATE_FILE="$REPO_DIR/.core_module_mode"

# 模块安装方式：1=deb，0=source
USE_DEB_OCS2=1
USE_DEB_ARMS=1
USE_DEB_COMMON=1
# deb 发布通道：latest | pre-release | conf（按 deb_versions.conf 固定 tag）
DEB_CHANNEL="latest"
# arms deb 变体：full（ros-jazzy-arms-ros2-control-full，含 ht_ros2_control）| standard（ros-jazzy-arms-ros2-control）
ARMS_VARIANT="full"
FLOW="init"  # init | deb_only | deb_uninstall | switch | rosdep

run_rosdep_install() {
    print_info "运行 rosdep install（仅源码子模块路径）..."
    if ! command -v rosdep >/dev/null 2>&1; then
        print_error "未找到 rosdep，请先安装 ROS 环境后重试"
        print_info "  sudo apt install python3-rosdep"
        print_info "  sudo rosdep init && rosdep update"
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

save_module_mode_state() {
    cat > "$MODE_STATE_FILE" <<EOF
# 上次 init_repo.sh 选择的模块安装方式（deb=1 / source=0）
USE_DEB_OCS2=$USE_DEB_OCS2
USE_DEB_ARMS=$USE_DEB_ARMS
USE_DEB_COMMON=$USE_DEB_COMMON
ARMS_VARIANT=$ARMS_VARIANT
DEB_CHANNEL=$DEB_CHANNEL
EOF
}

# 从状态文件加载上次选择（仅覆盖合法值，不存在的字段保持当前默认）
load_state_file() {
    local v
    [ -f "$MODE_STATE_FILE" ] || return 0
    v="$(grep -E '^USE_DEB_OCS2=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo 1)"
    [[ "$v" =~ ^[01]$ ]] && USE_DEB_OCS2="$v"
    v="$(grep -E '^USE_DEB_ARMS=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo 1)"
    [[ "$v" =~ ^[01]$ ]] && USE_DEB_ARMS="$v"
    v="$(grep -E '^USE_DEB_COMMON=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo 1)"
    [[ "$v" =~ ^[01]$ ]] && USE_DEB_COMMON="$v"
    v="$(grep -E '^ARMS_VARIANT=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || true)"
    case "$v" in full|standard) ARMS_VARIANT="$v" ;; esac
    v="$(grep -E '^DEB_CHANNEL=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || true)"
    case "$v" in latest|pre-release|conf) DEB_CHANNEL="$v" ;; esac
}

# 从 deb_versions.conf 读取 arms 对应的变体（full / standard）
arms_variant_from_conf() {
    local line
    line="$(grep -E '^(ros-jazzy-arms-ros2-control|ros-jazzy-arms-ros2-control-full)\|' "$REPO_DIR/deb_versions.conf" 2>/dev/null | head -n1 || true)"
    case "$line" in
        ros-jazzy-arms-ros2-control-full*) echo "full" ;;
        *) echo "standard" ;;
    esac
}

# 选择 arms deb 变体（full / standard）；conf 通道读取 deb_versions.conf 并允许切换（仅本次生效）
prompt_arms_variant() {
    local choice other conf_variant conf_pkg
    if [ "$DEB_CHANNEL" = "conf" ]; then
        conf_variant="$(arms_variant_from_conf)"
        if [ "$conf_variant" = "full" ]; then
            other="standard"
            conf_pkg="ros-jazzy-arms-ros2-control-full"
        else
            other="full"
            conf_pkg="ros-jazzy-arms-ros2-control"
        fi
        echo ""
        echo "conf 通道当前 arms 变体: ${conf_variant}（${conf_pkg}）"
        echo "  1) 确认安装 ${conf_variant}（默认）"
        echo "  2) 更换为 ${other}（仅本次运行生效，不改 deb_versions.conf）"
        read -rp "请选择 [1/2]（默认: 1）: " choice
        choice="${choice:-1}"
        case "$choice" in
            2) ARMS_VARIANT="$other" ;;
            *) ARMS_VARIANT="$conf_variant" ;;
        esac
    else
        echo ""
        echo "arms=deb，请选择安装的 arms deb 变体："
        echo "  1) ros-jazzy-arms-ros2-control-full  （含 ht_ros2_control，跳过 src/ht-ros2-control，默认）"
        echo "  2) ros-jazzy-arms-ros2-control       （标准包，需源码初始化 src/ht-ros2-control）"
        read -rp "请选择 [1/2]（默认: 1）: " choice
        choice="${choice:-1}"
        case "$choice" in
            2) ARMS_VARIANT="standard" ;;
            *) ARMS_VARIANT="full" ;;
        esac
    fi
    print_info "arms deb 变体: ${ARMS_VARIANT}（$(module_short_to_deb arms)）"
}

prompt_deb_channel() {
    local choice
    local default_num=1
    case "$DEB_CHANNEL" in
        pre-release) default_num=2 ;;
        conf) default_num=3 ;;
        *) DEB_CHANNEL="latest"; default_num=1 ;;
    esac
    echo ""
    echo "deb 发布通道（覆盖各仓库 tag）："
    echo "  1) latest      — GitHub Latest 稳定版（排除 pre-release）"
    echo "  2) pre-release — 各仓库 pre-release 浮动标签（更新更勤）"
    echo "  3) conf        — 使用 deb_versions.conf 中的固定版本"
    read -rp "请选择通道 [1/2/3]（默认: ${default_num}）: " choice
    choice="${choice:-$default_num}"
    case "$choice" in
        1|l|L|latest|stable) DEB_CHANNEL="latest" ;;
        2|p|P|pre|pre-release|prerelease) DEB_CHANNEL="pre-release" ;;
        3|c|C|conf|config) DEB_CHANNEL="conf" ;;
        *)
            print_warn "未知选项，保持: ${DEB_CHANNEL}"
            ;;
    esac
    print_info "deb 通道: ${DEB_CHANNEL}"
}

is_pkg_installed() {
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
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
        # arms=deb 且变体=full 时 arms-full 已含 ht_ros2_control，不再拉源码
        src/ht-ros2-control) [ "$USE_DEB_ARMS" -eq 1 ] && [ "$ARMS_VARIANT" = "full" ] && return 0 ;;
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
    # arms=deb 且变体=full 时 arms-full 已含 ht_ros2_control，自动清理其源码
    if [ "$USE_DEB_ARMS" -eq 1 ] && [ "$ARMS_VARIANT" = "full" ]; then
        path_has_submodule_content "src/ht-ros2-control" && \
            paths_to_clean+=("src/ht-ros2-control")
    fi

    if [ ${#paths_to_clean[@]} -eq 0 ]; then
        return 0
    fi

    print_warn "以下目录将改由 deb 提供，检测到已有源码内容："
    for p in "${paths_to_clean[@]}"; do
        print_warn "  - $p"
        if [ "$p" = "src/ht-ros2-control" ]; then
            print_warn "    （arms-full deb 已包含 ht_ros2_control）"
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

# 将指定子模块更新到当前分支的最新提交。
# 安全约束：
#   1) 绝不切换分支（不执行 git checkout），只对当前分支做 --ff-only 快进；
#   2) 本地修改先 git stash 暂存，更新成功后 stash pop 恢复，绝不 reset --hard；
#   3) 顶层保护：若进入的 git 仓库就是工作空间自身，直接跳过，避免改动工作空间分支。
# $1 = 子模块路径（相对 REPO_DIR）
update_submodule_to_latest() {
    local path="$1"
    local stashed=0

    if [ ! -d "$path" ]; then
        print_warn "子模块路径不存在: $path"
        return 0
    fi

    print_info "处理子模块: $path"
    # 子 shell 执行，避免 cd 泄漏影响后续路径
    (
        cd "$path" || { print_warn "无法进入目录 $path，跳过"; exit 0; }

        if ! git rev-parse --git-dir > /dev/null 2>&1; then
            print_warn "子模块 $path 不是有效的 git 仓库，跳过"
            exit 0
        fi

        # 顶层保护：绝不操作工作空间自身所在仓库
        if [ "$(git rev-parse --show-toplevel 2>/dev/null || true)" = "$REPO_DIR" ]; then
            print_warn "跳过 $path：它指向顶层工作空间仓库（避免修改工作空间分支）"
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
        if [ "$current_branch" = "HEAD" ]; then
            # 游离 HEAD：仅尝试快进到 FETCH_HEAD，不切换分支
            git merge --ff-only FETCH_HEAD 2>/dev/null \
                && print_info "  已快进到远端提交" \
                || print_warn "  游离 HEAD 且无法快进，跳过（不改动分支）"
        elif git rev-parse --verify --quiet "refs/remotes/origin/$current_branch" >/dev/null 2>&1; then
            print_info "  更新当前分支 $current_branch 到最新..."
            # 仅快进合并：不切分支、不强制、不改写历史
            git merge --ff-only "origin/$current_branch" 2>/dev/null \
                || print_warn "  无法快进更新（本地有未推送提交或冲突），保留当前状态"
        else
            print_warn "  远端无分支 origin/$current_branch，跳过更新"
        fi

        # 恢复暂存的本地修改
        if [ "$stashed" -eq 1 ]; then
            print_info "  恢复本地暂存的修改..."
            git stash pop 2>/dev/null \
                || print_warn "  暂存恢复失败（可手动执行: git -C \"$path\" stash pop）"
        fi
    )
    print_info "✓ $path 已更新到当前分支最新（不改动分支）"
}

# 递归初始化并更新子模块（含嵌套内容）
# $1 = 父路径（顶层为 ""）；$2 = 子路径（相对父路径）
init_submodule_recursive() {
    local parent="$1" nested="$2"
    local path rel_branch sub

    if [ -n "$parent" ]; then
        path="$parent/$nested"
    else
        path="$nested"
    fi

    # 分支名：读取最近一层父 .gitmodules 中的配置（仅用于提示，不做切换）
    if [ -n "$parent" ] && [ -f "$parent/.gitmodules" ]; then
        rel_branch="$(git -C "$parent" config --file "$parent/.gitmodules" --get "submodule.$nested.branch" 2>/dev/null || echo main)"
    else
        rel_branch="$(git config --file .gitmodules --get "submodule.$path.branch" 2>/dev/null || echo main)"
    fi

    # 初始化当前子模块的嵌套子模块（arms=源码 时跳过 hardwares/*）
    if [ -e "$path/.git" ] && [ -f "$path/.gitmodules" ]; then
        while IFS= read -r sub; do
            if should_skip_nested_submodule "$path" "$sub"; then
                print_info "跳过嵌套子模块（arms 源码模式）: $path/$sub"
                continue
            fi
            git -C "$path" submodule update --init -- "$sub" 2>/dev/null \
                || print_warn "初始化嵌套子模块失败: $path/$sub"
        done < <(git -C "$path" config --file "$path/.gitmodules" --get-regexp path 2>/dev/null | awk '{print $2}')
    fi

    # 更新当前子模块到当前分支最新
    update_submodule_to_latest "$path"

    # 递归处理嵌套子模块
    if [ -f "$path/.gitmodules" ]; then
        while IFS= read -r sub; do
            if should_skip_nested_submodule "$path" "$sub"; then
                continue
            fi
            init_submodule_recursive "$path" "$sub"
        done < <(git -C "$path" config --file "$path/.gitmodules" --get-regexp path 2>/dev/null | awk '{print $2}')
    fi
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

print_info "工作空间目录: $REPO_DIR"
cd "$REPO_DIR"

if [ ! -d ".git" ]; then
    print_error "当前目录不是 git 仓库！"
    print_info "请先克隆 panthera-ht 分支："
    print_info "  git clone -b panthera-ht git@github.com:fiveages-sim/open-deploy-ws.git ht-deploy-ws"
    exit 1
fi

echo ""
echo "请选择操作："
echo ""
echo "  1) 初始化工作空间（逐模块 source/deb）"
echo "     默认推荐: ocs2=deb，arms=deb(full，含 ht_ros2_control)，common=deb"
echo "     arms=deb 时可选择 full / standard 变体；HT 描述包仍源码"
echo "     deb 可选手动选择通道: latest / pre-release / conf"
echo "  2) 切换模块安装方式（源码 ↔ deb）"
echo "  3) 仅安装/更新核心 deb 包（跳过 Git 子模块拉取）"
echo "  4) 卸载核心 deb 包"
echo "  5) 仅运行 rosdep 安装依赖"
echo ""
read -rp "请输入选项 [1/2/3/4/5]（默认: 1）: " flow_choice
case "$flow_choice" in
    2) FLOW="switch" ;;
    3) FLOW="deb_only" ;;
    4) FLOW="deb_uninstall" ;;
    5) FLOW="rosdep" ;;
    *) FLOW="init" ;;
esac

if [ "$FLOW" = "rosdep" ]; then
    print_info "模式: 仅运行 rosdep"
    echo ""
    run_rosdep_install || exit 1
    exit 0
fi

if [ "$FLOW" = "deb_only" ]; then
    echo ""
    echo "选择要安装/更新的包（逗号分隔短名，回车=全部）："
    echo "  ocs2, common, arms（arms 变体随后选择）"
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
        print_info "  - ros-jazzy-ocs2 (ocs2)"
        _detected+=("ocs2")
    fi
    if is_pkg_installed "ros-jazzy-robot-descriptions-common"; then
        print_info "  - ros-jazzy-robot-descriptions-common (common)"
        _detected+=("common")
    fi
    if is_pkg_installed "ros-jazzy-arms-ros2-control-full"; then
        print_info "  - ros-jazzy-arms-ros2-control-full (arms, full)"
        _detected+=("arms")
    elif is_pkg_installed "ros-jazzy-arms-ros2-control"; then
        print_info "  - ros-jazzy-arms-ros2-control (arms, standard)"
        _detected+=("arms")
    fi
    if [ ${#_detected[@]} -eq 0 ]; then
        print_info "  未检测到已安装的核心 deb 包"
    fi
    echo ""
    echo "选择要卸载的包（逗号分隔短名，回车=全部）："
    echo "  ocs2, common, arms"
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
    echo "  robot-descriptions-ht 始终源码；arms=deb 时 ht-ros2-control 视变体而定"
    prompt_sd "  ocs2_ros2                   默认 $(mode_label "$USE_DEB_OCS2")" \
        "$( [ "$USE_DEB_OCS2" -eq 1 ] && echo d || echo s )" USE_DEB_OCS2
    prompt_sd "  arms_ros2_control           默认 $(mode_label "$USE_DEB_ARMS")" \
        "$( [ "$USE_DEB_ARMS" -eq 1 ] && echo d || echo s )" USE_DEB_ARMS
    prompt_sd "  robot-descriptions-common   默认 $(mode_label "$USE_DEB_COMMON")" \
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
        print_info "  $m ($p): $st"
    done
    echo ""
    echo "为每个模块选择目标（s=source, d=deb, k=保持），回车=保持："
    for m in ocs2 arms common; do
        p="$(module_short_to_path "$m")"
        pkg="$(module_short_to_deb "$m")"
        st="$(detect_module_state "$p" "$pkg")"
        read -rp "  $m 当前=$st [s/d/K]: " tgt
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
        print_info "arms=deb(full) → 安装 arms-full（含 ht_ros2_control），将跳过/清理 src/ht-ros2-control"
    else
        print_info "arms=deb(standard) → 安装标准 arms 包，将源码初始化 src/ht-ros2-control"
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

print_info "将源码子模块更新到当前分支的最新提交（含嵌套子模块，不切换分支）..."

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
print_info "  1. source /opt/ros/jazzy/setup.bash"
print_info "  2. ./quick_start.sh → 编译仿真/真机所需包"
if [ "$USE_DEB_ARMS" -eq 1 ] && [ "$ARMS_VARIANT" = "full" ]; then
    print_info "     （arms=deb(full) 时 ht_ros2_control 已在 arms-full 中，通常只需编译 panthera_ht_description）"
else
    print_info "     （arms=源码 或 arms=deb(standard) 时 ht_ros2_control 由源码编译）"
fi
print_info ""
print_info "如需在源码与 deb 间切换，重新运行 ./init_repo.sh 并选择选项 2"
if [ "$USE_DEB_ARMS" -eq 0 ] || [ "$ARMS_VARIANT" = "standard" ]; then
    print_info "如需更新 HT 源码子模块，可以运行："
    print_info "  git submodule update --remote src/robot-descriptions-ht src/ht-ros2-control"
else
    print_info "如需更新 HT 描述子模块，可以运行："
    print_info "  git submodule update --remote src/robot-descriptions-ht"
fi
