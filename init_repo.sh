#!/bin/bash

# HighTorque Panthera HT ROS2 部署工作空间初始化脚本
# 功能：逐模块 source/deb 初始化子模块，支持 ocs2 / arms / common 核心包 deb 安装

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
    print_info "安装核心 deb 包（顺序: ocs2 → common → arms_ros2_control）..."
    local install_script="$REPO_DIR/scripts/install_core_debs.sh"
    if [ ! -x "$install_script" ]; then
        chmod +x "$install_script" 2>/dev/null || true
    fi
    if [ ! -f "$install_script" ]; then
        print_error "未找到安装脚本: $install_script"
        return 1
    fi
    if [ -n "$only_list" ]; then
        bash "$install_script" --only "$only_list"
    else
        bash "$install_script"
    fi
}

run_uninstall_core_debs() {
    local only_list="${1:-}"
    print_info "卸载核心 deb 包（顺序: arms_ros2_control → common → ocs2）..."
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
EOF
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
        arms) echo "ros-jazzy-arms-ros2-control" ;;
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

    if [ ${#paths_to_clean[@]} -eq 0 ]; then
        return 0
    fi

    print_warn "以下目录将改由 deb 提供，检测到已有源码内容："
    for p in "${paths_to_clean[@]}"; do
        print_warn "  - $p"
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

switch_uninstall_debs_for_source_targets() {
    local to_uninstall=()
    local m pkg
    for m in ocs2 arms common; do
        if [ "$(get_use_deb_for_module "$m")" -eq 0 ]; then
            pkg="$(module_short_to_deb "$m")"
            if is_pkg_installed "$pkg"; then
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
echo "     默认推荐: ocs2=deb，arms=deb，common=deb（HT 描述/驱动仍源码编译）"
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
    echo "  ocs2, common, arms"
    read -rp "包列表: " only_choice
    only_choice="$(echo "$only_choice" | tr -d '[:space:]')"
    print_info "模式: 仅安装核心 deb 包（不拉取 Git 子模块）"
    echo ""
    run_install_core_debs "$only_choice" || exit 1
    print_info ""
    print_info "完成后请执行: source /opt/ros/jazzy/setup.bash"
    exit 0
fi

if [ "$FLOW" = "deb_uninstall" ]; then
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
    _def_ocs2=1
    _def_arms=1
    _def_common=1
    if [ -f "$MODE_STATE_FILE" ]; then
        _def_ocs2="$(grep -E '^USE_DEB_OCS2=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo 1)"
        _def_arms="$(grep -E '^USE_DEB_ARMS=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo 1)"
        _def_common="$(grep -E '^USE_DEB_COMMON=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo 1)"
        [[ "$_def_ocs2" =~ ^[01]$ ]] || _def_ocs2=1
        [[ "$_def_arms" =~ ^[01]$ ]] || _def_arms=1
        [[ "$_def_common" =~ ^[01]$ ]] || _def_common=1
        print_info "检测到上次选择: ocs2=$(mode_label "$_def_ocs2"), arms=$(mode_label "$_def_arms"), common=$(mode_label "$_def_common")"
    fi
    USE_DEB_OCS2="$_def_ocs2"
    USE_DEB_ARMS="$_def_arms"
    USE_DEB_COMMON="$_def_common"

    echo ""
    echo "核心模块安装方式（d=deb, s=source，回车用括号内默认）："
    echo "  HT 专用包 robot-descriptions-ht / ht-ros2-control 始终源码编译"
    prompt_sd "  ocs2_ros2                   默认 $(mode_label "$USE_DEB_OCS2")" \
        "$( [ "$USE_DEB_OCS2" -eq 1 ] && echo d || echo s )" USE_DEB_OCS2
    prompt_sd "  arms_ros2_control           默认 $(mode_label "$USE_DEB_ARMS")" \
        "$( [ "$USE_DEB_ARMS" -eq 1 ] && echo d || echo s )" USE_DEB_ARMS
    prompt_sd "  robot-descriptions-common   默认 $(mode_label "$USE_DEB_COMMON")" \
        "$( [ "$USE_DEB_COMMON" -eq 1 ] && echo d || echo s )" USE_DEB_COMMON
fi

if [ "$FLOW" = "switch" ]; then
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

print_info "模块方式: ocs2=$(mode_label "$USE_DEB_OCS2"), arms=$(mode_label "$USE_DEB_ARMS"), common=$(mode_label "$USE_DEB_COMMON")"
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

print_info "将源码子模块切换到对应分支的最新提交..."

submodule_paths=$(git config --file .gitmodules --get-regexp path | awk '{print $2}')

for submodule_path in $submodule_paths; do
    if should_skip_top_submodule "$submodule_path"; then
        print_info "跳过子模块（deb 安装）: $submodule_path"
        continue
    fi
    branch_name=$(git config --file .gitmodules --get "submodule.$submodule_path.branch" || echo "main")

    if [ -d "$submodule_path" ]; then
        print_info "处理子模块: $submodule_path -> 分支: $branch_name"
        cd "$submodule_path"

        if ! git rev-parse --git-dir > /dev/null 2>&1; then
            print_warn "子模块 $submodule_path 不是有效的 git 仓库，跳过"
            cd "$REPO_DIR"
            continue
        fi

        if ! git diff-index --quiet HEAD -- 2>/dev/null; then
            print_warn "  检测到本地修改，先暂存..."
            git stash push -m "Auto-stash before branch switch" || print_warn "  暂存失败，尝试重置..."
            if ! git diff-index --quiet HEAD -- 2>/dev/null; then
                print_warn "  暂存失败，重置本地修改..."
                git reset --hard HEAD || true
            fi
        fi

        print_info "  获取远程更新..."
        git fetch origin || print_warn "  获取远程更新失败，继续..."

        if ! git ls-remote --exit-code --heads origin "$branch_name" > /dev/null 2>&1; then
            print_warn "  远程分支 $branch_name 不存在，跳过 $submodule_path"
            cd "$REPO_DIR"
            continue
        fi

        current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")

        if [ "$current_branch" = "$branch_name" ]; then
            print_info "  已在 $branch_name 分支"
        else
            print_info "  从 $current_branch 切换到 $branch_name 分支..."

            if [ "$current_branch" = "HEAD" ] || [ -z "$current_branch" ]; then
                if git show-ref --verify --quiet refs/heads/"$branch_name"; then
                    if ! git checkout "$branch_name" 2>/dev/null; then
                        print_warn "  切换失败，尝试强制切换..."
                        git checkout -f "$branch_name" || print_error "  无法切换到 $branch_name 分支"
                    fi
                else
                    if ! git checkout -b "$branch_name" "origin/$branch_name" 2>/dev/null; then
                        print_warn "  创建分支失败，尝试直接切换..."
                        git checkout "$branch_name" || print_error "  无法创建/切换到 $branch_name 分支"
                    fi
                fi
            else
                if git show-ref --verify --quiet refs/heads/"$branch_name"; then
                    if ! git checkout "$branch_name" 2>/dev/null; then
                        print_warn "  切换失败，尝试强制切换..."
                        git checkout -f "$branch_name" || print_error "  无法切换到 $branch_name 分支"
                    fi
                else
                    if ! git checkout -b "$branch_name" "origin/$branch_name" 2>/dev/null; then
                        print_error "  无法创建/切换到 $branch_name 分支"
                    fi
                fi
            fi

            final_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")
            if [ "$final_branch" != "$branch_name" ]; then
                print_error "  切换失败：当前仍在 $final_branch，目标分支是 $branch_name"
                cd "$REPO_DIR"
                continue
            fi
        fi

        print_info "  更新到最新提交..."
        git pull origin "$branch_name" || print_warn "  拉取更新失败"

        cd "$REPO_DIR"
        print_info "✓ $submodule_path 已切换到 $branch_name 分支"
    else
        print_warn "子模块路径不存在: $submodule_path"
    fi
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
print_info "  2. ./quick_start.sh → 编译仿真所需包"
print_info "     （核心包已 deb 时，仅编译 panthera_ht_description 等 HT 包）"
print_info ""
print_info "如需在源码与 deb 间切换，重新运行 ./init_repo.sh 并选择选项 2"
print_info "如需更新 HT 源码子模块，可以运行："
print_info "  git submodule update --remote src/robot-descriptions-ht src/ht-ros2-control"
