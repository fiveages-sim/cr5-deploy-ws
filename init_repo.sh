#!/bin/bash

# 第五纪双臂轮式人形机器人W1 ROS2部署工作空间初始化脚本
# 功能：按嵌套可见性 + 逐模块 source/deb 选择初始化子模块，并支持源码↔deb 切换

# 不设置 set -e，允许某些命令失败后继续执行
set -u  # 遇到未定义变量时退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$SCRIPT_DIR"
MODE_STATE_FILE="$REPO_DIR/.core_module_mode"

# 模块安装方式：1=deb，0=source
USE_DEB_OCS2=1
USE_DEB_ARMS=0
USE_DEB_COMMON=0
INIT_MODE="public"
FLOW="init"  # init | deb_only | deb_uninstall | switch | rosdep

run_rosdep_install() {
    print_info "运行: rosdep install --from-paths src --ignore-src -r -y"
    if ! command -v rosdep >/dev/null 2>&1; then
        print_error "未找到 rosdep，请先安装 ROS 环境后重试"
        print_info "  sudo apt install python3-rosdep"
        print_info "  sudo rosdep init && rosdep update"
        return 1
    fi
    if [ ! -d "$REPO_DIR/src" ]; then
        print_error "未找到 src 目录: $REPO_DIR/src"
        return 1
    fi
    rosdep install --from-paths src --ignore-src -r -y \
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
INIT_MODE=$INIT_MODE
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
    # 输出: deb | source | mixed | none
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
        common) echo "src/robot-descriptions/common" ;;
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
    # $1=提示 $2=默认 d|s → 设置变量名为 $3 的 1/0
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

print_info "工作空间目录: $REPO_DIR"
cd "$REPO_DIR"

# 检查是否是 git 仓库
if [ ! -d ".git" ]; then
    print_error "当前目录不是 git 仓库！"
    print_info "请先克隆主仓库："
    print_info "  git clone git@github.com:fiveages-sim/open-deploy-ws.git ros2_ws"
    exit 1
fi

# 选择流程
echo ""
echo "请选择操作："
echo ""
echo "  1) 初始化工作空间（嵌套可见性 + 逐模块 source/deb）"
echo "     默认推荐: ocs2=deb，arms=source，common=source"
echo "  2) 切换模块安装方式（源码 ↔ deb）"
echo "  3) 仅安装/更新核心 deb 包（跳过 Git 子模块拉取）"
echo "  4) 卸载核心 deb 包"
echo "  5) 仅运行 rosdep 安装依赖（rosdep install --from-paths src --ignore-src -r -y）"
echo ""
read -rp "请输入选项 [1/2/3/4/5]（默认: 1）: " flow_choice
case "$flow_choice" in
    2) FLOW="switch" ;;
    3) FLOW="deb_only" ;;
    4) FLOW="deb_uninstall" ;;
    5) FLOW="rosdep" ;;
    *) FLOW="init" ;;
esac

# ---------- 快捷：仅 rosdep ----------
if [ "$FLOW" = "rosdep" ]; then
    print_info "模式: 仅运行 rosdep"
    echo ""
    run_rosdep_install || exit 1
    exit 0
fi

# ---------- 快捷：仅 deb / 卸载 ----------
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

# ---------- 可见性 + 模块方式（init / switch 共用提示） ----------
if [ "$FLOW" = "init" ]; then
    echo ""
    echo "嵌套子模块可见性："
    echo "  1) 仅 public（外部用户，无需私有仓库权限）"
    echo "  2) 全部（含 private，需要内部仓库访问权限）"
    read -rp "请输入选项 [1/2]（默认: 1）: " vis_choice
    case "$vis_choice" in
        2) INIT_MODE="private" ;;
        *) INIT_MODE="public" ;;
    esac

    # 若有上次状态，仅用作模块方式默认值（不覆盖刚选的 INIT_MODE）
    _def_ocs2=1
    _def_arms=0
    _def_common=0
    if [ -f "$MODE_STATE_FILE" ]; then
        _def_ocs2="$(grep -E '^USE_DEB_OCS2=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo 1)"
        _def_arms="$(grep -E '^USE_DEB_ARMS=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo 0)"
        _def_common="$(grep -E '^USE_DEB_COMMON=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo 0)"
        [[ "$_def_ocs2" =~ ^[01]$ ]] || _def_ocs2=1
        [[ "$_def_arms" =~ ^[01]$ ]] || _def_arms=0
        [[ "$_def_common" =~ ^[01]$ ]] || _def_common=0
        print_info "检测到上次选择: ocs2=$(mode_label "$_def_ocs2"), arms=$(mode_label "$_def_arms"), common=$(mode_label "$_def_common")"
    fi
    USE_DEB_OCS2="$_def_ocs2"
    USE_DEB_ARMS="$_def_arms"
    USE_DEB_COMMON="$_def_common"

    echo ""
    echo "核心模块安装方式（d=deb, s=source，回车用括号内默认）："
    prompt_sd "  ocs2_ros2              默认 $(mode_label "$USE_DEB_OCS2")" \
        "$( [ "$USE_DEB_OCS2" -eq 1 ] && echo d || echo s )" USE_DEB_OCS2
    prompt_sd "  arms_ros2_control      默认 $(mode_label "$USE_DEB_ARMS")" \
        "$( [ "$USE_DEB_ARMS" -eq 1 ] && echo d || echo s )" USE_DEB_ARMS
    prompt_sd "  robot-descriptions/common 默认 $(mode_label "$USE_DEB_COMMON")" \
        "$( [ "$USE_DEB_COMMON" -eq 1 ] && echo d || echo s )" USE_DEB_COMMON
fi

# switch 流程：仅对「安装方式真正变化」的模块动手；未改动的源码模块绝不 sync/update/pull/reset
CHANGED_OCS2=0
CHANGED_ARMS=0
CHANGED_COMMON=0

set_module_changed() {
    case "$1" in
        ocs2) CHANGED_OCS2="$2" ;;
        arms) CHANGED_ARMS="$2" ;;
        common) CHANGED_COMMON="$2" ;;
    esac
}

get_module_changed() {
    case "$1" in
        ocs2) echo "$CHANGED_OCS2" ;;
        arms) echo "$CHANGED_ARMS" ;;
        common) echo "$CHANGED_COMMON" ;;
        *) echo "0" ;;
    esac
}

# 当前有效安装方式：deb=1 / source=0（mixed 视为需要处理，按目标对齐）
current_use_deb_from_state() {
    case "$1" in
        deb) echo 1 ;;
        source) echo 0 ;;
        mixed) echo "mixed" ;;
        none) echo "none" ;;
        *) echo "none" ;;
    esac
}

if [ "$FLOW" = "switch" ]; then
    # 先读上次可见性，避免覆盖稍后选择的 USE_DEB_*
    _saved_init="public"
    if [ -f "$MODE_STATE_FILE" ]; then
        _saved_init="$(grep -E '^INIT_MODE=' "$MODE_STATE_FILE" 2>/dev/null | cut -d= -f2- || echo public)"
        [ -z "$_saved_init" ] && _saved_init="public"
    fi

    echo ""
    echo "嵌套可见性（仅当有模块改为源码时才会用到）："
    echo "  1) public  2) private/全部"
    read -rp "请输入选项 [1/2]（默认: $([ "$_saved_init" = private ] && echo 2 || echo 1)）: " vis_choice
    case "$vis_choice" in
        2) INIT_MODE="private" ;;
        1) INIT_MODE="public" ;;
        *) INIT_MODE="$_saved_init" ;;
    esac

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
    echo "  注意：选「保持」或目标与当前一致时，不会对该模块做任何 git/deb 操作。"
    for m in ocs2 arms common; do
        p="$(module_short_to_path "$m")"
        pkg="$(module_short_to_deb "$m")"
        st="$(detect_module_state "$p" "$pkg")"
        cur_deb="$(current_use_deb_from_state "$st")"
        read -rp "  $m 当前=$st [s/d/K]: " tgt
        tgt="${tgt:-k}"
        local_target=""
        case "$tgt" in
            s|S|source) local_target=0 ;;
            d|D|deb) local_target=1 ;;
            *)
                # 保持：沿用当前状态；mixed/none 默认偏向 source 以便后续可初始化
                case "$st" in
                    deb) local_target=1 ;;
                    *) local_target=0 ;;
                esac
                # 明确保持且已是纯 deb/source → 不标记变更
                if [ "$st" = "deb" ] || [ "$st" = "source" ]; then
                    set_use_deb_for_module "$m" "$local_target"
                    set_module_changed "$m" 0
                    continue
                fi
                ;;
        esac
        set_use_deb_for_module "$m" "$local_target"
        if [ "$cur_deb" = "$local_target" ]; then
            set_module_changed "$m" 0
            print_info "  → $m 目标与当前一致 ($(mode_label "$local_target"))，跳过"
        else
            set_module_changed "$m" 1
            print_info "  → $m 将切换: $st → $(mode_label "$local_target")"
        fi
    done

    if [ "$CHANGED_OCS2" -eq 0 ] && [ "$CHANGED_ARMS" -eq 0 ] && [ "$CHANGED_COMMON" -eq 0 ]; then
        print_info "没有模块需要切换安装方式，仅保存可见性设置后退出。"
        save_module_mode_state
        exit 0
    fi
    echo ""
    _changed_list=()
    [ "$CHANGED_OCS2" -eq 1 ] && _changed_list+=("ocs2")
    [ "$CHANGED_ARMS" -eq 1 ] && _changed_list+=("arms")
    [ "$CHANGED_COMMON" -eq 1 ] && _changed_list+=("common")
    print_info "本次仅处理有变化的模块: $(IFS=,; echo "${_changed_list[*]}")"
    unset _changed_list
fi

# init 流程：视为三个核心模块都需要按选择对齐（保持原有全量初始化语义）
if [ "$FLOW" = "init" ]; then
    CHANGED_OCS2=1
    CHANGED_ARMS=1
    CHANGED_COMMON=1
fi

# 依赖提示
if [ "$USE_DEB_ARMS" -eq 1 ] && [ "$USE_DEB_OCS2" -eq 0 ]; then
    print_warn "arms 选择 deb 而 ocs2 选择 source：arms deb 通常依赖 ocs2 包，安装可能失败。"
fi

print_info "初始化模式（嵌套）: $INIT_MODE"
print_info "模块方式: ocs2=$(mode_label "$USE_DEB_OCS2"), arms=$(mode_label "$USE_DEB_ARMS"), common=$(mode_label "$USE_DEB_COMMON")"
echo ""

should_skip_top_submodule() {
    local path="$1"
    case "$path" in
        src/ocs2_ros2) [ "$USE_DEB_OCS2" -eq 1 ] && return 0 ;;
        src/arms_ros2_control) [ "$USE_DEB_ARMS" -eq 1 ] && return 0 ;;
    esac
    return 1
}

# switch/init 共用：该顶层路径是否需要本次 git 操作（仅「有变化且目标为 source」；robot-descriptions 仅当 common 有变化）
should_touch_top_submodule() {
    local path="$1"
    case "$path" in
        src/ocs2_ros2)
            [ "$CHANGED_OCS2" -eq 1 ] && [ "$USE_DEB_OCS2" -eq 0 ]
            return $?
            ;;
        src/arms_ros2_control)
            [ "$CHANGED_ARMS" -eq 1 ] && [ "$USE_DEB_ARMS" -eq 0 ]
            return $?
            ;;
        src/robot-descriptions)
            if [ "$FLOW" = "switch" ]; then
                # common→source：父仓已存在则不 sync/pull（只 init nested common）；缺失才 clone
                if [ "$CHANGED_COMMON" -eq 1 ] && [ "$USE_DEB_COMMON" -eq 0 ]; then
                    path_is_git_checkout "src/robot-descriptions" && return 1
                    return 0
                fi
                return 1
            fi
            # init：始终需要父仓（common 用 deb 时仍要其他模型包）
            return 0
            ;;
        *)
            # 未知顶层子模块：init 时照常处理，switch 时不动以免误伤
            [ "$FLOW" = "init" ]
            return $?
            ;;
    esac
}

should_skip_nested_path() {
    local relative_path="$1"
    if [ "$relative_path" = "common" ] && [ "$USE_DEB_COMMON" -eq 1 ]; then
        return 0
    fi
    return 1
}

should_skip_nested_spec() {
    local parent_dir="$1"
    local relative_path="$2"

    if [ "$FLOW" = "switch" ]; then
        case "$parent_dir" in
            src/ocs2_ros2)
                [ "$CHANGED_OCS2" -eq 1 ] && [ "$USE_DEB_OCS2" -eq 0 ] || return 0
                ;;
            src/arms_ros2_control)
                [ "$CHANGED_ARMS" -eq 1 ] && [ "$USE_DEB_ARMS" -eq 0 ] || return 0
                ;;
            src/robot-descriptions)
                # 切换 common→source 时只初始化 common，不碰其他模型子模块
                if [ "$CHANGED_COMMON" -eq 1 ] && [ "$USE_DEB_COMMON" -eq 0 ] \
                    && [ "$relative_path" = "common" ]; then
                    return 1
                fi
                return 0
                ;;
            *)
                return 0
                ;;
        esac
        return 1
    fi

    # init：父仓本身用 deb 时，其全部嵌套都跳过
    case "$parent_dir" in
        src/arms_ros2_control)
            [ "$USE_DEB_ARMS" -eq 1 ] && return 0
            ;;
        src/ocs2_ros2)
            [ "$USE_DEB_OCS2" -eq 1 ] && return 0
            ;;
    esac
    should_skip_nested_path "$relative_path"
}

remove_submodule_path() {
    local path="$1"
    print_info "清理源码目录: $path"

    case "$path" in
        src/robot-descriptions/common)
            if [ -d "$REPO_DIR/src/robot-descriptions" ]; then
                (cd "$REPO_DIR/src/robot-descriptions" && git submodule deinit -f -- common) \
                    2>/dev/null || print_warn "  deinit $path 失败，继续删除目录..."
            fi
            rm -rf "$REPO_DIR/$path"
            rm -rf "$REPO_DIR/.git/modules/src/robot-descriptions/modules/common" 2>/dev/null || true
            ;;
        *)
            git -C "$REPO_DIR" submodule deinit -f -- "$path" \
                2>/dev/null || print_warn "  deinit $path 失败，继续删除目录..."
            rm -rf "$REPO_DIR/$path"
            rm -rf "$REPO_DIR/.git/modules/$path" 2>/dev/null || true
            ;;
    esac
    print_info "✓ 已清理 $path"
}

cleanup_deb_module_sources() {
    local paths_to_clean=() p relative_path full_path clean_choice

    # 仅清理「本次改为 deb」的模块源码，避免误删未切换模块的本地修改
    [ "$CHANGED_OCS2" -eq 1 ] && [ "$USE_DEB_OCS2" -eq 1 ] && path_has_submodule_content "src/ocs2_ros2" && \
        paths_to_clean+=("src/ocs2_ros2")
    [ "$CHANGED_ARMS" -eq 1 ] && [ "$USE_DEB_ARMS" -eq 1 ] && path_has_submodule_content "src/arms_ros2_control" && \
        paths_to_clean+=("src/arms_ros2_control")
    if [ "$CHANGED_COMMON" -eq 1 ] && [ "$USE_DEB_COMMON" -eq 1 ]; then
        full_path="src/robot-descriptions/common"
        path_has_submodule_content "$full_path" && paths_to_clean+=("$full_path")
    fi

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

# 切换：仅对「本次改为 source」且仍装着 deb 的模块卸载
switch_uninstall_debs_for_source_targets() {
    local to_uninstall=()
    local m p pkg st
    for m in ocs2 arms common; do
        if [ "$(get_module_changed "$m")" -eq 1 ] && [ "$(get_use_deb_for_module "$m")" -eq 0 ]; then
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

# 嵌套子模块可见性配置文件
VISIBILITY_CONF="$REPO_DIR/submodules_visibility.conf"
if [ ! -f "$VISIBILITY_CONF" ]; then
    print_error "未找到配置文件: $VISIBILITY_CONF"
    exit 1
fi
trim() { local v="$1"; v="${v#"${v%%[![:space:]]*}"}"; echo "${v%"${v##*[![:space:]]}"}"; }
NESTED_PUBLIC_SPECS=()
NESTED_PRIVATE_SPECS=()
while IFS= read -r line || [ -n "$line" ]; do
    line="${line%%#*}"
    line="${line#"${line%%[![:space:]]*}"}"
    line="${line%"${line##*[![:space:]]}"}"
    [ -z "$line" ] && continue
    IFS='|' read -r parent_dir relative_path visibility <<< "$line"
    parent_dir=$(trim "$parent_dir")
    relative_path=$(trim "$relative_path")
    visibility=$(trim "$visibility")
    gitmodules_file="${parent_dir}/.gitmodules"
    spec="${parent_dir}:${gitmodules_file}:${relative_path}"
    case "$visibility" in
        public)  NESTED_PUBLIC_SPECS+=("$spec") ;;
        private) NESTED_PRIVATE_SPECS+=("$spec") ;;
        *)       print_warn "未知可见性 '$visibility'，跳过: $parent_dir/$relative_path" ;;
    esac
done < "$VISIBILITY_CONF"
print_info "已从 $VISIBILITY_CONF 加载嵌套子模块配置（public: ${#NESTED_PUBLIC_SPECS[@]} 项, private: ${#NESTED_PRIVATE_SPECS[@]} 项）"
echo ""

if [ "$FLOW" = "switch" ]; then
    switch_uninstall_debs_for_source_targets
fi

cleanup_deb_module_sources

print_info "开始初始化子模块..."

# 同步/初始化：仅处理本次需要拉源码的顶层路径
touch_paths=()
while IFS= read -r submodule_path; do
    should_touch_top_submodule "$submodule_path" || continue
    touch_paths+=("$submodule_path")
done < <(git config --file .gitmodules --get-regexp path | awk '{print $2}')

if [ ${#touch_paths[@]} -eq 0 ]; then
    print_info "本次无需拉取/更新任何顶层源码子模块"
else
    print_info "同步子模块配置（仅: ${touch_paths[*]})..."
    git submodule sync -- "${touch_paths[@]}"

    print_info "初始化顶层子模块（仅有变化且目标为 source）..."
    git submodule update --init -- "${touch_paths[@]}"
fi

# 遍历需要处理的源码子模块并切换到对应分支
print_info "将源码子模块切换到对应分支的最新提交..."

submodule_paths=$(git config --file .gitmodules --get-regexp path | awk '{print $2}')

repo_has_local_changes() {
    # 忽略仅子模块指针变化；有普通文件改动或未跟踪文件则视为脏（不自动 stash/reset）
    if ! git diff-files --quiet --ignore-submodules=all 2>/dev/null; then
        return 0
    fi
    if ! git diff-index --cached --quiet --ignore-submodules=all HEAD -- 2>/dev/null; then
        return 0
    fi
    if [ -n "$(git ls-files --others --exclude-standard 2>/dev/null)" ]; then
        return 0
    fi
    return 1
}

for submodule_path in $submodule_paths; do
    if ! should_touch_top_submodule "$submodule_path"; then
        if should_skip_top_submodule "$submodule_path"; then
            print_info "跳过子模块（deb 安装）: $submodule_path"
        else
            print_info "跳过子模块（本次未切换）: $submodule_path"
        fi
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

        if repo_has_local_changes; then
            print_error "  检测到本地修改，跳过以免丢失工作。"
            print_error "  请先在该目录手动提交或 stash，再重新运行切换。"
            print_info "  目录: $REPO_DIR/$submodule_path"
            cd "$REPO_DIR"
            continue
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
                        print_error "  无法切换到 $branch_name 分支（工作区可能非干净），已跳过"
                        cd "$REPO_DIR"
                        continue
                    fi
                else
                    if ! git checkout -b "$branch_name" "origin/$branch_name" 2>/dev/null; then
                        print_error "  无法创建/切换到 $branch_name 分支，已跳过"
                        cd "$REPO_DIR"
                        continue
                    fi
                fi
            else
                if git show-ref --verify --quiet refs/heads/"$branch_name"; then
                    if ! git checkout "$branch_name" 2>/dev/null; then
                        print_error "  无法切换到 $branch_name 分支（工作区可能非干净），已跳过"
                        cd "$REPO_DIR"
                        continue
                    fi
                else
                    if ! git checkout -b "$branch_name" "origin/$branch_name" 2>/dev/null; then
                        print_error "  无法创建/切换到 $branch_name 分支，已跳过"
                        cd "$REPO_DIR"
                        continue
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

# 初始化构建所需的嵌套子模块
print_info "初始化构建所需的嵌套子模块（根据配置文件）..."
for spec in "${NESTED_PUBLIC_SPECS[@]}"; do
    parent_dir="${spec%%:*}"
    rest="${spec#*:}"
    relative_path="${rest#*:}"
    if should_skip_nested_spec "$parent_dir" "$relative_path"; then
        print_info "跳过嵌套子模块（deb 安装）: $parent_dir/$relative_path"
        continue
    fi
    [ ! -d "$parent_dir" ] && continue
    (cd "$parent_dir" && git submodule update --init "$relative_path") || print_warn "$parent_dir/$relative_path 初始化失败，跳过"
done
if [ "$INIT_MODE" = "private" ]; then
    for spec in "${NESTED_PRIVATE_SPECS[@]}"; do
        parent_dir="${spec%%:*}"
        rest="${spec#*:}"
        relative_path="${rest#*:}"
        if should_skip_nested_spec "$parent_dir" "$relative_path"; then
            print_info "跳过嵌套子模块（deb 安装）: $parent_dir/$relative_path"
            continue
        fi
        [ ! -d "$parent_dir" ] && continue
        (cd "$parent_dir" && git submodule update --init "$relative_path") || print_warn "$parent_dir/$relative_path 初始化失败，跳过"
    done
fi

print_info "将构建所需的嵌套子模块切换到对应分支..."
nested_specs=("${NESTED_PUBLIC_SPECS[@]}")
if [ "$INIT_MODE" = "private" ]; then
    nested_specs+=("${NESTED_PRIVATE_SPECS[@]}")
fi
for nested_spec in "${nested_specs[@]}"; do
    parent_dir="${nested_spec%%:*}"
    rest="${nested_spec#*:}"
    gitmodules_file="${rest%%:*}"
    relative_path="${rest#*:}"
    if should_skip_nested_spec "$parent_dir" "$relative_path"; then
        continue
    fi
    full_path="$REPO_DIR/$parent_dir/$relative_path"
    if [ ! -d "$full_path" ]; then continue; fi
    if ! (cd "$full_path" && git rev-parse --git-dir >/dev/null 2>&1); then continue; fi
    gf="$REPO_DIR/$gitmodules_file"
    branch_name=$(git config --file "$gf" --get "submodule.$relative_path.branch" 2>/dev/null)
    if [ -z "$branch_name" ]; then
        config_key=$(git config --file "$gf" --get-regexp 'submodule\..*\.path' 2>/dev/null | awk -v p="$relative_path" '$2==p {k=$1; gsub(/^submodule\.|\.path$/,"",k); print k; exit}')
        branch_name=$(git config --file "$gf" --get "submodule.${config_key}.branch" 2>/dev/null)
    fi
    branch_name=${branch_name:-main}
    print_info "处理嵌套子模块: $parent_dir/$relative_path -> 分支: $branch_name"
    cd "$full_path"
    if repo_has_local_changes; then
        print_error "  检测到本地修改，跳过以免丢失工作: $parent_dir/$relative_path"
        cd "$REPO_DIR" || exit 1
        continue
    fi
    git fetch origin 2>/dev/null || print_warn "  获取远程更新失败，继续..."
    if git ls-remote --exit-code --heads origin "$branch_name" >/dev/null 2>&1; then
        actual_branch="$branch_name"
    else
        actual_branch=$(git ls-remote --symref origin HEAD 2>/dev/null | awk '/^ref: refs\/heads\// {sub(/refs\/heads\//,""); print $2; exit}')
        if [ -z "$actual_branch" ]; then
            print_warn "  远程分支 $branch_name 不存在且无法获取远程默认分支，跳过"
            cd "$REPO_DIR" || exit 1
            continue
        fi
        print_warn "  远程分支 $branch_name 不存在，改用远程默认分支: $actual_branch"
    fi
    current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")
    if [ "$current_branch" != "$actual_branch" ]; then
        if git show-ref --verify --quiet "refs/heads/$actual_branch"; then
            git checkout "$actual_branch" 2>/dev/null || {
                print_error "  无法切换到 $actual_branch，已跳过"
                cd "$REPO_DIR" || exit 1
                continue
            }
        else
            git checkout -b "$actual_branch" "origin/$actual_branch" 2>/dev/null || git checkout "$actual_branch" 2>/dev/null || {
                print_error "  无法创建/切换到 $actual_branch，已跳过"
                cd "$REPO_DIR" || exit 1
                continue
            }
        fi
    fi
    git pull origin "$actual_branch" 2>/dev/null || print_warn "  拉取更新失败"
    print_info "✓ $parent_dir/$relative_path 已切换到 $actual_branch 分支"
    cd "$REPO_DIR" || exit 1
done

print_info ""
print_info "=========================================="
print_info "子模块初始化完成！"
print_info "=========================================="
print_info ""
print_info "当前子模块状态："
git submodule status

# 安装 rosdep 依赖（仅对本次处理的源码路径）
print_info ""
print_info "安装 rosdep 依赖..."
if command -v rosdep >/dev/null 2>&1; then
    rosdep_paths=()
    while IFS= read -r submodule_path; do
        should_touch_top_submodule "$submodule_path" || continue
        rosdep_paths+=("$submodule_path")
    done < <(git config --file .gitmodules --get-regexp path | awk '{print $2}')
    if [ ${#rosdep_paths[@]} -gt 0 ]; then
        rosdep install --from-paths "${rosdep_paths[@]}" --ignore-src -r -y \
            || print_warn "rosdep 安装部分依赖失败，可稍后重试或检查 package.xml"
    else
        print_info "无源码路径需要 rosdep"
    fi
else
    print_warn "未找到 rosdep，请先安装 ROS 环境后手动运行："
    print_info "  cd $REPO_DIR && rosdep install --from-paths src --ignore-src -r -y"
fi

# 安装选中的 deb 包（switch 仅安装「本次改为 deb」的模块）
deb_only_parts=()
for m in ocs2 common arms; do
    if [ "$(get_use_deb_for_module "$m")" -eq 1 ]; then
        if [ "$FLOW" = "switch" ] && [ "$(get_module_changed "$m")" -eq 0 ]; then
            continue
        fi
        deb_only_parts+=("$m")
    fi
done
deb_only_list=""
if [ ${#deb_only_parts[@]} -gt 0 ]; then
    deb_only_list="$(IFS=,; echo "${deb_only_parts[*]}")"
fi
if [ -n "$deb_only_list" ]; then
    print_info ""
    run_install_core_debs "$deb_only_list" || print_error "deb 安装失败，请检查 deb_versions.conf 或网络连接"
fi

save_module_mode_state

print_info ""
print_info "后续步骤："
print_info "  1. source /opt/ros/jazzy/setup.bash"
if [ "$USE_DEB_OCS2" -eq 1 ] && [ "$USE_DEB_ARMS" -eq 1 ] && [ "$USE_DEB_COMMON" -eq 1 ]; then
    print_info "  2. 仅需编译 robot-descriptions 中的机器人模型包，例如："
    print_info "     colcon build --packages-up-to <robot>_description --symlink-install"
else
    print_info "  2. 按需 colcon build 编译源码模块"
fi
print_info ""
print_info "如需在源码与 deb 间切换，重新运行 ./init_repo.sh 并选择选项 2"
print_info "如需更新子模块到最新提交，可以运行："
print_info "  git submodule update --remote"
