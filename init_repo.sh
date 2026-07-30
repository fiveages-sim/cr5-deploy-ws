#!/bin/bash

# ARX Lift2S / ACone ROS2 部署工作空间初始化脚本
# 参考：https://github.com/fiveages-sim/open-deploy-ws/tree/dobot-cr5
# 功能：初始化顶层 + Lift2S 必需嵌套子模块，并准备 arxlift2s external（SDK symlink / 预编译 libsoem.so）

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

print_info "工作空间目录: $REPO_DIR"
cd "$REPO_DIR"

if [ ! -d ".git" ]; then
    print_error "当前目录不是 git 仓库！"
    print_info "请先克隆主仓库，例如："
    print_info "  git clone git@github.com:fiveages-sim/open-deploy-ws.git arx_lift2s_ws"
    exit 1
fi

# ---------------------------------------------------------------------------
# 将单个 git 仓库切到配置分支的最新提交
# ---------------------------------------------------------------------------
checkout_submodule_branch() {
    local submodule_path="$1"

    local branch_name
    branch_name=$(git config --file .gitmodules --get "submodule.$submodule_path.branch" 2>/dev/null || echo "main")

    if [ ! -d "$submodule_path" ]; then
        print_warn "子模块路径不存在: $submodule_path"
        return 1
    fi

    print_info "处理子模块: $submodule_path -> 分支: $branch_name"
    (
        cd "$submodule_path" || exit 1

        if ! git rev-parse --git-dir > /dev/null 2>&1; then
            print_warn "  不是有效的 git 仓库，跳过"
            exit 0
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
            print_warn "  远程分支 $branch_name 不存在，跳过"
            exit 0
        fi

        local current_branch
        current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")

        if [ "$current_branch" = "$branch_name" ]; then
            print_info "  已在 $branch_name 分支"
        else
            print_info "  从 $current_branch 切换到 $branch_name 分支..."
            if git show-ref --verify --quiet refs/heads/"$branch_name"; then
                git checkout -f "$branch_name" || print_error "  无法切换到 $branch_name"
            else
                git checkout -b "$branch_name" "origin/$branch_name" || \
                  git checkout "$branch_name" || print_error "  无法创建/切换到 $branch_name"
            fi
        fi

        print_info "  更新到最新提交..."
        git pull origin "$branch_name" || print_warn "  拉取更新失败"

        print_info "✓ $submodule_path 已切换到 $branch_name 分支"
    )
}

print_info "开始初始化顶层子模块..."
print_info "同步子模块配置..."
git submodule sync

print_info "初始化所有顶层子模块..."
git submodule update --init

print_info "将顶层子模块切换到对应分支的最新提交..."
submodule_paths=$(git config --file .gitmodules --get-regexp path | awk '{print $2}')
for submodule_path in $submodule_paths; do
    checkout_submodule_branch "$submodule_path"
done

# ---------------------------------------------------------------------------
# Lift2S 真机必需：arms_ros2_control 内嵌套 HI
# ---------------------------------------------------------------------------
ARMS_DIR="src/arms_ros2_control"
ARMS_GITMODULES="${ARMS_DIR}/.gitmodules"
LIFT2S_HI_REL="hardwares/arxlift2s_ros2_control"

if [ -f "$ARMS_GITMODULES" ]; then
    print_info ""
    print_info "初始化 Lift2S 嵌套硬件子模块（arms_ros2_control）..."
    (
        cd "$ARMS_DIR" || exit 1
        git submodule sync -- "$LIFT2S_HI_REL" 2>/dev/null || true
        git submodule update --init -- "$LIFT2S_HI_REL" || \
          print_warn "嵌套子模块 $LIFT2S_HI_REL 初始化失败（若目录已有本地代码可忽略）"
    )
    if [ -d "${ARMS_DIR}/${LIFT2S_HI_REL}" ]; then
        (
            cd "$ARMS_DIR" || exit 1
            branch_name=$(git config --file .gitmodules --get "submodule.${LIFT2S_HI_REL}.branch" 2>/dev/null || echo "main")
            print_info "处理嵌套子模块: ${ARMS_DIR}/${LIFT2S_HI_REL} -> 分支: ${branch_name}"
            cd "$LIFT2S_HI_REL" || exit 1
            if git rev-parse --git-dir > /dev/null 2>&1; then
                git fetch origin || true
                if git ls-remote --exit-code --heads origin "$branch_name" > /dev/null 2>&1; then
                    git checkout -B "$branch_name" "origin/$branch_name" 2>/dev/null || \
                      git checkout "$branch_name" || true
                    git pull origin "$branch_name" || true
                fi
            fi
        )
    fi
else
    print_warn "未找到 ${ARMS_GITMODULES}，跳过嵌套 HI 初始化"
fi

# ---------------------------------------------------------------------------
# arxlift2s_ros2_control external：Stanford SDK symlink + 预编译 libsoem.so
# ---------------------------------------------------------------------------
setup_arxlift2s_external() {
    local pkg="${REPO_DIR}/src/arms_ros2_control/hardwares/arxlift2s_ros2_control"
    local sdk_src="${REPO_DIR}/src/arx-ros2-control/external/arx5-sdk"
    local ext="${pkg}/external"
    local soem_arch="x86_64"
    case "$(uname -m)" in
        aarch64|arm64|armv7l|armv6l) soem_arch="aarch64" ;;
    esac
    local soem_lib="${ext}/SOEM/lib/${soem_arch}/libsoem.so"

    if [ ! -d "$pkg" ]; then
        print_warn "未找到 arxlift2s_ros2_control，跳过 external 准备"
        return 0
    fi

    print_info ""
    print_info "准备 arxlift2s_ros2_control/external（Stanford SDK + 预编译 SOEM）..."
    mkdir -p "$ext"

    if [ ! -e "${ext}/arx5-sdk/include/app/joint_controller.h" ]; then
        if [ -f "${sdk_src}/include/app/joint_controller.h" ]; then
            rm -f "${ext}/arx5-sdk"
            ln -sfn ../../../../arx-ros2-control/external/arx5-sdk "${ext}/arx5-sdk"
            print_info "  已创建 external/arx5-sdk -> arx-ros2-control/external/arx5-sdk"
        else
            print_warn "  未找到 Stanford SDK：${sdk_src}"
            print_warn "  请确认 src/arx-ros2-control 已初始化"
        fi
    else
        print_info "  external/arx5-sdk 已就绪"
    fi

    if [ -f "${soem_lib}" ]; then
        print_info "  external/SOEM/lib/${soem_arch}/libsoem.so 已就绪"
    else
        print_warn "  缺少预编译 SOEM：${soem_lib}"
        print_warn "  见 arxlift2s_ros2_control/external/SOEM/README.md（不再克隆 SOEM 源码）"
    fi
}

setup_arxlift2s_external

print_info ""
print_info "=========================================="
print_info "子模块初始化完成！"
print_info "=========================================="
print_info ""
print_info "顶层子模块状态："
git submodule status

if [ -d "$ARMS_DIR" ]; then
    print_info ""
    print_info "arms_ros2_control 嵌套 HI（节选）："
    git -C "$ARMS_DIR" submodule status -- hardwares/arxlift2s_ros2_control 2>/dev/null \
      || print_warn "  无法读取 arxlift2s 嵌套状态（可能为本地非 submodule 检出）"
fi

print_info ""
print_info "下一步："
print_info "  ./quick_start.sh          # 编译 / 启动（含 Lift2S 真机）"
print_info "  git submodule update --remote   # 更新顶层子模块到远程分支最新"
print_info ""
print_info "Lift2S 真机：臂 can1/can3（Stanford full_control MIX）+ 升降 can5（Hybrid MIT）。"
print_info "升降增益：URDF hybrid_kp/kd 或运行时 arx_lift.hybrid_kp / arx_lift.hybrid_kd。"
print_info "OCS2 配置（fa-w2 风格）：config/ocs2/task_arm.info（分体）/ fixed_base.info（全身）。"
print_info "勿与官方 X5Controller / lift_controller 同总线并行。"
