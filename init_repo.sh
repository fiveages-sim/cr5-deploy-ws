#!/bin/bash

# ARX Lift2S / ACone ROS2 部署工作空间初始化脚本
# 架构对齐：https://github.com/fiveages-sim/open-deploy-ws/tree/panthera-ht
# 风格参考：dobot-cr5 / main
#
# 顶层子模块：
#   arms_ros2_control / arx-ros2-control / ocs2_ros2 /
#   robot-descriptions-arx / robot-descriptions-common
#
# 真机 HI：src/arx-ros2-control（包名 arx_ros2_control）
# 描述：src/robot-descriptions-arx

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
    print_info "请先克隆，例如："
    print_info "  git clone git@github.com:fiveages-sim/open-deploy-ws.git lift2s-ws"
    exit 1
fi

# ---------------------------------------------------------------------------
# 将单个 git 仓库切到 .gitmodules 配置分支的最新提交
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
# ocs2_ros2：按需初始化其嵌套子模块（若存在 .gitmodules）
# ---------------------------------------------------------------------------
OCS2_DIR="src/ocs2_ros2"
if [ -f "${OCS2_DIR}/.gitmodules" ]; then
    print_info ""
    print_info "初始化 ocs2_ros2 嵌套子模块..."
    (
        cd "$OCS2_DIR" || exit 1
        git submodule sync
        git submodule update --init --recursive || \
          print_warn "ocs2_ros2 嵌套子模块初始化失败（可稍后手动处理）"
    )
fi

# ---------------------------------------------------------------------------
# 校验真机 HI 依赖：src/arx-ros2-control/external/{arx5-sdk, arx_lift_src}
# ---------------------------------------------------------------------------
check_arx_hi_external() {
    local pkg="${REPO_DIR}/src/arx-ros2-control"
    local sdk="${pkg}/external/arx5-sdk"
    local lift="${pkg}/external/arx_lift_src"
    local arch="x86_64"
    case "$(uname -m)" in
        aarch64|arm64|armv7l|armv6l) arch="aarch64" ;;
    esac

    print_info ""
    print_info "检查 arx-ros2-control 真机依赖..."

    if [ ! -d "$pkg" ]; then
        print_error "未找到 ${pkg}（请确认 .gitmodules 中 src/arx-ros2-control 已初始化）"
        return 1
    fi

    if [ -f "${sdk}/include/app/joint_controller.h" ]; then
        print_info "  arx5-sdk 头文件就绪"
    else
        print_warn "  缺少 Stanford SDK：${sdk}/include/app/joint_controller.h"
    fi

    if [ -f "${sdk}/lib/${arch}/libhardware.so" ] && [ -f "${sdk}/lib/${arch}/libsolver.so" ]; then
        print_info "  arx5-sdk 预编译库就绪 (${arch})"
    else
        print_warn "  缺少 arx5-sdk/lib/${arch}/libhardware.so 或 libsolver.so"
    fi

    if [ -f "${lift}/lib/${arch}/libarx_lift_src.so" ]; then
        print_info "  arx_lift_src 库就绪 (${arch})"
    else
        print_warn "  缺少 ${lift}/lib/${arch}/libarx_lift_src.so（Lift2S 升降真机需要）"
    fi
}

check_arx_hi_external

print_info ""
print_info "=========================================="
print_info "子模块初始化完成！"
print_info "=========================================="
print_info ""
print_info "顶层子模块状态："
git submodule status

print_info ""
print_info "下一步："
print_info "  rosdep install --from-paths src --ignore-src -r -y"
print_info "  ./quick_start.sh          # 编译 / 启动"
print_info "  git submodule update --remote   # 更新顶层子模块到远程分支最新"
print_info ""
print_info "真机说明："
print_info "  HI 包：src/arx-ros2-control（arx_ros2_control）"
print_info "  描述：src/robot-descriptions-arx"
print_info "  臂：仅 full_control（MIT MIX）；单臂可选左 can1 / 右 can3"
print_info "  升降：can5，hybrid（默认）或 soft_p/position"
print_info "  勿与官方 X5Controller / lift_controller 同总线并行。"
