#!/bin/bash

# 第五纪双臂轮式人形机器人W1 ROS2部署工作空间初始化脚本
# 功能：自动初始化仓库并将所有子模块切换到对应分支的最新提交

# 不设置 set -e，允许某些命令失败后继续执行
set -u  # 遇到未定义变量时退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的消息
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

print_info "工作空间目录: $REPO_DIR"
cd "$REPO_DIR"

# 检查是否是 git 仓库
if [ ! -d ".git" ]; then
    print_error "当前目录不是 git 仓库！"
    print_info "请先克隆主仓库："
    print_info "  git clone git@github.com:fiveages-sim/open-deploy-ws.git ros2_ws"
    exit 1
fi

# 选择初始化模式
echo ""
echo "请选择初始化模式："
echo "  1) 仅初始化 public 仓库（适用于外部用户，无需私有仓库访问权限）"
echo "  2) 初始化所有仓库，包含 private 仓库（需要内部仓库访问权限）"
read -rp "请输入选项 [1/2]（默认: 1）: " mode_choice
case "$mode_choice" in
    2) INIT_MODE="private" ;;
    *) INIT_MODE="public" ;;
esac
print_info "初始化模式: $INIT_MODE"
echo ""

# 嵌套子模块可见性配置文件（可编辑此文件以调整 public/private）
VISIBILITY_CONF="$REPO_DIR/submodules_visibility.conf"
if [ ! -f "$VISIBILITY_CONF" ]; then
    print_error "未找到配置文件: $VISIBILITY_CONF"
    exit 1
fi
trim() { local v="$1"; v="${v#"${v%%[![:space:]]*}"}"; echo "${v%"${v##*[![:space:]]}"}"; }
# 从配置文件加载嵌套子模块列表：格式 父目录|相对路径|public|private
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

print_info "开始初始化子模块..."

# 同步子模块配置（不递归，只处理第一层子模块）
print_info "同步子模块配置..."
git submodule sync

# 初始化所有子模块（不递归，只处理第一层子模块）
print_info "初始化所有子模块..."
git submodule update --init

# 嵌套子模块的初始化延后到第一层子模块切换分支之后，避免旧提交缺失路径导致 pathspec 报错

# 遍历所有子模块并切换到对应分支
print_info "将子模块切换到对应分支的最新提交..."

# 获取所有子模块路径
submodule_paths=$(git config --file .gitmodules --get-regexp path | awk '{print $2}')

for submodule_path in $submodule_paths; do
    # 获取对应的分支配置
    branch_name=$(git config --file .gitmodules --get "submodule.$submodule_path.branch" || echo "main")
    
        if [ -d "$submodule_path" ]; then
        print_info "处理子模块: $submodule_path -> 分支: $branch_name"
        cd "$submodule_path"
        
        # 确保在 git 仓库中（子模块的 .git 可能是文件或目录）
        if ! git rev-parse --git-dir > /dev/null 2>&1; then
            print_warn "子模块 $submodule_path 不是有效的 git 仓库，跳过"
            cd "$REPO_DIR"
            continue
        fi
        
        # 检查是否有本地修改，如果有则先 stash
        if ! git diff-index --quiet HEAD -- 2>/dev/null; then
            print_warn "  检测到本地修改，先暂存..."
            git stash push -m "Auto-stash before branch switch" || print_warn "  暂存失败，尝试重置..."
            # 如果 stash 失败，尝试重置（丢弃本地修改）
            if ! git diff-index --quiet HEAD -- 2>/dev/null; then
                print_warn "  暂存失败，重置本地修改..."
                git reset --hard HEAD || true
            fi
        fi
        
        # 获取远程最新更新
        print_info "  获取远程更新..."
        git fetch origin || print_warn "  获取远程更新失败，继续..."
        
        # 检查远程分支是否存在
        if ! git ls-remote --exit-code --heads origin "$branch_name" > /dev/null 2>&1; then
            print_warn "  远程分支 $branch_name 不存在，跳过 $submodule_path"
            cd "$REPO_DIR"
            continue
        fi
        
        # 切换到指定分支
        # 如果当前处于 detached HEAD 状态，先切换到分支
        current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")
        
        # 如果已经在目标分支，跳过切换
        if [ "$current_branch" = "$branch_name" ]; then
            print_info "  已在 $branch_name 分支"
        else
            print_info "  从 $current_branch 切换到 $branch_name 分支..."
            
            if [ "$current_branch" = "HEAD" ] || [ -z "$current_branch" ]; then
                # 处于 detached HEAD 状态，创建或切换到分支
                if git show-ref --verify --quiet refs/heads/"$branch_name"; then
                    # 本地分支存在，切换到它
                    if ! git checkout "$branch_name" 2>/dev/null; then
                        print_warn "  切换失败，尝试强制切换..."
                        git checkout -f "$branch_name" || print_error "  无法切换到 $branch_name 分支"
                    fi
                else
                    # 创建新的本地分支跟踪远程分支
                    if ! git checkout -b "$branch_name" "origin/$branch_name" 2>/dev/null; then
                        print_warn "  创建分支失败，尝试直接切换..."
                        git checkout "$branch_name" || print_error "  无法创建/切换到 $branch_name 分支"
                    fi
                fi
            else
                # 当前在其他分支，切换到目标分支
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
            
            # 验证是否成功切换到目标分支
            final_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")
            if [ "$final_branch" != "$branch_name" ]; then
                print_error "  切换失败：当前仍在 $final_branch，目标分支是 $branch_name"
                cd "$REPO_DIR"
                continue
            fi
        fi
        
        # 确保在最新提交
        print_info "  更新到最新提交..."
        git pull origin "$branch_name" || print_warn "  拉取更新失败"
        
        cd "$REPO_DIR"
        print_info "✓ $submodule_path 已切换到 $branch_name 分支"
    else
        print_warn "子模块路径不存在: $submodule_path"
    fi
done

# 初始化构建所需的嵌套子模块（根据 submodules_visibility.conf）
print_info "初始化构建所需的嵌套子模块（根据配置文件）..."
for spec in "${NESTED_PUBLIC_SPECS[@]}"; do
    parent_dir="${spec%%:*}"
    rest="${spec#*:}"
    relative_path="${rest#*:}"
    [ ! -d "$parent_dir" ] && continue
    (cd "$parent_dir" && git submodule update --init "$relative_path") || print_warn "$parent_dir/$relative_path 初始化失败，跳过"
done
if [ "$INIT_MODE" = "private" ]; then
    for spec in "${NESTED_PRIVATE_SPECS[@]}"; do
        parent_dir="${spec%%:*}"
        rest="${spec#*:}"
        relative_path="${rest#*:}"
        [ ! -d "$parent_dir" ] && continue
        (cd "$parent_dir" && git submodule update --init "$relative_path") || print_warn "$parent_dir/$relative_path 初始化失败，跳过"
    done
fi

# 将构建所需的嵌套子模块切换到对应分支并更新到最新提交（根据配置文件）
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
    git fetch origin 2>/dev/null || print_warn "  获取远程更新失败，继续..."
    if git ls-remote --exit-code --heads origin "$branch_name" >/dev/null 2>&1; then
        actual_branch="$branch_name"
    else
        # 配置的分支（如 main）在远程不存在，改用远程默认分支
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
            git checkout "$actual_branch" 2>/dev/null || git checkout -f "$actual_branch" 2>/dev/null || true
        else
            git checkout -b "$actual_branch" "origin/$actual_branch" 2>/dev/null || git checkout "$actual_branch" 2>/dev/null || true
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

# 安装 rosdep 依赖（需已安装 ROS 与 rosdep）
print_info ""
print_info "安装 rosdep 依赖..."
if command -v rosdep >/dev/null 2>&1; then
    rosdep install --from-paths src --ignore-src -r -y || print_warn "rosdep 安装部分依赖失败，可稍后重试或检查 package.xml"
else
    print_warn "未找到 rosdep，请先安装 ROS 环境后手动运行："
    print_info "  cd $REPO_DIR && rosdep install --from-paths src --ignore-src -r -y"
fi

print_info ""
print_info "如需更新子模块到最新提交，可以运行："
print_info "  git submodule update --remote"


