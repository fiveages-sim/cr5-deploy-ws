#!/usr/bin/env bash

# 快速启动脚本（ARX ACone / Lift2S Deploy Workspace）
# - 自动识别当前 workspace 路径（脚本所在目录）
# - 运行模式与 fa_w2_ws 对齐：真机 / 仿真 / Isaac / headless / 仅可视化

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="${WS_DIR}/src/arx-ros2-control/external/arx5-sdk"

need_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 缺少命令：$cmd${NC}"
    return 1
  fi
  return 0
}

ensure_ros_env() {
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_DIR}/install/setup.bash"
    return 0
  fi

  echo -e "${YELLOW}[WARN] 未找到 ${WS_DIR}/install/setup.bash${NC}"
  echo -e "${YELLOW}      请先在此 workspace 编译，然后再运行启动选项：${NC}"
  echo -e "${YELLOW}      cd ${WS_DIR} && colcon build --symlink-install${NC}"
  return 1
}

# Stanford 真机 HI 控制模式（xacro_control_mode → robot.xacro control_mode）
# 对齐 panthera-ht；ARX 无 position_velocity（Stanford 无 MAXtqe 等价路径）
control_mode_menu() {
  echo "" >&2
  echo "请选择真机控制模式（参考 panthera-ht）:" >&2
  echo "  1) full_control — OCS2 MIX（位置+速度+力矩+kp/kd，推荐真机）" >&2
  echo "  2) position     — 保留真机位置环（仅 position + HI joint_k/d_gains）" >&2
  echo "  3) pd_control   — position 的 HT 别名" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3] (默认 1): " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  echo "${choice}"
}

resolve_control_mode() {
  case "$1" in
    1) echo "full_control" ;;
    2) echo "position" ;;
    3) echo "pd_control" ;;
    0) echo "" ;;
    *) echo "INVALID" ;;
  esac
}

# 通用启动：对齐 fa_w2_ws
# $1=描述 $2=launch 文件 $3=基础参数 $4=(可选)硬件覆盖 $5=(可选)预选运行模式
# $6=1 时：单/双臂 Stanford 真机（模式 1/2）额外询问 control_mode（不动 Lift2S）
do_launch() {
  local description="$1"
  local launch_file="$2"
  local base_args="$3"
  local hardware_override="${4:-}"
  local mode_choice="${5:-}"
  local ask_control_mode="${6:-0}"
  local control_mode_arg=""
  local mode_label=""

  if [ -z "${mode_choice}" ]; then
    mode_choice="$(launch_mode_menu)"
  fi

  # 仅 Stanford 单/双臂真机询问控制模式；仿真 / Lift2S 不走此分支
  if [ "${ask_control_mode}" = "1" ] && { [ "${mode_choice}" = "1" ] || [ "${mode_choice}" = "2" ]; }; then
    local cm_choice control_mode
    cm_choice="$(control_mode_menu)"
    control_mode="$(resolve_control_mode "${cm_choice}")"
    if [ "${control_mode}" = "INVALID" ]; then
      echo -e "${YELLOW}无效选项${NC}"
      exit 1
    fi
    if [ -z "${control_mode}" ]; then
      echo "返回"
      return 0
    fi
    control_mode_arg="xacro_control_mode:=${control_mode}"
    mode_label="，控制模式=${control_mode}"
  fi

  case "${mode_choice}" in
    1)
      echo -e "${GREEN}启动${description}（真机${mode_label}）...${NC}"
      ensure_ros_env || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=${hardware_override:-real} ${control_mode_arg}
      ;;
    2)
      echo -e "${GREEN}启动${description}（真机 headless${mode_label}）...${NC}"
      ensure_ros_env || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=${hardware_override:-real} launch_mode:=control_only ${control_mode_arg}
      ;;
    3)
      echo -e "${GREEN}启动${description}（仿真）...${NC}"
      ensure_ros_env || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=mock_components
      ;;
    4)
      echo -e "${GREEN}启动${description}（仿真 headless）...${NC}"
      ensure_ros_env || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=mock_components launch_mode:=control_only
      ;;
    5)
      echo -e "${GREEN}启动${description}（Isaac）...${NC}"
      ensure_ros_env || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=isaac
      ;;
    6)
      echo -e "${GREEN}启动${description}（Isaac headless）...${NC}"
      ensure_ros_env || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=isaac launch_mode:=control_only
      ;;
    7)
      echo -e "${GREEN}启动${description}（仅可视化）...${NC}"
      ensure_ros_env || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} launch_mode:=rviz_only
      ;;
    0)
      echo "返回"
      ;;
    *)
      echo -e "${YELLOW}无效选项${NC}"
      exit 1
      ;;
  esac
}

menu() {
  echo -e "${BLUE}========================================${NC}" >&2
  echo -e "${BLUE}  快速启动（ARX ACone Deploy Workspace）${NC}" >&2
  echo -e "${BLUE}  Workspace: ${WS_DIR}${NC}" >&2
  echo -e "${BLUE}========================================${NC}" >&2
  echo "" >&2
  echo "请选择操作:" >&2
  echo "  1) 编译 (Build)" >&2
  echo "  2) 启动 (Launch)" >&2
  echo "  0) 退出 (Exit)" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

build_menu() {
  echo "" >&2
  echo "请选择编译目标:" >&2
  echo "  1) 编译仿真所需包 (Build Simulation Packages)" >&2
  echo "  2) 编译 LIFT2S 真机包 (Build LIFT2S Real Hardware — official SDK)" >&2
  echo "  3) 编译单/双臂真机包 (Stanford arx_ros2_control — arx5 / arx_acone)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3]: " choice
  echo "${choice}"
}

launch_menu() {
  echo "" >&2
  echo "请选择启动项:" >&2
  echo "  1) 单臂 X5 (Stanford：仿真 / 真机 MIX)" >&2
  echo "  2) 双臂 ACone (Stanford：仿真 / 真机 MIX)" >&2
  echo "  3) Lift2S 分体控制 (split_body)" >&2
  echo "  4) Lift2S 全身控制 (full_body)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-4]: " choice
  echo "${choice}"
}

# 与 fa_w2_ws 对齐的运行模式菜单
launch_mode_menu() {
  echo "" >&2
  echo "请选择运行模式:" >&2
  echo "  1) 真机启动 (Real Hardware)" >&2
  echo "  2) 真机 headless 模式启动 (Real Hardware, No RViz)" >&2
  echo "  3) 仿真启动 (Simulation / mock_components)" >&2
  echo "  4) 仿真 headless 模式启动 (mock_components, No RViz)" >&2
  echo "  5) Isaac 仿真启动 (hardware:=isaac)" >&2
  echo "  6) Isaac 仿真 headless 启动 (hardware:=isaac, control_only)" >&2
  echo "  7) 仅可视化模式启动 (Visualization Only / rviz_only)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-7]: " choice
  echo "${choice}"
}

build_arx_sdk() {
  echo -e "${YELLOW}[INFO] 编译 ARX SDK...${NC}"

  if ! command -v conda >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 未找到 conda，请先安装 Anaconda/Miniconda${NC}"
    return 1
  fi

  local conda_base
  conda_base=$(conda info --base 2>/dev/null)
  # shellcheck disable=SC1090
  source "${conda_base}/etc/profile.d/conda.sh"

  if ! conda env list | grep -qE "^arx-py312[[:space:]]"; then
    echo -e "${YELLOW}[INFO] 创建 conda 环境 arx-py312...${NC}"
    if command -v mamba >/dev/null 2>&1; then
      mamba env create -f "${SDK_DIR}/conda_environments/py312_environment.yaml" || return 1
    else
      conda env create -f "${SDK_DIR}/conda_environments/py312_environment.yaml" || return 1
    fi
  fi

  # ── 第一步：系统 GCC 编译 C++ 库，供 ROS2 运行时使用 ─────────────
  echo -e "${YELLOW}[INFO] 第一步：使用系统 GCC 编译 libArxJointController.so（供 ROS2 使用）...${NC}"
  (
    ros_distro="${ROS_DISTRO:-jazzy}"
    [ -f "/opt/ros/${ros_distro}/setup.bash" ] && source "/opt/ros/${ros_distro}/setup.bash"

    pybind11_dir=$(conda run -n arx-py312 python3 -c \
      "import pybind11; print(pybind11.get_cmake_dir())" 2>/dev/null || true)

    cmake_args=(-DCMAKE_CXX_COMPILER=/usr/bin/g++ -DCMAKE_C_COMPILER=/usr/bin/gcc)
    if [ -n "${pybind11_dir}" ]; then
      cmake_args+=("-Dpybind11_DIR=${pybind11_dir}")
    fi

    mkdir -p "${SDK_DIR}/build"
    cd "${SDK_DIR}/build" || exit 1
    cmake .. "${cmake_args[@]}" || exit 1
    make -j"$(nproc)" ArxJointController ArxCartesianController || exit 1
  ) || return 1
  echo -e "${GREEN}[INFO] 第一步完成：libArxJointController.so 已生成${NC}"

  # ── 第二步：conda 环境编译 Python 绑定 ───────────────────────────
  echo -e "${YELLOW}[INFO] 第二步：使用 conda 环境编译 Python 绑定...${NC}"
  (
    conda activate arx-py312 || exit 1
    ros_distro="${ROS_DISTRO:-jazzy}"
    [ -f "/opt/ros/${ros_distro}/setup.bash" ] && source "/opt/ros/${ros_distro}/setup.bash"

    mkdir -p "${SDK_DIR}/build-conda"
    cd "${SDK_DIR}/build-conda" || exit 1
    cmake .. || exit 1
    make -j"$(nproc)" arx5_interface || exit 1
  ) || return 1
  echo -e "${GREEN}[INFO] 第二步完成：arx5_interface Python 绑定已生成${NC}"

  echo -e "${GREEN}ARX SDK 编译完成！${NC}"
}

need_cmd git || exit 1
need_cmd colcon || echo -e "${YELLOW}[WARN] 未找到 colcon，编译选项会失败（通常需要安装 ROS 发行版环境）。${NC}"

top_choice="$(menu)"

case "${top_choice}" in
  1)
    build_choice="$(build_menu)"
    case "${build_choice}" in
      1)
        echo -e "${GREEN}开始编译仿真所需包...${NC}"
        cd "${WS_DIR}" || exit 1
        colcon build --packages-up-to \
          ocs2_arm_controller \
          arx_acone_description \
          arx_lift2s_description \
          component_models \
          arx5_description \
          arms_teleop \
          adaptive_gripper_controller \
          basic_joint_controller \
          --symlink-install
        if [ $? -eq 0 ]; then
          echo -e "${GREEN}编译完成！${NC}"
        else
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
        fi
        ;;
      2)
        echo -e "${GREEN}开始编译 LIFT2S 真机所需包（官方 SDK）...${NC}"
        cd "${WS_DIR}" || exit 1
        colcon build --packages-up-to \
          arxlift2s_ros2_control \
          ocs2_arm_controller \
          arx_acone_description \
          arx_lift2s_description \
          component_models \
          arx5_description \
          arms_teleop \
          adaptive_gripper_controller \
          basic_joint_controller \
          --symlink-install
        if [ $? -eq 0 ]; then
          echo -e "${GREEN}编译完成！${NC}"
        else
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
        fi
        ;;
      3)
        echo -e "${GREEN}开始编译单/双臂真机所需包（Stanford SDK：arx5 / arx_acone）...${NC}"
        build_arx_sdk || exit 1
        cd "${WS_DIR}" || exit 1
        colcon build --packages-up-to \
          arx_ros2_control \
          ocs2_arm_controller \
          arx_acone_description \
          arx_lift2s_description \
          arx5_description \
          arms_teleop \
          adaptive_gripper_controller \
          basic_joint_controller \
          --symlink-install
        if [ $? -eq 0 ]; then
          echo -e "${GREEN}编译完成！${NC}"
        else
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
        fi
        ;;
      0)
        echo "返回"
        ;;
      *)
        echo -e "${YELLOW}无效选项${NC}"
        exit 1
        ;;
    esac
    ;;

  2)
    launch_choice="$(launch_menu)"
    case "${launch_choice}" in
      1)
        # Stanford 单臂：仿真走 mock/isaac；真机询问 full_control / position
        do_launch "单臂 X5" "ocs2_arm_controller demo.launch.py" "robot:=arx5" "" "" "1"
        ;;
      2)
        # Stanford 双臂 AC One：同上（can1 / can3）
        do_launch "双臂 ACone" "ocs2_arm_controller demo.launch.py" "robot:=arx_acone" "" "" "1"
        ;;
      3)
        do_launch "Lift2S 分体控制" "ocs2_arm_controller split_body.launch.py" "robot:=arx_lift2s"
        ;;
      4)
        do_launch "Lift2S 全身控制" "ocs2_arm_controller full_body.launch.py" "robot:=arx_lift2s"
        ;;
      0)
        echo "返回"
        ;;
      *)
        echo -e "${YELLOW}无效选项${NC}"
        exit 1
        ;;
    esac
    ;;

  0)
    echo "退出"
    exit 0
    ;;

  *)
    echo -e "${YELLOW}无效选项，请重新运行脚本${NC}"
    exit 1
    ;;
esac
