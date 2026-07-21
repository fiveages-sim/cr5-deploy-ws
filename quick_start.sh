#!/usr/bin/env bash

# 快速启动脚本（ARX ACone Deploy Workspace）
# - 自动识别当前 workspace 路径（脚本所在目录）
# - 启动选项参考：README.md

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

need_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 缺少命令：$cmd${NC}"
    return 1
  fi
  return 0
}

ensure_ros_env() {
  # 启动类命令需要 ROS 环境
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
  echo "  3) 编译单臂 arx5 真机包 (Build Single-Arm Real Hardware — Stanford SDK)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3]: " choice
  echo "${choice}"
}

launch_menu() {
  echo "" >&2
  echo "请选择启动项:" >&2
  echo "  1) 双臂 (ACone)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-1]: " choice
  echo "${choice}"
}

launch_mode_menu() {
  echo "" >&2
  echo "请选择运行模式:" >&2
  echo "  1) 仿真 (Simulation / mock_components)" >&2
  echo "  2) 真机 (Real Hardware)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

SDK_DIR="${WS_DIR}/src/arx-ros2-control/external/arx5-sdk"

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
  # conda 环境内含 cxx-compiler（GCC 14）和 conda 版 KDL，若在 conda 激活状态下编译
  # libArxJointController.so，会与 ROS2 运行时的系统 KDL 产生 ABI 不兼容导致 segfault。
  # 因此必须在系统 GCC（/usr/bin/g++）+ 系统 KDL（ROS2 Jazzy）下单独编译。
  echo -e "${YELLOW}[INFO] 第一步：使用系统 GCC 编译 libArxJointController.so（供 ROS2 使用）...${NC}"
  (
    ros_distro="${ROS_DISTRO:-jazzy}"
    [ -f "/opt/ros/${ros_distro}/setup.bash" ] && source "/opt/ros/${ros_distro}/setup.bash"

    # cmake 配置阶段需要 pybind11（python/CMakeLists.txt 中 REQUIRED），
    # 但我们只编译 C++ 目标，不实际链接它。从 conda 获取路径仅用于配置通过。
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
  # arx5_interface.cpython-312-x86_64-linux-gnu.so 需要 pybind11 和 conda Python，
  # 输出到 python/ 目录，供独立 Python 脚本调用，与 ROS2 运行时无关。
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
    echo -e "${GREEN}开始编译单臂 arx5 真机所需包（Stanford SDK）...${NC}"
    build_arx_sdk || exit 1
    cd "${WS_DIR}" || exit 1
    colcon build --packages-up-to \
      arx_ros2_control \
      ocs2_arm_controller \
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
        mode_choice="$(launch_mode_menu)"
        case "${mode_choice}" in
          1)
            echo -e "${GREEN}启动双臂仿真（ACone）...${NC}"
            ensure_ros_env || exit 1
            ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_lift2s type:=acone_x5
            ;;
          2)
            echo -e "${GREEN}启动 LIFT2S 真机（官方 SDK + split body）...${NC}"
            ensure_ros_env || exit 1
            ros2 launch arx_lift2s_description ocs2_real.launch.py type:=acone_x5 hardware:=real
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
