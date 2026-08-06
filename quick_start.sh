#!/usr/bin/env bash

# 快速启动脚本（ARX Lift2S / ACone Deploy Workspace）
# 架构对齐：https://github.com/fiveages-sim/open-deploy-ws/tree/panthera-ht
# 风格参考：dobot-cr5 / main
#
# - 真机 HI：src/arx-ros2-control（包 arx_ros2_control）
# - 描述：src/robot-descriptions-arx
# - 臂：仅 full_control（MIT MIX）；单臂真机可选左/右（can1/can3）
# - 升降：hybrid（默认）| soft_p/position

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARX_HI_DIR="${WS_DIR}/src/arx-ros2-control"
SDK_DIR="${ARX_HI_DIR}/external/arx5-sdk"
LIFT_DIR="${ARX_HI_DIR}/external/arx_lift_src"

need_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 缺少命令：$cmd${NC}"
    return 1
  fi
  return 0
}

source_ros_underlay() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    return 0
  fi
  echo -e "${YELLOW}[WARN] 未找到 /opt/ros/jazzy/setup.bash${NC}"
  return 1
}

ensure_ros_env() {
  source_ros_underlay || true
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_DIR}/install/setup.bash"
    return 0
  fi
  echo -e "${YELLOW}[WARN] 未找到 ${WS_DIR}/install/setup.bash${NC}"
  echo -e "${YELLOW}      请先选择「编译」生成 install/，再启动。${NC}"
  return 1
}

soem_arch_dir() {
  case "$(uname -m)" in
    aarch64|arm64|armv7l|armv6l) echo "aarch64" ;;
    *) echo "x86_64" ;;
  esac
}

print_can_hint() {
  echo -e "${BLUE}真机 CAN 提示：${NC}"
  echo -e "${BLUE}  - 左臂 can1 / 右臂 can3（ArxX5Hardware：仅 full_control / MIT MIX）${NC}"
  echo -e "${BLUE}  - 升降 can5（ArxLiftHardware：hybrid | soft_p/position）${NC}"
  echo -e "${BLUE}  - 升降 hybrid：HI 重力+摩擦前馈，忽略上层 effort${NC}"
  echo -e "${BLUE}  - 升降热调：ros2 param set /controller_manager arx_lift.hybrid_kp 5.0${NC}"
  echo -e "${BLUE}  - 升降热调：ros2 param set /controller_manager arx_lift.hybrid_kd 2.0${NC}"
  echo -e "${BLUE}  - OCS2：分体 task_arm.info / 全身 fixed_base.info${NC}"
  echo -e "${BLUE}  - 勿与官方 X5Controller / lift_controller 同总线并行${NC}"
  echo -e "${BLUE}  - 检查：ip link show can1 can3 can5${NC}"
}

# 编译真机前校验 HI 外部依赖
ensure_arx_hi_external() {
  local arch
  arch="$(soem_arch_dir)"

  if [ ! -d "${ARX_HI_DIR}" ]; then
    echo -e "${RED}[ERROR] 未找到 ${ARX_HI_DIR}${NC}"
    echo -e "${YELLOW}      请先运行 ./init_repo.sh${NC}"
    return 1
  fi

  if [ ! -f "${SDK_DIR}/include/app/joint_controller.h" ]; then
    echo -e "${RED}[ERROR] 缺少 Stanford SDK：${SDK_DIR}${NC}"
    return 1
  fi

  if [ ! -f "${SDK_DIR}/lib/${arch}/libhardware.so" ] || [ ! -f "${SDK_DIR}/lib/${arch}/libsolver.so" ]; then
    echo -e "${RED}[ERROR] 缺少 arx5-sdk 预编译库（${arch}）${NC}"
    return 1
  fi

  if [ ! -f "${LIFT_DIR}/lib/${arch}/libarx_lift_src.so" ]; then
    echo -e "${YELLOW}[WARN] 缺少 libarx_lift_src.so（Lift2S 升降真机需要）${NC}"
    echo -e "${YELLOW}      路径：${LIFT_DIR}/lib/${arch}/libarx_lift_src.so${NC}"
  fi

  echo -e "${GREEN}[INFO] arx-ros2-control external 依赖检查通过${NC}"
  return 0
}

# 单臂真机：左 / 右
arm_side_menu() {
  echo "" >&2
  echo "请选择单臂侧（CAN）：" >&2
  echo "  1) 左臂 (Left)  — can1" >&2
  echo "  2) 右臂 (Right) — can3" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2] (默认 1): " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  echo "${choice}"
}

# Maps menu → can_interface；空表示返回
resolve_arm_can() {
  case "$1" in
    1) echo "can1" ;;
    2) echo "can3" ;;
    0) echo "" ;;
    *) echo "INVALID" ;;
  esac
}

# 升降：soft_p/position | hybrid
lift_motor_mode_menu() {
  echo "" >&2
  echo "请选择真机升降控制模式：" >&2
  echo "  1) soft_p / position — Soft-P setHeight（只用 position）" >&2
  echo "  2) hybrid           — sendLiftHybrid（MIT；默认，推荐）" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2] (默认 2): " choice
  if [ -z "${choice}" ]; then
    choice="2"
  fi
  echo "${choice}"
}

resolve_lift_motor_mode() {
  case "$1" in
    1) echo "soft_p" ;;
    2) echo "hybrid" ;;
    0) echo "" ;;
    *) echo "INVALID" ;;
  esac
}

# $1=描述 $2=launch 包+文件 $3=基础参数
# $5=1 时：单臂真机询问左/右 CAN（写入 xacro_can_interface）
# $6=1 时：真机询问升降 lift_motor_mode
do_launch() {
  local description="$1"
  local launch_file="$2"
  local base_args="$3"
  local mode_choice="${4:-}"
  local ask_arm_side="${5:-0}"
  local ask_lift_mode="${6:-0}"
  local can_arg=""
  local lift_motor_mode_arg=""
  local mode_label=""

  if [ -z "${mode_choice}" ]; then
    mode_choice="$(launch_mode_menu)"
  fi

  # 真机 / 真机 headless
  if { [ "${mode_choice}" = "1" ] || [ "${mode_choice}" = "2" ]; }; then
    if [ "${ask_arm_side}" = "1" ]; then
      local side_choice can_if
      side_choice="$(arm_side_menu)"
      can_if="$(resolve_arm_can "${side_choice}")"
      if [ "${can_if}" = "INVALID" ]; then
        echo -e "${YELLOW}无效选项${NC}"
        exit 1
      fi
      if [ -z "${can_if}" ]; then
        echo "返回"
        return 0
      fi
      can_arg="xacro_can_interface:=${can_if}"
      if [ "${can_if}" = "can1" ]; then
        mode_label="，左臂 can1"
      else
        mode_label="，右臂 can3"
      fi
    fi

    # 臂固定 full_control（不询问 position）
    mode_label="${mode_label}，臂=full_control"

    if [ "${ask_lift_mode}" = "1" ]; then
      local lm_choice lift_motor_mode
      lm_choice="$(lift_motor_mode_menu)"
      lift_motor_mode="$(resolve_lift_motor_mode "${lm_choice}")"
      if [ "${lift_motor_mode}" = "INVALID" ]; then
        echo -e "${YELLOW}无效选项${NC}"
        exit 1
      fi
      if [ -z "${lift_motor_mode}" ]; then
        echo "返回"
        return 0
      fi
      lift_motor_mode_arg="xacro_lift_motor_mode:=${lift_motor_mode}"
      mode_label="${mode_label}，升降=${lift_motor_mode}"
    fi
  fi

  case "${mode_choice}" in
    1)
      echo -e "${GREEN}启动${description}（真机${mode_label}）...${NC}"
      ensure_ros_env || exit 1
      print_can_hint
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=real ${can_arg} ${lift_motor_mode_arg}
      ;;
    2)
      echo -e "${GREEN}启动${description}（真机 headless${mode_label}）...${NC}"
      ensure_ros_env || exit 1
      print_can_hint
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=real launch_mode:=control_only ${can_arg} ${lift_motor_mode_arg}
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
  echo -e "${BLUE}  快速启动（ARX Lift2S Deploy Workspace）${NC}" >&2
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
  echo "  1) 编译仿真所需包 (Simulation)" >&2
  echo "  2) 编译真机所需包 (Real — arx_ros2_control + 描述)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

launch_menu() {
  echo "" >&2
  echo "请选择启动项:" >&2
  echo "  1) 单臂 X5 (arx5)" >&2
  echo "  2) 双臂 ACone (arx_acone)" >&2
  echo "  3) Lift2S 分体控制 (split_body)" >&2
  echo "  4) Lift2S 全身控制 (full_body)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-4]: " choice
  echo "${choice}"
}

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

run_colcon_build() {
  cd "${WS_DIR}" || exit 1
  source_ros_underlay || true
  colcon build "$@"
}

need_cmd git || exit 1
need_cmd colcon || echo -e "${YELLOW}[WARN] 未找到 colcon，编译选项会失败。${NC}"

top_choice="$(menu)"

case "${top_choice}" in
  1)
    build_choice="$(build_menu)"
    case "${build_choice}" in
      1)
        echo -e "${GREEN}开始编译仿真所需包...${NC}"
        if ! run_colcon_build --packages-up-to \
          ocs2_arm_controller \
          arx_acone_description \
          arx_lift2s_description \
          component_models \
          arx5_description \
          arms_teleop \
          adaptive_gripper_controller \
          basic_joint_controller \
          --symlink-install; then
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
        fi
        echo -e "${GREEN}编译完成！${NC}"
        ;;
      2)
        echo -e "${GREEN}开始编译真机所需包（arx_ros2_control）...${NC}"
        ensure_arx_hi_external || exit 1
        if ! run_colcon_build --packages-up-to \
          arx_ros2_control \
          ocs2_arm_controller \
          arx_acone_description \
          arx_lift2s_description \
          component_models \
          arx5_description \
          arms_teleop \
          adaptive_gripper_controller \
          basic_joint_controller \
          --symlink-install; then
          echo -e "${YELLOW}编译失败。检查 src/arx-ros2-control/external（见 init_repo / 包 README）。${NC}"
          exit 1
        fi
        echo -e "${GREEN}编译完成！${NC}"
        echo -e "${BLUE}启动：单臂可选左/右 CAN；臂仅 full_control；升降可选 hybrid/soft_p${NC}"
        print_can_hint
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
        # 单臂：真机时询问左/右 CAN；臂固定 full_control
        do_launch "单臂 X5" "ocs2_arm_controller demo.launch.py" "robot:=arx5" "" "1" "0"
        ;;
      2)
        # 双臂：can1+can3 固定；不问侧
        do_launch "双臂 ACone" "ocs2_arm_controller demo.launch.py" "robot:=arx_acone" "" "0" "0"
        ;;
      3)
        do_launch "Lift2S 分体控制" "ocs2_arm_controller split_body.launch.py" "robot:=arx_lift2s" "" "0" "1"
        ;;
      4)
        do_launch "Lift2S 全身控制" "ocs2_arm_controller full_body.launch.py" "robot:=arx_lift2s" "" "0" "1"
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
