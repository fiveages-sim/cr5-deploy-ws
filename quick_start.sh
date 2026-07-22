#!/usr/bin/env bash

# 快速启动脚本（Panthera HT Deploy Workspace）
# - 自动识别当前 workspace 路径（脚本所在目录）
# - 启动选项参考：README.md

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 核心包由 deb 提供时，src 下无 arms_ros2_control / ocs2_ros2 等
core_deb_mode() {
  if dpkg-query -W -f='${Status}' ros-jazzy-arms-ros2-control 2>/dev/null | grep -q "install ok installed"; then
    return 0
  fi
  [ ! -d "${WS_DIR}/src/arms_ros2_control" ] && [ ! -d "${WS_DIR}/src/ocs2_ros2" ]
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

source_workspace_env() {
  source_ros_underlay || return 1
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_DIR}/install/setup.bash"
    echo -e "${GREEN}[INFO] 已 source 环境:${NC}"
    echo -e "${BLUE}  /opt/ros/jazzy/setup.bash${NC}"
    echo -e "${BLUE}  ${WS_DIR}/install/setup.bash${NC}"
    return 0
  fi
  echo -e "${YELLOW}[WARN] 未找到 ${WS_DIR}/install/setup.bash${NC}"
  return 1
}

ensure_ros_env() {
  if source_workspace_env; then
    return 0
  fi

  if core_deb_mode; then
    echo -e "${YELLOW}[WARN] deb 已提供核心控制包；请先编译 HT 包：${NC}"
    echo -e "${YELLOW}      ./quick_start.sh → 1) 编译${NC}"
    return 1
  fi

  echo -e "${YELLOW}[WARN] 请先在此 workspace 编译，然后再运行启动选项：${NC}"
  echo -e "${YELLOW}      cd ${WS_DIR} && colcon build --symlink-install${NC}"
  return 1
}


menu() {
  echo -e "${BLUE}========================================${NC}" >&2
  echo -e "${BLUE}  快速启动（Panthera HT Deploy Workspace）${NC}" >&2
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
  echo "  2) 编译真机所需包 (Build Real Hardware Packages)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

launch_menu() {
  echo "" >&2
  echo "请选择启动项:" >&2
  echo "  1) 单臂 (Single)" >&2
  echo "  2) 双臂 (Dual)" >&2
  echo "  3) 手柄遥操作 (Joystick Teleop)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3]: " choice
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

# Real-hardware HI control mode (passed as xacro_control_mode → robot.xacro control_mode)
control_mode_menu() {
  echo "" >&2
  echo "请选择真机控制模式:" >&2
  echo "  1) full_control   — OCS2 MIX（位置+速度+力矩+kp/kd）" >&2
  echo "  2) pd_control     — 位置+力矩，kp/kd" >&2
  echo "  3) position_velocity — 位置+速度+最大力矩" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3] (默认 1): " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  echo "${choice}"
}

# Maps menu choice → control_mode string; empty means back/cancel
resolve_control_mode() {
  case "$1" in
    1) echo "full_control" ;;
    2) echo "pd_control" ;;
    3) echo "position_velocity" ;;
    0) echo "" ;;
    *) echo "INVALID" ;;
  esac
}

# Launch ocs2 demo. Args: arm_label type_args...
# Uses CONTROL_MODE env if set (real only).
_run_ocs2_demo() {
  local arm_label="$1"
  shift
  local -a extra_args=("$@")
  local mode_label=""

  ensure_ros_env || exit 1

  if [ -n "${CONTROL_MODE:-}" ]; then
    extra_args+=("xacro_control_mode:=${CONTROL_MODE}")
    mode_label="，控制模式=${CONTROL_MODE}"
  fi

  echo -e "${GREEN}启动${arm_label}${mode_label}...${NC}"
  ros2 launch ocs2_arm_controller demo.launch.py robot:=panthera_ht "${extra_args[@]}"
}

_joystick_enumerate_devices() {
  if ! command -v ros2 >/dev/null 2>&1; then
    return 1
  fi
  ros2 run joy joy_enumerate_devices 2>/dev/null
}

_joystick_count() {
  local count
  count="$(_joystick_enumerate_devices | grep -cE '^[[:space:]]*[0-9]+[[:space:]]*:' 2>/dev/null)" || count=0
  echo "${count}"
}

_joystick_first_id() {
  local device_id
  device_id="$(_joystick_enumerate_devices | awk -F ':' '/^[[:space:]]*[0-9]+[[:space:]]*:/ {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $1); print $1; exit}')"
  echo "${device_id:-0}"
}

_run_joystick_teleop_launch() {
  local joy_dev="${1:-}"
  ensure_ros_env || exit 1
  if [ -z "${joy_dev}" ]; then
    echo -e "${GREEN}启动手柄遥操作...${NC}"
    ros2 launch arms_teleop joystick_teleop.launch.py
  else
    echo -e "${GREEN}启动手柄遥操作（${joy_dev}）...${NC}"
    ros2 launch arms_teleop joystick_teleop.launch.py "joy_dev:=${joy_dev}"
  fi
}

joystick_device_menu() {
  local menu_lines
  echo "" >&2
  echo "请选择手柄设备:" >&2
  menu_lines="$(_joystick_enumerate_devices | awk -F ':' '/^[[:space:]]*[0-9]+[[:space:]]*:/ {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $1); gsub(/^[[:space:]]+|[[:space:]]+$/, "", $5); printf "  %s) %s\n", $1, $5}')"
  if [ -n "${menu_lines}" ]; then
    echo "${menu_lines}" >&2
  else
    echo "  0) Joy ID 0 (未枚举到手柄，使用默认 ID)" >&2
  fi
  echo "  q) 返回" >&2
  echo "" >&2
  read -r -p "请输入 Joy ID (默认: 0，输入 q 返回): " choice
  if [ -z "${choice}" ]; then
    choice="0"
  fi
  if [ "${choice}" = "q" ] || [ "${choice}" = "Q" ]; then
    choice="back"
  fi
  echo "${choice}"
}

do_launch_joystick_teleop() {
  local device_choice device_count device_id

  ensure_ros_env || exit 1
  device_count="$(_joystick_count)"

  if [ "${device_count}" -eq 1 ]; then
    device_id="$(_joystick_first_id)"
    if [ "${device_id}" = "0" ]; then
      _run_joystick_teleop_launch ""
    else
      _run_joystick_teleop_launch "/dev/input/js${device_id}"
    fi
    return
  fi

  device_choice="$(joystick_device_menu)"

  case "${device_choice}" in
    back)
      echo "返回"
      return
      ;;
    0)
      _run_joystick_teleop_launch ""
      ;;
    *)
      if ! [[ "${device_choice}" =~ ^[0-9]+$ ]]; then
        echo -e "${YELLOW}无效 Joy ID: ${device_choice}${NC}"
        exit 1
      fi
      _run_joystick_teleop_launch "/dev/input/js${device_choice}"
      ;;
  esac
}

need_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 缺少命令：$cmd${NC}"
    return 1
  fi
  return 0
}

run_colcon_build() {
  cd "${WS_DIR}" || exit 1
  source_ros_underlay
  colcon build "$@"
}
need_cmd colcon || echo -e "${YELLOW}[WARN] 未找到 colcon，编译选项会失败（通常需要安装 ROS 发行版环境）。${NC}"

top_choice="$(menu)"

case "${top_choice}" in
  1)
    build_choice="$(build_menu)"
    case "${build_choice}" in
      1)
    echo -e "${GREEN}开始编译仿真所需包...${NC}"
    if core_deb_mode; then
      echo -e "${BLUE}  模式: 核心包 deb + 仅编译 HT 描述包${NC}"
      if ! run_colcon_build --packages-select panthera_ht_description --symlink-install; then
        echo -e "${YELLOW}编译过程中出现错误${NC}"
        exit 1
      fi
    else
      if ! run_colcon_build --packages-up-to \
        ocs2_arm_controller \
        panthera_ht_description \
        arms_teleop \
        adaptive_gripper_controller \
        basic_joint_controller \
        --symlink-install; then
        echo -e "${YELLOW}编译过程中出现错误${NC}"
        exit 1
      fi
    fi
    echo -e "${GREEN}编译完成！${NC}"
    source_workspace_env || echo -e "${YELLOW}[WARN] workspace overlay 未加载，请检查 install/setup.bash${NC}"
    ;;

      2)
    echo -e "${GREEN}开始编译真机所需包...${NC}"
    if core_deb_mode; then
      echo -e "${BLUE}  模式: 核心包 deb + 编译 HT 描述与真机驱动${NC}"
      if ! run_colcon_build --packages-select panthera_ht_description panthera_ros2_control --symlink-install; then
        echo -e "${YELLOW}编译过程中出现错误${NC}"
        exit 1
      fi
    else
      if ! run_colcon_build --packages-up-to \
        panthera_ros2_control \
        ocs2_arm_controller \
        panthera_ht_description \
        arms_teleop \
        adaptive_gripper_controller \
        basic_joint_controller \
        --symlink-install; then
        echo -e "${YELLOW}编译过程中出现错误${NC}"
        exit 1
      fi
    fi
    echo -e "${GREEN}编译完成！${NC}"
    source_workspace_env || echo -e "${YELLOW}[WARN] workspace overlay 未加载，请检查 install/setup.bash${NC}"
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
      1|2)
        arm_label="单臂"
        # panthera_ht xacro 用 type=single 表示单臂；若不传 type，robot_common_launch
        # 会把 OCS2 planning URDF 默认成 dual（14 DOF），与 6 关节控制器不匹配。
        type_arg="type:=single"
        if [ "${launch_choice}" = "2" ]; then
          arm_label="双臂"
          type_arg="type:=dual"
        fi

        mode_choice="$(launch_mode_menu)"
        case "${mode_choice}" in
          1)
            _run_ocs2_demo "${arm_label}仿真（Panthera HT）" "${type_arg}"
            ;;
          2)
            cm_choice="$(control_mode_menu)"
            CONTROL_MODE="$(resolve_control_mode "${cm_choice}")"
            if [ "${CONTROL_MODE}" = "INVALID" ]; then
              echo -e "${YELLOW}无效选项${NC}"
              exit 1
            fi
            if [ -z "${CONTROL_MODE}" ]; then
              echo "返回"
            else
              _run_ocs2_demo "${arm_label}真机（Panthera HT）" "${type_arg}" "hardware:=real"
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
      3)
        do_launch_joystick_teleop
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
