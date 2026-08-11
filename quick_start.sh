#!/usr/bin/env bash

# 快速启动脚本（ARX Lift2S / ACone Deploy Workspace）
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

# 工作区配置（编译包列表 / 发布保留子模块）
BUILD_SIM_PACKAGES=()
BUILD_REAL_PACKAGES=()
BUILD_DEB_PACKAGES=()
BUILD_DEB_REAL_PACKAGES=()
QS_CONFIG="${WS_DIR}/config/quick_start.conf"
if [[ -f "${QS_CONFIG}" ]]; then
  # shellcheck source=/dev/null
  source "${QS_CONFIG}"
fi

# conf 未定义时的回退包列表
if [[ ${#BUILD_SIM_PACKAGES[@]} -eq 0 ]]; then
  BUILD_SIM_PACKAGES=(
    ocs2_arm_controller
    arx5_description
    arx_acone_description
    arx_lift2s_description
    arms_teleop
    adaptive_gripper_controller
    basic_joint_controller
  )
fi
if [[ ${#BUILD_REAL_PACKAGES[@]} -eq 0 ]]; then
  BUILD_REAL_PACKAGES=(
    arx_ros2_control
    ocs2_arm_controller
    arx5_description
    arx_acone_description
    arx_lift2s_description
    arms_teleop
    adaptive_gripper_controller
    basic_joint_controller
  )
fi
if [[ ${#BUILD_DEB_PACKAGES[@]} -eq 0 ]]; then
  BUILD_DEB_PACKAGES=(
    arx5_description
    arx_acone_description
    arx_lift2s_description
  )
fi
if [[ ${#BUILD_DEB_REAL_PACKAGES[@]} -eq 0 ]]; then
  BUILD_DEB_REAL_PACKAGES=(
    arx_ros2_control
    arx5_description
    arx_acone_description
    arx_lift2s_description
  )
fi

# 核心栈由 deb 提供（发布包现场）：系统已装 arms，或工作区无控制器源码
_pkg_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

core_deb_mode() {
  if _pkg_installed "ros-jazzy-arms-ros2-control" \
    || _pkg_installed "ros-jazzy-arms-ros2-control-full"; then
    return 0
  fi
  # 发布包占位：目录在但无 arms 控制器源码树
  if [ -d "${WS_DIR}/src/arms_ros2_control" ] \
    && [ ! -d "${WS_DIR}/src/arms_ros2_control/controller" ] \
    && [ ! -d "${WS_DIR}/src/arms_ros2_control/command" ]; then
    return 0
  fi
  if [ ! -d "${WS_DIR}/src/arms_ros2_control" ] && [ ! -d "${WS_DIR}/src/ocs2_ros2" ]; then
    return 0
  fi
  return 1
}

# arms-full 已提供 arx_ros2_control，真机编译无需再编该包
arx_from_deb() {
  _pkg_installed "ros-jazzy-arms-ros2-control-full"
}

warn_hi_overlay_conflict() {
  if arx_from_deb && [ -d "${WS_DIR}/install/arx_ros2_control" ]; then
    echo -e "${YELLOW}[WARN] 检测到 install/arx_ros2_control，可能遮住 arms-full deb 中的 HI 插件${NC}"
    echo -e "${YELLOW}      建议删除该目录后重新 source，再只编译描述包${NC}"
  fi
}

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
  if core_deb_mode; then
    echo -e "${YELLOW}      deb 已提供核心控制栈；请先：./quick_start.sh → 1) 编译（将只编描述 / 真机 HI）${NC}"
  else
    echo -e "${YELLOW}      请先选择「编译」生成 install/，再启动。${NC}"
  fi
  return 1
}

# Zenoh router must be up BEFORE ros2 launch. Starting it inside the same launch
# races robot_state_publisher vs controller_manager on /robot_description
# (CM stays on "Waiting for data on 'robot_description' topic").
is_zenoh_router_running() {
  pgrep -x rmw_zenohd >/dev/null 2>&1 || pgrep -x zenohd >/dev/null 2>&1
}

ensure_zenoh_router() {
  if [ "${RMW_IMPLEMENTATION:-}" != "rmw_zenoh_cpp" ]; then
    return 0
  fi

  if is_zenoh_router_running; then
    echo -e "${GREEN}[INFO] Zenoh router 已在运行，跳过启动${NC}"
    return 0
  fi

  if ! command -v ros2 >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] ros2 不可用，无法启动 Zenoh router${NC}"
    return 1
  fi
  if ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1; then
    echo -e "${YELLOW}[WARN] 未安装 rmw_zenoh_cpp；请: sudo apt install ros-jazzy-rmw-zenoh-cpp${NC}"
    echo -e "${YELLOW}      或 unset RMW_IMPLEMENTATION 后重试${NC}"
    return 1
  fi

  local log_file="${WS_DIR}/log/rmw_zenohd.log"
  mkdir -p "${WS_DIR}/log"
  echo -e "${GREEN}[INFO] 预启动 Zenoh router（避免 /robot_description 竞态）...${NC}"
  nohup ros2 run rmw_zenoh_cpp rmw_zenohd >"${log_file}" 2>&1 &
  disown || true

  local i=0
  while [ "${i}" -lt 50 ]; do
    if is_zenoh_router_running; then
      sleep 0.8
      echo -e "${GREEN}[INFO] Zenoh router 就绪（log: ${log_file}）${NC}"
      return 0
    fi
    sleep 0.1
    i=$((i + 1))
  done

  echo -e "${RED}[ERROR] Zenoh router 启动超时，见 ${log_file}${NC}"
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

  # libhardware.so → libsoem.so (SOEM 1.4.x)；缺了真机 HI 会 dlopen 失败
  local soem=""
  for candidate in \
    "${CONDA_PREFIX:-}/lib/libsoem.so" \
    "${HOME}/miniconda3/envs/arx-py312/lib/libsoem.so" \
    "${HOME}/mambaforge/envs/arx-py312/lib/libsoem.so" \
    "${HOME}/miniforge3/envs/arx-py312/lib/libsoem.so" \
    "${ARX_HI_DIR}/external/SOEM/lib/${arch}/libsoem.so"
  do
    if [ -n "${candidate}" ] && [ -f "${candidate}" ]; then
      soem="${candidate}"
      break
    fi
  done
  if [ -n "${soem}" ]; then
    echo -e "${GREEN}[INFO] 找到兼容 SOEM：${soem}${NC}"
  else
    echo -e "${YELLOW}[WARN] 未找到 libsoem.so；真机加载 arx_ros2_control 会失败${NC}"
    echo -e "${YELLOW}      安装：conda install -n arx-py312 conda-forge::soem=1.4.0${NC}"
    echo -e "${YELLOW}      或放入：${ARX_HI_DIR}/external/SOEM/lib/${arch}/libsoem.so${NC}"
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

# ==================== 上次启动记录（方案 A：完整复现） ====================
QS_LAST_LAUNCH_FILE="${WS_DIR}/config/launch_last.conf"

_save_launch_last() {
  # $1=desc $2=launch_file $3=base_args $4=mode_choice
  # $5=ask_arm_side $6=ask_lift_mode $7=can_arg $8=lift_motor_mode_arg
  local f="${QS_LAST_LAUNCH_FILE}"
  mkdir -p "$(dirname "${f}")"
  {
    printf 'LAST_DESC=%q\n' "${1:-}"
    printf 'LAST_LAUNCH_FILE=%q\n' "${2:-}"
    printf 'LAST_BASE_ARGS=%q\n' "${3:-}"
    printf 'LAST_MODE_CHOICE=%q\n' "${4:-}"
    printf 'LAST_ASK_ARM_SIDE=%q\n' "${5:-0}"
    printf 'LAST_ASK_LIFT_MODE=%q\n' "${6:-0}"
    printf 'LAST_CAN_ARG=%q\n' "${7:-}"
    printf 'LAST_LIFT_MODE_ARG=%q\n' "${8:-}"
  } > "${f}"
}

_load_launch_last() {
  LAST_DESC="" LAST_LAUNCH_FILE="" LAST_BASE_ARGS="" LAST_MODE_CHOICE=""
  LAST_ASK_ARM_SIDE="" LAST_ASK_LIFT_MODE="" LAST_CAN_ARG="" LAST_LIFT_MODE_ARG=""
  if [ -f "${QS_LAST_LAUNCH_FILE}" ]; then
    # shellcheck disable=SC1090
    source "${QS_LAST_LAUNCH_FILE}" 2>/dev/null || true
  fi
}

_mode_choice_label() {
  case "$1" in
    1) echo "真机" ;;
    2) echo "真机headless" ;;
    3) echo "仿真" ;;
    4) echo "仿真headless" ;;
    5) echo "Isaac" ;;
    6) echo "Isaac headless" ;;
    7) echo "仅可视化" ;;
    *) echo "模式${1}" ;;
  esac
}

# 使用上次记录完整复现（不再询问侧/升降/运行模式）
_do_last_launch() {
  _load_launch_last
  if [ -z "${LAST_LAUNCH_FILE}" ] || [ -z "${LAST_MODE_CHOICE}" ]; then
    echo -e "${YELLOW}[WARN] 无有效上次启动记录（${QS_LAST_LAUNCH_FILE}）${NC}"
    return 1
  fi
  echo -e "${GREEN}使用上次启动：${LAST_DESC}${NC}"
  do_launch "${LAST_DESC}" "${LAST_LAUNCH_FILE}" "${LAST_BASE_ARGS}" \
    "${LAST_MODE_CHOICE}" "${LAST_ASK_ARM_SIDE:-0}" "${LAST_ASK_LIFT_MODE:-0}" \
    "1" "${LAST_CAN_ARG}" "${LAST_LIFT_MODE_ARG}"
}

# ==================== 手柄遥操作 ====================
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

_run_joystick_teleop_launch() {
  local joy_dev="${1:-}"
  ensure_ros_env || exit 1
  echo -e "${BLUE}[INFO] 手柄遥操需机器人 stack 已在运行（另开终端启动真机/仿真）${NC}"
  if [ -z "${joy_dev}" ]; then
    echo -e "${GREEN}启动手柄遥操作...${NC}"
    ros2 launch arms_teleop joystick_teleop.launch.py
  else
    echo -e "${GREEN}启动手柄遥操作（${joy_dev}）...${NC}"
    ros2 launch arms_teleop joystick_teleop.launch.py "joy_dev:=${joy_dev}"
  fi
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

  if [ "${device_count}" -eq 0 ]; then
    echo -e "${YELLOW}[WARN] 未枚举到手柄，使用默认 /dev/input/js0${NC}"
    _run_joystick_teleop_launch ""
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

# $1=描述 $2=launch 包+文件 $3=基础参数
# $4=预选运行模式（空则询问）
# $5=1 时：单臂真机询问左/右 CAN
# $6=1 时：真机询问升降 lift_motor_mode
# $7=1 时：复现模式，跳过侧/升降询问，使用 $8/$9
# $8=preset can_arg  $9=preset lift_motor_mode_arg
do_launch() {
  local description="$1"
  local launch_file="$2"
  local base_args="$3"
  local mode_choice="${4:-}"
  local ask_arm_side="${5:-0}"
  local ask_lift_mode="${6:-0}"
  local replay="${7:-0}"
  local can_arg="${8:-}"
  local lift_motor_mode_arg="${9:-}"
  local mode_label=""
  local save_desc=""

  if [ -z "${mode_choice}" ]; then
    mode_choice="$(launch_mode_menu)"
  fi

  # 真机 / 真机 headless
  if { [ "${mode_choice}" = "1" ] || [ "${mode_choice}" = "2" ]; }; then
    if [ "${replay}" = "1" ]; then
      if [ -n "${can_arg}" ]; then
        if [[ "${can_arg}" == *can1* ]]; then
          mode_label="，左臂 can1"
        elif [[ "${can_arg}" == *can3* ]]; then
          mode_label="，右臂 can3"
        fi
      fi
      mode_label="${mode_label}，臂=full_control"
      if [ -n "${lift_motor_mode_arg}" ]; then
        mode_label="${mode_label}，升降=${lift_motor_mode_arg#xacro_lift_motor_mode:=}"
      fi
    else
      can_arg=""
      lift_motor_mode_arg=""
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
  fi

  case "${mode_choice}" in
    [1-7])
      save_desc="${description}"
      if [[ "${description}" != *"+"* ]]; then
        save_desc="${description} + $(_mode_choice_label "${mode_choice}")${mode_label}"
      fi
      _save_launch_last "${save_desc}" "${launch_file}" "${base_args}" \
        "${mode_choice}" "${ask_arm_side}" "${ask_lift_mode}" \
        "${can_arg}" "${lift_motor_mode_arg}"
      ;;
  esac

  case "${mode_choice}" in
    1)
      echo -e "${GREEN}启动${description}（真机${mode_label}）...${NC}"
      ensure_ros_env || exit 1
      ensure_zenoh_router || exit 1
      print_can_hint
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=real ${can_arg} ${lift_motor_mode_arg}
      ;;
    2)
      echo -e "${GREEN}启动${description}（真机 headless${mode_label}）...${NC}"
      ensure_ros_env || exit 1
      ensure_zenoh_router || exit 1
      print_can_hint
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=real launch_mode:=control_only ${can_arg} ${lift_motor_mode_arg}
      ;;
    3)
      echo -e "${GREEN}启动${description}（仿真）...${NC}"
      ensure_ros_env || exit 1
      ensure_zenoh_router || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=mock_components
      ;;
    4)
      echo -e "${GREEN}启动${description}（仿真 headless）...${NC}"
      ensure_ros_env || exit 1
      ensure_zenoh_router || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=mock_components launch_mode:=control_only
      ;;
    5)
      echo -e "${GREEN}启动${description}（Isaac）...${NC}"
      ensure_ros_env || exit 1
      ensure_zenoh_router || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=isaac
      ;;
    6)
      echo -e "${GREEN}启动${description}（Isaac headless）...${NC}"
      ensure_ros_env || exit 1
      ensure_zenoh_router || exit 1
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=isaac launch_mode:=control_only
      ;;
    7)
      echo -e "${GREEN}启动${description}（仅可视化）...${NC}"
      ensure_ros_env || exit 1
      ensure_zenoh_router || exit 1
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
  if core_deb_mode; then
    echo -e "  ${BLUE}（已检测核心包 deb / 发布包模式）${NC}" >&2
    if arx_from_deb; then
      echo -e "  ${BLUE}  arms-full 含 arx_ros2_control：仿真/真机均只编 ARX 描述包${NC}" >&2
    else
      echo -e "  ${BLUE}  arms-standard：真机须额外编译 arx_ros2_control（源码 HI）${NC}" >&2
    fi
  fi
  echo "  1) 编译仿真所需包 (Simulation)" >&2
  echo "  2) 编译真机所需包 (Real — arx_ros2_control + 描述)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

run_colcon_build() {
  cd "${WS_DIR}" || exit 1
  source_ros_underlay || true
  colcon build "$@"
}

# 按包列表 --packages-up-to ... --symlink-install
run_colcon_packages_up_to() {
  local -a pkgs=("$@")
  local pkg_args=()
  local p
  for p in "${pkgs[@]}"; do
    [ -n "${p}" ] || continue
    pkg_args+=("${p}")
  done
  if [ "${#pkg_args[@]}" -eq 0 ]; then
    echo -e "${RED}[ERROR] 编译包列表为空${NC}"
    return 1
  fi
  run_colcon_build --packages-up-to "${pkg_args[@]}" --symlink-install
}

launch_menu() {
  local last_desc="${1:-}"
  local base=1 max_opt

  echo "" >&2
  echo "请选择启动项:" >&2
  if [ -n "${last_desc}" ]; then
    echo "  1) 使用上次 — ${last_desc}" >&2
    base=2
  fi
  echo "  ${base}) 单臂 X5 (arx5)" >&2
  echo "  $((base + 1))) 双臂 ACone (arx_acone)" >&2
  echo "  $((base + 2))) Lift2S 分体控制 (split_body)" >&2
  echo "  $((base + 3))) Lift2S 全身控制 (full_body)" >&2
  echo "" >&2
  echo -e "  ${BLUE}━━━━━━━━━━━━━━━━ 辅助功能 ━━━━━━━━━━━━━━━━${NC}" >&2
  echo "" >&2
  echo "  $((base + 4))) 手柄遥操作 (Joystick Teleop)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  max_opt=$((base + 4))
  read -r -p "请输入选项 [0-${max_opt}]: " choice
  echo "${choice}"
}

# 解析 launch_menu → last | single | dual | split | full | joystick | back | invalid
_resolve_launch_menu_choice() {
  local choice="$1"
  local offset=0
  [ -n "${LAST_DESC}" ] && offset=1

  if [ "${choice}" = "0" ]; then
    echo "back"
    return 0
  fi
  if [ "${offset}" -eq 1 ] && [ "${choice}" = "1" ]; then
    echo "last"
    return 0
  fi

  local idx=$((choice - offset))
  case "${idx}" in
    1) echo "single" ;;
    2) echo "dual" ;;
    3) echo "split" ;;
    4) echo "full" ;;
    5) echo "joystick" ;;
    *) echo "invalid" ;;
  esac
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

need_cmd git || exit 1
need_cmd colcon || echo -e "${YELLOW}[WARN] 未找到 colcon，编译选项会失败。${NC}"

top_choice="$(menu)"

case "${top_choice}" in
  1)
    build_choice="$(build_menu)"
    case "${build_choice}" in
      1)
        echo -e "${GREEN}开始编译仿真所需包...${NC}"
        if core_deb_mode; then
          warn_hi_overlay_conflict
          echo -e "${BLUE}  模式: 核心包 deb + 仅编译 ARX 描述包${NC}"
          if ! run_colcon_packages_up_to "${BUILD_DEB_PACKAGES[@]}"; then
            echo -e "${YELLOW}编译过程中出现错误${NC}"
            exit 1
          fi
        else
          if ! run_colcon_packages_up_to "${BUILD_SIM_PACKAGES[@]}"; then
            echo -e "${YELLOW}编译过程中出现错误${NC}"
            exit 1
          fi
        fi
        echo -e "${GREEN}编译完成！${NC}"
        ;;
      2)
        echo -e "${GREEN}开始编译真机所需包...${NC}"
        if core_deb_mode; then
          warn_hi_overlay_conflict
          if arx_from_deb; then
            echo -e "${BLUE}  模式: 核心包 deb(arms-full 含 arx_ros2_control) + 仅编译 ARX 描述包${NC}"
            if ! run_colcon_packages_up_to "${BUILD_DEB_REAL_PACKAGES[@]}"; then
              echo -e "${YELLOW}编译过程中出现错误${NC}"
              exit 1
            fi
          else
            echo -e "${BLUE}  模式: 核心包 deb(standard) + 编译描述与 arx_ros2_control（源码 HI）${NC}"
            ensure_arx_hi_external || exit 1
            local -a real_pkgs=("${BUILD_DEB_REAL_PACKAGES[@]}" arx_ros2_control)
            if ! run_colcon_packages_up_to "${real_pkgs[@]}"; then
              echo -e "${YELLOW}编译失败。检查 src/arx-ros2-control/external（见 init_repo / 包 README）。${NC}"
              exit 1
            fi
          fi
        else
          ensure_arx_hi_external || exit 1
          if ! run_colcon_packages_up_to "${BUILD_REAL_PACKAGES[@]}"; then
            echo -e "${YELLOW}编译失败。检查 src/arx-ros2-control/external（见 init_repo / 包 README）。${NC}"
            exit 1
          fi
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
    _load_launch_last
    if [ -n "${LAST_DESC}" ]; then
      echo -e "${BLUE}[INFO] 上次启动记录: config/launch_last.conf${NC}"
    fi
    launch_choice="$(launch_menu "${LAST_DESC}")"
    launch_action="$(_resolve_launch_menu_choice "${launch_choice}")"
    case "${launch_action}" in
      last)
        _do_last_launch
        ;;
      single)
        do_launch "单臂 X5" "ocs2_arm_controller demo.launch.py" "robot:=arx5" "" "1" "0"
        ;;
      dual)
        do_launch "双臂 ACone" "ocs2_arm_controller demo.launch.py" "robot:=arx_acone" "" "0" "0"
        ;;
      split)
        do_launch "Lift2S 分体控制" "ocs2_arm_controller split_body.launch.py" "robot:=arx_lift2s" "" "0" "1"
        ;;
      full)
        do_launch "Lift2S 全身控制" "ocs2_arm_controller full_body.launch.py" "robot:=arx_lift2s" "" "0" "1"
        ;;
      joystick)
        do_launch_joystick_teleop
        ;;
      back)
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
