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
BUILD_LIFT2S_SIM_PACKAGES=()
BUILD_LIFT2S_REAL_PACKAGES=()
BUILD_ALL_SIM_PACKAGES=()
BUILD_DEB_LIFT2S_PACKAGES=()
BUILD_DEB_ALL_PACKAGES=()
QS_CONFIG="${WS_DIR}/config/quick_start.conf"
if [[ -f "${QS_CONFIG}" ]]; then
  # shellcheck source=/dev/null
  source "${QS_CONFIG}"
fi

# conf 未定义时的回退包列表
if [[ ${#BUILD_LIFT2S_SIM_PACKAGES[@]} -eq 0 ]]; then
  BUILD_LIFT2S_SIM_PACKAGES=(
    ocs2_arm_controller
    ocs2_wbc_controller
    topic_based_ros2_control
    arx_acone_description
    arx_lift2s_description
    arms_teleop
    adaptive_gripper_controller
    basic_joint_controller
  )
fi
if [[ ${#BUILD_LIFT2S_REAL_PACKAGES[@]} -eq 0 ]]; then
  BUILD_LIFT2S_REAL_PACKAGES=(
    arx_ros2_control
    ocs2_arm_controller
    ocs2_wbc_controller
    arx_acone_description
    arx_lift2s_description
    arms_teleop
    adaptive_gripper_controller
    basic_joint_controller
  )
fi
if [[ ${#BUILD_ALL_SIM_PACKAGES[@]} -eq 0 ]]; then
  BUILD_ALL_SIM_PACKAGES=(
    ocs2_arm_controller
    ocs2_wbc_controller
    topic_based_ros2_control
    arx5_description
    arx_acone_description
    arx_lift_description
    arx_lift2s_description
    arx_x7s_description
    arms_teleop
    adaptive_gripper_controller
    basic_joint_controller
  )
fi
if [[ ${#BUILD_DEB_LIFT2S_PACKAGES[@]} -eq 0 ]]; then
  BUILD_DEB_LIFT2S_PACKAGES=(
    arx_acone_description
    arx_lift2s_description
  )
fi
if [[ ${#BUILD_DEB_ALL_PACKAGES[@]} -eq 0 ]]; then
  BUILD_DEB_ALL_PACKAGES=(
    arx5_description
    arx_acone_description
    arx_lift_description
    arx_lift2s_description
    arx_x7s_description
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

# Zenoh 同域下残留 robot_state_publisher / ros2_control 会让 RViz 显示成上一台机型
warn_stale_ros_nodes() {
  local stale
  stale="$(ps -eo args= 2>/dev/null | grep -E 'ros2_control_node|robot_state_publisher|rviz2' | grep -v grep || true)"
  if [ -n "${stale}" ]; then
    echo -e "${YELLOW}[WARN] 检测到仍在运行的控制/可视化进程（同 ROS_DOMAIN / Zenoh 下会串 /robot_description）：${NC}"
    echo "${stale}" | sed 's/^/  /' >&2
    echo -e "${YELLOW}      若 RViz 机型不对：先 Ctrl+C 停掉旧 launch，或 kill 上述进程后再启动${NC}"
  fi
}

_print_launch_cmd() {
  echo -e "${BLUE}[CMD] ros2 launch $*${NC}"
}

# Extra ros2 launch args that may contain spaces (each array element = one CLI token).
# Set by do_launch_with_body_mode for Lift split mounts; cleared at start of do_launch.
EXTRA_LAUNCH_ARGS=()

_clear_extra_launch_args() {
  EXTRA_LAUNCH_ARGS=()
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
  echo -e "${BLUE}  - 升降 hybrid：pos+vel + 重力/摩擦；soft_p：position + 常值重力（无摩擦）；底盘均 sendChassisOnly${NC}"
  echo -e "${BLUE}  - 升降热调：ros2 param set /controller_manager arx_lift.hybrid_kp 5.0${NC}"
  echo -e "${BLUE}  - 升降热调：ros2 param set /controller_manager arx_lift.hybrid_kd 2.0${NC}"
  echo -e "${BLUE}  - OCS2：分体 robot_name→分体规划包 / 全身本包 task.info${NC}"
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

# ==================== 最近启动记录（最多 3 条不同配置，MRU） ====================
QS_LAST_LAUNCH_FILE="${WS_DIR}/config/launch_last.conf"
QS_LAST_LAUNCH_MAX=3

# 指纹：launch + 参数 + 运行模式 + CAN/升降（用于去重）
_launch_fingerprint() {
  printf '%s\t%s\t%s\t%s\t%s' "${1:-}" "${2:-}" "${3:-}" "${4:-}" "${5:-}"
}

_clear_launch_history_vars() {
  local i
  unset LAST_COUNT LAST_DESC
  for i in 0 1 2; do
    unset "LAST_DESC_${i}" "LAST_LAUNCH_FILE_${i}" "LAST_BASE_ARGS_${i}" \
      "LAST_EXTRA_ARGS_${i}" \
      "LAST_MODE_CHOICE_${i}" "LAST_ASK_ARM_SIDE_${i}" "LAST_ASK_LIFT_MODE_${i}" \
      "LAST_CAN_ARG_${i}" "LAST_LIFT_MODE_ARG_${i}"
  done
  unset LAST_LAUNCH_FILE LAST_BASE_ARGS LAST_MODE_CHOICE \
    LAST_ASK_ARM_SIDE LAST_ASK_LIFT_MODE LAST_CAN_ARG LAST_LIFT_MODE_ARG
}

# Encode/decode EXTRA_LAUNCH_ARGS for launch_last.conf (RS = ASCII unit separator).
_encode_extra_launch_args() {
  local out="" a
  for a in "$@"; do
    out+="${a}"$'\x1f'
  done
  printf '%s' "${out}"
}

_decode_extra_launch_args_to_global() {
  local encoded="${1:-}"
  EXTRA_LAUNCH_ARGS=()
  if [ -z "${encoded}" ]; then
    return 0
  fi
  local IFS=$'\x1f'
  # shellcheck disable=SC2206
  EXTRA_LAUNCH_ARGS=(${encoded})
  # Drop empty trailing field from final RS
  if [ "${#EXTRA_LAUNCH_ARGS[@]}" -gt 0 ] && [ -z "${EXTRA_LAUNCH_ARGS[-1]}" ]; then
    unset 'EXTRA_LAUNCH_ARGS[-1]'
  fi
}

# Strip legacy broken Lift xyz from base_args into EXTRA_LAUNCH_ARGS.
_migrate_legacy_lift_xyz_base_args() {
  local base="${1:-}"
  if [[ ! "${base}" =~ xacro_left_xyz ]]; then
    printf '%s' "${base}"
    return 0
  fi
  # Always use canonical mounts; ignore mangled quoting in old history.
  EXTRA_LAUNCH_ARGS=(
    "xacro_left_xyz:=0.208 0.25000 0.092"
    "xacro_right_xyz:=0.208 -0.25000 0.092"
  )
  base="$(printf '%s' "${base}" | sed -E \
    -e "s/[[:space:]]*xacro_left_xyz:=('[^']*'|\"[^\"]*\"|[^[:space:]]+)//g" \
    -e "s/[[:space:]]*xacro_right_xyz:=('[^']*'|\"[^\"]*\"|[^[:space:]]+)//g")"
  # Collapse leftover spaces
  base="$(printf '%s' "${base}" | sed -E 's/[[:space:]]+/ /g; s/^ //; s/ $//')"
  printf '%s' "${base}"
}

_load_launch_last() {
  _clear_launch_history_vars
  LAST_COUNT=0
  LAST_DESC=""
  if [ ! -f "${QS_LAST_LAUNCH_FILE}" ]; then
    return 0
  fi
  # shellcheck disable=SC1090
  source "${QS_LAST_LAUNCH_FILE}" 2>/dev/null || true

  # 兼容旧版单条 LAST_*（无下标字段）
  if [ -n "${LAST_LAUNCH_FILE:-}" ] && [ -z "${LAST_LAUNCH_FILE_0:-}" ]; then
    LAST_COUNT=1
    LAST_DESC_0="${LAST_DESC:-}"
    LAST_LAUNCH_FILE_0="${LAST_LAUNCH_FILE}"
    LAST_BASE_ARGS_0="${LAST_BASE_ARGS:-}"
    LAST_MODE_CHOICE_0="${LAST_MODE_CHOICE:-}"
    LAST_ASK_ARM_SIDE_0="${LAST_ASK_ARM_SIDE:-0}"
    LAST_ASK_LIFT_MODE_0="${LAST_ASK_LIFT_MODE:-0}"
    LAST_CAN_ARG_0="${LAST_CAN_ARG:-}"
    LAST_LIFT_MODE_ARG_0="${LAST_LIFT_MODE_ARG:-}"
  fi

  if [ -z "${LAST_COUNT:-}" ] || ! [[ "${LAST_COUNT}" =~ ^[0-9]+$ ]]; then
    LAST_COUNT=0
  fi
  if [ "${LAST_COUNT}" -gt "${QS_LAST_LAUNCH_MAX}" ]; then
    LAST_COUNT="${QS_LAST_LAUNCH_MAX}"
  fi
  LAST_DESC="${LAST_DESC_0:-}"
}

_write_launch_history() {
  local f="${QS_LAST_LAUNCH_FILE}"
  local i n="${LAST_COUNT:-0}"
  local desc launch_file base_args mode_choice ask_arm ask_lift can_arg lift_arg extra_args
  mkdir -p "$(dirname "${f}")"
  {
    printf 'LAST_COUNT=%q\n' "${n}"
    for ((i = 0; i < n; i++)); do
      desc="LAST_DESC_${i}"
      launch_file="LAST_LAUNCH_FILE_${i}"
      base_args="LAST_BASE_ARGS_${i}"
      mode_choice="LAST_MODE_CHOICE_${i}"
      ask_arm="LAST_ASK_ARM_SIDE_${i}"
      ask_lift="LAST_ASK_LIFT_MODE_${i}"
      can_arg="LAST_CAN_ARG_${i}"
      lift_arg="LAST_LIFT_MODE_ARG_${i}"
      printf 'LAST_DESC_%d=%q\n' "${i}" "${!desc:-}"
      printf 'LAST_LAUNCH_FILE_%d=%q\n' "${i}" "${!launch_file:-}"
      printf 'LAST_BASE_ARGS_%d=%q\n' "${i}" "${!base_args:-}"
      extra_args="LAST_EXTRA_ARGS_${i}"
      printf 'LAST_EXTRA_ARGS_%d=%q\n' "${i}" "${!extra_args:-}"
      printf 'LAST_MODE_CHOICE_%d=%q\n' "${i}" "${!mode_choice:-}"
      printf 'LAST_ASK_ARM_SIDE_%d=%q\n' "${i}" "${!ask_arm:-0}"
      printf 'LAST_ASK_LIFT_MODE_%d=%q\n' "${i}" "${!ask_lift:-0}"
      printf 'LAST_CAN_ARG_%d=%q\n' "${i}" "${!can_arg:-}"
      printf 'LAST_LIFT_MODE_ARG_%d=%q\n' "${i}" "${!lift_arg:-}"
    done
  } > "${f}"
}

_save_launch_last() {
  # $1=desc $2=launch_file $3=base_args $4=mode_choice
  # $5=ask_arm_side $6=ask_lift_mode $7=can_arg $8=lift_motor_mode_arg
  # Uses global EXTRA_LAUNCH_ARGS (or caller-set) for spaced xacro args.
  local new_desc="${1:-}"
  local new_launch="${2:-}"
  local new_base="${3:-}"
  local new_mode="${4:-}"
  local new_ask_arm="${5:-0}"
  local new_ask_lift="${6:-0}"
  local new_can="${7:-}"
  local new_lift="${8:-}"
  local new_extra new_fp i j n
  local d l b e m aa al c lm
  local -a descs launches bases extras modes ask_arms ask_lifts cans lifts
  local -a keep_d keep_l keep_b keep_e keep_m keep_aa keep_al keep_c keep_lm

  new_extra="$(_encode_extra_launch_args "${EXTRA_LAUNCH_ARGS[@]}")"
  new_fp="$(_launch_fingerprint "${new_launch}" "${new_base}"$'\t'"${new_extra}" "${new_mode}" "${new_can}" "${new_lift}")"

  _load_launch_last
  n="${LAST_COUNT:-0}"
  for ((i = 0; i < n; i++)); do
    d="LAST_DESC_${i}"; l="LAST_LAUNCH_FILE_${i}"; b="LAST_BASE_ARGS_${i}"
    e="LAST_EXTRA_ARGS_${i}"
    m="LAST_MODE_CHOICE_${i}"; aa="LAST_ASK_ARM_SIDE_${i}"; al="LAST_ASK_LIFT_MODE_${i}"
    c="LAST_CAN_ARG_${i}"; lm="LAST_LIFT_MODE_ARG_${i}"
    descs+=("${!d:-}")
    launches+=("${!l:-}")
    bases+=("${!b:-}")
    extras+=("${!e:-}")
    modes+=("${!m:-}")
    ask_arms+=("${!aa:-0}")
    ask_lifts+=("${!al:-0}")
    cans+=("${!c:-}")
    lifts+=("${!lm:-}")
  done

  for ((i = 0; i < n; i++)); do
    if [ "$(_launch_fingerprint "${launches[$i]}" "${bases[$i]}"$'\t'"${extras[$i]}" "${modes[$i]}" "${cans[$i]}" "${lifts[$i]}")" = "${new_fp}" ]; then
      continue
    fi
    keep_d+=("${descs[$i]}")
    keep_l+=("${launches[$i]}")
    keep_b+=("${bases[$i]}")
    keep_e+=("${extras[$i]}")
    keep_m+=("${modes[$i]}")
    keep_aa+=("${ask_arms[$i]}")
    keep_al+=("${ask_lifts[$i]}")
    keep_c+=("${cans[$i]}")
    keep_lm+=("${lifts[$i]}")
  done

  _clear_launch_history_vars
  LAST_DESC_0="${new_desc}"
  LAST_LAUNCH_FILE_0="${new_launch}"
  LAST_BASE_ARGS_0="${new_base}"
  LAST_EXTRA_ARGS_0="${new_extra}"
  LAST_MODE_CHOICE_0="${new_mode}"
  LAST_ASK_ARM_SIDE_0="${new_ask_arm}"
  LAST_ASK_LIFT_MODE_0="${new_ask_lift}"
  LAST_CAN_ARG_0="${new_can}"
  LAST_LIFT_MODE_ARG_0="${new_lift}"
  LAST_COUNT=1

  n="${#keep_d[@]}"
  if [ "${n}" -gt $((QS_LAST_LAUNCH_MAX - 1)) ]; then
    n=$((QS_LAST_LAUNCH_MAX - 1))
  fi
  for ((j = 0; j < n; j++)); do
    i=$((j + 1))
    printf -v "LAST_DESC_${i}" '%s' "${keep_d[$j]}"
    printf -v "LAST_LAUNCH_FILE_${i}" '%s' "${keep_l[$j]}"
    printf -v "LAST_BASE_ARGS_${i}" '%s' "${keep_b[$j]}"
    printf -v "LAST_EXTRA_ARGS_${i}" '%s' "${keep_e[$j]}"
    printf -v "LAST_MODE_CHOICE_${i}" '%s' "${keep_m[$j]}"
    printf -v "LAST_ASK_ARM_SIDE_${i}" '%s' "${keep_aa[$j]}"
    printf -v "LAST_ASK_LIFT_MODE_${i}" '%s' "${keep_al[$j]}"
    printf -v "LAST_CAN_ARG_${i}" '%s' "${keep_c[$j]}"
    printf -v "LAST_LIFT_MODE_ARG_${i}" '%s' "${keep_lm[$j]}"
    LAST_COUNT=$((i + 1))
  done
  LAST_DESC="${LAST_DESC_0:-}"
  _write_launch_history
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

# $1=历史下标（0=最近）；完整复现，不再询问侧/升降/运行模式
_do_last_launch() {
  local idx="${1:-0}"
  local desc launch_file base_args mode_choice ask_arm ask_lift can_arg lift_arg
  local v_desc v_launch v_base v_extra v_mode v_ask_arm v_ask_lift v_can v_lift
  _load_launch_last
  if ! [[ "${idx}" =~ ^[0-9]+$ ]] || [ "${idx}" -ge "${LAST_COUNT:-0}" ]; then
    echo -e "${YELLOW}[WARN] 无有效启动记录 #$((idx + 1))（${QS_LAST_LAUNCH_FILE}）${NC}"
    return 1
  fi
  v_desc="LAST_DESC_${idx}"
  v_launch="LAST_LAUNCH_FILE_${idx}"
  v_base="LAST_BASE_ARGS_${idx}"
  v_extra="LAST_EXTRA_ARGS_${idx}"
  v_mode="LAST_MODE_CHOICE_${idx}"
  v_ask_arm="LAST_ASK_ARM_SIDE_${idx}"
  v_ask_lift="LAST_ASK_LIFT_MODE_${idx}"
  v_can="LAST_CAN_ARG_${idx}"
  v_lift="LAST_LIFT_MODE_ARG_${idx}"
  desc="${!v_desc:-}"
  launch_file="${!v_launch:-}"
  base_args="${!v_base:-}"
  mode_choice="${!v_mode:-}"
  ask_arm="${!v_ask_arm:-0}"
  ask_lift="${!v_ask_lift:-0}"
  can_arg="${!v_can:-}"
  lift_arg="${!v_lift:-}"
  if [ -z "${launch_file}" ] || [ -z "${mode_choice}" ]; then
    echo -e "${YELLOW}[WARN] 启动记录 #$((idx + 1)) 不完整（${QS_LAST_LAUNCH_FILE}）${NC}"
    return 1
  fi
  # Restore extras; migrate legacy Lift xyz embedded in base_args.
  _decode_extra_launch_args_to_global "${!v_extra:-}"
  if [ "${#EXTRA_LAUNCH_ARGS[@]}" -eq 0 ]; then
    base_args="$(_migrate_legacy_lift_xyz_base_args "${base_args}")"
  fi
  echo -e "${GREEN}使用最近启动 #$((idx + 1))：${desc}${NC}"
  do_launch "${desc}" "${launch_file}" "${base_args}" \
    "${mode_choice}" "${ask_arm}" "${ask_lift}" \
    "1" "${can_arg}" "${lift_arg}"
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
  ensure_ros_env || return 1
  echo -e "${BLUE}[INFO] 手柄遥操需机器人 stack 已在运行（另开终端启动真机/仿真）${NC}"
  if [ -z "${joy_dev}" ]; then
    echo -e "${GREEN}启动手柄遥操作...${NC}"
    ros2 launch arms_teleop joystick_teleop.launch.py
  else
    echo -e "${GREEN}启动手柄遥操作（${joy_dev}）...${NC}"
    ros2 launch arms_teleop joystick_teleop.launch.py "joy_dev:=${joy_dev}"
  fi
  return $?
}

do_launch_joystick_teleop() {
  local device_choice device_count device_id

  ensure_ros_env || return 1
  device_count="$(_joystick_count)"

  if [ "${device_count}" -eq 1 ]; then
    device_id="$(_joystick_first_id)"
    if [ "${device_id}" = "0" ]; then
      _run_joystick_teleop_launch ""
    else
      _run_joystick_teleop_launch "/dev/input/js${device_id}"
    fi
    return $?
  fi

  if [ "${device_count}" -eq 0 ]; then
    echo -e "${YELLOW}[WARN] 未枚举到手柄，使用默认 /dev/input/js0${NC}"
    _run_joystick_teleop_launch ""
    return $?
  fi

  device_choice="$(joystick_device_menu)"
  case "${device_choice}" in
    back)
      echo "返回"
      return 2
      ;;
    0)
      _run_joystick_teleop_launch ""
      return $?
      ;;
    *)
      if ! [[ "${device_choice}" =~ ^[0-9]+$ ]]; then
        echo -e "${YELLOW}无效 Joy ID: ${device_choice}${NC}"
        return 1
      fi
      _run_joystick_teleop_launch "/dev/input/js${device_choice}"
      return $?
      ;;
  esac
}

# $1=描述 $2=launch 包+文件 $3=基础参数
# $4=预选运行模式（空则询问）
# $5=1 时：单臂真机询问左/右 CAN
# $6=1 时：真机询问升降 lift_motor_mode
# $7=1 时：复现模式，跳过侧/升降询问，使用 $8/$9
# $8=preset can_arg  $9=preset lift_motor_mode_arg
# $10=1 时运行模式菜单含真机（仅 X5 / ACone-X5 / Lift2S）
# 返回码：0=完成/启动结束；2=用户选 0 返回上一级；1=无效/失败
do_launch() {
  local description="$1"
  local launch_file="$2"
  local base_args="$3"
  local preset_mode="${4:-}"
  local ask_arm_side="${5:-0}"
  local ask_lift_mode="${6:-0}"
  local replay="${7:-0}"
  local can_arg="${8:-}"
  local lift_motor_mode_arg="${9:-}"
  local allow_real="${10:-1}"
  local mode_choice=""
  local mode_label=""
  local save_desc=""
  local side_choice can_if lm_choice lift_motor_mode
  # Snapshot extras for this launch (caller may set EXTRA_LAUNCH_ARGS before invoke).
  local -a extra_args=("${EXTRA_LAUNCH_ARGS[@]}")

  while true; do
    mode_choice="${preset_mode}"
    mode_label=""
    if [ -z "${mode_choice}" ]; then
      mode_choice="$(launch_mode_menu "${allow_real}")"
    fi

    case "${mode_choice}" in
      0)
        echo "返回"
        return 2
        ;;
      1|2)
        if [ "${allow_real}" != "1" ] && [ "${replay}" != "1" ]; then
          echo -e "${YELLOW}该机型不支持真机启动${NC}"
          if [ -n "${preset_mode}" ]; then
            return 1
          fi
          continue
        fi
        ;;
      [3-7]) ;;
      *)
        echo -e "${YELLOW}无效选项${NC}"
        if [ -n "${preset_mode}" ]; then
          return 1
        fi
        continue
        ;;
    esac

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
          side_choice="$(arm_side_menu)"
          can_if="$(resolve_arm_can "${side_choice}")"
          if [ "${can_if}" = "INVALID" ]; then
            echo -e "${YELLOW}无效选项${NC}"
            if [ -n "${preset_mode}" ]; then
              return 1
            fi
            continue
          fi
          if [ -z "${can_if}" ]; then
            echo "返回"
            # 交互选模式时回到运行模式菜单；预选模式则交给上一级
            if [ -z "${preset_mode}" ]; then
              continue
            fi
            return 2
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
          lm_choice="$(lift_motor_mode_menu)"
          lift_motor_mode="$(resolve_lift_motor_mode "${lm_choice}")"
          if [ "${lift_motor_mode}" = "INVALID" ]; then
            echo -e "${YELLOW}无效选项${NC}"
            if [ -n "${preset_mode}" ]; then
              return 1
            fi
            continue
          fi
          if [ -z "${lift_motor_mode}" ]; then
            echo "返回"
            if [ -z "${preset_mode}" ]; then
              continue
            fi
            return 2
          fi
          lift_motor_mode_arg="xacro_lift_motor_mode:=${lift_motor_mode}"
          mode_label="${mode_label}，升降=${lift_motor_mode}"
        fi
      fi
    fi

    break
  done

  case "${mode_choice}" in
    [1-7])
      save_desc="${description}"
      if [[ "${description}" != *"+"* ]]; then
        save_desc="${description} + $(_mode_choice_label "${mode_choice}")${mode_label}"
      fi
      EXTRA_LAUNCH_ARGS=("${extra_args[@]}")
      _save_launch_last "${save_desc}" "${launch_file}" "${base_args}" \
        "${mode_choice}" "${ask_arm_side}" "${ask_lift_mode}" \
        "${can_arg}" "${lift_motor_mode_arg}"
      ;;
  esac

  case "${mode_choice}" in
    1)
      echo -e "${GREEN}启动${description}（真机${mode_label}）...${NC}"
      ensure_ros_env || return 1
      ensure_zenoh_router || return 1
      warn_stale_ros_nodes
      print_can_hint
      # shellcheck disable=SC2086
      _print_launch_cmd ${launch_file} ${base_args} "${extra_args[@]}" hardware:=real ${can_arg} ${lift_motor_mode_arg}
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} "${extra_args[@]}" hardware:=real ${can_arg} ${lift_motor_mode_arg}
      return $?
      ;;
    2)
      echo -e "${GREEN}启动${description}（真机 headless${mode_label}）...${NC}"
      ensure_ros_env || return 1
      ensure_zenoh_router || return 1
      warn_stale_ros_nodes
      print_can_hint
      # shellcheck disable=SC2086
      _print_launch_cmd ${launch_file} ${base_args} "${extra_args[@]}" hardware:=real launch_mode:=control_only ${can_arg} ${lift_motor_mode_arg}
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} "${extra_args[@]}" hardware:=real launch_mode:=control_only ${can_arg} ${lift_motor_mode_arg}
      return $?
      ;;
    3)
      echo -e "${GREEN}启动${description}（仿真）...${NC}"
      ensure_ros_env || return 1
      ensure_zenoh_router || return 1
      warn_stale_ros_nodes
      # shellcheck disable=SC2086
      _print_launch_cmd ${launch_file} ${base_args} "${extra_args[@]}" hardware:=mock_components
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} "${extra_args[@]}" hardware:=mock_components
      return $?
      ;;
    4)
      echo -e "${GREEN}启动${description}（仿真 headless）...${NC}"
      ensure_ros_env || return 1
      ensure_zenoh_router || return 1
      warn_stale_ros_nodes
      # shellcheck disable=SC2086
      _print_launch_cmd ${launch_file} ${base_args} "${extra_args[@]}" hardware:=mock_components launch_mode:=control_only
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} "${extra_args[@]}" hardware:=mock_components launch_mode:=control_only
      return $?
      ;;
    5)
      echo -e "${GREEN}启动${description}（Isaac）...${NC}"
      ensure_ros_env || return 1
      ensure_zenoh_router || return 1
      warn_stale_ros_nodes
      # shellcheck disable=SC2086
      _print_launch_cmd ${launch_file} ${base_args} "${extra_args[@]}" hardware:=isaac
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} "${extra_args[@]}" hardware:=isaac
      return $?
      ;;
    6)
      echo -e "${GREEN}启动${description}（Isaac headless）...${NC}"
      ensure_ros_env || return 1
      ensure_zenoh_router || return 1
      warn_stale_ros_nodes
      # shellcheck disable=SC2086
      _print_launch_cmd ${launch_file} ${base_args} "${extra_args[@]}" hardware:=isaac launch_mode:=control_only
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} "${extra_args[@]}" hardware:=isaac launch_mode:=control_only
      return $?
      ;;
    7)
      echo -e "${GREEN}启动${description}（仅可视化）...${NC}"
      ensure_ros_env || return 1
      ensure_zenoh_router || return 1
      warn_stale_ros_nodes
      # shellcheck disable=SC2086
      _print_launch_cmd ${launch_file} ${base_args} "${extra_args[@]}" launch_mode:=rviz_only
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} "${extra_args[@]}" launch_mode:=rviz_only
      return $?
      ;;
  esac
  return 0
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
  echo "  1) 编译 Lift2S 真机 (arx_ros2_control + acone/lift2s)" >&2
  echo "  2) 编译 Lift2S 仿真 (acone/lift2s)" >&2
  echo "  3) 编译所有仿真 (x5/r5/acone/lift/lift2s/x7s)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3]: " choice
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
  local base=1 max_opt
  local i n="${LAST_COUNT:-0}" desc v_desc

  echo "" >&2
  echo "请选择启动项:" >&2
  if [ "${n}" -gt 0 ]; then
    for ((i = 0; i < n; i++)); do
      v_desc="LAST_DESC_${i}"
      desc="${!v_desc:-}"
      echo "  $((i + 1))) 使用最近 — ${desc}" >&2
    done
    base=$((n + 1))
    echo "" >&2
    echo -e "  ${BLUE}━━━━━━━━━━━━━━━━ 机型选项 ━━━━━━━━━━━━━━━━${NC}" >&2
    echo "" >&2
  fi
  echo "  ${base}) 单臂 (X5 / R5)" >&2
  echo "  $((base + 1))) 双臂 (ACone)" >&2
  echo "  $((base + 2))) 整机 (Lift / Lift2S / X7S)" >&2
  echo "" >&2
  echo -e "  ${BLUE}━━━━━━━━━━━━━━━━ 辅助功能 ━━━━━━━━━━━━━━━━${NC}" >&2
  echo "" >&2
  echo "  $((base + 3))) 手柄遥操作 (Joystick Teleop)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  max_opt=$((base + 3))
  read -r -p "请输入选项 [0-${max_opt}]: " choice
  echo "${choice}"
}

# 解析 launch_menu → last:N | single | dual | full | joystick | back | invalid
_resolve_launch_menu_choice() {
  local choice="$1"
  local n="${LAST_COUNT:-0}"
  local offset="${n}"

  if [ "${choice}" = "0" ]; then
    echo "back"
    return 0
  fi
  if ! [[ "${choice}" =~ ^[0-9]+$ ]]; then
    echo "invalid"
    return 0
  fi
  if [ "${n}" -gt 0 ] && [ "${choice}" -ge 1 ] && [ "${choice}" -le "${n}" ]; then
    echo "last:$((choice - 1))"
    return 0
  fi

  local idx=$((choice - offset))
  case "${idx}" in
    1) echo "single" ;;
    2) echo "dual" ;;
    3) echo "full" ;;
    4) echo "joystick" ;;
    *) echo "invalid" ;;
  esac
}

single_arm_menu() {
  echo "" >&2
  echo "请选择单臂机型:" >&2
  echo "  1) X5" >&2
  echo "  2) R5" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

dual_arm_menu() {
  echo "" >&2
  echo "请选择双臂 ACone 臂型:" >&2
  echo "  1) X5" >&2
  echo "  2) R5" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

full_robot_menu() {
  echo "" >&2
  echo "请选择整机机型:" >&2
  echo "  1) Lift" >&2
  echo "  2) Lift2S" >&2
  echo "  3) X7S" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3]: " choice
  echo "${choice}"
}

# 单臂：X5 支持真机；R5 仅仿真/Isaac/可视化
do_launch_single_arm() {
  local arm_choice rc
  EXTRA_LAUNCH_ARGS=()
  while true; do
    arm_choice="$(single_arm_menu)"
    case "${arm_choice}" in
      1)
        do_launch "单臂 X5" "ocs2_arm_controller demo.launch.py" "robot:=arx5" \
          "" "1" "0" "0" "" "" "1"
        rc=$?
        [ "${rc}" -eq 2 ] && continue
        return "${rc}"
        ;;
      2)
        do_launch "单臂 R5" "ocs2_arm_controller demo.launch.py" "robot:=arx5 type:=r5" \
          "" "1" "0" "0" "" "" "0"
        rc=$?
        [ "${rc}" -eq 2 ] && continue
        return "${rc}"
        ;;
      0)
        echo "返回"
        return 2
        ;;
      *)
        echo -e "${YELLOW}无效选项${NC}"
        ;;
    esac
  done
}

# 双臂 ACone：X5 支持真机；R5 仅仿真/Isaac/可视化
do_launch_dual_arm() {
  local arm_choice rc
  EXTRA_LAUNCH_ARGS=()
  while true; do
    arm_choice="$(dual_arm_menu)"
    case "${arm_choice}" in
      1)
        do_launch "双臂 ACone X5" "ocs2_arm_controller demo.launch.py" "robot:=arx_acone" \
          "" "0" "0" "0" "" "" "1"
        rc=$?
        [ "${rc}" -eq 2 ] && continue
        return "${rc}"
        ;;
      2)
        do_launch "双臂 ACone R5" "ocs2_arm_controller demo.launch.py" "robot:=arx_acone type:=r5" \
          "" "0" "0" "0" "" "" "0"
        rc=$?
        [ "${rc}" -eq 2 ] && continue
        return "${rc}"
        ;;
      0)
        echo "返回"
        return 2
        ;;
      *)
        echo -e "${YELLOW}无效选项${NC}"
        ;;
    esac
  done
}

# 整机：Lift2S 支持真机；Lift / X7S 仅仿真/Isaac/可视化
do_launch_full_robot() {
  local robot_choice rc
  EXTRA_LAUNCH_ARGS=()
  while true; do
    robot_choice="$(full_robot_menu)"
    case "${robot_choice}" in
      1)
        do_launch_with_body_mode "Lift" "robot:=arx_lift" "0"
        rc=$?
        [ "${rc}" -eq 2 ] && continue
        return "${rc}"
        ;;
      2)
        do_launch_with_body_mode "Lift2S" "robot:=arx_lift2s" "1"
        rc=$?
        [ "${rc}" -eq 2 ] && continue
        return "${rc}"
        ;;
      3)
        do_launch_with_body_mode "X7S" "robot:=arx_x7s" "0"
        rc=$?
        [ "${rc}" -eq 2 ] && continue
        return "${rc}"
        ;;
      0)
        echo "返回"
        return 2
        ;;
      *)
        echo -e "${YELLOW}无效选项${NC}"
        ;;
    esac
  done
}

# 分体 / 全身（lift / lift2s / x7s）
body_control_mode_menu() {
  local robot_label="${1:-}"
  echo "" >&2
  if [ -n "${robot_label}" ]; then
    echo "请选择 ${robot_label} 控制方式:" >&2
  else
    echo "请选择控制方式:" >&2
  fi
  echo "  1) 分体控制 (Split Body)" >&2
  echo "  2) 全身控制 (Full Body)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

# $1=机型短名(Lift|Lift2S|X7S) $2=base_args $3=是否允许真机(1/0)
# 询问分体/全身后 do_launch；选 0 返回上一级整机菜单
do_launch_with_body_mode() {
  local robot_label="$1"
  local base_args="$2"
  local allow_real="${3:-0}"
  local body_choice launch_file mode_label rc

  while true; do
    body_choice="$(body_control_mode_menu "${robot_label}")"
    case "${body_choice}" in
      1)
        launch_file="ocs2_arm_controller split_body.launch.py"
        mode_label="分体控制"
        EXTRA_LAUNCH_ARGS=()
        # Lift 分体规划用 arx_acone，须覆盖为经典 Lift 臂座（≠ acone 默认）。
        # 带空格的 xyz 必须用数组元素传入，不能塞进未加引号展开的 base_args。
        if [[ "${base_args}" =~ (^|[[:space:]])robot:=arx_lift([[:space:]]|$) ]]; then
          EXTRA_LAUNCH_ARGS=(
            "xacro_left_xyz:=0.208 0.25000 0.092"
            "xacro_right_xyz:=0.208 -0.25000 0.092"
          )
        fi
        # X7S 分体规划：同包 robot.xacro topology:=dual（根 body）；硬件仍 full。
        if [[ "${base_args}" =~ (^|[[:space:]])robot:=arx_x7s([[:space:]]|$) ]]; then
          base_args="${base_args} xacro_topology:=dual"
        fi
        ;;
      2)
        launch_file="ocs2_arm_controller full_body.launch.py"
        mode_label="全身控制"
        EXTRA_LAUNCH_ARGS=()
        # X7S 全身规划须 full（默认即是）；显式写出以免残留 dual。
        if [[ "${base_args}" =~ (^|[[:space:]])robot:=arx_x7s([[:space:]]|$) ]]; then
          base_args="${base_args} xacro_topology:=full"
        fi
        ;;
      0)
        echo "返回"
        return 2
        ;;
      *)
        echo -e "${YELLOW}无效选项${NC}"
        continue
        ;;
    esac
    do_launch "${robot_label} ${mode_label}" "${launch_file}" "${base_args}" \
      "" "0" "1" "0" "" "" "${allow_real}"
    rc=$?
    # 运行模式/子选项返回 → 回到分体/全身；真正启动结束后回到整机菜单
    if [ "${rc}" -eq 2 ]; then
      continue
    fi
    return "${rc}"
  done
}

# $1=1 时显示真机选项（内部编号 1/2）；否则仅仿真类（菜单 1-5 → 内部 3-7）
launch_mode_menu() {
  local allow_real="${1:-1}"
  local choice
  echo "" >&2
  echo "请选择运行模式:" >&2
  if [ "${allow_real}" = "1" ]; then
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
    return 0
  fi

  echo "  1) 仿真启动 (Simulation / mock_components)" >&2
  echo "  2) 仿真 headless 模式启动 (mock_components, No RViz)" >&2
  echo "  3) Isaac 仿真启动 (hardware:=isaac)" >&2
  echo "  4) Isaac 仿真 headless 启动 (hardware:=isaac, control_only)" >&2
  echo "  5) 仅可视化模式启动 (Visualization Only / rviz_only)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-5]: " choice
  case "${choice}" in
    0) echo "0" ;;
    1) echo "3" ;;
    2) echo "4" ;;
    3) echo "5" ;;
    4) echo "6" ;;
    5) echo "7" ;;
    *) echo "${choice}" ;;
  esac
}

need_cmd git || exit 1
need_cmd colcon || echo -e "${YELLOW}[WARN] 未找到 colcon，编译选项会失败。${NC}"

# 主菜单循环：子菜单选 0 返回上一级，仅主菜单 0 退出
while true; do
  top_choice="$(menu)"
  case "${top_choice}" in
    1)
      while true; do
        build_choice="$(build_menu)"
        case "${build_choice}" in
          1)
            echo -e "${GREEN}开始编译 Lift2S 真机所需包...${NC}"
            if core_deb_mode; then
              warn_hi_overlay_conflict
              if arx_from_deb; then
                echo -e "${BLUE}  模式: 核心包 deb(arms-full 含 arx_ros2_control) + 仅编译 Lift2S 描述包${NC}"
                if ! run_colcon_packages_up_to "${BUILD_DEB_LIFT2S_PACKAGES[@]}"; then
                  echo -e "${YELLOW}编译过程中出现错误${NC}"
                  continue
                fi
              else
                echo -e "${BLUE}  模式: 核心包 deb(standard) + 编译 Lift2S 描述与 arx_ros2_control（源码 HI）${NC}"
                if ! ensure_arx_hi_external; then
                  continue
                fi
                real_pkgs=("${BUILD_DEB_LIFT2S_PACKAGES[@]}" arx_ros2_control)
                if ! run_colcon_packages_up_to "${real_pkgs[@]}"; then
                  echo -e "${YELLOW}编译失败。检查 src/arx-ros2-control/external（见 init_repo / 包 README）。${NC}"
                  continue
                fi
              fi
            else
              if ! ensure_arx_hi_external; then
                continue
              fi
              if ! run_colcon_packages_up_to "${BUILD_LIFT2S_REAL_PACKAGES[@]}"; then
                echo -e "${YELLOW}编译失败。检查 src/arx-ros2-control/external（见 init_repo / 包 README）。${NC}"
                continue
              fi
            fi
            echo -e "${GREEN}编译完成！${NC}"
            echo -e "${BLUE}启动：单臂可选左/右 CAN；臂仅 full_control；升降可选 hybrid/soft_p${NC}"
            print_can_hint
            ;;
          2)
            echo -e "${GREEN}开始编译 Lift2S 仿真所需包...${NC}"
            if core_deb_mode; then
              warn_hi_overlay_conflict
              echo -e "${BLUE}  模式: 核心包 deb + 仅编译 Lift2S 描述包${NC}"
              if ! run_colcon_packages_up_to "${BUILD_DEB_LIFT2S_PACKAGES[@]}"; then
                echo -e "${YELLOW}编译过程中出现错误${NC}"
                continue
              fi
            else
              if ! run_colcon_packages_up_to "${BUILD_LIFT2S_SIM_PACKAGES[@]}"; then
                echo -e "${YELLOW}编译过程中出现错误${NC}"
                continue
              fi
            fi
            echo -e "${GREEN}编译完成！${NC}"
            ;;
          3)
            echo -e "${GREEN}开始编译所有仿真所需包...${NC}"
            if core_deb_mode; then
              warn_hi_overlay_conflict
              echo -e "${BLUE}  模式: 核心包 deb + 仅编译全部 ARX 描述包${NC}"
              if ! run_colcon_packages_up_to "${BUILD_DEB_ALL_PACKAGES[@]}"; then
                echo -e "${YELLOW}编译过程中出现错误${NC}"
                continue
              fi
            else
              if ! run_colcon_packages_up_to "${BUILD_ALL_SIM_PACKAGES[@]}"; then
                echo -e "${YELLOW}编译过程中出现错误${NC}"
                continue
              fi
            fi
            echo -e "${GREEN}编译完成！${NC}"
            ;;
          0)
            echo "返回"
            break
            ;;
          *)
            echo -e "${YELLOW}无效选项${NC}"
            ;;
        esac
      done
      ;;

    2)
      while true; do
        launch_rc=0
        _load_launch_last
        if [ "${LAST_COUNT:-0}" -gt 0 ]; then
          echo -e "${BLUE}[INFO] 最近启动记录（最多 ${QS_LAST_LAUNCH_MAX} 条）: config/launch_last.conf${NC}"
        fi
        launch_choice="$(launch_menu)"
        launch_action="$(_resolve_launch_menu_choice "${launch_choice}")"
        case "${launch_action}" in
          last:*)
            _do_last_launch "${launch_action#last:}"
            launch_rc=$?
            ;;
          single)
            do_launch_single_arm
            launch_rc=$?
            ;;
          dual)
            do_launch_dual_arm
            launch_rc=$?
            ;;
          full)
            do_launch_full_robot
            launch_rc=$?
            ;;
          joystick)
            do_launch_joystick_teleop
            launch_rc=$?
            ;;
          back)
            echo "返回"
            break
            ;;
          *)
            echo -e "${YELLOW}无效选项${NC}"
            continue
            ;;
        esac
        # 与 fa_w2 一致：真正启动（或启动失败）后结束脚本；仅「返回」(2) 留在启动菜单
        if [ "${launch_rc}" -eq 2 ]; then
          continue
        fi
        exit "${launch_rc}"
      done
      ;;

    0)
      echo "退出"
      exit 0
      ;;

    *)
      echo -e "${YELLOW}无效选项${NC}"
      ;;
  esac
done
