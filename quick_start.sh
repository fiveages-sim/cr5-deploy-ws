#!/usr/bin/env bash
# Wuji Hand2 工作空间快速编译 / 启动（参考 fa_w2_ws/quick_start.sh，精简版）

set -u

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WUJI_WS="${WS_DIR}"
QS_CONFIG="${WS_DIR}/config/quick_start.conf"
HAND_LOCAL_CONF="${WS_DIR}/config/hand.local.conf"
QS_LAST_LAUNCH_FILE="${WS_DIR}/config/launch_last.conf"

print_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

_load_quick_start_config() {
  if [[ ! -f "${QS_CONFIG}" ]]; then
    print_error "找不到配置: ${QS_CONFIG}"
    return 1
  fi
  # shellcheck source=config/quick_start.conf
  source "${QS_CONFIG}"
}

_load_hand_local_conf() {
  DEVICE_ADDRESS_LEFT="${DEFAULT_DEVICE_ADDRESS_LEFT:-192.168.1.110:50001}"
  DEVICE_ADDRESS_RIGHT="${DEFAULT_DEVICE_ADDRESS_RIGHT:-192.168.1.111:50001}"
  SERIAL_NUMBER_LEFT=""
  SERIAL_NUMBER_RIGHT=""
  if [[ -f "${HAND_LOCAL_CONF}" ]]; then
    # shellcheck source=/dev/null
    source "${HAND_LOCAL_CONF}"
  fi
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    print_error "缺少命令: $1"
    return 1
  }
}

# ROS setup.bash 会引用可能未设置的 AMENT_* 变量；set -u 下 source 会报错
_source_setup_bash() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

ensure_ros_underlay_for_build() {
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    _source_setup_bash "/opt/ros/${ROS_DISTRO}/setup.bash"
  else
    local distro
    for distro in jazzy humble iron rolling; do
      if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
        _source_setup_bash "/opt/ros/${distro}/setup.bash"
        print_info "已加载 /opt/ros/${distro}/setup.bash"
        break
      fi
    done
  fi
  [[ -n "${ROS_DISTRO:-}" ]] || {
    print_error "未检测到 ROS_DISTRO，请先 source /opt/ros/jazzy/setup.bash"
    return 1
  }
  need_cmd ros2 || return 1
  need_cmd colcon || return 1
}

ensure_ros_env() {
  if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
    _source_setup_bash "${WS_DIR}/install/setup.bash"
    return 0
  fi
  print_warn "未找到 ${WS_DIR}/install/setup.bash"
  print_warn "请先编译: ./quick_start.sh → 1) 编译"
  return 1
}

_resolve_wuji_sdk_root() {
  if [[ -n "${WUJI_SDK_ROOT:-}" && -f "${WUJI_SDK_ROOT}/include/wuji_sdk.h" ]]; then
    printf '%s' "${WUJI_SDK_ROOT}"
    return 0
  fi
  local pattern path
  for pattern in "${WUJI_SDK_SEARCH_PATHS[@]}"; do
    [[ -z "${pattern}" ]] && continue
    # shellcheck disable=SC2086
    for path in ${pattern}; do
      if [[ -d "${path}" && -f "${path}/include/wuji_sdk.h" ]]; then
        printf '%s' "${path}"
        return 0
      fi
    done
  done
  return 1
}

ensure_wuji_sdk_env() {
  local sdk_root
  sdk_root="$(_resolve_wuji_sdk_root)" || {
    print_error "未找到 Wuji C SDK。请："
    print_info "  1) 从 https://docs.wuji.tech 下载 wuji-sdk-c tarball 并解压"
    print_info "  2) export WUJI_SDK_ROOT=/path/to/wuji-sdk-c-*-linux-gnu"
    return 1
  }
  export WUJI_SDK_ROOT="${sdk_root}"
  if [[ -d "${WUJI_SDK_ROOT}/lib" ]]; then
    export LD_LIBRARY_PATH="${WUJI_SDK_ROOT}/lib:${WUJI_SDK_ROOT}:${LD_LIBRARY_PATH:-}"
  else
    export LD_LIBRARY_PATH="${WUJI_SDK_ROOT}:${LD_LIBRARY_PATH:-}"
  fi
  print_info "WUJI_SDK_ROOT=${WUJI_SDK_ROOT}"
}

do_build() {
  local description="$1"
  shift
  local -a packages=("$@")
  local pkg_args="" pkg

  echo -e "${GREEN}开始${description}...${NC}"
  cd "${WS_DIR}" || exit 1
  ensure_ros_underlay_for_build || exit 1
  ensure_wuji_sdk_env || exit 1

  for pkg in "${packages[@]}"; do
    pkg_args+=" ${pkg}"
  done

  # WUJI_SDK_ROOT 仅 wujihand2_ros2_control 的 FindWujiSdk.cmake 会读（含 $ENV{WUJI_SDK_ROOT}）。
  # 勿全局 --cmake-args -DWUJI_SDK_ROOT=，否则其它包会报 "variable was not used" 警告。
  # shellcheck disable=SC2086
  colcon build --packages-up-to ${pkg_args} --symlink-install
  if [[ $? -eq 0 ]]; then
    print_info "编译完成。请执行: source install/setup.bash"
  else
    print_error "编译失败"
    exit 1
  fi
}

_save_launch_last() {
  mkdir -p "$(dirname "${QS_LAST_LAUNCH_FILE}")"
  {
    printf 'LAST_DESC=%q\n' "${1:-}"
    printf 'LAST_MODE=%q\n' "${2:-}"
    printf 'LAST_DIRECTION=%q\n' "${3:-}"
    printf 'LAST_DEVICE_ADDRESS=%q\n' "${4:-}"
    printf 'LAST_USE_RVIZ=%q\n' "${5:-}"
  } > "${QS_LAST_LAUNCH_FILE}"
}

_load_launch_last() {
  LAST_DESC="" LAST_MODE="" LAST_DIRECTION="" LAST_DEVICE_ADDRESS="" LAST_USE_RVIZ=""
  [[ -f "${QS_LAST_LAUNCH_FILE}" ]] && source "${QS_LAST_LAUNCH_FILE}" 2>/dev/null || true
}

launch_mode_menu() {
  echo "" >&2
  echo "请选择运行模式:" >&2
  echo "  1) 仿真 (mock_components + RViz)" >&2
  echo "  2) 仿真 headless (mock_components, 无 RViz)" >&2
  echo "  3) 真机 (hardware:=real + RViz)" >&2
  echo "  4) 真机 headless (hardware:=real, 无 RViz)" >&2
  echo "  0) 返回" >&2
  read -r -p "请输入选项 [0-4]: " choice
  echo "${choice:-0}"
}

hand_side_menu() {
  echo "" >&2
  echo "请选择手:" >&2
  echo "  1) 左手 (direction:=1)" >&2
  echo "  2) 右手 (direction:=-1)" >&2
  echo "  0) 返回" >&2
  read -r -p "请输入选项 [0-2] (默认: 1): " choice
  choice="${choice:-1}"
  case "${choice}" in
    1) echo "1" ;;
    2) echo "-1" ;;
    0) echo "0" ;;
    *) echo "invalid" ;;
  esac
}

prompt_device_address() {
  local side_label="$1"
  local default_addr="$2"
  local addr
  echo "" >&2
  echo -e "${BLUE}连接方式: 回车=SDK scan | d=默认 ${default_addr} | 或输入 IP:port${NC}" >&2
  read -r -p "${side_label} device_address: " addr
  case "${addr}" in
    d|D) addr="${default_addr}" ;;
  esac
  printf '%s' "${addr}"
}

do_launch_hand() {
  local mode_choice="$1"
  local direction="$2"
  local device_address="$3"
  local use_rviz="$4"
  local desc="$5"

  ensure_ros_env || exit 1
  ensure_wuji_sdk_env || {
    [[ "${mode_choice}" =~ ^[12]$ ]] || exit 1
    print_warn "仿真模式继续（无需 SDK 运行时库）"
  }

  local hardware="mock_components"
  [[ "${mode_choice}" =~ ^[34]$ ]] && hardware="real"

  local launch_file launch_args
  if [[ "${hardware}" == "real" ]]; then
    launch_file="${LAUNCH_FILE_REAL}"
    launch_args="${LAUNCH_BASE_ARGS_REAL} direction:=${direction} hardware:=real use_rviz:=${use_rviz}"
    if [[ -n "${device_address}" ]]; then
      launch_args+=" device_address:=${device_address}"
    fi
  else
    launch_file="${LAUNCH_FILE_MOCK}"
    launch_args="${LAUNCH_BASE_ARGS_MOCK} direction:=${direction} hardware:=${hardware} use_rviz:=${use_rviz}"
  fi

  _save_launch_last "${desc}" "${mode_choice}" "${direction}" "${device_address}" "${use_rviz}"

  print_info "启动: ros2 launch ${launch_file} ${launch_args}"
  # shellcheck disable=SC2086
  ros2 launch ${launch_file} ${launch_args}
}

do_launch_flow() {
  _load_hand_local_conf
  _load_launch_last

  local choice
  echo ""
  echo "请选择启动项:"
  if [[ -n "${LAST_DESC}" ]]; then
    echo "  1) 使用上次 — ${LAST_DESC}"
    echo "  2) 左手 Hand2"
    echo "  3) 右手 Hand2"
    read -r -p "请输入选项 [0-3]: " choice
    if [[ "${choice}" == "1" ]]; then
      do_launch_hand "${LAST_MODE}" "${LAST_DIRECTION}" "${LAST_DEVICE_ADDRESS}" "${LAST_USE_RVIZ}" "${LAST_DESC}"
      return
    fi
    [[ "${choice}" == "2" ]] && choice="left"
    [[ "${choice}" == "3" ]] && choice="right"
  else
    echo "  1) 左手 Hand2"
    echo "  2) 右手 Hand2"
    read -r -p "请输入选项 [0-2]: " choice
    [[ "${choice}" == "1" ]] && choice="left"
    [[ "${choice}" == "2" ]] && choice="right"
  fi

  case "${choice}" in
    left|right) ;;
    0|"") echo "返回"; return ;;
    *) print_warn "无效选项"; return ;;
  esac

  local direction default_addr side_label device_address mode_choice use_rviz desc
  if [[ "${choice}" == "left" ]]; then
    direction="1"
    default_addr="${DEVICE_ADDRESS_LEFT}"
    side_label="左手"
  else
    direction="-1"
    default_addr="${DEVICE_ADDRESS_RIGHT}"
    side_label="右手"
  fi

  mode_choice="$(launch_mode_menu)"
  [[ "${mode_choice}" == "0" ]] && { echo "返回"; return; }

  device_address=""
  use_rviz="true"
  case "${mode_choice}" in
    1) desc="${side_label} Hand2 仿真" ;;
    2) desc="${side_label} Hand2 仿真 headless"; use_rviz="false" ;;
    3)
      desc="${side_label} Hand2 真机"
      device_address="$(prompt_device_address "${side_label}" "${default_addr}")"
      ;;
    4)
      desc="${side_label} Hand2 真机 headless"
      use_rviz="false"
      device_address="$(prompt_device_address "${side_label}" "${default_addr}")"
      ;;
    *) print_warn "无效选项"; return ;;
  esac

  do_launch_hand "${mode_choice}" "${direction}" "${device_address}" "${use_rviz}" "${desc}"
}

extract_package_version_from_xml() {
  sed -n 's:.*<version>\(.*\)</version>.*:\1:p' <<<"$1" | head -n 1
}

print_package_versions_table() {
  local -n _pkgs=("$1")
  local pkg resolved xml ver

  ensure_ros_env || return 1
  echo ""
  echo -e "${BLUE}================ 包版本 ================${NC}"
  printf "%-40s | %-16s | %s\n" "Package" "Version" "Status"
  printf "%-40s-+-%-16s-+-%s\n" "----------------------------------------" "----------------" "----------"
  for pkg in "${_pkgs[@]}"; do
    xml="$(ros2 pkg xml "${pkg}" 2>/dev/null || true)"
    if [[ -n "${xml}" ]]; then
      ver="$(extract_package_version_from_xml "${xml}")"
      printf "%-40s | %-16s | %b\n" "${pkg}" "${ver:-N/A}" "${GREEN}FOUND${NC}"
    else
      printf "%-40s | %-16s | %b\n" "${pkg}" "-" "${RED}NOT FOUND${NC}"
    fi
  done
  echo -e "${BLUE}========================================${NC}"
}

menu() {
  echo -e "${BLUE}========================================${NC}" >&2
  echo -e "${BLUE}  Wuji Hand2 Workspace — quick_start${NC}" >&2
  echo -e "${BLUE}  ${WS_DIR}${NC}" >&2
  echo -e "${BLUE}========================================${NC}" >&2
  echo "" >&2
  echo "  1) 编译 (Build)" >&2
  echo "  2) 启动 Hand2 (Launch)" >&2
  echo "  3) 查询包版本" >&2
  echo "  0) 退出" >&2
  read -r -p "请输入选项 [0-3]: " choice
  echo "${choice}"
}

build_menu() {
  echo "" >&2
  echo "  1) 仿真包（mock，含 wujihand2_ros2_control）" >&2
  echo "  2) 真机包（同上，需 WUJI_SDK_ROOT）" >&2
  echo "  0) 返回" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

# --- main ---
need_cmd git || exit 1
_load_quick_start_config || exit 1

if [[ -f "${HAND_LOCAL_CONF}" ]]; then
  print_info "本机 Hand 配置: config/hand.local.conf"
else
  print_warn "未找到 config/hand.local.conf（真机 scan 可不建；固定 IP 可复制 hand.local.template.conf）"
fi
print_info "真机连接: 回车=SDK scan | d=默认 IP | 或输入 IP:port（详见 wujihand2_ros2_control/README.md）"

case "$(menu)" in
  1)
    case "$(build_menu)" in
      1) do_build "编译仿真包" "${BUILD_SIM_PACKAGES[@]}" ;;
      2) do_build "编译真机包" "${BUILD_REAL_PACKAGES[@]}" ;;
      0) echo "返回" ;;
      *) print_warn "无效选项"; exit 1 ;;
    esac
    ;;
  2)
    do_launch_flow
    ;;
  3)
    print_package_versions_table VERSION_QUERY_MAIN_PACKAGES
    ;;
  0)
    echo "退出"
    ;;
  *)
    print_warn "无效选项"
    exit 1
    ;;
esac
