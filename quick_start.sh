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

print_info() { echo -e "${GREEN}[INFO]${NC} $1" >&2; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $1" >&2; }
print_error() { echo -e "${RED}[ERROR]${NC} $1" >&2; }

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

_wuji_host_arch() {
  case "$(uname -m)" in
    aarch64|arm64) printf 'aarch64' ;;
    *) printf 'x86_64' ;;
  esac
}

# Resolve SDK tree with include/wuji_sdk.h.
# Order: explicit WUJI_SDK_ROOT → vendored external/ → WUJI_SDK_SEARCH_PATHS.
_resolve_wuji_sdk_root() {
  local vendor="${WS_DIR}/src/wujihand2-ros2-control/external/wuji-sdk-c"
  local arch pattern path
  arch="$(_wuji_host_arch)"
  if [[ -n "${WUJI_SDK_ROOT:-}" && -f "${WUJI_SDK_ROOT}/include/wuji_sdk.h" ]]; then
    printf '%s' "${WUJI_SDK_ROOT}"
    return 0
  fi
  if [[ -f "${vendor}/include/wuji_sdk.h" && -f "${vendor}/lib/${arch}/libwuji_sdk_c.so" ]]; then
    printf '%s' "${vendor}"
    return 0
  fi
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

# Lib dir: vendored uses lib/<arch>/; classic tarball uses lib/.
_wuji_sdk_lib_dir() {
  local sdk_root="${1:-}"
  local arch
  arch="$(_wuji_host_arch)"
  if [[ -f "${sdk_root}/lib/${arch}/libwuji_sdk_c.so" ]]; then
    printf '%s' "${sdk_root}/lib/${arch}"
  elif [[ -f "${sdk_root}/lib/libwuji_sdk_c.so" ]]; then
    printf '%s' "${sdk_root}/lib"
  elif [[ -f "${sdk_root}/libwuji_sdk_c.so" ]]; then
    printf '%s' "${sdk_root}"
  else
    return 1
  fi
}

ensure_wuji_sdk_env() {
  local sdk_root lib_dir
  sdk_root="$(_resolve_wuji_sdk_root)" || {
    print_error "未找到 Wuji C SDK（期望仓内 external/wuji-sdk-c）。"
    print_info "  见 src/wujihand2-ros2-control/external/wuji-sdk-c/README.md"
    print_info "  调试覆盖: export WUJI_SDK_ROOT=/path/to/wuji-sdk-c-*-linux-gnu"
    return 1
  }
  lib_dir="$(_wuji_sdk_lib_dir "${sdk_root}")" || {
    print_error "SDK 目录缺少 libwuji_sdk_c.so: ${sdk_root}"
    return 1
  }
  export WUJI_SDK_ROOT="${sdk_root}"
  export WUJI_SDK_LIB_DIR="${lib_dir}"
  export LD_LIBRARY_PATH="${lib_dir}:${LD_LIBRARY_PATH:-}"
  # Also prepend install lib when present (RPATH usually enough for HI).
  if [[ -d "${WS_DIR}/install/wujihand2_ros2_control/lib" ]]; then
    export LD_LIBRARY_PATH="${WS_DIR}/install/wujihand2_ros2_control/lib:${LD_LIBRARY_PATH}"
  fi
  print_info "Wuji SDK: ${sdk_root} (lib=${lib_dir})"
}

do_build() {
  local description="$1"
  shift
  local -a packages=("$@")
  local pkg_args="" pkg

  echo -e "${GREEN}开始${description}...${NC}"
  cd "${WS_DIR}" || exit 1
  ensure_ros_underlay_for_build || exit 1
  # Verify vendored (or override) SDK exists; CMake uses external/ by default.
  ensure_wuji_sdk_env || exit 1

  for pkg in "${packages[@]}"; do
    pkg_args+=" ${pkg}"
  done

  # 默认链仓内 external/；调试覆盖用:
  #   colcon build --packages-select wujihand2_ros2_control --cmake-args -DWUJI_SDK_ROOT=/path/to/tarball
  # shellcheck disable=SC2086
  colcon build --packages-up-to ${pkg_args} --symlink-install
  if [[ $? -eq 0 ]]; then
    print_info "编译完成。请执行: source install/setup.bash"
  else
    print_error "编译失败"
    exit 1
  fi
}

# Sentinel from prompt_device_address when user selects 0) 返回
QS_CONN_BACK="__BACK__"

_save_launch_last() {
  local desc="${1:-}" mode="${2:-}" direction="${3:-}" device_address="${4:-}" use_rviz="${5:-}"
  # Preserve LAST_REAL_* across sim launches; refresh only on real (mode 3/4).
  local real_desc="${LAST_REAL_DESC:-}"
  local real_mode="${LAST_REAL_MODE:-}"
  local real_direction="${LAST_REAL_DIRECTION:-}"
  local real_device_address="${LAST_REAL_DEVICE_ADDRESS:-}"
  local real_use_rviz="${LAST_REAL_USE_RVIZ:-}"
  if [[ "${mode}" =~ ^[34]$ ]]; then
    real_desc="${desc}"
    real_mode="${mode}"
    real_direction="${direction}"
    real_device_address="${device_address}"
    real_use_rviz="${use_rviz}"
    LAST_REAL_DESC="${real_desc}"
    LAST_REAL_MODE="${real_mode}"
    LAST_REAL_DIRECTION="${real_direction}"
    LAST_REAL_DEVICE_ADDRESS="${real_device_address}"
    LAST_REAL_USE_RVIZ="${real_use_rviz}"
  fi

  mkdir -p "$(dirname "${QS_LAST_LAUNCH_FILE}")"
  {
    printf 'LAST_DESC=%q\n' "${desc}"
    printf 'LAST_MODE=%q\n' "${mode}"
    printf 'LAST_DIRECTION=%q\n' "${direction}"
    printf 'LAST_DEVICE_ADDRESS=%q\n' "${device_address}"
    printf 'LAST_USE_RVIZ=%q\n' "${use_rviz}"
    printf 'LAST_REAL_DESC=%q\n' "${real_desc}"
    printf 'LAST_REAL_MODE=%q\n' "${real_mode}"
    printf 'LAST_REAL_DIRECTION=%q\n' "${real_direction}"
    printf 'LAST_REAL_DEVICE_ADDRESS=%q\n' "${real_device_address}"
    printf 'LAST_REAL_USE_RVIZ=%q\n' "${real_use_rviz}"
  } > "${QS_LAST_LAUNCH_FILE}"
}

_load_launch_last() {
  LAST_DESC="" LAST_MODE="" LAST_DIRECTION="" LAST_DEVICE_ADDRESS="" LAST_USE_RVIZ=""
  LAST_REAL_DESC="" LAST_REAL_MODE="" LAST_REAL_DIRECTION=""
  LAST_REAL_DEVICE_ADDRESS="" LAST_REAL_USE_RVIZ=""
  [[ -f "${QS_LAST_LAUNCH_FILE}" ]] && source "${QS_LAST_LAUNCH_FILE}" 2>/dev/null || true
  # Migrate older launch_last.conf that only had LAST_* from a real run.
  if [[ -z "${LAST_REAL_MODE:-}" && "${LAST_MODE:-}" =~ ^[34]$ ]]; then
    LAST_REAL_DESC="${LAST_DESC}"
    LAST_REAL_MODE="${LAST_MODE}"
    LAST_REAL_DIRECTION="${LAST_DIRECTION}"
    LAST_REAL_DEVICE_ADDRESS="${LAST_DEVICE_ADDRESS}"
    LAST_REAL_USE_RVIZ="${LAST_USE_RVIZ}"
  fi
}

_format_real_last_label() {
  local side="左手"
  [[ "${LAST_REAL_DIRECTION:-}" == "-1" ]] && side="右手"
  local addr_part
  if [[ -n "${LAST_REAL_DEVICE_ADDRESS:-}" ]]; then
    addr_part="@ ${LAST_REAL_DEVICE_ADDRESS}"
  else
    addr_part="(启动时 scan)"
  fi
  local rviz=""
  [[ "${LAST_REAL_USE_RVIZ:-}" == "true" ]] && rviz=" +RViz"
  printf '%s %s%s' "${side}" "${addr_part}" "${rviz}"
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

# Compile/cache scripts/wuji_scan_devices.c → .cache/wuji_scan_devices
_ensure_wuji_scan_bin() {
  local src="${WS_DIR}/scripts/wuji_scan_devices.c"
  local bin="${WS_DIR}/.cache/wuji_scan_devices"
  local sdk_root
  [[ -f "${src}" ]] || {
    print_error "缺少扫描源码: ${src}"
    return 1
  }
  ensure_wuji_sdk_env || return 1
  sdk_root="${WUJI_SDK_ROOT}"
  local lib_dir="${WUJI_SDK_LIB_DIR}"
  mkdir -p "${WS_DIR}/.cache"
  if [[ ! -x "${bin}" || "${src}" -nt "${bin}" || "${lib_dir}/libwuji_sdk_c.so" -nt "${bin}" ]]; then
    need_cmd gcc || return 1
    # Absolute rpath so the binary works regardless of cwd.
    if ! gcc -O2 -o "${bin}" "${src}" \
      -I"${sdk_root}/include" \
      -L"${lib_dir}" \
      -lwuji_sdk_c \
      -Wl,-rpath,"${lib_dir}" 2>/tmp/wuji_scan_build.err
    then
      print_error "编译 wuji_scan_devices 失败:"
      cat /tmp/wuji_scan_build.err >&2 || true
      return 1
    fi
  fi
  printf '%s' "${bin}"
}

# Compile/cache scripts/wuji_hand2_info.c → .cache/wuji_hand2_info
_ensure_wuji_hand2_info_bin() {
  local src="${WS_DIR}/scripts/wuji_hand2_info.c"
  local bin="${WS_DIR}/.cache/wuji_hand2_info"
  local sdk_root lib_dir
  [[ -f "${src}" ]] || {
    print_error "缺少检验源码: ${src}"
    return 1
  }
  ensure_wuji_sdk_env || return 1
  sdk_root="${WUJI_SDK_ROOT}"
  lib_dir="${WUJI_SDK_LIB_DIR}"
  mkdir -p "${WS_DIR}/.cache"
  if [[ ! -x "${bin}" || "${src}" -nt "${bin}" || "${lib_dir}/libwuji_sdk_c.so" -nt "${bin}" ]]; then
    need_cmd gcc || return 1
    if ! gcc -O2 -o "${bin}" "${src}" \
      -I"${sdk_root}/include" \
      -L"${lib_dir}" \
      -lwuji_sdk_c \
      -Wl,-rpath,"${lib_dir}" 2>/tmp/wuji_hand2_info_build.err
    then
      print_error "编译 wuji_hand2_info 失败:"
      cat /tmp/wuji_hand2_info_build.err >&2 || true
      return 1
    fi
  fi
  printf '%s' "${bin}"
}

# Connect + print Hand2 info; confirm launch. expect_side=left|right. return 0 to proceed.
_run_hand2_info_check() {
  local addr="$1"
  local expect_side="$2"
  local bin rc confirm
  bin="$(_ensure_wuji_hand2_info_bin)" || return 1
  print_info "正在检验真机信息 (${addr}, expect=${expect_side}) ..."
  rc=0
  "${bin}" --address "${addr}" --expect "${expect_side}" || rc=$?
  if [[ "${rc}" -eq 2 ]]; then
    print_warn "左右手与所选不符，请换设备或改选另一侧后再启动。"
    return 1
  fi
  if [[ "${rc}" -ne 0 ]]; then
    print_error "真机信息检验失败（rc=${rc}）。请检查网线/地址/电源。"
    return 1
  fi
  read -r -p "确认启动真机？[Y/n]: " confirm
  confirm="${confirm:-Y}"
  case "${confirm}" in
    y|Y|yes|YES) return 0 ;;
    *)
      print_warn "已取消启动"
      return 1
      ;;
  esac
}

# Run SDK scan; fill arrays SCAN_SNS / SCAN_ADDRS / SCAN_MODELS (Hand2 only).
_wuji_scan_fill() {
  SCAN_SNS=()
  SCAN_ADDRS=()
  SCAN_MODELS=()
  local bin line sn addr model
  bin="$(_ensure_wuji_scan_bin)" || return 1
  print_info "正在 SDK 扫描 Hand2 ..."
  while IFS=$'\t' read -r sn addr model; do
    [[ -z "${sn}" ]] && continue
    SCAN_SNS+=("${sn}")
    SCAN_ADDRS+=("${addr}")
    SCAN_MODELS+=("${model}")
  done < <("${bin}" 2>/dev/null)
  return 0
}

# Interactive pick from scan results → stdout address; return 1 on cancel/fail.
_prompt_pick_scanned_device() {
  _wuji_scan_fill || {
    print_warn "扫描工具不可用，请改选手动输入 IP:port 或启动时扫描"
    return 1
  }
  local n="${#SCAN_ADDRS[@]}"
  if [[ "${n}" -eq 0 ]]; then
    print_warn "未发现 WujiHand2。请检查网线/同网段/电源后重试。"
    return 1
  fi
  if [[ "${n}" -eq 1 ]]; then
    print_info "发现 1 台: SN=${SCAN_SNS[0]}  ${SCAN_ADDRS[0]}  (${SCAN_MODELS[0]})"
    printf '%s' "${SCAN_ADDRS[0]}"
    return 0
  fi
  local i pick
  echo "" >&2
  echo "发现 ${n} 台 Hand2:" >&2
  for ((i = 0; i < n; i++)); do
    echo "  $((i + 1))) SN=${SCAN_SNS[i]}  ${SCAN_ADDRS[i]}  (${SCAN_MODELS[i]})" >&2
  done
  echo "  0) 取消（回到连接方式）" >&2
  read -r -p "请选择设备 [0-${n}]: " pick
  if [[ "${pick}" =~ ^[1-9][0-9]*$ ]] && [[ "${pick}" -ge 1 && "${pick}" -le "${n}" ]]; then
    printf '%s' "${SCAN_ADDRS[$((pick - 1))]}"
    return 0
  fi
  print_warn "已取消扫描选择"
  return 1
}

# Prompt for IP:port on stdout; return 1 on invalid.
_prompt_manual_address() {
  local addr
  read -r -p "请输入 IP:port: " addr
  if [[ "${addr}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+:[0-9]+$ ]]; then
    printf '%s' "${addr}"
    return 0
  fi
  print_warn "格式无效，应为 IP:port（例 192.168.2.110:7447）"
  return 1
}

# Args: side_label expect_side (left|right)
# Prints device_address (may be empty → launch-time scan), or QS_CONN_BACK.
prompt_device_address() {
  local side_label="$1"
  local expect_side="${2:-left}"
  local choice addr=""

  while true; do
    echo "" >&2
    echo -e "${BLUE}${side_label} 连接方式:${NC}" >&2
    echo "  1) 检验真机信息后启动（推荐）" >&2
    echo "     扫描选设备 → 连接读取左右手/在线关节等 → 确认后再 launch。" >&2
    echo "  2) SDK 扫描并选择（不检验）" >&2
    echo "     启动前扫，列 SN+IP:port；多手/同手性点选。" >&2
    echo "  3) 启动时自动扫描" >&2
    echo "     不传地址；activate 按左右手匹配。单手或左右各一可用；两只同侧勿用。" >&2
    echo "  4) 手动输入 IP:port" >&2
    echo "     已知地址时直连（例 192.168.2.110:7447）。" >&2
    echo "  0) 返回" >&2
    read -r -p "请输入选项 [0-4] (默认: 1): " choice
    choice="${choice:-1}"

    case "${choice}" in
      0)
        printf '%s' "${QS_CONN_BACK}"
        return 0
        ;;
      1)
        addr="$(_prompt_pick_scanned_device)" || {
          print_warn "扫描失败。可选手动输入地址后检验。"
          addr="$(_prompt_manual_address)" || continue
        }
        if _run_hand2_info_check "${addr}" "${expect_side}"; then
          print_info "将使用 device_address:=${addr}"
          printf '%s' "${addr}"
          return 0
        fi
        continue
        ;;
      2)
        if addr="$(_prompt_pick_scanned_device)"; then
          print_info "将使用 device_address:=${addr}"
          printf '%s' "${addr}"
          return 0
        fi
        continue
        ;;
      3)
        print_info "未指定地址 → 启动时由硬件接口 wuji_scan + direction 匹配"
        printf '%s' ""
        return 0
        ;;
      4)
        if addr="$(_prompt_manual_address)"; then
          print_info "将使用 device_address:=${addr}"
          printf '%s' "${addr}"
          return 0
        fi
        continue
        ;;
      *)
        print_warn "无效选项"
        continue
        ;;
    esac
  done
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
  if [[ -n "${LAST_REAL_MODE:-}" ]]; then
    echo "  1) 使用上次真机 — $(_format_real_last_label)"
    echo "     （一键；建议新手上机先走「左手/右手 → 真机 → 检验信息」）"
    echo "  2) 左手 Hand2"
    echo "  3) 右手 Hand2"
    echo "  0) 返回"
    read -r -p "请输入选项 [0-3]: " choice
    case "${choice}" in
      1)
        do_launch_hand "${LAST_REAL_MODE}" "${LAST_REAL_DIRECTION}" \
          "${LAST_REAL_DEVICE_ADDRESS}" "${LAST_REAL_USE_RVIZ}" "${LAST_REAL_DESC}"
        return
        ;;
      2) choice="left" ;;
      3) choice="right" ;;
      0|"") echo "返回"; return ;;
      *) print_warn "无效选项"; return ;;
    esac
  else
    echo "  1) 左手 Hand2"
    echo "  2) 右手 Hand2"
    echo "  0) 返回"
    read -r -p "请输入选项 [0-2]: " choice
    case "${choice}" in
      1) choice="left" ;;
      2) choice="right" ;;
      0|"") echo "返回"; return ;;
      *) print_warn "无效选项"; return ;;
    esac
  fi

  local direction side_label expect_side device_address mode_choice use_rviz desc
  if [[ "${choice}" == "left" ]]; then
    direction="1"
    side_label="左手"
    expect_side="left"
  else
    direction="-1"
    side_label="右手"
    expect_side="right"
  fi

  mode_choice="$(launch_mode_menu)"
  [[ "${mode_choice}" == "0" || -z "${mode_choice}" ]] && { echo "返回"; return; }

  device_address=""
  use_rviz="true"
  case "${mode_choice}" in
    1) desc="${side_label} Hand2 仿真" ;;
    2) desc="${side_label} Hand2 仿真 headless"; use_rviz="false" ;;
    3)
      desc="${side_label} Hand2 真机"
      device_address="$(prompt_device_address "${side_label}" "${expect_side}")"
      if [[ "${device_address}" == "${QS_CONN_BACK}" ]]; then
        echo "返回"
        return
      fi
      ;;
    4)
      desc="${side_label} Hand2 真机 headless"
      use_rviz="false"
      device_address="$(prompt_device_address "${side_label}" "${expect_side}")"
      if [[ "${device_address}" == "${QS_CONN_BACK}" ]]; then
        echo "返回"
        return
      fi
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
  echo "  2) 真机包（同上；SDK 已 vendor 于 external/）" >&2
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
  print_warn "未找到 config/hand.local.conf（真机可不建；固定地址可用连接菜单「手动输入」）"
fi
print_info "真机: 建议「检验真机信息后启动」；或使用上次 / 扫描 / 手输 IP（见启动菜单）"

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
