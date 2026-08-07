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
# HT 工作空间默认 arms-full（含 ht_ros2_control）
core_deb_mode() {
  if dpkg-query -W -f='${Status}' ros-jazzy-arms-ros2-control-full 2>/dev/null | grep -q "install ok installed"; then
    return 0
  fi
  if dpkg-query -W -f='${Status}' ros-jazzy-arms-ros2-control 2>/dev/null | grep -q "install ok installed"; then
    return 0
  fi
  [ ! -d "${WS_DIR}/src/arms_ros2_control" ] && [ ! -d "${WS_DIR}/src/ocs2_ros2" ]
}

# arms-full 已提供 ht_ros2_control，真机编译无需再编该包
ht_from_deb() {
  dpkg-query -W -f='${Status}' ros-jazzy-arms-ros2-control-full 2>/dev/null | grep -q "install ok installed"
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

# ===================== 启动历史记忆 =====================
HIST_FILE="${HOME}/.config/panthera_ht/quick_start_history.json"

# 历史条数 (0/1/2)
history_count() {
  [ -f "${HIST_FILE}" ] || { echo 0; return 0; }
  python3 - "${HIST_FILE}" <<'PYEOF'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    d = []
print(len(d) if isinstance(d, list) else 0)
PYEOF
}

# 第 idx 条历史的描述 (idx: 0=最近, 1=次近)
hist_desc_at() {
  python3 - "${HIST_FILE}" "$1" <<'PYEOF'
import json, sys
path, idx = sys.argv[1], int(sys.argv[2])
try:
    d = json.load(open(path))
except Exception:
    d = []
if isinstance(d, list) and idx < len(d):
    print(d[idx].get("desc", ""))
PYEOF
}

# 记录一次启动配置 (按 type/hardware/control_mode/drag 去重, 保留最近 2 条不同配置)
record_launch() {
  python3 - "${HIST_FILE}" "$1" "$2" "$3" "$4" <<'PYEOF'
import json, os, sys, time
path, jtype, jhw, jcm, jdrag = sys.argv[1:6]
drag = (jdrag == "true")
entry = {
    "type": jtype,
    "hardware": jhw,
    "control_mode": jcm,
    "drag": drag,
    "ts": int(time.time()),
}
parts = ["单臂" if jtype == "single" else "双臂",
         "真机" if jhw == "real" else "仿真"]
if jcm:
    parts.append(jcm)
if drag:
    parts.append("拖动")
entry["desc"] = " · ".join(parts)
try:
    with open(path) as f:
        data = json.load(f)
    if not isinstance(data, list):
        data = []
except Exception:
    data = []

def same(e):
    return (e.get("type") == entry["type"] and e.get("hardware") == entry["hardware"]
            and e.get("control_mode", "") == entry["control_mode"]
            and bool(e.get("drag", False)) == drag)

data = [e for e in data if not same(e)]
data.insert(0, entry)
data = data[:2]
os.makedirs(os.path.dirname(path), exist_ok=True)
with open(path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)
PYEOF
}


# 主菜单: $1 = 历史条数 (0/1/2); 有历史时选项 1..N 为历史启动, 回车默认选 1
menu() {
  local n="${1:-0}"
  local last
  local d
  echo -e "${BLUE}========================================${NC}" >&2
  echo -e "${BLUE}  快速启动（Panthera HT Deploy Workspace）${NC}" >&2
  echo -e "${BLUE}  Workspace: ${WS_DIR}${NC}" >&2
  echo -e "${BLUE}========================================${NC}" >&2
  echo "" >&2
  echo "请选择操作:" >&2
  if [ "${n}" -ge 1 ]; then
    d="$(hist_desc_at 0)"
    echo "  1) 上次启动: ${d} (默认)" >&2
  fi
  if [ "${n}" -ge 2 ]; then
    d="$(hist_desc_at 1)"
    echo "  2) 另一次启动: ${d}" >&2
  fi
  echo "  $((n + 1))) 编译 (Build)" >&2
  echo "  $((n + 2))) 启动 (Launch)" >&2
  echo "  0) 退出 (Exit)" >&2
  echo "" >&2
  last=$((n + 2))
  read -r -p "请输入选项 [0-${last}] (默认 1): " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
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

# Real hardware: check /dev/ttyACM* and grant rw if needed.
ensure_ttyacm_access() {
  local devices=()
  local d

  echo -e "${BLUE}[INFO] 检查电机串口 /dev/ttyACM* ...${NC}"
  shopt -s nullglob
  devices=(/dev/ttyACM*)
  shopt -u nullglob

  if [ ${#devices[@]} -eq 0 ]; then
    echo -e "${RED}[ERROR] 未找到 /dev/ttyACM* 设备${NC}"
    echo -e "${YELLOW}      请检查 USB 连接、供电，以及 Panthera.yaml 中 Serial_Type=/dev/ttyACM${NC}"
    echo -e "${YELLOW}      排查: ls /dev/ttyACM*${NC}"
    read -r -p "仍要继续启动真机吗？[y/N]: " cont
    case "${cont}" in
      y|Y|yes|YES) return 0 ;;
      *) return 1 ;;
    esac
  fi

  echo -e "${GREEN}[INFO] 发现串口设备:${NC}"
  for d in "${devices[@]}"; do
    echo -e "${BLUE}  ${d}${NC}"
  done

  for d in "${devices[@]}"; do
    if [ -r "${d}" ] && [ -w "${d}" ]; then
      continue
    fi
    echo -e "${YELLOW}[INFO] ${d} 无读写权限，尝试: sudo chmod a+rw ${d}${NC}"
    if ! sudo chmod a+rw "${d}"; then
      echo -e "${RED}[ERROR] 无法设置 ${d} 权限${NC}"
      return 1
    fi
  done

  echo -e "${GREEN}[INFO] 串口权限正常${NC}"
  return 0
}

# 询问是否启用拖动模式（低刚度）; echo "true"/"false"
ask_drag_mode() {
  local yn
  read -r -p "启用拖动模式(低刚度, 夹爪可掰动)? [y/N]: " yn
  case "${yn}" in
    y|Y|yes|YES) echo "true" ;;
    *) echo "false" ;;
  esac
}

# 生成拖动模式 profile（通过 control.patch 覆盖 default_gains）
make_drag_profile() {
  local f="/tmp/panthera_ht_drag_profile.yaml"
  cat > "${f}" <<'EOF'
# Auto-generated by quick_start.sh (drag mode)
control:
  patch:
    ocs2_arm_controller:
      ros__parameters:
        default_gains: [0.01, 0.1]  # 拖动模式: [kp, kd]
EOF
  echo "${f}"
}

# 重现一条历史启动配置 (idx: 0=最近, 1=次近)
run_history_launch() {
  local idx="$1"
  local jtype jhw jcm jdrag
  IFS=',' read -r jtype jhw jcm jdrag <<EOF
$(python3 - "${HIST_FILE}" "${idx}" <<'PYEOF'
import json, sys
path, idx = sys.argv[1], int(sys.argv[2])
try:
    d = json.load(open(path))
except Exception:
    d = []
if not isinstance(d, list) or idx >= len(d):
    print("single,mock_components,,false")
    sys.exit(0)
e = d[idx]
print(",".join([
    e.get("type", "single"),
    e.get("hardware", "mock_components"),
    e.get("control_mode", ""),
    "true" if e.get("drag") else "false",
]))
PYEOF
)
EOF
  local -a args=()
  args+=("type:=${jtype}")
  if [ "${jdrag}" = "true" ]; then
    args+=("xacro_drag_mode:=true" "robot_profile:=$(make_drag_profile)")
  fi
  if [ "${jhw}" = "real" ]; then
    CONTROL_MODE="${jcm}"
    args+=("hardware:=real")
  else
    CONTROL_MODE=""
  fi
  local label
  label="$(hist_desc_at "${idx}")"
  _run_ocs2_demo "${label}" "${args[@]}"
}

# Launch ocs2 demo. Args: arm_label type_args...
# Uses CONTROL_MODE env if set (real only). Records the launch config to history.
_run_ocs2_demo() {
  local arm_label="$1"
  shift
  local -a extra_args=("$@")
  local mode_label=""
  local arg
  local jtype="single" jhw="mock_components" jcm="${CONTROL_MODE:-}" jdrag="false"

  ensure_ros_env || exit 1

  for arg in "${extra_args[@]}"; do
    if [ "${arg}" = "hardware:=real" ]; then
      ensure_ttyacm_access || exit 1
      break
    fi
  done

  if [ -n "${CONTROL_MODE:-}" ]; then
    extra_args+=("xacro_control_mode:=${CONTROL_MODE}")
    mode_label="，控制模式=${CONTROL_MODE}"
  fi

  # 解析本次启动配置并写入历史
  for arg in "${extra_args[@]}"; do
    case "${arg}" in
      type:=single) jtype="single" ;;
      type:=dual) jtype="dual" ;;
      hardware:=real) jhw="real" ;;
      xacro_drag_mode:=true) jdrag="true" ;;
    esac
  done
  record_launch "${jtype}" "${jhw}" "${jcm}" "${jdrag}"

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

HIST_COUNT="$(history_count)"
top_choice="$(menu "${HIST_COUNT}")"

case "${top_choice}" in
  0)
    echo "退出"
    exit 0
    ;;
esac

# 历史记忆入口: [1..HIST_COUNT] 为历史启动; 之后依次为 编译 / 启动
if [ "${HIST_COUNT}" -ge 1 ] && [ "${top_choice}" -ge 1 ] && [ "${top_choice}" -le "${HIST_COUNT}" ]; then
  run_history_launch $((top_choice - 1))
  exit 0
fi

if [ "${top_choice}" = "$((HIST_COUNT + 1))" ]; then
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
      if ht_from_deb; then
        echo -e "${BLUE}  模式: 核心包 deb(arms-full 含 ht_ros2_control) + 仅编译 HT 描述包${NC}"
        if ! run_colcon_build --packages-select panthera_ht_description --symlink-install; then
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
        fi
      else
        echo -e "${BLUE}  模式: 核心包 deb + 编译 HT 描述与真机驱动${NC}"
        if ! run_colcon_build --packages-select panthera_ht_description ht_ros2_control --symlink-install; then
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
        fi
      fi
    else
      if ! run_colcon_build --packages-up-to \
        ht_ros2_control \
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
    exit 0
fi

if [ "${top_choice}" = "$((HIST_COUNT + 2))" ]; then
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

        # 拖动模式: 低刚度 (gripper kp/kd + default_gains)
        drag_args=()
        if [ "$(ask_drag_mode)" = "true" ]; then
          drag_profile="$(make_drag_profile)"
          drag_args=("xacro_drag_mode:=true" "robot_profile:=${drag_profile}")
          echo -e "${BLUE}[INFO] 拖动模式已启用（低刚度）${NC}"
        fi

        case "${mode_choice}" in
          1)
            _run_ocs2_demo "${arm_label}仿真（Panthera HT）" "${type_arg}" "${drag_args[@]}"
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
              _run_ocs2_demo "${arm_label}真机（Panthera HT）" "${type_arg}" "${drag_args[@]}" "hardware:=real"
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
    exit 0
fi

echo -e "${YELLOW}无效选项，请重新运行脚本${NC}"
exit 1
