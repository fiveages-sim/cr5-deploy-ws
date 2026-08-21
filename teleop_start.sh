#!/usr/bin/env bash

# 遥操作启动脚本（Panthera HT Deploy Workspace）
# - 启动遥操作链路：drag_teleop_controller（后台）+ teleop_joint_mapper（前台）
# - 含两个记忆启动项、编译、启动选择（真机/仿真）
# - 真机：力反馈(y/N 默认关) → 控制盒(默认自动) → 配置文件
# - 仿真：配置文件
# - 配置文件来自 src/teleop-joint-mapper/config，默认 joint_mapper.yaml
# - 默认选项前加 '*' 标识

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"

. "${SCRIPT_DIR}/scripts/lib_common.sh"

# ===================== 常量 / 默认参数（集中在此，方便查看与修改） =====================

# 启动历史记忆文件（与 quick_start.sh 共用；条目用 kind 字段区分）
HIST_FILE="${HOME}/.config/panthera_ht/quick_start_history.json"
# 本脚本历史条目的 kind 标识
HIST_KIND="teleop"
# 历史保留条数（最近 N 条不同配置）
HIST_MAX=2

# drag_teleop_controller 启动包 / launch 文件
DRAG_LAUNCH_PKG="drag_teleop_controller"
DRAG_LAUNCH_FILE="drag_teleop_controller.launch.py"

# teleop_joint_mapper 启动包 / launch 文件
MAPPER_LAUNCH_PKG="teleop_joint_mapper"
MAPPER_LAUNCH_FILE="teleop_joint_mapper.launch.py"

# 机器人 / 臂组合
ROBOT_NAME="panthera_ht"
ARM_TYPE="dual"

# 遥操作配置文件目录与默认配置
CONFIG_DIR="${WS_DIR}/src/teleop-joint-mapper/config"
DEFAULT_CONFIG="joint_mapper.yaml"

# drag 固定启动参数
USE_SIM_TIME="false"
RViz="false"

# 真机 hardware 透传参数（hardware_ 前缀，仅 hardware:=real/real_usb 生效；
# 关节 kp/kd 为数组，每臂 6 值，dual 时 xacro 自动拼接为 12 值）
HARDWARE_JOINT_KP=(0.01 0.01 0.01 0.01 0.01 0.01)
HARDWARE_JOINT_KD=(0.1 0.1 0.1 0.1 0.1 0.1)
HARDWARE_GRIPPER_KP=0.001
HARDWARE_GRIPPER_KD=0.01

# ===================== 环境 =====================

source_ros_underlay() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    # ROS2 ament 生成的 setup 脚本会引用未定义变量（如 AMENT_TRACE_SETUP_FILES、
    # AMENT_PYTHON_EXECUTABLE），在 set -u 下会报 "unbound variable"，
    # 因此 source 期间临时关闭 nounset
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
    return 0
  fi
  echo -e "${YELLOW}[WARN] 未找到 /opt/ros/jazzy/setup.bash${NC}"
  return 1
}

source_workspace_env() {
  source_ros_underlay || return 1
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "${WS_DIR}/install/setup.bash"
    set -u
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
  echo -e "${YELLOW}[WARN] 请先在此 workspace 编译，然后再运行启动选项：${NC}"
  echo -e "${YELLOW}      cd ${WS_DIR} && colcon build --symlink-install${NC}"
  return 1
}

run_colcon_build() {
  cd "${WS_DIR}" || exit 1
  source_ros_underlay
  colcon build "$@"
}

# ===================== 启动历史记忆（kind=teleop） =====================
# 与 quick_start.sh 共用历史文件；本脚本只统计/去重/重现 kind=="teleop" 的条目。

# 历史条数 (0/1/2)
teleop_history_count() {
  [ -f "${HIST_FILE}" ] || { echo 0; return 0; }
  python3 - "${HIST_FILE}" "${HIST_KIND}" <<'PYEOF'
import json, sys
path, kind = sys.argv[1], sys.argv[2]
try:
    d = json.load(open(path))
except Exception:
    d = []
if not isinstance(d, list):
    d = []
d = [e for e in d if e.get("kind") == kind]
print(len(d))
PYEOF
}

# 第 idx 条历史的描述 (idx: 0=最近, 1=次近)
teleop_hist_desc_at() {
  python3 - "${HIST_FILE}" "${HIST_KIND}" "$1" <<'PYEOF'
import json, sys
path, kind, idx = sys.argv[1], sys.argv[2], int(sys.argv[3])
try:
    d = json.load(open(path))
except Exception:
    d = []
if not isinstance(d, list):
    d = []
d = [e for e in d if e.get("kind") == kind]
if idx < len(d):
    print(d[idx].get("desc", ""))
PYEOF
}

# 记录一次遥操作启动配置 (按 hardware/feedback/usb_select/config_file 去重,
# 保留最近 HIST_MAX 条不同配置；其他 kind 条目原样保留)
record_teleop_launch() {
  python3 - "${HIST_FILE}" "${HIST_KIND}" "$1" "$2" "$3" "$4" "${HIST_MAX}" <<'PYEOF'
import json, os, sys, time
path, kind, jhw, jfb, jusb, jconfig, hist_max = sys.argv[1:8]
hist_max = int(hist_max)
fb = (jfb == "true")
entry = {
    "kind": kind,
    "hardware": jhw,
    "feedback": fb,
    "usb_select": jusb,
    "config_file": jconfig,
    "ts": int(time.time()),
}
parts = ["真机" if jhw == "real" else "仿真"]
if fb:
    parts.append("力反馈")
if jusb and jusb != "auto":
    parts.append(f"盒:{jusb}")
parts.append(os.path.basename(jconfig))
entry["desc"] = " · ".join(parts)
try:
    with open(path) as f:
        data = json.load(f)
    if not isinstance(data, list):
        data = []
except Exception:
    data = []

def same(e):
    return (e.get("hardware") == entry["hardware"]
            and bool(e.get("feedback", False)) == fb
            and e.get("usb_select", "auto") == jusb
            and e.get("config_file", "") == jconfig)

mine = [e for e in data if e.get("kind") == kind]
other = [e for e in data if e.get("kind") != kind]
mine = [e for e in mine if not same(e)]
mine.insert(0, entry)
mine = mine[:hist_max]
data = mine + other
os.makedirs(os.path.dirname(path), exist_ok=True)
with open(path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)
PYEOF
}

# 重现一条历史遥操作启动配置 (idx: 0=最近, 1=次近)
run_teleop_history_launch() {
  local idx="$1"
  local jhw jfb jusb jconfig
  IFS=',' read -r jhw jfb jusb jconfig <<EOF
$(python3 - "${HIST_FILE}" "${HIST_KIND}" "${idx}" <<'PYEOF'
import json, sys
path, kind, idx = sys.argv[1], sys.argv[2], int(sys.argv[3])
try:
    d = json.load(open(path))
except Exception:
    d = []
if not isinstance(d, list):
    d = []
d = [e for e in d if e.get("kind") == kind]
if idx >= len(d):
    print("mock_components,false,auto,")
    sys.exit(0)
e = d[idx]
print(",".join([
    e.get("hardware", "mock_components"),
    "true" if e.get("feedback") else "false",
    e.get("usb_select", "auto"),
    e.get("config_file", ""),
]))
PYEOF
)
EOF
  local label
  label="$(teleop_hist_desc_at "${idx}")"
  _run_teleop "${jhw}" "${jfb}" "${jusb}" "${jconfig}" "${label}"
}

# ===================== 真机串口 / 控制盒 =====================

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

# 列出当前检测到的控制盒（按 USB 设备路径去重）
# 输出格式: 每行 "ID_PATH|样例口"（ID_PATH 如 pci-0000:00:14.0-usb-0:1.2，已去掉接口段）
detect_usb_boxes() {
  shopt -s nullglob
  local p path box
  declare -A seen
  for p in /dev/ttyACM*; do
    path="$(udevadm info -q property -n "${p}" 2>/dev/null | awk -F= '/^ID_PATH=/{print $2; exit}')"
    [ -n "${path}" ] || continue
    # 去掉接口段（如末尾 :1.0），只保留控制盒自身的 USB 路径（一盒一行）
    box="$(printf '%s' "${path}" | sed -E 's/:[0-9]+\.[0-9]+$//')"
    if [ -z "${seen[${box}]:-}" ]; then
      seen[${box}]="1"
      echo "${box}|${p}"
    fi
  done
  shopt -u nullglob
}

# 询问选择控制盒（多套机械臂同机时）；echo "auto" 或 USB 路径，或 "back"
ask_usb_select() {
  local boxes=() paths=() i=1 choice
  echo "" >&2
  echo "请选择控制盒 (USB 设备):" >&2
  echo " *0) 自动检测（仅 1 个控制盒时自动连接；多个时启动会报错并列出路径）" >&2
  while IFS='|' read -r path dev; do
    paths+=("${path}")
    echo "  ${i}) ${path}   (样例口: ${dev})" >&2
    i=$((i + 1))
  done < <(detect_usb_boxes)
  if [ "${i}" -eq 1 ]; then
    echo "  （当前未检测到 /dev/ttyACM*，请检查 USB 连接与供电）" >&2
  fi
  echo "  q) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-$((${#paths[@]}))]（回车=默认 0）: " choice
  if [ -z "${choice}" ]; then
    choice="0"
  fi
  case "${choice}" in
    0) echo "auto" ;;
    q|Q) echo "back" ;;
    *)
      if [[ "${choice}" =~ ^[0-9]+$ ]] && [ "${choice}" -ge 1 ] && [ "${choice}" -le "${#paths[@]}" ]; then
        echo "${paths[$((choice - 1))]}"
      else
        echo -e "${YELLOW}无效选项，使用自动检测${NC}" >&2
        echo "auto"
      fi
      ;;
  esac
}

# ===================== 力反馈 / 配置文件 =====================

# 询问是否开启力反馈（从臂碰撞回推主臂）；echo "true"/"false"（默认关）
ask_feedback() {
  local yn
  read -r -p "开启力反馈（从臂碰撞回推主臂）? [y/N]: " yn
  case "${yn}" in
    y|Y|yes|YES) echo "true" ;;
    *) echo "false" ;;
  esac
}

# 列出 config 目录下所有 yaml 配置文件（仅文件名）
list_config_files() {
  shopt -s nullglob
  local f
  for f in "${CONFIG_DIR}"/*.yaml; do
    echo "$(basename "${f}")"
  done
  shopt -u nullglob
}

# 选择配置文件；echo 绝对路径，或 "back"（默认 DEFAULT_CONFIG）
ask_config_file() {
  local -a files=()
  local i=1 choice default_idx=1 f
  echo "" >&2
  echo "请选择遥操作配置文件:" >&2
  while IFS= read -r f; do
    files+=("${f}")
    if [ "${f}" = "${DEFAULT_CONFIG}" ]; then
      default_idx="${i}"
      echo " *${i}) ${f}" >&2
    else
      echo "  ${i}) ${f}" >&2
    fi
    i=$((i + 1))
  done < <(list_config_files)
  if [ "${#files[@]}" -eq 0 ]; then
    echo -e "${RED}[ERROR] 未找到配置文件: ${CONFIG_DIR}${NC}" >&2
    echo "back"
    return
  fi
  echo "  q) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [1-${#files[@]}]（回车=默认 ${DEFAULT_CONFIG}）: " choice
  if [ -z "${choice}" ]; then
    choice="${default_idx}"
  fi
  case "${choice}" in
    q|Q) echo "back" ;;
    *)
      if [[ "${choice}" =~ ^[0-9]+$ ]] && [ "${choice}" -ge 1 ] && [ "${choice}" -le "${#files[@]}" ]; then
        echo "${CONFIG_DIR}/${files[$((choice - 1))]}"
      else
        echo -e "${YELLOW}无效选项，使用默认 ${DEFAULT_CONFIG}${NC}" >&2
        echo "${CONFIG_DIR}/${DEFAULT_CONFIG}"
      fi
      ;;
  esac
}

# ===================== 启动编排 =====================
# drag_teleop_controller 后台 + teleop_joint_mapper 前台；
# mapper 结束（Ctrl+C）后 trap 清理后台 drag。
# 参数: hardware feedback usb_select config_file label
_run_teleop() {
  local jhw="$1" jfb="$2" jusb="$3" jconfig="$4" label="$5"
  local -a drag_args=()
  local drag_pid=""

  ensure_ros_env || exit 1

  # 真机：检查串口权限
  if [ "${jhw}" = "real" ]; then
    ensure_ttyacm_access || exit 1
  fi

  # 记录历史
  record_teleop_launch "${jhw}" "${jfb}" "${jusb}" "${jconfig}"

  # 组装 drag 启动参数
  drag_args+=("robot:=${ROBOT_NAME}" "type:=${ARM_TYPE}" "hardware:=${jhw}")
  drag_args+=("use_sim_time:=${USE_SIM_TIME}" "rviz:=${RViz}" "feedback:=${jfb}")
  if [ "${jhw}" = "real" ]; then
    # 真机：透传低刚度 kp/kd（hardware_ 前缀，xacro 展开进 URDF <param>）
    local kp kd
    kp="$(IFS=', '; echo "${HARDWARE_JOINT_KP[*]}")"
    kd="$(IFS=', '; echo "${HARDWARE_JOINT_KD[*]}")"
    drag_args+=("hardware_joint_kp:=${kp}" "hardware_joint_kd:=${kd}")
    drag_args+=("hardware_gripper_kp:=${HARDWARE_GRIPPER_KP}" "hardware_gripper_kd:=${HARDWARE_GRIPPER_KD}")
    if [ "${jusb}" != "auto" ]; then
      drag_args+=("xacro_usb_select:=${jusb}")
    fi
  fi

  # 后台启动 drag
  echo -e "${GREEN}启动 ${label}：drag_teleop_controller（后台）...${NC}"
  echo -e "${BLUE}  ${DRAG_LAUNCH_PKG}/${DRAG_LAUNCH_FILE} ${drag_args[*]}${NC}"
  ros2 launch "${DRAG_LAUNCH_PKG}" "${DRAG_LAUNCH_FILE}" "${drag_args[@]}" &
  drag_pid=$!

  # 前台启动 mapper
  echo -e "${GREEN}启动 teleop_joint_mapper（前台，Ctrl+C 结束）...${NC}"
  echo -e "${BLUE}  配置文件: ${jconfig}${NC}"
  ros2 launch "${MAPPER_LAUNCH_PKG}" "${MAPPER_LAUNCH_FILE}" "config_file:=${jconfig}"
  local mapper_rc=$?

  # mapper 结束后清理后台 drag
  if [ -n "${drag_pid}" ] && kill -0 "${drag_pid}" 2>/dev/null; then
    echo -e "${YELLOW}[INFO] 遥操作结束，清理后台 drag_teleop_controller (pid ${drag_pid})${NC}"
    kill "${drag_pid}" 2>/dev/null
    wait "${drag_pid}" 2>/dev/null
  fi
  return "${mapper_rc}"
}

# ===================== 菜单 =====================

# 主菜单: $1 = 历史条数 (0/1/2); 有历史时选项 1..N 为历史启动, 回车默认选 1
menu() {
  local n="${1:-0}"
  local last d
  echo -e "${BLUE}========================================${NC}" >&2
  echo -e "${BLUE}  遥操作启动（Panthera HT Deploy Workspace）${NC}" >&2
  echo -e "${BLUE}  Workspace: ${WS_DIR}${NC}" >&2
  echo -e "${BLUE}========================================${NC}" >&2
  echo "" >&2
  echo "请选择操作:" >&2
  if [ "${n}" -ge 1 ]; then
    d="$(teleop_hist_desc_at 0)"
    echo " *1) 上次启动: ${d}" >&2
  fi
  if [ "${n}" -ge 2 ]; then
    d="$(teleop_hist_desc_at 1)"
    echo "  $((n + 0))) 另一次启动: ${d}" >&2
  fi
  echo "  $((n + 1))) 编译 (Build)" >&2
  echo "  $((n + 2))) 启动 (Launch)" >&2
  echo "  0) 退出 (Exit)" >&2
  echo "" >&2
  last=$((n + 2))
  read -r -p "请输入选项 [0-${last}]（回车=默认 1）: " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  echo "${choice}"
}

build_menu() {
  echo "" >&2
  echo "请选择编译目标:" >&2
  echo "  1) 编译遥操作包 (teleop_joint_mapper + drag_teleop_controller)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-1]: " choice
  echo "${choice}"
}

launch_menu() {
  echo "" >&2
  echo "请选择启动项:" >&2
  echo " *1) 真机启动" >&2
  echo "  2) 仿真启动" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]（回车=默认 1）: " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  echo "${choice}"
}

# ===================== 主流程 =====================

need_cmd colcon || echo -e "${YELLOW}[WARN] 未找到 colcon，编译选项会失败（通常需要安装 ROS 发行版环境）。${NC}"

HIST_COUNT="$(teleop_history_count)"
top_choice="$(menu "${HIST_COUNT}")"

case "${top_choice}" in
  0)
    echo "退出"
    exit 0
    ;;
esac

# 历史记忆入口: [1..HIST_COUNT] 为历史启动; 之后依次为 编译 / 启动
if [ "${HIST_COUNT}" -ge 1 ] && [ "${top_choice}" -ge 1 ] && [ "${top_choice}" -le "${HIST_COUNT}" ]; then
  run_teleop_history_launch $((top_choice - 1))
  exit 0
fi

if [ "${top_choice}" = "$((HIST_COUNT + 1))" ]; then
  build_choice="$(build_menu)"
  case "${build_choice}" in
    1)
      echo -e "${GREEN}开始编译遥操作包...${NC}"
      if ! run_colcon_build --packages-select teleop_joint_mapper drag_teleop_controller --symlink-install; then
        echo -e "${YELLOW}编译过程中出现错误${NC}"
        exit 1
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
    1)
      # 真机：力反馈 → 控制盒 → 配置文件
      fb="$(ask_feedback)"
      usb="$(ask_usb_select)"
      if [ "${usb}" = "back" ]; then
        echo "返回"
      else
        cfg="$(ask_config_file)"
        if [ "${cfg}" = "back" ]; then
          echo "返回"
        else
          _run_teleop "real" "${fb}" "${usb}" "${cfg}" "真机遥操作"
        fi
      fi
      ;;
    2)
      # 仿真：配置文件（无力反馈/控制盒）
      cfg="$(ask_config_file)"
      if [ "${cfg}" = "back" ]; then
        echo "返回"
      else
        _run_teleop "mock_components" "false" "auto" "${cfg}" "仿真遥操作"
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
  exit 0
fi

echo -e "${YELLOW}无效选项，请重新运行脚本${NC}"
exit 1
