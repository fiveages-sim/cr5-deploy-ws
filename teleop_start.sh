#!/usr/bin/env bash

# 遥操作启动脚本（Panthera HT Deploy Workspace）
# - 主菜单：两次启动历史记忆 / 编译 / 启动
# - 编译：真机包（含 ht-ros2-control）与仿真包两组，缺失的源码目录自动跳过
# - 启动流程：master/slave → 真机/仿真 → 控制模式 → （master）力反馈 → USB 口 → 配置文件
#   * 控制模式：mit（slave 默认）/ effort（master 默认）/ position（仅 slave）
#     高擎机械臂真机：控制模式同时透传 hardware_control_mode；
#     master 额外设置 hardware_gripper_kp/kd:=0；仿真不传任何 hardware 参数
#   * 力反馈（仅 master）：*none / position / effort
#   * USB 口：仅 1 个控制盒时默认 auto；多个时第一个为默认
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

# 机器人 / 臂组合
ROBOT_NAME="panthera_ht"
ARM_TYPE="dual"

# drag 固定启动参数
USE_SIM_TIME="false"
RViz="false"

# 真机 hardware 透传参数（hardware_ 前缀，仅 hardware:=real/real_usb 生效；
# 关节 kp/kd 为数组，每臂 6 值，dual 时 xacro 自动拼接为 12 值）
HARDWARE_JOINT_KP=(30.0 30.0 30.0 30.0 30.0 30.0)
HARDWARE_JOINT_KD=(3.0 3.0 3.0 3.0 3.0 3.0)

# 编译目标源码目录（真机多 ht-ros2-control；arms_ros2_control / ocs2_ros2 存在才编译）
BUILD_PKGS_BASE=(drag_teleop_controller robot-descriptions-ht)
BUILD_DIRS_OPTIONAL=(arms_ros2_control ocs2_ros2)
BUILD_DIR_REAL_ONLY=(ht-ros2-control)

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

# 记录一次遥操作启动配置 (按 role/hardware/mode/feedback/moveJ_pub/usb_select 去重,
# 保留最近 HIST_MAX 条不同配置；其他 kind 条目原样保留)
record_teleop_launch() {
  # $1=role $2=hardware $3=mode $4=feedback $5=moveJ_pub $6=usb_select
  python3 - "${HIST_FILE}" "${HIST_KIND}" "$1" "$2" "$3" "$4" "$5" "$6" "${HIST_MAX}" <<'PYEOF'
import json, os, sys, time
path, kind, role, jhw, mode, fb, jmj, jusb, hist_max = sys.argv[1:10]
hist_max = int(hist_max)
entry = {
    "kind": kind,
    "role": role,
    "hardware": jhw,
    "mode": mode,
    "feedback": fb,
    "moveJ_pub": jmj,
    "usb_select": jusb,
    "ts": int(time.time()),
}
parts = ["真机" if jhw == "real" else "仿真", role, mode]
if role == "master":
    parts.append(f"反馈:{fb}")
    if jmj == "true":
        parts.append("ocs2发布")
if jusb and jusb != "auto":
    parts.append(f"盒:{jusb}")
entry["desc"] = " · ".join(parts)
try:
    with open(path) as f:
        data = json.load(f)
    if not isinstance(data, list):
        data = []
except Exception:
    data = []

def same(e):
    return (e.get("role") == role
            and e.get("hardware") == jhw
            and e.get("mode") == mode
            and e.get("feedback", "none") == fb
            and e.get("moveJ_pub", "false") == jmj
            and e.get("usb_select", "auto") == jusb)

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
  local jrole jhw jmode jfb jjmj jusb
  IFS=',' read -r jrole jhw jmode jfb jjmj jusb <<EOF
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
    print("master,mock_components,mit,none,false,auto")
    sys.exit(0)
e = d[idx]
print(",".join([
    e.get("role", "master"),
    e.get("hardware", "mock_components"),
    e.get("mode", "mit"),
    e.get("feedback", "none"),
    e.get("moveJ_pub", "false"),
    e.get("usb_select", "auto"),
]))
PYEOF
)
EOF
  local label
  label="$(teleop_hist_desc_at "${idx}")"
  _run_teleop "${jrole}" "${jhw}" "${jmode}" "${jfb}" "${jjmj}" "${jusb}" "${label}"
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

# 询问选择控制盒（多套机械臂同机时）
# 仅 1 个控制盒时默认 auto；多个时第一个为默认。echo USB 路径/"auto"，或 "back"
ask_usb_select() {
  local -a lines=() paths=()
  local i=1 choice default_choice line path dev
  while IFS= read -r line; do
    lines+=("${line}")
  done < <(detect_usb_boxes)

  echo "" >&2
  echo "请选择控制盒 (USB 设备):" >&2
  if [ "${#lines[@]}" -le 1 ]; then
    # 0 或 1 个控制盒：auto 为默认
    echo " *0) 自动检测（仅 1 个控制盒时自动连接；多个时启动会报错并列出路径）" >&2
    default_choice="0"
    for line in "${lines[@]}"; do
      IFS='|' read -r path dev <<<"${line}"
      paths+=("${path}")
      echo "  ${i}) ${path}   (样例口: ${dev})" >&2
      i=$((i + 1))
    done
  else
    # 多个控制盒：第一个为默认
    for line in "${lines[@]}"; do
      IFS='|' read -r path dev <<<"${line}"
      paths+=("${path}")
      if [ "${i}" -eq 1 ]; then
        echo " *${i}) ${path}   (样例口: ${dev})" >&2
      else
        echo "  ${i}) ${path}   (样例口: ${dev})" >&2
      fi
      i=$((i + 1))
    done
    echo "  0) 自动检测" >&2
    default_choice="1"
  fi
  if [ "${#paths[@]}" -eq 0 ] && [ "${default_choice}" != "0" ]; then
    echo "  （当前未检测到 /dev/ttyACM*，请检查 USB 连接与供电）" >&2
  fi
  echo "  q) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-${#paths[@]}]（回车=默认 ${default_choice}）: " choice
  if [ -z "${choice}" ]; then
    choice="${default_choice}"
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

# ===================== 角色 / 真机仿真 / 控制模式 / 力反馈 =====================

# 选择启动角色；echo "master"/"slave"（默认 master）
ask_role() {
  local choice
  echo "" >&2
  echo "请选择启动角色:" >&2
  echo " *1) master（主臂：操作者拖动，重力补偿）" >&2
  echo "  2) slave（从臂：跟随主臂）" >&2
  echo "  q) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [1-2]（回车=默认 1）: " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  case "${choice}" in
    2) echo "slave" ;;
    q|Q) echo "back" ;;
    *) echo "master" ;;
  esac
}

# 选择真机 / 仿真；echo "real"/"mock_components"（默认真机）
ask_hw_target() {
  local choice
  echo "" >&2
  echo "请选择启动目标:" >&2
  echo " *1) 真机" >&2
  echo "  2) 仿真" >&2
  echo "  q) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [1-2]（回车=默认 1）: " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  case "${choice}" in
    2) echo "mock_components" ;;
    q|Q) echo "back" ;;
    *) echo "real" ;;
  esac
}

# 选择控制模式；$1 = role。
# master: mit / *effort；slave: *mit / effort / position。echo 模式名或 "back"
ask_mode() {
  local role="$1" choice default_idx max_idx
  if [ "${role}" = "master" ]; then
    default_idx=2; max_idx=2
    echo "" >&2
    echo "请选择控制模式:" >&2
    echo "  1) mit" >&2
    echo " *2) effort" >&2
  else
    default_idx=1; max_idx=3
    echo "" >&2
    echo "请选择控制模式:" >&2
    echo " *1) mit" >&2
    echo "  2) effort" >&2
    echo "  3) position" >&2
  fi
  echo "  q) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [1-${max_idx}]（回车=默认 ${default_idx}）: " choice
  if [ -z "${choice}" ]; then
    choice="${default_idx}"
  fi
  case "${choice}" in
    q|Q) echo "back" ;;
    1) echo "mit" ;;
    2) echo "effort" ;;
    3) echo "position" ;;
    *)
      echo -e "${YELLOW}无效选项，使用默认${NC}" >&2
      [ "${role}" = "master" ] && echo "effort" || echo "mit"
      ;;
  esac
}

# 选择力反馈类型（仅 master）；echo "none"/"position"/"effort"（默认 none）
ask_feedback() {
  local choice
  echo "" >&2
  echo "请选择力反馈类型（从臂碰撞回推主臂）:" >&2
  echo " *1) none（关闭）" >&2
  echo "  2) position（基于位置误差）" >&2
  echo "  3) effort（基于外部力矩）" >&2
  echo "  q) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [1-3]（回车=默认 1）: " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  case "${choice}" in
    2) echo "position" ;;
    3) echo "effort" ;;
    q|Q) echo "back" ;;
    *) echo "none" ;;
  esac
}

# 选择是否发布 ocs2 命令（仅 master）；echo "true"/"false"（默认 false）
ask_movej_pub() {
  local choice
  echo "" >&2
  echo "是否发布 ocs2 命令（moveJ + 夹爪位置命令）:" >&2
  echo " *1) 不发布" >&2
  echo "  2) 发布 (moveJ_pub:=true)" >&2
  echo "  q) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [1-2]（回车=默认 1）: " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  case "${choice}" in
    2) echo "true" ;;
    q|Q) echo "back" ;;
    *) echo "false" ;;
  esac
}

# ===================== 启动编排 =====================
# 仅启动 drag_teleop_controller（前台），Ctrl+C 结束。
# 参数: role hardware mode feedback moveJ_pub usb_select label
_run_teleop() {
  local jrole="$1" jhw="$2" jmode="$3" jfb="$4" jjmj="$5" jusb="$6" label="$7"
  local -a drag_args=()

  ensure_ros_env || exit 1

  # 真机：检查串口权限
  if [ "${jhw}" = "real" ]; then
    ensure_ttyacm_access || exit 1
  fi

  # 记录历史
  record_teleop_launch "${jrole}" "${jhw}" "${jmode}" "${jfb}" "${jjmj}" "${jusb}"

  # 组装 drag 启动参数
  drag_args+=("robot:=${ROBOT_NAME}" "type:=${ARM_TYPE}" "role:=${jrole}")
  drag_args+=("hardware:=${jhw}")
  drag_args+=("use_sim_time:=${USE_SIM_TIME}" "rviz:=${RViz}")
  drag_args+=("mode:=${jmode}")
  if [ "${jrole}" = "master" ]; then
    # none → false（控制器 feedback 参数取值 false|position|effort）
    local fb="${jfb}"
    [ "${fb}" = "none" ] && fb="false"
    drag_args+=("feedback:=${fb}")
    drag_args+=("moveJ_pub:=${jjmj}")
  fi
  if [ "${jhw}" = "real" ]; then
    # 真机：透传低刚度 kp/kd（hardware_ 前缀，xacro 展开进 URDF <param>）
    local kp kd
    kp="$(IFS=', '; echo "${HARDWARE_JOINT_KP[*]}")"
    kd="$(IFS=', '; echo "${HARDWARE_JOINT_KD[*]}")"
    drag_args+=("hardware_joint_kp:=${kp}" "hardware_joint_kd:=${kd}")
    # 高擎机械臂：控制模式同步到硬件（mit/effort/position）
    drag_args+=("hardware_control_mode:=${jmode}")
    if [ "${jrole}" = "master" ]; then
      # 主臂：夹爪增益清零（避免位置环对抗拖动）
      drag_args+=("hardware_gripper_kp:=0" "hardware_gripper_kd:=0")
    fi
    if [ "${jusb}" != "auto" ]; then
      drag_args+=("xacro_usb_select:=${jusb}")
    fi
  fi

  # 前台启动 drag（Ctrl+C 结束）
  echo -e "${GREEN}启动 ${label}：drag_teleop_controller ...${NC}"
  echo -e "${BLUE}  ${DRAG_LAUNCH_PKG}/${DRAG_LAUNCH_FILE} ${drag_args[*]}${NC}"
  ros2 launch "${DRAG_LAUNCH_PKG}" "${DRAG_LAUNCH_FILE}" "${drag_args[@]}"
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
  echo " *1) 编译真机包 (drag_teleop_controller + robot-descriptions-ht" >&2
  echo "     + ht-ros2-control + arms_ros2_control/ocs2_ros2 若存在)" >&2
  echo "  2) 编译仿真包 (drag_teleop_controller + robot-descriptions-ht" >&2
  echo "     + arms_ros2_control/ocs2_ros2 若存在)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]（回车=默认 1）: " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  echo "${choice}"
}

# 收集目录下全部 colcon 包名（存在 package.xml 才算；每行一个）
collect_pkgs() {
  local dir="$1"
  [ -d "${dir}" ] || return 0
  find "${dir}" -name package.xml 2>/dev/null | while read -r f; do
    sed -n 's/.*<name>\([^<]*\)<\/name>.*/\1/p' "${f}" | head -1
  done
}

# 编译目标包：$1 = real | sim
do_build() {
  local target="$1"
  local -a pkgs=() arr=()
  local d

  # 基础包（源码目录名，colcon 按 --paths 不支持 select，这里转成包名）
  for d in "${BUILD_PKGS_BASE[@]}"; do
    mapfile -t arr < <(collect_pkgs "${WS_DIR}/src/${d}")
    pkgs+=("${arr[@]}")
  done
  # 可选目录（存在才编译）
  for d in "${BUILD_DIRS_OPTIONAL[@]}"; do
    mapfile -t arr < <(collect_pkgs "${WS_DIR}/src/${d}")
    pkgs+=("${arr[@]}")
  done
  # 真机专属驱动
  if [ "${target}" = "real" ]; then
    for d in "${BUILD_DIR_REAL_ONLY[@]}"; do
      mapfile -t arr < <(collect_pkgs "${WS_DIR}/src/${d}")
      pkgs+=("${arr[@]}")
    done
  fi

  if [ "${#pkgs[@]}" -eq 0 ]; then
    echo -e "${RED}[ERROR] 未找到可编译的包${NC}"
    return 1
  fi

  cd "${WS_DIR}" || return 1
  source_ros_underlay
  echo -e "${BLUE}  编译包: ${pkgs[*]}${NC}"
  colcon build --packages-select "${pkgs[@]}" --symlink-install
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
    1|2)
      if [ "${build_choice}" = "1" ]; then
        echo -e "${GREEN}开始编译真机所需包...${NC}"
      else
        echo -e "${GREEN}开始编译仿真所需包...${NC}"
      fi
      if ! do_build "$([ "${build_choice}" = "1" ] && echo real || echo sim)"; then
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
  # 启动流程：角色 → 真机/仿真 → 控制模式
  #   → （master 真机）力反馈 → （master）ocs2 发布 → （真机）USB 口
  role="$(ask_role)"
  if [ "${role}" = "back" ]; then
    echo "返回"
    exit 0
  fi

  hw="$(ask_hw_target)"
  if [ "${hw}" = "back" ]; then
    echo "返回"
    exit 0
  fi

  mode="$(ask_mode "${role}")"
  if [ "${mode}" = "back" ]; then
    echo "返回"
    exit 0
  fi

  fb="none"
  if [ "${role}" = "master" ] && [ "${hw}" = "real" ]; then
    # 力反馈仅 master + 真机（仿真无力反馈）
    fb="$(ask_feedback)"
    if [ "${fb}" = "back" ]; then
      echo "返回"
      exit 0
    fi
  fi

  mj="false"
  if [ "${role}" = "master" ]; then
    mj="$(ask_movej_pub)"
    if [ "${mj}" = "back" ]; then
      echo "返回"
      exit 0
    fi
  fi

  usb="auto"
  if [ "${hw}" = "real" ]; then
    usb="$(ask_usb_select)"
    if [ "${usb}" = "back" ]; then
      echo "返回"
      exit 0
    fi
  fi

  local_label="$([ "${hw}" = "real" ] && echo "真机遥操作(${role})" || echo "仿真遥操作(${role})")"
  _run_teleop "${role}" "${hw}" "${mode}" "${fb}" "${mj}" "${usb}" "${local_label}"
  exit 0
fi

echo -e "${YELLOW}无效选项，请重新运行脚本${NC}"
exit 1
