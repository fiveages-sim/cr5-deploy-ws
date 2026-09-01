#!/usr/bin/env bash

# 快速启动脚本（Fairino ART7 Deploy Workspace）
# - 自动识别当前 workspace 路径（脚本所在目录）
# - 真机启动前检查 Fairino 控制器 TCP 连通性
# - 启动选项参考：README.md

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"

. "${SCRIPT_DIR}/scripts/lib_common.sh"

# ===================== 常量 / 默认参数（集中在此，方便查看与修改） =====================

# 启动历史记忆文件
HIST_FILE="${HOME}/.config/fairino/quick_start_history.json"
# 历史保留条数（最近 N 条不同配置）
HIST_MAX=2

# OCS2 启动包 / launch 文件 / 机器人名
OCS2_LAUNCH_PKG="ocs2_arm_controller"
OCS2_LAUNCH_FILE="demo.launch.py"
ROBOT_NAME="art7"

# Fairino 控制器 TCP 连接（fairino_ros2_control 硬件接口，默认 192.168.58.1:8081）
# 可用环境变量覆盖: FAIRINO_IP / FAIRINO_PORT
FAIRINO_IP="${FAIRINO_IP:-192.168.58.1}"
FAIRINO_PORT="${FAIRINO_PORT:-8081}"

# ===================== 核心包由 deb 提供时，src 下无 arms_ros2_control / ocs2_ros2 等 =====================
# Fairino 工作空间默认 arms-full（含 fairino_ros2_control）
core_deb_mode() {
  if dpkg-query -W -f='${Status}' ros-jazzy-arms-ros2-control-full 2>/dev/null | grep -q "install ok installed"; then
    return 0
  fi
  if dpkg-query -W -f='${Status}' ros-jazzy-arms-ros2-control 2>/dev/null | grep -q "install ok installed"; then
    return 0
  fi
  [ ! -d "${WS_DIR}/src/arms_ros2_control" ] && [ ! -d "${WS_DIR}/src/ocs2_ros2" ]
}

# arms-full 已提供 fairino_ros2_control，真机编译无需再编该包
fairino_from_deb() {
  dpkg-query -W -f='${Status}' ros-jazzy-arms-ros2-control-full 2>/dev/null | grep -q "install ok installed"
}

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
    # colcon 生成的 setup 脚本同样引用未定义变量（COLCON_TRACE 等），source 期间临时关闭
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

  if core_deb_mode; then
    echo -e "${YELLOW}[WARN] deb 已提供核心控制包；请先编译 Fairino 描述包：${NC}"
    echo -e "${YELLOW}      ./quick_start.sh → 1) 编译${NC}"
    return 1
  fi

  echo -e "${YELLOW}[WARN] 请先在此 workspace 编译，然后再运行启动选项：${NC}"
  echo -e "${YELLOW}      cd ${WS_DIR} && colcon build --symlink-install${NC}"
  return 1
}

# ===================== 启动历史记忆 =====================
# 历史文件条目用 kind 字段区分（保留结构以兼容旧文件）：
#   quick_start 条目: 无 kind 或 kind=="quick_start"
#   teleop 条目:      kind=="teleop"（旧版本遗留，本脚本忽略）
# 各脚本只统计/去重/重现自己 kind 的条目，互不干扰。

# 历史条数 (0/1/2)
history_count() {
  [ -f "${HIST_FILE}" ] || { echo 0; return 0; }
  python3 - "${HIST_FILE}" <<'PYEOF'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    d = []
if not isinstance(d, list):
    d = []
d = [e for e in d if e.get("kind", "quick_start") == "quick_start"]
print(len(d))
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
if not isinstance(d, list):
    d = []
d = [e for e in d if e.get("kind", "quick_start") == "quick_start"]
if idx < len(d):
    print(d[idx].get("desc", ""))
PYEOF
}

# 记录一次启动配置 (按 type/hardware/control_mode/drag/usb_select 去重,
# 保留最近 HIST_MAX 条不同配置；其他 kind 条目原样保留)
record_launch() {
  python3 - "${HIST_FILE}" "$1" "$2" "$3" "$4" "$5" "${HIST_MAX}" <<'PYEOF'
import json, os, sys, time
path, jtype, jhw, jcm, jdrag, jusb, hist_max = sys.argv[1:8]
hist_max = int(hist_max)
drag = (jdrag == "true")
entry = {
    "kind": "quick_start",
    "type": jtype,
    "hardware": jhw,
    "control_mode": jcm,
    "drag": drag,
    "usb_select": jusb,
    "ts": int(time.time()),
}
type_label = {"single": "单臂", "dual": "双臂", "left": "左臂", "right": "右臂"}.get(jtype, jtype)
parts = [type_label,
         "真机" if jhw == "real" else "仿真"]
if jcm:
    parts.append(jcm)
if drag:
    parts.append("拖动")
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
    return (e.get("type") == entry["type"] and e.get("hardware") == entry["hardware"]
            and e.get("control_mode", "") == entry["control_mode"]
            and bool(e.get("drag", False)) == drag
            and e.get("usb_select", "auto") == jusb)

# 分离本 kind 与其他 kind 条目；只对本 kind 去重并限条数
qs = [e for e in data if e.get("kind", "quick_start") == "quick_start"]
other = [e for e in data if e.get("kind", "quick_start") != "quick_start"]
qs = [e for e in qs if not same(e)]
qs.insert(0, entry)
qs = qs[:hist_max]
data = qs + other
os.makedirs(os.path.dirname(path), exist_ok=True)
with open(path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)
PYEOF
}

# 主菜单: $1 = 历史条数 (0/1/2); 有历史时选项 1..N 为历史启动, 回车默认选 1
menu() {
  local n="${1:-0}"
  local last d
  echo -e "${BLUE}========================================${NC}" >&2
  echo -e "${BLUE}  快速启动（Fairino ART7 Deploy Workspace）${NC}" >&2
  echo -e "${BLUE}  Workspace: ${WS_DIR}${NC}" >&2
  echo -e "${BLUE}========================================${NC}" >&2
  echo "" >&2
  echo "请选择操作:" >&2
  if [ "${n}" -ge 1 ]; then
    d="$(hist_desc_at 0)"
    echo " *1) 上次启动: ${d}" >&2
  fi
  if [ "${n}" -ge 2 ]; then
    d="$(hist_desc_at 1)"
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
  echo "  1) 编译仿真所需包" >&2
  echo "  2) 编译真机所需包" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]: " choice
  echo "${choice}"
}

launch_menu() {
  echo "" >&2
  echo "请选择启动项:" >&2
  echo " *1) 双臂 (dual)" >&2
  echo "  2) 左臂 (left)" >&2
  echo "  3) 右臂 (right)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3]（回车=默认 1）: " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  echo "${choice}"
}

launch_mode_menu() {
  echo "" >&2
  echo "请选择运行模式:" >&2
  echo " *1) 仿真" >&2
  echo "  2) 真机" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2]（回车=默认 1）: " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  echo "${choice}"
}

# Real hardware: check Fairino controller TCP reachability (fairino_ros2_control connects via TCP).
ensure_controller_reachable() {
  local ip="${FAIRINO_IP}" port="${FAIRINO_PORT}"

  echo -e "${BLUE}[INFO] 检查 Fairino 控制器 TCP 连通性 ${ip}:${port} ...${NC}"
  if timeout 3 bash -c "exec 3<>/dev/tcp/${ip}/${port}" 2>/dev/null; then
    echo -e "${GREEN}[INFO] 控制器可达${NC}"
    return 0
  fi

  echo -e "${RED}[ERROR] 无法连接 Fairino 控制器 ${ip}:${port}${NC}"
  echo -e "${YELLOW}      请检查网线连接、控制器供电，以及 fairino_ros2_control 的 device_ip 参数${NC}"
  echo -e "${YELLOW}      可用环境变量覆盖: FAIRINO_IP=<ip> FAIRINO_PORT=<port> ./quick_start.sh${NC}"
  read -r -p "仍要继续启动真机吗？[y/N]: " cont
  case "${cont}" in
    y|Y|yes|YES) return 0 ;;
    *) return 1 ;;
  esac
}

# 重现一条历史启动配置 (idx: 0=最近, 1=次近)
# 旧历史中的 control_mode/drag/usb_select 字段已不再使用（Fairino 仅 position 接口），仅解析 type/hardware
run_history_launch() {
  local idx="$1"
  local jtype jhw
  IFS=',' read -r jtype jhw _jcm _jdrag _jusb <<EOF
$(python3 - "${HIST_FILE}" "${idx}" <<'PYEOF'
import json, sys
path, idx = sys.argv[1], int(sys.argv[2])
try:
    d = json.load(open(path))
except Exception:
    d = []
if not isinstance(d, list):
    d = []
d = [e for e in d if e.get("kind", "quick_start") == "quick_start"]
if idx >= len(d):
    print("single,mock_components,,false,auto")
    sys.exit(0)
e = d[idx]
print(",".join([
    e.get("type", "single"),
    e.get("hardware", "mock_components"),
    e.get("control_mode", ""),
    "true" if e.get("drag") else "false",
    e.get("usb_select", "auto"),
]))
PYEOF
)
EOF
  local -a args=()
  args+=("type:=${jtype}")
  if [ "${jhw}" = "real" ]; then
    args+=("hardware:=real")
  fi
  local label
  label="$(hist_desc_at "${idx}")"
  _run_ocs2_demo "${label}" "${args[@]}"
}

# Launch ocs2 demo. Args: arm_label type_args...
# Records the launch config to history.
_run_ocs2_demo() {
  local arm_label="$1"
  shift 1
  local -a extra_args=("$@")
  local arg
  local jtype="single" jhw="mock_components"

  ensure_ros_env || exit 1

  for arg in "${extra_args[@]}"; do
    if [ "${arg}" = "hardware:=real" ]; then
      ensure_controller_reachable || exit 1
      break
    fi
  done

  # 解析本次启动配置并写入历史
  for arg in "${extra_args[@]}"; do
    case "${arg}" in
      type:=single) jtype="single" ;;
      type:=dual) jtype="dual" ;;
      type:=left) jtype="left" ;;
      type:=right) jtype="right" ;;
      hardware:=real) jhw="real" ;;
    esac
  done
  record_launch "${jtype}" "${jhw}" "" "false" "auto"

  echo -e "${GREEN}启动${arm_label}...${NC}"
  ros2 launch "${OCS2_LAUNCH_PKG}" "${OCS2_LAUNCH_FILE}" robot:=${ROBOT_NAME} "${extra_args[@]}"
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
      echo -e "${BLUE}  模式: 核心包 deb + 仅编译 Fairino 描述包${NC}"
      if ! run_colcon_build --packages-select art7_description --symlink-install; then
        echo -e "${YELLOW}编译过程中出现错误${NC}"
        exit 1
      fi
    else
      if ! run_colcon_build --packages-up-to \
        ocs2_arm_controller \
        art7_description \
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
      if fairino_from_deb; then
        echo -e "${BLUE}  模式: 核心包 deb(arms-full 含 fairino_ros2_control) + 仅编译 Fairino 描述包${NC}"
        if ! run_colcon_build --packages-select art7_description --symlink-install; then
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
        fi
      else
        echo -e "${BLUE}  模式: 核心包 deb + 编译 Fairino 描述与真机驱动${NC}"
        if ! run_colcon_build --packages-select art7_description fairino_ros2_control --symlink-install; then
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
        fi
      fi
    else
      if ! run_colcon_build --packages-up-to \
        fairino_ros2_control \
        ocs2_arm_controller \
        art7_description \
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
      1|2|3)
        # art7 xacro 的 type: dual=双臂 / left=左臂 / right=右臂；
        # 若不传 type，robot_common_launch 会把 OCS2 planning URDF
        # 默认成 dual（14 DOF），与单臂 7 关节控制器不匹配。
        case "${launch_choice}" in
          1) arm_label="双臂"; type_arg="type:=dual" ;;
          2) arm_label="左臂"; type_arg="type:=left" ;;
          3) arm_label="右臂"; type_arg="type:=right" ;;
        esac

        mode_choice="$(launch_mode_menu)"

        case "${mode_choice}" in
          1)
            # 仿真
            _run_ocs2_demo "${arm_label}仿真" "${type_arg}"
            ;;
          2)
            # 真机：检查 Fairino 控制器 TCP 连通性后直接启动（fairino_ros2_control 走 TCP）
            _run_ocs2_demo "${arm_label}真机" "${type_arg}" "hardware:=real"
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
    exit 0
fi

echo -e "${YELLOW}无效选项，请重新运行脚本${NC}"
exit 1
