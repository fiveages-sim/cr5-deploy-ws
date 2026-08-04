#!/usr/bin/env bash

# 快速启动脚本（ARX Lift2S / ACone Deploy Workspace）
# 架构参考：https://github.com/fiveages-sim/open-deploy-ws/tree/dobot-cr5
# - 菜单：编译 / 启动（仿真 vs 真机）
# - Lift2S 真机：Stanford 臂（full_control | position）+ 升降 position/soft_p|hybrid；无 conda
# - 单/双臂桌面：可选 arx_ros2_control（Stanford）

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIFT2S_HI_DIR="${WS_DIR}/src/arms_ros2_control/hardwares/arxlift2s_ros2_control"
SDK_DIR="${WS_DIR}/src/arx-ros2-control/external/arx5-sdk"

need_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 缺少命令：$cmd${NC}"
    return 1
  fi
  return 0
}

ensure_ros_env() {
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_DIR}/install/setup.bash"
    return 0
  fi
  echo -e "${YELLOW}[WARN] 未找到 ${WS_DIR}/install/setup.bash${NC}"
  echo -e "${YELLOW}      请先选择「编译」生成 install/，再启动。${NC}"
  return 1
}

print_can_hint() {
  echo -e "${BLUE}真机 CAN 提示（Lift2S）：${NC}"
  echo -e "${BLUE}  - 左臂 can1 / 右臂 can3（Stanford ArxX5Hardware：full_control|position）${NC}"
  echo -e "${BLUE}  - 升降 can5（ArxLiftHardware：position/soft_p | hybrid；分体/全身启动均可选）${NC}"
  echo -e "${BLUE}  - 升降 hybrid：只用 HI 重力前馈，忽略 OCS2 effort（防双重前馈）${NC}"
  echo -e "${BLUE}  - 臂热调：ros2 param set /arx_lift2s_left_system control_mode position${NC}"
  echo -e "${BLUE}  - 臂位置增益：ros2 param set /arx_lift2s_left_system joint_k_gains_position \"[80,...]\"${NC}"
  echo -e "${BLUE}  - 臂位置阻尼：ros2 param set /arx_lift2s_left_system joint_d_gains_position \"[2,...]\"${NC}"
  echo -e "${BLUE}  - 升降热调：ros2 param set /controller_manager arx_lift.hybrid_kp 6.0${NC}"
  echo -e "${BLUE}  - 升降重力：ros2 param set /controller_manager arx_lift.gravity_compensation_torque -1.01${NC}"
  echo -e "${BLUE}  - 升降摩擦：ros2 param set /controller_manager arx_lift.coulomb_friction_torque 0.32${NC}"
  echo -e "${BLUE}  - 调试日志（默认关）：ros2 param set /arx_lift2s_lift_system status_debug true${NC}"
  echo -e "${BLUE}  - OCS2：分体 task_arm.info / 全身 fixed_base.info${NC}"
  echo -e "${BLUE}  - 勿与官方 X5Controller / lift_controller 同总线并行${NC}"
  echo -e "${BLUE}  - 检查：ip link show can1 can3 can5${NC}"
}

# ---------------------------------------------------------------------------
# Lift2S HI external（与 init_repo.sh 一致；编译前再确保一次）
# ---------------------------------------------------------------------------
soem_arch_dir() {
  case "$(uname -m)" in
    aarch64|arm64|armv7l|armv6l) echo "aarch64" ;;
    *) echo "x86_64" ;;
  esac
}

ensure_lift2s_external() {
  local pkg="${LIFT2S_HI_DIR}"
  local ext="${pkg}/external"
  local sdk_src="${SDK_DIR}"
  local soem_lib="${ext}/SOEM/lib/$(soem_arch_dir)/libsoem.so"

  if [ ! -d "$pkg" ]; then
    echo -e "${RED}[ERROR] 未找到 ${pkg}${NC}"
    echo -e "${YELLOW}      请先运行 ./init_repo.sh${NC}"
    return 1
  fi

  mkdir -p "$ext"

  if [ ! -e "${ext}/arx5-sdk/include/app/joint_controller.h" ]; then
    if [ -f "${sdk_src}/include/app/joint_controller.h" ]; then
      rm -f "${ext}/arx5-sdk"
      ln -sfn ../../../../arx-ros2-control/external/arx5-sdk "${ext}/arx5-sdk"
      echo -e "${GREEN}[INFO] 已链接 external/arx5-sdk${NC}"
    else
      echo -e "${RED}[ERROR] 缺少 Stanford SDK：${sdk_src}${NC}"
      return 1
    fi
  fi

  # Prebuilt libsoem.so only (no SOEM source clone)
  if [ ! -f "${soem_lib}" ]; then
    echo -e "${RED}[ERROR] 缺少预编译 SOEM：${soem_lib}${NC}"
    echo -e "${YELLOW}      见 arxlift2s_ros2_control/external/SOEM/README.md${NC}"
    return 1
  fi
  echo -e "${GREEN}[INFO] external/SOEM/lib/$(soem_arch_dir)/libsoem.so 已就绪${NC}"
  return 0
}

# panthera-ht 对齐：桌面单/双臂与 Lift2S 臂均可选 control_mode（可启动时选，也可运行时热调）
control_mode_menu() {
  echo "" >&2
  echo "请选择真机臂控制模式（full_control / position；运行中也可 ros2 param set）：" >&2
  echo "  1) full_control — OCS2 MIX（位置+速度+力矩；MIT kp/kd=HI joint_k/d_gains）" >&2
  echo "  2) position     — 真机位置环（仅 position；vel/torque=0；可设 position_forward_effort=true 保留 effort 前馈）" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-2] (默认 1): " choice
  if [ -z "${choice}" ]; then
    choice="1"
  fi
  echo "${choice}"
}

resolve_control_mode() {
  case "$1" in
    1) echo "full_control" ;;
    2) echo "position" ;;
    0) echo "" ;;
    *) echo "INVALID" ;;
  esac
}

# $1=描述 $2=launch 包+文件 $3=基础参数
# $5=0 不问 / 1 询问 control_mode（桌面单双臂与 Lift2S 臂）
# $6=0 不问 / 1 询问 lift_motor_mode（Lift2S 升降软启动/混合 MIT）
do_launch() {
  local description="$1"
  local launch_file="$2"
  local base_args="$3"
  local mode_choice="${4:-}"
  local ask_control_mode="${5:-0}"
  local ask_lift_mode="${6:-0}"
  local control_mode_arg=""
  local lift_motor_mode_arg=""
  local mode_label=""

  if [ -z "${mode_choice}" ]; then
    mode_choice="$(launch_mode_menu)"
  fi

  # 真机 / 真机 headless
  if { [ "${mode_choice}" = "1" ] || [ "${mode_choice}" = "2" ]; }; then
    if [ "${ask_control_mode}" = "1" ]; then
      local cm_choice control_mode
      cm_choice="$(control_mode_menu)"
      control_mode="$(resolve_control_mode "${cm_choice}")"
      if [ "${control_mode}" = "INVALID" ]; then
        echo -e "${YELLOW}无效选项${NC}"
        exit 1
      fi
      if [ -z "${control_mode}" ]; then
        echo "返回"
        return 0
      fi
      control_mode_arg="xacro_control_mode:=${control_mode}"
      mode_label="，臂控制模式=${control_mode}"
    fi

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
      mode_label="${mode_label}，升降模式=${lift_motor_mode}"
    fi
  fi

  case "${mode_choice}" in
    1)
      echo -e "${GREEN}启动${description}（真机${mode_label}）...${NC}"
      ensure_ros_env || exit 1
      print_can_hint
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=real ${control_mode_arg} ${lift_motor_mode_arg}
      ;;
    2)
      echo -e "${GREEN}启动${description}（真机 headless${mode_label}）...${NC}"
      ensure_ros_env || exit 1
      print_can_hint
      # shellcheck disable=SC2086
      ros2 launch ${launch_file} ${base_args} hardware:=real launch_mode:=control_only ${control_mode_arg} ${lift_motor_mode_arg}
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

# 对齐 dobot-cr5：仿真 / 真机 两档为主；保留桌面臂独立真机包
build_menu() {
  echo "" >&2
  echo "请选择编译目标:" >&2
  echo "  1) 编译仿真所需包 (Simulation)" >&2
  echo "  2) 编译 Lift2S 真机包 (Real — Stanford 臂 + Hybrid 升降，推荐验证)" >&2
  echo "  3) 编译单/双臂桌面真机包 (arx5 / arx_acone — arx_ros2_control)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3]: " choice
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

# 升降 position(soft_p) / Hybrid MIT
lift_motor_mode_menu() {
  echo "" >&2
  echo "请选择真机升降控制模式（position / hybrid；运行中也可 ros2 param set）：" >&2
  echo "  1) position — Soft-P setHeight/loop（只用 position，忽略 vel/effort）" >&2
  echo "  2) hybrid   — sendLiftHybrid（MIT；HI 重力前馈，忽略上层 effort）" >&2
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
    1) echo "soft_p" ;;   # menu「position」→ xacro soft_p
    2) echo "hybrid" ;;
    0) echo "" ;;
    *) echo "INVALID" ;;
  esac
}

# 桌面 arx_ros2_control 仍可能走旧 conda SDK（可选路径）
build_arx_sdk_legacy() {
  echo -e "${YELLOW}[INFO] 预编译 Stanford SDK（arx_ros2_control 旧路径，需 conda）...${NC}"
  if ! command -v conda >/dev/null 2>&1; then
    echo -e "${YELLOW}[WARN] 未找到 conda；若 arx_ros2_control 已能 colcon 自建 SDK 可忽略。${NC}"
    return 0
  fi
  if [ ! -d "${SDK_DIR}" ]; then
    echo -e "${RED}[ERROR] 未找到 ${SDK_DIR}${NC}"
    return 1
  fi

  local conda_base
  conda_base=$(conda info --base 2>/dev/null)
  # shellcheck disable=SC1090
  source "${conda_base}/etc/profile.d/conda.sh"

  if ! conda env list | grep -qE "^arx-py312[[:space:]]"; then
    echo -e "${YELLOW}[INFO] 创建 conda 环境 arx-py312...${NC}"
    if command -v mamba >/dev/null 2>&1; then
      mamba env create -f "${SDK_DIR}/conda_environments/py312_environment.yaml" || return 1
    else
      conda env create -f "${SDK_DIR}/conda_environments/py312_environment.yaml" || return 1
    fi
  fi

  echo -e "${YELLOW}[INFO] 系统 GCC 编译 libArxJointController.so...${NC}"
  (
    ros_distro="${ROS_DISTRO:-jazzy}"
    [ -f "/opt/ros/${ros_distro}/setup.bash" ] && source "/opt/ros/${ros_distro}/setup.bash"
    pybind11_dir=$(conda run -n arx-py312 python3 -c \
      "import pybind11; print(pybind11.get_cmake_dir())" 2>/dev/null || true)
    cmake_args=(-DCMAKE_CXX_COMPILER=/usr/bin/g++ -DCMAKE_C_COMPILER=/usr/bin/gcc)
    if [ -n "${pybind11_dir}" ]; then
      cmake_args+=("-Dpybind11_DIR=${pybind11_dir}")
    fi
    mkdir -p "${SDK_DIR}/build"
    cd "${SDK_DIR}/build" || exit 1
    cmake .. "${cmake_args[@]}" || exit 1
    make -j"$(nproc)" ArxJointController ArxCartesianController || exit 1
  ) || return 1
  echo -e "${GREEN}[INFO] Stanford SDK（legacy）编译完成${NC}"
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
        cd "${WS_DIR}" || exit 1
        colcon build --packages-up-to \
          ocs2_arm_controller \
          arx_acone_description \
          arx_lift2s_description \
          component_models \
          arx5_description \
          arms_teleop \
          adaptive_gripper_controller \
          basic_joint_controller \
          --symlink-install
        if [ $? -eq 0 ]; then
          echo -e "${GREEN}编译完成！${NC}"
        else
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
        fi
        ;;
      2)
        echo -e "${GREEN}开始编译 Lift2S 真机包（Stanford 臂 + Hybrid 升降，无 conda）...${NC}"
        ensure_lift2s_external || exit 1
        cd "${WS_DIR}" || exit 1
        colcon build --packages-up-to \
          arxlift2s_ros2_control \
          ocs2_arm_controller \
          arx_acone_description \
          arx_lift2s_description \
          component_models \
          arx5_description \
          arms_teleop \
          adaptive_gripper_controller \
          basic_joint_controller \
          --symlink-install
        if [ $? -eq 0 ]; then
          echo -e "${GREEN}编译完成！${NC}"
          echo -e "${BLUE}启动建议：Launch → 分体/全身 → 真机（臂 full_control/position；升降 position/hybrid）${NC}"
          echo -e "${BLUE}  OCS2：分体 task_arm.info / 全身 fixed_base.info（arx_lift2s_description/config/ocs2）${NC}"
          echo -e "${BLUE}  升降 hybrid：HI 重力+摩擦前馈，忽略 OCS2 effort；kp/kd/τ_ff：arx_lift.*${NC}"
          print_can_hint
        else
          echo -e "${YELLOW}编译失败。检查 external/arx5-sdk 与 external/SOEM/lib/<arch>/libsoem.so（见 init_repo / 包 README）。${NC}"
          exit 1
        fi
        ;;
      3)
        echo -e "${GREEN}开始编译单/双臂桌面真机包（arx_ros2_control）...${NC}"
        build_arx_sdk_legacy || exit 1
        cd "${WS_DIR}" || exit 1
        colcon build --packages-up-to \
          arx_ros2_control \
          ocs2_arm_controller \
          arx_acone_description \
          arx5_description \
          arms_teleop \
          adaptive_gripper_controller \
          basic_joint_controller \
          --symlink-install
        if [ $? -eq 0 ]; then
          echo -e "${GREEN}编译完成！${NC}"
        else
          echo -e "${YELLOW}编译过程中出现错误${NC}"
          exit 1
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

  2)
    launch_choice="$(launch_menu)"
    case "${launch_choice}" in
      1)
        do_launch "单臂 X5" "ocs2_arm_controller demo.launch.py" "robot:=arx5" "" "1"
        ;;
      2)
        do_launch "双臂 ACone" "ocs2_arm_controller demo.launch.py" "robot:=arx_acone" "" "1"
        ;;
      3)
        # 分体：双臂 OCS2 + body_joint 升降（可选 position/soft_p | hybrid）
        do_launch "Lift2S 分体控制" "ocs2_arm_controller split_body.launch.py" "robot:=arx_lift2s" "" "1" "1"
        ;;
      4)
        # 全身 WBC：臂可选 full_control/position；升降可选 position(soft_p)/hybrid（默认 hybrid）
        do_launch "Lift2S 全身控制" \
          "ocs2_arm_controller full_body.launch.py" \
          "robot:=arx_lift2s" "" "1" "1"
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
