#!/usr/bin/env bash

# 快速启动脚本（CR5 Deploy Workspace）
# - 自动识别当前 workspace 路径（脚本所在目录）
# - 启动选项参考：README.md / cr5_description & cr5_dual_description 的 “Real Dobot CR5 Deploy”

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

need_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 缺少命令：$cmd${NC}"
    return 1
  fi
  return 0
}

ensure_ros_env() {
  # 启动类命令需要 ROS 环境
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_DIR}/install/setup.bash"
    return 0
  fi

  echo -e "${YELLOW}[WARN] 未找到 ${WS_DIR}/install/setup.bash${NC}"
  echo -e "${YELLOW}      请先在此 workspace 编译，然后再运行启动选项：${NC}"
  echo -e "${YELLOW}      cd ${WS_DIR} && colcon build --symlink-install${NC}"
  return 1
}

print_ip_hint() {
  echo -e "${BLUE}提示：CR5 机器人 IP 需要与你的现场网络一致${NC}"
  echo -e "${BLUE}- cr5_description bringup 默认 IP 在：src/robot-descriptions-dobot/cr5_description/launch/dobot_bringup_ros2.launch.py${NC}"
  echo -e "${BLUE}- ros2_control 的 robot_ip 通常在 xacro/ros2_control 配置中（例如 cr5_description/xacro/ros2_control/robot.xacro）${NC}"
}

menu() {
  echo -e "${BLUE}========================================${NC}" >&2
  echo -e "${BLUE}  快速启动（CR5 Deploy Workspace）${NC}" >&2
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
  echo "  1) 单臂 (CR5)" >&2
  echo "  2) 双臂 (CR5 Dual)" >&2
  echo "  3) 单臂 vla39 (CR5 vla39)" >&2
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
  echo "  3) 真机 + HTTP Bridge (Real Hardware + HTTP Bridge bringup)" >&2
  echo "  0) 返回" >&2
  echo "" >&2
  read -r -p "请输入选项 [0-3]: " choice
  echo "${choice}"
}

need_cmd git || exit 1
need_cmd colcon || echo -e "${YELLOW}[WARN] 未找到 colcon，编译选项会失败（通常需要安装 ROS 发行版环境）。${NC}"

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
      cr5_description \
      cr5_dual_description \
      arms_teleop \
      adaptive_gripper_controller \
      vla_http_bridge \
      --symlink-install
    if [ $? -eq 0 ]; then
      echo -e "${GREEN}编译完成！${NC}"
    else
      echo -e "${YELLOW}编译过程中出现错误${NC}"
      exit 1
    fi
    ;;

      2)
    echo -e "${GREEN}开始编译真机所需包...${NC}"
    cd "${WS_DIR}" || exit 1
    echo -e "${YELLOW}[INFO] 参考：cr5_dual_description/README.md 的 “Real Dobot CR5 Deploy”。${NC}"
    colcon build --packages-up-to \
      dobot_ros2_control \
      ocs2_arm_controller \
      cr5_description \
      cr5_dual_description \
      arms_teleop \
      adaptive_gripper_controller \
      vla_http_bridge \
      --symlink-install
    if [ $? -eq 0 ]; then
      echo -e "${GREEN}编译完成！${NC}"
    else
      echo -e "${YELLOW}编译过程中出现错误（如果提示找不到 cr_robot_ros2，请先安装/拉取该包）${NC}"
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
        robot="cr5"
        mode_choice="$(launch_mode_menu)"
        case "${mode_choice}" in
          1)
            echo -e "${GREEN}启动单臂仿真（CR5）...${NC}"
            ensure_ros_env || exit 1
            ros2 launch ocs2_arm_controller demo.launch.py robot:="${robot}" type:=AG2F90-C-Soft
            ;;
          2)
            echo -e "${GREEN}启动单臂真机（CR5）...${NC}"
            ensure_ros_env || exit 1
            print_ip_hint
            ros2 launch ocs2_arm_controller demo.launch.py robot:="${robot}" hardware:=real type:=AG2F90-C-Soft
            ;;
          3)
            echo -e "${GREEN}启动单臂真机（CR5，带HTTP Bridge）...${NC}"
            ensure_ros_env || exit 1
            print_ip_hint
            ros2 launch cr5_description dobot_bringup_ros2.launch.py robot:="${robot}" hardware:=real type:=AG2F90-C-Soft
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
        robot="cr5_dual"
        mode_choice="$(launch_mode_menu)"
        case "${mode_choice}" in
          1)
            echo -e "${GREEN}启动双臂仿真（CR5 Dual）...${NC}"
            ensure_ros_env || exit 1
            ros2 launch ocs2_arm_controller demo.launch.py robot:="${robot}" type:=AG2F90-C-Soft
            ;;
          2)
            echo -e "${GREEN}启动双臂真机（CR5 Dual）...${NC}"
            ensure_ros_env || exit 1
            print_ip_hint
            echo -e "${YELLOW}[INFO] 默认夹爪类型：AG2F90-C-Soft（如需修改请到命令行手动改 type:=...）${NC}"
            ros2 launch ocs2_arm_controller demo.launch.py robot:="${robot}" hardware:=real type:=AG2F90-C-Soft
            ;;
          3)
            echo -e "${GREEN}启动双臂真机（CR5 Dual，带HTTP Bridge）...${NC}"
            ensure_ros_env || exit 1
            print_ip_hint
            echo -e "${YELLOW}[INFO] 默认夹爪类型：AG2F90-C-Soft（如需修改请到命令行手动改 type:=...）${NC}"
            ros2 launch cr5_description dobot_bringup_ros2.launch.py robot:="${robot}" hardware:=real type:=AG2F90-C-Soft
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
        robot="cr5"
        mode_choice="$(launch_mode_menu)"
        case "${mode_choice}" in
          1)
            echo -e "${GREEN}启动单臂仿真（CR5 vla39）...${NC}"
            ensure_ros_env || exit 1
            ros2 launch ocs2_arm_controller demo.launch.py robot:="${robot}" type:=vla39
            ;;
          2)
            echo -e "${GREEN}启动单臂真机（CR5 vla39）...${NC}"
            ensure_ros_env || exit 1
            print_ip_hint
            ros2 launch ocs2_arm_controller demo.launch.py robot:="${robot}" hardware:=real type:=vla39
            ;;
          3)
            echo -e "${GREEN}启动单臂真机（CR5 vla39，带HTTP Bridge）...${NC}"
            ensure_ros_env || exit 1
            print_ip_hint
            ros2 launch cr5_description dobot_bringup_ros2.launch.py robot:="${robot}" hardware:=real type:=vla39
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
