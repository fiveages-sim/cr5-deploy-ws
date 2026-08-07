# ARX Lift2S / ACone 机械臂 ROS2 部署工作空间

![ARX Lift2S](.images/arx_lift2s.jpg)

本仓库用于部署 ARX Lift2S（含升降）与 ACone / X5 机械臂的 ROS 2 工作空间，基于 OCS2 MPC 控制框架。

## 目录

1. [前置条件](#1-前置条件)
2. [部署](#2-部署)
   - [2.1 克隆与子模块](#21-克隆与子模块)
   - [2.2 之后如何更新子模块](#22-之后如何更新子模块)
   - [2.3 目录结构（节选）](#23-目录结构节选)
   - [2.4 常见问题](#24-常见问题)
   - [2.5 发布包（`release.sh`）](#25-发布包releasesh)
3. [环境配置（RMW Zenoh）](#3-环境配置rmw-zenoh)
4. [编译](#4-编译)
   - [4.1 依赖安装](#41-依赖安装)
   - [4.2 程序编译（推荐：`quick_start.sh`）](#42-程序编译推荐quick_startsh)
5. [启动（请用 quick_start）](#5-启动请用-quick_start)
   - [5.1 仿真 / 可视化](#51-仿真--可视化)
   - [5.2 真机要点](#52-真机要点)
6. [子模块说明](#6-子模块说明)

## 1. 前置条件

- 已配置 Git SSH 密钥并可访问相关私有仓库
- 系统已安装 Git（建议 2.30+）
- ROS 2 Jazzy（Ubuntu 24.04）

## 2. 部署

### 2.1 克隆与子模块

#### 克隆到 ~/lift2s-ws

```bash
cd ~
git clone git@github.com:fiveages-sim/open-deploy-ws.git lift2s-ws
cd ~/lift2s-ws
```

#### 初始化并更新子模块

```bash
./init_repo.sh
```

脚本会：同步顶层子模块 → 切到配置分支最新提交 → 初始化 `ocs2_ros2` 嵌套子模块（如有）→ 检查 `arx-ros2-control/external` 依赖。

### 2.2 之后如何更新子模块

```bash
git submodule update --remote
```

### 2.3 目录结构（节选）

```
src/
  ├─ arms_ros2_control              # 控制器 / 遥操作 / 公共控制栈
  ├─ arx-ros2-control               # 真机 HI（包名 arx_ros2_control：臂 + 升降）
  ├─ ocs2_ros2                      # OCS2 ROS2（含嵌套子模块）
  ├─ robot-descriptions-arx         # arx5 / arx_acone / arx_lift2s 描述
  └─ robot-descriptions-common      # 通用 launch / 夹爪等
```

### 2.4 常见问题

- SSH 权限：确认本机 SSH key 已添加到 GitHub，`ssh -T git@github.com` 可握手。
- 网络问题：可重试或改用代理；必要时改为 HTTPS 克隆。

### 2.5 发布包（`release.sh`）

机制对齐 [panthera-ht](https://github.com/fiveages-sim/open-deploy-ws/tree/panthera-ht)：核心栈用 deb，描述与真机 HI 保留源码。

**现场：**

```bash
# 解压 dist/lift2s_ws_*.zip 后
./release.sh --install    # 装 .deb_cache/（缺包时先 --download）
./quick_start.sh          # 编译描述 + arx_ros2_control，再启动
```

**维护者打包（临时目录，不改当前工作区）：**

```bash
./release.sh --package
./release.sh --package-no-git --arch amd64
./release.sh --package-no-git --arch arm64
```

保留源码：`src/robot-descriptions-arx`、`src/arx-ros2-control`；其余子模块占位，由 `deb_versions.conf` 中的 ocs2 / common / arms deb 提供。更细说明见 [`arx_lift2s_description/README.md`](src/robot-descriptions-arx/arx_lift2s_description/README.md) §5。

## 3. 环境配置（RMW Zenoh）

部署机器建议使用 RMW Zenoh，避免局域网 DDS 互相污染。

* 安装
  ```bash
  sudo apt install ros-jazzy-rmw-zenoh-cpp
  ```
* 配置 bashrc
  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ```
* 临时取消：`unset RMW_IMPLEMENTATION`
* **先起 Zenoh router，再 launch**（否则 `controller_manager` 会卡在
  `Waiting for data on 'robot_description' topic`）
  - 推荐：`./quick_start.sh` → Launch（会自动预启 router）
  - 手动 `ros2 launch` 时先另开终端：`ros2 run rmw_zenoh_cpp rmw_zenohd`

## 4. 编译

### 4.1 依赖安装

```bash
cd ~/lift2s-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4.2 程序编译（推荐：`quick_start.sh`）

```bash
cd ~/lift2s-ws
chmod +x ./init_repo.sh ./quick_start.sh
./init_repo.sh
./quick_start.sh
```

- **`1) 编译 (Build)`**
  - **`1) 仿真所需包`**：不依赖真机驱动
  - **`2) 真机所需包`**：编译 `arx_ros2_control` + 描述与控制栈（单臂 / 双臂 / Lift2S 共用）

<details>
<summary><strong>（可选）手动编译命令</strong></summary>

```bash
cd ~/lift2s-ws
# 仿真
colcon build --packages-up-to \
  ocs2_arm_controller \
  arx_acone_description \
  arx_lift2s_description \
  arx5_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install

# 真机（需 src/arx-ros2-control/external/{arx5-sdk,arx_lift_src}）
colcon build --packages-up-to \
  arx_ros2_control \
  ocs2_arm_controller \
  arx_acone_description \
  arx_lift2s_description \
  arx5_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

</details>

## 5. 启动（请用 quick_start）

### 5.1 仿真 / 可视化

```bash
source ~/lift2s-ws/install/setup.bash
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_acone
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_lift2s
```

推荐用 `./quick_start.sh` → **`2) 启动`**：

| 选项 | 说明 |
|------|------|
| 1) 单臂 X5 | `robot:=arx5`；**真机时再选左臂(can1) / 右臂(can3)** |
| 2) 双臂 ACone | `robot:=arx_acone`（can1 + can3） |
| 3) Lift2S 分体 | `split_body.launch.py`；真机可选升降 hybrid / soft_p |
| 4) Lift2S 全身 | `full_body.launch.py`；同上 |

运行模式：真机 / 真机 headless / 仿真 / 仿真 headless / Isaac / Isaac headless / 仅可视化。

### 5.2 真机要点

**臂控制模式：仅 `full_control`（MIT MIX）**

- 电机层始终是 MIT 阻抗（`kp, kd, pos, vel, torque`），部署脚本**不再提供**臂 `position` 菜单。
- 单臂真机：`xacro_can_interface:=can1`（左）或 `can3`（右）。
- 双臂 / Lift2S：左右固定 can1 / can3。

**升降（仅 Lift2S）**

- `hybrid`（默认）：`sendLiftHybrid`，跟踪 pos+vel，HI 重力/摩擦前馈。
- `soft_p` / `position`：Soft-P `setHeight`，仅跟踪 position。

```bash
./quick_start.sh
# Build → 2) 真机包
# Launch → 1) 单臂 → 真机 → 选左/右
# Launch → 3/4) Lift2S → 真机 → 升降模式（默认 hybrid）
```

<details>
<summary><strong>（可选）手动启动真机</strong></summary>

```bash
source ~/lift2s-ws/install/setup.bash

# 单臂左 / 右
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real xacro_can_interface:=can1
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real xacro_can_interface:=can3

# 双臂
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_acone hardware:=real

# Lift2S 分体 / 全身（升降默认 hybrid）
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s hardware:=real
ros2 launch ocs2_arm_controller full_body.launch.py robot:=arx_lift2s hardware:=real \
  xacro_lift_motor_mode:=hybrid
```

</details>

## 6. 子模块说明

- **arms_ros2_control** — 机械臂通用 ROS2 控制（OCS2 控制器、遥操作等）
- **arx-ros2-control** — Lift2S / ACone / X5 真机硬件接口（`ArxX5Hardware` + `ArxLiftHardware`）
- **ocs2_ros2** — OCS2 的 ROS2 版本（MPC）
- **robot-descriptions-arx** — ARX 机型描述（`arx5` / `arx_acone` / `arx_lift2s`）
- **robot-descriptions-common** — 通用组件与 launch
