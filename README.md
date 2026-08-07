# ARX Lift2S / ACone 机械臂 ROS2 部署工作空间

![ARX Lift2S](.images/arx_lift2s.jpg)

本仓库用于部署 ARX Lift2S（含升降）与 ACone / X5 机械臂的 ROS 2 工作空间，基于 OCS2 MPC 控制框架。

## 目录

1. [前置条件](#1-前置条件)
2. [部署](#2-部署)
   - [2.1 方式概览](#21-方式概览)
   - [2.2 发布包部署（deb，推荐现场）](#22-发布包部署deb推荐现场)
   - [2.3 完整源码：克隆与子模块](#23-完整源码克隆与子模块)
3. [环境配置（RMW Zenoh）](#3-环境配置rmw-zenoh)
4. [编译](#4-编译)
   - [4.1 依赖安装](#41-依赖安装)
   - [4.2 程序编译（推荐：`quick_start.sh`）](#42-程序编译推荐quick_startsh)
5. [启动（请用 quick_start）](#5-启动请用-quick-start)
   - [5.1 仿真 / 可视化](#51-仿真--可视化)
   - [5.2 真机要点](#52-真机要点)
6. [子模块说明](#6-子模块说明)

## 1. 前置条件

- 已配置 Git SSH 密钥并可访问相关私有仓库
- 系统已安装 Git（建议 2.30+）
- ROS 2 Jazzy（Ubuntu 24.04）

## 2. 部署

### 2.1 方式概览

| 方式 | 适用场景 | 主要步骤 |
|------|----------|----------|
| **发布包（推荐现场）** | 机器人本体 / 产线，控制器由 deb 提供 | 解压 zip → `release.sh --install` → `quick_start.sh` |
| **完整源码** | 开发、调试、改控制器栈 | `init_repo.sh` → 编译子模块 → `quick_start.sh` |

发布包内已包含 **`robot-descriptions-arx`、`arx-ros2-control`** 源码，以及预下载的 deb（ocs2、robot-descriptions-common、arms-ros2-control）。其余子模块目录在打包时已清空，由 deb 安装到系统。真机 HI（`arx_ros2_control`）不在 arms deb 内，故随发布包保留源码。

#### 目录结构（节选）

```
lift2s-ws/
  ├─ init_repo.sh              # 子模块初始化
  ├─ release.sh                # deb 安装、发布打包（维护者）
  ├─ quick_start.sh            # 编译与启动
  ├─ config/quick_start.conf   # RELEASE_SUBMODULE_PATHS 等
  ├─ .deb_cache/               # 发布包内预置 deb（解压后安装）
  └─ src/
      ├─ robot-descriptions-arx      # 发布包保留源码
      ├─ arx-ros2-control            # 发布包保留源码（真机 HI）
      ├─ robot-descriptions-common   # 发布包中由 deb 提供
      ├─ arms_ros2_control           # 发布包中由 deb 提供
      └─ ocs2_ros2                   # 发布包中由 deb 提供
```

完整源码开发时，`arms_ros2_control` / `ocs2_ros2` 下还可有嵌套子模块。

### 2.2 发布包部署（deb，推荐现场）

从维护者处获取 `lift2s-ws_*.zip` 后，在目标机器上：

```bash
unzip lift2s-ws_*.zip -d ~
cd ~/lift2s-ws

# 1) 安装预置 deb（需 sudo；若 .deb_cache/ 为空或需更新，先 ./release.sh --download）
./release.sh --install
source /opt/ros/jazzy/setup.bash   # 若尚未写入 shell 配置

# 2) 编译工作区内源码，再启动
./quick_start.sh
#   Build → 1) 仿真：仅 ARX 描述包
#         → 2) 真机：描述 + arx_ros2_control（HI）
#   （已装 arms/ocs2 deb 时自动走上述「deb 模式」，不会再编占位的控制器源码）
#   Launch → Lift2S 分体 / 全身 等
```

`release.sh --install` 会按顺序安装 `.deb_cache/` 中的 deb：

1. `ros-jazzy-ocs2`（[legubiao/ocs2_ros2](https://github.com/legubiao/ocs2_ros2/releases)）
2. `ros-jazzy-robot-descriptions-common`（[fiveages-sim/robot-descriptions-common](https://github.com/fiveages-sim/robot-descriptions-common/releases)）
3. `ros-jazzy-arms-ros2-control`（[fiveages-sim/arms_ros2_control](https://github.com/fiveages-sim/arms_ros2_control/releases)，standard；不含 arx HI）

现场若需换 deb 通道（默认打包为 **latest**），可：

```bash
./release.sh --download --release-channel prerelease
./release.sh --install
```

#### 部署后更新（git + deb）

发布 zip **包含主仓 `.git`** 时，可在现场拉取脚本与配置；大组件仍通过 deb 更新。

```bash
cd ~/lift2s-ws
git pull --ff-only
git submodule update --init -- src/robot-descriptions-arx src/arx-ros2-control
./release.sh --download
./release.sh --install
./quick_start.sh   # Build → 真机/仿真（deb 模式）后启动
```

由 deb 提供的子模块目录在发布包中为占位（`.gitkeep`）；若需完整源码开发，见下方 [2.3](#23-完整源码克隆与子模块)。

<details>
<summary><strong>维护者：生成发布 zip</strong></summary>

在开发机上（需 git、rsync 与网络）。打包在**临时目录**进行，**不修改**当前工作区。保留子模块由 `config/quick_start.conf` → `RELEASE_SUBMODULE_PATHS` 决定（当前为 `robot-descriptions-arx`、`arx-ros2-control`）。

```bash
cd ~/lift2s-ws
./release.sh --package                                    # 含 .git，deb 架构=本机，channel=latest
./release.sh --package-no-git --arch amd64                # 不含 .git，x64
./release.sh --package-no-git --arch arm64                # 不含 .git，ARM
./release.sh --package --update-submodules                # 打包时拉取保留子模块最新
./release.sh --package --release-channel prerelease       # 打入 prerelease deb
```

产物：`dist/lift2s-ws_<arms版本>_<latest|prerelease>_<架构>[_nogit].zip`。  
打包时会清空临时 `.deb_cache/` 后只拉取目标架构的对应通道 deb，不会把开发机残留的旧 deb 打进 zip。

</details>

### 2.3 完整源码：克隆与子模块

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

#### 之后如何更新子模块

```bash
git submodule update --remote
```



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
  - **`1) 仿真所需包`**：完整源码编控制器+描述；**已装 arms/ocs2 deb（发布包）时只编 ARX 描述**
  - **`2) 真机所需包`**：完整源码编 `arx_ros2_control`+控制器+描述；**deb 模式只编描述 + `arx_ros2_control`**

包列表见 `config/quick_start.conf`（`BUILD_*` / `BUILD_DEB_*`）。

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
