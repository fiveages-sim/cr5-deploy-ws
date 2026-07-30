# ARX ACone 机械臂 ROS2 部署工作空间

本仓库用于部署 ARX ACone 机械臂的 ROS 2 工作空间，基于 OCS2 MPC 控制框架的完整控制生态系统。

### 前置条件
- 已配置 Git SSH 密钥并可访问相关私有仓库
- 系统已安装 Git（建议 2.30+）

## 1. 仓库初始化
### 将仓库克隆到 ~/open-deploy-ws
```bash
  # 1) 切换到用户主目录
  cd ~
  
  # 2) 克隆仓库到 open-deploy-ws（目录名可按需修改）
  git clone git@github.com:fiveages-sim/open-deploy-ws.git open-deploy-ws
  
  # 3) 进入仓库目录
  cd ~/open-deploy-ws
```

### 初始化并更新子模块

```bash
  # 运行初始化脚本，自动将所有子模块切换到对应分支的最新提交
  cd ~/open-deploy-ws
  ./init_repo.sh
```

### 之后如何更新子模块
```bash
git submodule update --remote
```

### 目录结构（节选）
```
src/
  ├─ arms_ros2_control              # 子模块（分支：main）
  ├─ arx-ros2-control               # 子模块（分支：main）
  ├─ ocs2_ros2                      # 子模块（分支：ros2，包含嵌套子模块）
  ├─ robot-descriptions-arx         # 子模块（分支：main）
  └─ robot-descriptions-common      # 子模块（分支：main）
```

### 常见问题
- SSH 权限：若克隆/更新失败，请确认本机 SSH key 已添加到 GitHub 账户，并能通过 `ssh -T git@github.com` 成功握手。
- 网络问题：可重试或改用代理；必要时改为 HTTPS 方式克隆。


## 2. 安装 RMW Zenoh C++

部署机器需要使用RMW Zenoh以避免使用dds时会被局域网内设备污染消息的问题。
* 安装
  ```bash
  sudo apt install ros-jazzy-rmw-zenoh-cpp
  ```
* 配置Bashrc
  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ```
* 如需临时取消 Zenoh（恢复默认 DDS），在当前终端执行：
  ```bash
  unset RMW_IMPLEMENTATION
  ```
* 如需永久取消，从 `~/.bashrc` 中删除 `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` 那一行
* 后续在使用 `robot-descriptions-common` 中的 `launch` 文件启动时，会自动拉起来一个 zenoh 路由

## 3. 程序编译与仿真验证
### 3.1 依赖安装
* Rosdep 依赖安装
```bash
cd ~/open-deploy-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.2 程序编译（推荐：使用 quick_start.sh）

本工作空间已经提供一键脚本 `quick_start.sh`（菜单结构对齐 [dobot-cr5](https://github.com/fiveages-sim/open-deploy-ws/tree/dobot-cr5)），用于**按场景编译**与**按模式启动**。

```bash
cd ~/arx_lift2s_ws   # 或你的 workspace 路径
chmod +x ./init_repo.sh ./quick_start.sh
./init_repo.sh       # 顶层 + Lift2S 嵌套 HI + external/arx5-sdk、预编译 libsoem.so
./quick_start.sh
```

- 在菜单中选择 **`1) 编译 (Build)`**
  - **`1) 编译仿真所需包`**：仿真/开发（不依赖真机驱动）
  - **`2) 编译 Lift2S 真机包`（推荐验证）**：Stanford 臂 + Hybrid 升降（`arxlift2s_ros2_control`，**无 conda**；can1/can3/can5）
  - **`3) 编译单/双臂桌面真机包`**：`arx_ros2_control`（`arx5` / `arx_acone`，旧路径可能仍需 conda）

<details>
<summary><strong>（可选）手动编译命令</strong></summary>

```bash
cd ~/open-deploy-ws
# 仿真所需包（对应 quick_start.sh -> Build -> Simulation Packages）
colcon build --packages-up-to \
  ocs2_arm_controller \
  arx_acone_description \
  arx_lift2s_description \
  arx5_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

```bash
cd ~/arx_lift2s_ws
# Lift2S 真机（Stanford 臂 + Hybrid 升降；先保证 external/arx5-sdk 与 SOEM/lib/<arch>/libsoem.so）
colcon build --packages-up-to \
  arxlift2s_ros2_control \
  ocs2_arm_controller \
  arx_acone_description \
  arx_lift2s_description \
  arx5_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

```bash
cd ~/open-deploy-ws
# 单臂 arx5 真机（Stanford SDK，需先编译 arx5-sdk）
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

### 3.3 仿真验证
#### 3.3.1 模型可视化
```bash
source ~/open-deploy-ws/install/setup.bash
# 仅双臂 AC One
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_acone
# 完整 Lift2S（底盘 + 升降 + AC One）
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_lift2s
```

#### 3.3.2 启动仿真中的控制
推荐直接用 `quick_start.sh` 启动（会自动 `source install/setup.bash`，前提是已成功编译生成 `install/`）。

```bash
cd ~/open-deploy-ws
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - **`1) 单臂 X5`**：Stanford 仿真 / 真机（真机时询问 `full_control` / `position`）
  - **`2) 双臂 ACone`**：Stanford 仿真 / 真机（同上，can1/can3）
  - **`3) Lift2S 分体`**：双臂 OCS2 + 升降 BasicJoint（规划 URDF 来自 `arx_acone`）
  - **`4) Lift2S 全身`**：升降 + 双臂同一 OCS2

运行模式与 `fa_w2_ws` 对齐：真机 / 真机 headless / 仿真 / 仿真 headless / Isaac / Isaac headless / 仅可视化。  
单臂 / 双臂选真机时，会再询问控制模式（默认 `full_control`；`position` / `pd_control` 为真机位置环）。Lift2S 不询问该菜单。

<details>
<summary><strong>（可选）手动启动仿真控制</strong></summary>

```bash
source ~/open-deploy-ws/install/setup.bash
# 单臂 X5
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5
# 仅双臂
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_acone
# Lift2S 分体
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s
# Lift2S 全身
ros2 launch ocs2_arm_controller full_body.launch.py robot:=arx_lift2s
# Isaac / headless 示例
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_acone hardware:=isaac
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s hardware:=isaac launch_mode:=control_only
```

</details>

#### 3.3.3 启动真机的控制
- **单臂 / 双臂 AC One（Phase 1 MIT）**：Stanford `arx_ros2_control`，控制模式对齐 [panthera-ht](https://github.com/fiveages-sim/open-deploy-ws/tree/panthera-ht)：默认 **`full_control`（OCS2 MIX，推荐真机）**，并保留真机 **`position`** 路径（`pd_control` 为 HT 别名）。部署流程对齐 [arx-acone](https://github.com/fiveages-sim/open-deploy-ws/tree/arx-acone)（`hardware:=real`，can1/can3）。
- **Lift2S**：`arxlift2s_ros2_control`（臂 `can1`/`can3` Stanford **full_control MIX**；升降 `can5` **Hybrid MIT**，可调 `arx_lift.hybrid_kp/kd`）。

```bash
cd ~/open-deploy-ws
./quick_start.sh
# Build → 3) 单/双臂真机包（Stanford）
# Launch → 1) 单臂 X5 或 2) 双臂 ACone → 1) 真机 → 控制模式（默认 full_control）
```

<details>
<summary><strong>（可选）手动启动真机控制</strong></summary>

```bash
source ~/open-deploy-ws/install/setup.bash
# 单臂 X5（Stanford SDK + full_control / MIX）
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real
# 仅双臂 AC One（Stanford SDK + full_control / MIX）
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_acone hardware:=real
# Legacy position write path（经 xacro_ 前缀）
# ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_acone hardware:=real xacro_control_mode:=position
# Lift2S 分体（臂 OCS2 + 升降 body_joint / Hybrid）
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s hardware:=real
# Lift2S 全身
ros2 launch ocs2_arm_controller full_body.launch.py robot:=arx_lift2s hardware:=real
# headless
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s hardware:=real launch_mode:=control_only
```

**AC One MIX 验收：** HI `control_mode=full_control`；OCS2 `kp=30, kd=3` / Mixed mode；节点 `/arx_acone_left_system`、`/arx_acone_right_system`。详情见 `arx_acone_description/README.md`。

</details>

## 4. 子模块说明

- **arms_ros2_control** - 机械臂通用 ROS2 控制实现（含 `arxlift2s_ros2_control`：Stanford 臂 + Hybrid 升降）
- **arx-ros2-control** - ARX 单/双臂硬件驱动（Stanford arx5-sdk；`full_control` / `position`≈HT `pd_control`）
- **ocs2_ros2** - OCS2 的 ROS2 版本（MPC 控制框架）
- **robot-descriptions-arx** - ARX 机械臂描述文件
- **robot-descriptions-common** - 通用机器人组件（夹爪、相机等）
