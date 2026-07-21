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

本工作空间已经提供一键脚本 `quick_start.sh`，用于**按场景编译**与**按模式启动**（双臂 ACone，仿真/真机）。

```bash
cd ~/open-deploy-ws
chmod +x ./quick_start.sh
./quick_start.sh
```

- 在菜单中选择 **`1) 编译 (Build)`**
  - **`1) 编译仿真所需包`**：用于仿真/开发（不依赖真机驱动）
  - **`2) 编译 LIFT2S 真机包`**：官方 SDK（`arxlift2s_ros2_control`，can1/can3/can5）
  - **`3) 编译单臂 arx5 真机包`**：Stanford arx5-sdk（`arx_ros2_control`，需 conda）

<details>
<summary><strong>（可选）手动编译命令</strong></summary>

```bash
cd ~/open-deploy-ws
# 仿真所需包（对应 quick_start.sh -> Build -> Simulation Packages）
colcon build --packages-up-to \
  ocs2_arm_controller \
  arx_lift2s_description \
  arx5_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

```bash
cd ~/open-deploy-ws
# LIFT2S 真机（官方 SDK，对应 quick_start.sh -> Build -> LIFT2S Real Hardware）
colcon build --packages-up-to \
  arxlift2s_ros2_control \
  ocs2_arm_controller \
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
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_lift2s type:=acone_x5
```

#### 3.3.2 启动仿真中的控制
推荐直接用 `quick_start.sh` 启动（会自动 `source install/setup.bash`，前提是已成功编译生成 `install/`）。

```bash
cd ~/open-deploy-ws
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - 选择 **`1) 双臂 (ACone)`**
  - 选择 **`1) 仿真 (Simulation / mock_components)`**

<details>
<summary><strong>（可选）手动启动仿真控制</strong></summary>

```bash
source ~/open-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_lift2s type:=acone_x5
```

</details>

#### 3.3.3 启动真机的控制
LIFT2S 真机通过 CAN 总线连接（left `can1`、right `can3`、lift `can5`），使用官方 SDK 与 split body 控制器架构。

```bash
cd ~/open-deploy-ws
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - 选择 **`1) 双臂 (ACone)`**
  - 选择 **`2) 真机 (Real Hardware)`**

<details>
<summary><strong>（可选）手动启动真机控制</strong></summary>

```bash
source ~/open-deploy-ws/install/setup.bash
ros2 launch arx_lift2s_description ocs2_real.launch.py type:=acone_x5 hardware:=real
```

等效命令：

```bash
ros2 launch ocs2_arm_controller split_body.launch.py \
  robot:=arx_lift2s type:=acone_x5 hardware:=real enable_body:=true
```

</details>

## 4. 子模块说明

- **arms_ros2_control** - 机械臂通用 ROS2 控制实现（含 `arxlift2s_ros2_control` 官方 SDK 硬件插件）
- **arx-ros2-control** - ARX 单臂硬件驱动（Stanford arx5-sdk，CAN 总线）
- **ocs2_ros2** - OCS2 的 ROS2 版本（MPC 控制框架）
- **robot-descriptions-arx** - ARX 机械臂描述文件
- **robot-descriptions-common** - 通用机器人组件（夹爪、相机等）
