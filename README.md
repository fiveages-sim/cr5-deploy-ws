# ARX Lift2S ROS2 部署工作空间

![ARX Lift2S](.images/arx_lift2s.jpg)

本仓库用于部署 **ARX Lift2S** 的 ROS 2 工作空间，基于 OCS2 MPC 控制框架。

方舟（ARX）各机型描述在子模块 [`src/robot-descriptions-arx`](src/robot-descriptions-arx)（X5/R5、ACone、Lift、Lift2S、X7S 等）。`quick_start` 也可启动这些机型便于联调；本仓产品定位是 Lift2S。

### 前置条件
- 已配置 Git SSH 密钥并可访问相关私有仓库
- 系统已安装 Git（建议 2.30+）
- ROS 2 Jazzy（Ubuntu 24.04）

## 1. 仓库初始化
### 将仓库克隆到 ~/lift2s-ws
```bash
  # 1) 切换到用户主目录
  cd ~
  
  # 2) 克隆仓库到 lift2s-ws（目录名可按需修改）
  git clone git@github.com:fiveages-sim/open-deploy-ws.git lift2s-ws
  
  # 3) 进入仓库目录
  cd ~/lift2s-ws
```

现场若使用维护者提供的发布 zip：解压后执行 `./release.sh --install`，再 `./quick_start.sh`（描述源码在工作区，控制器 / HI 由 deb 提供）。

### 初始化并更新子模块

```bash
  # 运行初始化脚本（交互选择 source / deb）
  cd ~/lift2s-ws
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
  ├─ robot-descriptions-arx         # 子模块（分支：main；方舟全系描述）
  └─ robot-descriptions-common      # 子模块（分支：main）
```

### 常见问题
- SSH 权限：若克隆/更新失败，请确认本机 SSH key 已添加到 GitHub 账户，并能通过 `ssh -T git@github.com` 成功握手。
- 网络问题：可重试或改用代理；必要时改为 HTTPS 方式克隆。

## 2. 安装 RMW Zenoh C++

部署机器需要使用 RMW Zenoh，以避免使用 DDS 时被局域网内设备污染消息。
* 安装
  ```bash
  sudo apt install ros-jazzy-rmw-zenoh-cpp
  ```
* 配置 Bashrc
  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ```
* 如需临时取消 Zenoh（恢复默认 DDS），在当前终端执行：
  ```bash
  unset RMW_IMPLEMENTATION
  ```
* 如需永久取消，从 `~/.bashrc` 中删除 `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` 那一行
* 后续在使用 `robot-descriptions-common` 中的 `launch` 文件启动时，会自动拉起一个 zenoh 路由

## 3. 程序编译与仿真验证
### 3.1 依赖安装
* Rosdep 依赖安装
```bash
cd ~/lift2s-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.2 程序编译（推荐：使用 quick_start.sh）

本工作空间已经提供一键脚本 `quick_start.sh`，用于**按场景编译**与**按模式启动**（Lift2S 分体/全身，仿真/真机）。

```bash
cd ~/lift2s-ws
chmod +x ./quick_start.sh
./quick_start.sh
```

- 在菜单中选择 **`1) 编译 (Build)`**
  - **`1) 编译仿真所需包`**：用于仿真/开发（不依赖真机驱动）
  - **`2) 编译真机所需包`**：用于连接真机（包含 `arx_ros2_control` 等）

<details>
<summary>（可选）手动编译命令</summary>

```bash
cd ~/lift2s-ws
# 仿真所需包（对应 quick_start.sh -> Build -> Simulation Packages）
colcon build --packages-up-to \
  ocs2_arm_controller \
  ocs2_wbc_controller \
  topic_based_ros2_control \
  arx_acone_description \
  arx_lift2s_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

```bash
cd ~/lift2s-ws
# 真机所需包（对应 quick_start.sh -> Build -> Real Hardware Packages）
colcon build --packages-up-to \
  arx_ros2_control \
  ocs2_arm_controller \
  ocs2_wbc_controller \
  arx_acone_description \
  arx_lift2s_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

</details>

### 3.3 仿真验证
#### 3.3.1 模型可视化
```bash
source ~/lift2s-ws/install/setup.bash
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_lift2s
```

#### 3.3.2 启动仿真中的控制
推荐直接用 `quick_start.sh` 启动（会自动 `source install/setup.bash`，前提是已成功编译生成 `install/`）。

```bash
cd ~/lift2s-ws
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - 选择 **Lift2S**
  - 选择 **分体** 或 **全身**
  - 选择 **`1) 仿真 (Simulation / mock_components)`**

菜单中的 X5 / R5 / ACone / Lift / X7S 来自 `robot-descriptions-arx`，用于同仓联调。

<details>
<summary>（可选）手动启动仿真控制</summary>

```bash
source ~/lift2s-ws/install/setup.bash
# 分体
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s
# 全身
ros2 launch ocs2_arm_controller full_body.launch.py robot:=arx_lift2s
```

</details>

#### 3.3.3 启动真机的控制
ARX Lift2S 通过 CAN 总线连接（左右臂 can1 / can3），无需配置网络 IP。升降默认 `hybrid`（可在 `quick_start` 中改为 `soft_p`）。

```bash
cd ~/lift2s-ws
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - 选择 **Lift2S**
  - 选择 **分体** 或 **全身**
  - 选择 **`2) 真机 (Real Hardware)`**

<details>
<summary>（可选）手动启动真机控制</summary>

```bash
source ~/lift2s-ws/install/setup.bash
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s hardware:=real
ros2 launch ocs2_arm_controller full_body.launch.py robot:=arx_lift2s hardware:=real
```

</details>

## 4. 子模块说明

- **arms_ros2_control** - 机械臂通用 ROS2 控制实现
- **arx-ros2-control** - ARX 硬件驱动（CAN：臂 + Lift2S 升降/底盘）
- **ocs2_ros2** - OCS2 的 ROS2 版本（MPC 控制框架）
- **robot-descriptions-arx** - 方舟全系描述（本仓主要使用其中的 Lift2S）
- **robot-descriptions-common** - 通用机器人组件（夹爪、相机、launch 等）
