# Dobot CR5双臂机器人ROS2部署工作空间

本仓库用于部署 Dobot CR5 双臂机器人的 ROS 2 工作空间，基于 OCS2 MPC 控制框架的完整控制生态系统。

### 前置条件
- 已配置 Git SSH 密钥并可访问相关私有仓库
- 系统已安装 Git（建议 2.30+）

## 1. 仓库初始化
### 将仓库克隆到 ~/cr5-deploy-ws
```bash
# 1) 切换到用户主目录
cd ~

# 2) 克隆仓库到 cr5-deploy-ws（目录名可按需修改）
git clone git@github.com:fiveages-sim/cr5-deploy-ws.git cr5-deploy-ws

# 3) 进入仓库目录
cd ~/cr5-deploy-ws
```

### 初始化并更新子模块

#### 方法一：使用自动化脚本（推荐）
```bash
# 运行初始化脚本，自动将所有子模块切换到对应分支的最新提交
./init_repo.sh
```

<details>
<summary><strong>方法二：手动初始化</strong></summary>

```bash
# 仅初始化这4个子模块，不递归（不会初始化 ocs2_ros2 的嵌套子模块）
# 注意：此方法会将子模块切换到主仓库记录的特定提交，而不是分支的最新提交
git submodule update --init

# 如果需要获取分支的最新提交，需要手动切换到对应分支
git submodule foreach 'git checkout $(git config -f ../.gitmodules --get submodule.$name.branch || echo main) && git pull'
```

</details>

### 之后如何更新子模块
```bash
git submodule update --remote
```

### 目录结构（节选）
```
src/
  ├─ arms_ros2_control              # 子模块（分支：main）
  ├─ ocs2_ros2                      # 子模块（分支：ros2，包含嵌套子模块）
  ├─ robot-descriptions-dobot       # 子模块（分支：main）
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
* 后续在使用 `robot-descriptions-common` 中的 `launch` 文件启动时，会自动拉起来一个 zenoh 路由

## 3. 程序编译与仿真验证
### 3.1 依赖安装
* Rosdep 依赖安装
```bash
cd ~/cr5-deploy-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.2 程序编译（推荐：使用 quick_start.sh）

本工作空间已经提供一键脚本 `quick_start.sh`，用于**按场景编译**与**按模式启动**（单臂/双臂/vla39，仿真/真机/真机+HTTP Bridge）。

```bash
cd ~/cr5-deploy-ws
chmod +x ./quick_start.sh
./quick_start.sh
```

- 在菜单中选择 **`1) 编译 (Build)`**
  - **`1) 编译仿真所需包`**：用于仿真/开发（不依赖真机驱动）
  - **`2) 编译真机所需包`**：用于连接真机（包含 `dobot_ros2_control` 等）

<details>
<summary><strong>（可选）手动编译命令</strong></summary>

```bash
cd ~/cr5-deploy-ws
# 仿真所需包（对应 quick_start.sh -> Build -> Simulation Packages）
colcon build --packages-up-to \
  ocs2_arm_controller \
  cr5_description \
  cr5_dual_description \
  arms_teleop \
  adaptive_gripper_controller \
  vla_http_bridge \
  --symlink-install
```

```bash
cd ~/cr5-deploy-ws
# 真机所需包（对应 quick_start.sh -> Build -> Real Hardware Packages）
colcon build --packages-up-to \
  dobot_ros2_control \
  ocs2_arm_controller \
  cr5_description \
  cr5_dual_description \
  arms_teleop \
  adaptive_gripper_controller \
  vla_http_bridge \
  --symlink-install
```

</details>

### 3.3 仿真验证
#### 3.3.1 模型可视化
* 单臂
  ```bash
  source ~/cr5-deploy-ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5
  ```
* 带夹爪
  ```bash
  source ~/cr5-deploy-ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5 type:=AG2F90-C-Soft
  ```
* 双臂
  ```bash
  source ~/cr5-deploy-ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5_dual
  ```

#### 3.3.2 启动仿真中的控制
推荐直接用 `quick_start.sh` 启动（会自动 `source install/setup.bash`，前提是已成功编译生成 `install/`）。

```bash
cd ~/cr5-deploy-ws
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - **`1) 单臂 (CR5)`** 或 **`2) 双臂 (CR5 Dual)`**
  - 选择 **`1) 仿真 (Simulation / mock_components)`**

<details>
<summary><strong>（可选）手动启动仿真控制</strong></summary>

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5 type:=AG2F90-C-Soft
```

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual type:=AG2F90-C-Soft
```

</details>

#### 3.3.3 启动真机的控制
同样推荐使用 `quick_start.sh`（脚本会提示你检查真机 IP 配置）：

```bash
cd ~/cr5-deploy-ws
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - **`1) 单臂 (CR5)`** / **`2) 双臂 (CR5 Dual)`** / **`3) 单臂 vla39 (CR5 vla39)`**
  - 选择运行模式：
    - **`2) 真机 (Real Hardware)`**：直接启动 `ocs2_arm_controller demo.launch.py ... hardware:=real`
    - **`3) 真机 + HTTP Bridge`**：启动 `cr5_description dobot_bringup_ros2.launch.py ...`（包含 HTTP Bridge bringup）

**真机 IP 配置提示**
- **bringup 默认 IP**：`src/robot-descriptions-dobot/cr5_description/launch/dobot_bringup_ros2.launch.py`
- **ros2_control 的 robot_ip**：通常在 `cr5_description/xacro/ros2_control/robot.xacro`（或同路径下的 ros2_control xacro 配置）

<details>
<summary><strong>（可选）手动启动真机控制</strong></summary>

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5 hardware:=real type:=AG2F90-C-Soft
```

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=real type:=AG2F90-C-Soft
```

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch cr5_description dobot_bringup_ros2.launch.py robot:=cr5_dual hardware:=real type:=AG2F90-C-Soft
```

</details>

## 4. 子模块说明

- **arms_ros2_control** - 机械臂ROS2控制实现
- **ocs2_ros2** - OCS2的ROS2版本（MPC控制框架）
- **robot-descriptions-dobot** - Dobot机械臂描述文件
- **robot-descriptions-common** - 通用机器人组件（夹爪、相机等）
