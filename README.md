# HighTorque Panthera HT 机械臂 ROS2 部署工作空间

本仓库用于部署 HighTorque Panthera HT 机械臂的 ROS 2 工作空间，基于 OCS2 MPC 控制框架的完整控制生态系统。

### 前置条件
- 已配置 Git SSH 密钥并可访问相关私有仓库
- 系统已安装 Git（建议 2.30+）

## 1. 仓库初始化
### 将仓库克隆到 ~/open-deploy-ws-ht
```bash
  # 1) 切换到用户主目录
  cd ~
  
  # 2) 克隆仓库（目录名可按需修改）
  git clone -b panthera-ht git@github.com:fiveages-sim/open-deploy-ws.git open-deploy-ws-ht
  
  # 3) 进入仓库目录
  cd ~/open-deploy-ws-ht
```

### 初始化并更新子模块

```bash
  # 运行初始化脚本，自动将所有子模块切换到对应分支的最新提交
  cd ~/open-deploy-ws-ht
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
  ├─ ht-ros2-control                 # 子模块（分支：main，panthera_ros2_control 硬件接口）
  ├─ ocs2_ros2                      # 子模块（分支：ros2，包含嵌套子模块）
  ├─ robot-descriptions-ht          # 子模块（分支：main，含 panthera_ht_description）
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
cd ~/open-deploy-ws-ht
rosdep install --from-paths src --ignore-src -r -y
```

### 3.2 程序编译（推荐：使用 quick_start.sh）

本工作空间已经提供一键脚本 `quick_start.sh`，用于**按场景编译**与**按模式启动**（单臂 / 双臂，仿真 / 真机）。

```bash
cd ~/open-deploy-ws-ht
chmod +x ./quick_start.sh
./quick_start.sh
```

- 在菜单中选择 **`1) 编译 (Build)`**
  - **`1) 编译仿真所需包`**：用于仿真/开发（不依赖真机驱动）
  - **`2) 编译真机所需包`**：用于连接真机（包含 `panthera_ros2_control`）

<details>
<summary><strong>（可选）手动编译命令</strong></summary>

```bash
cd ~/open-deploy-ws-ht
# 仿真所需包
colcon build --packages-up-to \
  ocs2_arm_controller \
  panthera_ht_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

```bash
cd ~/open-deploy-ws-ht
# 真机所需包
colcon build --packages-up-to \
  panthera_ros2_control \
  ocs2_arm_controller \
  panthera_ht_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

</details>

### 3.3 仿真验证
#### 3.3.1 模型可视化
```bash
source ~/open-deploy-ws-ht/install/setup.bash
ros2 launch robot_common_launch manipulator.launch.py robot:=panthera_ht
```

双臂：
```bash
ros2 launch robot_common_launch manipulator.launch.py robot:=panthera_ht type:=dual
```

#### 3.3.2 启动仿真中的控制
推荐直接用 `quick_start.sh` 启动（会自动 `source install/setup.bash`，前提是已成功编译生成 `install/`）。

```bash
cd ~/open-deploy-ws-ht
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - 选择单臂或双臂
  - 选择 **`1) 仿真 (Simulation / mock_components)`**

<details>
<summary><strong>（可选）手动启动仿真控制</strong></summary>

```bash
source ~/open-deploy-ws-ht/install/setup.bash
# 单臂
ros2 launch ocs2_arm_controller demo.launch.py robot:=panthera_ht
# 双臂
ros2 launch ocs2_arm_controller demo.launch.py robot:=panthera_ht type:=dual
```

</details>

#### 3.3.3 启动真机的控制

```bash
cd ~/open-deploy-ws-ht
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - 选择单臂或双臂
  - 选择 **`2) 真机 (Real Hardware)`**

<details>
<summary><strong>（可选）手动启动真机控制</strong></summary>

```bash
source ~/open-deploy-ws-ht/install/setup.bash
# 单臂
ros2 launch ocs2_arm_controller demo.launch.py robot:=panthera_ht hardware:=real
# 双臂
ros2 launch ocs2_arm_controller demo.launch.py robot:=panthera_ht type:=dual hardware:=real
```

</details>

#### 3.3.4 手柄遥操作（Joystick Teleop）

手柄遥操与控制进程分开启动（与 `fa_w2_ws` 相同）：先开单臂/双臂控制，再开手柄。

依赖（若未安装）：
```bash
sudo apt install ros-jazzy-joy
```

用法：
1. 终端 A：启动单臂或双臂控制（仿真/真机）
2. 终端 B：`./quick_start.sh` → **`2) 启动`** → **`3) 手柄遥操作`**

或手动：
```bash
source ~/open-deploy-ws-ht/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py
# 多手柄时指定设备：
# ros2 launch arms_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1
```

常用操作（Xbox 类手柄）：
- **右摇杆按下**：启用/禁用遥操（默认禁用）
- **LB + A**：HOLD → HOME
- **LB + START**：HOLD → OCS2（进入 MPC 后再拖末端）
- **LB + B**：→ HOLD
- **左/右摇杆**：平移 / 旋转
- **A**：切换左/右臂（双臂）
- **X / LT / RT**：夹爪开关或开合比例

## 4. 子模块说明

- **arms_ros2_control** - 机械臂通用 ROS2 控制实现（含 `arms_teleop` 手柄遥操）
- **ht-ros2-control** - Panthera HT 硬件驱动（`panthera_ros2_control`，vendored motor_cpp）
- **ocs2_ros2** - OCS2 的 ROS2 版本（MPC 控制框架）
- **robot-descriptions-ht** - HighTorque 描述仓库（含 `panthera_ht_description`）
- **robot-descriptions-common** - 通用机器人组件（夹爪、相机、launch 等）
