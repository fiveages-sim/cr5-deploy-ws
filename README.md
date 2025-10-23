# ROS2 Workspace

这是一个基于ROS2的机器人控制工作空间，包含OCS2 MPC控制框架和Dobot机械臂的完整控制生态系统。

## Deploy

### 克隆仓库

```bash
git clone <your-repo-url> ~/ros2_ws
cd ~/ros2_ws
```

### 初始化子模块

```bash
# 初始化并更新所有子模块
git submodule update --init --recursive
```

### 安装依赖

```bash
# 使用rosdep安装所有依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 构建工作空间

```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_arm_controller dobot_ros2_control cr5_dual_description arms_teleop adaptive_gripper_controller --symlink-install
```

### 启动双臂真机控制
```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=real type:=AG2F90-C-Soft
  ```
## 子模块

- **arms_ros2_control** - 机械臂ROS2控制实现
- **ocs2_ros2** - OCS2的ROS2版本（MPC控制框架）
- **robot-descriptions-dobot** - Dobot机械臂描述文件
- **robot-descriptions-common** - 通用机器人组件（夹爪、相机等）

