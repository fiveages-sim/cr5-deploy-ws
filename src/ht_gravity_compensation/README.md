# ht_gravity_compensation

Panthera HT（单臂 / 双臂）**独立重力补偿控制器包**，基于 **Pinocchio RNEA** 计算静态重力矩。

本包**完全自包含**：控制器实现、动力学封装、控制器配置（`config/`）与启动文件（`launch/`）都在本包内，
**不修改** `ht_ros2_control`、`panthera_ht_description` 等任何现有包。

## 工作原理

```
┌──────────────────────────── ros2_control ────────────────────────────┐
│                                                                      │
│  GravityCompensationController (本包, 500Hz update)                  │
│    ┌─────────────────────────────────────────────┐                   │
│    │ 读 position 状态 → 组装 Pinocchio q（按名称） │                   │
│    │ rnea(q, v=0, a=0) → 静态重力矩 τ_g          │                   │
│    │ 写入命令接口:                                │                   │
│    │   effort  = clamp(τ_g)      ← 重力前馈      │                   │
│    │   position = 当前值          ← 保位（关键）  │                   │
│    │   kp/kd   = 软刚度（可选）    ← 弱刚度悬停   │                   │
│    └─────────────────────────────────────────────┘                   │
│                          │ command interfaces                        │
│                          ▼                                           │
│  ht_ros2_control / PantheraHardwareInterface (复用现有包，零改动)      │
│    full_control: pos_vel_tqe_kp_kd() → 电机                           │
└──────────────────────────────────────────────────────────────────────┘
```

- 动力学模型：从 `/robot_description`（URDF 参数）用 `pinocchio::urdf::buildModelFromXML` 构建。
- 重力矩：`pinocchio::rnea`（零速度、零加速度），即纯静态重力补偿。
- **保位语义**：硬件接口在 `full_control` 下会把 position 命令直接下发电机，
  因此控制器每个周期把 `position` 写为**当前实测值**，防止未写 position 时被当作 0 目标（回零）。
- 夹爪（`hold_joints`）：只 claim `position` 状态/命令，位置跟随当前值，保持不动作。
- 模型中没有状态来源的关节（如 mimic 的 `gripper_joint2`）q 置 0，对重力矩影响可忽略。

## 依赖

- ROS2: `controller_interface` `hardware_interface` `pluginlib` `rclcpp` `rclcpp_lifecycle`
- 动力学: `pinocchio`（`ros-jazzy-pinocchio`）`urdf`
- 运行时: `controller_manager` `joint_state_broadcaster` `robot_state_publisher` `xacro`

## 编译

```bash
cd ~/ht-deploy-ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ht_gravity_compensation --symlink-install
source install/setup.bash
```

## 使用

### Mock 验证（无需硬件/电机）

```bash
ros2 launch ht_gravity_compensation gravity_compensation.launch.py
# 默认 hardware:=mock_components、type:=dual；launch 自动置 use_pd:=false
```

检查控制器状态：

```bash
ros2 control list_controllers --controller-manager /drag_controller/controller_manager
# gravity_compensation_controller 应显示 active
```

### 真机双臂

```bash
ros2 launch ht_gravity_compensation gravity_compensation.launch.py hardware:=real
```

- `hardware:=real` 时 launch 自动 `use_pd:=true`（真机硬件导出 kp/kd 命令接口）。
- 单臂/单侧：`type:=single` / `type:=left` / `type:=right`（launch 按 type 生成关节列表并覆盖参数）。

### Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `type` | `dual` | 臂组合：`single` / `left` / `right` / `dual` |
| `hardware` | `mock_components` | `real`（ht_ros2_control）\| `mock_components` \| `gz` \| `isaac` |
| `control_mode` | `full_control` | 硬件接口控制模式（`full_control` \| `pd_control`） |
| `use_pd` | `auto` | 写 kp/kd 命令接口：`auto`（real 才开）\| `true` \| `false` |
| `namespace` | `drag_controller` | 所有节点（RSP/CM/RViz）的 ROS namespace；`/` = 全局 |
| `rviz` | `false` | 是否启动 RViz2（默认不启动） |
| `description_package` | `panthera_ht_description` | 描述包：URDF 与关节接口均从该包解析 |

### 关节信息自动解析（不再硬编码）

`ARM_JOINTS` / `GRIPPER_JOINT` / `ARM_MAX_EFFORT` 从描述包的
`xacro/ros2_control/interfaces.xacro` 动态解析：

- 关节名：`<joint name="${prefix}jointN">` → 去掉 `${prefix}` 得到裸名；
- 夹爪关节：名字含 `gripper` 的关节；
- 力矩限幅：`<param name="max_torque">` 数值（按关节顺序）；
- 解析失败时回退到内置默认值（`joint1..6` / `gripper_joint` / `[21,36,36,21,10,10]`）。

### RViz

```bash
ros2 launch ht_gravity_compensation gravity_compensation.launch.py rviz:=true
# 可选：rviz:=true namespace:=my_ns
```

RViz 与 RSP/CM 处于同一 namespace，配置（`config/gravity_compensation.rviz`）使用
相对话题名（`tf`、`robot_description`），固定坐标系为 `world`。

### 参数（`config/gravity_compensation.yaml`，launch 按 type/hardware 自动覆盖部分参数）

| 参数 | 说明 | 默认（双臂） |
|---|---|---|
| `joints` | 重力补偿关节（臂关节） | 12 个 `left/right_joint1..6` |
| `hold_joints` | 仅位置保持关节 | `left/right_gripper_joint` |
| `use_pd` | 写 kp/kd 命令接口（需硬件导出） | true（real） |
| `hold_kp` | 悬停刚度 | `[5.0]×12` |
| `hold_kd` | 悬停阻尼 | `[0.5]×12` |
| `max_effort` | 力矩限幅 Nm（空 = 不限幅） | `[21,36,36,21,10,10]×2` |
| `gravity_vector` | 世界系重力加速度 | `[0, 0, -9.81]` |
| `urdf_param` | URDF 来源参数 | `robot_description` |

## 注意事项（已确认的硬件行为）

1. **kp/kd 不能为 0**：硬件接口 `full_control` 分支在 `kp <= 0` 时会回退到
   `interfaces.xacro` 的默认增益（20~40）。如需"更软"请给正小值（如 0.5~5）。
2. **与 OCS2 控制器互斥**：`ocs2_arm_controller` 与本控制器都 claim 相同的命令接口，
   不能同时激活；可用 `ros2 control swap_controllers` 切换。
3. **双臂 `body_rpy` 非零**时，`gravity_vector` 需按安装方向旋转（默认假设竖直安装）。
4. mock 硬件（`GenericSystem`）无 kp/kd 命令接口，`use_pd` 必须为 false（launch 已自动处理）。

## 验证（已完成）

- **编译**：`colcon build --packages-select ht_gravity_compensation` 通过（Jazzy，C++17 + pinocchio 4.0）。
- **Mock 冒烟**：`gravity_compensation.launch.py`（`hardware:=mock_components type:=dual`）下控制器
  `configure → activate` 成功，日志确认 `Configured: 12 compensated joints, 2 hold joints,
  Pinocchio model nq=16`；`/joint_states` 以 500Hz 发布；`ros2 control list_controllers` 显示 active。
- **动力学数值**（pinocchio python 对同一 URDF 在零位 RNEA）：

  | 关节 | τ_g (Nm) | 说明 |
  |---|---|---|
  | joint1 | 0.000 | 绕垂直轴，无重力分量 ✓ |
  | joint2 | 1.857 | 前臂重力负载 |
  | joint3 | 4.161 | 最大重力负载关节 |
  | joint4 | 0.977 | |
  | joint5/6 | ≈0 | 零位竖直 |
  | gripper* | 0 | 平移/对称关节 |

  左右臂对称，数值在 `max_torque` 限幅（21/36 Nm）范围内，合理。

## 与现有包的边界

| 现有包 | 关系 |
|---|---|
| `src/ht-ros2-control` | 只读复用：硬件接口插件 `ht_ros2_control/PantheraHardwareInterface`（零改动） |
| `src/robot-descriptions-ht` | 只读复用：launch 时用 xacro 展开 `xacro/ros2_control/robot.xacro` 生成 URDF |
| `src/arms_ros2_control` | 不依赖（动力学自实现，参考其 `GravityCompensation` 思路） |
