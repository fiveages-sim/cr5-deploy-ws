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
│    │ 写入命令接口（均可选，缺失自动跳过）:          │                   │
│    │   effort  = clamp(τ_g)      ← 重力前馈      │                   │
│    │   position = 当前值          ← 保位（关键）  │                   │
│    │   velocity = 0               ← 速度前馈清零  │                   │
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
- **命令接口可选**：`command_interface_configuration()` 返回 `ALL`（只 claim 硬件实际
  导出的接口），`on_activate` 按名挑选 `position`/`velocity`/`effort`：
  - `position` 有 → 写当前值；无 → 跳过（警告）
  - `velocity` 有 → 写 0；无 → 跳过（警告）
  - `effort` 有 → 写重力矩；无 → 警告一次，控制器以位置保持模式继续运行（不退出）
- 夹爪（`hold_joints`）：`position` 状态必需、命令可选，位置跟随当前值，保持不动作。
- 模型中没有状态来源的关节（如 mimic 的 `gripper_joint2`）q 置 0，对重力矩影响可忽略。

## 依赖

- ROS2: `controller_interface` `hardware_interface` `pluginlib` `rclcpp` `rclcpp_lifecycle`
- 动力学: `pinocchio`（`ros-jazzy-pinocchio`）`urdf`
- 运行时: `controller_manager` `joint_state_broadcaster` `robot_state_publisher` `xacro`

## 编译

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ht_gravity_compensation --symlink-install
```

## 使用

### Mock 验证（无需硬件/电机）

```bash
source install/setup.bash
ros2 launch ht_gravity_compensation gravity_compensation.launch.py
# 默认 hardware:=mock_components、type:=dual
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

- 单臂/单侧：`type:=single` / `type:=left` / `type:=right`（URDF 由 xacro 按 type 展开）。

### 多套机械臂同机（控制盒选择）

一台电脑插多套真机（每套一个 Livelybot 控制盒，各 7 路 ttyACM，VID/PID 相同）时，
默认 `auto` 模式检测到多个控制盒会**报错退出并列出各盒路径**，需用
`xacro_usb_select:=<路径>` 指定本启动连接哪个控制盒（路径可用
`udevadm info -n /dev/ttyACMx` 查询，支持 `1-1.2` / `usb-0:1.2` / 完整 ID_PATH，
子串匹配）：

```bash
# 盒 A（路径 usb-0:1.2）跑重力补偿，盒 B（路径 usb-0:4.2）跑 OCS2
ros2 launch ht_gravity_compensation gravity_compensation.launch.py hardware:=real \
  xacro_usb_select:=usb-0:1.2 \
  hardware_joint_kp:="0.01, 0.01, 0.01, 0.01, 0.01, 0.01" \
  hardware_joint_kd:="0.2, 0.2, 0.2, 0.2, 0.2, 0.2" \
  hardware_gripper_kp:=0.001 hardware_gripper_kd:=0.01
```

选择由 `motor_cpp` 驱动层实现（不依赖 udev 规则、不修改电机 YAML）：
- `auto`：仅允许 1 个控制盒；0 个报错退出；多个报错退出并列出各盒路径供选择。
- 指定路径：只保留该控制盒的端口（按 USB 设备 sysname 分组过滤）。
- 端口顺序确定性：`list_serial_ports` 由 `reverse(readdir)` 改为**名称排序**，
  `serial_id 1,2` 恒对应控制盒通道 1/2（接线口），左右臂不再随重启乱序。
- 串口数量不足（如 `serial_id` 超出可用端口）时报错退出，避免越界。

> 另一套机械臂用 `quick_start.sh` 真机启动时同样选"控制盒"菜单（或手动加
> `xacro_usb_select:=`），即可两套同时运行、互不抢口。

### 低刚度（拖动）模式

kp/kd 是硬件接口参数，通过 `hardware_` 前缀启动参数设置（仅 `hardware:=real/real_usb`
生效；每臂 6 值 CSV，dual 时 xacro 自动拼接为 12 值）：

```bash
ros2 launch ht_gravity_compensation gravity_compensation.launch.py hardware:=real \
  hardware_joint_kp:="0.01, 0.01, 0.01, 0.01, 0.01, 0.01" \
  hardware_joint_kd:="0.2, 0.2, 0.2, 0.2, 0.2, 0.2" \
  hardware_gripper_kp:=0.001 hardware_gripper_kd:=0.01
```

kp/kd 经 xacro 写入 URDF hardware 段，硬件接口加载即生效（无需启动后调参数服务器）。

### Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `type` | `dual` | 臂组合：`single` / `left` / `right` / `dual` |
| `hardware` | `mock_components` | `real` / `real_usb`（ht_ros2_control）\| `mock_components` \| `gz` \| `isaac` |
| `namespace` | `drag_controller` | 所有节点（RSP/CM/RViz）的 ROS namespace；`/` = 全局 |
| `rviz` | `false` | 是否启动 RViz2（默认不启动） |
| `description_package` | `panthera_ht_description` | 描述包：URDF 与关节接口均从该包解析 |

#### 前缀参数（与 ocs2 demo.launch.py 同格式，经 `build_xacro_mappings` 透传）

| 前缀 | 生效条件 | 示例 |
|---|---|---|
| `xacro_xxx:=` | 总是生效 | `xacro_control_mode:=pd_control`、`xacro_config_file:=/path/PantheraDual.yaml`、`xacro_usb_select:=usb-0:1.2` |
| `hardware_xxx:=` | 仅 `hardware:=real/real_usb` | `hardware_joint_kp:=`、`hardware_config_file:=` |
| `robot_profile:=` | 提供 `hardware:`/`xacro:` 段（robot.local.yaml） | `robot_profile:=/path/robot.local.yaml` |

kp/kd/control_mode 等硬件参数**没有专用 launch 参数**，统一走前缀参数
（`hardware_` 真机 / `xacro_` 通用），默认值由描述包 xacro 决定。

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

### 参数（`config/gravity_compensation.yaml`）

| 参数 | 说明 | 默认（双臂） |
|---|---|---|
| `joints` | 重力补偿关节（臂关节） | 12 个 `left/right_joint1..6` |
| `hold_joints` | 仅位置保持关节 | `left/right_gripper_joint` |
| `max_effort` | 力矩限幅 Nm（空 = 不限幅） | `[21,36,36,21,10,10]×2` |
| `gravity_vector` | 世界系重力加速度 | `[0, 0, -9.81]` |
| `urdf_param` | URDF 来源参数 | `robot_description` |

> kp/kd 由硬件接口（`ht_ros2_control`）作为 ROS 参数管理（rqt 可调），
> 本控制器不写 kp/kd 命令接口，因此无 `use_pd`/`hold_kp`/`hold_kd` 参数。

## 注意事项（已确认的硬件行为）

1. **kp/kd 不能为 0**：硬件接口 `full_control` 分支在 `kp <= 0` 时会回退到
   `interfaces.xacro` 的默认增益（20~40）。如需"更软"请给正小值（如 0.5~5）。
2. **与 OCS2 控制器互斥**：`ocs2_arm_controller` 与本控制器都 claim 相同的命令接口，
   不能同时激活；可用 `ros2 control swap_controllers` 切换。
3. **双臂 `body_rpy` 非零**时，`gravity_vector` 需按安装方向旋转（默认假设竖直安装）。

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
