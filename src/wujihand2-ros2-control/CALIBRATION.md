# Hand2 关节索引标定（人工）

`include/wujihand2_ros2_control/hands/hand2/wuji_hand2_protocol.hpp` 中的映射目前是**假设值**：
SDK 命令/反馈索引 `i` 与 `hand2.yaml` 中的关节顺序一一对应。

官方文档只保证索引 `0..19` 及 `{手指}_S{1..4}` 标签，并未固定与 URDF 关节名的对应关系。
**上线前必须在真机上标定。**

## 标定目标

对每个假设的 SDK 索引 `k`（0..19），当你只命令 ROS 关节 `kJointNameSuffixes[k]` 时，
**应当只有对应的物理关节在动**。若实际动的是别的指节，需调整 `kJointNameSuffixes` 中的顺序，
然后重新编译 `wujihand2_ros2_control`。

**不要**通过翻转命令符号来“修正” `wuji_description` 里 `hand_base` +Z（`wrist_align`）带来的差异。

## 1. 准备工作

```bash
export WUJI_SDK_ROOT=/path/to/wuji-sdk-c-*-linux-gnu
source /path/to/wuji_ws/install/setup.bash

# 左手 — SDK 扫描（或指定 device_address:=192.168.1.110:50001）
ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=1

# 右手
# ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=-1
```

安全注意：

- 手固定在台架上，工作空间内无障碍物；**不要**与 Wuji Studio 同时控制电机。
- activate 后确认**无跳变**（命令位置与反馈位置一致，可在日志或 RViz 中观察）。
- activate 日志中**不得**出现 `Initial hold send failed` 或 `Timed out waiting for joint_states`；若有则先排查网络/SDK 再标定。
- 每次只小步进，建议约 **0.05–0.10 rad**。

## 2. 进入 MOVEJ 模式

控制器启动后默认处于 **HOLD（保持）**。需先切换到 **MOVEJ（关节运动）** 才能接收关节目标。

**RViz：** 添加面板 `OCS2FSMPanel` → 点击 **HOLD → MOVEJ**。

**命令行：**

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 4}"
```

## 3. 逐关节小步进

使用 RViz 的 **JointControlPanel** 或命令行均可。**每次测试只动一个关节。**

### 方式 A — RViz JointControlPanel（推荐）

1. 添加面板 `JointControlPanel`。
2. 确认 `/joint_states` 在更新，面板中能看到 Hand2 的 20 个关节。
3. 记下当前各关节位置；**只调整一个滑条**（幅度约 0.05–0.10 rad），其余不动。
4. 发送目标（面板会发布到 `hand_joint_controller/target_joint_position`）。
5. 同时观察 RViz 模型、真机，以及：

```bash
ros2 topic echo /joint_states --field name,position
```

6. 记录：你命令的是哪个关节，实际动的是哪个关节。

### 方式 B — 命令行

读取当前位置，只改其中一个索引，发布完整的 20 维数组：

```bash
# 示例：索引 0（thumb_cmc_flex）小步 +0.08 rad — 请替换为当前实际读数
ros2 topic pub --once /hand_joint_controller/target_joint_position \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

数组顺序与 `hand2.yaml`、`kJointNameSuffixes` 一致：拇指 → 食指 → 中指 → 无名指 → 小指。

测下一个关节前，先**回到原姿态**（滑条复位，或重新发布原始位置数组）。

## 4. 填写记录表

| SDK 索引 | 预期 ROS 关节（`kJointNameSuffixes[k]`） | 实际运动的关节 | 通过？ |
|--------:|------------------------------------------|----------------|--------|
| 0 | thumb_cmc_flex | | |
| 1 | thumb_cmc_abd | | |
| … | … | | |
| 19 | pinky_dip | | |

## 5. 修正映射并重新编译

若索引 `k` 命令后动的是错误的指节：

1. 编辑 `wuji_hand2_protocol.hpp` 中的 `kJointNameSuffixes`（调整条目顺序）。
2. 重新编译并 source：

```bash
colcon build --packages-select wujihand2_ros2_control --symlink-install
source install/setup.bash
```

3. 重复第 1–4 步，直到 20 行全部通过。
4. 将确认后的映射表提交到 git。

## 6. 结束标定，回到 HOLD

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 2}"
```

## 标定前假设表

| SDK 索引 | ROS 关节后缀 |
|----------|--------------|
| 0–3 | thumb_cmc_flex, thumb_cmc_abd, thumb_mcp, thumb_ip |
| 4–7 | index_finger_* |
| 8–11 | middle_finger_* |
| 12–15 | ring_* |
| 16–19 | pinky_* |

标定完成后，`hand2.yaml` 中的关节顺序须与上表保持一致。
