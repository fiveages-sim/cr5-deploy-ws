# wujihand2_ros2_control

基于 [wuji-sdk](https://github.com/wuji-technology/wuji-sdk)（`libwuji_sdk_c`）的 **Wuji Hand 2** ROS 2 Jazzy 以太网硬件接口，实现 `hardware_interface::SystemInterface`。

插件名：`wujihand2_ros2_control/WujiHand2Hardware`

## 代码结构（O6 / XHand1 风格）

不单独封装 SDK 客户端类；`libwuji_sdk_c` 的调用直接作为 `WujiHand2Hardware` 的私有方法。

| 控制器管理器钩子 | 行为 |
|------------------|------|
| `read()` | 复制最新的 `joint_states` 快照（订阅回调 → 互斥缓冲） |
| `write()` | 同步 `publisher.send`，发送 20 路 `JointCommand(pos, 0, 0)`；**send 失败即 `safe_shutdown()` + ERROR**（对齐 Wuji 官方） |
| activate | effort_limit → MIT → enable → subscribe → sync+hold → publish；**首条 hold 失败即 ERROR** |
| deactivate / shutdown / 析构 | `safe_shutdown()`：disable → 关闭发布 → 停止订阅 → 断开连接 |

## 依赖

- ROS 2 Jazzy：`hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`
- Wuji C SDK 压缩包（头文件 + `libwuji_sdk_c.so`）

```bash
export WUJI_SDK_ROOT=/path/to/wuji-sdk-c-*-linux-gnu
# 本机示例：
# export WUJI_SDK_ROOT=$HOME/Documents/wuji/wuji-sdk-c-2026.8.17-x86_64-linux-gnu

cd /path/to/wuji_ws
colcon build --packages-select wujihand2_ros2_control --symlink-install
source install/setup.bash
```

运行时需能找到动态库（`LD_LIBRARY_PATH=$WUJI_SDK_ROOT` 或 `$WUJI_SDK_ROOT/lib`）。

## 启动（配合 `wuji_description`）

```bash
# 仿真 / mock（共用 BJC launch，无 Wuji 专用参数）
ros2 launch basic_joint_controller hand.launch.py hand:=wuji type:=hand2

# 真机 Hand2（本包专用 launch）
export WUJI_SDK_ROOT=/path/to/wuji-sdk-c-*-linux-gnu
ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=1
```

也可使用 `./quick_start.sh` → **Launch** → 真机模式（地址留空则 SDK 扫描）。

## 设备连接（真机）

**不必强制传入 `device_address`。** activate 时硬件接口按以下优先级选择目标设备：

| 优先级 | Launch 参数 | 适用场景 |
|--------|-------------|----------|
| 1 | `device_address:=IP:port` | 固定 IP（多手同网、量产）。左手示例 `192.168.1.110:50001`，右手 `.111`（见 [Wuji 文档](https://docs.wuji.tech/docs/zh/wuji-hand/latest/)） |
| 2 | `serial_number:=SN` | 按序列号连接（仅当 `device_address` 为空时生效） |
| 3 | *两者皆空* | **`wuji_scan()`** — 扫描网络发现 Hand2，再按 **`direction`** 匹配 `hand_side`（左/右） |

示例：

```bash
# SDK 扫描 + 匹配左手
ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=1

# SDK 扫描 + 匹配右手
ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=-1

# 直连 IP（跳过扫描）
ros2 launch wujihand2_ros2_control hand2.launch.py \
  hardware:=real direction:=1 device_address:=192.168.1.110:50001

# 序列号连接（跳过扫描）
ros2 launch wujihand2_ros2_control hand2.launch.py \
  hardware:=real direction:=1 serial_number:=YOUR_SN
```

**提示：** 台架 / 单手调试可用扫描；双手同网段时建议用 `device_address` 或 `config/hand.local.conf`。`./quick_start.sh` 真机模式：按 **Enter** 扫描，输入 **`d`** 使用配置默认 IP，或直接输入 `IP:port`。

### 硬件接口参数（URDF / xacro）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `hand_side` | 由 `direction` 推导 left/right | 扫描匹配左右手 |
| `serial_number` | `""` | 按 SN 连接 |
| `device_address` | `""` | `IP:port` |
| `mit_kp` / `mit_kd` | `3.0` / `0.05` | MIT 控制参数 |
| `effort_limit` | `1.5` | 电流限制（A） |
| `read_feedback` | `true` | 订阅 joint_states |
| `require_initial_feedback` | `true` | activate 等待首帧反馈 |
| `command_deadband` | `0.0` | 弧度；`0` = 每控制周期都 send（对齐 Wuji 官方 HOLD）；`>0` = 仅变化超过阈值时 send（带宽优化，真机验证后再用） |
| `connect_timeout_ms` | `5000` | 连接超时（ms） |
| `enable_timeout_s` | `5.0` | 等待首帧反馈超时（s） |

## send 失败策略（真机）

与 [Wuji SDK 官方示例](https://github.com/wuji-technology/wuji-sdk) 一致：**任意一次** `joint_command` send 失败即停止控制并 disable 电机。

| 阶段 | 行为 |
|------|------|
| activate 首条 hold | `safe_shutdown()` + `on_activate` 返回 ERROR（不进入 activated） |
| 运行时 `write()` | `safe_shutdown()` + `write()` 返回 ERROR → controller_manager 停用控制器 |

**恢复：** deactivate 后重新 activate，或重启 launch / `quick_start.sh`。

**注意：** 以太网单次抖动也会触发上述流程（与官方 teleop 相同）。若 HOLD @ 100 Hz 因带宽/load 偶发误触发，可后续将 `command_deadband` 设为 `0.001` 等（见真机带宽测试）。

## 多手同网（运维）

| 场景 | 推荐 |
|------|------|
| 双手同网段 | 固定 `device_address` 或 `serial_number`，**不要**依赖 scan |
| 单手台架 | scan 可用，显式 `direction` |
| 本机配置 | 复制 `config/hand.local.template.conf` → `config/hand.local.conf` |

## 关节映射（假设值 — 需在真机标定）

SDK 索引 `0..19` 目前假设与 `hand2.yaml` 顺序一致（拇指 → 小指）。详见 `wuji_hand2_protocol.hpp`。

**标定：** 见 `CALIBRATION.md`（人工逐关节，在 MOVEJ 模式下小步进验证）。

## 与 LinkerHand 的差异

- 总线：以太网 + wuji-sdk（非 Modbus / CAN）
- 固件 MIT 在 activate 时配置；BJC 侧仍只发位置命令
- 退出时必须调用 `disable()`（不能仅关闭 socket）
