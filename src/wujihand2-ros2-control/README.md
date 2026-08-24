# wujihand2_ros2_control

ROS 2 Jazzy `hardware_interface::SystemInterface` for **Wuji Hand 2** over Ethernet via [wuji-sdk](https://github.com/wuji-technology/wuji-sdk) (`libwuji_sdk_c`).

Plugin: `wujihand2_ros2_control/WujiHand2Hardware`

## Layout (O6 / XHand1 style)

No separate SDK client class. `libwuji_sdk_c` calls live as private methods on `WujiHand2Hardware`.

| CM hook | Behavior |
|---------|----------|
| `read()` | Copy latest `joint_states` snapshot (subscription callback → mutex buffer) |
| `write()` | Sync `publisher.send` of 20×`JointCommand(pos, 0, 0)` |
| activate | Official order: effort_limit → MIT → enable → subscribe → sync+hold → publish |
| deactivate / shutdown / destructor | `safe_shutdown()`: disable → close pub → stop sub → disconnect |

## Dependencies

- ROS 2 Jazzy: `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`
- Wuji C SDK tarball (headers + `libwuji_sdk_c.so`)

```bash
export WUJI_SDK_ROOT=/path/to/wuji-sdk-c-*-linux-gnu
# example on this machine:
# export WUJI_SDK_ROOT=$HOME/Documents/wuji/wuji-sdk-c-2026.8.17-x86_64-linux-gnu

cd /path/to/wuji_ws
colcon build --packages-select wujihand2_ros2_control --symlink-install
source install/setup.bash
```

Ensure the library is discoverable at runtime (`LD_LIBRARY_PATH=$WUJI_SDK_ROOT` or `$WUJI_SDK_ROOT/lib`).

## Launch (with `wuji_description`)

```bash
# mock (shared BJC launch — no Wuji-specific args)
ros2 launch basic_joint_controller hand.launch.py hand:=wuji type:=hand2

# real Hand2 (Wuji-specific launch in wujihand2_ros2_control)
export WUJI_SDK_ROOT=/path/to/wuji-sdk-c-*-linux-gnu
ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=1
```

Or use `./quick_start.sh` → **Launch** → real mode (empty address = SDK scan).

## Device connection (real hardware)

**You do not have to pass `device_address`.** On activate, the HI picks a target in this order:

| Priority | Launch arg | When to use |
|----------|------------|-------------|
| 1 | `device_address:=IP:port` | Fixed IP (multi-hand LAN, production). Example left: `192.168.1.110:50001`, right: `.111` ([Wuji docs](https://docs.wuji.tech/docs/zh/wuji-hand/latest/)) |
| 2 | `serial_number:=SN` | Connect by serial (only if `device_address` is empty) |
| 3 | *(both empty)* | **`wuji_scan()`** — discover Hand2 on the network, then match **`direction`** → `hand_side` (left/right) |

Examples:

```bash
# SDK scan + match left hand (single device or handedness match)
ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=1

# SDK scan + match right hand
ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=-1

# Direct IP (skip scan)
ros2 launch wujihand2_ros2_control hand2.launch.py \
  hardware:=real direction:=1 device_address:=192.168.1.110:50001

# Serial number (skip scan)
ros2 launch wujihand2_ros2_control hand2.launch.py \
  hardware:=real direction:=1 serial_number:=YOUR_SN
```

**Tips:** Use scan for bench / single-hand setup. Use `device_address` or `config/hand.local.conf` when two hands share a subnet. `./quick_start.sh` real mode: press **Enter** for scan, type **`d`** for default IP from config, or type a custom `IP:port`.

### HI `<param>` (URDF / xacro)

| Param | Default | Meaning |
|-------|---------|---------|
| `hand_side` | left/right from `direction` | scan match |
| `serial_number` | `""` | connect by SN |
| `device_address` | `""` | `IP:port` |
| `mit_kp` / `mit_kd` | `3.0` / `0.05` | MIT params |
| `effort_limit` | `1.5` | amps |
| `read_feedback` | `true` | subscribe joint_states |
| `require_initial_feedback` | `true` | activate waits for first frame |
| `command_deadband` | `0.0` | rad; `0` = always send |
| `connect_timeout_ms` | `5000` | |
| `enable_timeout_s` | `5.0` | wait for first feedback |

## Joint mapping (assumed — calibrate on device)

SDK index `0..19` is assumed to match `hand2.yaml` order (thumb → pinky). See `wuji_hand2_protocol.hpp`.

**Calibration:** enable one joint at a time, confirm only the expected URDF joint moves, then freeze the table. Do not flip command signs for `hand_base` +Z remapping (`wrist_align` in `wuji_description`).

## Differences vs LinkerHand

- Bus: Ethernet + wuji-sdk (not Modbus / CAN)
- Firmware MIT configured at activate; commands still position-only from BJC
- Must `disable()` on exit (not only close the socket)
