# open-deploy-ws（feature/wujihand2）

Wuji Hand2 精简部署工作空间：机械臂控制核心 + [robot-descriptions-common](https://github.com/fiveages-sim/robot-descriptions-common)（含 dexhands / wuji）+ [wujihand2-ros2-control](https://github.com/fiveages-sim/wujihand2-ros2-control)（Hand2 以太网 HI）+ OCS2。

完整 / 通用工作空间见 `main`；其它机型精简分支例如 `dobot-cr5`、`arx-lift2s`、`panthera-ht`。

## 工作空间结构

```
wuji_ws/
├── src/
│   ├── arms_ros2_control/          # 机械臂 / 灵巧手控制器（basic_joint_controller）
│   ├── robot-descriptions-common/  # 公共描述（跟踪 feature/wuji，含 dexhands/wuji_description）
│   ├── wujihand2-ros2-control/     # Wuji Hand2 ros2_control 硬件接口（以太网 + wuji-sdk）
│   └── ocs2_ros2/                  # OCS2 MPC 框架
├── config/
│   ├── quick_start.conf            # 编译 / 启动菜单配置
│   └── hand.local.template.conf    # 本机 Hand2 IP 模板（复制为 hand.local.conf）
├── init_repo.sh                    # 子模块初始化
├── quick_start.sh                  # 编译与启动（推荐）
└── README.md
```

## 快速开始

在开始前，请先完成 **ROS 2 Jazzy 及 rosdep 环境** 安装（Ubuntu 24.04）：

```bash
# 1. 安装 ROS 2 管理工具（fishros）
wget http://fishros.com/install -O fishros && bash fishros

# 2. 安装 ROS 2 Jazzy 桌面版
sudo apt update
sudo apt install ros-jazzy-desktop

# 3. 初始化 rosdep（首次在本机使用 rosdep 时需要）
sudo rosdep init
rosdep update
```

完成以上步骤后，再执行仓库初始化：

```bash
git clone git@github.com:fiveages-sim/open-deploy-ws.git ros2_ws
cd ros2_ws
./init_repo.sh
```

### `init_repo.sh` 操作说明

**1) 初始化工作空间（推荐）**

分两步选择：

1. **嵌套可见性**（只影响各仓库内部的嵌套子模块；顶层三个仓在选 source 时都会初始化）
   - **public**：仅公开嵌套，适用于外部用户
   - **private**：含私有嵌套，需要内部仓库权限
2. **核心模块安装方式**（逐项 `d`=deb / `s`=source，回车用默认）

| 模块 | 路径 | deb 包 | 默认 |
|------|------|--------|------|
| ocs2 | `src/ocs2_ros2` | `ros-jazzy-ocs2` | **deb** |
| arms | `src/arms_ros2_control` | `ros-jazzy-arms-ros2-control` | source |
| common | `src/robot-descriptions-common` | `ros-jazzy-robot-descriptions-common` | source |

推荐业务组合（脚本默认）：**ocs2=deb，arms/common=source**。全源码即三模块都选 `s`；全 deb 即三模块都选 `d`。

**2) 切换模块安装方式**

探测当前 dpkg / 源码目录状态，按模块在 source ↔ deb 之间切换（会清理冲突源码或卸载对应 deb），然后按目标重新同步。

**3) 仅安装/更新核心 deb**

跳过 Git 拉取；可输入 `ocs2`、`common`、`arms`（逗号分隔），回车表示全部。也可直接运行：

```bash
./scripts/install_core_debs.sh --only ocs2
```

**4) 卸载核心 deb**

可指定包或全部；也可运行 `./scripts/uninstall_core_debs.sh --only ocs2`。

**5) 仅运行 rosdep**

对整个 `src` 安装系统依赖（不拉取子模块、不装 deb）：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

deb 版本与仓库见 [`deb_versions.conf`](deb_versions.conf)；嵌套 public/private 见 [`submodules_visibility.conf`](submodules_visibility.conf)。

### 脚本随后会

1. 同步并初始化选为 **source** 的顶层子模块（选 deb 的跳过；若已有源码会提示清理）
2. 按可见性与配置初始化嵌套子模块（父仓或 common 为 deb 时跳过）
3. 将源码子模块切换到配置分支并拉取最新提交
4. 对源码路径运行 `rosdep install`
5. 安装选为 deb 的包（顺序：ocs2 → common → arms）
6. 将选择写入本地 `.core_module_mode`（已 gitignore），供下次默认参考

### Wuji C SDK 与 Hand2 硬件接口

`src/wujihand2-ros2-control` 为**仅源码**子模块（无 deb）。编译前需安装官方 [wuji-sdk-c](https://docs.wuji.tech)：

```bash
export WUJI_SDK_ROOT=/path/to/wuji-sdk-c-*-linux-gnu
export LD_LIBRARY_PATH=$WUJI_SDK_ROOT/lib:$WUJI_SDK_ROOT:$LD_LIBRARY_PATH
```

真机 IP 可选：见下方「设备连接」；也可复制 `config/hand.local.template.conf` → `config/hand.local.conf`。

## 编译与启动（quick_start）

```bash
./quick_start.sh
# 1) 编译 — 仿真或真机包列表见 config/quick_start.conf
# 2) 启动 — 左/右手、mock 或 real
```

### 设备连接（真机）

**不必写死 `device_address`。** 优先级：

1. `device_address:=IP:port` — 直连（双手同网、产线推荐）
2. `serial_number:=SN` — 按序列号（`device_address` 为空时）
3. 两者都留空 — **`wuji_scan()`** 扫描局域网 Hand2，再按 `direction`（1=左 / -1=右）匹配

```bash
# scan + 匹配左手（单只手或能区分左右时）
ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=1

# 指定 IP（跳过 scan）
ros2 launch wujihand2_ros2_control hand2.launch.py \
  hardware:=real direction:=1 device_address:=192.168.1.110:50001

# 按序列号
ros2 launch wujihand2_ros2_control hand2.launch.py \
  hardware:=real direction:=1 serial_number:=YOUR_SN
```

`quick_start.sh` 真机模式：**回车** = SDK scan；输入 **`d`** = 使用 `hand.local.conf` / 模板默认 IP；或直接输入 `IP:port`。

等价 mock 命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=wuji type:=hand2
```

关节索引标定见 `src/wujihand2-ros2-control/CALIBRATION.md`。

## 测试环境

- **ROS2 Jazzy**（Ubuntu 24.04）


## License

Apache License 2.0. See [LICENSE](LICENSE) for details.
