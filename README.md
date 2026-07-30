# HighTorque Panthera HT 机械臂 ROS2 部署工作空间

本仓库用于部署 HighTorque Panthera HT 机械臂的 ROS 2 工作空间，基于 OCS2 MPC 控制框架的完整控制生态系统。

### 前置条件
- 系统已安装 ROS 2 Jazzy
- **快速部署（发布 zip）**：一般不需要 Git / SSH
- **开发方式（git clone）**：需配置 Git SSH 密钥并可访问相关私有仓库；Git 建议 2.30+

## 三个核心脚本

| 脚本 | 作用 |
|------|------|
| [`init_repo.sh`](init_repo.sh) | **仓库初始化**：按模块选择 source/deb、拉子模块、安装/切换/卸载核心 deb、rosdep |
| [`quick_start.sh`](quick_start.sh) | **日常编译与启动**：按场景编译（仿真/真机）、启动单臂/双臂（仿真/真机/手柄） |
| [`release.sh`](release.sh) | **发布与现场 deb**：下载/安装/卸载核心 deb；维护者打包发布 zip（含/不含 `.git`） |

辅助脚本：[`scripts/install_core_debs.sh`](scripts/install_core_debs.sh)、[`scripts/uninstall_core_debs.sh`](scripts/uninstall_core_debs.sh)；版本映射见 [`deb_versions.conf`](deb_versions.conf)。

核心 deb 安装顺序：`ocs2` → `robot-descriptions-common` → `arms-ros2-control-full`（**含 `ht_ros2_control`**）。  
`arms=deb` 时不再拉 `ht-ros2-control` 源码；HT 描述包 `robot-descriptions-ht` 仍源码编译。

---

## A. 快速部署方式（推荐现场）

面向「解压即用」：发布包已含（或可下载）核心 deb，只需装 deb、编译 HT 描述、启动。

```bash
# 1) 解压发布 zip 到目标目录，例如 ~/ht-deploy-ws
cd ~/ht-deploy-ws

# 2) 安装核心 deb（需 sudo；zip 内已有 .deb_cache/ 时可直接装）
./release.sh --install
# 若缺少 deb 或需更新：
# ./release.sh --download && ./release.sh --install

# 3) 编译与启动
source /opt/ros/jazzy/setup.bash
./quick_start.sh
# → 1) 编译 → 仿真或真机所需包
# → 2) 启动 → 单臂/双臂 / 仿真或真机
```

也可运行 `./release.sh` 进入交互菜单（下载 / 安装 / 卸载 deb）。

可选：安装 RMW Zenoh（见下文「安装 RMW Zenoh C++」）。

---

## B. 开发方式（git clone）

面向改子模块、切 source/deb、跟主仓开发。

### 1. 克隆仓库

```bash
cd ~
git clone -b panthera-ht git@github.com:fiveages-sim/open-deploy-ws.git ht-deploy-ws
cd ~/ht-deploy-ws
```

### 2. 初始化（`init_repo.sh`）

```bash
./init_repo.sh
```

| 选项 | 说明 |
|------|------|
| 1) 初始化 | 逐模块 source/deb；**默认 ocs2/arms/common=deb**；arms=deb 时自动清理 `ht-ros2-control` 源码 |
| 2) 切换 | 源码 ↔ deb（会清理冲突目录或卸载 deb） |
| 3) 仅 deb | 只安装/更新核心 deb，不拉 Git 子模块 |
| 4) 卸载 deb | 卸载核心 deb |
| 5) rosdep | 仅对源码子模块路径运行 rosdep |

初始化时还会询问 **deb 发布通道**：`1) latest` / `2) pre-release` / `3) conf`（见 `deb_versions.conf`）。

**推荐开发起步（核心用 deb，只编 HT 描述）：**

```bash
./init_repo.sh                    # ocs2/arms/common 选 deb（通道按需）
source /opt/ros/jazzy/setup.bash
./quick_start.sh                  # 编译仿真/真机所需包
```

若要改 `arms` / `ocs2` / `ht_ros2_control` 源码：在 `init_repo.sh` 选项 2 将对应模块切到 **source**，再编译。

### 3. 更新子模块

```bash
# 仅更新仍保留为源码的 HT 描述（deb 模式下常见）
git submodule update --remote src/robot-descriptions-ht

# 若 arms 等为 source，再按需：
# git submodule update --remote
```

### 目录结构（节选）

```
ht-deploy-ws/
├── init_repo.sh                  # 初始化 / source-deb 切换
├── quick_start.sh                # 编译与启动
├── release.sh                    # 现场 deb 安装 / 维护者打包
├── deb_versions.conf             # deb 版本与仓库映射
├── scripts/
│   ├── install_core_debs.sh
│   └── uninstall_core_debs.sh
└── src/
    ├─ robot-descriptions-ht      # 通常源码（HT 模型）
    ├─ ht-ros2-control            # arms=deb 时由 arms-full 提供，可跳过
    ├─ arms_ros2_control          # deb 模式下可跳过
    ├─ ocs2_ros2                  # deb 模式下可跳过
    └─ robot-descriptions-common  # deb 模式下可跳过
```

### 常见问题
- SSH 权限：若克隆/更新失败，请确认本机 SSH key 已添加到 GitHub，并能通过 `ssh -T git@github.com` 握手。
- 网络问题：可重试或改用代理；必要时改为 HTTPS 克隆。
- 切到 arms=deb 后若仍有 `install/ht_ros2_control` 残留，会遮住系统 deb 插件；删除该目录后重新 `source /opt/ros/jazzy/setup.bash` 与 workspace `install/setup.bash`。

---

## C. 维护者：发布打包（`release.sh`）

在开发机上生成可发给现场的 zip（在临时目录打包，**不改动当前工作区**）：

```bash
# 含 .git（现场可 git pull 更新脚本；体积较大）
./release.sh --package

# 不含 .git（纯快照，体积更小；需指定架构）
./release.sh --package-no-git --arch amd64
./release.sh --package-no-git --arch arm64
```

也可 `./release.sh` → 菜单 **4) 含 .git** / **5) 不含 .git**。

发布包会：
1. 保留 `src/robot-descriptions-ht` 源码；其余子模块改为占位（由 deb 提供）
2. 下载目标架构最新核心 deb 到 `.deb_cache/`
3. 输出到 `dist/ht_deploy_ws_<时间>_<架构>[_nogit].zip`

现场使用见上文「快速部署方式」。

---

## 1. 安装 RMW Zenoh C++

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

## 2. 程序编译与仿真验证
### 2.1 依赖安装
* Rosdep 依赖安装
```bash
cd ~/ht-deploy-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2.2 程序编译（推荐：使用 quick_start.sh）

本工作空间已经提供一键脚本 `quick_start.sh`，用于**按场景编译**与**按模式启动**（单臂 / 双臂，仿真 / 真机）。

```bash
cd ~/ht-deploy-ws
chmod +x ./quick_start.sh
./quick_start.sh
```

- 在菜单中选择 **`1) 编译 (Build)`**
  - **`1) 编译仿真所需包`**：用于仿真/开发（不依赖真机驱动）
  - **`2) 编译真机所需包`**：用于连接真机（`arms-full` deb 已含 `ht_ros2_control` 时通常只需编描述包）

<details>
<summary><strong>（可选）手动编译命令</strong></summary>

```bash
cd ~/ht-deploy-ws
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
cd ~/ht-deploy-ws
# 真机所需包（ht_ros2_control 若已由 arms-full deb 提供则可省略）
colcon build --packages-up-to \
  ht_ros2_control \
  ocs2_arm_controller \
  panthera_ht_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

</details>

### 2.3 仿真验证
#### 2.3.1 模型可视化
```bash
source ~/ht-deploy-ws/install/setup.bash
ros2 launch robot_common_launch manipulator.launch.py robot:=panthera_ht
```

双臂：
```bash
ros2 launch robot_common_launch manipulator.launch.py robot:=panthera_ht type:=dual
```

**双臂间距**：只改一处即可，文件为
`src/robot-descriptions-ht/panthera_ht_description/xacro/robot.xacro`
中的 `left_mount_xyz` / `right_mount_xyz`（默认约为 `0 ±0.35 0`，单位 m）。
姿态用 `left_mount_rpy` / `right_mount_rpy`。

当前 `ocs2_arm_controller` 启动时会从同一份 `xacro/robot.xacro` 生成规划 URDF
（缓存到 `/tmp/...`）。
改默认值后重新编译/安装描述包（或 `--symlink-install` 下直接重启 launch）即可对可视化与 OCS2 同时生效。
日常仿真/真机控制以 xacro 为准。
更完整说明见 `panthera_ht_description` 子模块 README 的 *Mount parameters* 一节。

#### 2.3.2 启动仿真中的控制
推荐直接用 `quick_start.sh` 启动（会自动 `source install/setup.bash`，前提是已成功编译生成 `install/`）。

```bash
cd ~/ht-deploy-ws
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - 选择单臂或双臂
  - 选择 **`1) 仿真 (Simulation / mock_components)`**

<details>
<summary><strong>（可选）手动启动仿真控制</strong></summary>

```bash
source ~/ht-deploy-ws/install/setup.bash
# 单臂
ros2 launch ocs2_arm_controller demo.launch.py robot:=panthera_ht
# 双臂
ros2 launch ocs2_arm_controller demo.launch.py robot:=panthera_ht type:=dual
```

</details>

#### 2.3.3 启动真机的控制

**启动真机前先确认电机串口：**

```bash
# 1) 查看设备是否存在（应列出 /dev/ttyACM0 等）
ls /dev/ttyACM*

# 2) 若无输出：检查 USB 线、供电与驱动；确认 Panthera.yaml 中 Serial_Type 为 /dev/ttyACM
# 3) 有设备后赋予当前用户读写权限（每次插拔后可能需重新执行）
sudo chmod a+rw /dev/ttyACM*
```

也可将用户加入 `dialout` 组后重新登录，减少反复 `chmod`：
`sudo usermod -aG dialout $USER`

然后：

```bash
cd ~/ht-deploy-ws
./quick_start.sh
```

- 选择 **`2) 启动 (Launch)`**
  - 选择单臂或双臂
  - 选择 **`2) 真机 (Real Hardware)`**

<details>
<summary><strong>（可选）手动启动真机控制</strong></summary>

```bash
source ~/ht-deploy-ws/install/setup.bash
# 单臂（请显式 type:=single）
ros2 launch ocs2_arm_controller demo.launch.py robot:=panthera_ht type:=single hardware:=real
# 双臂
ros2 launch ocs2_arm_controller demo.launch.py robot:=panthera_ht type:=dual hardware:=real
```

</details>

#### 2.3.4 手柄遥操作（Joystick Teleop）

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
source ~/ht-deploy-ws/install/setup.bash
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

## 3. 子模块说明

- **arms_ros2_control** - 机械臂通用 ROS2 控制（含 `arms_teleop`）；deb 可用 `arms-ros2-control-full`
- **ht-ros2-control** - Panthera HT 硬件驱动（`ht_ros2_control`）；**已包含在 arms-full deb 中**
- **ocs2_ros2** - OCS2 的 ROS2 版本（MPC 控制框架）
- **robot-descriptions-ht** - HighTorque 描述仓库（含 `panthera_ht_description`，发布包保留源码）
- **robot-descriptions-common** - 通用机器人组件（夹爪、相机、launch 等）
