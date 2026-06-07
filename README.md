# open-deploy-ws

ROS2 部署工作空间，集成双臂机械臂控制、机器人描述模型与 OCS2 MPC 框架。

## 工作空间结构

```
open-deploy-ws/
├── src/
│   ├── arms_ros2_control/     # 机械臂控制核心（控制器 / 命令 / 硬件接口 / 公共库）
│   ├── robot-descriptions/    # 机器人描述（common / manipulator / humanoid）
│   └── ocs2_ros2/             # OCS2 MPC 框架
├── init_repo.sh               # 一键初始化 + 编译
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

**运行 `init_repo.sh` 时可选择初始化模式：**

**源码模式**
- **public**（选项 1，默认）：仅初始化公开子模块，适用于外部用户，无需私有仓库权限。
- **private**（选项 2）：初始化所有子模块（含私有仓库），需要具备内部仓库访问权限。

**deb 模式**
- **快速模式**（选项 3）：public 子模块 + 通过 deb 安装 `ocs2_ros2`、`arms_ros2_control`、`robot-descriptions/common`，无需克隆和编译这三个大型仓库。deb 默认从 GitHub 最新 release 下载，安装顺序为 ocs2 → common → arms，仓库配置见 [`deb_versions.conf`](deb_versions.conf)。
- **仅 deb**（选项 4）：只下载并安装核心 deb 包，**不拉取 Git 子模块**，适合已初始化工作空间后单独更新 deb。也可直接运行 `./scripts/install_core_debs.sh`。
- **卸载 deb**（选项 5）：一键卸载上述三个 deb 包（逆序）。也可直接运行 `./scripts/uninstall_core_debs.sh`。

**脚本随后会：**
1. 同步并初始化顶层子模块（快速模式跳过 ocs2 / arms 子模块；若此前已初始化会提示清理）
2. 根据所选模式与 `submodules_visibility.conf` 初始化嵌套子模块（快速模式跳过 `common`）
3. 将所有子模块切换到配置的分支并拉取最新提交
4. 运行 `rosdep install` 安装系统依赖
5. **快速模式**：运行 `scripts/install_core_debs.sh` 安装预编译 deb；**源码模式**：自行 `colcon build` 编译工作空间


## 测试环境

- **ROS2 Jazzy**（Ubuntu 24.04）


## License

Apache License 2.0. See [LICENSE](LICENSE) for details.
