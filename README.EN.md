# open-deploy-ws

A ROS 2 deployment workspace that integrates dual-arm manipulator control, robot description models, and the OCS2 MPC framework.

## About other branches

`main` targets a full / general-purpose deployment workspace. The repository also provides lean branches for specific robot models that **pull only the packages and submodules required for that setup**, reducing clone and deployment size:

- **`dobot-cr5`**: Dobot CR5 deployment workspace, containing only CR5-related description, driver, and control dependencies
- **`arx-acone`**: ARX Acone deployment workspace, containing only Acone-related description, driver, and control dependencies

When cloning, specify the branch and use a directory name that matches the robot model, for example:

```bash
git clone -b dobot-cr5 git@github.com:fiveages-sim/open-deploy-ws.git dobot_cr5_ws
git clone -b arx-acone git@github.com:fiveages-sim/open-deploy-ws.git arx_acone_ws
```

## Workspace layout

```
open-deploy-ws/
├── src/
│   ├── arms_ros2_control/     # Core arm control (controllers / commands / hardware interfaces / shared libs)
│   ├── robot-descriptions/    # Robot descriptions (common / manipulator / humanoid)
│   └── ocs2_ros2/             # OCS2 MPC framework
├── init_repo.sh               # One-shot initialization (visibility + per-module source/deb)
└── README.md
```

## Quick start

Before you begin, install the **ROS 2 Jazzy and rosdep environment** (Ubuntu 24.04):

```bash
# 1. Install ROS 2 management tools (fishros)
wget http://fishros.com/install -O fishros && bash fishros

# 2. Install ROS 2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop

# 3. Initialize rosdep (required the first time you use rosdep on this machine)
sudo rosdep init
rosdep update
```

After completing the steps above, initialize the repository:

```bash
git clone git@github.com:fiveages-sim/open-deploy-ws.git ros2_ws
cd ros2_ws
./init_repo.sh
```

### `init_repo.sh` usage

**1) Initialize the workspace (recommended)**

Two selection steps:

1. **Nested visibility** (affects only nested submodules inside each repository; the three top-level repositories are always initialized when source is selected)
   - **public**: public nested submodules only — suitable for external users
   - **private**: includes private nested submodules — requires internal repository access
2. **Core module install mode** (per module: `d` = deb / `s` = source; press Enter to accept the default)

| Module | Path | Deb package | Default |
|--------|------|-------------|---------|
| ocs2 | `src/ocs2_ros2` | `ros-jazzy-ocs2` | **deb** |
| arms | `src/arms_ros2_control` | `ros-jazzy-arms-ros2-control` | source |
| common | `src/robot-descriptions/common` | `ros-jazzy-robot-descriptions-common` | source |

Recommended production combination (script default): **ocs2=deb, arms/common=source**. Full source means select `s` for all three modules; full deb means select `d` for all three.

**2) Switch module install mode**

Detects the current dpkg / source-tree state, switches each module between source ↔ deb (cleans conflicting source trees or uninstalls the corresponding deb), then resynchronizes to the target mode.

**3) Install / update core debs only**

Skips Git fetch; you may enter `ocs2`, `common`, `arms` (comma-separated); pressing Enter means all. You can also run directly:

```bash
./scripts/install_core_debs.sh --only ocs2
```

**4) Uninstall core debs**

You may specify packages or uninstall all; you can also run `./scripts/uninstall_core_debs.sh --only ocs2`.

**5) Run rosdep only**

Installs system dependencies for the entire `src` tree (does not fetch submodules or install debs):

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Deb versions and repositories are defined in [`deb_versions.conf`](deb_versions.conf); nested public/private visibility is defined in [`submodules_visibility.conf`](submodules_visibility.conf).

### What the script does next

1. Sync and initialize top-level submodules selected as **source** (skip those selected as deb; if source trees already exist, prompt to clean them)
2. Initialize nested submodules according to visibility and configuration (skipped when the parent repository or common is installed as deb)
3. Check out configured branches for source submodules and pull the latest commits
4. Run `rosdep install` on source paths
5. Install packages selected as deb (order: ocs2 → common → arms)
6. Persist choices to local `.core_module_mode` (gitignored) for use as defaults next time

## Tested environment

- **ROS 2 Jazzy** (Ubuntu 24.04)

## License

Apache License 2.0. See [LICENSE](LICENSE) for details.
