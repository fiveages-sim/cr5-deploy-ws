# open-deploy-ws (`feature/wujihand2`)

Lean Wuji Hand2 deployment workspace: arm control core + [robot-descriptions-common](https://github.com/fiveages-sim/robot-descriptions-common) (dexhands / wuji) + [wujihand2-ros2-control](https://github.com/fiveages-sim/wujihand2-ros2-control) (Hand2 Ethernet HI) + OCS2.

See `main` for the full workspace; other lean branches include `dobot-cr5`, `arx-lift2s`, and `panthera-ht`.

## Workspace layout

```
wuji_ws/
├── src/
│   ├── arms_ros2_control/          # Arm / hand controllers (basic_joint_controller)
│   ├── robot-descriptions-common/  # Shared descriptions (tracks feature/wuji; dexhands/wuji_description)
│   ├── wujihand2-ros2-control/     # Wuji Hand2 ros2_control hardware interface (Ethernet + wuji-sdk)
│   └── ocs2_ros2/                  # OCS2 MPC framework
├── config/
│   ├── quick_start.conf            # Build / launch menu config
│   └── hand.local.template.conf    # Local Hand2 IP template (copy to hand.local.conf)
├── init_repo.sh
├── quick_start.sh                  # Build and launch (recommended)
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
| common | `src/robot-descriptions-common` | `ros-jazzy-robot-descriptions-common` | source |

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

### Wuji C SDK and Hand2 hardware interface

### Wuji C SDK and Hand2 hardware interface

`src/wujihand2-ros2-control` is a **source-only** submodule (no deb). The official C SDK is vendored at:

`src/wujihand2-ros2-control/external/wuji-sdk-c/` (`include/` + `lib/x86_64` + `lib/aarch64`)

`WUJI_SDK_ROOT` is not required by default. Optional override for debugging:

```bash
export WUJI_SDK_ROOT=/path/to/wuji-sdk-c-*-linux-gnu   # optional
```

Optional IPs: see **Device connection** below, or copy `config/hand.local.template.conf` → `config/hand.local.conf`.

## Build and launch (`quick_start`)

```bash
./quick_start.sh
# 1) Build — package lists in config/quick_start.conf
# 2) Launch — left/right hand, mock or real
```

### Device connection (real hardware)

**`device_address` is optional.** Priority:

1. `device_address:=IP:port` — direct connect (recommended for dual-hand / production)
2. `serial_number:=SN` — by serial (when `device_address` is empty)
3. Both empty — **`wuji_scan()`** on the LAN, then match **`direction`** (1=left, -1=right)

```bash
ros2 launch wujihand2_ros2_control hand2.launch.py hardware:=real direction:=1
ros2 launch wujihand2_ros2_control hand2.launch.py \
  hardware:=real direction:=1 device_address:=192.168.1.110:50001
ros2 launch wujihand2_ros2_control hand2.launch.py \
  hardware:=real direction:=1 serial_number:=YOUR_SN
```

In `./quick_start.sh` real mode:

- **Launch item “last real hand”** — remembers side + IP (`LAST_REAL_*`; sim does not overwrite)
- **Connect menu**
  1. SDK scan and pick — lists SN + real `IP:port` (recommended)
  2. Scan at launch — omit address; hardware matches `direction`
  3. Type `IP:port` manually
  0. Back

Prefer scan: some devices advertise `:7447` instead of the docs' `:50001`.

Mock:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=wuji type:=hand2
```

Joint index calibration: `src/wujihand2-ros2-control/CALIBRATION.md`.

## Tested environment

- **ROS 2 Jazzy** (Ubuntu 24.04)

## License

Apache License 2.0. See [LICENSE](LICENSE) for details.
