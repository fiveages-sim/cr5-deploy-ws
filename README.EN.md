# Fairino ART7 Manipulator ROS 2 Deployment Workspace

This repository deploys a ROS 2 workspace for the Fairino ART7 manipulator, a complete control ecosystem based on the OCS2 MPC control framework.

### Prerequisites
- ROS 2 Jazzy installed
- **Quick deployment (release zip)**: Git / SSH generally not required
- **Development workflow (git clone)**: Git SSH keys configured with access to the relevant private repositories; Git 2.30+ recommended

## Three core scripts

| Script | Role |
|--------|------|
| [`init_repo.sh`](init_repo.sh) | **Repository initialization**: choose source/deb per module, fetch submodules, install/switch/uninstall core debs, rosdep |
| [`quick_start.sh`](quick_start.sh) | **Day-to-day build and launch**: scenario-based build (simulation / real hardware), launch single/dual arm (simulation / real hardware / joystick) |
| [`release.sh`](release.sh) | **Release and on-site debs**: download/install/uninstall core debs; maintainer packaging of release zips (with/without `.git`) |

Helper scripts: [`scripts/install_core_debs.sh`](scripts/install_core_debs.sh), [`scripts/uninstall_core_debs.sh`](scripts/uninstall_core_debs.sh), shared function library [`scripts/lib_deb_common.sh`](scripts/lib_deb_common.sh); version mapping in [`deb_versions.conf`](deb_versions.conf).

Core deb install order: `ocs2` → `robot-descriptions-common` → `arms-ros2-control-full` (**includes `fairino_ros2_control`**).  
When `arms=deb`, the `fairino-ros2-control` source tree is not fetched; the Fairino description package `robot-descriptions-fairino` is still built from source.

---

## A. Quick deployment (recommended on site)

Intended for “extract and run”: the release package already contains (or can download) core debs; you only need to install debs, build Fairino descriptions, and launch.

```bash
# 1) Extract the release zip to the target directory, e.g. ~/fairino-deploys-ws
cd ~/fairino-deploys-ws

# 2) Install core debs (requires sudo; if .deb_cache/ is already in the zip, install directly)
./release.sh --install
# If debs are missing or need updating:
# ./release.sh --download && ./release.sh --install

# 3) Build and launch
source /opt/ros/jazzy/setup.bash
./quick_start.sh
# → 1) Build → simulation or real-hardware packages
# → 2) Launch → single/dual arm / simulation or real hardware
```

You can also run `./release.sh` for an interactive menu (download / install / uninstall debs).

Optional: install RMW Zenoh (see “Install RMW Zenoh C++” below).

---

## B. Development workflow (git clone)

For editing submodules, switching source/deb, and following mainline development.

### 1. Clone the repository

```bash
cd ~
git clone -b fairino-art7 git@github.com:fiveages-sim/open-deploy-ws.git fairino-deploys-ws
cd ~/fairino-deploys-ws
```

### 2. Initialize (`init_repo.sh`)

```bash
./init_repo.sh
```

| Option | Description |
|--------|-------------|
| 1) Initialize | Per-module source/deb; **defaults ocs2/arms/common=deb**; when arms=deb you can choose `full` (includes `fairino_ros2_control`, skips its source) or `standard` (requires source init of `fairino-ros2-control`); when arms=source, nested `arms_ros2_control/hardwares/*` submodules are not fetched |
| 2) Switch | Source ↔ deb (cleans conflicting directories or uninstalls debs; switching arms→deb also offers the variant choice) |
| 3) Debs only | Install/update core debs only, without fetching Git submodules (when the list includes arms, the variant can also be chosen) |
| 4) Uninstall debs | Detects and lists installed ocs2 / common / arms(-full), then lets you choose packages to uninstall |
| 5) rosdep | Runs rosdep only on source submodule paths |

Initialization also asks for the **deb release channel**: `1) latest` / `2) pre-release` / `3) conf` (see `deb_versions.conf`).

**arms deb variants (full / standard)**:
- For channels `latest` / `pre-release`, arms=deb asks whether to install `ros-jazzy-arms-ros2-control-full` (default, includes `fairino_ros2_control`) or `ros-jazzy-arms-ros2-control` (standard package).
- For channel `conf`, the arms-line variant in `deb_versions.conf` is read and you are prompted to confirm/switch (switch applies to this run only; the config file is not modified).

**Submodule update safety**: `init_repo.sh` only fast-forwards each submodule to the latest commit on its **current branch** and **does not switch branches**; local submodule changes are `git stash`ed before update and restored afterward — your edits are never wiped, and the workspace’s own branch is not changed.

**Recommended development start (core via deb, build only Fairino descriptions):**

```bash
./init_repo.sh                    # choose deb for ocs2/arms/common (channel as needed)
source /opt/ros/jazzy/setup.bash
./quick_start.sh                  # build simulation / real-hardware packages
```

To edit `arms` / `ocs2` / `fairino_ros2_control` source: use `init_repo.sh` option 2 to switch the corresponding module to **source**, then build.

### 3. Update submodules

```bash
# Update only Fairino descriptions still kept as source (common in deb mode)
git submodule update --remote src/robot-descriptions-fairino

# If arms etc. are source, then as needed:
# git submodule update --remote
```

### Directory layout (excerpt)

```
fairino-deploys-ws/
├── init_repo.sh                  # Init / source-deb switching
├── quick_start.sh                # Build and launch
├── release.sh                    # On-site deb install / maintainer packaging
├── deb_versions.conf             # Deb version and repository mapping
├── scripts/
│   ├── install_core_debs.sh   # Core deb download/install (supports --arms-variant)
│   ├── uninstall_core_debs.sh
│   └── lib_deb_common.sh      # Shared helpers (sourced by release.sh / install_core_debs.sh)
└── src/
    ├─ robot-descriptions-fairino # Usually source (Fairino models)
    ├─ fairino-ros2-control       # Provided by arms-full when arms=deb; may be skipped
    ├─ arms_ros2_control          # May be skipped in deb mode
    ├─ ocs2_ros2                  # May be skipped in deb mode
    └─ robot-descriptions-common  # May be skipped in deb mode
```

### Troubleshooting
- SSH permissions: if clone/update fails, confirm your local SSH key is added to GitHub and that `ssh -T git@github.com` succeeds.
- Network issues: retry or use a proxy; switch to HTTPS cloning if necessary.
- After switching to arms=deb, leftover `install/fairino_ros2_control` can shadow the system deb plugin; delete that directory, then re-`source /opt/ros/jazzy/setup.bash` and the workspace `install/setup.bash`.

---

## C. Maintainer: release packaging (`release.sh`)

On a development machine, produce a zip for on-site use (packaged in a temporary directory; **does not modify the current workspace**):

```bash
# With .git (on-site can git pull script updates; larger size)
./release.sh --package

# Without .git (pure snapshot, smaller; architecture required)
./release.sh --package-no-git --arch amd64
./release.sh --package-no-git --arch arm64
```

Or run `./release.sh` for an interactive menu:
- **1) Download deb dependencies**: fetch into `.deb_cache/` per `deb_versions.conf`; if the cache already has matching-version debs, reuse them without re-downloading
- **2) Install deb dependencies**: install from `.deb_cache/` (requires sudo)
- **3) One-shot uninstall debs** (reverse install order, requires sudo)
- **4) Package release zip (with .git)** / **5) Package release zip (without .git, smaller)**

The release package:
1. Keeps `src/robot-descriptions-fairino` as source; other submodules become placeholders (provided by debs)
2. Downloads the latest core debs for the target architecture into `.deb_cache/` (reuses matching cache if present)
3. Writes `dist/fairino_deploy_ws_<timestamp>_<arch>[_nogit].zip`

On-site usage is described under “Quick deployment” above.

---

## 1. Install RMW Zenoh C++

Deployment hosts should use RMW Zenoh to avoid DDS message interference from other devices on the LAN.
* Install
  ```bash
  sudo apt install ros-jazzy-rmw-zenoh-cpp
  ```
* Configure bashrc
  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ```
* To temporarily disable Zenoh (restore default DDS), run in the current terminal:
  ```bash
  unset RMW_IMPLEMENTATION
  ```
* To disable permanently, remove the `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` line from `~/.bashrc`
* When launching via `launch` files in `robot-descriptions-common`, a Zenoh router is started automatically

## 2. Build and simulation verification

### 2.1 Install dependencies
* Rosdep dependency installation
```bash
cd ~/fairino-deploys-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2.2 Build (recommended: `quick_start.sh`)

This workspace provides a one-shot script `quick_start.sh` for **scenario-based builds** and **mode-based launch** (single arm / dual arm, simulation / real hardware).

```bash
cd ~/fairino-deploys-ws
chmod +x ./quick_start.sh
./quick_start.sh
```

- In the menu, select **`1) Build`**
  - **`1) Build simulation packages`**: for simulation / development (no real-hardware drivers)
  - **`2) Build real-hardware packages`**: for connecting to real hardware (when the `arms-full` deb already includes `fairino_ros2_control`, you typically only need to build description packages)

<details>
<summary><strong>(Optional) Manual build commands</strong></summary>

```bash
cd ~/fairino-deploys-ws
# Simulation packages
colcon build --packages-up-to \
  ocs2_arm_controller \
  art7_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

```bash
cd ~/fairino-deploys-ws
# Real-hardware packages (omit fairino_ros2_control if already provided by the arms-full deb)
colcon build --packages-up-to \
  fairino_ros2_control \
  ocs2_arm_controller \
  art7_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

</details>

### 2.3 Simulation verification

#### 2.3.1 Model visualization
```bash
source ~/fairino-deploys-ws/install/setup.bash
ros2 launch robot_common_launch manipulator.launch.py robot:=art7
```

Dual arm:
```bash
ros2 launch robot_common_launch manipulator.launch.py robot:=art7 type:=dual
```

**End-effector configuration**: left/right end-effector types are selected in
`src/robot-descriptions-fairino/art7_description/xacro/robot.xacro` via the
`type` / `left_type` / `right_type` arguments (e.g. `rg75`, `ag2f90`, etc.; see the end-effector key table in the `art7_description` README);
left/right TCP offsets use `left_tcp_offset_xyz/rpy` / `right_tcp_offset_xyz/rpy`.

At launch, `ocs2_arm_controller` generates the planning URDF from the same `xacro/robot.xacro`
(cached under `/tmp/...`).
After changing defaults, rebuild/install the description package (or restart launch under `--symlink-install`) so visualization and OCS2 both pick up the change.
Day-to-day simulation / real-hardware control follows the xacro.
See the `art7_description` submodule README for full detail.

#### 2.3.2 Launch simulation control
Prefer launching via `quick_start.sh` (it automatically `source`s `install/setup.bash`, provided a successful build has produced `install/`).

```bash
cd ~/fairino-deploys-ws
./quick_start.sh
```

- Select **`2) Launch`**
  - Choose single or dual arm
  - Select **`1) Simulation (Simulation / mock_components)`**

<details>
<summary><strong>(Optional) Manual simulation control launch</strong></summary>

```bash
source ~/fairino-deploys-ws/install/setup.bash
# Single arm
ros2 launch ocs2_arm_controller demo.launch.py robot:=art7
# Dual arm
ros2 launch ocs2_arm_controller demo.launch.py robot:=art7 type:=dual
```

</details>

#### 2.3.3 Launch real-hardware control

**Before launching real hardware, confirm the Fairino controller network reachability:**

```bash
# 1) Check that the controller IP is reachable (default 192.168.58.1, port 8081)
ping 192.168.58.1
# or test the TCP port directly:
timeout 3 bash -c "exec 3<>/dev/tcp/192.168.58.1/8081" && echo OK

# 2) If unreachable: check the network cable, controller power, and the device_ip parameter of fairino_ros2_control
# 3) If the controller IP differs from the default, override with environment variables:
FAIRINO_IP=<controller-ip> FAIRINO_PORT=<port> ./quick_start.sh
```

Then:

```bash
cd ~/fairino-deploys-ws
./quick_start.sh
```

- Select **`2) Launch`**
  - Choose single or dual arm
  - Select **`2) Real Hardware`** (TCP reachability is checked automatically before launch)

<details>
<summary><strong>(Optional) Manual real-hardware control launch</strong></summary>

```bash
source ~/fairino-deploys-ws/install/setup.bash
# Single arm (explicitly set type:=single)
ros2 launch ocs2_arm_controller demo.launch.py robot:=art7 type:=single hardware:=real
# Dual arm
ros2 launch ocs2_arm_controller demo.launch.py robot:=art7 type:=dual hardware:=real
```

</details>

#### 2.3.4 Joystick teleop

Joystick teleop is started separately from the control process (same as `fa_w2_ws`): start single/dual-arm control first, then the joystick.

Dependency (if not installed):
```bash
sudo apt install ros-jazzy-joy
```

Usage:
1. Terminal A: start single- or dual-arm control (simulation / real hardware)
2. Terminal B: `./quick_start.sh` → **`2) Launch`** → **`3) Joystick teleop`**

Or manually:
```bash
source ~/fairino-deploys-ws/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py
# With multiple joysticks, specify the device:
# ros2 launch arms_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1
```

Common operations (Xbox-style controller):
- **Right stick click**: enable/disable teleop (disabled by default)
- **LB + A**: HOLD → HOME
- **LB + START**: HOLD → OCS2 (drag the end-effector after entering MPC)
- **LB + B**: → HOLD
- **Left/right sticks**: translate / rotate
- **A**: switch left/right arm (dual arm)
- **X / LT / RT**: gripper open/close or aperture ratio

## 3. External control interface (topics / services)

This section describes how external ROS 2 nodes send **end-effector pose commands**, switch **FSM states**, and which service / action / feedback interfaces are available.

**Control pipeline overview**: an external node publishes the target pose to the target topic; the controller’s `PoseBasedReferenceManager` subscribes and converts it into an OCS2 MPC target trajectory. **Prerequisite: the controller must be in OCS2(3) state** to track target poses (see “FSM state switching” below).

> Frames: the OCS2 base frame is `base_link`; left end-effector `left_gripper_center`, right end-effector `right_gripper_center`.

### 3.1 FSM state switching

FSM state is commanded and read via topics:

| Topic | Message type | Role |
|-------|--------------|------|
| `/fsm_command` | `std_msgs/msg/Int32` | **Send state-switch commands** (publish 1/2/3/4; see table below) |
| `/fsm_state` | `std_msgs/msg/Int32` | **Read current FSM state** (same value meanings as above) |

`/fsm_command` values:

| Value | State | Description |
|-------|-------|-------------|
| `1` | HOME | Home / return to zero |
| `2` | HOLD | Hold (effective e-stop/pause; collisions detected in OCS2 also auto-switch here) |
| `3` | **OCS2** | **MPC tracks target poses** (required before sending pose commands) |
| `4` | MOVEJ | Joint motion |

Example:

```bash
ros2 topic pub -1 /fsm_command std_msgs/msg/Int32 "{data: 3}"
```

### 3.2 Target pose topics (send end-effector pose commands)

| Topic | Message type | Role |
|-------|--------------|------|
| `/left_target` | `geometry_msgs/msg/Pose` | Left-arm target pose (also used for single-arm robots). No header; **interpreted directly in `base_link`**; takes effect on receipt |
| `/left_target/stamped` | `geometry_msgs/msg/PoseStamped` | Left-arm target pose. `header.frame_id` may be any frame; the controller TF-transforms to `base_link` and moves with **moveL interpolation** |
| `/right_target` | `geometry_msgs/msg/Pose` | Right-arm target pose (dual-arm mode) |
| `/right_target/stamped` | `geometry_msgs/msg/PoseStamped` | Right-arm target pose (dual-arm mode); same TF and interpolation support |
| `/dual_target/stamped` | `nav_msgs/msg/Path` | Set both arms at once: 2–3 poses, `[left, right]` or `[left, right, body]`, planned with unified interpolation |
| `/target_path` | `nav_msgs/msg/Path` | Multi-waypoint pose path (continuous trajectory) |

- External nodes should **prefer `*_target/stamped`**: specify the frame, automatic TF, and built-in smooth interpolation.
- Note: plain `*_target` (Pose) is ignored while moveL interpolation is active (interpolation takes priority); for high-rate commands, use `/left_target/stamped` consistently.

Example:
```bash
ros2 topic pub -1 /left_target geometry_msgs/msg/Pose \
  "{position: {x: 0.30, y: 0.35, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

ros2 topic pub -1 /left_target/stamped geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.30, y: 0.35, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 3.3 Incremental control (joystick / keyboard style)

| Topic | Message type | Role |
|-------|--------------|------|
| `/control_input` | `arms_ros2_control_msgs/msg/Inputs` | Incremental control: `x/y/z/roll/pitch/yaw` are displacement/angle **deltas**; `target=1/2` selects left/right arm; `hand_command` controls the gripper (`nan`=no command, `0/1`=open/close, otherwise 0–1 aperture ratio) |

Processed by `arms_target_manager` and converted to `*_target` publications. Suited to continuous fine adjustment — **not** absolute pose commands.

### 3.4 Feedback topics (read)

| Topic | Message type | Role |
|-------|--------------|------|
| `/left_current_pose` | `geometry_msgs/msg/PoseStamped` | Left-arm current end-effector pose |
| `/right_current_pose` | `geometry_msgs/msg/PoseStamped` | Right-arm current end-effector pose |
| `/body_current_pose` | `geometry_msgs/msg/PoseStamped` | Current torso pose |
| `/left_current_target` | `geometry_msgs/msg/PoseStamped` | Left-arm current target pose (actual MPC tracking target) |
| `/right_current_target` | `geometry_msgs/msg/PoseStamped` | Right-arm current target pose |

### 3.5 Gripper control (optional)

| Topic | Message type | Role |
|-------|--------------|------|
| `/left_gripper_controller/target_command` | `std_msgs/msg/Int32` | Left gripper open/close: `0` close / `1` open |
| `/left_gripper_controller/target_percent` | `std_msgs/msg/Float64` | Left gripper aperture ratio (0–1) |
| `/right_gripper_controller/target_command` | `std_msgs/msg/Int32` | Right gripper open/close: `0` close / `1` open |
| `/right_gripper_controller/target_percent` | `std_msgs/msg/Float64` | Right gripper aperture ratio (0–1) |

> Topic / service / action names and types above were verified against a live system (`ros2 topic/service/action list`). Single-arm mode retains left-arm topics only; dual-arm mode exposes both arms.

## 4. Submodule notes

- **arms_ros2_control** — Generic ROS 2 manipulator control (includes `arms_teleop`); deb available as `arms-ros2-control-full`
- **fairino-ros2-control** — Fairino hardware driver (`fairino_ros2_control`, connects to the controller over TCP); **included in the arms-full deb**
- **ocs2_ros2** — ROS 2 port of OCS2 (MPC control framework)
- **robot-descriptions-fairino** — Fairino description repo (includes `art7_description`; kept as source in release packages)
- **robot-descriptions-common** — Shared robot components (grippers, cameras, launch, etc.)
