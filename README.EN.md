# ARX Lift2S / ACone Manipulator ROS 2 Deployment Workspace

![ARX Lift2S](.images/arx_lift2s.jpg)

This repository deploys a ROS 2 workspace for ARX Lift2S (with lift axis) and ACone / X5 manipulators, based on the OCS2 MPC control framework.

## Table of contents

1. [Prerequisites](#1-prerequisites)
2. [Deployment](#2-deployment)
   - [2.1 Overview](#21-overview)
   - [2.2 Release zip deployment (deb, recommended for field)](#22-release-zip-deployment-deb-recommended-for-field)
   - [2.3 Full source: clone and submodules](#23-full-source-clone-and-submodules)
3. [Environment (RMW Zenoh)](#3-environment-rmw-zenoh)
4. [Build](#4-build)
   - [4.1 Install dependencies](#41-install-dependencies)
   - [4.2 Build (recommended: `quick_start.sh`)](#42-build-recommended-quick_startsh)
5. [Launch (use quick_start)](#5-launch-use-quick_start)
   - [5.1 Simulation / visualization](#51-simulation--visualization)
   - [5.2 Real-hardware launch](#52-real-hardware-launch)
   - [5.3 Joystick teleop](#53-joystick-teleop)
   - [5.4 Viser and VR teleop](#54-viser-and-vr-teleop)
6. [Submodule notes](#6-submodule-notes)

## 1. Prerequisites

- Git SSH keys configured with access to the relevant private repositories
- Git installed on the system (2.30+ recommended)
- ROS 2 Jazzy (Ubuntu 24.04)

## 2. Deployment

### 2.1 Overview

| Method | Use case | Main steps |
|--------|----------|------------|
| **Release zip (recommended for field)** | On-robot / production; controllers and HI from deb | Unzip → `release.sh --install` → `quick_start.sh` |
| **Full source** | Development, debugging controllers / HI | `./init_repo.sh` (source/deb menu) → `quick_start.sh` |

The release zip keeps **only `robot-descriptions-arx` source**, plus three pre-downloaded deb packages (ocs2, robot-descriptions-common, **arms-ros2-control-full**). Directories such as `arx-ros2-control` are cleared to placeholders when packaging; real-hardware HI (`arx_ros2_control`) is provided by the **arms-full deb**.

#### Directory layout (excerpt)

```
lift2s-ws/
  ├─ init_repo.sh              # Submodule init (source/deb menu; --init-release for descriptions only)
  ├─ release.sh                # deb install and release packaging (maintainers)
  ├─ quick_start.sh            # Build and launch
  ├─ config/quick_start.conf   # RELEASE_SUBMODULE_PATHS, etc.
  ├─ deb_versions.conf         # Fixed deb tags for release (conf channel)
  ├─ .deb_cache/               # Pre-bundled deb in release zip (install after unzip)
  └─ src/
      ├─ robot-descriptions-arx      # Source kept in release zip
      ├─ arx-ros2-control            # Placeholder (HI from arms-full deb; init source for dev)
      ├─ robot-descriptions-common   # Provided by deb in release zip
      ├─ arms_ros2_control           # Provided by deb in release zip (full includes arx_ros2_control)
      └─ ocs2_ros2                   # Provided by deb in release zip
```

### 2.2 Release zip deployment (deb, recommended for field)

After obtaining `lift2s-ws_*.zip` from maintainers, on the target machine:

```bash
unzip lift2s-ws_*.zip -d ~
cd ~/lift2s-ws

# 1) Install bundled deb (sudo required; if .deb_cache/ is empty or outdated, run ./release.sh --download first)
./release.sh --install
source /opt/ros/jazzy/setup.bash   # if not already in your shell config

# 2) Build description packages in the workspace, then launch
./quick_start.sh
#   Build → 1) Simulation / 2) Real hardware: deb mode builds only ARX description packages (HI from arms-full deb)
#   Launch → Lift2S split body / full body, etc.
```

`release.sh --install` installs deb from `.deb_cache/` in order:

1. `ros-jazzy-ocs2` ([legubiao/ocs2_ros2](https://github.com/legubiao/ocs2_ros2/releases))
2. `ros-jazzy-robot-descriptions-common` ([fiveages-sim/robot-descriptions-common](https://github.com/fiveages-sim/robot-descriptions-common/releases))
3. `ros-jazzy-arms-ros2-control-full` ([fiveages-sim/arms_ros2_control](https://github.com/fiveages-sim/arms_ros2_control/releases), **includes arx_ros2_control HI**)

Deb channels (packaging default **conf**, reads fixed tags from `deb_versions.conf`):

| Channel | Description |
|---------|-------------|
| **conf** | Fixed tags in `deb_versions.conf` (release packaging default) |
| **latest** | GitHub Latest stable release per repo |
| **pre-release** | Rolling pre-release |

```bash
./release.sh --download --release-channel latest
./release.sh --install
```


#### Post-deployment updates (git + deb)

When the release zip **includes the main repo `.git`**, you can pull scripts and config on-site; large components are still updated via deb.

```bash
cd ~/lift2s-ws
git pull --ff-only
./init_repo.sh --init-release    # or --update-release
./release.sh --download
./release.sh --install
./quick_start.sh   # Build → sim/real (deb mode: descriptions only), then launch
```

Submodule directories provided by deb are placeholders (`.gitkeep`) in the release zip; for full source development, see [2.3](#23-full-source-clone-and-submodules) below.

<details>
<summary><strong>Maintainers: create release zip</strong></summary>

On a dev machine (requires git, rsync, and network). Packaging runs in a **temporary directory** and **does not modify** your working tree. Submodules kept in the zip are defined by `config/quick_start.conf` → `RELEASE_SUBMODULE_PATHS` (currently only `robot-descriptions-arx`).

```bash
cd ~/lift2s-ws
./release.sh --package --release-channel conf              # includes .git; deb arch = host; default conf
./release.sh --package-no-git --arch amd64                 # no .git; x64
./release.sh --package-no-git --arch arm64                 # no .git; ARM
./release.sh --package --update-submodules                 # pull latest kept submodules when packaging
./release.sh --package --release-channel pre-release       # bundle pre-release deb
```

Artifact: `dist/lift2s-ws_<YYYYMMDD_HHMMSS>_<arch>[_nogit].zip`.

</details>

### 2.3 Full source: clone and submodules

#### Clone to ~/lift2s-ws

```bash
cd ~
git clone git@github.com:fiveages-sim/open-deploy-ws.git lift2s-ws
cd ~/lift2s-ws
```

#### Initialize and update submodules

```bash
./init_repo.sh
```

Interactive menu: per-module **source ↔ deb**, deb channels (conf / latest / pre-release), arms variant (default **full**).  
To update descriptions on a release deployment:

```bash
./init_repo.sh --init-release     # init robot-descriptions-arx only
./init_repo.sh --update-release   # pull latest description submodule
```

With `arms=deb(full)`, `src/arx-ros2-control` is skipped; with `arms=standard` or `arms=source`, HI source is initialized and external deps are checked.

#### Updating submodules later

```bash
git submodule update --remote
```



## 3. Environment (RMW Zenoh)

Deployment hosts should use RMW Zenoh to avoid DDS interference across the LAN.

* Install
  ```bash
  sudo apt install ros-jazzy-rmw-zenoh-cpp
  ```
* Configure bashrc
  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ```
* Temporary disable: `unset RMW_IMPLEMENTATION`
* **Start the Zenoh router before launch** (otherwise `controller_manager` stalls on
  `Waiting for data on 'robot_description' topic`)
  - Recommended: `./quick_start.sh` → Launch (pre-starts the router automatically)
  - For manual `ros2 launch`, open another terminal first: `ros2 run rmw_zenoh_cpp rmw_zenohd`

## 4. Build

### 4.1 Install dependencies

```bash
cd ~/lift2s-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4.2 Build (recommended: `quick_start.sh`)

After [§2 Deployment](#2-deployment) (release zip: `release.sh --install`; full source: `./init_repo.sh`):

```bash
cd ~/lift2s-ws
./quick_start.sh
```

- **`1) Build`**
  - **`1) Simulation packages`**: full source builds controllers + descriptions; **deb mode (arms-full) builds only the 3 ARX description packages**
  - **`2) Real-hardware packages`**: full source builds `arx_ros2_control` + controllers + descriptions; **deb + arms-full same as simulation (descriptions only, HI from deb)**; arms-standard additionally builds source `arx_ros2_control`

Package lists: `config/quick_start.conf` (`BUILD_*` / `BUILD_DEB_*`).

<details>
<summary><strong>(Optional) Manual build commands</strong></summary>

```bash
cd ~/lift2s-ws
# Simulation
colcon build --packages-up-to \
  ocs2_arm_controller \
  arx_acone_description \
  arx_lift2s_description \
  arx5_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install

# Real hardware (requires src/arx-ros2-control/external/{arx5-sdk,arx_lift_src})
colcon build --packages-up-to \
  arx_ros2_control \
  ocs2_arm_controller \
  arx_acone_description \
  arx_lift2s_description \
  arx5_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install

# Release zip / deb mode (after release.sh --install; arms-full includes HI — do not build arx_ros2_control)
colcon build --packages-up-to \
  arx5_description \
  arx_acone_description \
  arx_lift2s_description \
  --symlink-install
```

</details>

## 5. Launch (use quick_start)

### 5.1 Simulation / visualization

```bash
source ~/lift2s-ws/install/setup.bash
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_acone
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_lift2s
```

Prefer `./quick_start.sh` → **`2) Launch`**:

| Option | Description |
|--------|-------------|
| 1) Single-arm X5 | `robot:=arx5`; **on real hardware, also choose left arm (can1) / right arm (can3)** |
| 2) Dual-arm ACone | `robot:=arx_acone` (can1 + can3) |
| 3) Lift2S split body | `split_body.launch.py`; on real hardware, choose lift mode hybrid / soft_p |
| 4) Lift2S full body | `full_body.launch.py`; same as above |

Run modes: real hardware / real hardware headless / simulation / simulation headless / Isaac / Isaac headless / visualization only.

### 5.2 Real-hardware launch

**Arm control mode: `full_control` only (MIT MIX)**

- At the motor layer this is always MIT impedance (`kp, kd, pos, vel, torque`); deployment scripts **no longer offer** an arm `position` menu.
- Single-arm real hardware: `xacro_can_interface:=can1` (left) or `can3` (right).
- Dual arm / Lift2S: left/right fixed to can1 / can3.

**Lift axis (Lift2S only)**

- `hybrid` (default): `sendLiftHybrid`, tracks pos+vel, with HI gravity/friction feedforward.
- `soft_p` / `position`: Soft-P `setHeight`, position tracking only.

```bash
./quick_start.sh
# Build → 2) Real-hardware packages
# Launch → 1) Single arm → Real hardware → choose left/right
# Launch → 3/4) Lift2S → Real hardware → lift mode (default hybrid)
```

<details>
<summary><strong>(Optional) Manual real-hardware launch</strong></summary>

```bash
source ~/lift2s-ws/install/setup.bash
# If RMW=zenoh, start router in another terminal first: ros2 run rmw_zenoh_cpp rmw_zenohd

# Single arm left / right
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real xacro_can_interface:=can1
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real xacro_can_interface:=can3

# Dual arm
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_acone hardware:=real

# Lift2S split / full body (lift default hybrid)
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s hardware:=real
ros2 launch ocs2_arm_controller full_body.launch.py robot:=arx_lift2s hardware:=real \
  xacro_lift_motor_mode:=hybrid
```

</details>

### 5.3 Joystick teleop

Joystick teleop runs in a **separate process** from the robot control stack (same pattern as [panthera-ht](https://github.com/fiveages-sim/open-deploy-ws/tree/panthera-ht)): start arm / Lift2S control first, then the joystick node.

**Dependencies**

- Control stack built (`arms_teleop` is in `BUILD_SIM_PACKAGES` / `BUILD_REAL_PACKAGES`; in release deployments it comes from **arms-full deb**)
- If the joy package is missing:

```bash
sudo apt install ros-jazzy-joy
```

**Usage**

1. **Terminal A**: start control (simulation or real hardware), e.g. `./quick_start.sh` → `2) Launch` → single-arm X5 / dual-arm ACone / Lift2S split or full body
2. **Terminal B**: `./quick_start.sh` → **`2) Launch`** → **`5) Joystick Teleop`** (enumerates devices when multiple controllers are connected)

Or manually:

```bash
source ~/lift2s-ws/install/setup.bash
# If RMW=zenoh, start router in another terminal first: ros2 run rmw_zenoh_cpp rmw_zenohd

ros2 launch arms_teleop joystick_teleop.launch.py
# Multiple controllers:
# ros2 launch arms_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1
```

The joystick node publishes `control_input`; **`arms_target_manager`** (started with `ocs2_arm_controller` launch) converts it to end-effector targets. **Switch to OCS2 first** (LB combos below) before pose tracking works.

**Common controls (SDL standard layout / Xbox-style gamepad)**

| Action | Description |
|--------|-------------|
| **Right stick press** | Enable / disable teleop (disabled by default) |
| **LB + A** | → HOME |
| **LB + B** | → HOLD |
| **LB + START** | → OCS2 (enter MPC, then drag end effector) |
| **LB + Y** | → MOVEJ |
| **Left / right stick** | Translate / rotate (when teleop enabled) |
| **A** | Switch left / right arm (dual arm / Lift2S) |
| **X** | Gripper open / close |
| **LT / RT** | Gripper open ratio |
| **Left stick press** | Toggle high / low speed |

For detailed joystick teleop docs (mapping files, unmapped device calibration, etc.), see the [arms_teleop README](https://github.com/fiveages-sim/arms_ros2_control/tree/main/command/arms_teleop).

### 5.4 Viser and VR teleop

Viser visualization and VR teleop live in the separate **[fa-py-libraries](https://github.com/fiveages-sim/fa-py-libraries)** repo (`ros2-viser`, `vr_pose_publisher`, `ros2_robot_interface`). It is **not bundled in the lift2s-ws release zip**. The Lift2S stack consumes VR input via **`VRInputHandler`** in `arms_target_manager` (subscribes to `/xr/*`).

#### One-time setup (fa-py-libraries)

Complete lift2s-ws §2 deployment and §4 build first (`~/lift2s-ws/install/setup.bash` must exist).

```bash
cd ~
git clone git@github.com:fiveages-sim/fa-py-libraries.git
cd ~/fa-py-libraries

# 1) Submodules + Python env + install
./init.sh all

# 2) Configure ROS 2 workspace (enter ~/lift2s-ws at the prompt)
#    Writes .fa-env.toml + activate hooks; run.sh will auto-source lift2s-ws
./init.sh ros2-workspace
```

Alternatively: `./init.sh` → menu option **10) Configure ROS 2 workspace** (same effect). Per-user overrides: `.fa-env.local.toml` (see [fa-py-libraries README](https://github.com/fiveages-sim/fa-py-libraries)).

#### Typical workflow

1. **Terminal A (lift2s-ws)**: launch robot control (sim or real)  
   `./quick_start.sh` → Launch → single arm / dual arm / Lift2S split or full body  
   The stack starts `arms_target_manager` (including VR input handling).
2. **Terminal B (fa-py-libraries)**: start Viser or VR (`run.sh` auto-sources lift2s-ws after the step above):

```bash
cd ~/fa-py-libraries
./run.sh viser          # 3D visualization (ros2-viser)
./run.sh vr             # VR teleop (Vuer / WebXR, publishes /xr/*)
```

3. **Before VR control**: switch FSM to **OCS2** (VR face buttons / combos, or **LB + START** on the joystick in terminal A). `VRInputHandler` enables tracking after it detects `/xr_target_node`.

**VR topics** (published by `vr_pose_publisher`, consumed by the Lift2S stack)

| Topic | Description |
|-------|-------------|
| `/xr/left_ee_pose`, `/xr/right_ee_pose` | Left / right controller poses |
| `/xr/head_pose` | Headset pose |
| `/xr/controller_state` | Face button / combo events |
| `/xr/thumbstick_axes` | Thumbstick axes |
| `/xr/trigger_values` | Trigger values |

On fixed-base Lift2S, VR chassis mode is usually unused; lift motion stays with the control stack / split `body_joint_controller` or full-body WBC.

**Environment**

- Both repos should use **RMW Zenoh** from §3; start `rmw_zenohd` before manual launches.
- For Viser and VR teleop usage, see [fa-py-libraries](https://github.com/fiveages-sim/fa-py-libraries).

## 6. Submodule notes

- **robot-descriptions-arx** — ARX model descriptions (`arx5` / `arx_acone` / `arx_lift2s`); **only source kept in release zip**
- **arx-ros2-control** — Real-hardware HI (`ArxX5Hardware` + `ArxLiftHardware`); **provided by arms-full deb in release deployments**; optional source for development
- **arms_ros2_control** — Generic ROS 2 manipulator control (OCS2 controllers, teleop, etc.); deb **full** includes `arx_ros2_control`
- **ocs2_ros2** — ROS 2 port of OCS2 (MPC); provided by deb in release zip
- **robot-descriptions-common** — Shared components and launch files; provided by deb in release zip
