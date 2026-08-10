# ARX Lift2S / ACone Manipulator ROS 2 Deployment Workspace

![ARX Lift2S](.images/arx_lift2s.jpg)

This repository deploys a ROS 2 workspace for ARX Lift2S (with lift axis) and ACone / X5 manipulators, based on the OCS2 MPC control framework.

## Contents

1. [Prerequisites](#1-prerequisites)
2. [Deployment](#2-deployment)
   - [2.1 Clone and submodules](#21-clone-and-submodules)
   - [2.2 Updating submodules later](#22-updating-submodules-later)
   - [2.3 Directory layout (excerpt)](#23-directory-layout-excerpt)
   - [2.4 Troubleshooting](#24-troubleshooting)
   - [2.5 Release package (`release.sh`)](#25-release-package-releasesh)
3. [Environment setup (RMW Zenoh)](#3-environment-setup-rmw-zenoh)
4. [Build](#4-build)
   - [4.1 Install dependencies](#41-install-dependencies)
   - [4.2 Build (recommended: `quick_start.sh`)](#42-build-recommended-quick_startsh)
5. [Launch (use quick_start)](#5-launch-use-quick_start)
   - [5.1 Simulation / visualization](#51-simulation--visualization)
   - [5.2 Real-hardware notes](#52-real-hardware-notes)
6. [Submodule notes](#6-submodule-notes)

## 1. Prerequisites

- Git SSH keys configured with access to the relevant private repositories
- Git installed on the system (2.30+ recommended)
- ROS 2 Jazzy (Ubuntu 24.04)

## 2. Deployment

### 2.1 Clone and submodules

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

The script: syncs top-level submodules → checks out the latest commit on each configured branch → initializes nested `ocs2_ros2` submodules (if any) → checks `arx-ros2-control/external` dependencies.

### 2.2 Updating submodules later

```bash
git submodule update --remote
```

### 2.3 Directory layout (excerpt)

```
src/
  ├─ arms_ros2_control              # Controllers / teleop / shared control stack
  ├─ arx-ros2-control               # Real-hardware HI (package arx_ros2_control: arm + lift)
  ├─ ocs2_ros2                      # OCS2 ROS 2 (with nested submodules)
  ├─ robot-descriptions-arx         # arx5 / arx_acone / arx_lift2s descriptions
  └─ robot-descriptions-common      # Shared launch / grippers / etc.
```

### 2.4 Troubleshooting

- SSH permissions: confirm your local SSH key is added to GitHub and that `ssh -T git@github.com` succeeds.
- Network issues: retry or use a proxy; switch to HTTPS cloning if necessary.

### 2.5 Release package (`release.sh`)

The mechanism aligns with [panthera-ht](https://github.com/fiveages-sim/open-deploy-ws/tree/panthera-ht): the core stack is delivered as debs; descriptions and real-hardware HI remain as source.

**On site:**

```bash
# After extracting dist/lift2s_ws_*.zip
./release.sh --install    # install from .deb_cache/ (use --download first if packages are missing)
./quick_start.sh          # build descriptions + arx_ros2_control, then launch
```

**Maintainer packaging (temporary directory; does not modify the current workspace):**

```bash
./release.sh --package
./release.sh --package-no-git --arch amd64
./release.sh --package-no-git --arch arm64
```

Source retained: `src/robot-descriptions-arx`, `src/arx-ros2-control`; other submodules are placeholders, provided by the ocs2 / common / arms debs in `deb_versions.conf`. For more detail, see [`arx_lift2s_description/README.md`](src/robot-descriptions-arx/arx_lift2s_description/README.md) §5.

## 3. Environment setup (RMW Zenoh)

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

```bash
cd ~/lift2s-ws
chmod +x ./init_repo.sh ./quick_start.sh
./init_repo.sh
./quick_start.sh
```

- **`1) Build`**
  - **`1) Simulation packages`**: no real-hardware drivers required
  - **`2) Real-hardware packages`**: builds `arx_ros2_control` plus descriptions and the control stack (shared by single arm / dual arm / Lift2S)

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

### 5.2 Real-hardware notes

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

## 6. Submodule notes

- **arms_ros2_control** — Generic ROS 2 manipulator control (OCS2 controllers, teleop, etc.)
- **arx-ros2-control** — Lift2S / ACone / X5 real-hardware interfaces (`ArxX5Hardware` + `ArxLiftHardware`)
- **ocs2_ros2** — ROS 2 port of OCS2 (MPC)
- **robot-descriptions-arx** — ARX model descriptions (`arx5` / `arx_acone` / `arx_lift2s`)
- **robot-descriptions-common** — Shared components and launch files
