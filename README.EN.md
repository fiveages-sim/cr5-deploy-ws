# ARX Lift2S ROS 2 Deployment Workspace

![ARX Lift2S](.images/arx_lift2s.jpg)

This repository deploys a ROS 2 workspace for **ARX Lift2S**, based on the OCS2 MPC control framework.

ARX (Fangzhou) model descriptions live in the submodule [`src/robot-descriptions-arx`](src/robot-descriptions-arx) (X5/R5, ACone, Lift, Lift2S, X7S, …). `quick_start` can also launch those models for co-debug; this workspace’s product focus is Lift2S.

### Prerequisites
- Git SSH keys configured with access to the relevant private repositories
- Git installed on the system (2.30+ recommended)
- ROS 2 Jazzy (Ubuntu 24.04)

## 1. Repository initialization
### Clone the repository to ~/lift2s-ws
```bash
  # 1) Change to the home directory
  cd ~
  
  # 2) Clone into lift2s-ws (directory name can be changed as needed)
  git clone git@github.com:fiveages-sim/open-deploy-ws.git lift2s-ws
  
  # 3) Enter the repository directory
  cd ~/lift2s-ws
```

For field use with a maintainer release zip: unzip, run `./release.sh --install`, then `./quick_start.sh` (description source stays in the workspace; controllers / HI come from deb).

### Initialize and update submodules

```bash
  # Run the init script (interactive source / deb menu)
  cd ~/lift2s-ws
  ./init_repo.sh
```

### Updating submodules later
```bash
git submodule update --remote
```

### Directory layout (excerpt)
```
src/
  ├─ arms_ros2_control              # Submodule (branch: main)
  ├─ arx-ros2-control               # Submodule (branch: main)
  ├─ ocs2_ros2                      # Submodule (branch: ros2, with nested submodules)
  ├─ robot-descriptions-arx         # Submodule (branch: main; full ARX description tree)
  └─ robot-descriptions-common      # Submodule (branch: main)
```

### Troubleshooting
- SSH permissions: if clone/update fails, confirm that your local SSH key is added to your GitHub account and that `ssh -T git@github.com` succeeds.
- Network issues: retry or use a proxy; switch to HTTPS cloning if necessary.

## 2. Install RMW Zenoh C++

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

## 3. Build and simulation verification
### 3.1 Install dependencies
* Rosdep dependency installation
```bash
cd ~/lift2s-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.2 Build (recommended: `quick_start.sh`)

This workspace provides a one-shot script `quick_start.sh` for **scenario-based builds** and **mode-based launch** (Lift2S split / full body, simulation / real hardware).

```bash
cd ~/lift2s-ws
chmod +x ./quick_start.sh
./quick_start.sh
```

- In the menu, select **`1) Build`**
  - **`1) Build simulation packages`**: for simulation / development (no real-hardware drivers)
  - **`2) Build real-hardware packages`**: for connecting to real hardware (includes `arx_ros2_control`, etc.)

<details>
<summary>(Optional) Manual build commands</summary>

```bash
cd ~/lift2s-ws
# Simulation packages (corresponds to quick_start.sh -> Build -> Simulation Packages)
colcon build --packages-up-to \
  ocs2_arm_controller \
  ocs2_wbc_controller \
  topic_based_ros2_control \
  arx_acone_description \
  arx_lift2s_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

```bash
cd ~/lift2s-ws
# Real-hardware packages (corresponds to quick_start.sh -> Build -> Real Hardware Packages)
colcon build --packages-up-to \
  arx_ros2_control \
  ocs2_arm_controller \
  ocs2_wbc_controller \
  arx_acone_description \
  arx_lift2s_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

</details>

### 3.3 Simulation verification
#### 3.3.1 Model visualization
```bash
source ~/lift2s-ws/install/setup.bash
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_lift2s
```

#### 3.3.2 Launch simulation control
Prefer launching via `quick_start.sh` (it automatically `source`s `install/setup.bash`, provided a successful build has produced `install/`).

```bash
cd ~/lift2s-ws
./quick_start.sh
```

- Select **`2) Launch`**
  - Select **Lift2S**
  - Select **split body** or **full body**
  - Select **`1) Simulation (Simulation / mock_components)`**

X5 / R5 / ACone / Lift / X7S in the menu come from `robot-descriptions-arx` for co-debug.

<details>
<summary>(Optional) Manual simulation control launch</summary>

```bash
source ~/lift2s-ws/install/setup.bash
# Split body
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s
# Full body
ros2 launch ocs2_arm_controller full_body.launch.py robot:=arx_lift2s
```

</details>

#### 3.3.3 Launch real-hardware control
ARX Lift2S connects over the CAN bus (left/right arms: can1 / can3); no network IP configuration is required. Lift axis defaults to `hybrid` (can be changed to `soft_p` in `quick_start`).

```bash
cd ~/lift2s-ws
./quick_start.sh
```

- Select **`2) Launch`**
  - Select **Lift2S**
  - Select **split body** or **full body**
  - Select **`2) Real Hardware`**

<details>
<summary>(Optional) Manual real-hardware control launch</summary>

```bash
source ~/lift2s-ws/install/setup.bash
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s hardware:=real
ros2 launch ocs2_arm_controller full_body.launch.py robot:=arx_lift2s hardware:=real
```

</details>

## 4. Submodule notes

- **arms_ros2_control** - Generic ROS 2 control for manipulators
- **arx-ros2-control** - ARX hardware driver (CAN: arms + Lift2S lift/chassis)
- **ocs2_ros2** - ROS 2 port of OCS2 (MPC control framework)
- **robot-descriptions-arx** - Full ARX product descriptions (this workspace primarily uses Lift2S)
- **robot-descriptions-common** - Shared robot components (grippers, cameras, launch, etc.)
