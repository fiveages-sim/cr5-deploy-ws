# ARX ACone Manipulator ROS 2 Deployment Workspace

This repository deploys a ROS 2 workspace for the ARX ACone manipulator, a complete control ecosystem based on the OCS2 MPC control framework.

### Prerequisites
- Git SSH keys configured with access to the relevant private repositories
- Git installed on the system (2.30+ recommended)

## 1. Repository initialization

### Clone the repository to ~/open-deploy-ws
```bash
  # 1) Change to the home directory
  cd ~

  # 2) Clone into open-deploy-ws (directory name can be changed as needed)
  git clone git@github.com:fiveages-sim/open-deploy-ws.git open-deploy-ws

  # 3) Enter the repository directory
  cd ~/open-deploy-ws
```

### Initialize and update submodules

```bash
  # Run the init script to switch all submodules to the latest commit on their configured branches
  cd ~/open-deploy-ws
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
  ├─ robot-descriptions-arx         # Submodule (branch: main)
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
cd ~/open-deploy-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.2 Build (recommended: `quick_start.sh`)

This workspace provides a one-shot script `quick_start.sh` for **scenario-based builds** and **mode-based launch** (dual-arm ACone, simulation / real hardware).

```bash
cd ~/open-deploy-ws
chmod +x ./quick_start.sh
./quick_start.sh
```

- In the menu, select **`1) Build`**
  - **`1) Build simulation packages`**: for simulation / development (no real-hardware drivers)
  - **`2) Build real-hardware packages`**: for connecting to real hardware (includes `arx_ros2_control`, etc.)

<details>
<summary><strong>(Optional) Manual build commands</strong></summary>

```bash
cd ~/open-deploy-ws
# Simulation packages (corresponds to quick_start.sh -> Build -> Simulation Packages)
colcon build --packages-up-to \
  ocs2_arm_controller \
  arx_lift2s_description \
  arx5_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

```bash
cd ~/open-deploy-ws
# Real-hardware packages (corresponds to quick_start.sh -> Build -> Real Hardware Packages)
colcon build --packages-up-to \
  arx_ros2_control \
  ocs2_arm_controller \
  arx_lift2s_description \
  arx5_description \
  arms_teleop \
  adaptive_gripper_controller \
  basic_joint_controller \
  --symlink-install
```

</details>

### 3.3 Simulation verification

#### 3.3.1 Model visualization
```bash
source ~/open-deploy-ws/install/setup.bash
ros2 launch robot_common_launch manipulator.launch.py robot:=arx_lift2s type:=acone_x5
```

#### 3.3.2 Launch simulation control
Prefer launching via `quick_start.sh` (it automatically `source`s `install/setup.bash`, provided a successful build has produced `install/`).

```bash
cd ~/open-deploy-ws
./quick_start.sh
```

- Select **`2) Launch`**
  - Select **`1) Dual arm (ACone)`**
  - Select **`1) Simulation (Simulation / mock_components)`**

<details>
<summary><strong>(Optional) Manual simulation control launch</strong></summary>

```bash
source ~/open-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_lift2s type:=acone_x5
```

</details>

#### 3.3.3 Launch real-hardware control
ARX ACone connects over the CAN bus; no network IP configuration is required.

```bash
cd ~/open-deploy-ws
./quick_start.sh
```

- Select **`2) Launch`**
  - Select **`1) Dual arm (ACone)`**
  - Select **`2) Real Hardware`**

<details>
<summary><strong>(Optional) Manual real-hardware control launch</strong></summary>

```bash
source ~/open-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_lift2s type:=acone_x5 hardware:=real
```

</details>

## 4. Submodule notes

- **arms_ros2_control** — Generic ROS 2 control implementation for manipulators
- **arx-ros2-control** — ARX manipulator hardware driver (CAN bus)
- **ocs2_ros2** — ROS 2 port of OCS2 (MPC control framework)
- **robot-descriptions-arx** — ARX manipulator description files
- **robot-descriptions-common** — Shared robot components (grippers, cameras, etc.)
