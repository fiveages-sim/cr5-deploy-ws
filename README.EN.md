# Dobot CR5 Dual-Arm Robot ROS 2 Deployment Workspace

This repository deploys a ROS 2 workspace for the Dobot CR5 dual-arm robot, a complete control ecosystem based on the OCS2 MPC control framework.

### Prerequisites
- Git SSH keys configured with access to the relevant private repositories
- Git installed on the system (2.30+ recommended)

## 1. Repository initialization

### Clone the repository to ~/cr5-deploy-ws
```bash
# 1) Change to the home directory
cd ~

# 2) Clone into cr5-deploy-ws (directory name can be changed as needed)
git clone git@github.com:fiveages-sim/cr5-deploy-ws.git cr5-deploy-ws

# 3) Enter the repository directory
cd ~/cr5-deploy-ws
```

### Initialize and update submodules

#### Method 1: Automated script (recommended)
```bash
# Run the init script to switch all submodules to the latest commit on their configured branches
./init_repo.sh
```

<details>
<summary><strong>Method 2: Manual initialization</strong></summary>

```bash
# Initialize only these 4 submodules, non-recursively (does not initialize nested submodules of ocs2_ros2)
# Note: this checks out the specific commits recorded by the parent repository, not the latest tip of each branch
git submodule update --init

# To fetch the latest commits on each branch, check out the configured branch and pull manually
git submodule foreach 'git checkout $(git config -f ../.gitmodules --get submodule.$name.branch || echo main) && git pull'
```

</details>

### Updating submodules later
```bash
git submodule update --remote
```

### Directory layout (excerpt)
```
src/
  ├─ arms_ros2_control              # Submodule (branch: main)
  ├─ ocs2_ros2                      # Submodule (branch: ros2, with nested submodules)
  ├─ robot-descriptions-dobot       # Submodule (branch: main)
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
* When launching via `launch` files in `robot-descriptions-common`, a Zenoh router is started automatically

## 3. Build and simulation verification

### 3.1 Install dependencies
* Rosdep dependency installation
```bash
cd ~/cr5-deploy-ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.2 Build (recommended: `quick_start.sh`)

This workspace provides a one-shot script `quick_start.sh` for **scenario-based builds** and **mode-based launch** (single arm / dual arm / vla39, simulation / real hardware / real hardware + HTTP Bridge).

```bash
cd ~/cr5-deploy-ws
chmod +x ./quick_start.sh
./quick_start.sh
```

- In the menu, select **`1) Build`**
  - **`1) Build simulation packages`**: for simulation / development (no real-hardware drivers)
  - **`2) Build real-hardware packages`**: for connecting to real hardware (includes `dobot_ros2_control`, etc.)

<details>
<summary><strong>(Optional) Manual build commands</strong></summary>

```bash
cd ~/cr5-deploy-ws
# Simulation packages (corresponds to quick_start.sh -> Build -> Simulation Packages)
colcon build --packages-up-to \
  ocs2_arm_controller \
  cr5_description \
  cr5_dual_description \
  arms_teleop \
  adaptive_gripper_controller \
  vla_http_bridge \
  --symlink-install
```

```bash
cd ~/cr5-deploy-ws
# Real-hardware packages (corresponds to quick_start.sh -> Build -> Real Hardware Packages)
colcon build --packages-up-to \
  dobot_ros2_control \
  ocs2_arm_controller \
  cr5_description \
  cr5_dual_description \
  arms_teleop \
  adaptive_gripper_controller \
  vla_http_bridge \
  --symlink-install
```

</details>

### 3.3 Simulation verification

#### 3.3.1 Model visualization
* Single arm
  ```bash
  source ~/cr5-deploy-ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5
  ```
* With gripper
  ```bash
  source ~/cr5-deploy-ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5 type:=AG2F90-C-Soft
  ```
* Dual arm
  ```bash
  source ~/cr5-deploy-ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5_dual
  ```

#### 3.3.2 Launch simulation control
Prefer launching via `quick_start.sh` (it automatically `source`s `install/setup.bash`, provided a successful build has produced `install/`).

```bash
cd ~/cr5-deploy-ws
./quick_start.sh
```

- Select **`2) Launch`**
  - **`1) Single arm (CR5)`** or **`2) Dual arm (CR5 Dual)`**
  - Select **`1) Simulation (Simulation / mock_components)`**

<details>
<summary><strong>(Optional) Manual simulation control launch</strong></summary>

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5 type:=AG2F90-C-Soft
```

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual type:=AG2F90-C-Soft
```

</details>

#### 3.3.3 Launch real-hardware control
Prefer `quick_start.sh` as well (the script prompts you to verify the real-hardware IP configuration):

```bash
cd ~/cr5-deploy-ws
./quick_start.sh
```

- Select **`2) Launch`**
  - **`1) Single arm (CR5)`** / **`2) Dual arm (CR5 Dual)`** / **`3) Single arm vla39 (CR5 vla39)`**
  - Select a run mode:
    - **`2) Real Hardware`**: launches `ocs2_arm_controller demo.launch.py ... hardware:=real` directly
    - **`3) Real Hardware + HTTP Bridge`**: launches `cr5_description dobot_bringup_ros2.launch.py ...` (includes HTTP Bridge bringup)

**Real-hardware IP configuration tips**
- **Default bringup IP**: `src/robot-descriptions-dobot/cr5_description/launch/dobot_bringup_ros2.launch.py`
- **ros2_control `robot_ip`**: typically in `cr5_description/xacro/ros2_control/robot.xacro` (or a ros2_control xacro config under the same path)

<details>
<summary><strong>(Optional) Manual real-hardware control launch</strong></summary>

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5 hardware:=real type:=AG2F90-C-Soft
```

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=real type:=AG2F90-C-Soft
```

```bash
source ~/cr5-deploy-ws/install/setup.bash
ros2 launch cr5_description dobot_bringup_ros2.launch.py robot:=cr5_dual hardware:=real type:=AG2F90-C-Soft
```

</details>

## 4. Submodule notes

- **arms_ros2_control** — ROS 2 control implementation for manipulators
- **ocs2_ros2** — ROS 2 port of OCS2 (MPC control framework)
- **robot-descriptions-dobot** — Dobot manipulator description files
- **robot-descriptions-common** — Shared robot components (grippers, cameras, etc.)
