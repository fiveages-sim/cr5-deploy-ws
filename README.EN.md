# open-deploy-ws

A ROS 2 deployment workspace that integrates dual-arm manipulator control, robot description models, and the OCS2 MPC framework.

## Workspace layout

```
open-deploy-ws/
├── src/
│   ├── arms_ros2_control/     # Core arm control (controllers / commands / hardware interfaces / shared libs)
│   ├── robot-descriptions/    # Robot descriptions (common / manipulator / humanoid)
│   └── ocs2_ros2/             # OCS2 MPC framework
├── init_repo.sh               # One-shot initialization + build
└── README.md
```

## Quick start

```bash
git clone git@github.com:fiveages-sim/open-deploy-ws.git ros2_ws
cd ros2_ws
./init_repo.sh
```

**When running `init_repo.sh`, choose an initialization mode:**
- **public** (default; press Enter to select): initializes public submodules only — suitable for external users; no private repository access required.
- **private**: initializes all submodules (including private repositories) — requires internal repository access.

**The script then:**
1. Syncs and initializes top-level submodules
2. Initializes nested submodules according to the selected mode and `submodules_visibility.conf`
3. Checks out the configured branch for each submodule and pulls the latest commits
4. Runs `rosdep install` to install system dependencies
5. Runs `colcon build` to build all core packages

## Tested environment

- **ROS 2 Jazzy** (Ubuntu 24.04)

## License

Apache License 2.0. See [LICENSE](LICENSE) for details.
