# SUPER — Safety-assured High-speed Navigation for MAVs

## Project Overview
SUPER is the planning stack for autonomous high-speed flight of micro aerial vehicles
(quadrotors), published in *Science Robotics* '25 by the HKU MaRS Lab. It takes a goal
(waypoint file or RViz click), an occupancy map built from LiDAR/point-cloud input, and
the vehicle state, and produces a safe, dynamically feasible trajectory in real time.

The core idea is a **dual-trajectory** formulation: an aggressive *exploration* trajectory
through known-free + unknown space, plus a guaranteed-safe *backup* trajectory that keeps
the vehicle collision-free even when the exploration path enters unknown space. This is
what the "safety-assured" in the name refers to.

This is a ROS workspace. The actual source repo lives in `src/SUPER/`; `build/`,
`install/`, `devel/`, and `log/` are generated build artifacts (gitignored) — never edit them.

## Tech Stack
- **Language:** C++17 (header-heavy; `pragma once` / include-guard headers).
- **Middleware:** ROS — **both ROS1 (Noetic, Tier-1) and ROS2 (Humble)** from one codebase,
  selected at build time. See `.claude/docs/build_and_ros_versions.md`.
- **Build:** `catkin_make` (ROS1) or `colcon` (ROS2), each package `ament_cmake`/`catkin`.
- **Math/geometry:** Eigen 3, PCL (point clouds), custom MINCO trajectory optimization, L-BFGS.
- **Config:** YAML (`yaml-cpp`) loaded through a custom `YamlLoader`.
- **Vendored header libs** (in each package's `include/`, do not modify): `fmt`, `cereal`,
  `rapidjson`, `backward-cpp` (`-ldw`, crash stack traces), `matplotlibcpp`, `tinycolormap`.

## Key Directories (`src/SUPER/`)
| Path | Purpose |
|------|---------|
| `super_planner/` | **Core planner.** Front-end A* path search, CIRI safe-flight-corridor generation, MINCO exploration + backup trajectory optimization, yaw optimization, and the flight FSM. Builds the `super` library + `fsm_node`. |
| `rog_map/` | **ROG-Map**: efficient robocentric occupancy grid map (prob/inflation/ESDF/free-count layers + raycasting). Consumed by `super_planner`. |
| `mission_planner/` | Top-level mission node (`waypoint_mission`): reads a waypoint/goal file and drives the planner. Entry point for the benchmark launches. |
| `mars_uav_sim/` | Simulation: `perfect_drone_sim` (ideal drone dynamics + sensor/point-cloud sim), `marsim_render` (LiDAR rendering), `mars_quadrotor_msgs` (custom msgs). |
| `*/ros/` | Per-package ROS1/ROS2 CMakeLists + package.xml *templates* swapped in by the version-select script. |
| `*/config/`, `*/launch/` | YAML parameter sets and launch files (`.launch` for ROS1, `.launch.py` for ROS2). |
| `scripts/select_ros_version.sh` | Switches the whole workspace between ROS1 and ROS2. |

## Essential Commands
Run from the workspace root (`/home/ideas-ad/super_ws`). **First, pick a ROS version** —
this rewrites every package's `CMakeLists.txt` / `package.xml` from its `ros/` templates:

```bash
bash src/SUPER/scripts/select_ros_version.sh ROS1   # or ROS2
```

**ROS1 (Noetic):**
```bash
catkin_make -DBUILD_TYPE=Release
source devel/setup.bash
roslaunch mission_planner benchmark_high_speed.launch   # or benchmark_dense / click_demo
```

**ROS2 (Humble):**
```bash
colcon build --symlink-install            # add --event-handlers console_direct+ to debug
source install/local_setup.bash
ros2 launch mission_planner benchmark_high_speed.launch.py   # or *_dense / click_demo
```

For the click demo, press `G` in RViz, enable `2D Goal Pose`, and click a goal.

**Testing:** there is no unit-test suite. The `benchmark_*` and `click_demo` launches are the
de-facto integration tests — a change is "working" when these run end-to-end without the
planner erroring. Trajectory quality is evaluated offline from logs
(`python3 plotCmdLog.py`); see `.claude/docs/configuration_and_logging.md`.

## Code Style
Assume linters/formatters handle style. Match the conventions of the file you are editing
(headers carry the LGPL license banner; some comments are in Chinese — keep them).

## Additional Documentation
Read these when the task touches the topic:
- **`.claude/docs/architectural_patterns.md`** — the recurring design patterns: ROS1/ROS2
  abstraction layer, `RosInterface` dependency injection, the `::Ptr` convention, per-module
  YAML `Config` classes, the flight FSM, and the planner module composition. **Read this
  before adding a class, a ROS-facing feature, or a new tunable parameter.**
- **`.claude/docs/build_and_ros_versions.md`** — how the ROS1/ROS2 dual build works,
  the version-select script, CMake macros (`USE_ROS1`/`USE_ROS2`, `ROOT_DIR`, `FMT_HEADER_ONLY`,
  `ORIGIN_AT_CORNER`), and where each executable is defined. Read before build/CMake changes.
- **`.claude/docs/configuration_and_logging.md`** — the YAML config system and tuning,
  loading custom `.pcd` maps, and the replan/command logging + offline analysis tools.
