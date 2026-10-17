# super_ws Build Error Fixes

This document records every error encountered during `colcon build` in the `super_ws` ROS 2 (Humble) workspace and the exact fix applied. Paths are relative to the workspace root (`super_ws/`).

---

## Fix 1 — Duplicate package names blocked colcon from starting

**Error:**
```
ERROR:colcon:colcon build: Duplicate package names not supported:
- marsim_render: SUPER/mars_uav_sim/marsim_render AND src/SUPER/mars_uav_sim/marsim_render
- mission_planner: SUPER/mission_planner AND src/SUPER/mission_planner
- perfect_drone_sim: SUPER/mars_uav_sim/perfect_drone_sim AND src/SUPER/mars_uav_sim/perfect_drone_sim
- rog_map: SUPER/rog_map AND src/SUPER/rog_map
- super_planner: SUPER/super_planner AND src/SUPER/super_planner
```

**Root cause:** A copy of the `SUPER` package tree existed at `super_ws/SUPER/` (workspace root level) in addition to the correct location at `super_ws/src/SUPER/`. colcon scans the entire workspace directory tree and found both copies, producing duplicate package name errors.

**Fix:** Created an empty `COLCON_IGNORE` file inside the root-level `SUPER/` directory so colcon skips it during package discovery.

| File | Action |
|------|--------|
| `super_ws/SUPER/COLCON_IGNORE` | Created (empty file) |

---

## Fix 2 — ROS 1 `fast_lio` package (catkin) in a ROS 2 colcon workspace

**Error:**
```
CMake Error at CMakeLists.txt:45 (find_package):
  Could not find a package configuration file provided by "catkin"
Failed <<< fast_lio
```

**Root cause:** The `FAST_LIO/` directory at the workspace root is a ROS 1 package that uses `catkin` as its build system. colcon (ROS 2) does not provide catkin, so the build fails immediately.

**Fix:** Created an empty `COLCON_IGNORE` file inside `FAST_LIO/` so colcon skips it entirely.

| File | Action |
|------|--------|
| `super_ws/FAST_LIO/COLCON_IGNORE` | Created (empty file) |

---

## Fix 3 — `pcl_conversions/pcl_conversions.h` not found in `perfect_drone_sim`

**Error:**
```
fatal error: pcl_conversions/pcl_conversions.h: No such file or directory
 #include "pcl_conversions/pcl_conversions.h"
compilation terminated.
Failed <<< perfect_drone_sim
```

**Root cause:** `perfect_drone_sim/CMakeLists.txt` never called `find_package(pcl_conversions REQUIRED)`, so ament never added the `pcl_conversions` include path to the compiler flags. The header `pcl_conversions/pcl_conversions.h` is used in `include/perfect_drone_sim/ros2_perfect_drone_model.hpp` line 14.

**Fix:** Added `find_package(pcl_conversions REQUIRED)` and added `pcl_conversions` to the `ament_target_dependencies` list in `CMakeLists.txt`.

| File | Line | Change |
|------|------|--------|
| `super_ws/src/SUPER/mars_uav_sim/perfect_drone_sim/CMakeLists.txt` | 34 | Added `find_package(pcl_conversions REQUIRED)` |
| `super_ws/src/SUPER/mars_uav_sim/perfect_drone_sim/CMakeLists.txt` | 67 | Added `pcl_conversions` to the `set(dependencies ...)` block used in `ament_target_dependencies` |

---

## Fix 4 — `isnan` undeclared / ambiguous in `super_planner` under C++17 with Eigen

**Error:**
```
error: 'isnan' was not declared in this scope
note: suggested alternatives: 'std::isnan', 'Eigen::isnan', 'Eigen::numext::isnan'
```

**Root cause:** C++17 (enforced by the `-std=c++17` flag in the `super_planner` CMakeLists) no longer guarantees that `<cmath>` functions appear in the global namespace. Eigen further injects its own `isnan` overloads into multiple namespaces (`Eigen::`, `Eigen::numext::`, `Eigen::half_impl::`, `Eigen::bfloat16_impl::`). When plain `isnan(x)` is written, the compiler cannot pick an unambiguous candidate and refuses to compile.

**Fix:** Added `#include <cmath>` to the two `.cpp` files that did not have it, and replaced every bare `isnan(` call with the fully-qualified `std::isnan(` across all affected files.

### 4a — `fsm.cpp`

| File | Line | Change |
|------|------|--------|
| `super_ws/src/SUPER/super_planner/src/super_core/fsm.cpp` | 26 | Added `#include <cmath>` |
| `super_ws/src/SUPER/super_planner/src/super_core/fsm.cpp` | 206 | `isnan(q.w())` → `std::isnan(q.w())` (and same for `.x()`, `.y()`, `.z()` on the same line) |

### 4b — `super_planner.cpp`

| File | Line | Change |
|------|------|--------|
| `super_ws/src/SUPER/super_planner/src/super_core/super_planner.cpp` | 26 | Added `#include <cmath>` |
| `super_ws/src/SUPER/super_planner/src/super_core/super_planner.cpp` | 351 | `isnan(yaw)` → `std::isnan(yaw)` |
| `super_ws/src/SUPER/super_planner/src/super_core/super_planner.cpp` | 357 | `isnan(yaw_dot)` → `std::isnan(yaw_dot)` |
| `super_ws/src/SUPER/super_planner/src/super_core/super_planner.cpp` | 795 | `!isnan(gi_.goal_yaw)` → `!std::isnan(gi_.goal_yaw)` |
| `super_ws/src/SUPER/super_planner/src/super_core/super_planner.cpp` | 1048 | `!isnan(gi_.goal_yaw)` → `!std::isnan(gi_.goal_yaw)` |

### 4c — `ros2_adapter.hpp`

| File | Line | Change |
|------|------|--------|
| `super_ws/src/SUPER/super_planner/include/ros_interface/ros2/ros2_adapter.hpp` | 179 | `isnan(t_sum)` → `std::isnan(t_sum)` |
| `super_ws/src/SUPER/super_planner/include/ros_interface/ros2/ros2_adapter.hpp` | 316 | `isnan(planes.sum())` → `std::isnan(planes.sum())` |
| `super_ws/src/SUPER/super_planner/include/ros_interface/ros2/ros2_adapter.hpp` | 345 | `isnan(mesh.sum())` → `std::isnan(mesh.sum())` |
| `super_ws/src/SUPER/super_planner/include/ros_interface/ros2/ros2_adapter.hpp` | 655 | `isnan(pt.x())`, `isnan(pt.y())`, `isnan(pt.z())` → `std::isnan(...)` |

### 4d — `ros1_adapter.hpp`

| File | Line | Change |
|------|------|--------|
| `super_ws/src/SUPER/super_planner/include/ros_interface/ros1/ros1_adapter.hpp` | 162 | `isnan(t_sum)` → `std::isnan(t_sum)` |
| `super_ws/src/SUPER/super_planner/include/ros_interface/ros1/ros1_adapter.hpp` | 299 | `isnan(planes.sum())` → `std::isnan(planes.sum())` |
| `super_ws/src/SUPER/super_planner/include/ros_interface/ros1/ros1_adapter.hpp` | 328 | `isnan(mesh.sum())` → `std::isnan(mesh.sum())` |
| `super_ws/src/SUPER/super_planner/include/ros_interface/ros1/ros1_adapter.hpp` | 637 | `isnan(pt.x())`, `isnan(pt.y())`, `isnan(pt.z())` → `std::isnan(...)` |

### 4e — `polytope.cpp`

| File | Line | Change |
|------|------|--------|
| `super_ws/src/SUPER/super_planner/src/utils/polytope.cpp` | 122 | `isnan(planes.sum())` → `std::isnan(planes.sum())` |

---

## Final build result

After all four fixes, `colcon build --symlink-install` completed with:

```
Summary: 6 packages finished
  mars_quadrotor_msgs  ✓
  marsim_render        ✓
  mission_planner      ✓
  rog_map              ✓
  perfect_drone_sim    ✓
  super_planner        ✓
```

No errors. Only pre-existing sign-compare and -Wreorder warnings from upstream code remain (not errors, do not block the build).
