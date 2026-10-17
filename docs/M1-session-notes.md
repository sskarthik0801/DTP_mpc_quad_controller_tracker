# M1 session notes — what was done and decided

This captures the decisions and the debugging journey behind M1, so the reasoning survives
beyond the original Claude Code chat. Current source of truth alongside `../README.md` and
`../CLAUDE.md`.

## Goal
Replace SUPER's simulated map source (`perfect_drone_sim`, which fakes `/cloud_registered` +
`/lidar_slam/odom`) with **real LiDAR-inertial odometry** from the MID360, so ROG-Map builds
occupancy from the live sensor. No flight in M1.

## Decisions
- **FAST-LIO2 now, FAST-LIVO2 later.** The doc referenced FAST-LIO2; FAST-LIVO2 was considered but
  rejected for M1: it needs a hardware-synced/calibrated camera, only has a community ROS2 port, and
  adds visual compute on the Orin Nano (the #1 risk). FAST-LIO2 uses the MID360's internal IMU, has
  native ROS2 support, is lighter. The swap point later is just the two output topics — nothing
  downstream changes.
- **Pure state estimation, not the old hybrid.** A prior session ran a *hybrid* (real LiDAR cloud +
  simulated drone odom). M1's `m1_super_lidar.launch.py` uses FAST-LIO for **both** cloud and odom,
  with **no** `perfect_drone_sim` (it would clobber the same topics).

## What had to be redone / fixed
1. **FAST-LIO was missing.** A fresh `git clone` of SUPER (which wiped earlier in-tree work) left no
   `FAST_LIO_ROS2`. Re-cloned `hku-mars/FAST_LIO` (ROS2 branch) + ikd-Tree submodule.
2. **C++17 build fix.** Build failed with `invalid field 'callback_variant_'` — root cause was
   `-std=c++14` in FAST-LIO's `CMakeLists.txt` vs Humble's C++17-only `rclcpp` 16.x. Bumped to C++17.
3. **Dense cloud** tuning in `mid360.yaml` (`point_filter_num:1`, `filter_size_surf:0.2`,
   `dense_publish_en:true`).
4. **`virtual_ground_height: -1.0`** in SUPER `click_smooth_ros2.yaml` (clone had reverted it to -0.1).
5. **Persistent network.** The MID360 host IP kept vanishing — NetworkManager flushes a manually-added
   IP. Created a persistent NM profile (`livox`, `192.168.1.50/24`, `never-default`, autoconnect).
6. **`super_bringup`** package created to hold the M1 launch (and future M2/M3 bridges).

## Gate results (verified live)
- **A — driver:** `/livox/imu` 199.96 Hz, `/livox/lidar` 10.0 Hz, CustomMsg.
- **B — odometry:** `/cloud_registered` 10 Hz (world frame) + `/lidar_slam/odom` 9.9 Hz; pose
  ≈ origin, finite, non-divergent.
- **C — map ingest:** `rog_map/occ` ≈ 6073 voxels, `rog_map/inf_occ` ≈ 12076 voxels from the live
  cloud; no odom-timeout / below-ground warnings. FSM correctly idles in `INIT` awaiting a goal.

## Known-good pinned commits
SUPER `2ad3419` · FAST_LIO(ROS2) `a4743b0` · livox_ros_driver2 `6b9356c` · Livox-SDK2 `6a94015`.

## Next (M2/M3) — pointers
> ⛔ **The paragraph that was here was WRONG and has been removed.** It said to target
> **ArduCopter + MAVROS**. The FC actually runs **PX4 v1.17.0 + uXRCE-DDS** (confirmed live over
> MAVLink: `MAV_AUTOPILOT_PX4`, firmware `v1.17.0`). External-vision odom goes to
> `/fmu/in/vehicle_visual_odometry` via `px4_odom_bridge`, **not** `/mavros/vision_pose/pose`.
> `plantofly.md` and `mavros-setup-notes.md` are superseded. See `ImplementationDoc.md`.

## Final verification (2026-06-20)
- **World-stability confirmed in RViz** (from a remote laptop): walls stay locked in the `world`
  frame and the odom frame tracks correctly while carrying the rig → FAST-LIO odometry is accurate.
  This closes the dynamic half of Gate B. **M1 is 5/5 complete and verified.**
- **Remote laptop viewer added** (`scripts/laptop_viewer.sh` + `rviz/m1_viewer.rviz`), discovering
  the Jetson's topics over DDS (same Wi-Fi, `ROS_DOMAIN_ID=0`). Two laptop gotchas fixed in the
  script: (1) snap leaks `/snap/.../lib` into `LD_LIBRARY_PATH` → apt `rviz2` crashes with
  `undefined symbol __libc_pthread_init`; strip `/snap/` paths. (2) `set -u` aborts while sourcing
  ROS `setup.bash` on a cold shell (`AMENT_TRACE_SETUP_FILES` unbound) → use `set -eo pipefail`.
- **MID360 point-stream stall observed:** after ~30–40 min the LiDAR point stream stopped while the
  IMU kept running at 200 Hz; FAST-LIO diverged (pose blew up to ~−467 km, ROG-Map reset on loop).
  Fix = restart the Livox driver (re-issues start/work-mode), then restart FAST-LIO. In the README
  troubleshooting; an auto-restart watchdog is an M5 item.

---

## Later findings (2026-07-29) — M1 is NOT as finished as the gates suggested

The gates above were real, but every one of them was a **short** run. Long-duration behaviour was
never tested, and it is where M1 breaks.

### ⛔ FAST-LIO grows RAM without bound and saturates a CPU core

| Observation | Value |
|---|---|
| `fastlio_mapping` RSS, original config | **2.97 GB after 18 min**, growing ~1.7 MB/s, linear, no plateau |
| CPU | **100%** of one core (`point_filter_num: 1`) |
| Consequence | Jetson (7.4 GB) drove 2.6 GB into swap and thrashed |
| `/lidar_slam/odom` | fell **10 Hz → 0.55 Hz** |
| `/livox/lidar` meanwhile | healthy **9.9 Hz** — the LiDAR and driver were fine |

**Root cause:** the ikd-Tree map is only trimmed by `lasermap_fov_segment()`, which deletes regions
as the vehicle approaches the `cube_side_length` boundary. **A stationary bench rig never triggers
trimming**, so the map grows forever.

**Ruled out — do not re-investigate:**
- `pcd_save_en` — the `pcl_wait_save` accumulation block in `laserMapping.cpp` (~L520-543) sits
  inside a `/* */` comment and never executes. Disabling it changed nothing.
- Odometry drift smearing the map — position stayed within ~10 cm over 6 min while stationary.
- Duplicate publishers — there *was* a window with two Livox drivers (orphans from repeated
  restarts, `/livox/lidar` Publisher count: 2) which inflated one sample, but growth continued at
  the same rate after cleanup.

**Mitigation applied** (`config/mid360.yaml`): `point_filter_num: 1 → 3` (CPU 100% → 64%, growth
1.7 → ~0.55 MB/s), `cube_side_length: 1000 → 200`, `det_range: 100 → 70`, `pcd_save_en: false`.

> Note this supersedes the earlier "dense cloud tuning (`point_filter_num:1`)" decision in the
> fixes list above — `point_filter_num: 1` is what pegged the CPU.

### ✅ RESOLVED (2026-08-26): the residual ~0.55 MB/s was `/Laser_map`, never `pcl_wait_pub` cleared

The four params above didn't touch the real leak. Read `laserMapping.cpp` directly on this
machine: `map_publish_callback()` is a **1-second wall timer that runs for the process lifetime**
and calls `publish_map()`, which does `*pcl_wait_pub += *laserCloudWorld;` — appending to a
file-scope point cloud that **is never cleared anywhere in the file**. It's gated by
`publish.map_en`, publishing to `/Laser_map`. `ros2 topic info /Laser_map` showed
**`Subscription count: 0`** — SUPER's ROG-Map only ever ingests `/cloud_registered` +
`/lidar_slam/odom`. The topic, and everything backing it, was pure waste.

**Fix:** `publish.map_en: true → false`.

**Verified live, same 18-min window as the original crisis measurement:**

| | Before (`map_en:true`) | After (`map_en:false`) |
|---|---|---|
| RSS growth, first 60s | +37 MB (0.62 MB/s) | +8.5 MB, decaying fast |
| RSS, full 18 min | (extrapolates to ~+600 MB on the 2.97 GB baseline) | **157,704 KB → 154,168 KB — net −3.5 MB, i.e. flat** |
| `/lidar_slam/odom` | degraded over time | held **10 Hz** for the full 18 min |

RSS dipped *below* its starting value at several points during the run — no more unbounded
growth on a stationary rig. **Still open:** re-run this endurance test while moving the rig, since
`cube_side_length`'s ikd-Tree trimming only engages with motion and hasn't been isolated from
this fix yet.

### Config files silently revert on rebuild

Both repo configs were found reverted to upstream defaults on 2026-07-29:
- `MID360_config.json` had **stock IPs** (host `192.168.1.5`, lidar `.12`) instead of `.50`/`.154`,
  so the driver could never have connected.
- `mid360.yaml` was in **ROS 1 format**, so FAST-LIO **crashed at startup** with
  `Cannot have a value before ros__parameters`. The visible symptom was `fsm_node` logging
  `No point cloud input` / `No odom` forever — which looks like a sensor fault and isn't.

Re-run `scripts/setup.sh` after any `colcon build` of `fast_lio` or rebuild of the Livox driver.
Configs must land in **both** `src/` and `install/`.

### The wired interface name is not stable

`network_setup.sh` hardcoded `enP8p1s0`; this board (Orin Nano **Engineering Reference** Devkit)
uses **`eno1`**. The script now auto-detects (prefers an iface with a live carrier, `IFACE=` to
override).
