---
paths:
  - "config/**"
  - "patches/**"
  - "scripts/run.sh"
  - "scripts/network_setup.sh"
  - "scripts/jetson_swap_setup.sh"
  - "scripts/jetson_clocks_setup.sh"
  - "scripts/laptop_dds_profile.sh"
  - "super_bringup/launch/m1_super_lidar.launch.py"
  - "super_bringup/config/**"
  - "scripts/px4/estimator_preflight.py"
  - "drone-dtp/scripts/m53_soak.py"
  - "drone-dtp/scripts/odom_lag_probe.py"
  - "docs/fact-fastlio-*.md"
  - "docs/fact-fastdds-blocking-send.md"
  - "docs/M1-session-notes.md"
  - "docs/livoxconnection.md"
---

# FAST-LIO, the MID360 link, and compute (M1)

Scope: the LiDAR → `livox_ros_driver2` → FAST-LIO chain, its latency, memory, frames and rate; the estimator-divergence guard (G0.2); CPU pinning, swap and the DDS send path. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

## M1 status

- **M1 (sensing & state estimation): DONE. RAM/CPU regression RESOLVED for the stationary case.**
  MID360 → `livox_ros_driver2` → FAST-LIO2 → SUPER/ROG-Map works. The prior open regression — RSS
  growing to 2.97 GB in 18 min, swap-thrashing the Jetson and dropping `/lidar_slam/odom` from
  10 Hz to 0.55 Hz — is fixed: `point_filter_num: 3` (CPU 100%→64%) plus `publish.map_en: false`
  (the actual leak — see below) held RSS **flat** over an 18-min re-run with odom steady at 10 Hz.
  **Re-verified from scratch on the Orin NX, 2026-08-30**: 8 packages build clean,
  `/lidar_slam/odom` steady at **9.98 Hz**, `/cloud_registered` 10.01 Hz, FAST-LIO RSS flat over
  5 min (157.8 → 156.9 MB) at 39.6% CPU.
  **Re-verified again 2026-09-01 on the NX**: 9 packages build clean, `/lidar_slam/odom` 9.98 Hz,
  `/cloud_registered` 9.91 Hz, `/livox/imu` 199.96 Hz, ROG-Map building (6714 occ / 7244 inf
  voxels), FAST-LIO RSS flat 150.7 → 151.8 MB over 9.5 min, `fsm_node` flat at 2033 MB, swap 0 B.
  **Odometry latency fixed 2026-09-01** — the ROS2 port was publishing poses that were correct but
  **~1.4 s stale**, silently (see the FAST-LIO latency bullet below). Now **49-57 ms, 0/10 runs
  lagged**. This was a hard blocker for M2/M3: 1.4 s stale vision into EKF2 or a control loop.
  **Still open: endurance-test on a *moving* rig** — only stationary has ever been verified, on
  either board, and that now matters more than before: the latency fix drops the cloud queue to
  depth 2, so under load stale scans are **dropped** rather than queued, and a stationary bench
  never produces the load spike that would exercise that. Re-run the frame-matched latency check
  (method in `docs/fact-fastlio-latency.md`) on a moving rig first.
  **Also settled 2026-09-01 (don't re-derive):** 10 Hz is the right LiDAR rate and should stay.
  100 Hz is impossible — the MID360 emits a fixed ~200k pts/s, so 100 Hz gives only ~1920
  pts/frame and the ESIKF diverges by tens of metres on a *stationary* rig (measured, with
  `point_filter_num` 3 **and** 1). 50 Hz works but only with `point_filter_num: 1`, undoing the
  CPU fix. 20 Hz showed no benefit in the full stack. `livox_ros2_driver` (first-gen, Livox-SDK v1)
  does **not** support the MID360 — `livox_ros_driver2` + Livox-SDK2 is correct and already in use.

## FAST-LIO facts that cost time

- **FAST-LIO needs C++17.** The `hku-mars/FAST_LIO` ROS2 branch hard-codes `-std=c++14`, which fails
  against Humble's C++17-only `rclcpp` 16.x (`error: invalid field 'callback_variant_'`). `setup.sh`
  patches it. See `docs/fact-fastlio-cpp17.md`.
- **NetworkManager flushes a manual LiDAR IP** within ~1 min → driver `bind failed`. Use a persistent
  NM profile: `scripts/network_setup.sh`. **The wired iface name is not stable across boards** — it
  was `enP8p1s0` on one unit and is `eno1` on the Orin Nano Engineering Reference devkit, so the
  script now auto-detects it (override with `IFACE=`).

- **The TF frames are renamed from upstream: `camera_init`→`lidar_init`, `body`→`lidar_imu`**
  (2026-09-02, `patches/fastlio_frames.py`, applied by `setup.sh`). Upstream's **`body` is not the
  drone body** — it is the MID360's *internal IMU*, which is what FAST-LIO's ESIKF estimates. It is
  4.41 cm from the LiDAR's optical origin (`extrinsic_T`) and 7 cm from the FC, which is PX4's
  actual body origin (`EKF2_IMU_POS_*=0`). Reading "body" as the vehicle body is precisely how you
  get a wrong `EKF2_EV_POS_*`. `camera_init` had no camera behind it. The live tree is
  **`world → lidar_init → lidar_imu`**. Nothing downstream reads these names — ROG-Map ignores
  `frame_id`/TF entirely, `rviz/m1_viewer.rviz` uses only `Fixed Frame: world`, and
  `px4_odom_bridge` never inspects them. **When reading upstream FAST-LIO issues/docs, translate.**
- **`xfer_format = 1`** (Livox CustomMsg) is required so FAST-LIO gets per-point timestamps.

- **FAST-LIO's RAM leak is fixed — the real cause was `/Laser_map`, not the ikd-Tree.** The
  ikd-Tree map is only trimmed near the `cube_side_length` boundary (a **stationary rig never
  trims**), but `point_filter_num:3`/`cube_side_length:200` only got CPU down, not RAM. The actual
  leak: `map_publish_callback()` in `laserMapping.cpp` runs every 1s for the process lifetime and
  appends to `pcl_wait_pub`, **never cleared**, behind `/Laser_map` — a topic with **zero
  subscribers** in this pipeline. Fixed by `publish.map_en: false` in `config/mid360.yaml`.
  Verified 2026-08-26: flat RSS over an 18-min re-run (vs. +2.97 GB before). Ruled out already
  (don't redo): `pcd_save_en` (dead code, inside a `/* */` comment), odometry drift, duplicate
  publishers. See `docs/ImplementationDoc.md` §M1 regression. **Still untested: a moving rig.**

- **FAST-LIO's ROS2 port shipped ~1.4 s of odometry latency — fixed 2026-09-01.** Values and
  timestamps were both *correct*; they were just 1.4 s old, and **nothing reports it** (SUPER's
  freshness check uses wall-clock arrival, not the stamp), so it is invisible on a stationary
  rig. Two compounding defects in the port (PR #381), both patched by
  `patches/fastlio_executor.py` via `setup.sh`: a single-threaded executor with **zero** callback
  groups (the always-overdue 100 Hz timer starves the 10 Hz cloud callback), and a depth-20 cloud
  queue that fills once at startup and never drains. Now **49-57 ms, 0/10 lagged** (was
  1386-1424 ms, 9/9 lagged). The patch also adds a `mtx_buffer` lock in `sync_packages()` —
  **don't drop that hunk**, it is a data race under the multi-threaded executor.
  Full write-up incl. measurement gotchas: `docs/fact-fastlio-latency.md`.

- **The FAST-LIO patch lives in `patches/fastlio_executor.py`, applied by `setup.sh`.** The
  workspace copy at `~/super_ws/src/FAST_LIO_ROS2` is already patched and built. The script is
  idempotent, and a fresh clone reproduces byte-identical source to what was tested.

- `patches/` — source patches applied to the **third-party** repos by `setup.sh`. Both idempotent.
  `fastlio_executor.py` fixes FAST-LIO's ~1.4 s odometry latency (callback groups +
  `MultiThreadedExecutor` + queue depths + a `mtx_buffer` lock in `sync_packages()`).
  `fastlio_frames.py` renames the TF frames (`camera_init`→`lidar_init`, `body`→`lidar_imu`) —
  it must run **after** the executor patch, which anchors on unrenamed text. It rewrites only
  string literals, never identifiers (`body` is a very common variable name in that file).

## Estimator health: divergence, rate, the LiDAR link

- **✅ G0.2 CLOSED 2026-09-13 — the 09-12 "86 m" divergence was 3.74 km, its cause is a SECOND
  LIVOX DRIVER, and it is now refused at three layers.** Read
  [`docs/fact-fastlio-divergence-2026-09-12.md`](docs/fact-fastlio-divergence-2026-09-12.md).
  In `20260912_2006_stageA` FAST-LIO sat at **(−3340, −1672, −86.7) m** and the aircraft was armed
  twice on it. `dtp_system.launch.py` had been started on top of `run.sh` and brought a second
  driver that took the MID360's stream and published it as PointCloud2. FAST-LIO (CustomMsg) got
  IMU and **no scans for 225 s**, silently, then integrated the backlog and came back kilometres
  off with mm covariance. **Reproduced on the bench** (+2.1 km in 120 s, stationary).
  **Don't re-derive:** `extrinsic_est_en` is *not* the trigger (on since the initial commit, every
  gate passed with it). **Now in place, all hardware-verified:**
  (1) `run.sh`, `sensors.launch.py` and `super.launch.py` refuse a second driver / FAST-LIO /
  `fsm_node`, and `dtp_system.launch.py`'s `xfer_format` default is 1, not 0;
  (2) **`px4_odom_bridge` latches DIVERGED** on a step > max(0.5 m, 5 m/s·dt), a *stamp* gap
  > 1.0 s or a first sample > 1.0 m from origin. It never re-trusts the same FAST-LIO, resumes only
  on a restarted one with `reset_counter`+1, and reports on `/px4_odom_bridge/status`;
  (3) **`python3 scripts/px4/estimator_preflight.py` (60 s) must say GO before every arm**. It is
  also inside `stage_a_preflight.py` and on the dashboard.
  **Deployed:** the bridge is copied into `~/super_ws/src/super_bringup/scripts/` (an install
  symlink points there). **Restarting only the bridge far from the origin needs
  `max_start_radius_m:=-1.0`**, deliberately. Carrying the aircraft after bring-up now means
  restart FAST-LIO at the takeoff spot, or the preflight refuses (E5).
  **Open:** slow in-flight drift with no jump/gap is not detected; the guard's thresholds have
  never seen a moving rig.

- **⚠️ FAST-LIO was delivering 3.75 Hz, not 10 Hz (2026-09-12, undiagnosed).** `/livox/lidar`
  measured a clean 10.00 Hz while `/lidar_slam/odom` and `/cloud_registered` were **3.75 Hz**, and
  `fastlio_mapping` RSS went 645 → 696 MB in ~4 min (every earlier check was flat at ~155 MB).
  That odometry feeds EKF2. Unconfirmed suspect: a laptop RViz subscribed to the dense
  `/cloud_registered` over Wi-Fi. **Not the G0.2 divergence** (that was a second Livox driver).
  But the same 09-12 bag has a related symptom: odometry *age* median 100–311 ms, p95 up to 571 ms
  over bag 180–350 s, past `EKF2_DELAY_MAX` (200 ms), coinciding with 17 EKF2 resets. The
  estimator preflight now refuses p95 > 150 ms (E4). Cause still undiagnosed.
- **⚠️ The LiDAR link was DOWN for this entire session, and the follower ran blind for 15 h 57 m
  without anyone noticing.** `eno1` was `DOWN`/`NO-CARRIER` with no IPv4, `ping 192.168.1.154`
  failed, `/livox/lidar` and `/lidar_slam/odom` had publishers but produced **no messages**, and
  **two** Livox driver instances were running (the older holding `192.168.1.50:56xxx`, the newer
  bound to the Wi-Fi address instead). Everything downstream behaved correctly — the watchdog said
  `NEVER PUBLISHED`, perception published empty arrays at 15 Hz, the mux sat in `HOLD`.
  **Check `ip -brief link show eno1` and `ping 192.168.1.154` before believing any perception
  result**, and check for a duplicate driver before starting one.

## Compute: pinning, swap, soak, DDS non-blocking send

### Added 2026-09-21 — `jetson_clocks` now runs at every boot
- **`sudo bash scripts/jetson_clocks_setup.sh install`** (done 2026-09-21 18:40) enables
  `super-dtp-jetson-clocks.service`: a oneshot `jetson_clocks`, ordered after `nvpmodel.service`.
  `show` needs no sudo. Survives a reboot, not a reflash.
- **Why:** the Jetson rebooted twice in the 2026-09-21 field session (16:23, 18:28), and each time
  the GPU came back at 306 of 918 MHz. The live preflight FAILed `jetson_clocks` both times. The
  first time, perception also went intermittently BLIND (~10 % of frames, `extrapolation into the
  future`: its starved tf2 listener, `docs/dtp-follower-stack.md`), so `/dtp/sensors_ok` held the
  mux in `HOLD|SENSORS`. Pinning the clocks stopped it at once: 0 new blind frames in 31 s.
- **Confirmed across a real reboot (2026-09-21 23:19):** the unit ran at boot
  (`active (exited)`, status 0), and `show` read GPU 918 of 918 MHz, pinned. The desk check and
  both preflights still check the clock, so a boot where it did not take is caught.

### Added 2026-09-13 (sixth session) — M5.3 compute: pinning, raycasting cost, swap fix, soak
Bench only, disarmed, at the desk, full graph observe-only (`~/dtp_datasets/20260913_m53_soak_bench/`).
Nothing here is the M5.3 exit (that needs the aircraft and a walker). Details: `docs/m5-m6-plan.md`
§4 M5.3 item 3.
- **CPU pinning is ON BY DEFAULT.** `run.sh`: FAST-LIO on cores `6,7` (`FASTLIO_CPUS`), Livox +
  `fsm_node` on `0-5` (`STACK_CPUS`); `dtp_follower.sh`: the DTP tree on `0-5` (`DTP_CPUS`); the
  GCS's bridge rows: `taskset -c 0-5`. `FASTLIO_CPUS=` (empty) turns it off; fewer than 8 cores
  turns it off by itself. Check: `grep Cpus_allowed_list /proc/<pid>/status`.
  - **Trap:** a launch `prefix` relabels the node's log lines `[taskset-N]`; the launch file sets
    `exec_name` so they stay `[fastlio_mapping-1]` / `[fsm_node-3]`. Nothing in the repo parsed
    the labels, but a human reading the log would have.
  - `taskset` execs in place, so pids, argv and the GCS's group adoption are unchanged.
- **Raycasting costs ~7 points of a core, not 48.** Same graph, config swapped only, 3 min each:
  `fsm_node` 22.5 % (on) vs 15.2 % (off). The earlier 88 % / 48 % readings were planning load and a
  20 s sample, not the map. Raycasting stays on.
- **The C2 swap FAIL is fixed: `sudo bash scripts/jetson_swap_setup.sh install`** — sysctl drop-in
  `vm.swappiness = 10` (stock 60). The rehearsal's 4 MB swap-out was reclaim parking idle pages in
  zram while a bag filled the page cache, with 7 GB available. Survives a reboot, not a reflash
  — the script is idempotent; `show` prints the state and `pswpout` since boot.
- **20-min soak, pinned, swappiness 10, recorder running: 5 of 6 pass; O2 FAILS on ONE ~10 s
  episode at minute 5.5 (max odom gap 944 ms).** 10.00 Hz, CPU 292 %, **0 pages swapped**,
  perception 65 %. In the episode odom/`/tf`/the bridge output all gapped 940 ms, the XRCE agent's
  50 Hz topics did not, FAST-LIO's CPU did not rise, and a 300 ms stamp gap shows two scans dropped
  afterwards: **FAST-LIO waited on something other than CPU.** A laptop RViz was pulling
  `/cloud_registered` over Wi-Fi at **48 Mbit/s** the whole run (BEST_EFFORT, but FastDDS's UDP
  socket is blocking, so a Wi-Fi stall blocks the publish inside FAST-LIO's callback) — the same
  suspect as the 09-12 3.75 Hz event, still unproven. **Field rule: nobody subscribes the laptop
  to `/cloud_registered` while flying.** Candidate bench fix: FastDDS ASYNCHRONOUS publish mode
  for FAST-LIO via an XML profile, then repeat the soak with the laptop subscribed. Bag and
  per-window CSV: `~/dtp_datasets/20260913_m53_soak_bench/`.
  **→ REPRODUCED AND CLOSED the same evening, at the user's request: read
  [`docs/fact-fastdds-blocking-send.md`](docs/fact-fastdds-blocking-send.md).** The Wi-Fi cannot
  be throttled on this box (no shaping qdiscs in the Tegra kernel, Realtek driver refuses rate/power
  control), so `strace -e inject=sendto:delay_exit` stood in for a slow link: **20 ms per send took
  odom from 10 Hz to 1.9 Hz with 1.1 s gaps and FAST-LIO's CPU to 13 %** — the episode's signature.
  **ASYNCHRONOUS publish mode does NOT fix it** (3.8–4.2 Hz, same CPU drop: one sender thread per
  process, the callback waits behind it). **The fix is FastDDS `non_blocking_send`**
  (`super_bringup/config/fastdds_nonblocking.xml`, `FASTLIO_DDS_PROFILE=nonblocking` in `run.sh`,
  `fastlio_dds_profile:=nonblocking` in the launch), verified by the `O_NONBLOCK` flag on FAST-LIO's
  UDP sockets. **Also found: the laptop is sent every datagram twice** — once to its Wi-Fi IP, once
  to its Tailscale IP (48 Mbit/s of air for a 24 Mbit/s cloud); **fixed 2026-09-14 in
  `laptop_viewer.sh` (interface whitelist via `scripts/laptop_dds_profile.sh`, default on),
  verified on the Jetson at socket level AND by a real laptop launch 2026-09-14: FAST-LIO sends
  to the laptop's Wi-Fi address only, Wi-Fi transmit 47 → 19 Mbit/s** — the acceptance `strace`
  line is in the fact file. Trap: a whitelisted and a default participant on one host do
  not talk, so other `ros2` commands on the laptop need the same profile exported. `strace` is now
  installed. **The 20-min soak on the fix, laptop
  subscribed at 47 Mbit/s throughout: 6/6 PASS, max odom gap 144 ms** (was 944). **`nonblocking`
  is now `run.sh`'s default** (`FASTLIO_DDS_PROFILE=` empties it); the laptop-subscription field
  rule stands until it has flown.
- **`super-gcs.service` was restarted**, so the dashboard now serves the fifth session's follower
  API as well as its page.

- **M5.3 soak:** `drone-dtp/scripts/m53_soak.py` (20 min, scores the §2.2 compute exit).
  - **Bench, full graph, logloader stopped:** odom 10.00 Hz / max gap 126 ms, CPU 384 % (80 % of it
    desk load), perception 70 %.
  - **C2 FAIL:** ~4 MB swapped out, with MemAvailable 7.2 GB and swappiness 60.

See also: `setup-and-build.md` (rebuilds that revert `config/`), `px4-estimator.md` (what EKF2 does with this odometry).
