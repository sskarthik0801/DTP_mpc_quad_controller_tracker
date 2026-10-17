# Durable fact — SUPER's `yaw_mode`, and why `click_smooth_ros2.yaml` runs 3

**Established 2026-09-12**, when `yaw_mode` on the click config was changed from 1 to 3.
Everything below is read from source; **none of mode 3's behaviour on the click config has been
measured**, on the bench or in flight.

Source, all under `~/super_ws/src/SUPER/super_planner/`: `include/super_core/config.hpp`,
`src/super_core/super_planner.cpp` (`generateExpTraj`), `src/traj_opt/yaw_traj_opt.cpp`,
`include/ros_interface/ros2/fsm_ros2.hpp`.

---

## 1. Upstream never reads `yaw_mode` — 1 and 2 are the same setting

`config.hpp` defines `YAW_TO_VEL = 1`, `YAW_TO_GOAL = 2` and loads `super_planner/yaw_mode`.
At the pinned upstream commit, **nothing else references it** (`git grep yaw_mode HEAD` finds only
the enum, the load, and the YAML files). The config comment "2 heading to goal" describes code
that does not exist. **Switching 1 → 2 is not a lever** — don't reach for it to fix yaw.

What actually decides the heading in modes 1 and 2:

- **Goal yaw at the end** only if `super_planner/goal_yaw_en` **and** the goal carries a yaw **and**
  the trajectory reaches the goal (`connected_goal`). On the click path `/goal_pose` also needs
  `fsm/click_yaw_en`, or the yaw is discarded (`fsm.cpp:205`).
- **Otherwise the end is free**, meaning the heading of the final ≥0.5 m of the path
  (`yaw_traj_opt.cpp`, `free_goal_`). Waypoints along the way face the direction of travel over
  0.5 m windows. Where the path moves less than 0.1 m, the heading is held.

## 2. What 3 (`YAW_TO_TARGET`) does — DTP patch only

Added by `drone-dtp/patches/0001-super-yaw-to-target-and-follow-goal.patch`; it is the only code
that reads `yaw_mode`, and it tests only for `== 3` (`super_planner.cpp:795`).

- **Input:** `/planning/yaw_target`, `geometry_msgs/PointStamped` in `lidar_init`
  (`fsm/yaw_target_topic`). **The subscription exists only when `fsm/follow_goal_en` is true**
  (`fsm_ros2.hpp:370`). The click config has had that since 2026-09-10.
- **Publisher:** drone-dtp's `follow_planner`. Nothing in the M1 stack publishes it. Measured
  2026-09-12 with `fsm_node` on the click config: **0 publishers, 1 subscriber**.
- **Target fresh** (`header.stamp` within `kYawTargetTimeoutSec` = **0.5 s**, `super_planner.h:102`):
  the end heading is `atan2` from the **trajectory's end point** to the target. This works when the
  goal is beyond the planning horizon (`connected_goal` false), which is the follow case.
- **Target directly above/below** (<1 mm horizontal): end left free.
- **No target, or stale:** end left free, i.e. heading-to-velocity — the same as mode 1.

**The cost: under 3 the goal yaw is never applied.** The goal-yaw check is the `else if` of the
mode-3 branch, so it is skipped even when no target exists. Consequences:

| | Under 1 / 2 | Under 3 |
|---|---|---|
| RViz *2D Goal Pose* arrow direction | used (click path flags on) | **ignored** |
| GCS remote-control **Rotate** | works | **refused** by `rc.py` (`yaw_mode: 3 (YAW_TO_TARGET)…`) |
| GCS forward/back/sideways, coordinate bar | unaffected | unaffected |
| `scripts/px4/goal_forward.py` | unaffected | unaffected (straight ahead, velocity-aligned = current heading) |
| M1 stack with nothing on `/planning/yaw_target` | heading-to-velocity | heading-to-velocity |

## 3. Where it is set, and how to undo it

`click_smooth_ros2.yaml` is what `super_bringup/launch/m1_super_lidar.launch.py` loads, so this
covers `scripts/run.sh`, the GCS and the flight-test plan. `follow_test_ros2.yaml` (the DTP
follower's config) already had `yaw_mode: 3`.

It lives in three places, and all three must agree:
- `~/super_ws/install/super_planner/share/super_planner/config/click_smooth_ros2.yaml` — **the copy
  `fsm_node` loads**, and the one `rc.py` reads first. It is a plain copy, not a symlink.
- `~/super_ws/src/SUPER/super_planner/config/click_smooth_ros2.yaml` — copied over the install
  copy by the next `colcon build`.
- `scripts/setup.sh` — re-applies it after a re-clone. Idempotent; harmless on an unpatched SUPER.

`fsm_node` reads the file only at startup: **restart the stack after changing it.** The GCS panel
reads the file directly, so it changes before the planner does.

**To get Rotate / goal yaw back:** set `yaw_mode: 1` in all three places and restart.

## 4. `yaw_dot_max` is a pacing hint, not a clamp

Relevant because mode 3 aims at a moving point. `yaw_traj_opt.cpp`:

- `interp_dt = π / yaw_dot_max` is a **time allocation**. Yaw waypoints are spaced so each segment
  can turn up to π at roughly `yaw_dot_max` on average. At the shipped **0.5 rad/s** that is
  6.3 s per segment.
- A trajectory shorter than `2·interp_dt` (12.6 s at 0.5) gets **no intermediate waypoints**: yaw
  is one polynomial from the start heading to the end heading over the whole duration, with no
  rate bound.
- Exceeding `yaw_dot_max + 2.0` only prints `Yaw rate too large`. The `return false` after it is
  commented out.

`px4_offboard_bridge` forwards `yaw_dot` unclamped. So `yaw_dot_max: 0.5` does **not** guarantee
29°/s. A short hop whose end heading differs a lot from the start is where it will be exceeded.
Check the logged `TrajectorySetpoint.yawspeed` rather than trusting the config number.
