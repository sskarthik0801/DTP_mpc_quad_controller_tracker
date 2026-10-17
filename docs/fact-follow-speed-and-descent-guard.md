# Why the follower flew slower than 0.5 m/s, and what changed (2026-09-16/17)

**Status: diagnosed from the 2026-09-15 20:01 flight; changes built and bench-gated; NOTHING
FLOWN since.** Everything below is from one bag and its logs unless it says otherwise. All
changes take effect the next time the stack and the follower are launched.

## TL;DR

- The follower was in control for **13 s**. Commanded horizontal speed had a **median of
  0.39 m/s and never exceeded 0.496**; PX4 tracked it (EKF2 median 0.36, peak 0.70). **The
  planner set the pace, not PX4.**
- **Why the planner held it under 0.5:**
  1. `max_vel` is a **ceiling on 3-D speed**, not a cruise speed, and every SUPER plan **ends at
     rest** at a goal that sat only ~2 m ahead.
  2. The walker stood still for the first ~4 s.
  3. Once walking, the walker (0.5–0.8 m/s) was faster than the cap, so the aircraft fell behind
     and the lock was lost at 6.5 m.
  4. **The goal's altitude flapped between 0.5 m and the aircraft's altitude**, because the
     follow standoff sits right on the descent guard's 3.0 m radius.
  5. **About 40 % of SUPER's replans failed** during the follow; for two seconds every one did.
- **User decisions:**
  - `max_vel` raised to **1.0** (2026-09-16);
  - `max_acc` **stays 0.5 for now** (2026-09-17), then raised to **1.0** (2026-09-21, §4);
  - altitude fixes **A + B** (2026-09-17): a fixed follow height of 1.2 m, and a descent guard
    that latches an altitude floor.
- **Also found:** after the lock was lost, auto-reacquire locked a **different person** and the
  aircraft was sent toward a goal 6 m behind it for 0.6 s, in Offboard.

## 1. The flight and the data

| | |
|---|---|
| Bring-up | `drone-dtp/scripts/deploy_follow.sh` (live), 2026-09-15 19:55 |
| Logs | `~/dtp_datasets/20260915_1955_deploy/` (`stack.log`, `follower.log`, both bridge logs, `record.log`) |
| Bag | `~/dtp_datasets/20260915_2001_stageB_salvaged` (552 s, 8.2 GB; the original `20260915_2001_stageB` was re-indexed into it) |
| SUPER config then | `follow_test_ros2.yaml`: `max_vel 0.5`, `max_acc 0.5`, `goal_vel_en false`, `replan_rate 15` |
| Bag t = 0 | epoch 1789516864.75 = 20:01:04.75 EDT |

Timeline in bag seconds, with local time:

| t (s) | local | event |
|---|---|---|
| 195 | 20:04:19 | FOLLOW engaged, track 1 locked at 3.9 m (aircraft on the ground) |
| 451.1 | 20:08:36 | armed (Altitude mode), then Position mode at 458.4 |
| **463.2** | **20:08:47** | **Offboard**, mux `FOLLOW\|OK` |
| 463–467 | | walker standing still (0.03–0.08 m/s); goal 1.3 m ahead, fixed |
| 467–476 | 20:08:51–09:01 | walker walking at 0.5–0.8 m/s; range to aircraft grows 4.4 → 6.5 m |
| 475.5 | | lock COASTING ("target occluded") at 6.3 m |
| **476.3** | 20:09:01 | mux `HOLD\|NO_SOURCE`; lock LOST at 476.8 |
| 481.9 | 20:09:06 | auto-reacquire locks **track 3** at 7.9 m (§3) |
| 482.45–483.1 | 20:09:07 | mux `FOLLOW\|OK` with goal (−1.50, 1.50, 0.71); LOST again at 483.4 |
| **497.6** | 20:09:22 | Offboard off (Position); disarmed 512.5 |

## 2. Why it flew slower than 0.5 m/s

Measured in the FOLLOW window, 463.2–476.3 s (`follow_speed_report.py`, §8):

| quantity | value |
|---|---|
| commanded horizontal speed | median 0.39, p95 0.48, **max 0.496 m/s** |
| commanded vertical speed | median 0.12, p95 0.22 m/s |
| commanded 3-D speed ≥ 0.45 m/s (90 % of max_vel) | 37 % of the time (49 % in the walking part, 467–476) |
| commanded path | 4.52 m horizontal, **1.47 m vertical**, z 0.67–1.14 |
| EKF2 horizontal speed | median 0.36, p95 0.62, max 0.70 m/s |
| FAST-LIO | 3.77 m net (0.29 m/s), 4.65 m path (0.35 m/s) |
| walker | median 0.51, p90 0.74 m/s; 5.47 m net (0.42 m/s) |
| goal ahead of the aircraft | median 2.1 m (p10 1.3, p90 3.2) |
| goal to walker | 2.77–3.07 m while walking (3.4–3.6 m while the walker stood) |

### 2.1 `max_vel` is a 3-D ceiling, and every plan stops at its goal

- **The limit is on 3-D speed.** SUPER's trajectory optimiser penalises
  `vel.squaredNorm() - vmax²` (`super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp:158`).
  Vertical motion therefore uses up the same 0.5 m/s budget as forward motion.
- **Every plan ends at rest.** With `goal_vel_en: false`, each trajectory ends with zero velocity
  at the goal (`super_planner.cpp:724-732`). The goal sat only ~2 m ahead (the 3–4 m follow band
  behind a walker), so SUPER was always partly braking for it.
- **Turning `goal_vel_en` on would not help here.** It only applies when the goal is more than
  `planning_horizon/2` = 3.5 m away (`super_planner.cpp:726`).
- **Consequence for the new `max_vel` of 1.0:** with `max_acc` still 0.5, reaching 1.0 m/s takes
  2 s and about 1 m of distance. With goals ~2 m ahead, the aircraft will rarely reach 1.0. This
  is a known consequence of keeping `max_acc` at 0.5.

### 2.2 Only 13 s of following, and 4 of them standing

- Offboard lasted 34 s, but the mux was in FOLLOW for only 13 s of it.
- For the first ~4 s the walker stood still. The goal was a fixed point 1.3 m ahead, which SUPER
  reached and held.

### 2.3 The walker was faster than the cap

- From 467 s the walker moved at 0.5–0.8 m/s against a 0.5 m/s ceiling.
- The range grew 4.4 → 6.5 m, well past the 3–4 m band.
- At 6.3–6.5 m the lock coasted and was lost (476.8 s), and the mux held for the rest of the
  Offboard time.
- The S1 envelope (`docs/m5-m6-plan.md` M5.2) assumed a *standing* subject at about 0.3 m/s; a
  walking subject belongs to S2. `max_vel` is now 1.0 (§4).

### 2.4 The goal's altitude flapped (the fixable one)

- **How `follow_planner` sets the height.** Goal z is the target's z + `goal_height_offset_m`
  (0.5), floored at `goal_z_min_m` (was 0.5) (`follow_planner_node.hpp:910, 929`).
- **The target height is wrong.** The code assumes the target z is mid-torso, ~1.0–1.2 m
  (`follow_planner_node.hpp:221`). The tracked point actually reads near the ground: median
  −0.04 m in this window, and 0.03–0.20 m across the whole flight. So **every follow goal sat on
  the 0.5 m floor, below the aircraft** (which flew at 0.7–1.2 m).
- **What the descent guard did.** `command_mux` raised any goal below the aircraft to the
  aircraft's *current* altitude whenever a person was within 3.0 m of the aircraft or of the goal
  (`command_mux_node.py`, the old `_apply_descent_guard`). It decided this afresh on every tick.
- **Why it flapped.** `follow_planner` places the goal about 2.75 m from the person by
  construction (its lead budget). In flight the goal sat **2.77–3.07 m** from the walker, right on
  the 3.00 m radius. So the guard switched on and off from one goal to the next.
- **Measured:**
  - **32 of 59** goals were raised to the aircraft's altitude (±2 cm), 27 stayed at 0.5 m, and
    the goal height changed by ≥ 0.1 m **30 times** in 13 s;
  - `follower.log` shows the guard firing (throttled to one line per 2 s: "goal z 0.50 raised to
    1.03 / 0.72 / 0.97 / 1.19 / 0.87");
  - SUPER replanned to each new height, so the commanded vertical speed swung ±0.2 m/s and the
    aircraft's altitude wandered 1.08 → 0.71 → 1.21 → 0.80 m.
- **Why speed suffered.** Vertical speed counts against the 3-D cap (§2.1), and each flip reshaped
  the plan. The worst commanded-speed dip (0.46 → 0.27 m/s, 473–474 s) came while the height
  flipped between 0.5 and 1.2 m several times a second, and a lateral goal jump (y −0.17 → −1.28)
  happened at the same time.
- **A second defect in the old guard.** It followed the aircraft down: after a dip, the "current
  altitude" was low and the goal stayed low.

### 2.5 SUPER failed ~40 % of its replans

**Correction to the first analysis (2026-09-16):** it said replans "mostly succeeded (0–4
failures per second)", because it counted only `… return FAILED` and missed the
`generateBackupTrajectory return OPT_FAILED` lines. The corrected numbers:

- **77 failed replans in the 13.1 s window.** SUPER replans at up to 15 Hz, so that is ≤ 197
  attempts and at least 39 % failed.
- **The failures cluster at 471–475 s: 7, 11, 15, 15, 9 per second.** At 473 and 474 s every
  replan failed.
- **Almost all were the backup trajectory** failing its optimisation (`[BackOpt] Opt failed, Omg
  or thr or Pos violation` → `generateBackupTrajectory return OPT_FAILED, replan Failed return`).
  The exploratory trajectory failed only 8 times.
- **What a failed replan does.** `callReplanOnce` does nothing (`fsm.cpp:69`), so SUPER keeps
  flying the trajectory it committed earlier, which ends at rest at an older goal. This is
  consistent with the 473–474 s dip.
- **Cause: NOT established.** Two plausible contributors, both removed by the changes in §4:
  - goals at z 0.5 in the LiDAR start frame (~0.65 m above ground), near the inflated ground,
    where the backup corridor is thin (the yaw-hold harness found the same: "at 0 the inflated
    floor leaves a 5 cm corridor → backup optimiser fails", CLAUDE.md);
  - the altitude flips.
- **Against both:** 467–470 s had flips and a low aircraft too, with no failures.
- **Check on the next flight** with `follow_speed_report.py --stack-log`.
- **Measurement trap:** SUPER prints a *successful* replan with a bare `cout` (`fsm.cpp:71`). Those
  lines have no timestamp and are block-buffered: they arrive in bursts of ~90 regardless of when
  they happened. Only the failures (ROS logger, timestamped) can be binned by time.

### 2.6 PX4 was not the limit

- The bridge forwarded SUPER's command unchanged (`sp|v|` equals `cmd|v|` in every 0.5 s bin),
  and EKF2's velocity followed it, overshooting to 0.70 m/s at times.
- **Altitude tracking:** FAST-LIO z minus the commanded z had a median of +0.07 m and a max of
  +0.215 m in Offboard. The +0.07 m median is **consistent with** the LiDAR sitting 7 cm above the
  FC origin (`EKF2_EV_POS_Z = −0.07`). PX4 flies the FC origin to a setpoint that SUPER computed
  for the LiDAR's position. This is not verified. The +0.215 m maximum sized the guard's raise
  margin (§5).

## 3. Also found: auto-reacquire locked a different person, in Offboard

- `auto_reacquire_after_lost` has been true since 2026-09-15 (user). After the LOST at 476.8 s,
  `target_manager` locked **track 3 at 7.94 m, at (−3.3, 1.6)**.
- **It was not the walker.** Track 1 had been lost 5 s earlier at (11.5, 0.05), 15 m away; the
  walker did not cover that in 5 s.
- **What the aircraft was sent.** The mux took FOLLOW and published goal (−1.50, 1.50, 0.71), with
  the fence clamp applied. The raw goal would have been further back. SUPER **accepted** it
  (adopted (−1.45, 1.55, 0.75)), **6.3 m behind the aircraft**. It was commanded toward it at
  0.38 m/s for 0.6 s, until that lock was lost too (483.4 s).
- This is exactly the risk `target_manager` logs an ERROR about at startup ("defeats D13/A10").
  **Nothing was changed about it.** It is recorded here for the user's decision.

## 4. Decisions and changes

| date | decision (user) | where |
|---|---|---|
| 2026-09-16 | `max_vel` 0.5 → **1.0** (the S2 speed) | `follow_test_ros2.yaml` (src + install, identical), `drone-dtp/patches/0001-…` (the `+` line, so a fresh install keeps it), `deploy_follow.sh` (pre-arm gate `--max-vel 1.0`, else it refuses the config), `flight-day-runbook.md` |
| 2026-09-17 | `max_acc` **stays 0.5** "for now" — S2 in the plan pairs 1.0/1.0 | unchanged; the yaml comment says so |
| 2026-09-17 | altitude fixes **A + B** | below |
| 2026-09-21 | `max_acc` 0.5 → **1.0**, so the S2 envelope is complete at 1.0/1.0. Reaching 1.0 m/s now takes 1 s and 0.5 m (was 2 s, 1 m) | `follow_test_ros2.yaml` (src + install, identical) and patch 0001's `+` line. Check: upstream 2ad3419 + the isnan fix + patches 0001–0007 rebuilds the live yaml byte for byte, and 0008 still applies. No gate reads `max_acc` (the preflights gate `max_vel` only and print the rest) |

**A — fixed follow height.**
- **Change:** `follow_planner_node/goal_z_min_m: 1.2` in
  `drone-dtp/ros2_ws/src/dtp_bringup/config/dtp_params.yaml`, with a comment giving the reasoning.
  That file is live-linked into the install tree, and `follow_planner` reads it at construction,
  so **restart the follower** after changing it.
- **Where 1.2 m is:** the LiDAR start frame (`lidar_init`) has z = 0 at the LiDAR's height at
  stack start, ~0.15 m above the ground, so 1.2 is **~1.35 m above ground**. That is inside the
  runbook's 1–1.5 m hover.
- **Effect:** because the target z reads near the ground, target + 0.5 never exceeds 1.2, and
  **every follow goal is exactly 1.2 m.** Verified against the real `follow_planner_node` with
  this file: 178/178 goals at z 1.2, goal 2.80 m from a walking target.
- **Also affected:** the stock-baseline arm and the M6.5 loss-viewpoint path, which use the same
  clamp (`follow_planner_node.hpp:729, 840`).
- **Not affected:** `command_mux`'s own `goal_z_min_m` (0.5), which clamps NAV and HOLD goals, and
  `loss_conditioning_check.py`, which launches the node on its defaults.

**B — latched descent guard.** New `command_mux/descent_guard.py`, used by
`command_mux_node.py`. §5 explains it.

## 5. The latched descent guard

**Rule (R-F9, the goal-level half):** never command a goal below the aircraft while a person is
within `descent_guard_radius_m` (3.0 m, horizontal) of the aircraft or of the goal. The check
covers every person on `/dtp/targets`, not just the one being followed.

| phase | behaviour |
|---|---|
| TRIP | goal below the aircraft and a person within 3.0 m → **floor = the aircraft's altitude at that moment** |
| ACTIVE | every goal's z is raised to at least the floor. The floor is **not** re-read each tick: it does not follow the aircraft down, and it does not creep up on tracking error |
| RAISE | only when the aircraft is **> 0.3 m** above the floor (`descent_guard_raise_margin_m`) — a pass over an obstacle. **Not a mux HOLD's own climb (+0.75 m), since 2026-09-21** (below) |
| RELEASE | nobody within **3.5 m** (radius + `descent_guard_release_margin_m` 0.5) of the aircraft or the goal for **1.0 s** (`descent_guard_release_dwell_s`) — the proximity veto's own numbers |
| RESET | on disarm (`vehicle_control_mode` armed → not armed), so a floor never carries into the next flight |

- **Logging.** Latch, raise and release are logged once each, not per tick. `/mux/state` appends
  `| alt floor Z` while latched. Only the first two `|` fields are machine-read anywhere
  (`stage_a_preflight`, `flight_timeline`, `field.sh`), so the suffix is safe; the fence already
  appends one the same way.
- **In a follow.** The goal sits 2.75–3.5 m from the person, inside the 3.5 m release distance, so
  the guard stays latched for the whole follow.
  - With A, the goal is at 1.2 m. The guard only trips if the aircraft is above that, and then it
    holds the aircraft's height.
  - **Operating rule, in the runbook: engage Offboard at about 1.2 m.** Engaged higher with anyone
    within 3 m, the aircraft follows at its engage height instead of descending.
- **Cost, stated plainly.** Between the floor and floor + 0.3 m, a goal can sit up to 0.3 m below
  the aircraft's *current* altitude with a person near; the old guard allowed 0. Without that
  margin the floor would ratchet upward on tracking error. That error sat +0.07 m above command
  on average in flight, so a pure running maximum would climb continuously.
- **Changed 2026-09-21 (user): a HOLD's climb no longer moves the floor.** In the 2026-09-21
  18:41 flight a 5 s `HOLD|NO_SOURCE` climbed the aircraft 0.75 m, and the floor went
  1.46 → 2.10 m. It never released in a 3–4 m follow, so the rest of the follow flew at 2.10 m.
  On a field sloping away, that was 3.4–3.7 m above the ground, and the walker left the camera
  frame (`.claude/rules/flight-log.md`).
  - **How.** `command_mux_node._report_hold_to_guard` calls `hold_started(z)` on the first tick
    of a HOLD, `hold_ended()` when it leaves, and `clear_hold()` whenever PX4 is not armed +
    Offboard. From a HOLD's start until the aircraft is back within 0.3 m of that altitude, the
    guard sees the aircraft at min(its altitude, the altitude at HOLD entry), for the trip and
    the raise. The floor and any new latch are where they would have been without the HOLD.
  - **Why only in Offboard.** A HOLD on the ground, or while the pilot flies Position, commands
    nothing. Counting it would register an altitude near the ground, and the guard would then
    never trip all flight. The 2026-09-21 flight had exactly that sequence: HOLD on the ground,
    Position takeoff, Offboard engaged mid-HOLD (gate case 1b).
  - **Cost.** After a HOLD, a goal can sit up to 0.75 + 0.3 m below the aircraft with a person
    near, and the aircraft comes back down to the floor while they are there. That includes a
    HOLD raised by the proximity veto, where the climb was taken to get away from someone. The
    proximity veto (1.5 m) still guards the path.
- **Fail-closed properties kept:**
  - a non-finite person is skipped, as before (the proximity veto vetoes on it);
  - with no odom, the floor is kept and the release dwell restarts;
  - the proximity veto still checks both the requested goal and the rewritten one.
- **A known edge.** The guard runs only on ticks that have a goal source. After a long NO_SOURCE
  gap the old floor is still latched and is released one dwell after goals resume, if nobody is
  near. That is the safe direction.

### What B does not do: obstacles (the user's branch question)

**Question:** if the aircraft never descends, how does it get past a branch in front of it?

**Answer: B pins the goal, not the path.** The earlier wording "never descends" was wrong. Three
things happen after the mux, and B changes none of them:

1. **SUPER's front end is a 3-D A\*.** Its cost is plain distance with a Euclidean heuristic
   (`astar.cpp:490-504`), so it has no preference for up or down. It takes the shortest free
   detour: under, over, or around.
2. **The corridors and trajectory are 3-D.** They keep `robot_r` (0.30 m) from mapped occupied
   cells (`corridor_generator.cpp`, `ciri.cpp`), and the path may pass below the goal's height
   before ending at it.
3. **A goal inside an obstacle is moved** to the nearest non-occupied inflated cell within 3 m,
   **in any direction, including down** (`fsm.cpp:230`, `rog_map.cpp:113`, spherical neighbour
   order). `command_mux` never sees that move. As the walker moves, the moved goal can flip
   between above and below a thin obstacle. That already happens today and B does not change it.

**Example.** Branch at 1.0–1.3 m, floor 1.2 m: SUPER plans a dip to ~0.5 m, passes under, and
climbs back to 1.2 m. The latch is what brings it back up. The old guard re-read the current
altitude, so after a dip it would have stayed low.

**Limits.** These are estimates from the config, NOT measured:
- **Going under needs a gap of ~1 m.** ROG-Map inflation is 2 × 0.2 m, so the path's centre stays
  ~0.4 m from obstacles above and below, plus 0.2 m of cell size.
- **Going over only exists below ~2.5 m.** The planner's ceiling is ~2.9 m in `lidar_init`
  (`virtual_ceil_height` 3.5, inset by `inflation_resolution × (1 + inflation_step)`,
  `inf_map.cpp:302`), so the branch top must be below about 2.5 m. Most canopies have no "over".
- **Going around can leave the 4 m box sideways.** The fence clamps goals only, and lands the
  aircraft only 3 m past the edge.
- **If nothing fits, the aircraft stops short.** Replans fail and it flies its old trajectory to
  rest. The walker is then occluded, the lock coasts, is lost, and the mux holds. It never forces
  its way through.
- **The ground only protects a dip if it has been mapped.**
  - The planner's hard floor (~−0.4 m, inflated virtual ground) is **below** the real ground
    (~−0.15 m).
  - At follow height the MID360 cannot see the ground within ~11 m (it looks no lower than 7°
    below horizontal).
  - During a follow, that ground was probably mapped earlier from further back. Never tested.
- **Near the walker, the guard does not stop a dip.** It never controlled paths. The path-level
  protection there is the proximity veto (1.5 m), and it tests the **straight drone→goal
  segment**, not SUPER's curved path (`_proximity_violation`).

## 6. Options considered for the altitude flapping

| option | what | verdict |
|---|---|---|
| **A** fixed follow height | `goal_z_min_m` 1.2 | **chosen.** Config only. Alone it stops the flapping only if Offboard is engaged at or below 1.2 m |
| **B** latched guard | hysteresis + a remembered altitude | **chosen.** Stops the flapping from any engage height; the aircraft never gets a goal below its latched height while someone is near |
| C | `descent_guard_radius_m` below the 2.5 m standoff floor | **rejected.** Switches off "don't descend near a person" exactly where it was written for, and the aircraft follows at ~0.65 m above ground (low flight put the depth reading on grass in the prop wash on 09-14) |
| D | smooth / rate-limit goal z | **rejected.** Hides the flips, still descends toward the person in steps, still replans every flip |
| (fix target z) | make perception's tracked point the torso | worth doing for its own reasons; **does not stop the flapping by itself** (whenever the aircraft is above the goal the old guard flips); open |

## 7. Verification (bench; nothing flown)

| gate | result |
|---|---|
| `command_mux` unit tests (`python3 -m pytest drone-dtp/ros2_ws/src/command_mux/test`) | `test_descent_guard.py` **17 passed**; `test_fence.py` + `test_abort.py` 21 passed; `test_flake8` passed |
| **`python3 drone-dtp/scripts/descent_guard_check.py`** (real `command_mux_node`, DDS **domain 86**, localhost only, ~45 s, fake PX4 state armed but not Offboard) | **11/11**: pass-through, latch, **no flap while the person crosses 3.0 m**, no drag-down on a dip, raise only on a real climb, release after margin + dwell, reset on disarm, goal above the floor untouched, one log line per event, FOLLOW\|OK throughout |
| same gate, `--node-script` = the pre-change node from git | **4/11**. Reproduces the flight: **13 height flips in 14 goals**, and it followed a dip down to 0.8 m |
| `python3 drone-dtp/scripts/fence_check.py` | 13/13 |
| **2026-09-21, HOLD climb change:** unit tests | `test_descent_guard.py` **26 passed** (9 new); all four test files 48 passed, flake8 clean |
| **2026-09-21:** `descent_guard_check.py` | **13/13**. New: 1b (a HOLD outside Offboard is not counted) and 4b (the flight: a HOLD climb to 2.25 m in Offboard, then FOLLOW, and the floor stays 1.50) |
| **2026-09-21:** same gate, `--node-script` = the node at `f4e31fa` | **9/13**. 4b fails as the flight did (floor 2.25), and 5, 6, 9 follow from it |
| **2026-09-21:** `fence_check.py` | 13/13 |
| real `follow_planner_node` with `dtp_params.yaml` (scratch harness, domain 86) | 178/178 goals at z 1.2 |
| `follow_test_ros2.yaml` | parses: `max_vel 1.0`, `max_acc 0.5` (1.0 since 2026-09-21, §4); src == install; patch 0001 still parses (`git apply --stat`) |

**Re-run `descent_guard_check.py` and `fence_check.py` after any `command_mux` change.** The
gate's domain 86 must not collide with the others (87 loss tools, 88 `target_filter_check`,
89 `fence_check` / `unhit_decay_check` / `loss_conditioning_check`).

## 8. Reproducing the analysis

```bash
source ~/super_ws/install/setup.bash
python3 drone-dtp/scripts/follow_speed_report.py ~/dtp_datasets/20260915_2001_stageB_salvaged \
    --max-vel 0.5 --stack-log ~/dtp_datasets/20260915_1955_deploy/stack.log
```

- **What it prints.** For every armed+Offboard segment and every FOLLOW window inside it:
  - the mux and lock transitions, with a note when more than one track was locked;
  - commanded, EKF2 and FAST-LIO speeds;
  - altitude tracking error;
  - walker speed and range;
  - goal distance;
  - goal-altitude levels, jumps, and how many goals sat at the aircraft's altitude (the old
    guard's signature) or under a latched floor (the new one's);
  - SUPER's failed replans per second.
- **`--max-vel`:** pass the value the flight used. The default reads *today's* config.
- **It refuses a bag without `metadata.yaml`.** Never read a bag that is still being recorded.
- **After the next flight, the numbers to compare against this one:** goal-altitude jumps (30
  here), goals at the aircraft altitude (32), failed replans (≥ 39 %), commanded speed against the
  new 1.0 cap, and the range to the walker.

## 9. Open

- **None of this has flown.** The latched guard, the 1.2 m follow height and `max_vel` 1.0 are
  bench-gated only.
- **~~`max_acc` 0.5 with `max_vel` 1.0~~ — `max_acc` is 1.0 since 2026-09-21 (§4), not flown.**
  With goals ~2 m ahead, accelerating to 1.0 and braking to the goal now takes ~1 m, so the
  commanded speed should reach 1.0. Check with `follow_speed_report.py --max-vel 1.0`.
- **The cause of the backup-trajectory failures** (§2.5) is not established.
- **Perception's target z reads near the ground**, not the torso (§2.4). A is a workaround, not a
  fix.
- **`auto_reacquire_after_lost: true` locked a different person** in Offboard (§3).
