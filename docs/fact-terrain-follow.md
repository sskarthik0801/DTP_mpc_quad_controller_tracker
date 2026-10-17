# Terrain-relative follow height (2026-09-21) — bench-gated, never flown, ON since 2026-09-22

**The switch:** `command_mux: terrain_follow_enable` in
`drone-dtp/ros2_ws/src/dtp_bringup/config/dtp_params.yaml`, relaunch the follower after changing
it. **It is `true` since 2026-09-22 (the user); it shipped `false`.** `follower.launch.py` reads that
one switch and starts `terrain_estimator` only when it is true. With it `false`, everything below
is off and the absolute behaviour is unchanged.

## 1. Why

The 2026-09-21 18:41 follow flight (`bags/stage_b_2026-09-21_183532`, `.claude/rules/flight-log.md`)
followed for 23 m, then lost the walker. Every follow goal's altitude is absolute in `lidar_init`
(z = 0 is the LiDAR's height where the stack started), and the field fell ~1.2 m along the
corridor. Holding 2.1 m in `lidar_init` put the aircraft 3.4–3.7 m above the ground at the far end
(ToF 3.55–3.79). The camera looks level with a 30.5° half-FOV, and at a ~3.4 m standoff the walker
slid out of the bottom of the frame. Two causes stacked:

1. A mux HOLD's +0.75 m climb latched the descent-guard floor at 2.10 m for the rest of the follow.
   **Fixed the same evening**: a HOLD's climb no longer moves the floor
   (`docs/fact-follow-speed-and-descent-guard.md` §5).
2. The slope. That is what this change is for.

## 2. The user's question, and why not the downward rangefinder

"If there is a sudden huge ditch, will the drone misbehave? If it goes by a tree and the range
finder hits the tree, the height changes." A goal slaved to the single-beam ToF does exactly that:

- **Ditch.** The reading jumps long at the edge, the goal drops by the ditch's depth, and SUPER sees
  the ditch as free space and flies into it.
- **Tree, bush, car, person.** The reading goes short, the goal pops up, then drops again past it.
- **Dropouts.** No reading at the range limit or over some surfaces.
- **The descent guard.** Absolute, it would have cancelled any terrain following: it will not put
  a goal below the aircraft with a person within 3 m, so downhill it would have held the takeoff
  altitude.

## 3. What was built

| piece | file | what it does |
|---|---|---|
| ground map (pure) | `command_mux/terrain_map.py` | FAST-LIO scans → 0.5 m cells, 32 pooled points each, cell = p20 of z. Ground at a point = median of the known cells within 1.5 m, ≥ 8 of 29 needed. Tracked people cut out to 1 m |
| estimator node | `command_mux/terrain_estimator_node.py` (`ros2 run command_mux terrain_estimator`) | `/cloud_registered_body` + `/lidar_slam/odom` (stamp-matched), `/dtp/follow_goal`, `/dtp/targets` → `/dtp/terrain` (PointStamped, z = the HIGHER of the ground under the aircraft and under the goal, NaN if neither is known). Every 2nd scan, 5 Hz |
| offset (pure) | `command_mux/terrain.py` | ground → offset from the takeoff ground (−0.15). In Offboard: ≤ 0.2 m/s down, ≤ 0.3 m/s up, 0.05 m deadband. Out of Offboard: snaps. Clamped to −1.2 / +1.5. HOLDS when `none` / `unknown` / `stale`, never decays to 0 |
| mux integration | `command_mux_node.py`, section TERRAIN-RELATIVE HEIGHT | FOLLOW z = `terrain_follow_z_m` 1.2 + offset. The goal clamps, the descent guard and the HOLD goal all work in height above the local ground. Absolute clamp [−0.3, 3.0] for ROG-Map. `/mux/state` appends `\| terrain +X.XX status` |

**Design choices, each for a stated failure:**
- **The HIGHER of the two samples** (under the aircraft, under the goal): climb early, descend late.
  A ditch must be wider than the aircraft–goal spacing plus the patch before the aircraft follows it
  down at all.
- **Median of cells, not the lowest point.** A narrow ditch or a bush is a minority of the patch.
- **FOLLOW z replaced, not offset.** follow_planner's z is target z + 0.5, floored at 1.2, and the
  target z reads the walker's shins. On this slope it stayed at −0.2 to +0.2 while the ground under
  the walker dropped 1.2 m, and it has been the 1.2 floor on every flight so far. Adding the offset
  to it would count the slope twice whenever the target z did track the ground.
- **The descent guard works in height above the ground.** Following the ground down with a person
  near is not "descending toward them".
- **NAV goals stay absolute** (an RViz click means a `lidar_init` point). The proximity veto and the
  geofence are unchanged: they compare absolute positions.

## 4. Evidence

**Replay of the 18:41 flight, open loop** (`drone-dtp/scripts/terrain_replay.py`). The check is
independent of the LiDAR: height above ground if terrain-relative = ToF + (the goal z the terrain
mode would have commanded − the altitude flown).

| run | flown (absolute), Offboard | terrain-relative, Offboard |
|---|---|---|
| estimator from the bag start (on the ground, as the launch does it) | p50 2.81, max 3.81 m | **p50 1.36, range 1.23–1.51 m** |
| estimator started in the air at 370 s | p50 2.81, max 3.81 m | p50 1.42, range 1.27–1.89 m: holds 0 until ground is known, then ramps at ~0.12 m/s |

The map tracked the slope causally: ground under the aircraft −0.20 → −1.33 as x went 0 → 27.5 m.
The aircraft landed on z −1.20, and the map said −1.32 there.

**Replay of the 20:05 flight (2026-09-22)**, `bags/stage_b_2026-09-21_195658_salvaged`.
- **Result.** In Offboard, height above ground was flown at p50 2.50 m, max 2.82 m.
  Terrain-relative, it would have been **p50 1.34 m, range 1.25–1.43 m**. The offset ran
  −0.07 → −0.97 as x went 2 → 23.6 m.
- **How it was run.** That bag has no `/planning/follow_goal` (a recorder QoS race), so the run
  used a scratch copy of `terrain_replay.py` that reads the goal from `/mux/state`.
- **⚠️ What the replay leaves out: the descent guard.** On that flight the guard latched at
  20:05:53, before Offboard, at the engage altitude of 1.67 m (~1.9 m above the ground), and it held
  through the whole follow. The guard works in the terrain-shifted frame, so with the switch on
  it would have held that height above the *local* ground instead: **~1.9 m above ground all the
  way**, flat, not 1.34 m.
- **So engage near 1.2 m.** The terrain-relative height is 1.35 m only when the floor does not
  latch above it. The same rule already applies to the absolute follow height
  (`dtp-follow-behaviour.md`).

**Cost on the NX, real-time playback of the bag:** 4.6 % of one core, RSS 59 MB, ~1.7 ms of CPU per
processed scan, 0 scans unmatched to odometry.

**Gates, all on the bench:**

| gate | result |
|---|---|
| unit tests (`python3 -m pytest drone-dtp/ros2_ws/src/command_mux/test`) | **79 passed**: `test_terrain_map.py` 19, `test_terrain.py` 12, plus the existing descent guard / fence / abort tests and flake8 |
| **`python3 drone-dtp/scripts/terrain_check.py`** (real mux + real estimator, closed loop, **domain 91**) | **10/10**: flat; a 1.4 m downhill with the walker 3.3 m ahead (goal follows at ≤ 0.2 m/s, steepest 0.13); clamp at −1.2; aircraft 1.50–1.75 m above ground throughout; a narrow ditch ignored; a bush ignored; a HOLD climbs 0.75 m from where the aircraft is and FOLLOW returns, floor not raised; estimator killed → `stale`, height held; mux alive, no traceback |
| `descent_guard_check.py`, `fence_check.py` (switch off) | 13/13, 13/13 |
| `follower.launch.py` `_launch_setup` with the switch false / true | `terrain_estimator` absent / present, the other six nodes unchanged |

## 5. Traps found while building it (don't re-derive)

1. **A per-scan minimum learned no ground ahead of the aircraft at all.** From 2 m up, the MID360
   sees the ground only in its bottom ~2°, 17–30 m out, at about one point per cell per scan. Three
   points per cell in one scan were never reached. Cells now pool 32 points across scans.
2. **The walker was read as the ground.** Started in the air, the patch under the goal had only
   the three cells the walker had stood in for 30 s. Their p20 was 0.70 against a true −0.35, and
   the goal climbed 0.9 m for 10 s. Fixed two ways: tracked people are cut out to 1 m
   (`/dtp/targets`), and `min_cells` 4 → 8.
3. **Keeping the last 32 points in cloud order** kept whatever the sensor scanned last. A cell
   over its budget now keeps 32 samples spread over that scan's sorted z.
4. **One logging call site at two severities killed the mux.** rclpy caches a call site's severity
   and RAISES ("Logger severity cannot be changed between calls"). The stale-terrain message
   logged WARN on the ground and ERROR in Offboard from one line. `terrain_check` T7 watched it take
   the node down, the moment the estimator died in Offboard. It is now two call sites, and T9 asserts
   the mux survives. **Any rclpy log line whose level varies needs one call per level.**
5. **With `abort_enable` false the mux never sees Offboard**: it subscribes to
   `vehicle_control_mode` only with the abort on. The first harness disabled the abort, and the
   rate limit and HOLD exclusion silently never engaged (the goal stepped at 0.8 m/s). Live follows
   always have it on. Observe-only runs snap the offset, which is harmless because nothing is flown.
6. **Scans that beat their odometry were dropped** (~25 % in playback). The node now holds one and
   retries when the next odometry arrives.
7. **Gates on different domains still collide under load.** `fence_check` + `descent_guard_check`
   + `terrain_check` together, on top of the live stack (load average 5.2), failed 8 and 2 cases on
   discovery timing. Alone: 13/13 each. Run gates one at a time.

## 6. Limits, stated

- **Never flown.** The first flight should be over known ground: the replay covers one field.
- **Relief is bounded to about −1.2 / +1.5 m from the takeoff spot.** ROG-Map's
  `virtual_ground_height` −1.0 and `virtual_ceil_height` 3.5 are absolute in `lidar_init`, and past
  them SUPER drops the goal. The offset clamps there and `/mux/state` says `clamped`. Following
  bigger relief means changing those two in `follow_test_ros2.yaml`: a SUPER config change, not made.
- **The estimator must run from the ground.** On the ground the LiDAR sees the ground from 1.2 m
  out, and that seeds the map where the aircraft takes off. Started in the air, it holds the offset
  at 0 until ground is mapped, and the first ~40 s of the replay flew absolute.
- **The ground under the aircraft itself is unknown on the ground** (nothing within 1.2 m). Before a
  lock, `/dtp/terrain` is NaN and the mux WARNs every 5 s. That is expected and not a fault. It
  ERRORs only in Offboard.
- **Only tracked people are cut out.** A parked car or a dog is read as terrain over its cells; it
  moves the median only when it covers most of the 1.5 m patch.
- **Under a dense canopy with no ground seen beneath it**, cells read the canopy's underside if it
  is below the sensor. That is the climb direction, and SUPER avoids the canopy itself. On this
  field the canopy was above the sensor and never entered the map.
- **The camera is still level.** Terrain-relative height keeps the aircraft ~1.35–1.5 m above the
  ground. At a 3 m standoff that frames the whole walker up to ~2 m. A 15° down-tilt would buy
  framing to ~3.4 m, and needs the camera↔LiDAR calibration redone.
