# Fact: SUPER pirouettes on a re-planned hold goal, and patch 0006 stops it

**Found 2026-09-14 on the first Offboard flight of the person-following stack (Sortie 3, bag
`~/dtp_datasets/20260914_2020_stageB`). Fixed 2026-09-15, bench-gated, NOT yet flown.**

## What happened in the air

The follower had lost its lock one second after arming (see `CLAUDE.md`, flight day 2026-09-14),
so when the pilot engaged Offboard at bag t = 137 s `command_mux` was in `HOLD|NO_SOURCE`,
re-sending its latched hold goal (the takeoff spot, 0.76 m up) about twice a second. For the whole
44 s of Offboard:

| | value |
|---|---|
| setpoint position (NED) | (0.0, 0.0, −0.75) throughout |
| SUPER's commanded yaw rate on `/planning/pos_cmd` | median −52 °/s, min −129 °/s |
| `/planning/yaw_target` messages in the window | 0 |
| hold goal re-adopted (`/planning/goal_status`) | 86 times |
| aircraft heading | one full turn every ~7 s |
| aircraft position vs setpoint | p50 0.29 m, p95 0.61 m, max 0.94 m, orbiting with the heading |

Position mode just before, 125–136 s, held heading within 7° with no wander, and EKF2 agreed
with FAST-LIO to 10 cm median while flying. So this is not the yaw-mount spiral and not the
estimator. The M3 bridge only negates SUPER's yaw (`ψ → −ψ`); the spin was SUPER's own.

## Why

`follow_test_ros2.yaml` runs `yaw_mode: 3` (YAW_TO_TARGET). With no yaw target fresher than
0.5 s, `super_planner.cpp` leaves the trajectory's end yaw **free**, and
`YawTrajOpt::optimize()` (`super_planner/src/traj_opt/yaw_traj_opt.cpp`) then sets it to the
direction of the trajectory's displacement over its last 0.5 s, walking back toward the start
until it finds 0.5 m of motion. A hold correction never has 0.5 m: the loop reaches the start
and takes the direction of the whole trajectory, i.e. from wherever the aircraft is to the hold
point, a few centimetres of hover wobble, different on every replan. The heading slews toward
that new direction at up to `yaw_dot_max`; the yawing moves the aircraft; the next replan sees a
new direction. A limit cycle: heading and position orbit together.

The waypoint allocator in the same file already holds the previous yaw when a 0.5 s window moves
less than 0.1 m (`getYawWaypointAllocation`). Only the free end state lacked that floor.

## The fix: `drone-dtp/patches/0006-super-yaw-hold-on-short-trajectory.patch`

One file. In the free-goal branch, after the search loop, if the displacement found is under
`kMinAlignDispM = 0.3 m` the end yaw is set to the start yaw (hold heading) instead of
`atan2(dir)`. Above 0.3 m nothing changes: a real move still turns the nose into the direction of
travel, which the follower relies on. Independent of patches 0001–0005, applied by
`drone-dtp/scripts/apply_super_patch.sh` (marker `kMinAlignDispM`), live tree rebuilt 2026-09-15
00:00 (`colcon build --packages-select super_planner`, plain build). Backup of the source file
and the previous binary: `~/backups/backup-pre-yawhold-20260914-2356/`.

**The 0.3 m threshold is a judgment.** The flight's orbit reached 0.45 m, but that orbit was the
spin's product; a hover with a still heading should stay well inside 0.3 m under EV position
aiding. If a future hold still turns, the threshold is the knob, and the gate's `WOBBLE_M` is
where to test it.

## The gate: `python3 drone-dtp/scripts/yaw_hold_check.py` (~2 min)

Runs the real `fsm_node` on the private DDS domain 88 (the `target_filter_check` harness), on
`follow_test_ros2.yaml`, with a **360°** synthetic scene (floor, ceiling, box walls, so the map
around the aircraft is known: the 120° cone of the filter harness leaves the aircraft's own
cells unknown and the corridor fails) and an odometry that follows the planner's own
`/planning/pos_cmd` plus a 0.2 m hover offset rotating once per 20 s. Hover at 0.8 m as flown;
at 0 m the inflated floor (−0.45 + 0.4) leaves a 5 cm corridor and the backup optimiser fails.

- **H1** hold goal at the aircraft's own position, re-sent at 2 Hz for 25 s, planner re-planning
  0.02–0.29 m from the goal: commanded yaw excursion **0.0°**, |yaw_dot| p99 **0.0 °/s** over
  2192 commands. Same run on the pre-0006 binary (`--binary … --expect-spin`): **394° net turn,
  |yaw_dot| p99 45 °/s** — the bench reproduces the flight.
- **H2** a 3 m goal at +90°: yaw 0 → 92°, commanded travel 3.01 m. The nose still turns for a
  real move. (On the old binary the spin carries into the move; reported as info only there.)

7/7 on the new binary. Re-run it after any SUPER change, alongside `target_filter_check.py` and
`loss_oracle_check.py`.

## What this does not fix

- The lock loss that put the mux into HOLD in the first place (depth range jump at arming in
  tall grass, 1.2 s coast floor on a standing person). See the flight-day entry in `CLAUDE.md`.
- A hold with a **fresh yaw target** was never the problem: with a target the nose aims at it.
- The running stack keeps the old binary in memory until `fsm_node` is restarted (dashboard
  Stack Stop → Start, aircraft at its takeoff spot).
