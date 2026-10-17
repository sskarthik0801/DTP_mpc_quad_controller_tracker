# Flight day — the person-following test, step by step

**Written 2026-09-14 for the first flight with a person to follow.** This is the day's script, in
order, with three ways to do each step: a button on the web GCS (`http://gypsy.local:8080`), a
terminal command, or "ask Claude to run step N".

**Since 2026-09-20 the GCS's Flight day card is NOT this file row for row.** It is the short
standard bring-up — pick Observe only or Live, then press down twelve buttons — and **the step
numbers below are this file's own, not the card's.** The GCS column still names real buttons:
bring-up, follower, record, preflight, engage and the stops are on the card; *Desk check*, *RC
switches* and the mannequin preflight are chips on the Checks card; the `m51` recording preset
has no button any more (record from the Recording card, or a terminal). The card's Live preflight
is `live_preflight`, which passes `--max-vel 3.0 --waive-d21` as `deploy_follow.sh` does.

**Three sorties, and the order is not optional.** Sortie 1 (Stage A: the pilot flies, the follower
only watches) has never been flown on a sane estimate. Sortie 2 (M5.1: props off, the follower in
command) has only been rehearsed on a bench. Sortie 3 is the person-following flight. Each needs
the one before it *scored*, not just flown. The runbooks behind this file:
[`m5.4-m5.5-field-runbook.md`](m5.4-m5.5-field-runbook.md) (Stage A),
[`m5.1-one-graph-runbook.md`](m5.1-one-graph-runbook.md) (props off),
[`m5-m6-plan.md`](m5-m6-plan.md) §2.2 (thresholds), [`fact-geofence-command-mux.md`](fact-geofence-command-mux.md).

**Two flight blockers the docs still list, which are the pilot's call, not this file's:** the
autopilot's `uxrce_dds_client` hard fault (486 clean reconnect cycles on the client-side fix, but
the 14 h powered soak and the shell-path reproducer have not been run — `firmware/faults/README.md`),
and the MID360→FC **yaw** mounting, never measured (a mechanical check: is the sensor square to the
nose? a wrong angle spirals the aircraft under position control — `CLAUDE.md` M2).

## Roles, venue, envelope

- **Pilot:** transmitter, channel 5 on **Position**, thumb on **channel 11** (Offboard ON/OFF).
  Channel 11 OFF is the only abort; after a mux ABORT the follower must be relaunched before the
  next engage. Arming is always the pilot's.
- **Operator:** the Jetson over SSH (`ssh ideas-ad@gypsy.local`) or the GCS in a browser.
- **Walker:** knows the corridor, the 4–6 m band, and which trunk to step behind.
- **Venue:** a corridor ≥ 30 × 4 m, occluders that rise above chest height at 4 m (trunks, not
  bushes — the LiDAR sees no lower than 7° under its horizon: 0.5 m below the sensor at 4 m),
  nobody else inside the box, nobody within 1.5 m of the aircraft's path (the mux holds).
- **Envelope:** `max_vel 1.0 m/s`, `max_acc 1.0 m/s²` (S2) **since the 2026-09-30 rollback to the
  pre-2026-09-26 code** (branch `rollback-pre-0926`; it was 3.0 / 2.0 from 2026-09-26, see `CLAUDE.md`;
  baked into `follow_test_ros2.yaml`), standoff 3–4 m, hard minimum 2.5 m. Low wind.
- **Follow height 1.2 m in `lidar_init` (~1.35 m above ground)** since 2026-09-17
  (`follow_planner_node/goal_z_min_m` in `dtp_params.yaml`). Engage Offboard at about that
  height: the mux's descent guard latches the aircraft's altitude while anyone is within 3 m, so
  engaging higher means following at the engage height, not descending to 1.2. **With terrain
  follow on (since 2026-09-22) the same holds above the local ground.**
- **The engage gate (2026-09-22, bench-gated, not flown).** The mux sends SUPER nothing until PX4
  is armed + Offboard, so SUPER's first plan starts from the aircraft. Before ch11, `/mux/state`
  ends `to SUPER: held until Offboard`, and no planned path shows. At ch11 ON the aircraft hovers
  for up to ~1 s, then moves. It replaces the 3.0 m lurch of the 2026-09-21 20:05 engage. **Not
  covered:** re-engaging within a few seconds of a takeover. Wait ~5 s in Position before ch11 ON
  again.
- **`goal_mode` stays `geometric` today.** `follow_planner` has a second goal implementation
  (`goal_mode: nmpc`, 2026-09-19/20) that is bench-gated only and is not in the installed binary.
  This runbook's sorties are its BASELINE: do not pass `goal_mode:=nmpc` or
  `deploy_follow.sh --goal-mode=nmpc`. The preflight prints the arm, and under `--stage-b` it
  refuses nmpc without `--nmpc-cleared`. When it does fly, it is two Stage A legs on one sortie
  (geometric, relaunch, nmpc), both scored with
  `analyze_shadow_goals.py --goal-topic /dtp/follow_goal`. → `docs/fact-nmpc-follow-goal.md`
- **Laptop RViz must not subscribe to `/cloud_registered` while flying** (it stalled FAST-LIO by
  38 s on 2026-09-13). The `field_check` step catches it.

## 1. At the desk, before leaving (steps 1–4; GCS: the Checks card)

| # | Step | GCS | Terminal / ask Claude | Pass |
|---|---|---|---|---|
| 1 | Desk check: log loader disabled, ≥ 20 GB free, GPU clocks pinned, swap, LiDAR link, ARK services | **Run** (Desk check) | `bash scripts/desk_check.sh` | `DESK CHECK: PASS`. Swap > 64 MB is a WARN: reboot, then `sudo jetson_clocks` |
| 2 | Fault card empty | **Run** (Fault card) | `python3 scripts/px4/clear_fault_logs.py` | "No fault_* files". If not: archive + clear from the Checks card |
| 3 | Transmitter on; read the Offboard switch | **Run** (RC switches) | `python3 scripts/px4/px4_shell.py "listener manual_control_switches"` | `offboard_switch` 3 with ch11 down, 1 up. `never published` = transmitter off |
| 4 | Bench pre-check, 8 min | — | `bash drone-dtp/scripts/bench_bringup.sh` or *ask Claude: run the bench pre-check* | estimator GO, Stage A preflight GO (2 expected warnings), 0 survivors |

Done 2026-09-14 17:24 on this box: 1 PASS (swap WARN, 220 MB), 2 empty, 3 transmitter was off
(parameters confirm ch11 Offboard, ch5 modes, Position in slot 4), 4 passed.

## 2. At the venue: bring-up (steps 5–11)

Aircraft on the ground **at the takeoff spot, nose down the corridor**, transmitter on, before
anything starts. The map frame and the geofence are anchored where FAST-LIO starts; if the stack
came up anywhere else, restart it here or the estimator preflight refuses (E5).

| # | Step | GCS | Terminal | Pass |
|---|---|---|---|---|
| 5 | Geofence for the venue | — | edit `fence_x/y_*_m`, `fence_yaw_deg` in `drone-dtp/ros2_ws/src/dtp_bringup/config/dtp_params.yaml` if the corridor is not the configured box along the boot heading (**30 × 4 m, x [−2, 28], y [−2, 2], yaw 0.0 since the 2026-09-30 rollback to the pre-2026-09-26 code**; the 38 × 6 m, −17° box of 2026-09-29/30 is in tag `checkpoint-2026-09-30`) — point the nose down the corridor at stack start | the Stage B preflight prints the box; read it back against the ground |
| 6 | Stack on the **follow** config | **Start** (Stack, follow) | `SUPER_CONFIG=follow_test_ros2.yaml RVIZ=false bash scripts/run.sh` | `fsm_node on follow_test_ros2.yaml`; odom ≥ 9.5 Hz after 1–3 min |
| 7 | Odometry bridge | **Start** | `ros2 run super_bringup px4_odom_bridge.py` | Estimator card shows position aiding |
| 8 | Trajectory bridge | **Start** | `ros2 run super_bringup px4_offboard_bridge.py` | row running; it commands nothing until ch11 |
| 9 | FAST-LIO gates | **Run** (field_check) | `bash drone-dtp/scripts/field.sh check` | no foreign cloud subscriber, no stamp lag |
| 10 | Follower **observe-only** (Admin control) | **Start** | `DTP_MODE=observe bash scripts/dtp_follower.sh` | log: "GPU cloud verified", "extrinsic verified" |
| 11 | Stage A preflight | **Run** | `python3 drone-dtp/scripts/stage_a_preflight.py` | GO; the two expected WARNs are `extrinsic_est_en` and "bag not running yet" |

Or, in one go from a terminal: `bash drone-dtp/scripts/field.sh prep` does 6–11 with a check
after each step and stops on the first failure with the fix.

## 3. Sortie 1 — Stage A: the pilot flies, the follower watches (steps 12–17)

| # | Step | GCS | Terminal | Pass |
|---|---|---|---|---|
| 12 | Stage A recording (camera frames → Admin control) | **Record** (preset stage_a) | `bash drone-dtp/scripts/stage_a_record.sh` | Recording card: recording |
| 13 | Estimator preflight, 60 s — **before every arm** | **Run** | `python3 scripts/px4/estimator_preflight.py` | GO |
| 14 | Walker at 4–6 m, moving; engage; wait for LOCKED | — | `bash drone-dtp/scripts/field.sh arm`, then `field.sh watch` | LOCKED; watch prints one line per second and says when to re-engage |
| 15 | Pilot: arm in Position, hover 1–1.5 m, yaw sweeps | — | — | slow yaw ±90° at ~0.2, 0.5, 0.8 rad/s; walker on slow lines at 4–6 m; three ~5 min sorties |
| 16 | Land in Position, disarm, stop the recording | **Stop** (Recording card) | `field.sh land` | metadata.yaml written (a killed recorder writes none) |
| 17 | Score | — | `python3 drone-dtp/scripts/analyze_shadow_goals.py ~/dtp_datasets/<date>_stageA` — *ask Claude: score Stage A* | section 6 exit 0. Exit 3 = could not score = **not a pass**. Same bag: `bag_to_calib_pairs.py` (motor-on calibration) and the moving-rig latency check (`fact-fastlio-latency.md`) |

## 4. Sortie 2 — M5.1: props OFF, the follower in command (steps 18–26)

| # | Step | GCS | Terminal | Pass |
|---|---|---|---|---|
| 18 | **Props off** | — | — | motors will spin to hover throttle the instant Offboard engages armed: feedforward, not a fault |
| 19 | Stop the observe-only follower | **Stop** (DTP row) | Ctrl-C its shell | row stopped |
| 20 | Follower **LIVE** | **Start LIVE** | `DTP_MODE=live SUPER_CONFIG=follow_test_ros2.yaml bash scripts/dtp_follower.sh` | log: `command_mux "ABORT armed"` |
| 21 | Stage B preflight, mannequin | **Run** | `python3 drone-dtp/scripts/stage_a_preflight.py --stage-b --target mannequin` | GO: mux on `/planning/follow_goal`, shadow topics silent, fence on, filter as intended |
| 22 | Estimator preflight | **Run** | as 13 | GO |
| 23 | M5.1 recording | **Record** (preset m51) | topic list in `m5.1-one-graph-runbook.md` §4 | recording |
| 24 | Engage FOLLOW, walker at 4–6 m moving | — | `ros2 param set /target_manager follow_enabled true` | `/dtp/lock_state` LOCKED |
| 25 | Eight trials, one arm cycle ≤ 10 s each | — | pilot + operator | 1–4 follow (ch11 ON, `/mux/state` FOLLOW, wait for self-disarm); 5–7 takeover (ch11 OFF after ~3 s, hold ≥ 3 s); 8 abort (`kill -STOP <perception_node pid>` at ~2 s → NAV_LAND → `kill -CONT`). **After 8: stop and restart the follower** |
| 26 | Score | — | `python3 scripts/px4/m51_one_graph_check.py <bag>` — *ask Claude: score M5.1* | all four sections S/G/A/T scored; exit 3 = INCOMPLETE = not a pass |

## 5. Sortie 3 — the person-following flight, S1 (steps 27–35)

| # | Step | GCS | Terminal | Pass |
|---|---|---|---|---|
| 27 | Props on; everyone briefed | — | — | abort rules agreed: ch11 OFF on anything unexpected |
| 28 | Follower LIVE (restart it if it ran trial 8) | **Start LIVE** | as 20 | running · live |
| 29 | Stage B preflight, **person** | **Run** | `python3 drone-dtp/scripts/stage_a_preflight.py --stage-b --target person` | GO. It refuses `loss_conditioning` on, the target filter on, a missing fence, an unpinned GPU |
| 30 | Estimator preflight | **Run** | as 13 | GO |
| 31 | Stage B recording | **Record** (preset stage_b) | M5.1 list + perception + `/planning/loss_event` + camera | recording |
| 32 | Engage FOLLOW, walker at 4–6 m moving until LOCKED | — | as 24 | LOCKED |
| 33 | **First flights:** pilot takes off in Position to 1.5 m, hovers, ch11 ON for 20 s | — | — | aircraft moves to the 3–4 m standoff and points the camera at the walker; walker stands, then walks slowly; ch11 OFF, land in Position |
| 34 | **Longer legs:** a stroll down the corridor and back, then behind a trunk and out the other side | — | — | `/dtp/lock_state` re-locks; the oracle answers every coast (recorded, nothing acts on it: `loss_conditioning` is false) |
| 35 | Land in Position, disarm; stop the recording; stop the follower | **Stop** ×2 | `field.sh land` | bag playable, row stopped |

## 6. After landing (steps 36–37)

| # | Step | GCS | Pass |
|---|---|---|---|
| 36 | Scan the fault card | **Run** (Fault card) | empty. A dump = the FC reset in flight: archive before clearing |
| 37 | STOP ALL, then power off the Jetson from the Host card | **STOP ALL**, **Power off** | the ordered shutdown stops the GCS last, which keeps a running bag playable |

Then, at the desk, *ask Claude to*: score the Stage B bag (`analyze_shadow_goals.py`, the
follow-target jitter, the loss events with `loss_label.py sheets`), run the moving-rig latency
check, score the follow speed (`follow_speed_report.py <bag> --max-vel 3.0 --stack-log
<deploy dir>/stack.log`, compared against 2026-09-15 in
[`fact-follow-speed-and-descent-guard.md`](fact-follow-speed-and-descent-guard.md) §8), and write
the day into `CLAUDE.md`.

## What the GCS card cannot do, on purpose

Arm, change a mode, engage Offboard, or send a stop — none of these has a button. Arming and
channel 11 are the pilot's; there is no stop message for a page to send, so there is no STOP
button on the RC card either.

**Since 2026-09-15 the page CAN set `follow_enabled`: "Engage FOLLOW" / "Re-engage FOLLOW" on the
DTP row and on the Flight day card's engage step.** Added after Sortie 3, where a lock lost one second after
arming was never re-engaged and the whole Offboard segment flew a hold. It is behind Admin control,
it is refused without a follower, without telemetry, or after a mux ABORT, and it is deliberately
**not** refused on an armed aircraft and asks no confirm question: that is the moment it is for.
It does exactly what `field.sh reengage` does (`follow_enabled` false, then true), so the same
button engages and re-engages. The lock state (`/dtp/lock_state`) is the pill beside it.

## Traps, each of which has already happened

- **A follower left running from an earlier launch** (even just its viz nodes) makes the DTP row
  read "running · not ours" and refuses Start. *Stop outside nodes* on the Stack card, then Start.
  Two orphaned viz nodes from a 01:27 launch were found this way on 2026-09-14.
- **FAST-LIO started away from the takeoff spot** → E5 refuses; carrying the aircraft after
  bring-up means restarting FAST-LIO.
- **A recorder stopped by anything but SIGINT-by-pid** writes no `metadata.yaml`. The Recording
  card's Stop is the right way; so is `field.sh land`.
- **After an FC power cycle `mavlink-router` does not reconnect**: `sudo systemctl restart
  mavlink-router`, or the checks say "no PX4 heartbeat".
- **`pkill -f` kills your own shell.** Kill by pid, or use the dashboard.
- **A lock lost at arming, never re-engaged (2026-09-14 Sortie 3).** The prop wash in tall grass
  put a 3 m range jump on the walker's torso patch; a standing walker has only the 1.2 s coast
  floor, so LOST came 1.1 s later, and with auto-reacquire off the whole Offboard segment flew a
  hold, not a follow. Run `field.sh watch` from arming on, `field.sh reengage` on LOST, and have
  the walker keep moving (a moving walker has a multi-second forecast).
- **A hold goal made SUPER pirouette (2026-09-14, fixed by patch 0006, not yet flown).** With no
  target, `yaw_mode: 3` aligned the heading to the hover wobble on every replan: 52 °/s median
  for 44 s. `yaw_hold_check.py` must pass on the binary you fly.
- **The geofence box points where the nose pointed at stack start.** An aircraft that lands
  yawed (108° on 2026-09-14) then follows only to the box edge. Restart the stack at the spot with
  the nose down the corridor, or set `fence_yaw_deg` and relaunch the follower.
