---
paths:
  - "super_bringup/**"
  - "scripts/px4/m3_*.py"
  - "scripts/px4/mpc_tracker_*.py"
  - "docs/mpc-tracker*.md"
  - "scripts/px4/offboard_engage.py"
  - "scripts/px4/fake_odom_pub.py"
  - "scripts/px4/goal_forward.py"
  - "docs/ImplementationDoc.md"
  - "docs/fact-super-cmd-stream.md"
  - "docs/fact-offboard-rc-arming.md"
  - "docs/fact-ev-frd-yaw-align.md"
---

# The PX4 bridges (`super_bringup/`) and Offboard (M3)

Scope: `px4_odom_bridge.py` (FAST-LIO → EKF2), `px4_offboard_bridge.py` (SUPER → PX4 setpoints), its MPC alternative `px4_mpc_tracker.py`, `offboard_viz.py`, and how Offboard is engaged. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

## 2026-10-17 — `px4_mpc_tracker.py`: an MPC alternative to the offboard bridge, integrated behind a switch (OFF by default, NEVER FLOWN, not installed)

The design is `docs/mpc-tracker.md`; the how-to (install, select, gate, fly, roll back) is
`docs/mpc-tracker-integration.md`. First written 2026-10-05 on `nmpc-follow-goal`, then ported
here and brought in line with this branch's bridge.

- **What it is.** It replaces `px4_offboard_bridge.py`; it does not run alongside it. It refuses
  to start if `/fmu/in/trajectory_setpoint` already has a publisher. It tracks
  `/planning/pos_cmd` with a linear MPC and sends PX4 **acceleration** setpoints
  (`ocm.acceleration`, position and velocity NaN). PX4 keeps attitude, rate and mixing.
  - The maths is in `super_bringup/scripts/mpc_tracker_core.py` (numpy only, installed as a
    module like `yaw_servo.py`): a per-axis double integrator with a tilt lag `tau`, plus an
    estimated constant disturbance `d` for offset-free hover.
  - It is solved as a condensed QP: the unconstrained solution when it fits the limits, else
    FISTA onto a 2.5 m/s² tilt disc and a [−2.0, +1.5] m/s² vertical box.
- **Kept identical to this branch's bridge:**
  - setpoints only while PX4 reports Offboard (the 2026-09-28 contract). Outside Offboard it
    sends the heartbeat only, and in disarmed Offboard a position setpoint at the estimate;
  - a silent `vehicle_control_mode` never stops the stream (a warning after 2 s);
  - the stream's start/stop edges are logged;
  - the camera yaw servo, through the same `yaw_servo.py`, with the bridge's parameter names and
    defaults. OFF by default. It rewrites only yaw and yawspeed in TRACK and HOLD, never the
    acceleration;
  - read-only parameters; a bad servo value stops the node at start-up.
- **What it adds over the bridge:**
  - the far-setpoint clamp: SUPER's present point more than `mpc_max_pos_err` (1 m) away shifts
    the horizon in to 1 m. That is the "refuse a far setpoint" the 2026-09-21 flight asked for;
  - FALLBACK: a stale or invalid estimate, or an MPC failure, hands back to a PX4 position setpoint.
- **How it is selected (default everywhere: the bridge):**
  - the GCS: `GCS_TRACKER=mpc` when the GCS starts (`scripts/gcs.sh`, or
    `sudo GCS_TRACKER=mpc bash scripts/gcs_service.sh install`). The row keeps the key
    `offboard_bridge`, so the Flight day card is unchanged. Its patterns name both nodes, so
    neither can be started beside the other from the dashboard;
  - `deploy_follow.sh --tracker=mpc`, and `TRACKER=mpc bash drone-dtp/scripts/bench_bringup.sh`.
- **Gates (2026-10-17):**
  - `python3 scripts/px4/mpc_tracker_sim.py` 8/8: hover in wind 1.2 cm; figure-eight 3.4 cm RMS
    against a plant with the wrong lag, a 20 ms delay and noise; far setpoint peak 0.97 m/s.
  - **New `python3 scripts/px4/mpc_tracker_node_check.py` 21/21:** the real node, ROS stubbed.
    Mutation-tested: a y sign, `π/2 − ψ`, setpoints outside Offboard, the servo touching the
    acceleration, and an engage holding the origin each fail it.
  - `scripts/gcs/test_supervisor.py` all passed.
  - **Not done:** a real-DDS sandbox like `m3_bridge_check.py`, anything with real `px4_msgs`,
    and anything on the FC.
- **Traps, don't re-derive:**
  - **`m3_gate_check.py`'s "setpoint stayed frozen" passes vacuously** against the tracker: its
    setpoint positions are NaN, and `max(0.0, nan)` is 0.0.
  - `offboard_viz.py` draws no setpoint marker in TRACK/HOLD, for the same reason.
  - PX4's `MPC_XY_VEL_*` / `MPC_XY_P` do not act on it; `MPC_TILTMAX_AIR` and `MPC_THR_HOVER` do.
  - `mpc_v_ref_max` 1.5 m/s and `mpc_a_xy_max` 2.5 m/s² were chosen for the S2 envelope. This
    branch's SUPER flies `max_vel` 3.0 / `max_acc` 2.0, so the tracker will lag SUPER above
    1.5 m/s until those are raised.
  - `deploy_follow.sh`'s survivor check matched `px4_offboard_bridge` against `ps -o comm`, which
    prints 15 characters (`px4_offboard_br`), so a surviving bridge was never counted. Both
    names are matched now.
- **Before it flies:** the integration doc §8. Measure `tau` from a ULog (the 2026-09-27 study
  says 0.15–0.19 s + 40–60 ms, against the default 0.12). Confirm props off that v1.17 takes
  acceleration-only Offboard. The first flight is hover HOLD only, with ch11 as the takeover.

## 2026-09-29 — the camera yaw servo (fix 1, fix 2): built, installed, both switches OFF, NOT flown

The plan is `docs/plan-camera-yaw-servo.md`. **The replay gate does not pass (10/18), so it is
not ready to fly as configured.** The open decision is `yaw_servo_min_range_m` (the user's call).

**What was built**
- **`super_bringup/scripts/yaw_servo.py`:** the whole algorithm, pure Python on `math`, no ROS
  and no clock. The bridge and `drone-dtp/scripts/fix1_yaw_replay.py` both import it. There is
  one copy.
- **`px4_offboard_bridge.py`:** a stage that rewrites only `yaw` and `yawspeed` of a **copy** of
  the setpoint just before it is published. It never writes into the latched `self.sp`, so in
  HOLD SUPER's frozen yaw stays the fall-back.
  - **The bridge imports `yaw_servo.py` even with the switch off.** Commit the two together, or
    a clean install gives a bridge that does not start.
  - With `yaw_servo_enable` false there is no new subscription, no new publisher, and the
    publish call is the old one. Byte-identical to the committed bridge over 12 phases (A/B on
    an isolated domain).
- **Parameters** (read-only, validated at start-up; a bad value stops the node):

  | Name | Default |
  |---|---|
  | `yaw_servo_enable` | **false** (fix 1) |
  | `yaw_servo_ff_enable` | **false** (fix 2) |
  | `yaw_servo_aim_in_hold` | true |
  | `yaw_target_topic` | `/planning/yaw_target` |
  | `yaw_servo_debug_topic` | `/bridge/yaw_servo` |
  | `yaw_servo_target_timeout` | 0.5 s |
  | `yaw_servo_max_rate` | 90°/s |
  | `yaw_servo_ff_max_rate` | 60°/s |
  | `yaw_servo_ff_max_accel` | 90°/s² |
  | `yaw_servo_min_range_m` | 2.5 m |
  | `yaw_servo_min_range_hysteresis_m` | 0.3 m |
  | `yaw_servo_stale_hold_s` | 0.5 s |
  | `yaw_servo_camera_offset_deg` | 0 |

- **`/bridge/yaw_servo`** (`Float32MultiArray`, 50 Hz, only while engaged): `[active, psi_des,
  psi_out, yawspeed, range_m, target_age_s, source]`. Source: 0 SUPER / fall-back, 1 servo,
  2 close-range hold, 3 stale hold. It is in `bagger._M51`, so the m51 and stage_b presets and
  `deploy_follow.sh` record it.

**Where the build departs from the plan, and why**
- **The feed-forward's step reset.** The plan's "a step over 5° resets the average" zeroes the
  feed-forward on every message of a fast sweep (100°/s at 15 Hz is 6.7° per message), which
  contradicts the plan's own gate. A step over 5° now counts as a jump only if no step of the
  same sign and similar size (0.4–2.5×) came in the last 0.25 s. A one-off jump or a flicker
  still gives a feed-forward of exactly 0.
- **The fall-back yawspeed.** SUPER's rate is sent only when the slew clamp did not clip on that
  tick and the heading is within 2° of SUPER's, and it is clamped to ±90°/s. The first build
  tested the 2° band alone and passed 96–126°/s in the 09-26 20:06 replay.
- **Close-range hysteresis** (0.3 m): in below 2.5 m, out at 2.8 m. Without it the source
  changed 8 times in 0.42 s at 2.49–2.50 m in the 09-26 19:51 replay.
- **A stale hold** (0.5 s, source 3): when the target becomes unusable, the heading holds its
  last aim before falling back to SUPER. The yaw target is ~250 ms old on arrival, so a brief
  dip past 0.5 s happens during a LOCKED follow; without the hold the camera swung ~20° toward
  SUPER's heading and back.

**Gates** (all re-run independently after the agents' reports)
- **`scripts/px4/yaw_servo_unit_check.py`: 139/139.** No ROS, ~10 s. It includes a 310,000-tick
  randomized soak with 0 violations.
- **`scripts/px4/m3_bridge_check.py`: 120/120, three consecutive runs on the installed node**
  (was 51). ~170 s. **Its default `BRIDGE` is the repo copy;** to test the installed node set
  `BRIDGE=~/super_ws/install/super_bringup/lib/super_bringup/px4_offboard_bridge.py`.
  - Mutation tests: a sign flip in the FLU→NED conversion fails 17 checks, no slew limit 10,
    a write-back into `self.sp` 2, the three round-two rules 1–7 each.
- **`drone-dtp/scripts/fix1_yaw_replay.py --gate`: 10/18. FAIL.** Person off-centre, p50 / p95 /
  max in degrees:

  | Bag | As flown | Fix 1 | Fix 1 + 2 |
  |---|---|---|---|
  | `stage_b_2026-09-26_200650` | 2.1 / 16.8 / 20.6 | 0.6 / 4.2 / 5.4 | 0.5 / 1.8 / 5.4 |
  | `stage_b_2026-09-26_195108` | 7.4 / 16.9 / 20.0 | 1.8 / **15.8 / 38.0** | 1.3 / **14.2 / 38.0** |
  | `stage_b_2026-09-21_195658_salvaged` | 4.6 / 16.7 / 20.6 | 1.1 / 6.3 / 8.8 | 0.8 / 2.1 / 3.2 |

  - **The two 19:51 failures are the close-range rule.** 2.5 m is the follower's standoff
    floor, so the rule engages in a normal close follow. The heading froze for 2.58 s and the
    release was a 0.48 s turn at the 90°/s limit. With `min_range_m` 2.0, 1.5 or 0 the bag
    gives 1.8 / 8.4 / 11.2 and 1.3 / 5.7 / 7.5, and all six off-centre criteria pass.
  - **The bags do not support the rule's premise down to 2.16 m.** Below 2.5 m the person's
    bearing moved at most 27°/s, and 60°/s anywhere, against the 90°/s limit. Below 2 m there
    is no data. `--study` prints the table.
  - **The six "≤ 15°/s while fresh" failures are the criterion.** Per 20 ms tick it measures
    the 15 Hz staircase (a 1.5° step reads as 75°/s). Over 0.2 s the servo follows the
    person's own bearing rate, which reached 29–49°/s. The criterion needs restating.
  - **The 90°/s limit holds:** commanded max 90.000°/s, modelled vehicle max 89.9°/s.
- **Bag bench of the installed node: `scripts/px4/yaw_servo_bag_bench.py`.** It plays a bag's
  four input topics into the real node in real time (isolated domain 96, topics under
  `/soak/`), with the yaw target re-stamped so its age on arrival is as recorded. Three bags ×
  off / fix 1 / fix 1 + 2:
  - heading slew 0 violations, worst 90.004°/s; feed-forward slew 0 violations;
  - position, velocity, acceleration and jerk byte-identical to the switch-off run;
  - the node agrees with the offline replay: off-centre p95 within 0.31°, the source on
    99.56–100 % of ticks;
  - tick p99 ≤ 20.6 ms with and without the servo; the takeover stops the stream and the
    debug topic, 0 messages after the report;
  - sign 40/40, 162/165, 146/146. The 3 exceptions are the catch-up after the 19:51
    close-range release, scored against the heading as flown.
- **Soak of the installed node: `scripts/px4/yaw_servo_soak.py`, 45 min, seed 672573.** Servo
  bridge (fix 1 + 2) beside a switch-off control, 39 engagements, 118,473 setpoints.
  - Every servo invariant: 0 violations (slew, feed-forward, stale hold ≤ 0.5 s, close range
    in < 2.5 m and out ≥ 2.8 m, no setpoint or debug outside Offboard, first engaged setpoint
    within one slew step of the heading, heartbeat gap ≤ 37 ms).
  - RSS 66.7 → 68.4 MB, flat after 330 s. CPU 16.2 % of a core against 13.2 % for the control.
    Tick p99 20.2 ms in the first and the last 5 min. No traceback.
  - **One finding, not the servo's: 5 setpoints above 90°/s (92.97).** In IDLE in disarmed
    Offboard with `xy_valid` false, the latch cannot run and the bridge sends SUPER's stored
    command with its raw `yawspeed`. The switch-off bridge sent the same setpoints. It is the
    documented 2026-09-27 behaviour, and PX4 is disarmed then. Not changed.
- **Props off on the aircraft (plan §4.4): NOT done.** It needs the pilot and a walker.

**Facts not to re-derive**
- **The yaw target is 236–256 ms old (p50) when it arrives,** because its stamp is the
  perception stamp. Only ~250 ms of the 0.5 s freshness budget is left. During a coast
  `follow_planner` republishes the same stamp, so the target goes stale ~0.25 s into it.
- **The servo alone does not prevent the swing after a loss.** In the 09-26 20:06 replay SUPER
  swung at 88.6°/s, under the limit, and the servo followed it (84.6° against 84.2° as flown).
  Preventing it depends on the 2026-09-26 yaw hold in `follow_planner`, which is in none of
  these bags.
- **`follow_planner` stamps the yaw target `world`** and publishes it BEST_EFFORT / KEEP_LAST(1)
  / VOLATILE. The servo accepts `world` and `lidar_init` only.
- **The old replay's fix 1 + 2 numbers used a centred average** (~0.1 s of look-ahead). Made
  causal, the p95 did not change measurably.
- **Commanded yawspeed is bounded, the vehicle's rate is not quite:** PX4's rate is P·error +
  yawspeed.

## ⚠️ 2026-09-21 — the offboard bridge's IDLE stream is mixed into Altitude and Position mode

**Found in the retune's Flight 2 log** (`/fs/microsd/log/2026-09-21/19_17_02.ulg`,
`~/px4_logs/2026-09-21_19_18_48.ulg`). **Fixed on the bench 2026-09-28 (the user's decision),
installed, NOT flown: see "2026-09-28 status" at the end of this section.** The bullets below are
the 2026-09-21 finding as written.

- **What happens.** When not engaged, `px4_offboard_bridge.py` sits in `IDLE`, and it still
  publishes `/fmu/in/trajectory_setpoint` at `publish_rate` (50 Hz): a position-only setpoint
  latched to the live estimate (`on_tick` → `_latch_current_position`). uXRCE-DDS writes that
  into uORB `trajectory_setpoint`, **the same topic PX4's own flight task writes in Position and
  Altitude mode.** `mc_pos_control` runs on whichever sample arrived last.
- **What it does to the aircraft.** A bridge sample means "hold here, velocity zero". While the
  pilot flies the aircraft somewhere, every such sample commands a hard brake.
  - In Flight 2's Altitude mode, 27 % of attitude-setpoint samples jumped > 3° from their
    neighbours, 11 % > 10°, max 45°.
  - At 55.7 s the stick held +18° of pitch; the interleaved samples said −23°, and the aircraft
    pitched forward from 17° to ~0°.
  - The thrust setpoint toggled by up to 0.15. The descent's ±0.15 m/s vertical-speed
    oscillation is probably this too.
- **Evidence it is the bridge.** In Stabilized, where PX4 runs no flight task, 65 of 66 logged
  `trajectory_setpoint` samples are position-only (velocity/accel NaN) and trail the vehicle's
  own position. `offboard_control_mode` is logged from 0.06 s to the end. Nothing else in this
  repo publishes either.
- **Where it matters:**
  - **Stabilized is immune:** `mc_pos_control` does not run there.
  - **Position mode is affected more mildly:** the task's own setpoint is also a hold near the
    vehicle. In Flight 2's 6 s of Position, the tilt setpoint jumped ≤ 3°, and the thrust
    setpoint up to 0.14.
  - **It applies to every flight with the bridge up.** That includes the M5 flights' manual
    phases and **the manual takeover from Offboard into Position or Altitude**: the bridge drops
    to IDLE and keeps publishing.
  - **Terrain following is diluted.** Under `MPC_ALT_MODE=1` the bridge's samples hold the
    current EKF height.
- **Until it is fixed:** keep the offboard bridge **stopped** for any manual flight that is not an
  Offboard sortie, the tuning flights included (`docs/px4-tuning-notes.md` §3.1 already lists it
  as not needed).
- **Candidate fix (not built).** In IDLE, publish only `offboard_control_mode`; publish
  `trajectory_setpoint` only while `vehicle_control_mode.flag_control_offboard_enabled`. In
  v1.17 Offboard availability needs only a recent `offboard_control_mode` with a mode flag set
  and a valid local position (`commander/HealthAndArmingChecks/checks/offboardCheck.cpp:44–51`). Keep
  latching internally, so the first Offboard setpoint is "here". Two things must be checked:
  - what `mc_pos_control` flies in the few ms between the mode switch and the bridge's first
    sample (the flight task's last setpoint);
  - `m3_bridge_check.py`'s IDLE re-latch test.
- **Also hit, found 2026-09-28: the abort's landing.** Every auto mode (AUTO_LAND, Hold, RTL)
  runs FlightTaskAuto, which writes `trajectory_setpoint` too (`FlightModeManager.cpp:188–193`).
  So the IDLE stream was also mixed into the mux ABORT's NAV_LAND descent and into any Hold/RTL
  failsafe.

### 2026-09-28 status — built, bench-gated, installed; NOT flown
**What was built** (the candidate fix, behind a parameter):
- `px4_offboard_bridge.py` sends `trajectory_setpoint` only while the last
  `vehicle_control_mode` it received says `flag_control_offboard_enabled`.
  - `offboard_control_mode` still goes out every tick.
  - The IDLE latch still runs every tick, so the first sample after the switch is "here".
  - On the IDLE → HOLD edge (arm-then-engage, SUPER silent) it re-latches once, so the
    frozen point is that tick's estimate.
- **It gates on the Offboard flag, not armed+Offboard.** Disarmed Offboard (engage-then-arm)
  gets the latch, as before.
- **A report that stops arriving never stops the stream.** Only a received report saying
  Offboard is off does. After 2 s of silence it logs a warning, and that is all.
- `setpoint_only_in_offboard` (default **true**); `:=false` restores the old always-on stream,
  for the bench only.
- The bridge logs each edge: "setpoint stream started (…) NED […]" and "PX4 left Offboard:
  setpoint stream stopped". The 5 s line prints `(withheld: PX4 not in Offboard)`.
- Installed: `~/super_ws/src/super_bringup` re-copied and rebuilt the same day. No running
  bridge picked it up: a bridge must be restarted to get it.

**The engage window: what PX4 flies between the switch and the bridge's first setpoint** (v1.17,
`d6f12ad`). Don't re-derive.
- **Nothing refuses or fails over.** Commander needs only the heartbeat and a valid position
  (`offboardCheck.cpp:46–61`). The offboard-loss failsafe reads only the heartbeat, which never
  stopped.
- **FlightModeManager stops publishing at the switch.** OFFBOARD matches no task →
  `switchTask(None)` (`FlightModeManager.cpp:278–279`).
- **`mc_pos_control` keeps its last `_setpoint`.** It is replaced only by a new sample (`:438`),
  and nothing times it out. What that last setpoint is depends on the mode you came from:
  - **From Position or Altitude**, position control was already on, so
    `_time_position_control_enabled` is not reset (`:401–411`). It keeps flying the flight
    task's LAST setpoint:
    - Position mode with the sticks centred: a position lock, i.e. a hold;
    - Position mode with a stick deflected: that velocity, with NaN position;
    - Altitude mode: the stick's horizontal acceleration.
  - **From Stabilized**, the old setpoint was cleared (`:408–410`). Until a sample stamped after
    the switch arrives, it flies `generateFailsafeSetpoint`: zero velocity on all axes, no
    position target (`:445–448`, `:647–684`).
- **Here the setpoint timestamp matters.** uXRCE converts ours with the timesync offset and caps
  it at "now" (`Tools/msg/templates/ucdr/msg.h.em:168–169`). A sample stamped before the switch
  is ignored, and the zero-velocity failsafe runs until a later one arrives. The bridge's first
  sample is stamped after it hears the switch, so this matters only if timesync is off by more
  than the window. The 09-26 20:06 bag shows a 780 ms timesync step, after landing.
- **How long the window is on this link:**
  - **`vehicle_control_mode` is NOT only 2 Hz.** Commander publishes it at 2 Hz **and
    immediately on a mode change** (`Commander.cpp:1931–1932`). uXRCE limits it to 50 Hz
    (`dds_topics.yaml:66–68`), and the interval subscription delays an update rather than
    dropping it (`SubscriptionInterval.cpp:41`, `uORBDeviceNode.cpp:335`; the client polls every
    ≤ 10 ms, `uxrce_dds_client.cpp:689–700`).
  - `vehicle_status_v1` is limited to 5 Hz, so it is the slower signal of the two.
  - **Measured:** all 10 Offboard edges (5 engages, 5 exits) in the five Stage B bags of 09-21
    (18:30, 18:35, 19:56) and 09-26 (19:51, 20:06) came 22–488 ms after the previous report,
    i.e. off the 2 Hz grid. They were received −13 to +3 ms from their PX4 stamp, which is the
    timesync noise.
  - The bench bridge's first setpoint came 13–22 ms after the report (4 gate runs). The
    Jetson → FC leg was measured at 19–25 ms of data age on 2026-09-02.
  - **Estimate: ~40–70 ms from the switch to our first setpoint in uORB,** against ≤ 20 ms before
    the fix. A link stall stretches it: `vehicle_local_position` had a 335 ms delivery in the
    09-26 20:06 bag.
- **Residual risk, stated plainly:**
  - For those ~40–70 ms PX4 flies the flight task's last setpoint. Engaging with the sticks
    centred makes that a hold.
  - **If the FC → Jetson direction is dead at the engage,** the bridge never hears Offboard and
    sends nothing. The heartbeat keeps Offboard alive, and PX4 keeps flying the stale task
    setpoint **indefinitely**: a hold if the sticks were centred, a constant velocity if not.
    The old bridge would have sent its last latch. The pilot's ch11 takeover is the remedy.
  - **The takeover has a mirror-image window.** The bridge sends TRACK samples for ~the same
    ~40–70 ms after the pilot's switch, interleaved with the Position task's first samples,
    where before it was forever.

**Gate: `m3_bridge_check.py` 51/51, three consecutive runs** (was 39).
- T1 and T5 were rewritten: the old ones asserted a setpoint stream in IDLE.
- 12 new checks: no setpoint when disarmed or armed in a manual mode; the heartbeat at rate;
  the first sample = the estimate for both engage orders, within 0.2 s of the report; HOLD at
  the engage point; T4 "a silent mode report keeps the stream"; T5 "a fresh Offboard-off report
  stops it, and it stays stopped while SUPER commands"; a third sandboxed bridge with
  `setpoint_only_in_offboard:=false` still streams.
- **The same gate on the pre-change bridge: 47/51.** The 4 new-contract checks fail: 75 and 50
  setpoints outside Offboard, 60 and 50 after the takeover.

**Other things that assumed the IDLE stream, and what was done:**
- **Fixed:**
  - `scripts/px4/offboard_engage.py` refused to engage without `trajectory_setpoint` > 2 Hz.
    The trigger is now the heartbeat alone. After the switch it counts setpoints, and with none
    it exits 1 **without arming**.
  - `m51_one_graph_check.py`: the unmodified scorer failed a healthy new bridge. On a synthetic
    bag it gave A 22.4 Hz < 40 and T 0/3 won.
    - A and T now read the heartbeat, falling back to `trajectory_setpoint` if the heartbeat was
      not recorded.
    - T gained a line: "bridge setpoints stop ≤ 0.25 s after each takeover".
    - Synthetic new-bridge bag: 7/7. The 09-26 flight 1 bag (old bridge): the takeover WINs, and
      the new line FAILs with 88 setpoints mixed into POSCTL.
  - `m3_gate_check.py`: engaged after arming, the drift baseline was `None`, so "setpoint stayed
    frozen" passed vacuously. It now takes the first setpoint after the engage.
  - `offboard_viz.py` would have kept drawing the last Offboard point after a takeover, with the
    line growing red. A setpoint older than 0.5 s is now DELETEd (sandbox-tested).
  - GCS: a heartbeat chip was added next to the setpoint chip (`telemetry.py`; tested on
    isolated domain 94). The Flight day bridge step's text was updated, and so were the gate
    label (51/51), `docs/m5.1-one-graph-runbook.md` ("sp_out 0 … until ch11"),
    `docs/flight-test-plan.md` and `docs/px4-velocity-tuning-plan.md`.
- **Unaffected** (they read publisher presence, not messages):
  - `stage_a_preflight.py`;
  - `goal_forward.py`;
  - `desk_check.sh` (no ROS);
  - the bag presets (m51 and stage_b record the heartbeat).
- **Not changed, reported:**
  - `drone-dtp/scripts/bench_bringup.sh` step 2 prints the `trajectory_setpoint` rate. It will
    now print ~0 Hz, which is informational, not a gate.
  - `docs/m6.3-handheld-runbook.md` and `docs/px4-tuning-notes.md` still say to keep the
    bridge stopped for manual flights. That stays right until the props-off check below.
  - The running `super-gcs.service` still serves the old telemetry until it is restarted.

**Owed, in order:**
1. **Props off:**
   - `listener trajectory_setpoint` on the FC, in Position with the bridge up: only the task's
     samples;
   - `m3_gate_check.py` through Offboard → arm → takeover, in both engage orders;
   - `offboard_engage.py` on the bench.
2. **A manual flight with the bridge up:** the attitude setpoint no longer flips (the Flight 2
   signature).
3. **An Offboard engage in the air, sticks centred:** no lurch at the switch. The bridge log's
   "setpoint stream started" time against the ULog's `vehicle_status` switch gives the window.
4. **A takeover:** the stream stops within ~0.1 s (M5.1 scorer T).

Until 1–2 are done, the old rule stands: stop the bridge for manual and tuning flights.

### 2026-09-28 19:40–23:40 — item 1 (props off) run on the aircraft: passed, but one order was not run on hardware
Stack on the follow config, both bridges from the GCS, the pilot on the transmitter. A scratch
recorder watched both ends at once: the bridge's `/fmu/in/trajectory_setpoint` on the Jetson
against `vehicle_status`, and `listener trajectory_setpoint -n 1` on the FC every ~2.5 s. (Its
printed time is ~2 s after the listener ran, so read the samples against that.)
- **Outside Offboard nothing goes out.**
  - Stabilized, Position (armed and disarmed) and Hold: the Jetson sent 0 setpoints/s.
  - The FC's samples were all PX4's own task: finite velocity and acceleration, a position lock
    in x/y and the 0.7 m/s descent of a low throttle stick. None had the old IDLE shape
    (position only, velocity NaN).
  - In Stabilized nothing writes the topic, and its age just climbs.
  - `offboard_control_mode` was 3 ms old throughout.
- **The engage.** First setpoint 4–6 ms after the Offboard report, at the estimate. This held on
  every engage: disarmed (RC and `offboard_engage.py`) and armed.
- **The takeover.** The last setpoint was ≤ 20 ms before the report of leaving Offboard, and
  none came after.
- **Acceleration.**
  - HOLD, armed in Offboard with SUPER silent: the FC showed acceleration NaN.
  - After a GCS RC-card goal, SUPER tracked and 50/50 setpoints/s carried a finite
    acceleration. The FC showed, e.g., `acc [-1.71 -0.12 -0.88]` and `[1.46 1.32 0.40]`, all
    ≤ `max_acc` 2.0.
  - On the bench the aircraft cannot move, so SUPER keeps replanning from rest and its
    reference swings around (x from +1.98 to −0.83 m in 2.6 s). That says nothing about flight.
- **`m3_gate_check.py`, arm then engage: 9/9.** 16 s armed in Offboard, drift 0.000 m, takeover
  to POSCTL seen.
- **`offboard_engage.py`:** 0 Hz before, `ACCEPTED`, 50 Hz in Offboard. `--hold` also worked.
- **`MPC_ACC_DECOUPLE` is 1**, read live with `param show`.
- **NOT run on hardware: the gate in the engage-then-arm order.** It is covered only by
  `m3_bridge_check.py` (first sample = the estimate, both orders). See the next finding for why.
- **Leave SUPER without a goal afterwards.** A goal SUPER cannot reach on the bench leaves it
  replanning forever, or failing `PlanFromRest` once the aircraft has been carried somewhere
  else. The next Offboard engage then tracks that stale plan. Restart the stack after any
  bench goal. It was restarted at 23:41.

**⚠️ PX4 v1.17 will not arm from the transmitter in Offboard.** `Commander.cpp:599–607` denies
an arm whose reason is `stick_gesture`, `rc_switch` or `rc_button` in any non-manual mode:
"Arming denied: switch to manual mode first". So engage-then-arm needs a commanded arm
(`offboard_engage.py --arm`, or QGC). The field order is arm in Position, then ch11, and it is
the only one the transmitter can fly. The 09-02 note below ("engage-then-arm is the safer
order") does not say how that arm was made.

**Arming in Position needs the throttle at or below centre** (`Commander.cpp:574–583`,
"throttle above center"). Arm with it down, then bring it to the middle before ch11, so that
PX4's last Position setpoint is a hold and not a descent.

**`m3_gate_check.py` stopped early under-reports the LiDAR rate.** It divides the odometry count
by the requested duration, not the elapsed time. Let the window run out, or recompute the rate
from the printed message count.

## 2026-09-27 — why the aircraft overshoots SUPER's reference on a sprint (acceleration feedforward was off; on since 2026-09-27)

Read from the 2026-09-26 bags (`/fmu/in/trajectory_setpoint` vs `/fmu/out/vehicle_local_position_v1`,
NED, horizontal). **Turned on 2026-09-27 by the user's decision: bench-gated, not flown** (last
bullet).
- **What was flown.** Flight 1's engage (walker 5.3 m away): the reference peaked at 1.82 m/s. The
  aircraft fell **0.75 m behind** it while it sped up, then peaked at **2.25 m/s** and ended **0.81 m
  past** the point where the reference stopped. Range to the walker fell to 2.57 m. Flight 2's
  FOLLOW sprint (bag t 369.6) did the same: 1.26 m behind, 2.57 m/s against 1.92, 0.50 m ahead.
- **Why.** The bridge sends position + velocity and leaves acceleration NaN
  (`publish_acceleration_ff` False). With no acceleration term, PX4's stock velocity loop
  (`MPC_XY_P` 0.95, `MPC_XY_VEL_P/I/D_ACC` 1.8 / 0.4 / 0.2) makes its 2 m/s² only by falling
  behind. When the reference brakes, that stored position error and the velocity integrator keep
  pushing, so the aircraft runs faster than the reference and past it. SUPER's reference itself
  stops where it should.
- **Model (scratch, not in the repo).** A lagged double integrator (τ 0.10–0.15 s, 30–80 ms delay)
  under PX4's position and velocity law, driven by flight 1's recorded setpoint, gave 0.70–0.80 m
  behind, 0.83–1.04 m past and a 2.37–2.63 m/s peak. That matches the flight to within ~0.2 m and is
  a little pessimistic. With SUPER's `pos_cmd` acceleration fed forward: 0.21 behind, 0.22 past,
  2.07 peak. Projected for a rest-to-rest move at `max_acc` 2.0, the overshoot past the goal was:
  - 0.8–0.9 m at 2 m/s, 1.2 m at 3 m/s and 1.6 m at 4 m/s without the feedforward;
  - ~0.25 m at all three speeds with it.
- **ON since 2026-09-27 (user's decision). Bench-gated, not flown.**
  - `publish_acceleration_ff` now **defaults to True**. No launch path passes it (`run.sh` has no
    bridge; the GCS row, `deploy_follow.sh`, `bench_bringup.sh` and the runbooks all use a bare
    `ros2 run`), so all of them get it. `:=false` turns it off.
  - **It is sent only in TRACK.** IDLE and HOLD send NaN. A real IDLE leak was closed at the same
    time: with no valid estimate, `_latch_current_position` keeps the old setpoint, and that can be
    a SUPER command stored since the last tick. It used to go out with its acceleration; now the
    acceleration is NaN. Its position and velocity still go out, as before.
  - **No clamp was added.** Across 22,781 `pos_cmd` in the 09-26 and 09-21 bags there were 0
    non-finite accelerations, the max was 2.05 m/s², and the largest step between consecutive
    commands was 0.34. Every silence long enough to reach HOLD began at rest (|a| = 0). The bridge
    already sends all three axes NaN if any is non-finite, which PX4's `_inputValid` needs.
  - `m3_bridge_check.py` is now **39/39**: 7 new acceleration checks in T2, T4, T5 and T7, plus a
    second sandboxed bridge with `:=false`.
  - **Still owed:** a short Offboard move before a follow. The props-off look at `listener
    trajectory_setpoint` on the FC was done on 2026-09-28: acceleration is finite in TRACK and
    NaN in HOLD (see "2026-09-28 19:40–23:40" above).
  - **PX4 v1.17 facts.** `_acc_sp` = our feedforward + the velocity PID. `MPC_ACC_DECOUPLE` 1 keeps
    z out of the tilt. z scales thrust as `a_z·hover/g − hover`, so SUPER's ±2 m/s² is ±20 % of
    hover thrust. PX4 NaNs z itself during the takeoff ramp. An upward z counts as `want_takeoff`
    when armed in Offboard.

## The package

- `super_bringup/` — the ROS 2 package. `launch/m1_super_lidar.launch.py` is the M1 stack
  (FAST-LIO with odom remap + `world→lidar_init` TF + SUPER `fsm_node`, **no simulator**);
  `scripts/px4_odom_bridge.py` is the M2 odometry bridge; `scripts/px4_offboard_bridge.py` is the
  M3 trajectory bridge; `scripts/offboard_viz.py` republishes the Offboard setpoint vs EKF2's
  estimate as plain `MarkerArray` on `/viz/offboard` (NED→FLU, `world` frame) **so the laptop
  RViz can show the hold without having `px4_msgs` built** — that is the whole point, the laptop
  stays build-free. Used in flight-test phase P5. **`setup.sh` COPIES this directory into `~/super_ws/src/` — it is not a
  symlink.** Editing the repo does nothing until you re-copy and rebuild:
  `rm -rf ~/super_ws/src/super_bringup && cp -r super_bringup ~/super_ws/src/ && (cd ~/super_ws &&
  colcon build --packages-select super_bringup --symlink-install)`.

**Repo bug fixed 2026-09-02:** `super_bringup/scripts/px4_odom_bridge.py` was committed
**mode 100644**, so `ros2 run super_bringup px4_odom_bridge.py` fails with `No executable found`
on any clean `setup.sh` install. Fixed via `git update-index --chmod=+x`.

## M3 status

- **M3 (controller bridge): ✅ GATE SIGNED OFF 2026-09-02, props off, 7/7.**
  `scripts/px4/m3_gate_check.py` (a passive recorder — commands nothing, cannot arm) over 210 s /
  10 293 samples with the pilot driving the RC: **armed in Offboard 34 s** across three
  engagements; **setpoint frozen, max drift 0.013 m**; **manual takeover verified twice**
  (Offboard → Stabilized, ±0.5 s — `vehicle_control_mode` is 2 Hz, so that is the resolution;
  **corrected 2026-09-28:** it is also published on every mode change, so edges arrive within
  ~15 ms, see the 2026-09-28 status above);
  0 invalid position samples; never dead-reckoning; no EKF resets.
  **It took `RC_MAP_OFFB_SW = 11`.** The first attempt failed because there was **no Offboard
  entry among the six `COM_FLTMODE*` slots** (1=Stabilized, 4=Position, 6=Altitude; 2/3/5 unset) —
  the pilot cycled Stabilized/Position/Altitude and never reached Offboard. A dedicated
  two-position Offboard switch is better than spending a mode slot: RC **engage and takeover on
  one channel**, costing none of the three existing modes. Check it with
  `listener manual_control_switches` → `offboard_switch` 3=OFF, 1=ON, **0=NOT MAPPED**.
  **Two behaviours that look alarming and are correct:** (a) motors spin to ~`MPC_THR_HOVER`
  (0.41) the instant you arm in Offboard — a position setpoint hands PX4 the throttle and hover
  thrust is its baseline; the altitude error was under **7 mm** the whole run, so it is pure
  feedforward, not a wind-up. (b) PX4 self-disarms after ~10 s — `COM_DISARM_PRFLT`, because with
  props off it never detects a takeoff. Also: **Offboard can be entered disarmed** (held 12.8 s
  that way), and engage-then-arm is the safer order.
  **Bridge, built and verified earlier the same day:** The SITL-era `px4_bridge.py` was never found anywhere on this machine and was
  **rewritten from scratch** as `super_bringup/scripts/px4_offboard_bridge.py` (named to pair with
  `px4_odom_bridge.py`). Verified in three layers, none of which needed the drone to move:
  **bench gate 16/16, now 32/32 with the G0.2 odom-bridge cases** (`scripts/px4/m3_bridge_check.py` runs the real node in a sandbox with every
  topic remapped under `/m3_check/`); **end-to-end into PX4's own uORB** (`listener
  trajectory_setpoint` / `listener offboard_control_mode` on the FC show our data 19-25 ms old,
  `Payload rx` 1154 → **5140 B/s**, `offboard_control_signal_lost: False`); and **against the real
  planner** — with a `/goal_pose` set, **910 of 913** emitted setpoints were exactly a real SUPER
  command under `(x,−y,−z)`/`ψ→−ψ`, worst residual **1.18e-07**.
  **`offboard_engage.py` engaged Offboard on hardware: `ACCEPTED`, `nav_state = OFFBOARD`.**
  (Historical: arming was blocked until a receiver was bound — `can_arm_mode_flags: 0`,
  `arming_check_error_flags` = `remote_control` + `global_position_estimate`. Resolved.)
  **Corrected, don't repeat it:** these docs said for months that PX4 refuses **Offboard** without
  RC or `COM_RCL_EXCEPT` bit 2. False — measured with `COM_RCL_EXCEPT=0` and no RC, the mode switch
  was accepted. RC gates **arming**, not the mode switch. See `docs/fact-offboard-rc-arming.md`.
  **Three traps, all documented in `docs/ImplementationDoc.md` §M3 and all handled in the node:**
  (1) The yaw conversion is **`ψ→-ψ`, NOT `π/2−ψ`** — SUPER plans in FAST-LIO's `lidar_init`, which
  is not ENU, and EKF2's local frame *is that same frame flipped*, because `px4_odom_bridge`
  declares `POSE_FRAME_FRD` and EKF2 adopts it wholesale. They share a heading, so there is no 90°
  rotation; `π/2−ψ` would put a constant 90° error into every setpoint. (Bench gate T2 asserts both
  the right answer *and* that it isn't the `π/2` one.)
  (2) The bridge must **not** publish `/lidar_slam/odom` — FAST-LIO already owns it, and a second
  publisher gives SUPER an interleaved mix of two estimates. Verify
  `ros2 topic info /lidar_slam/odom` shows `Publisher count: 1`.
  (3) **`/planning/pos_cmd` is BEST_EFFORT and it goes silent on purpose.** A default (RELIABLE)
  rclpy subscription never matches and never errors — the node just receives nothing while
  `ros2 topic echo` works fine. And SUPER publishes *only* in `FOLLOW_TRAJ`/`EMER_STOP`, so the
  topic is completely silent in `INIT`/`WAIT_GOAL`/`GENERATE_TRAJ` — **including every time it
  reaches a goal**. A bridge that forwarded 1:1 would drop Offboard mid-flight the first time the
  drone arrived somewhere. Hence the node's own 50 Hz timer and its IDLE/TRACK/HOLD states.
  See `docs/fact-super-cmd-stream.md`.
  **Cost, measured:** 34% of one core / 68.7 MB RSS with SUPER at 92 Hz (vs 3% / 67.8 MB for the
  odom bridge at 10 Hz) — that's `rclpy`/DDS overhead at ~150 msg/s, not the arithmetic.
  `publish_rate` is a parameter if M5's CPU budget needs it back.

## Facts not to re-derive

- **`px4_odom_bridge` publishes FRD, not NED, and omits velocity on purpose.** Both are deliberate
  and documented in `docs/ImplementationDoc.md` §8 — don't "fix" either. Consequence:
  `EKF2_EV_CTRL=11`, not 15.

- **Do not go looking for `px4_bridge.py` again.** It was never on this machine, it was not
  recovered, and it has been rewritten as `super_bringup/scripts/px4_offboard_bridge.py`. The
  SITL-era spec it came from was wrong about the frame anyway (see the M3 bullet's trap 1).
- **Run `python3 scripts/px4/m3_bridge_check.py` after any change to either bridge.** It
  sandboxes the real nodes (offboard T1–T7, odom-bridge divergence guard T8–T14, the yaw servo
  Y; 120 checks since the yaw servo of 2026-09-29, 51 from the Offboard-only setpoint cases of
  2026-09-28, 39 from the acceleration-feedforward cases of 2026-09-27, 32 before), needs no
  flight controller, cannot arm anything, and takes ~170 s (~50 s before the yaw servo).
  It is the thing standing between a one-character sign slip and a spiral.
  `scripts/px4/m3_gate_check.py` is the hardware counterpart — passive, 40 s by default, re-run it
  after any change to frames, mounting or EKF2 aiding.
- **Three things M2/M3 established that the next step depends on — do not re-derive:**
  (1) **Offboard yaw works.** v1.17 has no Offboard FlightTask, so `heading_good_for_control:
  False` (from `POSE_FRAME_FRD`) does *not* affect it. See `docs/fact-ev-frd-yaw-align.md`.
  (2) EKF2 is holding position on LiDAR alone, verified tracking hand motion with correct signs on
  all three axes. Re-run `scripts/px4/m2_gate_check.py` if anything about the frames changes.
  (3) **PX4 accepts the Offboard mode switch with no RC at all.** Only arming is gated on RC.
  See `docs/fact-offboard-rc-arming.md`, which also has the reusable "why won't it arm" method.

The odom bridge's `DIVERGED` latch and the pre-arm estimator preflight are described with their cause (G0.2) in `fast-lio-and-lidar.md`.
