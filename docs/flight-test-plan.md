# Flight test plan — first flights on LiDAR position

**Written 2026-09-02, for the next session.** Everything through M3 is signed off, and
**nothing has flown or even moved.** This plan turns that into data in the smallest safe
steps, in an order where each phase de-risks the next.

Read [`ImplementationDoc.md`](ImplementationDoc.md) §M2/§M3 for what is already proven.
The abort ladder in §5 is the part to memorise before you leave the bench.

**Revised 2026-09-12 — this plan is now Gate 0 of [`m5-m6-plan.md`](m5-m6-plan.md), which gives
P1/P3/P4/P5 numeric pass thresholds (§2.2) and dates, and adds two blockers to clear before any
of them.**
- **Hardfaults (updated 2026-09-13).** 22 field crashes of the `uxrce_dds_client` serial-driver
  fault. Reproduced on demand. `CONFIG_UART7_RXDMA=n` was flashed and **failed**. The client-side
  no-close-on-reconnect fix is flashed and passed **30/30** real session losses. The 14 h soak and
  the ≥ 500-cycle gate are **not done**, so treat an XRCE session loss in the air as a possible FC
  reset until they are. **Never run the 3 Mbaud flood test before a flight** — it wedges the FC on
  any firmware.
- **A diverged estimate armed and accepted — root-caused and guarded 2026-09-12 (G0.2).** In
  `~/dtp_datasets/20260912_2006_stageA` the aircraft was armed twice while FAST-LIO was
  **3.74 km** wrong (x −3340, y −1672, z −86.7 m), and EKF2 called it valid. Cause: a second Livox
  driver (from `dtp_system.launch.py`) took the MID360's stream, FAST-LIO got IMU and no scans for
  225 s, and it came back diverged but steady. Reproduced on the bench. Full write-up:
  [`fact-fastlio-divergence-2026-09-12.md`](fact-fastlio-divergence-2026-09-12.md).
  **Now, before EVERY arm: `python3 scripts/px4/estimator_preflight.py` must print `GO`** (60 s,
  §1a). `px4_odom_bridge` also refuses to forward a jumped, stalled or far-from-origin estimate
  and says so on `/px4_odom_bridge/status`.

**Revised 2026-09-04, after the first P5 attempt — which is VOID: the LiDAR stack had been shut
down 78 s before the recording and the aircraft hovered on optical flow. Read
[P5 — attempt 1](#p5--attempt-1-2026-09-04-void--the-lidar-chain-was-not-running); the tooling
now refuses to record that run.**

**Revised 2026-09-03, after the first P3 attempt (2026-09-02) — which did not pass.** Read
[P3 — attempt 1](#p3--attempt-1-2026-09-02-not-passed) before flying it again: the aircraft
never left the ground and EKF2 reset its position nine times. Two verification habits this
document relied on turned out to be wrong and are corrected in §1a. (The six Jetson reboots that
session were the operator powering the board down — **not** a brownout. Resolved 2026-09-03.)

---

## 0. What is actually unproven (why the order below is what it is)

| Unknown | Bites when | First phase that tests it |
|---|---|---|
| **MID360→FC yaw mounting offset — never measured** | Under *any* position control. Wrong ⇒ the drone accelerates off-axis: toilet-bowl or spiral | P1 (mechanical), P3 (hover is the real test) |
| Does the airframe fly on LiDAR position at all? | First hover | P3 |
| Odometry latency **on a moving rig** | Under load — the fix left the cloud queue at depth 2, so stale scans are *dropped*, and a bench never generates that load | P4, recorded during P3 |
| Bridge `TRACK` path with a vehicle that can move | First SUPER trajectory | P6 |
| `uxrce_dds_client` FC hardfault | On XRCE session loss — an FC **reset**, i.e. a fall | Monitored every phase (§6) |
| Battery current accuracy | Endurance / low-battery failsafe | Not tested here — keep flights short (§6) |

**The yaw offset is the one that matters most, and no script can measure it.** EKF2's yaw is
*defined* by the yaw we send it, so the two agree by construction however the LiDAR is bolted
on. It is a mechanical check plus a behavioural one (P3).

---

## 1. Phase 0 — Bench, before you leave the desk

```bash
# 1. nothing regressed in the bridge (no FC needed, ~15 s)
python3 scripts/px4/m3_bridge_check.py                 # expect 120/120 (T1-T14 incl. the G0.2 odom guard, Ya-Yn + Y2 the yaw servo; ~3 min)

# 2. baseline the crash dumps -- the card was cleared 2026-09-02, so ANY new
#    fault_*.log means the FC hardfaulted since, and you want to know before flying
python3 scripts/px4/clear_fault_logs.py                # read-only; expect "Nothing to clear"

# 3. params are still what we think
python3 scripts/px4/probe_params.py
python3 scripts/px4/ekf2_profile.py --show             # expect vision-only
```

Expected: `EKF2_EV_CTRL=11`, `EKF2_HGT_REF=3`, `EKF2_GPS_CTRL=0`, `UXRCE_DDS_CFG=101`,
`SER_TEL1_BAUD=3000000`. RC: `RC_MAP_FLTMODE=5`, `RC_MAP_OFFB_SW=11`, `RC_MAP_KILL_SW=10`,
`RC_MAP_ARM_SW=12`.

**Take a param backup before changing anything:**
`python3 scripts/px4/dump_params.py firmware/px4-params-$(date +%F).params`

### Two config changes to make before any autonomous flight (P6 only)

`super_planner/config/click_smooth_ros2.yaml` ships the **benchmark tune**, not a first-flight
tune. **Set 2026-09-04** in both `src/` and `install/`, and raise only after a trajectory looks
boring:

| key | upstream | now | why |
|---|---|---|---|
| `traj_opt/boundary/max_vel` | 5.0 | **0.1 m/s** | walking pace is still 10× this |
| `traj_opt/boundary/max_acc` | 5.0 | **0.3 m/s²** | 1.0 was 10 body-lengths/s² of jump at 0.1 m/s |
| `traj_opt/boundary/max_omg` | 2.5 | **1.5 rad/s** | body-rate bound (tilt rate), secondary |
| `super_planner/yaw_dot_max` | 3.0 | **0.5 rad/s** | **the yaw knob** — 172°/s → 29°/s |

`max_vel` is the optimiser's velocity bound, so it caps the whole planned trajectory, not just
cruise. At 0.1 m/s the P6 leg below takes **~30 s** to cover 3 m — expect it to look like it is
barely moving; that is the point. The optimiser treats these as *soft* penalties (`penna_vel`,
`penna_omg`), so a small overshoot is normal and not a fault.

**`yaw_dot_max` is what makes it rotate fast**, not `max_vel`. With nothing publishing
`/planning/yaw_target`, the heading follows the velocity, so at low speed the commanded heading
chases a nearly-zero velocity vector and can slew hard. `px4_offboard_bridge` forwards `yaw_dot`
straight through as `TrajectorySetpoint.yawspeed` (negated). FC-side caps for reference:
`MPC_YAWRAUTO_MAX` 60°/s, `MC_YAWRATE_MAX` 200°/s — both were *above* the old 172°/s request.
**Caveat (2026-09-12): `yaw_dot_max` is a time-allocation hint, not a clamp.** SUPER only warns above
`yaw_dot_max + 2.0`, and a trajectory shorter than 12.6 s gets no rate bound at all. Check the
logged `yawspeed`; don't assume 29°/s. See [`fact-super-yaw-mode.md`](fact-super-yaw-mode.md).

**The click config runs `yaw_mode: 3` since 2026-09-12**, and `setup.sh` re-applies it. With no
yaw target that behaves like heading-to-velocity, except that **the goal yaw is never applied**:
an RViz arrow's direction is ignored and the GCS Rotate is refused. `yaw_mode: 2` is **not** a
fallback to try — upstream never reads the key, so 2 behaves exactly like 1.

Nothing in `px4_offboard_bridge` clamps speed or yaw rate — it forwards whatever SUPER plans — so
this file is the only limit in the chain. `scripts/setup.sh` now re-applies all four values, so a
re-clone cannot silently restore the benchmark tune.

**`robot_r` is set: 0.30 m** (measured 2026-09-14 with props on: 0.25 m hub centre to prop tip,
plus a 0.05 m margin) in both `click_smooth_ros2.yaml` and `follow_test_ros2.yaml`, `src/` and
`install/`, re-applied by `setup.sh` and carried in patch 0001. Upstream's 0.2 had SUPER planning
corridors 5 cm too tight. Re-measure after any prop or arm change.

Remember configs must land in **both** `src/` and `install/` — re-run `scripts/setup.sh` or copy
by hand, or a rebuild silently reverts them.

---

## 1a. Bring-up and shutdown — the exact cycle

Worked out on 2026-09-02/03. Getting the **order** wrong is what produces EKF position resets.

### Starting

```bash
# shell 1 -- the stack (Livox driver + FAST-LIO + SUPER fsm_node)
RVIZ=false bash scripts/run.sh
#   wait for /lidar_slam/odom at ~10 Hz before going on

# shell 2 -- the odometry bridge. THIS is the only thing that feeds the FC.
ros2 run super_bringup px4_odom_bridge.py
#   want: in=/out= climbing +50 every 5 s, rejected=0, last_input ~0.0s, guard=TRACKING

# shell 3 -- BEFORE EVERY ARM (G0.2, 2026-09-12). 60 s, aircraft still, where it takes off.
python3 scripts/px4/estimator_preflight.py
#   want: GO. Any FAIL = do not arm. Most FAILs are fixed the same way: stop the bridge,
#   restart the stack HERE, start the bridge, run it again.
```

**Why shell 3 is not optional.** On 2026-09-12 every readout above said healthy — 10 Hz odometry,
`xy_valid`/`z_valid`, `eph 0.06 m` — while FAST-LIO was 3.74 km wrong, and the aircraft was armed
on it. A diverged-then-settled LiDAR estimate is precise and self-consistent; only *where it is*
gives it away. The preflight checks that (|odom| ≤ 1.0 m), plus one Livox driver and one FAST-LIO,
rate, age, EKF2 agreement, recent EKF2 resets and the bridge's guard state
([`fact-fastlio-divergence-2026-09-12.md`](fact-fastlio-divergence-2026-09-12.md)).

**Never start a second Livox driver on top of this stack** — not `dtp_system.launch.py` with
`lidar:=true`, not a second `run.sh`. A second driver takes the MID360's stream from the first.
The DTP launch files and `run.sh` now refuse to start one; `stage_a_preflight.py` and
`estimator_preflight.py` refuse if one is running.

Before starting shell 1 the rig must be **level, in its final takeoff spot, and sitting still for
~5 s** — FAST-LIO runs a stationary IMU initialisation at startup and moving during it corrupts
the gravity vector. That startup pose becomes the origin of every frame in the system, and the
usable Z band `[-0.5, +2.9] m` is measured from it.

**The workspaces are sourced by default since 2026-09-03.** `~/.bashrc` now sources
`/opt/ros/humble` → `~/ws_livox/install` → `~/super_ws/install`, each guarded with `[ -f ... ]`
so an unbuilt workspace cannot break the shell. Any **new** terminal can run
`ros2 run super_bringup ...` directly. This does not apply to non-interactive shells
(`bash script.sh`), which is why `scripts/run.sh` still sources all three itself — **do not strip
those lines out of the scripts.**

### Stopping — reverse order, and the bridge does not die on its own

```bash
kill -INT $(pgrep -f px4_odom_bridge.py)   # 1. the bridge FIRST
#                                          # 2. then Ctrl-C the run.sh shell
```

- **The bridge is not a child of `run.sh`.** It is started with a separate `ros2 run`, so Ctrl-C
  on the stack leaves it orphaned and spinning. Found exactly this on 2026-09-03 at
  `in=2562 out=2562 rejected=0 last_input=272.9s ago`. Always stop it explicitly.
- **Stop the bridge before the stack.** If FAST-LIO restarts while the bridge is still feeding
  the FC, EKF2 watches its vision source teleport back to (0,0,0) and resets position — the same
  signature as the nine resets in P3 attempt 1. (Since 2026-09-12 the bridge's guard latches on
  that jump and resumes on the new FAST-LIO with `reset_counter` incremented, so EKF2 resets onto
  the new frame deliberately. The order is still the clean one.)
- **Never `pkill -f px4_odom_bridge`.** The pattern matches your own shell's command line and
  kills your terminal; it has done so twice in this project. `pgrep -f` only lists — that is safe.

Verify clean before restarting:

```bash
ps -eo pid,args --no-headers \
  | grep -E "fastlio_mapping|fsm_node|livox_ros_driver2_node|px4_odom_bridge" | grep -v grep
```

Empty output means clean. Do **not** use `ros2 node list` for this — a laptop RViz's discovery
cache lists nodes that have already exited.

### Moving the rig after bring-up

Stop the bridge, stop the stack, move the rig, set it down level, let it settle ~5 s, then start
the stack and the bridge again. **Since 2026-09-12 this is required before flight, not optional:**
`estimator_preflight.py` refuses an aircraft more than 1.0 m from FAST-LIO's origin, and
`px4_odom_bridge` refuses to start forwarding one. The reason is that "carried after bring-up" and
"FAST-LIO diverged" look identical from the estimate alone, and on 09-12 it was the second.

### ⚠️ `local position: 1` does NOT mean the bridge is working

Corrected 2026-09-03. With the bridge stopped, `ekf2 status` **still** reported
`local position: 1`, because optical flow and the rangefinder took over the aiding:

```
cs_ev_pos:  False          cs_opt_flow: True
cs_ev_yaw:  False          cs_rng_hgt:  True
cs_ev_hgt:  False
```

`CLAUDE.md` states the FC reports `local position: 0` with the bridge stopped. **That is stale** —
it predates the 2026-09-02 discovery that flow and range are fitted and fused. Check the vision
chain with **`cs_ev_pos: True`** (`px4_shell.py "listener estimator_status_flags"`) or with the
bridge's own `in=/out=` counters. Never with `local position`.

### `ros2 topic hz` cannot see `/lidar_slam/odom`

It prints nothing at all, indefinitely, while the topic publishes perfectly — a QoS mismatch, the
same trap as `/planning/pos_cmd`. Use instead:

```bash
ros2 topic echo --once --qos-reliability best_effort /lidar_slam/odom
```

or a subscriber built with `ReliabilityPolicy.BEST_EFFORT`. **Do not conclude the stack is dead
because `hz` is silent** — verify with `ros2 topic info` (publisher count) and an echo.

---

## 2. Phase 1 — The yaw mounting check (mechanical, 10 minutes, do it first)

Sight down the airframe's nose axis and confirm the **MID360 is square to it**. Any yaw rotation
between LiDAR and airframe is uncorrected anywhere in the stack: `EKF2_EV_POS_*` carries only
translation, PX4 has **no EV rotation parameter at all**, and `px4_odom_bridge` does only the
FLU→FRD convention flip.

- **Square (within a few degrees):** proceed.
- **Rotated by a known angle** (e.g. 90° for cable routing): that angle must go **into
  `px4_odom_bridge`** as a yaw rotation before the position quaternion is emitted. Do not fly
  until it does.
- Roll/pitch tilt is already measured and fine: **roll −1.55°, pitch +0.09°**
  (`scripts/px4/mount_tilt_check.py`, 2026-09-02). Re-run only if the mount was disturbed.

---

## 3. Phase 2 — Venue

**FAST-LIO needs geometric structure to match against. A wide-open field is the wrong venue** —
that is the usual drone advice and it is backwards here. With nothing in range the scan match is
degenerate and the estimate drifts or diverges.

- Want: walls, buildings, trees, parked vehicles, containers — **structure within ~20 m** in
  several directions. An indoor sports hall, a courtyard, or a treeline edge is ideal.
- Avoid: open field, large flat car park, anything with one flat wall and nothing else.
- **Height: the usable Z band is [−0.5, +2.9] m relative to where FAST-LIO initialised**
  (ROG-Map insets the configured bounds — see [`fact-rogmap-bounds-and-memory.md`](fact-rogmap-bounds-and-memory.md)).
  Plan to hover **1.0–1.5 m**. Above 2.9 m you leave the map.
- Start the stack with the aircraft **on the ground, level, in its final spot** — that pose is the
  origin of every frame in the system.
- People: a pilot on the sticks at all times, plus someone watching the laptop/terminal. Do not
  run this solo.

---

## 4. The phases

Each phase has a **pass** condition and an **abort** condition. Do not carry a failed phase
forward — a failure here is cheap, and the same failure in the next phase is not.

### P3 — Position-mode hover. NO Offboard, NO SUPER. ★ the important one

The single highest-value test in this plan: it validates the whole M2 chain in flight, tells you
whether the airframe flies on LiDAR position at all, and is the real test of the yaw offset.

```bash
RVIZ=false bash scripts/run.sh                       # shell 1
ros2 run super_bringup px4_odom_bridge.py            # shell 2
# do NOT start px4_offboard_bridge, and do NOT send a /goal_pose
python3 scripts/px4/m2_gate_check.py 180 /tmp/hover.csv   # shell 3, records the whole hover
```

Fly it manually: arm in **Position**, lift to ~1.2 m, hold, land. Two minutes is plenty.

Save the CSV somewhere that is **not** `/tmp` — a Jetson reboot wiped the first one.

#### P3 — attempt 1 (2026-09-02): NOT PASSED

`m2_gate_check.py` printed `==> PASS`. **That verdict was misleading** — the script only asserts
sign and correlation, and it cannot tell a hover from a hand-carry. The aircraft never left the
ground.

| window | N | E | height above start |
|---|---|---|---|
| 0–60 s | −0.03…−0.01 | +0.02…+0.03 | 0.00 m |
| 60–80 s | −0.03…**+2.41** | +0.02…**+1.07** | **0.31 m** |
| 100–180 s | +1.98…+1.99 | +0.91…+0.93 | −0.02 m |

Peak height **0.31 m** (PX4 agreed: 0.26 m) against the ~1.2 m this phase calls for, and for the
last 80 s the position sits inside a **1 cm** band — no hovering multirotor holds 1 cm. The
profile is a hand-carry: still, translated ~2.4 m N / 1.0 m E over 20 s while lifted about a
foot, set down, still. **The yaw mounting offset therefore remains unmeasured**, which was the
entire point of the phase.

Two findings, both still open:

1. **Nine EKF horizontal position resets** (`xy_reset_counter` 16 → 25), all between t=7.7 s and
   t=49.4 s **while the rig was completely stationary**, each spiking `eph` to 14.142 m
   (= 10√2, EKF2's post-reset covariance default) before reconverging to 0.14 m. Not the bridge:
   `in=out`, `rejected=0`, steady 10 Hz, no gaps — and they landed 284–326 s after bridge start,
   so not startup convergence either. **Unexplained.** This phase's pass condition forbids resets.
2. **The run's raw data was lost.** The Jetson was powered down between tests (operator, not a
   fault — confirmed 2026-09-03), and that took `/tmp/hover.csv` and the gate output with it. The
   numbers above survive only because they were read before the reboot. **Record outside `/tmp`.**

**What the run did prove, and it is worth keeping:** frames and signs are correct under real
motion — N **+0.9997**, E **+0.9994**, D **+0.9785**, all three signs `CORRECT`, **0/1775**
invalid samples, never dead-reckoning, `epv` max 0.100 m. That is the M2 gate re-passed on a
moving rig at the venue, which had never been done before. No FC hardfaults.

**Before attempt 2:** settle the Jetson power question, watch for the resets, and get the
aircraft actually airborne to ~1.2 m — a gate run that never leaves the ground proves nothing
about the offset.

- **Pass:** it holds position without wandering; no toilet-bowling or slow circling; PX4's
  `vehicle_local_position` stays valid; no EKF resets; `m2_gate_check` correlations stay >0.95
  with correct signs on all three axes.
- **Abort → land immediately:** it circles or spirals (**that is the yaw offset — go back to
  P1**); position visibly drifts; the estimate jumps.

Yaw offset behaves distinctively: the aircraft flies a **circle around** the commanded point
rather than sitting on it, and the circle gets bigger with a bigger offset.

### P4 — Moving-rig verification (recordings, not a separate flight)

Do these from the P3 hover data plus one extra run — no new flying risk.

1. `m2_gate_check.py` was already recording during P3. Check its per-axis output: this is the
   first time the M2 gate has ever seen real motion rather than a hand-waved bench rig.
2. **Re-run the frame-matched latency check** from
   [`fact-fastlio-latency.md`](fact-fastlio-latency.md) during a hover with movement. This is the
   one that has never been done under load, and the failure mode is specific: the fix set the
   cloud queue to depth 2, so under load stale scans are **dropped** rather than queued. Expect
   ~50 ms; investigate anything over ~150 ms before P5.

### P5 — Offboard hold, in the air

Hover in Position, then engage Offboard with **SUPER idle** (no `/goal_pose`), so the bridge sits
in `HOLD` with a frozen setpoint. This exercises the M3 chain in flight without SUPER commanding
any motion.

```bash
ros2 run super_bringup px4_offboard_bridge.py        # shell 4, before you take off
ros2 run super_bringup offboard_viz.py               # shell 5, so the laptop can see the hold
python3 scripts/px4/m3_gate_check.py 90 ~/p5.csv     # records the engage -- NOT /tmp
```

`m3_gate_check.py` now spends 4 s on preflight and **exits without recording** if nothing is
publishing `/lidar_slam/odom`. That is not politeness — see [P5 — attempt 1](#p5--attempt-1-2026-09-04-void--the-lidar-chain-was-not-running)
below. It prints the publisher count and `cs_ev_pos` first, so read those three lines before you
lift off; `ALLOW_NO_LIDAR=1` records a deliberate flow-only baseline anyway.

**Give yourself a window you can finish inside.** The takeover check can only see a takeover it
recorded, so 90 s and two deliberate switch-offs beats 60 s that expires mid-hover.

**Watching it from the laptop.** You are outside and the Jetson is headless, so run
`bash scripts/laptop_viewer.sh` on the laptop (see §4a). `offboard_viz` is what makes P5
visible there: a **cyan sphere** at the commanded setpoint, an **orange sphere** at EKF2's
estimate, and a line labelled with the gap in centimetres — green under 20 cm, amber to 50 cm,
red beyond. During P5 with SUPER idle the cyan sphere should be **nailed in place** with the
orange one on top of it. A growing line is exactly the failure this phase exists to catch, and
it is readable from across a field.

- **Pass:** it holds the point it was at when you engaged; setpoint drift stays small; you flip
  the channel-11 switch off and get instant control back.
- **Abort:** any drift away from the engage point → switch off, land.
- **Practise the takeover here, deliberately, at least twice**, before P6 makes the aircraft move
  on its own. Note the aircraft returns to whatever the **channel-5 mode switch** is on — set it
  to **Position** before engaging.

#### P5 — attempt 1 (2026-09-04): VOID — the LiDAR chain was not running

The gate printed 4/7 and the flight looked good. **It does not count, because FAST-LIO, the Livox
driver, SUPER and `px4_odom_bridge` had all been shut down 78 seconds before the recording
started.** EKF2 flew the whole hover on **optical flow + rangefinder**. From `~/.ros/log`:

| clock | event |
|---|---|
| 14:45:16–14:45:24 | Livox, FAST-LIO, `fsm_node`, `px4_odom_bridge` all start; bridge reaches `in=1797 out=1797 rejected=0` |
| **14:48:29.7** | `run.sh` Ctrl-C'd — `signal_handler(SIGINT/SIGTERM)` in every node's log. **The odom bridge was never restarted.** |
| 14:49:00 / 14:49:07 | `px4_offboard_bridge` and `offboard_viz` (re)started — these two only |
| **14:49:48** | `m3_gate_check.py 60 ~/air.csv` begins |

Confirmed afterwards on the FC: `cs_ev_pos: False`, `cs_ev_yaw: False`, `cs_ev_hgt: False`,
`cs_rng_hgt: True`. `EKF2_EV_CTRL` was still `11` — the parameter says "fuse vision", but nothing
was sending any.

**Why nothing caught it.** `px4_offboard_bridge` takes its setpoint from PX4's *own*
`vehicle_local_position` (it subscribes to it for safety, and IDLE tracks it), so with the LiDAR
dead it still emitted a flawless 50 Hz stream and a rock-steady frozen setpoint. Every number the
gate printed was real; none of them were about the LiDAR. This is the same class of error as
`local position: 1` in §1a — a healthy-looking readout downstream of the thing that is broken.

**What the run does prove** (worth keeping — it is the first time the aircraft has hovered at all):
the airframe took off to **0.985 m**, was flown to Position, and **held Offboard armed for 43 s**
with the setpoint frozen at `[-0.48, -1.58, -0.87]` and **0.000 m** setpoint drift. Tracking against
that frozen point: mean **0.116 m**, p95 0.209 m, max **0.236 m**; mean offset from the setpoint
**0.027 m**; no net drift over 44 s. No EKF resets, no FC hardfault (the SD card was empty
afterwards). So the M3 chain — bridge → uXRCE → PX4 → Offboard → a vehicle that can actually
move — works in the air.

**What it does not prove: anything about the LiDAR, and nothing at all about the yaw offset.**
The error vector's rotation reversed direction between windows (−31, −64, −57, **+90**, −12,
−27 °/s), i.e. a limit cycle rather than a toilet bowl — but with no vision in the loop that
result is about the flow sensor's frame, not the MID360's. **The MID360→FC yaw offset is still
unmeasured.**

The 118 invalid and 317 dead-reckoning samples are explained by the same cause and are not a
separate fault: on the ground EKF2 fuses fake position, so it is neither invalid nor
dead-reckoning; at liftoff that stops, external vision was absent, and optical flow does not fuse
until it is off the ground — leaving a gap through the climb until flow took over.

**Fixed in the tool, 2026-09-04.** `m3_gate_check.py` now (a) refuses to record when nothing
publishes `/lidar_slam/odom`, (b) adds two checks — *LiDAR odometry live* and *EKF2 fused external
vision throughout* (so 9 checks, not 7), (c) reports invalid/dead-reckoning/no-vision as **time
spans** rather than bare counts, and (d) logs `xy_valid,z_valid,dead_reck,eph,epv,ev_pos` per
sample in the CSV. It also no longer counts samples it did not record — the old script started its
counters at node start but its CSV at the first setpoint, so a couple of seconds of the counts had
no rows behind them.

**Before attempt 2:** bring the full stack up in the §1a order, confirm `cs_ev_pos: True`, and let
the preflight banner tell you the chain is live before you take off.

### 4a — Seeing any of this from the laptop

The Jetson is headless and you are away from it, so the laptop is your instrument panel.

```bash
# laptop, one-time (Ubuntu 22.04): installs ros-humble-desktop
bash scripts/laptop_setup.sh
# laptop, each session -- JETSON_IP is the Wi-Fi address, NOT 192.168.1.50
JETSON_IP=<jetson-wifi-ip> bash scripts/laptop_viewer.sh
```

**The laptop needs no colcon build, and that is deliberate.** Everything RViz displays is a
standard ROS 2 type: `/cloud_registered`, `/rog_map/*`, `/lidar_slam/odom`, `/tf`, and now
`/viz/offboard`. `px4_msgs` is *not* installed there, so the laptop cannot deserialise
`/fmu/...` topics at all — which is precisely why `offboard_viz` runs on the Jetson and
republishes plain markers.

Gotchas, all previously paid for:

- **Use the Wi-Fi IP.** `192.168.1.50` is the wired `eno1` link dedicated to the MID360 and is
  not routable from a laptop. Get the right one with `hostname -I` on the Jetson; it is DHCP
  and can change between sessions.
- **Same Wi-Fi segment.** DDS discovery is multicast. An access point with client isolation
  enabled breaks it silently — you see no topics and no error.
- **`sudo ufw disable`** on the laptop if topics do not appear.
- For the numbers rather than the picture, **SSH into the Jetson** and run
  `m3_gate_check.py` there. It needs `px4_msgs`, so it cannot run on the laptop without a
  ~12-minute build.

### P6 — First SUPER trajectory (only if P3–P5 were boring)

`max_vel: 0.1` from Phase 0 — **~30 s to cover the 3 m.** Hover in Position, engage Offboard,
then send **one short goal** ~3 m ahead, at hover height, from RViz's *2D Goal Pose* or:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 3.0, y: 0.0, z: 1.2}, orientation: {w: 1.0}}}"
```

- **Pass:** it flies a smooth 3 m and stops. That is M3 genuinely complete and M4 unblocked.
  At 0.5 m/s it is a walking-pace glide — that is the point; it should look boring.
- **Abort:** any acceleration in an unexpected direction → channel-11 switch off *immediately*.
- Expect the bridge to log `SUPER switched to its BACKUP (emergency) trajectory` occasionally —
  normal, and not a reason to abort on its own.

---

## 5. The abort ladder — memorise this

| # | Action | Effect |
|---|---|---|
| **1** | **Channel-11 Offboard switch → OFF** | Drops Offboard; you fly. **Verified twice on hardware.** Returns to whatever channel 5 is on — set it to **Position** beforehand |
| 2 | Channel-5 mode switch | Also drops Offboard. **Position** if the estimate is good; **Altitude** if the *LiDAR* is the problem |
| 3 | — | **⚠️ Moving the sticks does NOTHING.** `COM_RC_OVERRIDE = 1` has the Offboard bit clear |
| 4 | Channel-10 kill switch | Motors stop, **it falls.** Only if it is about to hurt someone |

Automatic, already configured: bridge/Jetson dies → **Position mode** after 1.0 s
(`COM_OBL_RC_ACT=0`, `COM_OF_LOSS_T=1.0`). RC lost → **Return** (`NAV_RCL_ACT=2`).

**What Position mode falls back *onto* if the Jetson dies** (measured 2026-09-03 — not what we
assumed): EKF2 keeps `local position: 1` on **optical flow + rangefinder**, with
`cs_opt_flow: True` and `cs_rng_hgt: True` while all three `cs_ev_*` flags drop to False. So the
aircraft does not lose position control the instant the Jetson goes — it holds on flow, which
drifts, has no absolute reference, and degrades badly over low-texture ground. Treat it as
**enough to get it down with**, not enough to keep flying.

---

## 6. Standing risks during every phase

- **FC hardfault.** `uxrce_dds_client` has hard-faulted **22 times in the field** (newest
  2026-09-13), and it reproduces on demand. It is `uart_close` racing `uart_datareceived` in NuttX's
  serial driver, triggered on UART close/reopen, i.e. on XRCE session loss. Disabling RX DMA did
  not fix it. The client-side fix now on the FC avoids the close on reconnect and passed 30/30 on
  the bench, but it is **not soak-proven**. There are two other signatures, once each:
  a null-pointer fault in `wq:SPI1` and a null semaphore in `logger`. None is fixed. A hard fault is an FC **reset** —
  in flight, a fall. **After every flight day, run
  `python3 scripts/px4/clear_fault_logs.py`**; a new `fault_*.log` means it happened. See
  [`../firmware/faults/README.md`](../firmware/faults/README.md).
- **ARK OS `logloader.service` looped for a day — DISABLED and cleaned up 2026-09-13.** It
  re-uploaded one ULog (`LOG0135_2026-09-12T23:03:08Z.ulg`) to the local flight-review every ~2 s
  from 2026-09-12 19:03, through reboots:
  - **21,022 copies, 344 GB**, the root disk at 95 %, ~42 % of a core;
  - a **3.9 s `/lidar_slam/odom` delivery stall** on the bench while it ran (stamps on time,
    arrival late, at two independent subscribers).

  **The cause** is a logloader bug: it fetched the log while PX4 was still writing it, then could
  never mark the finished log uploaded. **The fix:**
  - the service disabled;
  - the stuck row fixed;
  - the verified duplicates deleted (364.8 GB freed);
  - the FC's complete copy kept.

  Details: [`fact-ark-logloader-loop.md`](fact-ark-logloader-loop.md).

  **Before every flight day check both:**
  - `systemctl is-enabled logloader` says `disabled` (an ARK OS update or the ARK UI could turn it
    back on);
  - `df -h /` has ≥ 20 GB free.

  Every 09-12 Stage A flight ran with it looping.
- **Battery.** Current sensing works but is **uncalibrated** (datasheet shunt value, never checked
  against a watt meter), and `COM_LOW_BAT_ACT = 0` — warning only, no automatic action. Do not
  trust `discharged_mah`. Keep flights short and land on the voltage you would normally trust.
- **Jetson power loss — the aircraft's single point of failure, but NOT currently faulty.**
  The Jetson is the only absolute position source, so losing it in the air ends Position hold
  (see the abort ladder for what flow + range still give you). The six unclean reboots on
  2026-09-02 were **the operator powering the board down between tests** — confirmed 2026-09-03,
  not a brownout, and no longer a blocker.
  **The check is still worth keeping**, because a real brownout would look identical: an unclean
  cut leaves **zero clean-shutdown markers** — no `systemd-shutdown`, no power-off target, the
  journal just stops mid-line.
  ```bash
  journalctl --list-boots                              # unexpected boots?
  journalctl -b <id> | grep -c systemd-shutdown        # 0 = the power was cut
  ```
  If boots appear that nobody performed, treat it as a blocker again. Either way, an unclean
  power-off wipes `/tmp` — which is how attempt 1's `hover.csv` was lost, and why recordings go
  somewhere else.
- **No GPS aiding** (`EKF2_GPS_CTRL=0`). FAST-LIO is the only absolute position source. If it
  degrades there is no fallback — that is what the abort ladder is for.
- **Stabilized has degraded heading hold** on this aircraft (`POSE_FRAME_FRD` ⇒ `yaw_align=false`,
  [`fact-ev-frd-yaw-align.md`](fact-ev-frd-yaw-align.md)). Prefer **Position** or **Altitude** as
  the takeover mode.

---

## 7. Realistic scope for one session

**P0 → P3 → P4 is a full, successful session.** If the hover is clean and the recordings are good,
that converts the three biggest unknowns in this project into data.

P5 and P6 only if everything before was uneventful. There is no prize for reaching P6 on day one,
and a bad P6 is the phase that breaks the aircraft.

**Write down what happened**, pass or fail, into `docs/` — including the abort reason if you
aborted. The failure that costs this project time is not a crash, it is a session whose result
nobody recorded.
