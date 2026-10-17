# The geofence lives in `command_mux`, at the goal level (M5.2, decided and built 2026-09-14)

**The question.** The M5 plan flies every follow flight inside a 30 m × 4 m box. On 2026-09-13
it turned out that nothing on this aircraft enforced one. What should?

**Answer: a fence on the goals `command_mux` publishes, plus a land-in-place if the aircraft is
carried far outside anyway.** The user chose this (option 1 of `m5-m6-plan.md` §4 M5.2) over GPS
fusion and over "venue and pilot only". It is `drone-dtp/ros2_ws/src/command_mux/command_mux/fence.py`
with eight `fence_*` parameters in `dtp_params.yaml`, verified by `drone-dtp/scripts/fence_check.py`
(13/13 on the real node). **It has never run on the aircraft.**

## Why PX4's own fence does nothing here

`GF_MAX_HOR_DIST` and `GF_MAX_VER_DIST` are both 50 on the FC and both inert.
`src/modules/navigator/geofence.cpp:354` evaluates the distance only when
`home_global_position_valid()`, and the `vision-only` EKF2 profile (`EKF2_GPS_CTRL=0`) never
produces a global home. A GPS *is* fitted; it is switched off by configuration
(CLAUDE.md, "A GPS is fitted"). So the FC's fence cannot be turned on without changing the
estimator profile every M2/M3 gate was signed off on. That is the case against option 2 the week
of a first follow flight, not a case against it forever.

## The three options, and why this one

| Option | For | Against |
|---|---|---|
| **1. Clamp/veto in `command_mux`** (chosen) | Works under the current estimator; no FC parameter change; bench-testable like the rest of the mux; the mux already owned the z clamp as "the last line before SUPER" | Bounds where the drone is *asked* to go, not where it *is* — hence the hard boundary below |
| 2. GPS fusion (`ekf2_profile.py vision-gps`) so PX4's fence works | A real position fence with a PX4 failsafe behind it | An estimator change that re-opens M2; ties the local frame to a global origin; `vision-gps` does not manage flow/range |
| 3. Venue + pilot only | Already the S1 assumption at 0.5 m/s | Not a fence; a convention, which is what M5.2 exists to replace |

## What it is

**Frame.** A rectangle in `lidar_init` — FAST-LIO's origin at the spot the stack was started.
`x` runs along the boot heading unless `fence_yaw_deg` turns it. The estimator preflight's E5
refuses an arm more than 1 m from that origin, so "the aircraft starts at the origin" is a
checked fact, not an assumption. **The box moves with the start point and the boot heading:
walk it before arming.**

**Default.** The plan's 30 × 4 m: `x ∈ [−2, 28]`, `y ∈ [−2, 2]`. The aircraft starts 2 m from
the near end facing down the corridor, because the follower flies *behind* the subject, who
walks away from the start. **Re-cut it per venue** in `dtp_params.yaml`.

**Soft boundary = the edge.** Every goal the node publishes — FOLLOW, NAV, and the node's own
HOLD goals — is clamped `fence_margin_m` (0.5 m) inside the edge. A subject who walks out of the
box is followed to the edge and no further; the state stays FOLLOW, no veto. The margin exists
because SUPER plans a path *around* a goal, not to a point, and `robot_r` will be ~0.3 m after
it is measured. The clamp runs twice:
- where the goal is shaped, **before** the proximity veto, so the veto sees the clamped
  drone→goal segment under the same both-goals rule the descent guard already obeys;
- at the single publish point, where a bite is logged at **ERROR** — every path above already
  clamps, so a bite there means a code path skipped the fence.

**Hard boundary = the edge + `fence_breach_land_m` (3 m).** The *aircraft's* own position is
checked every tick. Past the hard boundary it raises a `FENCE` veto: HOLD, with the hold goal
clamped **inside** the box (still commanding a return), and — if it persists `abort_after_s`
with follow authority — `NAV_LAND` through `abort.py`, the same path as a sensor loss. That is
PX4's `GF_ACTION=land` reproduced at the goal level. Between the two boundaries the node warns
and writes `fence: aircraft N m outside` into `/mux/state`; there is no veto.

Why the hard boundary is not optional: a goal clamp alone is not a fence. SUPER routes *around*
obstacles and can leave the box doing so, and an estimator that drifts carries the aircraft out
while every goal reads "inside". At S1's 0.5 m/s the aircraft covers 1 m in the 2 s abort window,
so in practice crossing the hard line *is* the land decision. `fence_breach_land_m < 0` disables
the hard boundary (clamp only) and is logged at startup as a choice.

**Refusals.** The node refuses to start on a box that does not contain the origin with its
margin: the aircraft would be in breach on the first tick, and — because the mux publishes
nothing until one healthy tick — a large enough breach would leave SUPER unarmed with a log
line as the only symptom. `fence_enable:=false` is a bench flag, logged at ERROR every 5 s, and
`stage_a_preflight.py` FAILs on it in both stages.

## What it does not cover

- **Goals that never pass through `command_mux`.** An RViz *2D Goal Pose* on `/goal_pose` and
  the web GCS's remote-control card publish straight into `fsm_node`. Those are operator
  actions with a human watching; the follower's goals are the ones nobody is clicking.
- **The aircraft with no odom.** No position, no check — and that case is already the `ODOM`
  veto, which escalates on its own.
- **Altitude.** `goal_z_min_m`/`goal_z_max_m` already clamp z; the fence is horizontal only.

## Parameters (`dtp_params.yaml`, `command_mux:`)

| Parameter | Default | Meaning |
|---|---|---|
| `fence_enable` | `true` | Bench flag when false; ERROR every 5 s; preflight FAIL |
| `fence_x_min_m` / `fence_x_max_m` | −2.0 / 28.0 | Box extent along the (rotated) x axis |
| `fence_y_min_m` / `fence_y_max_m` | −2.0 / 2.0 | Box extent along the (rotated) y axis |
| `fence_yaw_deg` | 0.0 | Rotation of the box about the origin from the boot heading |
| `fence_margin_m` | 0.5 | Goals are clamped this far inside the edge |
| `fence_breach_land_m` | 3.0 | Aircraft this far outside → `FENCE` veto → land; < 0 disables |

## A side effect worth knowing: the hold re-anchor now measures from the anchor

`_hold_goal` re-anchors a latched hold when the drone ends up more than `HOLD_REANCHOR_M` (3 m)
away. It used to measure that distance to the **hold goal**. A fence-clamped hold goal can sit
metres from an aircraft outside the box, so that comparison would have re-anchored every tick
and logged at 50 Hz while the aircraft flew back in. It now measures to the **odom the hold was
latched on**, which is what the check always meant. The gate's case 5b asserts exactly one
re-anchor in the whole run — the deliberate one when the aircraft is moved 7.8 m back to the
origin.

## How it was verified

`python3 drone-dtp/scripts/fence_check.py` runs the **real** `command_mux_node` on a private
localhost-only DDS domain (89) with a 10 × 4 m test box, the PX4 command topic remapped under
`/fence_check/`, fake odom/targets/sensors_ok, and the bench flag so the abort can fire:

| Case | Result |
|---|---|
| NAV goal inside passes untouched | goal (5.0, 0.0, 1.5), `NAV\|OK` |
| NAV goal (20, 3) outside | clamped to (7.5, 1.5), state still `NAV\|OK` |
| Aircraft 2 m outside | reported in `/mux/state`, no veto, goal still inside |
| Aircraft 4 m outside | `HOLD\|FENCE`, hold goal (5.0, 1.5, 1.75) — inside |
| Hold latched | 1 distinct goal over the window; 1 re-anchor in the whole run |
| Persisting with authority | `ABORT\|LAND`, `NAV_LAND` (21) observed ×6, terminal after returning inside |
| Box excluding the origin | refused at startup, exit 1, reason named |
| `fence_enable:=false` | far goal (20, 3) passes, `FENCE OFF` in state, ERROR in log |
| Publish-point re-check | never bit |

Pure tests: `test_fence.py` (10: inset edge, nearest-point sliding, Euclidean corners,
rotation round-trip, NaN reads as the worst case never "inside", refusal of a box with no room)
and `test_abort.py` (12, `FENCE` escalates with authority and not without). Package flake8 clean.

**Safe with the stack up.** The gate signals only process groups it created; the flight stack
happened to be stopped from the dashboard three minutes before the first run, which the launch
log shows as a Ctrl-C, unrelated.

## Open

- **Never run on the aircraft.** The 3 m hard boundary and the 0.5 m margin have never seen a
  moving rig; the first Stage B flight should show at least one soft clamp (walk the subject to
  the edge) before anyone trusts the hard boundary.
- **`analyze_shadow_goals.py` does not score the fence.** A count of clamped goals per bag would
  say how often the venue box actually binds.
- **The box must be set per venue and there is no tool for it.** Pace the corridor, note the
  boot heading, edit `dtp_params.yaml`, relaunch, read the preflight's box line back.
