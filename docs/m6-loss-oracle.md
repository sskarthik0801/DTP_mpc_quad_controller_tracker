# M6 — the loss oracle, the reappearance belief, and loss conditioning (as built, 2026-09-14)

**Status: built and gated on the bench and on bag replay. Nothing here has run on the aircraft,
and no M6 number in this file is a flight result.** The plan it implements is
[`m5-m6-plan.md`](m5-m6-plan.md) §5 (design decisions §5.0, rows M6.1, M6.2, M6.4, M6.5, M6.8).
The paper's contribution — classify *why* the followed person was lost against the map the
planner already builds, and condition the follower on it — now exists in code. What does not
exist yet is any evidence it works on a real person in a real forest: that is M6.3 (the handheld
dataset, a field day) and M6.7 (flights).

## What was built

| Piece | Where | Gate |
|---|---|---|
| `dtp_msgs/LossEvent` | `drone-dtp/ros2_ws/src/dtp_msgs/msg/LossEvent.msg` | builds; every field documented in the file |
| **The oracle** (M6.1 + M6.4) | `super_planner/include/super_core/loss_oracle.hpp` (header-only, in the live SUPER tree) + `fsm_ros2.hpp` glue + `fsm/config.hpp` params + the `loss_oracle:` block in `follow_test_ros2.yaml`; all in **`drone-dtp/patches/0004-super-loss-oracle.patch`**, applied by `apply_super_patch.sh` | `drone-dtp/scripts/loss_oracle_check.py` — **28/28** on the real `fsm_node` |
| **Conditioning** (M6.5) | `target_manager.py` (`loss_conditioning`, the coast, the region gate) and `follow_planner_node.hpp` (`loss_conditioning`, viewpoint + yaw); one launch argument `loss_conditioning:=` on `follower.launch.py` sets both | `test_target_manager_loss.py` **27/27** (pure); `drone-dtp/scripts/loss_conditioning_check.py` — **16/16** on the real nodes, both arms |
| **Stock baseline** (M6.8) | `target_manager` `stock_baseline` (no coast, no abstain) + `follow_planner` `stock_baseline` (goal behind the present position, CV heading, no lead) | unit-tested; a `--arm stock` in the replay |
| **Replay harness** (M6.2) | `loss_replay.py` (bag → real `fsm_node` → `events.jsonl`, `episodes.csv`), `loss_label.py` (contact sheets, one-key labelling, D1 scoring), `loss_condition_replay.py` (bag + recorded events → real `target_manager`, per arm, `--compare`), **`loss_closed_loop_replay.py`** (bag → real `fsm_node` + `target_manager` + `follow_planner` together, the oracle answering live, per arm, with per-process CPU) | run on `20260909_closedloop_run5`, below |

Everything is **off by default in code**. `follow_test_ros2.yaml` turns the oracle **on** (it only
publishes). `dtp_params.yaml` leaves `loss_conditioning: false` on both nodes — the carry-forward
arm — so the flown stack is unchanged until someone launches with `loss_conditioning:=true`.

## How the oracle works (what the code does, not what the plan said)

It lives inside `fsm_node` and reads `map_ptr_` — the same ROG-Map SUPER avoids obstacles with —
through the public query API only (`getGridType`, `getInfGridType`, the voxel `RayCaster`). It
answers every `/dtp/follow_target` with `missed_frames > 0` (target_manager COASTING) on its own
`MutuallyExclusive` callback group and publishes `/planning/loss_event` (BEST_EFFORT, like every
DTP topic).

1. **Camera pose.** Body pose from the latest odom + `camera_offset_body` (the deployed
   `lidar_imu → camera` translation, 8 cm — under a voxel) + `camera_yaw_offset_deg` (0).
2. **OUT_OF_VIEW first.** Bearing of the CTRV point from the camera axis outside ±`fov_half_deg`
   (35°, = `follow_planner`'s cone) → no rays cast.
3. **Eq. 1 over a cylinder.** 15 rays: 5 lateral offsets across ±0.25 m at 3 heights (−0.5, 0,
   +0.4 m about the reported waist point), each from the camera to the sample, **skipping the
   first 0.3 m** (the aircraft's own cells) and **stopping `r_excl` = 0.4 m short** (the person's
   own still-occupied cells are never an occluder). Heights below the map's virtual ground are
   dropped, not counted. A ray is labelled by its first non-free cell: OCC, UNK (unknown or off the
   map) or free. OCCLUDED if OCC-fraction ≥ ρ\* (0.5), else UNMAPPED if UNK-fraction ≥ ρ\*, else
   LOST.
3b. **Only heights the map can know.** Two floors are applied to the sample heights before any
   ray is cast, and a height under either is dropped (counted in `heights_dropped`, and if none
   survives the cylinder is sampled at the floor itself, with a loud once-per-10-s warning):
   the map's **virtual ground** (everything below reads OCCUPIED), and the **sensor's view
   floor** — the MID360 looks no lower than `fov_floor_deg` = 7° under its horizon, so at range
   d the map knows nothing below z_sensor − d·tan 7° (0.5 m under the sensor at 4 m, 1 m at
   8 m), and those cells are UNKNOWN for ever. Both were found on run5, below. **Precondition
   for the paper:** an occluder is classifiable only where it rises above the sensor's floor at
   its range. A trunk or a 1.8 m tube does; a chest-high cabinet 4 m from a rig at head height
   does not.
4. **The target's own shadow is not "unmapped".** Found on the gate: a person walking straight
   away leaves a corridor of UNKNOWN cells on the line of sight — returns decaying out of
   OCCUPIED, or (filter on) cells the M5.3a filter never let a ray through — and the coast point
   is behind it, so "first non-free cell" said UNMAPPED for a plain LOST. `fsm_node` keeps the
   target's SEEN positions for `trail_window_s` (6 s); an UNKNOWN cell within `trail_radius`
   (0.6 m, the filter's radius) of one is treated as free. **OCCUPIED cells are never excused** —
   a trunk the person brushed past is on the trail too. So with the filter off, a walk-away trail
   still reads OCCLUDED (the paper's documented failure); with it on, LOST.
5. **Occluder extent** (OCCLUDED only). From the blocked ray nearest the centre, sweep the bearing
   left and right in one-cell angular steps at the target's range, 3 heights per column; a
   column is free when nothing blocks it; `edge_free_columns` (3) consecutive free columns end the
   occluder → `edge_left`, `edge_right` (+ validity), at most `edge_max_m` (4 m) out.
6. **Rollouts.** K = 32 CTRV samples: speed ~ N(|v|, 0.3), turn rate ~ N(ω₀, 0.5) with ω₀ fitted
   from the forecast's heading change, heading ~ N(h₀, 0.2); 3.2 s at 0.1 s. A rollout
   re-emerges at its first point outside the occluder's angular interval **with a camera ray
   containing no OCCUPIED cell** (unknown allowed — the map behind a trunk is unknown). Clustered
   by side (+1 left of the occluder as seen from the camera); mass that never exits is
   `still_hidden_w`. Per cluster: mean point, dt median / min / max, RMS spread σ, mass.
   Seeded per event (`track_id`, `missed_frames`, stamp) so replay reproduces it.
   **Since patch 0005 (M6.4b, below): with `rollout_collision` the rollouts cannot cross an
   OCCUPIED cell — they slide round it and resume their course — and a cluster mean that lands in
   occupied cells is replaced by the cluster's medoid.**
7. **Reduction** (paper §V-E's TBD). `yaw_target_p` = the dominant cluster's mean.
   `viewpoint_p` = the candidate on the [3, 4] m annulus about it (24 azimuths × 3 radii) that
   is outside inflated occupancy, ≥ `min_standoff` (2.5 m) from the hidden person's position,
   has a free line of sight to the re-emergence point, and is nearest the drone; z = target
   z + 0.5 clamped to [0.5, 3.0], all `follow_planner`'s numbers.
   **Since 0005: with `hidden_look`, when the still-hidden mass is ≥ `hidden_look_min_w` or no
   rollout exits, `yaw_target_p` is instead the hidden rollouts' mean position at
   `hidden_look_s`, and the same viewpoint search runs about that point — a clear ray to it is
   the sidestep that looks behind the occluder. `reappear_*` are untouched.**
8. **Cost.** `classify_us` and `eval_us` in every event, and a periodic log line
   (`-- [LossOracle] N events (...) classify p50/p99, total p50/p99`).

The `LossEvent` echoes `query_p` (the point tested) and `camera_p`, carries `onset` (first answer
of a coast), and `eval_stamp` — the oracle's clock at evaluation, which is what `reappear_dt*` are
offsets from (`header.stamp` is the target's last-seen stamp, already a coast old).

## M6.4b — the belief walks round obstacles (B) and looks behind them (B2) — as built 2026-09-14

**Patch `drone-dtp/patches/0005-super-belief-around-obstacles.patch`** (applied by
`apply_super_patch.sh` after 0004; its own marker `loss_oracle_rollout_collision`; the same four
files as 0004 and nothing else; verified byte-identical to the live tree on a pristine
`2ad3419`+isnan export through the applier). The plan it was built from, with every deviation
noted at its head: [`m6-belief-around-trees-plan.md`](m6-belief-around-trees-plan.md). **Both
switches are OFF in code and ON in `follow_test_ros2.yaml`.** With them off the belief is
bit-for-bit the 0004 belief (proved below). Bench only; nothing has flown; with
`loss_conditioning: false` nothing acts on any of it.

**B — `loss_oracle/rollout_collision`.** Inside the rollout loop, each 0.1 s step is split into
sub-steps no longer than a cell. A position is *blocked* when the person's body there touches an
OCCUPIED cell of the **raw** grid: the body is sampled every half cell along its lateral diameter
(±`rollout_body_radius`, 0.25 m) and along the body radius ahead, at every height the map can know
(the same `heights` as Eq. 1). UNKNOWN is free. The person's own cells are not obstacles: an
occupied probe within `rollout_body_radius` of a trail point is excused (the trail rule's 0.6 m is
deliberately *not* used — a trunk the person brushed past must still block). A blocked sub-step
tries headings h ± k·`rollout_deflect_step_deg` (15°), the rollout's own turn side first, up to
±`rollout_deflect_max_deg` (90°), and takes the first free one **for that sub-step only** — the
intended heading (integrating the drawn turn rate) is tried again on the next sub-step, so the
rollout slides round the obstacle and resumes its course. Nothing free = stuck: the rollout stops
and counts as hidden (the periodic log line counts them). A start inside an obstacle is backed off
along −h₀ in half-cell steps up to `rollout_backoff_m` (1.0 m; logged as `rollout start … backed
off`); if nothing there is free the event's rollouts run without collision and that is logged too.
A cluster whose mean lands in occupied cells (front-of-trunk and back-of-trunk exits averaged) reports
its **medoid** — the member exit nearest the mean — as `reappear_p`; σ and dt keep every member. **No
new random draws**: the three draws per rollout happen in the same order whatever the switches.

**B2 — `loss_oracle/hidden_look`.** After clustering, if `still_hidden_w` ≥ `hidden_look_min_w`
(0.5) or there is no cluster at all, `yaw_target_p` becomes the mean position at
t = `hidden_look_s` (1.5 s) of the rollouts that never exited (a stuck one contributes where it
stopped; the medoid rule applies), and the existing viewpoint search runs about that point. The
search already demands a clear camera ray to its centre, ≥ `min_standoff` from the hidden person
and inflated-free space, nearest the drone — that *is* the sidestep. `reappear_*` are untouched,
so `target_manager`'s window and region logic see exactly what they saw before. No message change:
`yaw_target_p == reappear_p[0]` exactly is the cluster branch, `== query_p` the none branch,
anything else the look point (the `LossEvent.msg` comment says so).

**What B can and cannot do — the finding that shaped the gate.** ROG-Map holds an obstacle as the
shell of the face the sensor has *seen*: at 4.2 m an r 0.3 trunk was six cells per height, a flat
column x ∈ [3.9, 4.0) for y ∈ [−0.6, −0.2] plus a corner; its back half and its interior are
UNKNOWN, and both tangent zones (grazing rays, marginal cells) were missing at that moment. So a
rollout that gets round the edge of the lit face walks through the unseen back unopposed, and the
map cannot say otherwise. **B keeps the belief out of the *mapped* body of an obstacle** — every
re-emergence point and the yaw target ≥ body radius − one cell (0.15 m) from every occupied cell
(measured min 0.19 m; without B the dominant mean sits *on* a mapped cell, 0.03 m) — **and out of
nothing the map has never seen.** For the paper: the belief is obstacle-aware to the extent the map
is; a trunk mapped from two sides is a closed shell, a trunk mapped from one side is a wall.

**Gate — `loss_oracle_check.py` scenes O8/O9, real `fsm_node`, both switches OFF and ON in the same
map, 12 coasts per arm, plus the pairing check against the pre-0005 binary:**

| Scene | OFF (= 0004) | ON (B + B2) |
|---|---|---|
| O8 AROUND: O1's tube + an r 0.3 trunk 0.2 m behind the walker's line; person seen at (4.0, 1.2) walking −y, coasts at (4.0, 0.2) | OCCLUDED 12/12 by the tube; dominant cluster right 12/12, its mean (4.01, −0.34) **inside the trunk's mapped half, 0.03 m from an occupied cell** — the documented defect, dt 0.7 s | OCCLUDED 12/12; dominant right 12/12; every cluster mean and yaw target ≥ 0.19 m from every mapped cell, none in the mapped half; the medoid is the round-the-front exit (3.69, −0.28) or the round-the-unseen-back exit (4.22, −0.35), dt 0.8 s; still-hidden ≤ 0.19 so B2 stays quiet; a viewpoint ≥ 2.5 m from the person is still offered |
| O8b: the coast point (3.95, −0.25) *inside* the trunk's mapped face | OCCLUDED 12/12, belief at (3.95, −0.33) on the face | `rollout start … backed off` logged (4 lines), every event still carries a belief, yaw target ≥ 0.15 m from every mapped cell |
| O9 BEHIND: an r 0.5 trunk at 3 m, person seen beside it at (3.5, 0.9), coasts straight behind it at (4.2, 0) walking +x at 0.7 m/s | OCCLUDED 12/12; **still-hidden 0.62–0.84** (median 0.72); two minority clusters 0.22 / 0.16; yaw target = the minority mean (6.09, −1.29); viewpoint (2.23, −2.32) | same clusters and sides event for event; **yaw target = the look point (5.33, 0.01)** 12/12; viewpoint (1.46, −1.03) 12/12: 2.9 m from the person, off the camera–trunk line, ray ≥ 0.37 m from the trunk centre |
| PAIRING (O8, O8b, O9 with the switches OFF, new binary vs the backed-up 0004 binary) | **36/36 events identical field for field** (minus `*_us`) | — |
| O1–O7 on the follow config (switches ON) | — | all pass as before; O1 still-hidden ≤ 0.09, B2 never fires |
| O6 budget, 130 events, switches ON in every scene | — | classify p50 31 / **p99 108 µs**; full evaluation p50 310 / **p99 2.5 ms** (0004: 81 / 406 µs; the lines are ~48 grid lookups per sub-step). Budgets 2 / 10 ms. |

**Replay regression (`loss_closed_loop_replay.py`, run5, ground −2.0, both arms on the 0005
binary with the switches ON):** coasts 66 / 64, re-acquired 65 / 61, lost 1 / 3, re-acquire median
0.173 / 0.172 s, region re-locks 0 / 3, `fsm_node` **32.8 / 32.9 %** of a core (0004: 32.7 / 32.4;
table in "Replay on a real bag" below). Events OFF 20 OCCLUDED / 8 UNMAPPED / 84 LOST, ON 21 / 10 /
79 (0004 OFF: 23 / 6 / 83 — the per-run scatter of a non-deterministic replay, see M5.3). The live
oracle on the ON arm: **4 start back-offs, 94 stuck rollouts, 15 look-behind reductions** over 21
OCCLUDED events, evaluation p99 1.8 ms. Nothing moved, as it should not: run5's occluder is below
the sensor's floor and the bag has no walker behind a trunk. Outputs:
`~/dtp_datasets/20260909_closedloop_run5/loss_replay/closed_{off,on}_0005/`.

**Why the thresholds are what they are (measured before they were set, as the plan required).**
`hidden_look_min_w` 0.5 = the hidden mass is the majority. Behind an r 0.3 trunk at 1.0 m/s
(the plan's O9) only 0.31–0.59 of the rollouts stay hidden (median 0.39) — CTRV noise of σ_ω 0.5
rad/s spreads them out of a 0.6 m shadow within 2–3 s — and the cluster viewpoint already looks
behind the trunk there; r 0.5 at 1.0 m/s gives 0.47–0.72, r 0.3 at 0.6 m/s 0.44–0.78, **r 0.5 at
0.7 m/s 0.62–0.84** (O9 as gated, 0.12 of margin), O1 ≤ 0.09. `hidden_look_s` 1.5 = half the
horizon. There is **no hysteresis**: an event stream straddling 0.5 alternates between the
cluster mean and the look point at the answer rate; `follow_planner` slews the yaw either way.

**Harness additions** (`drone-dtp/scripts/loss_oracle_check.py`): scenes E (O8, O8b) and F (O9)
each run OFF then ON on config variants `belief_off.yaml` / `belief_on.yaml`; scene stamps start
at a fixed epoch (`FIXED_T0`) so the per-event seed repeats across runs and binaries; `--dump FILE`
/ `--compare FILE` is the pairing check; O8's assertions read the map back off `/rog_map/occ` and
measure distances to occupied cells rather than to the geometric trunk. `O9_V` / `O9_SEEN` / `WIDE`
are module-level so a sweep can vary them. Full run ≈ 20 min (8 `fsm_node` starts).

**Traps found building it (all bit, all documented in the code):**
- **Point probes straddle a one-cell shell.** The plan's five probes 0.25 m apart let every
  rollout through the O8 trunk on the first build (cluster mean unchanged, 12/12). Lines every
  half cell fixed it.
- **A persisting deflection is not "going round".** The plan kept the deflected heading; on a
  trunk the first free heading is ±90°, and the walker would have left along the trunk for good.
- **Cell boundaries are knife-edges.** ROG-Map's cells are floor-indexed with centres at
  (k+½)·res in this map; a coast point at (4.0, −0.2) sat on the boundary of the mapped column and
  read free, so the "start inside" case never triggered. The scene now starts at a cell centre.
- **The tangent cells of a trunk flicker.** Grazing hits compete with misses from rays that reach
  the wall behind, so the oracle's clear-ray answer and the harness's read-back a few seconds later
  can disagree by one cell at a tangent (O9: 6 of 12 viewpoint rays graze a cube corner in the
  read-back map, nearest centre 0.037 m). The gate asserts the geometric floor on every event and
  the cell-centre test on the majority.
- **A pairing reference goes stale the moment a scene moves.** Re-record it with the backed-up
  binary (`cp` it over the install for one run) — three of the reference streams were re-recorded
  this way.
- **Sub-stepping changes floating point** even when nothing is blocked, so the OFF path keeps the
  original single-step arithmetic verbatim; that is what makes the pairing exact.

**Open after M6.4b:**
- The viewpoint search's clear-ray criterion runs on the raw shell, which is sparse near tangents;
  O9's viewpoint ray passes 0.37 m from the centre of an r 0.5 trunk, i.e. 0.13 m inside the true
  trunk. An inflated-map ray or a margin in that search would settle it (0004 code, untouched here).
- The coasted CTRV point itself still walks through trunks (`target_manager`); B fixes the belief,
  not the coast.
- A hysteresis on `hidden_look_min_w` if flight logs show the reduction flapping.
- Nothing has flown; the switches have never seen a real trunk.

## Patch 0008 — the visibility field (built 2026-09-20, bench-gated, NOT applied to the live tree)

`drone-dtp/patches/0008-super-visibility-field.patch`. The oracle's viewpoint sweep (the one that
picks `viewpoint_p`), run for EVERY locked `/dtp/follow_target` — seen as well as coasting — and
published whole, not reduced to its winner: `dtp_msgs/VisibilityField` on `/planning/visibility`,
behind `loss_oracle/visibility_enable` (off in code, on in `follow_test_ros2.yaml` in the patch).
It exists for `follow_planner`'s `goal_mode nmpc`, which cannot query ROG-Map from another
process. The consumer, the design and every trap are in
[`fact-nmpc-follow-goal.md`](fact-nmpc-follow-goal.md) ("Stage 2"); what matters *here*:

- **`evaluate()` is untouched, and the `LossEvent` stream is unchanged: 30/30 events identical,
  old binary vs patched, field by field** (scenario A on each, `pair_compare` over O1/O5/O4).
  The field has its own method, `visibilityField()`, its own message and its own switch, so it and
  `loss_conditioning` stay two independent variables.
- **A seen frame now gets a field and still gets no `LossEvent`** — O7 holds. The hook is the
  seen branch of `lossOracleCallback`, after the trail update, and again for a coasting frame.
- **Per bin (24 of 15°):** `clear_r`, how far out from `standoff_near` a viewpoint stays out of
  the *inflated* occupancy; then from that viewpoint, 3 azimuths across the bin × the sample
  heights the map can know, one ray each: `vis` (free), `unk` (first non-free cell UNKNOWN).
  One ray per bin is NOT enough — a trunk's ~11° shadow is narrower than a bin, and a
  centre-only field put it 7° out.
- **One rule `evaluate()` does not have:** an UNKNOWN first cell that the *inflated* map calls
  occupied counts as blocked. ROG-Map holds a trunk as a shell of its lit face, so a ray from
  any other viewpoint grazes into the unseen side and reads UNKNOWN; without the rule a tube dead
  between viewpoint and person read 0.56 blocked. It is deliberately confined to the field.
- **No stand-in height.** Where `evaluate()` samples at the floor when every height is dropped,
  the field reports `heights_used` 0 and the consumer treats it as absent — "cannot know" must
  not steer the goal.
- **What it can know:** occlusion near where the aircraft has looked from. An open scene reads
  vis 0.19 / unk 0.81: most candidate viewpoints' rays start in unobserved space.
- **Cost:** 216 rays, eval p50 ~400 µs, p99 530–730 µs (budget 1.5 ms), ~8 Hz.
- **Gate: `python3 drone-dtp/scripts/visibility_field_check.py` — 13/13**, domain 88 (never with
  `loss_oracle_check` or `target_filter_check`). For a scratch build pass `--super-cfg-dir`:
  `fsm_node` resolves `config_name` against the source dir it was compiled from.
- **Trap:** `loss_oracle_check.py --compare` pairs only its M6.4b scenes (`PAIR_KEYS`), so
  `--only A --compare FILE` prints "0 matched" and FAILS. That is its scope, not a mismatch:
  `--dump` both binaries and call `pair_compare` on the two files.

## How conditioning works

One parameter, `loss_conditioning`, on both `target_manager` and `follow_planner_node`, set
together by `ros2 launch dtp_bringup follower.launch.py loss_conditioning:=true` (or through
`dtp_system.launch.py`, which passes it down). `false` is today's behaviour and the paper's
**carry-forward arm**; LossEvents are still recorded in the bag.

| Class | target_manager | follow_planner |
|---|---|---|
| **OCCLUDED** | coast to **max(coast_timeout, min(occluded_coast_cap_s = 3.0, t_max + δ))**, the polyline extended at constant velocity; inside the belief's window [t_min − δ, t_max + δ] (δ = 0.5 s) the re-acquisition gate is the **region** (radius max(0.5 m, 2σ)) about the dominant re-emergence point, not the band; outside the window the band. ABSTAIN unchanged. The window follows the **newest** event and needs a **fresh** one (≤ 0.5 s old): an oracle that stops answering drops the coast back to the plain timeout | goal = the oracle's `viewpoint_p` (floor and z-clamp still enforced here), yaw target = the re-emergence point, **stamped now** |
| **LOST / UNMAPPED** | end the coast once the verdict has stood **`lost_confirm_s` = 0.3 s** unbroken (an OCCLUDED answer resets it) | nothing — the target goes stale, the mux holds and climbs 0.75 m (`hold_climb_m`), which is the plan's "search" |
| **OUT_OF_VIEW** | today's coast and band | yaw target = the CTRV point, stamped now |

**Two deviations from the plan's table, both deliberate and pre-registered here:**
- *LOST does not end the coast "immediately".* On `run5` the recorded lock coasted **48 times in
  132 s** and re-acquired in 0.09–0.38 s — and the oracle reads **38 of those 48 as LOST with a
  clear line of sight**, i.e. detector misses of one or two frames. Ending the coast on the first
  LOST answer would turn each into an operator re-designation and make the conditioned arm
  unflyable for a reason unrelated to the thesis. 0.3 s is 2–3 frames at 8 Hz and still ~4× sooner
  than the carry-forward coast.
- *The OCCLUDED coast never ends sooner than the carry-forward coast* (floored at
  `coast_timeout_s`), so an optimistic belief cannot cost a re-acquisition the baseline would
  have made.

**The yaw-stamp rule.** `follow_planner` republishes the *perception* stamp on `/planning/yaw_target`
so SUPER's 0.5 s gate measures real staleness. Under conditioning, only while the target is
coasting and the oracle has a fresh verdict, the yaw target is a *predicted* point stamped `now` —
pointing the camera at the prediction is the whole point of the arm. The exposure is bounded by
the coast cap and `loss_event_timeout_s` (1.0 s), and it is the conditioned arm only.

## Gates and what they showed

**`loss_oracle_check.py` — 28/28, real `fsm_node`, synthetic MID360 scans** (same harness as
`target_filter_check.py`):

| Scene | Verdict | Detail (last event) |
|---|---|---|
| O1 person steps behind a 0.4 m tube 3 m off the nose, walking right | OCCLUDED 12/12, blocked 1.00 | occluder on the near face (x 2.85), edges at y ±0.15, re-emergence on the right 91–100 % mass, dt ≈ 0.4–0.5 s, still-hidden ≤ 0.09, viewpoint offered ≥ 2.5 m from the person |
| O5 CTRV point 60° off the nose | OUT_OF_VIEW 6/6, 0 rays | |
| O4 person vanishes in the open (their cells still occupied) | LOST 12/12, blocked 0.00 | `r_excl` keeps their own cells out |
| O2 scan never covered the bearing | UNMAPPED 12/12, unknown 1.00 | |
| O3 walk straight away, filter OFF | OCCLUDED 8/8 on the trail (the documented false verdict) | before the shadow rule it read UNMAPPED |
| O3 walk straight away, filter ON | LOST 8/8 | the trail is never written |
| O6 58 events | classify p50 35 µs / **p99 81 µs**; full evaluation p50 60 µs / **p99 406 µs** | budgets: 2 ms / 10 ms |

**`loss_conditioning_check.py` — 16/16, real `target_manager` + `follow_planner`, this script
playing the oracle, both arms from fresh nodes:** S1 LOST verdicts — OFF coasts 1.26 s to the
forecast's end, ON goes LOST at 0.38 s naming the oracle. S2 OCCLUDED — OFF stops at 1.13 s with
the standoff goal and perception-stamped yaw; ON coasts to 1.9 s (the belief's t_max + δ, short of
the 3 s cap), every late goal is the viewpoint, every late yaw target the re-emergence point,
fresh-stamped. S3 — a stranger 1.0 m from the CTRV point inside the window: OFF re-locks onto them
(the wrong lock), ON refuses and then re-locks the candidate 3 m from the CTRV point that only the
region gate can reach. S4 OUT_OF_VIEW — ON fresh-stamps the yaw to the CTRV point, OFF does not.

**`test_target_manager_loss.py` — 27/27** covers the same contract on a bare node with a fake
clock, including clock-skew refusal (an event whose `eval_stamp` is > 2 s from our clock is
ignored, once, loudly) and the stock baseline.

## Replay on a real bag (M6.2, first input) — and what it taught

`loss_replay.py` rebuilds the map from `/cloud_registered_body` + `/lidar_slam/odom` in the real
`fsm_node` and feeds it the recorded `/dtp/follow_target`. On **`20260909_closedloop_run5`**
(132 s, one subject, lab, filter off — as flown): **82 LossEvents in 48 coast episodes.** The
recorded target_manager re-acquired 47 of them in 0.09–0.38 s.

**Provisional labels.** All 48 episodes were labelled from the 480-px contact sheets
(`loss_replay/label_sheets/labels.csv`; 46 labelled, 2 left as "could not tell the subject from
the humanoid robot"; 11 marked UNSURE where nobody is visible in the ±2 s window and the
occlusion is inferred from the neighbouring episodes). The labeller was Claude, not a person, and
the sheets are not the video: **treat these as provisional, and re-label before any number goes
into the paper.** 38 OCCLUDED (almost all: the subject steps behind, or stands behind, a ~1.7 m
black cabinet about 4 m off the nose, head sometimes showing above it), 8 LOST (detector misses
with the subject in plain view).

**D1 on run5, three ways** (`loss_label.py score`, onset verdict, rows = label):

| map configuration | OCCLUDED → OCC / UNM / LOST | LOST → OCC / UNM / LOST | E1 recall | E2 false-OCC |
|---|---|---|---|---|
| as flown (`virtual_ground_height` −1.0) | 5 / 1 / 32 | 2 / 2 / 4 | **0.13** | 0.25 |
| ground lowered to −2.0, no view-floor rule | 1 / 34 / 3 | 0 / 5 / 3 | 0.03 | 0.00 |
| ground −2.0 **+ view-floor rule** (the code now) | 4 / 1 / 33 | 3 / 1 / 4 | **0.11** | 0.38 |

D1 **fails** on this bag, and the two rows explain why, in order of discovery:

1. **The virtual ground.** The bench rig's `lidar_init` origin is ~1.6 m up, so the subject's
   waist (z ≈ −0.7 … −1.0) is a metre *below* the map's effective virtual ground (−0.5). Every
   sample height under it was dropped and the verdict rested on the one row above the plane —
   head height (`rays_cast` 5 or 1 instead of 15). Lowering the ground made the rows exist…
2. **…and exposed the sensor's floor.** With the ground at −2.0, 39 of 48 episodes read
   UNMAPPED: the torso-height cells are UNKNOWN because the MID360 never looks that low at 4 m
   (−7° → 0.5 m under the sensor). They are not "unmapped space"; they are unmappable from
   here. Hence the view-floor rule (step 3b), after which the verdicts return to the head-height
   answer: **the cabinet does not block at the only heights this map can know, and the honest
   verdict is LOST.** run5's occluder is below the sensor's floor; it is the wrong occluder for
   this sensor, not a classifier failure that tuning ρ\*, N or `r_excl` would fix.

   The M6.3 course (1.8 m tubes, rig carried at 1.5 m, FAST-LIO started on the ground) is above
   the floor by design; the protocol in `m5-m6-plan.md` M6.3 now says so, and `fsm_node` warns
   when a target falls under either floor.

   The 3 false-OCCLUDED (episodes 2, 20, 48 — the subject emerging from behind the cabinet or
   passing the wheeled cart) are the same edge geometry with the verdict landing the other way;
   they are worth a look on the video before being counted.

**Open-loop two-arm replay through the real `target_manager`** (`loss_condition_replay.py`,
recorded events, `auto_reacquire_after_lost` forced true, so the replayed node coasts 66 times
where the recording logged 48 episodes):

| | OFF (carry-forward) | ON (conditioned) |
|---|---|---|
| coasts | 66 | 61 |
| re-acquired | 65 (98.5 %) | 58 (95.1 %) |
| lost | 1 (forecast exhausted) | 3 (**oracle LOST, verdict stood ≥ 0.3 s**) |
| re-acquire median | 0.173 s | 0.172 s |
| re-acquired via same id / band / region | 37 / 28 / 0 | 35 / 22 / **1** |

**Closed loop** (`loss_closed_loop_replay.py`: the oracle answers the *replayed* coasts live,
so the ON arm is no longer a lower bound), ground −2.0 + view-floor rule:

| | OFF | ON |
|---|---|---|
| coasts | 66 | 64 |
| re-acquired | 65 (98.5 %) | 61 (95.3 %) |
| lost | 1 (forecast) | 3 (oracle LOST) |
| re-acquired via region | 0 | 2 |
| LossEvents (OCC / UNM / LOST) | 112 (20 / 8 / 84) | 110 (21 / 10 / 79) |
| CPU, % of a core over the bag: `fsm_node` (map + oracle) / `target_manager` / `follow_planner` | 32.7 / 3.4 / 0.9 | 32.4 / 3.4 / 0.9 |

Same reading as before: on a bag whose losses are detector glitches, the conditioned arm's
*cost* is visible (three glitch coasts ended at 0.3 s that the baseline re-acquired a frame
later) and its *benefit* cannot be. The oracle's own cost is invisible next to map building
(the OFF and ON `fsm_node` figures are equal). `lost_confirm_s` is the knob; D2 is decided on
the tube course.

## Second bag: the Stage A recording (aircraft on the ground, dusk, a field)

`20260913_1955_stageA` (570 s, disarmed throughout, the rig on the ground, then carried) through
`loss_replay.py`: **80 LossEvents in 27 coast episodes** — 8 OCCLUDED, 5 UNMAPPED, 14 LOST by
first verdict; **no floor warnings** (the origin is at ground level, so all 15 rays exist for
most events). Provisionally labelled the same way (sheets, not video): **24 LOST** (the subject
standing or walking in plain view 5–8 m away at dusk — detector misses), **2 OCCLUDED** (walks
behind the house wall; episodes 20 and 21), **1 OUT_OF_VIEW** (passes the camera at ~1 m and
leaves the frame).

| | oracle OCC / UNM / LOST / OOV |
|---|---|
| label LOST (24) | 8 / 4 / 12 / 0 |
| label OCCLUDED (2) | 0 / 0 / 2 / 0 |
| label OUT_OF_VIEW (1) | 0 / 1 / 0 / 0 |

E2 false-OCCLUDED **0.33 (8/24)**, E1 0/2, E3 0/1. Each of the 8 false OCCLUDED had all 15 rays
and the target waist at z ≈ +0.5: the aircraft sat on the ground **in tall grass**, the camera at
~0.1 m, and the grass 0.3–2 m in front of the sensor is mapped as occupied cells that the rays to
the person cross. Ground clutter at the sensor's own height is a real false-occluder — but only
because the sensor was on the ground; at flight height there is no grass at 1.5 m (bushes and
branches will be the equivalent, and the paper should say so). The two OCCLUDED misses are the
same edge case as run5's cabinet: the last seen position is beside the wall's edge and the CTRV
point at onset has not entered the wall's shadow yet, so the ray is clear.

**So neither bag on this box can score D1 honestly:** run5's occluder is below the sensor's
floor, and the Stage A rig is on the ground inside the clutter. Both point at the same protocol
for M6.3, now in the plan: **the rig at flight height (1.5 m), FAST-LIO started on the ground,
occluders that rise above the sensor's floor at 4 m, and the walk-behind events scripted so the
subject is hidden with the CTRV point inside the shadow** (walk *through* the tube's shadow, not
stop at its edge).

## Parameters (starting values, all in one place)

SUPER side, `follow_test_ros2.yaml` `loss_oracle:` — `enable`, `target_topic`, `event_topic`,
`camera_offset_body`, `camera_yaw_offset_deg`, `fov_half_deg` 35, `fov_floor_deg` 7, `cyl_radius` 0.25,
`sample_dz` [−0.5, 0, 0.4], `lateral_n` 5, `r_excl` 0.4, `rho_star` 0.5, `ray_skip_start_m` 0.3,
`edge_max_m` 4, `edge_free_columns` 3, `rollouts` 32, `horizon_s` 3.2, `step_s` 0.1, `sigma_v`
0.3, `sigma_omega` 0.5, `sigma_heading` 0.2, `standoff_near/far` 3/4, `min_standoff` 2.5,
`viewpoint_height_offset` 0.5, `viewpoint_z_min/max` 0.5/3.0, `trail_radius` 0.6,
`trail_z_below/above` 1.0/1.3, `trail_window_s` 6, `log_period_s` 5. **M6.4b (0005), off in code,
on here:** `rollout_collision` true, `rollout_body_radius` 0.25, `rollout_deflect_step_deg` 15,
`rollout_deflect_max_deg` 90, `rollout_backoff_m` 1.0, `hidden_look` true, `hidden_look_min_w` 0.5,
`hidden_look_s` 1.5 (the measured bracket behind the last two is in the YAML's comment).

DTP side, `dtp_params.yaml` / node defaults — `target_manager`: `loss_conditioning`,
`loss_event_topic`, `occluded_coast_cap_s` 3.0, `reacquire_window_margin_s` 0.5,
`region_min_radius_m` 0.5, `region_sigma_k` 2.0, `lost_confirm_s` 0.3, `loss_event_max_age_s` 0.5,
`stock_baseline`; `follow_planner_node`: `loss_conditioning`, `loss_event_topic`,
`loss_event_timeout_s` 1.0, `stock_baseline`. All live-settable on `target_manager` except the
topic.

M6.3 sweeps ρ\*, N (`lateral_n` × heights), `r_excl`, K and the coast cap, per plan §2.4.

## Traps found while building it

- **A ray's "first non-free cell" is brittle against the target's own shadow** — see step 4.
  Without the trail rule, both filter arms of the walk-away scene read UNMAPPED.
- **The belief's offsets are relative to `eval_stamp`, not to the coast onset**, and the oracle
  re-rolls from the *current* coasted point every frame, so the window slides with each answer.
  `target_manager` derives the limit from the newest fresh event; the first version of the harness
  fed constant offsets and the window never opened.
- **Cross-node clock arithmetic** (`eval_stamp` vs `target_manager`'s arrival clock) is exactly
  what this repo's docstrings warn against; it is guarded (> 2 s skew → the event is ignored and
  an ERROR is logged once), and `use_sim_time` must match on `fsm_node` and `target_manager`.
- `loss_replay.py` must write its config variant **under the SUPER source `config/` dir**
  (`fsm_node` joins `config_name` onto its compile-time `ROOT_DIR`); it cleans up after itself.
- The open-loop `--arm on` replay is a **lower bound**: the recorded events answer the *recorded*
  coasts, which under the ON arm run longer, and the extra frames have no event.
  `loss_closed_loop_replay.py` is the fix (the oracle answers live).
- **Two replays on one DDS domain collide silently** — both publish `/dtp/targets`, and the coast
  count went 61 → 554 before it was noticed. Run arms sequentially; check with
  `ps -eo cmd | grep lib/dtp_perception/target_manager` (never `pgrep -f` with a pattern that is
  also in your own shell's command line: a chained script that waited on `pgrep -f "colcon
  build"` deadlocked on the shell that launched it, the CLAUDE.md trap yet again).
- **Read a background job's outputs after its notification, not before.** An ON-arm directory
  that "had no summary" was simply still running; the foreground re-run then collided with it.

## Still open (the honest list)

- **Nothing on the aircraft.** Not even observe-only: the oracle has never answered a real
  `/dtp/follow_target` live. The next `run.sh` + `dtp_follower.sh` observe-only session records
  `/planning/loss_event` for free (the follow config enables it).
- **D1 needs real labels on a real occlusion dataset** (M6.3's 140 scripted events). run5's
  provisional labels say its occluder is below the sensor's floor; the Stage A bags have few
  coasts and no walker behind anything.
- **D2 needs flights**; the closed-loop replay on run5 has no occlusion to reward the arm.
- ~~**`fsm_node` CPU on the full graph** with the oracle live is still unmeasured~~ **— MEASURED
  2026-09-23, and it passes.** `bench_bringup.sh` on the full graph (stack + both PX4 bridges +
  the follower, observe-only): `fsm_node` **22.5 % of a core idle → 25.9 % answering** synthetic
  coasts at 8 Hz, **RSS 2064 → 2065 MB**. §2.4's budgets are +≤ 5 % of a core and +≤ 10 MB, so
  **+3.4 points and +1 MB pass both.** 464 of 480 coasts answered, eval p50 27 / p99 161 /
  max 421 µs, the node's own line reporting classify p99 59–92 µs. (Replay had said 33 % for map
  + oracle, identical answering or not; on real hardware the oracle's cost is visible but small.)
  Run: `~/dtp_datasets/20260923_1656_bench_bringup/`.
- **The belief walks through trees — BUILT later the same day as patch 0005 (M6.4b, its own
  section above):** rollouts slide round mapped occupancy and a look-behind reduction fires when
  most of them stay hidden. Bench-gated, never flown; what it cannot do (the unseen back of a
  trunk) is stated there.
- **M6.6 (offline predictor study) — DONE later the same day** from the public EigenTrajectory
  sources and pretrained checkpoints (the flown assets never reached this box):
  [`m6.6-predictor-study.md`](m6.6-predictor-study.md). The learned branch stays disabled in
  flight.
- **Flight-day readiness on the bench is scripted:** `drone-dtp/scripts/bench_bringup.sh` brings
  the full graph up observe-only in the runbook's order, runs every check (odom rate and stamp
  lag, foreign cloud subscribers, bridge guard, perception, `estimator_preflight`,
  `stage_a_preflight`, CPU with the oracle idle and answering synthetic coasts at 8 Hz, a bag),
  and stops everything by process group. `bench_probe.py` holds the probes it uses.
- **`stage_a_preflight.py` now checks the M6 switch:** `loss_conditioning` must match on
  `target_manager` and `follow_planner_node` (FAIL if not), be false for any run with follow
  authority (`--stage-b` FAILs on true; Stage A WARNs), and `/planning/loss_event` should have a
  publisher (WARN if the oracle is off).
