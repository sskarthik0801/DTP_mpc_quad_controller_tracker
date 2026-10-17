# `goal_mode: nmpc` — a receding-horizon follow goal (Stages 1 and 2 built 2026-09-19/20, NEVER FLOWN)

**The question.** The 2026-09-15 20:01 follow flight followed for 13 s at a median 0.36 m/s, the
gap grew 4.4 → 6.5 m and the lock was lost
([`fact-follow-speed-and-descent-guard.md`](fact-follow-speed-and-descent-guard.md)). Tracking
was not the problem (+0.07 m median altitude error in Offboard). Would a model-predictive
controller help, and where would it go?

**Answer: not in the control loops — in the goal.** `follow_planner` places the standoff goal
geometrically and re-decides it from nothing on every 15 Hz tick: no memory, no horizon.
`goal_mode: nmpc` replaces that one computation with a receding-horizon nonlinear optimisation
of the goal over the person's forecast. PX4 keeps attitude and position control, SUPER keeps
obstacle avoidance and the safe corridor, `command_mux` keeps the fence and the descent guard.

**Status: Stage 1 (map-free) and Stage 2 (a visibility cost from ROG-Map, SUPER patch 0008) are
both built and bench-gated. Neither has flown, and `~/super_ws` has NOT been rebuilt with either**
— the installed `follow_planner_node` and `fsm_node` are still the pre-change binaries, by
decision (the user: fly the baseline first), and patch 0008 is NOT applied to the live SUPER
tree. `dtp_params.yaml` ships `goal_mode: geometric`, and Stage 2's weights default to 0.
Branch `nmpc-follow-goal`. The plan is `~/.claude/plans/okay-give-me-a-mellow-storm.md`.

## Don't re-derive: the five findings that shaped it

1. **The variables are the GOAL's positions, not the aircraft's.** A horizon of aircraft
   positions is speed-limited, so it ends ≤ `max_vel · T` ≈ 1.2 m ahead; publishing that hands
   SUPER a short rest-to-rest hop (peak √(a·d) ≈ 0.77 m/s) — the 09-15 flight by construction.
   SUPER already models the aircraft, with corridors and a map. So: `g_1 … g_N` continuing from
   the last goal, a double integrator on the *goal*, its velocity and acceleration as finite
   differences of the variables. No equality constraints, so it is unconstrained — the structure
   SUPER's own MINCO optimiser has, and what lets an L-BFGS solve it.
2. **No new node.** `follow_planner_node.hpp` carries a hardware-debugged contract (staleness
   gates, `TargetBehind`, `TransitCrossesKeepOut`, the twice-applied floor, the Z clamp, deadband
   and park guard, the `GoalStatus` escalation and `Suspend()`, silence-as-signal, both M6
   ablation arms). Only the body of `ComputeStandoffGoal` is replaced, behind a dispatcher;
   every guard still wraps the answer, and `ApplyHardGuards()` (floor vs predicted, floor vs
   present, Z clamp last) is ONE function both arms end with.
3. **SUPER still plans to rest at the goal; this does not change that.** `super_planner.cpp:728`
   gives a terminal velocity only when drone-to-goal > `planning_horizon/2` = 3.5 m, and
   `goal_vel_en` is false; at a 3 m standoff that distance is 0.5–1.5 m. The gain is that SUPER
   replans at 15 Hz *from the current state*, so consistent goals make successive plans chain.
   **It does not raise top speed** (`max_vel` / `max_acc` set that, 1.0 / 1.0 since 2026-09-21). Whether it cuts the ≥ 39 %
   `OPT_FAILED` replans is plausible and **untested**. A goal further ahead (to make
   `goal_vel_en` bite) breaks the standoff contract and every scorer: it is
   `nmpc_publish_stage` > 1, refused without `nmpc_carrot_experiment`, and not part of this.
4. **ROG-Map cannot be queried out of process** (no service; `/rog_map/occ` publishes only when
   subscribed, `rog_map_ros2.hpp:390`, ~8 × 10⁶ cell reads/s if you do). So Stage 2's
   visibility data must be computed inside `fsm_node`, by the loss oracle, and shipped as a
   compact per-azimuth summary. Stage 1 has no map access and needs none.
5. **The solver is LBFGS-Lite, vendored from its MIT upstream — NOT from SUPER.** SUPER is
   LGPL-3.0, its copy of the same solver carries no licence notice, and `follow_planner` is
   declared `Proprietary`. 219 of SUPER's 261 code lines are verbatim upstream. See
   `drone-dtp/ros2_ws/src/follow_planner/THIRDPARTY.md`. No other optimiser exists on this box
   (no CasADi, OSQP, acados, nlopt, Ceres), and nothing was installed.

## What it is

`drone-dtp/ros2_ws/src/follow_planner/include/follow_planner/nmpc_follow.hpp` — header-only,
Eigen only, no ROS, so it is unit-tested. Horizon **1.2 s** (= `target_manager`'s
`min_coast_horizon_s`, the shortest forecast upstream guarantees; inside the CTRV's 1.5 s),
**18 stages of one goal tick** (so the warm start is an exact index shift), 54 variables.
Stage *k* is the person at *(first forecast point) + (age of the message) + k ticks*: the first
forecast point already carries the closed-loop latency and nothing is added to it.

Every term is a C¹ penalty, normalised, stage sum ÷ N:

| Term | Weight | Note |
|---|---|---|
| standoff **band**, a dead zone | 4 | inset `nmpc_band_margin_m` 0.25 inside [3, 4] m |
| angle off the trail ray (its sine, in metres at the nominal radius) + anti-mirror hinge | 1 | `az_des` is computed by the geometric code, so the 45° near-side bias and the re-projection escalation survive verbatim; range-independent on purpose (trap 15) |
| altitude | 8 | the largest shaping weight: goal-height jumps were the measured defect |
| `min_standoff` floor vs the **present** person | 500 | shadows the hard floor |
| Z range, fence mirror (off) | 200 | shadow the clamp and the mux |
| agreement with the previous solution, decay 0.85 | 2 | |
| published-stage anchor | 3 | to `g_0 + v_0·t`, see trap 1 |
| goal vel / acc limits (squared-norm, smoothed L1) | 60 | 1.5 m/s and 3.0 m/s² — the GOAL's, deliberately above SUPER's envelope, 3-D norm |
| acceleration, jerk | 2 / 0.1 | |
| goal velocity RELATIVE to the person's | 4 | the friction (trap 16): free for a walker, damping beside a stander |
| Stage 2: known-occluded fraction, clear radius | 0 / 0 | off = exactly Stage 1; 10 / 4 when on |

**A commanded step is not smoothed.** Nothing to continue (a new track, a `Hold`, a refused
solve), a horizon older than 0.25 s, a re-projection **escalation**, or the geometric goal
jumping > 1.0 m restarts the signal from the geometric goal. The node logs the count of each.
A thin forecast shortens the horizon to 6 stages; a deep coast shortens it and stiffens it.

**It only ever offers a candidate.** Refused when: not finite, cost increased, > 3.0 m from the
geometric goal, range to the present person outside `[min_standoff − 0.5, far + 1.5]`, or more
than 5 budget cancels in a row. Then the node publishes **this tick's geometric goal** — it
never `Hold`s, never `Suspend`s, never freezes — and after 30 refusals in a row it latches
`DEGRADED` to geometric for the process lifetime. One cancelled solve (8 ms budget) is accepted:
its iterate descends from a warm start. The L-BFGS return code is recorded, not judged — every
exit leaves a descent iterate, and with a warm start "line search failed" mostly means "already
there".

## Traps found on the bench — each would have flown otherwise

1. **The published-stage anchor penalised speed.** Anchored to `g_0`, the goal trailed a walker
   by a distance proportional to pace: 1.1 m outside the band at 1 m/s. Anchored to the
   constant-velocity continuation it is the same point at rest and has no lag in motion.
2. **`goal_acc_max` 1.0 cannot brake inside a 1.2 s horizon.** After a 90° turn at 1 m/s the
   goal overshot 1.5 m sideways and swung for 10 s. At 3.0: 0.27 m, settled in 3 s. The step per
   tick is bounded by the velocity limit either way (0.1 m), so the goal stays continuous.
3. **A 40-iteration cap left turns unconverged** (1.0 m outside the band). 150, under the 8 ms
   budget. The cheap stop test (`past 3`, `delta 1e-4`: ~16 warm iterations, 0.06 ms) and the
   generous cap are a pair — either alone fails.
4. **A 1.5 m sanity bracket fired in an ordinary turn**, where the solver cuts the corner the
   geometric goal swings round, and the fallback then published a 1.5 m jump. It is a garbage
   filter, not a leash: 3.0 m. Safety is the range check and `ApplyHardGuards`.
5. **A dead zone rides its far edge.** On the full [3, 4] m band, 14 % of goals spilled up to
   0.43 m outside when a person set off after a pause. Hence the 0.25 m inset.
6. **A term repelling the goal from the aircraft is wrong twice.** A hovering follower *should*
   sit on its goal, so the term makes the goal creep away from an aircraft that has arrived; and
   a person backing towards the goal could not push it past the aircraft (2.65 m where the
   geometric goal kept 3.74 m). Removed; there is deliberately no such term.
7. **Persistent starvation never converges.** One iteration a tick gave 69 % of goals in the
   band. A run of cancels is now a refusal.
8. **Unoptimised Eigen is 10–30× slower**, and the flight workspace is Release only because its
   CMake cache says so. `follow_planner/CMakeLists.txt` now defaults to Release.
9. **A reset rule that watched the trail azimuth fired on heading noise.** It tested "`az_des`
   stepped > 0.2 rad", to catch the re-projection's alternating azimuth offset. Found only by
   replaying a REAL bag, not by the synthetic gate: on run5 it fired **44 times in 1587 solves**,
   because the heading is `atan2` of a differenced velocity, and each firing was a jump back to
   the geometric goal — the noise this mode exists to smooth. The rule now tests the
   re-projection's own two variables, and only an *escalation* (magnitude up, sign flipped,
   radius out): a relaxation is a decay on every ACCEPTED status and would otherwise reset at
   status rate for seconds. The other 300 resets on run5 are legitimate — `Hold()` resets the
   servo state, and a handheld rig holds often.
10. **The deadband still quantises the wire.** The solver's signal is continuous (0.1 m/tick),
   but `goal_deadband_m` 0.15 holds the published value, so on the wire both arms step by
   0.15–0.2 m. That deadband exists because SUPER sets `new_goal` on every message (risk A13);
   it was left alone. The solver's gain is in consistency, not in finer steps.

## Gates (all on a scratch build; the flight binary was never touched)

- **`colcon test --packages-select follow_planner` — gtest 15/15** (`test/test_nmpc_follow.cpp`,
  the package's first C++ test): analytic gradient of every term within 1e-5 of central
  differences, and proof the sample activates every term; C¹ knots; fixed point; monotone
  descent on 100 random starts; bit-identical determinism; garbage and starvation; exact
  warm-start shift; forecast sampling; a 30 s closed loop (walk, 90° turn, stop): max goal step
  0.068 m, max z step 0.0000 m, band excursion 0.000 m, 27 warm iterations a solve, solve
  p50 0.22 ms / p99 0.53 ms; and four Stage 2 tests (below).
- **`python3 drone-dtp/scripts/nmpc_check.py` — 41/41** (35 for Stage 1, 6 for Stage 2's N12), the
  real node, ROS_DOMAIN_ID **90**, both arms from fresh processes, ~9 min. **Re-run after any `follow_planner` change.**
  N1: `goal_mode geometric` reproduces the pre-change binary to 0.00e+00 over 8 poses (2 withheld
  in both); `nmpc_check_geometric_ref.json` was recorded from that binary with `--record-ref`.
  nmpc vs geometric: largest goal step across a pause **0.24 vs 0.75 m**, direction reversals
  **0 vs 2** (pause and turn), 100 % in band in every scene, 0 fallbacks; solve in the node
  p50 0.2–0.7 ms, **p99 3.9 ms, max 4.3–5.1 ms against a 5 / 8 ms budget** — the tail is cold
  starts at 150 iterations, it is an idle-box number, and the margin is not generous: measure
  it under the real stack (`bench_bringup.sh`) before flying. Also: fail-loud on a bad mode, weight or publish stage; the
  floor and the transit guard; Z clamp and a NaN target; refused and starved solves latch
  DEGRADED with the stream on contract; silence on the same tick; `DROPPED` ×5 re-projects then
  suspends; with `loss_conditioning` the goal IS the oracle's viewpoint in both arms.
- `loss_conditioning_check.py` **16/16** with `--goal-mode geometric` and `--goal-mode nmpc`.
  **Its "nodes did not come up" flake is FIXED (2026-09-20).** It failed about one run in three
  all day, including before any node change, always with both nodes logged up. Cause:
  `target_manager` has no timer — `/dtp/lock_state` goes out at startup and then only from its
  `/dtp/targets` callback — and the harness waited in silence, so it saw a state only if its
  subscription matched before the startup message (on failure: 0 `lock_state` messages, parameter
  service ready, `/dtp/targets` matched). The harness now publishes empty target arrays while it
  waits, and says what it saw if it still gives up. Five runs in five since.
- Domains in use: 0, 42, 86, 87, 88, 89, 90. **89 is shared by `fence_check`,
  `unhit_decay_check` and `loss_conditioning_check`** — never two of them at once.

## Stage 2 — the visibility cost (built 2026-09-20, bench-gated, NEVER FLOWN, NOT applied to the live SUPER tree)

**What it is.** SUPER patch `drone-dtp/patches/0008-super-visibility-field.patch` makes the loss
oracle inside `fsm_node` publish `dtp_msgs/VisibilityField` on `/planning/visibility` for EVERY
locked `/dtp/follow_target`, seen or coasting: 24 azimuth bins about the person, and per bin how
far out a viewpoint stays clear of the *inflated* map (`clear_r`), the fraction of the person it
would see (`vis`) and the fraction it cannot know (`unk`). `follow_planner` (goal_mode nmpc, with
`nmpc_w_vis` / `nmpc_w_clear` > 0) turns it into a cost. **Both weights default to 0, and at 0
the problem IS Stage 1** — a stale, absent or blind field degrades to a configuration that is
already gated. Recommended when on: `nmpc_w_vis 10`, `nmpc_w_clear 4`.
`loss_oracle/visibility_enable`: off in code, on in `follow_test_ros2.yaml` *in the patch*.

**It is a new message, not more fields on `LossEvent`**, because a `LossEvent` answers only a
coasting target and the point here is to avoid the occlusion while the person is still seen;
because riding on it would tie this to the `loss_conditioning` switch (two contributions on one
switch is not an ablation); and because new fields would stale every `--dump/--compare` baseline.
`evaluate()` is untouched: **30/30 `LossEvent`s identical, old binary vs patched, field by field**
(`loss_oracle_check.py --only A --dump` on each, `pair_compare` over O1/O5/O4 — the script's own
`--compare` pairs only its M6.4b scenes, so it reports "0 matched" for scenario A; that is its
scope, not a mismatch).

**Cost in `fsm_node`:** 24 bins × 3 sub-azimuths × 3 heights = 216 rays, eval **p50 ~400 µs,
p99 530–730 µs** over four runs (budget 1.5 ms) at ~8 Hz ≈ 0.3–0.6 % of a core. ~320 bytes a
message.

**Two mechanisms, and the first is the one that gets a goal out of a shadow.**
(1) *The trail bearing is made visibility-aware, outside the solver* (`nmpc::ClearBearing`): if the
bearing the node asked for is dark in the field, it is replaced by the nearest clear one (5° steps,
up to 90° round, with 25° of side hysteresis — more than one bin, so a flickering bin cannot flip
the goal to the other side of the person), and the solver's ordinary bearing term carries the goal
there at the goal's own speed limit. (2) *A visibility cost inside the solver* shapes the path
locally. Trap 17 is why (2) alone is not enough.

**In the solver.** The binned field becomes two Fourier series in azimuth — 8 harmonics (resolves
~22°, a trunk's shadow) and 3 (a broad hill: which way out of a wide one) — each windowed with
Lanczos sigma factors, and the cost is `w_vis · (hinge(O₈)² + hinge(O₃)²)` on the *known* occluded
fraction `1 − vis − unk`, plus `w_clear · hinge(r − R(az))²`. Azimuths are about the *forecast*
person, so the field rides along with them. The node averages successive fields bin by bin,
counts the person's displacement since the field was built as age (at 1 m/s), and past
`nmpc_vis_timeout_s` 0.5 **ramps** the weight to zero over `nmpc_vis_fade_s` 0.3 — a step would
move the goal by the whole visibility offset in one tick. A field that knows no height of the
person (`heights_used` 0: the MID360 sees no lower than 7°) is dropped whole: it says nothing.

**What the map can and cannot know — read before trusting it.** ROG-Map holds an obstacle as a
one-cell shell of the face the sensor lit, and rays from a candidate viewpoint mostly start in
space the sensor never observed. In an open scene the field reads **vis 0.19, unk 0.81**. So it
knows occlusion only *near where the aircraft has looked from* — which is where a solver that
moves the goal 0.1 m a tick operates, and the case that matters (something comes between the
aircraft and the person) is the geometry it knows best, because that obstacle's lit face is the
one facing the aircraft. It cannot plan a route round the far side of anything.

### Traps found building Stage 2 — the worst of them only by testing end to end against the real map

11. **One ray per bin puts a narrow shadow in the wrong place.** A tube 2 m from the person
    casts an ~11° shadow, narrower than a 15° bin. Sampled at bin centres the map put the shadow
    7° from where it was and called the neighbouring bin clear, and end to end the goal moved
    **deeper into** the real shadow (line of sight 0.17 → 0.07 m from the tube's axis). Three
    azimuths inside every bin fixed it (0.67 / 0.33 in the two bins that straddle the bearing).
12. **A 4-harmonic fit made a trunk's shadow a bump ~45° wide and 0.375 high** — too shallow to
    leave. 8 harmonics. The flicker the low-pass was for (ROG-Map's tangent cells) is handled by
    the sub-azimuth sampling and by averaging successive fields.
13. **A bare truncated Fourier series of a box rings** — ±9 % Gibbs ripples across a wide
    shadow's plateau, each one a well. The goal sat 2° from the middle of a 60° shadow. Lanczos
    sigma factors: a box now comes back with one maximum.
14. **A wide shadow is a plateau with zero slope**, so the 8-harmonic series alone gives a
    gradient method no way out. Hence the second, coarse series.
15. **Bearing as a lateral offset in metres buys range.** It is smaller the closer the goal is,
    so a goal held off-bearing was pulled inward (to 2.99 m). It is now the *sine* of the angle
    off the trail ray, scaled to metres at the nominal radius.
16. **The goal never settled beside a STANDING person.** The constant-velocity anchor (trap 1's
    fix) made the goal a frictionless integrator: it wandered ±5° and across the whole 3.1–3.9 m
    band for as long as it was watched, and at `w_vis` 25 it ran right round the person. Fixed
    by `nmpc_w_vrel` (4): a penalty on the goal's velocity *relative to the person's*, so a
    walker costs nothing and a stander is friction. It also helped Stage 1: fewer iterations
    (44.6 → 26.7 warm) and a smaller largest step (0.100 → 0.068 m) in the closed-loop test.
17. **A visibility COST cannot be relied on to get a goal out of a shadow it starts in.** The
    middle of a shadow is the top of a hill: slope zero, "converged", nothing moves — and that is
    not a corner case, because a person standing still is trailed from wherever the aircraft
    already is, so whatever comes between them lands *exactly* on the trail bearing. Real map,
    weight fully on: the goal did not move a millimetre. (A 1 cm offset in my unit test had been
    hiding it.) Worse, in a WIDE shadow the middle is *stable*: the bearing term pulls back at
    ~24.5 per radian of offset and a 60° shadow's coarse hill pushes out at only ~21.7. A first
    fix nudged the solver's start 3° a tick; it won or lost that race **run to run on the same
    binaries** — 0.43 m clear of a tube in one run, 0.02 m in the next. Hence `ClearBearing`,
    which has no race in it: three runs, 1.01 / 1.05 / 1.04 m clear.
18. **UNKNOWN beside a known obstacle is not "cannot know".** Rays grazing past the lit shell
    into a tube's unseen side read UNKNOWN, so a tube dead between viewpoint and person read
    0.56 blocked, not 1.0. In the field only (never in `evaluate()`), an UNKNOWN first cell that
    the *inflated* map calls occupied counts as blocked.
19. **A test scene that lies gets a true answer.** Steering the trail bearing with a made-up
    velocity on a person who never moved worked until the friction term (16) rightly followed
    the phantom motion. The end-to-end scene is now physically consistent.

### Stage 2 gates

- **gtest 15/15** — adds: a box comes back without ripple and one bin survives; `w_vis` 0 is
  bit-identical to Stage 1 whatever the field holds; `ClearBearing` leaves a clear bearing alone,
  moves out of a 60° and of a one-bin shadow to a bearing that really is clear, holds its side
  when one bin toggles and changes it when three do; from a DEAD-CENTRE start the goal leaves a
  60° shadow (49° off, occluded fraction 0.00, in the band) and settles.
- **`python3 drone-dtp/scripts/visibility_field_check.py` — 13/13**, the real patched `fsm_node`
  on `loss_oracle_check`'s synthetic scenes (domain **88** — never with that script or
  `target_filter_check`). Seen ⇒ a field and no `LossEvent`; shape; nothing *known* occluded in
  the open; a tube and a wide trunk block the bearings round them and nothing known elsewhere;
  coasting gets both; the floor; the budget; the switch. **V9 is end to end against GEOMETRY:**
  the real `fsm_node` feeding the real `follow_planner_node`, a tube exactly between the
  aircraft and a standing person. Stage 1 parks the goal in the shadow (line of sight 0.00 m
  from the tube's axis); Stage 2 ends with it **0.98–1.05 m from the axis over four runs (radius
  0.20), 3.6–3.7 m from the person.** `--only-e2e` runs V9 alone (~4 min). Needs `--super-cfg-dir` for a scratch build (`fsm_node` resolves its config
  against the source dir it was compiled from) and `--planner-exe`.
- **`nmpc_check.py` — 41/41.** N12 plays `fsm_node` (`--only-n12` runs it alone): the goal leaves
  a 30°-half-width shadow (40–42° off over four runs) and stays in the band; when the field stops the weight ramps off and the goal walks back
  (slowly, ~5°/s: the friction term) with no step larger than before; no field, and a field that knows no height, are Stage 1 (asserted on
  the node's own `| vis 0.00`, not on a range tolerance tighter than the dead zone).

### To install Stage 2 (NOT done — it rebuilds the flight `fsm_node`)

```bash
cd ~/super_ws && colcon build --symlink-install --packages-select dtp_msgs   # the new message FIRST
bash ~/super_ws/src/drone-dtp/scripts/apply_super_patch.sh                    # applies 0008, idempotent
colcon build --packages-select super_planner                                  # NOT --symlink-install
colcon build --symlink-install --packages-select follow_planner
python3 drone-dtp/scripts/visibility_field_check.py && python3 drone-dtp/scripts/nmpc_check.py
python3 drone-dtp/scripts/loss_oracle_check.py && python3 drone-dtp/scripts/target_filter_check.py
```
Then `goal_mode:=nmpc` plus `-p nmpc_w_vis:=10.0 -p nmpc_w_clear:=4.0` on `follow_planner_node`.
Never two `colcon build`s at once. `git apply --check` of 0008 passes on the live tree (2026-09-20).

## Bag replay (Stage 1) — `drone-dtp/scripts/nmpc_replay.py`

Plays a bag's `/dtp/follow_target` and odometry into the real node, one `goal_mode` per run, on
domain 90 (it refuses if a `follow_planner_node` is already there). **Open loop**: the aircraft
in the bag flew the goals of the day, so this scores the goal *signal* each arm makes from the
same inputs, and says nothing about how an aircraft would have flown it. **Not deterministic**
(a 15 Hz wall-clock timer samples an ~8 Hz stream), so each arm runs twice and
`--compare` prints the within-arm scatter first. Results in `~/dtp_datasets/20260920_nmpc_replay/`.

| run5 (132 s, handheld, ~75 s of goals) | geometric ×2 | nmpc ×2 |
|---|---|---|
| direction reversals | 52 / 52 | **15 / 18** |
| goal step on the wire, p50 | 0.250 / 0.251 m | **0.175 / 0.171 m** |
| goal step on the wire, p95 | 0.726 / 0.726 m | **0.341 / 0.315 m** |
| jumps > 1 m (input-driven, see below) | 27 / 27 | 30 / 30 |
| goal-height jumps ≥ 0.1 m | 0 / 0 | 0 / 0 |
| goals in the band | 99.6 % | 99.4 / 99.2 % |
| paired \|Δgoal\| within the arm, p95 | 0.004 m | 0.269 m |

Across the arms the goals differ by p50 0.42 m / p95 1.63 m, well above either arm's own scatter.
Solve p50 0.17 ms, p99 2.1 ms, max 3.4 ms; 1 fallback in 3175 solves, 0 cancels.
`20260915_2001_stageB_salvaged` is consistent with it — reversals 22 → 9, step p95 0.476 →
0.189 m, 9 input jumps in both — but it is one run per arm over a short follow: quote run5.

**What the replay showed that the synthetic gate could not:**
- **The solver inherits heading flips.** Both arms take the same 4–7 m goal jumps at the same
  bag times, with no gap and no jump of the person: a slow walker's heading (`atan2` of a
  differenced velocity, floor 0.25 m/s) flips, the trail side flips 180°, and the goal goes with
  it. `az_des` comes from the geometric code, and a geometric goal that jumps > 1.0 m restarts
  the solver, so it follows. **Stage 1 does not fix this, and was not designed to.** Letting the
  solver ride a flip out instead (it would barely move if the flip reverted within a second) is
  a candidate change with its own risk — the 3.0 m sanity bracket would fire — and is not done.
- Those jumps are ~5 % of the moving steps, so **left in, they ARE the p95**: the first version
  of the scorer reported nmpc as worse (4.0 vs 1.9 m) purely because of which side of that cliff
  the 5 % line fell. `--compare` now counts them and scores the rest.
- **Most "resets" are `Hold`s.** On run5 276 of 305 resets, and on the flight bag 2687 of 2697,
  are "nothing to continue": `Hold()` resets the servo state, and on the flight bag two ticks in
  three compute a goal and then withhold it (the transit guard), in both arms. There the nmpc
  arm is the geometric goal plus a wasted 0.16 ms.
- It is where trap 9 was found.

## How to run it

```bash
ros2 launch dtp_bringup dtp_system.launch.py goal_mode:=nmpc …      # or follower.launch.py
bash drone-dtp/scripts/deploy_follow.sh --observe --goal-mode=nmpc   # shadowed, commands nothing
```
The arm is a **launch argument**, printed by the launch and read back by
`stage_a_preflight.py`, which WARNs on nmpc in Stage A, FAILs on nmpc + `stock_baseline`, and
under `--stage-b` FAILs on nmpc until `--nmpc-cleared` is passed. The node reads it at
construction: changing the arm is a relaunch. Every `nmpc_*` parameter stays on the node's
default, where its reasoning lives.

## What is still owed

1. **Rebuild in `~/super_ws`** — deliberately not done. It is safe in principle (`goal_mode`
   defaults to geometric and N1 showed that bit-identical), but it is the user's call and comes
   after the baseline flight. **`dtp_msgs` must be rebuilt BEFORE `follow_planner`**: the node now
   includes `dtp_msgs/msg/visibility_field.hpp`, so building `follow_planner` alone fails. For
   Stage 2 also apply patch 0008 and rebuild `super_planner` ("To install Stage 2", above).
2. **The baseline flight** on the current config, scored with `follow_speed_report.py`: the
   numbers every later comparison needs. If the lag is not goal-placement-bound, re-scope.
3. **Stage A, two legs on one sortie** (`goal_mode:=geometric`, relaunch, `nmpc`), both scored
   with `analyze_shadow_goals.py --goal-topic /dtp/follow_goal` — raw against raw, never raw
   against the arbitrated shadow topic. Stage A cannot say whether a goal is *reachable* (no
   occupancy in the bag, SUPER out of the loop); M5.1 props-off is where a real `GoalStatus`
   first answers that, and its ACCEPTED/SNAPPED/DROPPED mix is the number Stage 2 is argued from.
4. **A same-flight A/B (a second node in shadow) is NOT safe yet:** `stage_a_preflight.py` is
   fail-open for it (`OUR_NODES` is name-based; `/planning/yaw_target` is not in
   `SINGLE_PUBLISHER_TOPICS`), and a second node name misses the `follow_planner_node:` params
   block, including `goal_z_min_m: 1.2`. Harden the preflight first.
5. **Stage 2 has never seen a real map.** Every Stage 2 gate is a synthetic scene. The field's
   honesty rests on what ROG-Map knows away from the aircraft's own line of sight, which a real
   forest will answer and a tube on a bench cannot. Replay a bag with `/cloud_registered_body`
   through the patched `fsm_node` (the `loss_replay.py` plumbing) and look at the fields before
   flying it; then its own Stage A legs, `nmpc_w_vis` as the one variable.
6. **The solver inherits heading flips** (see the replay section) and Stage 2 does not change that.
