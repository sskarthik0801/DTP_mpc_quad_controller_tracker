# Contributions.md — what this project claims as research, and what backs each claim

**Written 2026-09-16.** Sources, all in this repo: [`docs/paper/main.tex`](docs/paper/main.tex),
[`docs/paper/README.md`](docs/paper/README.md), [`docs/m5-m6-plan.md`](docs/m5-m6-plan.md),
[`docs/m6-loss-oracle.md`](docs/m6-loss-oracle.md),
[`docs/m6.6-predictor-study.md`](docs/m6.6-predictor-study.md), the `docs/fact-*.md` files, the
`drone-dtp/patches/` series, and `CLAUDE.md`.

This is the **claim list**, not the schedule. The schedule — dated rows, owners, pass thresholds
and sample sizes — is `docs/m5-m6-plan.md`. This file exists so that every claim the paper makes
has one place that says: *what it is, why it is new, what exists today, what evidence backs it,
what would retract it, and which experiment closes it.*

## How to read the status column

| Tag | Means |
|---|---|
| **PLANNED** | Designed, not written. |
| **BUILT** | Code exists and runs. |
| **BENCH-GATED** | A named gate script passes on the real node, bench or synthetic scene. **Never flown.** |
| **REPLAYED** | Exercised on a recorded bag through the real node. Still not a flight result. |
| **FLOWN** | Ran on the aircraft in the air. Not the same as *evaluated*. |
| **EVALUATED** | Scored against labelled ground truth with a pre-registered threshold. |

**Nothing in this project is EVALUATED yet.** Two Offboard flights exist (2026-09-14 and
2026-09-15); the second followed a walking person under mux arbitration for 103 consecutive
seconds. Every number in the paper that is not from those two bags is a bench figure, and
`main.tex` wraps each one in `\bench{}` precisely so it cannot be promoted to a flight result by
accident. **Do not remove a `\bench{}` wrapper except by replacing the number with its in-flight
equivalent.**

---

## 0. The thesis

> Every prior aerial-tracking system treats target loss as **one** undifferentiated event and
> answers it with **one** policy: extrapolate and keep flying. We classify the **cause** of loss
> against the onboard-built 3-D occupancy map — occluded by known geometry, genuinely lost,
> unmapped, or out of the camera's cone — and condition planner behaviour on that classification.
> The geometry required is already computed, for free, as a by-product of obstacle avoidance.

The argument in one sentence: *the information needed to know **why** you lost someone is already
in the map you built to avoid hitting things.* Two losses with identical sensor signatures — a
person who stepped behind a trunk, and a person who walked out of range in plain view — have
opposite correct responses, and an unconditional extrapolation policy is right for one and
wasteful or dangerous for the other.

**Target venue:** RA-L (8 pages) ≈ 2027-01-18, or IROS 2027 (deadline typically ~1 March). ICRA
2027's mid-September deadline is out of reach. Full draft **2026-12-18**.

---

## 1. Primary contributions — the five the paper claims

### C1 — Loss-cause classification against the onboard map
**Status: BENCH-GATED + REPLAYED. Flown once with the oracle live; never evaluated.**

**The claim.** A line-of-sight test against the planner's own occupancy map labels every
target-loss event, at the first frame the tracker reports no detection for the locked target.
Four classes, not two: `OCCLUDED`, `UNMAPPED`, `LOST`, `OUT_OF_VIEW`.

**Why it is new.** Sec. II of `main.tex` surveys six systems — Elastic Tracker, Wang et al.,
SVPTO, BPOD, Eva-Tracker, SUPER — and none asks *why* the target disappeared. Occlusion is
treated either as a cost to **avoid** or as an event to **extrapolate through**. Eva-Tracker
(ICRA 2026), the closest system and the one a reviewer will name, does unconditional prediction
carry-forward: a target that walked out of range and a target standing behind a trunk get
identical treatment.

**What exists.** `super_core/loss_oracle.hpp`, inside SUPER's `fsm_node`, sharing the planner's
`map_ptr_` — so the architecture figure's "one map, two consumers" edge is literally true and the
raycasting CPU is not paid twice. SUPER patch **`drone-dtp/patches/0004-super-loss-oracle.patch`**.
It answers every `COASTING` `/dtp/follow_target` with `dtp_msgs/LossEvent` on
`/planning/loss_event`.

- Eq. 1 over **15 rays** (3 heights × 5 lateral) on the target's cylinder, cast from the **camera**
  optical centre via TF — the camera is what lost them, not the LiDAR.
- Rays stop `r_excl` = **0.4 m** short of the sample, so the subject's own still-occupied cells
  never count as their own occluder.
- `OCCLUDED` if blocked fraction ≥ ρ\* = **0.5**; else `UNMAPPED` if unknown fraction ≥ ρ\*; else
  `LOST`. `OUT_OF_VIEW` is returned first, from the ±35° cone.
- `UNMAPPED` is **reported** separately but **conditioned as** `LOST`.
- `LOST` with a clear line of sight includes detector false negatives. That is correct by design
  and the paper says so.
- The message echoes `query_p` — the point actually tested — for the same reason the goal contract
  echoes `requested_p` (see S1).

**Evidence.** `python3 drone-dtp/scripts/loss_oracle_check.py` — **28/28** on patch 0004, **56/56**
with 0005's tree scenes, on the real `fsm_node` process against synthetic LiDAR scans.
**Cost: classification p99 81 µs (108 µs with 0005's switches on); full evaluation p99 406 µs, or
2.5 ms with the obstacle-aware rollouts.** Budgets are 2 ms and 10 ms inside one 8 Hz (125 ms)
perception frame — so the classifier is 25× inside budget, and "a cheap contribution is a feature"
is a quantified statement, not a hope.

**Replay.** `run5` (2026-09-09): 82 events in 48 coast episodes — 7 `OCCLUDED`, 3 `UNMAPPED`,
38 `LOST`. The Stage A bag (2026-09-13): 27 episodes. **Both unlabelled beyond a provisional pass
by Claude, and neither can score D1** — see N2 and N3 below, which is a finding, not a failure.

**Flight.** The oracle was enabled in the configuration flown on 2026-09-14 and its verdict on
that sortie's single lock loss is on record: `LOST`, 7–33 % of rays blocked, i.e. no mapped
occluder — consistent with the post-mortem's independent finding that the loss was a depth-patch
glitch in prop wash, not an occlusion. **n = 1, unlabelled.**

**What closes it.** **M6.3**: the 140-event handheld dataset (below), labelled, → **D1**.
**D1 passes when** E1 `OCCLUDED` recall ≥ **0.85** (n = 60; 95 % Wilson lower bound ≈ 0.74), E2
false-`OCCLUDED` ≤ **0.10** (n = 40; upper bound ≈ 0.23), E3 `OUT_OF_VIEW` correct ≥ **0.90**, and
the timing budgets hold.

**What would retract it.** D1 failing on a dataset that *does* satisfy the two preconditions in N2
and N3. If the classifier cannot separate on correctly-collected data, the paper is an RA-L
systems paper, not an algorithms paper — and that call is due **2026-11-10**.

---

### C2 — A reappearance belief, not a point — and planner conditioning on it
**Status: BENCH-GATED + REPLAYED (open-loop and closed-loop). Never flown with conditioning on.**

**The claim.** Under `OCCLUDED`, propagate the predicted target state across the occluder's
silhouette to produce a **distribution over where and when** the subject re-emerges, and condition
the viewpoint planner on it — pre-position for the edge rather than chase the extrapolated point.

**Why it is new.** BPOD is the closest formalisation of reasoning about future visibility, but its
probability of detection is propagated over continuous stochastic states and **weights a cost**;
ours is a discrete-event, map-conditioned classification that changes the planner's **mode**.
Eva-Tracker's FoV-ESDF is a planner-side visibility construct, not a belief over re-emergence.

**What exists,** in the order the code runs it (patch 0004, extended by
**`0005-super-belief-around-obstacles.patch`**):

1. **Occluder extent.** From the blocked ray nearest the target's centre, sweep the bearing left
   and right in one-cell angular steps at the target's range; three consecutive free columns end
   the occluder → two edges and an angular interval.
2. **Rollouts.** K = **32** constant-turn-rate samples from the coasted state (speed, turn rate and
   heading each drawn once per rollout), **3.2 s at 0.1 s**. A rollout re-emerges at its first
   point outside the interval with a camera ray containing no occupied cell — `UNKNOWN` is
   allowed, because the space behind a trunk is unknown by nature. Exits are clustered by side
   (mean, spread, arrival-time min/median/max, mass); the mass that never exits is the
   **still-hidden mass** `w_h`. **Seeded per event, so a replay reproduces the belief.**
3. **Obstacle-aware rollouts** (`rollout_collision`, patch 0005). Rollouts cannot cross an occupied
   cell of the raw grid; the body is sampled every half cell along its lateral diameter (±0.25 m)
   and the radius ahead; a blocked sub-step deflects ±15°…±90° (own turn side first) **for that
   sub-step only**, so the rollout slides round and resumes its course. **No new random draws:
   with the switch off the belief is bit-for-bit patch 0004's — 36/36 events identical.**
4. **Look-behind** (`hidden_look`, patch 0005). When `w_h` ≥ 0.5 or no rollout exits,
   pre-positioning for an edge is pointless: the yaw target becomes the hidden rollouts' mean
   position at 1.5 s and the viewpoint search runs about that point — its clear-ray requirement
   forces the goal off the camera–occluder line, which is the sidestep a pilot makes to see behind
   a tree.
5. **Reduction** to what the planner consumes: yaw target = the dominant cluster's mean (or the
   look point); goal = the candidate on the [3, 4] m standoff annulus (24 azimuths × 3 radii)
   outside inflated occupancy, ≥ 2.5 m from the subject, with a clear camera ray, nearest the
   vehicle. `target_manager` uses the cluster's time window and region as the re-acquisition gate.

**Conditioning** lives in `target_manager` + `follow_planner` behind **one parameter,
`loss_conditioning`**, set on both nodes — so both ablation arms are the same binary and
`loss_conditioning:=false` **is** the paper's carry-forward arm, which is what the system did
before M6 existed. `OCCLUDED` coasts to the belief's window (cap 3.0 s, never shorter than the
plain coast) with the re-emergence region as the re-acquisition gate; `LOST`/`UNMAPPED` end the
coast after the verdict has stood **0.3 s**.

**Two pre-registered deviations from the plan, both stated here so they are not silently absorbed:**
(a) `LOST` ends the coast after 0.3 s rather than immediately — 38 of `run5`'s 48 coasts are
one-frame `LOST`s that re-acquire in 0.1–0.3 s, so "immediately" would be a glitch amplifier;
`lost_confirm_s` is the knob that trades glitch tolerance for early `LOST`, and M6.3 sweeps it.
(b) The `OCCLUDED` coast is never shorter than the carry-forward coast, so the conditioned arm
cannot lose on a case the baseline would have held through.

**Three ablation switches, which is the point.** `loss_conditioning`, `rollout_collision` and
`hidden_look` are each one binary, off in code and on in the flight configuration, so Table IV can
report the belief with and without each.

**Evidence.**
- `loss_oracle_check.py` **56/56** including scenes O8 (round a trunk), O8b (start inside one) and
  O9 (stay behind one), each run off and on with `--dump/--compare` pairing.
- `loss_conditioning_check.py` **16/16** on the real `target_manager` + `follow_planner`, both
  arms, with the script playing the oracle. `test_target_manager_loss.py` **27/27** pure.
- **The defect step 3 removes, measured:** without it the dominant re-emergence point sits 0.03 m
  from a mapped cell of the trunk the walker's path crosses; with it every re-emergence point and
  the yaw target keep ≥ 0.19 m from every mapped cell, dominant side unchanged.
- **Step 4's trigger is well separated, measured:** behind an r 0.3 m trunk at 1 m/s only 0.31–0.59
  of rollouts stay hidden and the cluster viewpoint already looks behind it; behind an r 0.5 m
  trunk at 0.7 m/s, 0.62–0.84 stay hidden and the look point fires 12/12 with the goal 1.0 m off
  the camera–trunk line. **The plan's premise — that most rollouts stay hidden behind a trunk —
  was wrong for the smaller trunk, and the threshold was measured rather than assumed.**
- **Closed-loop replay exists** (`loss_closed_loop_replay.py`): `fsm_node` + `target_manager` +
  `follow_planner` on a bag with the oracle answering live. On `run5`, OFF 65/66 re-acquired, ON
  61/64, with 3 coasts ended by the oracle and 2 re-locks via the region. **On a bag whose losses
  are detector glitches rather than occlusions, the conditioned arm's cost is visible and its
  benefit is not** — which is exactly why M6.3 exists.

**What closes it.** **M6.4 exit:** on E1 events, correct edge side ≥ **80 %**, median
|t_pred − t_obs| ≤ **0.5 s**, median position error ≤ **0.5 m**, and the 90 %-mass region contains
the true re-emergence ≥ **80 %** of the time. **D2:** open-loop replay on E1 gives a region-gated
re-lock rate ≥ band rate + **15 points**, **0** wrong locks in both arms, ABSTAIN count ≤ band's
+ 10 %. Then **ablation 1** in flight: median re-acquisition time on E1, ON vs OFF, ≥ 30 usable
events per arm.

**Known open item.** `hidden_look_min_w` has **no hysteresis** — a stream straddling 0.5 will flap
between the cluster mean and the look point. If flight logs show it, add hysteresis and say so.

---

### C3 — Marker-free LiDAR-and-RGB-D human following
**Status: FLOWN 2026-09-15. Not yet evaluated against the M5 gate.**

**The claim.** Prior LiDAR aerial tracking — including the planner we build on and its swarm
extension — instruments the target with retroreflective tape, which makes its occlusion story
untested by construction: the target is trivially detectable whenever it is visible at all. We
track an **unmarked** human.

**What exists.** The full stack in [`drone-dtp/`](drone-dtp/) (`dtp_perception`, `follow_planner`,
`command_mux`, `dtp_bringup`, `dtp_msgs`): YOLO11s + BoT-SORT on the Orbbec Gemini 336L, depth
lifted into `lidar_init`, one locked target with identity persistence and abstention under
ambiguity, a standoff viewpoint, and a moving goal into the patched SUPER. The organized cloud is
built on the GPU (`cloud_backend: cuda`), bit-identical to numpy on 200/200 real frames, with an
in-node self-check and a numpy fallback.

**Flight evidence.** 2026-09-15: three Offboard segments totalling **87 s**, `FOLLOW|OK` held
**103 consecutive seconds** — the mux arbitrating real follow goals into SUPER while armed, which
had never happened before. Armed 123 s, max altitude 2.44 m, no abort, no fence clamp, no EKF
reset, no FC fault.

**What closes it.** The **M5 gate**: ≥ 20 min autonomous following at S3 (1.5 m/s) over ≥ 5
flights; horizontal distance to the subject ≥ 2.5 m **100 %** of LOCKED time and in [2.5, 4.5] m
≥ 90 %; bearing error median ≤ 15°, p95 ≤ 35°; **0** proximity-veto trips from the subject walking
normally; **0** stack-caused interventions; **0** new FC faults.

**The honest caveat this claim carries.** Active-stereo depth under canopy and in direct sun is
**uncharacterised** (M5.4: pass needs ≥ 50 % of box pixels returning at 4 m in sun, else the
follower moves to the LiDAR depth path). And the depth patch at 0.65 of box height lands on the
thigh of a standing person, not the torso the code's comments describe — it read the **shin** on
the 2026-09-15 flight, which is why that sortie's follow goal was low. Both belong in Limitations.

---

### C4 — Forest evaluation with an uninstrumented human subject
**Status: PLANNED. Highest schedule risk in the project.**

**The claim.** A controlled tube course *and* a real forest segment, with a subject wearing no
marker, no tag and no beacon — evaluated against three baselines.

**Design.**
- **Course:** 8 tubes, 30 cm diameter, ≥ 1.8 m tall, two staggered rows 2.5 m apart, 2.0 m between
  tubes, each position recorded to ±5 cm. The course is what gives reviewers the **controlled
  trunk statistics** they ask for; the forest segment is the replication.
- **Handheld dataset first (M6.3):** carry the powered, disarmed rig at ~1.5 m while the subject
  walks scripted events. **140 events** — E1 walk behind a tube (`OCCLUDED`, n = 60), E2 walk away
  > 10 m in clear view (`LOST`, n = 40), E3 leave the camera cone (`OUT_OF_VIEW`, n = 20), E4
  crouch or turn in clear view (detector-FN `LOST`, n = 20) — ≈ 95 min of recording. **This answers
  "is there a paper" before any evaluation flight.**
- **Flights:** arms interleaved **ABBA** by flight. ≥ **30 usable E1 per arm** on the course →
  two-sided Mann–Whitney at α = 0.05, power 0.8, detects a standardised effect **d ≥ 0.74**.
  ≥ **15 per arm** in the forest, reported as replication (d ≥ 1.05), not as the primary test.
  **Pre-registered primary endpoint: median re-acquisition time on E1.**
- **Baselines:** (1) SUPER stock — a `follow_planner` mode with a goal behind the current position,
  CV model, no coast, no abstain (**BUILT** as `stock_baseline` on both nodes); (2) carry-forward
  = `loss_conditioning:=false` (**BUILT**, and it is what the system did before M6); (3)
  Eva-Tracker's loss policy re-implemented as `baseline_mode: eva` (**PLANNED**); (4) ours.
- **Ground truth:** scripted events called aloud, filmed from a tripod, synced by a clap.
- **Season:** the forest flights will be **leaf-off**. Report the season. The handheld tree segment
  and the outdoor depth test are placed before mid-October, while the canopy is still on.

**What must happen first.** Gate 0 (airworthiness) and the M5 gate. The one Gate 0 item still open
is G0.1: the `uxrce_dds_client` autopilot hardfault — 22 classic field crashes at a single program
counter, reproduced on demand, with a client-side fix flashed that has passed **486 clean
session-loss cycles with 0 resets**. **The 14 h powered soak and the shell-path reproducer on that
build are still owed, and until they pass this is a flight blocker.**

---

### C5 — A layered safety architecture in which follow behaviour cannot outlive its evidence
**Status: BUILT + BENCH-GATED. Partly flown. Induced-failure experiments PLANNED.**

**The claim.** Three properties, composed, make an autonomous follower safe to fly near the person
it is following — and give the loss response for free.

1. **One publishing path.** Exactly one function may emit a goal, from a single periodic timer,
   because a second path is a path that can bypass the veto.
2. **The veto is independent of the follow product.** The human-proximity veto (1.5 m) subscribes
   to the **raw** detection stream — every person, every frame, including empty frames — so a
   defect in viewpoint geometry cannot defeat it.
3. **Silence is not a stop at the goal layer.** A stored goal keeps the vehicle flying, so every
   stop condition **publishes** a hold near the current position rather than going quiet.

Arbitration is `Safety ≻ Manual ≻ Follow ≻ Nav`, with manual authority enforced by the autopilot.
On top: the **land-on-blindness ABORT**, and a **goal-level geofence** in `command_mux` — PX4's
`GF_MAX_HOR_DIST` is inert without a global home, and `EKF2_GPS_CTRL=0` gives none, so the box
lives where the goals are made. Soft boundary clamps FOLLOW, NAV **and** HOLD goals 0.5 m inside
the box; hard boundary (edge + 3 m) escalates to NAV_LAND through the abort path.

**Evidence.** `fence_check.py` **13/13** on the real node (clamp, soft breach, hard breach →
`HOLD|FENCE` → `ABORT|LAND` with NAV_LAND observed, latch, refusal to start on a box excluding the
origin); `test_fence.py` 10 and `test_abort.py` 12 pure; `m3_bridge_check.py` **32/32**;
`m3_gate_check.py` **7/7** on hardware with the pilot driving the RC, including manual takeover
verified twice. Flown behaviour: the proximity veto fired correctly on the 2026-09-15 flight when
the walker closed to 1.1 m.

**What closes it.** **M5.7**, and Sec. VI-D of the paper: occlude the subject; kill perception
mid-follow; introduce a second person inside the re-acquisition band; starve the state estimate.
Show the commanded response is the designed one in each of the four. This is cheap to run and it
is what distinguishes the paper from tracking-accuracy-only submissions.

**Positioning note.** Under the current thesis this is a **secondary** contribution. Keep it — it
is what makes flying near the followed human defensible — but it must not crowd C1. Cut to half a
column if page-limited.

---

## 2. Enabling contributions — reportable in their own right

These came out of making C1 and C2 work. Each is a small, general, checkable result.

### S1 — The goal-acknowledgment contract
**BUILT, FLOWN.** Safety-assured planners do not accept arbitrary goals: ours may accept a goal,
relocate it to the nearest free cell, or reject it and keep the previous one — **reporting none of
these**. Benign for a static waypoint; for a continuously moving standoff goal the follower
believes the vehicle is heading somewhere it is not. We extend the planner to preserve commanded
altitude, emit an explicit `ACCEPTED`/`SNAPPED`/`DROPPED` acknowledgment per request, and **echo
the requested point** — necessary because under `DROPPED` the reported active goal is the previous
one, geometrically unrelated to the request, so without the echo a subscriber cannot tell which
request an acknowledgment answers. Patch 0001. **The same lesson produced `LossEvent.query_p`.**

### S2 — Visible-but-unhit decay for raycast-cleared occupancy maps
**BENCH-GATED (10/10), REPLAYED, ran live on the ground 2026-09-15; its effect on a walker is
unscored.** Full record: [`docs/fact-rogmap-unhit-decay.md`](docs/fact-rogmap-unhit-decay.md).

**The failure mode, which is general and which we can state cleanly: no return, no ray.** A map
casts a clearing ray only to a point that **returned**. With the aircraft on the ground the LiDAR
is 0.15 m up, every part of a person is above the horizon, and the beams through them go into the
sky — so the cells a person occupied are never swept and never cleared. Measured on a real bag: of
457 body cells, **113 were still occupied 7 minutes after the walker left**, median 12 rays through
them (30 with none) against 718 for the cells that did clear. Where a backdrop existed — a hedge at
10 m — the same walker's cells cleared in **1.5 s**. It is not tunable (`p_max`/`p_miss` need rays)
and not recoverable from the sensor.

**The fix, `rog_map/unhit_decay`** (patch 0007): every 0.5 s, an occupied cell within 8 m, inside
the sensor's vertical FOV, with no *observed* occupied cell on its line of sight, that **no
ordinary ray has reached for 3 s, hit or miss**, receives as many misses per pass as rays the
sensor should have sent through it. Replay: **338 → 83 cells 2 s after the walk, 10 at 30 s, 3 at
7 min** (was 113). Pass cost 3–4 ms, max 9.9 ms.

**Three rules learned on the gate, each of which is a claim worth one sentence in the paper:**
UNKNOWN must not block the view, or tangent cells stall; an unobserved occupied cell must be
transparent to the cells behind it, or clearing takes 26 s instead of 7; **any** ray must reset the
clock, not only a hit, or far grazing ground is double-counted and eroded.

### S3 — Keeping the followed person out of the map that classifies their occlusion
**BUILT + BENCH-GATED (17/17), REPLAYED.** `rog_map/target_exclusion` (patch 0002): inside a
cylinder at the locked target's predicted position (r 0.6 m, z ∈ [−1.0, +1.3], centred on the
**waist** — perception's target z is 0.65 of the box from the top, which the plan got wrong), a
scan inserts **no hits and no misses**. Coasted and held frames are never used; an observation
older than 0.5 s is dropped. Replay on `run5`: line-of-sight crossings of the subject's own trail
**2.6 % → 0.7 %**. **The stated cost: with it on, the planner does not avoid the followed person**,
so the Stage A preflight fails on it and enabling it in the flight config waits on a hardware walk
test. This is the mechanism that answers the "trail false-`OCCLUDED`" objection to C1 directly.

### S4 — An upstream ray-clipping defect in the occupancy map
**BENCH-GATED, deployed.** `ProbMap::raycastProcess` clipped a ray to a return above the virtual
ceiling or below the virtual ground to a point **within 1 m of the sensor**, so the ray cleared
nothing. It bites whenever the floor is more than 1 m below the map origin — a bench, a table,
terrain dropping away — and a person's lower body then **never** clears. Measured on the real node:
stock left **27** cells that never cleared; fixed left **0**. Patch 0003.

### S5 — Free-end yaw on a short trajectory
**BENCH-GATED (7/7), FLOWN and held.** In the first Offboard flight the aircraft pirouetted for the
whole 44 s: with the mux in HOLD re-sending its goal at 2 Hz, a free-end yaw aligns to the
trajectory's displacement, which on a hold correction **is the hover wobble**, re-planned every
0.5 s — 52 °/s median, 129 °/s peak. Patch 0006 holds the current heading when the whole trajectory
moves < 0.3 m. The gate reproduces the flight on the old binary (394° net turn) and shows 0.0° on
the new one while still turning 0 → 92° for a real 3 m move. **Flown 2026-09-15: yaw rate median
52 °/s → 0.0–1.2 °/s.** Record: [`docs/fact-super-yaw-hold.md`](docs/fact-super-yaw-hold.md).

### S6 — An offline predictor study with four results that are not the obvious ones
**DONE (off-aircraft).** [`docs/m6.6-predictor-study.md`](docs/m6.6-predictor-study.md). Full
ETH/UCY under the TrajImpute missing-observation protocol, 33 654 trajectories.

1. **Best-of-1 from a learned multimodal model is a *selector* problem, not a model problem.** The
   flown adapter's raw sample 0 scores **1.39 / 2.88 m** ADE/FDE — worse than constant velocity
   (**0.52 / 1.14**). Best-of-20 (**0.23 / 0.38**) is an oracle a single-goal follower cannot use.
2. **The "needs a complete history" sensitivity is the *imputation*, not the model.** Linear
   interpolation holds 0.54 m ADE from 0 % to 50 % missing; the repo's forward-fill degrades to
   1.44.
3. **The flown CTRV code cannot run verbatim on 0.4 s-spaced ETH/UCY** — its heading baselines need
   > 8 points, giving 100 % fallback — so Table II's CTRV row is a gap-native least-squares fit of
   the same model, **labelled as such**.
4. **CTRV ≈ CV on a 4.8 s horizon**, and the plan's α schedule changes ADE by ≤ 0.07 m.

**Consequence for the paper:** fly **CTRV only**; the learned/fused branch is an
**offline-evaluated option**, and `eigen_enabled` stays off in flight. A separate negative result:
the AGX's EigenTrajectory assets, checked 2026-09-14, are byte-identical public weights whose
wrapper returns NaN for any pedestrian group containing a standing person — **nothing to adopt**.

---

### S7 — A receding-horizon follow goal, and a visibility field a planner can publish (CANDIDATE)
**BENCH-GATED, REPLAYED (Stage 1); BENCH-GATED only (Stage 2). Never flown, not installed.
Not in `main.tex` — whether it becomes a claim, an arm or a footnote is the authors' call.**
Added 2026-09-19/20; record [`docs/fact-nmpc-follow-goal.md`](docs/fact-nmpc-follow-goal.md).
- **What it is.** The standoff goal a follower hands a safety-assured planner is usually
  re-decided from nothing every tick. Here it is a receding-horizon optimisation over the *goal's*
  positions (not the aircraft's — the planner already models the aircraft), behind the same hard
  guards as the closed form, falling back to the closed form on any refused solve. Stage 2: the
  planner's own map, which no other process can query, publishes a ~300-byte per-azimuth
  visibility field, and the follower moves its trail bearing out of a known shadow.
- **Evidence.** Replay of run5, two runs per arm, open loop: direction reversals of the goal
  52 → 15–18, goal step p95 0.73 → 0.33 m, band occupancy unchanged. Stage 2 end to end on a
  synthetic scene, real `fsm_node` → real `follow_planner`, scored against the scene's geometry:
  a tube dead between aircraft and person; the closed form parks in its shadow, Stage 2 ends
  ~1.0 m clear of it (four runs). `LossEvent` stream unchanged by the patch (30/30).
- **What would retract it.** A flight where the nmpc arm's goals are SNAPPED/DROPPED more than
  the geometric arm's (M5.1 props-off is the first place a real `GoalStatus` answers that); a
  Stage A leg where its standoff error or smoothness is worse; Stage 2 on a REAL map showing
  the field is "unknown" wherever it matters.
- **Negative results worth a sentence each** (all measured): a visibility *cost* alone cannot
  move a goal out of the middle of a shadow — zero slope, and in a wide shadow the point is
  stable — so a fix that relied on it passed or failed run to run on the same binaries; one ray
  per 15° bin misplaces a trunk's ~11° shadow by 7° and sent the goal *deeper* into it; the field
  knows occlusion only near where the aircraft has looked from (open scene: 81 % unknown); and
  Stage 1 inherits heading flips from the closed form (both arms take the same 4–7 m goal jumps
  when a slow walker's heading estimate flips).

## 3. Negative results and stated preconditions

`main.tex` Sec. VIII exists because ICRA rewards these when they are specific and generalisable.
Format each as *symptom → why it was invisible → what it implies*. **Cut this section first if at
6 pages.** The first three below are preconditions of C1 and belong beside Eq. 1 regardless.

### N1 — An occupancy map cannot forget a person, and that corrupts the classifier
Cells are freed only by raycasting, which is off by default in the high-rate configuration, so a
passing person is inserted as permanent occupancy and a **followed** person stamps a wall behind
themselves along the corridor the vehicle must fly — whereupon Eq. 1 queries that stale occupancy
and reports `OCCLUDED` against the target's own history. **Implication:** dynamic-obstacle handling
is a **precondition** for aerial following, not an enhancement. Even with raycasting on, clearing
is slow by design: at `p_max` 0.95 a saturated cell needs ~31 clean rays to leave `OCCUPIED` and
74 to become `FREE`, reading `UNKNOWN` in between. S2, S3 and S4 are the three answers.

### N2 — The sensor's vertical field of view bounds what is classifiable
**Found on a bench replay, and it invalidated our first D1 attempt.** The MID360 looks no lower
than **7°** below its horizon, so at range *d* nothing below `z_sensor − d·tan 7°` is ever
observed — 0.5 m under the sensor at 4 m. The oracle now drops sample heights below that floor
(`loss_oracle/fov_floor_deg`) and warns loudly. **The statement for the paper: an occluder is
classifiable only where it rises above the sensor's floor at its range.** Trunks and the 1.8 m
tubes do; a chest-high cabinet seen from head height at 4 m does not. On `run5` the honest verdict
at the only knowable height is `LOST`, and **E1 recall there is 0.11–0.13** — the dataset, not the
classifier, is what fails.

### N3 — The map's virtual ground, and a protocol change that follows from it
ROG-Map's virtual ground sits 0.5 m below the map origin and everything under it reads `OCCUPIED`.
A rig started at 1.5 m puts the subject's waist a metre below that plane. **M6.3 protocol: start
the odometry with the rig ON THE GROUND, then lift it.** Bags already recorded the other way take
`--virtual-ground` on the replay tools.

### N4 — What a surface map can vouch for
The map holds an obstacle as the **shell of the face the sensor has seen**: a 0.6 m trunk at 4 m
was six cells per height, its back half, interior and tangent zones `UNKNOWN`. So C2's step 3 keeps
the belief out of the *mapped* body of an obstacle and out of **nothing the map has never seen** —
a rollout that gets round the lit face's edge walks through the unseen back unopposed. A trunk
mapped from two sides is a closed shell; from one side it is a wall. **General to any surface map**,
and it is why the gate's assertions measure distance to occupied cells, not to the geometric trunk.

### N5 — Latency no consumer detects
Odometry produced correct values with correct timestamps that were **≈ 1.4 s old**. Every
downstream freshness check used message **arrival** time, so nothing reported a fault, and a
stationary vehicle is indistinguishable either way. Corrected to **49–57 ms**. **Implication:**
validate freshness against the **stamp**, on a **moving** platform — and note that fixing latency
by shrinking a queue converts a queuing failure into a dropping failure **that a bench cannot
exercise**, which is why the moving-rig latency check is still owed.

### N6 — Frame conventions fail silently
Every frame error we made produced a plausible, self-consistent, **wrong** system: a body origin
that was the sensor's internal IMU; an estimator delay double-counting a latency already carried in
the timestamp; a yaw convention differing from the obvious guess by 90°; a mounting yaw
**unobservable from the estimator that consumes it**. None raised an error. **Implication: assert
against the *plausible* wrong answer, not merely against a wrong one.** (The MID360→FC yaw mounting
offset is still unmeasured and is a mechanical check — it is one of the two remaining flight
blockers.)

### N7 — Middleware determines architecture
Process boundaries were dictated by transport bandwidth, not design preference — and a blocking
DDS send inside a publish callback took odometry from 10 Hz to 1.9 Hz when a laptop subscribed to a
dense cloud over Wi-Fi. **Implication: measure the transport before drawing the node diagram.**
Record: [`docs/fact-fastdds-blocking-send.md`](docs/fact-fastdds-blocking-send.md).

### N8 — The companion link is flight-critical and unprofiled
**22 autopilot hard faults across two firmware builds at a single program counter** in the
companion-link client, traced to a use-after-free in the serial driver's close/reopen path — i.e.
triggered by the link loss most likely to occur when something else has already gone wrong. A
client-side fix that never closes the UART across a session loss has passed **486 induced session
losses with 0 resets**; the 14 h soak is still owed. **Decide before submission: paper, footnote,
or cut.** It is honest and unusual to report, and it may read as off-topic for a tracking paper.
**Report the fix only once the soak passes.**

---

## 4. The experiments that substantiate the claims

| Study | Closes | Where | Status |
|---|---|---|---|
| Offline prediction under induced missing observations (TrajImpute protocol) | S6, Table II | off-aircraft | **DONE** |
| M6.3 handheld dataset, 140 scripted events, labelled → **D1** | C1 | tube course + trees | **NOT STARTED** — the single highest-value item |
| M6.4 reappearance accuracy on E1 | C2 | same dataset | after M6.3 |
| M6.5 open-loop + closed-loop replay → **D2** | C2 | bags | harness **BUILT**, run on unlabelled bags only |
| **Ablation 1 — conditioning ON vs OFF** | C1+C2 | course flights | **the primary test.** ≥ 30 usable E1/arm, ABBA |
| Ablation 2 — learned vs CTRV vs fused during occlusion | S6 | offline | **DONE** |
| Ablation 3 — reappearance as a point vs a region | C2 | offline, from M6.3 | after M6.3 |
| Ablation 4 — ρ\*, ray count, `r_excl` sensitivity | C1 | offline, from M6.3 | after M6.3 |
| M5.7 safety layer under four induced failures | C5 | props off + in air | **NOT RUN** |
| M6.7 forest replication | C4 | forest, leaf-off | after the course |

**The decision point:** if ablation 1 does not separate cleanly on re-acquisition time, this is an
**RA-L systems paper, not an ICRA algorithms paper**. That call is scheduled for **2026-11-10**,
before the course field days, and D2 is the evidence it rests on.

---

## 5. Claim changes proposed against `main.tex` — the authors' call, not applied

`docs/m5-m6-plan.md` §1 maps all 14 claims. Four need an author's decision, and two are plain
factual updates that should just be made:

1. **Drop live Eva-Tracker as a flight baseline.** Its code is ROS 1 Noetic + MAVROS + MonoLoco,
   AGPL-3.0 — porting is weeks of work unrelated to the thesis. **Re-implement its loss policy** as
   `baseline_mode: eva` (Bézier-QP prediction, 2.5 m standoff from the predicted point, arc search
   for a free line of sight, no verdict) and its predictor as an offline arm. Assessment:
   [`docs/eva-tracker-faster-lio-assessment.md`](docs/eva-tracker-faster-lio-assessment.md).
2. **Replace RTK ground truth with scripted events + synced tripod video.** There is no RTK kit in
   this project's inventory, and consumer GNSS on the subject is 1–2 m — useless for "behind a
   30 cm trunk". Say the mechanism explicitly, or a reviewer will assume we cheated.
3. **Fly CTRV only.** The learned/fused predictor is an offline-evaluated option (S6).
4. **Define "search" as hold-and-climb + operator re-designation**, which is what the mux does.
   Autonomous re-lock after `LOST` is how a drone starts following a stranger.

**Factual updates:** "11 autopilot hard faults" → **22** classic field crashes (35 dumps including
bench-induced); "within one 15 Hz perception frame" → **8 Hz (125 ms)**.

---

## 6. What we explicitly do not claim

- **Not a new planner.** SUPER is the base; our patches add a goal contract, a loss oracle, a
  belief, a map decay rule and a yaw hold. The trajectory optimiser is theirs.
- **Not a new detector or tracker.** YOLO11s + BoT-SORT off the shelf.
- **Not a new odometry.** FAST-LIO2, patched for latency and frame naming only.
- **Not a learned occlusion model.** The whole point of C1 is that no learning is required.
- **Not multi-target.** Single subject; group following and handoff are out of scope.
- **Not appearance-based re-identification.** Re-identification is geometric, and abstention under
  ambiguity is the correct conservative behaviour — but it ends follows an appearance model could
  continue, and we should quantify how often.
- **Not a consent mechanism.** The system distinguishes the locked subject from bystanders and
  keeps distance from both, but "the person the operator engaged on" is not "a person who agreed to
  be followed by an aircraft". The proximity veto is the only mechanism in the system that protects
  a non-consenting bystander. This is a real limitation of aerial following generally and we say so.

---

## 7. Status ledger

| # | Contribution | Status | Gate / evidence | What closes it |
|---|---|---|---|---|
| C1 | Loss-cause classification | BENCH-GATED, REPLAYED, live n=1 | `loss_oracle_check.py` 28/28 → 56/56; p99 81 µs | **D1** from M6.3 |
| C2 | Reappearance belief + conditioning | BENCH-GATED, REPLAYED both arms | `loss_conditioning_check.py` 16/16; 27 unit tests; closed-loop replay | M6.4 exit, **D2**, ablation 1 |
| C3 | Marker-free LiDAR+RGB-D following | **FLOWN** 2026-09-15 | 87 s Offboard, `FOLLOW\|OK` 103 s | **M5 gate** |
| C4 | Forest evaluation, uninstrumented subject | PLANNED | — | Gate 0 → M5 → M6.7 |
| C5 | Layered safety architecture | BUILT, BENCH-GATED, partly flown | `fence_check` 13/13, `m3_gate_check` 7/7 | M5.7 induced failures |
| S1 | Goal-acknowledgment contract | BUILT, FLOWN | patch 0001 | reported, not gated |
| S2 | Visible-but-unhit decay | BENCH-GATED, live on ground | `unhit_decay_check.py` 10/10 | walker test on hardware |
| S3 | Target-point exclusion | BENCH-GATED, REPLAYED | `target_filter_check.py` 17/17 | hardware walk test |
| S4 | Ray-clip defect fix | BENCH-GATED, deployed | 27 → 0 stale cells | — |
| S5 | Yaw hold on short trajectory | BENCH-GATED, **FLOWN and held** | `yaw_hold_check.py` 7/7 | walker-bearing correlation from the bag |
| S6 | Offline predictor study | **DONE** | 33 654 trajectories, ETH/UCY | Table II is fillable now |
| S7 | Receding-horizon follow goal + visibility field (**candidate, not in `main.tex`**) | BENCH-GATED; Stage 1 REPLAYED; **not installed** | `nmpc_check.py` 41/41, `visibility_field_check.py` 13/13, gtest 15/15; run5 replay ×2 per arm | baseline flight → Stage A two legs → M5.1 `GoalStatus` mix; Stage 2 on a real map |
| N1–N8 | Preconditions and negative results | measured | the `docs/fact-*.md` series | — |

---

## 8. Maintaining this file

- A claim moves status **only** when a named gate or a scored bag says so. "It ran" is not
  "it passed"; "it passed on the bench" is not "it flew".
- When a `\bench{}` figure in `main.tex` is replaced by its in-flight equivalent, update the row
  here in the same commit.
- New contributions get a row in §7 **and** a paragraph in §1 or §2, with the gate named. A
  contribution with no gate is a hypothesis.
- If a claim is retracted, leave the row and mark it **RETRACTED** with the date and the reason.
  This project has retracted two "dead wire" conclusions and one hardfault reading already; the
  record of *why* was worth more than the tidy list.
