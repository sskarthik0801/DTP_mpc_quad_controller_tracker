# `docs/paper/` — ICRA / RA-L draft

**Status: skeleton + method + system characterization. No flight results.**

- `main.tex` — the paper (788 lines).
- `references.bib` — 71 entries, from the annotated bibliography.

## The thesis

> Every prior aerial-tracking system treats target loss as **one** undifferentiated
> event answered by **one** policy: extrapolate and keep flying. We classify the
> **cause** of loss against the onboard-built 3-D map — occluded by known geometry
> vs. genuinely lost — and condition planner behaviour on that classification. The
> geometry needed is already computed as a by-product of obstacle avoidance.

Secondary claims: marker-free LiDAR+RGB-D human tracking (prior LiDAR aerial
tracking, including SUPER's own demo and the HKU swarm work, uses retroreflective
tape); first forest evaluation with an **uninstrumented** human subject; and the
layered safety architecture.

## Building

No TeX toolchain on the Jetson, so **this has never been compiled.** Build on
Overleaf (pdfLaTeX) or `latexmk -pdf main.tex`. Environments and braces are
balanced and every `\cite` key resolves, but expect first-build fixes.

`IEEEtran`/`conference`. For ICRA's own template swap in
`\documentclass[letterpaper, 10pt, conference]{ieeeconf}` and drop
`\IEEEoverridecommandlockouts`. **ICRA = 6 pages** (+2 paid); **RA-L = 8**. The
draft is written to the 8-page shape — cut Sec. VIII (Lessons) first for ICRA,
then halve Sec. V-H (Safety layer).

## Drafting macros

| Macro | Renders | Means | Count |
|---|---|---|---|
| `\tbd{...}` | red | not written or not measured yet | 79 |
| `\bench{...}` | blue | **real measurement, but bench/handheld — never in flight** | 29 |
| `\note{...}` | gray | instruction to ourselves; delete before submitting | 18 |

`\bench` exists so a bench number cannot be quietly promoted to a flight result
while editing. Drop the wrapper only when the figure is replaced by its in-flight
equivalent. Strip the colors before submitting.

## Read these six first

Priority order from the bibliography: **Eva-Tracker** (the one a reviewer will
compare you to), **BPOD**, **Scene Informer**, **TrajImpute**, **Elastic Tracker**,
**Adaptive Tracking and Perching**. Sec. II-D of the draft has a
one-sentence-per-system differentiation paragraph; sharpen it once you have read
all four Tier-0 papers in full, because that paragraph is where the paper is won
or lost.

## The plan that closes these

**[`../m5-m6-plan.md`](../m5-m6-plan.md) (2026-09-12)** turns the list below into dated
rows with pass thresholds.
- **§1** maps every claim in `main.tex` to what is doable.
- **§2.3** has the sample sizes: ≥ 30 usable occlusion events per arm for ablation 1, which detects
  d ≥ 0.74.
- **§5** designs the classifier.

**The contribution itself — loss-cause classification, reappearance belief, planner conditioning —
was BUILT on 2026-09-14 and bench-gated; nothing has flown.** The as-built record is
[`../m6-loss-oracle.md`](../m6-loss-oracle.md) (SUPER patch 0004 = the oracle, patch 0005 = the
belief's obstacle-aware and look-behind steps, below). The follower's CTRV coast is still the
carry-forward ablation arm (`loss_conditioning: false`); the stock follower is `stock_baseline`.

## The reappearance belief as built — what Sec. V-E should say (2026-09-14)

Sec. V-E's `	bd{Write the goal-generation math}` is now drafted in `main.tex` from the code,
every number wrapped in `ench{}`. The method the paper can claim, in the order the code runs it
(all inside SUPER's `fsm_node`, reading the planner's own ROG-Map; `super_core/loss_oracle.hpp`):

1. **Occluder extent.** From the blocked ray nearest the target's centre, sweep the bearing left
   and right in one-cell angular steps at the target's range; three consecutive free columns end
   the occluder → its two edges and an angular interval.
2. **Rollouts.** K = 32 constant-turn-rate samples from the coasted state (speed, turn rate and
   heading each drawn once per rollout), 3.2 s at 0.1 s. A rollout re-emerges at its first point
   outside the occluder's interval with a camera ray containing no occupied cell. Exits are
   clustered by side (mean, spread, arrival-time min/median/max, mass); the mass that never exits
   is the *still-hidden* mass. Seeded per event, so a replay reproduces the belief.
3. **Obstacle-aware rollouts (B, `rollout_collision`, patch 0005).** Rollouts cannot cross an
   occupied cell of the raw grid. Each step is sub-stepped per cell; the walker's body is sampled
   every half cell along its lateral diameter (±0.25 m) and the radius ahead, at every height the
   map can know; a blocked sub-step deflects ±15°…±90° (own turn side first) *for that sub-step
   only*, so the rollout slides round the obstacle and resumes its course. A start inside an
   obstacle is backed off. A cluster mean that lands in occupied cells is replaced by the
   cluster's medoid. No new random draws, so the switch off reproduces step 2 bit for bit.
4. **Look-behind (B2, `hidden_look`, patch 0005).** If the still-hidden mass is ≥ 0.5, or no
   rollout exits, the yaw target is the hidden rollouts' mean position at 1.5 s and the viewpoint
   search runs about that point instead of a cluster mean. The exit clusters are untouched.
5. **Reduction** to what the planner consumes: yaw target = the dominant cluster's mean (or the
   look point); goal = the candidate on the [3, 4] m annulus about it, outside inflated occupancy,
   ≥ 2.5 m from the hidden person, with a clear camera ray to the point, nearest the vehicle.
   `target_manager` uses the cluster's time window and region as the re-acquisition gate.

**Two ablation switches, which is the point for the paper:** each of steps 3 and 4 is its own
parameter, off in code, on in the flight config, so Table IV can report the belief with and
without each. Both arms are one binary.

**The precondition to state with it (found on the gate, not assumed).** The map holds an
obstacle as the shell of the face the sensor has seen: a 0.6 m trunk 4 m away was six cells per
height, its back half, interior and tangent zones unknown. Step 3 therefore keeps the belief out
of the *mapped* body of an obstacle and out of nothing the map has never seen — a rollout that
gets round the lit face's edge walks through the unseen back. A trunk mapped from two sides is a
closed shell; from one side it is a wall. Say this next to the raycasting precondition in Sec.
V-D; it is general to any surface map.

**Bench figures available now (all `ench`, from `loss_oracle_check.py` on the real planner
process with synthetic LiDAR scans; `docs/m6-loss-oracle.md` "M6.4b"):**
- Cost: classification p99 108 µs; full evaluation with the belief, both switches on, p99
  2.5 ms (0.4 ms without step 3) — Sec. V-D's "why this is nearly free" `	bd`.
- The defect step 3 removes: without it the dominant re-emergence point sits *on* a mapped cell
  of the trunk the walker's path crosses (0.03 m from one); with it every re-emergence point and
  the yaw target are ≥ 0.19 m from every mapped cell, the dominant side unchanged.
- Step 4's trigger, measured: behind an r 0.3 trunk at 1 m/s only 0.31–0.59 of the rollouts stay
  hidden (the CTRV noise spreads them out of a 0.6 m shadow within 2–3 s) and the cluster
  viewpoint already looks behind the trunk; behind an r 0.5 trunk at 0.7 m/s 0.62–0.84 stay
  hidden and the look point fires 12/12, with the viewpoint 1.0 m off the camera–trunk line.
- The switches-off belief is identical to the pre-0005 build in 36/36 events, field for field.

**Still `	bd`:** everything in flight (D1 on the M6.3 handheld dataset, D2 on flights); the
`\lost{}` search policy; whether the reduction flaps when the hidden mass straddles 0.5 (no
hysteresis today).

Four claim edits are **proposed there and deliberately not applied** to `main.tex`, because they
are the authors' call:
1. Drop live Eva-Tracker as a flight baseline.
2. Ground truth from scripted events plus tripod video, not an RTK tag.
3. Fly CTRV; evaluate the learned predictor offline only.
4. "Search" means hold-and-climb plus operator re-designation.

## Six things blocking submission

1. **Raycasting is ON in the follow config since 2026-09-12 (user's instruction);
   clearing is still unverified.** `follow_test_ros2.yaml` — what
   `dtp_bringup/super.launch.py` defaults to, i.e. the config the contribution runs
   on — now has `raycasting.enable: true` in `src/`, `install/` and
   `drone-dtp/patches/0001-…` (line 201). Before that it was `false`, inherited by
   copy from the bench config; with it off the map only ever *adds* occupancy, so a
   walking person becomes a permanent obstacle **and** the classifier queries that
   stale occupancy.
   **"On" is not enough for this paper.** ROG-Map needs 31 clean rays to take a
   saturated cell out of OCCUPIED at `p_max` 0.95, and 74 to make it free (54/98 at the
   follow config's 0.98). Until then the cell reads UNKNOWN. A subject walking straight away
   leaves that trail on the camera's line of sight, only ~2.9 s old at 3.5 m standoff and
   1.2 m/s. The plan therefore filters the locked target's points out of ROG-Map's input
   (M5.3) and measures trail false-OCCLUDED directly (D1). **Still unmeasured:** the CPU
   cost alongside perception, and clearing time on hardware.
   Sec. V-D states the dependency as a precondition of the method, which it is
   regardless of our config history.
2. **The learned predictor ships disabled** (`eigen_enabled=False`) and its
   checkpoint and config live outside version control. Either enable and
   characterize it, or describe the system as CTRV with the learned branch as an
   evaluated option — the paper must not imply it flew if it did not.
3. **Ablation 1 is the paper.** With/without loss classification, measured on
   re-acquisition time. If it does not separate cleanly, this is an RA-L systems
   paper rather than an ICRA algorithms paper — still a fine outcome, rolling
   deadline. Run it before writing the abstract.
4. **Ground truth for classification accuracy.** `Occluded` vs `Lost` needs a
   labelled reference — an RTK tag on the subject used *for evaluation only,
   never for detection*. Say so explicitly in the paper or a reviewer will assume
   you cheated.
5. **Outdoor and under-canopy depth.** The camera-vs-LiDAR choice rests on indoor
   numbers for an *active* stereo sensor whose projector competes with solar IR.
   Canopy is neither indoor nor open-sky.
6. **Two figures.** The teaser (three panels: camera / map with blocked ray and
   shaded reappearance region / flown trajectory) and the architecture diagram.
   The teaser *is* the argument — worth real time.

## Hardware

**Jetson Orin NX 16 GB** — settled 2026-09-10. The bibliography header said
"Orin Nano Super"; the NX is the module that flies, and it is also the board every
`\bench` figure in Table III was measured on, so the compute table needs no
re-taking.

## Bibliography verification

Three classes, marked in the file:

- Transcribed from your bibliography — low risk, still worth a DBLP pass.
- `% VERIFY` — reconstructed from memory offline; author lists, volumes and pages
  are the risky fields.
- `{FILL AUTHORS}` (**12 entries**) — your bibliography gave a title and venue but
  no authors, and I did not invent any. These will not render until filled.
  Includes BPOD, SVPTO, and the Track A\* / OA-NBV / instance-level 2026 preprints.

Every arXiv ID in the 2602–2605 range is a 2026 preprint whose venue may not be
final. Confirm before citing any as ICRA/IROS/RA-L.

## Where the `\bench` numbers come from

| In the paper | Source in this repo |
|---|---|
| Odometry rate/latency, the ~1.4 s defect | `docs/fact-fastlio-latency.md` |
| Lever arm, mounting tilt | `CLAUDE.md` M2; `docs/fact-ev-frd-yaw-align.md` |
| `EKF2_EV_DELAY = 0` reasoning | `docs/fact-ev-delay.md` |
| Setpoint-bridge cost, yaw sign trap | `CLAUDE.md` M3; `docs/fact-super-cmd-stream.md` |
| Map never clearing a voxel | `CLAUDE.md` M4 blocker; `docs/ImplementationDoc.md` §M4 |
| Perception CPU/RSS, depth coverage, 11 MB cloud | `docs/dtp-follower-stack.md` |
| Loss classifier cost, sensor-floor precondition, run5 / Stage A replays | `docs/m6-loss-oracle.md` |
| Reappearance belief: obstacle-aware rollouts, look-behind, thresholds, the mapped-shell precondition | `docs/m6-loss-oracle.md` "M6.4b"; plan and deviations in `docs/m6-belief-around-trees-plan.md` |
| Calibration solve quality | `docs/dtp-follower-stack.md` |
| Autopilot hard faults | `firmware/faults/README.md` |

## Note on the system being described

The follower stack is `drone-dtp/` in this repository, on branch
`dtp-follower-stack` since 2026-09-12 — **pushed to `origin` (2026-09-13) but not on
`main`** — see `docs/dtp-follower-stack.md` and `drone-dtp/README.md`. Get it merged
(and the GPU-cloud work committed) before publishing about it; ICRA
increasingly expects a code link, and the differentiation against Eva-Tracker is
much stronger if reviewers can run yours the way you can run theirs.

**Numbers in `main.tex` that went stale on 2026-09-12:**
- **Table III perception CPU**, 60.1 %, is not edited; it has been measured since at 137 %
  (8 Hz, field stack) to 250 % (15 Hz). **Since 2026-09-13 the organized cloud runs on the GPU**
  (`cloud_backend: cuda`): a bag replay puts `perception_node` at ~0.7 cores against 1.9 with numpy.
  Re-measure in flight with that backend, and state it in Sec. VI.
- **Sec. VI's "sharing one deprojection implementation, so what is recorded deprojects exactly as
  what is flown" is no longer literally true** with `cloud_backend: cuda`. The flown cloud comes
  from `cuda_cloud.py`, while `fusion_node`'s debug publisher still uses numpy. It is still true
  in effect: bit-identical on 200/200 real frames, and re-checked by the node on its first frames
  every run. Say it that way; a `\note` marks the sentence in `main.tex`.
- **Hard-fault count:** now **22** field crashes at one PC (35 dumps including bench-induced),
  updated in Sec. VIII, which also gains a `\note` on the client-side fix's status.
- **Perception frame:** now **8 Hz** (was 15 Hz), updated in Sec. III and Sec. V-D, along with the
  organized-cloud rate in Sec. VI.
- **Raycasting \tbd in Sec. V-D:** it is now on, so the note is rewritten to say what remains
  (clearing and CPU unmeasured).
