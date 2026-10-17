# M6.4b — rollouts that walk around obstacles (B) and a look-behind viewpoint (B2)

**Status: BUILT 2026-09-14 (same day, later session) — patch
`drone-dtp/patches/0005-super-belief-around-obstacles.patch`, bench-gated, never flown.** The
as-built record, gate numbers and the deviations from this plan are in
[`m6-loss-oracle.md`](m6-loss-oracle.md), section "M6.4b". The sections below are the plan as
written before building; where the build deviated, the reason is in a **Built:** note in place.
The deviations, in one line each:
- **§3.2 probes are half-cell LINE samples, not five points.** ROG-Map holds a trunk 4 m away
  as a one-cell shell of its lit face (six cells per height at 4.2 m, read back on the gate);
  five points 0.25 m apart straddled it and every rollout walked through.
- **§3.4 the deflected heading does NOT persist.** The intended heading (integrating the drawn
  turn rate) is kept and tried again every sub-step, so a rollout slides round the obstacle and
  resumes its course. A persisting 90° deflection would turn a −y walker into a permanent ±x
  walker, which is not "going round".
- **§1/§6.2 what B can guarantee.** A trunk seen from one side is mapped on that side only; its
  back half and interior are UNKNOWN and a rollout may pass there. B keeps the belief out of
  the *mapped* body of an obstacle (≥ body radius − one cell from every occupied cell, measured
  0.19 m vs 0.03 m without B) — not out of the geometric footprint. O8's assertions say that.
- **§6.2 O9's geometry moved.** Behind an r 0.3 trunk at 1 m/s only 0.31–0.59 of the rollouts
  stay hidden (median 0.39) and the cluster viewpoint already looks behind the trunk; the scene
  that matches this plan's premise is an r 0.5 trunk at 0.7 m/s (0.62–0.84 hidden). Both
  `hidden_look_min_w` 0.5 and `hidden_look_s` 1.5 stayed at their starting values; the bracket
  they sit in is recorded in `follow_test_ros2.yaml`.
- **§6.2 pairing** needed deterministic stamps: the harness now stamps scenes from a fixed
  epoch, so the per-event seed repeats across binaries (36/36 events identical, old vs new,
  switches off).

**What it changes in flight: nothing unless `loss_conditioning:=true`.** The oracle only publishes
`/planning/loss_event`. With `loss_conditioning: false` (the shipped value in `dtp_params.yaml`)
no node acts on the belief. With it on, B and B2 change where `follow_planner` sends the drone and
points the camera while the person is hidden behind something.

## 1. Why: two tree cases the belief gets wrong today

The reappearance belief (`super_core/loss_oracle.hpp:341-416`) rolls 32 CTRV samples forward from
the coasted point and records each one's first position that is outside the occluder's angular
interval **and** has a camera ray with no OCCUPIED cell. **The rollouts never look at the map
while they move**: they walk through trunks, walls and people.

**Case B: a guessed path runs into something.** The person steps around the trunk they vanished
behind, or, in a forest, the next trunk is in their path.
- Real people go around. The rollouts go straight through.
- **A concrete defect follows.** The exit test casts its ray with `stop_short = r_excl` (0.4 m,
  `loss_oracle.hpp:382`). A rollout point up to 0.4 m *inside* an obstacle therefore has a ray
  that stops before the obstacle's near face, reads FREE, and is **recorded as a re-emergence
  inside the obstacle**. The cluster mean, and with it `yaw_target_p` and the viewpoint's centre,
  can land inside a trunk.

**Case B2: the person is straight behind the tree and keeps walking away.**
- The rollouts start *behind* the trunk and move away from it, so they never touch it: B changes
  nothing here.
- Most of them stay inside the widening shadow for the whole 3.2 s horizon, so `still_hidden_w`
  is high and there may be no cluster at all.
- **What happens today with no cluster** (all traced in code):
  - `yaw_target_p` = the coasted point itself, behind the trunk (`loss_oracle.hpp:202`, never
    overwritten).
  - `viewpoint_valid` = false.
  - `target_manager`: `_belief_window` returns None for an event with no `reappear_p`
    (`target_manager.py:768`), so an OCCLUDED coast runs to the 3.0 s cap with the ordinary band
    (`:1023-1024`).
  - `follow_planner`: yaws at `yaw_target_p` (the trunk) and, with no valid viewpoint, falls
    back to the plain standoff goal behind the coasted point (`follow_planner_node.hpp:694-736`).
    The drone lines up *behind the tree* and waits for the coast to run out.
- **What a pilot would do: sidestep until you can see behind the tree.** That is B2.

## 2. Facts already established — do not re-derive

| Fact | Where |
|---|---|
| Rollout loop; heading/speed/turn draws per rollout, **3 draws each, in a fixed order** from `std::mt19937_64 rng(seed)` | `loss_oracle.hpp:360-388` |
| Exit test = outside `[−th_right, th_left]` about `occ_bearing`, then `castRay(c, look, r_excl)` at `heights.back()` | `:377-383` |
| Clusters by side (+1 left), mean / σ / dt min-med-max / mass; sorted by mass; `yaw_target_p` = dominant mean | `:391-416` |
| Viewpoint = 24 azimuths × 3 radii about a centre; skips < `min_standoff` from `target_p`, inflated OCCUPIED / OUT_OF_MAP, and candidates with no clear ray to the centre; nearest the drone wins | `:418-444` |
| `heights` = sample heights that survive the virtual ground and the MID360's 7° view floor (at least one) | `:239-253` |
| `castRay` labels by first non-free cell; UNKNOWN on the target's trail is excused (`onTrail`, 0.6 m, band −1.0/+1.3 m) | `:454-488` |
| Params struct / YAML loader / glue | `loss_oracle.hpp:99-140`; `fsm/config.hpp:68` (members), `:117` (`LoadParam`); `fsm_ros2.hpp:587-613` (copy into `Params`); `follow_test_ros2.yaml:239-297` (the `loss_oracle:` block) |
| Evaluate call (trail passed in) and LossEvent fill | `fsm_ros2.hpp:429-432`, `:452-487` |
| `LossEvent` has `yaw_target_p`, `viewpoint_valid`, `viewpoint_p` independent of `reappear_*`; its comment says `yaw_target_p` = dominant cluster mean, `query_p` when there is none | `dtp_msgs/msg/LossEvent.msg` |
| Downstream consumers need no change for B or B2: `target_manager` reads `reappear_*` for the window/region, `follow_planner` reads `yaw_target_p` / `viewpoint_p` | `target_manager.py:761-774, 1013-1032`; `follow_planner_node.hpp:691-736` |
| Cost today: classify p99 81 µs, full evaluation p99 406 µs; budgets 2 ms / 10 ms (gate O6) | `m6-loss-oracle.md` |
| `fsm_node` reads its config from the **source** tree (`ROOT_DIR` = `CMAKE_CURRENT_SOURCE_DIR`); the install copy is refreshed by the build | `super_planner/CMakeLists.txt:15` |

## 3. Design B — rollouts that cannot walk through OCCUPIED cells

Inside the per-rollout step loop (`loss_oracle.hpp:373-386`), before accepting a step:

1. **Sub-step.** The step length is `s * step_s` (up to ~0.3 m at 3 m/s). Split it into
   `ceil(len / res)` sub-steps, so a 0.1 m pole cannot be jumped.
2. **Body probe.** A position p with heading h is *blocked* when `map_->getGridType(q) == OCCUPIED`
   for any probe q:
   - probe positions: p, p ± `rollout_body_radius` · lateral(h), and p + `rollout_body_radius` ·
     forward(h);
   - probe heights: every `heights` entry (the map's knowable band — never below the sensor
     floor).

   Use the **raw** grid, not `getInfGridType`: inflation is sized for the drone, not a person.
   UNKNOWN is free (the space behind a trunk is unknown).
3. **The person's own cells are not obstacles.** With the M5.3a filter off, where the person
   stood is OCCUPIED for ~10 s. Excuse an OCCUPIED probe cell within `rollout_body_radius`
   (horizontally, inside the trail's vertical band) of any trail point. Use this radius, **not**
   `trail_radius` (0.6): a trunk the person brushed past must still block.
4. **Deflect.** If the next sub-step is blocked, try headings h + σ·k·Δ, then h − σ·k·Δ, for
   k = 1 … ⌈`rollout_deflect_max_deg` / Δ⌉, with Δ = `rollout_deflect_step_deg`.
   - σ = sign of the rollout's own turn rate `w`. It is a continuous draw, so exactly 0 does not
     happen; use +1 if it does.
   - Take the first free heading, set `h` to it (the turn persists, like a person going round),
     and continue.
   - **No new random draws.** The draw order must stay identical, so `rollout_collision: false`
     reproduces today's beliefs bit for bit and on/off runs are paired per rollout.
5. **Stuck.** No free heading within ±`rollout_deflect_max_deg`: the rollout stops there. It
   counts as hidden unless it has already exited.
6. **Start inside an obstacle.** The coasted CTRV point can itself be inside the trunk: the
   forecast walked through it, which is exactly case B. If the start is blocked, back off along
   −h0 in `res/2` steps up to `rollout_backoff_m` and start all rollouts from the first free
   point. If none is free, run this event's rollouts **without** collision and count it
   (log line); never invent a start.
7. **A cluster mean can still fall inside an obstacle.** Deflection splits one side's exits
   into "round the front" and "round the back", and their mean can sit inside the trunk. When a
   cluster mean's raw cell is OCCUPIED (or its body probe is blocked), use the cluster's
   **medoid** (the member exit nearest the mean) as `mean`. σ and dt stay computed from all
   members.

Only the rollouts change. Eq. 1 (the classification), the occluder extent and the viewpoint
search are untouched by B.

## 4. Design B2 — look behind the occluder when the person is expected to stay hidden

After clustering (`loss_oracle.hpp:414`), before the viewpoint:

1. **Trigger:** `hidden_look` is on and `still_hidden_w ≥ hidden_look_min_w`, or there are no
   clusters at all (while `hidden_look` is on).
2. **Look point:** the mean position, at `t = min(hidden_look_s, horizon_s)`, of the rollouts
   that never exited. Store each rollout's position at that step, which costs one Vec3f per
   rollout. A rollout that got stuck (B) contributes where it stopped. Apply §3 item 7's medoid
   rule to this mean.
3. **Reduction:** `yaw_target_p` = the look point. Run the **existing** viewpoint search with the
   look point as its centre instead of the dominant cluster mean. It already requires a clear
   camera ray from the candidate to the centre, ≥ `min_standoff` from the hidden person's
   position, outside inflated occupancy, nearest the drone. That *is* the sidestep.
4. **Leave `reappear_*` alone.** They still describe real exits, so `target_manager`'s
   window/region logic is unchanged. When exits exist but hidden mass dominates, the region gate
   keeps using the minority exit cluster: that is still a valid place to re-lock.
5. **No message change.** Which branch produced the reduction is recoverable from any bag:
   - `yaw_target_p == reappear_p[0]` exactly → cluster;
   - `== query_p` → none;
   - otherwise → hidden-look. These are exact copies of the same doubles.

   Update the comment on `yaw_target_p` in `LossEvent.msg` to say so. That is comment only;
   rebuilding `dtp_msgs` for it is optional.

**Speed caveat to write into the results.** At the S1 envelope (`max_vel 0.5`) a 1–2 m sidestep
takes 2–4 s against a 3.0 s coast cap. B2 can only pay off at S2/S3 speeds, or with a longer cap.
Measure it; don't assume it.

## 5. Parameters

`loss_oracle/*` in `follow_test_ros2.yaml`, loaded in `fsm/config.hpp`, copied in `fsm_ros2.hpp`,
fields in `loss_oracle::Params`. **Off in code, on in `follow_test_ros2.yaml`**, the same
convention as `loss_oracle/enable`. The switches exist so the paper can ablate them.

| Parameter | Code default | Follow config | Meaning |
|---|---|---|---|
| `rollout_collision` | false | true | B on/off |
| `rollout_body_radius` | 0.25 | 0.25 | person half-width for probes and the own-cell excuse (harness person is 0.20) |
| `rollout_deflect_step_deg` | 15 | 15 | heading search step |
| `rollout_deflect_max_deg` | 90 | 90 | give up (stuck) past this |
| `rollout_backoff_m` | 1.0 | 1.0 | how far back a start inside an obstacle may be moved |
| `hidden_look` | false | true | B2 on/off |
| `hidden_look_min_w` | 0.5 | 0.5 | still-hidden mass that switches the reduction to the look point |
| `hidden_look_s` | 1.5 | 1.5 | when along the hidden rollouts the look point is taken |

The 0.5 and 1.5 are starting values, not measurements. Set them from the harness print-out
(§6.1) and say so in `m6-loss-oracle.md`.

## 6. Gates

### 6.1 Measure first, then write the thresholds
Add the scenes below to `drone-dtp/scripts/loss_oracle_check.py`. Before implementing, run them
**with both switches off** on today's binary and print `summarise()` for every event. That is the
baseline the new assertions compare against. Add a config variant pair the way the script
already does for the target filter (`re.sub` on the base YAML, `:197-200`).

**Harness facts that bite:**
- `tfc.scan()` always adds a **wall at x = 7.0, |y| ≤ 5, up to 2.5 m** (`target_filter_check.py:67,97-99`).
  Rollouts heading +x hit it after a few metres, and with B on they deflect along it. Keep the
  B2 scene's look point well in front of x = 7, or assert with that in mind.
- The floor is −0.45, the person r 0.20 with top 1.20, and the waist sample is `P_Z`.
- The effective virtual ground is −0.5.
- Each `run_scenario` starts a fresh `fsm_node` (~20–40 s for the A* preallocation). Count the
  added runtime.

### 6.2 New scenes (geometry proposals; numbers to be fixed from 6.1)
- **O8 AROUND.** `TUBE` (r 0.2 at (3.0, 0)) is the occluder, plus a second trunk r 0.3 at
  (4.0, −0.6) in the rollouts' path.
  - Setup: the person is seen beside the tube, as in O1: (4.0, 1.2) walking −y at 0.8 m/s, then
    coasts at (4.0, 0.2).
  - **Collision off (expected today):** the right-side cluster mean lies inside the second
    trunk's footprint. That is the defect of §1, and this assertion documents it.
  - **Collision on:**
    - no cluster mean within 0.3 m + 0.1 m of the second trunk's centre;
    - dominant exit still on the right;
    - dt shorter than off (exits round the front of the second trunk).
  - Also a start-inside variant: coast point at (4.0, −0.5), inside the second trunk. With
    collision on, events still carry a belief, and the back-off log line appears.
- **O9 BEHIND.** A wide occluder r 0.3 at (3.0, 0).
  - Setup: seen beside it at y = 0.7, walking +x at 1.0 m/s; coast straight behind it on y = 0,
    starting near x = 4.2.
  - **B2 off:** `still_hidden_w` high (record it), `yaw_target_p == query_p` when no cluster,
    `viewpoint_valid` false.
  - **B2 on:**
    - `yaw_target_p` is neither `query_p` nor `reappear_p[0]`;
    - `viewpoint_valid` true;
    - the viewpoint has a clear camera ray to the look point, sits ≥ 2.5 m from the hidden
      person, and is **laterally offset** from the camera–trunk line by at least the shadow's
      half-width at the viewpoint's range.
- **Pairing check (both scenes):** with both switches off, the event stream must equal today's
  binary's, field by field, excluding `*_us` and stamps. This proves the draw order is untouched.

### 6.3 Existing gates that must still pass
- `loss_oracle_check.py` O1–O7 (28/28 today) with the new follow-config defaults ON.
  - O1's "still-hidden ≤ 0.3" and "yaw target = dominant cluster mean" must hold: B2 must not
    trigger there.
  - **O6 budgets:** classify p99 ≤ 2 ms, eval p99 ≤ 10 ms. B adds up to ~20 k grid lookups per
    OCCLUDED event; expect ~1 ms, and record the real figure.
- `target_filter_check.py` 17/17. Same binary; CLAUDE.md says re-run it after any SUPER change.
  Remember its `SUPER_CFG_DIR` trap if you gate a scratch build.
- `loss_conditioning_check.py` 16/16 and `test_target_manager_loss.py` 27/27. They don't exercise
  `fsm_node` (the script plays the oracle), so they only need re-running if a DTP node is
  touched. B/B2 as designed touch none.

### 6.4 Replay regression
`loss_closed_loop_replay.py` on `20260909_closedloop_run5`, both arms, compared with the table in
`m6-loss-oracle.md`: coasts, re-acquired, lost, events by class, `fsm_node` CPU. run5's occluder
is below the sensor floor, so little should change; a big change is a bug. If time allows, run
`loss_replay.py` on `20260913_1955_stageA` too (2 OCCLUDED behind the house wall).

## 7. Build and deploy — procedure and traps

1. **Preconditions.**
   - **Stack not running:** `ps -eo pid,args | grep -E "lib/super_planner/fsm_node"`. Never
     `pkill -f`.
   - **No other `colcon build` running.** Two at once delete each other's objects.
   - **Branch:** this tree is on `drone-dtp` (2026-09-14).
2. **Backup** to `~/backups/backup-pre-m64b-<stamp>/`:
   - `~/super_ws/install/super_planner/lib/super_planner/fsm_node`;
   - `super_planner/include/super_core/loss_oracle.hpp`;
   - `super_planner/include/fsm/config.hpp`;
   - `super_planner/include/ros_interface/ros2/fsm_ros2.hpp`;
   - `super_planner/config/follow_test_ros2.yaml`.

   **This backup is also the diff base for the new patch** (step 5).
3. **Edit the live tree** (`~/super_ws/src/SUPER`): the four files above.
4. **Build:** `cd ~/super_ws && colcon build --packages-select super_planner`. It is a plain build:
   **no `--symlink-install`** for `super_planner`. Source `install/setup.bash` and run the gates.
5. **Patch: make it a NEW `drone-dtp/patches/0005-super-belief-around-obstacles.patch`. Do not
   regenerate 0004.** Two reasons:
   - **Idempotency.** `apply_super_patch.sh` skips 0004 when `loss_oracle_en` is present
     (`:127`), so a tree with the old 0004 would silently never receive a regenerated one.
   - **The `robot_r` trap.** 0004's own diff base (`~/backups/backup-pre-m6-20260914-0044/`)
     predates today's `robot_r` 0.2 → 0.30 change (`410d86d`, carried by 0001). A regenerated
     0004 would drag that hunk in.

   How to build 0005:
   - Generate it with `diff -u` of each step-2 backup against the edited file, with
     `a/super_planner/…` / `b/super_planner/…` paths, as 0004 was.
   - Add a `PATCH5` block to `apply_super_patch.sh`, mirroring the 0004 block. Use its own marker
     (e.g. `loss_oracle_rollout_collision` in `fsm/config.hpp`) and an error message naming 0004
     as the prerequisite. Update the header comment and the preflight file checks.
   - **Verify on a fresh checkout:** upstream `2ad3419` + the isnan fix, then
     `apply_super_patch.sh`. The four resulting files must be byte-identical to the live tree.
     That is how 0003 was verified.
6. Leave `dtp_msgs` alone unless you change the `LossEvent.msg` comment. If you do, rebuild
   `dtp_msgs` first, then `super_planner`.

## 8. Docs to update when it lands
- `docs/m6-loss-oracle.md`: steps 6–7 (rollouts, reduction), the Parameters paragraph, the gate
  table (O8/O9 and the new O6 figures), the replay table if it moved, and a "traps" line for
  anything found.
- `dtp_msgs/msg/LossEvent.msg`: the `yaw_target_p` comment (§4 item 5).
- `drone-dtp/scripts/apply_super_patch.sh` header, and `drone-dtp/README.md`'s patch row (0005).
- `CLAUDE.md` "Where the work is right now": replace the pointer to this plan with the as-built
  result.
- `docs/paper/README.md` if the authors adopt it: the belief method (paper §V) gains two steps
  and two ablation switches.

## 9. Out of scope, and decisions that are not ours
- **The paper's method is the authors' call.** B and B2 change M6.4's belief. Both switches
  exist so either version can be reported.
- **Flight.** Nothing here has flown. `loss_conditioning` stays false for any flight with
  authority until M6.3/M6.5 say otherwise.
- **Learned-sample rollouts** (option C, `m6.6-predictor-study.md` §7) and **fine-tuning**
  (option D) are separate. The AGX assets add nothing (`m6.6-predictor-study.md` §8).
- **Obstacle-aware coasting in `target_manager`.** The coasted CTRV point itself still walks
  through trunks. B fixes the belief, not the coast. Not planned.

## 10. Order and effort
1. Scenes O8/O9 with both switches off → baseline print-out (half a day).
2. B in `loss_oracle.hpp` plus params and glue → O8, pairing check, O1–O7, O6 (a day).
3. B2 → O9, O1 regression (half a day).
4. `target_filter_check.py`, the run5 closed-loop replay, patch 0005 verified on a fresh
   checkout, docs (half a day).
