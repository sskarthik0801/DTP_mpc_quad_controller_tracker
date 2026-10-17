# Stage A flight results -- 2026-09-13, Kevin's, two hops (M5.5)

Bags on the NX: `~/dtp_datasets/20260913_1919_stageA` (680 s, 8.1 GB) and `20260913_1955_stageA`
(464 s, 5.5 GB). Analyses: `scripts/analyze_shadow_goals.py <bag>` (Kevin's M5.5 scorer) and
`scripts/flight_timeline.py` (arm epochs, airborne, lock, stamp lag, detection vs range/light). Raw
reports in `plan/dtp-imp/results-2026-09-13/` in john-2424/drone-dtp (bag 2 still to copy) and next to each bag on the NX.

Light 92-101/255 throughout (19:23-19:59), i.e. NOT the dusk that killed 09-12. Aircraft flown
by the pilot on the transmitter; our stack observe-only (`/dtp/shadow/*`), commanding nothing.

## 1. What was flown

| | hop 1 (1919) | hop 2 (1955) |
|---|---|---|
| armed | 216 s (t=220-436); a 1 s arm at t=198 first | 103 s (t=275-378); a 1 s arm at t=267 first |
| airborne (odom z > 0.3 m) | 206 s, z p50 0.98 m, max 2.43 m; +7 s hop later | 94 s, z p50 1.48 m, max 1.69 m |
| platform motion | 5.1 m net, x-span 9.9 m, 1.57 m/s, yaw 0.82 rad/s max | 6.9 m net, x-span 14.2 m, 2.25 m/s, yaw 0.60 rad/s |
| auto_reacquire_after_lost | false (D13) | **true** (live param, Stage A only) |

The 1 s arms before each flight are the pilot's arm-check, not the 2 s hardfault drop: both
real arms held 216 s and 103 s. The uxrce fix has now flown.

## 2. Lock while airborne

| | hop 1 | hop 2 |
|---|---|---|
| LOCKED | 18 s (8.5 %) | **75 s (80.1 %)** |
| COASTING / ACQUIRING / LOST | 0.9 / 0 / 193.7 s | 4.7 / 13.7 / 0.3 s |
| LOST -> LOCKED again | never (needed a human; none ran `reengage`) | 3 times, median 7.9 s |
| longest LOCKED run | 24.3 s (from before takeoff) | 66.3 s (from before takeoff), then 19.8, 13.7, 13.7 s |
| mux FOLLOW/OK (time-weighted) | 8.2 % | **76.5 %** |
| LOST reason, every time | forecast exhausted after a full coast (walker gone from the frame) | same |

Hop 1 lost the walker 24 s into the flight when they walked AWAY to 9.8 m and, with D13 in
force, stayed LOST for the remaining 3 min. Hop 2 with auto-reacquire held 80 % and recovered
every loss by itself. **Both losses are the walker leaving the 4-6 m band, not the stack.**

## 3. Perception: detection fraction vs range (empties credited to the last range seen <= 2 s)

| range | hop 1 frames / seen | hop 2 frames / seen |
|---|---|---|
| 3-4 m | 583 / 99.5 % | 149 / 86.6 % |
| 4-6 m | 3013 / **99.6 %** | 2514 / **97.9 %** |
| 6-8 m | 302 / 100 % | 547 / 89.6 % |
| 8-10 m | 496 / 78.8 % | 177 / 85.9 % |
| > 10 m | 70 / 87.1 % | 88 / 81.8 % |
| range when seen p50 / p95 | 4.7 / 8.9 m | 5.3 / 8.6 m |

In this light the detector is essentially perfect to 8 m and ~80 % at 8-13 m -- far better than
the 09-12 dusk figure (9 % at 6.4 m at 52/255). The lock still dies at 9-10 m because an
80 % hit rate means 1-2 s gaps, and the coast horizon is 1.5 s. Airborne detection: 74 % (hop 1,
walker mostly at 9-10 m) and 84 % (hop 2). `perception_ok` False: 0.2 % and **0.0 % airborne**.

## 4. Shadow goals (Kevin's M5.5 scorer)

| criterion | hop 1 | hop 2 |
|---|---|---|
| S1 standoff error p95 outside [2.75, 4.0] m | PASS 0.000 m (1007/1033 FOLLOW goals) | PASS 0.000 m (823/955) |
| S2 lifted-target jitter p95 | PASS 0.026 m | PASS 0.051 m |
| S3 jitter slope vs yaw rate (the A8 question) | n/a -- aircraft barely yawed (0.13 rad/s span) | **PASS -0.023 m per rad/s (n 823), standoff-error slope -0.007** |
| S4 perception blind | PASS 0.3 % | FAIL 7.1 % -- **all of it on the ground** (the RViz stall, t=115-145 s; 0 % airborne) |
| verdict as scored | INCOMPLETE | FAIL (S4) |

**S3 is the first flight evidence that the stamp-exact TF lift (tf_at_latest false) is right:**
jitter and standoff error are flat against yaw rate up to 0.6 rad/s. A latest-available lift
would have climbed at yaw-rate x range. This is the figure the paper needs
(`shadow_goals.csv`: jitter, standoff sd vs yaw_rate).

S4 on hop 2 is the pre-arm RViz stall inside the bag, not flight. Re-scored on airborne frames
only, hop 2 passes all four. The scorer should window on armed/airborne time -- ask Kevin, or
we pass `--from/--to`.

Other scorer findings to look at, none flight-critical today:
- hop 1: 5 goals with standoff 2.10-2.28 m, "below the 2.5 m hard floor" -- **resolved, see
  section 7: they are the mux's HOLD goal at the parked aircraft, not planner goals.**
- goal steps > 1 m: 8.5 % / 10.5 %. Hop 2's 7-8 m teleports at t=24-25 s are on the ground
  with auto-reacquire hopping between two people near the aircraft -- the D13 hazard made
  visible. Hop 1's 2-3 m steps at t=22-24 s (walker steady) need a look.
- goals "in front of" the walker: 3.8 % / 14 %, mostly not at reversals. On a left-right line
  the heading estimate at 0.3 m/s is noisy; check the trail-azimuth definition before reading
  much into it.

## 5. FAST-LIO stamp lag (bag receive time - header.stamp)

| | hop 1 | hop 2 |
|---|---|---|
| airborne p50 / p95 / max | 0.064 / 0.115 / 0.971 s | **0.053 / 0.072 / 0.107 s** |
| stalls > 1 s | one, 1 s, on the ground at t=8 | one, **30 s, max 9.5 s, on the ground** (t=115-145: RViz) |

The 09-13 stall mechanism (laptop RViz on `/cloud_registered` over wifi -> publisher blocks ->
38 s stamp lag on the ground before hop 1, caught by hand; 9.5 s before hop 2, caught by hand)
never reached a flight. `field.sh` now refuses on either (commit f5cee10).

## 6. Verdict and what changes

- **M5.5 Stage A: achieved in substance** -- lock held in the air, goals geometrically right, S3
  passed, zero perception blindness airborne. Formally INCOMPLETE/FAIL only because the scorer
  windows the whole bag and hop 1 had no yaw. One more hop with the scorer windowed on airborne
  time closes it.
- **Walker protocol is the limiting factor**, not perception: stay 4-6 m, left-right, never away.
  In S1 the aircraft keeps that range itself.
- **auto_reacquire true for Stage A** is a clear win (8.5 % -> 80 % locked). It stays false in
  S1 until re-ID exists (D13) -- hop 2's ground teleports show why.
- The walker-side detection envelope is 8 m in daylight, so the 3-6 m envelope in the paper is
  the DUSK number. (The apparent D6 breach was a scorer artefact -- section 7.)

## 7. Addendum 2026-09-14 00:30 -- the "D6 floor breach" was the scorer, not the planner

The five hop-1 goals at 2.10-2.28 m (t=674-678 s) are the mux's **HOLD goal**, not standoff
goals: their position is constant (4.83, 1.48, 0.99) while the person walks from 7.1 to 6.9 m,
every row is `HOLD|NO_SOURCE`, and command_mux's contract is "no source -> publish a HOLD goal
near the drone's current position, raised by the hold climb". They are exactly the moment
auto_reacquire was switched on after landing (19:30:55) and target_manager locked "track 4 at
2.53 m" -- a person walking up to the parked aircraft. follow_planner enforces its 2.5 m floor
horizontally against both the predicted and the present position (`EnforceStandoffFloor`, twice)
and never produced a goal inside it. **No planner change needed.**

What did need changing is the scorer, and it is patched and run on both bags over Tailscale
(2026-09-14 01:00). **Scored on the airborne window: hop 2 = M5.5 PASS on all four**
(S1 0.000 m over 342 FOLLOW goals, S2 0.061 m, S3 -0.022 m per rad/s, S4 0.0 %); hop 1 stays
INCOMPLETE on S3 (0.12 rad/s of yaw). In flight, no goal was ever below the floor (hop 2 min
2.54 m), and the nearest anyone came to the hovering aircraft was 4.4 m / 4.9 m.
Still to look at: hop 2 has 22 goal steps > 1 m in 94 s (up to 6.7 m) with the walker at
0.8 m/s -- almost certainly the trail azimuth swinging round the walker at each turn of the
left-right line (speed dips under the 0.3 m/s reversal gate, so the scorer calls it
'steady'). Real for S1: every reversal is a ~2 x standoff replan. Plan S1's walk as a long
straight line with one turn, not pacing.

The scorer changes:
- `--window airborne|armed|all` (default airborne: odom z > 0.30 m, runs >= 10 s); applied to
  the goal rows, `/dtp/perception_ok` (S4) and the arbitration histogram. Hop 2's S4 becomes the
  airborne 0.0 %, so hop 2 should score **PASS on all four**.
- sections 1, 2, 4 on FOLLOW goals only; HOLD goals reported as "person-to-hold-point distance"
  (how close someone came to the parked/hovering aircraft), which is the useful reading of them.
- CSV keeps every paired row, all mux states.
