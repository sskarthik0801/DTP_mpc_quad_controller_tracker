# Fact: a person seen from the grounded aircraft never clears from ROG-Map, and patch 0007 fixes it

**Found 2026-09-14 in the flight-day bags (the user walked up to the parked aircraft and back;
their cells stayed). Diagnosed and fixed 2026-09-15: SUPER patch
`drone-dtp/patches/0007-rog-map-unhit-decay.patch`, `rog_map/unhit_decay`, OFF in code, ON in
`follow_test_ros2.yaml`. Bench-gated on the real `fsm_node` and on the flight-day bag replayed
through it. NOT yet run live on the aircraft.**

## What was seen

Bag `bags/2026-09-14_195701` (42 s, recorded from the GCS at 19:57) and the Stage A bag
`~/dtp_datasets/20260914_1943_stageA` that spans the same walk plus the 7 minutes after it. The
aircraft was on the ground, so the MID360 sat **0.15 m above the grass**. The walker came in to
1.5 m, stepped back to 2.0 and 2.9 m, stood at 1.6–1.9 m, then moved out to 3–5 m for minutes.

457 occupied cells could be attributed to the walker's body (cells above the ground plane,
inside 0.6 m of the LiDAR-tracked person, not in the static map). The Stage A bag replayed
through the real `fsm_node` (flown follow config, `drone-dtp/scripts/loss_replay.py` machinery):

| after the walk | occupied among those 457 cells |
|---|---|
| +2 s | 338 |
| +3 min | 190 |
| +7 min, walker gone | 113 |

The cells that cleared and the cells that did not differ in one thing only, whether any LiDAR
return exists behind them:

| | cleared (275) | survived 7 min (113) |
|---|---|---|
| rays through the cell after the walker left (median) | 718 | 12 |
| cells with zero rays | 0 | 30 |
| cells with fewer than the 31 a cell at p_max 0.95 needs | 0 | 73 |
| elevation from the sensor (median) | 15° | 22° |
| height above ground (median) | 0.85 m | 1.25 m |
| range (median) | 2.5 m | 3.2 m |

RViz magnifies it: `rviz/m1_viewer.rviz` shows `/rog_map/inf_occ`, so each surviving 0.1 m cell
draws as a roughly 1 m blob.

## Why: no return, no ray

ROG-Map casts a ray only to a point that **returned** (`ProbMap::raycastProcess`). A beam that
returns nothing casts no ray and clears no cell. From 15 cm off the ground every part of a person
is above the horizon, and a beam that passes through their chest or head keeps climbing into the
sky. In the sector where the walker stood (azimuth −40° to +10°) only 7–31 % of beams at 10–30°
elevation returned at all, from 50–60 m away; elsewhere ~70 % returned and the walker's cells there
cleared with p50 1.5 s. The clearest demonstration is 3.5 min into the replay: 104 cells cleared in
one 50 s window, exactly when the walker came back to 2.7–3.3 m and their own body became the
backdrop behind their earlier spot.

**Not fixable by tuning.** `p_max`/`p_miss` change how many rays a cell needs; these cells get
none. **Not recoverable from the sensor.** The MID360 reports its no-returns (37 % of the points in
the raw `CustomMsg` are zero-coordinate points with `line` and `offset_time` intact) but their
direction is gone: neighbouring returns on the same beam are a median 47° apart, so it cannot be
interpolated, and the SDK exposes no scan-pattern model.

## The fix: visible-but-unhit decay (patch 0007)

`ProbMap::unhitDecayPass()`, every `period_s` (0.5 s = 5 scans), sweeps the occupied cells within
`range_max` (8 m) of the sensor. Each cell inside the sensor's vertical field of view (in the body
frame of the odometry pose; `fov_elev_min/max_deg` −6/+50, one degree inside the MID360's −7/+52)
has a one-byte clock that counts passes during which **no ordinary ray reached the cell, hit or
miss** (`probabilisticMapFromCache()` resets it on either). Once the cell has gone `window_s` (3 s)
unobserved, and long enough that the sensor would have sent `min_expected_rays` (20) through it
(`rays_per_sr_s` × res² / r² × t, 10 000 rays/sr/s = the MID360's 200k pts/s over 5.72 sr after
`point_filter_num` 3, rounded down), and its line of sight from the sensor holds no occupied cell
observed within the last period, it receives that many misses per period: at 3 m ~11 rays/s, so 5
misses per pass and 7 passes from p_max to below p_occ. A cell the pass has emptied keeps decaying
until it is known free, and an occupied cell that is itself unobserved does not block the cells
behind it, so a remnant's layers go together instead of one per window.

Four rules were learned on the gates, in this order, and all four are in the code:

1. **UNKNOWN does not block the line of sight.** With a strict known-free line, 26 of 112 cells
   never cleared: the discretised line to a person's tangent cells grazes their never-scanned
   interior. In the field of view, within range, "no return" is the very evidence the decay acts on.
2. **An unobserved occupied cell is transparent to the cells behind it.** Otherwise the walker's
   4–5 cell layers cleared one window at a time: 26 s, now 7 s.
3. **Any ordinary ray resets the clock, not only a hit.** With hits only, far ground at grazing
   incidence (a beam crossing a 0.1 m ground cell at 1° descends 2 mm and hits the ground in a later
   cell) was decayed ~280 cells per pass and re-hit later, and those line-of-sight walks were the
   whole cost (31 ms). A ray that crosses the cell is a miss ROG-Map already weighs; adding to it
   double counts. Now the ground is untouched and the pass costs 3–4 ms.
4. **A ray the target exclusion (0002) masks still counts as reaching the cell.** The exclusion
   cylinder drops hits and masks misses around the followed person, so a tube standing beside them
   received no weighed ray at all and the decay read it as sky: `target_filter_check.py` F2 lost 26
   tube cells. The masked branches now reset the clock too; F2 is back to 0 lost, 17/17.

Never touched: cells out of the field of view, beyond `range_max`, behind an observed occupied
cell, at or beyond the virtual planes, and any cell an ordinary ray reached within the window. The
guard for a thin real object at long range is `min_expected_rays`: at 8 m a cell expects 1.6 rays/s,
so it must go 13 s with no ray before anything happens, and one hit (+1.79 log-odds) undoes 45 of
the 1-per-pass misses.

## Gate: `python3 drone-dtp/scripts/unhit_decay_check.py` — 10/10 (2026-09-15)

Real `fsm_node` on domain 89, synthetic MID360 scan at the flown density, sensor 0.15 m above a
ground plane, both arms:

| | OFF | ON |
|---|---|---|
| U1 walker at 3.1 m, sky behind, leaves at t = 11 s: cells at t = 45 s | 112 | **0, gone 7 s after leaving** |
| U2 same walker on the other side, a tube appears in front when they leave | 113 | 113 |
| U3 object above +52° mapped once by wide scans, never scanned again | 12 | 12 |
| U4 walker 8.5 m away (beyond `range_max`), sky behind | 112 | 112 |
| U5 wall at 7 m / ground plane / the tube | 2000 / 4648 / 90 | 2000 / 4991 / 90 |
| U6 decay pass cost | | 3–4 ms, max 9.9 ms (budget 15) |

Ground by range (OFF/ON): 0–3 m 953/952, 3–5 m 1653/1693, 5–7.5 m 2042/2346 — the far ground is
noise between runs, not erosion. Re-run this after any ROG-Map change, next to
`target_filter_check.py` (17/17 on this build, with the decay ON in its base config, so it now
covers the exclusion interaction) and `loss_oracle_check.py` (54/54, eval p99 4.6 ms).

## Acceptance on the flight-day bag

The same Stage A window replayed through the new binary with the switch ON:

| after the walk | OFF | ON |
|---|---|---|
| +2 s | 338 | 83 |
| +10 s | 322 | 55 |
| +30 s | 239 | 10 |
| +60 s | 217 | 2 |
| +4 min (walker back at 2.7–3.3 m) | 119 | 19 |
| +7 min | **113** | **3** |

The 3 left are at 2.7 m, 0.25 m above the ground, elevation 2°: rays through them reach the grass
at grazing incidence and ordinary ROG-Map handles them. Of all new cells within 4 m at the end of
the bag, 411 sit in one cluster at azimuth 100°, 3.5 m, first seen in the last 10 s, with over a
thousand LiDAR returns there in the same window: the walker standing behind the aircraft, out of
the camera's view. A real person is not decayed.

Static cells present before the walk and still there at the end: OFF 396/517, ON 388/521.

## What it means for flight day, and what it is not

- A walker positioning themselves near the parked aircraft after ROG-Map starts no longer leaves a
  phantom wall at follow altitude at the takeoff spot. In the air a person's body drops below the
  horizon and clears against the ground behind, but the head stays above it; this handles both.
- It is not a replacement for `target_exclusion` (0002): that keeps the *locked* person out of the
  map while they are there; this removes what any person, in any direction, leaves behind.
- It changes nothing for cells no beam is pointed at. A remnant behind the aircraft in flight is
  out of the field of view only if it is below −7°; the MID360 sees 360° in azimuth.
- Cost: 3–4 ms every 0.5 s in the map update thread, one byte per cell (15 MB), no new topic. The
  log line ` -- [ROG] UNHIT DECAY: N passes, V unobserved cells in clear view, D decayed, E left
  occupied; pass X ms (max Y)` every 5 s while it has something to say.
- To turn it off: `rog_map/unhit_decay/enable: false` in `follow_test_ros2.yaml` (src and
  install) and restart `fsm_node`. With it off the binary behaves exactly as before: the only
  other change on the hot path is one byte store per updated cell.

## Files

- `drone-dtp/patches/0007-rog-map-unhit-decay.patch` — `rog_map/include/rog_map/rog_map_core/config.hpp`,
  `rog_map/include/rog_map/prob_map.h`, `rog_map/src/rog_map/prob_map.cpp`,
  `rog_map/include/rog_map_ros/rog_map_ros2.hpp`, `super_planner/config/follow_test_ros2.yaml`.
  Applied by `drone-dtp/scripts/apply_super_patch.sh` after 0006 (marker `unhit_decay_en`).
  Generated with `diff -u` against `~/backups/backup-pre-0007-unhit-decay-20260915-*/` (the
  pre-0007 files and `fsm_node` binary), verified to reproduce the live tree byte for byte.
- `drone-dtp/scripts/unhit_decay_check.py` — the gate.
- The bag analysis and the replay driver live in the session scratchpad only; the method is
  `loss_replay.py`'s (bag → real `fsm_node`, `/cloud_registered_body` transformed with the odom of
  the same stamp) plus a subscriber on `/rog_map/occ` tracking a fixed cell list.
