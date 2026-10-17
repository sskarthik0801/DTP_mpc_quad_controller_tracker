# TODO — what is left after the 2026-09-27/28 changes

Written 2026-09-28. Everything below is for the pilot. The code is built, bench-gated, committed
and pushed (`nmpc-follow-goal` at `76d7f8b`). **None of it has flown.** Do the steps in order:
each one clears the next.

What changed, in one line each (records in `CLAUDE.md` "Latest changes"):
- **Acceleration feedforward** is on in the offboard bridge (`acc_ff`).
- **The idle-bridge fix:** the bridge sends setpoints only while PX4 is in Offboard.
- **Speed-scaled standoff and braking-envelope veto** are on in `dtp_params.yaml`.
- **Velocity tuning:** a flight plan only. No gain has been changed.

## Every flight
- [ ] `python3 scripts/px4/estimator_preflight.py` says GO at the take-off spot, before every arm.
- [ ] Exactly one Livox driver, one FAST-LIO and one `fsm_node` are running.
- [ ] Nobody subscribes the laptop to `/cloud_registered`.
- [ ] `systemctl is-enabled logloader` says `disabled`, and `df -h /` shows at least 20 GB free.
- [ ] After stopping a bag, run `sync` before pulling the battery.
- [ ] **Centre the sticks before switching to Offboard.** For an estimated 40–70 ms after the
      switch, PX4 flies its last manual setpoint.

## Still open, and your call
- [ ] The `uxrce_dds_client` hardfault: the 14 h powered soak and the shell-path reproducer.
- [ ] The MID360→FC yaw mounting offset, still unmeasured.

## 1. Props off, on the bench — done 2026-09-28 (record: `.claude/rules/px4-bridges-offboard.md`)
- [x] Start the stack and both bridges from the GCS.
- [x] The offboard bridge's startup log line shows `acc_ff=True`.
- [x] In Position mode, `listener trajectory_setpoint` on the FC shows no samples from the bridge.
- [x] In Offboard with SUPER tracking a goal, `listener trajectory_setpoint` shows real
      acceleration numbers. In hold it shows NaN.
- [x] `m3_gate_check.py`, arm then engage: 9/9.
- [ ] `m3_gate_check.py`, engage then arm: **not run on hardware.** The transmitter cannot arm in
      Offboard (PX4 refuses), so it needs `offboard_engage.py --arm`. The field order is arm
      then engage. Optional.
- [x] `offboard_engage.py` engages and reports setpoints streaming after the switch.
- [x] `MPC_ACC_DECOUPLE` is 1 on the FC (read live).

**Until step 1 and step 2 pass, keep the offboard bridge stopped for manual flights.**

## 2. A short manual flight, bridge running
- [ ] Fly Position and Altitude mode for a minute or two with the offboard bridge running.
- [ ] Fetch the ULog (`scripts/px4/fetch_ulog.py`).
- [ ] Check the attitude setpoint no longer flips (the 09-21 Flight 2 signature).
- [ ] This is also the first flight of `MC_BAT_SCALE_EN` 1: check the hover thrust stays near 0.41.

## 3. Flight O1 — the feedforward alone
Plan: `docs/px4-velocity-tuning-plan.md` §7.1.
- [ ] Stack on the **follow** config. The click config caps SUPER at 0.1 m/s.
- [ ] **DTP follower down.** With it up, the GCS RC card refuses to send goals.
- [ ] Start a bag.
- [ ] Take off, hover at about 1.5 m with 30 m of clear space ahead.
- [ ] Switch to Offboard (channel 11). **Only then** send a goal about 10 m ahead from the GCS RC
      card. With no follower there is no engage gate.
- [ ] Fly 2–3 legs out and back. Keep them level.
- [ ] **Take over at once** if it moves the wrong way at the start of a leg, oscillates, or goes
      more than about 0.7 m past the stopping point.
- [ ] Score the bag against 09-26. Model prediction: at most about 0.5 m behind and 0.35 m past.

## 4. A follow flight with the two switches on
- [ ] **The follow band is 4–5 m since 2026-09-29** (it was 3–4 m). Check in the bag that the
      lock held: no lock has been held in flight beyond 4.4 m, and a sprint now pushes the
      band to 5.5–6.5 m.
- [ ] The preflight shows a WARN for the two switches. That is expected.
- [ ] Engage Offboard near 1.2 m height, with the walker 5–6 m away, so there is one sprint.
- [ ] Afterwards, check in the bag:
  - the aircraft, not only the goal, stayed outside the floor in the sprint;
  - a braking-envelope HOLD, if one fired, stopped outside 1.5 m;
  - the lock held at 4–5 m;
  - the trailing distance, and SUPER's replan failures.
- [ ] If the feedforward flew well in step 3, consider `speed_track_s` 0.25 and
      `speed_track_max_m` 0.25 in both nodes.

**Not addressed by any of this:** the 09-26 turnaround approach (2.16 m) and SUPER's own descent
to 0.58 m above ground. Nothing guards SUPER's path.

## Camera yaw servo (fix 1 + 2) — built 2026-09-29, both switches off, not flown
Plan: `docs/plan-camera-yaw-servo.md`. Record: `.claude/rules/px4-bridges-offboard.md`.
- [x] Built in `px4_offboard_bridge`: one pure module (`yaw_servo.py`) shared with the replay,
      both switches off by default. Installed.
- [x] Unit check 139/139. `m3_bridge_check.py` 120/120 (51 old + 69 new).
- [ ] **The replay gate fails, 10/18. Your call, before any servo flight:**
  - [ ] `yaw_servo_min_range_m`: 2.5 m is the follow floor, and the rule froze the heading for
        2.58 s in the 09-26 19:51 replay (off-centre max 38° against 20° as flown). At 2.0 m
        all six off-centre criteria pass. No bag has data below 2.16 m.
  - [ ] Restate the "≤ 15°/s while the target is fresh" criterion. A walker at 2–4 m moved
        their bearing at 29–49°/s.
  - [ ] `yaw_servo_aim_in_hold` (default true): the nose turns to the person at the engage,
        before SUPER's first command.
- [ ] Props off on the aircraft (plan §4.4), both switches on through `--ros-args -p`.
- [ ] After step 4 (the servo-off baseline): flight Y1, servo on, then Y2, with the feed-forward
      added. Score both with `drone-dtp/scripts/yaw_chain_report.py`.
- [ ] Commit `yaw_servo.py` together with the bridge: the bridge imports it even when off.

## 5. The tuning ladder V0–V6
Plan: `docs/px4-velocity-tuning-plan.md` §7.2. One change per flight.
- [ ] Dump the params first:
      `python3 scripts/px4/dump_params.py firmware/px4-params-$(date +%F)-pre-veltune.params`.
- [ ] Offboard bridge **stopped** for every flight of the ladder.
- [ ] Setup: `MPC_POS_MODE` 0, `MPC_VEL_MANUAL` 2.0, `MPC_XY_VEL_I_ACC` 0.2.
- [ ] V0: setup only.
- [ ] V1: `MPC_XY_VEL_D_ACC` 0.2 → 0.1.
- [ ] V2–V5: `MPC_XY_VEL_P_ACC` 1.8 → 2.2 → 2.6 → 3.1 (→ 3.7 only if 3.1 was clean). Keep the
      last clean value.
- [ ] V6: `MPC_XY_VEL_I_ACC` → 0.2 × the final P.
- [ ] Afterwards: restore `MPC_POS_MODE` 4 and `MPC_VEL_MANUAL` 10. Dump the params again.
- [ ] Record the final values, dated, in `.claude/rules/px4-estimator.md`.

## 6. Flights O2 and O3 — feedforward with the new gains
Plan: `docs/px4-velocity-tuning-plan.md` §7.4.
- [ ] O2: Offboard legs at `max_vel` 3.0. Model prediction: 0.25 m behind, 0.14 m past.
- [ ] O3: Offboard legs at `max_vel` 4.0 (a SUPER config change). Prediction: 0.30 / 0.18.
- [ ] Plan at most 3.5 minutes airborne per pack. PX4 only warns on low battery.

## Later, if the flights show it
- [ ] If the trailing distance at 2–3 m/s is as large as modelled (6 m and 9–10 m), try SUPER's
      `goal_vel_en`. The standoff must then run on `speed_standoff_signal: aircraft`; the node
      refuses to start otherwise.
- [ ] A shorter age limit for acceleration in the bridge (about 50 ms), against a stalled SUPER.
- [ ] The bridge refusing a far setpoint at engage (a re-engage after a takeover, and goals that
      bypass the mux).
- [ ] `drone-dtp/scripts/bench_bringup.sh` step 2 prints the setpoint rate, which now reads about
      0 Hz outside Offboard. It is informational only.
- [ ] Merge `nmpc-follow-goal` into `main` once steps 1–4 have passed.
