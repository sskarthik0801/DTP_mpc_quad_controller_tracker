---
paths:
  - "docs/**"
  - "README.md"
  - "AGENTS.md"
  - "Contributions.md"
---

# About `docs/`

Scope: which notes in `docs/` are current and which are superseded. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

- `docs/` — prior-session notes + saved durable facts (see `docs/README.md`).
  **`docs/calibrate.md` is the teardown/reassembly runbook** — what to re-measure after taking the
  drone apart, scoped by what was actually removed. FC params and firmware survive a teardown;
  geometry and wiring do not.
  `docs/uart_bringup_log.md` has the FC-link post-mortem; read it before debugging any serial link.

- `docs/plantofly.md` and `docs/mavros-setup-notes.md` are superseded MAVROS/ArduCopter-era notes.

- **`docs/fact-terrain-follow.md` (2026-09-21) is current** and is the only record of the
  terrain-relative follow height (`command_mux terrain_follow_enable`, `terrain_estimator`).
- **`docs/px4-velocity-tuning-plan.md` (2026-09-27) is current** and is the record of the
  horizontal velocity/position loop study and its tuning flight plan (O1 → V0–V6 → O2/O3).
  Nothing in it is flown and no gain was changed. Its scripts are in
  `docs/evidence/velocity-loop-2026-09-27/`. It corrects `docs/px4-tuning-notes.md` §2:
  `MC_ROLL_P`/`MC_PITCH_P` 4.0 is stock in v1.17.
- **`docs/plan-veto-height-and-lidar-range.md` (2026-09-30) is current and is the build plan for
  the next session (user).**
  - **Part A:** the proximity veto's height band, `proximity_veto_half_height_m` 3.0, plus
    `close_pass_check` cases.
  - **Part B:** LiDAR ranging past 10 m, which starts with a detector go/no-go on the 09-29 bag.
  - **Both parts were built 2026-09-30, and neither is flown.** Part A is bench-gated; Part B
    (Steps 0–3) is bench-gated and replay-scored. A built part is recorded in
    `dtp-follow-behaviour.md`; the plan's text gets one status line only.
- **`docs/plan-camera-yaw-servo.md` (2026-09-29) is current and is the build plan** for aiming
  the camera from the aircraft's position in `px4_offboard_bridge` (fix 1, fix 2). **It was
  built on 2026-09-29 (switches off, not flown), and the build departs from it in four places.**
  The build record is in `px4-bridges-offboard.md`; the plan's text is unchanged.
- **`docs/mpc-tracker.md` and `docs/mpc-tracker-integration.md` (2026-10-17) are current** and
  are the only record of `px4_mpc_tracker`: the design, and the how-to. It is integrated behind
  a switch, off by default, never flown. The dated record is in `px4-bridges-offboard.md`.
- **`docs/flight-video.md` (2026-09-26) is current.** It covers how the flight-review videos are
  made (`drone-dtp/scripts/flight_video.sh`), what is on them, and which videos exist.
- **`docs/fact-nmpc-follow-goal.md` (2026-09-20) is current and is the ONLY record of `goal_mode
  nmpc`** — both stages, the 19 bench traps, the gates, the install steps and what is still owed.
  `docs/dtp-follower-stack.md`'s `follow_planner` section and `docs/m6-loss-oracle.md`'s patch
  0008 section point to it and do not repeat it. It describes code that is NOT installed: the
  flight binaries predate it.

## A note on docs/
`docs/livoxconnection.md` and `docs/plantofly.md` are prior-session notes; parts describe an intended
end-state or steps that were later redone. Treat them as **reference, not ground truth** —
`CLAUDE.md` with `.claude/rules/`, the top-level `README.md`, `docs/mavlink-setup.md`, and `docs/M1-session-notes.md` are
the current source of truth.

**Anything describing ARK OS services as a given is Nano-era.** That includes
`docs/uart_bringup_log.md` and parts of `scripts/px4/README.md`. The *lessons* there (port
ownership, heartbeat targeting, INT32 param packing) are still correct and still bite; the
*mechanism* (ARK's `dds-agent`/`mavlink-router`) does not exist on the NX.

*(Note, 2026-09-17: the last paragraph predates 2026-08-31, when the NX was reimaged with ARK OS. ARK's `dds-agent` and `mavlink-router` do run on the NX now. See "Board differences" in `px4-link-and-firmware.md`.)*
