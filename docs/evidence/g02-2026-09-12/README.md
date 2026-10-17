# G0.2 evidence, 2026-09-12/13

Raw outputs behind [`../../fact-fastlio-divergence-2026-09-12.md`](../../fact-fastlio-divergence-2026-09-12.md).
Times are Unix seconds, and the aircraft was disarmed throughout. Every hardware test ran on the
real FC with no arming, no mode change and no parameter write.

| File | What it is |
|---|---|
| `repro_dual_driver.sh`, `repro_recorder.py` | The bench reproduction script and its 1 Hz recorder, as last run (repro 2). **Read the header first**: it diverges FAST-LIO on purpose, and the fixed launch files now refuse the second driver it starts |
| `repro1_recorder.csv` | Repro 1, the control: second driver publishing CustomMsg. **Its rate columns are wrong.** That recorder deserialised every CustomMsg in Python and capped itself at ~7 msg/s (the `fact-fastlio-latency.md` trap). Positions and publisher counts are valid |
| `repro2_recorder.csv`, `repro2_events.log` | Repro 2: second driver publishing PointCloud2. Raw subscriptions, so rates are real. CustomMsg drops to 0 Hz while IMU stays at 200 Hz; after the SIGINT the pose runs away |
| `e2e_events.log`, `e2e_bridge_guard_transitions.log` | Hardware end-to-end with the guarded bridge: healthy, then starved, then FAST-LIO restarted |
| `e2e_phase1_preflight_healthy.txt` | `estimator_preflight.py` on the healthy stack: GO |
| `e2e_phase3_preflight_diverged.txt` | The same on the diverged stack: NO-GO (E5, E6, E8) |
| `e2e_phase5_preflight_after_restart.txt` | After a `ros2 run` FAST-LIO restart. **E1 false-fails**: the wrapper process matched by argv token. Fixed afterwards, see `v2_*` |
| `v2_events.log`, `v2_preflight_deployed.txt` | Final run with the deployed pieces: second `run.sh` refused; DTP `sensors.launch.py` / `super.launch.py` refused their duplicates; bridge resumed with `reset_counter=1`; preflight GO 14/14 |
| `replay_guard.py` | Replays every `/lidar_slam/odom` in `~/dtp_datasets/*` through `DivergenceGuard` offline |
