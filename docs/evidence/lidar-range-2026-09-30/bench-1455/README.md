# Evidence: the 2026-09-30 14:55 bench bag — hybrid ON vs OFF, the ground-rejection misread, the cylinder

The findings are in `.claude/rules/dtp-follow-behaviour.md` ("Measured 2026-09-30 14:55") and
`.claude/rules/dtp-calibration.md` (the floor-plane pitch). Scripts, run from one scratch
directory in this order (each reads the previous one's cache file from the working directory):

| script | what it does |
|---|---|
| `cache.py BAG.db3 bag1455.pkl` | LiDAR scans (lidar_init), `/dtp/detections_2d`, `/dtp/range_source`, targets, odom, TF |
| `truth.py` | static/dynamic LiDAR points → `lidar1455.npz` |
| `floor_cam.py BAG.db3` | floor plane from the depth image: camera 0.897 m up, axis +2.67° |
| `att.py BAG.db3` | PX4 pitch +2.67°; the TF chain's camera axis −2.54°; the floor in lidar_init |
| `floor_lidar2.py` | why the LiDAR alone cannot find this (glossy) floor |
| `perframe.py` | live frames: camera vs LiDAR vs truth, and z above the floor → `rows1455.pkl` |
| `headproj.py` | LiDAR head-top projected into the box (a lower bound on the pitch error) |
| `stamps.py`, `lat.py` | why the bag does not replay as recorded (arrival lag, 77 `/tf`) |
| `retime2.py BAG.db3 OUT` | the replayable copy: header stamp + healthy latency, `/tf` from odom |
| `repro.py`, `repro2.py` | `LidarBoxRange.range_for_box` headless on the bag's scans; the strip-ground dump |
| `proto.py` | the proposed ground-height guard, offline, on every detection frame |
| `score.py RUN LABELS`, `score2.py RUN LABELS` | replay scoring by truth range bin, and by walker speed |
| `rs1422.py`, `truth1422.py` | the same misread in the 14:22 live run (`~/dtp_datasets/20260930_1422_stageA_range`) |
| `guard/replay_perception_probe.py` | the probe wrapper plus the guard patched in at runtime (NOT RUN): drop it beside a copy of `replay_perception.sh`, `record_targets.py`, `replay_bag_helper.py` |

Replays: `WARM_S=60 RATE=0.25 PROBE=1 bash drone-dtp/scripts/replay_perception.sh cuda <out> <label> <retimed bag>`,
OFF with `NODE_ARGS="-p lidar_range_enable:=false -p max_depth_m:=10.0 -p prediction_depth_gate_m:=10.0"`.
Outputs in `~/dtp_datasets/20260930_1455_replay/retimed/` (ON_a = ON_b and OFF_a = OFF_b exactly).
