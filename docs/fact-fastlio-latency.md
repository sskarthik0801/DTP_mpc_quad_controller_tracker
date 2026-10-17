# FAST-LIO ROS2 port: ~1.4 s odometry latency (fixed 2026-09-01)

**Symptom.** `/lidar_slam/odom` carried *correct* values with *correct* timestamps, but
those timestamps were ~1.4 s behind wall clock. Nothing in the stack reports it: SUPER's
freshness check (`rog_map.cpp:271`) uses wall-clock **arrival** time, not the message stamp,
so ROG-Map plans happily on a 1.4 s-stale pose. On a stationary rig it is completely
invisible — position accuracy stays millimetre-level either way.

It was **intermittent**: decided at startup, then stable for the process lifetime.
Measured 9/9 lagged with the full M1 stack; FAST-LIO alone hit it roughly half the time.

## Root cause (two compounding defects, both in the ROS2 port)

The pinned SHA `a4743b0` is a merge of community PR #381 (`mfassler/ROS2`). The ROS 1
original spins a plain loop; this port does not.

1. **Single-threaded executor, no callback groups.** `main()` called
   `rclcpp::spin(...)` and `laserMapping.cpp` declared **zero** callback groups, so one
   thread serialised the 200 Hz IMU callback, the 10 Hz cloud callback (which preprocesses
   ~20k points inline) and a 100 Hz timer whose callback is slower than its 10 ms period.
   The timer is therefore permanently overdue, the executor keeps servicing it, and the
   cloud subscription is starved.
2. **Depth-20 cloud subscription queue.** With drain rate == input rate in steady state, a
   queue that fills once during the startup stall (ikd-Tree build) **never empties** — a
   permanent standing queue. ~13 frames at 10 Hz == ~1.3 s. Classic bufferbloat.

Timestamps were never wrong: `odom_stamp - cloud_stamp` measured **+0.01…+0.07 s**
throughout (the odom is correctly stamped after its source cloud). The frame was simply
held. Frame-matched arrival-to-arrival delay was **+1.33…+1.37 s**.

## Fix — `patches/fastlio_executor.py`, applied by `scripts/setup.sh`

- sensor subscriptions and the mapping timer get **separate callback groups**;
  `main()` spins a **`MultiThreadedExecutor`** (3 threads)
- cloud queue depth **20 -> 2** (drop stale scans; odometry wants the freshest scan)
- IMU queue depth **10 -> 400** (IMU samples are *integrated* — they must never be dropped)
- `sync_packages()` now takes `mtx_buffer`. It touches `lidar_buffer`/`imu_buffer`/
  `time_buffer` with no lock, which was only safe under a single-threaded executor.
  **Do not drop this hunk** — without it the multi-threaded executor is a data race.

The script is idempotent and safe to re-run.

## Result (stationary, full M1 stack, 10 Hz LiDAR)

| | before | after |
|---|---|---|
| odom stamp lag | 1386-1424 ms | **49-57 ms** |
| lagged-mode runs | 9 / 9 | **0 / 10** |
| position std x/y/z | 4.5 / 5.2 / 3.9 mm | 2.1-3.8 mm |
| FAST-LIO CPU | 42-52 % | 44-47 % |
| FAST-LIO RSS | 151-160 MB | 161 MB |

Driver was never at fault: `/livox/lidar` stamp lag measured **117-122 ms** throughout,
which is exactly one frame period (`base_time` is the first point of a 100 ms frame).

## Measurement gotchas (these produced three wrong conclusions before the real one)

- **`ros2 topic hz` under-reports badly at high rates** — it claimed 28.6 Hz where a
  counting subscriber saw 100.4 Hz. Do not trust it above ~20 Hz.
- **Python cannot keep up with 500 KB `CustomMsg` at 10 Hz.** A probe that deserializes
  every cloud falls behind and its "latency" is its own queueing. Subscribe with
  `raw=True` and parse only the CDR header (4-byte encapsulation, then `int32 sec`,
  `uint32 nanosec`) — validated identical to full deserialization.
- **Probe-side lag is not pipeline lag.** In the healthy case the tiny odom message
  arrives *before* its own 500 KB source cloud. Only **frame-matched** arrival-to-arrival
  delay (match odom to its source cloud by stamp) measures what FAST-LIO actually adds.
- **`kill -9` on the driver discards queued shared-memory samples**, so a "does it drain?"
  test wrongly suggests there is no backlog.
- `pgrep -f fastlio_mapping` matches your own shell; `pgrep -x` needs the **15-char**
  truncated comm (`livox_ros_drive`, not `livox_ros_driver2_node`).
