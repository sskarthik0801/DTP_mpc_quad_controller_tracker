# `EKF2_EV_DELAY` must stay 0 — it is ADDITIVE to `timestamp_sample` (settled 2026-09-02)

**The question that kept coming back.** `px4_odom_bridge` sets `timestamp_sample` from the
FAST-LIO header stamp, and the measured gap to `timestamp` is ~50 ms. Should
`EKF2_EV_DELAY` be set to 50 to model that, or does the timestamp already cover it?

**Answer: leave it at 0.** Setting 50 would double-count and place every vision measurement
50 ms *too early*.

## Why — PX4 subtracts EV_DELAY on top of the timestamp

`src/modules/ekf2/EKF/estimator_interface.cpp:381-384`:

```c
// calculate the system time-stamp for the mid point of the integration period
const int64_t time_us = evdata.time_us
                        - static_cast<int64_t>(_params.ekf2_ev_delay * 1000)
                        - static_cast<int64_t>(_dt_ekf_avg * 5e5f);
```

and `evdata.time_us` is our field, unmodified — `EKF2.cpp:2337`:

```c
ev_data.time_us = ev_odom.timestamp_sample;
```

So the two are **additive**, not alternatives. `EKF2_EV_DELAY` exists for sources that stamp
on *arrival* rather than on *sample*; ours stamps on sample, so the correction is already in
the number. (PX4's default is 175 ms — `EKF/common.h:437` — which assumes exactly such an
arrival-stamping source. It is wrong for us.)

## And the stamp really is the sample instant

FAST-LIO stamps odometry with `lidar_end_time` — `laserMapping.cpp:635`:

```c
odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
```

`lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime` (`laserMapping.cpp:399-413`),
i.e. the **end of the sweep** — and the ESIKF back-propagates the state to exactly that
instant. So the pose is valid *at* the stamp. There is no residual scan-accumulation bias to
model; the only residual is `lidar_mean_scantime` estimation error, which is small.

The ~50 ms is therefore pure pipeline transport (cloud arrival ≈120 ms after `lidar_beg_time`,
plus ~30 ms FAST-LIO processing, minus the 100 ms frame), and it is carried by the timestamp.
This is consistent with `docs/fact-fastlio-latency.md`, which measured odom stamp lag at
**49-57 ms** after the executor fix.

## Confirmed empirically (2026-09-02 hand-motion test)

Cross-correlating PX4's `vehicle_local_position` against `/lidar_slam/odom` over a 120 s
test with ~1 m excursions on all three axes: PX4 **leads** FAST-LIO by about one 10 Hz
sample on N/E and 0 on D. EKF2's output being *fresher* than the vision it fuses is exactly
right — it forward-propagates on IMU to present time. A double-counted delay would show up
as PX4 **lagging**. It does not.

## One thing to keep an eye on

`EKF2_EV_DELAY` also feeds the buffer sizing at `EKF2.cpp:948-952`, which raises
`EKF2_DELAY_MAX` to cover the slowest sensor. With `EV_DELAY=0` the EV path contributes
nothing to that calculation, so `EKF2_DELAY_MAX` must independently be large enough for our
real ~50 ms of transport. It is: **`EKF2_DELAY_MAX = 200` ms** on this FC. Do not lower it
below ~100 ms without re-measuring.
