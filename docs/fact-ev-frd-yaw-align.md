# External vision in FRD costs `yaw_align` — and that is the right trade (decided 2026-09-02)

**Symptom.** With the M2 stack live and healthy, `vehicle_local_position` reports
**`heading_good_for_control: False`**, and `estimator_status_flags` shows
`cs_yaw_align: False` — even though `cs_ev_pos`, `cs_ev_yaw` and `cs_ev_hgt` are all
`True` with zero innovation rejections.

**This is not a fault.** It is a deliberate PX4 response to the frame we declare.

## Cause

`px4_odom_bridge` publishes `pose_frame = POSE_FRAME_FRD` (2). PX4 v1.17
`ekf2/EKF/aid_sources/external_vision/ev_yaw_control.cpp:172-181`:

```c
} else if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_FRD) {
        resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
        _control_status.flags.yaw_align = false;   // <-- explicit, only in the FRD branch
        _control_status.flags.ev_yaw    = true;
}
```

Only the `LOCAL_FRAME_NED` branch sets `yaw_align = true`. PX4's reasoning is in the
message definition itself (`px4_msgs/msg/VehicleOdometry.msg`): FRD means "constant
**arbitrary** heading offset from True North", so the yaw is internally consistent but not
globally anchored, and PX4 refuses to call it aligned.

That flag is the whole chain:
`EKF2.cpp:1617  lpos.heading_good_for_control = _ekf.isYawFinalAlignComplete()`, and
`ekf.h:254` returns `_control_status.flags.yaw_align && (...)`.

**FRD is the honest declaration.** FAST-LIO's world frame (`lidar_init`) is gravity-aligned
from the IMU, but its x-axis points wherever the MID360 faced at init. No magnetometer is in
that loop — nothing tells FAST-LIO where North is.

## What it actually costs — narrower than it first looks

| Mode | Heading hold | Why |
|---|---|---|
| **Offboard (M3)** | ✅ unaffected | v1.17 has **no Offboard FlightTask** (`tasks/` has no Offboard dir); setpoints bypass it and `mc_pos_control` never reads the flag |
| **Position** | ✅ unaffected | `FlightTaskManualPosition` → `_stick_yaw.generateYawSetpoint()`, no gate |
| **Altitude** | ✅ unaffected | `FlightTaskManualAltitude:285`, same |
| **Stabilized** | ❌ no heading hold | `mc_att_control_main.cpp:144` sets `_yaw_setpoint_stabilized = NAN` every cycle |
| **Auto (RTL/Land/mission)** | ❌ no yaw command | `FlightTaskAuto.cpp:491` → `_yaw_setpoint = NAN, _yawspeed_setpoint = 0` |

The Stabilized path is guarded at `mc_att_control_main.cpp:293` by
`manual && !altitude && !velocity && !position` — **Stabilized only**. Position and Altitude,
the modes you would actually hand back to a pilot, are untouched. RTL still navigates home;
it just crabs there without rotating.

## Decision: keep FRD. Do not "fix" this.

`EKF2_EV_CTRL` stays **11**, `pose_frame` stays **FRD**.

## The escape hatch (if you ever need real North-anchored heading)

PX4 **does** support EV position in FRD *without* EV yaw —
`ev_pos_control.cpp:88-101` estimates the constant rotation between the EV frame and the EKF
nav frame and rotates the measurement in:

```c
case PositionFrame::LOCAL_FRAME_FRD:
        if (_control_status.flags.ev_yaw) {
                pos = ev_sample.pos - pos_offset_earth;              // use EV frame as-is
        } else {
                const Dcmf R_ev_to_ekf = Dcmf(_ev_q_error_filt.getState());
                pos = R_ev_to_ekf * ev_sample.pos - pos_offset_earth;
                ...
        }
```

So `EKF2_EV_CTRL: 11 -> 3` (drop the yaw bit) hands yaw to the **magnetometer**, keeps EV
position, keeps the frame honest, and sets `yaw_align = true`.

**Rejected for now because the magnetometer is not trustworthy on this airframe.** The live
flags already say so: **`cs_mag_heading_consistent: False`** — EKF2 observes that the compass
disagrees with the current heading. An ARK 12S power module and its harness sit next to the
compass. Trading FAST-LIO's yaw (sub-degree, drift-free over a flight) for a compass PX4
already flags as inconsistent, on a drone meant for forests and indoors, is a much larger
risk than losing heading hold in Stabilized. Bad mag yaw is the classic PX4 flyaway.

**If you take this path later:** recalibrate the compass with the power module energised
first, and verify `cs_mag_heading_consistent` goes `True` before trusting it.

## Do NOT switch the bridge to NED

It would set `yaw_align = true`, and today it would even be self-consistent — the mag is
unused (`cs_mag: False`, suppressed by EV yaw) and `EKF2_GPS_CTRL=0`, so nothing would
contradict it. But it asserts a North-alignment that was never measured, and
`ev_yaw_control.cpp:76` **drops EV yaw entirely once GNSS aiding is on**:

```c
// if GNSS is enabled don't allow EV yaw if EV isn't NED
if ((_control_status.flags.gnss_vel || _control_status.flags.gnss_pos)
    && _control_status.flags.yaw_align
    && (ev_sample.pos_frame != PositionFrame::LOCAL_FRAME_NED)) {
        continuing_conditions_passing = false;
}
```

So NED would look fine on the bench and silently break the day you switch to the
`vision-gps` profile. Worst of the three options.
