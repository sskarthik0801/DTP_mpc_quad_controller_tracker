# Calibration & re-verification after taking the drone apart

Everything you must re-do, re-measure, or re-check after disassembling the airframe and
putting it back together — in dependency order, with the command for each and the number it
should print.

**Props off for every step in this document.** Nothing here requires spinning motors, and
several steps involve picking the aircraft up.

> **The one-line version:** parameters and firmware live on the **FC** and survive anything you
> do with a screwdriver. What a disassembly actually invalidates is **geometry** (§3) and
> **wiring** (§1). Do not blanket-recalibrate — §0 tells you which sections you actually need.

---

## 0. Scope it first — what did you actually take apart?

Run only the sections that apply. Running the rest wastes an hour and risks replacing
known-good calibration with a worse one.

| What you removed | §1 links | §2 FC sensors | §3 geometry | §4 config | §5 estimator | §6 gate |
|---|---|---|---|---|---|---|
| **MID360 only** | LiDAR link only | — | **all of §3** | — | ✅ | ✅ |
| **Flight controller** | ✅ | **all of §2** | **lever arm + tilt** | — | ✅ | ✅ |
| **Jetson** | ✅ | — | — | ✅ | ✅ | ✅ |
| **Cables/harness only** | ✅ | — | — | — | ✅ | ✅ |
| **Power module / battery wiring** | ✅ + §1.5 | mag, if it moved | — | — | ✅ | ✅ |
| **Full teardown** | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |

§5 and §6 are **always** required — they are what proves the reassembly is airworthy.

### Before you pick up a screwdriver

```bash
# 1. Full FC parameter backup. Do this FIRST, every time.
python3 scripts/px4/dump_params.py firmware/px4-params-pre-teardown-$(date +%F).params

# 2. Record the geometry you are about to destroy, so you can compare afterwards.
python3 scripts/px4/px4_shell.py "param show EKF2_EV_POS*"
```

And **photograph the MID360's mounting orientation** before it comes off — specifically which
way the cable exit faces relative to the drone's nose. §3.3 explains why that photo is the
only cheap way to get the yaw alignment back.

---

## 1. Links and wiring

Nothing below this works if these don't. Verify bottom-up.

### 1.1 LiDAR Ethernet

```bash
sudo bash scripts/network_setup.sh          # auto-detects the wired iface
ping -c 3 192.168.1.154                     # the MID360
```

Expect **0% packet loss**. The IP is `192.168.1.1XX` where `XX` is the last two digits of that
unit's serial — if you swapped in a different MID360, update `config/MID360_config.json` and
pass the override with **`sudo -E`** so the env var survives:
`LIDAR_IP=192.168.1.123 sudo -E bash scripts/network_setup.sh`.

> A manual `ip addr add` is flushed by NetworkManager within ~1 min and the driver then fails
> with `bind failed`. Always use the script — it writes a persistent NM profile.

### 1.2 MAVLink (USB-C)

```bash
python3 scripts/px4/px4_shell.py "ver all"
```

Expect `HW arch: ARK_FPV`, `PX4 version: Release 1.17.0`. If this times out, the USB-C cable or
`mavlink-router` is the problem — see [`mavlink-setup.md`](mavlink-setup.md).

### 1.3 uXRCE-DDS (TELEM1)

```bash
python3 scripts/px4/px4_shell.py "uxrce_dds_client status"
```

Expect **`Running, connected`**, `Payload tx` ~35 kB/s, **`timesync converged: true`**.

**If it says `disconnected`, do NOT start guessing at the wire.** That mistake has been made
twice on this project and cost weeks both times. Go straight to:

```bash
# unplug the TELEM1 harness, jumper 40-pin pin 8 -> pin 10, then:
bash scripts/uart_loopback_test.sh
```

Exact echo at every size = the port is healthy, so the fault is the harness or the FC. All
zeros above ~4 bytes = the Tegra DMA fault is back (a JetPack update reverts the DTB override)
— see [`xrce-agent-nx.md`](xrce-agent-nx.md). **Reconnect the harness afterwards.**

### 1.4 ROS topic surface

```bash
source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash
ros2 topic list | grep -c '^/fmu'           # expect 65
```

### 1.5 Power module (only if battery wiring was touched)

```bash
python3 scripts/px4/px4_shell.py "listener battery_status"
```

Expect `current_a` non-zero (~0.4 A idle) and `temperature` ~45 °C, **not** `0.00000` / `nan`.
If current reads zero, the INA238 driver is gone — that means the FC was reflashed with stock
firmware, not that a wire is loose. Rebuild from `~/PX4-Autopilot`; see
[`ark-12s-payload-power-module.md`](ark-12s-payload-power-module.md).

---

## 2. Flight-controller sensor calibration

**Only if the FC itself was unmounted, remounted, or replaced.** If you only removed the
LiDAR, skip this entire section — recalibrating a good IMU gains nothing and can lose you a
good calibration.

Do these in **QGroundControl** (Vehicle Setup → Sensors), connected on UDP **14550**. The
`commander calibrate {gyro,accel,level,mag}` commands exist on the NuttX shell but are
interactive and prompt-driven, which `px4_shell.py` handles badly.

| Calibration | When | Why |
|---|---|---|
| **Gyro** | always, if the FC moved | cheap, 15 s, vehicle still |
| **Accel** | FC unmounted/replaced | 6 orientations |
| **Level horizon** | **FC remounted, or the frame's resting attitude changed** | defines "level" for *this* airframe — skipping it puts a constant tilt bias into every hover |
| **Mag** | only if you need it | see below |

**On the magnetometer:** this build does **not** use it. `EKF2_EV_CTRL=11` takes yaw from the
LiDAR, which suppresses mag fusion (`cs_mag: False`), and `EKF2_GPS_CTRL=0`. So a mag
calibration is optional *for the current profile*. Do it anyway if you might ever use
`vision-gps`, or the `EKF2_EV_CTRL=3` escape hatch in
[`fact-ev-frd-yaw-align.md`](fact-ev-frd-yaw-align.md) — and if you do, calibrate **with the
power module energised**, because its harness is what makes the compass inconsistent here.

Afterwards:

```bash
python3 scripts/px4/diag_health.py
```

> On USB power with no battery, `MAV_STATE_UNINIT`, battery `65535 mV` and EKF
> `CONST_POS_MODE` are **benign** — no power module, no horizontal aiding yet.

---

## 3. Geometry — the part a disassembly really invalidates

This is the section people skip and then chase a "tuning problem" that is actually a
measurement problem.

### 3.1 Lever arm — where the LiDAR sits relative to the FC

`EKF2_EV_POS_X/Y/Z` tell EKF2 that our vision measurement describes the **MID360's internal
IMU**, not the FC. Left wrong, EKF2 charges the rotation-induced motion of that offset point to
translation — worst exactly during the yaw a human-following drone does constantly.

**Measure**, in the PX4 body frame (**FRD**: x forward, y right, **z down**), from the **FC
IMU** to the **MID360's internal IMU**:

```
FC IMU --(measure this)--> dome seam --d--> LiDAR origin --4.41 cm down--> MID360 IMU
```

The MID360's IMU is **4.41 cm below** the LiDAR optical origin, 1.1 cm forward and 2.33 cm
left — that is `extrinsic_T` in `config/mid360.yaml`, and it is a property of the sensor, not
of your mounting. `d` (dome seam → optical origin) is in Livox's drawing and is not recorded
here; ~1 cm is the working assumption.

Remember the sign convention: in FRD, **"above the FC" is negative z** and "left" is negative y.

Then edit the three values in `scripts/px4/ev_lever_arm.py` (the derivation lives in its
docstring) and:

```bash
python3 scripts/px4/ev_lever_arm.py            # dry run -- read this before applying
python3 scripts/px4/ev_lever_arm.py --apply
```

**Current airframe reference:** `(+0.011, -0.023, -0.07)` — LiDAR directly above the FC,
10.5 cm to the dome seam.

> **EKF2 reads these at init.** After applying: `ekf2 stop` then `ekf2 start`, or reboot the
> FC. Then re-read them after a power cycle before trusting them — a `SER_TEL1_BAUD` change
> once read back correctly and then silently reverted.

### 3.2 Mounting tilt — verify, don't assume

Bring the stack up (§5.1), keep the rig **completely still**, then:

```bash
python3 scripts/px4/mount_tilt_check.py 20
```

This compares FAST-LIO's roll/pitch (MID360 IMU) against EKF2's (FC IMU). Because
`EKF2_EV_CTRL=11` fuses EV **yaw only**, those two are independent gravity references, so the
difference *is* the physical mounting tilt.

**Reference (2026-09-02): roll −1.55°, pitch +0.09°.** Under ~3° needs no action. Above that,
**re-seat the MID360 rather than compensating in software** — there is nowhere good to put a
tilt correction (see §3.3).

### 3.3 Mounting **yaw** — the one that can hurt you, and the one no script can measure

`mount_tilt_check.py` prints a yaw row and labels it IGNORE. That is not laziness: EKF2's yaw
is *defined* by the EV yaw we send it, so the two agree by construction no matter how the LiDAR
is bolted on. Gravity gives you roll and pitch for free. **Nothing gives you yaw on a bench.**

Why it matters: a yaw mounting error leaves positions consistent — they're in the world frame —
but makes EKF2's attitude wrong by that angle. The controller then tilts the wrong way to move
north. That is a spiral, not a wobble.

**So this is a mechanical check.** Compare against the photo you took in §0:

- Is the MID360 square to the airframe, cable exit facing the same way as before? → offset is
  0, nothing to do.
- Rotated, even by 90° for cable routing? → **that angle has to go into
  `super_bringup/scripts/px4_odom_bridge.py`.**

There is no parameter for it. PX4's complete external-vision set is `EKF2_EV_CTRL`,
`EKF2_EV_DELAY`, `EKF2_EV_NOISE_MD`, `EKF2_EV_POS_X/Y/Z`, `EKF2_EV_QMIN` — **three translation
params and no rotation param at all.** The bridge's `(w, x, −y, −z)` flip is a *convention*
change (FLU→FRD), not a mounting rotation; `mid360.yaml`'s `extrinsic_R` is the wrong place too
(it covers LiDAR optical origin → MID360 IMU, and is correctly identity).

### 3.4 Optical flow and range finder

Both are fitted and fused (`EKF2_OF_CTRL=1`, `EKF2_RNG_CTRL=2`). If either sensor moved,
re-measure `EKF2_OF_POS_Z` (currently `0.14`). Confirm they are alive:

```bash
python3 scripts/px4/px4_shell.py "listener sensor_optical_flow" "listener distance_sensor"
```

Expect flow `quality` > 0 publishing continuously, and a downward (`orientation: 25`) range
reading with `signal_quality: 100`.

---

## 4. Jetson-side config sanity

**Only if the Jetson was reflashed, or anything was rebuilt.** A `colcon build` of `fast_lio`,
or a Livox driver rebuild, **silently restores upstream defaults** — this has bitten this repo
before, and the symptom (`fsm_node` logging "No odom" forever) looks exactly like a sensor
fault.

```bash
bash scripts/setup.sh        # idempotent; re-applies configs and both patches
```

Then spot-check the three things that go wrong:

```bash
grep -E "lidar_ip|host_ip" config/MID360_config.json    # NOT the stock .5 / .12
grep -n "ros__parameters" config/mid360.yaml            # must be ROS 2 format, not ROS 1
grep -n "map_en" config/mid360.yaml                     # publish.map_en must be false
```

Configs must land in **both** `src/` and `install/`. Also confirm the bridge is executable —
it was committed non-executable once and `ros2 run` fails with `No executable found`:

```bash
test -x super_bringup/scripts/px4_odom_bridge.py && echo OK || chmod +x super_bringup/scripts/px4_odom_bridge.py
```

---

## 5. Estimator

### 5.1 Bring the stack up

```bash
RVIZ=false bash scripts/run.sh &
# in another shell:
ros2 run super_bringup px4_odom_bridge.py &
```

Check Gates A/B/C in the top-level [`README.md`](../README.md): `/livox/imu` ~200 Hz,
`/livox/lidar` ~10 Hz, `/lidar_slam/odom` ~10 Hz, `/rog_map/occ` width > 0.

### 5.2 Confirm the profile, then restart EKF2

```bash
python3 scripts/px4/ekf2_profile.py --show          # expect: matches profile: vision-only
python3 scripts/px4/ekf2_profile.py vision-only --apply   # only if it does not
python3 scripts/px4/px4_shell.py "ekf2 stop" "ekf2 start" "ekf2 status"
```

Expect `attitude: 1, local position: 1, global position: 0`.

The restart is **mandatory** after §3.1 — EKF2 reads `EKF2_EV_POS_*` only at init.

### 5.3 Confirm the FC is receiving, and that it believes it

```bash
python3 scripts/px4/px4_shell.py "uxrce_dds_client status" "listener vehicle_visual_odometry"
```

| Field | Expected |
|---|---|
| `Payload rx` | **~1150 B/s** (10 Hz × 115 B) — 0 means the bridge isn't reaching the FC |
| `pose_frame` | **2** (FRD) |
| `velocity` | `[nan, nan, nan]` — deliberate, matches `EKF2_EV_CTRL=11` |
| `timestamp_sample` | ~50 ms before `timestamp` |
| `position_variance` | `0.01` floors |

```bash
python3 scripts/px4/px4_shell.py "listener estimator_status_flags"
```

Expect `cs_ev_pos`, `cs_ev_yaw`, `cs_ev_hgt` all **True**, `cs_gnss_pos` False, and **every
`reject_*` and `fs_*` flag False**.

> `heading_good_for_control: False` and `cs_yaw_align: False` are **expected and correct** —
> they are PX4's deliberate response to `POSE_FRAME_FRD`. Not a fault, do not "fix" it.
> [`fact-ev-frd-yaw-align.md`](fact-ev-frd-yaw-align.md).

---

## 6. The gate — prove it actually tracks

**This is the step that catches a reassembly error, and it is the one that is easiest to skip.**
Everything above can pass with an axis inverted or the LiDAR yawed 90°, because a stationary rig
sits near zero on every axis.

```bash
source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash
python3 scripts/px4/m2_gate_check.py 120 /tmp/motion.csv
```

Then pick the aircraft up and move it, pausing ~5 s between each:

1. **forward ~0.5 m**, pause, back, pause
2. **right ~0.5 m**, pause, back, pause
3. **up ~0.3 m**, pause, down, pause

**Keep the orientation fixed while translating** — rotating mid-move couples the axes and
muddies the per-axis read.

### Pass criteria

| Check | Threshold | 2026-09-02 reference |
|---|---|---|
| correlation, all 3 axes | > 0.95 | +0.9935 / +0.9985 / +0.9997 |
| **sign, all 3 axes** | **CORRECT** | all correct |
| max tracking error | < 0.1 m (the declared std) | 0.104 m |
| invalid `xy_valid`/`z_valid` | 0 | 0 / 1195 |
| `dead_reckoning` | never | 0 / 1195 |
| EKF resets during moves | none | `xy_reset_counter` 4 → 4 |

The script refuses to pass a run where the rig didn't actually move — a stationary run cannot
prove a sign.

**A sign that comes back INVERTED means the frame conversion no longer matches the hardware.**
Do not adjust gains. Re-check §3.3 (mounting yaw) first, then the FLU→FRD flip in the bridge.

### Free confirmation of the lever arm

The constant offset the script reports should be **the negative of `EKF2_EV_POS_*`** —
`(−0.014, +0.024, +0.064)` against `(+0.011, −0.023, −0.070)` on 2026-09-02. That is EKF2
correctly reporting the **FC body origin** rather than the LiDAR IMU. If the offset is not
roughly `−EKF2_EV_POS`, §3.1 is wrong.

### Soak

Leave the stack running **≥ 15 minutes** and re-check. An earlier build regressed at ~8 min.

```bash
ps -o rss= -p $(pgrep -f fastlio_mapping)    # must be FLAT, ~156 MB
free -m | awk '/Swap/{print $3}'             # must stay 0
```

---

## 7. Finally

```bash
python3 scripts/px4/dump_params.py firmware/px4-params-post-teardown-$(date +%F).params
diff <(sort firmware/px4-params-pre-teardown-*.params) \
     <(sort firmware/px4-params-post-teardown-*.params)
```

Every difference should be one you made deliberately. Anything else is a param that didn't
survive — investigate before flying.

**Still not proven by any of this:** the aircraft has never been tested **in flight**, on either
board. Everything above is stationary-and-handheld. See the M2 bullet in
[`../CLAUDE.md`](../CLAUDE.md).

---

## Related

- [`ImplementationDoc.md`](ImplementationDoc.md) — M2 detail and the signed-off gate results
- [`fact-ev-frd-yaw-align.md`](fact-ev-frd-yaw-align.md) — why `heading_good_for_control` is False
- [`fact-ev-delay.md`](fact-ev-delay.md) — why `EKF2_EV_DELAY` stays 0
- [`xrce-agent-nx.md`](xrce-agent-nx.md) — the Tegra UART DMA fault
- [`ark-12s-payload-power-module.md`](ark-12s-payload-power-module.md) — battery current sensing
- [`../firmware/README.md`](../firmware/README.md) — the custom firmware and param restore points
