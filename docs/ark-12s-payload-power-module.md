# ARK 12S Payload Power Module on the ARK FPV — DONE (custom firmware required)

**Date:** 2026-09-01 · **Board:** ARK FPV (`HW arch: ARK_FPV`, *not* ARKV6X) · **Status: WORKING**
on a locally built firmware.

## TL;DR

The module was wired correctly and visible on I2C from the start, but **the stock ARK FPV firmware
does not contain the `ina238` driver**, so `SENS_EN_INA238` and `INA238_SHUNT` — the two parameters
[ARK's setup page](https://docs.arkelectron.com/products/power/ark-12s-payload-power-module/px4-instructions)
tells you to set — **did not exist on this aircraft**. There was no parameter-only fix.
ARK's instructions are written for **ARKV6X** (`boards/ark/fmu-v6x`), which does ship the driver;
ARK FPV is a different board target and does not.

Fixed by rebuilding `ark_fpv_default` from the FC's own commit with two extra lines in the board
config, flashing it, and then setting three parameters.

> ### ⚠️ This FC now runs a locally built firmware, not ARK's
> `boards/ark/fpv` upstream has no power monitor, so **any stock ARK/PX4 firmware update will
> silently remove the `ina238` driver** and battery current will go back to reading 0.00 A. Re-apply
> the patch below and rebuild after any firmware update — the same trap as the Tegra DTB fix in
> `xrce-agent-nx.md`. The build tree is at `~/PX4-Autopilot` (patch applied, uncommitted).
> Pre-flash parameter backup: `~/px4-params-backup-2026-09-01.params` (all 1053).

## Result (verified on the aircraft, 2026-09-01)

```
nsh> ina238 status
INFO  [SPI_I2C] Running on I2C Bus 1, Address 0x45
ina238_read: 3935 events, 2104.52us avg
ina238_com_err: 0 events                       <- clean bus, no retries
poll interval:  99993                          <- 10 Hz

nsh> listener battery_status
    voltage_v: 22.03750
    current_a: 0.41000          <- was 0.00000; the whole point
    discharged_mah: 49.58482    <- was 0.00000, now accumulating
    temperature: 45.00000       <- was nan; INA238 die temp, proof the driver is the publisher
    source: 0   id: 1   cell_count: 6   connected: True
```

Cross-check on the voltage: the board ADC read 22.33 V ~15 min before the swap, the INA238 reads
22.04 V after ~50 mAh out of the pack. Same pack, ~1.3% apart — the bus-voltage scaling is right.

**Not yet verified: absolute current accuracy.** `INA238_SHUNT` is set from the datasheet value,
not calibrated against a reference. 0.41 A at 22 V (~9 W) is plausible for FC + GPS + the module's
own regulators on the bench, but confirm against a watt meter or against what the charger puts back
after a flight before trusting `discharged_mah` for a failsafe.

Also confirmed unaffected by the reflash: `uxrce_dds_client` → `Running, connected`,
`timesync converged: true`, `SER_TEL1_BAUD=3000000`, `UXRCE_DDS_CFG=101`, and the `vision-only`
EKF2 profile. Parameter count went 1053 → 1056 (the three new INA238 params).

## Evidence (how this was diagnosed)

**The module is present and healthy on the bus.** ARK FPV declares exactly one *external* I2C bus —
`initI2CBusExternal(1)` in `boards/ark/fpv/src/i2c.cpp` — and that is the GPS/I2C port:

```
nsh> i2cdetect -b 1
40: -- -- -- -- -- 45 -- -- ...      <- INA238 (INA238_BASEADDR = 0x45)
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1e --   <- GPS compass
```

Sharing the line with the GPS is fine: different addresses, no conflict. (Bus 2 = internal, BMP388
at 0x76. Bus 4 = internal, IIS2MDC at 0x1e. Bus 3 does not exist.)

**The driver is absent.**

```
nsh> ina238 status
nsh: ina238: command not found
nsh> param show SENS_EN_INA238
 1053/1600 parameters used.          <- no match; the param does not exist
```

`help` confirms it: no `ina226`/`ina228`/`ina238` among the builtin apps.

**Both halves are missing upstream too** — checked against v1.17.0 *and* `main`:

| File | ark/fpv | ark/fmu-v6x (ARKV6X) |
|---|---|---|
| `default.px4board` → `CONFIG_DRIVERS_POWER_MONITOR_INA238` | **absent** | present |
| `init/rc.board_sensors` → `ina238 … start` | **absent** | present (buses 1/2/3) |

**The FC firmware is a plain upstream build**, not an ARK fork — `d6f12ad` resolves in
`PX4/PX4-Autopilot` ("Update NuttX fmu-v6x config use with Zenoh", #26213, 2026-04-24) and not in
`ARK-Electronics/PX4-Autopilot`. So rebuilding from that commit reproduces what is flashed today,
plus the INA238 addition, with nothing ARK-specific lost.

## What the FC was reading before the fix

`BAT1_SOURCE=0` (power module ADC), `BAT1_V_CHANNEL=-1` (board default channel), `BAT1_V_DIV=21.0`,
`BAT1_N_CELLS=6`, `BAT1_A_PER_V=-1.0`, `BAT1_CAPACITY=3300`.

```
listener battery_status:  voltage_v: 22.33   current_a: 0.00000   remaining: 0.42   source: 0
```

Voltage comes from the ARK FPV's **own battery pads**, via `board_adc` — not from the module. The
module's 6-pin CLIK-Mate carries only `VBRICK`/`SCL`/`SDA`/`GND`; **monitoring is I2C-only, there is
no analog sense output**, so no `BAT1_V_DIV`/`BAT1_A_PER_V` value can ever make PX4 read it.
`current_a` is pinned at 0 because `BAT1_A_PER_V=-1` and there is no analog current source at all —
that is the gap this module is meant to close.

## The fix

### 1. Firmware (two lines)

`boards/ark/fpv/default.px4board` — add:

```
CONFIG_DRIVERS_POWER_MONITOR_INA238=y
```

`boards/ark/fpv/init/rc.board_sensors` — add (bus 1 = the external/GPS I2C bus; `-t 1` binds it to
the `BAT1_*` params; `-k` keeps retrying so the driver survives being started before the pack is
plugged in):

```sh
if param compare SENS_EN_INA238 1
then
	ina238 -X -b 1 -t 1 -k start
fi
```

Build (done on the Jetson itself, 2026-09-01 — see the aarch64 gotchas below):

```bash
git clone --filter=blob:none https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
git checkout d6f12ad1c4f70ad3230afd7d86e971421e02fef4   # what the FC runs today
git submodule update --init --recursive
# ...apply the two edits above...

unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO ROS_VERSION LD_LIBRARY_PATH
source .venv/bin/activate
export PATH=/opt/arm-gnu-toolchain-13.2.Rel1-aarch64-arm-none-eabi/bin:$PATH
make ark_fpv_default
```

Output: `build/ark_fpv_default/ark_fpv_default.px4`.

**Build result, 2026-09-01:** clean, exit 0. `board_id: 59`, `magic: ARKFPVFWv1`,
`git_identity: v1.17.0` — QGC will accept it for this board. Verified the driver actually landed:
`ina238` appears as an NSH command string in the `.bin`, `ina238_main` and the `INA238::*` symbols
(~3.3 KB) are in the ELF, and the generated ROMFS `etc/init.d/rc.board_sensors` carries the
`if param compare SENS_EN_INA238 1 / ina238 -X -b 1 -t 1 -k start` block.

⚠️ **Flash is tight: 1 802 908 B of 1792 KB = 98.25%, leaving 31.3 KB.** The INA238 driver is a
small part of that — the ark_fpv target is simply near full. It links and fits, but there is not
room for much more. If a future addition overflows, drop something unused from
`boards/ark/fpv/default.px4board` (`CONFIG_DRIVERS_UAVCAN`, the airspeed/VTOL/rover modules) rather
than trying to shrink the power monitor.

**Three things `Tools/setup/ubuntu.sh` will not do for you on this board:**

1. **It does not install `arm-none-eabi-gcc` on aarch64.** Read it: the `gcc-arm-none-eabi` line is
   inside an `if [[ "${INSTALL_ARCH}" == "x86_64" ]]` block. On aarch64 it installs only
   `g++-aarch64-linux-gnu` / `g++-arm-linux-gnueabihf`, neither of which builds NuttX firmware.
   Fetch ARM's aarch64-hosted bare-metal toolchain yourself, matching what the FC already runs
   (`ver all` → `Toolchain: GNU GCC, 13.2.1 20231009`):

   ```bash
   wget https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-aarch64-arm-none-eabi.tar.xz
   sudo tar -xf arm-gnu-toolchain-13.2.rel1-aarch64-arm-none-eabi.tar.xz -C /opt
   # the extracted dir capitalises the R: /opt/arm-gnu-toolchain-13.2.Rel1-aarch64-arm-none-eabi
   ```

   (Ubuntu 22.04's own `gcc-arm-none-eabi` is 10.3.1, and PX4 CI does build with it — CI runs
   `ubuntu.sh` on `ubuntu:22.04` — but matching the shipped toolchain keeps the binary comparable
   to what ARK flashed.)

2. **Use a venv, not `pip3 install --user`.** This box runs ROS 2 Humble; PX4's `requirements.txt`
   asks for `numpy`, `pandas`, `matplotlib` and friends, and a `--user` install shadows the system
   `dist-packages` ROS links against. `python3 -m venv --system-site-packages .venv` keeps PX4's
   extras (`jinja2 jsonschema kconfiglib pyros-genmsg pyserial cerberus lark pkgconfig
   pycryptodome pyulog six sympy nunavut`) out of ROS's way.

3. **Unset the ROS environment before building.** `/opt/ros/humble` on `PYTHONPATH` shadows
   build-time Python modules. The `unset` line above is not optional if your shell sources ROS.

Apt packages needed on top of stock Ubuntu 22.04: `build-essential cmake ninja-build ccache
automake bison flex gperf genromfs gettext libtool pkg-config texinfo u-boot-tools vim-common
util-linux binutils-dev libelf-dev libexpat1-dev libgmp-dev libisl-dev libmpc-dev libmpfr-dev
libncurses5-dev libncursesw5-dev zlib1g-dev libssl-dev python3-venv`.

### Flashing

**Back the parameters up first** (QGC → Parameters → Tools → Save to file). The flight-critical set
for this project is reproducible anyway: `scripts/px4/set_params.py --apply` (UXRCE-DDS transport)
and `scripts/px4/ekf2_profile.py vision-only`.

Either flash from QGroundControl (Firmware page → "Advanced settings" → "Custom firmware file",
pick the `.px4`), or from the Jetson — but **stop `mavlink-router` first**, it owns
`/dev/ttyACM0` and a second opener corrupts the stream:

```bash
sudo systemctl stop mavlink-router.service
make ark_fpv_default upload
sudo systemctl start mavlink-router.service
```

After flashing, re-check `UXRCE_DDS_CFG=101` and `SER_TEL1_BAUD=3000000` before assuming the
XRCE link will come back (`scripts/px4/probe_params.py`).


### 2. Parameters (after flashing)

```bash
python3 scripts/px4/power_module_params.py          # dry run
python3 scripts/px4/power_module_params.py --apply
```

| Param | Value | Why |
|---|---|---|
| `SENS_EN_INA238` | `1` | starts the driver from `rc.board_sensors` |
| `INA238_SHUNT` | `0.0001` | the module's 0.1 mΩ shunt (PX4 default is 0.0005) |
| `BAT1_V_CHANNEL` | `-2` | **not in ARK's instructions, but needed here** |

`BAT1_V_CHANNEL = -2` is PX4's documented "disable analog monitoring … use digital monitoring
exclusively". Without it, `AnalogBattery` (index 1, `SOURCE_POWER_MODULE`) and the INA238 (`-t 1`,
also `SOURCE_POWER_MODULE`) **both** publish `battery_status` id 1 — `Battery::publishBatteryStatus`
only gates on `_source == BAT1_SOURCE`, not on who got there first — and the topic ends up
alternating between the ADC's guess and the module's measurement. On an ARKV6X this doesn't bite,
because there the ADC brick and the INA are separate battery indices.

Leave alone:
- `BAT1_SOURCE = 0` — `ina238.cpp` constructs its `Battery` with
  `battery_status_s::SOURCE_POWER_MODULE`, which *is* 0. Setting it to 1 ("External") would silence
  the module.
- `INA238_CURRENT = 327.68` (default) — with a 0.1 mΩ shunt that is 32.77 mV full-scale, inside the
  INA238's ±40.96 mV high-resolution ADC range, giving a 10 mA current LSB. Correct as-is.
- `BAT1_A_PER_V` — dead once the analog path is off.

Check these match the pack actually fitted: `BAT1_N_CELLS` (currently `6`; the module is rated to
12S, so this must change if the pack does) and `BAT1_CAPACITY` (currently `3300` mAh).

### 3. Verify

```bash
python3 scripts/px4/px4_shell.py "ina238 status" "listener battery_status"
```

Expect the driver on bus 1 @ 0x45 and a **non-zero `current_a`**. Still 0.00000 → the analog battery
is still the publisher; re-check `BAT1_V_CHANNEL=-2` and that you rebooted twice (the shunt is only
read in `INA238::init()`).

## Gotcha worth remembering

**"ARK FPV" and "ARKV6X" are different PX4 board targets with different driver sets.** `CLAUDE.md`
described the FC as "ARK FPV (ARKV6X)"; `ver all` says `HW arch: ARK_FPV` / `HW type: ARKFPV`. Any
ARK doc or PX4 page that assumes ARKV6X — including every power-module setup guide — may reference
parameters this board's firmware simply does not have. `param show <NAME>` printing no match is the
quick test.
