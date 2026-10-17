# firmware/ — the custom ARK FPV build that reads the power module

**This aircraft does not run stock ARK firmware.** Everything here exists because
`boards/ark/fpv` upstream has **no power monitor driver** — see
[`../docs/ark-12s-payload-power-module.md`](../docs/ark-12s-payload-power-module.md) for the full
diagnosis.

> ### ⚠️ Re-apply after any FC firmware update
> A stock ARK/PX4 flash silently removes the `ina238` driver and `battery_status.current_a` goes
> back to `0.00000`. There is no warning and no error — it just looks like the power module died.
> Rebuild from `ark-fpv-ina238.patch` and reflash. Same class of trap as the Tegra DTB fix in
> [`../docs/xrce-agent-nx.md`](../docs/xrce-agent-nx.md).

## Contents

| File | What it is |
|---|---|
| `ark-fpv-ina238.patch` | The whole change: `CONFIG_DRIVERS_POWER_MONITOR_INA238=y` in `boards/ark/fpv/default.px4board`, and an `ina238 -X -b 1 -t 1 -k start` block in `boards/ark/fpv/init/rc.board_sensors`. Apply against PX4 `d6f12ad`. |
| `ark-fpv-uxrce-no-uart-close-on-reconnect.patch` | **⏳ THE FC RUNS THIS (flashed 2026-09-13, 01:24 build) — bench-tested, NOT yet through the G0.1 gate.** `xrce_cycle_test.py --rx none` (real session losses, idle line): **30/30 with 0 resets and 0 dumps**, where the pre-fix firmware crashed 1/1. `--rx random` (3 Mbaud flood): **inconclusive** — after cycle 1 the FC's `boot_ms` froze at 967776 for cycles 2–15 (`g01/cycles-20260913_015105.csv`), i.e. it stopped reporting, the known flood wedge; only cycle 1 carries evidence. Still to do: `xrce_fault_repro.py` on this build, the 14 h powered soak, and the ≥ 500-cycle count in `docs/m5-m6-plan.md` §2.2. Patches `uxrce_dds_client.cpp/.h` (code we build) so a session loss rebuilds only the XRCE session on the already-open UART instead of closing/reopening it — `uart_close` never runs, so the race that hard-faults the FC cannot fire. Bounded fallback: a full reopen only after 20 failed reconnects (rare, wedged-transport case). Keeps RX DMA ON (unlike the failed patch below). Apply ON TOP of `ark-fpv-ina238.patch`, `d6f12ad`. **Validate:** `xrce_cycle_test.py --rx none/--rx random` and `xrce_fault_repro.py` must give 0 crashes; then `m2_gate_check.py`/`m3_bridge_check.py`. See [`faults/README.md`](faults/README.md). |
| `ark-fpv-uart7-rxdma-off.patch` | **⛔ FAILED FIX — flashed and disproven 2026-09-13, kept as negative evidence, DO NOT FLASH.** Disables `CONFIG_UART7_RXDMA` on UART7 (TELEM1/XRCE). The hypothesis was that the crash was RX-DMA teardown; it is not. On this firmware the reconnect path still hardfaults with the same `uart_datareceived`←`uart_close` signature, and a 3 Mbaud RX flood now HANGS the FC instead of auto-resetting. Superseded on the FC by the client-side fix above (built on `ark-fpv-ina238.patch` with RX DMA back ON). Full writeup: [`faults/README.md`](faults/README.md) §'THE CONFIG_UART7_RXDMA=n FIX FAILED'. |
| `ark_fpv_default-ina238-2026-09-01.px4` | The binary flashed to the FC on 2026-09-01. `board_id: 59`, `magic: ARKFPVFWv1`, `git_identity: v1.17.0`, image 1 802 908 B. |
| `px4-params-preflash-2026-09-01.params` | All 1053 params read off the FC immediately **before** flashing. The restore point. |
| `px4-params-postflash-2026-09-01.params` | All 1056 params **after** flashing and configuring. |
| `px4-params-pre-evpos-2026-09-02.params` | All 1056 params **before** the external-vision lever arm was set (2026-09-02). The restore point for that change. |
| `px4-params-post-evpos-2026-09-02.params` | All 1056 params **after** it. Diffing the two shows only `EKF2_EV_POS_X/Y/Z` and `_HASH_CHECK` moved. |
| `px4-params-pre-altmode-2026-09-17.params` | All 1056 params **before** `MPC_ALT_MODE` was changed 2 → 1 (terrain following) on 2026-09-17. The restore point for that change. |
| `px4-params-2026-09-21-pre-retune.params` | All 1056 params **before** the first retune change, `IMU_DGYRO_CUTOFF` 55 → 40 (2026-09-21). Against the 09-17 dump only `MPC_ALT_MODE` and calibration/bookkeeping values differ, so it is also the retune's baseline. The restore point for the retune. |
| `px4-params-2026-09-21-post-batscale-baroautocal.params` | All 1056 params **after** the user set `MC_BAT_SCALE_EN` 0 → 1 and `SENS_BAR_AUTOCAL` 1 → 0 (2026-09-21, from QGC, after the fourth retune flight). Against the pre-retune dump only three settings differ: those two and `IMU_DGYRO_CUTOFF` 55 → 40. The rest is bookkeeping PX4 writes itself: `CAL_BARO0_OFF` 470 → 543 (the last auto-cal before it was switched off, now frozen), `CAL_ACC0_*OFF` (in-run accel bias committed on landing), `COM_FLIGHT_UUID`, `LND_FLIGHT_T_LO`. No dump exists between the two user changes and the flights, because they were made outside one. |
| [`faults/`](faults/) | Crash artifacts pulled off the FC. **The `uxrce_dds_client` hardfault (`uart_datareceived`←`uart_close`, `r0:0x24000000`) is REPRODUCED ON DEMAND** (2026-09-13, `xrce_cycle_test.py`): a real session loss resets the FC 5/5, exact field signature. **The `CONFIG_UART7_RXDMA=n` fix was flashed and FAILED** — the crash is `uart_close` racing `uart_datareceived`, not the DMA teardown. **The client-side no-close-on-reconnect fix is flashed and passed 30/30 reconnects** (flood inconclusive; soak pending). See [`faults/README.md`](faults/README.md). |

Take these with **`python3 scripts/px4/dump_params.py <out.params>`** — a full read-only dump in
QGC's format, so it loads straight back via QGC → Parameters → Tools → Load from file. Do it
before *any* parameter change, not just a reflash.

`sha256(ark_fpv_default-ina238-2026-09-01.px4)`
`= ac3915344564d4b33456071e66727f6aa77bca8aae88b4e8b50153286e5e9f37`

## Provenance

Built on the Jetson from **upstream** PX4 at the exact commit the FC was already running —
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4` (`PX4/PX4-Autopilot`, v1.17.0 stable; verified it is
*not* an ARK fork commit) — with `arm-gnu-toolchain-13.2.rel1`, matching the `GNU GCC, 13.2.1
20231009` the shipped firmware reported. So the only functional delta from what ARK flashed is the
INA238 driver.

Working build tree: `~/PX4-Autopilot` (patch applied, uncommitted). Not vendored here — the patch
plus the commit hash reproduces it.

**Flash is 98.25% full** (1 802 908 B of 1792 KB, 31.3 KB free). If a future addition overflows,
drop something unused from `default.px4board` rather than trying to shrink the power monitor.

## Rebuild

`Tools/setup/ubuntu.sh` will **not** install `arm-none-eabi-gcc` on aarch64 — that line sits inside
an `x86_64` guard. Full gotcha list (venv vs. ROS, unsetting `PYTHONPATH`) is in
[`../docs/ark-12s-payload-power-module.md`](../docs/ark-12s-payload-power-module.md).

```bash
git clone --filter=blob:none https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
git checkout d6f12ad1c4f70ad3230afd7d86e971421e02fef4
git submodule update --init --recursive
git apply /path/to/SUPER-Human-DTP/firmware/ark-fpv-ina238.patch

wget https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-aarch64-arm-none-eabi.tar.xz
sudo tar -xf arm-gnu-toolchain-13.2.rel1-aarch64-arm-none-eabi.tar.xz -C /opt

python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install jinja2 jsonschema kconfiglib pyros-genmsg pyserial cerberus lark pkgconfig \
            pycryptodome pyulog six sympy nunavut

unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO ROS_VERSION LD_LIBRARY_PATH
export PATH=/opt/arm-gnu-toolchain-13.2.Rel1-aarch64-arm-none-eabi/bin:$PATH   # capital R
make ark_fpv_default
```

## Reflash

Back the params up first (`python3 scripts/px4/probe_params.py` is not enough — take a full dump,
or QGC → Parameters → Tools → Save to file). **Props off** — the board reboots into its bootloader.

`mavlink-router` owns `/dev/ttyACM0`; a second opener corrupts the stream, so stop it:

```bash
sudo systemctl stop mavlink-router.service
cd ~/PX4-Autopilot && make ark_fpv_default upload
sudo systemctl start mavlink-router.service

python3 scripts/px4/power_module_params.py --apply   # then reboot the FC
python3 scripts/px4/px4_shell.py "ina238 status" "listener battery_status"
```

## What the 2026-09-01 reflash actually changed

Diffing the pre- and post-flash dumps, **only these params moved** — no silent resets:

```
BAT1_V_CHANNEL   -1  ->  -2                 set by us
SENS_EN_INA238   (absent)  ->  1            set by us
INA238_SHUNT     (absent)  ->  0.0001       set by us
INA238_CURRENT   (absent)  ->  327.68       new param, left at default
CAL_BARO0_OFF    407.715   ->  412.598      baro re-zeroed on boot, expected
_HASH_CHECK      changed                    PX4's param-set hash, expected
```

Notably intact: `UXRCE_DDS_CFG=101`, `SER_TEL1_BAUD=3000000` (this one has silently reverted across
a power cycle before — always re-read it), and the whole `vision-only` EKF2 profile
(`EKF2_EV_CTRL=11`, `EKF2_HGT_REF=3`, `EKF2_GPS_CTRL=0`).
