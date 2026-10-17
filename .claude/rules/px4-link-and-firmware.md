---
paths:
  - "scripts/px4/**"
  - "firmware/**"
  - "scripts/uart_loopback_test.sh"
  - "scripts/01_test_uart.sh"
  - "scripts/desk_check.sh"
  - "scripts/gcs/telemetry.py"
  - "docs/xrce-agent-nx.md"
  - "docs/mavlink-setup.md"
  - "docs/uart_bringup_log.md"
  - "docs/ark-12s-payload-power-module.md"
  - "docs/fact-ark-logloader-loop.md"
---

# PX4 flight controller: link, tools, firmware, crash dumps

Scope: how the Jetson talks to the ARK FPV (uXRCE-DDS on TELEM1, MAVLink over USB), the tooling in `scripts/px4/`, the custom firmware in `firmware/`, and the `uxrce_dds_client` hardfault, which is still a flight blocker. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

## The FC and how to reach it

- **Never open the FC serial devices directly** — a second reader corrupts the stream, and `stty`
  is per-device, not per-fd. This holds on both boards, but for *different* reasons:
  - **Orin Nano (ARK OS):** ARK's own services owned both links (`dds-agent.service`,
    `mavlink-router.service`), running as user `jetson`.
  - **Orin NX (was stock JetPack; REIMAGED WITH ARK OS 2026-08-31):** ARK's services are back and
    own both links — `dds-agent.service` (which hardcodes `-b 3000000` in
    `/usr/lib/ark-os/libexec/start_dds_agent.sh`) and `mavlink-router.service`, as user `jetson`.
    Our `/etc/mavlink-router/main.conf` is **no longer the active config** — ARK's
    `/etc/ark-os/mavlink-router.conf` is — and our `xrce-agent.service` is disabled. ARK's config
    does still expose `companion_aux` on 14569, so `scripts/px4/*.py` work unchanged.
    See `docs/mavlink-setup.md`.
  Either way, go through the MAVLink UDP endpoints: **14569** (`companion_aux`) for
  `scripts/px4/`, **14550** for QGroundControl. This cost weeks — see `docs/uart_bringup_log.md`.
- **Flight controller is ARK FPV running PX4 v1.17.0 + uXRCE-DDS**, **NOT ArduCopter + MAVROS**.
  **ARK FPV is not ARKV6X** — `ver all` reports `HW arch: ARK_FPV`. They are different PX4 board
  targets with different compiled-in driver sets (`boards/ark/fpv` vs `boards/ark/fmu-v6x`), so an
  ARK or PX4 doc written for ARKV6X can name parameters this firmware does not have. `param show
  <NAME>` printing no match is the quick test; `docs/ark-12s-payload-power-module.md` is the worked
  example. XRCE runs on **TELEM1 (FC `/dev/ttyS6`, UART7) ↔ Jetson `/dev/ttyTHS1` @ 3000000**
  (it must match ARK's agent, which hardcodes 3000000, and it does).
  MAVLink runs separately on USB-C. `px4_msgs` is at `release/1.17`. Frame conversion for M2 lives in `px4_odom_bridge` (**FLU→FRD**); M3's
  `px4_bridge.py` was missing and was rewritten as `px4_offboard_bridge.py` (see
  `px4-bridges-offboard.md`). `docs/ImplementationDoc.md` is the current source of
  truth for M2/M3; `docs/mavros-setup-notes.md` and `docs/plantofly.md` are superseded
  ArduCopter/MAVROS-era notes — historical, not ground truth.

## Board differences (Orin Nano  vs.  Orin NX)
The repo was developed on the first and now runs on the second. **As of 2026-08-31 the NX has been
reimaged with ARK OS**, so most of the ARK-era pain in this file applies again — any note saying
the NX runs stock JetPack with no ARK OS is stale. Re-confirmed on the NX, 2026-08-31:

| | Orin Nano (ARK OS) | **Orin NX (current, ARK OS since 2026-08-31)** |
|---|---|---|
| FC serial links | owned by ARK services | owned by ARK services again |
| `mavlink-router` | ARK's `mavlink-router.service` | ARK's, `/etc/ark-os/mavlink-router.conf` (ours, `/etc/mavlink-router/main.conf`, is inactive) |
| uXRCE-DDS agent | ARK's `dds-agent.service` | ARK's `dds-agent.service`, `-b 3000000` hardcoded |
| `pymavlink` | preinstalled | installed via `sudo pip3 install pymavlink` |
| MAVROS leftovers | present (ArduCopter era) | absent — which is why `setup.sh` needed `ros-humble-mavros-msgs` added |
| Wired iface | `eno1` | `eno1` (same; the script still auto-detects) |
| RAM | 8 GB | 16 GB |

**The FC itself is unchanged and keeps its parameters** — they live on the flight controller, not
the Jetson. Verified on the NX via `scripts/px4/probe_params.py`: `UXRCE_DDS_CFG=101` (TELEM1),
`SER_TEL1_BAUD=3000000`, and the full `vision-only` EKF2 profile (`EKF2_EV_CTRL=11`,
`EKF2_HGT_REF=3`, `EKF2_GPS_CTRL=0`) all survived the board swap. Note a `SER_TEL1_BAUD` change
made over MAVLink during debugging did **not** persist across the FC power cycle — it read back
correctly at the time, then reverted to 3000000. Re-read the param after any reboot before
trusting it.

**Resolved 2026-08-31:** TELEM1 *is* physically wired to the Jetson's `/dev/ttyTHS1` — proven by
the received bit rate tracking `SER_TEL1_BAUD` across an FC reboot, and by the line going silent
when the harness is unplugged. The harness is not the problem; the Jetson's own UART data path is.
See `docs/xrce-agent-nx.md`.

## M2 link bring-up: the Tegra UART DMA fault

- **M2 (FC link + external odometry): DONE — GATE SIGNED OFF 2026-09-02.** uXRCE-DDS is live:
  `uxrce_dds_client` reports **`Running, connected`**, `Payload tx: 35910 B/s`,
  `timesync converged: true`; **65 `/fmu/*` topics**; `/fmu/out/vehicle_odometry` at **~100 Hz**.
  **The root cause was Tegra HSUART DMA on the Jetson, not the wire and not the FC.**
  `serial@3100000` declared `dmas`/`dma-names`, and the DMA path delivered **zero-filled buffers**
  to the tty layer. Proof, with no FC in the picture: TX jumpered to its own RX, writes of ≤4 bytes
  echoed exactly, writes of ≥8 bytes came back all zeros — correct byte count, `frame=0`, at
  115200/921600/3000000 alike. Removing those two DT properties forces PIO; the loopback then
  passed 18/18 and the XRCE session came up immediately. The agent's replies had been leaving the
  Jetson as runs of zeros, so the FC discarded them and the client retried forever.
  **The fix is a boot-time DTB override** (`FDT` line in `/boot/extlinux/extlinux.conf` →
  `…-nv-pio.dtb`). **It does NOT survive a JetPack upgrade or reflash** — `nv-update-extlinux`
  rewrites that file and the stock DTB is still in place, so an update silently restores the broken
  DMA path and the symptom looks exactly like a dead wire again.
  **Run `bash scripts/uart_loopback_test.sh` after any system update.** PIO cost is mild:
  1286 IRQ/s, agent 7.3% CPU, load 0.66. `ttyTHS2` still has its `dmas` — same fix if you move there.
  **Both earlier "dead wire" conclusions are retracted.** They came from reading captures taken
  *through the broken receiver*: the FC's session frame arriving as `00×16 fd 02 00 00 00 cf 12` was
  the zeroed-payload signature, and the `0x7E` begin flag was inside the zeroed region.
  **Always loopback-test a UART before trusting anything measured through it.**
  Read `docs/xrce-agent-nx.md` before touching this. MAVLink is working — see `docs/mavlink-setup.md`.

## Tools and firmware

- `scripts/px4/` — **FC tooling, all over MAVLink UDP 14569** (never a serial port; on the NX that
  port is served by *our* mavlink-router, `/etc/mavlink-router/main.conf` — `docs/mavlink-setup.md`):
  `px4_shell.py` (PX4 NuttX shell — the fastest way to answer "is the client even running?", and
  the only way to see what the FC actually *received*: `listener trajectory_setpoint`,
  `listener health_report`), `probe_params.py`, `ekf2_profile.py` (vision-only | vision-gps |
  stock), `diag_health.py`, `fake_odom_pub.py` (bench-test the bridge with no LiDAR),
  `set_params.py`. See its `README.md`.
  **Three of them are ROS 2, not MAVLink** — source the workspace instead: `m2_gate_check.py`
  (the M2 gate), `m3_bridge_check.py` (the M3 bench gate), `offboard_engage.py` (engage Offboard /
  arm / disarm — the human half of M3; the bridge itself never arms and never changes mode).
- `firmware/` — **the custom `ark_fpv` PX4 build this FC runs**, its two-line board patch, and the
  **full FC parameter dumps** (the restore points; `firmware/README.md` indexes them). Take one with
  `python3 scripts/px4/dump_params.py firmware/px4-params-$(date +%F).params` **before any param
  change** — read-only, QGC format. `probe_params.py` reads a fixed shortlist and is NOT a backup.
  Params live on the FC, so they survive a Jetson reflash but not a PX4 reflash. Needed because upstream `boards/ark/fpv` has no power monitor
  driver. **Any stock firmware update reverts it** — see `firmware/README.md`.

## Gotchas talking to the FC

- **After an FC power cycle (or any USB re-enumeration), `mavlink-router` does NOT auto-reconnect
  — restart it.** `sudo systemctl restart mavlink-router`, then the PX4 comp-1 heartbeat returns on
  14569. Symptom when you forget: `scripts/px4/*` say "no PX4 heartbeat" while `ttyACM0` is present
  and enumerates as the app (`lsusb` → `3185:003b`), and a raw listen on 14569 shows the ARK
  components (191/192/236) but **no comp-1**. Cost time twice on 2026-09-13. This is distinct from a
  wedged FC: if a mavlink-router restart does not bring comp-1 back, the FC itself is hung and needs
  a power cycle. (A 3 Mbaud garbage flood on the XRCE UART, `xrce_cycle_test.py --rx random`, hangs
  the FC regardless of firmware — a UART DoS unrelated to the hardfault; don't run it routinely.)
- **`wait_heartbeat()` grabs the wrong component.** ARK OS components (comp 191/192/236,
  `autopilot=8`) also heartbeat on the shared UDP bus; you get sysid 0 (broadcast) and param reads
  silently return `None`. Filter for `autopilot == MAV_AUTOPILOT_PX4 and srcComponent == 1`.
- **PX4 ships INT32 params bit-packed in MAVLink's float `param_value`.** Reading `.param_value`
  directly shows denormal garbage (`1.4153e-43` is really `101`). Decode with
  `struct.unpack("<i", struct.pack("<f", raw))`.
- **A `/proc/*/fd` scan as `ideas-ad` reports "no process holds it"** even when ARK's services
  (running as `jetson`) do. Check `/etc/ark-os/*.conf` or use `sudo lsof`.

## Firmware: the power module

- **Battery voltage AND current sensing both work now (2026-09-01) — but only on a locally built
  firmware.** An **ARK 12S Payload Power Module** (INA238, 0.1 mΩ shunt) is fitted on the GPS/I2C
  port, sharing SCL/SDA with the GPS+compass (fine — 0x45 vs 0x1e). `battery_status` now reports
  **`current_a` ≈ 0.41 A idle, `discharged_mah` accumulating, `temperature` 45 °C** from the INA238;
  before this it was `current_a: 0.00000` and `temperature: nan` with voltage coming off the ARK
  FPV's own battery pads.
  **The catch: `boards/ark/fpv` has no power monitor driver upstream** — not in v1.17.0, not in
  `main`. `SENS_EN_INA238`/`INA238_SHUNT` simply don't exist in stock ARK FPV firmware, so no
  parameter change could ever have worked. ARK's setup page is written for **ARKV6X**. The FC is
  now running a firmware built here from its own commit (`d6f12ad`) with
  `CONFIG_DRIVERS_POWER_MONITOR_INA238=y` plus an `ina238 -X -b 1 -t 1 -k start` line in
  `rc.board_sensors`. Params: `SENS_EN_INA238=1`, `INA238_SHUNT=0.0001`, and **`BAT1_V_CHANNEL=-2`**
  (not in ARK's instructions — without it the board ADC and the INA238 both publish
  `battery_status` id 1 and fight over the topic).
  **⚠️ Any stock firmware update silently removes the driver and current goes back to 0.00 A** —
  same trap as the Tegra DTB fix. Rebuild from `~/PX4-Autopilot` (patch applied) after any update.
  Pre-flash param backup: `~/px4-params-backup-2026-09-01.params`. Flash sits at **98.25%** full,
  31 KB free. **Absolute current accuracy is still uncalibrated** — the shunt is the datasheet
  value; check against a watt meter before trusting `discharged_mah` for a failsafe.
  Full write-up: `docs/ark-12s-payload-power-module.md`. Params: `scripts/px4/power_module_params.py`.

## ⛔ The `uxrce_dds_client` hardfault (G0.1) — a flight blocker

- **⛔ The `uxrce_dds_client` hardfault — REPRODUCED ON DEMAND; the RXDMA fix FAILED; a CLIENT-SIDE FIX
  IS FLASHED and passed 30/30 reconnects on 09-13 and **455/455 on 09-14 (486 clean cycles, 0
  resets)**; the G0.1 cycle count is met, the 14 h soak and the shell-path reproducer are not
  (see "G0.1 cycle count MET" below).**
  - **The archive: 35 dumps.** 24 are from the field: 22 of the classic crash, the newest
    2026-09-13, plus 1 uORB and 1 logger. The other 11 were induced on the bench.
  - **Reproduced.** `scripts/px4/xrce_cycle_test.py` stops `dds-agent` and drives or idles the XRCE
    UART across the client's reconnect. On the original firmware it reset the FC `--rx random`
    5/5, every dump the exact field signature: `pc→uart_datareceived`, `lr→uart_close`,
    `r0:0x24000000`, `cfsr:0x8200`, corrupted TCB at `0x24001000`.
    `scripts/px4/xrce_fault_repro.py` (FC-shell `stop`/`start`, no sudo) is a second, milder
    reproducer: 5/34, scattered PCs.
  - **`CONFIG_UART7_RXDMA=n` was built, flashed and DISPROVEN.** The fault is `uart_close` racing
    `uart_datareceived` in the NuttX serial driver, not the DMA teardown (`up_dma_shutdown` was a
    red-herring `r3` value). That build also *hangs* under a 3 Mbaud flood.
  - **Now on the FC: `firmware/ark-fpv-uxrce-no-uart-close-on-reconnect.patch`** (01:24 build, RX
    DMA back on). The client rebuilds only the XRCE session on its open UART, so `uart_close` never
    runs on a reconnect. Results (`firmware/g01/`):
    - **`--rx none`** (a real session loss): **30/30, 0 resets, 0 dumps**. Pre-fix it crashed 1/1.
    - **`--rx random` flood: INCONCLUSIVE, despite the commit message's "15/15".** The FC's
      `boot_ms` froze at 967776 for cycles 2–15, so it stopped reporting after cycle 1 — the known
      flood wedge.
  - **Not yet done:** the shell-path reproducer on the fix, the 14 h soak, and plan §2.2's
    ≥ 500-cycle count. **Still a flight blocker until they pass.** Full write-up:
    `firmware/faults/README.md`.
  - **Flooding the XRCE UART at 3 Mbaud wedges this FC on any firmware** (power cycle needed) — it is
    a UART DoS, not a regression test. After an FC power cycle, restart `mavlink-router.service` if
    `scripts/px4/*` see no PX4 heartbeat.

- **G0.1 cycle count MET: `xrce_cycle_test.py --rx none --cycles 455 --yes` → 455/455
  session losses, 0 FC resets, 0 dumps, card empty after** (2.6 h, bench, disarmed;
  `firmware/g01/cycles-20260914_115558.csv`). With 09-13's 30 that is 486 clean cycles on the
  client fix against a baseline that crashed on cycle 1. `dds-agent` came back by itself
  (27 `/fmu/out` topics). **Still open for G0.1: the 14 h powered soak with the full stack
  and the shell-path reproducer (`xrce_fault_repro.py`) on this build** — the hardfault stays
  a flight blocker until those pass, but the reconnect path itself is now well measured.

- **⛔ `firmware/faults/` holds a FLIGHT BLOCKER, and the previous session's read of it was wrong.**
  That session concluded the FC hardfaults were confined to the Tegra-UART-DMA window and that
  there had been **none since the fix**. It never looked at the SD card. **There were three more
  on it** — and the "none since" claim was committed 21 minutes after the newest one.
  *(Superseded 2026-09-13 — the current state is the G0.1 bullet above: 35 dumps, 22 classic field crashes, RXDMA fix failed, client-side fix flashed.
  The history below is kept as written on 2026-09-12.)*
  Full picture then (twenty-two archived and decoded; five added 2026-09-10, nine more 2026-09-12):
  **The `uxrce_dds_client` hard fault has happened TWENTY times, across two firmware builds,
  eighteen AFTER the DMA fix** (2026-08-29, 08-31 on ARK's factory build; 09-01 06:38, 09-01
  14:21, 09-04 01:54, 09-04 04:50, 09-07 00:00, 09-08 01:52, 09-08 02:44, 09-09 09:23, 09-11
  11:25, 09-11 16:58, 09-11 17:06, 09-12 01:21, 09-12 01:38, 09-12 01:44, 09-12 02:08, 09-12
  23:15, 09-12 23:29, and once with the RTC unset — logged `2000-01-01 00:02:48`, i.e. **2m48s
  into a boot**, before GPS time sync — all on *our* build). Same `pc:0x08022f60` every time.
  The DMA window was coincidence, not cause. **It is accelerating: nine of the twenty are from
  2026-09-11 to 09-12, ~1 per 4.5 h.**
  **Root cause is now resolved** — the addresses decode against our own ELF:
  `pc → uart_datareceived` (`serial.c:1757`), `lr → uart_close` (`serial.c:725`),
  `r3 → up_dma_shutdown` (`stm32_serial.c:2353`). A **use-after-free in NuttX's serial driver**:
  `uart_close` tears down the RX DMA while `uart_datareceived` is still live on that buffer
  (`r0=0x24000000` = AXI SRAM, `r1=0x80`). It fires when the XRCE client **closes and reopens its
  UART**, i.e. on session loss — the event most likely to happen in flight when something goes
  wrong. **Our own build enables RX DMA on that UART** (`CONFIG_UART7_RXDMA=y`); disabling it was
  the candidate fix — **tried 2026-09-13 and it FAILED** (see the G0.1 bullet above).
  **A different fault, once** (2026-09-02 02:09): `wq:SPI1`, `pc:0x00000000` — a branch through a
  **null function pointer**, from `uORB::SubscriptionCallbackWorkItem::call()` →
  `uORB::DeviceNode::write()`. A subscriber-lifetime race in uORB.
  **A third, also once** (2026-09-07 22:20): task `logger`, `pc:0x08023d5e` → `nxsem_wait`
  (`sem_wait.c:197`) with `r0:0x00000000` — a **null semaphore pointer**. Distinct from both of
  the above; don't fold it into either.
  None is caused by anything in this repo; all three are upstream PX4/NuttX concurrency bugs. None
  is fixed. A hard fault is an FC **reset** — on the bench an inconvenience, in flight a fall.
  **Check the card, don't trust the directory:** `python3 scripts/px4/clear_fault_logs.py`
  (read-only) lists what is actually on `/fs/microsd`; `--clear` archives into `firmware/faults/`,
  verifies, and only then deletes. Cleared this way on 2026-09-02, 2026-09-04 (three dumps) and
  2026-09-10 (five). See `firmware/faults/README.md`.

## ARK OS: the logloader loop

- **✅ ARK OS `logloader.service` looped for a day — now DISABLED and cleaned up (2026-09-13,
  16:27–16:45, at the user's instruction).** Full record: `docs/fact-ark-logloader-loop.md`.
  - **What it did.** It re-uploaded ONE ULog (`LOG0135_2026-09-12T23:03:08Z.ulg`) to the local
    flight-review every ~2 s from **2026-09-12 19:03**, surviving reboots:
    - **21,022** entries, **344 GB**, ~500 MB/min, ~42 % of a core;
    - the root disk reached **95 % (23 GB free)**.
  - **Why it matters.** On the bench full graph, `/lidar_slam/odom` arrived **3.9 s late** once while
    it ran (normal stamps, two independent subscribers): a whole-Jetson stall. **Every 09-12 Stage A
    flight ran with it looping** — the best lead yet for that day's odometry-age problem, not proven.
  - **Cause (from the installed legacy source, don't re-derive):**
    - logloader identifies a log by `hash(date + size)`.
    - It downloaded LOG0135 **while PX4 was still writing it**, so the staged copy is truncated
      (17,416,192 B).
    - The FC then listed the finished 18,378,936 B log as a new row.
    - Each upload re-derives the identity from the local file's size and marks the OLD row, so the
      new one never clears.
  - **The SD card was NOT the trigger.** The FC's `/fs/microsd/log/2026-09-12/23_02_29.ulg`
    (18,378,936 B) was **kept**: it is the only complete copy.
  - **Done:**
    - `systemctl disable` (removed from `multi-user.target` and `ark-os.target`);
    - the stuck row marked uploaded (DB backup `~/backups/logloader-db-20260913/`);
    - all 20,945 copies sha256-verified identical, then 21,021 entries deleted with flight-review's
      own delete steps, keeping the first upload (DB backup `~/backups/flight-review-db-20260913/`);
    - `Vehicle.LatestLogId` re-pointed; flight-review restarted, 176 logs.
    - **364.8 GB freed → root disk 18 %, 364 GB free.**
  - **Check before any flight day:**
    - `systemctl is-enabled logloader` says `disabled` — an ARK OS update or the ARK UI could
      re-enable it;
    - `df -h /` shows ≥ 20 GB free.

    Re-enabling is safe for LOG0135, but the same bug can repeat for any log fetched mid-write.

See also: `px4-estimator.md` (EKF2 and params), `px4-bridges-offboard.md`.
