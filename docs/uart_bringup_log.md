# FC UART Bring-up Log

> ⚠️ **This log covers the FIRST UART investigation (`UXRCE_DDS_CFG=0` + port ownership).**
> A **second, genuine hardware-level fault** was found on 2026-08-31 and is *not* described here:
> Tegra HSUART **DMA** on `serial@3100000` was delivering **zero-filled buffers** — correct byte
> count, zero framing errors, payload replaced by `0x00`. It made the link look dead and produced
> two incorrect "the wire is broken" conclusions. Fix: remove `dmas`/`dma-names` from that DT node
> to force PIO. See [`xrce-agent-nx.md`](xrce-agent-nx.md).
>
> **Before trusting anything in this log, or any new serial measurement, run
> `bash ../scripts/uart_loopback_test.sh`.** Captures taken through a broken receiver are worse
> than no data — they look authoritative and are wrong.

Replaces `plantofly.md` / `nohardware.md` / `README.md`, which described an
ArduCopter-based setup. **The flight controller actually runs PX4** — those
docs' parameter names (`ARMING_CHECK`, `EK3_SRC1_*`, `SERIALx_PROTOCOL`,
GUIDED-mode arming) don't exist on this FC and were deleted to avoid
confusion. This log tracks the real, PX4-based bring-up as it happens.

## Hardware

- FC: ARK FPV, running PX4 (confirmed via `MAV_0_CONFIG`/`MAV_1_CONFIG`/
  `SER_TEL1_BAUD`/`SER_TEL2_BAUD` parameter names in QGroundControl — these
  are PX4-only; ArduPilot's equivalents would be `SERIALx_PROTOCOL`/
  `SERIALx_BAUD`, which don't exist here).
- Companion computer: Jetson Orin Nano, `/dev/ttyTHS1` (40-pin header,
  pins 8/10).
- Reference: [ARK FPV pinout docs](https://docs.arkelectron.com/products/flight-controller/ark-fpv/pinout),
  [ARK UART/TELEM troubleshooting guide](https://docs.arkelectron.com/knowledge-base/knowledge-base/serial-communication-uart).

### FC serial connectors (from ARK's pinout page)

| Connector | UART | PX4 name | Pins |
|---|---|---|---|
| VTX connector (6-pin JST-GH) | UART5 | Telem2 | 1=12V 2=GND 3=TX 4=RX 5=USART2_RX(Telem3) 6=GND — **defaults to DisplayPort/OSD, not MAVLink** |
| TELEM1 (6-pin JST-GH) | UART7 | Telem1 | 1=5V 2=TX 3=RX 4=CTS 5=RTS 6=GND — **ships already configured for MAVLink2 @ 57600** |

## Bring-up sequence (2026-07-06)

1. Initial probe on `/dev/ttyTHS1` found a continuous break condition
   (line held low) — no real signal, nothing wired yet.
2. Wrote `jetson_px4_setup/scripts/01_test_uart.sh` — opens the port raw/
   non-blocking, scans for MAVLink v1 (`0xFE`)/v2 (`0xFD`) frame headers,
   counts HEARTBEATs (msgid 0). Usage: `01_test_uart.sh [port] [baud] [seconds]`,
   defaults `/dev/ttyTHS1 57600 10`.
3. First wiring attempt used the **VTX connector** (TX/RX only, no GND) —
   defaulted to OSD protocol, and lacking a GND reference produced garbage/
   noise on the line.
4. Set `MAV_1_CONFIG = TELEM 2` (MAVLink2) and `SER_TEL2_BAUD = 57600` in
   QGC to get MAVLink onto that connector — didn't fully resolve it; moved
   to TELEM1 instead, which is MAVLink by default.
5. Rewired to **TELEM1** (TX, RX, GND, and — unintentionally — 5V, which
   landed on the Jetson's dedicated 5V rail pins rather than a GPIO pin, so
   not damaging, but should be disconnected since the Jetson has its own
   supply and doesn't need FC power).
6. Multimeter continuity test confirmed the FC-TX↔Jetson-RX /
   FC-RX↔Jetson-TX crossing is wired correctly and solidly connected.
7. Found `MAV_0_FLOW_CTRL` set to `Auto` — with CTS/RTS physically
   unwired, PX4 was gating its TX buffer waiting for a CTS that never
   comes. Set to `Off`, rebooted FC. Byte rate roughly doubled after this
   change.
8. **(historical)** kernel UART error counters
   (`TIOCGICOUNT`) show zero framing errors on the current byte stream —
   meaning what's arriving is real, correctly-clocked 57600 8N1 serial
   data, not electrical noise — but no `0xFD`/`0xFE` MAVLink start byte has
   ever appeared across thousands of sampled bytes. Next step: check
   `mavlink status` in QGC's MAVLink Console to see whether the FC is
   actually transmitting on the TELEM1/MAV_0 instance at all, independent
   of the Jetson-side wiring.

## ✅ RESOLVED (2026-07-27)

uXRCE-DDS is live: `uxrce_dds_client status` → `Running, connected`, tx 33 kB/s,
`timesync converged: true`, and `/fmu/*` topics are up.

**The wiring was never the problem.** Three separate issues were stacked on top of
each other, which is why the symptoms looked like an electrical fault:

1. **`UXRCE_DDS_CFG = 0`** — the real blocker. The XRCE client had never been started
   on the FC at all. Everything downstream was chasing a ghost.
2. **`/dev/ttyTHS1` was already owned by ARK OS.** This Jetson runs a preinstalled
   **ARK OS** image whose `dds-agent.service` runs
   `MicroXRCEAgent serial -b 3000000 -D /dev/ttyTHS1`. Every `01_test_uart.sh` run on
   that port was a *second* reader fighting the agent — and `stty` is per-device, not
   per-fd, so forcing 57600 corrupted the agent's link too. That is the actual source
   of "correctly-clocked bytes that are never valid MAVLink".
3. **Wrong protocol and baud.** We were scanning for MAVLink `0xFD` headers at 57600 on
   a port carrying XRCE framing at 3 Mbaud.

A fourth, later: the TELEM1 jumpers were physically unplugged while switching to USB-C,
which is why the link still showed `disconnected` after the params were fixed.

### What actually fixed it

| Param | → | Note |
|---|---|---|
| `UXRCE_DDS_CFG` | `101` | TELEM1 = UART7 = `/dev/ttyS6` (confirmed by PX4's own boot log **and** ARK's pinout page) |
| `SER_TEL1_BAUD` | `3000000` | match ARK's agent |
| `MAV_0_CONFIG` | `0` | free TELEM1; MAVLink keeps running on USB |

Wiring (vendor-confirmed, ARK FPV TELEM1 6-pin JST-GH):
pin2 TX → Jetson **pin 10** (RXD) · pin3 RX → **pin 8** (TXD) · pin6 GND → **pin 6**.
Leave pin 1 (5.0V) and pins 4/5 (CTS/RTS) open. 3.3V logic both ends.
⚠️ If ever falling back to TELEM2, note the **VTX connector's pin 1 is 12V** — that would
destroy a Jetson GPIO. TELEM2 is UART5 = `/dev/ttyS4` = `UXRCE_DDS_CFG=102`.

### Lessons that generalise

- **Never open the FC serial devices directly on this machine.** ARK OS owns them. Use the
  MAVLink UDP endpoints `mavlink-router` already publishes — port **14569** (`companion_aux`)
  is unbound and reserved for a companion computer. Tooling: `scripts/px4/`.
- A `/proc/*/fd` scan as `ideas-ad` reports "no process holds it" even when ARK's services
  (running as `jetson`) do. Check `/etc/ark-os/*.conf` or use `sudo lsof`.
- PX4's MAVLink shell (`scripts/px4/px4_shell.py`) answers "is the client even running?"
  in one command and would have short-circuited most of this.
