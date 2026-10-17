# uXRCE-DDS on the Orin NX — FIXED: Tegra UART DMA was zeroing the data

**Status 2026-08-31: ROOT-CAUSED AND FIXED.** `/dev/ttyTHS1` could not send data to itself: a
loopback jumper between the Jetson's own TX and RX pins — **no flight controller, no harness, no
ARK software in the path** — returned the correct byte count with the payload replaced by zeros.

**The fault was Tegra HSUART DMA.** `serial@3100000` declared `dmas`/`dma-names`, and the DMA
path delivered zero-filled buffers to the tty layer. Removing those two properties forces PIO and
the port echoes perfectly. The FC was fine. The wire was fine.

## The fix

Boot with a DTB that has DMA removed from `serial@3100000` only:

```bash
cp /boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv.dtb pio.dtb
fdtput -d pio.dtb /bus@0/serial@3100000 dmas
fdtput -d pio.dtb /bus@0/serial@3100000 dma-names
sudo cp pio.dtb /boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv-pio.dtb
# then add to LABEL primary in /boot/extlinux/extlinux.conf, after INITRD:
#   FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv-pio.dtb
sudo reboot
```

`/boot/extlinux/extlinux.conf.pre-pio` is the pre-change backup, and a `LABEL stockdtb` entry
boots the stock DTB from the 30-second menu on the `ttyTCU0` debug console if needed.

Verify the override took: `/proc/device-tree/bus@0/serial@3100000/` must have **no `dmas`**.

**Before vs after**, `bash scripts/uart_loopback_test.sh`:

| Size | Stock DTB (DMA) | PIO DTB |
|---|---|---|
| 1, 4 | exact echo | exact echo |
| 8, 16 | **ALL ZEROS** | **exact echo** |
| 32, 64 | ALL ZEROS | **exact echo** |

18/18 exact echo after the change, versus 100% payload loss at every size ≥8 before.

**`ttyTHS2` still has its `dmas`** — if you ever move the link there, expect the same fault and
apply the same removal.

## End-to-end result

With the PIO DTB live and the harness reconnected, the link came up on the first try:

| Check | Before | After |
|---|---|---|
| Raw line @ 3 Mbaud | `00×16 fd 02 00 00 00 cf 12`, no `0x7E` | `7e 01 00 10 00 80 00 …` — valid XRCE framing |
| `uxrce_dds_client status` | `Running, disconnected`, `cycle: 0 events` | **`Running, connected`**, `cycle: 3721 events` |
| Payload tx | 0 B/s | **35910 B/s** |
| timesync | `converged: false` | **`converged: true`** |
| `ros2 topic list \| grep ^/fmu` | 0 topics | **65 topics** |
| `/fmu/out/vehicle_odometry` | — | **~100 Hz** |

ARK's stock `dds-agent.service` (`-b 3000000`) works unmodified — no need for our
`xrce-agent.service`, which stays disabled.

```
$ stty -F /dev/ttyTHS1 115200 raw -echo -crtscts min 0 time 20
$ printf 'ABCDEFGHIJKLMNOPQRSTUVWXYZ012345' > /dev/ttyTHS1   # with TX jumpered to RX
$ timeout 3 head -c 32 /dev/ttyTHS1 | xxd
00000000: 0000 0000 0000 0000 0000 0000 0000 0000  ................
00000010: 0000 0000 0000 0000 0000 0000 0000 0000  ................
```

32 bytes in, 32 bytes out, every one of them zero. Standard tools only — no custom code.

> **Two corrections to earlier revisions of this file.**
>
> 1. "The TELEM1 wire delivers no recoverable data — this is a hardware fault" (2026-08-30) is
>    **wrong and retracted**. The wire is sound; it was carrying real data the whole time.
> 2. The revision before that claimed TELEM1 was *proven* wired. That was right for the wrong
>    reason, and was itself retracted. It is now re-established on solid evidence — see
>    "The wire is good" below.
>
> The recurring error was inferring properties of the *link* from data that had already been
> corrupted by the *receiver*. Every capture in this file was taken through a broken RX path.

## The fault

With TX jumpered to RX on `/dev/ttyTHS1` and the FC harness **unplugged** (verified: the line is
passively silent, `rx=0` over 6 s, so nothing else is driving it):

Measured with `stty`/`printf`/`head`, reader opened before the write, **8 repetitions per size**:

| Bytes written | Result (8 reps) |
|---|---|
| 1 | **exact echo** 5/8 (3 empty reads) |
| 4 | **exact echo** 7/8 |
| 8 | **all zeros** 7/8 |
| 16 | **all zeros** 7/8 |

**The threshold is between 4 and 8 bytes.** At or below 4 the content is correct; at or above 8
it comes back zeroed. Reproduced independently in Python (`os.write`/`os.read` with raw termios)
and in pure shell, and at 115200, 921600 and 3000000 alike. `frame`, `parity`, `brk` and
`overrun` are all 0 throughout — the UART clocks and frames correctly; only the payload dies.

The size-dependence is the important part: a loose jumper or marginal contact would fail as a
function of *time*, not of *transfer length*. A threshold at 4–8 bytes points at a FIFO trigger
level or the PIO→DMA handover.

> **Do not over-read the fine structure.** Larger transfers give messy, partly-reproducible
> results — short reads (24 sent → 19 back), occasional total loss, and sometimes a surviving
> tail of 4–8 non-zero bytes. An earlier revision of this file stated a precise "first 8 (PIO) /
> first 16 (DMA) bytes zeroed" rule; that was over-fit to a single run and did **not** survive
> repetition. What *is* reproducible is the 4-vs-8-byte threshold above.

**This single fault explains every symptom in this file**, including the two captures that
previously looked like a dead wire:

- The XRCE client's session frame arrives as `00×16 fd 02 00 00 00 cf 12` — that is the
  **first-16-zeroed signature**, not a mangled wire. The `0x7E` begin flag is in the part that
  got zeroed, which is why the agent discards the stream.
- The MAVLink test's "8192 bytes, 98.9% zeros, zero `0xFD` start bytes" is the same signature at
  a sustained 46 kB/s, where transfers are large enough to be zeroed wholesale.

## The wire is good — established, not assumed

Changing `SER_TEL1_BAUD` from `3000000` to `115200` (with an FC reboot — `SER_*` params are
`@reboot_required`; a param write alone does nothing) **moved the clean-decode point with it**:

| Jetson sample rate | Before (FC @ 3 Mbaud) | After (FC @ 115200) |
|---|---|---|
| 115200 | garbage | **`frame=0`, clean** |
| 921600 | garbage | 168 framing errors |
| 3000000 | **`frame=0`, clean** | 208 framing errors |

The received bit rate tracks the FC's configured baud. Therefore the Jetson's RX **is** bonded to
TELEM1's TX, and the harness carries data cleanly at **both** 3 Mbaud and 115200. Unplugging the
harness drops the line to `rx=0`; reconnecting restores it. The link is not marginal.

Also measured, with the harness connected: 30 bursts in 30 s, **30/30 byte-identical**, period
1001.1 ms (jitter 3.9 ms), zero framing errors. Deterministic, error-free, exactly at the client's
retry cadence.

## Jetson static configuration — all correct, all verified

Checked 2026-08-31; none of this is the problem, do not re-derive it:

| Item | Value |
|---|---|
| Driver bound to `3100000.serial` | `serial-tegra` (**not** `8250` — so 3 Mbaud is genuinely supported) |
| DT node | `nvidia,tegra194-hsuart`, `status = okay` |
| UART clock | `uarta` = **48 MHz** = exactly 16 × 3,000,000 (16× oversampled: zero divisor error) |
| termios | `speed 3000000`, `cs8`, `-parenb`, `-crtscts`, `clocal`, `-hupcl` |
| Console / getty | kernel console on `ttyTCU0`; no `serial-getty@ttyTHS1`; `nvgetty` absent |
| GPIO conflict | none — no GPIO claims on the UART pins |
| TX backpressure | `TIOCOUTQ` = 0 |
| Pinmux | **not inspectable from Linux** — applied by MB1 from the flashed BCT; `/sys/kernel/debug/pinctrl/*/pingroups` contains no UART groups. Ruled out empirically instead: the loopback returns the correct byte count, which requires working TX and RX pinmux. |

## Caveats on this fix

- **PIO costs CPU, but not much here.** Every byte is moved by the interrupt handler instead of
  the DMA engine. Measured with the link live at 3 Mbaud and `Payload tx: 35910 B/s`:
  **1286 IRQ/s on `3100000.serial`, `MicroXRCEAgent` at 7.3% CPU, load average 0.66.** Acceptable.
  Re-check if the payload rate grows much beyond this.
- **The DTB override is not survivable across a JetPack upgrade or a reflash.** `nv-update-extlinux`
  rewrites `extlinux.conf`; re-apply the `FDT` line afterwards. The stock DTB is untouched, so an
  upgrade will silently revert to the broken DMA path — and the symptom looks like a dead wire.
  **Run `scripts/uart_loopback_test.sh` after any system update.**
- **Why DMA failed was not established.** Removing it fixes the symptom; the underlying
  `serial-tegra` / GPCDMA defect on L4T R36.5.2 is not diagnosed here. Worth checking NVIDIA's
  errata if it recurs.

## Run the loopback first — always

Before debugging *any* serial link on this box, prove the port can talk to itself. Jumper the
UART's TX pin to its own RX pin (40-pin header **pin 8 → pin 10** for `ttyTHS1`; confirmed
empirically by the byte counts returning), unplug everything else, then:

```bash
sudo systemctl stop dds-agent          # never read the port with the agent running
stty -F /dev/ttyTHS1 115200 raw -echo -echoe -echok -crtscts min 0 time 20
timeout 3 head -c 32 /dev/ttyTHS1 > /tmp/lb.bin &
sleep 0.4; printf 'ABCDEFGHIJKLMNOPQRSTUVWXYZ012345' > /dev/ttyTHS1; wait
xxd /tmp/lb.bin
```

Expect the ASCII back. Zeros mean the port is broken and **nothing measured through it can be
trusted** — which is how this file came to contain two retracted hardware conclusions.

This is packaged as **`scripts/uart_loopback_test.sh`** — run that rather than retyping the
above. It refuses to run while `dds-agent` holds the port, repeats each size, and exits non-zero
on failure:

```bash
sudo systemctl stop dds-agent
bash scripts/uart_loopback_test.sh                 # defaults: ttyTHS1, 115200, 3 reps
bash scripts/uart_loopback_test.sh -d /dev/ttyTHS2 -b 921600 -r 8
```

Baseline on this board before any fix (2026-08-31): sizes 1 and 4 **EXACT ECHO**, sizes 8 and 16
**ALL ZEROS**, 2/2 reps each.

## Reading UART counters without disturbing the owner

`TIOCGICOUNT` on a second fd reports kernel-side `rx`/`frame`/`brk`/`overrun` without ever
calling `read()`, so it cannot steal bytes from a running agent. This is the safe way to answer
"is anything arriving?" while `dds-agent` holds the port — it is how the 23 B/s was first spotted.

## Still installed and working

| Piece | State |
|---|---|
| `Micro-XRCE-DDS-Agent` **v2.4.3** → `/usr/local/bin/MicroXRCEAgent` | ✅ installed |
| ARK OS agent (this box was reimaged with ARK OS 2026-08-31) | ✅ `dds-agent.service`, hardcodes `-b 3000000` in `/usr/lib/ark-os/libexec/start_dds_agent.sh` |
| `px4_msgs` @ `release/1.17` (pinned `86d8239`, 235 msgs) | ✅ builds; `from px4_msgs.msg import VehicleOdometry` works |
| `scripts/px4/xrce_agent_setup.sh` | ✅ reproducible installer |
| `/etc/systemd/system/xrce-agent.service` | ⚠️ installed, **disabled** — ARK's `dds-agent` owns the port now |

v2.4.3 targets Fast DDS 2.x (what Humble uses); v3.x pulls Fast DDS 3.x. **Never ruled in or out**
as a protocol-compatibility factor — there has still never been a working baseline to test against.

## FC side — eliminated, do not redo

All tested with the FC healthy and reachable over USB MAVLink.

| Hypothesis | How it was tested | Result |
|---|---|---|
| **Wrong physical port** | Started MAVLink on *every* FC UART in turn | `ttyS1`=0, `ttyS3`=27, `ttyS4`=115, **`ttyS6`=4096 (capture limit)**. Harness is on TELEM1/`ttyS6`. |
| **Baud mismatch** | Both ends forced to 921600, 3000000, then 115200 with FC reboot | Sample rate tracks FC baud (table above). Matched rates give `frame=0`. Not a mismatch. |
| **Hardware flow control** | TELEM1 alone defaulted to `flow control: ON` | Forced off with `mavlink start ... -Z`. **No change.** |
| **Client not running / other owner of `ttyS6`** | `uxrce_dds_client stop`, re-read the port | 138 bytes/6 s → **0**. The burst is definitively the XRCE client. |
| **FC not transmitting** | `mavlink status` TX counters | `tx: 22640.5 B/s, txerr: 0.0` |
| **Client misconfigured** | `dmesg` | `init serial /dev/ttyS6 @ 3000000 baud`, no error |
| **Only one MAVLink instance** | `mavlink status`, post-reboot | instance #0 on USB CDC only — nothing else contends for TELEM1 |

Post-reboot the client reports `Running, disconnected`, `timesync converged: false`, and
`cycle: 0 events` — its main loop has never run, consistent with never completing a session.

## Jetson side — eliminated

- **Flow control** — `-crtscts -ixon -ixoff`; forcing it changes nothing.
- **Getty / console** — none on `ttyTHS*`; console is `ttyTCU0`.
- **Second reader** — the agent was stopped before every raw read.
- **Signal inversion** — an inverted line idles low and streams 0x00 at full rate. We see 23 B/s
  with `brk=0`: the line idles high, polarity correct.
- **Wrong Jetson UART** — `ttyTHS2` carries nothing.
- **Non-standard bauds** — swept 1.5–4.0 MHz via `termios2`/`BOTHER` (the fixed `B` constants are
  *not* the limit; see `finesweep.py`). **No `0x7E` at any rate.** Consistent with the flag byte
  being inside the zeroed region, not with a rate error.
- **TX/RX swapped** — tried; produced total silence on both UARTs at all bauds. Original
  orientation is correct.

### A false lead, recorded so nobody repeats it

Bit-shifting the captured stream appeared to reveal ten `0x7E` frame flags at shift 7. **This is
an arithmetic artifact:** `0xFD >> 1 == 0x7E` exactly, so shifting a stream containing ten `0xFD`
bytes manufactures ten `0x7E` bytes regardless of content. The test finds its own input.

## Why the agent logs nothing

Agent v2.4.3 defines `framing_begin_flag = 0x7E`
(`include/uxr/agent/transport/stream_framing/StreamFramingProtocol.hpp:73`) and **discards octets
until it sees one**. The client's frame reaches the agent with its leading bytes zeroed, so the
`0x7E` is gone and everything is dropped — which is why even `-v 6` produces only the two startup
lines. Consistent behaviour, not an agent bug.

## After the data path is fixed

- **TELEM1 defaults to `flow control: ON`.** MAVLink can be forced off with `-Z`, but
  `uxrce_dds_client` has no equivalent flag. If the harness is 3-wire, verify this explicitly.
- Re-test agent **v2.4.3** first; if frames arrive but no session establishes, try **v3.0.1** in
  case PX4 v1.17's client speaks the v3 protocol.
- Restore `SER_TEL1_BAUD=3000000` (or leave it lower — nothing requires 3 Mbaud but ARK's
  hardcoded agent baud) and confirm with `ros2 topic list | grep fmu` and
  `uxrce_dds_client status` → `Running, connected`.

**Keep USB connected while debugging TELEM1.** Every `scripts/px4/*.py` tool reaches the FC over
`mavlink-router` → `/dev/ttyACM0`; without it there is no way to query or reconfigure the FC. USB-C
MAVLink and TELEM1 uXRCE-DDS are designed to run simultaneously on separate ports.

Until this links, `ros2 topic list | grep fmu` is empty and `px4_odom_bridge.py` has nothing to
publish to. **M2 is blocked on the Jetson's serial data path — not the wire, not the FC.**
