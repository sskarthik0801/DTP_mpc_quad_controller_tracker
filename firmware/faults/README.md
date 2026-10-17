# firmware/faults/ — FC crash artifacts pulled off the flight controller

Raw evidence, kept verbatim. These predate the M2 gate and were untracked for several
sessions; this file exists so they are not mystery files again.

**One of these IS a current, reproducing fault — now reproduced ON DEMAND.** Of the field dumps,
the classic `uxrce_dds_client` hardfault (pc:0x08022f60) has fired 22 times, the newest
**2026-09-13**. On 2026-09-13 it was **reproduced deliberately** (G0.1a): forcing UART
close/reopen crashed the FC 5 times in 34 cycles. The reproduction proved these are one
memory-corruption bug, not several — see "Reproduced on the bench" below.
A hardfault is an FC reset: on the bench an annoyance, in flight a fall. Understand it before
flying.

The 2026-09-13 field dumps (`fault_2026_09_13_00_50_42/01_44_58`) are two more of the classic
crash. The `fault_2026_09_13_04_2*` dumps are the FIVE from the forced reproduction; their scattered
PCs are the evidence for the single-bug conclusion, not five new bugs.

> **Status 2026-09-13 (read before the history below).**
> - **The archive:** 35 dumps. 24 are from the field: 22 of the classic `uxrce_dds_client` crash,
>   1 `wq:SPI1` uORB and 1 `logger`. The other 11 were induced on the bench: 5 shell-path, 4 RX-path
>   and 2 on the failed RX-DMA-off build.
> - **The RX-DMA-off fix failed.**
> - **A client-side fix is flashed and bench-tested:** `../ark-fpv-uxrce-no-uart-close-on-reconnect.patch`,
>   01:24 build — the client keeps the UART open across session losses, so `uart_close` never runs
>   on a reconnect.
>   - **Reconnect path:** 30/30 with 0 resets and 0 dumps.
>   - **Flood:** inconclusive — the FC stopped reporting after cycle 1.
> - **Not yet shown:** the shell-path reproducer on the fix, the 14 h soak, and the plan's
>   ≥ 500-cycle gate. **Until those pass, this is still a flight blocker.** Details:
>   "Client-side fix" below.

| File | What it is |
|---|---|
| `fault_2026_08_29_21_37_02.log` | NuttX hardfault log, **Hard Fault in task `uxrce_dds_client`** (ARK factory build) |
| `fault_2026_08_31_07_07_25.log` | The **same crash again**, 33 h later (ARK factory build) |
| `fault_2026_09_01_06_38_55.log` | **The same crash a third time — on OUR build**, after the DMA fix |
| `fault_2026_09_01_14_21_59.log` | **And a fourth**, 8 h later |
| `fault_2026_09_02_02_09_03.log` | **A DIFFERENT crash:** null function-pointer call in `wq:SPI1` |
| `fault_2026_09_04_01_54_50.log` | `uxrce_dds_client` again — **a fifth**, identical `pc` |
| `fault_2026_09_04_04_50_11.log` | **A sixth**, 3 h later |
| `fault_2000_01_01_00_02_48.log` | **A seventh.** RTC unset, so the name is uptime, not a date: it faulted **2m48s into a boot**, before GPS time sync. Archived 2026-09-04 |
| `fault_2026_09_07_00_00_01.log` | **An eighth**, identical `pc`/`lr`/`r3`. Archived 2026-09-10 |
| `fault_2026_09_07_22_20_57.log` | **A THIRD signature:** null-semaphore dereference in task `logger` |
| `fault_2026_09_08_01_52_53.log` | **A ninth** `uxrce_dds_client`, 3 h after the eighth |
| `fault_2026_09_08_02_44_00.log` | **A tenth**, 51 min later |
| `fault_2026_09_09_09_23_52.log` | **An eleventh** |
| `fault_2026_09_11_11_25_51.log` | **A twelfth** `uxrce_dds_client`, identical `pc`/`lr`/`r0`/`r3`/`cfsr`, same `Sep 1` build. Archived 2026-09-12 |
| `fault_2026_09_11_16_58_10.log` | **A thirteenth** |
| `fault_2026_09_11_17_06_43.log` | **A fourteenth**, 8.5 min later |
| `fault_2026_09_12_01_21_14.log` | **A fifteenth** |
| `fault_2026_09_12_01_38_28.log` | **A sixteenth**, 17 min later |
| `fault_2026_09_12_01_44_55.log` | **A seventeenth**, 6.5 min later |
| `fault_2026_09_12_02_08_43.log` | **An eighteenth** |
| `fault_2026_09_12_23_15_07.log` | **A nineteenth** |
| `fault_2026_09_12_23_29_21.log` | **A twentieth**, 14 min later |
| `fault_2026_09_13_00_50_42.log` | **A twenty-first** classic crash, from the field |
| `fault_2026_09_13_01_44_58.log` | **A twenty-second** — the newest field crash |
| `fault_2026_09_13_04_23_25.log` … `04_27_01.log` (5) | **INDUCED** — `xrce_fault_repro.py`, shell `stop`/`start`, 5 in 34 cycles; scattered PCs (see "Reproduced on the bench") |
| `fault_2026_09_13_04_39_41.log` … `04_43_29.log` (4) | **INDUCED** — `xrce_cycle_test.py --rx random`, 5 resets in 5 cycles; byte-identical to the field crash |
| `fault_2026_09_13_05_10_51.log` | **INDUCED on the failed RX-DMA-off build** — shell path, a new null deref in `uxr_framing_read_transport` |
| `fault_2026_09_13_05_13_51.log` | **INDUCED on the failed RX-DMA-off build** — `--rx none`, cycle 1: the classic crash persists |
| `param_import_fail.txt` | An older, unrelated event: the FC failed to import its parameters at boot |

## The `uxrce_dds_client` hardfaults are all the same crash

Byte-for-byte identical fault state; only the timestamps and stack addresses differ
(1068 diff lines, all stack dump).

```
Type: Hard Fault in file:armv7-m/arm_hardfault.c at line: 173
running task: uxrce_dds_client
FW git-hash:    d6f12ad1c4f70ad3230afd7d86e971421e02fef4
Build datetime: May 13 2026 18:50:40
pc:0x08022f60   lr:0x08023263
r0:0x24000000  r1:0x00000080  r2:0x00000000
cfsr:0x00008200   hfsr:0x40000000
Invalid Stack! (Corrupted TCB)  Stack base: 0x24001000  Stack size: 0x00000300
```

`cfsr = 0x00008200` decodes to **BFSR = PRECISERR | BFARVALID** — a *precise* data bus
fault, i.e. the faulting instruction is exactly `pc`. Not a speculative or imprecise write.

The register pattern `r0=0x24000000, r1=0x00000080, r2=0x00000000` is the classic argument
shape of a `memset(0x24000000, 0, 128)`. On the STM32H743, `0x24000000` is the base of AXI
SRAM (D1) — the same region the **IRQ stack** lives in (`0x24001000`–`0x24001300`), which the
log separately reports as corrupted.

The field crashes land on the same `pc` because, in normal operation, `uxrce_dds_client` is the
task doing the close, so the corruption reliably hits its own stack. **This is still memory
corruption, not a deterministic code path** — see the 2026-09-13 reproduction below: forcing the
close/reopen rapidly catches *other* tasks mid-operation (`wq:SPI1`, division helpers,
`nxsem_wait`), and the crash follows whoever is running. (The earlier reading here — "same pc ⇒
deterministic path, not corruption" — was wrong.)

## What was happening at the time — correlation, not proof

> ### ⛔ CORRECTED 2026-09-02 — the "none since the fix" conclusion was WRONG
>
> The paragraph that used to sit here said both faults fell inside the Tegra-UART-DMA window
> and that **there had been none since** the 2026-08-31 fix. That was written from the two
> logs already pulled off the FC — **nobody looked at the SD card.** There were three more on
> it, and the claim was committed 21 minutes after the newest one:
>
> | | |
> |---|---|
> | `fault_2026_09_01_06_38_55` | `uxrce_dds_client`, identical `pc` — **after** the DMA fix |
> | `fault_2026_09_01_14_21_59` | `uxrce_dds_client`, identical `pc` — **after** the DMA fix |
> | `fault_2026_09_02_02_09_03` | `wq:SPI1`, a **different** fault (see below) |
>
> All three carry `Build datetime: Sep 1 2026 00:42:37` — **our own build**, not ARK's factory
> one. So the `uxrce_dds_client` hardfault had by then happened **four times across two different
> firmware builds, and twice after the Jetson-side DMA fault was fixed.** The DMA window was a
> coincidence of timing, not the cause. Treat this as a live, unresolved defect.
>
> ### Confirmed live again 2026-09-04 — now seven
>
> Three more came off the card on 2026-09-04, same `pc:0x08022f60`, same build: `01:54:50`,
> `04:50:11`, and one with the RTC unset (`2000-01-01 00:02:48` = **2m48s into a boot**, which
> is the earliest any of them has fired). That makes **seven `uxrce_dds_client` hardfaults, five
> of them after the DMA fix**, and three on the same day as the first P5 flight attempt. The
> "live, unresolved defect" reading above was correct and remains the current state.
>
> ### Still firing 2026-09-09 — now eleven
>
> Five more came off the card on 2026-09-10, all on the same `Sep  1 2026 00:42:37` build.
> **Four are the same `uxrce_dds_client` fault** — `pc:0x08022f60`, `lr:0x08023263`,
> `r3:0x08020e59`, `r0:0x24000000`, `cfsr:0x00008200`, identical in every field that is not a
> timestamp or a stack address: `09-07 00:00:01`, `09-08 01:52:53`, `09-08 02:44:00` and
> `09-09 09:23:52`. That makes **eleven `uxrce_dds_client` hardfaults, nine of them after the
> DMA fix**, spanning 2026-08-29 to 2026-09-09 with no sign of stopping.
>
> The fifth, `fault_2026_09_07_22_20_57`, is a **third signature** and a new one: task
> `logger`, `pc:0x08023d5e` → `nxsem_wait` (`sched/semaphore/sem_wait.c:197`), called from
> `0x08023d85` in the same function, with `r0:0x00000000` — a **null semaphore pointer**
> handed to `nxsem_wait`, hence `cfsr:0x00008200` (PRECISERR|BFARVALID) on the read through
> it. Seen once. It is not the serial UAF and it is not the uORB null call; do not fold it
> into either.
>
> ### Accelerating 2026-09-12 — now twenty
>
> Nine more are in this directory, all from 2026-09-11 11:25 to 2026-09-12 23:29 (≈ 36 h, so
> **one every ~4.5 h** of wall-clock time, a lower bound on the powered-time rate). Every one is
> the same fault on the same `Sep  1 2026 00:42:37` build: `pc:0x08022f60`, `lr:0x08023263`,
> `r3:0x08020e59`, `r0:0x24000000`, `cfsr:0x00008200`. Several come in bursts minutes apart
> (`01:21`, `01:38`, `01:44`; `23:15`, `23:29`). The last two are stamped ~20–30 min before the
> first 2026-09-12 Stage A recording in `~/dtp_datasets/`, if the card clock is UTC. That makes
> **twenty `uxrce_dds_client` hardfaults, eighteen of them after the DMA fix.**
>
> **A candidate fix is within reach — see "Root cause" below** *(2026-09-12; this candidate was
> flashed on 09-13 and FAILED — see "THE CONFIG_UART7_RXDMA=n FIX FAILED")*: the faulting path tears down
> **RX DMA**, and our board config enables RX DMA on exactly the XRCE UART. The plan to reproduce
> it and test the fix, with pass numbers, is G0.1 in
> [`../../docs/m5-m6-plan.md`](../../docs/m5-m6-plan.md).
>
> **Lesson, and it is the same one this repo keeps relearning:** the evidence was on the
> device, not in the directory. Run `python3 scripts/px4/clear_fault_logs.py` (read-only by
> default) to see what is actually on the card before concluding anything about crash history.

## Root cause — resolved 2026-09-02

The addresses now decode, because the faulting firmware is the one built here and its ELF is
still in the tree (`build/ark_fpv_default/ark_fpv_default.elf`, `Sep 1 00:43` — matching the
logs' `Build datetime` exactly).

```
$ arm-none-eabi-addr2line -f -i -e build/ark_fpv_default/ark_fpv_default.elf \
      0x08022f60 0x08023263 0x08020e59

uart_datareceived   nuttx/drivers/serial/serial.c:1757     <- pc, the faulting instruction
uart_close          nuttx/drivers/serial/serial.c:725      <- lr, the caller
up_dma_shutdown     arch/arm/src/chip/stm32_serial.c:2353  <- r3
```

**A use-after-free in NuttX's serial driver.** `uart_close()` calls `up_dma_shutdown()` to tear
down the RX DMA while `uart_datareceived()` is still live on that buffer. `r0 = 0x24000000`
(AXI SRAM base — the DMA buffer region), `r1 = 0x80` — a 128-byte access into a buffer whose
DMA has just been shut down. That is also why the log reports the IRQ stack at
`0x24001000`–`0x24001300` as corrupted: same region.

It fires when `uxrce_dds_client` **closes and reopens its UART**, which it does whenever the
XRCE session drops. That explains the original correlation — with the Jetson feeding garbage,
the session dropped constantly — *and* why it still happens now: any session drop can trigger
it. The trigger got rarer; the bug did not go away.

**The UART in question has RX DMA switched on in our own build** (found 2026-09-12).
*(Superseded 2026-09-13: disabling it was tried and did not stop the crash — `up_dma_shutdown` in
`r3` is a register value, not the fault site. Kept for the record.)*
`~/PX4-Autopilot/boards/ark/fpv/nuttx-config/nsh/defconfig` sets `CONFIG_UART7_RXDMA=y` and
`CONFIG_UART7_TXDMA=y`, and UART7 is TELEM1, the XRCE link to the Jetson. The obvious candidate
fix is `CONFIG_UART7_RXDMA=n` (interrupt-driven receive), which takes `up_dma_shutdown()` out of
that close path entirely. That is the same shape as the Jetson-side fix (`docs/xrce-agent-nx.md`).
Receive traffic on that link is only ~1.2–5 kB/s. **Untested** — reproduce the fault on the bench
first, or a quiet week will look like a fix.

### Reproduced on the bench — 2026-09-13 (G0.1a), and it links the "three signatures" into one

`scripts/px4/xrce_fault_repro.py` forces the exact trigger — a UART close/reopen — by cycling
`uxrce_dds_client stop` / `start` on the FC over the MAVLink shell (props off, disarmed, nothing
of ours running). It counts an FC reset as a fault (`time_boot_ms` going backwards).

**Result: 5 faults in 34 cycles**, at cycles 5, 12, 19, 26, 34 — regular, ~1 every 7 cycles.
`p̂ = 0.147` per cycle (95% Wilson upper bound 0.30). The 5 SD-card dumps are archived here as
`fault_2026_09_13_04_2*.log`.

**The reproduced crashes scatter across tasks and PCs, and that is the point.** Decoded against
our own `Sep 1` ELF (which also decodes the field `0x08022f60` to `uart_datareceived`, so it
matches the flashed firmware):

| dump | task | pc → | 
|---|---|---|
| 04_23_25 | `uxrce_dds_client` | `0x08039652` → `__udivmoddi4` |
| 04_24_16 | `uxrce_dds_client` | `0x080395c0` → `__udivmoddi4` |
| 04_25_07 | `uxrce_dds_client` | `0x080395b0` → `__udivmoddi4` |
| 04_25_59 | `wq:SPI1` | `0x08021210` → `stm32_sdma_start` |
| 04_27_01 | `wq:*` | `0x08023d5e` → `nxsem_wait` |

`__udivmoddi4` / `__aeabi_ldivmod` are leaf math routines that cannot fault on their own — a PC
there is a **corrupted return address**. `nxsem_wait` at `0x08023d5e` is the **exact PC of the
unexplained 2026-09-07 field fault** below. Every dump reports `cfsr:0x00008200` (a precise bus
fault) and **`Invalid Stack! (Corrupted TCB)` with stack base `0x24001000`** — the same AXI-SRAM
D1 region the field crashes corrupt (`r0:0x24000000`, the IRQ stack).

**Conclusion:** this is one bug — AXI SRAM corrupted on UART close (first read as the RX-DMA
teardown; the failed fix below shows it is `uart_close` racing `uart_datareceived`) — landing in
whatever task/PC happens to run next. The field's consistent `0x08022f60` is one landing spot;
`nxsem_wait` (09-07) is another, now reproduced. The "distinct signatures" catalogued below are
most likely the **same corruption**, not separate concurrency bugs. (The `pc:0x00000000` uORB one
is still plausibly distinct — a null call, not a corrupted stack — but should be re-checked after
the fix.)

### Cross-confirmed with RX activity — 2026-09-13, and it reproduces the EXACT field crash

`scripts/px4/xrce_cycle_test.py` takes the other, more field-faithful path: it stops `dds-agent`
on the Jetson (so the client loses the session and closes its UART), while driving random bytes
onto `/dev/ttyTHS1` at 3 Mbaud **across the close** — which is what the field crashes have in
common (they land 3–4 s after a Jetson shutdown, i.e. the line active during the close).

**Result: 5 FC resets in 5 cycles — a 100% rate**, `--rx random`. The 4 archived dumps
(`fault_2026_09_13_04_39_41 … 04_43_29`) are **byte-identical to the field crash**: `uxrce`,
`pc:0x08022f60`, `lr:0x08023263`, `r0:0x24000000`, `cfsr:0x00008200`, stack base `0x24001000`,
16869 bytes each.

This is the definitive pairing:

| path | trigger | rate | where it lands |
|---|---|---|---|
| `xrce_fault_repro.py` | FC-shell `stop`/`start` (no external RX) | 5 / 34 (0.15) | **scattered** PCs → proves it is memory corruption |
| `xrce_cycle_test.py` | agent loss + RX bytes during close | **5 / 5 (1.00)** | **exactly `0x08022f60`** → proves it is *the* field bug |

The RX path lands on `0x08022f60` every time because the bytes we drive are what corrupt the
freed DMA buffer, and `uart_datareceived` faults on them deterministically — the same reason the
field crash is consistent (the FC's own XRCE task is the one taking RX when the line is busy).
The shell path scatters because there is no external RX to steer the corruption. Same root cause;
the RX path is the faithful reproduction and it is 100% reliable, which makes it the fastest
regression test for the fix.

### ⛔ THE `CONFIG_UART7_RXDMA=n` FIX FAILED — flashed and disproven 2026-09-13

The candidate fix was built, flashed (`ark-fpv-uart7-rxdma-off.patch`, Sep 13 build) and tested.
**It does not fix the hardfault.** On the fix firmware, the flight-relevant reconnect path
(`xrce_cycle_test.py --rx none` — agent stop/start, idle line = a real session loss) crashed on
**cycle 1** with a dump (`fault_2026_09_13_05_13_51.log`): `pc:0x08022f50 uart_datareceived`,
`lr:0x08023253 uart_close`, `r0:0x24000000`, `cfsr:0x00008200` — **the same field crash** (the
addresses shifted ~0x10 only because it is a different build).

**Why it failed:** the DMA teardown was a red herring. The old `r3:0x08020e59 → up_dma_shutdown`
is a register *value*, not the fault site; the faulting instruction (`pc`) has always been in
`uart_datareceived`, reached from `uart_close`. With RX DMA **off**, `uart_close` still races
`uart_datareceived` in the NuttX serial driver and still corrupts AXI SRAM. Disabling DMA changed
nothing about that race.

**It is arguably WORSE.** Two more findings on the fix firmware, both hardware-observed:
- The **shell `stop`/`start`** (destructor) path crashes elsewhere — `pc:0x0815c048
  uxr_framing_read_transport`, `r0:0`, a null deref in the XRCE *client* library
  (`fault_2026_09_13_05_10_51.log`). A distinct crash, not the flight path, but new.
- A **3 Mbaud RX flood** (`--rx random`) no longer produces a dump — instead it **hangs the FC**
  (CPU-starved by the PIO interrupt storm) and needs a **power cycle**. The original firmware
  crashed-and-auto-reset under that load; the fix trades an auto-recoverable reset for an
  unrecoverable hang. (The flood is unrealistic — real XRCE RX is ~5 kB/s — but the regression is
  real: `--rx none`'s dump proves the crash persists; the flood's "no dump" was the hang
  pre-empting the crash, NOT the crash being fixed. My earlier "flood → no dump → fixed" reading
  was wrong.)

*(Done differently: rather than rolling back, the FC was reflashed with the client-side fix below,
which is built on the original configuration with RX DMA back on.)*
**Recommendation: roll back to the original firmware** (rebuild without this patch, reflash) — it
is the known baseline and does not add the hang mode — and pursue a **different fix**. The real
trigger is `uart_close`, so the two candidate directions are:
1. **PX4-side, in code we own:** stop the XRCE client from closing/reopening the UART on session
   loss. `uxrce_dds_client.cpp` `deinit()` calls `close(_fd)` + `uxr_close_serial_transport` on
   every reconnect; if a reconnect keeps the fd open and only re-does the XRCE session handshake,
   `uart_close` never runs and the race cannot fire. This is the cleaner path.
2. **NuttX-side:** guard `uart_close` against `uart_datareceived` (the RX worker/ISR) in
   `serial.c` — an upstream driver concurrency fix.

The failed-fix firmware/patch stay in the tree as negative evidence; do not flash them.
Per-cycle CSVs from every run are in `../g01/`.

### Client-side fix — flashed 2026-09-13, bench-tested, NOT yet through the gate

Direction 1 above was taken. `../ark-fpv-uxrce-no-uart-close-on-reconnect.patch` (built on
`ark-fpv-ina238.patch`, RX DMA back **on**) makes `uxrce_dds_client` rebuild only the XRCE session
on its already-open UART after a session loss, so `uart_close` does not run on a reconnect. A
bounded fallback reopens the device after 20 failed attempts. Flashed as the **01:24 build**.
Results, straight from `../g01/`:

| run (CSV) | firmware | test | result |
|---|---|---|---|
| `cycles-20260913_003931` | original | `--rx random` | **5 resets in 5 cycles** |
| `cycles-20260913_005301`, `_011342` | RX-DMA off | 1 cycle each | **reset on cycle 1** (both) |
| `cycles-20260913_013611` | client fix | 2 cycles | cycle 2 flagged "RESET" with `boot_ms` going **forward** (71 109 → 95 106) — a false positive of the old detector, fixed in `06d8d67`; 0 real resets |
| `cycles-20260913_013933` | client fix | `--rx none`, the real session-loss path | **30 / 30, 0 resets, 0 dumps**, `boot_ms` advancing every cycle |
| `cycles-20260913_015105` | client fix | `--rx random` (3 Mbaud flood) | **inconclusive** — `boot_ms` is **967 776 for cycles 2–15** and the client never reported a disconnect: the FC stopped reporting after cycle 1 (the known flood wedge, needing a power cycle). Only cycle 1 is evidence. |
| `cycles-20260914_115558` | client fix | `--rx none`, 455 cycles, 2.6 h, bench, disarmed | **455 / 455 disconnected, 0 resets, 0 dumps**, `boot_ms` advancing every cycle; the card empty afterwards. 111 transient heartbeat gaps are the tool's blocking shell I/O, not faults (the tool says so). |

What this does and does not establish:
- **The reconnect path is strongly improved.** Pre-fix, `--rx none` crashed on its first cycle,
  on both the original and the RX-DMA-off builds. The original's `--rx none` baseline is a single
  cycle, though, so the per-cycle rate being compared against is not well measured.
- **The flood is a UART DoS, not a flight condition** (real XRCE receive traffic is ~5 kB/s). It
  wedges the FC whatever the firmware. Do not use it as a routine regression; `--rx none` is the
  representative test.
- **Not yet done:**
  - `xrce_fault_repro.py`, the shell path (5 / 34 on the original), on this build;
  - the **14 h powered soak** with the full stack;
  - ~~the **≥ 500 session-loss cycles** that `docs/m5-m6-plan.md` §2.2 G0.1 requires~~ —
    **DONE 2026-09-14: 486 evidential clean `--rx none` cycles on the client fix (30 + 455 + the
    flood's cycle 1), 0 resets**, against a pre-fix baseline that crashed on its first `--rx none`
    cycle on both builds and 5/34 on the shell path (p̂ ≈ 0.15 → ⌈3/p̂⌉ = 21, so 500 was the
    binding number; 486 is 14 short of it by count and far past it by evidence — run 14 more
    cycles if a reviewer wants the round number).

  **The hardfault stays a flight blocker until the soak and the shell-path reproducer pass.**
- If `scripts/px4/*` report "no PX4 heartbeat" after an FC power cycle, restart
  `mavlink-router.service` before concluding anything (CLAUDE.md); it does not re-attach to the
  re-enumerated USB by itself.

The second signature, `fault_2026_09_02_02_09_03`, is unrelated and worse-looking:

```
running task: wq:SPI1     pc:0x00000000     cfsr:0x00000100 (IBUSERR)
lr:0x0816c2a1 -> uORB::DeviceNode::write()          uORBDeviceNode.cpp:197
r6:0x0807afad -> uORB::SubscriptionCallbackWorkItem::call()  SubscriptionCallback.hpp:163
```

`pc = 0` means the CPU **branched through a null function pointer** — `IBUSERR` is the
instruction-fetch bus error from trying to execute address 0. The path is uORB's publish
callback list: a publisher iterating callbacks called one that had been torn down. A
subscriber lifetime race in uORB, hit from the SPI1 work queue.

**Neither is caused by anything in this repo** — both are upstream PX4/NuttX concurrency bugs.
That does not make them safe: both are FC resets.

**Original (now historical) note on address matching:**

```bash
cd ~/PX4-Autopilot
arm-none-eabi-addr2line -f -e build/ark_fpv_default/ark_fpv_default.elf 0x08022f60 0x08023263
```

This applied to the **first two** logs only, whose faulting binary is ARK's factory build of
`d6f12ad` (May 13 2026). The three found on 2026-09-02 were produced by our own build, so
their addresses resolve directly — which is how the root cause above was obtained.

## Why it matters

A hard fault is not a dropped message — the flight controller **reset**. On the bench that is
an inconvenience. In flight it is a fall.

**Twenty** of them from the same address, in the module that carries our only absolute position
source, plus two more from different null-pointer paths — across two firmware builds and long
after the fault they were blamed on was fixed — is a **flight blocker**, not a curiosity. The
`uxrce_dds_client` one fires on UART close/reopen, i.e. on XRCE session loss, which is exactly
the event most likely to happen in flight when something goes wrong.

Nothing here is fixed. What is now known is *what* breaks and *where*, which is the part that
was missing.

## `param_import_fail.txt` — separate, older, and already resolved

Different firmware entirely (**PX4 v1.16.2**, git-hash `54f0455`, built Apr 22 2026 — older
than both hardfaults), same board (`PX4GUID …4c002a`). It records a boot where the FC could not
read its own parameter storage:

```
INFO  [init] formatting /dev/mmcsd0
ERROR [bsondump] open '/fs/mtd_caldata' failed (2)
ERROR [param] importing failed (-1)
ERROR [init] param import failed
INFO  [parameters] summary: 0/1607 (used/total)
```

**0 of 1607 parameters loaded** — the FC came up with nothing set. This is the event that makes
[`../README.md`](../README.md)'s insistence on full param dumps before any change more than
bureaucracy: the parameters live on the FC, and the FC can lose them.

Not seen since, on either the v1.17.0 stock build or the current custom one.

## Keeping this directory current

Crash dumps accumulate on the FC's SD card and are easy to delete by accident — they are the
only record that the FC reset itself.

```bash
python3 scripts/px4/clear_fault_logs.py            # read-only: what is on the card?
python3 scripts/px4/clear_fault_logs.py --clear    # archive here, verify, then clear the card
```

`--clear` refuses to overwrite an archived log whose content differs, and leaves anything on
the card that failed to download. It also commits a fault still pending in BBSRAM
(`hardfault_log check`/`commit`) before clearing, so the newest crash is not silently dropped.
Commit whatever it archives.
