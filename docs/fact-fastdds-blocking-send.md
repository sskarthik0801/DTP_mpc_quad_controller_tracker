# Fact: a Wi-Fi subscriber to `/cloud_registered` can stall FAST-LIO, and how it was closed

**Measured 2026-09-13 (sixth session), bench, full graph observe-only, disarmed.** Evidence in
`~/dtp_datasets/20260913_m53_soak_bench/` (the failing soak) and
`~/dtp_datasets/20260913_m53_soak_bench_nonblocking/` (the fix). Scratch runs are summarised
below; the numbers are from `drone-dtp/scripts/m53_soak.py` 10 s windows.

## The symptom

The first 20-min M5.3 soak passed everything but O2: **one ~10 s episode** at minute 5.5 with a
944 ms `/lidar_slam/odom` gap. In it, odom, `/tf` and the bridge's `/fmu/in/vehicle_visual_odometry`
all gapped 940 ms; the XRCE agent's 50 Hz PX4 topics never gapped past 40 ms; FAST-LIO's CPU did
**not** rise (21.8 % vs 23.3 %); and a 300 ms *stamp* gap shows two scans were then dropped.
FAST-LIO was waiting, not computing. A laptop RViz was subscribed to `/cloud_registered` over Wi-Fi
the whole run.

## The mechanism, reproduced with `strace` delay injection

`rmw_fastrtps` publishes **synchronously**: `publish()` runs the UDP `sendto` in the calling thread.
`strace -f -e trace=sendto` on `fastlio_mapping` shows the ~1 MB/scan cloud leaving from the
**executor threads** — FAST-LIO's scan callback — as ~60 datagrams per scan, on **blocking** sockets
(`/proc/<pid>/fdinfo` flags `02`). A stalled Wi-Fi link fills the socket's send buffer and the
callback blocks inside `sendto`.

With `strace -e inject=sendto:delay_exit=<us>` as a stand-in for a slow link (the Tegra kernel has no
`htb`/`tbf`/`netem`/`NFQUEUE`, and the Realtek `rtl88x2ce` driver refuses `iw set bitrates`/`txpower`,
so the Wi-Fi itself cannot be throttled from this box):

| Config | Injected on | Delay per `sendto` | odom during injection | FAST-LIO CPU |
|---|---|---|---|---|
| SYNCHRONOUS (stock) | all threads | 5 ms | 9.6–9.7 Hz, gaps 270 ms | 25–27 % |
| SYNCHRONOUS (stock) | all threads | 20 ms | **1.9 Hz, gaps 1.1 s**, then a 26 Hz catch-up burst of stale poses | **13 %** (waiting) |
| ASYNCHRONOUS (`RMW_FASTRTPS_PUBLICATION_MODE`) | the one FastDDS sender thread only | 20 ms | **3.8–4.2 Hz, gaps 0.7–1.0 s** | **13 %** |

The 20 ms row is the soak episode's signature exactly. **Asynchronous mode does not fix it:** all
of the process's writers share one FastDDS sender thread, and the callback still ends up waiting
behind it (its CPU falls the same way). Don't re-derive this; it is the tempting fix and it fails.

## The fix: `non_blocking_send`

`super_bringup/config/fastdds_nonblocking.xml` — a FastDDS participant profile with a UDPv4
transport whose `non_blocking_send` is true (plus SHM, since `useBuiltinTransports` must be off).
FastDDS 2.6.12 documents it as: a full buffer returns immediately, "as if the datagram is sent and
lost". Loaded into **FAST-LIO's process only** via `FASTRTPS_DEFAULT_PROFILES_FILE`
(`m1_super_lidar.launch.py` arg `fastlio_dds_profile:=nonblocking`, `run.sh` env
`FASTLIO_DDS_PROFILE=nonblocking`).

**Verified it took:** the four user-data UDP sockets of `fastlio_mapping` show flags `04002`
(`O_NONBLOCK`) in `/proc/<pid>/fdinfo`; the discovery sockets stay blocking (tiny traffic). No XML
complaint in the log, the laptop's RViz still matched, local delivery (SHM to `fsm_node` and the
bridges) unchanged, 0 `EAGAIN` in 4 s of good Wi-Fi. Under a stalled link the *viewer* loses cloud
frames; the estimator never waits. Local topics never touch these sockets.

`strace` delay injection cannot exercise this config (it delays the syscall whether or not the
socket would block), so the proof is the flag plus the soak, not a repeat of the table.

**The 20-min soak on it, laptop RViz subscribed at 47 Mbit/s the whole time, recorder running
(`~/dtp_datasets/20260913_m53_soak_bench_nonblocking/`): 6/6 PASS** — odom 10.00 Hz, **max gap
144 ms** (the earlier run: 944 ms), CPU 281 %, 0 pages swapped, perception 56 %. One soak is weak
evidence on its own (the failing run had one episode in 20 min); the mechanism table above is the
proof. **`nonblocking` is `run.sh`'s default since this session**, so the GCS Stack row gets it too.

## Also found: the laptop is sent every datagram twice

The laptop's DDS participant advertises **both** its Wi-Fi address and its **Tailscale** address as
unicast locators, and FastDDS sends every user-data datagram to both: 48 Mbit/s on the Jetson's Wi-Fi
for a 24 Mbit/s cloud. The second copy goes through `tailscale0` → `tailscaled` → WireGuard → the
same Wi-Fi (or a DERP relay on the **internet** if the direct path fails). The fix is on the
laptop: an `interfaceWhiteList` of its Wi-Fi IP in a FastDDS profile.

**Done 2026-09-14: `scripts/laptop_dds_profile.sh` writes the profile and `laptop_viewer.sh` exports
it by default (`DDS_WHITELIST=0` skips it, `DDS_IFACE_IP=` overrides the address, which is otherwise
the source of the route to `JETSON_IP`).** Verified on the Jetson with the same FastDDS 2.6.12: a
participant under the profile binds its UDP sockets to the whitelisted address only, where a
default participant also holds one on the Tailscale address (that socket *is* the second locator).
Two whitelisted participants on one host talk normally (that is RViz, the image decoder and the
marker publisher). **Trap, measured: a whitelisted participant and a default one on the SAME host
do not exchange data in either direction** — so any other `ros2` command on the laptop needs the
same `FASTRTPS_DEFAULT_PROFILES_FILE` exported, or it sees nothing; the script says so at start.
A network-namespace emulation of the cross-host case was attempted and abandoned: FastDDS's
multicast discovery did not cross the veth, so it proved nothing either way. **The acceptance
test is real:** relaunch the viewer on the laptop, then on the Jetson
`sudo strace -f -e trace=sendto -p $(pgrep -x fastlio_mapping) 2>&1 | grep -o 'inet_addr("[0-9.]*")' | sort | uniq -c`
must show the laptop's Wi-Fi address only, no `100.x`.

**PASSED 2026-09-14 00:15, real laptop launch** (`laptop_dtp_viewer.sh` from the pulled branch):
5 s of FAST-LIO `sendto` went to `192.168.8.200` only (308 datagrams, 2.0 MB/s), plus discovery
on `127.0.0.1` and `239.255.0.1`; **no `100.x` destination**. Jetson Wi-Fi transmit **47 → 19
Mbit/s** with the same RViz view open.

## Field rules

- **Nobody subscribes a laptop to `/cloud_registered` while the aircraft flies** until the fix has
  flown; `ros2 topic info /cloud_registered -v` must list only `fsm_node`.
- `FASTLIO_DDS_PROFILE=nonblocking` is what the flight stack should run; check the `04002` flags
  after any FastDDS or rmw upgrade.
- `strace` is installed on the Jetson since 2026-09-13 (`apt`), for this kind of question.
