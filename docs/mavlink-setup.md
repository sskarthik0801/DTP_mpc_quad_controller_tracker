# MAVLink on the Orin NX module

**Status: working, verified 2026-08-30.**

> ⚠️ **Superseded mechanism (2026-08-31): this NX was reimaged with ARK OS.** ARK's
> `mavlink-router.service` + `/etc/ark-os/mavlink-router.conf` and `dds-agent.service` now own both
> FC links as user `jetson`; our `/etc/mavlink-router/main.conf` is **inactive** and our
> `xrce-agent.service` is **disabled**. ARK's config still exposes `companion_aux` on **14569**, so
> everything below about endpoints and `scripts/px4/*.py` still applies unchanged — only the
> *owner* differs. The "no ARK OS" framing in this file is stale.
>
> ⚠️ **M2 is no longer blocked.** The uXRCE-DDS link is up as of 2026-08-31; the blocker was a
> Tegra UART DMA fault, not MAVLink or wiring. See [`xrce-agent-nx.md`](xrce-agent-nx.md).

## Why this file exists

The Orin Nano ran a preinstalled **ARK OS** image whose services already owned both FC links, so
the rule there was *never open those serial devices*. **This Orin NX module runs a stock JetPack
image** (L4T R36.5.2). There is no ARK OS, no `dds-agent.service`, and no `mavlink-router.service`
— nothing owned the FC links, and nothing bridged MAVLink to the network. QGroundControl could not
reach the FC through the Jetson at all until this was set up.

The port-ownership rule still applies, but for a different reason: **we** now run mavlink-router,
and it is the single owner of `/dev/ttyACM0`. A second reader still corrupts the stream.

## What was installed

`mavlink-router` **v4** (release tag), built from source — it is not packaged for Ubuntu 22.04:

```bash
sudo apt-get install -y meson ninja-build pkg-config
git clone --recurse-submodules https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router && git checkout v4 && git submodule update --init --recursive
meson setup build . && ninja -C build && sudo ninja -C build install
```

`pymavlink` (2.4.49) via `sudo pip3 install pymavlink` — also not in apt for jammy. The
`scripts/px4/*.py` tooling needs it.

Config lives at **`/etc/mavlink-router/main.conf`**; the service is
`mavlink-router.service`, enabled at boot.

## Endpoints

| Endpoint | Mode | Address | For |
|---|---|---|---|
| `fc` | UART | `/dev/serial/by-id/usb-ARK_ARK_FPV.x_0-if00` @ 2000000 | the ARK FPV flight controller |
| `qgc` | UDP **Server** | `0.0.0.0:14550` | QGroundControl from a laptop |
| `companion_aux` | UDP **Normal** | `127.0.0.1:14569` | `scripts/px4/*.py` |
| — | TCP Server | `:5760` | QGC over TCP / Tailscale |

Two details that are easy to get wrong:

- **The FC is addressed by its `by-id` path**, not `/dev/ttyACM0`. The `ttyACM` number is not
  stable across reboots or re-plugs; `usb-ARK_ARK_FPV.x_0-if00` is.
- **`companion_aux` must be `Mode = Normal`, not `Server`.** The scripts bind
  `udpin:127.0.0.1:14569` and *wait to be sent to*, so the router has to act as the client and
  push to that port. Setting it to `Server` makes both ends listen and nothing ever flows.

Baud `2000000` on a USB CDC-ACM device is ignored by the kernel, but mavlink-router requires the
key. The value matches what ARK OS used, and PX4 reports the link as
`transport protocol: serial (/dev/ttyACM0 @2000000)`.

## Connecting QGroundControl from a laptop

The `qgc` endpoint is a **UDP server**, so it binds and waits rather than pushing to a fixed
address. That is deliberate: it works no matter what address your laptop has. QGC must therefore
send first, which means adding a link manually — auto-connect alone will not find it.

1. Find the Jetson's address: `hostname -I` on the Jetson. Use the **Wi-Fi** address
   (`wlP1p1s0`, currently `192.168.8.130`) or the **Tailscale** one (`100.124.49.34`).
   **Not** `192.168.1.50` — that is the point-to-point link to the MID360 and is not routable.
2. QGC → *Application Settings* → *Comm Links* → *Add*.
3. Type **UDP**, port `14550`, and add the Jetson's address as a **Target Host**.
   (Or type **TCP**, host = Jetson address, port `5760`.)
4. Connect.

Over Tailscale this works from anywhere, which is the easier option when the lab Wi-Fi has client
isolation enabled.

## Verifying

```bash
systemctl is-active mavlink-router
python3 scripts/px4/px4_shell.py "mavlink status"
python3 scripts/px4/probe_params.py
```

Known-good output on this module:

```
shell on sys=1 comp=1
	type:		USB CDC
	transport protocol: serial (/dev/ttyACM0 @2000000)
```

`rx loss` of ~13% on the FC's MAVLink counters is normal here — it counts the gaps while a client
is not attached, not corruption on the wire.

## What this does NOT give you

MAVLink is not the M2 path. **uXRCE-DDS is still missing on this module** — see
[`ImplementationDoc.md`](ImplementationDoc.md) §M2. The FC side is already configured and running:

```
uxrce_dds_client status  ->  Running, disconnected
UXRCE_DDS_CFG = 101   (TELEM1)
SER_TEL1_BAUD = 3000000
```

`disconnected` is because there is no agent on the Jetson. To finish M2 here you need:

1. **Micro-XRCE-DDS-Agent** built from source (not in apt for Humble), run against
   `/dev/ttyTHS1 @ 3000000`.
2. **`px4_msgs`** at `release/1.17` cloned into `~/super_ws/src` and built — it is absent, and
   `setup.sh` does not fetch it. `px4_odom_bridge.py` does `from px4_msgs.msg import
   VehicleOdometry` and cannot run without it.
3. **Confirmation that TELEM1 is physically wired to the Jetson's `/dev/ttyTHS1`** on this module.
   `UXRCE_DDS_CFG=101` proves the *FC* is configured for it; it says nothing about whether the
   harness on this airframe actually connects those pins.
