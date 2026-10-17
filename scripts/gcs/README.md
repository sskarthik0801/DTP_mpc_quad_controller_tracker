# Web GCS — browser control panel for the Jetson stack

Start the SUPER stack and the PX4 bridges from a browser, and watch battery,
flight mode, estimator health, topic rates, swap — and the camera's live
picture — while you do it.

```bash
# on the Jetson
bash scripts/gcs.sh
```

It prints a URL with an access token — open that on your laptop (or phone).
**Nothing needs installing on the laptop.** No ROS, no `colcon build`, no npm,
no pip: it is Python's standard library on one end and a browser on the other.

```
  http://gypsy.local:8080/?k=<token>              same Wi-Fi
  http://gypsy.tail4e9b1c.ts.net:8080/?k=<token>  anywhere, over Tailscale
```

Ctrl-C stops the server **and everything it started**.

## The URL to bookmark

**Use the names, not the IPs.** The Jetson's Wi-Fi address is DHCP-assigned and
changes between sessions — the same thing that makes `laptop_viewer.sh`'s
`JETSON_IP` a recurring annoyance — so a bookmarked `http://192.168.x.y:8080`
goes stale at exactly the wrong moment. Two names do not:

| URL | Where it works | How |
|---|---|---|
| `http://gypsy.local:8080` | same Wi-Fi | mDNS via `avahi-daemon`. Needs no internet, no DNS server, no config on the laptop or phone. |
| `http://gypsy.tail4e9b1c.ts.net:8080` | anywhere | Tailscale MagicDNS. Off-LAN, through NAT — which the RViz/DDS path can never do. With MagicDNS on the client, plain `http://gypsy:8080` works too. |

The dashboard prints both in its startup banner (**before** the IPs, because
whichever line is printed first is the one that gets bookmarked) and shows them
with copy buttons under **Host → Bookmark**, token included.

### Why the Tailscale URL keeps the `:8080`

`tailscale serve` can put a service on a bare `https://gypsy.tail4e9b1c.ts.net`
with no port. It was tried and **does not work on this box as-is**, for two
independent reasons — recorded so nobody spends the afternoon again:

1. **HTTPS certs are not enabled on this tailnet.** `tailscale serve --https=443`
   just hangs with no output; the actual error only appears if you ask for the
   cert directly: `tailscale cert gypsy.tail4e9b1c.ts.net` →
   *"500 Internal Server Error: your Tailscale account does not support getting
   TLS certs"*. It is an admin-console toggle (**DNS → HTTPS Certificates**),
   not something on the Jetson.
2. **Port 80 is already ARK OS's.** `--http=80` *appears* to succeed and
   `tailscale serve status` cheerfully lists the proxy, but nginx holds
   `0.0.0.0:80` for the ARK UI and wins — so `http://gypsy.tail4e9b1c.ts.net/`
   serves **ARK's page, not this one**, and `/api/*` returns nginx's own 503
   with `Server: nginx/1.18.0`. A `200` on `/` is not evidence the proxy works;
   check the `Server` header.

Port **443 is free**, so if you ever enable HTTPS certificates in the admin
console this becomes available and is worth doing:

```bash
sudo tailscale serve --bg --https=443 http://127.0.0.1:8080   # -> https://gypsy.tail4e9b1c.ts.net
sudo tailscale serve --https=443 off                          # undo
```

Besides dropping the port it would make the page a **secure context**, which
matters concretely: `navigator.clipboard` does not exist on plain `http://`
anywhere but localhost, so every copy button on this page falls back to the
deprecated `document.execCommand('copy')` path. That fallback works, and is
there precisely because plain HTTP on a LAN name is the normal case here — but
the modern API is the one with a future.

Until then, `http://gypsy.tail4e9b1c.ts.net:8080` is the Tailscale URL, and
`http://gypsy:8080` works from any client with MagicDNS enabled.

The board ships as `ubuntu`, the Ubuntu default, which is not safe to rely on:
mDNS names must be unique on a segment, so the moment a second stock Ubuntu
machine joins the field Wi-Fi one of them is silently renamed to
`ubuntu-2.local` — possibly this one, breaking the bookmark while you are
standing in a field. It was renamed to **`gypsy`** on 2026-09-07:

```bash
sudo bash scripts/set_hostname.sh gypsy     # rename, idempotent
bash scripts/set_hostname.sh --show         # no sudo; print the URLs
```

The script also does the three things that are easy to forget:

- rewrites `/etc/hosts`' `127.0.1.1` line, or every `sudo` prints *unable to
  resolve host*;
- sets the **Tailscale** machine name, which does not follow the system
  hostname on its own;
- restarts ARK's `flight-review.service`, which derives its
  `--allow-websocket-origin` from `hostname -f` at start
  (`/usr/lib/ark-os/libexec/start_flight_review.sh`) — nothing of ARK's is
  edited, it just needs a restart to re-derive.

It also sets `deny-interfaces=docker0,eno1` in `avahi-daemon.conf`. avahi
publishes on every interface by default, and two of this board's four are dead
ends for a laptop: docker0, and **eno1 (192.168.1.50), the point-to-point link
dedicated to the MID360** — the exact address `CLAUDE.md` warns not to give
`laptop_viewer.sh`. Before the fix `gypsy.local` resolved to `172.17.0.1`;
after it, to the Wi-Fi address.

**It survives a reboot but not a Jetson reflash** — re-run it after any reimage,
along with the DTB fix (`docs/xrce-agent-nx.md`), the PX4 power-monitor build
(`firmware/README.md`) and `scripts/gcs_service.sh install`.

## Start it at boot

```bash
sudo bash scripts/gcs_service.sh install     # enable + start now
bash scripts/gcs_service.sh status           # no sudo
sudo bash scripts/gcs_service.sh uninstall
```

It installs `/etc/systemd/system/super-gcs.service`, running as the invoking
user (not root — the ROS workspaces and the token live in that user's home).
The token is reused, so a bookmarked URL keeps working across reboots.

**It starts the dashboard only.** Not the stack, not the bridges, nothing that
touches the aircraft. Powering on the Jetson never makes anything fly.

Four details in the unit that are deliberate:

- **`After=network.target`, not `network-online.target`.** The GCS binds
  `0.0.0.0` and works before any interface is up, and `/api/config` re-resolves
  the host IPs per request, so waiting buys nothing — but it would cost up to
  90 s of boot delay if anyone ever enabled `NetworkManager-wait-online` (both
  wait-online units are disabled here today, so `Wants=` would be a trap that
  springs later, not now). At a flying field you want the page immediately.
- **`dds-agent.service` is ordering only, never a requirement.** The dashboard
  must not fail to start because the flight controller is unplugged; it shows
  "NO LINK" and picks the telemetry up when the agent appears.
- **`KillMode=mixed` + `TimeoutStopSec=90`.** systemd sends SIGTERM to the main
  process only, and the GCS then stops every process group it started, in
  order. That teardown can take ~30 s with the stack and three bridges up
  (`fsm_node` holds a ~2.3 GB allocation). Default `KillMode` would signal the
  whole cgroup at once and pre-empt the orderly shutdown.
- **`Restart=on-failure` with `StartLimitBurst=5`.** It comes back from a
  crash, but a broken install does not crash-loop forever. Note that after a
  crash-restart the new server does not own the old stack's process groups, so
  anything still running shows as **`running · not ours`** — still stoppable
  (see *Stop outside nodes*), just not by a group it started.

Verified without rebooting: enabled with the `multi-user.target.wants` symlink
present, `systemd-analyze verify` clean, API live under systemd, `systemctl
stop` with the stack running tore it down in **3.4 s with zero orphans**, and
`kill -9` of the main PID was recovered automatically.

The unit survives apt upgrades but **not a Jetson reflash** — re-run the
installer after any reimage, along with the DTB fix (`docs/xrce-agent-nx.md`)
and the PX4 power-monitor build (`firmware/README.md`).

## Why it is built this way

| Decision | Reason |
|---|---|
| Served from the Jetson, not an app on the laptop | Every value on the dashboard comes from `/fmu/*`, which needs `px4_msgs` **built**. The laptop deliberately has no colcon workspace — the same constraint that produced `super_bringup/scripts/offboard_viz.py`. A browser is the strongest form of "no build". |
| Stdlib `http.server`, vanilla JS, no CDN | There is no Flask, FastAPI, Node or npm on this board, and no internet at a flying field. |
| ROS 2, not MAVLink | `/fmu/out/battery_status_v1`, `vehicle_status_v1`, `estimator_status_flags` and `vehicle_local_position_v1` carry everything the GUI shows, and ARK's `dds-agent.service` is always up — so telemetry is live even with the stack stopped. It also means the GUI never holds `udpin:127.0.0.1:14569`, so every `scripts/px4/*.py` tool keeps working alongside it. |
| Polling at 2 Hz, not SSE/WebSocket | A few hundred bytes per poll, and it reconnects for free when field Wi-Fi drops. |

## What it can and cannot do

**Can:** start/stop the SUPER stack, the M2 odometry bridge, the M3 offboard
bridge, `offboard_viz` and — behind its own password — the **Gemini 336L
camera**, whose live picture it also shows; **record ROS bags** into `bags/`;
**restart cleanly**; **stop nodes that were started
from a terminal**; tail their output; run the checks (`m3_bridge_check`,
`probe_params`, `ekf2_profile --show`, `diag_health`, the FC hardfault scan,
`uxrce_dds_client status`, a `/cloud_registered` rate probe); **archive + clear
the FC's crash dumps**; **reboot the flight controller**; and **power off or
reboot the Jetson itself**.

**Can, behind Admin control:** watch the camera, **move the aircraft** — the
RC card publishes `/goal_pose` and SUPER plans the path (see below) — and
**start and stop the DTP follower**, observe-only or live (see
[The DTP follower](#the-dtp-follower)).

The Stack row starts `fsm_node` on either **click** (`click_smooth_ros2.yaml`,
the default) or **follow** (`follow_test_ros2.yaml`, the M5.2 first-flight
envelope a live follower needs). A running stack's row says which config
`fsm_node` actually loaded, read off its own launch parameters, so a stack
started in a terminal is described truthfully too.

**Cannot, by design:** arm, disarm, change flight mode, engage Offboard, or write
any FC parameter. Those stay on the RC and on
`scripts/px4/offboard_engage.py`, where the pilot is looking at the aircraft
rather than at a browser tab on field Wi-Fi.

### Stop outside nodes

The stack started by hand in a terminal used to show as **`running · not ours`**
with Stop greyed out — honest, and useless at a field where the terminal is on a
laptop you have closed. It is now stoppable: **Stop outside nodes** in the Stack
card sweeps them, the per-row **Stop** works on them, and **STOP ALL** includes
them.

Adopting someone else's process is exactly where `pkill -f` goes wrong, so the
half of the rule that protects anything is kept: the **matching** is still
`sysinfo.find_by_cmdline`'s argv-basename match, never a flattened command line.
What is new is escalating from a matched pid to its **process group**, and the
guard there is that a group is adopted **only when its leader is recognisably
ours** — the leader's own argv basenames must contain one of the node names, or
a wrapper that starts them (`run.sh`, `ros2`). The three shapes that turn up:

| how it was started | group leader | what happens |
|---|---|---|
| `ros2 run super_bringup px4_odom_bridge.py` | the node itself | group adopted |
| `bash scripts/run.sh` as a shell job | `run.sh` | group adopted — run.sh, both `ros2 launch`es and all three nodes go together |
| a node from inside `bash -c '…'` | a **shell** | group refused; the node is signalled by pid, with its children |

A login shell's argv is just `{bash}`, which matches neither a node name nor a
wrapper, so it can never be adopted — and the fallback path cannot take a
terminal with it. `scripts/gcs/test_supervisor.py` asserts all three cases plus
"our own group is never a target", using stand-ins under a pattern name that
matches nothing real, so it is safe to run with the stack up.

The escalation is the same as for our own children — **SIGINT → 15 s → SIGTERM
→ 5 s → SIGKILL** — because these are the same ROS 2 nodes and need the same
grace. Liveness comes from `/proc` rather than `waitpid()`: you cannot wait on a
process that is not your child. Measured: a hand-started full stack (7
processes) went down in **~1 s**, on the SIGINT alone.

**`Ctrl-C` on the server still stops only what it started.** Closing the
dashboard must not kill a stack someone has running in another terminal;
`stop_all(external=True)` is reached from the STOP ALL button and nowhere else.

### Reboot the flight controller

Runs `scripts/px4/fc_reboot.py --yes`, which sends
`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` over UDP 14569, waits for the heartbeat to
drop, waits for it to come back, and confirms the reboot by
`SYSTEM_TIME.time_boot_ms` going *backwards*. Measured on this aircraft: ACK
`ACCEPTED`, link down, **back in 14.1 s**, no new hardfault.

Interlocked twice over, on purpose. The dashboard refuses unless live
`/fmu/out/*` telemetry says **fresh link, disarmed, and not in air**
(`reboot_blocked()` in `server.py`), and `fc_reboot.py` then asks the FC the
same question again over MAVLink. Two independent sources have to agree the
aircraft is on the ground before anything resets the flight controller. Under
`--no-ros` the dashboard refuses outright rather than falling back to the
script's single check — a dashboard that cannot see the vehicle should not be
resetting it.

The reboot lives on its own endpoint (`/api/fc/reboot`) and is deliberately
**not** in the `CHECKS` dict, so `/api/check/<name>` cannot reach it and skip
the interlock. The button is disabled with the reason spelled out beside it
(`unavailable: ARMED — disarm first`) rather than failing when pressed.

**What to expect afterwards, none of it a fault:** every `/fmu/*` topic drops
and comes back as the XRCE session re-forms; EKF2 restarts with no absolute
position until `px4_odom_bridge` feeds it again; and this is precisely the
close-and-reopen of the client's UART behind the NuttX serial use-after-free
that accounts for **22 field crashes** as of 2026-09-13 (`firmware/faults/README.md`). The FC now
runs a client-side fix that keeps the UART open across a *session* loss (30/30 on the bench, soak
pending), but a **reboot** still closes and reopens the UART from scratch, so it remains the risky
case.
Check the card afterwards — the *Scan FC SD card for hardfaults* button does it.

## Remote control

The **Remote control** card, behind Admin control. Forward / back / sideways,
rotate, an altitude slider **and a box to type a height into**, a **coordinate
bar** for one absolute goal, an attitude ball, a compass and an altitude tape.

**Every button publishes exactly one goal message** — the same one RViz's *2D
Goal Pose* tool sends — and SUPER plans a collision-free trajectory to it,
which `px4_offboard_bridge` streams to PX4 only while the pilot has it armed
and in Offboard. The page is not in the control loop: obstacle avoidance stays
with the planner, and a Wi-Fi drop means *no new goal*, not *no setpoints*.

**The pad is relative to where the aircraft is NOW**, not to the last goal, so
tapping forward three times while it is moving does not queue up three metres.
Same rule as `scripts/px4/goal_forward.py`. Forward is where the **nose**
points: the body-frame rotation is `rc.plan_move()`, and `test_rc.py` asserts
each of the four signs, because a sign error there flies it the wrong way and
no HTTP status code would show that.

### Two goal paths, and the card picks one

This is the thing to understand before anything else on the card, because it
decides what the controls are *able* to do. SUPER here has **two** goal inputs:

| | `/goal_pose` (**click**) | `/planning/follow_goal` (**follow**) |
|---|---|---|
| Handler | `setGoalPosiAndYaw()`, `fsm.cpp:186` | `setFollowGoal()`, `fsm.cpp:226` |
| Z | **overwritten** by `fsm/click_height` (`fsm.cpp:188`) | **preserved** — the code says so in a comment |
| Goal < 0.1 m away | **silently dropped** (`fsm.cpp:201`) | accepted |
| Yaw gate | `click_yaw_en` **and** `goal_yaw_en` | `goal_yaw_en` only |
| Result | nothing is ever reported | **`/planning/goal_status`**: ACCEPTED / SNAPPED / DROPPED, with the adopted point and the request echoed |

The follow path is the DTP patch's moving-goal ingestion, enabled with
`fsm/follow_goal_en: true` — which `scripts/setup.sh` now writes into
`click_smooth_ros2.yaml`. **`click_height` is deliberately left at 1.5**, so an
RViz *2D Goal Pose* click (which always sends z = 0) still flies at 1.5 m
rather than at the floor. Only this card moves to the other path.

`rc.choose_path()` prefers follow **wherever the planner is actually
subscribed to it** — a live graph query, not a config read, because what
matters is the node that is running. With no subscriber it falls back to the
click path, which is what a clean `setup.sh` install has: the follow path comes
from drone-dtp's SUPER patch, which this repo does not apply, and unpatched
SUPER never reads the key. The card names the path it chose and what that costs.

**Contention is refused, not merged.** `/planning/follow_goal` legitimately has
other publishers — `follow_planner`'s standoff goals and `command_mux`'s HOLD
and NAV goals — and two sources interleaving goals is worse than either alone.
Our publisher is created at node init, so **more than one publisher means
someone else is driving**, and every command is refused naming that. Stop the
follower, or use its own controls.

**There is no STOP button, deliberately.** SUPER's FSM has exactly three
inputs — click goal, follow goal, yaw target — and none of them is a stop, so a
button here would imply an abort this page cannot perform. The abort is the RC:
**channel-11 Offboard switch OFF**, and you get whatever channel 5 is on (leave
it on Position). The card says so, in those words.

### What the config lets the buttons do

A control the config would ignore is **refused with the reason**, rather than
shipped as a button that silently does nothing — which reads as a broken
aircraft rather than a setting. Capabilities are read off the file the planner
loads (`click_smooth_ros2.yaml`, installed copy first, source fallback) and
combined with the live path, so an edit + restart changes the UI with no code
change. The reader does **not** discover an externally launched DTP
`follow_test_ros2.yaml`; do not treat it as a live inspection of an arbitrary
SUPER process.

| | |
|---|---|
| **Altitude** | Live on the **follow** path, which keeps the z. On the **click** path `fsm/click_height: 1.5` overwrites the z in every goal (`fsm.cpp:188`), so up/down would produce the *same* goal twice — they are refused, naming both ways out (`follow_goal_en: true`, or `click_height: -6` with the RViz-click caveat). |
| **Rotate** | Needs `super_planner/goal_yaw_en` and a trajectory that reaches the goal (`super_planner.cpp:814`) — which short steps do. On the click path it also needs `fsm/click_yaw_en`. Under `yaw_mode: 3` (YAW_TO_TARGET) the goal yaw is never applied, even with no target, so the buttons are refused. **`click_smooth_ros2.yaml` ships 3 since 2026-09-12** (`setup.sh`), so Rotate is refused by default. Set `yaw_mode: 1` and restart to get it back ([`fact-super-yaw-mode.md`](../../docs/fact-super-yaw-mode.md)). |

The asymmetry worth knowing: **up/down is refused when the z would be
overridden, but the coordinate bar is not.** An x/y goal is still a real goal
whatever height it ends up at, so it goes through with a note saying what
height it will really fly at. Up/down would be a no-op, which is the worst
thing a control on this page can be.

Two thresholds SUPER applies **silently**:

* a goal within **0.1 m** of the current position is dropped without a word
  (`fsm.cpp:201`) — **on the click path only**. There, the rotate buttons place
  the goal `YAW_NUDGE_M` = 0.15 m along the new heading so the aircraft turns
  and creeps rather than the button doing nothing. On the follow path there is
  no such threshold and a rotation is a **pure rotation**;
* the goal is **snapped** to the nearest non-inflated-occupied cell up to 3 m
  away, so what it flies to is not necessarily what was sent. On the follow
  path the status message says `SNAPPED` and gives the adopted point; on the
  click path, watch the `-- [Fsm] Get goal at` line in the stack log.

### Typing a value instead of picking one

Every preset has a box beside it, and both drive the same value:

| | |
|---|---|
| **step** | metres per pad press, `STEP_MIN`…`STEP_MAX` (0.25–3 m) |
| **turn** | degrees per rotate press, `YAW_MIN`…`YAW_MAX` (1–180°) |
| **altitude** | an absolute height; Enter or **Go** sends it, exactly as releasing the slider does |

Out of range is **clamped and said**, never silently ignored. The bounds come
from `/api/rc`'s `limits` rather than a constant in `app.js`: the server is
what refuses, and a bound duplicated in the browser is a second bound.
`test_render.js` asserts the boxes advertise the server's numbers, and
`test_rc.py` checks each limit at exactly its edge.

### The coordinate bar

One **absolute** goal, in `lidar_init` — the frame SUPER plans in, whose origin
is wherever FAST-LIO started. Blank `z` holds the current height; **blank yaw
holds the current heading**, which is not the same as `0` (that is a real
command to turn to map north, and a blank box must never mean "rotate").

Beside it, read-only, is the same point **as PX4 would report it** — EKF2's
local frame is `lidar_init` with y and z negated, because `px4_odom_bridge`
declares `POSE_FRAME_FRD` and EKF2 adopts it wholesale. A number copied off
QGroundControl is then visibly a different number *before* it is sent rather
than after. The conversion is `rc.to_px4_ned()`, one helper, so the front end
cannot invent its own signs.

Capped at **`GOTO_MAX_M` = 10 m** from the aircraft. The sliding ROG-Map is
50×50 m so ~25 m is the physical limit; 10 m is well inside it and already
about 100 s of flight at the first-flight `max_vel` of 0.1 m/s. The button
shows the distance and that time as you type, and asks for confirmation above
5 m. A mistyped digit should not be able to send the aircraft across a field.

Altitude is clamped to ROG-Map's **effective** band, which is not the configured
one: it insets both bounds at init, so the shipped `[-1.0, 3.5]` is really
`[-0.5, 2.9]` (CLAUDE.md). Outside it the planner answers *"Odom below virtual
ground"*, which looks like a sensor fault and is not.

### Did it get there?

"Goal sent" is not the interesting confirmation — every press produces one. The
block under the pad tracks the last goal to its conclusion and says which of
six things happened:

| | |
|---|---|
| **Moving** | armed in Offboard and getting closer, with the metres remaining and the time left at the current `max_vel` |
| **Arrived** ✓ | inside **`ARRIVE_M` = 0.1 m** of the goal — which is not a taste, it is SUPER's *own* `closeToGoal(0.1)` (`fsm.cpp:134`), the test the FSM uses to declare the goal reached. This page and the planner therefore mean the same thing by "there" |
| **Sent — but nothing is flying** | SUPER has it and is planning, but the aircraft is disarmed or not in Offboard, so it will not move. **This is the normal bench case**, and reporting it as "moving" would be a lie that never resolves |
| **No progress** ⚠ | armed in Offboard, but no closer for `STALL_AFTER_S` = 20 s — SUPER probably has no free path. At `max_vel` 0.1 m/s a 1 m step takes 10 s and it replans at 15 Hz, so 20 s of nothing is a stall, not a slow patch |
| **SUPER REFUSED the goal** ✗ | `/planning/goal_status` came back `DROPPED`: the point is deeply occupied and the **previous** goal is still active. Only the follow path can tell you this |
| **no odometry** | the pose is stale, so it says it cannot tell rather than guessing |

Arrival is measured against **the point SUPER adopted**, from
`/planning/goal_status`, not the one that was typed — the snap can move a goal
by up to 3 m. Where there is no confirmation the block says so explicitly:
`(the point we sent — SUPER did not confirm one)` on the follow path, and on
the click path a blunter warning that it never reports what it adopted.

Each terminal state is written to the **check log once**, not at the 5 Hz poll
rate — so a session leaves a readable record (`[rc] goal goto …` then
`[rc] Arrived: …`) rather than three hundred identical lines.

### The readiness row

Four things, each of which fails on its own and none of which is visible from
the others — a goal is accepted by a planner that is not running, and flown by
a bridge PX4 is ignoring, without a single error message:

`FAST-LIO odometry` · `SUPER listening` · `Armed` · `Offboard`

`SUPER listening` counts subscribers on **the topic the card actually uses**,
and turns red when the follow topic is contended.

Only the first is a hard gate (no odometry, no frame to put a goal in — the
buttons grey out). The rest are shown, not enforced: sending goals **disarmed**
is exactly how you bench-test the whole path with props off, and the panel says
plainly that nothing will fly.

### What it costs

The one thing on this dashboard with a real CPU price. `/fmu/out/vehicle_attitude`
is **100 Hz** and rclpy cannot be asked for less, so the attitude ball costs
**0.75 ms per message** even `raw=True`. Measured on this Jetson, with the FC
linked:

| | |
|---|---|
| dashboard open, RC card **off screen** | 14.4% of a core |
| dashboard open, RC card **on screen** | **22.7%** |

So the card costs **+8.2% of one core while you are looking at it**, and
**nothing when you are not**: an `IntersectionObserver` stops the 5 Hz poll when
the card scrolls out of view, and the server drops the attitude subscription
15 s later. Altitude and heading are free — they ride the
`vehicle_local_position` subscription the estimator panel already creates.

`/planning/goal_status` rides the same tier and is free in practice: it is one
message per goal, not a stream. It is subscribed **BEST_EFFORT** to match what
fsm offers (`fsm_ros2.hpp:328`) — a default RELIABLE subscription never matches
and never errors, the same trap `docs/fact-super-cmd-stream.md` documents for
`/planning/pos_cmd`. `dtp_msgs` is imported inside a `try`, so a machine
without the DTP tree loses the status and nothing else.

### Power off / reboot the Jetson

The two buttons at the bottom of the **Host** card. They stop the *computer*,
not a program — the flight stack, both bridges, the dashboard and (on a
poweroff) any way of getting it back without walking to the aircraft. At a
field the alternative is an ssh session on a laptop you have already closed, or
pulling power on a board with a mounted filesystem, so the button is worth
having; it just needs more than one click behind it.

**Three gates, and none of them is the others' fallback.**

| | |
|---|---|
| the page asks **twice** | one misplaced tap on a phone is not a shutdown. The second question names the host (`Really power off gypsy now?`), because a dashboard is often open next to another machine's |
| the POST needs `?confirm=<mode>` | `curl` does not run `app.js`, so the asking cannot live only in the page. Nothing reaches this endpoint by accident — not a stray request, not a click on a stale tab |
| `host_power_blocked()` | refuses while the FC reports **ARMED** or **IN AIR**, and while a start/stop is mid-flight |

The interlock is deliberately **not** the same rule as `reboot_blocked()`'s.
That one refuses when it cannot see the aircraft at all — a dashboard that has
lost the FC has no business resetting it. Here, *no telemetry* is the ordinary
end-of-day shape: the FC is off and you want the computer off too. So this
refuses only on **positive evidence that the vehicle is flying**, with one
addition — a link that **said ARMED and then went stale** blocks, because
that is the one shape that could be a vehicle in the air with a dead link.
`hostpower.flight_blocked()` takes the telemetry dict rather than reaching for
it, so the rule that matters is testable with no flight controller in the room.

It runs `sudo -n systemctl poweroff|reboot`:

* **`-n`** so it can never sit on a password prompt nobody will ever see — a
  systemd service has no terminal, and plain `sudo` would block until its own
  timeout with the button apparently doing nothing. If passwordless sudo is not
  available the buttons grey out and say so, rather than failing when pressed.
* **`systemctl`**, not `shutdown -h now`, so the ordinary ordered shutdown
  runs. That is what stops `super-gcs.service`, and *its* teardown SIGINTs a
  running `ros2 bag record` so the bag keeps the `metadata.yaml` without which
  it is not playable at all. If a recording is running, the first question says
  so.
* after a **2 s** delay, so the JSON reply reaches the browser first. A request
  that dies mid-flight just says *lost the Jetson*, which is what a crash looks
  like too — the one ambiguity worth two seconds.

If systemd takes the job and the box is somehow *still here* two minutes
later — an inhibitor, a stop job on its timeout — the pending state clears
itself and says so, rather than leaving a page that insists it is going down
until someone restarts the GCS.

The page then stops calling the link it is about to lose a fault: while a
shutdown is pending it says *powering off — switch the Jetson on again to use
this dashboard*, and after a reboot it says **the Jetson is back** when polls
resume. Note what does **not** come back: `super-gcs.service` is
dashboard-only, so after a reboot you get the dashboard and start the stack
from it.

**Testing it without turning a drone off.** `GCS_POWER_DRY_RUN=1` replaces the
commands with an `echo`, and `/api/state` and the page both say **DRY RUN** so
nobody mistakes a no-op for a working button. `scripts/gcs/test_hostpower.py`
starts its own server on a free port with that set — so a gate that turned out
to be broken echoes a line instead of shutting the Jetson down to prove it.
By hand: `python3 scripts/gcs/hostpower.py` reports whether sudo is available
and what would run, and `sudo -n systemctl --dry-run poweroff` makes systemd
print what it would do.

### Restart cleanly

Stops the processes **owned by this GCS that are currently running**, waits
`RESTART_SETTLE` (3 s) for DDS to drop the old participants, then brings the
same set back in dependency order — stack first with a 6 s head start, then
the bridges.

It restarts what *was* running, not a fixed list: if only the stack was up, you
get the stack back and not three surprise nodes — and on the config it was on.
With nothing running it refuses and says to use Start, rather than silently
doing nothing.

**It is refused while the DTP follower runs** (ours or a terminal's). The
follower's Start has checks a restart thread cannot re-run halfway through, and
taking the stack down under a running follower removes the odometry it places
people in — on an armed aircraft, exactly the sensor loss that trips
`command_mux`'s NAV_LAND abort. Stop the follower first.

The settle is not cosmetic. Starting a new `fastlio_mapping` while the old
one's discovery data is still live gives you two publishers on
`/lidar_slam/odom` for a moment, which is exactly the condition
`docs/ImplementationDoc.md` §M3 warns about. Verified after a restart:
`ros2 topic info /lidar_slam/odom` → `Publisher count: 1`.

Only one such operation runs at a time (`busy` in `/api/state`); while it is in
flight every Start/Stop/check button is disabled, so a double-click cannot have
two threads walking the same process groups. If a restart stops things and
cannot bring them back, it still attempts every remaining start and reports what
failed — stopping everything and silently restoring nothing is the worst thing
this page could do.

### Archive + clear crash dumps

The one button that changes something on the aircraft. It runs
`clear_fault_logs.py --clear`, which copies each `fault_*.log` off the SD card
into `firmware/faults/`, **verifies the local copy, and only then deletes it**.
The UI confirms first and the button is styled differently, because a hardfault
log is the only evidence the FC reset itself.

It never passes `--no-archive`: that path deletes the only copy, and it prompts
on stdin — which a subprocess does not have. (Every child here gets
`stdin=DEVNULL` so a tool that stops to ask a question fails fast instead of
hanging until its timeout with no clue why.)

Archived dumps land in the git working tree as new untracked files. Commit them;
`firmware/faults/README.md` explains how they are decoded.

## Stopping is the part that is actually new

`scripts/run.sh` backgrounds the Livox driver and foregrounds the M1 launch;
its `EXIT` trap kills only the driver PID. Killing `run.sh` from another
session therefore leaves `fastlio_mapping`, `fsm_node` and
`livox_ros_driver2_node` running. That is also why `CLAUDE.md` warns about
`pkill -f` — the pattern matches the killing shell's own argv.

The supervisor here starts every child with `start_new_session=True` and stops
it with `os.killpg()`: **SIGINT → 15 s → SIGTERM → 5 s → SIGKILL**. SIGINT
first, so `run.sh`'s own trap and `ros2 launch`'s shutdown both get to run. It
signals exactly the group it started, by pgid, never by name. Measured
teardown: **4-5 s, no survivors**, for the stack alone or all four at once.
The grace periods are generous because one run in four needed SIGKILL at 8 s —
`fsm_node` holds a ~2.3 GB A* preallocation and FAST-LIO an ikd-Tree, so a
graceful unwind under load is not instant, and a SIGKILLed ROS 2 node can leave
DDS shared-memory segments behind in `/dev/shm`.

The same lesson shows up in `sysinfo.find_by_cmdline()`, which decides whether
something is "running outside this GUI". It matches the **basename of an argv
token**, never a flattened command line — because during development the curl
command *testing* the API contained the string `offboard_viz.py`, and the test
shell was duly reported as a running node.

Stopping something we did **not** start is `stop_foreign()` — same escalation,
and a process group is only adopted when its leader is recognisably ours. See
*Stop outside nodes* above.

### When adopting is allowed at all (fixed 2026-09-10)

Adopting is a per-row **button**, never something a sweep does on its own.
`Managed.stop()` takes an `adopt` flag: the row's own Stop and *Stop outside
nodes* pass `True`, and every sweep — the Ctrl-C teardown and *Restart
cleanly* — passes `False`.

That flag did not exist before, and its absence was a real bug with real
consequences. `stop_all(external=False)` — the server's own shutdown path,
whose docstring promises it "must stop what it started and nothing else" —
walked every row, each row found *its own* outside processes and fell straight
through to `stop_foreign()`. **Closing the dashboard killed a stack somebody
had running in a terminal.** Observed exactly that way: a test GCS on `--port
8099` was Ctrl-C'd and took a hand-started Livox driver, FAST-LIO and all three
PX4 bridges with it, logging `stack: killed 5 process(es) started outside this
GUI` on the way out. **STOP ALL** still reaches them — through
`stop_external()`, which is the explicit ask, and always was.

`test_supervisor.py` case 5 is the regression test: a sweep must answer *not
running* and leave the process alive, and the row button must still kill it.

## CPU cost

Measured on the Orin NX, GCS process only, `/proc/<pid>/stat` over 20-25 s.
For scale, the stack it is watching costs ~165% of a core
(`fastlio_mapping` 63%, `fsm_node` 58%, Livox driver 43%).

| | stack down | stack running |
|---|---|---|
| **nobody watching** (no tab open, or every tab hidden) | **1.50%** | **1.55%** |
| **one browser tab open and visible** | **16.0%** | **20.9%** |

So the dashboard costs about **a fifth of one core, or 2.6% of this 8-core
board, while you are actually looking at it**, and essentially nothing when you
are not. A second visible tab is not free and costs more than the first (GIL
contention between handler threads) — keep one open, not three.

Of the ~21%: roughly 10% is the 50 Hz `vehicle_local_position` subscription,
~5% is the rate-topic subscriptions once the stack is publishing, and ~5% is the
web layer serving one tab (2 Hz `/api/state` + 1 Hz `/api/logs`).

Getting there took four things, all in `telemetry.py`, `sysinfo.py` and `app.js`:

1. **Do not subscribe to a topic whose health another subscription already
   proves.** A first cut also watched `/fmu/out/vehicle_odometry` (100 Hz) and
   `/livox/imu` (200 Hz) and burned **28.5% doing nothing**.
   `vehicle_odometry` only showed "the XRCE link is alive", which
   `vehicle_status` freshness shows; `/livox/imu` only showed "the driver is
   alive", which `/lidar_slam/odom` at 10 Hz shows — FAST-LIO cannot produce
   odometry without IMU and LiDAR. Both are now publisher-presence only.
2. **Every subscription is `raw=True`**, and values are deserialised lazily in
   `snapshot()` at the poll rate, not the publish rate.
3. **The high-rate subscriptions exist only while a browser is polling.**
   `vehicle_local_position` alone is ~8% of a core at 50 Hz even raw.
4. **A hidden tab does not poll** (`document.hidden` in `app.js`), which lets
   (3) tear down. A tab you switched away from used to cost exactly as much as
   the one you were looking at; now the GCS drops to 1.5% within `IDLE_AFTER`
   (15 s) and comes straight back on `visibilitychange`.

rclpy costs roughly 1.6 ms of Python per message on this Jetson, so *what you
subscribe to* dominates everything else — the whole `/api/state` handler is
only 0.94 ms per call.

Point clouds are never subscribed: deserialising ~200k points at 10 Hz would
recreate exactly the CPU pressure M1 spent a session removing.
`/cloud_registered` publishes from the same FAST-LIO callback as
`/lidar_slam/odom`, so the odom rate chip already covers it; the **Deep check**
button runs `ros2 topic hz` as a short subprocess if you want the real number.

**Measuring this yourself:** put the GCS on its own port
(`bash scripts/gcs.sh --port 8095`) so an open browser tab elsewhere does not
skew it, and remember that every measurement of "idle" must wait out
`IDLE_AFTER` first. Both mistakes cost me an hour: a browser tab left open on
another machine made a healthy server look like it had a runaway leak.

## Recording

`ros2 bag record`, from the dashboard, into **`bags/`** at the repo root —
which is in `.gitignore`, because a five-minute bag here is gigabytes.

Two ways to choose what goes in:

* **All topics** — `ros2 bag record -a`. Everything with a publisher, including
  topics that appear *after* the recording starts (rosbag2 keeps discovering).
  This is the default, because it is the choice that cannot be got wrong.
* **Choose topics** — a picker listing every topic that currently has a
  publisher, with its type and a filter box. Chips marked amber are the ones
  that dominate the size (`PointCloud2`, `CustomMsg`, images).

An optional **label** goes in front of the timestamp, so a bag is findable a
week later: `bags/p3-hover_2026-09-10_143302` rather than rosbag2's own
`rosbag2_2026_09_10-14_33_02`. It is sanitised to `[A-Za-z0-9._-]`.

`python3 scripts/gcs/bagger.py` prints the same list from a terminal, with no
ROS sourced.

### The three things that are not obvious

**1. Stop is not optional.** rosbag2 writes `metadata.yaml` when the recorder
*shuts down*. Kill it instead and every message is on disk with no way to read
it back — `ros2 bag play` refuses the directory outright. So a recording is
stopped with **SIGINT**, given the same 15 s grace as `fsm_node`, and the panel
reports what actually happened: a bag with no `metadata.yaml` is listed as
**`NO metadata.yaml`** in red, not as a recording that finished. (`ros2 bag
reindex` can sometimes rebuild one.) The GCS's own Ctrl-C stops a running
recording *first*, before the four process groups, so it finalises while the
box is calm.

**2. The disk guard is not paranoia.** Measured on this aircraft: **87 MB in
5.4 seconds** of *All topics* — close to a gigabyte a minute — and that run had
the LiDAR powered down. Bags land on the same rootfs the flight stack swaps
against, and `swap != 0` is the signature of the M1 RAM regression this repo
spent a session on. So: starting is refused below **5 GB free**, and a running
recording **stops itself** at that floor — cleanly, while there is still room
to write the metadata that makes it readable.

**3. It obeys the camera gate.** `ros2 bag record` is a subscriber like any
other, so recording is the second door to the camera and gets the same lock:
while **Admin control** is locked, image topics are absent from the picker,
naming one explicitly is **403**, and *All topics* is run with
`--exclude 'image_raw|image_rect|image_color|image_color_rect|/compressed'`.
`test_admin.py` asserts all three against a live server.

Two limits worth knowing rather than discovering:

* The exclusion is by **name**, because that is all `ros2 bag record -x` can
  match. It is deliberately broad — over-excluding costs a topic nobody was
  recording, under-excluding leaks the thing the gate exists to withhold.
* It hides the camera's **pictures**, not the fact that a camera exists:
  `/camera/*/camera_info` and `/camera/depth/points` stay in the list for
  everyone, because a depth cloud is flight-test data and `/cloud_registered`
  images a room just as well. The gate is where the repo put it — on the
  imagery — not on the word "camera".

`-x` is also dropped whenever an explicit topic list is given, because Humble's
`ros2 bag record` refuses the combination outright (*"--exclude argument cannot
be used when specifying a list of topics explicitly"*) despite its own `--help`
promising it works. Nothing is lost: an explicit list is already filtered by
name before it gets there.

### Cost

The picker's topic list is fetched **on demand and cached for 3 s**, never from
the 2 Hz poll: listing means one `count_publishers()` per topic, ~100 of them on
this aircraft, and paying that twice a second to render chips nobody is looking
at is the mistake `telemetry.py`'s header exists to warn about. The recording
itself is a separate process — `ros2 bag record` was ~59 MB RSS here — and its
CPU is rosbag2's, not the dashboard's.

## The camera

### It is behind a second password

The camera is the one thing on this dashboard that watches **people**, so it
has its own gate: **Admin control**, in the header. Until you unlock it there
is no camera panel, no camera row in the Stack card and no camera log tab.

The dashboard token and this password say different things — *may fly this
aircraft* and *may switch on a camera and watch what it sees* — and on a shared
field bookmark those come apart.

**It is enforced by the server, not the page.** Every `/api/camera/*` route and
`/api/proc/camera/*` answers **403**, and `/api/state` and `/api/config` do not
mention the camera at all: not the topic, not the device serial, not whether it
is running. The panel is hidden because the data is not there, which is the
right way round — a page that hides a panel whose contents it has already
received is theatre, and `curl` does not run `app.js`. `test_admin.py` asserts
exactly that.

**The gate names nothing.** Its heading is `Admin control`, its button is
`Unlock`, its placeholder is `password`, its message is *"Locked. Enter the
password to continue."*, and the API's own refusal is *"restricted — unlock it
with Admin control"*. All of that is rendered for a client that has unlocked
nothing, so a heading reading "Camera" would hand straight back what the
redaction withholds. `test_render.js` checks both halves — the served markup,
and the message written when the button is actually pressed (asserting on the
untouched element passes on an empty string and proves nothing).

**What that does not cover:** `index.html` and `app.js` are served to everyone,
and `app.js` necessarily contains the panel's own code — `renderCamera`,
`/api/camera/stream`, the string `Gemini 336L`. So *View Source* still shows
that a camera feature exists. Every **fact about the camera** is withheld — the
topic, the serial, whether it is plugged in, whether it is running, and the
frames themselves — but its *existence* is not a secret from anyone who reads
the page source. Hiding that too means serving the panel's markup and script
only to an unlocked client, which is a real change and is not done here.

- The password is stored as a **PBKDF2-SHA256 hash** (200 000 iterations,
  ~93 ms here), never in plaintext. What ships in the repo is the *hash* of the
  default, so the word itself is not in git.
- The session lives in `sessionStorage` — **this browser tab only**. It dies
  when the tab closes, when the GCS restarts, and after 30 minutes in which no
  request carried it. The page polls at 2 Hz, so that timer only ever runs down
  on a backgrounded tab.
- Five wrong passwords in five minutes triggers a 30 s cooldown, so it cannot
  be ground down over Tailscale.
- A wrong password answers **200 with `ok:false`**, not 401 — a 401 makes the
  page throw away the *dashboard* token and show the wrong gate entirely.

```bash
python3 scripts/gcs/admin.py            # which password is in force
python3 scripts/gcs/admin.py --set      # change it (prompts twice, writes 0600)
python3 scripts/gcs/admin.py --check    # does this password work?
```

`~/.cache/super-gcs/admin` wins over the built-in default, so a fresh clone
works out of the box and a machine you have locked down stays locked down
across a `git pull`. **Restart the GCS after `--set`.**

**What this is not:** a defence against someone who can read this repository.
The shipped default is a hash rather than the word, but a short known password
is a short known password. Run `--set` on any machine where that matters —
`admin.py` says so on stdout when the default is still in force.

### What the panel does

The **Camera** panel starts the Gemini 336L and shows its picture in the page.
The feed is an `<img>` pointed at `/api/camera/stream`, which is
`multipart/x-mixed-replace` — **MJPEG**, the oldest trick in the book and the
only kind of video this dashboard could have had: no player, no websocket, no
build step, and it works over Tailscale like everything else here.

**Nothing decodes anything.** The Orbbec driver publishes the camera's own
MJPEG straight through as `CompressedImage`, so a frame goes camera → USB →
DDS → socket as the same bytes. `cv2.imdecode` plus a re-encode per frame would
cost more than the flight stack does.

```bash
bash scripts/gemini_camera.sh          # the same process, from a terminal
python3 scripts/gcs/camera.py          # who has the camera, and on what topic
curl -o shot.jpg 'http://gypsy.local:8080/api/camera/snapshot?k=<token>'
```

### Two separate paths: run the driver, or watch what is already there

They are different actions and the panel keeps them apart:

- **Start camera** launches `scripts/gemini_camera.sh`, which publishes under
  `/gemini`. Offered only when the camera is free (below).
- **The source chips** choose which already-published topic to subscribe to.
  Watching starts nothing and stops nothing — it is an ordinary DDS
  subscription, so somebody else's `/camera` stream is a first-class thing to
  put on the dashboard, not a consolation prize. **Auto** picks ours first and
  otherwise whatever is publishing; clicking a topic pins it
  (`POST /api/camera/source?topic=…`, empty topic = back to Auto).

A pin only holds **while that topic is publishing**; when it stops, the source
falls back to the automatic choice and the panel says the pinned one is gone —
otherwise it would quietly show you a different camera. Switching source does
not interrupt an open stream: the next frame simply comes from the new topic.

### Start is offered only when the camera is free

A Gemini can be opened by **exactly one process**. A second reader dies with
`uvc_open ... Return Code: -6`, and on this Jetson the camera is regularly
already owned by another launch under the `/camera` namespace. So the panel
answers "is it free?" before offering anything, from three sources:

| | |
|---|---|
| a live `CompressedImage` topic | somebody is publishing — **watch that instead** |
| `/sys/bus/usb/devices` | is a Gemini plugged in at all, and at USB 3 speed |
| `/proc/*/fd` → `/dev/bus/usb/…` | which pid holds the device with nothing on ROS |

When it is not free the panel says **which topic** or **which pid** has it, and
both Start buttons (the panel's and the Stack row's) are disabled — the server
refuses the same call with the same sentence, so neither can be bypassed. The
feed still works: watching another process's stream is always allowed, and is
usually what you actually wanted.

The fd walk cannot see another user's file descriptors (CLAUDE.md), so an empty
holder list means "nothing **of ours** holds it". That is why the publisher
count is checked first and the fd walk is only the tiebreak.

**Stopping never touches a foreign camera.** The supervisor row matches
`gemini_camera.sh` / `gemini_camera.launch.py` only — never
`gemini_330_series.launch.py` — so STOP ALL cannot quietly take down another
project's perception stack that happens to share the camera.

### What it costs

Measured on this Jetson with the stack running, GCS on its own port, source
`/camera/color/image_raw/compressed` at **1280×720/30, 132 KB/frame**:

| | |
|---|---|
| idle, nobody looking | **0.95%** of a core — no subscription exists |
| a browser polling, no feed | **10.9%** |
| ... plus one open MJPEG stream | **14.3%** |

So the video is ~3.4 points, and only while somebody is watching. Three things
buy that:

- The image subscription is **created on demand** and dropped `VIEW_TTL` (6 s)
  after the last viewer leaves — the same rule as `telemetry.py`'s 50 Hz
  subscriptions. Check it in `/api/state`: `camera.subscribed` is `null` when
  nobody is looking.
- Closing the tab, hiding it, or pressing **Hide feed** clears the `<img>`'s
  `src`, which is what actually closes the socket. Hiding the element does not:
  the browser keeps reading the body and the Jetson keeps sending.
- The browser stream is capped at **10 fps** (`?fps=` to change it, `0` for
  uncapped). That is a **network** limit — 720p/30 is ~40 Mbit/s, which field
  Wi-Fi will not carry to a phone. The frames still arrive on the Jetson, so it
  saves bandwidth, not CPU.

At most `MAX_STREAMS` (3) viewers at once; the fourth gets a 503 rather than
making the first three stutter.

## The Stack row's config chip: click or follow

The chip chooses which SUPER planner config `fsm_node` loads when the Stack row
starts the stack (`SUPER_CONFIG` for `scripts/run.sh`). The row shows which one
the running `fsm_node` actually loaded, read off its launch parameters, so a
stack started in a terminal reports truthfully too.

| Chip | File under `super_planner/config/` | For |
|---|---|---|
| **click** (default) | `click_smooth_ros2.yaml` | Bench work and manual flying. Goals come from an RViz *2D Goal Pose* click or this page's Remote control card. 0.1 m/s, click height 1.5 m. |
| **follow** | `follow_test_ros2.yaml` | The M5 follow flights. Carries the S1 first-flight envelope (0.5 m/s, 0.5 m/s²), the target-exclusion filter block and `p_max` 0.95. **The DTP follower's Live mode refuses to start unless `fsm_node` is on this one.** |

Why it exists: until 2026-09-13 `run.sh` always started the click config, and
`dtp_bringup` deliberately refuses to start a second `fsm_node` on top of a
running stack. So a follow flight started over that stack would have silently
planned on the click config, outside the M5 envelope and without the filter.
Rule of thumb: **click** for bench work and nudging the aircraft from the
Remote control card, **follow** whenever the DTP follower will be started on top.

## The DTP follower

A row in the Stack card: `scripts/dtp_follower.sh`, which is shell 4 of
[`docs/m5.1-one-graph-runbook.md`](../../docs/m5.1-one-graph-runbook.md) —
`dtp_system.launch.py lidar:=false super:=false`, riding on the running stack.
It never starts its own Livox driver, FAST-LIO or `fsm_node`.

| Mode | What it does |
|---|---|
| **Observe only** (default on every page load) | `observe_only:=true`. Goals and yaw targets go to `/dtp/shadow/*`, the NAV_LAND abort is off. It commands nothing. |
| **Live** | `observe_only:=false`. `command_mux` publishes `/planning/follow_goal` and holds NAV_LAND armed. It commands the aircraft once the pilot arms and flips channel 11 to Offboard. |

**In plain terms.** Both modes run the same perception, tracking and goal
planning; the difference is only where the output goes.

- **Observe only** is a dry run. The follower computes goals and yaw targets for
  the person it locks onto and publishes them to `/dtp/shadow/*`, where nothing
  subscribes. SUPER never sees them, the aircraft never moves because of them,
  and a sensor loss just holds. This is the Stage A mode: fly the aircraft by
  hand, record what the follower *would* have commanded, score it afterwards.
- **Live** is the real thing, in three ways:
  - `command_mux` publishes the goals on `/planning/follow_goal`, so SUPER plans
    toward the person and the M3 offboard bridge turns that into PX4 setpoints;
  - the abort is armed — on a sensor loss the mux can send NAV_LAND to the
    flight controller (in Observe only the abort is disabled);
  - the yaw target goes to SUPER, so the nose turns toward the person.

**What Live does not do.** It never arms the aircraft and never changes flight
mode. The pilot arms and engages Offboard on the RC channel-11 switch, and that
same switch is the takeover. Until the pilot does that, a Live follower is
producing goals that nothing flies.

**Where it stands.** Live has never been started from this page on the
aircraft — its tests and the wrapper's refusals only. The M5.1 runbook is the
first place it gets used for real: props off, a pilot on the RC, a walker.

### Start is refused unless

The server decides (`dtp.start_blocked()`), the row shows the reason for the
selected mode **as text, not a tooltip**, and the POST is refused with the same
sentence.

- **Both modes:**
  - not already running, here or in a terminal;
  - drone-dtp installed (`~/super_ws/src/drone-dtp/…/dtp_env_jetson.sh` and
    `install/dtp_bringup`);
  - the SUPER stack up (`livox_ros_driver2_node`, `fastlio_mapping`,
    `fsm_node`), with exactly one `fsm_node`;
  - the Gemini free: not the Camera panel's, no other image publisher, no USB
    holder;
  - the FC not saying **ARMED** (even on a link that has since gone stale) or
    **IN AIR**. This holds for observe too: perception's first TensorRT
    inference stalls the Jetson for seconds.
- **Live only:**
  - `?confirm=live` on the POST (the page asks first);
  - ROS telemetry, and a **fresh** FC link that says disarmed: silence is not
    "disarmed";
  - `fsm_node` on `follow_test_ros2.yaml`, read off its launch parameters;
  - `/lidar_slam/odom` at ≥ 9.5 Hz.

**The wrapper checks the config again.** Before a live launch,
`dtp_follower.sh` asks the running `/fsm_node` for `config_name` over ROS and
refuses anything but the follow config. That is two independent sources, like
Reboot FC.

### Stop

- **Never refused**, and it is the usual SIGINT → SIGTERM → SIGKILL on the
  launch's process group, so the camera driver goes with it.
- **The page asks first.** If the aircraft is armed, the question says what
  stopping does: a live follower's goals stop, SUPER flies to the last one and
  holds, and the NAV_LAND abort stops with it.
- **A live follower on an armed aircraft is asked about twice.**
- **After an ABORT, Stop then Start** before re-engaging channel 11 (the mux is
  terminal).

### What the row shows

- **The mode is read off the running nodes.** It comes from `command_mux`'s
  `arbitrated_goal_topic` and `follow_planner`'s `yaw_target_topic` launch
  parameters, not from what this page asked for. So a follower started in a
  terminal shows its real mode, and a half-shadowed one shows **MIXED**.
- **The row is shown to a locked client** (state and mode). Whether something
  is commanding the aircraft is not something to hide. Its controls, its log
  tab and its reasons are withheld, because the reasons name the camera.
- **STOP ALL stops it**, locked or not.
- **The Camera panel refuses to start** while a follower is running, even
  before its camera publishes.

### Engage / Re-engage FOLLOW (added 2026-09-15)

The one control on this page that **designates a target**: `POST /api/dtp/follow?set=on`
(and `?set=off`), the DTP row's **Engage FOLLOW** / **Re-engage FOLLOW** / **Disengage**
buttons, and the same button on the Flight day card's engage step. It does exactly what
`field.sh arm` and `field.sh reengage` do: `target_manager`'s `follow_enabled` parameter set
`false`, a 0.7 s beat, then `true`, over the node's own `set_parameters` service
(`telemetry.follow()`), so the same button engages from IDLE and re-designates from LOST.

Why it exists: on 2026-09-14 (Sortie 3) the lock was lost one second after arming, nobody
re-engaged from a shell, and the whole 44 s Offboard segment flew a hold. `auto_reacquire` is
off on purpose (it would follow a stranger), so a LOST in flight needs a human in seconds.

- **Behind Admin control**, like every `/api/dtp/*` route.
- **Refused** (`dtp.follow_blocked()`, pure, tested) without a running follower, without ROS
  telemetry, before `target_manager` serves its parameter service, and — for engage only —
  after a `command_mux` ABORT, which is terminal until the follower is relaunched.
- **Not refused on an armed aircraft, and the page asks no confirm question.** Both on
  purpose: that is the moment the button is for. On a LIVE follower with the aircraft armed in
  Offboard, pressing it makes the aircraft move toward the person it locks. Channel 11 OFF is
  still the override.
- **The lock state is shown to everyone**, as a pill from `/dtp/lock_state` (LOCKED green,
  ACQUIRING blue, COASTING amber, LOST red) with the mux state beside it; the buttons only to
  an unlocked client. Both topics join the hot tier (subscribed BEST_EFFORT/KEEP_LAST(1) only
  while a browser polls; `/mux/state` is ~35 Hz, kept raw and decoded once per poll).
- The `follow` field is absent from a server older than this page, and the page then shows
  no button — the usual "serves the new page but not the new API until restarted" shape.

Verified 2026-09-15: `test_dtp.py` (rule + endpoint), `test_render` (21 actionable steps), and
one end-to-end `follow(True)` against the real follower on the aircraft: LOST → ACQUIRING.

### Not yet verified

- **A start and stop from the dashboard has not been run on the aircraft.**
  - Covered so far:
    - `test_dtp.py`: every refusal, the process readers, the whitelists, and the
      server gates;
    - the render test: all row shapes;
    - the wrapper's refusals, including the live `ros2 param get`, against the
      real stack.
  - Still to confirm: the teardown time under TensorRT and the Orbbec container,
    and that the camera is released after Stop.
- **The mode and config readers need launch parameters.**
  - They read `ros2 launch`'s `/tmp/launch_params_*` files.
  - A node started without them, or by another user (the files are 0600), reads
    as "cannot tell". That refuses live and changes nothing else.

## The Flight day card

**Since 2026-09-20 it is the standard follower bring-up, not the runbook.** It used to render
`docs/flight-day-runbook.md` row for row (three sorties, 37 steps, most of them the pilot's or a
terminal's and so without a button). Now it holds only what is pressed on every bring-up, and
every row is a button. `scripts/gcs/flightday.py` is the step table (data only); `buildFlight()` /
`renderFlight()` in `app.js` draw it.

**One choice at the top: Observe only, or Live.** It is the same variable as the DTP row's mode
chips, so the two cannot disagree, and it comes back to Observe on every page load. While a
follower runs the card shows the mode read off its processes and the chips are fixed (stop it to
change mode). A step with `modes` shows in those modes only, and the showing steps are numbered
1..n.

| # | Step | Observe | Live |
|---|---|---|---|
| 1 | Stack on the follow config | same | same |
| 2–3 | Odometry bridge, trajectory bridge | same | same |
| 4 | FAST-LIO gates (`field_check`) | same | same |
| 5 | Start the follower | `mode=observe` | `mode=live` (asks, amber, the row's refusals) |
| 6 | Record | preset `stage_a` | preset `stage_b` |
| 7 | Preflight | `stage_a_preflight` | `live_preflight` |
| 8 | Engage FOLLOW | same | same |
| 9 | Estimator preflight, before every later arm | same | same |
| 10–12 | Stop the recording, stop the follower, scan the fault card | same | same |

- The recorder goes up **before** the preflight, as in `deploy_follow.sh`, so the preflight sees
  it instead of warning that it is absent.
- **`live_preflight` is a new check:** `stage_a_preflight.py --stage-b --target person --max-vel
  1.0 --waive-d21`, the flags `deploy_follow.sh` passes. The strict `stage_b_preflight` chip FAILs
  on the person ceiling and D21 every run, so it could not be the standard gate. **Keep `1.0` in
  step with `max_vel` in `follow_test_ros2.yaml` and with `deploy_follow.sh`.** It still FAILs on
  `auto_reacquire_after_lost` by design (set true 2026-09-15): read the log and check that is the
  only FAIL.
- **Stop the recording** is `POST /api/bag/stop`, what the Recording card's Stop sends.
- Every button is one the page already had, so the interlocks, the Admin gate and the LIVE
  confirmation apply unchanged. The pills are as before: a check's last verdict
  (`Supervisor.check_results`), a row's state, the recorder, `/dtp/lock_state`.
- **Dropped from the card, still chips on the Checks card:** `desk`, `rc_switches`,
  `stage_b_preflight_mannequin`, `stage_b_preflight`. Dropped entirely: the terminal and
  pilot/walker rows (bench pre-check, the geofence edit, props off, the M5.1 trials, scoring) and
  the `m51` recording step. They are in the runbook; the `m51` preset is still in the Recording
  card's API.
- `test_render.js` asserts a row per step, a button on every one, the mode chips, that only the
  picked mode's steps show, the numbering, and that both paths are complete.

**What it deliberately cannot do:** arm, change a mode, engage Offboard, or stop the aircraft.
Those stay with the pilot, as everywhere else on this page.

## The RViz button

RViz is a native 3D app, so a web page cannot launch it. On the laptop you want
RViz to open on, paste the line the dashboard shows you:

```bash
bash scripts/gcs_laptop.sh 'http://gypsy.local:8080/?k=<token>'
```

Quote it — the shell will eat the `?`. The helper needs the token because it
talks to the GCS API. It also opens the dashboard in your browser.

**The helper is a client, not a server.** It polls the Jetson once a second for
a pending command and reports whether RViz is up; the page's buttons just queue
`start`/`stop` on the Jetson, same-origin. It listens on nothing.

That is not the obvious design, and the obvious one is broken. A small HTTP
server on `127.0.0.1:8081` that the page calls is a **private-network request**
— a page served from `192.168.x.x` reaching loopback — which Chrome blocks
unless the preflight carries `Access-Control-Allow-Private-Network`, with more
restrictions coming. It also could never work from a phone, where `127.0.0.1`
is the phone. Polling outward has no cross-origin story at all, so there is
nothing for a browser to block, and the dashboard works from any device while
RViz still opens on the laptop.

RViz itself is started by running **`scripts/laptop_viewer.sh` unchanged**, and
stopped by process group. Reusing that script verbatim is the point — it carries
the verified `env -i` allow-list for the snap-glibc bug (`rviz2: undefined
symbol: __libc_pthread_init`, which bit twice) and the `/viz/drone_radius`
marker publisher. None of it is reimplemented here; reimplementing it is how
that bug came back the second time.

The helper is optional. Without it the buttons are **disabled** — not merely
inert — and the panel shows the command to paste instead. If RViz starts and
then dies (no `DISPLAY`, a Qt plugin problem), the helper says so on the
dashboard rather than silently flipping back to "not running", which would look
exactly like a button that does nothing.

## Access token

The server binds `0.0.0.0` and can start and stop the stack, and field Wi-Fi is
usually open, so `/api/*` requires a token (`X-GCS-Token` header or `?k=`). It
is generated once into `~/.cache/super-gcs/token` and persists, so a bookmark
keeps working. `--bind 127.0.0.1 --no-auth` disables it for local use.

**The browser remembers it.** A memorable hostname is one you *type*, and
typing it drops the `?k=…` — the page then loads while every API call 401s,
which reads as a broken dashboard rather than a missing credential. (It is
exactly what happened the first time `gypsy.local` was used.) So the first
successful load stores the token, later visits fall back to it, and a URL with
no token shows a **paste-it-once box** instead of a dead red banner. A token
that stops being accepted is forgotten rather than retried forever, and a 401
is never reported as "lost the Jetson" — that would send you debugging Wi-Fi.

`localStorage` and deliberately **not** a cookie: a cookie is attached
automatically to cross-site requests and this API can stop the stack, whereas
stored state is readable only by a page from the same origin and is only ever
sent because `app.js` puts it in an explicit header. Every access is wrapped in
try/catch — `localStorage` throws outright in some privacy modes, and a
dashboard must not fail to render because of a storage preference.
`scripts/gcs/test_gate.js` covers all of this, including the storage-throws
case (gjs has no `localStorage` at all, so the test gets it for free).

This is a shared secret over plain HTTP on a trusted LAN — enough to stop a
stray device on the same Wi-Fi from stopping your stack mid-flight. It is not
authentication against someone who can already sniff the network.

## Files

| | |
|---|---|
| `scripts/gcs.sh` | entry point (Jetson) — sources both workspaces, execs `server.py` |
| `scripts/gcs_laptop.sh` | entry point (laptop, optional) — browser + RViz helper |
| `scripts/gcs/server.py` | HTTP server, JSON API, token auth, shutdown |
| `scripts/gcs/telemetry.py` | rclpy node: `/fmu/out/*` + topic rates |
| `scripts/gcs/camera.py` | rclpy node: camera frames; USB probe; also a CLI |
| `scripts/gcs/admin.py` | the camera password: hash, sessions, cooldown; also a CLI |
| `scripts/gcs/supervisor.py` | process-group start/stop, log ring, one-shot checks, the bag `Recorder` |
| `scripts/gcs/bagger.py` | bag paths, naming, disk arithmetic, `bags/` listing; also a CLI |
| `scripts/gcs/rc.py` | remote control: goal maths, path choice, config capabilities, altitude band; also a CLI |
| `scripts/gcs/dtp.py` | the DTP follower's Start interlock; which config `fsm_node` loaded and which mode a follower is in, from launch parameters |
| `scripts/gcs/hostpower.py` | poweroff/reboot of the Jetson: the flight interlock, the sudo probe, the delayed command; also a CLI |
| `scripts/gcs/sysinfo.py` | `/proc` readers: load, RAM, swap, per-tree RSS/CPU |
| `scripts/gcs/rviz_agent.py` | laptop-side helper (polls the Jetson) |
| `scripts/gcs/static/` | `index.html`, `app.js`, `style.css` |
| `scripts/gcs/test_render.py` | front-end tests (render + token gate) — see below |
| `scripts/gcs/test_supervisor.py` | adopt-and-stop rules; needs no ROS and no FC |
| `scripts/gcs/test_camera.py` | the camera interlock + JPEG parser; needs no camera |
| `scripts/gcs/test_admin.py` | that the camera gate is enforced server-side, on both doors |
| `scripts/gcs/test_hostpower.py` | the three gates on powering the Jetson off — starts its own server, in dry-run |
| `scripts/gcs/test_rc.py` | RC goal maths (the sign errors that fly it the wrong way) + the admin gate |
| `scripts/gcs/test_dtp.py` | the follower interlock, stack-config whitelist, Restart refusal, and the server gates — starts its own server, starts no follower |
| `scripts/gemini_camera.sh` | the camera process this panel starts |
| `scripts/dtp_follower.sh` | the DTP follower process the Stack card starts (`DTP_MODE=observe\|live`) |
| `scripts/set_hostname.sh` | the stable `.local` / Tailscale name — see above |

## Options

```
bash scripts/gcs.sh --port 9000            # default 8080
bash scripts/gcs.sh --bind 127.0.0.1 --no-auth
bash scripts/gcs.sh --no-ros               # process control only, skip rclpy
```

`--no-ros` still probes USB, so the camera panel can still say *why* it will
not start one; it just cannot show the feed. Bag recording still works there
too — `ros2 bag record` is a subprocess with its own ROS context — but the
topic **picker** goes empty, because listing topics needs a graph query. *All
topics* is unaffected.

`--no-ros` is the useful one when debugging: it starts in a second and tells
you whether a problem is in the web layer or in ROS.

## One concurrency bug worth knowing about

`status()` runs on every HTTP poll while `stop()` may be running on another
thread. The first version had `status()` clear `self.proc` the moment it saw the
process exit, which pulled it out from under `stop()`'s
`self.proc.poll()` — `'NoneType' object has no attribute 'poll'`, thrown from
inside a restart, which stopped all four processes and started none.

It only reproduces while a browser is actually polling, which is always in real
use and never in a quick curl test. `Managed` now serialises `start`/`stop`
behind `self._op`, every reader takes a **local snapshot** of `self.proc`, and
`status()` does not clear it while a deliberate stop is in flight. If you add a
method that touches `self.proc`, do the same.

## Testing the front end

There is no browser on the Jetson, so `app.js` gets executed head-lessly against
a real API snapshot:

```bash
python3 scripts/gcs/test_render.py          # with the GCS running
```

It stubs a minimal DOM and `fetch`, runs the real render path against a real
`/api/state`, and asserts on what came out — flight mode is a name and not a raw
enum, an absent battery says so instead of showing 0 V, the RViz buttons are
disabled when no helper is connected, the FC-reboot button is offered only when
the vehicle is disarmed and landed, the bookmark URLs are name-based and carry
the token, and so on. A parse check cannot catch a renamed field; this can.

It then runs `test_gate.js` over the same snapshot for the **auth** path: no
`?k=` in the URL, 401 from every call, then a pasted token. Both passes run even
if the first fails. Needs `gjs` (`apt install gjs`).

`test_camera.py` covers the camera interlock separately, because that decision
is rendered twice (the panel's Start and the Stack row's) and must not be
trusted twice. `test_admin.py` covers the password gate against a **running
server**, which is the only place it means anything:

```bash
python3 scripts/gcs/test_camera.py          # no ROS, no camera, no GCS needed
python3 scripts/gcs/test_dtp.py             # its own server; starts no follower
python3 scripts/gcs/test_admin.py           # the locked half: 403 everywhere
GCS_ADMIN_PASSWORD='…' python3 scripts/gcs/test_admin.py    # both halves
```

`test_dtp.py` points its server's `HOME` at a scratch directory holding a test
admin password, so it covers the unlocked half without knowing the real one. It
never POSTs a follower start the server's own state did not already refuse, and
never POSTs a stop with a session: a stop adopts, and a follower is often running
in somebody's terminal. `test_render.py` also renders the follower and Stack rows
in synthetic **stopped** and **running** shapes, so the half this box is not in
right now (where *Start LIVE* lives) is still exercised.

The front-end tests default to the **locked** shape, which is what a browser
lands on. Set `GCS_ADMIN_PASSWORD` for `test_render.py` too and it logs in
first, so the camera panel's own render path is exercised as well. Its last
case deliberately trips the login cooldown, so wait ~30 s before logging in
again.

One thing the harness gets right that is easy to get wrong: it reads the
`hidden` **attribute** out of `index.html` rather than defaulting every element
to visible. Without that, "the gate is not open until you ask for it" passed
whatever the page did — and the tempting fix, having a render function hide it,
would have slammed the gate shut on the next 2 Hz poll while you were typing
into it.

Only the live state's own branches get exercised, so force the others with a
hand-edited snapshot when you touch them:

```bash
python3 -c "import json;d=json.load(open('/tmp/s.json'));d['ros']['aircraft']['armed']=True;json.dump(d,open('/tmp/armed.json','w'))"
gjs scripts/gcs/test_gate.js /tmp/armed.json /tmp/cfg.json scripts/gcs/static
```

## Troubleshooting

- **"Access token needed"** — the URL had no `?k=<token>` and this browser has
  not seen one before. Paste it into the box; it is remembered, so the bare
  `http://gypsy.local:8080` works from then on. The token is in the server's
  startup banner and in `~/.cache/super-gcs/token` on the Jetson.
- **`gypsy.local` does not resolve from the laptop** — mDNS is blocked on some
  guest/isolated Wi-Fi (the same client isolation that breaks DDS discovery).
  Use the Tailscale URL, which does not depend on the LAN at all. On Windows,
  `.local` needs Bonjour; Tailscale is the simpler answer there too.
- **Flight mode says "NO LINK"** but processes are fine — the XRCE link to the
  FC is down, not the GUI. Run the *uxrce_dds_client status* check.
- **`WARNING: telemetry disabled`** on startup — `server.py` was run directly
  instead of through `scripts/gcs.sh`, so the workspaces were not sourced.
- **A process shows `running · not ours`** — it was started from a terminal.
  Press **Stop** on that row, or **Stop outside nodes**; the detail lands in the
  *checks* log tab. If it reports that only pids were signalled and not a group,
  that is the shell guard doing its job, not a failure.
- **"Reboot FC" is greyed out** — the note beside it names the interlock that is
  holding: no link, ARMED, or IN AIR.
- **"Open RViz" is greyed out** — no helper is connected. Run the
  `scripts/gcs_laptop.sh` line the panel shows, on the laptop, and quote it.
- **RViz says "exited immediately"** — look at the terminal running
  `gcs_laptop.sh`; the cause is printed there. Usually no `DISPLAY`, or ROS 2
  is not installed on that laptop (`scripts/laptop_setup.sh`).
- **`Address already in use` on start** — a previous GCS is still shutting down.
  It now waits up to 20 s for the port; if it still fails, one really is running.
