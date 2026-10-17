---
paths:
  - "scripts/gcs/**"
  - "scripts/gcs.sh"
  - "scripts/gcs_service.sh"
  - "scripts/gcs_laptop.sh"
  - "scripts/set_hostname.sh"
  - "scripts/dtp_follower.sh"
---

# The web GCS (`scripts/gcs/`)

Scope: the browser dashboard: process supervision, host power, bag recording, the systemd unit, the hostname/URL, and the DTP follower row. The RC card is in `gcs-remote-control.md` and the camera panel in `gcs-camera.md`. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

- `scripts/gcs/` — **the web GCS**: `bash scripts/gcs.sh` on the Jetson serves a browser
  control panel (start/stop/**restart** the stack (click or follow config), both bridges and **the DTP follower**, **stop nodes started from a
  terminal**, battery, flight mode, EKF2 flags, topic rates, swap, the bench checks,
  **archive+clear the FC crash dumps**, **reboot the FC**, **power off or reboot the
  Jetson**, **record ROS bags into `bags/`**, **fly the aircraft by nudging SUPER's goal
  (behind the admin password)**,
  and **the Gemini 336L camera —
  started from the page, with its live picture in the page, behind its own password**). Stdlib `http.server` + vanilla JS
  — **no pip, no npm, no build**, because neither a web framework nor Node exists on this board
  and there is no internet at a flying field. Telemetry is **ROS 2 (`/fmu/out/*`), not MAVLink**, so it never
  holds UDP 14569 and every `scripts/px4/*.py` tool still works while it runs. It works over
  **Tailscale**, which the RViz/DDS path cannot. It deliberately **cannot arm, change mode, or
  write a parameter**. Two buttons do touch the aircraft, both interlocked: clearing the crash
  dumps (archives and verifies before deleting), and **Reboot FC** — refused unless live telemetry
  says fresh link + disarmed + not in air (`reboot_blocked()` in `server.py`), and refused *again*
  by `scripts/px4/fc_reboot.py`, which asks the FC the same question over MAVLink. It sits on its
  own endpoint and deliberately **not** in the `CHECKS` dict, so `/api/check/<name>` cannot skip
  the interlock.

## Process supervision and its cost

**Two things it settles, don't re-derive:** (a) **`scripts/run.sh` cannot be stopped from
another session** — its `EXIT` trap kills only the backgrounded Livox driver, so the
foreground `ros2 launch` and `fastlio_mapping`/`fsm_node`/`livox_ros_driver2_node` survive.
The GCS supervisor starts every child with `start_new_session=True` and stops it with
`os.killpg()` (SIGINT→15 s→SIGTERM→5 s→SIGKILL); teardown measured at **4-5 s, no survivors**,
for the stack alone or all four at once. That is the safe way to stop the stack remotely, and
the general fix for the `pkill -f` trap: **signal a process group you own, never a name you
matched.**
(a1) **A stack started by hand in a terminal is now stoppable from the dashboard too**
(2026-09-07) — per-row **Stop**, **Stop outside nodes**, and **STOP ALL** all sweep it; a
hand-started full stack (7 processes) went down in ~1 s on the SIGINT alone. The rule above is
not weakened, only halved: the **matching** is still `find_by_cmdline`'s argv-basename match,
and a process **group** is adopted only when its leader is recognisably ours (a node name, or
`run.sh`/`ros2`). A login shell's argv is just `{bash}`, so it can never be adopted; when the
group is refused, the matched pids and their children are signalled individually instead —
less tidy, never able to take a terminal with it. `scripts/gcs/test_supervisor.py` asserts all
three shapes plus "never our own group", using stand-ins that match nothing real, so it is safe
to run with the stack up. **`Ctrl-C` on the server still stops only what it started** — closing
the dashboard must not kill someone else's terminal session.
(a2) **`status()` racing `stop()` is the bug to watch for here.** The polling `/api/state`
handler and a stop run on different threads; the first version had `status()` clear
`self.proc` when it saw the exit, pulling it out from under `stop()`'s `self.proc.poll()` —
which threw mid-restart and left all four processes stopped and none restarted. It only
reproduces while a browser is actually polling. `Managed` now serialises start/stop behind
`self._op`, every reader snapshots `self.proc` into a local, and `status()` stays quiet during
a deliberate stop. Children also get `stdin=DEVNULL` so a tool that prompts (e.g.
`clear_fault_logs.py --no-archive`) fails fast instead of hanging until its timeout.
(b) **rclpy costs ~1.6 ms of Python per message on this Jetson**, so subscription choice
dominates everything (the whole `/api/state` handler is 0.94 ms). A first cut watching
`/fmu/out/vehicle_odometry` (100 Hz) and `/livox/imu` (200 Hz) burned **28.5% of a core doing
nothing**. Dropping both as redundant (vehicle_status freshness and `/lidar_slam/odom` already
prove those links), `raw=True` everywhere with lazy decode at poll rate, creating the 50 Hz
`vehicle_local_position` subscription only while a browser polls, and **not polling from a
hidden tab** gives the measured figures: **1.50% idle / 16.0% watched with the stack down;
1.55% idle / 20.9% watched with it running** — against ~165% of a core for the stack itself.
**Measuring this is easy to get wrong, twice over:** an open browser tab on another machine
makes a healthy server look like a runaway leak (a stale-DDS hunt that found nothing), and any
"idle" measurement must wait out `IDLE_AFTER` (15 s) or it includes the subscriptions still
timing out. Measure on a dedicated `--port`.

## Host power, the laptop half, the service, bag recording

**Power off / Reboot the *Jetson* (added 2026-09-10) — three gates, none a fallback for the
others.** Two buttons at the bottom of the Host card running `sudo -n systemctl poweroff|reboot`
(`scripts/gcs/hostpower.py`): the page asks **twice** and the second question names the host; the
POST must carry **`?confirm=<mode>`**, because `curl` does not run `app.js` and the asking cannot
live only in the page; and `host_power_blocked()` refuses while the FC says **ARMED** or **IN
AIR**. That interlock is deliberately **not** `reboot_blocked()`'s — *no telemetry at all is the
ordinary end-of-day case and does not block*, since refusing then only teaches people to ssh in
and `sudo`, which skips every gate. Only positive evidence of flight blocks, plus the one
asymmetric case: a link that **said ARMED and then went stale** (a vehicle that stopped talking
while armed may be in the air). `systemctl`, not `shutdown -h now`, so the ordered shutdown stops
`super-gcs.service`, whose teardown SIGINTs a running `ros2 bag record` — that is what keeps the
bag playable. `-n` so it can never sit on a password prompt no one will see; without passwordless
sudo the buttons grey out and say so. **`GCS_POWER_DRY_RUN=1` replaces both commands with an
`echo`** and both `/api/state` and the page say **DRY RUN** — that is how
`scripts/gcs/test_hostpower.py` exercises all three gates (28 assertions, its own server on a
free port) without any chance of shutting this box down to prove a refusal works. **The reboot
does not bring the stack back**: `super-gcs.service` is dashboard-only, by design. Measured on this aircraft 2026-09-07: ACK ACCEPTED, back in **14.1 s**, no new
hardfault — but a reboot **is** the close-and-reopen of the XRCE client's UART behind the known
NuttX use-after-free, so scan the card afterwards.
`scripts/gcs_laptop.sh` is the optional laptop half that makes the
"Open RViz" button work by driving `laptop_viewer.sh` unchanged; pass it the full GCS URL
including `?k=<token>`, quoted. **It is a polling CLIENT, not a localhost server** — a page on
192.168.x.x calling `127.0.0.1` is a private-network request that Chrome blocks without
`Access-Control-Allow-Private-Network`, and it could never work from a phone; polling outward
has no cross-origin story to get blocked. `python3 scripts/gcs/test_render.py` executes the
front end head-lessly against a live `/api/state` (needs `gjs`) — there is no browser here, and
a parse check cannot catch a renamed field. `sudo bash scripts/gcs_service.sh install` makes
it a systemd unit (`super-gcs.service`, runs as the invoking user, reuses the token so a
bookmark survives reboots) — **dashboard only, it never starts the stack at boot.** The unit
is `After=network.target` on purpose, *not* `network-online.target`: the GCS binds 0.0.0.0
and re-resolves IPs per request, so waiting would only risk a 90 s boot delay if anyone
enabled a wait-online unit. `KillMode=mixed`/`TimeoutStopSec=90` let the GCS do its own
ordered teardown on shutdown (measured 3.4 s, zero orphans, with the stack up).
**Survives apt upgrades, not a reflash** — re-run it after any reimage.
See `scripts/gcs/README.md`.
**Bag recording (added 2026-09-10) — four things not to re-derive:**
(0) **`bags/` at the repo root, gitignored, and *All topics* is ~1 GB/min here** — measured,
87 MB in 5.4 s, and that run had the LiDAR powered down. Starting is refused below **5 GB
free** and a running recording **stops itself** at that floor, because bags land on the same
rootfs the flight stack swaps against and `swap != 0` is the M1 regression signature.
**⚠️ 2026-09-30 14:55: *All topics* with the follower up blinded perception.**
`bags/2026-09-30_145506` is 4.0 GB in 54 s (4.4 GB/min: raw colour, raw depth,
`/camera/depth/points`, `/livox/lidar`, both clouds). While it recorded:
- perception published no people on 77 % of frames (211 of 331 on depth/colour skew 0.2–1.35 s,
  45 on TF);
- the overlay read "1256 ms behind";
- the camera topics reached the recorder 1.1 s (colour) and 0.5 s (depth) late, median.

The `stage_b` preset at 14:22 the same day was 8.6 % blind.
**With the follower running, record a preset (`stage_a` / `stage_b`), not *All topics*.** Such a
bag also does not replay as it ran: `ros2 bag play` keeps the arrival order, so the replay is just
as blind (395 frames, 0 people). It kept only 77 of FAST-LIO's `/tf` in 54 s.
(`docs/evidence/lidar-range-2026-09-30/bench-1455/`)
(i) **Stop is not optional: rosbag2 writes `metadata.yaml` only on SHUTDOWN.** Kill the
recorder instead and every message is on disk with no way to read it back — `ros2 bag play`
refuses the directory. Hence SIGINT with the same 15 s grace as `fsm_node`, and a bag missing
that file is listed as **`NO metadata.yaml`** in red rather than as one that finished.
The GCS's own Ctrl-C stops a recording **first**, before the four process groups.
(ii) **`-x/--exclude` cannot be combined with an explicit topic list in Humble** — it exits 1
with *"--exclude argument cannot be used when specifying a list of topics explicitly"*, despite
its own `--help` promising otherwise. `bagger.record_argv` only passes it alongside `-a`.
(iii) **Recording is the second door to the camera, so it has the same lock.** While Admin
control is locked, image topics are absent from the picker, naming one is 403, and *All topics*
gets the image `--exclude`. The gate is on the **imagery**, not on the word "camera":
`/camera/*/camera_info` and `/camera/depth/points` stay listed for everyone, deliberately —
a depth cloud is flight-test data and `/cloud_registered` images a room just as well.
**⚠️ FIXED 2026-09-10, and it had teeth: `stop_all(external=False)` was adopting and killing
outside processes.** The server's own Ctrl-C teardown walked every row; each row found *its
own* terminal-started processes and fell through to `stop_foreign()`. So **closing the
dashboard killed a stack somebody had running in a terminal** — the exact thing `stop_all`'s
docstring promises it will not do. Seen for real this session: a test GCS on `--port 8099` was
Ctrl-C'd and took a hand-started Livox driver, FAST-LIO and all three PX4 bridges with it
(`stack: killed 5 process(es) started outside this GUI`). `Managed.stop()` now takes an
`adopt` flag — **True for the row buttons, False for every sweep**; STOP ALL still reaches
them via `stop_external()`, which is the explicit ask. `test_supervisor.py` case 5 asserts it.

## The dashboard's URL

- `scripts/set_hostname.sh` — **the Jetson is named `gypsy`** (renamed from the stock `ubuntu`
  on 2026-09-07), so the dashboard has a URL that survives a DHCP change:
  **`http://gypsy.local:8080`** on the field Wi-Fi (mDNS, no internet needed) and
  **`http://gypsy.tail4e9b1c.ts.net:8080`** from anywhere (Tailscale MagicDNS).
  **The token no longer has to be in the URL** — a memorable name is one you *type*, and typing
  it drops the `?k=…`, whereupon the page loads and every API call 401s, which reads as a broken
  dashboard rather than a missing credential (it happened the first time the name was used). The
  browser now remembers the token after one successful load, a URL without one shows a
  paste-it-once box instead of a red banner, a rejected token is forgotten rather than retried
  forever, and a 401 is never reported as "lost the Jetson". `localStorage`, not a cookie — a
  cookie rides along on cross-site POSTs and this API can stop the stack.
  **`tailscale serve` cannot drop the `:8080` on this box** (tried 2026-09-07, don't redo):
  HTTPS certs are not enabled on the tailnet (`tailscale cert` → *"your Tailscale account does
  not support getting TLS certs"*; it is an admin-console toggle), and `--http=80` silently loses
  to **ARK's nginx**, which holds `0.0.0.0:80` for the ARK UI — `serve status` lists the proxy
  and `/` returns 200, but it is *ARK's page*, and `/api/*` gives nginx's 503. **A 200 on `/` is
  not proof the proxy works; check the `Server:` header.** Port 443 is free, so enabling certs
  would make `https://gypsy.tail4e9b1c.ts.net` work — worth it also because
  `navigator.clipboard` only exists in a secure context, so on plain HTTP the dashboard's copy
  buttons fall back to the deprecated `document.execCommand` path. `ubuntu` was the
  Ubuntu default and would have been silently renamed to `ubuntu-2.local` the moment another
  stock Ubuntu box joined the same Wi-Fi. The script also rewrites `/etc/hosts`' `127.0.1.1`
  line (or every `sudo` warns "unable to resolve host"), sets the **Tailscale** machine name
  (which does not follow the system hostname), restarts ARK's `flight-review.service` (it
  re-derives `--allow-websocket-origin` from `hostname -f` at start — nothing of ARK's is
  edited), and sets `deny-interfaces=docker0,eno1` in avahi so **`gypsy.local` cannot resolve to
  the MID360's point-to-point link** (192.168.1.50), which is the same unroutable address
  `laptop_viewer.sh` warns about. Idempotent; **survives a reboot, not a reflash.**

## The DTP follower row and Engage FOLLOW

### Added 2026-09-13 (fifth session) — GCS start/stop for the DTP follower
The web GCS's Stack card has a **DTP follower** row (`scripts/dtp_follower.sh`, riding on the stack,
`lidar:=false super:=false`). Details: `scripts/gcs/README.md`, "The DTP follower". **Never started
from the dashboard on the aircraft yet** — tests and the wrapper's refusals only.
- **Modes, the user's decision:** **Observe only** (default on every page load) and **Live**.
  - Live needs `?confirm=live`, a fresh FC link saying disarmed, `fsm_node` on
    `follow_test_ros2.yaml` and odom ≥ 9.5 Hz.
  - The wrapper re-asks `/fsm_node` over ROS before a live launch.
- **Both modes refuse** unless:
  - the stack is up with one `fsm_node`;
  - the Gemini is free;
  - the FC is not ARMED or IN AIR.
- **Controls, log and reasons are behind Admin control.** The row's state and mode are shown to
  everyone.
- **Stop is never refused.** It asks first, and asks twice for a live follower on an armed aircraft.
- **Restart is refused while any follower runs.** Otherwise it now keeps the stack's config.
- **Mode and config are read off the processes' `/tmp/launch_params_*`.** They are not remembered
  from the request, so they are true for terminal-started nodes too.
- **The RC card now reads the config `fsm_node` really loaded.** Before, it read
  `click_smooth_ros2.yaml` always.
- **Tests:** `test_dtp.py` (90 checks, own server, starts nothing) and synthetic stopped/running
  rows in `test_render.py`.
- **Two pre-existing test failures, not from this change:**
  - `test_admin.py`'s "an unlocked client may see image topics" checks after logging out;
  - `test_render.js`'s "PX4 echo negates y and z" uses a 1e-6 tolerance against a
    3-decimal-rounded echo, so it fails whenever the RC poll has a pose.
- **The running `super-gcs.service` serves the new page but not the new API** until it is
  restarted. The page degrades cleanly: no follower row, no config chips.

- **✅ "Engage FOLLOW" button on the GCS (2026-09-15, user's request) — the page can now set
  `follow_enabled`.** `POST /api/dtp/follow?set=on|off` (Admin control), pure interlock
  `dtp.follow_blocked()` (no follower / no telemetry / mux ABORT refuse; **armed does not**, and
  there is no confirm, by design: a LOST in flight is re-engaged in seconds or not at all),
  `telemetry.follow()` = `SetParameters` on `/target_manager` false → 0.7 s → true (what
  `field.sh reengage` does). The DTP row and the Flight day card's engage step show a lock pill from
  `/dtp/lock_state` (+ `/mux/state`, hot tier, BEST_EFFORT) and the button. `test_dtp.py` +11
  checks, `test_render` 21/21 buttons, and a live end-to-end call took the real follower
  LOST → ACQUIRING. **The running `super-gcs.service` serves the new page but not the new API
  until restarted — and restarting it stops the dashboard-owned stack and bridges** (its ordered
  teardown). Do it between flights: stop follower → `sudo systemctl restart super-gcs` →
  Start stack/bridges/follower again. Not yet restarted as of 00:40.

### Changed 2026-09-20 — the Flight day card is now the standard follower bring-up (user's request)
Details: `scripts/gcs/README.md`, "The Flight day card". Don't re-derive:
- **It no longer mirrors `docs/flight-day-runbook.md`.** The 37-row, three-sortie list is gone;
  the card holds 12 showing steps, every one a button: stack (follow) → both bridges → FAST-LIO
  gates → follower → record → preflight → Engage FOLLOW → estimator (later arms) → stop recording
  → stop follower → fault scan. **The runbook's step numbers are its own now**, not the card's.
- **One Observe only / Live choice at the top**, the same variable as the DTP row's chips
  (`DTP_MODE`), observe on every page load, fixed to the running follower's real mode. Steps carry
  `modes` in `flightday.py`; hidden ones stay in the DOM (`.step[hidden]` needs its own CSS rule,
  because `.step{display:flex}` beats the attribute).
- **New check `live_preflight`** = `--stage-b --target person --max-vel 1.0 --waive-d21`, the
  flags `deploy_follow.sh` passes. The strict `stage_b_preflight` chip FAILs on both every run.
  **`1.0` is in three places now** (`follow_test_ros2.yaml`, `deploy_follow.sh`,
  `supervisor.CHECKS`). It still FAILs on `auto_reacquire_after_lost`, by design.
- **The recorder goes up before the preflight**, as in `deploy_follow.sh`. A Stop-recording step
  was added (`POST /api/bag/stop`).
- `desk`, `rc_switches` and the two strict Stage B preflights are still chips on the Checks card.
- **Tests:** `test_render.py` all passed (4 passes, flight-day block rewritten), `test_dtp.py`
  and `test_supervisor.py` all passed, on a private `--port 8099` server. **No button was pressed
  against a running stack.** `goal_mode` is not on the card: the nmpc arm is not installed.
- **The running `super-gcs.service` needs a restart to serve the new step table and check** —
  between flights, as above (its teardown stops the dashboard-owned stack and bridges).

## Claude Remote Control row (2026-09-23)
- **A `claude` row starts `scripts/claude_remote.sh`**, which runs `claude remote-control` in this
  repo on the **personal profile**: `CLAUDE_CONFIG_DIR=~/.claude-personal`, set in the script and
  never inherited. You drive it from the Claude app's Code tab or claude.ai/code, as the session
  named after the host. It needs internet.
- **It is behind Admin control** (`RESTRICTED_PROCS`), because a session there is a shell on the
  flight computer. Its log is gated too.
- **Its pattern is `claude_remote.sh` only.** Matching `claude` would let STOP ALL kill every
  Claude Code session on the box, including ones run from a terminal.
- **Restart neither stops nor starts it** (`RESTART_IGNORES`). Stop, STOP ALL and the GCS's own
  shutdown do stop it.
- **`claude remote-control` asks "Enable Remote Control? (y/n)" on every start**, even with
  `/rc` on by default (that is the interactive `claude`'s setting). A `y` typed in a terminal was
  not remembered (`remoteEnabled` stayed `false`). With stdin `/dev/null` the row sat at the
  prompt, running and connected, and never appeared in the app. The script now pipes `y` and then
  keeps stdin open (`{ echo y; exec sleep infinity; }`). At EOF it stops reading its keys.
  Verified 2026-09-23: it connected and printed a session URL.
