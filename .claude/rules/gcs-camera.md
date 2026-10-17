---
paths:
  - "scripts/gcs/camera.py"
  - "scripts/gcs/admin.py"
  - "scripts/gcs/bagger.py"
  - "scripts/gcs/test_admin.py"
  - "scripts/gcs/test_camera.py"
  - "scripts/gcs/static/**"
  - "scripts/gcs/test_render.*"
  - "scripts/gemini_*"
  - "super_bringup/launch/gemini_camera.launch.py"
  - "rviz/gemini_viewer.rviz"
  - "docs/geminitest.md"
---

# GCS camera panel and the Admin-control gate

Scope: the Gemini 336L panel, the second password that guards it (and the RC card and the image topics in recordings), and who may open the camera. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

**The camera panel (added 2026-09-08) — four things not to re-derive:**
(0) **It is behind a SECOND password — `Admin control` in the header — and that gate is
enforced in the server, not the page.** Every `/api/camera/*` and `/api/proc/camera/*` route
answers **403** without a session, and `/api/state`/`/api/config` **omit the camera entirely**
(no topic, no device serial, no proc row, no log tab) — the panel is hidden because the data
is absent, not the other way round. `curl` does not run `app.js`, so a JS-only check would
have been theatre. The password is a **PBKDF2-SHA256 hash** (200k iterations, ~93 ms here) in
`scripts/gcs/admin.py`; **the plaintext is deliberately not in this repo** — `~/.cache/super-gcs/admin`
overrides the shipped default, set it with `python3 scripts/gcs/admin.py --set` (then restart
the GCS). Sessions are in-memory + `sessionStorage` (tab-scoped, 30 min idle, gone on GCS
restart); 5 wrong tries → 30 s cooldown. **A wrong camera password answers 200 + `ok:false`,
never 401** — a 401 makes `app.js` discard the *dashboard* token and show the wrong gate.
`scripts/gcs/test_admin.py` asserts the whole locked shape against a live server.
**The gate deliberately names nothing** — heading `Admin control`, button `Unlock`,
placeholder `password`, message "Locked. Enter the password to continue.", API refusal
"restricted -- unlock it with Admin control". All of it is rendered for a client that has
unlocked nothing, so naming the feature there would hand back what the redaction withholds;
`test_render.js` asserts it on both the served markup and the message written when the
button is pressed. **Known limit, don't be surprised by it:** `app.js` is served to everyone
and necessarily contains `renderCamera` and `/api/camera/stream`, so *View Source* reveals
that a camera feature exists. Every *fact* about it is withheld; its existence is not.
Closing that means serving the panel's markup+script only to unlocked clients — not done.
**The camera panel (added 2026-09-08) — three more things not to re-derive:**
(i) **The feed is MJPEG into an `<img>`, and nothing decodes anything.** The Orbbec driver
already forwards the camera's own MJPEG as `CompressedImage`, so `/api/camera/stream`
(`multipart/x-mixed-replace`) copies those bytes camera→USB→DDS→socket. A decode/re-encode
per frame would cost more than the flight stack. It needs no player, no websocket and no
build step — the only kind of video this dashboard could have had — and it works over
Tailscale, which the RViz/DDS path cannot.
(ii) **A Gemini can be opened by exactly ONE process** — a second reader dies with
`uvc_open ... Return Code: -6` — and on this Jetson it is *routinely already owned* by
another launch under `/camera` (`dtp_perception`'s, seen 2026-09-08 on serial CPC64630007M
at 1280×720/30). So Start is offered only when it is genuinely free, decided from three
places: a live `CompressedImage` publisher, `/sys/bus/usb/devices` for the hardware, and
`/proc/*/fd`→`/dev/bus/usb/…` for a non-ROS holder. When it is not free the page names the
**topic or the pid**, and `server.camera_blocked()` refuses the POST with the same sentence
— the interlock is one function because it is rendered twice. **Running the driver and
watching an existing publisher are two separate actions in the UI**: *Start camera* launches
ours under `/gemini`, while the source chips pin any publishing topic
(`POST /api/camera/source?topic=…`; empty = Auto, which prefers ours). Watching starts and
stops nothing, a pin holds only while that topic publishes and otherwise falls back with the
panel saying so, and switching source does not interrupt an open stream. Verified live
2026-09-08 with both topics up: Auto chose `/gemini`, pinning flipped it to `/camera`, and
frames flowed from each. **The supervisor row deliberately matches
only `gemini_camera.sh`/`gemini_camera.launch.py`, never `gemini_330_series.launch.py`**, so
STOP ALL cannot take down another project's camera. `python3 scripts/gcs/camera.py` answers
"who has the camera" from a terminal (its fd walk only sees your own user's fds).
(iii) **Cost, measured 2026-09-08 with a 720p/30 source:** 0.95% of a core idle, 10.9%
with a browser polling, **14.3% with one stream open** — the image subscription is created
on demand and dropped 6 s after the last viewer, exactly like telemetry.py's 50 Hz subs
(`camera.subscribed` in `/api/state` is `null` when nobody is looking). Clearing the
`<img>`'s `src` is what closes the socket; hiding the element does **not**. The browser
stream is capped at 10 fps (`?fps=`), which saves **bandwidth, not CPU** — 720p/30 is
~40 Mbit/s and field Wi-Fi will not carry it to a phone.
