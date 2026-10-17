---
paths:
  - "scripts/gcs/rc.py"
  - "scripts/gcs/test_rc.py"
  - "scripts/gcs/static/**"
  - "scripts/gcs/test_render.*"
  - "scripts/px4/goal_forward.py"
---

# GCS remote-control card: flying the aircraft through SUPER goals

Scope: `scripts/gcs/rc.py` and its card, and the two goal inputs SUPER has. The dashboard itself is `gcs-dashboard.md`. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

**Remote control card (added 2026-09-10; typed entry + coordinate bar 2026-09-10) — the
dashboard can now MOVE THE AIRCRAFT, behind Admin control.** Forward/back/sideways/rotate with
**typed step and turn boxes** beside the presets, an altitude slider **and a typed height**, a
**coordinate bar** for one absolute goal in `lidar_init` (with the PX4-NED echo beside it, capped
at 10 m), an attitude ball, a compass and an altitude tape (`scripts/gcs/rc.py`, `/api/rc/*`). It is **goal-based**: every button publishes
exactly one `/goal_pose` — the RViz *2D Goal Pose* message — and SUPER plans the path, so
avoidance stays in the loop and a Wi-Fi drop means *no new goal*, not *no setpoints*. Goals are
relative to **where the aircraft is now** and forward is where the **nose** points (body-frame
rotation, all four signs asserted in `test_rc.py` — a sign error there flies it the wrong way and
no status code would show it). **There is deliberately no STOP button: SUPER's FSM has three
inputs (click goal, follow goal, yaw target) and none is a stop, so the abort is the RC's
channel-11 switch** — the card says exactly that.
**The card tracks a goal to its conclusion, not just to "sent"**: Moving (metres left + ETA) →
**Arrived**, using SUPER's *own* `closeToGoal(0.1)` (`fsm.cpp:134`) so the page and the planner
mean the same thing by "there", and measured against the point `/planning/goal_status` says was
**adopted** (the snap moves goals by up to 3 m). Two states exist because a plain spinner gets
them wrong: **"Sent — but nothing is flying"** (disarmed / not in Offboard — the normal bench
case) and **"No progress"** (armed in Offboard, no closer for 20 s — SUPER has no free path).
Each terminal state is logged **once**, not at the 5 Hz poll rate.
**⭐ SUPER HERE HAS TWO GOAL INPUTS, AND THE SECOND ONE IS MUCH BETTER — don't re-derive, and
don't reach for `click_height` again.** The installed `fsm_node` (built 2026-09-09) carries the
DTP patch's `/planning/follow_goal` → `Fsm::setFollowGoal()` (`fsm.cpp:226`,
`fsm_ros2.hpp:293`). Against upstream's click path it **preserves z** (deliberately — the code
says so in a comment), has **no 0.1 m silent drop**, and **always answers on
`/planning/goal_status`** with ACCEPTED / SNAPPED / DROPPED plus the adopted point and the
request echoed. It is off by default (`follow_goal_en{false}`, `config.hpp:55`); `scripts/setup.sh`
now writes `fsm/follow_goal_en: true` into `click_smooth_ros2.yaml` (idempotent, and a **no-op on
a clean install** — unpatched SUPER never reads the key). **`click_height` is deliberately left at
1.5 so an RViz 2D-Goal-Pose click, which always sends z=0, still flies at 1.5 m and not at the
floor.** Only the GCS card moves to the other path. Measured end to end 2026-09-10 in an isolated
ROS domain: altitude 2.4 m accepted **with `click_height: 1.5` still set**, and a 45° rotate came
out as a *pure* rotation with the position unchanged — **under `yaw_mode: 1`. Since 2026-09-12 the
click config runs `yaw_mode: 3`, so Rotate is now refused** (see the `yaw_mode` note in
`super-planner-rogmap.md`).
**The click path's three traps still apply wherever it is in force:** (1) `fsm/click_height`
overwrites the z of every goal (`fsm.cpp:188`) — so there up/down and the slider are **refused
with that reason**, because both buttons would produce the same goal. (2) A goal within 0.1 m is
**dropped silently** (`fsm.cpp:201`), so rotate places the goal **0.15 m along the new heading**.
(3) Goal yaw needs `click_yaw_en` **and** `goal_yaw_en` and a `connected_goal`
(`super_planner.cpp:814`); the follow path skips `click_yaw_en` (it reads the quaternion
straight). Under `yaw_mode: 3` the goal yaw is skipped on both paths, even with no target.
**`rc.choose_path()` picks the path from the ROS GRAPH, not the config** — which topic the
*running* planner subscribes to — and falls back to click. **Contention is refused, not merged:**
`/planning/follow_goal` legitimately has other publishers (`follow_planner`, `command_mux`), ours
is created at node init, so **>1 publisher means someone else is driving** and every command is
refused naming it. The panel reads capabilities off the config the planner actually loads, so an
edit + restart changes the UI with no code change. **Cost, measured:** the attitude ball is `/fmu/out/vehicle_attitude` at **100 Hz** and
rclpy cannot be asked for less — **0.75 ms/msg even raw**, i.e. **+8.2% of a core** (14.4% → 22.7%
with a dashboard already open), and **zero when the card is off screen**: an `IntersectionObserver`
stops the 5 Hz poll and the server drops the subscription 15 s later.
`/planning/goal_status` rides the same tier for free (one message per goal) but **must be
subscribed BEST_EFFORT** to match `fsm_ros2.hpp:328` — a default RELIABLE subscription never
matches and never errors, the `docs/fact-super-cmd-stream.md` trap again. `dtp_msgs` is imported
in a `try`, so a box without the DTP tree loses the status and nothing else. Altitude and heading are
free (they ride the existing `vehicle_local_position` subscription).
