# Durable fact — Offboard engages **without RC**; arming does not

**Measured on the aircraft 2026-09-02 (Orin NX + ARK FPV, PX4 v1.17.0).** This corrects a claim
that had been repeated through `CLAUDE.md`, `ImplementationDoc.md` §M3 and `scripts/px4/README.md`:

> ⛔ *"PX4 refuses Offboard without either an RC link or `COM_RCL_EXCEPT` bit 2 set."*

**That is wrong for the mode switch.** With `COM_RCL_EXCEPT = 0`, no receiver ever bound
(`rc_update status` → `cycle: 0 events`) and `manual_control_signal_lost: True`, the Offboard mode
switch was **ACCEPTED** and `nav_state` went to `OFFBOARD`. It is right for *arming* — which is the
part that actually matters, and the part worth checking properly.

---

## Why the mode switch is allowed

`failsafe_flags` carries a per-mode requirement bitmask indexed by `nav_state`. Offboard is
`nav_state = 14`, so bit 14 (value 16384):

```
mode_req_offboard_signal: 16384   -> bit 14 SET   : Offboard needs the offboard signal
mode_req_manual_control:  34119   -> bit 14 CLEAR : Offboard does NOT need manual control
mode_req_prevent_arming: 3944480  -> bit 14 CLEAR : being in Offboard does not itself block arming
```

`COM_RCL_EXCEPT` governs the RC-**loss failsafe**, not mode entry. And with `COM_RC_IN_MODE = 3`
("RC or Joystick, keep first") and a receiver that has *never* produced a frame, PX4 does not
declare an RC loss at all — there was never a link to lose.

What PX4 *does* require before accepting Offboard, and what it verified here:

```
offboard_control_signal_lost: False     <- our heartbeat + setpoint stream is healthy
local_position_invalid:       False     <- EKF2 has position (from the M2 vision path)
```

Both streams must already be running at **>2 Hz before** the switch is commanded.
`scripts/px4/offboard_engage.py` refuses to send the command unless it can see them.

## ✅ Resolved 2026-09-02 — and what it took

A receiver was bound. That cleared `remote_control` from `arming_check_error_flags`, and once
both bridges were streaming, `can_arm_mode_flags` bit 14 (Offboard) went set. **But the first
gate attempt still failed**, and the reason is worth knowing:

**There was no Offboard position on the mode switch.** The six slots read
`COM_FLTMODE1=8` (Stabilized), `4=2` (Position), `6=1` (Altitude), with 2/3/5 unassigned —
PX4's classic three-position mapping. The pilot cycled Stabilized → Position → Altitude and
never reached Offboard; the recorder showed `offboard=False` for the whole run.

The fix was **`RC_MAP_OFFB_SW = 11`** — a dedicated two-position Offboard switch, which this
build fully supports: mapped at `rc_update.cpp:200`, read against `RC_OFFB_TH` (0.75) at
`rc_update.cpp:622`, and acted on at `ManualControl.cpp:221`. It is better than spending a mode
slot: it gives RC **engage and takeover on one channel**, and costs none of Stabilized /
Position / Altitude. Verify it took with `listener manual_control_switches` → `offboard_switch`
should read **3** (`SWITCH_POS_OFF`) or **1** (`ON`); **0 means `SWITCH_POS_NONE` — not mapped.**

The M3 gate then passed 7/7. Two behaviours surprised the pilot on that run; both are correct:

**Motors spin up to ~`MPC_THR_HOVER` the instant you arm in Offboard** (0.41 here). A position
setpoint hands PX4 ownership of throttle and its baseline output is hover thrust. This is *not*
an error-driven ramp — on the signed-off run the altitude error stayed under **7 mm** for all
34 armed seconds, so the thrust was pure feedforward. It is exactly why the gate is props-off,
and it means arming into Offboard on the ground is ~41 % throttle against a ground-level
setpoint, not idle.

**You get ~10 s armed, then PX4 disarms itself.** `COM_DISARM_PRFLT` (10 s) auto-disarms when no
takeoff is detected, and with props off none ever is. Not a fault. Repeat the engage/arm cycle
if you want more evidence; `m3_gate_check.py` accumulates across episodes. `COM_DISARM_PRFLT=0`
disables it for bench work — restore it before flight, it is what stops a forgotten aircraft
sitting armed.

**Offboard can be entered while disarmed** — the same run held it 12.8 s disarmed before arming.
Engage first, then arm into it; that is the safer order and the one the gate assumes.

## Why arming was blocked (historical — the state before the receiver was bound)

```
can_arm_mode_flags:       0           <- cannot arm in ANY mode
arming_check_error_flags: 16777344    = bit 24 | bit 7
health_error_flags:       1048576     = bit 20
```

Decode the bits with the enum PX4 generates into
`~/PX4-Autopilot/src/lib/events/enums.json` → `components.1.enums.health_component_t`
(**not** `libevents/events/common.json`, which does not contain it):

| bit | component | why it is set here |
|---|---|---|
| 7 | `remote_control` | **no receiver bound at all** — the real blocker |
| 24 | `global_position_estimate` | expected: `EKF2_GPS_CTRL=0`, GPS aiding deliberately off |
| 20 | `system` | generic system-level health error (health, not arming, flags) |

So the RC receiver was what stood between here and a props-off arm test. `commander check` prints
only `Preflight check: FAILED` on the NuttX console — the per-check reasons go out over the events
interface to QGC, **not** to the shell — so `listener health_report` plus this table is the way to
get an actual answer from the command line.

## The method, which is the reusable part

```bash
python3 scripts/px4/px4_shell.py "listener failsafe_flags"   # per-mode requirements + live faults
python3 scripts/px4/px4_shell.py "listener health_report"    # what blocks arming, as a bitmask
```

Then decode against `health_component_t`. This answers "why won't it arm" in one step, instead of
guessing at parameters.

## If you ever need to arm with no RC (not needed now — a receiver is bound)

`COM_RC_IN_MODE = 4` ("stick input disabled") makes PX4 stop requiring a receiver. **Understand what
that costs before setting it:** it removes RC manual takeover, which is the entire safety story of
the M3 props-off gate ("flip back to manual → instant takeover"). The abort path becomes a software
disarm over DDS (`scripts/px4/offboard_engage.py --disarm`). Bind a receiver instead unless there is
a specific reason not to. Take a param backup first:
`python3 scripts/px4/dump_params.py firmware/px4-params-$(date +%F).params`.

---

## Related

- [`ImplementationDoc.md`](ImplementationDoc.md) §M3 — the bridge and the gate.
- [`fact-super-cmd-stream.md`](fact-super-cmd-stream.md) — why the setpoint stream must be driven
  by our own timer rather than forwarded from SUPER.
- [`fact-ev-frd-yaw-align.md`](fact-ev-frd-yaw-align.md) — the *other* thing that looks like it
  should block Offboard and doesn't (`heading_good_for_control: False`).
